"""Regression coverage for the on-demand operator telemetry bridge."""

import asyncio
import math
from pathlib import Path
import struct
import sys
import threading
from types import SimpleNamespace
import unittest


sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "runtime" / "python"),
)

from geometry_msgs.msg import Quaternion  # noqa: E402
from builtin_interfaces.msg import Time as RosTime  # noqa: E402
from avg_msgs.msg import (  # noqa: E402
    AvgAprilTagPose,
    AvgBool,
    AvgOccupancyGrid,
    AvgServiceState,
)
from sensor_msgs.msg import Image, PointCloud2, PointField  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

from camrod_ui.ui_backend_node import (  # noqa: E402
    TELEMETRY_TOPIC_DEFAULTS,
    UiBackendNode,
    _bounded_telemetry_rate_hz,
    _decimate_xy,
    _encode_raw_image_jpeg,
    _finite_or_none,
    _marker_array_geometry,
    _quaternion_yaw_deg,
    _sample_occupancy_grid,
    _sample_pointcloud_xy,
    _wait_for_websocket_disconnect,
)


class OperatorTelemetryTest(unittest.TestCase):

    def test_telemetry_stream_rate_is_bounded_for_arm_runtime(self) -> None:
        self.assertEqual(_bounded_telemetry_rate_hz(10.0), 10.0)
        self.assertEqual(_bounded_telemetry_rate_hz(0.0), 1.0)
        self.assertEqual(_bounded_telemetry_rate_hz(100.0), 20.0)
        self.assertEqual(_bounded_telemetry_rate_hz(float("nan")), 10.0)

    def test_websocket_disconnect_watcher_consumes_asgi_close_event(self) -> None:
        class FakeWebSocket:
            def __init__(self) -> None:
                self.events = [
                    {"type": "websocket.receive", "text": "ignored"},
                    {"type": "websocket.disconnect", "code": 1000},
                ]
                self.receive_count = 0

            async def receive(self):
                self.receive_count += 1
                return self.events.pop(0)

        socket = FakeWebSocket()
        heartbeats = []
        asyncio.run(
            _wait_for_websocket_disconnect(
                socket, lambda: heartbeats.append("lease")
            )
        )
        self.assertEqual(socket.receive_count, 2)
        self.assertEqual(heartbeats, ["lease"])

    def test_manual_goal_validation_normalizes_yaw_and_rejects_nonfinite(self) -> None:
        goal = UiBackendNode._normalize_manual_goal("12.5", -3, 450)
        self.assertEqual(goal, (12.5, -3.0, 90.0))
        with self.assertRaisesRegex(ValueError, "finite"):
            UiBackendNode._normalize_manual_goal(float("nan"), 0, 0)

    def test_manual_goal_gate_enforces_readiness_service_and_battery(self) -> None:
        state = SimpleNamespace(
            ready=True,
            ready_message="ready",
            battery_percentage=80,
            service_state=AvgServiceState.DROP_ZONE_WAIT,
        )
        backend = SimpleNamespace(
            _lock=threading.Lock(),
            _state=state,
            _latest_platform_is_charging=False,
            require_battery_for_mission_dispatch=True,
            minimum_mission_dispatch_battery_percent=35,
        )
        self.assertIsNone(UiBackendNode._manual_goal_dispatch_block(backend))

        state.service_state = AvgServiceState.MOVING_TO_SITE
        blocked = UiBackendNode._manual_goal_dispatch_block(backend)
        self.assertEqual(blocked["error"], "service_state_busy")

        state.service_state = AvgServiceState.OPERATOR_STOPPED
        state.battery_percentage = 20
        blocked = UiBackendNode._manual_goal_dispatch_block(backend)
        self.assertEqual(blocked["error"], "battery_below_mission_minimum")

        state.battery_percentage = 80
        state.ready = False
        blocked = UiBackendNode._manual_goal_dispatch_block(backend)
        self.assertEqual(blocked["error"], "system_not_ready")

    def test_manual_goal_publishes_raw_pose_before_manual_engage(self) -> None:
        events = []

        class Publisher:
            def publish(self, message) -> None:
                events.append(("goal", message))

        class Clock:
            @staticmethod
            def now():
                return SimpleNamespace(to_msg=lambda: RosTime(sec=1, nanosec=2))

        class Logger:
            @staticmethod
            def info(_message: str) -> None:
                return None

        policy = SimpleNamespace(
            update_goal_received=lambda source: events.append(("policy", source))
        )
        backend = SimpleNamespace(
            _normalize_manual_goal=UiBackendNode._normalize_manual_goal,
            _manual_goal_dispatch_block=lambda: None,
            _yaw_deg_to_quaternion=lambda yaw: UiBackendNode._yaw_deg_to_quaternion(
                None, yaw
            ),
            _lock=threading.Lock(),
            _active_mission_site="B6",
            _state=SimpleNamespace(
                ws_site_states={"B6": True},
                destination={"site": "B6", "run": True},
            ),
            site_names=["B1", "B6"],
            default_goal_frame_id="map",
            manual_goal_pose_topic="/goal_pose",
            pub_manual_goal_pose=Publisher(),
            _runtime_policy=policy,
            _update_runtime_state=lambda update: update(),
            _publish_engage=lambda enabled, source: events.append(
                ("engage", enabled, source)
            ),
            _schedule_broadcast=lambda payload: events.append(("broadcast", payload)),
            get_clock=lambda: Clock(),
            get_logger=lambda: Logger(),
        )

        result = UiBackendNode.set_manual_goal(backend, 4.25, -2.5, 90.0)

        self.assertTrue(result["success"])
        goal_index = next(index for index, event in enumerate(events) if event[0] == "goal")
        engage_index = next(index for index, event in enumerate(events) if event[0] == "engage")
        self.assertLess(goal_index, engage_index)
        pose = events[goal_index][1]
        self.assertEqual(pose.header.frame_id, "map")
        self.assertAlmostEqual(pose.pose.position.x, 4.25)
        self.assertAlmostEqual(pose.pose.position.y, -2.5)
        self.assertAlmostEqual(pose.pose.orientation.z, math.sqrt(0.5), places=6)
        self.assertEqual(backend._active_mission_site, "")
        self.assertEqual(backend._state.destination, {"site": "", "run": False})

    def test_default_topics_match_public_runtime_contracts(self) -> None:
        # HH_260810 - Fail in CI before an ARM deployment opens a blank operator
        # view because a producer and the on-demand UI bridge drifted apart.
        self.assertEqual(
            TELEMETRY_TOPIC_DEFAULTS,
            {
                "gnss_fix": "/sensing/gnss/ublox_gps_node/fix",
                "gnss_navpvt": "/sensing/gnss/ublox_gps_node/navpvt",
                "gnss_navcov": "/sensing/gnss/navcov",
                "gnss_relpos": "/sensing/gnss/navrelposned",
                "imu": "/sensing/imu/data_ros",
                "localization_status": "/localization/status",
                "tracking_error": "/planning/tracking_error",
                "global_path": "/planning/global_path",
                "local_path": "/planning/local_path_ros",
                "lidar": "/sensing/lidar/points_filtered",
                "lidar_raw": "/sensing/lidar/vanjee/points_raw",
                "obstacle_cloud": "/perception/obstacles",
                "obstacle_boxes": "/perception/lidar/bboxes",
                "front_camera": (
                    "/sensing/camera/econ_front/image_rect/compressed"
                ),
                "front_camera_raw": "/sensing/camera/econ_front/image_raw",
                "rear_camera": (
                    "/sensing/camera/econ_rear/image_raw/compressed"
                ),
                "rear_camera_raw": "/sensing/camera/econ_rear/image_raw",
                "docking_debug_camera": (
                    "/perception/apriltag_parking_detector/debug_image/compressed"
                ),
                "docking_tag_pose": (
                    "/perception/apriltag_parking_detector/tag_pose"
                ),
                "docking_tag_detected": (
                    "/perception/apriltag_parking_detector/tag_detected"
                ),
                "map_markers": "/map/markers",
                "lanelet_cost_grid": "/map/cost_grid/lanelet",
                "lidar_cost_grid": "/sensing/cost_grid/lidar",
                "radar_cost_grid": "/sensing/cost_grid/radar",
                "inflation_cost_grid": "/planning/cost_grid/inflation",
                "planning_boundary": "/platform/robot/planning_boundary",
                "robot_markers": "/platform/robot/markers",
                "radar_evidence": "/sensing/radar/obstacle_evidence",
                "obstacle_replan": "/planning/obstacle_replan/status",
            },
        )

    def test_json_float_filter_rejects_non_finite_values(self) -> None:
        self.assertIsNone(_finite_or_none(float("nan")))
        self.assertIsNone(_finite_or_none(float("inf")))
        self.assertEqual(_finite_or_none("1.25"), 1.25)

    def test_quaternion_heading_uses_normalized_yaw(self) -> None:
        yaw = math.radians(90.0)
        orientation = Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        self.assertAlmostEqual(_quaternion_yaw_deg(orientation), 90.0, places=6)
        self.assertIsNone(_quaternion_yaw_deg(Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)))

    def test_path_decimation_preserves_endpoints_and_limit(self) -> None:
        points = [(float(index), float(index * 2)) for index in range(1000)]
        decimated = _decimate_xy(points, 50)
        self.assertEqual(len(decimated), 50)
        self.assertEqual(decimated[0], [0.0, 0.0])
        self.assertEqual(decimated[-1], [999.0, 1998.0])

    def test_pointcloud_sampling_is_bounded_and_filters_invalid_points(self) -> None:
        message = PointCloud2()
        message.height = 1
        message.width = 5
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.data = b"".join(
            struct.pack("<fff", *point)
            for point in (
                (0.0, 0.0, 0.0),
                (1.0, 2.0, 0.0),
                (float("nan"), 1.0, 0.0),
                (30.0, 1.0, 0.0),
                (-2.0, -3.0, 0.0),
            )
        )
        samples = _sample_pointcloud_xy(
            message,
            max_points=10,
            max_abs_xy_m=12.0,
        )
        self.assertEqual(samples, [[0.0, 0.0], [1.0, 2.0], [-2.0, -3.0]])

    def test_map_pointcloud_is_rotated_into_robot_local_frame(self) -> None:
        message = PointCloud2()
        message.height = 1
        message.width = 1
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        ]
        message.point_step = 8
        message.row_step = 8
        message.data = struct.pack("<ff", 10.0, 21.0)
        samples = _sample_pointcloud_xy(
            message,
            max_points=10,
            max_abs_xy_m=12.0,
            map_to_local_pose=(10.0, 20.0, math.radians(90.0)),
        )
        self.assertEqual(samples, [[1.0, 0.0]])

    def test_browser_lease_never_activates_when_feature_is_disabled(self) -> None:
        backend = SimpleNamespace(
            _lock=threading.Lock(),
            _normalize_telemetry_view=UiBackendNode._normalize_telemetry_view,
            enable_operator_telemetry=False,
            telemetry_session_timeout_s=12.0,
            _telemetry_session_deadline=0.0,
            _telemetry_requested_view="all",
            _telemetry_active_view="",
            _telemetry_capture_active=False,
        )
        result = UiBackendNode._request_telemetry_session(backend, True, "camera")
        self.assertFalse(result["enabled"])
        self.assertEqual(backend._telemetry_session_deadline, 0.0)

    def test_stale_tab_release_does_not_cancel_newer_view(self) -> None:
        backend = SimpleNamespace(
            _lock=threading.Lock(),
            _normalize_telemetry_view=UiBackendNode._normalize_telemetry_view,
            enable_operator_telemetry=True,
            telemetry_session_timeout_s=12.0,
            _telemetry_session_deadline=0.0,
            _telemetry_requested_view="gnss",
            _telemetry_active_view="gnss",
            _telemetry_capture_active=True,
        )
        UiBackendNode._request_telemetry_session(backend, True, "perception")
        deadline = backend._telemetry_session_deadline
        UiBackendNode._request_telemetry_session(backend, False, "gnss")
        self.assertEqual(backend._telemetry_session_deadline, deadline)
        UiBackendNode._request_telemetry_session(backend, False, "perception")
        self.assertEqual(backend._telemetry_session_deadline, 0.0)

    def test_new_view_resets_rate_history_from_previous_lease(self) -> None:
        backend = SimpleNamespace(
            _telemetry_source_rx={"gnss.fix": 10.0},
            _telemetry_source_history={"gnss.fix": [9.9, 10.0]},
        )
        UiBackendNode._reset_telemetry_source_timing_locked(backend)
        self.assertEqual(backend._telemetry_source_rx, {})
        self.assertEqual(backend._telemetry_source_history, {})

    def test_occupancy_grid_sampling_is_bounded_and_map_aligned(self) -> None:
        grid = AvgOccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.width = 4
        grid.info.height = 3
        grid.info.resolution = 0.5
        grid.info.origin.position.x = 10.0
        grid.info.origin.position.y = 20.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0, 100, 0, 0, 0, 0, 50, 0, 0, 0, 0, 80]
        samples = _sample_occupancy_grid(grid, max_cells=2)
        self.assertEqual(len(samples), 2)
        self.assertEqual(samples[0], [10.75, 20.25, 100])
        self.assertEqual(samples[-1], [11.75, 21.25, 80])

    def test_marker_cube_is_exposed_as_closed_bbox_outline(self) -> None:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.ns = "bbox"
        marker.pose.position.x = 2.0
        marker.pose.position.y = 3.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2.0
        marker.scale.y = 1.0
        geometry = _marker_array_geometry(
            MarkerArray(markers=[marker]), max_points=20
        )
        self.assertEqual(geometry["frame_id"], "map")
        self.assertEqual(geometry["outline_count"], 1)
        self.assertEqual(geometry["polylines"][0]["points"][0], [1.0, 2.5])
        self.assertEqual(
            geometry["polylines"][0]["points"][0],
            geometry["polylines"][0]["points"][-1],
        )

    def test_raw_camera_fallback_encodes_supported_layout(self) -> None:
        image = Image()
        image.width = 2
        image.height = 1
        image.encoding = "bgr8"
        image.step = 6
        image.data = bytes([0, 0, 255, 0, 255, 0])
        encoded = _encode_raw_image_jpeg(image, max_width=960, quality=72)
        self.assertIsNotNone(encoded)
        self.assertEqual(encoded[:2], b"\xff\xd8")

    def test_raw_camera_fallback_downscales_oversized_frame(self) -> None:
        # HH_260814 - Every real camera frame is wider than the fallback
        # max_width, so the resize branch is the only one the robot ever runs.
        # Exercising it here pins the Pillow 9.0.1 filter-constant location:
        # Image.Resampling arrived in 9.1, and the AttributeError from looking
        # it up escaped the subscription callback and killed the UI backend.
        width, height = 1920, 1080
        image = Image()
        image.width = width
        image.height = height
        image.encoding = "bgr8"
        image.step = width * 3
        image.data = bytes(width * height * 3)
        encoded = _encode_raw_image_jpeg(image, max_width=960, quality=72)
        self.assertIsNotNone(encoded)
        self.assertEqual(encoded[:2], b"\xff\xd8")

    def test_initial_contract_contains_every_operator_surface(self) -> None:
        snapshot = UiBackendNode._new_telemetry_snapshot()
        for key in (
            "gnss", "imu", "radar", "lidar", "cameras", "localization",
            "motion", "paths", "footprint", "perception", "safety", "docking",
            "mission",
        ):
            self.assertIn(key, snapshot)
        self.assertIn("stream_rate_hz", snapshot)
        self.assertFalse(snapshot["session_active"])

    def test_docking_pose_detection_and_charging_are_independent(self) -> None:
        # HH_260818 - A last-known tag pose remains useful for diagnosis, but
        # only tag_detected is allowed to claim the target is currently visible.
        snapshot = UiBackendNode._new_telemetry_snapshot()
        backend = SimpleNamespace(
            _lock=threading.Lock(),
            _telemetry=snapshot,
            _telemetry_source_rx={},
        )
        backend._touch_telemetry_locked = lambda source, now=None: (
            backend._telemetry_source_rx.__setitem__(source, now or 0.0)
        )

        pose = AvgAprilTagPose()
        pose.family = "tag36h11"
        pose.id = 3
        pose.tag_frame = "parking_tag_3"
        pose.pose.header.frame_id = "camera_rear"
        pose.pose.pose.position.x = 0.3
        pose.pose.pose.position.y = 0.4
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation.w = 1.0
        UiBackendNode._on_telemetry_docking_tag_pose(backend, pose)
        UiBackendNode._on_telemetry_docking_tag_detected(
            backend, AvgBool(data=False)
        )

        self.assertAlmostEqual(snapshot["docking"]["tag"]["distance_m"], 0.5)
        self.assertEqual(snapshot["docking"]["tag"]["id"], 3)
        self.assertFalse(snapshot["docking"]["tag_detected"])

    def test_docking_tab_prunes_unrelated_sensor_payloads(self) -> None:
        snapshot = UiBackendNode._new_telemetry_snapshot()
        snapshot["active_view"] = "docking"
        snapshot["docking"]["is_charging"] = True
        snapshot["cameras"]["docking"] = {"available": True}
        snapshot["lidar"]["points"] = [[1.0, 2.0]]

        pruned = UiBackendNode._prune_telemetry_snapshot(snapshot, "docking")

        self.assertTrue(pruned["docking"]["is_charging"])
        self.assertTrue(pruned["cameras"]["docking"]["available"])
        self.assertEqual(pruned["lidar"]["points"], [])

    def test_tab_payload_prunes_unrelated_high_volume_sections(self) -> None:
        snapshot = UiBackendNode._new_telemetry_snapshot()
        snapshot["active_view"] = "safety"
        snapshot["perception"]["cost_layers"]["lanelet"] = {
            "occupied_samples": [[float(index), 0.0, 100] for index in range(800)]
        }
        snapshot["localization"]["trace"] = [[float(index), 0.0] for index in range(600)]
        snapshot["safety"]["gate"] = {"operating_state": "STANDBY"}

        pruned = UiBackendNode._prune_telemetry_snapshot(snapshot, "safety")

        self.assertEqual(pruned["perception"]["cost_layers"]["lanelet"], {})
        self.assertEqual(pruned["localization"]["trace"], [])
        self.assertEqual(pruned["safety"]["gate"]["operating_state"], "STANDBY")


if __name__ == "__main__":
    unittest.main()
