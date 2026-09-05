"""Regression coverage for the on-demand operator telemetry bridge."""

import asyncio
from collections import defaultdict
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
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, PointField  # noqa: E402
from tf2_ros import (  # noqa: E402
    ConnectivityException,
    ExtrapolationException,
    LookupException,
)
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

    @staticmethod
    def _single_point_cloud(frame_id: str, x: float, y: float) -> PointCloud2:
        message = PointCloud2()
        message.header.frame_id = frame_id
        message.header.stamp.sec = 100
        message.height = 1
        message.width = 1
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        ]
        message.point_step = 8
        message.row_step = 8
        message.data = struct.pack("<ff", x, y)
        return message

    @staticmethod
    def _telemetry_backend(tf_buffer):
        warnings = []
        backend = SimpleNamespace(
            _lock=threading.Lock(),
            _telemetry=UiBackendNode._new_telemetry_snapshot(),
            _telemetry_source_rx={},
            _telemetry_last_cloud_decode=defaultdict(float),
            _telemetry_tf_warning_last=defaultdict(float),
            telemetry_lidar_min_period_s=0.0,
            telemetry_max_lidar_points=20,
            telemetry_lidar_max_abs_xy_m=12.0,
            telemetry_max_grid_cells=20,
            telemetry_tf_transform_enabled=True,
            telemetry_tf_latest_fallback_tolerance_s=0.0,
            _tf_buffer=tf_buffer,
            get_logger=lambda: SimpleNamespace(
                warning=lambda message: warnings.append(message)
            ),
        )
        backend._touch_telemetry_locked = lambda source, now=None: (
            backend._telemetry_source_rx.__setitem__(source, now or 0.0)
        )
        backend._lookup_telemetry_xy_transform = lambda **kwargs: (
            UiBackendNode._lookup_telemetry_xy_transform(backend, **kwargs)
        )
        backend._warn_telemetry_tf_failure = lambda **kwargs: (
            UiBackendNode._warn_telemetry_tf_failure(backend, **kwargs)
        )
        return backend, warnings

    @staticmethod
    def _planar_transform(
        *, stamp_s: float, x: float = 10.0, y: float = 20.0,
        yaw_deg: float = 90.0,
    ) -> SimpleNamespace:
        sec = math.floor(stamp_s)
        nanosec = round((stamp_s - sec) * 1_000_000_000)
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        yaw = math.radians(yaw_deg)
        return SimpleNamespace(
            header=SimpleNamespace(stamp=RosTime(sec=sec, nanosec=nanosec)),
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=x, y=y),
                rotation=Quaternion(
                    z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
                ),
            ),
        )

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

    def test_cloud_callbacks_transform_lidar_and_obstacles_to_ui_frames(self) -> None:
        class TfBuffer:
            def __init__(self) -> None:
                self.lookups = []

            def lookup_transform(self, target, source, _stamp):
                self.lookups.append((target, source))
                translation = (
                    SimpleNamespace(x=10.0, y=20.0)
                    if target == "map"
                    else SimpleNamespace(x=1.0, y=2.0)
                )
                yaw = math.radians(90.0)
                return SimpleNamespace(transform=SimpleNamespace(
                    translation=translation,
                    rotation=Quaternion(
                        z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
                    ),
                ))

        tf_buffer = TfBuffer()
        backend, warnings = self._telemetry_backend(tf_buffer)
        cloud = self._single_point_cloud("lidar_link", 2.0, 0.0)

        UiBackendNode._on_telemetry_cloud(backend, "filtered", cloud)
        UiBackendNode._on_telemetry_cloud(backend, "raw", cloud)
        UiBackendNode._on_telemetry_cloud(backend, "obstacles", cloud)

        filtered = backend._telemetry["lidar"]["streams"]["filtered"]
        raw = backend._telemetry["lidar"]["streams"]["raw"]
        obstacles = backend._telemetry["perception"]["obstacle_cloud"]
        self.assertEqual(filtered["points"], [[1.0, 4.0]])
        self.assertEqual(raw["points"], [[1.0, 4.0]])
        self.assertEqual(obstacles["points"], [[10.0, 22.0]])
        for payload in (filtered, raw):
            self.assertEqual(payload["frame_id"], "robot_center_link")
            self.assertEqual(payload["source_frame_id"], "lidar_link")
            self.assertTrue(payload["transform_available"])
            self.assertEqual(payload["transform_mode"], "exact")
            self.assertEqual(payload["transform_age_s"], 0.0)
            self.assertEqual(payload["transform_warning"], "")
            self.assertEqual(payload["transform_error"], "")
        self.assertEqual(obstacles["frame_id"], "map")
        self.assertEqual(obstacles["source_frame_id"], "lidar_link")
        self.assertTrue(obstacles["transform_available"])
        self.assertEqual(obstacles["transform_mode"], "exact")
        self.assertEqual(
            tf_buffer.lookups,
            [
                ("robot_center_link", "lidar_link"),
                ("robot_center_link", "lidar_link"),
                ("map", "lidar_link"),
            ],
        )
        self.assertEqual(warnings, [])

    def test_develop_default_does_not_require_telemetry_tf(self) -> None:
        class UnexpectedTfBuffer:
            @staticmethod
            def lookup_transform(_target, _source, _stamp):
                raise AssertionError("TF must remain opt-in")

        backend, warnings = self._telemetry_backend(UnexpectedTfBuffer())
        backend.telemetry_tf_transform_enabled = False

        UiBackendNode._on_telemetry_cloud(
            backend,
            "filtered",
            self._single_point_cloud("lidar_link", 2.0, 0.0),
        )

        payload = backend._telemetry["lidar"]["streams"]["filtered"]
        self.assertEqual(payload["frame_id"], "lidar_link")
        self.assertEqual(payload["source_frame_id"], "lidar_link")
        self.assertEqual(payload["points"], [[2.0, 0.0]])
        self.assertNotIn("transform_available", payload)
        self.assertEqual(warnings, [])

    def test_cloud_callback_drops_geometry_when_tf_is_unavailable(self) -> None:
        class MissingTfBuffer:
            @staticmethod
            def lookup_transform(_target, _source, _stamp):
                raise RuntimeError("transform is not connected")

        backend, warnings = self._telemetry_backend(MissingTfBuffer())
        cloud = self._single_point_cloud("lidar_link", 2.0, 0.0)

        UiBackendNode._on_telemetry_cloud(backend, "obstacles", cloud)

        payload = backend._telemetry["perception"]["obstacle_cloud"]
        self.assertEqual(payload["frame_id"], "")
        self.assertEqual(payload["source_frame_id"], "lidar_link")
        self.assertEqual(payload["target_frame_id"], "map")
        self.assertFalse(payload["transform_available"])
        self.assertEqual(payload["transform_mode"], "unavailable")
        self.assertIsNone(payload["transform_age_s"])
        self.assertEqual(payload["transform_warning"], "")
        self.assertIn("transform is not connected", payload["transform_error"])
        self.assertEqual(payload["points"], [])
        self.assertEqual(payload["point_count"], 1)
        self.assertEqual(len(warnings), 1)

    def test_future_cloud_extrapolation_uses_bounded_latest_tf(self) -> None:
        test_case = self

        class FutureTfBuffer:
            def __init__(self) -> None:
                self.lookup_times = []

            def lookup_transform(self, _target, _source, stamp):
                self.lookup_times.append(stamp.nanoseconds)
                if stamp.nanoseconds:
                    raise ExtrapolationException("requested stamp is in the future")
                return test_case._planar_transform(stamp_s=99.95)

        tf_buffer = FutureTfBuffer()
        backend, warnings = self._telemetry_backend(tf_buffer)
        backend.telemetry_tf_latest_fallback_tolerance_s = 0.075

        UiBackendNode._on_telemetry_cloud(
            backend,
            "obstacles",
            self._single_point_cloud("lidar_link", 2.0, 0.0),
        )

        payload = backend._telemetry["perception"]["obstacle_cloud"]
        self.assertTrue(payload["transform_available"])
        self.assertEqual(payload["frame_id"], "map")
        self.assertEqual(payload["points"], [[10.0, 22.0]])
        self.assertEqual(payload["transform_mode"], "latest_fallback")
        self.assertAlmostEqual(payload["transform_age_s"], 0.05, places=6)
        self.assertIn("bounded latest TF", payload["transform_warning"])
        self.assertEqual(payload["transform_error"], "")
        self.assertEqual(tf_buffer.lookup_times, [100_000_000_000, 0])
        self.assertEqual(warnings, [])

    def test_production_zero_tolerance_keeps_future_tf_fail_closed(self) -> None:
        class FutureTfBuffer:
            def __init__(self) -> None:
                self.lookup_count = 0

            def lookup_transform(self, _target, _source, _stamp):
                self.lookup_count += 1
                raise ExtrapolationException("requested stamp is in the future")

        tf_buffer = FutureTfBuffer()
        backend, _warnings = self._telemetry_backend(tf_buffer)

        UiBackendNode._on_telemetry_cloud(
            backend,
            "obstacles",
            self._single_point_cloud("lidar_link", 2.0, 0.0),
        )

        payload = backend._telemetry["perception"]["obstacle_cloud"]
        self.assertFalse(payload["transform_available"])
        self.assertEqual(payload["points"], [])
        self.assertEqual(payload["transform_mode"], "unavailable")
        self.assertIn("ExtrapolationException", payload["transform_error"])
        self.assertEqual(tf_buffer.lookup_count, 1)

    def test_non_extrapolation_tf_errors_never_attempt_latest_fallback(self) -> None:
        for error_type in (ConnectivityException, LookupException):
            with self.subTest(error_type=error_type.__name__):
                class TfBuffer:
                    def __init__(self) -> None:
                        self.lookup_count = 0

                    def lookup_transform(self, _target, _source, _stamp):
                        self.lookup_count += 1
                        raise error_type("exact transform unavailable")

                tf_buffer = TfBuffer()
                backend, _warnings = self._telemetry_backend(tf_buffer)
                backend.telemetry_tf_latest_fallback_tolerance_s = 0.075
                UiBackendNode._on_telemetry_cloud(
                    backend,
                    "obstacles",
                    self._single_point_cloud("lidar_link", 2.0, 0.0),
                )
                payload = backend._telemetry["perception"]["obstacle_cloud"]
                self.assertFalse(payload["transform_available"])
                self.assertEqual(payload["points"], [])
                self.assertIn(error_type.__name__, payload["transform_error"])
                self.assertEqual(tf_buffer.lookup_count, 1)

    def test_latest_tf_fallback_rejects_past_zero_and_excessive_age(self) -> None:
        test_case = self

        scenarios = {
            "past_extrapolation": (100.01, "newer than the source"),
            "zero_latest_stamp": (0.0, "latest TF timestamp is zero"),
            "over_tolerance": (99.90, "exceeding the 0.075000 s"),
        }
        for name, (latest_stamp_s, expected_error) in scenarios.items():
            with self.subTest(name=name):
                class TfBuffer:
                    @staticmethod
                    def lookup_transform(_target, _source, stamp):
                        if stamp.nanoseconds:
                            raise ExtrapolationException("exact unavailable")
                        return test_case._planar_transform(
                            stamp_s=latest_stamp_s
                        )

                backend, _warnings = self._telemetry_backend(TfBuffer())
                backend.telemetry_tf_latest_fallback_tolerance_s = 0.075
                UiBackendNode._on_telemetry_cloud(
                    backend,
                    "obstacles",
                    self._single_point_cloud("lidar_link", 2.0, 0.0),
                )
                payload = backend._telemetry["perception"]["obstacle_cloud"]
                self.assertFalse(payload["transform_available"])
                self.assertEqual(payload["frame_id"], "")
                self.assertEqual(payload["points"], [])
                self.assertIn(expected_error, payload["transform_error"])

    def test_zero_source_stamp_and_latest_lookup_failure_are_fail_closed(self) -> None:
        class CountingTfBuffer:
            def __init__(self) -> None:
                self.lookup_count = 0

            def lookup_transform(self, _target, _source, stamp):
                self.lookup_count += 1
                if stamp.nanoseconds:
                    raise ExtrapolationException("exact unavailable")
                raise LookupException("latest transform missing")

        tf_buffer = CountingTfBuffer()
        backend, _warnings = self._telemetry_backend(tf_buffer)
        backend.telemetry_tf_latest_fallback_tolerance_s = 0.075
        zero_stamp_cloud = self._single_point_cloud("lidar_link", 2.0, 0.0)
        zero_stamp_cloud.header.stamp = RosTime()

        UiBackendNode._on_telemetry_cloud(
            backend, "obstacles", zero_stamp_cloud
        )
        zero_payload = backend._telemetry["perception"]["obstacle_cloud"]
        self.assertFalse(zero_payload["transform_available"])
        self.assertIn("source timestamp is zero", zero_payload["transform_error"])
        self.assertEqual(tf_buffer.lookup_count, 0)

        UiBackendNode._on_telemetry_cloud(
            backend,
            "obstacles",
            self._single_point_cloud("lidar_link", 2.0, 0.0),
        )
        missing_payload = backend._telemetry["perception"]["obstacle_cloud"]
        self.assertFalse(missing_payload["transform_available"])
        self.assertIn("latest TF failed", missing_payload["transform_error"])
        self.assertIn("LookupException", missing_payload["transform_error"])
        self.assertEqual(tf_buffer.lookup_count, 2)

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

    def test_telemetry_lease_wakes_ros_executor_without_polling_delay(self) -> None:
        class Guard:
            def __init__(self) -> None:
                self.trigger_count = 0

            def trigger(self) -> None:
                self.trigger_count += 1

        guard = Guard()
        backend = SimpleNamespace(
            _lock=threading.Lock(),
            _normalize_telemetry_view=UiBackendNode._normalize_telemetry_view,
            enable_operator_telemetry=True,
            telemetry_session_timeout_s=12.0,
            _telemetry_session_deadline=0.0,
            _telemetry_requested_view="all",
            _telemetry_active_view="",
            _telemetry_capture_active=False,
            _telemetry_session_guard=guard,
        )

        UiBackendNode._request_telemetry_session(backend, True, "camera")

        # HH_260819 - Acquisition is event driven; the 1 Hz timer is only the
        # abandoned-lease expiry fallback on constrained ARM64 hardware.
        self.assertEqual(guard.trigger_count, 1)
        self.assertEqual(backend._telemetry_requested_view, "camera")

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

    def test_obstacle_box_callback_transforms_lidar_frame_to_map(self) -> None:
        class TfBuffer:
            @staticmethod
            def lookup_transform(target, source, _stamp):
                assert (target, source) == ("map", "lidar_link")
                yaw = math.radians(90.0)
                return SimpleNamespace(transform=SimpleNamespace(
                    translation=SimpleNamespace(x=10.0, y=20.0),
                    rotation=Quaternion(
                        z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
                    ),
                ))

        backend, warnings = self._telemetry_backend(TfBuffer())
        marker = Marker()
        marker.header.frame_id = "lidar_link"
        marker.header.stamp.sec = 100
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.ns = "bbox"
        marker.pose.position.x = 2.0
        marker.pose.position.y = 3.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2.0
        marker.scale.y = 1.0

        UiBackendNode._on_telemetry_obstacle_boxes(
            backend, MarkerArray(markers=[marker])
        )

        geometry = backend._telemetry["perception"]["obstacle_boxes"]
        self.assertEqual(geometry["frame_id"], "map")
        self.assertEqual(geometry["source_frame_id"], "lidar_link")
        self.assertEqual(geometry["target_frame_id"], "map")
        self.assertTrue(geometry["transform_available"])
        self.assertEqual(geometry["transform_mode"], "exact")
        self.assertEqual(geometry["transform_age_s"], 0.0)
        self.assertEqual(geometry["transform_warning"], "")
        self.assertEqual(geometry["transform_error"], "")
        self.assertEqual(
            geometry["polylines"][0]["points"][0], [7.5, 21.0]
        )
        self.assertEqual(warnings, [])

    def test_obstacle_box_future_extrapolation_uses_bounded_latest_tf(self) -> None:
        test_case = self

        class FutureTfBuffer:
            @staticmethod
            def lookup_transform(_target, _source, stamp):
                if stamp.nanoseconds:
                    raise ExtrapolationException("requested stamp is in the future")
                return test_case._planar_transform(stamp_s=99.95)

        backend, warnings = self._telemetry_backend(FutureTfBuffer())
        backend.telemetry_tf_latest_fallback_tolerance_s = 0.075
        marker = Marker()
        marker.header.frame_id = "lidar_link"
        marker.header.stamp.sec = 100
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.ns = "bbox"
        marker.pose.position.x = 2.0
        marker.pose.position.y = 3.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2.0
        marker.scale.y = 1.0

        UiBackendNode._on_telemetry_obstacle_boxes(
            backend, MarkerArray(markers=[marker])
        )

        geometry = backend._telemetry["perception"]["obstacle_boxes"]
        self.assertTrue(geometry["transform_available"])
        self.assertEqual(geometry["frame_id"], "map")
        self.assertEqual(geometry["transform_mode"], "latest_fallback")
        self.assertAlmostEqual(geometry["transform_age_s"], 0.05, places=6)
        self.assertIn("bounded latest TF", geometry["transform_warning"])
        self.assertEqual(geometry["transform_error"], "")
        self.assertEqual(
            geometry["polylines"][0]["points"][0], [7.5, 21.0]
        )
        self.assertEqual(warnings, [])

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

    def test_docking_rear_camera_subscription_is_explicitly_opt_in(self) -> None:
        source = (
            Path(__file__).resolve().parents[1]
            / "runtime"
            / "python"
            / "camrod_ui"
            / "ui_backend_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            '"telemetry_docking_rear_camera_fallback_enabled"', source
        )
        self.assertIn('wants("docking")', source)
        self.assertIn('self.telemetry_topics["rear_camera"]', source)
        self.assertIn('self.telemetry_topics["rear_camera_raw"]', source)

    def test_camera_raw_fallback_can_be_disabled_without_losing_jpeg_streams(
        self,
    ) -> None:
        class Logger:
            @staticmethod
            def info(_message) -> None:
                pass

        def subscribed_topics(raw_fallback_enabled: bool):
            subscriptions = []

            def create_subscription(message_type, topic, _callback, _qos):
                subscription = (message_type, topic)
                subscriptions.append(subscription)
                return subscription

            backend = SimpleNamespace(
                _telemetry_subscriptions=[],
                _normalize_telemetry_view=UiBackendNode._normalize_telemetry_view,
                _lock=threading.Lock(),
                _telemetry_capture_active=False,
                _telemetry_active_view="",
                _telemetry={"session_active": False, "active_view": ""},
                _reset_telemetry_source_timing_locked=lambda: None,
                telemetry_camera_raw_fallback_enabled=raw_fallback_enabled,
                telemetry_topics=TELEMETRY_TOPIC_DEFAULTS,
                create_subscription=create_subscription,
                get_logger=lambda: Logger(),
            )
            UiBackendNode._start_telemetry_subscriptions(backend, "camera")
            return subscriptions

        compressed_only = subscribed_topics(False)
        with_raw_fallback = subscribed_topics(True)

        self.assertEqual(
            compressed_only,
            [
                (CompressedImage, TELEMETRY_TOPIC_DEFAULTS["front_camera"]),
                (CompressedImage, TELEMETRY_TOPIC_DEFAULTS["rear_camera"]),
            ],
        )
        self.assertIn(
            (Image, TELEMETRY_TOPIC_DEFAULTS["front_camera_raw"]),
            with_raw_fallback,
        )
        self.assertIn(
            (Image, TELEMETRY_TOPIC_DEFAULTS["rear_camera_raw"]),
            with_raw_fallback,
        )

    def test_ui_launch_keeps_raw_camera_fallback_enabled_by_default(self) -> None:
        source = (
            Path(__file__).resolve().parents[1]
            / "camrod_ui_robot"
            / "launch"
            / "ui.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "'operator_telemetry_camera_raw_fallback_enabled'", source
        )
        self.assertIn("'telemetry_camera_raw_fallback_enabled'", source)
        tf_fallback_block = source.split(
            "'operator_telemetry_tf_latest_fallback_tolerance_s'", 1
        )[1].split("operator_telemetry_camera_raw_fallback_enabled_arg", 1)[0]
        self.assertIn("default_value='0.0'", tf_fallback_block)
        self.assertIn(
            "'telemetry_tf_latest_fallback_tolerance_s'", source
        )
        tf_transform_block = source.split(
            "'operator_telemetry_tf_transform_enabled'", 1
        )[1].split(
            "operator_telemetry_tf_latest_fallback_tolerance_s_arg", 1
        )[0]
        self.assertIn("default_value='false'", tf_transform_block)
        docking_fallback_block = source.split(
            "'operator_telemetry_docking_rear_camera_fallback_enabled'", 1
        )[1].split("enable_ui_guest_arg", 1)[0]
        self.assertIn("default_value='false'", docking_fallback_block)

    def test_raw_lidar_bbox_subscription_is_explicit_and_default_on(self) -> None:
        """CARLA can hide raw boxes without changing develop's UI default."""
        self.assertTrue(
            UiBackendNode._new_telemetry_snapshot()["options"][
                "raw_lidar_bbox_overlay_enabled"
            ]
        )

        class Logger:
            @staticmethod
            def info(_message) -> None:
                pass

        def subscribed_topics(raw_bbox_enabled: bool):
            subscriptions = []

            def create_subscription(message_type, topic, _callback, _qos):
                subscription = (message_type, topic)
                subscriptions.append(subscription)
                return subscription

            no_op = lambda *_args, **_kwargs: None
            backend = SimpleNamespace(
                _telemetry_subscriptions=[],
                _normalize_telemetry_view=UiBackendNode._normalize_telemetry_view,
                _lock=threading.Lock(),
                _telemetry_capture_active=False,
                _telemetry_active_view="",
                _telemetry={"session_active": False, "active_view": ""},
                _reset_telemetry_source_timing_locked=no_op,
                telemetry_raw_lidar_bbox_overlay_enabled=raw_bbox_enabled,
                telemetry_topics=TELEMETRY_TOPIC_DEFAULTS,
                create_subscription=create_subscription,
                _on_telemetry_map=no_op,
                _on_telemetry_footprint=no_op,
                _on_telemetry_robot_markers=no_op,
                _on_telemetry_obstacle_boxes=no_op,
                get_logger=lambda: Logger(),
            )
            UiBackendNode._start_telemetry_subscriptions(
                backend, "perception"
            )
            return subscriptions

        without_raw_boxes = subscribed_topics(False)
        with_raw_boxes = subscribed_topics(True)
        semantic = (
            PointCloud2, TELEMETRY_TOPIC_DEFAULTS["obstacle_cloud"]
        )
        raw_boxes = (
            MarkerArray, TELEMETRY_TOPIC_DEFAULTS["obstacle_boxes"]
        )

        self.assertIn(semantic, without_raw_boxes)
        self.assertIn(semantic, with_raw_boxes)
        self.assertNotIn(raw_boxes, without_raw_boxes)
        self.assertIn(raw_boxes, with_raw_boxes)

        launch_source = (
            Path(__file__).resolve().parents[1]
            / "camrod_ui_robot"
            / "launch"
            / "ui.launch.py"
        ).read_text(encoding="utf-8")
        raw_bbox_block = launch_source.split(
            "'operator_telemetry_raw_lidar_bbox_overlay_enabled'", 1
        )[1].split(
            "operator_telemetry_tf_latest_fallback_tolerance_s_arg", 1
        )[0]
        self.assertIn("default_value='true'", raw_bbox_block)
        self.assertIn(
            "'telemetry_raw_lidar_bbox_overlay_enabled'", launch_source
        )

    def test_proximity_tab_retains_only_authoritative_radar_cost_evidence(self) -> None:
        snapshot = UiBackendNode._new_telemetry_snapshot()
        snapshot["active_view"] = "proximity"
        snapshot["safety"]["radar_evidence"] = (
            "active SENSOR=LEFT2 range_m=0.090 cost=95;"
        )
        snapshot["safety"]["obstacle_replan"] = "heavy unrelated detail"
        snapshot["safety"]["gate"] = {"operating_state": "COST_STOP"}

        pruned = UiBackendNode._prune_telemetry_snapshot(snapshot, "proximity")

        self.assertEqual(
            pruned["safety"]["radar_evidence"],
            "active SENSOR=LEFT2 range_m=0.090 cost=95;",
        )
        self.assertEqual(pruned["safety"]["obstacle_replan"], "")
        self.assertEqual(pruned["safety"]["gate"], {})

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
