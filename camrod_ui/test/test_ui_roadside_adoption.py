"""Fail-closed UI adoption coverage for B11-B13 roadside service poses."""

import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest

from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import PoseStamped

sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "runtime" / "python"),
)

from avg_msgs.msg import AvgPoseStamped  # noqa: E402
from camrod_ui.ui_backend_node import (  # noqa: E402
    MAX_PENDING_SITE_ROUTE_GOALS,
    MissionKeypoint,
    UiBackendNode,
)


def _route_pose(
    x: float,
    y: float,
    yaw_rad: float = 0.0,
    *,
    frame_id: str = "map",
    stamp: tuple[int, int] = (10, 20),
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = RosTime(sec=stamp[0], nanosec=stamp[1])
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(0.5 * yaw_rad)
    pose.pose.orientation.w = math.cos(0.5 * yaw_rad)
    return pose


def _avg_pose(
    x: float,
    y: float,
    yaw_rad: float = 0.0,
    *,
    frame_id: str = "map",
) -> AvgPoseStamped:
    pose = AvgPoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(0.5 * yaw_rad)
    pose.pose.orientation.w = math.cos(0.5 * yaw_rad)
    return pose


class _Logger:

    def __init__(self) -> None:
        self.info_messages = []

    def info(self, message: str) -> None:
        self.info_messages.append(message)


class RoadsideArrivalContractTest(unittest.TestCase):

    @staticmethod
    def _backend(
        mission_key: str,
        keypoint: MissionKeypoint,
        current_x: float,
        current_y: float,
        *,
        route: PoseStamped | None = None,
        lanelet: AvgPoseStamped | None = None,
        now_s: float = 100.0,
        lanelet_time_s: float = 100.0,
    ) -> SimpleNamespace:
        route = route or _route_pose(0.0, 0.0)
        lanelet = lanelet or _avg_pose(0.0, 0.0)
        backend = SimpleNamespace(
            _lock=threading.Lock(),
            _site_route_anchors={mission_key: route},
            _latest_planning_lanelet_pose=lanelet,
            _latest_planning_lanelet_pose_time_s=lanelet_time_s,
            _latest_arrival_pose=_avg_pose(current_x, current_y),
            _latest_arrival_pose_time_s=now_s,
            _keypoints_by_mission_key={mission_key: keypoint},
            immediate_site_arrival_enabled=True,
            site_arrival_pose_timeout_s=2.0,
            site_arrival_center_radius_m=2.5,
            site_arrival_roadside_offset_m=0.30,
            site_arrival_roadside_lateral_tolerance_m=0.15,
            site_arrival_roadside_forward_tolerance_m=0.60,
            default_goal_frame_id="map",
        )
        backend._now_s = lambda: now_s
        backend._resolve_mission_key_for_site = lambda _site: mission_key
        backend._finite_planar_pose = UiBackendNode._finite_planar_pose
        backend._planar_pose_yaw = UiBackendNode._planar_pose_yaw
        backend._roadside_operational_arrival_match = (
            lambda key, point, x, y, frame: (
                UiBackendNode._roadside_operational_arrival_match(
                    backend, key, point, x, y, frame
                )
            )
        )
        backend._site_arrival_match = lambda site: UiBackendNode._site_arrival_match(
            backend, site
        )
        return backend

    def test_b11_b12_and_b13_operational_offsets_match_both_sides(self) -> None:
        fixtures = (
            # Active-map GoalSnapper outputs and authored site centers. B11's
            # test pose also locks the inclusive 0.60 m forward and 0.15 m
            # lateral early-handoff bounds.
            (
                "camping_site_11",
                (10.821530478, 20.516472190, -68.783648),
                (14.8445, 22.0782),
                (0.60, 0.15),
            ),
            (
                "camping_site_12",
                (10.103421888, 23.093735129, -78.439719),
                (6.85414, 22.4291),
                (0.00, -0.30),
            ),
            # B13's authored center is 9.018 m laterally from the snap. The
            # operational target must still cap at -0.30 m.
            (
                "camping_site_13",
                (9.626348588, 27.418616995, -91.177942),
                (0.610449, 27.604),
                (0.00, -0.30),
            ),
        )
        for mission_key, anchor, site, current_axes in fixtures:
            with self.subTest(mission_key=mission_key):
                anchor_x, anchor_y, yaw_deg = anchor
                yaw_rad = math.radians(yaw_deg)
                forward, lateral = current_axes
                current_x = (
                    anchor_x
                    + math.cos(yaw_rad) * forward
                    - math.sin(yaw_rad) * lateral
                )
                current_y = (
                    anchor_y
                    + math.sin(yaw_rad) * forward
                    + math.cos(yaw_rad) * lateral
                )
                keypoint = MissionKeypoint(
                    key=mission_key,
                    frame_id="map",
                    x=site[0],
                    y=site[1],
                    z=0.0,
                    yaw_deg=0.0,
                    service_mode="roadside_stop",
                )
                backend = self._backend(
                    mission_key,
                    keypoint,
                    current_x,
                    current_y,
                    route=_route_pose(anchor_x, anchor_y, yaw_rad),
                    lanelet=_avg_pose(anchor_x, anchor_y, yaw_rad),
                )

                matched, key, _, reason = UiBackendNode._site_arrival_match(
                    backend, mission_key
                )

                self.assertTrue(matched)
                self.assertEqual(key, mission_key)
                self.assertEqual(reason, "near_roadside_operational_target")

    def test_lanelet_snap_and_wrong_crab_side_do_not_match(self) -> None:
        key = "camping_site_11"
        keypoint = MissionKeypoint(
            key=key,
            frame_id="map",
            x=0.0,
            y=3.0,
            z=0.0,
            yaw_deg=0.0,
            service_mode="roadside_stop",
            corners=[(-1.0, -1.0), (1.0, -1.0), (1.0, 4.0), (-1.0, 4.0)],
        )
        for current_y in (0.0, -0.30):
            with self.subTest(current_y=current_y):
                backend = self._backend(key, keypoint, 0.0, current_y)
                matched, _, _, reason = UiBackendNode._site_arrival_match(
                    backend, "B11"
                )
                self.assertFalse(matched)
                self.assertEqual(reason, "outside_roadside_operational_target")

    def test_roadside_never_falls_back_to_polygon_or_center(self) -> None:
        key = "camping_site_11"
        keypoint = MissionKeypoint(
            key=key,
            frame_id="map",
            x=0.0,
            y=0.0,
            z=0.0,
            yaw_deg=0.0,
            service_mode="roadside_stop",
            corners=[(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)],
        )
        backend = self._backend(key, keypoint, 0.0, 0.0)
        backend._site_route_anchors = {}

        matched, _, _, reason = UiBackendNode._site_arrival_match(
            backend, "B11"
        )

        self.assertFalse(matched)
        self.assertEqual(reason, "missing_mission_route_anchor")

    def test_wrong_mission_stale_or_disagreeing_lanelet_fails_closed(self) -> None:
        key = "camping_site_12"
        keypoint = MissionKeypoint(
            key=key,
            frame_id="map",
            x=0.0,
            y=-4.0,
            z=0.0,
            yaw_deg=0.0,
            service_mode="roadside_stop",
        )
        backend = self._backend(key, keypoint, 0.0, -0.30)

        backend._site_route_anchors = {"camping_site_11": _route_pose(0.0, 0.0)}
        self.assertEqual(
            UiBackendNode._site_arrival_match(backend, "B12")[3],
            "missing_mission_route_anchor",
        )

        backend._site_route_anchors = {key: _route_pose(0.0, 0.0)}
        backend._latest_planning_lanelet_pose_time_s = 97.0
        self.assertTrue(
            UiBackendNode._site_arrival_match(backend, "B12")[3].startswith(
                "stale_lanelet_anchor:"
            )
        )

        backend._latest_planning_lanelet_pose_time_s = 100.0
        backend._latest_planning_lanelet_pose = _avg_pose(0.61, 0.0)
        self.assertEqual(
            UiBackendNode._site_arrival_match(backend, "B12")[3],
            "lanelet_anchor_disagrees_with_route",
        )

        backend._latest_planning_lanelet_pose = _avg_pose(0.0, 0.0)
        backend._latest_planning_lanelet_pose_time_s = 101.0
        self.assertEqual(
            UiBackendNode._site_arrival_match(backend, "B12")[3],
            "invalid_lanelet_anchor_freshness",
        )

        backend._latest_planning_lanelet_pose_time_s = 100.0
        backend.site_arrival_pose_timeout_s = 0.0
        self.assertEqual(
            UiBackendNode._site_arrival_match(backend, "B12")[3],
            "invalid_lanelet_anchor_freshness",
        )

    def test_invalid_route_quaternion_and_frame_fail_closed(self) -> None:
        key = "camping_site_13"
        keypoint = MissionKeypoint(
            key=key,
            frame_id="map",
            x=0.0,
            y=9.0,
            z=0.0,
            yaw_deg=0.0,
            service_mode="roadside_stop",
        )
        invalid_route = _route_pose(0.0, 0.0)
        invalid_route.pose.orientation.z = 0.0
        invalid_route.pose.orientation.w = 0.0
        backend = self._backend(
            key, keypoint, 0.0, 0.30, route=invalid_route
        )
        self.assertEqual(
            UiBackendNode._site_arrival_match(backend, "B13")[3],
            "invalid_roadside_route_yaw",
        )

        backend._site_route_anchors = {
            key: _route_pose(0.0, 0.0, frame_id="odom")
        }
        self.assertEqual(
            UiBackendNode._site_arrival_match(backend, "B13")[3],
            "roadside_frame_mismatch",
        )

    def test_stamp_correlates_route_anchor_to_exact_mission_and_is_bounded(self) -> None:
        backend = SimpleNamespace(
            _lock=threading.Lock(),
            _pending_site_route_goal_stamps={},
            _site_route_anchors={},
            _runtime_policy=SimpleNamespace(update_goal_received=lambda: None),
            logger=_Logger(),
        )
        backend.get_logger = lambda: backend.logger
        backend._route_goal_stamp_key = UiBackendNode._route_goal_stamp_key
        backend._update_runtime_state = lambda callback: callback()

        for index in range(MAX_PENDING_SITE_ROUTE_GOALS + 5):
            raw = _route_pose(1.0, 2.0, stamp=(20, index))
            UiBackendNode._remember_pending_site_route_goal(
                backend, raw, f"camping_site_{index}"
            )
        self.assertEqual(
            len(backend._pending_site_route_goal_stamps),
            MAX_PENDING_SITE_ROUTE_GOALS,
        )

        ambiguous = _route_pose(1.0, 2.0, stamp=(29, 39))
        UiBackendNode._remember_pending_site_route_goal(
            backend, ambiguous, "camping_site_11"
        )
        UiBackendNode._remember_pending_site_route_goal(
            backend, ambiguous, "camping_site_12"
        )
        UiBackendNode._on_planning_route_goal(backend, ambiguous)
        self.assertNotIn("camping_site_11", backend._site_route_anchors)
        self.assertNotIn("camping_site_12", backend._site_route_anchors)

        raw = _route_pose(1.0, 2.0, stamp=(30, 40))
        UiBackendNode._remember_pending_site_route_goal(
            backend, raw, "camping_site_13"
        )
        snapped = _route_pose(4.0, 5.0, yaw_rad=0.3, stamp=(30, 40))
        UiBackendNode._on_planning_route_goal(backend, snapped)
        self.assertEqual(
            backend._site_route_anchors["camping_site_13"].pose.position.x,
            4.0,
        )

        manual = _route_pose(99.0, 99.0, stamp=(31, 41))
        UiBackendNode._on_planning_route_goal(backend, manual)
        self.assertEqual(
            backend._site_route_anchors["camping_site_13"].pose.position.x,
            4.0,
        )

    def test_matched_reselection_publishes_adopt_without_new_nav_goal(self) -> None:
        key = "camping_site_11"
        keypoint = MissionKeypoint(
            key=key,
            frame_id="map",
            x=0.0,
            y=3.0,
            z=0.0,
            yaw_deg=0.0,
            service_mode="roadside_stop",
        )
        backend = self._backend(key, keypoint, 0.0, 0.30)
        events = []
        backend._is_site_occupied = lambda _site: False
        backend._service_metrics = None
        backend.publish_mission_engage_from_destination = False
        backend._publish_camping_site_maneuver_controller_adopt = (
            lambda site, mission, source: events.append(
                ("adopt", site, mission, source)
            )
        )
        backend._publish_service_state = (
            lambda state, source: events.append(("state", state, source))
        )
        backend._notify_site_arrival = lambda *args, **kwargs: events.append(
            ("notify", args, kwargs)
        )
        backend._publish_engage = lambda enabled, source: events.append(
            ("engage", enabled, source)
        )

        result = UiBackendNode._apply_destination_command(
            backend, "B11", True, "test"
        )

        self.assertFalse(result["goal_pose_published"])
        self.assertIn("arrival adopted", result["message"])
        self.assertEqual(events[0][:3], ("adopt", "B11", key))
        self.assertFalse(
            any(event[0] == "engage" and event[1] for event in events)
        )

    def test_source_and_launch_keep_the_full_roadside_contract(self) -> None:
        package_root = Path(__file__).resolve().parents[1]
        source = (
            package_root / "runtime/python/camrod_ui/ui_backend_node.py"
        ).read_text(encoding="utf-8")
        launch = (
            package_root / "camrod_ui_robot/launch/ui.launch.py"
        ).read_text(encoding="utf-8")

        assert "self.sub_planning_lanelet_pose = self.create_subscription(" in source
        assert "_remember_pending_site_route_goal(pose, mission_key)" in source
        assert "'planning_lanelet_pose_topic': '/planning/lanelet_pose'" in launch
        assert "'site_arrival_roadside_offset_m': 0.30" in launch
        assert "'site_arrival_roadside_lateral_tolerance_m': 0.15" in launch
        assert "'site_arrival_roadside_forward_tolerance_m': 0.60" in launch

if __name__ == "__main__":
    unittest.main()
