#!/usr/bin/env python3
"""Guest recall keeps site identity and never selects an authored site pose."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

from avg_msgs.msg import PlanningRecallRequest
from builtin_interfaces.msg import Time as RosTime
from rclpy.time import Time


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "planning_state_machine_node.py"
)
SPEC = importlib.util.spec_from_file_location("planning_state_machine_node", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
PlanningStateMachineNode = MODULE.PlanningStateMachineNode


class _RecordingPublisher:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


def _policy_fixture() -> SimpleNamespace:
    node = SimpleNamespace(
        site_mission_key_prefix="camping_site_",
        keypoints={
            **{f"camping_site_{index}": object() for index in range(1, 14)},
            # Legacy points must never win: they stop on the road centerline and
            # erase the site key needed for the bounded control-side wait pose.
            **{f"camping_site_{index}_road": object() for index in range(1, 14)},
        },
        default_recall_mission_key="camping_site_1",
        SCENARIO_RECALL_TO_SITE=PlanningStateMachineNode.SCENARIO_RECALL_TO_SITE,
        scenario_id=PlanningStateMachineNode.SCENARIO_RECALL_TO_SITE,
        recall_target_key="camping_site_1",
    )
    node._is_site_key = lambda key: PlanningStateMachineNode._is_site_key(node, key)
    node._default_site_key = lambda: PlanningStateMachineNode._default_site_key(node)
    node._resolve_recall_target_key = (
        lambda site: PlanningStateMachineNode._resolve_recall_target_key(node, site)
    )
    return node


class RecallTargetPolicyTest(unittest.TestCase):
    def test_return_handoff_margin_still_requires_fresh_nav2_success(self) -> None:
        now = Time(seconds=100)
        node = SimpleNamespace(
            require_nav2_success_for_goal_reached=True,
            _nav2_goal_succeeded=True,
            _nav2_terminal_time=Time(seconds=99),
            nav_success_latch_s=15.0,
        )
        node.get_clock = lambda: SimpleNamespace(now=lambda: now)
        node._active_goal_distance = lambda: 0.338

        self.assertFalse(PlanningStateMachineNode._goal_reached(node, 0.30))
        self.assertTrue(PlanningStateMachineNode._goal_reached(node, 0.35))

        node._active_goal_distance = lambda: 0.351
        self.assertFalse(PlanningStateMachineNode._goal_reached(node, 0.35))
        node._active_goal_distance = lambda: 0.338
        node._nav2_goal_succeeded = False
        self.assertFalse(PlanningStateMachineNode._goal_reached(node, 0.35))
        node._nav2_goal_succeeded = True
        node._nav2_terminal_time = Time(seconds=80)
        self.assertFalse(PlanningStateMachineNode._goal_reached(node, 0.35))

    def test_initial_drop_zone_raw_goal_uses_exact_auto_goal_stamp(self) -> None:
        auto_publisher = _RecordingPublisher()
        raw_publisher = _RecordingPublisher()
        fixed_now = Time(seconds=123, nanoseconds=456789)
        keypoint = MODULE.Keypoint(
            name="drop_zone",
            frame_id="map",
            x=1.25,
            y=-2.5,
            z=0.0,
            yaw_deg=90.0,
            source_id="drop_zone_1",
        )
        logger = SimpleNamespace(info=lambda *_: None, warn=lambda *_: None, error=lambda *_: None)
        node = SimpleNamespace(
            return_mission_key="drop_zone",
            pub_auto_goal_snapper=auto_publisher,
            pub_drop_zone_goal_raw=raw_publisher,
            pub_drop_zone_goal_raw_ros=None,
            auto_goal_snapper_input_topic="/planning/auto_goal_raw",
            min_goal_publish_interval_s=0.0,
            _last_goal_publish_time=Time(seconds=1),
            startup_goal_sent=False,
            _goal_reached_since=None,
            _goal_reached_latched=True,
            _drop_zone_arrival_notified=True,
        )
        node.get_clock = lambda: SimpleNamespace(now=lambda: fixed_now)
        node.get_logger = lambda: logger
        node._return_start_pose_ready = lambda: True
        node._select_nearest_return_keypoint_for_current_pose = lambda _source: None
        node._goal_keypoint = lambda _key: keypoint
        node._reset_nav2_goal_status = lambda: None
        node._publish_drop_zone_goal_raw_for_keypoint = (
            lambda target, stamp=None: (
                PlanningStateMachineNode._publish_drop_zone_goal_raw_for_keypoint(
                    node, target, stamp
                )
            )
        )

        self.assertTrue(
            PlanningStateMachineNode._publish_auto_goal(
                node, "drop_zone", "return_request", force=True
            )
        )
        self.assertEqual(len(auto_publisher.messages), 1)
        self.assertEqual(len(raw_publisher.messages), 1)
        auto_stamp = auto_publisher.messages[0].header.stamp
        raw_stamp = raw_publisher.messages[0].header.stamp
        self.assertEqual(
            (raw_stamp.sec, raw_stamp.nanosec),
            (auto_stamp.sec, auto_stamp.nanosec),
        )

    def test_recall_auto_goal_preserves_valid_request_stamp(self) -> None:
        auto_publisher = _RecordingPublisher()
        fixed_now = Time(seconds=123, nanoseconds=456789)
        keypoint = MODULE.Keypoint(
            name="camping_site_4",
            frame_id="map",
            x=19.31,
            y=-3.27,
            z=0.0,
            yaw_deg=-67.4,
        )
        node = SimpleNamespace(
            return_mission_key="drop_zone",
            pub_auto_goal_snapper=auto_publisher,
            auto_goal_snapper_input_topic="/planning/auto_goal_raw",
            min_goal_publish_interval_s=0.0,
            _last_goal_publish_time=Time(seconds=1),
            startup_goal_sent=False,
            _goal_reached_since=None,
            _goal_reached_latched=True,
            _drop_zone_arrival_notified=True,
        )
        node.get_clock = lambda: SimpleNamespace(now=lambda: fixed_now)
        node.get_logger = lambda: SimpleNamespace(
            info=lambda *_: None,
            warn=lambda *_: None,
            error=lambda *_: None,
        )
        node._goal_keypoint = lambda _key: keypoint
        node._reset_nav2_goal_status = lambda: None

        request_stamp = RosTime(sec=41, nanosec=73)
        self.assertTrue(
            PlanningStateMachineNode._publish_auto_goal(
                node,
                "camping_site_4",
                "recall:camping_site_4",
                force=True,
                correlation_stamp=request_stamp,
            )
        )
        emitted = auto_publisher.messages[0].header.stamp
        self.assertEqual((emitted.sec, emitted.nanosec), (41, 73))

        self.assertTrue(
            PlanningStateMachineNode._publish_auto_goal(
                node,
                "camping_site_4",
                "legacy_recall",
                force=True,
                correlation_stamp=RosTime(),
            )
        )
        fallback = auto_publisher.messages[1].header.stamp
        now_msg = fixed_now.to_msg()
        self.assertEqual(
            (fallback.sec, fallback.nanosec),
            (now_msg.sec, now_msg.nanosec),
        )

    def test_b1_through_b13_resolve_to_semantic_site_not_legacy_road_key(self) -> None:
        node = _policy_fixture()
        for index in range(1, 14):
            expected = f"camping_site_{index}"
            self.assertEqual(
                PlanningStateMachineNode._resolve_recall_target_key(
                    node, f"B{index}"
                ),
                expected,
            )
            self.assertEqual(
                PlanningStateMachineNode._resolve_recall_target_key(node, expected),
                expected,
            )

    def test_unknown_recall_fails_to_configured_default(self) -> None:
        node = _policy_fixture()
        self.assertEqual(
            PlanningStateMachineNode._resolve_recall_target_key(node, "B99"),
            "camping_site_1",
        )
        self.assertEqual(
            PlanningStateMachineNode._resolve_recall_target_key(node, ""),
            "camping_site_1",
        )

    def test_async_snapped_goal_echo_keeps_recall_scenario(self) -> None:
        node = _policy_fixture()
        node.recall_target_key = "camping_site_7"
        self.assertTrue(
            PlanningStateMachineNode._goal_update_belongs_to_active_recall(
                node, "camping_site_7"
            )
        )
        self.assertFalse(
            PlanningStateMachineNode._goal_update_belongs_to_active_recall(
                node, "camping_site_8"
            )
        )
        node.scenario_id = PlanningStateMachineNode.SCENARIO_DELIVERY_TO_SITE
        self.assertFalse(
            PlanningStateMachineNode._goal_update_belongs_to_active_recall(
                node, "camping_site_7"
            )
        )

    def test_recall_request_publishes_site_key_under_recall_scenario(self) -> None:
        node = _policy_fixture()
        events: list[tuple[object, ...]] = []
        node.recall_target_key = ""
        node.recall_site_name = ""
        node.last_recalled_mission_key = ""
        node.recall_requested = False
        node.warn_goal_sent = True
        node._publish_auto_goal = (
            lambda key, source, force=False, correlation_stamp=None: events.append(
                (
                    "goal",
                    key,
                    source,
                    force,
                    correlation_stamp.sec,
                    correlation_stamp.nanosec,
                )
            )
            or True
        )
        node._set_scenario = (
            lambda scenario, reason: events.append(("scenario", scenario, reason))
        )

        request = PlanningRecallRequest()
        request.header.stamp = RosTime(sec=41, nanosec=73)
        request.site_name = "B13"
        request.source = "guest:B13"
        PlanningStateMachineNode._on_camping_site_recall(node, request)

        self.assertEqual(node.recall_target_key, "camping_site_13")
        self.assertEqual(node.last_recalled_mission_key, "B13")
        self.assertFalse(node.recall_requested)
        self.assertFalse(node.warn_goal_sent)
        self.assertEqual(
            events,
            [
                (
                    "scenario",
                    PlanningStateMachineNode.SCENARIO_RECALL_TO_SITE,
                    "recall_topic",
                ),
                ("goal", "camping_site_13", "recall:B13", True, 41, 73),
            ],
        )

    def test_recall_intent_is_latched_before_publish_and_retries_on_failure(self) -> None:
        node = _policy_fixture()
        events: list[tuple[object, ...]] = []
        node.recall_site_name = ""
        node.last_recalled_mission_key = ""
        node.recall_requested = False
        node.warn_goal_sent = True
        node._set_scenario = (
            lambda scenario, reason: events.append(("scenario", scenario, reason))
        )
        node._publish_auto_goal = (
            lambda key, source, force=False, correlation_stamp=None: events.append(
                ("goal", key, source, force, correlation_stamp)
            )
            or False
        )
        request = PlanningRecallRequest()
        request.site_name = "B4"

        PlanningStateMachineNode._on_camping_site_recall(node, request)

        self.assertTrue(node.recall_requested)
        self.assertEqual(node.recall_target_key, "camping_site_4")
        self.assertEqual(events[0][0], "scenario")
        self.assertEqual(events[1][0], "goal")


if __name__ == "__main__":
    unittest.main()
