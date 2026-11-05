#!/usr/bin/env python3
"""B11-B13 return via the shared turnaround area before the drop zone."""

# HJ_261105 - Only the roadside forward-loop request is redirected to the area;
# the request sent after the area turnaround goes to the drop zone.

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from types import MethodType, SimpleNamespace
import unittest

from avg_msgs.msg import PlanningRecallRequest
import yaml


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
SITES_YAML = Path(__file__).resolve().parents[1] / "config" / "camping_sites.yaml"


def _node() -> SimpleNamespace:
    node = SimpleNamespace(
        keypoints={},
        turnaround_area_for_site={},
        return_mission_key="drop_zone",
        scenario_id=PlanningStateMachineNode.SCENARIO_UNLOAD_WAIT,
        SCENARIO_RETURN_TO_DROP_ZONE=PlanningStateMachineNode.SCENARIO_RETURN_TO_DROP_ZONE,
        SCENARIO_WAIT_DROP_ZONE=PlanningStateMachineNode.SCENARIO_WAIT_DROP_ZONE,
        active_mission_key="camping_site_12",
        return_requested=False,
        warn_goal_sent=True,
        published=[],
        scenarios=[],
    )
    for name in ("_merge_turnaround_areas", "_is_turnaround_area_key",
                 "_turnaround_area_for_return", "_on_return_to_drop_zone"):
        setattr(node, name, MethodType(getattr(PlanningStateMachineNode, name), node))
    node._return_keypoint_reached = lambda: False
    node._drop_zone_arrived_with_condition = lambda: False
    node._clear_maneuver_phase_override = lambda _reason: None
    node.get_logger = lambda: SimpleNamespace(info=lambda *_: None)

    def publish(key, source, force=False):
        node.published.append((key, source))
        node.active_mission_key = key
        return True

    def set_scenario(scenario, reason):
        node.scenario_id = scenario
        node.scenarios.append((scenario, reason))

    node._publish_auto_goal = publish
    node._set_scenario = set_scenario
    node._merge_turnaround_areas(yaml.safe_load(SITES_YAML.read_text()))
    return node


def _request(site: str, source: str) -> PlanningRecallRequest:
    message = PlanningRecallRequest()
    message.site_name = site
    message.source = source
    return message


class TurnaroundAreaReturnTest(unittest.TestCase):
    def test_area_serves_b11_to_b13_with_in_area_service_pose(self) -> None:
        node = _node()
        self.assertEqual(
            node.turnaround_area_for_site,
            {f"camping_site_{i}": "turnaround_area" for i in (11, 12, 13)},
        )
        area = node.keypoints["turnaround_area"]
        self.assertAlmostEqual(area.x, 23.843)
        self.assertAlmostEqual(area.y, -14.215)
        self.assertAlmostEqual(area.yaw_deg, -51.2)

    def test_roadside_forward_request_goes_to_area_then_drop_zone(self) -> None:
        node = _node()
        node._on_return_to_drop_zone(_request(
            "camping_site_12",
            "camping_site_maneuver_controller:done_roadside_forward_live_lanelet"))
        self.assertEqual(node.published, [("turnaround_area", "turnaround_area_return")])
        self.assertEqual(node.scenario_id, PlanningStateMachineNode.SCENARIO_RETURN_TO_DROP_ZONE)

        # Controller retries while the area goal is active must not re-publish.
        node._on_return_to_drop_zone(_request(
            "camping_site_12",
            "camping_site_maneuver_controller:done_roadside_forward_retry"))
        self.assertEqual(len(node.published), 1)

        node._on_return_to_drop_zone(_request(
            "turnaround_area", "camping_site_maneuver_controller:done_live_lanelet"))
        self.assertEqual(node.published[-1], ("drop_zone", "return_request"))

    def test_battery_urgent_roadside_return_also_uses_area(self) -> None:
        node = _node()
        node._on_return_to_drop_zone(_request(
            "camping_site_13",
            "camping_site_maneuver_controller:battery_urgent_return:"
            "battery_urgent_roadside_forward"))
        self.assertEqual(node.published[-1][0], "turnaround_area")

    def test_turnaround_sites_and_unserved_sites_go_to_drop_zone(self) -> None:
        node = _node()
        node._on_return_to_drop_zone(_request(
            "camping_site_4", "camping_site_maneuver_controller:done_current_pose_after_lateral_exit"))
        node._on_return_to_drop_zone(_request(
            "camping_site_1", "camping_site_maneuver_controller:done_roadside_forward_retry"))
        self.assertEqual([key for key, _ in node.published], ["drop_zone", "drop_zone"])


if __name__ == "__main__":
    unittest.main()
