"""Unit tests for atomic manual/regulated Nav2 selector policy."""

import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "nav2_selector_latch_node.py"
)
SPEC = importlib.util.spec_from_file_location("nav2_selector_latch_node", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LANELET_LAUNCH = PACKAGE_ROOT / "launch" / "nav2_lanelet.launch.py"
LANELET_SPEC = importlib.util.spec_from_file_location(
    "nav2_lanelet_launch", LANELET_LAUNCH
)
assert LANELET_SPEC is not None and LANELET_SPEC.loader is not None
LANELET_MODULE = importlib.util.module_from_spec(LANELET_SPEC)
LANELET_SPEC.loader.exec_module(LANELET_MODULE)


class Nav2SelectorPolicyTest(unittest.TestCase):
    """Checks that source changes select all three Nav2 policy IDs together."""

    def test_manual_default_is_long_range_lanelet_planner(self) -> None:
        self.assertEqual(MODULE.DEFAULT_MANUAL_PLANNER_ID, "LaneletRoute")

    def test_executable_fallbacks_match_develop_defaults(self) -> None:
        self.assertEqual(MODULE.DEFAULT_REGULATED_PLANNER_ID, "NavFn")
        self.assertEqual(MODULE.DEFAULT_REGULATED_CONTROLLER_ID, "MPPI")

    def test_manual_source_selects_manual_policy_as_one_tuple(self) -> None:
        source, recognized = MODULE.resolve_goal_source(" manual:rviz ")
        selected = MODULE.selector_ids_for_source(
            source,
            regulated=("LaneletRoute", "MPPI", "goal_checker"),
            manual=("LaneletRoute", "RotationShim", "manual_goal_checker"),
        )

        self.assertTrue(recognized)
        self.assertEqual(source, "manual")
        self.assertEqual(
            selected, ("LaneletRoute", "RotationShim", "manual_goal_checker")
        )

    def test_regulated_source_selects_regulated_policy_as_one_tuple(self) -> None:
        source, recognized = MODULE.resolve_goal_source("regulated:ui")
        selected = MODULE.selector_ids_for_source(
            source,
            regulated=("SmacLattice", "MPPI", "goal_checker"),
            manual=("LaneletRoute", "RotationShim", "manual_goal_checker"),
        )

        self.assertTrue(recognized)
        self.assertEqual(source, "regulated")
        self.assertEqual(selected, ("SmacLattice", "MPPI", "goal_checker"))

    def test_regulated_reverse_selects_only_the_reverse_controller(self) -> None:
        source, recognized = MODULE.resolve_goal_source("regulated_reverse:return")
        selected = MODULE.selector_ids_for_source(
            source,
            regulated=("LaneletRoute", "RPP", "goal_checker"),
            manual=("LaneletRoute", "RotationShim", "manual_goal_checker"),
            regulated_reverse=("LaneletRoute", "RPPReverse", "goal_checker"),
        )

        self.assertTrue(recognized)
        self.assertEqual(source, "regulated_reverse")
        self.assertEqual(selected, ("LaneletRoute", "RPPReverse", "goal_checker"))

    def test_unknown_source_fails_closed_to_regulated_policy(self) -> None:
        source, recognized = MODULE.resolve_goal_source("unexpected")
        selected = MODULE.selector_ids_for_source(
            source,
            regulated=("LaneletRoute", "MPPI", "goal_checker"),
            manual=("LaneletRoute", "RotationShim", "manual_goal_checker"),
        )

        self.assertFalse(recognized)
        self.assertEqual(source, "regulated")
        self.assertEqual(selected, ("LaneletRoute", "MPPI", "goal_checker"))

    def test_multi_pose_tree_uses_production_selector_ids(self) -> None:
        tree_path = (
            PACKAGE_ROOT
            / "config"
            / "bt"
            / "navigate_through_poses_w_planner_selector.xml"
        )
        root = ET.parse(tree_path).getroot()
        planners = root.findall(".//PlannerSelector")
        controllers = root.findall(".//ControllerSelector")
        goal_checkers = root.findall(".//GoalCheckerSelector")
        computes = root.findall(".//ComputePathThroughPoses")
        follows = root.findall(".//FollowPath")
        remove_passed = root.find(".//RemovePassedGoals")

        self.assertEqual(len(planners), 1)
        self.assertEqual(len(controllers), 1)
        self.assertEqual(len(goal_checkers), 1)
        self.assertEqual(
            planners[0].attrib,
            {
                "selected_planner": "{selected_planner}",
                "default_planner": "LaneletRoute",
                "topic_name": "/planning/planner_selector_ros",
            },
        )
        self.assertEqual(
            controllers[0].attrib,
            {
                "selected_controller": "{selected_controller}",
                "default_controller": "RPP",
                "topic_name": "/planning/controller_selector_ros",
            },
        )
        self.assertEqual(
            goal_checkers[0].attrib,
            {
                "selected_goal_checker": "{selected_goal_checker}",
                "default_goal_checker": "goal_checker",
                "topic_name": "/planning/goal_checker_selector_ros",
            },
        )
        self.assertTrue(computes)
        self.assertTrue(follows)
        self.assertTrue(
            all(node.attrib["planner_id"] == "{selected_planner}" for node in computes)
        )
        self.assertTrue(
            all(
                node.attrib["controller_id"] == "{selected_controller}"
                and node.attrib["goal_checker_id"] == "{selected_goal_checker}"
                for node in follows
            )
        )
        self.assertIsNotNone(remove_passed)
        self.assertEqual(remove_passed.attrib["input_goals"], "{goals}")
        self.assertEqual(remove_passed.attrib["output_goals"], "{goals}")
        self.assertEqual(remove_passed.attrib["global_frame"], "map")
        self.assertEqual(
            remove_passed.attrib["robot_base_frame"], "robot_center_link"
        )

        behavior = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "nav2_behavior.yaml").read_text(
                encoding="utf-8"
            )
        )
        plugin_lib_names = set(
            behavior["bt_navigator"]["ros__parameters"]["plugin_lib_names"]
        )
        self.assertTrue(
            {
                "nav2_compute_path_through_poses_action_bt_node",
                "nav2_follow_path_action_bt_node",
                "nav2_remove_passed_goals_action_bt_node",
                "nav2_planner_selector_bt_node",
                "nav2_controller_selector_bt_node",
                "nav2_goal_checker_selector_bt_node",
            }
            <= plugin_lib_names
        )

    def test_top_level_planning_forwards_both_behavior_tree_paths(self) -> None:
        source = (PACKAGE_ROOT / "launch" / "planning.launch.py").read_text(
            encoding="utf-8"
        )
        for argument in (
            "nav2_bt_xml_nav_to_pose",
            "nav2_bt_xml_nav_through_poses",
        ):
            self.assertIn("DeclareLaunchArgument(\n            '" + argument, source)
            self.assertIn("                    '" + argument + "',", source)
        self.assertIn("'nav2_bt_navigator'", source)
        self.assertIn(
            "'navigate_through_poses_w_replanning_and_recovery.xml'", source
        )

    def test_direct_lanelet_launch_uses_stock_multi_pose_tree(self) -> None:
        source = LANELET_LAUNCH.read_text(encoding="utf-8")
        self.assertIn(
            "nav2_bt_share = get_package_share_directory('nav2_bt_navigator')",
            source,
        )
        self.assertIn(
            "'navigate_through_poses_w_replanning_and_recovery.xml'", source
        )

    def test_direct_lanelet_default_selector_ids_match_develop(self) -> None:
        planner_id, controller_id = LANELET_MODULE.infer_nav2_combo_ids(
            "disabled.yaml"
        )
        self.assertEqual((planner_id, controller_id), ("LaneletRoute", "MPPI"))


if __name__ == "__main__":
    unittest.main()
