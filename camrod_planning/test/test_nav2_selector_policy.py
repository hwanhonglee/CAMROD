"""Unit tests for atomic manual/regulated Nav2 selector policy."""

import importlib.util
from pathlib import Path
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "nav2_selector_latch_node.py"
)
SPEC = importlib.util.spec_from_file_location("nav2_selector_latch_node", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Nav2SelectorPolicyTest(unittest.TestCase):
    """Checks that source changes select all three Nav2 policy IDs together."""

    def test_manual_default_is_long_range_lanelet_planner(self) -> None:
        self.assertEqual(MODULE.DEFAULT_MANUAL_PLANNER_ID, "LaneletRoute")

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


if __name__ == "__main__":
    unittest.main()
