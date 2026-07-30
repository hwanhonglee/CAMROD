from pathlib import Path
import sys
import unittest


sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "runtime" / "python"),
)

from camrod_ui.ui_state_policy import UiStatePolicy  # noqa: E402


REQUIRED_MODULES = (
    "map",
    "sensing",
    "localization",
    "planning",
    "control",
    "platform",
    "system",
)


def healthy_modules(*, starting_module=""):
    return {
        name: (
            0,
            "STARTING" if name == starting_module else "READY",
        )
        for name in REQUIRED_MODULES
    }


def degraded_modules():
    modules = healthy_modules()
    modules["sensing"] = (1, "DUMMY")
    modules["system"] = (1, "DEGRADED")
    return modules


def make_initialized_policy():
    policy = UiStatePolicy(REQUIRED_MODULES)
    policy.update_system(healthy_modules())
    policy.update_planning(
        state="WAIT_DZ",
        scenario="WAIT_DROP_ZONE",
        active_mission_key="",
        active_goal_source="none",
    )
    policy.update_action_server(True)
    policy.update_localization(0)
    policy.update_tf(True)
    policy.update_gate(
        level=0,
        operating_state="STANDBY",
        message="reasons=engage=false",
    )
    policy.update_platform(estop=False, error_code=0)
    policy.update_engaged(False)
    return policy


class UiStatePolicyTest(unittest.TestCase):
    """Exercise readiness and mission phases through colcon's unittest path."""

    def test_startup_requires_all_real_prerequisites(self):
        policy = UiStatePolicy(REQUIRED_MODULES)
        policy.update_system(healthy_modules(starting_module="sensing"))
        policy.update_planning(
            state="WAIT_DZ",
            scenario="WAIT_DROP_ZONE",
            active_mission_key="",
            active_goal_source="none",
        )
        policy.update_action_server(True)
        policy.update_localization(0)
        policy.update_gate(level=0, operating_state="STANDBY", message="")
        policy.update_platform(estop=False, error_code=0)
        policy.update_engaged(False)

        self.assertFalse(policy.ready)
        self.assertEqual(
            policy.mission_phase, UiStatePolicy.INITIALIZING
        )
        self.assertIn(
            "module_not_ready:sensing:starting",
            policy.readiness_reasons(),
        )
        self.assertIn("tf_unavailable", policy.readiness_reasons())

    def test_engage_without_a_goal_does_not_claim_driving(self):
        policy = make_initialized_policy()

        policy.update_engaged(True)
        policy.update_gate(level=0, operating_state="ENABLED", message="")

        self.assertTrue(policy.ready)
        self.assertEqual(policy.mission_source, "none")
        self.assertEqual(policy.mission_phase, UiStatePolicy.READY)

    def test_dummy_warn_is_degraded_ready_but_error_is_not(self):
        policy = make_initialized_policy()
        policy.update_system(degraded_modules())

        self.assertTrue(policy.ready)
        self.assertEqual(policy.mission_phase, UiStatePolicy.READY)
        self.assertNotIn(
            "module_unhealthy:sensing:1", policy.readiness_reasons()
        )

        failed_modules = degraded_modules()
        failed_modules["sensing"] = (2, "FAULT")
        policy.update_system(failed_modules)

        self.assertFalse(policy.ready)
        self.assertEqual(
            policy.mission_phase, UiStatePolicy.INITIALIZING
        )
        self.assertIn(
            "module_unhealthy:sensing:2", policy.readiness_reasons()
        )
        self.assertIn(
            "module_not_ready:sensing:fault",
            policy.readiness_reasons(),
        )

    def test_inactive_required_module_is_not_ready_even_at_warn(self):
        policy = make_initialized_policy()
        modules = healthy_modules()
        modules["planning"] = (1, "INACTIVE")
        policy.update_system(modules)

        self.assertFalse(policy.ready)
        self.assertIn(
            "module_not_ready:planning:inactive",
            policy.readiness_reasons(),
        )

    def _assert_goal_sequence(self, source, mission_key):
        policy = make_initialized_policy()
        policy.update_goal_source(source)
        policy.update_goal_received()
        self.assertEqual(
            policy.mission_phase, UiStatePolicy.GOAL_RECEIVED
        )

        policy.update_nav_status(1)
        self.assertEqual(
            policy.mission_phase, UiStatePolicy.PATH_PREPARING
        )

        policy.update_planning(
            state="RUNNING",
            scenario="DELIVERY_TO_SITE",
            active_mission_key=mission_key,
            active_goal_source=source,
        )
        policy.update_engaged(True)
        self.assertEqual(
            policy.mission_phase, UiStatePolicy.PATH_PREPARING
        )
        policy.update_gate(level=0, operating_state="ENABLED", message="")
        self.assertEqual(policy.mission_phase, UiStatePolicy.DRIVING)

        policy.update_gate(
            level=1,
            operating_state="ENABLED",
            message="reasons=route_safety_hold=lanelet_footprint_cost",
        )
        self.assertEqual(
            policy.mission_phase, UiStatePolicy.SAFETY_STOP
        )

        policy.update_gate(level=0, operating_state="ENABLED", message="")
        policy.update_planning(
            state="GOAL_REACHED",
            scenario="DELIVERY_TO_SITE",
            active_mission_key=mission_key,
            active_goal_source=source,
        )
        self.assertEqual(policy.mission_phase, UiStatePolicy.ARRIVED)

    def test_manual_and_ui_goals_use_the_same_phase_sequence(self):
        self._assert_goal_sequence("manual", "")
        self._assert_goal_sequence("regulated", "camping_site_3")

    def test_localization_reacquisition_returns_to_initialization(self):
        policy = make_initialized_policy()
        policy.update_planning(
            state="RUNNING",
            scenario="DELIVERY_TO_SITE",
            active_mission_key="",
            active_goal_source="manual",
        )
        policy.update_engaged(True)
        policy.update_gate(level=0, operating_state="ENABLED", message="")
        self.assertEqual(policy.mission_phase, UiStatePolicy.DRIVING)

        policy.update_localization(2)

        self.assertFalse(policy.ready)
        self.assertEqual(
            policy.mission_phase, UiStatePolicy.INITIALIZING
        )
        self.assertIn(
            "localization_mode:2", policy.readiness_reasons()
        )


if __name__ == "__main__":
    unittest.main()
