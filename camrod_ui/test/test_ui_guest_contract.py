"""Guest UI regression tests for the shared Robot UI mission contract."""

import unittest

from avg_msgs.msg import AvgServiceState

from camrod_ui.ui_guest_node import (
    guest_gate_safety_hold,
    guest_mission_dispatch_ready,
    normalize_platform_battery_percent,
)


class GuestUiContractTest(unittest.TestCase):
    def test_platform_battery_fraction_matches_robot_ui_percent(self) -> None:
        self.assertEqual(normalize_platform_battery_percent(0.80), 80)
        self.assertEqual(normalize_platform_battery_percent(0.349), 35)
        self.assertEqual(normalize_platform_battery_percent(1.20), 100)

    def test_guest_dispatch_requires_stationary_state_and_soc_margin(self) -> None:
        self.assertTrue(
            guest_mission_dispatch_ready(AvgServiceState.DROP_ZONE_WAIT, 35, 35)
        )
        self.assertTrue(
            guest_mission_dispatch_ready(AvgServiceState.CHARGING, 80, 35)
        )
        self.assertFalse(
            guest_mission_dispatch_ready(AvgServiceState.CHARGING, 34, 35)
        )
        self.assertFalse(
            guest_mission_dispatch_ready(AvgServiceState.MOVING_TO_SITE, 80, 35)
        )

    def test_guest_uses_control_hold_not_generic_warning_as_safety_overlay(self) -> None:
        self.assertTrue(guest_gate_safety_hold("ROUTE_SAFETY_HOLD"))
        self.assertTrue(
            guest_gate_safety_hold(
                "ENABLED", "route_safety_hold=lanelet_footprint_cost"
            )
        )
        self.assertFalse(guest_gate_safety_hold("ENABLED", "system warning"))


if __name__ == "__main__":
    unittest.main()
