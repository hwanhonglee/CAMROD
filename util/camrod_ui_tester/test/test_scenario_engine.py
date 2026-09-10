import math
import unittest

from camrod_ui_tester.simulation_state import (
    Pose2D,
    ScenarioEngine,
    parking_method_for_battery,
)


class ScenarioEngineTest(unittest.TestCase):
    def setUp(self):
        self.engine = ScenarioEngine(
            station_pose=Pose2D(0.0, 0.0, 0.0),
            sites={"B6": Pose2D(12.0, 4.0, 90.0)},
            mode="closed_loop",
            route_duration_s=10.0,
            maneuver_duration_s=2.0,
        )

    def test_reset_produces_ready_station_state(self):
        self.engine.apply_scenario("ready")
        self.assertEqual(self.engine.state.planning_state, "READY")
        self.assertEqual(self.engine.state.service_state, "DROP_ZONE_WAIT")
        self.assertEqual(self.engine.state.pose, Pose2D(0.0, 0.0, 0.0))

    def test_closed_loop_departure_emits_completion_signal(self):
        self.engine.apply_scenario("ready")
        self.engine.set_drive_enabled(True)
        self.engine.begin_departure()
        signal = None
        for _ in range(20):
            signal = self.engine.tick(0.2) or signal
        self.assertEqual(signal, "drop_zone_exit_complete")
        self.assertIsNone(self.engine.state.motion)
        self.assertAlmostEqual(self.engine.state.pose.x, 2.0)

    def test_site_goal_updates_pose_progress_and_arrival(self):
        self.engine.apply_scenario("ready")
        self.engine.select_destination("B6", "camping_site_6", "robot_ui")
        self.engine.set_drive_enabled(True)
        self.engine.receive_goal(Pose2D(12.0, 4.0, 90.0))
        self.engine.tick(5.0)
        self.assertGreater(self.engine.state.route_progress, 0.0)
        self.assertLess(self.engine.state.route_progress, 1.0)
        self.assertEqual(self.engine.state.service_state, "MOVING_TO_SITE")
        signal = None
        for _ in range(30):
            signal = self.engine.tick(0.5) or signal
        self.assertEqual(signal, "site_arrived")
        self.assertEqual(self.engine.state.service_state, "SITE_ARRIVED")
        self.assertAlmostEqual(self.engine.state.pose.x, 12.0)
        self.assertAlmostEqual(self.engine.state.pose.y, 4.0)
        self.assertTrue(math.isclose(self.engine.state.route_progress, 1.0))

    def test_safety_hold_pauses_motion_without_losing_route(self):
        self.engine.apply_scenario("ready")
        self.engine.set_drive_enabled(True)
        self.engine.receive_goal(Pose2D(12.0, 4.0, 90.0))
        self.engine.tick(1.0)
        progress = self.engine.state.route_progress
        self.engine.apply_scenario("safety_hold")
        for _ in range(10):
            self.engine.tick(0.5)
        self.assertEqual(self.engine.state.route_progress, progress)
        self.assertIsNotNone(self.engine.state.motion)
        self.assertEqual(self.engine.state.velocity_mps, 0.0)

    def test_stop_clears_every_motion_authority(self):
        self.engine.apply_scenario("ready")
        self.engine.set_engaged(mission=True)
        self.engine.set_drive_enabled(True)
        self.engine.receive_goal(Pose2D(12.0, 4.0, 90.0))
        self.engine.stop()
        self.assertEqual(self.engine.state.service_state, "OPERATOR_STOPPED")
        self.assertEqual(self.engine.state.planning_state, "ERROR_STOP")
        self.assertIsNone(self.engine.state.motion)
        self.assertFalse(self.engine.state.planning_engaged)
        self.assertFalse(self.engine.state.mission_engaged)
        self.assertFalse(self.engine.state.drive_enabled)

    def test_guest_recall_two_stage_completion_reaches_loading_wait(self):
        # HH_260908 - The recall completion contract accepts loading-complete
        # only in GUEST_LOADING_WAIT, and the RECALL_RETURN_WAIT confirmation
        # is armed by the controller-authorized in-site clearance turn.
        self.engine.apply_scenario("ready")
        self.engine.route_duration_s = 1.0
        self.assertTrue(self.engine.begin_recall("B6"))
        signal = None
        for _ in range(10):
            signal = self.engine.tick(0.2) or signal
        self.assertEqual(signal, "recall_arrived")
        self.assertEqual(self.engine.state.service_state, "GUEST_LOADING_WAIT")
        self.assertFalse(self.engine.state.recall_turn_complete)

        # First loading-complete runs the in-site clearance turn.
        self.assertTrue(self.engine.begin_recall_site_turn())
        self.assertEqual(self.engine.state.service_state, "SITE_ENTRY")
        signal = None
        for _ in range(20):
            signal = self.engine.tick(0.2) or signal
        self.assertEqual(signal, "recall_loading_wait")
        self.assertEqual(self.engine.state.service_state, "GUEST_LOADING_WAIT")
        self.assertTrue(self.engine.state.recall_turn_complete)
        self.assertEqual(self.engine.state.workflow_phase, "RECALL_RETURN_WAIT")
        # The armed turn never repeats; the next command is the real return.
        self.assertFalse(self.engine.begin_recall_site_turn())

    def test_closed_loop_parking_completes_to_charging_wait(self):
        # HH_260908 - The production docking controllers finish on their own;
        # the closed loop must not park forever in DROP_ZONE_PARKING.
        self.engine.apply_scenario("ready")
        self.engine.state.service_state = "DROP_ZONE_PARKING"
        signal = None
        for _ in range(30):
            signal = self.engine.tick(0.2) or signal
        self.assertEqual(signal, "parking_complete")
        self.assertEqual(self.engine.state.service_state, "WAITING_FOR_CHARGING")

    def test_battery_scenarios_match_ui_policy_boundaries(self):
        self.engine.apply_scenario("low_battery")
        self.assertAlmostEqual(self.engine.state.battery_percentage, 0.34)
        self.engine.apply_scenario("urgent_battery")
        self.assertAlmostEqual(self.engine.state.battery_percentage, 0.24)

    def test_parking_method_matches_production_battery_threshold(self):
        self.assertEqual(parking_method_for_battery(0.78), "reverse")
        self.assertEqual(parking_method_for_battery(0.35), "reverse")
        self.assertEqual(parking_method_for_battery(0.349), "apriltag")
        self.assertEqual(
            parking_method_for_battery(0.78, available=False),
            "apriltag",
        )
        self.assertEqual(parking_method_for_battery(float("nan")), "apriltag")

    def test_delivery_workflow_exposes_return_and_docking_as_separate_phases(self):
        self.engine.route_duration_s = 1.0
        self.engine.control_workflow("delivery", "B6", "move_to_site")
        self.assertEqual(self.engine.state.service_state, "MOVING_TO_SITE")

        for _ in range(10):
            self.engine.tick(0.2)
        self.assertEqual(self.engine.state.workflow_phase, "site_arrived")
        self.assertEqual(self.engine.state.service_state, "SITE_ARRIVED")

        self.engine.control_workflow("delivery", "B6", "return_to_drop_zone")
        self.assertEqual(self.engine.state.service_state, "RETURNING_TO_DROP_ZONE")
        for _ in range(10):
            self.engine.tick(0.2)
        self.assertEqual(self.engine.state.workflow_phase, "drop_zone_return_complete")
        self.assertEqual(self.engine.state.service_state, "RETURNING_TO_DROP_ZONE")

        self.engine.control_workflow("delivery", "B6", "docking_start")
        self.assertEqual(self.engine.state.service_state, "DROP_ZONE_PARKING")
        self.engine.control_workflow("delivery", "B6", "docking_complete")
        self.assertEqual(self.engine.state.service_state, "WAITING_FOR_CHARGING")
        self.assertFalse(self.engine.state.drive_enabled)

    def test_recall_workflow_uses_guest_service_states(self):
        expected = {
            "move_to_site": "RECALL_TO_SITE_ROAD",
            "site_arrived": "GUEST_LOADING_WAIT",
            "return_to_drop_zone": "RETURN_WITH_CARGO",
            "drop_zone_return_complete": "RETURN_WITH_CARGO",
            "docking_start": "DROP_ZONE_PARKING",
            "docking_complete": "WAITING_FOR_CHARGING",
        }
        for phase, service_state in expected.items():
            self.engine.control_workflow("recall", "B6", phase)
            self.assertEqual(self.engine.state.workflow_phase, phase)
            self.assertEqual(self.engine.state.service_state, service_state)

    def test_workflow_rejects_unknown_site(self):
        with self.assertRaisesRegex(ValueError, "invalid site"):
            self.engine.control_workflow("delivery", "B99", "move_to_site")

    def test_return_command_rearms_drive_after_site_arrival(self):
        self.engine.apply_scenario("ready")
        self.engine.set_drive_enabled(True)
        self.engine.arrive("B6")
        self.engine.set_engaged(mission=False)
        self.engine.set_drive_enabled(False)

        self.engine.begin_return("B6")
        self.assertTrue(self.engine.state.drive_enabled)
        self.assertTrue(self.engine.state.mission_engaged)
        self.assertEqual(self.engine.state.gate_state, "ENABLED")
        self.engine.tick(0.5)
        self.assertGreater(self.engine.state.route_progress, 0.0)


if __name__ == "__main__":
    unittest.main()
