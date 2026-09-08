"""Pure UI presentation regressions; no ROS node, service or motion is started."""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parent))
import test_ui_backend_stop as backend_fixture
from test_ui_state_policy import make_initialized_policy
from camrod_ui.ui_backend_node import UiBackendNode
from camrod_ui.ui_state_policy import UiStatePolicy
from avg_msgs.msg import AvgServiceState
from geometry_msgs.msg import PoseStamped


class CompletedStationPresentationTests(unittest.TestCase):
    def backend(self):
        frames = []
        policy = make_initialized_policy()
        policy.update_planning(state="WAIT_DZ", scenario="WAIT_DROP_ZONE",
            active_mission_key="drop_zone", active_goal_source="auto_snapper:drop_zone")
        policy.update_nav_status(4)
        backend = backend_fixture.UiBackendStopTest._mission_authority_backend(
            _return_requested_generation=41, _return_progress_generation=41,
            _latest_service_state=int(AvgServiceState.DROP_ZONE_PARKING),
            _runtime_policy=policy, _ui_last_route_goal_stamp=(100, 0),
            publish_mission_engage_from_destination=False,
            _schedule_broadcast=frames.append,
            _compute_operation_mode=lambda engaged, ready: "STOP",
            _log_readiness_transition=lambda *_: None,
            _publish_engage=lambda enabled, **_: policy.update_engaged(enabled),
            _update_low_battery_return_policy=lambda *_args, **_kwargs: None,
            _pending_site_route_goal_stamps={}, _site_route_anchors={},
            _route_goal_stamp_key=UiBackendNode._route_goal_stamp_key,
        )
        for name, value in dict(ready=True, ready_message="ready", engaged=False,
                operation_mode="STOP", mission_phase="GOAL_RECEIVED", mission_source="ui",
                battery_percentage=80).items():
            setattr(backend._state, name, value)
        backend._update_runtime_state = lambda callback, **kwargs: UiBackendNode._update_runtime_state(backend, callback, **kwargs)
        backend.frames = frames
        # Virtual backend retains its stricter physical terminal acceptance.
        # Supply a fresh in-station pose; do not mock away that guard.
        backend_fixture._set_fresh_departure_origin(backend, inside=True)
        backend._drop_zone_keypoint = SimpleNamespace(x=1.0, y=1.0, corners=[])
        return backend

    def complete(self, backend, state=AvgServiceState.DROP_ZONE_WAIT):
        event = AvgServiceState()
        event.state = state
        event.state_name = "DROP_ZONE_WAIT" if state == AvgServiceState.DROP_ZONE_WAIT else "CHARGING"
        event.description = "reverse_parking_controller:PARKED:station XY goal reached; parking_method=reverse"
        UiBackendNode._on_service_state(backend, event)

    def refresh(self, backend):
        backend._update_runtime_state(lambda: None, force_broadcast=True)

    def test_real_accepted_terminal_callback_corrects_api_and_ws_without_goal_reset(self):
        backend = self.backend()
        self.complete(backend)
        self.assertEqual(backend._active_mission_generation, 0)
        self.assertEqual(backend._state.mission_phase, "READY")
        self.assertEqual(backend._state.mission_source, "none")
        self.assertTrue(any(frame.get("mission_phase") == "READY" for frame in backend.frames))
        # Presentation correction does not erase planning/goal/controller state.
        self.assertEqual(backend._runtime_policy.mission_phase, "GOAL_RECEIVED")
        self.assertTrue(backend._runtime_policy.goal_received)

    def test_idle_alone_or_unaccepted_terminal_cannot_create_marker(self):
        for progress in (None, 0):
            with self.subTest(progress=progress):
                backend = self.backend()
                if progress is None:
                    backend._state.service_state = int(AvgServiceState.DROP_ZONE_WAIT)
                    self.refresh(backend)
                else:
                    backend._return_progress_generation = progress
                    self.complete(backend)
                self.assertIsNone(getattr(backend, "_ui_completed_station_goal", None))
                self.assertEqual(backend._state.mission_phase, "GOAL_RECEIVED")

    def test_cached_planning_and_duplicate_route_goal_do_not_revive_completed_header(self):
        backend = self.backend()
        self.complete(backend)
        backend._update_runtime_state(lambda: backend._runtime_policy.update_planning(
            state="WAIT_DZ", scenario="WAIT_DROP_ZONE", active_mission_key="drop_zone",
            active_goal_source="auto_snapper:drop_zone"))
        for sec in (99, 100):
            goal = PoseStamped()
            goal.header.stamp.sec = sec
            UiBackendNode._on_planning_route_goal(backend, goal)
            self.assertEqual(backend._state.mission_phase, "READY")

    def test_fresh_route_goal_releases_completed_presentation(self):
        backend = self.backend()
        self.complete(backend)
        goal = PoseStamped()
        goal.header.stamp.sec = 101
        UiBackendNode._on_planning_route_goal(backend, goal)
        self.assertIsNone(backend._ui_completed_station_goal)
        self.assertEqual(backend._state.mission_phase, "GOAL_RECEIVED")

    def test_new_accepted_robot_or_guest_intent_preserves_preengage_goal_display(self):
        for source in ("http_ui_destination", "robot_ui:recall", "guest:dispatch:r=new"):
            with self.subTest(source=source):
                backend = self.backend()
                self.complete(backend)
                UiBackendNode._claim_active_mission(backend, "B2", source)
                backend._update_runtime_state(lambda: backend._runtime_policy.update_goal_received("regulated"))
                self.assertIsNone(backend._ui_completed_station_goal)
                self.assertFalse(backend._state.engaged)
                self.assertEqual(backend._state.mission_phase, "GOAL_RECEIVED")

    def test_pending_departure_or_station_action_never_looks_completed(self):
        for name, value in (
                ("_active_mission_generation", 42), ("_active_mission_site", "B2"),
                ("_pending_site_after_drop_zone_exit", ("B2", "camping_site_2", "guest")),
                ("_charging_departure_delay_pending", True), ("_drop_zone_exit_active", True),
                ("_redock_after_disconnect_pending", True), ("_parking_rearm_waiting_for_can", True)):
            with self.subTest(name=name):
                backend = self.backend()
                self.complete(backend)
                setattr(backend, name, value)
                self.refresh(backend)
                self.assertEqual(backend._state.mission_phase, "GOAL_RECEIVED")

    def test_virtual_generation_zero_authority_is_not_reclassified(self):
        for owner in ("standalone_return", "manual_drive", "manual_goal"):
            with self.subTest(owner=owner):
                backend = self.backend()
                self.complete(backend)
                backend._generation_zero_authority = owner
                self.refresh(backend)
                self.assertEqual(backend._state.mission_phase, "GOAL_RECEIVED")

    def test_virtual_outside_station_terminal_does_not_create_completed_marker(self):
        backend = self.backend()
        backend_fixture._set_fresh_departure_origin(backend, inside=False)
        backend.drop_zone_arrival_radius_m = 0.5
        self.complete(backend)
        self.assertIsNone(getattr(backend, "_ui_completed_station_goal", None))
        self.assertEqual(backend._active_mission_generation, 41)
        self.assertEqual(backend._state.mission_phase, "GOAL_RECEIVED")

    def test_manual_and_changed_or_unknown_sources_are_not_reclassified(self):
        for source in ("manual", "manual:rviz", "unrecognized_external_source", "unknown"):
            with self.subTest(source=source):
                backend = self.backend()
                self.complete(backend)
                backend._update_runtime_state(lambda: backend._runtime_policy.update_goal_received(source))
                self.assertEqual(backend._state.mission_phase, backend._runtime_policy.mission_phase)
                self.assertEqual(backend._state.mission_source, backend._runtime_policy.mission_source)

    def test_unknown_service_and_active_planning_are_not_reclassified(self):
        for service, planning in ((999, "WAIT_DZ"), (12, "WAIT_DZ"), (0, "RUNNING"), (0, "RETURNING")):
            with self.subTest(service=service, planning=planning):
                backend = self.backend()
                self.complete(backend)
                backend._state.service_state = service
                backend._runtime_policy.planning_state = planning
                self.refresh(backend)
                self.assertEqual(backend._state.mission_phase, backend._runtime_policy.mission_phase)

    def test_error_safety_and_readiness_unknown_keep_priority_after_completion(self):
        for expected, update in (
                ("STOPPED", lambda p: setattr(p, "planning_state", "ERROR_STOP")),
                ("SAFETY_STOP", lambda p: p.update_gate(level=2, operating_state="SAFETY_HOLD", message="cost_hold=obstacle")),
                ("INITIALIZING", lambda p: p.update_localization(255))):
            with self.subTest(expected=expected):
                backend = self.backend()
                self.complete(backend)
                backend._update_runtime_state(lambda: update(backend._runtime_policy))
                self.assertEqual(backend._state.mission_phase, expected)

    def test_charging_completion_can_show_ready_but_not_dwell(self):
        backend = self.backend()
        backend._runtime_policy.update_gate(level=0, operating_state="CHARGING", message="")
        self.complete(backend, AvgServiceState.CHARGING)
        self.assertEqual(backend._state.mission_phase, "READY")
        backend._charging_departure_delay_pending = True
        self.refresh(backend)
        self.assertEqual(backend._state.mission_phase, "GOAL_RECEIVED")


if __name__ == "__main__":
    unittest.main()
