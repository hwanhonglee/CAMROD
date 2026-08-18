"""Regression coverage for the Robot UI operator-stop command path."""

from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest

from builtin_interfaces.msg import Time as RosTime

sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "runtime" / "python"),
)

from avg_msgs.msg import (  # noqa: E402
    AvgPlatformStatus,
    AvgServiceState,
    CampsiteOccupancy,
    MotionOperation,
)
from camrod_ui.ui_backend_node import UiBackendNode  # noqa: E402


class _FakeLogger:

    def __init__(self) -> None:
        self.info_messages = []
        self.warning_messages = []

    def info(self, message: str) -> None:
        self.info_messages.append(message)

    def warn(self, message: str) -> None:
        self.warning_messages.append(message)


class _FakeCancelClient:

    def __init__(self, ready: bool) -> None:
        self.ready = ready
        self.requests = []

    def service_is_ready(self) -> bool:
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return object()


class _FakeLoop:

    def __init__(self) -> None:
        self.wake_calls = 0

    def is_running(self) -> bool:
        return True

    def call_soon_threadsafe(self, callback) -> None:
        self.wake_calls += 1
        callback()


class _FakeThread:

    def __init__(self) -> None:
        self.alive = True
        self.join_timeouts = []

    def is_alive(self) -> bool:
        return self.alive

    def join(self, timeout: float) -> None:
        self.join_timeouts.append(timeout)
        self.alive = False


class _FakeTimer:

    def __init__(self, period_s: float, callback) -> None:
        self.period_s = period_s
        self.callback = callback
        self.cancelled = False

    def cancel(self) -> None:
        self.cancelled = True


class _FakeBackend:

    def __init__(self) -> None:
        self.nav2_cancel_action_topics = ["/ready/cancel", "/missing/cancel"]
        self.nav2_cancel_clients = [
            _FakeCancelClient(ready=True),
            _FakeCancelClient(ready=False),
        ]
        self.operations = []
        self.logger = _FakeLogger()

    def get_logger(self):
        return self.logger

    def _record_operation(self, owner: str, operation: int, source: str) -> None:
        self.operations.append((owner, operation, source))

    def _publish_camping_site_operation(self, operation: int, source: str) -> None:
        self._record_operation("camping_site", operation, source)

    def _publish_drop_zone_operation(self, operation: int, source: str) -> None:
        self._record_operation("drop_zone", operation, source)

    def _publish_parking_operation(self, operation: int, source: str) -> None:
        self._record_operation("parking", operation, source)

    def _request_nav2_cancel(self, source: str):
        return UiBackendNode._request_nav2_cancel(self, source)


class UiBackendStopTest(unittest.TestCase):

    def test_manual_return_defers_planning_until_site_exit_reaches_anchor(self) -> None:
        events = []

        class Publisher:
            @staticmethod
            def publish(message) -> None:
                events.append(("planning", message))

        backend = SimpleNamespace(
            _active_mission_site="B8",
            _latest_service_state=int(AvgServiceState.WAITING_FOR_RETURN_REQUEST),
            _latest_platform_is_charging=False,
            planning_return_to_drop_zone_topic=(
                "/planning/state_machine/return_to_drop_zone"
            ),
            pub_planning_return_to_drop_zone=Publisher(),
            _publish_camping_site_maneuver_controller_return=(
                lambda source: events.append(("site_exit", source))
            ),
            _publish_service_state=(
                lambda state, source: events.append(("state", state, source))
            ),
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(
                    to_msg=lambda: RosTime(sec=1, nanosec=2)
                )
            ),
            get_logger=lambda: _FakeLogger(),
        )

        action = UiBackendNode._request_return_to_drop_zone(backend, "test:manual")

        self.assertEqual(action, "site_exit_then_return")
        self.assertEqual([event[0] for event in events], ["site_exit", "state"])
        self.assertEqual(events[1][1], AvgServiceState.RETURNING_TO_DROP_ZONE)

    def test_manual_return_during_normal_travel_preempts_with_planning_recall(self) -> None:
        events = []
        timers = []
        runtime_lock = threading.Lock()

        def create_timer(period_s, callback):
            timer = _FakeTimer(period_s, callback)
            timers.append(timer)
            events.append(("timer", period_s))
            return timer

        def publish_return(source):
            # The real publisher updates runtime policy under the general lock.
            acquired = runtime_lock.acquire(blocking=False)
            self.assertTrue(acquired)
            if acquired:
                runtime_lock.release()
            events.append(("planning", source))

        backend = SimpleNamespace(
            _active_mission_site="B8",
            _latest_service_state=int(AvgServiceState.MOVING_TO_SITE),
            _latest_platform_is_charging=False,
            _manual_return_transition_pending=False,
            _manual_return_transition_timer=None,
            _manual_return_transition_source="",
            _manual_return_transition_lock=threading.Lock(),
            manual_return_preempt_hold_s=0.5,
            publish_mission_engage_from_destination=True,
            _lock=runtime_lock,
            _request_nav2_cancel=(
                lambda source: events.append(("cancel", source))
            ),
            _publish_mission_engage=(
                lambda enabled, source: events.append(
                    ("mission_engage", enabled, source)
                )
            ),
            _publish_engage=(
                lambda enabled, source, sync_drive_enable: events.append(
                    ("engage", enabled, source, sync_drive_enable)
                )
            ),
            _publish_platform_drive_enable=(
                lambda enabled, source: events.append(
                    ("platform", enabled, source)
                )
            ),
            _publish_planning_return_request=publish_return,
            create_timer=create_timer,
            destroy_timer=lambda timer: events.append(("destroy_timer", timer)),
            get_logger=lambda: _FakeLogger(),
        )
        backend._schedule_manual_return_transition = lambda source: (
            UiBackendNode._schedule_manual_return_transition(backend, source)
        )
        backend._complete_manual_return_transition = lambda: (
            UiBackendNode._complete_manual_return_transition(backend)
        )

        action = UiBackendNode._request_return_to_drop_zone(
            backend, "test:normal_travel"
        )

        self.assertEqual(action, "return_preempting")
        self.assertEqual(
            [event[0] for event in events],
            ["cancel", "mission_engage", "engage", "timer"],
        )
        self.assertFalse(events[1][1])
        self.assertFalse(events[2][1])
        self.assertFalse(events[2][3])
        self.assertEqual(timers[0].period_s, 0.5)
        self.assertNotIn("planning", [event[0] for event in events])

        # HH_260819 - Both visible Return buttons share this pending latch.
        repeated_action = UiBackendNode._request_return_to_drop_zone(
            backend, "test:second_button"
        )
        self.assertEqual(repeated_action, "return_preempting")
        self.assertEqual(len(timers), 1)

        UiBackendNode._complete_manual_return_transition(backend)
        self.assertTrue(timers[0].cancelled)
        self.assertEqual(events[-1], ("planning", "test:normal_travel"))
        self.assertFalse(backend._manual_return_transition_pending)

    def test_operator_stop_cancels_pending_return_before_route_publish(self) -> None:
        events = []
        timer = _FakeTimer(0.5, lambda: None)
        backend = SimpleNamespace(
            _manual_return_transition_pending=True,
            _manual_return_transition_timer=timer,
            _manual_return_transition_source="test:normal_travel",
            _manual_return_transition_lock=threading.Lock(),
            destroy_timer=lambda value: events.append(("destroy", value)),
            _publish_planning_return_request=(
                lambda source: events.append(("planning", source))
            ),
            get_logger=lambda: _FakeLogger(),
        )

        UiBackendNode._cancel_pending_manual_return_transition(
            backend, "operator_stop"
        )
        UiBackendNode._complete_manual_return_transition(backend)

        # HH_260819 - A delayed timer callback cannot reopen motion after the
        # operator has won ownership with Stop.
        self.assertTrue(timer.cancelled)
        self.assertFalse(backend._manual_return_transition_pending)
        self.assertNotIn("planning", [event[0] for event in events])

    def test_manual_return_request_reopens_gate_before_fresh_route(self) -> None:
        events = []

        class Publisher:
            @staticmethod
            def publish(message) -> None:
                events.append(("planning", message))

        backend = SimpleNamespace(
            _active_mission_site="B8",
            planning_return_to_drop_zone_topic=(
                "/planning/state_machine/return_to_drop_zone"
            ),
            pub_planning_return_to_drop_zone=Publisher(),
            publish_mission_engage_from_destination=True,
            _publish_service_state=(
                lambda state, source: events.append(("state", state, source))
            ),
            _publish_mission_engage=(
                lambda enabled, source: events.append(
                    ("mission_engage", enabled, source)
                )
            ),
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(
                    to_msg=lambda: RosTime(sec=1, nanosec=2)
                )
            ),
            get_logger=lambda: _FakeLogger(),
        )

        UiBackendNode._publish_planning_return_request(backend, "test:return")

        self.assertEqual(
            [event[0] for event in events],
            ["state", "mission_engage", "planning"],
        )
        self.assertEqual(events[0][1], AvgServiceState.RETURNING_TO_DROP_ZONE)
        self.assertTrue(events[1][1])
        self.assertEqual(events[2][1].site_name, "B8")
        self.assertEqual(events[2][1].source, "test:return")

    def test_manual_return_at_drop_zone_starts_alignment_without_nav_loop(self) -> None:
        operations = []
        backend = SimpleNamespace(
            _latest_service_state=int(AvgServiceState.DROP_ZONE_WAIT),
            _latest_platform_is_charging=False,
            _publish_drop_zone_operation=lambda operation, source: operations.append(
                (operation, source)
            ),
        )

        action = UiBackendNode._request_return_to_drop_zone(
            backend, "test:station"
        )

        self.assertEqual(action, "parking_alignment")
        self.assertEqual(operations[0][0], MotionOperation.ALIGN_FOR_PARKING)

    def test_second_return_button_does_not_restart_active_return_route(self) -> None:
        backend = SimpleNamespace(
            _manual_return_transition_pending=False,
            _latest_service_state=int(AvgServiceState.RETURNING_TO_DROP_ZONE),
            _latest_platform_is_charging=False,
        )

        action = UiBackendNode._request_return_to_drop_zone(
            backend, "test:second_button"
        )

        self.assertEqual(action, "return_in_progress")

    def test_manual_return_realigns_when_charging_contact_was_lost(self) -> None:
        # HH_260818 - A stale public CHARGING state without current CAN contact
        # is a parking retry, not a reason to create another drop-zone Nav2 loop.
        operations = []
        backend = SimpleNamespace(
            _latest_service_state=int(AvgServiceState.CHARGING),
            _latest_platform_is_charging=False,
            _publish_drop_zone_operation=lambda operation, source: operations.append(
                (operation, source)
            ),
        )

        action = UiBackendNode._request_return_to_drop_zone(
            backend, "test:lost_contact"
        )

        self.assertEqual(action, "parking_alignment")
        self.assertEqual(operations[0][0], MotionOperation.ALIGN_FOR_PARKING)

    def test_readiness_timer_can_rebroadcast_unchanged_authoritative_state(self) -> None:
        broadcasts = []
        state = SimpleNamespace(
            ready=True,
            ready_message="ready",
            engaged=False,
            operation_mode="STOP",
            mission_phase="READY",
            mission_source="none",
        )
        policy = SimpleNamespace(
            ready=True,
            engaged=False,
            mission_phase="READY",
            mission_source="none",
            readiness_reasons=lambda **_kwargs: (),
        )
        backend = SimpleNamespace(
            _lock=threading.Lock(),
            _state=state,
            _runtime_policy=policy,
            _compute_operation_mode=lambda _engaged, _ready: "STOP",
            _schedule_broadcast=broadcasts.append,
        )

        # HH_260810 - A periodic snapshot repairs an initial WebSocket/ROS
        # ordering race without waiting for a manual or campsite goal event.
        UiBackendNode._update_runtime_state(
            backend,
            lambda: None,
            force_broadcast=True,
        )

        self.assertEqual(len(broadcasts), 1)
        self.assertTrue(broadcasts[0]["ready"])
        self.assertEqual(broadcasts[0]["mission_phase"], "READY")

    def test_charging_contact_does_not_cancel_active_station_departure(self) -> None:
        published_states = []
        logger = _FakeLogger()
        backend = SimpleNamespace(
            _drop_zone_exit_active=True,
            _latest_platform_is_charging=False,
            _latest_service_state=int(AvgServiceState.DEPARTING_CHARGER),
            _runtime_policy=SimpleNamespace(update_platform=lambda **_kwargs: None),
        )
        backend.get_logger = lambda: logger
        backend._update_runtime_state = lambda callback: callback()
        backend._publish_service_state = (
            lambda state, source: published_states.append((state, source))
        )
        backend._update_low_battery_return_policy = lambda *_args, **_kwargs: None

        message = AvgPlatformStatus()
        message.is_charging = True
        message.battery_state_available = False
        UiBackendNode._on_platform_status(backend, message)

        self.assertTrue(backend._latest_platform_is_charging)
        self.assertEqual(published_states, [])
        self.assertIn("preserving departure authorization", logger.info_messages[-1])

    def test_duplicate_destination_is_idempotent_during_station_departure(self) -> None:
        logger = _FakeLogger()
        backend = SimpleNamespace(
            _drop_zone_exit_active=True,
            _pending_site_after_drop_zone_exit=("B2", "camping_site_2", "first"),
        )
        backend.get_logger = lambda: logger

        result = UiBackendNode._apply_destination_command(
            backend,
            site="B2",
            run=True,
            source="retry",
        )

        self.assertEqual(result["mission_key"], "camping_site_2")
        self.assertFalse(result["goal_pose_published"])
        self.assertIn("already pending", result["message"])

    def test_both_ui_nodes_accept_external_shutdown_as_clean_exit(self) -> None:
        runtime_dir = Path(__file__).resolve().parents[1] / "runtime" / "python" / "camrod_ui"
        for filename in ("ui_backend_node.py", "ui_guest_node.py"):
            source = (runtime_dir / filename).read_text(encoding="utf-8")
            self.assertIn("from rclpy.executors import ExternalShutdownException", source)
            self.assertIn("except (KeyboardInterrupt, ExternalShutdownException):", source)
            # HH_260807 - Keep Humble's shutdown-only take_message race from
            # producing exit code 1 while preserving live RuntimeError failures.
            self.assertIn("except RuntimeError:", source)
            self.assertIn("if rclpy.ok():\n            raise", source)

    def test_http_server_is_stopped_before_node_destruction(self) -> None:
        server = SimpleNamespace(should_exit=False)
        loop = _FakeLoop()
        thread = _FakeThread()
        backend = SimpleNamespace(
            _server_stop_requested=threading.Event(),
            _uvicorn_server=server,
            _main_loop=loop,
            _server_thread=thread,
        )
        backend.get_logger = lambda: _FakeLogger()

        UiBackendNode._stop_fastapi_server(backend, join_timeout_s=1.25)

        self.assertTrue(backend._server_stop_requested.is_set())
        self.assertTrue(server.should_exit)
        self.assertEqual(loop.wake_calls, 1)
        self.assertEqual(thread.join_timeouts, [1.25])

    def test_cancel_uses_rclpy_call_async_and_cancels_every_motion_owner(self) -> None:
        backend = _FakeBackend()

        UiBackendNode._cancel_active_motion(backend, source="http_stop")

        self.assertEqual(len(backend.nav2_cancel_clients[0].requests), 1)
        self.assertEqual(backend.nav2_cancel_clients[1].requests, [])
        self.assertEqual(
            backend.operations,
            [
                ("camping_site", MotionOperation.CANCEL, "http_stop:operator_stop"),
                ("drop_zone", MotionOperation.CANCEL, "http_stop:operator_stop"),
                ("parking", MotionOperation.CANCEL, "http_stop:operator_stop"),
            ],
        )
        self.assertIn("/ready/cancel", backend.logger.info_messages[0])

    def test_arrival_uses_active_mission_site_after_destination_is_cleared(self) -> None:
        backend = SimpleNamespace(
            _latest_service_state=None,
            _active_mission_site="B6",
            _lock=threading.Lock(),
            _state=SimpleNamespace(
                destination={"site": "", "run": False},
                service_state=-1,
                service_state_name="",
                service_state_description="",
                battery_percentage=80,
            ),
            publish_mission_engage_from_destination=False,
            arrival_notifications=[],
            broadcasts=[],
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._schedule_broadcast = backend.broadcasts.append
        backend._notify_site_arrival = lambda site, state, source: (
            backend.arrival_notifications.append((site, state, source))
        )
        backend._publish_engage = lambda *_args, **_kwargs: None
        backend._update_low_battery_return_policy = lambda *_args, **_kwargs: None

        message = AvgServiceState()
        message.state = AvgServiceState.WAITING_FOR_RETURN_REQUEST
        message.state_name = "WAITING_FOR_RETURN_REQUEST"
        message.description = "Waiting for return request"
        UiBackendNode._on_service_state(backend, message)

        self.assertEqual(
            backend.arrival_notifications,
            [
                (
                    "B6",
                    AvgServiceState.WAITING_FOR_RETURN_REQUEST,
                    f"service_state:{AvgServiceState.WAITING_FOR_RETURN_REQUEST}",
                )
            ],
        )

    def test_battery_rejected_destination_does_not_become_active_site(self) -> None:
        backend = SimpleNamespace(
            _active_mission_site="B4",
            _lock=threading.Lock(),
            broadcasts=[],
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._resolve_mission_key_for_site = lambda site: f"camping_site_{site[1:]}"
        backend._is_site_occupied = lambda _site: False
        backend._site_arrival_match = lambda _site: (
            False,
            "camping_site_6",
            20.0,
            "outside_arrival_radius",
        )
        backend._mission_dispatch_battery_block = lambda _site: {
            "message": "battery below mission minimum",
            "battery_percentage": 30,
            "minimum_battery_percentage": 35,
        }
        backend._schedule_broadcast = backend.broadcasts.append

        result = UiBackendNode._apply_destination_command(
            backend,
            site="B6",
            run=True,
            source="test",
        )

        self.assertTrue(result["blocked"])
        self.assertEqual(backend._active_mission_site, "B4")

    @staticmethod
    def _occupancy_backend(service_state: int, enabled: bool = False):
        state = SimpleNamespace(
            occupied_sites=[],
            ws_site_states={"B3": True},
            destination={"site": "B3", "run": True},
            service_state=int(service_state),
        )
        backend = SimpleNamespace(
            _lock=threading.Lock(),
            _state=state,
            site_names=["B3"],
            enable_campsite_occupancy_guard=enabled,
            broadcasts=[],
            stops=[],
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._resolve_mission_key_for_site = lambda site: f"camping_site_{site[1:]}"
        backend._schedule_broadcast = backend.broadcasts.append
        backend._apply_destination_command = (
            lambda site, run, source: backend.stops.append((site, run, source))
        )
        return backend

    @staticmethod
    def _occupancy_message(mission_key: str):
        message = CampsiteOccupancy()
        message.occupied_mission_keys = [mission_key]
        return message

    def test_campsite_occupancy_guard_is_disabled_end_to_end(self) -> None:
        # HH_260818 - False preserves the current tent-at-destination workflow.
        for service_state in (
            AvgServiceState.MOVING_TO_SITE,
            AvgServiceState.SITE_ENTRY,
            AvgServiceState.UNLOAD_WAIT,
            AvgServiceState.WAITING_FOR_RETURN_REQUEST,
        ):
            with self.subTest(service_state=int(service_state)):
                backend = self._occupancy_backend(service_state)

                UiBackendNode._on_campsite_occupancy(
                    backend, self._occupancy_message("camping_site_3")
                )

                # No dispatch cancelled, no toggle cleared, nothing broadcast.
                self.assertEqual(backend.stops, [])
                self.assertTrue(backend._state.destination["run"])
                self.assertTrue(backend._state.ws_site_states["B3"])
                self.assertEqual(backend._state.occupied_sites, [])
                self.assertEqual(backend.broadcasts, [])

    def test_enabled_occupancy_guard_cancels_only_pre_entry_dispatch(self) -> None:
        backend = self._occupancy_backend(
            AvgServiceState.MOVING_TO_SITE, enabled=True
        )

        UiBackendNode._on_campsite_occupancy(
            backend, self._occupancy_message("camping_site_3")
        )

        self.assertEqual(backend._state.occupied_sites, ["B3"])
        self.assertFalse(backend._state.ws_site_states["B3"])
        self.assertFalse(backend._state.destination["run"])
        self.assertEqual(
            backend.stops,
            [("B3", False, "perception_occupancy")],
        )
        self.assertEqual(backend.broadcasts[0], {"occupied_sites": ["B3"]})
        self.assertEqual(backend.broadcasts[1]["error"], "campsite_occupied")

    def test_enabled_occupancy_guard_does_not_cancel_committed_entry(self) -> None:
        backend = self._occupancy_backend(
            AvgServiceState.SITE_ENTRY, enabled=True
        )

        UiBackendNode._on_campsite_occupancy(
            backend, self._occupancy_message("camping_site_3")
        )

        self.assertEqual(backend._state.occupied_sites, ["B3"])
        self.assertTrue(backend._state.ws_site_states["B3"])
        self.assertTrue(backend._state.destination["run"])
        self.assertEqual(backend.stops, [])
        self.assertEqual(backend.broadcasts, [{"occupied_sites": ["B3"]}])

    def test_site_occupancy_lookup_follows_guard_toggle(self) -> None:
        backend = self._occupancy_backend(AvgServiceState.MOVING_TO_SITE)
        backend._state.occupied_sites = ["B3"]

        self.assertFalse(UiBackendNode._is_site_occupied(backend, "B3"))
        backend.enable_campsite_occupancy_guard = True
        self.assertTrue(UiBackendNode._is_site_occupied(backend, "B3"))


if __name__ == "__main__":
    unittest.main()
