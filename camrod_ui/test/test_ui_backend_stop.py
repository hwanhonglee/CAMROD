"""Regression coverage for the Robot UI operator-stop command path."""

from pathlib import Path
import json
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest import mock

from builtin_interfaces.msg import Time as RosTime

sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "runtime" / "python"),
)

from avg_msgs.msg import (  # noqa: E402
    AvgBool,
    AvgPlatformStatus,
    AvgServiceState,
    CampsiteOccupancy,
    ModuleState,
    MotionOperation,
    UiDestinationCommand,
)
from camrod_ui.ui_backend_node import UiBackendNode  # noqa: E402


class _FakeLogger:

    def __init__(self) -> None:
        self.error_messages = []
        self.info_messages = []
        self.warning_messages = []

    def error(self, message: str) -> None:
        self.error_messages.append(message)

    def info(self, message: str) -> None:
        self.info_messages.append(message)

    def warn(self, message: str) -> None:
        self.warning_messages.append(message)


class _FakeServiceMetrics:

    def __init__(self) -> None:
        self.starts = []
        self.velocities = []
        self.states = []

    def start_service(self, site: str, **kwargs) -> bool:
        self.starts.append((site, kwargs))
        return True

    def observe_velocity(self, vx, vy, sample_time) -> float:
        self.velocities.append((float(vx), float(vy), float(sample_time)))
        return 0.0

    def observe_service_state(self, state, state_name, **kwargs) -> bool:
        self.states.append((int(state), str(state_name), kwargs))
        return False


class _FakeCancelClient:

    def __init__(self, ready: bool) -> None:
        self.ready = ready
        self.requests = []

    def service_is_ready(self) -> bool:
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return object()


class _ControlledFuture:

    def __init__(self) -> None:
        self.completed = False

    def done(self) -> bool:
        return self.completed

    def result(self):
        return object()


class _BarrierCancelClient:

    def __init__(self) -> None:
        self.futures = []

    def service_is_ready(self) -> bool:
        return True

    def call_async(self, _request):
        future = _ControlledFuture()
        self.futures.append(future)
        return future


class _CapturedPublisher:

    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


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

    @staticmethod
    def _mission_authority_backend(**overrides):
        backend = SimpleNamespace(
            _destination_dispatch_lock=threading.RLock(),
            _lock=threading.Lock(),
            _active_mission_site="B1",
            _active_mission_source="http_ui_destination",
            _active_mission_generation=41,
            _active_mission_owner="operator",
            _active_mission_intent="delivery",
            _return_requested_generation=0,
            _return_progress_generation=0,
            _active_mission_retryable=False,
            _startup_recovery_pending=False,
            _latest_service_state=int(AvgServiceState.MOVING_TO_SITE),
            _state=SimpleNamespace(
                service_state=int(AvgServiceState.MOVING_TO_SITE),
                service_state_name="MOVING_TO_SITE",
                service_state_description="Moving from drop zone to site",
                ws_site_states={"B1": True, "B2": False},
                destination={"site": "B1", "run": True},
            ),
            pub_destination_dispatch_status=_CapturedPublisher(),
            get_logger=lambda: _FakeLogger(),
        )
        backend._resolve_mission_key_for_site = (
            lambda site: f"camping_site_{site[1:]}"
        )
        for name, value in overrides.items():
            setattr(backend, name, value)
        return backend

    def test_service_state_qos_preserves_return_transition_burst(
        self,
    ) -> None:
        source = (
            Path(__file__).resolve().parents[1]
            / "runtime"
            / "python"
            / "camrod_ui"
            / "ui_backend_node.py"
        ).read_text(encoding="utf-8")
        block_start = source.index("service_heartbeat_qos = QoSProfile(")
        block_end = source.index(")", block_start)
        qos_block = source[block_start:block_end]

        self.assertIn("history=HistoryPolicy.KEEP_LAST", qos_block)
        self.assertIn("depth=10", qos_block)
        self.assertIn("reliability=ReliabilityPolicy.RELIABLE", qos_block)
        self.assertIn("durability=DurabilityPolicy.VOLATILE", qos_block)
        self.assertGreaterEqual(
            int(qos_block.split("depth=", 1)[1].split(",", 1)[0]),
            3,
            "a bounded service-state burst must survive dispatch-lock delay",
        )

    def test_dispatch_status_uses_transient_local_state_qos(self) -> None:
        source = (
            Path(__file__).resolve().parents[1]
            / "runtime"
            / "python"
            / "camrod_ui"
            / "ui_backend_node.py"
        ).read_text(encoding="utf-8")

        self.assertIn(
            '"/ui/destination_dispatch_status"',
            source,
        )
        self.assertIn(
            "String, self.ui_destination_dispatch_status_topic, state_qos",
            source,
        )
        state_qos_start = source.index("state_qos = QoSProfile(")
        state_qos_end = source.index(")", state_qos_start)
        self.assertIn(
            "durability=DurabilityPolicy.TRANSIENT_LOCAL",
            source[state_qos_start:state_qos_end],
        )

    def test_competing_destination_preserves_authoritative_mission(self) -> None:
        backend = self._mission_authority_backend()

        result = UiBackendNode._apply_destination_command(
            backend, site="B2", run=True, source="robot_ui:recall"
        )

        self.assertTrue(result["blocked"])
        self.assertEqual(result["error"], "mission_already_active")
        self.assertEqual(backend._active_mission_site, "B1")
        self.assertEqual(backend._active_mission_generation, 41)
        status = json.loads(
            backend.pub_destination_dispatch_status.messages[-1].data
        )
        self.assertFalse(status["accepted"])
        self.assertEqual(status["active_site"], "B1")
        self.assertEqual(status["active_generation"], 41)

    def test_stale_destination_off_cannot_stop_newer_mission(self) -> None:
        backend = self._mission_authority_backend()

        result = UiBackendNode._apply_destination_command(
            backend,
            site="B1",
            run=False,
            source="http_ui_destination",
            mission_generation=40,
        )

        self.assertTrue(result["blocked"])
        self.assertEqual(
            result["error"], "stale_or_unowned_destination_stop"
        )
        self.assertEqual(backend._active_mission_site, "B1")
        self.assertEqual(backend._state.destination, {"site": "B1", "run": True})

    def test_stale_owned_return_cannot_change_current_mission(self) -> None:
        backend = self._mission_authority_backend(
            _latest_service_state=int(AvgServiceState.UNLOAD_WAIT)
        )

        result = UiBackendNode.request_owned_return_to_drop_zone(
            backend,
            "B1",
            40,
            source="robot_ui:usage_complete",
            allowed_owners={"operator", "robot"},
        )

        self.assertFalse(result["success"])
        self.assertEqual(result["error"], "stale_or_unowned_return")
        self.assertEqual(backend._return_requested_generation, 0)

    def test_dispatch_snapshot_and_status_expose_one_identity(self) -> None:
        backend = self._mission_authority_backend()

        snapshot = UiBackendNode._mission_dispatch_snapshot(backend)
        UiBackendNode._publish_destination_dispatch_status(
            backend,
            "B1",
            True,
            "http_ui_destination",
            {"message": "accepted"},
        )
        status = json.loads(
            backend.pub_destination_dispatch_status.messages[-1].data
        )

        self.assertTrue(snapshot["mission_dispatch_active"])
        self.assertEqual(snapshot["mission_dispatch_owner"], "operator")
        self.assertEqual(snapshot["mission_dispatch_intent"], "delivery")
        self.assertEqual(snapshot["mission_dispatch_generation"], 41)
        self.assertTrue(status["accepted"])
        self.assertEqual(status["active_site"], "B1")
        self.assertEqual(status["active_generation"], 41)

    def test_return_admission_rejects_stale_arrival_state(self) -> None:
        backend = self._mission_authority_backend(
            _latest_service_state=int(AvgServiceState.RETURN_WITH_CARGO),
            _return_requested_generation=41,
            _return_progress_generation=41,
        )
        message = AvgServiceState()
        message.state = AvgServiceState.UNLOAD_WAIT
        message.state_name = "UNLOAD_WAIT"
        message.description = "Waiting for unload at site"

        UiBackendNode._on_service_state(backend, message)

        self.assertEqual(
            backend._latest_service_state,
            int(AvgServiceState.RETURN_WITH_CARGO),
        )

    def test_restart_gate_blocks_admission_until_cancel_futures_complete(
        self,
    ) -> None:
        clients = [_BarrierCancelClient(), _BarrierCancelClient()]
        timer = _FakeTimer(0.5, lambda: None)
        events = []
        backend = self._mission_authority_backend(
            _active_mission_site="",
            _active_mission_source="",
            _active_mission_generation=0,
            _active_mission_owner="",
            _active_mission_intent="",
            _startup_recovery_pending=True,
            _startup_fail_closed_attempts=0,
            _startup_nav2_cancel_futures={},
            _startup_nav2_cancel_completed=set(),
            _startup_fail_closed_timer=timer,
            nav2_cancel_action_topics=["/first/cancel", "/second/cancel"],
            nav2_cancel_clients=clients,
            publish_mission_engage_from_destination=False,
        )
        backend._cancel_service_motion_writers = (
            lambda source: events.append(("cancel_writers", source))
        )
        backend._publish_engage = lambda value, source: events.append(
            ("engage", value, source)
        )
        backend._publish_service_state = lambda state, source: events.append(
            ("state", state, source)
        )
        backend._schedule_broadcast = lambda payload: events.append(
            ("broadcast", payload)
        )
        backend.destroy_timer = lambda value: events.append(
            ("destroy_timer", value)
        )

        blocked = UiBackendNode._apply_destination_command(
            backend, site="B2", run=True, source="http_ui_destination"
        )
        self.assertEqual(blocked["error"], "backend_startup_recovery")

        UiBackendNode._reassert_startup_fail_closed(backend)
        self.assertTrue(backend._startup_recovery_pending)
        self.assertEqual(sum(len(client.futures) for client in clients), 2)
        for client in clients:
            client.futures[0].completed = True
        UiBackendNode._reassert_startup_fail_closed(backend)

        self.assertFalse(backend._startup_recovery_pending)
        self.assertTrue(timer.cancelled)
        self.assertIsNone(backend._startup_fail_closed_timer)

    def test_ui_camping_operation_cancel_uses_full_operator_stop_path(self) -> None:
        events = []
        backend = SimpleNamespace(
            _destination_dispatch_lock=threading.RLock(),
            _active_mission_site="B8",
            _active_mission_source="guest:kiosk",
            _active_mission_generation=7,
            _return_requested_generation=0,
            get_logger=lambda: _FakeLogger(),
        )
        backend._stop_active_service_serialized = (
            lambda source: events.append(("stop", source))
        )
        message = MotionOperation()
        message.operation = MotionOperation.CANCEL
        message.source = "guest:site=B8:g=7:return_cancel"

        UiBackendNode._on_ui_camping_site_operation_request(backend, message)

        self.assertEqual(
            events, [("stop", "guest:site=B8:g=7:return_cancel")]
        )

    def test_ui_camping_operation_return_behavior_is_preserved(self) -> None:
        events = []
        backend = SimpleNamespace(
            _destination_dispatch_lock=threading.RLock(),
            _active_mission_site="B8",
            _active_mission_source="guest:kiosk",
            _active_mission_generation=7,
            _return_requested_generation=0,
            _latest_service_state=int(AvgServiceState.GUEST_LOADING_WAIT),
            _startup_recovery_pending=False,
            get_logger=lambda: _FakeLogger(),
        )
        backend.request_owned_return_to_drop_zone = lambda *args, **kwargs: (
            events.append(("return", args, kwargs)) or {"success": True}
        )
        message = MotionOperation()
        message.operation = MotionOperation.RETURN
        message.source = "guest:site=B8:g=7:return"

        UiBackendNode._on_ui_camping_site_operation_request(backend, message)

        self.assertEqual(events[0][0], "return")
        self.assertEqual(events[0][1], ("B8", 7))
        self.assertEqual(events[0][2]["source"], "guest:site=B8:g=7:return")

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
        events = []
        backend = SimpleNamespace(
            _latest_service_state=int(AvgServiceState.DROP_ZONE_WAIT),
            _latest_platform_is_charging=False,
            _latest_platform_control_mode=1,
            redock_require_can_control_mode=True,
            parking_rearm_hold_s=0.0,
            publish_engage_from_destination=True,
            publish_mission_engage_from_destination=True,
            _publish_parking_operation=lambda operation, source: events.append(
                ("parking", operation, source)
            ),
            _publish_drop_zone_operation=lambda operation, source: events.append(
                ("drop_zone", operation, source)
            ),
            _publish_engage=lambda enabled, source: events.append(
                ("engage", enabled, source)
            ),
            _publish_mission_engage=lambda enabled, source: events.append(
                ("mission_engage", enabled, source)
            ),
        )

        action = UiBackendNode._request_return_to_drop_zone(
            backend, "test:station"
        )

        self.assertEqual(action, "parking_alignment")
        self.assertEqual(
            [(event[0], event[1]) for event in events],
            [
                ("parking", MotionOperation.CANCEL),
                ("drop_zone", MotionOperation.CANCEL),
                ("engage", True),
                ("mission_engage", True),
                ("drop_zone", MotionOperation.ALIGN_FOR_PARKING),
            ],
        )

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
        events = []
        backend = SimpleNamespace(
            _latest_service_state=int(AvgServiceState.CHARGING),
            _latest_platform_is_charging=False,
            _latest_platform_control_mode=1,
            redock_require_can_control_mode=True,
            parking_rearm_hold_s=0.0,
            publish_engage_from_destination=False,
            publish_mission_engage_from_destination=True,
            _publish_parking_operation=lambda operation, source: events.append(
                ("parking", operation, source)
            ),
            _publish_drop_zone_operation=lambda operation, source: events.append(
                ("drop_zone", operation, source)
            ),
            _publish_mission_engage=lambda enabled, source: events.append(
                ("mission_engage", enabled, source)
            ),
        )

        action = UiBackendNode._request_return_to_drop_zone(
            backend, "test:lost_contact"
        )

        self.assertEqual(action, "parking_alignment")
        self.assertEqual(
            [(event[0], event[1]) for event in events],
            [
                ("parking", MotionOperation.CANCEL),
                ("drop_zone", MotionOperation.CANCEL),
                ("mission_engage", True),
                ("drop_zone", MotionOperation.ALIGN_FOR_PARKING),
            ],
        )

    def test_return_while_charging_waits_for_release_then_redocks_once(self) -> None:
        events = []
        broadcasts = []
        backend = SimpleNamespace(
            _latest_service_state=int(AvgServiceState.CHARGING),
            _latest_platform_is_charging=True,
            _latest_platform_control_mode=1,
            redock_require_can_control_mode=True,
            _redock_after_disconnect_lock=threading.Lock(),
            _redock_after_disconnect_pending=False,
            _redock_after_disconnect_source="",
            _redock_after_disconnect_requested_at_s=0.0,
            redock_request_timeout_s=10.0,
            parking_rearm_hold_s=0.0,
            publish_engage_from_destination=False,
            publish_mission_engage_from_destination=True,
            _runtime_policy=SimpleNamespace(update_platform=lambda **_kwargs: None),
            _update_runtime_state=lambda callback: callback(),
            _publish_service_state=lambda state, source: events.append(
                ("service_state", state, source)
            ),
            _publish_parking_operation=lambda operation, source: events.append(
                ("parking", operation, source)
            ),
            _publish_drop_zone_operation=lambda operation, source: events.append(
                ("drop_zone", operation, source)
            ),
            _publish_mission_engage=lambda enabled, source: events.append(
                ("mission_engage", enabled, source)
            ),
            _schedule_broadcast=broadcasts.append,
            get_logger=lambda: _FakeLogger(),
        )

        action = UiBackendNode._request_return_to_drop_zone(
            backend, "test:charged_redock"
        )
        self.assertEqual(action, "waiting_for_disconnect")
        self.assertTrue(backend._redock_after_disconnect_pending)
        self.assertEqual(events, [])

        released = AvgPlatformStatus()
        released.is_charging = False
        released.control_mode = 1
        released.battery_state_available = False
        UiBackendNode._on_platform_status(backend, released)

        self.assertFalse(backend._redock_after_disconnect_pending)
        self.assertEqual(
            [(event[0], event[1]) for event in events],
            [
                ("service_state", AvgServiceState.DROP_ZONE_WAIT),
                ("parking", MotionOperation.CANCEL),
                ("drop_zone", MotionOperation.CANCEL),
                ("mission_engage", True),
                ("drop_zone", MotionOperation.ALIGN_FOR_PARKING),
            ],
        )

        UiBackendNode._on_platform_status(backend, released)
        self.assertEqual(
            sum(
                event[0] == "drop_zone"
                and event[1] == MotionOperation.ALIGN_FOR_PARKING
                for event in events
            ),
            1,
        )

    def test_parking_rearm_waits_for_old_parked_heartbeat_interval(self) -> None:
        events = []
        timers = []
        backend = SimpleNamespace(
            parking_rearm_hold_s=0.6,
            _latest_platform_is_charging=False,
            _latest_platform_control_mode=1,
            redock_require_can_control_mode=True,
            publish_engage_from_destination=False,
            publish_mission_engage_from_destination=True,
            _parking_rearm_transition_lock=threading.Lock(),
            _parking_rearm_transition_pending=False,
            _parking_rearm_transition_timer=None,
            _parking_rearm_transition_source="",
            _publish_parking_operation=lambda operation, source: events.append(
                ("parking", operation, source)
            ),
            _publish_drop_zone_operation=lambda operation, source: events.append(
                ("drop_zone", operation, source)
            ),
            _publish_mission_engage=lambda enabled, source: events.append(
                ("mission_engage", enabled, source)
            ),
            create_timer=lambda period, callback: timers.append(
                _FakeTimer(period, callback)
            ) or timers[-1],
            destroy_timer=lambda timer: events.append(("destroy_timer", timer)),
        )

        UiBackendNode._start_drop_zone_parking_alignment(
            backend, "test:serialized_redock"
        )
        self.assertEqual(
            [(event[0], event[1]) for event in events],
            [
                ("parking", MotionOperation.CANCEL),
                ("drop_zone", MotionOperation.CANCEL),
            ],
        )
        hold_timer = next(timer for timer in timers if timer.period_s == 0.6)
        self.assertTrue(backend._parking_rearm_transition_pending)

        hold_timer.callback()
        self.assertFalse(backend._parking_rearm_transition_pending)
        self.assertEqual(
            [(event[0], event[1]) for event in events if event[0] != "destroy_timer"],
            [
                ("parking", MotionOperation.CANCEL),
                ("drop_zone", MotionOperation.CANCEL),
                ("mission_engage", True),
                ("drop_zone", MotionOperation.ALIGN_FOR_PARKING),
            ],
        )

    def test_redock_defers_align_until_can_then_resumes_same_generation(self) -> None:
        events = []
        backend = SimpleNamespace(
            _latest_platform_is_charging=False,
            _latest_platform_control_mode=3,
            redock_require_can_control_mode=True,
            parking_rearm_hold_s=0.0,
            redock_request_timeout_s=10.0,
            publish_engage_from_destination=False,
            publish_mission_engage_from_destination=True,
            _publish_parking_operation=lambda operation, source: events.append(
                ("parking", operation, source)
            ),
            _publish_drop_zone_operation=lambda operation, source: events.append(
                ("drop_zone", operation, source)
            ),
            _publish_mission_engage=lambda enabled, source: events.append(
                ("mission_engage", enabled, source)
            ),
            _schedule_broadcast=lambda payload: events.append(("broadcast", payload)),
        )

        self.assertTrue(UiBackendNode._start_drop_zone_parking_alignment(
            backend, "test:rc_wait"
        ))
        self.assertTrue(backend._parking_rearm_waiting_for_can)
        self.assertFalse(any(
            event[0] == "drop_zone"
            and event[1] == MotionOperation.ALIGN_FOR_PARKING
            for event in events
        ))

        backend._latest_platform_control_mode = 1
        request = UiBackendNode._take_parking_rearm_waiting_for_can(backend)
        self.assertIsNotNone(request)
        source, generation = request
        self.assertTrue(UiBackendNode._start_drop_zone_parking_alignment(
            backend, source, generation=generation
        ))
        self.assertEqual(
            sum(
                event[0] == "drop_zone"
                and event[1] == MotionOperation.ALIGN_FOR_PARKING
                for event in events
            ),
            1,
        )

    def test_stop_generation_invalidates_taken_release_before_start(self) -> None:
        events = []
        backend = SimpleNamespace(
            _latest_platform_is_charging=True,
            redock_request_timeout_s=10.0,
            parking_rearm_hold_s=0.0,
            _publish_parking_operation=lambda operation, source: events.append(
                ("parking", operation, source)
            ),
            _publish_drop_zone_operation=lambda operation, source: events.append(
                ("drop_zone", operation, source)
            ),
        )
        queued, _ = UiBackendNode._queue_redock_after_disconnect(
            backend, "test:race"
        )
        self.assertTrue(queued)
        backend._latest_platform_is_charging = False
        request = UiBackendNode._take_pending_redock_after_disconnect(backend)
        self.assertIsNotNone(request)

        UiBackendNode._cancel_pending_redock_after_disconnect(backend, "stop")
        source, generation = request
        self.assertFalse(UiBackendNode._start_drop_zone_parking_alignment(
            backend, source, generation=generation
        ))
        self.assertEqual(events, [])

    def test_concurrent_second_return_does_not_invalidate_first_rearm(self) -> None:
        events = []
        timers = []
        backend = SimpleNamespace(
            _latest_platform_is_charging=False,
            _latest_platform_control_mode=1,
            redock_require_can_control_mode=True,
            redock_request_timeout_s=10.0,
            parking_rearm_hold_s=0.6,
            _publish_parking_operation=lambda operation, source: events.append(
                ("parking", operation, source)
            ),
            _publish_drop_zone_operation=lambda operation, source: events.append(
                ("drop_zone", operation, source)
            ),
            create_timer=lambda period, callback: timers.append(
                _FakeTimer(period, callback)
            ) or timers[-1],
            destroy_timer=lambda _timer: None,
        )

        self.assertTrue(UiBackendNode._start_drop_zone_parking_alignment(
            backend, "test:first"
        ))
        first_generation = backend._redock_generation
        self.assertFalse(UiBackendNode._start_drop_zone_parking_alignment(
            backend, "test:second"
        ))
        self.assertEqual(backend._redock_generation, first_generation)
        self.assertEqual(
            [(event[0], event[1]) for event in events],
            [
                ("parking", MotionOperation.CANCEL),
                ("drop_zone", MotionOperation.CANCEL),
            ],
        )
        self.assertEqual(sum(timer.period_s == 0.6 for timer in timers), 1)

    def test_queue_rechecks_a_release_edge_before_installing_pending(self) -> None:
        events = []
        backend = SimpleNamespace(
            _latest_platform_is_charging=False,
            _latest_platform_control_mode=1,
            redock_require_can_control_mode=True,
            redock_request_timeout_s=10.0,
            parking_rearm_hold_s=0.0,
            publish_engage_from_destination=False,
            publish_mission_engage_from_destination=True,
            _publish_parking_operation=lambda operation, source: events.append(
                ("parking", operation, source)
            ),
            _publish_drop_zone_operation=lambda operation, source: events.append(
                ("drop_zone", operation, source)
            ),
            _publish_mission_engage=lambda enabled, source: events.append(
                ("mission_engage", enabled, source)
            ),
        )

        queued, generation = UiBackendNode._queue_redock_after_disconnect(
            backend, "test:edge_won"
        )
        self.assertFalse(queued)
        self.assertTrue(UiBackendNode._start_drop_zone_parking_alignment(
            backend, "test:edge_won", generation=generation
        ))
        self.assertFalse(backend._redock_after_disconnect_pending)
        self.assertTrue(any(
            event[0] == "drop_zone"
            and event[1] == MotionOperation.ALIGN_FOR_PARKING
            for event in events
        ))

    def test_can_wait_expires_and_cannot_move_later(self) -> None:
        events = []
        timers = []
        backend = SimpleNamespace(
            _latest_platform_is_charging=False,
            _latest_platform_control_mode=3,
            redock_require_can_control_mode=True,
            redock_request_timeout_s=10.0,
            parking_rearm_hold_s=0.0,
            publish_engage_from_destination=False,
            publish_mission_engage_from_destination=True,
            _publish_parking_operation=lambda operation, source: events.append(
                ("parking", operation, source)
            ),
            _publish_drop_zone_operation=lambda operation, source: events.append(
                ("drop_zone", operation, source)
            ),
            _schedule_broadcast=lambda payload: events.append(("broadcast", payload)),
            create_timer=lambda period, callback: timers.append(
                _FakeTimer(period, callback)
            ) or timers[-1],
            destroy_timer=lambda timer: events.append(("destroy_timer", timer)),
            get_logger=lambda: _FakeLogger(),
        )

        UiBackendNode._start_drop_zone_parking_alignment(backend, "test:expiry")
        self.assertTrue(backend._parking_rearm_waiting_for_can)
        expiry_timer = next(timer for timer in timers if timer.period_s == 10.0)
        expiry_timer.callback()
        self.assertFalse(backend._parking_rearm_waiting_for_can)
        expiry_updates = [
            event[1]
            for event in events
            if event[0] == "broadcast"
            and event[1].get("redock_status") == "expired"
        ]
        self.assertEqual(len(expiry_updates), 1)
        self.assertFalse(expiry_updates[0]["redock_waiting_for_can"])
        self.assertEqual(backend._redock_status, "expired")
        self.assertEqual(
            UiBackendNode._redock_status_snapshot(backend)["redock_status"],
            "expired",
        )
        backend._latest_platform_control_mode = 1
        self.assertIsNone(
            UiBackendNode._take_parking_rearm_waiting_for_can(backend)
        )
        self.assertFalse(any(
            event[0] == "drop_zone"
            and event[1] == MotionOperation.ALIGN_FOR_PARKING
            for event in events
        ))

    def test_contact_bounce_does_not_extend_original_redock_deadline(self) -> None:
        timers = []
        backend = SimpleNamespace(
            _latest_platform_is_charging=True,
            redock_request_timeout_s=10.0,
            create_timer=lambda period, callback: timers.append(
                _FakeTimer(period, callback)
            ) or timers[-1],
            destroy_timer=lambda _timer: None,
        )
        with mock.patch(
            "camrod_ui.ui_backend_node.time.monotonic", return_value=1.0
        ):
            queued, generation = UiBackendNode._queue_redock_after_disconnect(
                backend, "test:first_contact"
            )
        self.assertTrue(queued)
        original_timer = backend._redock_after_disconnect_timer
        with mock.patch(
            "camrod_ui.ui_backend_node.time.monotonic", return_value=8.0
        ):
            UiBackendNode._queue_redock_after_disconnect_locked(
                backend, "test:contact_bounce", generation
            )
        self.assertEqual(backend._redock_after_disconnect_requested_at_s, 1.0)
        self.assertIs(backend._redock_after_disconnect_timer, original_timer)

    def test_only_selected_parking_controller_error_enables_manual_retry(self) -> None:
        def make_backend(method):
            events = []
            backend = SimpleNamespace(
                parking_method=method,
                _parking_controller_operating_states={
                    "reverse_parking": "ERROR",
                    "apriltag_parking": "PARKED",
                },
                _latest_service_state=int(AvgServiceState.DROP_ZONE_PARKING),
                _latest_platform_is_charging=False,
                _latest_platform_control_mode=1,
                redock_require_can_control_mode=True,
                redock_request_timeout_s=10.0,
                parking_rearm_hold_s=0.0,
                publish_engage_from_destination=False,
                publish_mission_engage_from_destination=True,
                _publish_parking_operation=lambda operation, source: events.append(
                    ("parking", operation, source)
                ),
                _publish_drop_zone_operation=lambda operation, source: events.append(
                    ("drop_zone", operation, source)
                ),
                _publish_mission_engage=lambda enabled, source: events.append(
                    ("mission_engage", enabled, source)
                ),
            )
            return backend, events

        apriltag_backend, apriltag_events = make_backend("apriltag")
        self.assertEqual(
            UiBackendNode._request_return_to_drop_zone(
                apriltag_backend, "test:non_selected_error"
            ),
            "parking_in_progress",
        )
        self.assertEqual(apriltag_events, [])

        reverse_backend, reverse_events = make_backend("reverse")
        self.assertEqual(
            UiBackendNode._request_return_to_drop_zone(
                reverse_backend, "test:selected_error"
            ),
            "parking_alignment",
        )
        self.assertTrue(any(
            event[0] == "drop_zone"
            and event[1] == MotionOperation.ALIGN_FOR_PARKING
            for event in reverse_events
        ))

    def test_redock_started_in_rc_mode_reports_can_mode_wait(self) -> None:
        backend = SimpleNamespace(
            _latest_service_state=int(AvgServiceState.DROP_ZONE_WAIT),
            _latest_platform_is_charging=False,
            _latest_platform_control_mode=3,
            parking_rearm_hold_s=0.0,
            publish_engage_from_destination=False,
            publish_mission_engage_from_destination=True,
            _publish_parking_operation=lambda *_args, **_kwargs: None,
            _publish_drop_zone_operation=lambda *_args, **_kwargs: None,
            _publish_mission_engage=lambda *_args, **_kwargs: None,
            _schedule_broadcast=lambda *_args, **_kwargs: None,
        )

        action = UiBackendNode._request_return_to_drop_zone(
            backend, "test:rc_redock"
        )

        self.assertEqual(action, "parking_alignment_waiting_for_can")
        self.assertTrue(backend._parking_rearm_waiting_for_can)

    def test_queued_redock_expires_instead_of_moving_on_a_late_unplug(self) -> None:
        broadcasts = []
        logger = _FakeLogger()
        backend = SimpleNamespace(
            _redock_after_disconnect_lock=threading.Lock(),
            _redock_after_disconnect_pending=True,
            _redock_after_disconnect_source="test:stale_redock",
            _redock_after_disconnect_requested_at_s=1.0,
            redock_request_timeout_s=10.0,
            _schedule_broadcast=broadcasts.append,
            get_logger=lambda: logger,
        )

        with mock.patch(
            "camrod_ui.ui_backend_node.time.monotonic", return_value=20.0
        ):
            source = UiBackendNode._take_pending_redock_after_disconnect(backend)

        self.assertIsNone(source)
        self.assertFalse(backend._redock_after_disconnect_pending)
        self.assertEqual(broadcasts[-1]["redock_pending"], False)
        self.assertEqual(broadcasts[-1]["redock_status"], "expired")
        self.assertIn("expired", broadcasts[-1]["redock_message"])
        self.assertIn("queued re-dock expired", logger.warning_messages[-1])

    def test_redock_snapshot_preserves_async_wait_state_for_reconnect(self) -> None:
        backend = SimpleNamespace(
            _redock_after_disconnect_lock=threading.RLock(),
            _redock_after_disconnect_pending=True,
            _parking_rearm_waiting_for_can=False,
            _redock_status="waiting_for_disconnect",
            _redock_status_message="Charging release pending",
        )

        self.assertEqual(
            UiBackendNode._redock_status_snapshot(backend),
            {
                "redock_pending": True,
                "redock_waiting_for_can": False,
                "redock_status": "waiting_for_disconnect",
                "redock_message": "Charging release pending",
            },
        )

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

    def test_platform_velocity_is_recorded_without_battery_telemetry(self) -> None:
        metrics = _FakeServiceMetrics()
        backend = SimpleNamespace(
            _drop_zone_exit_active=False,
            _latest_platform_is_charging=False,
            _latest_service_state=int(AvgServiceState.MOVING_TO_SITE),
            _runtime_policy=SimpleNamespace(update_platform=lambda **_kwargs: None),
            _service_metrics=metrics,
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._update_runtime_state = lambda callback: callback()
        backend._publish_service_state = lambda *_args, **_kwargs: None
        backend._update_low_battery_return_policy = lambda *_args, **_kwargs: None
        backend._now_s = lambda: 99.0

        message = AvgPlatformStatus()
        message.velocity.header.stamp.sec = 12
        message.velocity.header.stamp.nanosec = 500_000_000
        message.velocity.twist.linear.x = 1.25
        message.velocity.twist.linear.y = -0.5
        message.battery_state_available = False

        UiBackendNode._on_platform_status(backend, message)

        self.assertEqual(metrics.velocities, [(1.25, -0.5, 12.5)])

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

    def test_duplicate_destination_is_idempotent_during_charging_dwell(self) -> None:
        logger = _FakeLogger()
        backend = SimpleNamespace(
            _drop_zone_exit_active=False,
            _charging_departure_delay_pending=True,
            _pending_site_after_drop_zone_exit=("B2", "camping_site_2", "first"),
        )
        backend.get_logger = lambda: logger

        result = UiBackendNode._apply_destination_command(
            backend, site="B2", run=True, source="retry"
        )

        self.assertEqual(result["mission_key"], "camping_site_2")
        self.assertFalse(result["goal_pose_published"])
        self.assertIn("already pending", result["message"])

    def test_guest_calls_b1_through_b13_publish_typed_roadside_recall(self) -> None:
        events = []
        recalls = []

        class RecallPublisher:
            @staticmethod
            def publish(message) -> None:
                recalls.append(message)
                events.append(("recall", message.site_name, message.source))

        backend = SimpleNamespace(
            _drop_zone_exit_active=False,
            _charging_departure_delay_pending=False,
            _pending_site_after_drop_zone_exit=None,
            _drop_zone_exit_handoff_ready=True,
            _drop_zone_exit_cancel_suppressed=False,
            _latest_platform_is_charging=False,
            _latest_service_state=int(AvgServiceState.MOVING_TO_SITE),
            _active_mission_site="",
            _service_metrics=None,
            _lock=threading.Lock(),
            _pending_site_route_goal_stamps={},
            _site_route_anchors={},
            _runtime_policy=SimpleNamespace(update_goal_received=lambda _mode: None),
            publish_engage_from_destination=True,
            publish_mission_engage_from_destination=True,
            planning_camping_site_recall_topic=(
                "/planning/state_machine/camping_site_recall"
            ),
            pub_planning_camping_site_recall=RecallPublisher(),
            charging_departure_delay_s=7.0,
        )
        recall_stamp_nanoseconds = iter(range(7, 20))
        backend.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: RosTime(
                    sec=9, nanosec=next(recall_stamp_nanoseconds)
                )
            )
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._resolve_mission_key_for_site = (
            lambda site: f"camping_site_{int(site[1:])}"
        )
        # A tent is expected at every called site. Guest recall must therefore
        # bypass both the delivery occupancy guard and in-site adoption check.
        backend._is_site_occupied = lambda site: self.fail(
            f"guest recall consulted delivery occupancy for {site}"
        )
        backend._site_arrival_match = lambda site: self.fail(
            f"guest recall consulted delivery arrival adoption for {site}"
        )
        backend._mission_dispatch_battery_block = lambda _site: None
        backend._update_runtime_state = lambda callback: callback()
        backend._publish_engage = lambda enabled, source: events.append(
            ("engage", enabled, source)
        )
        backend._publish_service_state = lambda state, source: events.append(
            ("state", state, source)
        )
        backend._publish_mission_engage = lambda enabled, source: events.append(
            ("mission_engage", enabled, source)
        )
        backend._publish_platform_drive_enable = lambda enabled, source: events.append(
            ("platform", enabled, source)
        )
        backend._publish_goal_for_site = lambda **kwargs: self.fail(
            f"guest recall published ordinary delivery goal: {kwargs}"
        )

        with mock.patch.object(
            UiBackendNode,
            "_cancel_pending_redock_after_disconnect",
            return_value=None,
        ), mock.patch.object(
            UiBackendNode,
            "_cancel_pending_parking_rearm_transition",
            return_value=None,
        ):
            results = []
            for index in range(1, 14):
                UiBackendNode._clear_active_mission_identity(backend)
                results.append(
                    UiBackendNode._apply_destination_command(
                        backend, site=f"B{index}", run=True, source="guest"
                    )
                )

        self.assertEqual(
            [message.site_name for message in recalls],
            [f"camping_site_{index}" for index in range(1, 14)],
        )
        self.assertEqual(
            backend._pending_site_route_goal_stamps,
            {
                (9, index + 6): f"camping_site_{index}"
                for index in range(1, 14)
            },
        )
        self.assertEqual([message.source for message in recalls], ["guest"] * 13)
        self.assertTrue(all(result["recall_request_published"] for result in results))
        self.assertTrue(all(not result["goal_pose_published"] for result in results))
        self.assertEqual(
            [event[1] for event in events if event[0] == "state"],
            [AvgServiceState.RECALL_TO_SITE_ROAD] * 13,
        )

    def test_destination_topic_preserves_guest_prefix_for_recall_dispatch(self) -> None:
        dispatches = []
        broadcasts = []
        backend = SimpleNamespace()
        backend.get_logger = lambda: _FakeLogger()
        backend._is_recent_direct_destination_echo = lambda *_args: False
        backend._apply_destination_command = lambda site, run, source: (
            dispatches.append((site, run, source))
            or {"mission_key": "camping_site_7", "recall_request_published": True}
        )
        backend._schedule_broadcast = broadcasts.append

        command = UiDestinationCommand()
        command.site = "B7"
        command.run = True
        command.source = "  guest:kiosk  "
        UiBackendNode._on_destination_command(backend, command)

        self.assertEqual(dispatches, [("B7", True, "guest:kiosk")])
        self.assertEqual(broadcasts, [{"guest_navigate": "B7"}])

    def test_robot_ui_recall_uses_explicit_roadside_source_and_skips_occupancy(self) -> None:
        events = []
        backend = SimpleNamespace(
            site_names=["B1", "B2"],
            _lock=threading.Lock(),
            _state=SimpleNamespace(ws_site_states={"B1": False, "B2": False}),
            _last_direct_destination_echo=None,
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(nanoseconds=12_500_000_000)
            ),
        )
        backend._mission_dispatch_battery_block = lambda _site: None
        backend._schedule_broadcast = lambda payload: events.append(
            ("broadcast", payload)
        )
        backend._publish_destination_command = lambda site, run, source: (
            events.append(("destination", site, run, source))
            or {"site": site, "run": run}
        )
        backend._apply_destination_command = lambda site, run, source: (
            events.append(("dispatch", site, run, source))
            or {
                "recall_request_published": False,
                "message": "recall request pending drop-zone exit",
            }
        )
        backend._is_site_occupied = lambda site: self.fail(
            f"robot recall incorrectly consulted occupancy for {site}"
        )

        result = UiBackendNode.set_campsite_recall(backend, "B2")

        self.assertTrue(result["success"])
        self.assertEqual(result["intent"], "recall")
        self.assertEqual(result["site"], "B2")
        self.assertEqual(backend._state.ws_site_states, {"B1": False, "B2": True})
        self.assertIn(
            ("destination", "B2", True, "robot_ui:recall"), events
        )
        self.assertIn(("dispatch", "B2", True, "robot_ui:recall"), events)
        self.assertTrue(
            UiBackendNode._is_guest_recall_source("robot_ui:recall")
        )
        self.assertFalse(
            UiBackendNode._is_guest_recall_source("http_ui_destination")
        )
        self.assertEqual(
            backend._last_direct_destination_echo,
            ("B2", True, "robot_ui:recall", 12.5),
        )

    def test_operator_destination_stays_normal_delivery_not_guest_recall(self) -> None:
        events = []

        class RecallPublisher:
            @staticmethod
            def publish(message) -> None:
                events.append(("recall", message))

        backend = SimpleNamespace(
            _drop_zone_exit_active=False,
            _charging_departure_delay_pending=False,
            _pending_site_after_drop_zone_exit=None,
            _drop_zone_exit_handoff_ready=True,
            _drop_zone_exit_cancel_suppressed=False,
            _latest_platform_is_charging=False,
            _latest_service_state=int(AvgServiceState.MOVING_TO_SITE),
            _active_mission_site="",
            _service_metrics=None,
            _lock=threading.Lock(),
            _runtime_policy=SimpleNamespace(update_goal_received=lambda _mode: None),
            publish_engage_from_destination=True,
            publish_mission_engage_from_destination=True,
            pub_planning_camping_site_recall=RecallPublisher(),
            charging_departure_delay_s=7.0,
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._resolve_mission_key_for_site = lambda _site: "camping_site_4"
        backend._is_site_occupied = lambda _site: False
        backend._site_arrival_match = lambda _site: (
            False,
            "camping_site_4",
            10.0,
            "outside_arrival_radius",
        )
        backend._mission_dispatch_battery_block = lambda _site: None
        backend._update_runtime_state = lambda callback: callback()
        backend._publish_engage = lambda enabled, source: events.append(
            ("engage", enabled, source)
        )
        backend._publish_mission_engage = lambda enabled, source: events.append(
            ("mission_engage", enabled, source)
        )
        backend._publish_service_state = lambda state, source: events.append(
            ("state", state, source)
        )
        backend._publish_goal_for_site = lambda site, source: {
            "mission_key": "camping_site_4",
            "goal_pose_published": events.append(("goal", site, source)) is None,
            "message": "ok",
        }

        with mock.patch.object(
            UiBackendNode,
            "_cancel_pending_redock_after_disconnect",
            return_value=None,
        ), mock.patch.object(
            UiBackendNode,
            "_cancel_pending_parking_rearm_transition",
            return_value=None,
        ):
            result = UiBackendNode._apply_destination_command(
                backend, site="B4", run=True, source="http_ui_destination"
            )

        self.assertTrue(result["goal_pose_published"])
        self.assertFalse(result["recall_request_published"])
        self.assertIn(("state", AvgServiceState.MOVING_TO_SITE, "http_ui_destination:start"), events)
        self.assertIn(("goal", "B4", "http_ui_destination"), events)
        self.assertNotIn("recall", [event[0] for event in events])

    def test_guest_recall_from_fresh_backend_exits_drop_zone_before_planning(self) -> None:
        events = []
        recalls = []

        class RecallPublisher:
            @staticmethod
            def publish(message) -> None:
                recalls.append(message)
                events.append(("recall", message.site_name, message.source))

        backend = SimpleNamespace(
            _drop_zone_exit_active=False,
            _charging_departure_delay_pending=False,
            _pending_site_after_drop_zone_exit=None,
            _drop_zone_exit_handoff_ready=False,
            _drop_zone_exit_failure_latched=False,
            _drop_zone_exit_waiting_for_fresh_status=False,
            _drop_zone_exit_cancel_suppressed=False,
            # This mirrors the field failure: the backend started without a
            # retained service-state sample while the robot was physically
            # parked at the drop zone.
            _latest_platform_is_charging=False,
            _latest_service_state=None,
            _active_mission_site="",
            _service_metrics=None,
            _lock=threading.Lock(),
            _pending_site_route_goal_stamps={},
            _site_route_anchors={},
            _runtime_policy=SimpleNamespace(update_goal_received=lambda _mode: None),
            publish_engage_from_destination=True,
            publish_mission_engage_from_destination=True,
            charging_departure_delay_s=0.0,
            _charging_departure_from_charger=False,
            planning_camping_site_recall_topic=(
                "/planning/state_machine/camping_site_recall"
            ),
            pub_planning_camping_site_recall=RecallPublisher(),
        )
        backend.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: RosTime(sec=11, nanosec=3))
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._resolve_mission_key_for_site = lambda _site: "camping_site_11"
        backend._is_site_occupied = lambda site: self.fail(
            f"guest recall consulted delivery occupancy for {site}"
        )
        backend._site_arrival_match = lambda site: self.fail(
            f"guest recall consulted delivery arrival adoption for {site}"
        )
        backend._mission_dispatch_battery_block = lambda _site: None
        backend._update_runtime_state = lambda callback: callback()
        backend._schedule_broadcast = lambda payload: events.append(
            ("broadcast", payload)
        )
        backend._publish_engage = lambda enabled, source, **kwargs: events.append(
            ("engage", enabled, source)
        )
        backend._publish_mission_engage = lambda enabled, source: events.append(
            ("mission_engage", enabled, source)
        )
        backend._publish_platform_drive_enable = lambda enabled, source: events.append(
            ("platform", enabled, source)
        )
        backend._publish_parking_operation = lambda operation, source: events.append(
            ("parking_operation", operation, source)
        )
        backend._publish_drop_zone_operation = lambda operation, source: events.append(
            ("drop_zone_operation", operation, source)
        )

        def publish_state(state, source):
            backend._latest_service_state = int(state)
            events.append(("state", state, source))

        backend._publish_service_state = publish_state
        backend._publish_site_mission_key = lambda *args: self.fail(
            f"guest recall published ordinary mission key: {args}"
        )
        backend._publish_site_goal_pose = lambda *args: self.fail(
            f"guest recall published ordinary site goal: {args}"
        )

        with mock.patch.object(
            UiBackendNode,
            "_cancel_pending_redock_after_disconnect",
            return_value=None,
        ), mock.patch.object(
            UiBackendNode,
            "_cancel_pending_parking_rearm_transition",
            return_value=None,
        ):
            result = UiBackendNode._apply_destination_command(
                backend, site="B11", run=True, source="guest:kiosk"
            )

        self.assertFalse(result["recall_request_published"])
        self.assertEqual(
            backend._pending_site_after_drop_zone_exit,
            ("B11", "camping_site_11", "guest:kiosk"),
        )
        self.assertEqual(recalls, [])
        self.assertEqual(
            [event[1] for event in events if event[0] == "drop_zone_operation"],
            [MotionOperation.CANCEL, MotionOperation.EXIT],
        )
        self.assertEqual(
            [event[1] for event in events if event[0] == "parking_operation"],
            [MotionOperation.CANCEL],
        )
        self.assertIn(
            (
                "state",
                AvgServiceState.DEPARTING_DROP_ZONE,
                "guest:kiosk:drop_zone_departure",
            ),
            events,
        )

        complete = AvgBool()
        complete.data = True
        UiBackendNode._on_drop_zone_exit_complete(backend, complete)

        self.assertEqual(len(recalls), 1)
        self.assertEqual(recalls[0].site_name, "camping_site_11")
        self.assertEqual(recalls[0].source, "guest:kiosk")
        self.assertEqual(
            [event[1] for event in events if event[0] == "state"][-1],
            AvgServiceState.RECALL_TO_SITE_ROAD,
        )
        self.assertFalse(backend._drop_zone_exit_active)
        self.assertIsNone(backend._pending_site_after_drop_zone_exit)

    def test_charging_destination_waits_seven_seconds_before_one_exit(self) -> None:
        events = []
        timers = []

        def create_timer(period_s, callback):
            timer = _FakeTimer(period_s, callback)
            timers.append(timer)
            events.append(("timer", period_s))
            return timer

        backend = SimpleNamespace(
            _drop_zone_exit_active=False,
            _pending_site_after_drop_zone_exit=None,
            _drop_zone_exit_handoff_ready=False,
            _drop_zone_exit_failure_latched=False,
            _drop_zone_exit_waiting_for_fresh_status=False,
            _drop_zone_exit_cancel_suppressed=False,
            _latest_platform_is_charging=True,
            _latest_service_state=int(AvgServiceState.CHARGING),
            _active_mission_site="",
            _service_metrics=None,
            _lock=threading.Lock(),
            _runtime_policy=SimpleNamespace(update_goal_received=lambda _mode: None),
            publish_engage_from_destination=False,
            publish_mission_engage_from_destination=True,
            charging_departure_delay_s=7.0,
            _charging_departure_transition_lock=threading.Lock(),
            _charging_departure_delay_pending=False,
            _charging_departure_transition_timer=None,
            _charging_departure_from_charger=False,
            create_timer=create_timer,
            destroy_timer=lambda timer: events.append(("destroy_timer", timer)),
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._resolve_mission_key_for_site = lambda _site: "camping_site_2"
        backend._is_site_occupied = lambda _site: False
        backend._site_arrival_match = lambda _site: (
            False,
            "camping_site_2",
            10.0,
            "outside_arrival_radius",
        )
        backend._mission_dispatch_battery_block = lambda _site: None
        backend._update_runtime_state = lambda callback: callback()
        backend._schedule_broadcast = lambda payload: events.append(
            ("broadcast", payload)
        )
        backend._publish_mission_engage = lambda enabled, source: events.append(
            ("mission_engage", enabled, source)
        )
        backend._publish_engage = lambda enabled, source, **kwargs: events.append(
            ("engage", enabled, source, kwargs)
        )
        backend._publish_platform_drive_enable = (
            lambda enabled, source: events.append(("platform", enabled, source))
        )
        backend._publish_parking_operation = lambda operation, source: events.append(
            ("parking_operation", operation, source)
        )
        backend._publish_drop_zone_operation = lambda operation, source: events.append(
            ("drop_zone_operation", operation, source)
        )
        backend._publish_service_state = lambda state, source: events.append(
            ("state", state, source)
        )

        result = UiBackendNode._apply_destination_command(
            backend, site="B2", run=True, source="charging_test"
        )

        self.assertTrue(result["run"])
        self.assertFalse(result["goal_pose_published"])
        self.assertIn("7.0 s", result["message"])
        self.assertTrue(backend._charging_departure_delay_pending)
        self.assertFalse(backend._drop_zone_exit_active)
        self.assertEqual(len(timers), 1)
        self.assertEqual(timers[0].period_s, 7.0)
        self.assertNotIn("drop_zone_operation", [event[0] for event in events])
        self.assertNotIn("state", [event[0] for event in events])
        self.assertFalse(
            any(event[0] == "mission_engage" and event[1] for event in events)
        )

        timers[0].callback()

        self.assertTrue(timers[0].cancelled)
        self.assertFalse(backend._charging_departure_delay_pending)
        self.assertTrue(backend._drop_zone_exit_active)
        self.assertEqual(
            [
                event[1]
                for event in events
                if event[0] == "drop_zone_operation"
            ],
            [MotionOperation.CANCEL, MotionOperation.EXIT],
        )
        self.assertTrue(
            any(event[0] == "mission_engage" and event[1] for event in events)
        )
        self.assertIn(
            ("state", AvgServiceState.DEPARTING_CHARGER, "charging_test:drop_zone_departure"),
            events,
        )

    def test_operator_stop_cancels_pending_charging_departure_timer(self) -> None:
        destroyed = []
        timer = _FakeTimer(7.0, lambda: None)
        backend = SimpleNamespace(
            _charging_departure_transition_lock=threading.Lock(),
            _charging_departure_delay_pending=True,
            _charging_departure_transition_timer=timer,
            _charging_departure_from_charger=True,
            destroy_timer=lambda value: destroyed.append(value),
        )
        backend.get_logger = lambda: _FakeLogger()

        # HH_260825 - Stop/shutdown must invalidate the one-shot callback before
        # it can reopen mission or platform authorization after a human stop.
        UiBackendNode._cancel_pending_charging_departure_transition(
            backend, "operator_stop"
        )

        self.assertTrue(timer.cancelled)
        self.assertEqual(destroyed, [timer])
        self.assertFalse(backend._charging_departure_delay_pending)
        self.assertIsNone(backend._charging_departure_transition_timer)
        self.assertFalse(backend._charging_departure_from_charger)

    def test_station_states_defer_campsite_identity_and_goal_until_exit(self) -> None:
        for service_state in (
            AvgServiceState.DROP_ZONE_PARKING,
            AvgServiceState.DROP_ZONE_WAIT,
            AvgServiceState.CHARGING,
            AvgServiceState.WAITING_FOR_CHARGING,
        ):
            with self.subTest(service_state=int(service_state)):
                events = []
                backend = SimpleNamespace(
                    _drop_zone_exit_active=False,
                    _pending_site_after_drop_zone_exit=None,
                    _latest_platform_is_charging=False,
                    _latest_service_state=int(service_state),
                    _active_mission_site="",
                    _service_metrics=None,
                    _lock=threading.Lock(),
                    _runtime_policy=SimpleNamespace(
                        update_goal_received=lambda mode: events.append(
                            ("runtime_goal", mode)
                        )
                    ),
                    publish_engage_from_destination=False,
                    publish_mission_engage_from_destination=False,
                )
                backend.get_logger = lambda: _FakeLogger()
                backend._resolve_mission_key_for_site = (
                    lambda _site: "camping_site_2"
                )
                backend._is_site_occupied = lambda _site: False
                backend._site_arrival_match = lambda _site: (
                    False,
                    "camping_site_2",
                    10.0,
                    "outside_arrival_radius",
                )
                backend._mission_dispatch_battery_block = lambda _site: None
                backend._schedule_broadcast = lambda payload: events.append(
                    ("broadcast", payload)
                )
                backend._update_runtime_state = lambda callback: callback()
                backend._publish_site_mission_key = (
                    lambda key, source: events.append(("mission_key", key, source))
                )
                backend._publish_site_goal_pose = (
                    lambda site, key, source: events.append(
                        ("goal", site, key, source)
                    )
                )
                backend._publish_parking_operation = (
                    lambda operation, source: events.append(
                        ("parking_operation", operation, source)
                    )
                )
                backend._publish_drop_zone_operation = (
                    lambda operation, source: events.append(
                        ("drop_zone_operation", operation, source)
                    )
                )
                backend._publish_service_state = (
                    lambda state, source: events.append(("state", state, source))
                )

                result = UiBackendNode._apply_destination_command(
                    backend,
                    site="B2",
                    run=True,
                    source="test",
                )

                self.assertTrue(result["run"])
                self.assertFalse(result["goal_pose_published"])
                self.assertTrue(backend._drop_zone_exit_active)
                self.assertEqual(
                    backend._pending_site_after_drop_zone_exit,
                    ("B2", "camping_site_2", "test"),
                )
                self.assertNotIn("mission_key", [event[0] for event in events])
                self.assertNotIn("goal", [event[0] for event in events])
                operation_events = [
                    event for event in events if event[0].endswith("operation")
                ]
                self.assertEqual(
                    [(event[0], event[1]) for event in operation_events],
                    [
                        ("drop_zone_operation", MotionOperation.CANCEL),
                        ("parking_operation", MotionOperation.CANCEL),
                        ("drop_zone_operation", MotionOperation.EXIT),
                    ],
                )
                self.assertEqual(events[-1][0], "state")

    def test_service_heartbeat_restores_restart_departure_without_duplicate_edges(self) -> None:
        events = []
        metrics = _FakeServiceMetrics()
        backend = SimpleNamespace(
            _drop_zone_exit_active=False,
            _pending_site_after_drop_zone_exit=None,
            _latest_platform_is_charging=False,
            _latest_service_state=None,
            _active_mission_site="",
            _service_metrics=metrics,
            _lock=threading.Lock(),
            _state=SimpleNamespace(
                destination={"site": "", "run": False},
                service_state=-1,
                service_state_name="",
                service_state_description="",
                battery_percentage=80,
            ),
            _runtime_policy=SimpleNamespace(
                update_goal_received=lambda mode: events.append(
                    ("runtime_goal", mode)
                )
            ),
            publish_engage_from_destination=False,
            publish_mission_engage_from_destination=False,
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._schedule_broadcast = lambda payload: events.append(
            ("broadcast", payload)
        )
        backend._publish_engage = lambda enabled, source, **_kwargs: events.append(
            ("engage", enabled, source)
        )
        backend._update_low_battery_return_policy = lambda *_args, **_kwargs: None

        heartbeat = AvgServiceState()
        heartbeat.state = AvgServiceState.WAITING_FOR_CHARGING
        heartbeat.state_name = "WAITING_FOR_CHARGING"
        heartbeat.description = "Waiting for charger connection"
        UiBackendNode._on_service_state(backend, heartbeat)
        first_event_count = len(events)
        UiBackendNode._on_service_state(backend, heartbeat)

        self.assertEqual(
            backend._latest_service_state,
            int(AvgServiceState.WAITING_FOR_CHARGING),
        )
        self.assertEqual(len(metrics.states), 1)
        self.assertEqual(len(events), first_event_count)
        self.assertEqual(
            len([event for event in events if event[0] == "engage"]),
            1,
        )

        # The first post-restart heartbeat must force physical departure.  It
        # may not dispatch the campsite identity/goal before exit_complete.
        events.clear()
        backend._resolve_mission_key_for_site = lambda _site: "camping_site_2"
        backend._is_site_occupied = lambda _site: False
        backend._site_arrival_match = lambda _site: (
            False,
            "camping_site_2",
            10.0,
            "outside_arrival_radius",
        )
        backend._mission_dispatch_battery_block = lambda _site: None
        backend._update_runtime_state = lambda callback: callback()
        backend._publish_site_mission_key = lambda key, source: events.append(
            ("mission_key", key, source)
        )
        backend._publish_site_goal_pose = lambda site, key, source: events.append(
            ("goal", site, key, source)
        )
        backend._publish_parking_operation = lambda operation, source: events.append(
            ("parking_operation", operation, source)
        )
        backend._publish_drop_zone_operation = lambda operation, source: events.append(
            ("drop_zone_operation", operation, source)
        )
        backend._publish_service_state = lambda state, source: events.append(
            ("state", state, source)
        )

        result = UiBackendNode._apply_destination_command(
            backend, site="B2", run=True, source="restart_test"
        )

        self.assertFalse(result["goal_pose_published"])
        self.assertNotIn("mission_key", [event[0] for event in events])
        self.assertNotIn("goal", [event[0] for event in events])
        self.assertIn(
            ("drop_zone_operation", MotionOperation.EXIT),
            [(event[0], event[1]) for event in events],
        )

    def test_restart_attaches_to_active_departure_and_defers_changed_site(self) -> None:
        events = []
        backend = SimpleNamespace(
            _drop_zone_exit_active=False,
            _pending_site_after_drop_zone_exit=None,
            _latest_platform_is_charging=False,
            _latest_service_state=None,
            _active_mission_site="",
            _service_metrics=None,
            _lock=threading.Lock(),
            _state=SimpleNamespace(
                destination={"site": "", "run": False},
                service_state=-1,
                service_state_name="",
                service_state_description="",
                battery_percentage=80,
            ),
            _runtime_policy=SimpleNamespace(update_goal_received=lambda _mode: None),
            publish_engage_from_destination=False,
            publish_mission_engage_from_destination=False,
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._schedule_broadcast = lambda payload: events.append(
            ("broadcast", payload)
        )
        backend._update_low_battery_return_policy = lambda *_args, **_kwargs: None
        backend._resolve_mission_key_for_site = (
            lambda site: f"camping_site_{site[1:]}"
        )
        backend._is_site_occupied = lambda _site: False
        backend._site_arrival_match = lambda site: (
            False,
            f"camping_site_{site[1:]}",
            10.0,
            "outside_arrival_radius",
        )
        backend._mission_dispatch_battery_block = lambda _site: None
        backend._update_runtime_state = lambda callback: callback()
        backend._publish_site_mission_key = lambda key, source: events.append(
            ("mission_key", key, source)
        )

        def publish_goal(site, key, source):
            events.append(("goal", site, key, source))
            return True

        backend._publish_site_goal_pose = publish_goal
        backend._publish_parking_operation = lambda operation, source: events.append(
            ("parking_operation", operation, source)
        )
        backend._publish_drop_zone_operation = lambda operation, source: events.append(
            ("drop_zone_operation", operation, source)
        )
        backend._publish_service_state = lambda state, source: events.append(
            ("state", state, source)
        )
        backend._on_service_state = lambda msg: UiBackendNode._on_service_state(
            backend, msg
        )

        heartbeat = AvgServiceState()
        heartbeat.state = AvgServiceState.DEPARTING_DROP_ZONE
        heartbeat.state_name = "DEPARTING_DROP_ZONE"
        heartbeat.description = "drop_zone_maneuver_controller:EXIT_STRAIGHT:heartbeat"
        UiBackendNode._on_service_state(backend, heartbeat)
        events.clear()

        first = UiBackendNode._apply_destination_command(
            backend, site="B2", run=True, source="restart"
        )
        self.assertFalse(first["goal_pose_published"])
        self.assertTrue(backend._drop_zone_exit_active)
        self.assertEqual(
            backend._pending_site_after_drop_zone_exit,
            ("B2", "camping_site_2", "restart"),
        )
        # Attach to the controller already in EXIT; never CANCEL/restart it.
        self.assertNotIn("drop_zone_operation", [event[0] for event in events])
        self.assertNotIn("mission_key", [event[0] for event in events])
        self.assertNotIn("goal", [event[0] for event in events])

        changed = UiBackendNode._apply_destination_command(
            backend, site="B3", run=True, source="changed"
        )
        self.assertFalse(changed["goal_pose_published"])
        self.assertTrue(changed["blocked"])
        self.assertEqual(changed["error"], "mission_already_active")
        self.assertEqual(
            backend._pending_site_after_drop_zone_exit,
            ("B2", "camping_site_2", "restart"),
        )
        self.assertNotIn("mission_key", [event[0] for event in events])
        self.assertNotIn("goal", [event[0] for event in events])

        completed = AvgBool()
        completed.data = True
        UiBackendNode._on_drop_zone_exit_complete(backend, completed)
        release_events = [
            event for event in events if event[0] in {"mission_key", "goal"}
        ]
        self.assertEqual(release_events[0][0:2], ("mission_key", "camping_site_2"))
        self.assertEqual(release_events[1][0:3], ("goal", "B2", "camping_site_2"))

    def test_orphan_exit_complete_clears_recovered_departing_state(self) -> None:
        events = []
        backend = SimpleNamespace(
            _drop_zone_exit_active=False,
            _pending_site_after_drop_zone_exit=None,
            _drop_zone_exit_handoff_ready=False,
            _latest_platform_is_charging=False,
            _latest_service_state=None,
            _active_mission_site="",
            _service_metrics=None,
            _lock=threading.Lock(),
            _state=SimpleNamespace(
                destination={"site": "", "run": False},
                service_state=-1,
                service_state_name="",
                service_state_description="",
                battery_percentage=80,
            ),
            _runtime_policy=SimpleNamespace(update_goal_received=lambda _mode: None),
            publish_engage_from_destination=False,
            publish_mission_engage_from_destination=False,
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._schedule_broadcast = lambda payload: events.append(
            ("broadcast", payload)
        )
        backend._update_low_battery_return_policy = lambda *_args, **_kwargs: None
        backend._publish_service_state = lambda state, source: events.append(
            ("state", state, source)
        )
        backend._on_service_state = lambda msg: UiBackendNode._on_service_state(
            backend, msg
        )

        heartbeat = AvgServiceState()
        heartbeat.state = AvgServiceState.DEPARTING_DROP_ZONE
        heartbeat.state_name = "DEPARTING_DROP_ZONE"
        heartbeat.description = "drop_zone_maneuver_controller:EXIT_STRAIGHT:heartbeat"
        UiBackendNode._on_service_state(backend, heartbeat)

        completed = AvgBool()
        completed.data = True
        UiBackendNode._on_drop_zone_exit_complete(backend, completed)
        self.assertTrue(backend._drop_zone_exit_handoff_ready)
        self.assertIsNone(backend._latest_service_state)
        self.assertEqual(backend._state.service_state_name, "ROAD_HANDOFF_READY")

        # A delayed heartbeat from the service-state writer may arrive after
        # exit_complete, but cannot roll the handoff back.
        UiBackendNode._on_service_state(backend, heartbeat)
        self.assertTrue(backend._drop_zone_exit_handoff_ready)
        terminal = ModuleState()
        terminal.operating_state = "ROAD_HANDOFF_READY"
        UiBackendNode._on_drop_zone_maneuver_status(backend, terminal)
        self.assertTrue(backend._drop_zone_exit_handoff_ready)

        backend._resolve_mission_key_for_site = lambda _site: "camping_site_2"
        backend._is_site_occupied = lambda _site: False
        backend._site_arrival_match = lambda _site: (
            False,
            "camping_site_2",
            10.0,
            "outside_arrival_radius",
        )
        backend._mission_dispatch_battery_block = lambda _site: None
        backend._update_runtime_state = lambda callback: callback()
        backend._publish_site_mission_key = lambda key, source: events.append(
            ("mission_key", key, source)
        )

        def publish_goal(site, key, source):
            events.append(("goal", site, key, source))
            return True

        backend._publish_site_goal_pose = publish_goal
        backend._publish_goal_for_site = lambda site, source: {
            "mission_key": "camping_site_2",
            "goal_pose_published": (
                backend._publish_site_mission_key("camping_site_2", source)
                is None
                and publish_goal(site, "camping_site_2", source)
            ),
            "message": "ok",
        }
        backend._publish_parking_operation = lambda operation, source: events.append(
            ("parking_operation", operation, source)
        )
        backend._publish_drop_zone_operation = lambda operation, source: events.append(
            ("drop_zone_operation", operation, source)
        )

        result = UiBackendNode._apply_destination_command(
            backend, site="B2", run=True, source="after_orphan_complete"
        )
        self.assertTrue(result["goal_pose_published"])
        self.assertNotIn("drop_zone_operation", [event[0] for event in events])
        release = [event[0] for event in events if event[0] in {"mission_key", "goal"}]
        self.assertEqual(release, ["mission_key", "goal"])

    def test_terminal_handoff_state_before_exit_complete_is_not_rolled_back(self) -> None:
        backend = SimpleNamespace(
            _drop_zone_exit_active=False,
            _pending_site_after_drop_zone_exit=None,
            _drop_zone_exit_handoff_ready=False,
            _latest_platform_is_charging=False,
            _latest_service_state=None,
            _service_metrics=None,
            _lock=threading.Lock(),
            _state=SimpleNamespace(
                service_state=-1,
                service_state_name="",
                service_state_description="",
                battery_percentage=80,
            ),
            publish_mission_engage_from_destination=False,
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._schedule_broadcast = lambda _payload: None
        backend._update_low_battery_return_policy = lambda *_args, **_kwargs: None

        terminal = AvgServiceState()
        terminal.state = AvgServiceState.DROP_ZONE_WAIT
        terminal.state_name = "ROAD_HANDOFF_READY"
        terminal.description = "Drop-zone exit complete; ready for route dispatch"
        UiBackendNode._on_service_state(backend, terminal)
        self.assertTrue(backend._drop_zone_exit_handoff_ready)

        completion = AvgBool()
        completion.data = True
        UiBackendNode._on_drop_zone_exit_complete(backend, completion)
        self.assertTrue(backend._drop_zone_exit_handoff_ready)
        self.assertEqual(backend._latest_service_state, AvgServiceState.DROP_ZONE_WAIT)

    def test_successful_station_exit_releases_key_then_goal_once(self) -> None:
        events = []
        logger = _FakeLogger()
        backend = SimpleNamespace(
            _drop_zone_exit_active=True,
            _pending_site_after_drop_zone_exit=(
                "B2",
                "camping_site_2",
                "test",
            ),
            _latest_platform_is_charging=False,
            _latest_service_state=int(AvgServiceState.DEPARTING_DROP_ZONE),
            _drop_zone_exit_handoff_ready=False,
            _drop_zone_exit_failure_latched=False,
            _drop_zone_exit_waiting_for_fresh_status=True,
            _drop_zone_exit_cancel_suppressed=False,
            publish_mission_engage_from_destination=False,
        )
        backend.get_logger = lambda: logger
        backend._publish_mission_engage = lambda enabled, source: events.append(
            ("mission_engage", enabled, source)
        )
        backend._schedule_broadcast = lambda payload: events.append(
            ("broadcast", payload)
        )
        backend._publish_site_mission_key = lambda key, source: events.append(
            ("mission_key", key, source)
        )

        def publish_goal(site, key, source):
            events.append(("goal", site, key, source))
            return True

        backend._publish_site_goal_pose = publish_goal
        def publish_state(state, source):
            backend._latest_service_state = int(state)
            events.append(("state", state, source))

        backend._publish_service_state = publish_state
        backend._on_service_state = lambda msg: UiBackendNode._on_service_state(
            backend, msg
        )

        # A fresh status acknowledges the current retry.  A delayed false from
        # the prior/already-active request remains observation-only.
        accepted = ModuleState()
        accepted.operating_state = "EXIT_STRAIGHT"
        UiBackendNode._on_drop_zone_maneuver_status(backend, accepted)
        old_false = AvgBool()
        old_false.data = False
        UiBackendNode._on_drop_zone_exit_complete(backend, old_false)
        self.assertTrue(backend._drop_zone_exit_active)
        self.assertIsNotNone(backend._pending_site_after_drop_zone_exit)
        self.assertEqual(events, [])

        message = AvgBool()
        message.data = True
        UiBackendNode._on_drop_zone_exit_complete(backend, message)

        self.assertFalse(backend._drop_zone_exit_active)
        self.assertIsNone(backend._pending_site_after_drop_zone_exit)
        self.assertEqual(
            [event[0] for event in events],
            ["mission_key", "goal", "state"],
        )
        self.assertEqual(events[0][1], "camping_site_2")
        self.assertEqual(events[1][1:3], ("B2", "camping_site_2"))
        self.assertEqual(events[2][1], AvgServiceState.MOVING_TO_SITE)

        # If the status terminal is delivered after exit_complete, it is an
        # acknowledgement and cannot roll MOVING_TO_SITE back to a wait state.
        delayed_exit_status = ModuleState()
        delayed_exit_status.operating_state = "EXIT_STRAIGHT"
        UiBackendNode._on_drop_zone_maneuver_status(backend, delayed_exit_status)
        self.assertTrue(backend._drop_zone_exit_handoff_ready)
        self.assertEqual(
            backend._latest_service_state,
            int(AvgServiceState.MOVING_TO_SITE),
        )
        terminal_status = ModuleState()
        terminal_status.operating_state = "ROAD_HANDOFF_READY"
        UiBackendNode._on_drop_zone_maneuver_status(backend, terminal_status)
        self.assertEqual(
            backend._latest_service_state,
            int(AvgServiceState.MOVING_TO_SITE),
        )
        self.assertEqual([event[0] for event in events], ["mission_key", "goal", "state"])

        # A reliable-topic duplicate cannot release the pending goal twice.
        UiBackendNode._on_drop_zone_exit_complete(backend, message)
        self.assertEqual(
            [event[0] for event in events],
            ["mission_key", "goal", "state"],
        )

    def test_failed_station_exit_never_releases_campsite_key_or_goal(self) -> None:
        events = []
        logger = _FakeLogger()
        backend = SimpleNamespace(
            _drop_zone_exit_active=True,
            _pending_site_after_drop_zone_exit=(
                "B2",
                "camping_site_2",
                "test",
            ),
            _latest_platform_is_charging=False,
            _latest_service_state=int(AvgServiceState.DEPARTING_DROP_ZONE),
            _drop_zone_exit_handoff_ready=False,
            _drop_zone_exit_failure_latched=False,
            _drop_zone_exit_waiting_for_fresh_status=False,
            _drop_zone_exit_cancel_suppressed=False,
            publish_mission_engage_from_destination=False,
        )
        backend.get_logger = lambda: logger
        backend._publish_mission_engage = lambda enabled, source: events.append(
            ("mission_engage", enabled, source)
        )
        backend._schedule_broadcast = lambda payload: events.append(
            ("broadcast", payload)
        )
        backend._publish_site_mission_key = lambda key, source: events.append(
            ("mission_key", key, source)
        )
        backend._publish_site_goal_pose = lambda site, key, source: events.append(
            ("goal", site, key, source)
        )
        def publish_state(state, source):
            backend._latest_service_state = int(state)
            events.append(("state", state, source))

        backend._publish_service_state = publish_state
        backend._mark_drop_zone_exit_failed = (
            lambda source: UiBackendNode._mark_drop_zone_exit_failed(
                backend, source
            )
        )

        message = AvgBool()
        message.data = False
        UiBackendNode._on_drop_zone_exit_complete(backend, message)

        # Bool(false) is ambiguous (invalid target or already-active request),
        # so it cannot terminate the current attempt by itself.
        self.assertTrue(backend._drop_zone_exit_active)
        self.assertEqual(
            backend._pending_site_after_drop_zone_exit,
            ("B2", "camping_site_2", "test"),
        )
        self.assertFalse(backend._drop_zone_exit_failure_latched)
        self.assertEqual(
            backend._latest_service_state,
            int(AvgServiceState.DEPARTING_DROP_ZONE),
        )
        self.assertNotIn("mission_key", [event[0] for event in events])
        self.assertNotIn("goal", [event[0] for event in events])
        self.assertEqual(events, [])
        self.assertTrue(logger.warning_messages)

        # The same-writer persistent ERROR is the authoritative real failure.
        error_status = ModuleState()
        error_status.operating_state = "ERROR"
        UiBackendNode._on_drop_zone_maneuver_status(backend, error_status)
        self.assertFalse(backend._drop_zone_exit_active)
        self.assertTrue(backend._drop_zone_exit_failure_latched)
        self.assertEqual(
            backend._latest_service_state,
            int(AvgServiceState.DROP_ZONE_WAIT),
        )
        self.assertEqual(events[-1][0], "state")

        event_count = len(events)
        delayed_exit = ModuleState()
        delayed_exit.operating_state = "EXIT_STRAIGHT"
        UiBackendNode._on_drop_zone_maneuver_status(backend, delayed_exit)
        self.assertEqual(len(events), event_count)
        self.assertEqual(
            backend._latest_service_state,
            int(AvgServiceState.DROP_ZONE_WAIT),
        )
        self.assertTrue(backend._drop_zone_exit_failure_latched)

    def test_orphan_failure_before_exit_status_blocks_recovery_until_retry(self) -> None:
        events = []
        backend = SimpleNamespace(
            _drop_zone_exit_active=False,
            _pending_site_after_drop_zone_exit=None,
            _drop_zone_exit_handoff_ready=False,
            _drop_zone_exit_failure_latched=False,
            _drop_zone_exit_waiting_for_fresh_status=False,
            _drop_zone_exit_cancel_suppressed=False,
            _latest_platform_is_charging=False,
            _latest_service_state=None,
            publish_mission_engage_from_destination=False,
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._schedule_broadcast = lambda payload: events.append(
            ("broadcast", payload)
        )

        def publish_state(state, source):
            backend._latest_service_state = int(state)
            events.append(("state", state, source))

        backend._publish_service_state = publish_state
        backend._mark_drop_zone_exit_failed = (
            lambda source: UiBackendNode._mark_drop_zone_exit_failed(
                backend, source
            )
        )

        failed = AvgBool()
        failed.data = False
        UiBackendNode._on_drop_zone_exit_complete(backend, failed)
        self.assertFalse(backend._drop_zone_exit_failure_latched)
        self.assertIsNone(backend._latest_service_state)

        error_status = ModuleState()
        error_status.operating_state = "ERROR"
        UiBackendNode._on_drop_zone_maneuver_status(backend, error_status)
        self.assertTrue(backend._drop_zone_exit_failure_latched)
        self.assertEqual(
            backend._latest_service_state,
            int(AvgServiceState.DROP_ZONE_WAIT),
        )

        delayed_exit = ModuleState()
        delayed_exit.operating_state = "EXIT_STRAIGHT"
        UiBackendNode._on_drop_zone_maneuver_status(backend, delayed_exit)
        self.assertEqual(
            backend._latest_service_state,
            int(AvgServiceState.DROP_ZONE_WAIT),
        )
        self.assertFalse(backend._drop_zone_exit_active)

    def test_prestart_idle_cannot_fail_new_exit_and_fresh_exit_wins_old_false(self) -> None:
        events = []
        backend = SimpleNamespace(
            _drop_zone_exit_active=True,
            _pending_site_after_drop_zone_exit=(
                "B2",
                "camping_site_2",
                "retry",
            ),
            _drop_zone_exit_handoff_ready=False,
            _drop_zone_exit_failure_latched=False,
            _drop_zone_exit_waiting_for_fresh_status=True,
            _drop_zone_exit_cancel_suppressed=False,
            _latest_platform_is_charging=False,
            _latest_service_state=int(AvgServiceState.DEPARTING_DROP_ZONE),
            _service_metrics=None,
            _lock=threading.Lock(),
            _state=SimpleNamespace(
                service_state=int(AvgServiceState.DEPARTING_DROP_ZONE),
                service_state_name="DEPARTING_DROP_ZONE",
                service_state_description="Departing drop zone",
                battery_percentage=80,
            ),
            publish_mission_engage_from_destination=False,
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._schedule_broadcast = lambda payload: events.append(
            ("broadcast", payload)
        )
        backend._update_low_battery_return_policy = lambda *_args, **_kwargs: None

        def publish_state(state, source):
            backend._latest_service_state = int(state)
            events.append(("state", state, source))

        backend._publish_service_state = publish_state
        backend._mark_drop_zone_exit_failed = (
            lambda source: UiBackendNode._mark_drop_zone_exit_failed(
                backend, source
            )
        )
        backend._on_service_state = lambda msg: UiBackendNode._on_service_state(
            backend, msg
        )

        stale_idle = ModuleState()
        stale_idle.operating_state = "IDLE"
        UiBackendNode._on_drop_zone_maneuver_status(backend, stale_idle)
        self.assertTrue(backend._drop_zone_exit_active)
        self.assertFalse(backend._drop_zone_exit_failure_latched)

        accepted_exit = ModuleState()
        accepted_exit.operating_state = "EXIT_STRAIGHT"
        UiBackendNode._on_drop_zone_maneuver_status(backend, accepted_exit)
        self.assertTrue(backend._drop_zone_exit_active)
        self.assertFalse(backend._drop_zone_exit_failure_latched)
        self.assertFalse(backend._drop_zone_exit_waiting_for_fresh_status)

        # A false from startExit(already-active) cannot fail the ongoing exit.
        delayed_failure = AvgBool()
        delayed_failure.data = False
        UiBackendNode._on_drop_zone_exit_complete(backend, delayed_failure)
        self.assertFalse(backend._drop_zone_exit_failure_latched)
        self.assertTrue(backend._drop_zone_exit_active)

        UiBackendNode._on_drop_zone_maneuver_status(backend, accepted_exit)
        self.assertFalse(backend._drop_zone_exit_failure_latched)
        self.assertTrue(backend._drop_zone_exit_active)
        self.assertFalse(backend._drop_zone_exit_waiting_for_fresh_status)
        self.assertEqual(
            backend._latest_service_state,
            int(AvgServiceState.DEPARTING_DROP_ZONE),
        )

    def test_operator_stop_suppresses_queued_departure_statuses(self) -> None:
        events = []
        backend = SimpleNamespace(
            _drop_zone_exit_active=True,
            _pending_site_after_drop_zone_exit=(
                "B2",
                "camping_site_2",
                "test",
            ),
            _drop_zone_exit_handoff_ready=False,
            _drop_zone_exit_failure_latched=False,
            _drop_zone_exit_waiting_for_fresh_status=False,
            _drop_zone_exit_cancel_suppressed=False,
            _latest_platform_is_charging=False,
            _latest_service_state=int(AvgServiceState.DEPARTING_DROP_ZONE),
            _active_mission_site="B2",
            _service_metrics=None,
            _lock=threading.Lock(),
            _state=SimpleNamespace(
                ws_site_states={"B2": True},
                destination={"site": "B2", "run": True},
                service_state=int(AvgServiceState.DEPARTING_DROP_ZONE),
                service_state_name="DEPARTING_DROP_ZONE",
                service_state_description="Departing drop zone",
                battery_percentage=80,
            ),
            site_names=["B2"],
            publish_mission_engage_from_destination=False,
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._cancel_pending_manual_return_transition = (
            lambda source: events.append(("cancel_pending", source))
        )
        backend._cancel_active_motion = (
            lambda source: events.append(("cancel_motion", source))
        )
        backend._publish_engage = (
            lambda enabled, source: events.append(("engage", enabled, source))
        )
        backend._schedule_broadcast = (
            lambda payload: events.append(("broadcast", payload))
        )

        def publish_state(state, source):
            backend._latest_service_state = int(state)
            events.append(("state", state, source))

        backend._publish_service_state = publish_state
        backend._on_service_state = lambda msg: UiBackendNode._on_service_state(
            backend, msg
        )
        backend._mark_drop_zone_exit_failed = (
            lambda source: UiBackendNode._mark_drop_zone_exit_failed(
                backend, source
            )
        )

        UiBackendNode._stop_active_service(backend, "test_stop")
        self.assertTrue(backend._drop_zone_exit_cancel_suppressed)
        self.assertFalse(backend._drop_zone_exit_active)
        self.assertIsNone(backend._pending_site_after_drop_zone_exit)
        self.assertEqual(
            backend._latest_service_state,
            int(AvgServiceState.OPERATOR_STOPPED),
        )

        queued_exit = ModuleState()
        queued_exit.operating_state = "EXIT_STRAIGHT"
        UiBackendNode._on_drop_zone_maneuver_status(backend, queued_exit)
        queued_departing = AvgServiceState()
        queued_departing.state = AvgServiceState.DEPARTING_DROP_ZONE
        queued_departing.state_name = "DEPARTING_DROP_ZONE"
        queued_departing.description = "queued departure heartbeat"
        UiBackendNode._on_service_state(backend, queued_departing)
        queued_road = ModuleState()
        queued_road.operating_state = "ROAD_HANDOFF_READY"
        UiBackendNode._on_drop_zone_maneuver_status(backend, queued_road)
        idle = ModuleState()
        idle.operating_state = "IDLE"
        UiBackendNode._on_drop_zone_maneuver_status(backend, idle)

        self.assertFalse(backend._drop_zone_exit_active)
        self.assertFalse(backend._drop_zone_exit_handoff_ready)
        self.assertEqual(
            backend._latest_service_state,
            int(AvgServiceState.OPERATOR_STOPPED),
        )

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
        metrics = _FakeServiceMetrics()
        backend = SimpleNamespace(
            _active_mission_site="B4",
            _lock=threading.Lock(),
            broadcasts=[],
            _service_metrics=metrics,
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
        self.assertEqual(metrics.starts, [])

    def test_accepted_already_arrived_destination_starts_service_evidence(self) -> None:
        metrics = _FakeServiceMetrics()
        backend = SimpleNamespace(
            _active_mission_site="",
            _lock=threading.Lock(),
            _service_metrics=metrics,
            publish_mission_engage_from_destination=False,
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._resolve_mission_key_for_site = lambda _site: "camping_site_6"
        backend._is_site_occupied = lambda _site: False
        backend._site_arrival_match = lambda _site: (
            True,
            "camping_site_6",
            0.4,
            "inside_site_polygon",
        )
        backend._publish_camping_site_maneuver_controller_adopt = (
            lambda *_args, **_kwargs: None
        )
        backend._publish_service_state = lambda *_args, **_kwargs: None
        backend._notify_site_arrival = lambda *_args, **_kwargs: None
        backend._publish_engage = lambda *_args, **_kwargs: None

        result = UiBackendNode._apply_destination_command(
            backend,
            site="B6",
            run=True,
            source="test_accept",
        )

        self.assertTrue(result["run"])
        self.assertEqual(backend._active_mission_site, "B6")
        self.assertEqual(metrics.starts[0][0], "B6")
        self.assertEqual(metrics.starts[0][1]["mission_key"], "camping_site_6")
        self.assertEqual(metrics.starts[0][1]["source"], "test_accept")

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

    def test_enabled_occupancy_guard_preserves_active_guest_recall(self) -> None:
        backend = self._occupancy_backend(
            AvgServiceState.RECALL_TO_SITE_ROAD, enabled=True
        )
        backend._active_mission_site = "B3"
        backend._active_mission_source = "guest:kiosk"

        UiBackendNode._on_campsite_occupancy(
            backend, self._occupancy_message("camping_site_3")
        )

        # A guest is calling the robot to an existing tent, so perception of
        # that tent must neither clear the target nor synthesize run=false.
        self.assertEqual(backend._state.occupied_sites, ["B3"])
        self.assertTrue(backend._state.ws_site_states["B3"])
        self.assertTrue(backend._state.destination["run"])
        self.assertEqual(backend.stops, [])
        self.assertEqual(backend.broadcasts, [{"occupied_sites": ["B3"]}])

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
