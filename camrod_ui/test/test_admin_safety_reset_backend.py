"""HH_261024 - Exercise the actual reset orchestrator without robot commands."""

import asyncio
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import AsyncMock, Mock, patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime/python"))
from camrod_ui.ui_backend_node import UiBackendNode


class Future:
    def __init__(self, response=None, done=True):
        self.response, self.complete = response, done

    def done(self):
        return self.complete

    def result(self):
        return self.response


def backend():
    events = []
    client = NS(service_is_ready=lambda: True)
    node = NS(
        _destination_dispatch_lock=threading.RLock(),
        _admin_safety_reset_pending=False,
        _admin_safety_reset_cancel_futures=[],
        _admin_safety_reset_result=None,
        _safety_reset_client=client,
        _now_s=lambda: 100.0,
        _runtime_policy=NS(nav_status=4),
        nav2_cancel_clients=[object(), object()],
        _schedule_broadcast=lambda payload: events.append(("broadcast", payload)),
        _request_nav2_cancel=Mock(),
        get_logger=lambda: Mock(),
        _safety_reset_motion_observation={
            "received_monotonic_s": time.monotonic(), "source_stamp_s": 99.9,
            "speed_mps": 0.0, "yaw_rate_radps": 0.0, "estop": False,
            "error_code": 0, "vehicle_state": 0,
        },
    )

    def stop(source):
        events.append(("stop", source))
        assert node._admin_safety_reset_pending
        node._admin_safety_reset_cancel_futures = [
            Future(NS(return_code=0, goals_canceling=[])) for _ in range(2)]
        node._admin_safety_reset_gate_observation = (
            {"level": 0, "operating_state": "STANDBY",
             "message": "state=STANDBY reasons=manual_disabled",
             "source_stamp_s": 99.9}, time.monotonic())

    def reset(request):
        events.append(("reset", request))
        assert node._admin_safety_reset_pending
        assert events[0][0] == "stop"
        return Future(NS(success=True, message="recovery_hold_reset_motion_disarmed"))

    node._stop_active_service = stop
    client.call_async = reset
    return node, events


def run(node, **kwargs):
    return asyncio.run(UiBackendNode.request_admin_safety_reset(node, **kwargs))


def test_success_is_stop_cancel_then_reset_not_an_automatic_restart():
    node, events = backend()
    result = run(node, confirm=True)
    assert result["success"] and result["requires_explicit_start"]
    assert [kind for kind, _ in events] == ["stop", "reset", "broadcast"]
    assert not node._admin_safety_reset_pending
    assert node._admin_safety_reset_result == result
    assert "engage" not in events[-1][1]


@pytest.mark.parametrize("confirm", [None, False, 0, 1, "true", "false", [], {}])
def test_missing_literal_confirmation_never_stops_or_resets(confirm):
    node, events = backend()
    assert not run(node, confirm=confirm)["success"]
    assert events == []


@pytest.mark.parametrize("blocker", ["busy", "unavailable", "moving", "estop", "stale", "startup"])
def test_preconditions_refuse_without_sending_reset(blocker):
    node, events = backend()
    if blocker == "busy":
        node._admin_safety_reset_pending = True
    elif blocker == "unavailable":
        node._safety_reset_client.service_is_ready = lambda: False
    elif blocker == "moving":
        node._safety_reset_motion_observation["speed_mps"] = .04
    elif blocker == "estop":
        node._safety_reset_motion_observation["estop"] = True
    elif blocker == "stale":
        node._safety_reset_motion_observation["source_stamp_s"] = 98.0
    else:
        node._startup_recovery_pending = True
    assert not run(node, confirm=True)["success"]
    assert events == []


def test_live_hazard_refusal_is_not_reported_as_reset_success():
    node, events = backend()
    node._safety_reset_client.call_async = lambda _: Future(NS(
        success=False, message="active_or_stale_obstacle_hold:radar"))
    result = run(node, confirm=True)
    assert not result["success"]
    assert result["controller_message"] == "active_or_stale_obstacle_hold:radar"
    assert result["requires_explicit_start"]
    assert not node._admin_safety_reset_pending


def test_canceling_ack_is_requeried_until_both_action_servers_confirm_idle():
    node, events = backend()
    original_stop = node._stop_active_service
    node._runtime_policy.nav_status = 0

    def stop(source):
        original_stop(source)
        for future in node._admin_safety_reset_cancel_futures:
            future.response.goals_canceling = [object()]

    def query(source):
        events.append(("cancel_requery", source))
        node._admin_safety_reset_cancel_futures = [
            Future(NS(return_code=3, goals_canceling=[])) for _ in range(2)]

    node._stop_active_service = stop
    node._request_nav2_cancel = query
    result = run(node, confirm=True)
    assert result["success"]
    assert [kind for kind, _ in events] == ["stop", "cancel_requery", "reset", "broadcast"]


def test_service_timeout_keeps_stopped_and_reports_unknown_result():
    node, events = backend()
    with patch.object(UiBackendNode, "_await_ros_future", new=AsyncMock(side_effect=TimeoutError)):
        result = run(node, confirm=True)
    assert not result["success"] and result["error"] == "reset_not_confirmed"
    assert [kind for kind, _ in events] == ["stop", "reset", "broadcast"]
    assert not node._admin_safety_reset_pending


def test_concurrent_request_does_not_stop_or_reset_twice():
    async def scenario():
        node, events = backend()
        future = Future(NS(success=True, message="recovery_hold_reset_motion_disarmed"), done=False)
        node._safety_reset_client.call_async = lambda _: future
        first = asyncio.create_task(UiBackendNode.request_admin_safety_reset(node, confirm=True))
        await asyncio.sleep(.01)
        second = await UiBackendNode.request_admin_safety_reset(node, confirm=True)
        assert second["busy"] and not second["success"]
        future.complete = True
        assert (await first)["success"]
        assert [kind for kind, _ in events].count("stop") == 1
    asyncio.run(scenario())


def test_stop_publication_failure_is_visible_and_not_followed_by_reset():
    node, events = backend()
    node._stop_active_service = Mock(side_effect=RuntimeError("unavailable publisher"))
    result = run(node, confirm=True)
    assert not result["success"] and result["error"] == "stop_not_confirmed"
    assert events == []
    assert not node._admin_safety_reset_pending


@pytest.mark.parametrize("unsettled", ["missing_cancel", "executing", "missing_gate", "stale_after_stop"])
def test_unconfirmed_stop_never_calls_the_reset_service(unsettled):
    node, events = backend()
    original_stop = node._stop_active_service

    def stop(source):
        original_stop(source)
        if unsettled == "missing_cancel":
            node._admin_safety_reset_cancel_futures.pop()
        elif unsettled == "executing":
            node._runtime_policy.nav_status = 2
        elif unsettled == "missing_gate":
            node._admin_safety_reset_gate_observation = None
        else:
            node._safety_reset_motion_observation["source_stamp_s"] = 90.0
        clock["advance"] = True

    node._stop_active_service = stop
    # HH_261024 - Advance only the local deadline, not safety timestamps or
    # robot state, so timeout regressions do not each sleep for five seconds.
    native_clock = time.monotonic
    clock = {"advance": False, "extra": 0.0}

    def now():
        if clock["advance"]:
            clock["extra"] += .6
        return native_clock() + clock["extra"]

    with patch("camrod_ui.ui_backend_node.time.monotonic", side_effect=now):
        result = run(node, confirm=True)
    assert not result["success"] and result["error"] == "stop_confirmation_timeout"
    assert "reset" not in [kind for kind, _ in events]
    assert not node._admin_safety_reset_pending


@pytest.mark.parametrize("method", ["_publish_engage", "_publish_mission_engage", "_publish_platform_drive_enable"])
def test_concurrent_policy_callbacks_cannot_rearm_during_reset(method):
    node = NS(_admin_safety_reset_pending=True)
    # No publisher exists: returning without touching one proves the barrier
    # precedes any attempted true publication, even from automatic callbacks.
    getattr(UiBackendNode, method)(node, True, "late_policy_echo")


def test_existing_admission_barrier_blocks_new_commands_during_reset():
    node = NS(_admin_safety_reset_pending=True)
    assert UiBackendNode._startup_recovery_block(node)["error"] == "admin_safety_reset_pending"


@pytest.mark.parametrize("source_stamp,receipt_age,fresh", [
    (99.9, 0.1, True), (0.0, 0.1, False), (98.0, 0.1, False),
    (99.9, 2.0, False), (100.2, 0.1, False), (None, 0.1, False),
    (float("nan"), 0.1, False),
])
def test_native_gate_heartbeat_requires_fresh_source_and_receipt(source_stamp, receipt_age, fresh):
    # HH_261026 - Native reset eligibility uses the existing gate heartbeat,
    # not an administrator browser lease or an optional simulator component.
    node = NS(_now_s=lambda: 100.0, _admin_safety_reset_gate_observation=(
        {"source_stamp_s": source_stamp, "level": 0, "operating_state": "STANDBY",
         "message": "state=STANDBY reasons=manual_disabled,none"}, 500.0 - receipt_age))
    with patch("camrod_ui.ui_backend_node.time.monotonic", return_value=500.0):
        result = UiBackendNode._admin_safety_reset_gate_snapshot(node)
    assert result["fresh"] is fresh
    assert result["blocked"] is (True if fresh else None)
    if fresh:
        assert result["reasons"] == ["manual_disabled"]


def test_native_gate_heartbeat_missing_observation_stays_unknown():
    node = NS(_now_s=lambda: 100.0)
    result = UiBackendNode._admin_safety_reset_gate_snapshot(node)
    assert not result["fresh"] and result["blocked"] is None
    assert result["operating_state"] == "UNKNOWN"


def test_native_gate_callback_reuses_existing_subscription_without_telemetry():
    # HH_261026 - A canonical gate message is retained even when the operator
    # telemetry session is inactive; no second subscriber or motion is needed.
    from avg_msgs.msg import ModuleState
    message = ModuleState()
    message.stamp.sec = 100
    message.level = 1
    message.operating_state = "ROUTE_SAFETY_HOLD"
    message.message = "reasons=route_recovery"
    node = NS(_update_runtime_state=lambda callback: None, _telemetry_capture_active=False)
    with patch("camrod_ui.ui_backend_node.time.monotonic", return_value=500.0):
        UiBackendNode._on_control_gate_status(node, message)
    values, received = node._admin_safety_reset_gate_observation
    assert received == 500.0 and values["source_stamp_s"] == 100.0
    assert values["operating_state"] == "ROUTE_SAFETY_HOLD"


def test_native_cancel_all_retains_actual_futures_only_for_explicit_reset():
    # HH_261026 - Keep ordinary stop behavior unchanged while the privileged
    # transaction retains each actual cancel reply for its completion barrier.
    first, second = Future(), Future()
    node = NS(nav2_cancel_action_topics=["/follow/_action/cancel_goal", "/navigate/_action/cancel_goal"],
              nav2_cancel_clients=[
                  NS(service_is_ready=lambda: True, call_async=lambda request: first),
                  NS(service_is_ready=lambda: True, call_async=lambda request: second)],
              _admin_safety_reset_cancel_futures=[], get_logger=lambda: Mock())
    UiBackendNode._request_nav2_cancel(node, "operator_stop")
    assert node._admin_safety_reset_cancel_futures == []
    topics = UiBackendNode._request_nav2_cancel(node, "admin_safety_reset")
    assert topics == node.nav2_cancel_action_topics
    assert node._admin_safety_reset_cancel_futures == [first, second]
