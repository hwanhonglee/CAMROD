"""Offline deterministic scheduling regression; no ROS, CDP or motion servers."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
from websocket import WebSocketTimeoutException


SCRIPT = Path(__file__).resolve().parents[2] / "scripts/virtual_carla/camping_site_matrix.py"
SPEC = importlib.util.spec_from_file_location("browser_observation_matrix", SCRIPT)
matrix = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = matrix
SPEC.loader.exec_module(matrix)


class Clock:
    value = 0.0

    def monotonic(self):
        return self.value


def observer_fixture(clock, *, publishing=True, unhealthy_at=None):
    observer = matrix.RosObservation.__new__(matrix.RosObservation)
    ready = dict(ready=True, physical_gate_accepted=True, physx_substep_control_verified=True,
        independent_wheel_drive_available=True, motion_backend="PHYSX_FOUR_WHEEL_STEERING",
        actor_id=82, physical_manifest_sha256="1" * 64, production_authorization_sha256="2" * 64,
        ros_integration_sha256="3" * 64, imported_libcarla_sha256="4" * 64, wheel_torque_safety_cap_nm=20.0)
    observer.latest = {"physical": ready}
    observer.expected_actor_id = 82
    observer._physical_received_monotonic = 0.0
    observer._physical_identity = None
    observer._physical_samples = 0
    observer._fatal_error = ""
    observer.node = object()
    calls = []

    def spin_once(node, *, timeout_sec):
        assert node is observer.node
        calls.append(timeout_sec)
        # Other high-rate subscriptions precede the physical heartbeat. A
        # single callback after the blocked browser is not a complete drain.
        if publishing and len(calls) % 4 == 0:
            observer._physical({**ready, "ready": unhealthy_at is None or clock.value < unhealthy_at})

    observer.rclpy = SimpleNamespace(spin_once=spin_once)
    return observer, calls


class DelayedSocket:
    def __init__(self, clock, duration=2.179):
        self.clock, self.duration = clock, duration
        self.timeout = 10.0
        self.timeouts, self.sent = [], []
        self.closed = False

    def settimeout(self, timeout):
        self.timeout = timeout
        self.timeouts.append(timeout)

    def send(self, value):
        self.sent.append(json.loads(value))

    def recv(self):
        remaining = self.duration - self.clock.value
        self.clock.value += min(self.timeout, max(remaining, 0.0))
        if self.clock.value + 1e-9 < self.duration:
            raise WebSocketTimeoutException("normal bounded receive slice")
        return json.dumps(dict(id=self.sent[-1]["id"], result={"test_response": True}))

    def close(self):
        self.closed = True


def client_fixture(clock, observer=None, cls=matrix.GuestBrowserClient, duration=2.179):
    client = cls.__new__(cls)
    client.timeout_s = 10.0
    client._command_id = 0
    client._connection = socket = DelayedSocket(clock, duration)
    if observer is not None:
        client._observation_hook = observer.pump_ui_callbacks
    return client, socket


def test_original_single_callback_after_2179ms_browser_wait_reproduces_stale(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(matrix.time, "monotonic", clock.monotonic)
    observer, calls = observer_fixture(clock)
    client, socket = client_fixture(clock)
    client._call("Runtime.evaluate", {"expression": "read_only_fixture"})
    assert clock.value == pytest.approx(2.179)
    with pytest.raises(matrix.MatrixError, match="heartbeat is stale"):
        observer.spin_once(0.0)
    assert len(calls) == 1
    assert len(socket.sent) == 1


@pytest.mark.parametrize("cls", [matrix.GuestBrowserClient, matrix.OperatorBrowserClient])
@pytest.mark.parametrize("method", ["Runtime.evaluate", "Page.captureScreenshot", "Input.dispatchMouseEvent"])
def test_bound_hook_observes_through_long_ui_wait_without_replaying_command(monkeypatch, cls, method):
    clock = Clock()
    monkeypatch.setattr(matrix.time, "monotonic", clock.monotonic)
    observer, calls = observer_fixture(clock)
    client, socket = client_fixture(clock, observer, cls)
    assert client._call(method, {})["result"]["test_response"] is True
    assert clock.value == pytest.approx(2.179)
    assert observer._physical_samples > 40
    assert observer._physical_received_monotonic == clock.value
    assert calls and set(calls) == {0.0}
    assert max(socket.timeouts) <= 0.05
    assert len(socket.sent) == 1, "do not retry pointer or mission commands"
    assert not socket.closed


def test_real_missing_heartbeat_still_aborts_at_unchanged_15s_boundary(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(matrix.time, "monotonic", clock.monotonic)
    observer, _ = observer_fixture(clock, publishing=False)
    client, socket = client_fixture(clock, observer)
    with pytest.raises(matrix.MatrixError, match="heartbeat is stale"):
        client._call("Runtime.evaluate", {})
    assert 1.5 <= clock.value <= 1.55
    assert socket.closed


def test_real_ready_false_during_wait_aborts_and_closes_cdp(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(matrix.time, "monotonic", clock.monotonic)
    observer, _ = observer_fixture(clock, unhealthy_at=.4)
    client, socket = client_fixture(clock, observer)
    with pytest.raises(matrix.MatrixError, match="readiness lost"):
        client._call("Page.captureScreenshot", {})
    assert .4 <= clock.value <= .46
    assert socket.closed


@pytest.mark.parametrize("failure", ["ready_false", "stale", "fatal"])
def test_unhealthy_observation_prevents_actual_pointer_send(monkeypatch, failure):
    clock = Clock()
    monkeypatch.setattr(matrix.time, "monotonic", clock.monotonic)
    observer, _ = observer_fixture(clock, publishing=False)
    if failure == "ready_false":
        observer.latest["physical"]["ready"] = False
    elif failure == "stale":
        clock.value = 1.6
    else:
        observer._fatal_error = "control safety gate ERROR: actual fixture"
    client, socket = client_fixture(clock, observer)
    with pytest.raises(matrix.MatrixError):
        client._call("Input.dispatchMouseEvent", {"type": "mousePressed"})
    assert socket.sent == []


def test_original_rpc_deadline_is_not_extended_by_receive_slices(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(matrix.time, "monotonic", clock.monotonic)
    observer, _ = observer_fixture(clock)
    client, socket = client_fixture(clock, observer, duration=20)
    client.timeout_s = .4
    with pytest.raises(matrix.MatrixError, match="command timed out"):
        client._call("Runtime.evaluate", {})
    assert .4 <= clock.value <= .41
    assert len(socket.sent) == 1
    assert socket.closed


def test_fatal_hook_never_prevents_existing_stop_endpoint():
    client = matrix.GuestBrowserClient.__new__(matrix.GuestBrowserClient)
    def fatal():
        raise matrix.MatrixError("must not be called during STOP cleanup")
    calls = []
    client._observation_hook = fatal
    client.stop_client = SimpleNamespace(stop=lambda: calls.append("actual_stop_endpoint") or {"success": True})
    assert client.stop() == {"success": True}
    assert calls == ["actual_stop_endpoint"]
    assert client._observation_hook is None


def test_final_robot_constructor_binds_hook_before_ready_and_probe(monkeypatch):
    hook = lambda: None
    calls = []
    def check(client):
        assert client._observation_hook is hook
        calls.append("ready_or_probe")
    monkeypatch.setattr(matrix.OperatorBrowserClient, "_connect_operator", check)
    monkeypatch.setattr(matrix.OperatorBrowserClient, "_install_transport_probe", check)
    client = matrix.OperatorBrowserClient("http://127.0.0.1:9224", "http://127.0.0.1:8010",
                                          observation_hook=hook)
    assert len(calls) == 2
    client.close()


def test_pump_callback_budget_is_bounded_without_changing_health_contract(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(matrix.time, "monotonic", clock.monotonic)
    observer = matrix.RosObservation.__new__(matrix.RosObservation)
    calls = []
    observer.spin_once = lambda timeout: calls.append(timeout)
    observer.pump_ui_callbacks()
    assert calls == [0.0] * 16
    def slow(timeout):
        calls.append(timeout)
        clock.value += .003
    calls.clear()
    observer.spin_once = slow
    observer.pump_ui_callbacks()
    assert calls == [0.0, 0.0]
