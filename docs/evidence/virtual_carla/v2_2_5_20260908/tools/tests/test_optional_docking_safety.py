"""Offline-only Dock helper regressions; never construct a real ROS/UI client.

The transport and observation objects below are local fakes. Production CDP
response parsing and the actual evidence helper are executed against them.
These results do not certify a real docking mission or charger.
"""
import ast
import copy
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
MATRIX_PATH = Path("/home/hong/camrod_ws/src/scripts/virtual_carla/camping_site_matrix.py")


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def helper():
    return load_module(ROOT / "run_optional_docking_ui.py", "optional_dock_under_test")


@pytest.fixture
def matrix():
    return load_module(MATRIX_PATH, "optional_dock_matrix_under_test")


class FakeSocket:
    def __init__(self, responses):
        self.responses = list(responses)
        self.sent = []
        self.timeouts = []

    def send(self, message):
        self.sent.append(json.loads(message))

    def recv(self):
        assert self.responses, "Unexpected extra socket read (no real transport available)"
        return json.dumps(self.responses.pop(0))

    def settimeout(self, timeout):
        self.timeouts.append(timeout)


def browser_without_network(helper, matrix, responses):
    cls = helper.make_dock_browser_class(matrix)
    assert "_call" not in cls.__dict__, "Dock must retain the shared observation-pumping CDP parser"
    browser = cls.__new__(cls)
    browser._connection = FakeSocket(responses)
    browser._command_id = 0
    browser.timeout_s = 1.0
    browser._observed_confirmation = None
    browser._accept_expected_confirmation = False
    browser._expected_confirmation_text = None
    pumps = []
    browser._observation_hook = lambda: pumps.append("pump")
    return browser, pumps


def test_actual_shared_cdp_call_pumps_and_records_exact_native_confirm(helper, matrix):
    text = "배터리 잔량과 관계없이 충전 도킹을 요청합니다. 진행하시겠습니까?"
    responses = [
        {"method": "Page.javascriptDialogOpening", "params": {"type": "confirm", "message": text}},
        {"id": 2, "result": {}},
        {"id": 1, "result": {"ok": True}},
    ]
    browser, pumps = browser_without_network(helper, matrix, responses)
    browser.accept_docking_dialog = True
    reply = browser._call("Input.dispatchMouseEvent", {"type": "mouseReleased"})
    assert reply["result"]["ok"] is True
    assert len(pumps) >= 3
    assert all(0 < value <= .05 for value in browser._connection.timeouts)
    confirmations = [item for item in browser._connection.sent if item["method"] == "Page.handleJavaScriptDialog"]
    assert len(confirmations) == 1 and confirmations[0]["params"] == {"accept": True}
    assert browser.dialogs[0]["message"] == text
    assert browser.dialogs[0]["accepted"] is True


@pytest.mark.parametrize("enabled,kind,text", [
    (False, "confirm", "배터리 잔량과 관계없이 충전 도킹을 요청합니다. 진행하시겠습니까?"),
    (True, "alert", "배터리 잔량과 관계없이 충전 도킹을 요청합니다. 진행하시겠습니까?"),
    (True, "confirm", "different action"),
])
def test_unexpected_or_unauthorized_dialog_never_accepted(helper, matrix, enabled, kind, text):
    browser, _ = browser_without_network(helper, matrix, [
        {"method": "Page.javascriptDialogOpening", "params": {"type": kind, "message": text}},
    ])
    browser.accept_docking_dialog = enabled
    connection = browser._connection
    with pytest.raises(matrix.MatrixError):
        browser._call("Input.dispatchMouseEvent", {})
    assert not any(item["method"] == "Page.handleJavaScriptDialog" for item in connection.sent)


def test_constructor_forwards_observation_hook_without_own_transport(helper):
    calls = []

    class Parent:
        def __init__(self, *args, **kwargs):
            calls.append((args, kwargs))

    hook = lambda: None
    cls = helper.make_dock_browser_class(SimpleNamespace(OperatorBrowserClient=Parent))
    cls("offline-cdp", "offline-ui", timeout_s=15, observation_hook=hook)
    assert calls == [(("offline-cdp", "offline-ui"), {"timeout_s": 15, "observation_hook": hook})]
    assert "_call" not in cls.__dict__


class Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += max(duration, .01)


class Observer:
    def __init__(self, clock, *, speed=0.0, state=16, fresh=True, spin_error=False):
        self.clock = clock
        self.speed = speed
        self.state = state
        self.fresh = fresh
        self.spin_error = spin_error
        self.count = 10
        self._latest_odom_stamp_ns = 10000000000
        self._carla_odom_samples = self.count

    def spin_once(self, duration):
        self.clock.now += max(duration, .05)
        if self.fresh:
            self.count += 1
            self._latest_odom_stamp_ns += 50000000
            self._carla_odom_samples = self.count
        if self.spin_error:
            raise RuntimeError("offline fatal observer sample")

    def snapshot(self):
        return {
            "service_state": {"state": self.state, "state_name": "OPERATOR_STOPPED" if self.state == 16 else "DRIVING"},
            "carla_odometry": {"speed_mps": self.speed, "stamp_sec": float(self.count), "sample_count": self.count},
            "sample_counts": {"carla_odometry": self.count},
        }


class StopBrowser:
    def __init__(self, error=None):
        self.error = error
        self.calls = 0
        self._observation_hook = lambda: (_ for _ in ()).throw(RuntimeError("stale observer must not block STOP"))

    def stop(self):
        assert self._observation_hook is None, "STOP must not be pre-empted by a failing observer hook"
        self.calls += 1
        if self.error:
            raise self.error
        return {"success": True, "action": "stopped"}


def test_stop_request_once_even_when_no_observation_time(helper, monkeypatch):
    clock = Clock()
    monkeypatch.setattr(helper, "time", clock)
    browser = StopBrowser()
    evidence = {"status": "FAIL", "error": "original dock failure"}
    result = helper.safety_stop_after_failure(SimpleNamespace(), Observer(clock), browser, "offline-ui", evidence, timeout_s=0)
    assert result is evidence["safety_stop"]
    assert browser.calls == 1
    assert result["requested"] is True and result["stop_observed"] is False
    assert evidence["status"] == "FAIL" and evidence["error"] == "original dock failure"


def test_stop_transport_failure_is_recorded_not_raised(helper, monkeypatch):
    clock = Clock()
    monkeypatch.setattr(helper, "time", clock)
    browser = StopBrowser(RuntimeError("offline STOP transport error"))
    evidence = {"status": "FAIL"}
    result = helper.safety_stop_after_failure(SimpleNamespace(), Observer(clock), browser, "offline-ui", evidence, timeout_s=0)
    assert browser.calls == 1
    assert "offline STOP transport error" in result["request_error"]
    assert result["stop_observed"] is False and evidence["status"] == "FAIL"


def test_missing_browser_uses_only_existing_bounded_stop_client(helper, monkeypatch):
    clock = Clock()
    monkeypatch.setattr(helper, "time", clock)
    calls = []

    class UIClient:
        def __init__(self, url, timeout_s):
            calls.append((url, timeout_s))

        def stop(self):
            calls.append("stop")
            return {"success": True}

    result = helper.safety_stop_after_failure(SimpleNamespace(UIClient=UIClient), Observer(clock), None, "offline-ui", {}, timeout_s=0)
    assert calls == [("offline-ui", 3.0), "stop"]
    assert result["requested"] is True and not result["stop_observed"]


@pytest.mark.parametrize("fresh,speed,state", [
    (False, 0.0, 16), (True, .2, 16), (True, 0.0, 9),
    (True, float("nan"), 16), (True, float("inf"), 16), (True, -.2, 16),
])
def test_stale_moving_or_wrong_state_never_confirms_stop(helper, monkeypatch, fresh, speed, state):
    clock = Clock()
    monkeypatch.setattr(helper, "time", clock)
    result = helper.safety_stop_after_failure(SimpleNamespace(), Observer(clock, fresh=fresh, speed=speed, state=state), StopBrowser(), "offline-ui", {}, timeout_s=.6)
    assert result["stop_observed"] is False
    assert clock.now <= 100.8


def test_fresh_stopped_observations_require_settled_hold_but_never_pass_dock(helper, monkeypatch):
    clock = Clock()
    monkeypatch.setattr(helper, "time", clock)
    evidence = {"status": "FAIL", "error": "original failure"}
    result = helper.safety_stop_after_failure(SimpleNamespace(), Observer(clock), StopBrowser(), "offline-ui", evidence, timeout_s=.8)
    assert result["stop_observed"] is True
    assert result["final_observation"]["elapsed_s"] >= .3
    assert result["final_observation"]["fresh_odometry"] is True
    assert evidence["status"] == "FAIL" and evidence["error"] == "original failure"


def test_short_stop_observation_window_cannot_claim_settled_hold(helper, monkeypatch):
    clock = Clock()
    monkeypatch.setattr(helper, "time", clock)
    result = helper.safety_stop_after_failure(SimpleNamespace(), Observer(clock), StopBrowser(), "offline-ui", {}, timeout_s=.2)
    assert result["stop_observed"] is False


def test_observer_error_cannot_prevent_stop_request_or_escape_cleanup(helper, monkeypatch):
    clock = Clock()
    monkeypatch.setattr(helper, "time", clock)
    browser = StopBrowser()
    result = helper.safety_stop_after_failure(SimpleNamespace(), Observer(clock, spin_error=True), browser, "offline-ui", {}, timeout_s=.2)
    assert browser.calls == 1
    assert result["observation_errors"]


@pytest.mark.parametrize("clicked", [False, True])
@pytest.mark.parametrize("snapshot_fails", [False, True])
@pytest.mark.parametrize("original", [RuntimeError("original failure"), KeyboardInterrupt("original interrupt")])
def test_actual_exception_handler_preserves_original_and_stops_only_after_click(helper, clicked, snapshot_fails, original):
    tree = ast.parse((ROOT / "run_optional_docking_ui.py").read_text())
    main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
    handlers = [node for node in ast.walk(main) if isinstance(node, ast.ExceptHandler)
                and isinstance(node.type, ast.Name) and node.type.id == "BaseException"
                and any(isinstance(child, ast.Call) and isinstance(child.func, ast.Name)
                        and child.func.id == "safety_stop_after_failure" for child in ast.walk(node))]
    assert len(handlers) == 1, "Production main must clean up interrupts as well as Exceptions"
    program = ast.fix_missing_locations(ast.Module(body=[ast.Try(
        body=[ast.Raise(exc=ast.Name(id="original", ctx=ast.Load()), cause=None)],
        handlers=handlers, orelse=[], finalbody=[])], type_ignores=[]))
    stop_calls = []
    evidence = {"status": "RUNNING"}
    def snapshot():
        if snapshot_fails:
            raise RuntimeError("offline snapshot error")
        return {}

    scope = dict(original=original, evidence=evidence, observer=SimpleNamespace(snapshot=snapshot),
                 extras={}, clicked=[clicked], matrix=None, browser=None, args=SimpleNamespace(ui_url="offline-ui"),
                 safety_stop_after_failure=lambda *args, **kwargs: stop_calls.append(args))
    with pytest.raises(type(original)) as caught:
        exec(compile(program, "<actual-dock-exception-handler>", "exec"), scope)
    assert caught.value is original
    assert evidence["status"] == ("INTERRUPTED" if isinstance(original, KeyboardInterrupt) else "FAIL")
    assert len(stop_calls) == int(clicked)


def initial_preflight_inputs():
    """Fresh inputs are explicit independent observations, not a ROS state fill."""
    snapshot = {
        "service_state": {"state": 0, "state_name": "DROP_ZONE_WAIT"},
        "parking": {
            "dispatcher": {"level": 0, "operating_state": "PARKED",
                           "message": "parking_method=reverse charging_required=false"},
            "reverse": {"level": 0, "operating_state": "PARKED"},
        },
        "gate": {"level": 0, "operating_state": "STANDBY"},
        "carla_odometry": {"speed_mps": 0.0},
    }
    extras = {
        "platform": {"is_charging": False, "received_monotonic": 99.8,
                     "control_mode": 1, "estop": False, "error_code": 0},
        "dummy_camera_active": False,
        "rear_camera_received_monotonic": 99.8,
    }
    ui = {"service_state": 0, "service_state_name": "DROP_ZONE_WAIT", "mission_phase": "READY",
          "ready": True, "engaged": False, "mission_dispatch_active": False}
    return snapshot, extras, ui


def call_initial_preflight(helper, snapshot, extras, ui, image_present=True):
    return helper.validate_initial_dock_preflight(snapshot, extras, image_present, ui, now=100.0)


def test_explicit_healthy_ros_and_fresh_ui_reverse_park_accepted(helper):
    snapshot, extras, ui = initial_preflight_inputs()
    call_initial_preflight(helper, snapshot, extras, ui)


def test_late_join_absent_ros_state_uses_independent_ui_without_state_injection(helper):
    snapshot, extras, ui = initial_preflight_inputs()
    snapshot["service_state"] = None
    before = copy.deepcopy((snapshot, extras, ui))
    call_initial_preflight(helper, snapshot, extras, ui)
    assert (snapshot, extras, ui) == before
    assert snapshot["service_state"] is None


def test_empty_received_ros_state_is_malformed_not_assumed_absent(helper):
    snapshot, extras, ui = initial_preflight_inputs()
    snapshot["service_state"] = {}
    with pytest.raises(RuntimeError):
        call_initial_preflight(helper, snapshot, extras, ui)


@pytest.mark.parametrize("field,value", [
    ("service_state", None), ("service_state", 13),
    ("service_state_name", None), ("service_state_name", "CHARGING"),
    ("mission_phase", None), ("mission_phase", "RECALL_TO_SITE_ROAD"),
    ("ready", None), ("ready", False),
    ("engaged", None), ("engaged", True),
    ("mission_dispatch_active", None), ("mission_dispatch_active", True),
])
def test_late_join_never_passes_missing_or_nonidle_ui_contract(helper, field, value):
    snapshot, extras, ui = initial_preflight_inputs()
    snapshot["service_state"] = None
    ui[field] = value
    with pytest.raises(RuntimeError):
        call_initial_preflight(helper, snapshot, extras, ui)


def test_late_join_no_ui_snapshot_is_rejected_without_none_dereference(helper):
    snapshot, extras, _ = initial_preflight_inputs()
    snapshot["service_state"] = None
    with pytest.raises(RuntimeError):
        call_initial_preflight(helper, snapshot, extras, None)


@pytest.mark.parametrize("ros_state", [
    {"state": 13, "state_name": "CHARGING"},
    {"state": 16, "state_name": "OPERATOR_STOPPED"},
    {"state": 0, "state_name": "CHARGING"},
    {"state": 13, "state_name": "DROP_ZONE_WAIT"},
])
def test_received_ros_state_cannot_contradict_fresh_idle_ui(helper, ros_state):
    snapshot, extras, ui = initial_preflight_inputs()
    snapshot["service_state"] = ros_state
    with pytest.raises(RuntimeError):
        call_initial_preflight(helper, snapshot, extras, ui)


@pytest.mark.parametrize("parent,child", [
    (None, "parking"), ("parking", "dispatcher"), ("parking", "reverse"), (None, "carla_odometry"),
])
def test_required_ros_observation_none_is_rejected_cleanly(helper, parent, child):
    snapshot, extras, ui = initial_preflight_inputs()
    target = snapshot if parent is None else snapshot[parent]
    target[child] = None
    with pytest.raises(RuntimeError):
        call_initial_preflight(helper, snapshot, extras, ui)


@pytest.mark.parametrize("controller", ["dispatcher", "reverse"])
def test_nonparked_controller_never_becomes_dock_preflight(helper, controller):
    snapshot, extras, ui = initial_preflight_inputs()
    snapshot["parking"][controller]["operating_state"] = "APPROACH"
    with pytest.raises(RuntimeError):
        call_initial_preflight(helper, snapshot, extras, ui)


@pytest.mark.parametrize("speed", [None, float("nan"), float("inf"), -float("inf"), -.1, .05, .2])
def test_initial_speed_must_be_finite_nonnegative_and_below_stationary_limit(helper, speed):
    snapshot, extras, ui = initial_preflight_inputs()
    snapshot["carla_odometry"]["speed_mps"] = speed
    with pytest.raises(RuntimeError):
        call_initial_preflight(helper, snapshot, extras, ui)


@pytest.mark.parametrize("timestamp", [None, 98.9, 100.2, float("nan"), float("inf")])
@pytest.mark.parametrize("sensor", ["platform", "camera"])
def test_initial_platform_and_camera_must_have_valid_fresh_receive_times(helper, timestamp, sensor):
    snapshot, extras, ui = initial_preflight_inputs()
    if sensor == "platform":
        extras["platform"]["received_monotonic"] = timestamp
    else:
        extras["rear_camera_received_monotonic"] = timestamp
    with pytest.raises(RuntimeError):
        call_initial_preflight(helper, snapshot, extras, ui)


@pytest.mark.parametrize("change", ["platform_none", "charging_true", "charging_unknown", "dummy_true", "dummy_unknown", "image_absent"])
def test_initial_camera_and_noncharging_contract_preserves_unknown_dummy_without_fabrication(helper, change):
    snapshot, extras, ui = initial_preflight_inputs()
    if change == "platform_none":
        extras["platform"] = None
    elif change.startswith("charging_"):
        extras["platform"]["is_charging"] = True if change == "charging_true" else None
    elif change.startswith("dummy_"):
        extras["dummy_camera_active"] = True if change == "dummy_true" else None
    if change == "dummy_unknown":
        before = copy.deepcopy((snapshot, extras, ui))
        call_initial_preflight(helper, snapshot, extras, ui)
        assert extras["dummy_camera_active"] is None
        assert (snapshot, extras, ui) == before
        return
    with pytest.raises(RuntimeError):
        call_initial_preflight(helper, snapshot, extras, ui, image_present=change != "image_absent")
