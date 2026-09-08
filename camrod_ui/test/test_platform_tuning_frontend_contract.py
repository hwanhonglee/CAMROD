"""Replay the production diagnostics callbacks; no ROS or rendered-UI substitutes."""

import json
from pathlib import Path
import re
import shutil
import subprocess

import pytest


APP = (Path(__file__).resolve().parents[1] / "camrod_ui_robot/assets/frontend/src/App.js")
SOURCE = APP.read_text(encoding="utf-8")
PREFIX = SOURCE.split("function DiagnosticsMonitor(", 1)[1].split(
    "  const requestManualReturn =", 1
)[0]
STATE_NAMES = re.findall(r"const \[(\w+),\s*\w+\] = useState\(", PREFIX)


def replay(body):
    if not shutil.which("node"):
        pytest.skip("Node.js is required for production frontend callback replay")
    script = r"""
const states = [], effects = [], requests = [], timers = new Map();
let nextTimer = 0;
function useState(value) {
  const index = states.push(value) - 1;
  return [value, next => { states[index] = next; }];
}
function useRef(value) { return {current: value}; }
function useEffect(callback) { effects.push(callback); }
function setTimeout(callback) { const id = ++nextTimer; timers.set(id, callback); return id; }
function clearTimeout(id) { timers.delete(id); }
function fetch(url, options) {
  return new Promise((resolve, reject) => requests.push({url, options, resolve, reject}));
}
const tick = async () => { for (let i = 0; i < 6; i++) await new Promise(setImmediate); };
async function respond(index, value, ok = true) {
  requests[index].resolve({ok, json: async () => value}); await tick();
}
function fireTimer() {
  const [id, callback] = [...timers][0]; timers.delete(id); callback();
}
const good = rate => ({success: true, available: true, steering_transition_rate_radps: rate});
"""
    script += "\nfunction DiagnosticsMonitor(" + PREFIX
    script += "\nreturn {requestSteeringTuning, handleSteeringRateChange, ref: steeringTuningRef};}\n"
    script += "const names = " + json.dumps(STATE_NAMES) + ";\n"
    script += r"""
const component = DiagnosticsMonitor({});
const snapshot = () => Object.fromEntries(names.map((name, index) => [name, states[index]]));
const mount = () => effects.find(callback => callback.toString().includes('requestSteeringTuning();'))();
const change = value => component.handleSteeringRateChange({target: {value: String(value)}});
(async () => {
""" + body + "\n})().catch(error => { console.error(error); process.exit(1); });\n"
    result = subprocess.run(["node"], input=script, capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    return json.loads(result.stdout)


def test_unreceived_value_is_not_a_default_and_cannot_post():
    result = replay("""
const cleanup = mount(); change(0.9); await component.requestSteeringTuning(0.9);
console.log(JSON.stringify({state: snapshot(), urls: requests.map(r => r.url), timers: timers.size}));
cleanup();
""")
    assert result["state"]["steeringRate"] is None
    assert result["state"]["steeringDraftRate"] is None
    assert result["state"]["steeringTuningAvailable"] is False
    assert result["state"]["steeringTuningPending"] is True
    assert result["urls"] == ["/ui/platform_tuning"]
    assert result["timers"] == 0


@pytest.mark.parametrize("body", [
    {"success": False, "available": False},
    {"success": True, "available": False, "steering_transition_rate_radps": 0.5},
    {"success": True, "steering_transition_rate_radps": 0.5},
    {"success": False, "available": True, "steering_transition_rate_radps": 0.5},
    {"success": True, "available": True},
    {"success": True, "available": True, "steering_transition_rate_radps": None},
    {"success": True, "available": True, "steering_transition_rate_radps": "0.50"},
    {"success": True, "available": True, "steering_transition_rate_radps": 0.0},
    {"success": True, "available": True, "steering_transition_rate_radps": 2.01},
])
def test_unavailable_or_invalid_success_never_enables_control(body):
    result = replay("""
mount(); await respond(0, BODY); change(0.9); await component.requestSteeringTuning(0.9);
console.log(JSON.stringify({state: snapshot(), count: requests.length, timers: timers.size}));
""".replace("BODY", json.dumps(body)))
    assert result["state"]["steeringRate"] is None
    assert result["state"]["steeringTuningAvailable"] is False
    assert result["state"]["steeringTuningPending"] is False
    assert "설정 서비스 미가용" in result["state"]["steeringTuningStatus"]
    assert result["count"] == 1
    assert result["timers"] == 0


@pytest.mark.parametrize("failure", [
    "await respond(0, good(0.5), false)",
    "requests[0].reject(new Error('offline')); await tick()",
    "requests[0].resolve({ok: true, json: async () => { throw new Error('bad JSON'); }}); await tick()",
    "await respond(0, good(NaN))",
    "await respond(0, good(Infinity))",
])
def test_http_network_json_and_nonfinite_failures_disable_control(failure):
    result = replay("mount(); " + failure + "; change(0.9); console.log(JSON.stringify({state:snapshot(),count:requests.length}));")
    assert result["state"]["steeringTuningAvailable"] is False
    assert result["state"]["steeringRate"] is None
    assert result["count"] == 1


def test_draft_is_not_confirmed_and_only_acknowledged_response_updates_applied_value():
    result = replay("""
mount(); await respond(0, good(0.5)); change(0.8); change(0.9);
const draft = snapshot(), debounceCount = timers.size; fireTimer();
change(1.5); await component.requestSteeringTuning(1.5);
const pending = snapshot(), postCount = requests.length;
await respond(1, good(0.85));
console.log(JSON.stringify({draft, pending, applied:snapshot(), debounceCount, postCount, post:requests[1]}));
""")
    assert result["draft"]["steeringRate"] == 0.5
    assert result["draft"]["steeringDraftRate"] == 0.9
    assert "변경값 0.90" in result["draft"]["steeringTuningStatus"]
    assert result["debounceCount"] == 1
    assert result["pending"]["steeringRate"] == 0.5
    assert result["pending"]["steeringTuningPending"] is True
    assert result["postCount"] == 2
    assert result["post"]["options"] == {"method": "POST"}
    assert result["post"]["url"].endswith("=0.90")
    assert result["applied"]["steeringRate"] == 0.85
    assert result["applied"]["steeringDraftRate"] == 0.85
    assert result["applied"]["steeringTuningAvailable"] is True
    assert result["applied"]["steeringTuningPending"] is False


@pytest.mark.parametrize("failure", [
    "await respond(1, {success:false,available:true})",
    "await respond(1, {success:false,available:false})",
    "requests[1].reject(new Error('offline')); await tick()",
])
def test_apply_failure_preserves_last_confirmed_value_but_hides_it_and_blocks_new_posts(failure):
    result = replay("""
mount(); await respond(0, good(0.5)); change(0.9); fireTimer();
FAILURE;
change(1.2); await component.requestSteeringTuning(1.2);
console.log(JSON.stringify({state:snapshot(),count:requests.length,timers:timers.size,ref:component.ref.current}));
""".replace("FAILURE", failure))
    assert result["state"]["steeringRate"] == 0.5
    assert result["state"]["steeringDraftRate"] == 0.5
    assert result["ref"]["rate"] == 0.5
    assert result["state"]["steeringTuningAvailable"] is False
    assert result["count"] == 2
    assert result["timers"] == 0


def test_unmount_cancels_draft_and_ignores_late_get_or_post_responses():
    result = replay("""
let cleanup = mount(); cleanup(); await respond(0, good(1.1));
const lateGet = snapshot();
cleanup = mount(); await respond(1, good(0.5)); change(0.9); cleanup();
const canceled = timers.size;
cleanup = mount(); await respond(2, good(0.6)); change(1.2); fireTimer();
cleanup(); const beforeLatePost = snapshot(); await respond(3, good(1.2));
await component.requestSteeringTuning(1.4);
console.log(JSON.stringify({lateGet,canceled,beforeLatePost,afterLatePost:snapshot(),count:requests.length}));
""")
    assert result["lateGet"]["steeringRate"] is None
    assert result["canceled"] == 0
    assert result["beforeLatePost"] == result["afterLatePost"]
    assert result["afterLatePost"]["steeringRate"] == 0.6
    assert result["count"] == 4


def test_strict_effect_remount_ignores_old_success_and_failure():
    result = replay("""
let cleanup = mount(); cleanup(); cleanup = mount();
await respond(1, good(0.7)); await respond(0, good(1.8));
const afterOldSuccess = snapshot();
change(1.1); fireTimer(); cleanup(); cleanup = mount();
await respond(3, good(0.8)); requests[2].reject(new Error('old failure')); await tick();
console.log(JSON.stringify({afterOldSuccess,afterOldFailure:snapshot()}));
""")
    assert result["afterOldSuccess"]["steeringRate"] == 0.7
    assert result["afterOldFailure"]["steeringRate"] == 0.8
    assert result["afterOldFailure"]["steeringTuningAvailable"] is True
    assert result["afterOldFailure"]["steeringTuningPending"] is False


def test_invalid_changes_do_not_schedule_or_post():
    result = replay("""
mount(); await respond(0, good(0.5));
for (const value of [NaN, Infinity, -1, 0, 2.1]) {
  change(value); await component.requestSteeringTuning(value);
}
console.log(JSON.stringify({state:snapshot(),count:requests.length,timers:timers.size}));
""")
    assert result["state"]["steeringDraftRate"] == 0.5
    assert result["count"] == 1
    assert result["timers"] == 0


def test_dom_shows_only_confirmed_available_value_and_disables_unavailable_or_pending_slider():
    card = SOURCE.split('<div className="steering-tuning-card">', 1)[1].split(
        '<div className="steering-tuning-scale">', 1
    )[0]
    assert "steeringTuningAvailable && steeringRate !== null ? steeringRate.toFixed(2) : '—'" in card
    assert "disabled={!steeringTuningAvailable || steeringTuningPending}" in card
    assert "value={steeringDraftRate ?? 0.05}" in card
    assert "확인된 적용값" in card
    assert "setSteeringRate(nextRate)" not in PREFIX
    assert "useState(0.5)" not in PREFIX
    assert "CARLA" not in PREFIX
