#!/usr/bin/env python3
"""HH_261102 - Observe one admitted B7 mission, stimulus34%, real Return, then stop at docking outcome."""

from datetime import datetime, timezone
import json
from pathlib import Path
import re
import subprocess
import time

from battery_observer import fetch_api, JsonlWriter, utc
from battery_watch import STATE_KEYS
from browser import LiveOperatorBrowser


ROOT = Path(__file__).parent
OUTPUT = ROOT / "battery_aligned_step"
GENERATION = 1793618887001001
RUNTIME = ["bash", str(ROOT / "runtime.sh"), "ros"]
OUTPUT.mkdir()
writer = JsonlWriter(OUTPUT / "observations.jsonl", 16 * 1024 * 1024)
result = {"schema": "camrod.live_carla.aligned_battery_cycle.v1",
          "HH_261102": "Preserve the real normal-Return low-SOC docking sequence without forced Dock or synthetic charging.",
          "started_utc": utc(), "generation": GENERATION, "site": "B7", "intent": "delivery",
          "status": "RUNNING", "scope": "Actual CARLA tag-only test scene; not physical charger hardware.",
          "screenshots": {}, "soc_inputs": [], "native_observer": "../battery_live_03.jsonl"}
client = None
last = None
returned = False
soc_sent = False
charging_since = None
deadline = time.monotonic() + 720


def save(name, value):
    with (OUTPUT / name).open("x") as output:
        json.dump(value, output, ensure_ascii=False, indent=2)


def capture(label):
    global client
    try:
        if client is None:
            client = LiveOperatorBrowser("http://127.0.0.1:9224", "http://127.0.0.1:8010")
        result["screenshots"][label] = {"utc": utc(), "image": client.screenshot(OUTPUT / (label + ".png")),
                                         "dom": client.inspect()}
    except Exception as error:
        result["screenshots"][label] = {"utc": utc(), "capture_error": str(error)}


try:
    while time.monotonic() < deadline:
        state_response = fetch_api("http://127.0.0.1:8010", "/ui/state")
        driving = fetch_api("http://127.0.0.1:8010", "/api/driving")
        if not state_response["ok"] or not driving["ok"]:
            raise RuntimeError("actual UI API observation unavailable")
        state = state_response["data"]
        brief = {key: state.get(key) for key in STATE_KEYS}
        controllers = [item for item in state.get("module_states", []) if item.get("name") in {"control", "parking"}]
        brief["controllers"] = controllers
        writer.write({"utc": utc(), "monotonic_s": time.monotonic(), "state": brief, "driving": driving})
        result["latest"] = {"utc": utc(), "state": brief, "driving": driving}
        parking = next((item for item in controllers if item["name"] == "parking"), {})
        message = parking.get("message", "")
        phase = next((token for token in message.split() if token.startswith("phase=")), "")
        signature = (state.get("service_state_name"), phase, state.get("platform_is_charging"), state.get("ready"))
        if signature != last:
            print(json.dumps({"utc": utc(), "state": brief}, ensure_ascii=False), flush=True)
            last = signature
        if state.get("mission_execution_error") or any(item.get("level", 0) >= 2 for item in controllers):
            raise RuntimeError("native controller reports ERROR; no automatic retry")

        if not soc_sent and state.get("service_state_name") == "WAITING_FOR_RETURN_REQUEST":
            # HH_261102 - Only the already admitted exact mission authorizes this one simulator input.
            assert state.get("mission_dispatch_site") == "B7" and state.get("mission_dispatch_generation") == GENERATION
            assert state.get("mission_dispatch_active") is True and state.get("battery_percentage") == 80 and state.get("ready") is True
            env = __import__("os").environ.copy()
            env["CAMROD_SERVICE_TAG_SCENE"] = "1"
            graph = subprocess.run(RUNTIME + ["ros2", "topic", "info", "-v", "/camrod_carla/platform_heartbeat/soc"],
                                   capture_output=True, text=True, timeout=25, env=env)
            names = re.findall(r"^Node name: (.+)$", graph.stdout, re.M)
            assert graph.returncode == 0 and sorted(names) == ["carla_battery_evidence_observer", "carla_platform_heartbeat"]
            assert "Publisher count: 0" in graph.stdout
            save("soc_graph_before.json", {"utc": utc(), "stdout": graph.stdout, "stderr": graph.stderr})
            stimulus = {"utc": utc(), "monotonic_s": time.monotonic(), "fraction": 0.34}
            command = subprocess.run(RUNTIME + ["ros2", "topic", "pub", "--once", "/camrod_carla/platform_heartbeat/soc",
                                               "std_msgs/msg/Float32", "{data: 0.34}"], capture_output=True, text=True, timeout=25, env=env)
            stimulus.update(exit_code=command.returncode, stdout=command.stdout, stderr=command.stderr)
            result["soc_inputs"].append(stimulus)
            save("soc_stimulus.json", stimulus)
            soc_sent = True
            assert command.returncode == 0
            continue

        if soc_sent and not returned and state.get("battery_percentage") == 34:
            assert state.get("service_state_name") == "WAITING_FOR_RETURN_REQUEST" and state.get("mission_dispatch_generation") == GENERATION
            capture("01_arrival_soc34")
            if client is None:
                raise RuntimeError("real browser unavailable; no Return fallback")
            # HH_261102 - The top arrival confirmation is reachable; never dismiss/force the lower informational notice.
            action = client.request_return({"mission_identity": {"site": "B7", "generation": GENERATION,
                                           "owner": "operator", "intent": "delivery"}, "final_return": False})
            save("return_action.json", {"utc": utc(), "monotonic_s": time.monotonic(), **action})
            returned = True
            capture("02_return_requested")

        if returned and "parking_method=reverse" in message and "03_reverse" not in result["screenshots"]:
            capture("03_reverse")
        if returned and "parking_method=apriltag" in message and "04_apriltag" not in result["screenshots"]:
            capture("04_apriltag")
        motion = driving["data"].get("motion", {})
        speed = motion.get("speed_mps")
        if (returned and state.get("platform_is_charging") is True and state.get("service_state_name") == "CHARGING"
                and state.get("mission_dispatch_active") is False and isinstance(speed, (int, float)) and abs(speed) <= 0.02):
            charging_since = charging_since or time.monotonic()
            if time.monotonic() - charging_since >= 2:
                capture("05_actual_charging")
                result["status"] = "CHARGING_OBSERVED_STOPPED"
                break
        else:
            charging_since = None
        time.sleep(0.5)
    else:
        raise TimeoutError("12-minute observation window ended; no PASS inferred")
except Exception as error:
    result.update(status="FAILED_OR_BLOCKED", error=str(error))
    capture("failure")
finally:
    result["completed_utc"] = utc()
    result["not_performed"] = ["SOC restoration", "35percent input", "cargo leave", "forced Dock", "synthetic charging", "safety changes", "automatic retry"]
    save("result.json", result)
    writer.stream.close()
    if client is not None:
        client.close()
    print(json.dumps({"status": result["status"], "error": result.get("error"),
                      "result": str(OUTPUT / "result.json")}, ensure_ascii=False), flush=True)
