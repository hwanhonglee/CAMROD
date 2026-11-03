#!/usr/bin/env python3
"""HH_261103 - Finish only an already admitted delivery through real UI Return and observed CARLA charging."""

import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time

# HH_261103 - Reuse the proven evidence/input helpers, not another browser or ROS control framework.
ROOT = Path(__file__).parent
sys.path.insert(0, str(ROOT.parents[1] / "service_usability_20261102/live_carla"))
from battery_observer import fetch_api, JsonlWriter, utc
from battery_watch import STATE_KEYS
from browser import LiveOperatorBrowser


def validate_identity(state, generation, site, arrival_soc=None):
    if (type(generation) is not int or generation <= 0
            or type(state.get("mission_dispatch_generation")) is not int
            or state.get("mission_dispatch_generation") != generation
            or state.get("mission_dispatch_site") != site
            or state.get("mission_dispatch_active") is not True
            or state.get("mission_dispatch_intent") != "delivery"
            or state.get("mission_dispatch_owner") != "operator"):
        raise RuntimeError("already admitted operator delivery identity changed; no action")
    if arrival_soc is not None and (state.get("service_state_name") != "WAITING_FOR_RETURN_REQUEST"
            or state.get("battery_percentage") != arrival_soc or state.get("ready") is not True):
        raise RuntimeError("arrival/SOC/readiness precondition failed; no action")


def validate_runtime(environment):
    if (environment.get("ROS_DOMAIN_ID") != "188" or environment.get("ROS_LOCALHOST_ONLY") != "1"
            or environment.get("CAMROD_RECORDING_ENVIRONMENT") != "simulation"):
        raise RuntimeError("run through this date's runtime.sh ros: isolated simulation domain188 required")


def validate_soc_graph(output):
    names = re.findall(r"^Node name: (.+)$", output, re.M)
    if (sorted(names) != ["carla_battery_evidence_observer", "carla_platform_heartbeat"]
            or "Publisher count: 0" not in output or "Type: std_msgs/msg/Float32" not in output):
        raise RuntimeError("SOC topic is not exclusively CARLA heartbeat plus passive observer")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generation", type=int, required=True)
    parser.add_argument("--site", choices=["B7"], required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    validate_runtime(os.environ)
    if args.generation <= 0:
        parser.error("generation must be positive")
    args.output.mkdir()  # HH_261103 - A new directory prevents overwriting an earlier trial.
    writer = JsonlWriter(args.output / "observations.jsonl", 16 * 1024 * 1024)
    result = {"schema": "camrod.live_carla.watch_return.v1", "HH_261103": "No dispatch, forced Dock or fabricated charging.",
              "started_utc": utc(), "generation": args.generation, "site": args.site,
              "status": "RUNNING", "photos": [], "soc_inputs": []}
    client = None
    returned = soc_sent = False
    last = None
    charging_since = None

    def save(name, value):
        with (args.output / name).open("x") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2)

    def photo(label, state=None, parking=None):
        nonlocal client
        row = {"utc": utc(), "monotonic_s": time.monotonic(), "label": label,
               "service_state": (state or {}).get("service_state_name"),
               "platform_is_charging": (state or {}).get("platform_is_charging"), "parking": parking}
        try:
            if client is None:
                client = LiveOperatorBrowser("http://127.0.0.1:9224", "http://127.0.0.1:8010")
            row.update(image=client.screenshot(args.output / f"{len(result['photos']) + 1:02d}_{label}.png"), dom=client.inspect())
        except Exception as error:
            row["capture_error"] = str(error)
        result["photos"].append(row)

    try:
        deadline = time.monotonic() + 720
        while time.monotonic() < deadline:
            responses = {route: fetch_api("http://127.0.0.1:8010", route)
                         for route in ("/ui/state", "/api/driving", "/api/charging-access")}
            if not all(response["ok"] for response in responses.values()):
                raise RuntimeError("actual API observation unavailable; no action")
            state = responses["/ui/state"]["data"]
            driving = responses["/api/driving"]["data"]
            access = responses["/api/charging-access"]["data"]
            brief = {key: state.get(key) for key in STATE_KEYS}
            controllers = [row for row in state.get("module_states", []) if row.get("name") in {"control", "parking"}]
            brief["controllers"] = controllers
            row = {"utc": utc(), "monotonic_s": time.monotonic(), "state": brief, "driving": driving, "charging_access": access}
            writer.write(row)
            result["latest"] = row
            parking = next((row for row in controllers if row["name"] == "parking"), {})
            phase = next((token for token in parking.get("message", "").split() if token.startswith("phase=")), "")
            signature = (state.get("service_state_name"), phase, state.get("platform_is_charging"), state.get("ready"))
            if signature != last:
                print(json.dumps({"utc": utc(), "state": brief}, ensure_ascii=False), flush=True)
                if returned and (phase or state.get("service_state_name") in {"CHARGING", "WAITING_FOR_CHARGING"}) and len(result["photos"]) < 24:
                    photo("stage", state, parking)
                last = signature
            if state.get("mission_execution_error") or any(row.get("level", 0) >= 2 for row in controllers):
                raise RuntimeError("native controller ERROR; no external retry")
            if not returned:
                validate_identity(state, args.generation, args.site)
            if not soc_sent and state.get("service_state_name") == "WAITING_FOR_RETURN_REQUEST":
                validate_identity(state, args.generation, args.site, arrival_soc=80)
                env = dict(os.environ, CAMROD_SERVICE_TAG_SCENE="1")
                command = ["bash", str(ROOT / "runtime.sh"), "ros", "ros2", "topic"]
                graph = subprocess.run(command + ["info", "-v", "/camrod_carla/platform_heartbeat/soc"],
                                       capture_output=True, text=True, timeout=25, env=env)
                save("soc_graph.json", {"utc": utc(), "exit_code": graph.returncode, "stdout": graph.stdout, "stderr": graph.stderr})
                if graph.returncode:
                    raise RuntimeError("CARLA SOC graph query failed")
                validate_soc_graph(graph.stdout)
                stimulus = {"utc": utc(), "monotonic_s": time.monotonic(), "fraction": .34}
                sent = subprocess.run(command + ["pub", "--once", "/camrod_carla/platform_heartbeat/soc",
                    "std_msgs/msg/Float32", "{data: 0.34}"], capture_output=True, text=True, timeout=25, env=env)
                stimulus.update(exit_code=sent.returncode, stdout=sent.stdout, stderr=sent.stderr)
                result["soc_inputs"].append(stimulus)
                save("soc_stimulus.json", stimulus)
                if sent.returncode:
                    raise RuntimeError("single CARLA SOC stimulus failed; no retry")
                soc_sent = True
                continue
            if soc_sent and not returned and state.get("battery_percentage") == 34:
                validate_identity(state, args.generation, args.site, arrival_soc=34)
                photo("arrival_soc34", state, parking)
                if client is None:
                    raise RuntimeError("real browser unavailable; no backend Return fallback")
                # HH_261103 - Click only the real reachable top Return; never force the lower informational alert.
                validate_identity(fetch_api("http://127.0.0.1:8010", "/ui/state")["data"], args.generation, args.site, arrival_soc=34)
                action = client.request_return({"mission_identity": {"site": args.site, "generation": args.generation,
                                               "owner": "operator", "intent": "delivery"}, "final_return": False})
                save("return_action.json", {"utc": utc(), "monotonic_s": time.monotonic(), **action})
                returned = True
                photo("return_requested", state, parking)
            motion = driving.get("motion", {})
            speed = motion.get("speed_mps")
            age = motion.get("age_s")
            fresh_stop = (type(speed) in (int, float) and math.isfinite(speed) and abs(speed) <= .02
                          and type(age) in (int, float) and 0 <= age <= .5)
            if (returned and state.get("platform_is_charging") is True and state.get("service_state_name") == "CHARGING"
                    and state.get("mission_dispatch_active") is False and access.get("platform_feedback_fresh") is True
                    and access.get("platform_is_charging") is True and fresh_stop):
                charging_since = charging_since or time.monotonic()
                if time.monotonic() - charging_since >= 2:
                    photo("actual_charging", state, parking)
                    result["status"] = "CHARGING_OBSERVED_STOPPED"
                    break
            else:
                charging_since = None
            time.sleep(.5)
        else:
            raise TimeoutError("12-minute observation bound reached; no PASS inferred")
    except Exception as error:
        result.update(status="FAILED_OR_BLOCKED", error=str(error))
        photo("failure")
    finally:
        result["completed_utc"] = utc()
        result["not_performed"] = ["dispatch", "SOC restoration", "35percent input", "cargo leave", "forced Dock", "synthetic feedback", "external retry"]
        save("result.json", result)
        writer.stream.close()
        if client is not None:
            client.close()
        print(json.dumps({"status": result["status"], "error": result.get("error"), "result": str(args.output / "result.json")}), flush=True)
    return 0 if result["status"] == "CHARGING_OBSERVED_STOPPED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
