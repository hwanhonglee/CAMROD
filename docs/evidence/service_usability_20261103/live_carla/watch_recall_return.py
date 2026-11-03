#!/usr/bin/env python3
"""HH_261103 - Observe an admitted Robot-UI B7 recall and use its two real completion confirmations; never set SOC."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).parent))
from run_cargo_exit import snapshot, LiveOperatorBrowser, JsonlWriter, utc
from watch_return import validate_runtime
from browser import matrix


def validate_recall_identity(state, generation, site):
    # HH_261103 - Robot-screen recall is canonically owner robot, not delivery owner operator or Guest.
    expected = {"site": site, "owner": "robot", "intent": "recall", "generation": generation}
    if type(generation) is not int or generation <= 0 or matrix.mission_identity(state) != expected:
        raise RuntimeError("admitted Robot recall identity changed; no action")
    return expected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generation", required=True, type=int)
    parser.add_argument("--site", required=True, choices=["B7"])
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    validate_runtime(os.environ)
    if args.generation <= 0:
        parser.error("generation must be positive")
    args.output.mkdir()
    writer = JsonlWriter(args.output / "observations.jsonl", 16 * 1024 * 1024)
    result = {"schema": "camrod.live_carla.recall_return.v1", "started_utc": utc(), "status": "RUNNING",
              "site": args.site, "generation": args.generation, "confirmations": [], "photos": []}
    client, last, stable_since = None, None, None

    def photo(label, row):
        nonlocal client
        if client is None:
            client = LiveOperatorBrowser("http://127.0.0.1:9224", "http://127.0.0.1:8010")
        result["photos"].append({"utc": utc(), "label": label, "service_state": row["/ui/state"].get("service_state_name"),
            "image": client.screenshot(args.output / f"{len(result['photos']) + 1:02d}_{label}.png"), "dom": client.inspect()})

    try:
        deadline = time.monotonic() + 720
        while time.monotonic() < deadline:
            row = snapshot(); writer.write(row); result["latest"] = row
            state, driving = row["/ui/state"], row["/api/driving"]
            controllers = [item for item in state.get("module_states", []) if item.get("name") in {"control", "parking"}]
            if state.get("mission_execution_error") or any(item.get("level", 0) >= 2 for item in controllers):
                raise RuntimeError("native controller ERROR; no retry")
            if state.get("battery_percentage") != 35:
                raise RuntimeError("observed SOC changed from35%; no override")
            identity = (validate_recall_identity(state, args.generation, args.site)
                        if state.get("mission_dispatch_active") or len(result["confirmations"]) < 2 else None)
            control = next((item.get("message", "") for item in controllers if item["name"] == "control"), "")
            phase = next((token[6:] for token in control.split() if token.startswith("phase=")), "")
            parking = next((item.get("message", "") for item in controllers if item["name"] == "parking"), "")
            signature = (state.get("service_state_name"), phase, parking.split("phase=")[-1])
            if signature != last:
                print(json.dumps({"utc": row["utc"], "service": signature[0], "phase": phase, "parking": parking}), flush=True)
                if len(result["photos"]) < 12:
                    photo("phase", row)
                last = signature
            final = phase == "RECALL_RETURN_WAIT"
            stage = 1 if final else 0
            if (identity and state.get("service_state") == 8 and phase in {"WAIT_RETURN", "RECALL_RETURN_WAIT"}
                    and stage == len(result["confirmations"]) and (not final or state.get("recall_final_return_ready") is True)):
                # HH_261103 - Preserve B7's separate road loading-clearance and post-turn loading-complete authority.
                context = matrix.return_context(state, {"site": {"operating_state": phase},
                    "service_state": {"state": state["service_state"]}}, identity, final_return=final)
                photo("before_final" if final else "before_first", row)
                fresh = snapshot()["/ui/state"]
                validate_recall_identity(fresh, args.generation, args.site)
                if fresh.get("service_state") != 8 or (final and fresh.get("recall_final_return_ready") is not True):
                    raise RuntimeError("confirmation phase changed before real pointer input")
                result["confirmations"].append({"utc": utc(), **client.request_return(context)})
            motion = driving.get("motion", {}); speed, age = motion.get("speed_mps"), motion.get("age_s")
            terminal = (len(result["confirmations"]) == 2 and state.get("mission_dispatch_active") is False
                and state.get("service_state_name") == "DROP_ZONE_WAIT" and "parking_method=reverse" in parking
                and "phase=PARKED" in parking and state.get("platform_is_charging") is False
                and type(speed) in (int, float) and math.isfinite(speed) and abs(speed) <= .02
                and type(age) in (int, float) and 0 <= age <= .5)
            stable_since = (stable_since or time.monotonic()) if terminal else None
            if stable_since and time.monotonic() - stable_since >= 2:
                photo("final_reverse_parked", row); result["status"] = "RECALL_TWO_CONFIRMATIONS_RETURN_PARKED_OBSERVED"; break
            time.sleep(.5)
        else:
            raise TimeoutError("12-minute observation bound reached")
    except Exception as error:
        result.update(status="FAILED_OR_BLOCKED", error=str(error))
        if client is not None:
            try: photo("failure", result["latest"])
            except Exception as capture_error: result["capture_error"] = str(capture_error)
    finally:
        result["completed_utc"] = utc()
        result["not_performed"] = ["dispatch", "SOC change", "forced Dock", "synthetic feedback", "retry"]
        with (args.output / "result.json").open("x") as stream: json.dump(result, stream, ensure_ascii=False, indent=2)
        writer.stream.close()
        if client is not None: client.close()
        print(json.dumps({"status": result["status"], "error": result.get("error"), "result": str(args.output / "result.json")}), flush=True)
    return 0 if result["status"] == "RECALL_TWO_CONFIRMATIONS_RETURN_PARKED_OBSERVED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
