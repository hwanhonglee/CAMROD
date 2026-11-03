#!/usr/bin/env python3
"""HH_261102 - Save bounded live battery Return observations without sending controls."""

import argparse
import json
from pathlib import Path
import time

from battery_observer import JsonlWriter, fetch_api, utc


STATE_KEYS = ("ready", "ready_message", "service_state", "service_state_name",
              "service_state_description", "battery_percentage", "platform_is_charging",
              "mission_dispatch_active", "mission_dispatch_site", "mission_dispatch_generation",
              "battery_return_pending", "battery_return_started", "battery_return_urgent",
              "mission_execution_error", "engaged", "operation_mode", "system_health")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=int, default=60)
    args = parser.parse_args()
    if not 1 <= args.duration <= 60:
        parser.error("duration must be 1..60 seconds")
    writer = JsonlWriter(args.output, 8 * 1024 * 1024)
    deadline = time.monotonic() + args.duration
    last = None
    try:
        while time.monotonic() < deadline:
            state = fetch_api("http://127.0.0.1:8010", "/ui/state")
            if state["ok"]:
                value = state["data"]
                state["data"] = {key: value.get(key) for key in STATE_KEYS}
                state["data"]["controllers"] = [item for item in value.get("module_states", [])
                    if item.get("name") in {"control", "parking"}]
            driving = fetch_api("http://127.0.0.1:8010", "/api/driving")
            writer.write({"utc": utc(), "monotonic_s": time.monotonic(), "state": state, "driving": driving})
            brief = state.get("data", {})
            signature = (brief.get("service_state_name"), brief.get("platform_is_charging"),
                         brief.get("mission_execution_error"), brief.get("ready"))
            if signature != last:
                print(json.dumps({"utc": utc(), "state": brief}, ensure_ascii=False), flush=True)
                last = signature
            # HH_261102 - Charging must be real current feedback, never inferred from a controller label.
            if brief.get("platform_is_charging") is True:
                print(json.dumps({"utc": utc(), "observed_charging": True}), flush=True)
                break
            if brief.get("mission_execution_error"):
                print(json.dumps({"utc": utc(), "observed_failure": True}), flush=True)
                break
            time.sleep(1)
    finally:
        writer.stream.close()
    print(json.dumps({"output": str(args.output), "rows": writer.rows, "bytes": writer.bytes_written}))


if __name__ == "__main__":
    main()
