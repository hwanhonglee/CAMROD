#!/usr/bin/env python3
"""HH_261102 - Passively retain live CARLA battery/charging evidence, never control it.

Run in the already isolated CARLA ROS environment. Default: 60 seconds; use
--duration 0 for SIGINT termination. Output is exclusive-create and size capped.
Receipt clocks and source stamps remain separate; this script assigns no PASS.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import time
from urllib.parse import urlsplit
from urllib.request import Request, urlopen


# HH_261102 - Subscribe to native contracts instead of importing an old motion-capable runner.
TOPICS = {
    "/platform/status": ("avg_msgs/msg/AvgPlatformStatus", "sample"),
    "/camrod_carla/platform_heartbeat/soc": ("std_msgs/msg/Float32", "command"),
    "/camrod_carla/platform_heartbeat/charging": ("std_msgs/msg/Bool", "state"),
    "/service/state": ("avg_msgs/msg/AvgServiceState", "state"),
    "/planning/service_state": ("avg_msgs/msg/AvgServiceState", "state"),
    "/planning/state_machine/state": ("avg_msgs/msg/PlanningState", "state"),
    "/parking/status": ("avg_msgs/msg/ModuleState", "state"),
    "/parking/private/reverse/status": ("avg_msgs/msg/ModuleState", "state"),
    "/parking/private/apriltag/status": ("avg_msgs/msg/ModuleState", "state"),
    "/control/drop_zone_maneuver_controller/status": ("avg_msgs/msg/ModuleState", "state"),
    "/control/cmd_vel_safety_gate/status": ("avg_msgs/msg/ModuleState", "state"),
    "/control/camping_site_maneuver_controller/status": ("avg_msgs/msg/ModuleState", "state"),
    "/localization/pose": ("avg_msgs/msg/AvgPoseStamped", "sample"),
    "/odom": ("nav_msgs/msg/Odometry", "sample"),
    "/carla/ego_vehicle/odometry": ("nav_msgs/msg/Odometry", "sample"),
    "/control/cmd_vel": ("avg_msgs/msg/AvgTwist", "sample"),
    "/control/drop_zone/exit_complete": ("std_msgs/msg/Bool", "command"),
    "/ui/selected_destination": ("avg_msgs/msg/UiDestinationCommand", "command"),
    "/planning/state_machine/camping_site_recall": ("avg_msgs/msg/PlanningRecallRequest", "command"),
    "/ui/camping_site_operation_request": ("avg_msgs/msg/MotionOperation", "command"),
    "/control/drop_zone_maneuver_controller/operation": ("avg_msgs/msg/MotionOperation", "command"),
    "/parking/operation": ("avg_msgs/msg/MotionOperation", "command"),
    "/ui/destination_dispatch_status": ("std_msgs/msg/String", "command"),
    "/ui/mission_recording/events": ("std_msgs/msg/String", "command"),
}


def utc():
    return datetime.now(timezone.utc).isoformat()


def bounded(value):
    """HH_261102 - Retain invalid numbers as null and bound diagnostic payloads on disk."""
    if isinstance(value, dict):
        return {str(key): bounded(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [bounded(item) for item in value[:64]]
    if isinstance(value, str):
        return value[:4096]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def compact_message(topic, value):
    if topic == "/platform/status":
        keys = ("stamp", "header", "battery_percentage", "battery_state_available",
                "is_charging", "battery_current_a", "control_mode", "estop",
                "error_code", "vehicle_state", "velocity", "motion_mode")
        value = {key: value[key] for key in keys if key in value}
    elif topic.endswith("odometry") or topic == "/odom":
        value = {"header": value.get("header"),
                 "pose": value.get("pose", {}).get("pose"),
                 "twist": value.get("twist", {}).get("twist")}
    return bounded(value)


def api_summary(path, value):
    # HH_261102 - Geometry/images are irrelevant to this recorder and would dominate evidence size.
    if path == "/api/driving":
        keys = ("schema_version", "connected", "mission", "motion", "battery", "pose", "progress")
        value = {key: value[key] for key in keys if key in value}
    return bounded(value)


def source_stamp(value):
    stamp = value.get("stamp") or value.get("header", {}).get("stamp")
    if isinstance(stamp, dict) and "sec" in stamp and "nanosec" in stamp:
        return {"sec": stamp["sec"], "nanosec": stamp["nanosec"]}
    return None


class EvidenceFull(Exception):
    pass


class JsonlWriter:
    def __init__(self, path, maximum_bytes):
        # HH_261102 - Never overwrite old evidence or silently rotate away the beginning of a test.
        self.stream = Path(path).open("x", encoding="utf-8")
        self.maximum_bytes = maximum_bytes
        self.bytes_written = 0
        self.rows = 0

    def write(self, row):
        line = json.dumps(bounded(row), ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n"
        size = len(line.encode("utf-8"))
        if self.bytes_written + size > self.maximum_bytes:
            raise EvidenceFull("evidence size cap reached; no existing row was removed")
        self.stream.write(line)
        self.stream.flush()
        self.bytes_written += size
        self.rows += 1


def fetch_api(base_url, path):
    started = time.monotonic()
    try:
        request = Request(base_url + path, headers={"Cache-Control": "no-cache"}, method="GET")
        with urlopen(request, timeout=0.4) as response:
            raw = response.read(1048577)
        if len(raw) > 1048576:
            raise ValueError("API response exceeds 1 MiB")
        value = json.loads(raw)
        if not isinstance(value, dict):
            raise ValueError("API response is not an object")
        return {"ok": True, "data": api_summary(path, value),
                "received_monotonic_s": time.monotonic(), "request_started_monotonic_s": started}
    except Exception as error:
        return {"ok": False, "error": str(error)[:512], "received_monotonic_s": time.monotonic()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ui-url", default="http://127.0.0.1:8010")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--max-mib", type=int, default=64)
    args = parser.parse_args()
    url = urlsplit(args.ui_url)
    if (url.scheme != "http" or url.hostname not in {"127.0.0.1", "localhost", "::1"}
            or url.username or url.password or url.query or url.fragment or url.path not in {"", "/"}):
        parser.error("--ui-url must be a local HTTP origin")
    if not math.isfinite(args.duration) or not 0 <= args.duration <= 60 or not 1 <= args.max_mib <= 1024:
        parser.error("duration must be 0 (SIGINT) or <=60 seconds; max-mib must be 1..1024")

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message

    writer = JsonlWriter(args.output, args.max_mib * 1024 * 1024)
    latest, counts, phases = {}, {}, {}
    stopped = False
    reason = "duration_complete"

    def stop(_number, _frame):
        nonlocal stopped
        stopped = True

    def receive(topic, mode, message):
        now = time.monotonic()
        value = compact_message(topic, message_to_ordereddict(message))
        counts[topic] = counts.get(topic, 0) + 1
        row = {"received_monotonic_s": now, "source_stamp": source_stamp(value), "data": value}
        latest[topic] = row
        fingerprint = (value.get("operating_state"), value.get("level")) if "operating_state" in value else {
            key: item for key, item in value.items() if key not in {"stamp", "header"}}
        if mode == "command" or (mode == "state" and phases.get(topic) != fingerprint):
            writer.write({"kind": "ros_event", "utc": utc(), "topic": topic, **row})
            phases[topic] = fingerprint

    rclpy.init(args=[])
    node = rclpy.create_node("carla_battery_evidence_observer")
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        subscriptions = [node.create_subscription(get_message(type_name), topic,
            lambda message, name=topic, kind=mode: receive(name, kind, message), qos)
            for topic, (type_name, mode) in TOPICS.items()]
        started = next_snapshot = time.monotonic()
        writer.write({"kind": "start", "utc": utc(), "monotonic_s": started,
                      "scope": "passive CARLA observation; no inferred PASS or physical charger proof",
                      "ros_domain_id": __import__("os").environ.get("ROS_DOMAIN_ID"),
                      "topics": TOPICS, "duration_s": args.duration, "maximum_bytes": writer.maximum_bytes})
        while rclpy.ok() and not stopped:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if args.duration and now - started >= args.duration:
                break
            if now >= next_snapshot:
                apis = {path: fetch_api(args.ui_url.rstrip("/"), path)
                        for path in ("/api/charging-access", "/api/driving")}
                now = time.monotonic()
                writer.write({"kind": "snapshot", "utc": utc(), "monotonic_s": now,
                              "topics": {topic: {**row, "receipt_age_s": now - row["received_monotonic_s"]}
                                         for topic, row in latest.items()},
                              "counts": counts, "api": apis})
                next_snapshot = now + 1.0
        if stopped:
            reason = "signal"
        writer.write({"kind": "end", "utc": utc(), "reason": reason, "counts": counts})
    except EvidenceFull:
        reason = "size_cap_reached"
    except Exception:
        # HH_261102 - A recorder error must not be reported as a completed observation window.
        reason = "observer_error"
        raise
    finally:
        writer.stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(json.dumps({"output": str(args.output), "reason": reason,
                          "rows": writer.rows, "bytes": writer.bytes_written, "counts": counts}))


if __name__ == "__main__":
    main()
