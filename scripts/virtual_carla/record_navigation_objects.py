#!/usr/bin/env python3
"""HH_261002 - Passively correlate live ROS object geometry with navigation UI.

This collector never publishes, calls a ROS service, or controls CARLA. It only
subscribes to measurement/status topics and performs GET /api/driving. Samples
are real received data; absence and transport errors remain explicit evidence.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import threading
import time
from urllib.request import urlopen

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from avg_msgs.msg import AvgPlatformStatus, AvgServiceState, ModuleState
from visualization_msgs.msg import Marker, MarkerArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=600.0)
    parser.add_argument("--ui-url", default="http://127.0.0.1:8010/api/driving")
    parser.add_argument("--attempt-label", default="unspecified")
    args = parser.parse_args()
    if not 1 <= args.duration <= 1800:
        parser.error("duration must be 1..1800 seconds")
    args.output.mkdir(parents=True, exist_ok=True)
    prefix = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    jsonl_path = args.output / f"passive_objects_{prefix}.jsonl"
    summary_path = args.output / f"passive_objects_{prefix}_summary.json"
    stream = jsonl_path.open("x", encoding="utf-8")
    rclpy.init()
    node = rclpy.create_node("navigation_objects_passive_evidence")
    qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
    lock = threading.Lock()
    stop = threading.Event()
    started = time.monotonic()
    counts = Counter()
    classes = Counter()
    latest = {}

    def record(kind, payload):
        row = {"wall_utc": datetime.now(timezone.utc).isoformat(),
               "elapsed_s": round(time.monotonic() - started, 3),
               "kind": kind, "data": payload}
        with lock:
            counts[kind] += 1
            latest[kind] = payload
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
            stream.flush()

    def marker_record(message, kind):
        # Keep source stamps, exact IDs, labels, centers and metre scales; do
        # not infer dimensions or replace missing extents with fixed glyphs.
        values = [message_to_ordereddict(marker) for marker in message.markers[:128]]
        added = [marker for marker in message.markers[:128] if marker.action == Marker.ADD]
        with lock:
            counts[kind + "_adds"] += len(added)
            if kind == "fusion_markers":
                for marker in added:
                    if marker.type == Marker.TEXT_VIEW_FACING:
                        classes[marker.text.split("\n")[0]] += 1
        record(kind, values)

    node.create_subscription(MarkerArray, "/perception/camera_lidar/navigation_boxes",
                             lambda msg: marker_record(msg, "navigation_boxes"), qos)
    node.create_subscription(MarkerArray, "/perception/camera_lidar/markers",
                             lambda msg: marker_record(msg, "fusion_markers"), qos)
    node.create_subscription(ModuleState, "/control/cmd_vel_safety_gate/status",
                             lambda msg: record("safety_gate", message_to_ordereddict(msg)), qos)
    node.create_subscription(AvgServiceState, "/service/state",
                             lambda msg: record("service_state", message_to_ordereddict(msg)), qos)

    def platform(message):
        velocity = message.velocity.twist
        record("platform", {"velocity": message_to_ordereddict(message.velocity),
                            "speed_mps": math.hypot(velocity.linear.x, velocity.linear.y),
                            "control_mode": message.control_mode,
                            "is_charging": message.is_charging})

    node.create_subscription(AvgPlatformStatus, "/platform/status", platform, qos)

    def poll_ui():
        while not stop.is_set():
            try:
                with urlopen(args.ui_url, timeout=1.0) as response:
                    snapshot = json.load(response)
                route = snapshot.get("route", {})
                points = route.get("points", [])
                perception = snapshot.get("perception", {})
                objects = perception.get("objects", [])
                with lock:
                    counts["ui_objects"] += len(objects)
                    counts["ui_observed_boxes"] += sum(
                        obj.get("geometry_source") == "observed_lidar_extent" for obj in objects)
                record("ui_driving", {
                    key: snapshot.get(key) for key in
                    ("connected", "mission", "motion", "pose", "progress")
                } | {"route": {key: value for key, value in route.items() if key != "points"}
                              | {"point_count": len(points), "endpoints": points[:1] + points[-1:]},
                     "perception": {key: value for key, value in perception.items() if key != "points"}})
            except Exception as exc:
                record("ui_error", {"error": str(exc)})
            stop.wait(0.5)

    def summary():
        with lock:
            return {"schema": "camrod.navigation_objects.passive.v1",
                    "attempt_label": args.attempt_label,
                    "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
                    "ros_localhost_only": os.environ.get("ROS_LOCALHOST_ONLY"),
                    "ui_url": args.ui_url, "duration_s": round(time.monotonic() - started, 3),
                    "jsonl": str(jsonl_path), "counts": dict(counts),
                    "fusion_label_counts": dict(classes),
                    "latest_gate": latest.get("safety_gate"),
                    "latest_platform": latest.get("platform"),
                    "latest_ui": latest.get("ui_driving"),
                    "latest_ui_error": latest.get("ui_error")}

    thread = threading.Thread(target=poll_ui, daemon=True)
    thread.start()
    print(f"Recording passive evidence: {jsonl_path}", flush=True)
    next_report = 0.0
    try:
        while time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            elapsed = time.monotonic() - started
            if elapsed >= next_report:
                status = summary()
                summary_path.write_text(json.dumps(status, indent=2, ensure_ascii=False), encoding="utf-8")
                print(json.dumps({"elapsed_s": status["duration_s"], "counts": status["counts"],
                                  "classes": status["fusion_label_counts"]}), flush=True)
                next_report = elapsed + 10.0
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        thread.join(timeout=2)
        summary_path.write_text(json.dumps(summary(), indent=2, ensure_ascii=False), encoding="utf-8")
        node.destroy_node()
        # HH_261002 - SIGINT may already close rclpy's context; still finish the
        # evidence file normally instead of failing on a second shutdown call.
        if rclpy.ok():
            rclpy.shutdown()
        stream.close()
        print(f"Saved {summary_path}", flush=True)


if __name__ == "__main__":
    main()
