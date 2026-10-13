#!/usr/bin/env python3
"""HH_261006 - Evidence-only CARLA mileage; never commands motion or opens a DB.

Run with the CARLA ROS environment sourced. --output must be a new directory.
The counter starts at zero; --target-m is informational, never a robot stop.
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


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def classification(metadata=None, now=None, maximum_age_s=2.0):
    """HH_261006 - Only positive, fresh UI authority evidence identifies manual."""
    now = time.monotonic() if now is None else now
    metadata = metadata or {}
    received = metadata.get("received_monotonic_s")
    fresh = finite(received) and 0 <= now - received <= maximum_age_s
    result = {key: str(metadata.get(key) or "unknown")[:160] if fresh else "unknown"
              for key in ("mission_id", "intent", "phase", "leg")}
    result["metadata_fresh"] = fresh
    result["mode_metadata_fresh"] = fresh
    # HH_261006 - Keep stale observations with freshness, never as authority.
    reason = metadata.get("manual_resume_reason")
    reason = reason[:160] if isinstance(reason, str) else "unknown"
    pending = metadata.get("manual_resume_pending")
    result["manual_resume_reason"] = reason
    result["manual_resume_pending"] = pending if type(pending) is bool else None
    manual = fresh and reason == "manual_drive_active"
    result["mode"] = "manual" if manual else "unknown"
    result["mode_basis"] = ("ui.state.manual_resume.reason=manual_drive_active" if manual
                            else "no_positive_authority_evidence" if fresh
                            else "stale_or_absent_ui_metadata")
    # HH_261006 - CAN is actuator enable, not the owner of motion commands.
    platform_mode = metadata.get("platform_control_mode")
    result["platform_control_mode"] = platform_mode if type(platform_mode) is int else None
    source = metadata.get("platform_mode_source")
    result["platform_mode_source"] = source[:160] if isinstance(source, str) else "unknown"
    stamp = metadata.get("platform_source_stamp_s")
    result["platform_source_stamp_s"] = stamp if finite(stamp) else None
    platform_received = metadata.get("platform_mode_received_monotonic_s")
    result["platform_mode_metadata_fresh"] = (
        finite(platform_received) and 0 <= now - platform_received <= maximum_age_s
        and metadata.get("platform_mode_source_fresh") is True)
    if result["leg"] not in {"delivery", "recall", "return"}:
        result["leg"] = "unknown"
    return result


def platform_metadata(control_mode, source_stamp_s, received, wall_now):
    """HH_261006 - Preserve raw platform provenance without inferring authority."""
    return {"platform_control_mode": control_mode if type(control_mode) is int else None,
            "platform_mode_source": "/platform/status.control_mode",
            "platform_source_stamp_s": source_stamp_s if finite(source_stamp_s) else None,
            "platform_mode_received_monotonic_s": received,
            "platform_mode_source_fresh": finite(source_stamp_s)
            and -0.25 <= wall_now - source_stamp_s <= 2.0}


class MileageAccumulator:
    """Pure accumulator: original simulation stamps and planar sensor samples."""

    def __init__(self, target_m=10000.0, maximum_gap_s=2.0, maximum_jump_m=2.0,
                 minimum_speed_mps=0.03, microjitter_m=0.0001):
        self.target_m = target_m
        self.maximum_gap_s = maximum_gap_s
        self.maximum_jump_m = maximum_jump_m
        self.minimum_speed_mps = minimum_speed_mps
        self.microjitter_m = microjitter_m
        self.previous = None
        self.samples = 0
        self.anchor_samples = 0
        self.accepted_intervals = 0
        self.xy_distance_m = 0.0
        self.velocity_integral_m = 0.0
        self.covered_sim_s = 0.0
        self.excluded_sim_s = 0.0
        self.exclusions = Counter()
        self.stationary_intervals = 0
        self.by_leg = Counter()
        self.by_mode = Counter()

    def observe(self, stamp_s, x, y, vx, vy, metadata=None, received_monotonic_s=None):
        self.samples += 1
        label = classification(metadata, received_monotonic_s)
        row = {"sample": self.samples, "classification": label,
               "xy_added_m": 0.0, "velocity_added_m": 0.0}
        values = (stamp_s, x, y, vx, vy)
        row.update(dict(zip(("sim_stamp_s", "x_m", "y_m", "vx_mps", "vy_mps"),
                            (v if finite(v) else None for v in values))))
        if not all(finite(v) for v in values):
            self.exclusions["nonfinite"] += 1
            self.previous = None
            return {**row, "status": "excluded", "reason": "nonfinite"}
        speed = math.hypot(vx, vy)
        if not math.isfinite(speed):
            self.exclusions["nonfinite_speed"] += 1
            self.previous = None
            return {**row, "status": "excluded", "reason": "nonfinite_speed"}
        current = (stamp_s, x, y, speed, label)
        previous = self.previous
        self.previous = current
        if previous is None:
            self.anchor_samples += 1
            return {**row, "status": "anchor"}
        old_stamp, old_x, old_y, old_speed, old_label = previous
        dt = stamp_s - old_stamp
        step = math.hypot(x - old_x, y - old_y)
        reason = ("nonpositive_dt" if dt <= 0 else "sample_gap" if dt > self.maximum_gap_s
                  else "position_jump" if step > self.maximum_jump_m else None)
        if reason:
            self.exclusions[reason] += 1
            self.excluded_sim_s += max(0.0, dt)
            # HH_261006 - No rejected sample is used to bridge a later interval.
            self.previous = None
            return {**row, "status": "excluded", "reason": reason,
                    "interval_s": dt, "displacement_m": step}
        v0 = old_speed if old_speed >= self.minimum_speed_mps else 0.0
        v1 = speed if speed >= self.minimum_speed_mps else 0.0
        integral = 0.5 * (v0 + v1) * dt
        distance = step if (v0 or v1) and step >= self.microjitter_m else 0.0
        self.stationary_intervals += int(not v0 and not v1)
        self.accepted_intervals += 1
        self.covered_sim_s += dt
        self.xy_distance_m += distance
        self.velocity_integral_m += integral
        # HH_261006 - A transition interval has no proven exact categorical split.
        for key, counter in (("leg", self.by_leg), ("mode", self.by_mode)):
            bucket = label[key] if label[key] == old_label[key] else "unknown"
            counter[bucket] += distance
        return {**row, "status": "accepted", "interval_s": dt,
                "xy_added_m": distance, "velocity_added_m": integral}

    def snapshot(self):
        total_known_s = self.covered_sim_s + self.excluded_sim_s
        return {"environment": "simulation", "purpose": "evidence_only",
                "measurement_source": "/carla/ego_vehicle/odometry",
                "distance_basis": "raw_sensor_planar_position_displacements",
                "clock_basis": "original_odometry_simulation_header",
                "session_baseline_m": 0.0, "target_m": self.target_m,
                "target_reached": self.xy_distance_m >= self.target_m,
                "xy_distance_m": self.xy_distance_m,
                "velocity_integral_m": self.velocity_integral_m,
                "xy_minus_velocity_m": self.xy_distance_m - self.velocity_integral_m,
                "samples": self.samples, "anchor_samples": self.anchor_samples,
                "accepted_intervals": self.accepted_intervals,
                "stationary_intervals": self.stationary_intervals,
                "covered_sim_s": self.covered_sim_s, "excluded_known_sim_s": self.excluded_sim_s,
                "known_time_coverage_fraction": self.covered_sim_s / total_known_s if total_known_s else None,
                "coverage_note": "Partial lower-bound travel; invalid samples and reset anchors leave unmeasured intervals. No gap extrapolation.",
                "excluded_intervals": dict(self.exclusions),
                "distance_by_leg_m": dict(self.by_leg), "distance_by_mode_m": dict(self.by_mode)}


def compact_metadata(driving, state, received):
    """HH_261006 - Whitelist scalar fields; never archive debug/map geometry."""
    mission = driving.get("mission") or {}
    if not isinstance(mission, dict):
        mission = {}
    intent = state.get("mission_dispatch_intent") or mission.get("intent")
    phase = state.get("mission_phase") or mission.get("phase") or state.get("service_state_name")
    leg = state.get("leg_kind") or mission.get("leg_kind")
    active = state.get("mission_dispatch_active") is True or mission.get("active") is True
    service = state.get("service_state")
    # HH_261006 - RETURN_WITH_CARGO(9) is ambiguous without explicit leg evidence.
    if not leg and active:
        if service in {3, 10}:
            leg = "return"
        elif service == 7:
            leg = "recall"
        elif service == 5:
            leg = "recall" if intent == "recall" else "delivery" if intent == "delivery" else None
        elif service in {1, 14, 15}:
            leg = intent if intent in {"delivery", "recall"} else None
    # HH_261006 - Neither CAN nor engaged UI AUTO proves autonomous ownership.
    resume = state.get("manual_resume")
    resume = resume if isinstance(resume, dict) else {}
    reason, pending = resume.get("reason"), resume.get("pending")
    identity = state.get("mission_id") or mission.get("mission_id")
    generation = state.get("mission_dispatch_generation") or mission.get("generation")
    if not identity and active and type(generation) is int and generation > 0:
        # This is a session-local UI identity, not the journal's opaque UUID.
        identity = "ui-generation:" + str(generation)
    return {"received_monotonic_s": received,
            "mission_id": identity or "unknown",
            "intent": intent, "phase": phase, "leg": leg,
            "manual_resume_reason": reason[:160] if isinstance(reason, str) else "unknown",
            "manual_resume_pending": pending if type(pending) is bool else None}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--target-m", type=float, default=10000.0)
    parser.add_argument("--duration-s", type=float, default=0.0, help="0 records until interrupted")
    parser.add_argument("--ui-url", default="", help="Optional UI base URL, e.g. http://127.0.0.1:8010")
    args = parser.parse_args(argv)
    if not finite(args.target_m) or args.target_m <= 0 or not finite(args.duration_s) or args.duration_s < 0:
        parser.error("target must be positive; duration must be nonnegative and finite")
    # HH_261006 - Imports stay here so the distance tests require no ROS install.
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry

    args.output.mkdir(parents=True, exist_ok=False)
    stream = (args.output / "samples.jsonl").open("x", encoding="utf-8")
    accumulator = MileageAccumulator(target_m=args.target_m)
    lock = threading.Lock()
    stopping = threading.Event()
    metadata = {}
    mode_metadata = {}
    http_status = {"enabled": bool(args.ui_url), "error": None}
    started = time.monotonic()
    started_utc = datetime.now(timezone.utc).isoformat()

    def fetch(path):
        with urlopen(args.ui_url.rstrip("/") + path, timeout=0.4) as response:
            raw = response.read(2 * 1024 * 1024 + 1)
        if len(raw) > 2 * 1024 * 1024:
            raise ValueError("metadata response exceeds 2 MiB")
        result = json.loads(raw)
        if not isinstance(result, dict):
            raise ValueError("metadata response is not an object")
        return result

    def poll_metadata():
        while not stopping.is_set():
            try:
                driving, state = fetch("/api/driving"), fetch("/ui/state")
                small = compact_metadata(driving, state, time.monotonic())
                with lock:
                    metadata.clear()
                    metadata.update(small)
                    http_status["error"] = None
            except Exception as exc:
                with lock:
                    metadata.clear()
                    http_status["error"] = str(exc)[:300]
            stopping.wait(0.5)

    def save_summary(status):
        with lock:
            stream.flush()
            result = {**accumulator.snapshot(), "status": status,
                      "started_utc": started_utc,
                      "updated_utc": datetime.now(timezone.utc).isoformat(),
                      "elapsed_wall_s": time.monotonic() - started,
                      "metadata_http": dict(http_status),
                      "robot_stop_requested": False}
        temporary = args.output / "summary.json.tmp"
        with temporary.open("w", encoding="utf-8") as output:
            json.dump(result, output, allow_nan=False, indent=2)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, args.output / "summary.json")

    rclpy.init()
    node = rclpy.create_node("simulated_mileage_evidence_only")

    def sample(message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        pose, velocity = message.pose.pose.position, message.twist.twist.linear
        received = time.monotonic()
        with lock:
            row = accumulator.observe(stamp, pose.x, pose.y, velocity.x, velocity.y,
                                      {**metadata, **mode_metadata}, received)
            row["elapsed_wall_s"] = received - started
            stream.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")
            stream.flush()

    node.create_subscription(Odometry, "/carla/ego_vehicle/odometry", sample,
                             QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        from avg_msgs.msg import AvgPlatformStatus
    except ImportError:
        AvgPlatformStatus = None
    if AvgPlatformStatus is not None:
        def platform_mode(message):
            # HH_261006 - Raw CAN metadata must never overwrite UI authority.
            source = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            with lock:
                mode_metadata.update(platform_metadata(message.control_mode, source,
                                                       time.monotonic(), time.time()))
        node.create_subscription(AvgPlatformStatus, "/platform/status", platform_mode,
                                 QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT))
    worker = threading.Thread(target=poll_metadata, daemon=True) if args.ui_url else None
    if worker:
        worker.start()
    status = "closed"
    try:
        save_summary("recording")
        last_save = time.monotonic()
        while rclpy.ok() and (not args.duration_s or time.monotonic() - started < args.duration_s):
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - last_save >= 0.5:
                save_summary("recording")
                last_save = time.monotonic()
    except KeyboardInterrupt:
        status = "interrupted"
    except Exception:
        status = "error"
        raise
    finally:
        stopping.set()
        if worker:
            worker.join(timeout=1.0)
        save_summary(status)
        stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
