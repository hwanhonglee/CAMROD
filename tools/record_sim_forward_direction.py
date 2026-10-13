#!/usr/bin/env python3
"""Passive CARLA direction evidence; never classifies a road/parking PASS.

HH_261007 - Position differences projected onto measured quaternion heading,
not command signs. Output JSONL and its .summary.json sidecar must both be new.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import time


SCHEMA = "camrod.sim_forward_direction.v1"


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def clean(value):
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    return None if isinstance(value, float) and not math.isfinite(value) else value


class DirectionAccumulator:
    """Pure 3D body-forward projection; rejected intervals never bridge gaps."""

    def __init__(self):
        self.previous = None
        self.samples = self.accepted = 0
        self.forward_m = self.reverse_m = self.path_m = self.covered_s = 0.0
        self.exclusions = Counter()

    def observe(self, stamp_ns, xyz, xyzw, frame_id, child_frame_id):
        self.samples += 1
        row = {"stamp_ns": stamp_ns, "xyz_m": list(xyz), "quaternion_xyzw": list(xyzw),
               "frame_id": frame_id, "child_frame_id": child_frame_id}

        def reject(reason):
            self.previous = None
            self.exclusions[reason] += 1
            return clean({**row, "valid": False, "reason": reason})

        if (type(stamp_ns) is not int or stamp_ns < 0 or len(xyz) != 3 or len(xyzw) != 4
                or not all(finite(v) for v in (*xyz, *xyzw))
                or not frame_id or not child_frame_id):
            return reject("invalid_raw_pose_or_stamp")
        norm = math.sqrt(sum(v * v for v in xyzw))
        if not 0.99 <= norm <= 1.01:
            return reject("invalid_quaternion_norm")
        x, y, z, w = (v / norm for v in xyzw)
        forward = (1 - 2 * (y*y + z*z), 2 * (x*y + w*z), 2 * (x*z - w*y))
        current = (stamp_ns, tuple(xyz), forward, frame_id, child_frame_id)
        previous = self.previous
        self.previous = current
        if previous is None:
            return clean({**row, "valid": False, "reason": "anchor"})
        old_stamp, old_xyz, old_forward, old_frame, old_child = previous
        dt = (stamp_ns - old_stamp) / 1e9
        delta = tuple(a - b for a, b in zip(xyz, old_xyz))
        step = math.sqrt(sum(v*v for v in delta))
        row.update(interval_s=dt, displacement_xyz_m=delta, displacement_3d_m=step)
        if frame_id != old_frame or child_frame_id != old_child:
            return reject("frame_changed")
        if dt <= 0:
            return reject("nonpositive_dt")
        if dt > 0.25:
            return reject("sample_gap")
        # HH_261007 - Evidence validity bound only, not a vehicle control limit.
        if step > 3.0 * dt + 0.02:
            return reject("implausible_position_jump")
        midpoint = tuple(a + b for a, b in zip(forward, old_forward))
        length = math.sqrt(sum(v*v for v in midpoint))
        if length < 1e-8:
            return reject("ambiguous_opposite_headings")
        signed = sum(d * f / length for d, f in zip(delta, midpoint))
        self.accepted += 1
        self.covered_s += dt
        self.path_m += step
        self.forward_m += max(0.0, signed)
        self.reverse_m += max(0.0, -signed)
        return clean({**row, "valid": True, "reason": "accepted",
                      "signed_forward_displacement_m": signed,
                      "mean_signed_forward_speed_mps": signed / dt})

    def summary(self):
        return {"samples": self.samples, "accepted_intervals": self.accepted,
                "forward_projected_m": self.forward_m, "reverse_projected_m": self.reverse_m,
                "net_signed_forward_m": self.forward_m - self.reverse_m,
                "accepted_3d_path_m": self.path_m, "covered_sim_s": self.covered_s,
                "exclusions": dict(self.exclusions), "road_parking_classification": "not_performed"}


class ContextCache:
    def __init__(self):
        self.latest = {}

    def update(self, name, value, received_s, received_ns):
        self.latest[name] = (clean(value), received_s, received_ns)

    def snapshot(self, now):
        return {name: {"value": value, "receipt_age_s": now - received,
                       "received_unix_ns": wall,
                       "receipt_fresh_within_1s": 0 <= now - received <= 1.0}
                for name, (value, received, wall) in self.latest.items()}


def stamp_ns(message):
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    return None if stamp is None else int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def vector(value):
    return [float(getattr(value, axis)) for axis in ("x", "y", "z")]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration-s", type=float, default=900.0)
    parser.add_argument("--max-bytes", type=int, default=128 * 1024**2)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration_s) or not 0 < args.duration_s <= 900:
        parser.error("duration must be finite and in (0,900] seconds")
    if not 65536 <= args.max_bytes <= 256 * 1024**2:
        parser.error("max-bytes must be between 65536 and 268435456")
    sidecar = args.output.with_suffix(args.output.suffix + ".summary.json")
    if args.output.exists() or sidecar.exists():
        parser.error("output and summary must both be new")

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from nav_msgs.msg import Odometry
    from avg_msgs.msg import AvgTwist, AvgServiceState, ModuleState
    from std_msgs.msg import String

    args.output.parent.mkdir(parents=True, exist_ok=True)
    accumulator, context = DirectionAccumulator(), ContextCache()
    stopped = [False]
    reason = ["duration"]
    written = [0]
    signal.signal(signal.SIGINT, lambda *_: stopped.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: stopped.__setitem__(0, True))
    node = None
    started = time.monotonic()
    status, error = "STOPPED", None
    with args.output.open("x", encoding="utf-8") as stream, sidecar.open("x", encoding="utf-8") as summary_stream:
        def write(record, *, terminal=False):
            text = json.dumps(clean(record), allow_nan=False, separators=(",", ":")) + "\n"
            size = len(text.encode("utf-8"))
            if written[0] + size > args.max_bytes - (0 if terminal else 4096):
                stopped[0], reason[0] = True, "byte_limit"
                return False
            stream.write(text)
            stream.flush()
            written[0] += size
            return True

        def receive_context(name, value):
            context.update(name, value, time.monotonic(), time.time_ns())

        def odometry(message):
            if stopped[0]:
                return
            now, wall = time.monotonic(), time.time_ns()
            pose = message.pose.pose
            q = pose.orientation
            # HH_261007 - A record refused by the disk cap must not enter totals.
            before = dict(accumulator.__dict__)
            before["exclusions"] = accumulator.exclusions.copy()
            row = accumulator.observe(stamp_ns(message), vector(pose.position),
                                      [q.x, q.y, q.z, q.w], message.header.frame_id,
                                      message.child_frame_id)
            recorded = write({"record_type": "sample", "sequence": accumulator.samples,
                   "received_unix_ns": wall,
                   "received_utc": datetime.fromtimestamp(wall / 1e9, timezone.utc).isoformat(),
                   "elapsed_wall_s": now - started, "odometry": row,
                   "raw_twist_linear": vector(message.twist.twist.linear),
                   "raw_twist_angular": vector(message.twist.twist.angular),
                   "context": context.snapshot(now)})
            if not recorded:
                accumulator.__dict__.update(before)

        topics = {"service": "/service/state", "command": "/control/cmd_vel",
                  "site": "/control/camping_site_maneuver_controller/status",
                  "drop_zone": "/control/drop_zone_maneuver_controller/status",
                  "parking": "/parking/status", "reverse_parking": "/parking/private/reverse/status",
                  "apriltag_parking": "/parking/private/apriltag/status",
                  "controller_selector": "/planning/controller_selector_ros"}
        write({"record_type": "header", "schema": SCHEMA, "environment": "simulation",
               "read_only": True, "purpose": "direction_evidence_not_pass",
               "odometry_topic": "/carla/ego_vehicle/odometry", "context_topics": topics,
               "duration_limit_s": args.duration_s, "byte_limit": args.max_bytes,
               "maximum_gap_s": 0.25, "jump_bound": "3.0*dt+0.02 metres (3D)",
               "projection": "position delta dot normalized sum of endpoint body+x quaternion axes",
               "context_policy": "receipt ages only; asynchronous, not proof of road/manual/parking authority"})
        try:
            # HH_261007 - Our handlers request a graceful evidence footer. Do
            # not let rclpy replace them and shut the context down mid-spin.
            # An unrelated context shutdown still follows the ERROR path.
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
            node = rclpy.create_node("sim_forward_direction_" + str(os.getpid()),
                                     enable_rosout=False, start_parameter_services=False)
            qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
            node.create_subscription(Odometry, "/carla/ego_vehicle/odometry", odometry, qos)
            node.create_subscription(AvgTwist, topics["command"], lambda msg: receive_context(
                "command", {"linear": vector(msg.linear), "angular": vector(msg.angular),
                            "source_stamp_ns": stamp_ns(msg)}), qos)
            node.create_subscription(AvgServiceState, topics["service"], lambda msg: receive_context(
                "service", {"state": int(msg.state), "state_name": str(msg.state_name)[:256],
                            "description": str(msg.description)[:512], "source_stamp_ns": stamp_ns(msg)}), qos)
            for name in ("site", "drop_zone", "parking", "reverse_parking", "apriltag_parking"):
                node.create_subscription(ModuleState, topics[name], lambda msg, name=name: receive_context(
                    name, {"level": int(msg.level), "operating_state": str(msg.operating_state)[:256],
                           "message": str(msg.message)[:512], "source_stamp_ns": stamp_ns(msg)}), qos)
            selector_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(String, topics["controller_selector"], lambda msg: receive_context(
                "controller_selector", {"data": str(msg.data)[:256], "source_stamp_ns": None,
                                        "may_be_retained": True}), selector_qos)
            while not stopped[0] and time.monotonic() - started < args.duration_s:
                rclpy.spin_once(node, timeout_sec=0.05)
            if stopped[0] and reason[0] != "byte_limit":
                reason[0] = "signal"
        except BaseException as exc:
            status, error, reason[0] = "ERROR", f"{type(exc).__name__}: {exc}"[:2048], "exception"
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            summary = {"record_type": "footer", "schema": SCHEMA, "status": status,
                       "stop_reason": reason[0], "error": error, "scenario_pass": None,
                       "elapsed_wall_s": time.monotonic() - started, **accumulator.summary()}
            write(summary, terminal=True)
            summary_stream.write(json.dumps(summary, allow_nan=False, indent=2) + "\n")
    print(json.dumps(summary), flush=True)
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
