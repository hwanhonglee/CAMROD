#!/usr/bin/env python3
"""Measure the production localization chain in one synchronized time window."""

import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import time

from avg_msgs.msg import AvgOdometry
from avg_msgs.msg import AvgPoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix


# HH_260730 - Measure every hand-off from the physical sensor boundary to the
# final selected pose in one process. Independent ``ros2 topic hz`` commands
# have different discovery/start windows and cannot localize intermittent lag.
DEFAULT_TOPIC_CHAIN = (
    (
        "gnss_fix",
        "/sensing/gnss/ublox_gps_node/fix",
        NavSatFix,
        "sensor_msgs/msg/NavSatFix",
    ),
    (
        "gnss_pose",
        "/sensing/gnss/pose_with_covariance_ros",
        PoseWithCovarianceStamped,
        "geometry_msgs/msg/PoseWithCovarianceStamped",
    ),
    (
        "imu",
        "/sensing/imu/data_ros",
        Imu,
        "sensor_msgs/msg/Imu",
    ),
    (
        "platform_odom",
        "/platform/status/odometry",
        AvgOdometry,
        "avg_msgs/msg/AvgOdometry",
    ),
    (
        "wheel_odom",
        "/localization/input/wheel_odometry_ros",
        Odometry,
        "nav_msgs/msg/Odometry",
    ),
    (
        "ekf_odom",
        "/localization/primary/odometry_ros",
        Odometry,
        "nav_msgs/msg/Odometry",
    ),
    (
        "adapter_pose",
        "/localization/primary/pose_with_covariance",
        AvgPoseWithCovarianceStamped,
        "avg_msgs/msg/AvgPoseWithCovarianceStamped",
    ),
    (
        "selected_pose",
        "/localization/pose_ros",
        PoseStamped,
        "geometry_msgs/msg/PoseStamped",
    ),
)

# HH_260731 - Match the production GNSS adapter's heading-validity ceiling.
# An orientation paired with the adapter's 1e6 rad^2 unavailable covariance is
# only a placeholder and must not be reported as a measured yaw error.
MAX_USABLE_YAW_VARIANCE_RAD2 = 100.0


def nearest_rank_percentile(values, quantile):
    """Return the nearest-rank percentile, or None for an empty sequence."""
    if not values:
        return None
    ordered = sorted(values)
    index = math.ceil(float(quantile) * len(ordered)) - 1
    index = max(0, min(len(ordered) - 1, index))
    return ordered[index]


def summarize_samples(receive_times_s, stamp_ages_ms):
    """Calculate rate, inter-arrival, and header-age statistics."""
    intervals_ms = [
        (current - previous) * 1000.0
        for previous, current in zip(receive_times_s, receive_times_s[1:])
    ]
    span_s = (
        receive_times_s[-1] - receive_times_s[0]
        if len(receive_times_s) > 1
        else 0.0
    )
    rate_hz = (
        (len(receive_times_s) - 1) / span_s
        if span_s > 0.0
        else None
    )

    return {
        "count": len(receive_times_s),
        "rate_hz": rate_hz,
        "interarrival_ms": {
            "p50": (
                statistics.median(intervals_ms)
                if intervals_ms
                else None
            ),
            "p95": nearest_rank_percentile(intervals_ms, 0.95),
            "max": max(intervals_ms) if intervals_ms else None,
        },
        "header_age_ms": {
            "p50": (
                statistics.median(stamp_ages_ms)
                if stamp_ages_ms
                else None
            ),
            "p95": nearest_rank_percentile(stamp_ages_ms, 0.95),
            "max": max(stamp_ages_ms) if stamp_ages_ms else None,
        },
    }


def normalize_angle(angle_rad):
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def summarize_scalar(values):
    """Return compact scalar statistics while preserving missing data."""
    return {
        "p50": statistics.median(values) if values else None,
        "p95": nearest_rank_percentile(values, 0.95),
        "max": max(values) if values else None,
    }


def summarize_motion_samples(pose_samples, twist_samples):
    """Summarize displacement, unwrapped yaw, and body velocity evidence."""
    yaw_pose_samples = [
        item for item in pose_samples
        if item.get("yaw_valid", True)
    ]
    output = {
        "pose_count": len(pose_samples),
        "yaw_pose_count": len(yaw_pose_samples),
        "twist_count": len(twist_samples),
        "position_displacement_m": None,
        "yaw_change_deg": None,
        "yaw_span_deg": None,
        "linear_x_abs_mps": summarize_scalar([]),
        "linear_y_abs_mps": summarize_scalar([]),
        "angular_z_abs_radps": summarize_scalar([]),
        "crab_sample_count": 0,
    }

    if pose_samples:
        first = pose_samples[0]
        last = pose_samples[-1]
        output["position_displacement_m"] = math.hypot(
            last["x"] - first["x"],
            last["y"] - first["y"],
        )
    if yaw_pose_samples:
        first = yaw_pose_samples[0]
        unwrapped = [first["yaw"]]
        for previous, current in zip(
            yaw_pose_samples,
            yaw_pose_samples[1:],
        ):
            unwrapped.append(
                unwrapped[-1]
                + normalize_angle(current["yaw"] - previous["yaw"])
            )
        output["yaw_change_deg"] = math.degrees(
            unwrapped[-1] - unwrapped[0]
        )
        output["yaw_span_deg"] = math.degrees(
            max(unwrapped) - min(unwrapped)
        )

    if twist_samples:
        linear_x_abs = [abs(item["linear_x"]) for item in twist_samples]
        linear_y_abs = [abs(item["linear_y"]) for item in twist_samples]
        angular_z_abs = [abs(item["angular_z"]) for item in twist_samples]
        output["linear_x_abs_mps"] = summarize_scalar(linear_x_abs)
        output["linear_y_abs_mps"] = summarize_scalar(linear_y_abs)
        output["angular_z_abs_radps"] = summarize_scalar(angular_z_abs)
        output["crab_sample_count"] = sum(
            abs(item["linear_y"]) >= 0.05
            and abs(item["linear_y"]) > abs(item["linear_x"])
            for item in twist_samples
        )
    return output


def compare_pose_samples(reference, target, max_stamp_delta_s=0.25):
    """Pair pose streams by header stamp and summarize XY/yaw differences."""
    position_errors = []
    yaw_errors_deg = []
    stamp_deltas_ms = []
    if not reference or not target:
        return {
            "pair_count": 0,
            "yaw_pair_count": 0,
            "max_stamp_delta_s": max_stamp_delta_s,
            "stamp_delta_ms": summarize_scalar([]),
            "position_error_m": summarize_scalar([]),
            "yaw_error_deg": summarize_scalar([]),
        }

    target_index = 0
    for reference_sample in reference:
        while (
            target_index + 1 < len(target)
            and abs(
                target[target_index + 1]["stamp"]
                - reference_sample["stamp"]
            )
            <= abs(
                target[target_index]["stamp"]
                - reference_sample["stamp"]
            )
        ):
            target_index += 1
        target_sample = target[target_index]
        stamp_delta_s = abs(
            target_sample["stamp"] - reference_sample["stamp"]
        )
        if stamp_delta_s > max_stamp_delta_s:
            continue
        stamp_deltas_ms.append(stamp_delta_s * 1000.0)
        position_errors.append(
            math.hypot(
                target_sample["x"] - reference_sample["x"],
                target_sample["y"] - reference_sample["y"],
            )
        )
        if (
            reference_sample.get("yaw_valid", True)
            and target_sample.get("yaw_valid", True)
        ):
            yaw_errors_deg.append(
                abs(
                    math.degrees(
                        normalize_angle(
                            target_sample["yaw"] - reference_sample["yaw"]
                        )
                    )
                )
            )

    return {
        "pair_count": len(position_errors),
        "yaw_pair_count": len(yaw_errors_deg),
        "max_stamp_delta_s": max_stamp_delta_s,
        "stamp_delta_ms": summarize_scalar(stamp_deltas_ms),
        "position_error_m": summarize_scalar(position_errors),
        "yaw_error_deg": summarize_scalar(yaw_errors_deg),
    }


def quaternion_yaw(quaternion):
    """Return planar yaw from a ROS quaternion, or None if it is invalid."""
    values = (
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-9:
        return None
    x, y, z, w = (value / norm for value in values)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def extract_pose_sample(
    message,
    stamp_s,
    max_yaw_variance_rad2=MAX_USABLE_YAW_VARIANCE_RAD2,
):
    """Extract a common pose sample from stamped pose or odometry messages."""
    pose_container = getattr(message, "pose", None)
    if pose_container is None:
        return None
    yaw_covariance = None
    yaw_valid = True
    covariance = getattr(pose_container, "covariance", None)
    if covariance is not None and len(covariance) > 35:
        yaw_covariance = float(covariance[35])
        yaw_valid = (
            math.isfinite(yaw_covariance)
            and yaw_covariance >= 0.0
            and yaw_covariance <= max_yaw_variance_rad2
        )

    pose = getattr(pose_container, "pose", pose_container)
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    if position is None or orientation is None:
        return None
    yaw = quaternion_yaw(orientation)
    if yaw is None or not all(
        math.isfinite(value) for value in (position.x, position.y)
    ):
        return None
    return {
        "stamp": stamp_s,
        "x": float(position.x),
        "y": float(position.y),
        "yaw": yaw,
        "yaw_covariance_rad2": yaw_covariance,
        "yaw_valid": yaw_valid,
    }


def extract_twist_sample(message, stamp_s):
    """Extract body velocity from odometry or IMU messages."""
    if hasattr(message, "angular_velocity"):
        angular_z = message.angular_velocity.z
        if not math.isfinite(angular_z):
            return None
        return {
            "stamp": stamp_s,
            "linear_x": 0.0,
            "linear_y": 0.0,
            "angular_z": float(angular_z),
        }

    twist = getattr(message, "twist", None)
    if twist is None:
        return None
    twist = getattr(twist, "twist", twist)
    linear = getattr(twist, "linear", None)
    angular = getattr(twist, "angular", None)
    if linear is None or angular is None:
        return None
    values = (linear.x, linear.y, angular.z)
    if not all(math.isfinite(value) for value in values):
        return None
    return {
        "stamp": stamp_s,
        "linear_x": float(linear.x),
        "linear_y": float(linear.y),
        "angular_z": float(angular.z),
    }


def _format_number(value, digits=1):
    if value is None or not math.isfinite(value):
        return "-"
    return f"{value:.{digits}f}"


def render_table(topic_results):
    """Render a compact human-readable summary table."""
    headers = (
        "stage",
        "count",
        "Hz",
        "gap p50/p95/max ms",
        "age p50/p95/max ms",
        "status",
    )
    rows = []
    for result in topic_results:
        gap = result["interarrival_ms"]
        age = result["header_age_ms"]
        rows.append(
            (
                result["name"],
                str(result["count"]),
                _format_number(result["rate_hz"], 2),
                "/".join(
                    _format_number(gap[key])
                    for key in ("p50", "p95", "max")
                ),
                "/".join(
                    _format_number(age[key])
                    for key in ("p50", "p95", "max")
                ),
                result["status"],
            )
        )

    widths = [
        max(len(headers[index]), *(len(row[index]) for row in rows))
        for index in range(len(headers))
    ]

    def format_row(row):
        return "  ".join(
            value.ljust(widths[index])
            for index, value in enumerate(row)
        )

    separator = "  ".join("-" * width for width in widths)
    return "\n".join(
        [format_row(headers), separator]
        + [format_row(row) for row in rows]
    )


def render_pose_comparisons(comparisons):
    """Render GNSS-to-filter XY/yaw comparisons."""
    lines = [
        "comparison              XY/yaw pairs  XY p50/p95/max m       "
        "yaw p50/p95/max deg",
        "----------------------  ------------  ---------------------  "
        "---------------------",
    ]
    for item in comparisons:
        position = item["position_error_m"]
        yaw = item["yaw_error_deg"]
        pair_counts = (
            f"{item['pair_count']}/{item['yaw_pair_count']}"
        )
        lines.append(
            f"{item['name'].ljust(22)}  "
            f"{pair_counts.rjust(12)}  "
            f"{'/'.join(_format_number(position[key], 3) for key in ('p50', 'p95', 'max')).ljust(21)}  "
            f"{'/'.join(_format_number(yaw[key], 2) for key in ('p50', 'p95', 'max'))}"
        )
    return "\n".join(lines)


class PoseLatencyProbe(Node):
    """Subscribe to the complete real localization chain concurrently."""

    def __init__(self):
        super().__init__("pose_latency_probe")
        self.samples = {}
        self._topic_subscriptions = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        for name, topic, message_type, expected_type in DEFAULT_TOPIC_CHAIN:
            self.samples[name] = {
                "name": name,
                "topic": topic,
                "expected_type": expected_type,
                "receive_times_s": [],
                "stamp_ages_ms": [],
                "header_error_count": 0,
                "header_errors": [],
                "pose_samples": [],
                "twist_samples": [],
            }
            self._topic_subscriptions.append(
                self.create_subscription(
                    message_type,
                    topic,
                    self._callback(name),
                    qos,
                )
            )

    def _record_header_error(self, sample, message):
        sample["header_error_count"] += 1
        if len(sample["header_errors"]) < 5:
            sample["header_errors"].append(message)

    def _callback(self, name):
        def receive(message):
            sample = self.samples[name]
            receive_time_s = time.monotonic()
            try:
                stamp = message.header.stamp
                stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
            except (AttributeError, TypeError, ValueError) as error:
                self._record_header_error(
                    sample,
                    f"unreadable header.stamp: {error}",
                )
                return

            if not math.isfinite(stamp_s) or stamp_s <= 0.0:
                self._record_header_error(
                    sample,
                    f"invalid header.stamp={stamp_s}",
                )
                return

            stamp_age_ms = (time.time() - stamp_s) * 1000.0
            if stamp_age_ms < -100.0:
                self._record_header_error(
                    sample,
                    f"header.stamp is {-stamp_age_ms:.1f} ms in the future",
                )
                return

            sample["receive_times_s"].append(receive_time_s)
            sample["stamp_ages_ms"].append(stamp_age_ms)
            pose_sample = extract_pose_sample(message, stamp_s)
            if pose_sample is not None:
                sample["pose_samples"].append(pose_sample)
            twist_sample = extract_twist_sample(message, stamp_s)
            if twist_sample is not None:
                sample["twist_samples"].append(twist_sample)

        return receive

    def discover_publishers(self, timeout_s):
        """Wait for publishers and return their graph-advertised types."""
        deadline = time.monotonic() + timeout_s
        graph_types = {}
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            graph_types = self._publisher_types()
            if all(graph_types[item[0]] for item in DEFAULT_TOPIC_CHAIN):
                break
        return graph_types

    def _publisher_types(self):
        graph_types = {}
        for name, topic, _, _ in DEFAULT_TOPIC_CHAIN:
            infos = self.get_publishers_info_by_topic(topic)
            graph_types[name] = sorted(
                {info.topic_type for info in infos if info.topic_type}
            )
        return graph_types

    def reset_samples(self):
        """Discard discovery traffic so every topic uses the same window."""
        for sample in self.samples.values():
            sample["receive_times_s"].clear()
            sample["stamp_ages_ms"].clear()
            sample["header_error_count"] = 0
            sample["header_errors"].clear()
            sample["pose_samples"].clear()
            sample["twist_samples"].clear()

    def measure(self, duration_s):
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < duration_s:
            rclpy.spin_once(self, timeout_sec=0.02)
        return time.monotonic() - start

    def build_report(self, graph_types, requested_s, measured_s):
        topic_results = []
        errors = []

        for name, topic, _, expected_type in DEFAULT_TOPIC_CHAIN:
            sample = self.samples[name]
            result = {
                "name": name,
                "topic": topic,
                "expected_type": expected_type,
                "publisher_types": graph_types.get(name, []),
            }
            result.update(
                summarize_samples(
                    sample["receive_times_s"],
                    sample["stamp_ages_ms"],
                )
            )
            result["header_error_count"] = sample["header_error_count"]
            result["header_errors"] = sample["header_errors"]
            result["motion"] = summarize_motion_samples(
                sample["pose_samples"],
                sample["twist_samples"],
            )

            topic_errors = []
            if not result["publisher_types"]:
                topic_errors.append("no publisher discovered")
            elif expected_type not in result["publisher_types"]:
                topic_errors.append(
                    "type mismatch: expected "
                    f"{expected_type}, graph has {result['publisher_types']}"
                )
            if result["count"] == 0:
                topic_errors.append("no valid messages received")
            elif result["count"] == 1:
                topic_errors.append("only one valid message; rate unavailable")
            if result["header_error_count"] > 0:
                topic_errors.append(
                    f"{result['header_error_count']} invalid message headers"
                )

            result["errors"] = topic_errors
            result["status"] = "ERROR" if topic_errors else "OK"
            errors.extend(f"{name}: {error}" for error in topic_errors)
            topic_results.append(result)

        pose_comparisons = []
        for target_name in ("ekf_odom", "adapter_pose", "selected_pose"):
            comparison = compare_pose_samples(
                self.samples["gnss_pose"]["pose_samples"],
                self.samples[target_name]["pose_samples"],
            )
            comparison["name"] = f"gnss->{target_name}"
            pose_comparisons.append(comparison)

        return {
            "status": "ERROR" if errors else "OK",
            "requested_duration_s": requested_s,
            "measured_duration_s": measured_s,
            "generated_at_unix_s": time.time(),
            "errors": errors,
            "topics": topic_results,
            # HH_260731 - The GNSS point is still the configured antenna point.
            # A constant XY error can therefore indicate an uncalibrated antenna
            # lever arm; changing EKF gain cannot correct that geometry.
            "pose_comparisons": pose_comparisons,
        }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Measure receive rate, inter-arrival jitter, and header age for "
            "the production localization topic chain."
        )
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        help="measurement window in seconds (default: 60)",
    )
    parser.add_argument(
        "--discovery-timeout",
        type=float,
        default=8.0,
        help="publisher discovery timeout in seconds (default: 8)",
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        help="write the full JSON report to this path",
    )
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 2.0:
        parser.error("--duration must be finite and at least 2 seconds")
    if (
        not math.isfinite(args.discovery_timeout)
        or args.discovery_timeout <= 0.0
    ):
        parser.error("--discovery-timeout must be finite and greater than zero")
    return args


def main(argv=None):
    args = parse_args(argv)
    rclpy.init()
    node = PoseLatencyProbe()
    try:
        graph_types = node.discover_publishers(args.discovery_timeout)
        node.reset_samples()
        measured_s = node.measure(args.duration)
        report = node.build_report(
            graph_types,
            args.duration,
            measured_s,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(render_table(report["topics"]))
    print("\n" + render_pose_comparisons(report["pose_comparisons"]))
    report_json = json.dumps(report, indent=2, ensure_ascii=False)
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(report_json + "\n", encoding="utf-8")
        print(f"\nJSON report: {args.output_json}")
    else:
        print("\n" + report_json)

    if report["errors"]:
        print("\nFAIL:", file=sys.stderr)
        for error in report["errors"]:
            print(f"  - {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
