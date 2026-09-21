"""HH_260915 - Browser-independent, observation-only mission recording node.

ROS callbacks only normalize/enqueue observations. One bounded worker owns the
journal and its files. There are no driving publishers, service calls, interface
changes, CAN transmissions, or automatic deletion of existing recordings.
"""

from __future__ import annotations

import copy
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import queue
import socket
import threading
import time

from .raw_can_capture import RawCanCapture


def default_mission_records_root() -> Path:
    state = os.environ.get("XDG_STATE_HOME", "").strip()
    return (Path(state).expanduser() if state else Path.home() / ".local" / "state") / "camrod" / "mission_records"


def recording_environment(requested, use_sim_time):
    requested = str(requested).strip().lower()
    effective = requested if requested in {"real", "simulation", "test"} else "unknown"
    reason = "explicit launch/default setting; hardware provenance not independently certified"
    if use_sim_time and effective == "real":
        effective, reason = "simulation", "ROS use_sim_time=true overrides requested real label"
    return {"requested_environment": requested, "effective_environment": effective,
            "environment_reason": reason, "use_sim_time": bool(use_sim_time)}


def _finite(value):
    try:
        result = float(value)
        return result if math.isfinite(result) and not isinstance(value, bool) else None
    except (TypeError, ValueError, OverflowError):
        return None


def _integer(value):
    value = _finite(value)
    return int(value) if value is not None and value.is_integer() else None


def _stamp(stamp):
    seconds = _integer(getattr(stamp, "sec", None))
    nanos = _integer(getattr(stamp, "nanosec", None))
    if seconds is None or nanos is None or seconds < 0 or not 0 <= nanos < 1_000_000_000:
        return {"sec": seconds, "nanosec": nanos, "seconds": None}
    value = seconds + nanos * 1e-9
    return {"sec": seconds, "nanosec": nanos, "seconds": value if value > 0 else None}


def normalize_platform(message, *, received_unix_s, received_ros_s,
                       received_monotonic_s, stale_timeout_s=1.0) -> dict:
    """Keep driver-decoded provenance and source stamps, not CAN freshness claims."""
    velocity_stamp = _stamp(message.velocity.header.stamp)
    header_stamp = _stamp(message.header.stamp)
    aggregate_stamp = _stamp(message.stamp)
    stamp_s, basis = velocity_stamp["seconds"], "velocity.header.stamp"
    if stamp_s is None:
        stamp_s, basis = header_stamp["seconds"], "header.stamp_fallback"
    if stamp_s is None:
        stamp_s, basis = _finite(received_ros_s), "receipt_ros_time_fallback"
    vx, vy = _finite(message.velocity.twist.linear.x), _finite(message.velocity.twist.linear.y)
    age = received_ros_s - stamp_s if stamp_s is not None else None
    valid = vx is not None and vy is not None and stamp_s is not None and stamp_s >= 0
    fresh = valid and age is not None and -0.25 <= age <= stale_timeout_s
    battery_available = bool(message.battery_state_available)
    return {
        "sample_time_s": stamp_s, "received_unix_s": received_unix_s,
        "received_ros_s": received_ros_s, "received_monotonic_s": received_monotonic_s,
        "vx": vx, "vy": vy, "yaw_rate_radps": _finite(message.velocity.twist.angular.z),
        "control_mode": _integer(message.control_mode), "estop": bool(message.estop),
        "error_code": _integer(message.error_code), "vehicle_state": _integer(message.vehicle_state),
        "motion_mode": _integer(message.motion_mode), "is_charging": bool(message.is_charging),
        "battery_state_available": battery_available,
        "battery_percentage": _finite(message.battery_percentage) if battery_available else None,
        "source_quality": {
            "source": "canonical_decoded_can_cached_driver", "valid": valid, "fresh": fresh,
            "can_frame_freshness": "not_observable_from_republished_platform_status",
            "timestamp_basis": basis, "source_stamp_age_s": age,
        },
        "decoded": {
            "header_stamp": header_stamp, "aggregate_stamp": aggregate_stamp,
            "velocity_stamp": velocity_stamp, "odometry_stamp": _stamp(message.odometry.header.stamp),
            "wheel_stamp": _stamp(message.wheel.header.stamp),
            "frame_id": str(message.header.frame_id), "velocity_frame_id": str(message.velocity.header.frame_id),
            "battery_voltage": _finite(message.battery_voltage),
            "battery_current_a": _finite(message.battery_current_a),
            "battery_temperature_c": _finite(message.battery_temperature_c),
            "battery_power_supply_status": _integer(message.battery_power_supply_status),
            "motor_rpm": [_finite(value) for value in message.motor_rpm],
            "motor_speed": [_finite(value) for value in message.motor_speed],
            "motor_angle": [_finite(value) for value in message.motor_angle],
            "state": {"stamp": _stamp(message.state.stamp), "module_name": str(message.state.module_name),
                      "level": _integer(message.state.level), "operating_state": str(message.state.operating_state),
                      "message": str(message.state.message)},
        },
    }


def normalize_gate(message, *, received_unix_s, received_ros_s, received_monotonic_s):
    return {
        "stamp": _stamp(message.stamp), "received_unix_s": received_unix_s,
        "received_ros_s": received_ros_s, "received_monotonic_s": received_monotonic_s,
        "module_name": str(message.module_name), "level": _integer(message.level),
        "operating_state": str(message.operating_state), "message": str(message.message),
        "missing_nodes": list(message.missing_nodes), "missing_topics": list(message.missing_topics),
        "missing_lifecycle_nodes": list(message.missing_lifecycle_nodes),
        "source_quality": "decoded_gate_status; mission correlation belongs to accepted event stream",
    }


class RecorderWorker:
    """One journal owner; bounded callbacks cannot turn disk delay into ROS delay."""

    def __init__(self, journal_factory, *, queue_capacity=2048, stale_timeout_s=1.0,
                 monotonic_fn=time.monotonic, now_fn=time.time):
        if queue_capacity < 1 or not math.isfinite(stale_timeout_s) or stale_timeout_s <= 0:
            raise ValueError("queue capacity and stale timeout must be positive")
        self._factory, self._monotonic, self._now = journal_factory, monotonic_fn, now_fn
        self._queue = queue.Queue(maxsize=queue_capacity)
        self._lock = threading.Lock()
        self._stop, self.ready = threading.Event(), threading.Event()
        self._thread = None
        self._journal = None
        self._dropped = 0
        self._reported_dropped = 0
        self._last_platform_received = None
        self._last_platform_valid = False
        self._ever_platform = False
        self._mission_seen = False
        self._stale_timeout = stale_timeout_s
        self._stale_reported = False
        self._last_error = None
        self._cached = {"recorder": {"status": "starting", "platform_stale": True}}

    def start(self):
        self._thread = threading.Thread(target=self._run, name="camrod-mission-journal", daemon=True)
        self._thread.start()

    def submit(self, kind, payload, *, received_unix=None):
        envelope = (kind, payload, self._now() if received_unix is None else received_unix)
        try:
            self._queue.put_nowait(envelope)
            return True
        except queue.Full:
            with self._lock:
                self._dropped += 1
            return False

    def note_platform_receipt(self, received_monotonic_s, *, valid):
        with self._lock:
            self._last_platform_received = received_monotonic_s
            self._last_platform_valid = valid
            self._ever_platform = True

    def cached_snapshot(self):
        with self._lock:
            snapshot = copy.deepcopy(self._cached)
            snapshot.setdefault("recorder", {})["queue_dropped_total"] = self._dropped
            if self._dropped:
                snapshot["recorder"]["capture_incomplete"] = True
            return snapshot

    def _process(self, kind, payload, received):
        if kind == "event":
            self._journal.observe_event(payload, received_unix=received)
            self._mission_seen = self._journal.snapshot().get("current_mission") is not None
        elif kind == "sample":
            if self._monotonic() - payload["received_monotonic_s"] > self._stale_timeout:
                payload = copy.deepcopy(payload)
                payload["source_quality"]["fresh"] = False
                payload["source_quality"]["queue_delay_stale"] = True
            self._journal.observe_sample(payload, received_unix=received)
        elif kind == "gate":
            self._journal.observe_gate(payload, received_unix=received)
        elif kind == "raw":
            self._journal.observe_raw_frame(payload, received_unix=received)
        elif kind == "raw_status":
            self._journal.set_raw_can_status(payload)
            if payload.get("state") in {"not_available", "error"} or payload.get("error"):
                self._journal.mark_incomplete("raw_can_" + str(payload.get("state")) + ":" + str(payload.get("error") or "unavailable"))
        elif kind == "gap":
            self._journal.mark_incomplete(str(payload))
        elif kind == "metadata":
            self._journal.set_recorder_status(payload)

    def _health(self):
        with self._lock:
            dropped, last = self._dropped, self._last_platform_received
            valid = self._last_platform_valid
        stale = last is None or not valid or self._monotonic() - last > self._stale_timeout
        if dropped > self._reported_dropped:
            self._journal.mark_incomplete("recorder_queue_overflow")
            self._reported_dropped = dropped
        if stale and not self._stale_reported and (self._ever_platform or self._mission_seen):
            self._journal.mark_incomplete("platform_status_stale_or_unavailable")
            self._stale_reported = True
        elif not stale:
            self._stale_reported = False
        return {"platform_stale": stale, "queue_dropped_total": dropped,
                "queue_size": self._queue.qsize(), "last_worker_error": self._last_error,
                "source_quality": "canonical_decoded_can_cached_driver; CAN frame freshness not observable"}

    def _publish_snapshot(self):
        self._journal.set_recorder_status(self._health())
        self._journal.flush_snapshot()
        with self._lock:
            self._cached = self._journal.snapshot()

    def _run(self):
        try:
            self._journal = self._factory()
        except Exception as exc:
            self._last_error = str(exc)
            with self._lock:
                self._cached = {"generated_at": datetime.now(timezone.utc).isoformat(),
                                "recorder": {"status": "error", "last_worker_error": str(exc),
                                             "capture_incomplete": True}}
            self.ready.set()
            return
        self.ready.set()
        next_snapshot = self._monotonic()
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    envelope = self._queue.get(timeout=0.1)
                except queue.Empty:
                    envelope = None
                try:
                    # Reset the integrator before processing any item after a
                    # known gap, including a dropped event/authority transition.
                    self._health()
                    if envelope is not None:
                        self._process(*envelope)
                    if self._monotonic() >= next_snapshot:
                        self._publish_snapshot()
                        next_snapshot = self._monotonic() + 1.0
                except Exception as exc:
                    self._last_error = str(exc)
                    with self._lock:
                        self._cached.setdefault("recorder", {}).update({
                            "last_worker_error": str(exc), "capture_incomplete": True, "status": "error"})
                    try:
                        self._journal.mark_incomplete("recorder_worker_error:" + str(exc))
                    except Exception:
                        pass
                finally:
                    if envelope is not None:
                        self._queue.task_done()
        finally:
            try:
                self._publish_snapshot()
                self._journal.close()
            except Exception as exc:
                with self._lock:
                    self._cached.setdefault("recorder", {}).update({"status": "error", "last_worker_error": str(exc)})

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            return not self._thread.is_alive()
        return True


def create_ros_node():
    # Lazy imports keep normalization/SocketCAN tests ROS- and hardware-free.
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from avg_msgs.msg import AvgPlatformStatus, ModuleState
    from std_msgs.msg import String
    from .mission_journal import MissionJournal

    class MissionRecorderNode(Node):
        def __init__(self):
            super().__init__("mission_recorder")
            value = lambda name, default: self.declare_parameter(name, default).value
            root = str(value("storage_root", str(default_mission_records_root())))
            robot_id = str(value("robot_id", socket.gethostname()))
            environment_info = recording_environment(str(value("environment", "real")),
                self.get_parameter("use_sim_time").value if self.has_parameter("use_sim_time") else False)
            environment = environment_info["effective_environment"]
            platform_topic = str(value("platform_status_topic", "/platform/status"))
            gate_topic = str(value("gate_status_topic", "/control/cmd_vel_safety_gate/status"))
            event_topic = str(value("event_topic", "/ui/mission_recording/events"))
            self._platform_topic, self._gate_topic = platform_topic, gate_topic
            interface = str(value("raw_can_interface", ""))
            self._stale_timeout = float(value("platform_status_timeout_s", 1.0))
            config = {
                "minimum_speed_mps": float(value("minimum_speed_mps", 0.03)),
                "maximum_speed_mps": float(value("maximum_speed_mps", 3.0)),
                "maximum_sample_gap_s": float(value("maximum_sample_gap_s", 2.0)),
                "rotation_bytes": int(value("rotation_bytes", 4 * 1024 * 1024)),
                "quota_bytes": int(value("quota_bytes", 256 * 1024 * 1024)),
            }
            capacity = int(value("queue_capacity", 2048))
            self._worker = RecorderWorker(
                lambda: MissionJournal(root, robot_id=robot_id, environment=environment, **config),
                queue_capacity=capacity, stale_timeout_s=self._stale_timeout)
            self._worker.start()
            self._worker.submit("metadata", environment_info)
            platform_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                                      durability=DurabilityPolicy.VOLATILE)
            gate_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            event_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._status_pub = self.create_publisher(String, "/ui/mission_recording/status", gate_qos)
            self._platform_sub = self.create_subscription(AvgPlatformStatus, platform_topic, self._platform, platform_qos)
            self._gate_sub = self.create_subscription(ModuleState, gate_topic, self._gate, gate_qos)
            self._event_sub = self.create_subscription(String, event_topic, self._event, event_qos)
            self._timer = self.create_timer(1.0, self._publish_status)
            self._raw = None
            try:
                self._raw = RawCanCapture(interface,
                    lambda frame: self._worker.submit("raw", frame, received_unix=frame["received_unix_ns"] * 1e-9),
                    lambda status: self._worker.submit("raw_status", status))
                self._raw.start()
            except Exception as exc:
                self._worker.submit("raw_status", {"state": "error", "configured": bool(interface),
                                                     "interface": interface, "error": str(exc)})
            self.get_logger().info(f"Observation-only mission recorder: root={root}, environment={environment}, raw_can={interface or 'disabled'}")
            if environment == "unknown" or environment != environment_info["requested_environment"]:
                self.get_logger().warning("Recording provenance: " + json.dumps(environment_info))

        def _times(self):
            return {"received_unix_s": time.time(), "received_ros_s": self.get_clock().now().nanoseconds * 1e-9,
                    "received_monotonic_s": time.monotonic()}

        def _platform(self, message):
            times = self._times()
            sample = normalize_platform(message, **times, stale_timeout_s=self._stale_timeout)
            sample["source_topic"] = self._platform_topic
            self._worker.note_platform_receipt(times["received_monotonic_s"], valid=sample["source_quality"]["fresh"])
            self._worker.submit("sample", sample, received_unix=times["received_unix_s"])

        def _gate(self, message):
            times = self._times()
            gate = normalize_gate(message, **times)
            gate["source"] = gate["source_topic"] = self._gate_topic
            self._worker.submit("gate", gate, received_unix=times["received_unix_s"])

        def _event(self, message):
            try:
                if len(message.data.encode("utf-8")) > 65536:
                    raise ValueError("mission event exceeds 64 KiB")
                event = json.loads(message.data, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
                if not isinstance(event, dict):
                    raise ValueError("mission event must be an object")
                self._worker.submit("event", event)
            except (ValueError, TypeError) as exc:
                self._worker.submit("gap", "invalid_mission_event:" + str(exc))

        def _publish_status(self):
            message = String()
            message.data = json.dumps(self._worker.cached_snapshot(), ensure_ascii=False, allow_nan=False)
            self._status_pub.publish(message)

        def destroy_node(self):
            if self._raw is not None:
                self._raw.stop()
            if not self._worker.close():
                self.get_logger().error("Mission recorder shutdown timed out; queued capture is incomplete")
            return super().destroy_node()

    return MissionRecorderNode()


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = None
    try:
        node = create_ros_node()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
