#!/usr/bin/env python3
# HH_260312-00:00 Sensor-kit module diagnostic-only publisher.

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Set

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from avg_msgs.msg import AvgSensorKitMsgs, ModuleHealth
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_msgs.msg import TFMessage


# Implements `_diag_level` behavior.
def _diag_level(value: object) -> bytes:
    # HH_260312-00:00 Humble DiagnosticStatus level expects one-byte values.
    if isinstance(value, (bytes, bytearray)):
        if len(value) == 1:
            return bytes(value)
        if len(value) > 1:
            return bytes([value[0]])
        return b"\x00"
    return bytes([int(value) & 0xFF])


@dataclass
class TFState:
    stamp: Time | None = None
    frames: Set[str] = field(default_factory=set)


class SensorKitDiagnosticNode(Node):
    # Implements `__init__` behavior.
    def __init__(self) -> None:
        super().__init__("sensor_kit_diagnostic")

        self.diagnostic_topic = str(
            self.declare_parameter("diagnostic_topic", "diagnostic").value
        )
        self.publish_period_s = float(self.declare_parameter("publish_period_s", 0.5).value)
        self.stale_timeout_s = float(self.declare_parameter("stale_timeout_s", 3.0).value)

        self.base_frame_id = str(
            self.declare_parameter("base_frame_id", "robot_base_link").value
        )
        self.sensor_kit_base_frame_id = str(
            self.declare_parameter("sensor_kit_base_frame_id", "sensor_kit_base_link").value
        )

        self.topic_tf = str(self.declare_parameter("topic_tf", "/tf").value)
        self.topic_tf_static = str(self.declare_parameter("topic_tf_static", "/tf_static").value)

        self._diag_pub = self.create_publisher(DiagnosticArray, self.diagnostic_topic, 10)

        tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        tf_static_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._tf_state = TFState()
        self._tf_static_state = TFState()

        self.create_subscription(TFMessage, self.topic_tf, self._on_tf, tf_qos)
        self.create_subscription(TFMessage, self.topic_tf_static, self._on_tf_static, tf_static_qos)

        self.create_timer(self.publish_period_s, self._on_timer)
        self.get_logger().info(
            "sensor_kit_diagnostic: diagnostic=%s period=%.2fs"
            % (self.diagnostic_topic, self.publish_period_s)
        )

    # Implements `_on_tf` behavior.
    def _on_tf(self, msg: TFMessage) -> None:
        now = self.get_clock().now()
        self._tf_state.stamp = now
        for tf in msg.transforms:
            if tf.child_frame_id:
                self._tf_state.frames.add(tf.child_frame_id)

    # Implements `_on_tf_static` behavior.
    def _on_tf_static(self, msg: TFMessage) -> None:
        now = self.get_clock().now()
        self._tf_static_state.stamp = now
        for tf in msg.transforms:
            if tf.child_frame_id:
                self._tf_static_state.frames.add(tf.child_frame_id)

    # Implements `_is_fresh` behavior.
    def _is_fresh(self, now: Time, state: TFState) -> bool:
        if state.stamp is None:
            return False
        age = (now - state.stamp).nanoseconds * 1e-9
        return age <= self.stale_timeout_s

    # Implements `_health` behavior.
    def _health(self, now: Time) -> ModuleHealth:
        out = ModuleHealth()
        out.stamp = now.to_msg()
        out.module_name = "sensor_kit"

        tf_static_ready = self._is_fresh(now, self._tf_static_state)
        tf_ready = self._is_fresh(now, self._tf_state)

        if not tf_static_ready:
            out.level = ModuleHealth.ERROR
            out.message = "missing/stale tf_static"
            out.missing_topics = [self.topic_tf_static]
            return out

        if not tf_ready:
            out.level = ModuleHealth.WARN
            out.message = "tf dynamic stream is not ready yet"
            out.missing_topics = [self.topic_tf]
            return out

        out.level = ModuleHealth.OK
        out.message = "sensor_kit tf streams healthy"
        return out

    # Implements `_on_timer` behavior.
    def _on_timer(self) -> None:
        now = self.get_clock().now()
        msg = AvgSensorKitMsgs()
        msg.stamp = now.to_msg()
        msg.health = self._health(now)
        msg.base_frame_id = self.base_frame_id
        msg.sensor_kit_base_frame_id = self.sensor_kit_base_frame_id
        msg.tf_static_ready = self._is_fresh(now, self._tf_static_state)
        msg.tf_ready = self._is_fresh(now, self._tf_state)
        msg.static_child_frames = sorted(self._tf_static_state.frames)
        msg.dynamic_child_frames = sorted(self._tf_state.frames)
        diag = DiagnosticArray()
        diag.header.stamp = msg.stamp
        st = DiagnosticStatus()
        st.name = "sensor_kit/diagnostic"
        st.hardware_id = "sensor_kit"
        st.level = _diag_level(msg.health.level)
        st.message = msg.health.message
        st.values.append(KeyValue(key="category", value="sensor_kit"))
        st.values.append(KeyValue(key="missing_topics", value=",".join(msg.health.missing_topics)))
        st.values.append(KeyValue(key="tf_static_ready", value=str(msg.tf_static_ready).lower()))
        st.values.append(KeyValue(key="tf_ready", value=str(msg.tf_ready).lower()))
        st.values.append(
            KeyValue(key="static_child_frames", value=",".join(msg.static_child_frames))
        )
        st.values.append(
            KeyValue(key="dynamic_child_frames", value=",".join(msg.dynamic_child_frames))
        )
        diag.status.append(st)
        self._diag_pub.publish(diag)


# Entry point for this executable.
def main() -> None:
    rclpy.init()
    node = SensorKitDiagnosticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"sensor_kit_diagnostic runtime exception: {exc}")
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
