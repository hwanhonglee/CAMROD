#!/usr/bin/env python3
# HH_260310-00:00 Module-level diagnostics publisher for perception outputs.

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from avg_msgs.msg import AvgPerceptionMsgs, ModuleHealth
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray


# Implements `_diag_level` behavior.
def _diag_level(value: object) -> bytes:
    # HH_260311-00:00 Humble DiagnosticStatus constants/values may be bytes.
    if isinstance(value, (bytes, bytearray)):
        if len(value) == 1:
            return bytes(value)
        if len(value) > 1:
            return bytes([value[0]])
        return b"\x00"
    return bytes([int(value) & 0xFF])


@dataclass
class StampState:
    stamp: Optional[Time] = None


class PerceptionDiagnosticNode(Node):
    # Implements `__init__` behavior.
    def __init__(self) -> None:
        super().__init__("perception_diagnostic")

        self.diagnostic_topic = str(
            # HH_260311-00:00 Single consolidated ROS diagnostic stream.
            self.declare_parameter("diagnostic_topic", "diagnostic").value
        )
        self.publish_period_s = float(self.declare_parameter("publish_period_s", 0.2).value)
        self.stale_timeout_s = float(self.declare_parameter("stale_timeout_s", 2.0).value)

        self.topic_obstacles = str(
            self.declare_parameter("topic_obstacles", "/perception/obstacles").value
        )
        self.topic_detections = str(
            self.declare_parameter("topic_detections", "/perception/camera/detections_2d").value
        )
        self.topic_camera_info = str(
            self.declare_parameter(
                "topic_camera_info", "/sensing/camera/processed/camera_info"
            ).value
        )

        self._diag_pub = self.create_publisher(DiagnosticArray, self.diagnostic_topic, 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._msg_obstacles = PointCloud2()
        self._msg_camera_info = CameraInfo()
        self._msg_detections = Detection2DArray()
        self._state: Dict[str, StampState] = {"obstacles": StampState()}

        self.create_subscription(PointCloud2, self.topic_obstacles, self._on_obstacles, sensor_qos)
        self.create_subscription(
            Detection2DArray, self.topic_detections, self._on_detections, sensor_qos
        )
        self.create_subscription(
            CameraInfo, self.topic_camera_info, self._on_camera_info, sensor_qos
        )

        self.create_timer(self.publish_period_s, self._on_timer)
        self.get_logger().info(
            "perception_diagnostic: diagnostic=%s period=%.2fs"
            % (self.diagnostic_topic, self.publish_period_s)
        )

    # Implements `_on_obstacles` behavior.
    def _on_obstacles(self, msg: PointCloud2) -> None:
        self._msg_obstacles = msg
        self._state["obstacles"].stamp = self.get_clock().now()

    # Implements `_on_detections` behavior.
    def _on_detections(self, msg: Detection2DArray) -> None:
        self._msg_detections = msg

    # Implements `_on_camera_info` behavior.
    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._msg_camera_info = msg

    # Implements `_health` behavior.
    def _health(self, now: Time) -> ModuleHealth:
        out = ModuleHealth()
        out.stamp = now.to_msg()
        out.module_name = "perception"
        st = self._state["obstacles"].stamp
        if st is None:
            out.level = ModuleHealth.ERROR
            out.message = "missing obstacles topic"
            out.missing_topics = ["obstacles"]
            return out
        age = (now - st).nanoseconds * 1e-9
        if age > self.stale_timeout_s:
            out.level = ModuleHealth.ERROR
            out.message = f"stale obstacles topic ({age:.2f}s)"
            out.missing_topics = [f"obstacles(stale:{age:.2f}s)"]
            return out
        out.level = ModuleHealth.OK
        out.message = "perception stream healthy"
        return out

    # Implements `_on_timer` behavior.
    def _on_timer(self) -> None:
        now = self.get_clock().now()
        msg = AvgPerceptionMsgs()
        msg.stamp = now.to_msg()
        msg.health = self._health(now)
        msg.obstacles = self._msg_obstacles
        msg.camera_info = self._msg_camera_info
        msg.detections = self._msg_detections
        diag = DiagnosticArray()
        diag.header.stamp = msg.stamp
        st = DiagnosticStatus()
        st.name = "perception/diagnostic"
        st.hardware_id = "perception"
        st.level = _diag_level(msg.health.level)
        st.message = msg.health.message
        st.values.append(KeyValue(key="category", value="perception"))
        st.values.append(KeyValue(key="missing_topics", value=",".join(msg.health.missing_topics)))
        st.values.append(KeyValue(key="missing_nodes", value=",".join(msg.health.missing_nodes)))
        st.values.append(
            KeyValue(
                key="missing_lifecycle_nodes",
                value=",".join(msg.health.missing_lifecycle_nodes),
            )
        )
        diag.status.append(st)
        self._diag_pub.publish(diag)


# Entry point for this executable.
def main() -> None:
    rclpy.init()
    node = PerceptionDiagnosticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        node.get_logger().error(f"perception_diagnostic runtime exception: {e}")
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
