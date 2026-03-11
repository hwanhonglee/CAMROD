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


@dataclass
class StampState:
    stamp: Optional[Time] = None


class PerceptionDiagnosticNode(Node):
    def __init__(self) -> None:
        super().__init__("perception_diagnostic")

        self.msgs_topic = str(self.declare_parameter("msgs_topic", "/perception/messages").value)
        self.diagnostic_topic = str(
            self.declare_parameter("diagnostic_topic", "/perception/diagnostic").value
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
            self.declare_parameter("topic_camera_info", "/sensing/camera/processed/camera_info").value
        )

        self._msgs_pub = self.create_publisher(AvgPerceptionMsgs, self.msgs_topic, 10)
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
        self.create_subscription(CameraInfo, self.topic_camera_info, self._on_camera_info, sensor_qos)

        self.create_timer(self.publish_period_s, self._on_timer)
        self.get_logger().info(
            "perception_diagnostic: messages=%s diagnostic=%s period=%.2fs"
            % (self.msgs_topic, self.diagnostic_topic, self.publish_period_s)
        )

    def _on_obstacles(self, msg: PointCloud2) -> None:
        self._msg_obstacles = msg
        self._state["obstacles"].stamp = self.get_clock().now()

    def _on_detections(self, msg: Detection2DArray) -> None:
        self._msg_detections = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._msg_camera_info = msg

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

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        msg = AvgPerceptionMsgs()
        msg.stamp = now.to_msg()
        msg.health = self._health(now)
        msg.obstacles = self._msg_obstacles
        msg.camera_info = self._msg_camera_info
        msg.detections = self._msg_detections
        self._msgs_pub.publish(msg)

        diag = DiagnosticArray()
        diag.header.stamp = msg.stamp
        st = DiagnosticStatus()
        st.name = "perception/diagnostic"
        st.hardware_id = "perception"
        st.level = bytes([int(msg.health.level)])
        st.message = msg.health.message
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


def main() -> None:
    rclpy.init()
    node = PerceptionDiagnosticNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
