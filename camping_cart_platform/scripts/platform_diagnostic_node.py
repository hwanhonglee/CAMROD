#!/usr/bin/env python3
# HH_260310-00:00 Module-level diagnostics publisher for platform outputs.

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from avg_msgs.msg import AvgPlatformMsgs, ModuleHealth
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PolygonStamped, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray


@dataclass
class StampState:
    stamp: Optional[Time] = None


class PlatformDiagnosticNode(Node):
    def __init__(self) -> None:
        super().__init__("platform_diagnostic")

        self.msgs_topic = str(self.declare_parameter("msgs_topic", "/platform/messages").value)
        self.diagnostic_topic = str(
            self.declare_parameter("diagnostic_topic", "/platform/diagnostic").value
        )
        self.publish_period_s = float(self.declare_parameter("publish_period_s", 0.2).value)
        self.stale_timeout_s = float(self.declare_parameter("stale_timeout_s", 2.0).value)

        self.topic_robot_markers = str(
            self.declare_parameter("topic_robot_markers", "/platform/robot/markers").value
        )
        self.topic_planning_boundary = str(
            self.declare_parameter(
                "topic_planning_boundary", "/platform/robot/planning_boundary"
            ).value
        )
        self.topic_localization_pose = str(
            self.declare_parameter("topic_localization_pose", "/localization/pose").value
        )
        self.topic_localization_pose_cov = str(
            self.declare_parameter(
                "topic_localization_pose_cov", "/localization/pose_with_covariance"
            ).value
        )

        self._msgs_pub = self.create_publisher(AvgPlatformMsgs, self.msgs_topic, 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, self.diagnostic_topic, 10)

        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        default_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._msg_robot_markers = MarkerArray()
        self._msg_planning_boundary = PolygonStamped()
        self._msg_localization_pose = PoseStamped()
        self._msg_localization_pose_cov = PoseWithCovarianceStamped()
        self._state: Dict[str, StampState] = {
            "robot_markers": StampState(),
            "planning_boundary": StampState(),
        }

        self.create_subscription(
            MarkerArray, self.topic_robot_markers, self._on_robot_markers, marker_qos
        )
        self.create_subscription(
            PolygonStamped,
            self.topic_planning_boundary,
            self._on_planning_boundary,
            default_qos,
        )
        self.create_subscription(
            PoseStamped, self.topic_localization_pose, self._on_localization_pose, default_qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.topic_localization_pose_cov,
            self._on_localization_pose_cov,
            default_qos,
        )

        self.create_timer(self.publish_period_s, self._on_timer)
        self.get_logger().info(
            "platform_diagnostic: messages=%s diagnostic=%s period=%.2fs"
            % (self.msgs_topic, self.diagnostic_topic, self.publish_period_s)
        )

    def _on_robot_markers(self, msg: MarkerArray) -> None:
        self._msg_robot_markers = msg
        self._state["robot_markers"].stamp = self.get_clock().now()

    def _on_planning_boundary(self, msg: PolygonStamped) -> None:
        self._msg_planning_boundary = msg
        self._state["planning_boundary"].stamp = self.get_clock().now()

    def _on_localization_pose(self, msg: PoseStamped) -> None:
        self._msg_localization_pose = msg

    def _on_localization_pose_cov(self, msg: PoseWithCovarianceStamped) -> None:
        self._msg_localization_pose_cov = msg

    def _health(self, now: Time) -> ModuleHealth:
        out = ModuleHealth()
        out.stamp = now.to_msg()
        out.module_name = "platform"
        missing = []
        for key, st in self._state.items():
            if st.stamp is None:
                missing.append(key)
                continue
            age = (now - st.stamp).nanoseconds * 1e-9
            if age > self.stale_timeout_s:
                missing.append(f"{key}(stale:{age:.2f}s)")
        if missing:
            out.level = ModuleHealth.ERROR
            out.message = "missing/stale platform streams"
            out.missing_topics = missing
        else:
            out.level = ModuleHealth.OK
            out.message = "platform streams healthy"
        return out

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        msg = AvgPlatformMsgs()
        msg.stamp = now.to_msg()
        msg.health = self._health(now)
        msg.robot_markers = self._msg_robot_markers
        msg.planning_boundary = self._msg_planning_boundary
        msg.localization_pose = self._msg_localization_pose
        msg.localization_pose_cov = self._msg_localization_pose_cov
        self._msgs_pub.publish(msg)

        diag = DiagnosticArray()
        diag.header.stamp = msg.stamp
        st = DiagnosticStatus()
        st.name = "platform/diagnostic"
        st.hardware_id = "platform"
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
    node = PlatformDiagnosticNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
