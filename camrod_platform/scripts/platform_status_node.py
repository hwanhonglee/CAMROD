#!/usr/bin/env python3
# HH_260310-00:00 Module-level status_stream publisher for platform outputs.

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from avg_msgs.msg import AvgPlatformMsgs, ModuleState
from status_msgs.msg import StatusArray, StatusStatus, KeyValue
from geometry_msgs.msg import PolygonStamped, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray


# Implements `_diag_level` behavior.
def _diag_level(value: object) -> bytes:
    # HH_260311-00:00 Humble StatusStatus constants/values may be bytes.
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


class PlatformStatusNode(Node):
    # Implements `__init__` behavior.
    def __init__(self) -> None:
        super().__init__("platform_status")

        self.status_topic = str(
            # HH_260311-00:00 Single consolidated ROS status stream.
            self.declare_parameter("status_topic", "status").value
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
            self.declare_parameter(
                "topic_localization_pose", "/localization/pose"
            ).value
        )
        self.topic_localization_pose_cov = str(
            self.declare_parameter(
                "topic_localization_pose_cov", "/localization/pose_with_covariance"
            ).value
        )

        self._diag_pub = self.create_publisher(StatusArray, self.status_topic, 10)

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
            PoseStamped,
            self.topic_localization_pose,
            self._on_localization_pose,
            default_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.topic_localization_pose_cov,
            self._on_localization_pose_cov,
            default_qos,
        )

        self.create_timer(self.publish_period_s, self._on_timer)
        self.get_logger().info(
            "platform_status: status=%s period=%.2fs"
            % (self.status_topic, self.publish_period_s)
        )

    # Implements `_on_robot_markers` behavior.
    def _on_robot_markers(self, msg: MarkerArray) -> None:
        self._msg_robot_markers = msg
        self._state["robot_markers"].stamp = self.get_clock().now()

    # Implements `_on_planning_boundary` behavior.
    def _on_planning_boundary(self, msg: PolygonStamped) -> None:
        self._msg_planning_boundary = msg
        self._state["planning_boundary"].stamp = self.get_clock().now()

    # Implements `_on_localization_pose` behavior.
    def _on_localization_pose(self, msg: PoseStamped) -> None:
        self._msg_localization_pose = msg

    # Implements `_on_localization_pose_cov` behavior.
    def _on_localization_pose_cov(self, msg: PoseWithCovarianceStamped) -> None:
        self._msg_localization_pose_cov = msg

    # Implements `_state` behavior.
    def _state(self, now: Time) -> ModuleState:
        out = ModuleState()
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
            out.level = ModuleState.ERROR
            out.message = "missing/stale platform streams"
            out.missing_topics = missing
        else:
            out.level = ModuleState.OK
            out.message = "platform streams statey"
        return out

    # Implements `_on_timer` behavior.
    def _on_timer(self) -> None:
        now = self.get_clock().now()
        msg = AvgPlatformMsgs()
        msg.stamp = now.to_msg()
        msg.state = self._state(now)
        msg.robot_markers = self._msg_robot_markers
        msg.planning_boundary = self._msg_planning_boundary
        msg.localization_pose = self._msg_localization_pose
        msg.localization_pose_cov = self._msg_localization_pose_cov
        diag = StatusArray()
        diag.header.stamp = msg.stamp
        st = StatusStatus()
        st.name = "platform/status"
        st.hardware_id = "platform"
        st.level = _diag_level(msg.state.level)
        st.message = msg.state.message
        st.values.append(KeyValue(key="category", value="platform"))
        st.values.append(KeyValue(key="missing_topics", value=",".join(msg.state.missing_topics)))
        st.values.append(KeyValue(key="missing_nodes", value=",".join(msg.state.missing_nodes)))
        st.values.append(
            KeyValue(
                key="missing_lifecycle_nodes",
                value=",".join(msg.state.missing_lifecycle_nodes),
            )
        )
        diag.status.append(st)
        self._diag_pub.publish(diag)


# Entry point for this executable.
def main() -> None:
    rclpy.init()
    node = PlatformStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        node.get_logger().error(f"platform_status runtime exception: {e}")
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
