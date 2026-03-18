#!/usr/bin/env python3
# HH_260310-00:00 Module-level diagnostics publisher for planning outputs.

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from action_msgs.msg import GoalStatusArray
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from avg_msgs.msg import AvgPlanningMsgs, ModuleHealth
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import MarkerArray


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


class PlanningDiagnosticNode(Node):
    # Implements `__init__` behavior.
    def __init__(self) -> None:
        super().__init__("planning_diagnostic")

        self.diagnostic_topic = str(
            # HH_260311-00:00 Single consolidated ROS diagnostic stream.
            self.declare_parameter("diagnostic_topic", "diagnostic").value
        )
        self.publish_period_s = float(self.declare_parameter("publish_period_s", 0.2).value)
        self.stale_timeout_s = float(self.declare_parameter("stale_timeout_s", 2.0).value)

        self.topic_goal = str(self.declare_parameter("topic_goal", "/planning/goal_pose").value)
        self.topic_lanelet_pose = str(
            self.declare_parameter("topic_lanelet_pose", "/planning/lanelet_pose").value
        )
        self.topic_global_path = str(
            self.declare_parameter("topic_global_path", "/planning/global_path").value
        )
        self.topic_navigate_status = str(
            self.declare_parameter(
                "topic_navigate_status", "/planning/navigate_to_pose/_action/status"
            ).value
        )
        self.topic_local_path = str(
            self.declare_parameter("topic_local_path", "/planning/local_path").value
        )
        self.topic_local_plan_raw = str(
            self.declare_parameter("topic_local_plan_raw", "/planning/local_plan").value
        )
        self.topic_local_path_controller = str(
            self.declare_parameter(
                "topic_local_path_controller", "/planning/local_path_controller"
            ).value
        )
        self.topic_local_path_dwb = str(
            self.declare_parameter("topic_local_path_dwb", "/planning/local_path_dwb").value
        )
        self.topic_global_costmap = str(
            self.declare_parameter("topic_global_costmap", "/planning/global_costmap/costmap").value
        )
        self.topic_local_costmap = str(
            self.declare_parameter("topic_local_costmap", "/planning/local_costmap/costmap").value
        )
        self.topic_global_cost_markers = str(
            self.declare_parameter(
                "topic_global_cost_markers", "/planning/cost_grid/global_path_markers"
            ).value
        )
        self.topic_local_cost_markers = str(
            self.declare_parameter(
                "topic_local_cost_markers", "/planning/cost_grid/local_path_markers"
            ).value
        )

        self._diag_pub = self.create_publisher(DiagnosticArray, self.diagnostic_topic, 10)

        default_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        grid_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._msg_goal = PoseStamped()
        self._msg_lanelet_pose = PoseStamped()
        self._msg_navigate_status = GoalStatusArray()
        self._msg_global_path = Path()
        self._msg_local_path = Path()
        self._msg_local_plan_raw = Path()
        self._msg_local_path_controller = Path()
        self._msg_local_path_dwb = Path()
        self._msg_global_costmap = OccupancyGrid()
        self._msg_local_costmap = OccupancyGrid()
        self._msg_global_markers = MarkerArray()
        self._msg_local_markers = MarkerArray()

        self._state: Dict[str, StampState] = {
            "goal": StampState(),
            "global_path": StampState(),
            "local_path": StampState(),
            "global_markers": StampState(),
            "local_markers": StampState(),
        }

        self.create_subscription(PoseStamped, self.topic_goal, self._on_goal, default_qos)
        self.create_subscription(PoseStamped, self.topic_lanelet_pose, self._on_lanelet_pose, default_qos)
        self.create_subscription(
            GoalStatusArray, self.topic_navigate_status, self._on_navigate_status, default_qos
        )
        self.create_subscription(Path, self.topic_global_path, self._on_global_path, default_qos)
        self.create_subscription(Path, self.topic_local_path, self._on_local_path, default_qos)
        self.create_subscription(Path, self.topic_local_plan_raw, self._on_local_plan_raw, default_qos)
        self.create_subscription(
            Path, self.topic_local_path_controller, self._on_local_path_controller, default_qos
        )
        self.create_subscription(Path, self.topic_local_path_dwb, self._on_local_path_dwb, default_qos)
        self.create_subscription(
            OccupancyGrid, self.topic_global_costmap, self._on_global_costmap, grid_qos
        )
        self.create_subscription(
            OccupancyGrid, self.topic_local_costmap, self._on_local_costmap, grid_qos
        )
        self.create_subscription(
            MarkerArray, self.topic_global_cost_markers, self._on_global_markers, marker_qos
        )
        self.create_subscription(
            MarkerArray, self.topic_local_cost_markers, self._on_local_markers, marker_qos
        )

        self.create_timer(self.publish_period_s, self._on_timer)
        self.get_logger().info(
            "planning_diagnostic: diagnostic=%s period=%.2fs"
            % (self.diagnostic_topic, self.publish_period_s)
        )

    # Implements `_set_stamp` behavior.
    def _set_stamp(self, key: str) -> None:
        self._state[key].stamp = self.get_clock().now()

    # Implements `_on_goal` behavior.
    def _on_goal(self, msg: PoseStamped) -> None:
        self._msg_goal = msg
        self._set_stamp("goal")

    # Implements `_on_lanelet_pose` behavior.
    def _on_lanelet_pose(self, msg: PoseStamped) -> None:
        self._msg_lanelet_pose = msg

    # Implements `_on_navigate_status` behavior.
    def _on_navigate_status(self, msg: GoalStatusArray) -> None:
        self._msg_navigate_status = msg

    # Implements `_on_global_path` behavior.
    def _on_global_path(self, msg: Path) -> None:
        self._msg_global_path = msg
        self._set_stamp("global_path")

    # Implements `_on_local_path` behavior.
    def _on_local_path(self, msg: Path) -> None:
        self._msg_local_path = msg
        self._set_stamp("local_path")

    # Implements `_on_local_plan_raw` behavior.
    def _on_local_plan_raw(self, msg: Path) -> None:
        self._msg_local_plan_raw = msg

    # Implements `_on_local_path_controller` behavior.
    def _on_local_path_controller(self, msg: Path) -> None:
        self._msg_local_path_controller = msg

    # Implements `_on_local_path_dwb` behavior.
    def _on_local_path_dwb(self, msg: Path) -> None:
        self._msg_local_path_dwb = msg

    # Implements `_on_global_costmap` behavior.
    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        self._msg_global_costmap = msg

    # Implements `_on_local_costmap` behavior.
    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        self._msg_local_costmap = msg

    # Implements `_on_global_markers` behavior.
    def _on_global_markers(self, msg: MarkerArray) -> None:
        self._msg_global_markers = msg
        self._set_stamp("global_markers")

    # Implements `_on_local_markers` behavior.
    def _on_local_markers(self, msg: MarkerArray) -> None:
        self._msg_local_markers = msg
        self._set_stamp("local_markers")

    # Implements `_health` behavior.
    def _health(self, now: Time) -> ModuleHealth:
        out = ModuleHealth()
        out.stamp = now.to_msg()
        out.module_name = "planning"

        goal_seen = self._state["goal"].stamp is not None
        missing = []
        if goal_seen:
            for key in ("global_path", "local_path", "global_markers", "local_markers"):
                st = self._state[key].stamp
                if st is None:
                    missing.append(key)
                    continue
                age = (now - st).nanoseconds * 1e-9
                if age > self.stale_timeout_s:
                    missing.append(f"{key}(stale:{age:.2f}s)")

            if missing:
                out.level = ModuleHealth.ERROR
                out.message = "goal active but planning outputs are missing/stale"
                out.missing_topics = missing
            else:
                out.level = ModuleHealth.OK
                out.message = "planning outputs healthy"
        else:
            out.level = ModuleHealth.WARN
            out.message = "idle (no goal received yet)"

        return out

    # Implements `_on_timer` behavior.
    def _on_timer(self) -> None:
        now = self.get_clock().now()
        msg = AvgPlanningMsgs()
        msg.stamp = now.to_msg()
        msg.health = self._health(now)
        msg.goal_pose = self._msg_goal
        msg.lanelet_pose = self._msg_lanelet_pose
        msg.navigate_to_pose_status = self._msg_navigate_status
        msg.global_path = self._msg_global_path
        msg.local_path = self._msg_local_path
        msg.local_plan_raw = self._msg_local_plan_raw
        msg.local_path_controller = self._msg_local_path_controller
        msg.local_path_dwb = self._msg_local_path_dwb
        msg.global_costmap = self._msg_global_costmap
        msg.local_costmap = self._msg_local_costmap
        msg.global_path_cost_markers = self._msg_global_markers
        msg.local_path_cost_markers = self._msg_local_markers
        diag = DiagnosticArray()
        diag.header.stamp = msg.stamp
        st = DiagnosticStatus()
        st.name = "planning/diagnostic"
        st.hardware_id = "planning"
        st.level = _diag_level(msg.health.level)
        st.message = msg.health.message
        st.values.append(KeyValue(key="category", value="planning"))
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
    node = PlanningDiagnosticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        node.get_logger().error(f"planning_diagnostic runtime exception: {e}")
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
