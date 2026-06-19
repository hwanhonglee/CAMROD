#!/usr/bin/env python3
"""Latch planner/controller selector topics for Nav2 PlannerSelector/ControllerSelector."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class Nav2SelectorLatchNode(Node):
    def __init__(self) -> None:
        super().__init__("nav2_selector_latch")

        # HH_260528: Keep combo-level planner/controller choice sticky via transient-local QoS.
        self._planner_id = str(self.declare_parameter("planner_id", "NavFn").value).strip()
        # HH_260618: Default to MPPI so normal autonomy uses local trajectory
        # sampling against the local costmap instead of pure path tracking.
        self._controller_id = str(
            self.declare_parameter("controller_id", "MPPI").value
        ).strip()
        self._planner_topic = str(
            self.declare_parameter("planner_topic", "/planning/planner_selector").value
        ).strip()
        self._controller_topic = str(
            self.declare_parameter("controller_topic", "/planning/controller_selector").value
        ).strip()
        self._repeat_hz = float(self.declare_parameter("repeat_hz", 1.0).value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._planner_pub = self.create_publisher(String, self._planner_topic, qos)
        self._controller_pub = self.create_publisher(String, self._controller_topic, qos)

        self._publish_once()
        self.create_timer(1.0 / max(0.2, self._repeat_hz), self._publish_once)

        self.get_logger().info(
            f"nav2_selector_latch active: planner={self._planner_id} controller={self._controller_id}"
        )

    def _publish_once(self) -> None:
        planner_msg = String()
        planner_msg.data = self._planner_id
        self._planner_pub.publish(planner_msg)

        controller_msg = String()
        controller_msg.data = self._controller_id
        self._controller_pub.publish(controller_msg)


def main() -> None:
    rclpy.init()
    node = Nav2SelectorLatchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # HH_260617: Ctrl+C during launch shutdown should exit cleanly instead
        # of printing a traceback and reporting this helper as a crashed process.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
