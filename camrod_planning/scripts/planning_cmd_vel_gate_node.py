#!/usr/bin/env python3
# HH_260331: Gate Nav2 controller cmd_vel with explicit planning engage trigger.

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class PlanningCmdVelGateNode(Node):
    # Implements `__init__` behavior.
    def __init__(self) -> None:
        super().__init__("planning_cmd_vel_gate")

        self.input_topic = str(
            self.declare_parameter("input_topic", "/planning/cmd_vel_raw").value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/planning/cmd_vel").value
        )
        self.engage_topic = str(
            self.declare_parameter("engage_topic", "/planning/engage").value
        )
        self.state_topic = str(
            self.declare_parameter("state_topic", "/planning/engaged").value
        )
        self.use_estop_topic = bool(
            self.declare_parameter("use_estop_topic", True).value
        )
        self.estop_topic = str(
            self.declare_parameter("estop_topic", "/planning/state_machine/estop").value
        )
        self.allow_on_start = bool(
            self.declare_parameter("allow_on_start", False).value
        )
        self.publish_zero_when_blocked = bool(
            self.declare_parameter("publish_zero_when_blocked", True).value
        )

        self._enabled = self.allow_on_start
        self._estop = False

        self.pub_cmd = self.create_publisher(Twist, self.output_topic, 10)
        self.pub_state = self.create_publisher(Bool, self.state_topic, 10)

        self.sub_cmd = self.create_subscription(Twist, self.input_topic, self._on_cmd, 10)
        self.sub_engage = self.create_subscription(
            Bool, self.engage_topic, self._on_engage, 10
        )
        self.sub_estop = None
        if self.use_estop_topic:
            self.sub_estop = self.create_subscription(
                Bool, self.estop_topic, self._on_estop, 10
            )

        self._state_timer = self.create_timer(0.5, self._publish_state)
        self._publish_state()
        self.get_logger().info(
            "planning_cmd_vel_gate ready: "
            f"in={self.input_topic} out={self.output_topic} "
            f"engage_topic={self.engage_topic} "
            f"estop_topic={self.estop_topic if self.use_estop_topic else '(disabled)'} "
            f"allow_on_start={'true' if self.allow_on_start else 'false'}"
        )

    # Implements `_effective_enabled` behavior.
    def _effective_enabled(self) -> bool:
        return self._enabled and not self._estop

    # Implements `_publish_state` behavior.
    def _publish_state(self) -> None:
        msg = Bool()
        msg.data = self._effective_enabled()
        self.pub_state.publish(msg)

    # Implements `_publish_zero` behavior.
    def _publish_zero(self) -> None:
        self.pub_cmd.publish(Twist())

    # Handles the `_on_cmd` callback.
    def _on_cmd(self, msg: Twist) -> None:
        if self._effective_enabled():
            self.pub_cmd.publish(msg)
            return
        if self.publish_zero_when_blocked:
            self._publish_zero()

    # Handles the `_on_engage` callback.
    def _on_engage(self, msg: Bool) -> None:
        self._enabled = bool(msg.data)
        self._publish_state()
        if not self._effective_enabled() and self.publish_zero_when_blocked:
            self._publish_zero()
        self.get_logger().info(
            "planning engage update: "
            f"enabled={'true' if self._enabled else 'false'} "
            f"estop={'true' if self._estop else 'false'} "
            f"effective={'true' if self._effective_enabled() else 'false'}"
        )

    # Handles the `_on_estop` callback.
    def _on_estop(self, msg: Bool) -> None:
        self._estop = bool(msg.data)
        self._publish_state()
        if self._estop and self.publish_zero_when_blocked:
            self._publish_zero()
        self.get_logger().warn(
            "planning estop update: "
            f"estop={'true' if self._estop else 'false'} "
            f"effective={'true' if self._effective_enabled() else 'false'}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanningCmdVelGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
