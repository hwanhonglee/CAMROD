#!/usr/bin/env python3
# HH_260327: Gate planning cmd_vel with explicit drive-enable trigger.

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class CmdVelGateNode(Node):
    # Initializes gate parameters, I/O endpoints, and runtime state publishers/subscribers.
    def __init__(self) -> None:
        super().__init__("cmd_vel_gate")

        self.input_cmd_vel_topic = str(
            self.declare_parameter("input_cmd_vel_topic", "/planning/cmd_vel").value
        )
        self.output_cmd_vel_topic = str(
            self.declare_parameter("output_cmd_vel_topic", "/platform/cmd_vel").value
        )
        self.enable_topic = str(
            self.declare_parameter("enable_topic", "/platform/drive_enable").value
        )
        # HH_260327: Planning engage trigger alias.
        self.engage_topic = str(
            self.declare_parameter("engage_topic", "/planning/engage").value
        )
        self.use_engage_topic = bool(
            self.declare_parameter("use_engage_topic", True).value
        )
        self.state_topic = str(
            self.declare_parameter("state_topic", "/platform/drive_enabled").value
        )
        self.use_estop_topic = bool(
            self.declare_parameter("use_estop_topic", True).value
        )
        self.estop_topic = str(
            # HH_260409: Default e-stop source moved to platform status topic.
            self.declare_parameter("estop_topic", "/platform/status/estop").value
        )
        self.allow_on_start = bool(
            self.declare_parameter("allow_on_start", False).value
        )
        self.publish_zero_when_blocked = bool(
            self.declare_parameter("publish_zero_when_blocked", True).value
        )

        self._enabled = self.allow_on_start
        self._estop = False
        self._last_input: Optional[Twist] = None

        self.pub_cmd = self.create_publisher(Twist, self.output_cmd_vel_topic, 10)
        self.pub_state = self.create_publisher(Bool, self.state_topic, 10)

        self.sub_cmd = self.create_subscription(
            Twist, self.input_cmd_vel_topic, self._on_cmd_vel, 10
        )
        self.sub_enable = self.create_subscription(
            Bool, self.enable_topic, self._on_enable, 10
        )
        self.sub_engage = None
        if self.use_engage_topic:
            self.sub_engage = self.create_subscription(
                Bool, self.engage_topic, self._on_engage, 10
            )
        self.sub_estop = None
        if self.use_estop_topic:
            self.sub_estop = self.create_subscription(
                Bool, self.estop_topic, self._on_estop, 10
            )

        self.srv_set_enabled = self.create_service(
            SetBool, "set_enabled", self._on_set_enabled
        )

        self._state_timer = self.create_timer(0.5, self._publish_state)
        self._publish_state()
        self.get_logger().info(
            f"cmd_vel_gate ready: in={self.input_cmd_vel_topic} "
            f"out={self.output_cmd_vel_topic} "
            f"enable_topic={self.enable_topic} "
            f"engage_topic={self.engage_topic if self.use_engage_topic else '(disabled)'} "
            f"estop_topic={self.estop_topic if self.use_estop_topic else '(disabled)'} "
            f"allow_on_start={'true' if self.allow_on_start else 'false'}"
        )

    # Returns the final drive-enable state after applying e-stop override.
    def _effective_enabled(self) -> bool:
        return self._enabled and not self._estop

    # Publishes current gate state so downstream modules can observe effective enable.
    def _publish_state(self) -> None:
        msg = Bool()
        msg.data = self._effective_enabled()
        self.pub_state.publish(msg)

    # Sends a zero-velocity command when the gate blocks motion.
    def _publish_zero(self) -> None:
        zero = Twist()
        self.pub_cmd.publish(zero)

    # Passes through or blocks incoming velocity commands based on gate state.
    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_input = msg
        if self._effective_enabled():
            self.pub_cmd.publish(msg)
            return

        if self.publish_zero_when_blocked:
            self._publish_zero()
        self.get_logger().debug(
            f"cmd_vel blocked: enabled={str(self._enabled).lower()} "
            f"estop={str(self._estop).lower()}"
        )

    # Updates gate state from `/platform/drive_enable` style control topic.
    def _on_enable(self, msg: Bool) -> None:
        new_enabled = bool(msg.data)
        if new_enabled == self._enabled:
            return
        self._enabled = new_enabled
        self._publish_state()
        if not self._effective_enabled() and self.publish_zero_when_blocked:
            self._publish_zero()
        self.get_logger().info(
            f"drive enable topic update: enabled={'true' if self._enabled else 'false'} "
            f"estop={'true' if self._estop else 'false'} "
            f"effective={'true' if self._effective_enabled() else 'false'}"
        )

    # Mirrors planning engage signal into the same gate state used by enable topic.
    def _on_engage(self, msg: Bool) -> None:
        # HH_260327: Mirror planning engage bool into platform drive-enable gate.
        new_enabled = bool(msg.data)
        if new_enabled == self._enabled:
            return
        self._enabled = new_enabled
        self._publish_state()
        if not self._effective_enabled() and self.publish_zero_when_blocked:
            self._publish_zero()
        self.get_logger().info(
            f"planning engage update: enabled={'true' if self._enabled else 'false'} "
            f"estop={'true' if self._estop else 'false'} "
            f"effective={'true' if self._effective_enabled() else 'false'}"
        )

    # Applies emergency-stop state and forces zero command when configured.
    def _on_estop(self, msg: Bool) -> None:
        new_estop = bool(msg.data)
        if new_estop == self._estop:
            return
        self._estop = new_estop
        self._publish_state()
        if self._estop and self.publish_zero_when_blocked:
            self._publish_zero()
        self.get_logger().warn(
            f"estop update: estop={'true' if self._estop else 'false'} "
            f"effective_enabled={'true' if self._effective_enabled() else 'false'}"
        )

    # Service callback for external tools to toggle gate state at runtime.
    def _on_set_enabled(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self._enabled = bool(request.data)
        self._publish_state()
        if not self._effective_enabled() and self.publish_zero_when_blocked:
            self._publish_zero()
        response.success = True
        response.message = (
            f"enabled={self._enabled} estop={self._estop} effective={self._effective_enabled()}"
        )
        return response


def main(args=None) -> None:
    # Entrypoint that spins the gate node until shutdown.
    rclpy.init(args=args)
    node = CmdVelGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
