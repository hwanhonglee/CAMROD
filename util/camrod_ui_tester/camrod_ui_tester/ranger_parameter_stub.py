"""Standard ROS parameter endpoint used by camrod_ui's tuning screen."""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class RangerParameterStub(Node):
    def __init__(self) -> None:
        super().__init__("ranger_base_node")
        value = float(
            self.declare_parameter("steering_transition_rate_radps", 1.0).value
        )
        self.get_logger().info(
            "simulated Ranger parameter service ready: "
            f"steering_transition_rate_radps={value:.3f}"
        )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RangerParameterStub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
