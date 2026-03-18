#!/usr/bin/env python3
# HH_260317-00:00
# Nav2 lifecycle startup retry helper.
# - Watches /planning/lifecycle_manager_planning/is_active
# - Retries STARTUP command when bringup stops in partially configured state
#   (observed intermittently during behavior_server configure transition).

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ManageLifecycleNodes
from std_srvs.srv import Trigger


class Nav2LifecycleStartupRetryNode(Node):
    # Implements `Nav2LifecycleStartupRetryNode` behavior.
    def __init__(self) -> None:
        super().__init__("nav2_lifecycle_startup_retry")

        self.manager_service = str(
            self.declare_parameter(
                "manager_service",
                "/planning/lifecycle_manager_planning/manage_nodes",
            ).value
        )
        self.is_active_service = str(
            self.declare_parameter(
                "is_active_service",
                "/planning/lifecycle_manager_planning/is_active",
            ).value
        )
        self.retry_period_s = float(self.declare_parameter("retry_period_s", 1.0).value)
        self.startup_cooldown_s = float(
            self.declare_parameter("startup_cooldown_s", 2.0).value
        )

        self._manager_client = self.create_client(ManageLifecycleNodes, self.manager_service)
        self._is_active_client = self.create_client(Trigger, self.is_active_service)

        self._active_future: Optional[rclpy.task.Future] = None
        self._startup_future: Optional[rclpy.task.Future] = None
        self._next_startup_time = self.get_clock().now()
        self._done = False

        self._timer = self.create_timer(self.retry_period_s, self._on_timer)

        # HH_260317-00:00 rclpy logger does not support printf-style variadic args.
        # Keep explicit f-string logging to avoid runtime TypeError on startup.
        self.get_logger().info(
            "nav2_lifecycle_startup_retry: "
            f"is_active={self.is_active_service} "
            f"manage={self.manager_service} "
            f"period={self.retry_period_s:.2f}s "
            f"cooldown={self.startup_cooldown_s:.2f}s"
        )

    # Implements `_on_timer` behavior.
    def _on_timer(self) -> None:
        if self._done:
            return

        # Process pending is_active result.
        if self._active_future is not None and self._active_future.done():
            future = self._active_future
            self._active_future = None
            try:
                response = future.result()
            except Exception as exc:  # pragma: no cover - runtime guard
                self.get_logger().warn(f"is_active call failed: {exc}")
                return

            if response.success:
                self._done = True
                self.get_logger().info("Nav2 lifecycle is active. Startup retry node idling.")
                return

            now = self.get_clock().now()
            if now >= self._next_startup_time and self._startup_future is None:
                request = ManageLifecycleNodes.Request()
                request.command = ManageLifecycleNodes.Request.STARTUP
                self._startup_future = self._manager_client.call_async(request)
                self._next_startup_time = now + rclpy.duration.Duration(
                    seconds=self.startup_cooldown_s
                )
            return

        # Process pending startup result.
        if self._startup_future is not None and self._startup_future.done():
            future = self._startup_future
            self._startup_future = None
            try:
                response = future.result()
            except Exception as exc:  # pragma: no cover - runtime guard
                self.get_logger().warn(f"STARTUP call failed: {exc}")
                return
            self.get_logger().info(f"STARTUP request result: success={response.success}")
            return

        if self._active_future is not None or self._startup_future is not None:
            return

        if not self._is_active_client.wait_for_service(timeout_sec=0.0):
            return
        if not self._manager_client.wait_for_service(timeout_sec=0.0):
            return

        self._active_future = self._is_active_client.call_async(Trigger.Request())


def main(args=None) -> None:
    # Implements `main` behavior.
    rclpy.init(args=args)
    node = Nav2LifecycleStartupRetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
