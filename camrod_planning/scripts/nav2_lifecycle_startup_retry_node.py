#!/usr/bin/env python3
# HH_260317-00:00
# Nav2 lifecycle startup retry helper.
# - Watches /planning/lifecycle_manager_planning/is_active
# - Retries STARTUP command when bringup stops in partially configured state
#   (observed intermittently during behavior_server configure transition).

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class Nav2LifecycleStartupRetryNode(Node):
    @staticmethod
    def _to_bool(value: object) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

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
        # HH_260327: Delay Nav2 lifecycle STARTUP until localization is ready.
        self.require_localization_ready = self._to_bool(
            self.declare_parameter("require_localization_ready", False).value
        )
        self.localization_ready_topic = str(
            self.declare_parameter(
                "localization_ready_topic", "/localization/initial_match_ok"
            ).value
        )
        self.localization_pose_cov_topic = str(
            self.declare_parameter(
                "localization_pose_cov_topic", "/localization/pose_with_covariance"
            ).value
        )
        self.localization_pose_timeout_s = float(
            self.declare_parameter("localization_pose_timeout_s", 1.0).value
        )
        self.max_position_variance = float(
            self.declare_parameter("max_position_variance", 9.0).value
        )
        self.max_yaw_variance = float(
            self.declare_parameter("max_yaw_variance", 1.0).value
        )
        self.require_covariance_sanity = self._to_bool(
            self.declare_parameter("require_covariance_sanity", True).value
        )

        self._manager_client = self.create_client(ManageLifecycleNodes, self.manager_service)
        self._is_active_client = self.create_client(Trigger, self.is_active_service)

        self._active_future: Optional[rclpy.task.Future] = None
        self._startup_future: Optional[rclpy.task.Future] = None
        self._next_startup_time = self.get_clock().now()
        self._done = False
        self._last_wait_reason = ""
        self._last_wait_log_time = self.get_clock().now()
        self._initial_match_ok = False
        self._pose_cov_received = False
        self._pose_cov_sane = False
        self._last_pose_cov_rx_time: Optional[rclpy.time.Time] = None

        self._timer = self.create_timer(self.retry_period_s, self._on_timer)

        if self.require_localization_ready:
            self._initial_match_sub = self.create_subscription(
                Bool, self.localization_ready_topic, self._on_localization_ready, 10
            )
            self._pose_cov_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                self.localization_pose_cov_topic,
                self._on_localization_pose_cov,
                10,
            )
        else:
            self._initial_match_sub = None
            self._pose_cov_sub = None

        # HH_260317-00:00 rclpy logger does not support printf-style variadic args.
        # Keep explicit f-string logging to avoid runtime TypeError on startup.
        self.get_logger().info(
            "nav2_lifecycle_startup_retry: "
            f"is_active={self.is_active_service} "
            f"manage={self.manager_service} "
            f"period={self.retry_period_s:.2f}s "
            f"cooldown={self.startup_cooldown_s:.2f}s "
            f"require_localization_ready={self.require_localization_ready}"
        )

    # Handles localization-ready latch topic.
    def _on_localization_ready(self, msg: Bool) -> None:
        self._initial_match_ok = bool(msg.data)

    # Handles localization pose covariance topic.
    def _on_localization_pose_cov(self, msg: PoseWithCovarianceStamped) -> None:
        self._pose_cov_received = True
        self._last_pose_cov_rx_time = self.get_clock().now()

        cov = msg.pose.covariance
        pos_var = max(float(cov[0]), float(cov[7]))
        yaw_var = float(cov[35])
        finite = math.isfinite(pos_var) and math.isfinite(yaw_var)
        self._pose_cov_sane = (
            finite
            and pos_var >= 0.0
            and yaw_var >= 0.0
            and pos_var <= self.max_position_variance
            and yaw_var <= self.max_yaw_variance
        )

    # Checks localization readiness state.
    def _is_localization_ready(self, now: rclpy.time.Time) -> tuple[bool, str]:
        if not self.require_localization_ready:
            return True, "disabled"

        if not self._initial_match_ok:
            return False, "waiting_initial_match_ok"

        if not self._pose_cov_received or self._last_pose_cov_rx_time is None:
            return False, "waiting_pose_with_covariance"

        age_s = (now - self._last_pose_cov_rx_time).nanoseconds * 1e-9
        if age_s > self.localization_pose_timeout_s:
            return False, f"pose_with_covariance_stale({age_s:.2f}s)"

        if self.require_covariance_sanity and not self._pose_cov_sane:
            return False, "pose_covariance_out_of_range"

        return True, "ready"

    # Throttled wait log helper.
    def _log_wait_reason(self, reason: str, now: rclpy.time.Time) -> None:
        elapsed = (now - self._last_wait_log_time).nanoseconds * 1e-9
        if reason != self._last_wait_reason or elapsed >= 2.0:
            self.get_logger().info(f"Nav2 STARTUP gated: {reason}")
            self._last_wait_reason = reason
            self._last_wait_log_time = now

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
                ready, reason = self._is_localization_ready(now)
                if not ready:
                    self._log_wait_reason(reason, now)
                    return
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
