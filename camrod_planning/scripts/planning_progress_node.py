#!/usr/bin/env python3
"""Publishes remaining path distance and estimated travel time from current pose.

Subscribes to the Nav2 global path and current localization pose.
Finds the closest point on the path to the robot, then sums segment
lengths from that point to the goal to compute remaining distance.
Remaining time = remaining_distance / smoothed_speed (min 0.1 m/s floor).

Published topics:
  /planning/progress/remaining_distance_m  (avg_msgs/AvgFloat32)
  /planning/progress/remaining_time_s      (avg_msgs/AvgFloat32)
  /planning/progress/completion_pct        (avg_msgs/AvgFloat32)  0-100 %
"""

import math

import rclpy
from avg_msgs.msg import AvgFloat32, AvgOdometry, AvgPath, AvgPoseStamped
from rclpy.node import Node


class PlanningProgressNode(Node):
    def __init__(self):
        super().__init__("planning_progress_node")

        self._path_topic = self.declare_parameter(
            "path_topic", "/planning/global_path_avg"
        ).value
        self._pose_topic = self.declare_parameter(
            "pose_topic", "/localization/pose"
        ).value
        self._odom_topic = self.declare_parameter(
            "odom_topic", "/localization/odometry"
        ).value
        self._publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 2.0).value
        )
        # Speed EMA factor (0–1): lower = smoother but slower to react.
        self._speed_alpha = float(
            self.declare_parameter("speed_ema_alpha", 0.2).value
        )
        # Floor speed used for time estimate when robot is stopped or slow.
        self._speed_floor_mps = float(
            self.declare_parameter("speed_floor_mps", 0.1).value
        )

        self._path: list[tuple[float, float]] = []
        self._total_path_dist: float = 0.0
        self._pose: tuple[float, float] | None = None
        self._speed_ema: float = 0.0

        self._pub_dist = self.create_publisher(
            AvgFloat32, "/planning/progress/remaining_distance_m", 10
        )
        self._pub_time = self.create_publisher(
            AvgFloat32, "/planning/progress/remaining_time_s", 10
        )
        self._pub_pct = self.create_publisher(
            AvgFloat32, "/planning/progress/completion_pct", 10
        )

        # HH_260720 - Consume generated CAMROD path and pose contracts.
        self.create_subscription(AvgPath, self._path_topic, self._on_path, 10)
        self.create_subscription(AvgPoseStamped, self._pose_topic, self._on_pose, 10)
        # HH_260720 - Consume the generated internal odometry contract.
        self.create_subscription(AvgOdometry, self._odom_topic, self._on_odom, 20)
        self.create_timer(1.0 / max(0.1, self._publish_rate_hz), self._publish)

        self.get_logger().info(
            f"planning_progress_node ready — path={self._path_topic} "
            f"pose={self._pose_topic} rate={self._publish_rate_hz:.1f} Hz"
        )

    def _on_path(self, msg: AvgPath) -> None:
        pts = [
            (float(ps.pose.position.x), float(ps.pose.position.y))
            for ps in msg.poses
        ]
        total = sum(
            math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            for i in range(len(pts) - 1)
        )
        self._path = pts
        self._total_path_dist = total

    def _on_pose(self, msg: AvgPoseStamped) -> None:
        self._pose = (float(msg.pose.position.x), float(msg.pose.position.y))

    def _on_odom(self, msg: AvgOdometry) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)
        self._speed_ema = (
            self._speed_alpha * speed + (1.0 - self._speed_alpha) * self._speed_ema
        )

    def _publish(self) -> None:
        if not self._path or self._pose is None:
            return

        rx, ry = self._pose
        pts = self._path

        # Find the index of the closest path point to the robot.
        best_idx = 0
        best_dist = float("inf")
        for i, (px, py) in enumerate(pts):
            d = math.hypot(rx - px, ry - py)
            if d < best_dist:
                best_dist = d
                best_idx = i

        # Sum remaining path length from best_idx to end.
        remaining = 0.0
        for i in range(best_idx, len(pts) - 1):
            remaining += math.hypot(
                pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
            )

        total = self._total_path_dist
        completed = max(0.0, total - remaining)
        pct = (completed / total * 100.0) if total > 1e-3 else 100.0

        effective_speed = max(self._speed_floor_mps, self._speed_ema)
        remaining_time = remaining / effective_speed

        # HH_260720 - Publish generated CAMROD progress values instead of std_msgs aliases.
        dist_msg = AvgFloat32()
        dist_msg.data = float(remaining)
        self._pub_dist.publish(dist_msg)

        time_msg = AvgFloat32()
        time_msg.data = float(remaining_time)
        self._pub_time.publish(time_msg)

        pct_msg = AvgFloat32()
        pct_msg.data = float(pct)
        self._pub_pct.publish(pct_msg)


def main():
    rclpy.init()
    node = PlanningProgressNode()
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
