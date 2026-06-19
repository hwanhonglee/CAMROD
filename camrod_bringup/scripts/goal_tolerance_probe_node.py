#!/usr/bin/env python3
"""Goal tolerance probe: measures actual XY error at goal arrival.

Subscribes to goal_pose_snapped and localization/pose.
When the robot stops moving near the last goal, prints the XY error
and accumulates statistics across multiple runs.

Usage:
    ros2 run camrod_bringup goal_tolerance_probe_node.py

    # Or with namespace:
    ros2 run camrod_bringup goal_tolerance_probe_node.py \
      --ros-args -p pose_topic:=/localization/pose \
                 -p goal_topic:=/planning/goal_pose_snapped \
                 -p stopped_speed_mps:=0.03 \
                 -p stopped_hold_s:=1.5
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class GoalToleranceProbe(Node):
    def __init__(self):
        super().__init__("goal_tolerance_probe")

        self.pose_topic = self.declare_parameter("pose_topic", "/localization/pose").value
        self.goal_topic = self.declare_parameter(
            "goal_topic", "/planning/goal_pose_snapped"
        ).value
        self.odom_topic = self.declare_parameter(
            "odom_topic", "/localization/odometry/filtered"
        ).value
        # Speed below which the robot is considered stopped.
        self.stopped_speed = float(
            self.declare_parameter("stopped_speed_mps", 0.03).value
        )
        # Must stay stopped for this duration before recording the error.
        self.stopped_hold_s = float(
            self.declare_parameter("stopped_hold_s", 1.5).value
        )

        self._goal: tuple[float, float] | None = None
        self._pose: tuple[float, float] | None = None
        self._stopped_since: float | None = None
        self._recorded_goals: set[tuple[float, float]] = set()

        self._errors: list[float] = []

        self.create_subscription(PoseStamped, self.pose_topic, self._on_pose, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 20)

        self.get_logger().info(
            f"goal_tolerance_probe ready — goal={self.goal_topic} pose={self.pose_topic}"
        )

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goal = (msg.pose.position.x, msg.pose.position.y)
        self._stopped_since = None

    def _on_pose(self, msg: PoseStamped) -> None:
        self._pose = (msg.pose.position.x, msg.pose.position.y)

    def _on_odom(self, msg: Odometry) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)
        now = self.get_clock().now().nanoseconds * 1e-9

        if speed > self.stopped_speed:
            self._stopped_since = None
            return

        if self._stopped_since is None:
            self._stopped_since = now
            return

        if (now - self._stopped_since) < self.stopped_hold_s:
            return

        if self._goal is None or self._pose is None:
            return

        # HH_260617: Use a coordinate-specific name; mission keys are handled
        # separately by the planning state machine.
        goal_xy_signature = (round(self._goal[0], 2), round(self._goal[1], 2))
        if goal_xy_signature in self._recorded_goals:
            return
        self._recorded_goals.add(goal_xy_signature)

        err = math.hypot(self._pose[0] - self._goal[0], self._pose[1] - self._goal[1])
        self._errors.append(err)

        n = len(self._errors)
        mean = sum(self._errors) / n
        worst = max(self._errors)

        self.get_logger().info(
            f"[run #{n}] goal=({self._goal[0]:.3f}, {self._goal[1]:.3f})  "
            f"pose=({self._pose[0]:.3f}, {self._pose[1]:.3f})  "
            f"XY_error={err*100:.1f} cm  |  "
            f"mean={mean*100:.1f} cm  worst={worst*100:.1f} cm"
        )


def main():
    rclpy.init()
    node = GoalToleranceProbe()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
