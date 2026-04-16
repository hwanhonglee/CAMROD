#!/usr/bin/env python3
# HH_260409: Leaderboard-style real-sensor scenario runner for planning pipeline verification.

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool


@dataclass
class StepResult:
    step_id: str
    success: bool
    reason: str
    elapsed_sec: float
    global_latency_sec: float | None
    local_latency_sec: float | None
    cmd_latency_sec: float | None
    global_path_points: int
    local_path_points: int


class PlanningScenarioLeaderboard(Node):
    # Initializes publishers/subscribers and runtime metrics for scenario validation.
    def __init__(self) -> None:
        super().__init__("planning_scenario_leaderboard")

        default_scenario = str(
            Path(get_package_share_directory("camrod_bringup"))
            / "config"
            / "scenario"
            / "leaderboard_default.yaml"
        )

        self.scenario_file = str(
            self.declare_parameter("scenario_file", default_scenario).value
        )
        self.engage_topic = str(
            self.declare_parameter("engage_topic", "/planning/engage").value
        )
        self.goal_topic = str(self.declare_parameter("goal_topic", "/goal_pose").value)
        self.global_path_topic = str(
            self.declare_parameter("global_path_topic", "/planning/global_path").value
        )
        self.local_path_topic = str(
            self.declare_parameter("local_path_topic", "/planning/local_path").value
        )
        self.cmd_vel_topic = str(
            self.declare_parameter("cmd_vel_topic", "/planning/cmd_vel").value
        )
        self.estop_topic = str(
            self.declare_parameter("estop_topic", "/platform/status/estop").value
        )
        self.step_timeout_sec = float(
            self.declare_parameter("step_timeout_sec", 25.0).value
        )
        self.wait_for_topics_sec = float(
            self.declare_parameter("wait_for_topics_sec", 30.0).value
        )
        self.min_path_points = int(self.declare_parameter("min_path_points", 3).value)
        self.require_cmd_vel = bool(
            self.declare_parameter("require_cmd_vel", True).value
        )
        self.cmd_vel_abs_threshold = float(
            self.declare_parameter("cmd_vel_abs_threshold", 0.01).value
        )
        self.engage_before_each_step = bool(
            self.declare_parameter("engage_before_each_step", True).value
        )

        self.pub_goal = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.pub_engage = self.create_publisher(Bool, self.engage_topic, 10)

        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(Bool, self.estop_topic, self._on_estop, 10)

        self._global_sub = None
        self._local_sub = None
        self._global_count = 0
        self._local_count = 0
        self._global_last_points = 0
        self._local_last_points = 0
        self._global_last_recv = 0.0
        self._local_last_recv = 0.0
        self._last_nonzero_cmd_recv = 0.0
        self._estop = False

    # Handles cmd_vel callbacks and tracks non-zero command activity timing.
    def _on_cmd_vel(self, msg: Twist) -> None:
        cmd_abs = (
            abs(msg.linear.x)
            + abs(msg.linear.y)
            + abs(msg.linear.z)
            + abs(msg.angular.x)
            + abs(msg.angular.y)
            + abs(msg.angular.z)
        )
        if cmd_abs >= self.cmd_vel_abs_threshold:
            self._last_nonzero_cmd_recv = time.monotonic()

    # Handles estop callbacks.
    def _on_estop(self, msg: Bool) -> None:
        self._estop = bool(msg.data)

    # Handles global-path callbacks.
    def _on_global_path(self, msg: Any) -> None:
        self._global_count += 1
        self._global_last_recv = time.monotonic()
        self._global_last_points = len(getattr(msg, "poses", []))

    # Handles local-path callbacks.
    def _on_local_path(self, msg: Any) -> None:
        self._local_count += 1
        self._local_last_recv = time.monotonic()
        self._local_last_points = len(getattr(msg, "poses", []))

    # Resolves a topic's runtime message type and returns the corresponding Python class.
    def _resolve_topic_class(self, topic: str, timeout_sec: float):
        end_time = time.monotonic() + timeout_sec
        while time.monotonic() < end_time and rclpy.ok():
            for name, types in self.get_topic_names_and_types():
                if name == topic and types:
                    return get_message(types[0])
            rclpy.spin_once(self, timeout_sec=0.1)
        return None

    # Lazily creates dynamic subscriptions for path topics that can differ by message type.
    def setup_dynamic_path_subscriptions(self) -> bool:
        global_cls = self._resolve_topic_class(self.global_path_topic, self.wait_for_topics_sec)
        local_cls = self._resolve_topic_class(self.local_path_topic, self.wait_for_topics_sec)
        if global_cls is None or local_cls is None:
            self.get_logger().error(
                "failed to resolve path topic types: "
                f"global={self.global_path_topic} local={self.local_path_topic}"
            )
            return False

        self._global_sub = self.create_subscription(
            global_cls, self.global_path_topic, self._on_global_path, 10
        )
        self._local_sub = self.create_subscription(
            local_cls, self.local_path_topic, self._on_local_path, 10
        )
        self.get_logger().info(
            f"path subscriptions ready: {self.global_path_topic}({global_cls.__name__}), "
            f"{self.local_path_topic}({local_cls.__name__})"
        )
        return True

    # Publishes engage command.
    def publish_engage(self, engaged: bool) -> None:
        msg = Bool()
        msg.data = bool(engaged)
        self.pub_engage.publish(msg)

    # Publishes a goal pose in map frame.
    def publish_goal(self, x: float, y: float, z: float, yaw_deg: float) -> None:
        yaw_rad = math.radians(yaw_deg)
        q = Quaternion()
        q.w = math.cos(yaw_rad * 0.5)
        q.z = math.sin(yaw_rad * 0.5)
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = float(z)
        goal.pose.orientation = q
        # HH_260409: Publish twice to reduce one-shot RViz drop risk under load.
        self.pub_goal.publish(goal)
        rclpy.spin_once(self, timeout_sec=0.05)
        self.pub_goal.publish(goal)

    # Loads scenario steps from YAML file.
    def load_scenario_steps(self) -> list[dict[str, Any]]:
        with open(self.scenario_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        scenario = data.get("scenario", {})
        steps = scenario.get("steps", [])
        if not isinstance(steps, list) or not steps:
            raise RuntimeError(f"invalid scenario file: {self.scenario_file}")
        return steps

    # Executes one scenario step and returns measured latencies and success state.
    def run_step(self, step: dict[str, Any]) -> StepResult:
        step_id = str(step.get("id", "unnamed_step"))
        goal = step.get("goal", {})
        x = float(goal.get("x", 0.0))
        y = float(goal.get("y", 0.0))
        z = float(goal.get("z", 0.0))
        yaw_deg = float(goal.get("yaw_deg", 0.0))

        if self.engage_before_each_step:
            self.publish_engage(True)
            rclpy.spin_once(self, timeout_sec=0.1)

        base_global_count = self._global_count
        base_local_count = self._local_count
        start = time.monotonic()

        self.publish_goal(x, y, z, yaw_deg)

        global_latency = None
        local_latency = None
        cmd_latency = None
        reason = "timeout"
        success = False

        while (time.monotonic() - start) <= self.step_timeout_sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()

            if self._estop:
                reason = "estop_asserted"
                break

            global_ok = (
                self._global_count > base_global_count
                and self._global_last_points >= self.min_path_points
            )
            local_ok = (
                self._local_count > base_local_count
                and self._local_last_points >= self.min_path_points
            )
            cmd_ok = (not self.require_cmd_vel) or (
                self._last_nonzero_cmd_recv >= start
            )

            if global_ok and global_latency is None:
                global_latency = self._global_last_recv - start
            if local_ok and local_latency is None:
                local_latency = self._local_last_recv - start
            if cmd_ok and cmd_latency is None:
                cmd_latency = max(0.0, self._last_nonzero_cmd_recv - start)

            if global_ok and local_ok and cmd_ok:
                success = True
                reason = "ok"
                break

            if now - start > self.step_timeout_sec:
                break

        elapsed = time.monotonic() - start
        return StepResult(
            step_id=step_id,
            success=success,
            reason=reason,
            elapsed_sec=elapsed,
            global_latency_sec=global_latency,
            local_latency_sec=local_latency,
            cmd_latency_sec=cmd_latency,
            global_path_points=self._global_last_points,
            local_path_points=self._local_last_points,
        )


def _fmt_latency(value: float | None) -> str:
    if value is None:
        return "-"
    return f"{value:.2f}s"


def main(args=None) -> None:
    # Entrypoint for scenario leaderboard execution.
    rclpy.init(args=args)
    node = PlanningScenarioLeaderboard()
    results: list[StepResult] = []
    rc = 0
    try:
        if not node.setup_dynamic_path_subscriptions():
            raise RuntimeError("failed to subscribe path topics")
        steps = node.load_scenario_steps()
        node.get_logger().info(
            f"scenario start: file={node.scenario_file} steps={len(steps)} "
            f"require_cmd_vel={'true' if node.require_cmd_vel else 'false'}"
        )
        for step in steps:
            result = node.run_step(step)
            results.append(result)
            node.get_logger().info(
                f"[{result.step_id}] success={result.success} reason={result.reason} "
                f"elapsed={result.elapsed_sec:.2f}s global={_fmt_latency(result.global_latency_sec)} "
                f"local={_fmt_latency(result.local_latency_sec)} cmd={_fmt_latency(result.cmd_latency_sec)} "
                f"points(g/l)=({result.global_path_points}/{result.local_path_points})"
            )
            if not result.success:
                rc = 2

        print("\n=== CAMROD Scenario Leaderboard ===")
        print("step_id | success | reason | elapsed | global_latency | local_latency | cmd_latency | g_pts | l_pts")
        for result in results:
            print(
                f"{result.step_id} | {result.success} | {result.reason} | {result.elapsed_sec:.2f}s | "
                f"{_fmt_latency(result.global_latency_sec)} | {_fmt_latency(result.local_latency_sec)} | "
                f"{_fmt_latency(result.cmd_latency_sec)} | {result.global_path_points} | {result.local_path_points}"
            )
    except Exception as exc:
        rc = 2
        node.get_logger().error(f"scenario runner failed: {exc}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
