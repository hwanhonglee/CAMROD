#!/usr/bin/env python3
# Leaderboard-style real-sensor scenario runner for planning pipeline verification.

from __future__ import annotations

import json
import math
import os
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
from std_msgs.msg import Bool, String


@dataclass
class StepResult:
    step_id: str
    success: bool
    reason: str
    elapsed_sec: float
    timeout_sec: float
    global_latency_sec: float | None
    local_latency_sec: float | None
    cmd_latency_sec: float | None
    global_path_points: int
    local_path_points: int
    expected_state_sequence: list[str]
    expected_mission_source_sequence: list[str]
    observed_state_sequence: list[str]
    observed_mission_source_sequence: list[str]
    state_sequence_ok: bool
    mission_source_sequence_ok: bool


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
        self.mission_key_topic = str(
            self.declare_parameter(
                "mission_key_topic", "/planning/mission_key"
            ).value
        )
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
        self.state_topic = str(
            self.declare_parameter("state_topic", "/planning/state_machine/state").value
        )
        self.mission_source_topic = str(
            self.declare_parameter(
                "mission_source_topic", "/planning/state_machine/mission_source"
            ).value
        )
        self.step_timeout_s = float(
            self.declare_parameter("step_timeout_s", 25.0).value
        )

        self.wait_for_topics_s = float(
            self.declare_parameter("wait_for_topics_s", 30.0).value
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
        # Print a fixed PASS/FAIL report block for CI/log scraping.
        self.print_fixed_report = bool(
            self.declare_parameter("print_fixed_report", True).value
        )
        # Optional JSON report output path.
        self.report_file = str(self.declare_parameter("report_file", "").value)

        self.pub_goal = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.pub_mission_key = self.create_publisher(String, self.mission_key_topic, 10)
        self.pub_engage = self.create_publisher(Bool, self.engage_topic, 10)

        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(Bool, self.estop_topic, self._on_estop, 10)
        self.create_subscription(String, self.state_topic, self._on_state, 10)
        self.create_subscription(
            String, self.mission_source_topic, self._on_mission_source, 10
        )

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
        self._state_history: list[tuple[float, str]] = []
        self._mission_source_history: list[tuple[float, str]] = []

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

    # Handles planning state-machine state topic updates.
    def _on_state(self, msg: String) -> None:
        self._state_history.append((time.monotonic(), str(msg.data)))

    # Handles planning state-machine mission-source topic updates.
    def _on_mission_source(self, msg: String) -> None:
        self._mission_source_history.append((time.monotonic(), str(msg.data)))

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
        global_cls = self._resolve_topic_class(self.global_path_topic, self.wait_for_topics_s)
        local_cls = self._resolve_topic_class(self.local_path_topic, self.wait_for_topics_s)
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
        # Publish twice to reduce one-shot RViz drop risk under load.
        self.pub_goal.publish(goal)
        rclpy.spin_once(self, timeout_sec=0.05)
        self.pub_goal.publish(goal)

    # Publishes a state-machine mission key (e.g. camping_site_1, drop_zone).
    def publish_mission_key(self, mission_key: str) -> None:
        msg = String()
        msg.data = str(mission_key).strip()
        if not msg.data:
            return
        self.pub_mission_key.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.05)
        self.pub_mission_key.publish(msg)

    # Loads scenario YAML and returns normalized dict with name + steps.
    def load_scenario(self) -> dict[str, Any]:
        with open(self.scenario_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        scenario = data.get("scenario", {})
        if not isinstance(scenario, dict):
            raise RuntimeError(f"invalid scenario file: {self.scenario_file}")
        steps = scenario.get("steps", [])
        if not isinstance(steps, list) or not steps:
            raise RuntimeError(f"invalid scenario file: {self.scenario_file}")
        return {
            "name": str(scenario.get("name", "unnamed_scenario")),
            "steps": steps,
        }

    @staticmethod
    # Converts YAML list/tuple values into stripped string list.
    def _normalize_str_list(raw_value: Any) -> list[str]:
        if not isinstance(raw_value, (list, tuple)):
            return []
        out: list[str] = []
        for item in raw_value:
            text = str(item).strip()
            if text:
                out.append(text)
        return out

    @staticmethod
    # Returns true when expected tokens appear in order as substring matches.
    def _sequence_seen_in_order(observed_values: list[str], expected_tokens: list[str]) -> bool:
        if not expected_tokens:
            return True
        cursor = 0
        for value in observed_values:
            if expected_tokens[cursor] in value:
                cursor += 1
                if cursor >= len(expected_tokens):
                    return True
        return False

    # Executes one scenario step and returns measured latencies and success state.
    def run_step(self, step: dict[str, Any]) -> StepResult:
        step_id = str(step.get("id", "unnamed_step"))
        goal = step.get("goal", {})
        mission_key = str(step.get("mission_key", "")).strip()
        x = float(goal.get("x", 0.0))
        y = float(goal.get("y", 0.0))
        z = float(goal.get("z", 0.0))
        yaw_deg = float(goal.get("yaw_deg", 0.0))
        step_timeout_s = float(step.get("timeout_s", self.step_timeout_s))
        expected_state_sequence = self._normalize_str_list(
            step.get("expect_state_sequence", [])
        )
        expected_mission_sequence = self._normalize_str_list(
            step.get("expect_mission_source_sequence", [])
        )

        if self.engage_before_each_step:
            self.publish_engage(True)
            rclpy.spin_once(self, timeout_sec=0.1)

        base_global_count = self._global_count
        base_local_count = self._local_count
        base_state_count = len(self._state_history)
        base_mission_count = len(self._mission_source_history)
        start = time.monotonic()

        if mission_key:
            # HH_260617: Prefer mission dispatch for state-machine-driven scenarios.
            self.publish_mission_key(mission_key)
        else:
            self.publish_goal(x, y, z, yaw_deg)

        global_latency = None
        local_latency = None
        cmd_latency = None
        reason = "timeout"
        success = False
        pipeline_ok = False
        state_sequence_ok = False
        mission_sequence_ok = False

        while (time.monotonic() - start) <= step_timeout_s and rclpy.ok():
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
                pipeline_ok = True
                observed_states = [v for _, v in self._state_history[base_state_count:]]
                observed_missions = [
                    v for _, v in self._mission_source_history[base_mission_count:]
                ]
                state_sequence_ok = self._sequence_seen_in_order(
                    observed_states, expected_state_sequence
                )
                mission_sequence_ok = self._sequence_seen_in_order(
                    observed_missions, expected_mission_sequence
                )
                if state_sequence_ok and mission_sequence_ok:
                    success = True
                    reason = "ok"
                    break

            if now - start > step_timeout_s:
                break

        elapsed = time.monotonic() - start
        observed_states = [v for _, v in self._state_history[base_state_count:]]
        observed_missions = [v for _, v in self._mission_source_history[base_mission_count:]]
        state_sequence_ok = self._sequence_seen_in_order(
            observed_states, expected_state_sequence
        )
        mission_sequence_ok = self._sequence_seen_in_order(
            observed_missions, expected_mission_sequence
        )
        if not success and pipeline_ok:
            if not state_sequence_ok and not mission_sequence_ok:
                reason = "state_and_mission_sequence_timeout"
            elif not state_sequence_ok:
                reason = "state_sequence_timeout"
            elif not mission_sequence_ok:
                reason = "mission_sequence_timeout"
        return StepResult(
            step_id=step_id,
            success=success,
            reason=reason,
            elapsed_sec=elapsed,
            timeout_sec=step_timeout_s,
            global_latency_sec=global_latency,
            local_latency_sec=local_latency,
            cmd_latency_sec=cmd_latency,
            global_path_points=self._global_last_points,
            local_path_points=self._local_last_points,
            expected_state_sequence=expected_state_sequence,
            expected_mission_source_sequence=expected_mission_sequence,
            observed_state_sequence=observed_states,
            observed_mission_source_sequence=observed_missions,
            state_sequence_ok=state_sequence_ok,
            mission_source_sequence_ok=mission_sequence_ok,
        )


def _fmt_latency(value: float | None) -> str:
    if value is None:
        return "-"
    return f"{value:.2f}s"


def _build_report_dict(
    scenario_name: str, scenario_file: str, results: list[StepResult], overall_pass: bool
) -> dict[str, Any]:
    return {
        "scenario_name": scenario_name,
        "scenario_file": scenario_file,
        "overall_pass": bool(overall_pass),
        "total_steps": len(results),
        "passed_steps": sum(1 for r in results if r.success),
        "failed_steps": sum(1 for r in results if not r.success),
        "steps": [
            {
                "id": r.step_id,
                "success": r.success,
                "reason": r.reason,
                "elapsed_sec": round(r.elapsed_sec, 3),
                "timeout_sec": round(r.timeout_sec, 3),
                "global_latency_sec": None
                if r.global_latency_sec is None
                else round(r.global_latency_sec, 3),
                "local_latency_sec": None
                if r.local_latency_sec is None
                else round(r.local_latency_sec, 3),
                "cmd_latency_sec": None
                if r.cmd_latency_sec is None
                else round(r.cmd_latency_sec, 3),
                "global_path_points": r.global_path_points,
                "local_path_points": r.local_path_points,
                "state_sequence_ok": r.state_sequence_ok,
                "mission_source_sequence_ok": r.mission_source_sequence_ok,
                "expected_state_sequence": r.expected_state_sequence,
                "expected_mission_source_sequence": r.expected_mission_source_sequence,
                "observed_state_sequence": r.observed_state_sequence,
                "observed_mission_source_sequence": r.observed_mission_source_sequence,
            }
            for r in results
        ],
    }


def _print_fixed_report(
    scenario_name: str, scenario_file: str, results: list[StepResult], overall_pass: bool
) -> None:
    total_steps = len(results)
    passed_steps = sum(1 for r in results if r.success)
    failed_steps = total_steps - passed_steps
    print("\n=== CAMROD_SCENARIO_REPORT ===")
    print(f"SCENARIO_FILE={scenario_file}")
    print(f"SCENARIO_NAME={scenario_name}")
    print(f"OVERALL={'PASS' if overall_pass else 'FAIL'}")
    print(f"TOTAL_STEPS={total_steps}")
    print(f"PASSED_STEPS={passed_steps}")
    print(f"FAILED_STEPS={failed_steps}")
    for idx, result in enumerate(results, start=1):
        prefix = f"STEP_{idx:02d}"
        print(f"{prefix}_ID={result.step_id}")
        print(f"{prefix}_RESULT={'PASS' if result.success else 'FAIL'}")
        print(f"{prefix}_REASON={result.reason}")
        print(f"{prefix}_ELAPSED_SEC={result.elapsed_sec:.3f}")
        print(f"{prefix}_TIMEOUT_SEC={result.timeout_sec:.3f}")
        print(f"{prefix}_STATE_SEQUENCE_OK={'true' if result.state_sequence_ok else 'false'}")
        print(
            f"{prefix}_MISSION_SEQUENCE_OK="
            f"{'true' if result.mission_source_sequence_ok else 'false'}"
        )
    print("=== CAMROD_SCENARIO_REPORT_END ===")


def main(args=None) -> None:
    # Entrypoint for scenario leaderboard execution.
    rclpy.init(args=args)
    node = PlanningScenarioLeaderboard()
    results: list[StepResult] = []
    rc = 0
    scenario_name = "unknown_scenario"
    runner_exception: Exception | None = None
    try:
        if not node.setup_dynamic_path_subscriptions():
            raise RuntimeError("failed to subscribe path topics")
        scenario = node.load_scenario()
        scenario_name = str(scenario.get("name", "unnamed_scenario"))
        steps = scenario["steps"]
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
                f"points(g/l)=({result.global_path_points}/{result.local_path_points}) "
                f"state_seq_ok={result.state_sequence_ok} "
                f"mission_seq_ok={result.mission_source_sequence_ok}"
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
        runner_exception = exc
        node.get_logger().error(f"scenario runner failed: {exc}")
        if not results:
            results.append(
                StepResult(
                    step_id="__runner__",
                    success=False,
                    reason=f"runner_exception:{type(exc).__name__}",
                    elapsed_sec=0.0,
                    timeout_sec=0.0,
                    global_latency_sec=None,
                    local_latency_sec=None,
                    cmd_latency_sec=None,
                    global_path_points=0,
                    local_path_points=0,
                    expected_state_sequence=[],
                    expected_mission_source_sequence=[],
                    observed_state_sequence=[],
                    observed_mission_source_sequence=[],
                    state_sequence_ok=False,
                    mission_source_sequence_ok=False,
                )
            )
    finally:
        overall_pass = rc == 0 and runner_exception is None
        if node.print_fixed_report:
            _print_fixed_report(scenario_name, node.scenario_file, results, overall_pass)
        if node.report_file.strip():
            report_path = os.path.abspath(node.report_file.strip())
            report_dir = os.path.dirname(report_path)
            if report_dir:
                os.makedirs(report_dir, exist_ok=True)
            report_obj = _build_report_dict(
                scenario_name, node.scenario_file, results, overall_pass
            )
            with open(report_path, "w", encoding="utf-8") as f:
                json.dump(report_obj, f, indent=2, ensure_ascii=False)
            node.get_logger().info(f"scenario report saved: {report_path}")
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
