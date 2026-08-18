#!/usr/bin/env python3
"""Measure the outbound-motion barrier before a UI Return recall."""

from __future__ import annotations

import argparse
import json
import math
import platform
import threading
import time
import urllib.parse
import urllib.request
from pathlib import Path

import rclpy
from avg_msgs.msg import AvgPoseStamped, AvgServiceState, PlanningRecallRequest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path as NavPath
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def path_length(path: NavPath) -> float:
    return sum(
        math.hypot(
            right.pose.position.x - left.pose.position.x,
            right.pose.position.y - left.pose.position.y,
        )
        for left, right in zip(path.poses, path.poses[1:])
    )


class ReturnProbe(Node):
    """Collect only the topics required to prove the Return handoff order."""

    def __init__(self) -> None:
        super().__init__("manual_return_preemption_probe")
        self.lock = threading.Lock()
        self.commands: list[dict] = []
        self.states: list[dict] = []
        self.recalls: list[dict] = []
        self.paths: list[dict] = []
        self.poses: list[dict] = []
        self.create_subscription(Twist, "/control/cmd_vel_ros", self.on_cmd, 50)
        self.create_subscription(
            AvgServiceState, "/service/state", self.on_state, 20
        )
        self.create_subscription(
            PlanningRecallRequest,
            "/planning/state_machine/return_to_drop_zone",
            self.on_recall,
            20,
        )
        self.create_subscription(NavPath, "/planning/global_path", self.on_path, 20)
        pose_qos = QoSProfile(depth=1)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            AvgPoseStamped, "/localization/pose", self.on_pose, pose_qos
        )

    def on_cmd(self, message: Twist) -> None:
        now = time.monotonic()
        magnitude = math.sqrt(
            message.linear.x**2 + message.linear.y**2 + message.angular.z**2
        )
        with self.lock:
            self.commands.append(
                {
                    "time": now,
                    "x": message.linear.x,
                    "y": message.linear.y,
                    "yaw": message.angular.z,
                    "magnitude": magnitude,
                }
            )

    def on_state(self, message: AvgServiceState) -> None:
        with self.lock:
            self.states.append(
                {
                    "time": time.monotonic(),
                    "state": message.state,
                    "name": message.state_name,
                }
            )

    def on_recall(self, message: PlanningRecallRequest) -> None:
        with self.lock:
            self.recalls.append(
                {
                    "time": time.monotonic(),
                    "site": message.site_name,
                    "source": message.source,
                }
            )

    def on_path(self, message: NavPath) -> None:
        with self.lock:
            self.paths.append(
                {
                    "time": time.monotonic(),
                    "poses": len(message.poses),
                    "length_m": path_length(message),
                }
            )

    def on_pose(self, message: AvgPoseStamped) -> None:
        with self.lock:
            self.poses.append(
                {
                    "time": time.monotonic(),
                    "x": message.pose.position.x,
                    "y": message.pose.position.y,
                }
            )


class UiClient:
    """Minimal HTTP client for production UI endpoints."""

    def __init__(self, base_url: str) -> None:
        self.base_url = base_url.rstrip("/")

    def post(self, path: str, query: dict | None = None) -> dict:
        suffix = ""
        if query:
            suffix = "?" + urllib.parse.urlencode(query)
        request = urllib.request.Request(
            self.base_url + path + suffix,
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=5) as response:
            return json.load(response)


def wait_for_initial_pose(node: ReturnProbe, timeout_s: float) -> dict:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        with node.lock:
            if node.poses:
                return node.poses[-1]
        time.sleep(0.05)
    raise RuntimeError("initial localization pose was not observed")


def wait_for_outbound_motion(
    node: ReturnProbe,
    initial_pose: dict,
    minimum_displacement_m: float,
    timeout_s: float,
) -> tuple[dict, float]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        with node.lock:
            moving = [
                sample
                for sample in node.commands
                if math.hypot(sample["x"], sample["y"]) > 0.02
            ]
            latest_pose = node.poses[-1] if node.poses else initial_pose
        displacement = math.hypot(
            latest_pose["x"] - initial_pose["x"],
            latest_pose["y"] - initial_pose["y"],
        )
        if moving and displacement >= minimum_displacement_m:
            return moving[-1], displacement
        time.sleep(0.05)
    raise RuntimeError("outbound motion was not observed")


def collect_result(
    node: ReturnProbe,
    site: str,
    dispatch: dict,
    outbound_motion: dict,
    outbound_displacement_m: float,
    request_time: float,
    first: dict,
    second: dict,
    third: dict,
) -> dict:
    with node.lock:
        commands = [item for item in node.commands if item["time"] >= request_time]
        states = [item for item in node.states if item["time"] >= request_time]
        recalls = [item for item in node.recalls if item["time"] >= request_time]
        paths = [item for item in node.paths if item["time"] >= request_time]

    zero_samples = [item for item in commands if item["magnitude"] <= 1e-6]
    first_zero_delay = (
        zero_samples[0]["time"] - request_time if zero_samples else None
    )
    hold_samples = [
        item
        for item in commands
        if request_time + 0.10 <= item["time"] <= request_time + 0.45
    ]
    hold_max = max((item["magnitude"] for item in hold_samples), default=None)
    recall_delay = recalls[0]["time"] - request_time if recalls else None
    state_sequence: list[str] = []
    for state in states:
        label = state["name"] or str(state["state"])
        if not state_sequence or state_sequence[-1] != label:
            state_sequence.append(label)

    passed = bool(
        first.get("action") == "return_preempting"
        and second.get("action") == "return_preempting"
        and third.get("action") == "return_in_progress"
        and outbound_displacement_m >= 2.0
        and first_zero_delay is not None
        and first_zero_delay <= 0.20
        and hold_max is not None
        and hold_max <= 1e-6
        and len(recalls) == 1
        and recall_delay is not None
        and 0.45 <= recall_delay <= 0.75
        and any(
            item["state"] == AvgServiceState.RETURNING_TO_DROP_ZONE
            for item in states
        )
        and bool(paths)
    )
    return {
        "classification": "amd64_ros2_isolated_full_graph",
        "architecture": platform.machine(),
        "site": site,
        "dispatch_success": bool(dispatch.get("success")),
        "outbound_motion_observed": outbound_motion,
        "outbound_displacement_before_return_m": outbound_displacement_m,
        "first_return_action": first.get("action"),
        "second_return_action": second.get("action"),
        "third_return_action": third.get("action"),
        "first_zero_delay_s": first_zero_delay,
        "hold_window_s": [0.10, 0.45],
        "hold_window_samples": len(hold_samples),
        "hold_window_max_command": hold_max,
        "recall_count": len(recalls),
        "recall_delay_s": recall_delay,
        "recalls": recalls,
        "service_state_sequence": state_sequence,
        "return_paths": paths,
        "pass": passed,
        "limit": (
            "AMD64 simulation only; ARM64 timing and physical braking pending"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-url", default="http://127.0.0.1:18101")
    parser.add_argument("--site", default="B6")
    parser.add_argument("--minimum-displacement-m", type=float, default=2.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    # HH_260819 - Exercise the production endpoints and live ROS graph so the
    # evidence covers both visible buttons' shared authority, not a mock path.
    rclpy.init()
    node = ReturnProbe()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client = UiClient(args.base_url)
    result: dict | None = None
    try:
        initial_pose = wait_for_initial_pose(node, 10.0)
        dispatch = client.post(
            "/ui/destination", {"site": args.site, "run": "true"}
        )
        outbound_motion, displacement = wait_for_outbound_motion(
            node,
            initial_pose,
            args.minimum_displacement_m,
            45.0,
        )

        request_time = time.monotonic()
        first = client.post("/ui/manual_return")
        second = client.post("/ui/manual_return")
        time.sleep(0.80)
        third = client.post("/ui/manual_return")

        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            with node.lock:
                recall_ready = any(
                    item["time"] >= request_time for item in node.recalls
                )
                state_ready = any(
                    item["time"] >= request_time
                    and item["state"]
                    == AvgServiceState.RETURNING_TO_DROP_ZONE
                    for item in node.states
                )
                path_ready = any(
                    item["time"] >= request_time for item in node.paths
                )
            if recall_ready and state_ready and path_ready:
                break
            time.sleep(0.05)
        time.sleep(1.0)
        result = collect_result(
            node,
            args.site,
            dispatch,
            outbound_motion,
            displacement,
            request_time,
            first,
            second,
            third,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(result, indent=2))
    finally:
        try:
            client.post("/ui/stop")
            time.sleep(1.0)
        except Exception as error:  # pragma: no cover - cleanup on broken graph
            print(f"warning: UI stop cleanup failed: {error}")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)
    return 0 if result and result.get("pass") else 1


if __name__ == "__main__":
    raise SystemExit(main())
