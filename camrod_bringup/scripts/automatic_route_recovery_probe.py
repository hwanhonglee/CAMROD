#!/usr/bin/env python3
"""Record an end-to-end automatic route-boundary recovery simulation."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import lanelet2
import rclpy
from avg_msgs.msg import AvgBool, AvgPoseStamped, AvgTwist, ModuleState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lanelet2.io import Origin
from lanelet2.projection import LocalCartesianProjector
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node


ORIGIN = Origin(36.8435737, 128.0925646, 0.0)
ROUTE_IDS = (754, 2751, 2720)


def map_metadata(map_path):
    """Return source identity recorded with every simulation result."""
    root = ET.parse(map_path).getroot()
    meta = root.find("MetaInfo")
    return {
        "source_file": map_path.name,
        "map_version": int(meta.attrib["map_version"]) if meta is not None else None,
        "sha256": hashlib.sha256(map_path.read_bytes()).hexdigest(),
    }


def quaternion_z_w(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def yaw_from_quaternion(orientation):
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def angular_delta(first, second):
    return math.atan2(math.sin(second - first), math.cos(second - first))


def resample(points, step=0.10):
    output = []
    for start, end in zip(points, points[1:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        count = max(1, math.ceil(math.hypot(dx, dy) / step))
        for index in range(count):
            ratio = index / count
            point = (start[0] + ratio * dx, start[1] + ratio * dy)
            if not output or math.dist(point, output[-1]) > 0.02:
                output.append(point)
    output.append(points[-1])
    return output


def load_route(map_path):
    lanelet_map = lanelet2.io.load(str(map_path), LocalCartesianProjector(ORIGIN))
    points = []
    for lanelet_id in ROUTE_IDS:
        current = [
            (point.x, point.y)
            for point in lanelet_map.laneletLayer[lanelet_id].centerline
        ]
        if points and math.dist(points[-1], current[0]) < 0.5:
            current = current[1:]
        points.extend(current)
    return resample(points)[5:]


def load_one_sided_crab_route(map_path):
    """Create a controlled right-edge contact with only left crab projected clear."""
    lanelet_map = lanelet2.io.load(str(map_path), LocalCartesianProjector(ORIGIN))
    centerline = [
        (point.x, point.y)
        for point in lanelet_map.laneletLayer[4677].centerline
    ][6:]
    # HH_260803 - This offset was selected against the actual 0.25 m raster
    # cost grid, not only the vector polygon: current/right are blocked and
    # the projected full footprint 0.25 m left is clear.
    offset = -0.40
    x, y = centerline[0]
    yaw = math.atan2(
        centerline[1][1] - centerline[0][1],
        centerline[1][0] - centerline[0][0],
    )
    initial_pose = (
        x - math.sin(yaw) * offset,
        y + math.cos(yaw) * offset,
        yaw,
    )
    # Only the robot starts at the boundary. Keeping the requested path on the
    # centerline lets full-footprint and saved front-path evidence both clear.
    return resample(centerline), initial_pose


class AutomaticRecoveryProbe(Node):
    """Drive one fixed route and observe the production recovery owner."""

    def __init__(self, path_points, initial_pose=None, lanelet_ids=ROUTE_IDS):
        super().__init__("automatic_route_recovery_probe")
        self.path_points = path_points
        self.initial_pose = initial_pose
        self.lanelet_ids = tuple(lanelet_ids)
        self.started = time.monotonic()
        self.latest_pose = None
        self.latest_gate = None
        self.latest_owner = None
        self.latest_candidate = AvgTwist()
        self.latest_output = AvgTwist()
        self.timeline = []
        self.last_pose_sample = None
        self.last_gate_signature = None
        self.last_owner_signature = None
        self.last_candidate_signature = None
        self.last_output_signature = None
        self.max_output_mps = 0.0
        self.max_recovery_output_mps = 0.0
        self.minimum_recovery_linear_x = 0.0
        self.maximum_recovery_abs_linear_y = 0.0

        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.engage_publisher = self.create_publisher(
            AvgBool, "/planning/engage", 10
        )
        self.mission_engage_publisher = self.create_publisher(
            AvgBool, "/planning/mission_engage", 10
        )
        self.drive_enable_publisher = self.create_publisher(
            AvgBool, "/platform/drive_enable", 10
        )
        self.create_subscription(
            AvgPoseStamped, "/localization/pose", self.on_pose, 20
        )
        self.create_subscription(
            ModuleState,
            "/control/cmd_vel_safety_gate/status",
            self.on_gate,
            20,
        )
        self.create_subscription(
            ModuleState,
            "/control/route_safety_recovery_controller/status",
            self.on_owner,
            20,
        )
        self.create_subscription(
            AvgTwist,
            "/control/route_safety_recovery/candidate",
            self.on_candidate,
            20,
        )
        self.create_subscription(AvgTwist, "/control/cmd_vel", self.on_output, 20)
        self.action = ActionClient(self, FollowPath, "/planning/follow_path")

    def relative_time(self):
        return time.monotonic() - self.started

    def append(self, event, **values):
        self.timeline.append({
            "t": round(self.relative_time(), 3),
            "event": event,
            **values,
        })

    def on_pose(self, message):
        self.latest_pose = message
        point = (message.pose.position.x, message.pose.position.y)
        if self.last_pose_sample is None or math.dist(point, self.last_pose_sample) >= 0.01:
            yaw = yaw_from_quaternion(message.pose.orientation)
            self.append(
                "pose",
                x=point[0],
                y=point[1],
                yaw_rad=yaw,
                yaw_deg=math.degrees(yaw),
            )
            self.last_pose_sample = point

    def on_gate(self, message):
        self.latest_gate = message
        signature = (message.operating_state, message.message)
        if signature != self.last_gate_signature:
            self.append(
                "gate",
                state=message.operating_state,
                message=message.message,
            )
            self.last_gate_signature = signature

    def on_owner(self, message):
        self.latest_owner = message
        signature = (message.operating_state, message.message)
        if signature != self.last_owner_signature:
            self.append(
                "owner",
                state=message.operating_state,
                message=message.message,
            )
            self.last_owner_signature = signature

    def on_candidate(self, message):
        self.latest_candidate = message
        signature = (
            round(message.linear.x, 4),
            round(message.linear.y, 4),
            round(message.angular.z, 4),
        )
        if signature != self.last_candidate_signature:
            self.append(
                "candidate",
                linear_x=message.linear.x,
                linear_y=message.linear.y,
                angular_z=message.angular.z,
            )
            self.last_candidate_signature = signature

    def on_output(self, message):
        self.latest_output = message
        speed = math.hypot(message.linear.x, message.linear.y)
        self.max_output_mps = max(self.max_output_mps, speed)
        if self.gate_in_hold():
            self.max_recovery_output_mps = max(self.max_recovery_output_mps, speed)
            self.minimum_recovery_linear_x = min(
                self.minimum_recovery_linear_x, message.linear.x
            )
            self.maximum_recovery_abs_linear_y = max(
                self.maximum_recovery_abs_linear_y, abs(message.linear.y)
            )
        signature = (
            round(message.linear.x, 3),
            round(message.linear.y, 3),
            round(message.angular.z, 3),
        )
        if signature != self.last_output_signature:
            self.append(
                "output",
                linear_x=message.linear.x,
                linear_y=message.linear.y,
                angular_z=message.angular.z,
            )
            self.last_output_signature = signature

    def authorize(self, enabled=True):
        engage = AvgBool()
        engage.data = enabled
        mission_engage = AvgBool()
        mission_engage.data = False
        drive = AvgBool()
        drive.data = enabled
        self.engage_publisher.publish(engage)
        self.mission_engage_publisher.publish(mission_engage)
        self.drive_enable_publisher.publish(drive)

    def spin(self, duration, authorize=False):
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            if authorize:
                self.authorize(True)
            rclpy.spin_once(self, timeout_sec=0.04)

    def reset_pose(self):
        if self.initial_pose is None:
            x, y = self.path_points[0]
            next_x, next_y = self.path_points[1]
            yaw = math.atan2(next_y - y, next_x - x)
        else:
            x, y, yaw = self.initial_pose
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = (
            quaternion_z_w(yaw)
        )
        for _ in range(6):
            message.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_publisher.publish(message)
            self.spin(0.12)
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            self.spin(0.10, authorize=True)
            if self.latest_pose and math.hypot(
                self.latest_pose.pose.position.x - x,
                self.latest_pose.pose.position.y - y,
            ) < 0.75:
                self.append("stage", name="pose_reset")
                return
        raise RuntimeError("initial pose reset did not converge")

    def make_goal(self):
        goal = FollowPath.Goal()
        goal.controller_id = "RPP"
        goal.goal_checker_id = "goal_checker"
        goal.path = NavPath()
        goal.path.header.frame_id = "map"
        goal.path.header.stamp = self.get_clock().now().to_msg()
        for index, (x, y) in enumerate(self.path_points):
            if index + 1 < len(self.path_points):
                next_x, next_y = self.path_points[index + 1]
            else:
                previous_x, previous_y = self.path_points[index - 1]
                next_x, next_y = 2 * x - previous_x, 2 * y - previous_y
            pose = PoseStamped()
            pose.header = goal.path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z, pose.pose.orientation.w = quaternion_z_w(
                math.atan2(next_y - y, next_x - x)
            )
            goal.path.poses.append(pose)
        return goal

    def start_goal(self):
        future = self.action.send_goal_async(self.make_goal())
        while rclpy.ok() and not future.done():
            self.spin(0.05, authorize=True)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("FollowPath goal rejected")
        self.append("stage", name="follow_path_started")
        return handle

    def pose_snapshot(self):
        if not self.latest_pose:
            raise RuntimeError("pose unavailable")
        return {
            "x": self.latest_pose.pose.position.x,
            "y": self.latest_pose.pose.position.y,
            "yaw_rad": yaw_from_quaternion(self.latest_pose.pose.orientation),
        }

    def wait_until(self, predicate, label, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.spin(0.05, authorize=True)
            if predicate():
                snapshot = self.pose_snapshot()
                self.append("milestone", name=label, **snapshot)
                return snapshot
        gate = self.latest_gate.message if self.latest_gate else "unavailable"
        owner = self.latest_owner.message if self.latest_owner else "unavailable"
        raise RuntimeError(
            f"timeout waiting for {label}; gate={gate}; owner={owner}"
        )

    def gate_in_hold(self):
        return bool(
            self.latest_gate
            and self.latest_gate.operating_state == "ROUTE_SAFETY_HOLD"
        )

    def owner_moving(self):
        return bool(
            self.latest_owner
            and self.latest_owner.operating_state
            in {"CRAB_LEFT", "CRAB_RIGHT", "REVERSE"}
        )

    def run(self, observe_retry=True):
        if not self.action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("FollowPath action unavailable")
        self.reset_pose()
        handle = self.start_goal()
        first_hold = self.wait_until(self.gate_in_hold, "first_hold", 45.0)
        recovery_start = self.wait_until(
            self.owner_moving, "automatic_recovery_started", 3.0
        )
        recovery_motion = self.latest_owner.operating_state
        release = self.wait_until(
            lambda: not self.gate_in_hold(), "hold_released", 12.0
        )
        second_hold = None
        if observe_retry:
            second_hold = self.wait_until(self.gate_in_hold, "second_hold", 25.0)
        else:
            self.spin(1.0, authorize=True)

        cancel = handle.cancel_goal_async()
        while rclpy.ok() and not cancel.done():
            self.spin(0.05, authorize=True)
        self.authorize(False)
        self.spin(0.4)

        recovery_distance = math.hypot(
            release["x"] - first_hold["x"], release["y"] - first_hold["y"]
        )
        retry_distance = None
        retry_yaw_delta = None
        if second_hold:
            retry_distance = math.hypot(
                second_hold["x"] - release["x"],
                second_hold["y"] - release["y"],
            )
            retry_yaw_delta = math.degrees(
                angular_delta(release["yaw_rad"], second_hold["yaw_rad"])
            )
        release_time = milestone_time(self.timeline, "hold_released")
        retry_latched = second_hold is not None and any(
            event.get("event") == "gate"
            and event.get("t", 0.0) >= release_time
            and any(
                marker in event.get("message", "")
                for marker in (
                    "route_safety_retry_latched",
                    "rapid_recontact_latched",
                    "rapid route recontact latched",
                )
            )
            for event in self.timeline
        )
        return {
            "route_lanelet_ids": list(self.lanelet_ids),
            "automatic_recovery_motion": recovery_motion,
            "first_hold": first_hold,
            "recovery_start": recovery_start,
            "hold_release": release,
            "second_hold": second_hold,
            "recovery_displacement_m": round(recovery_distance, 4),
            "same_goal_retry_displacement_m": (
                round(retry_distance, 4) if retry_distance is not None else None
            ),
            "same_goal_retry_yaw_delta_deg": (
                round(retry_yaw_delta, 4) if retry_yaw_delta is not None else None
            ),
            "rapid_recontact_after_release_s": (
                round(
                    milestone_time(self.timeline, "second_hold")
                    - release_time,
                    4,
                )
                if second_hold is not None
                else None
            ),
            "rapid_recontact_latched": retry_latched,
            "maximum_final_output_mps": round(self.max_output_mps, 4),
            "maximum_recovery_output_mps": round(
                self.max_recovery_output_mps, 4
            ),
            "minimum_recovery_linear_x_mps": round(
                self.minimum_recovery_linear_x, 4
            ),
            "maximum_recovery_abs_linear_y_mps": round(
                self.maximum_recovery_abs_linear_y, 4
            ),
            "final_output": {
                "linear_x": self.latest_output.linear.x,
                "linear_y": self.latest_output.linear.y,
                "angular_z": self.latest_output.angular.z,
            },
            "mission_completed": False,
            "timeline": self.timeline,
        }


def milestone_time(timeline, name):
    return next(
        event["t"]
        for event in timeline
        if event.get("event") == "milestone" and event.get("name") == name
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--scenario",
        choices=("route_retry", "static_reverse_retry", "one_sided_crab"),
        default="route_retry",
    )
    args = parser.parse_args()

    rclpy.init()
    initial_pose = None
    if args.scenario in {"route_retry", "static_reverse_retry"}:
        path_points = load_route(args.map)
        if args.scenario == "static_reverse_retry":
            # HH_260803 - Start from the observed route-contact pose with zero
            # platform velocity so the validated reverse output is observable,
            # rather than letting residual forward deceleration clear the hold.
            initial_pose = (3.93, 45.245, math.radians(-12.5))
    else:
        path_points, initial_pose = load_one_sided_crab_route(args.map)
    lanelet_ids = ROUTE_IDS if args.scenario != "one_sided_crab" else (4677,)
    node = AutomaticRecoveryProbe(
        path_points,
        initial_pose=initial_pose,
        lanelet_ids=lanelet_ids,
    )
    try:
        result = node.run(observe_retry=args.scenario != "one_sided_crab")
        result["scenario"] = args.scenario
        # HH_260804 - Bind evidence to the exact user-provided map revision;
        # regenerated images must not silently reuse an older map result.
        result["map"] = map_metadata(args.map)
        result["captured_at_utc"] = datetime.now(timezone.utc).isoformat()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2)
        print(json.dumps({k: v for k, v in result.items() if k != "timeline"}, indent=2))
    finally:
        node.authorize(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
