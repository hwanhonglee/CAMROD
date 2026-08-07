#!/usr/bin/env python3
"""Record an end-to-end automatic route-boundary recovery simulation."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from avg_msgs.msg import (
    AvgBool,
    AvgOccupancyGrid,
    AvgPoseStamped,
    AvgTwist,
    ModuleState,
)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
import lanelet2
from lanelet2.io import Origin
from lanelet2.projection import LocalCartesianProjector
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


ORIGIN = Origin(36.8435737, 128.0925646, 0.0)
ROUTE_IDS = (754, 2751, 2720)
# HH_260807 - Reproduce the map-v17 B2 charger-departure curve without waiting
# for a complete campsite/return/parking cycle. This still drives the production
# RPP, safety gate and bounded recovery owner through their ROS interfaces.
B2_RECONTACT_ROUTE_IDS = (2751, 2720, 2744, 2690)
# HH_260806 - Keep the runtime probe explicit about the measured physical
# and planning rectangles being evaluated against the live lanelet raster.
BODY_EXTENTS = (0.70837, 0.68323, 0.53505, 0.53495)
PLANNING_EXTENTS = (0.80837, 0.78323, 0.63505, 0.63495)
# HH_260806 - Scan the live raster around this recorded route pose instead of
# assuming a cached bag's cell alignment is identical to the running grid.
STATIC_CONTACT_BASE_POSE = (
    10.441374066512223,
    35.65365899946396,
    -1.6302914988208081,
)
STATIC_CONTACT_SCENARIOS = {
    "margin_contact",
    "margin_recovery",
    "physical_contact",
    "physical_hard_stop",
}


def geometry_contract():
    """Bind every new evidence file to the geometry used by its runtime probe."""
    names = ("front", "rear", "left", "right")
    return {
        "physical_body_extents_m": dict(zip(names, BODY_EXTENTS)),
        "planning_boundary_extents_m": dict(zip(names, PLANNING_EXTENTS)),
        "planning_margin_m": {
            name: round(planning - body, 5)
            for name, planning, body in zip(names, PLANNING_EXTENTS, BODY_EXTENTS)
        },
    }


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


def load_lanelet_route(map_path, lanelet_ids, skip_points=5):
    lanelet_map = lanelet2.io.load(str(map_path), LocalCartesianProjector(ORIGIN))
    points = []
    for lanelet_id in lanelet_ids:
        current = [
            (point.x, point.y)
            for point in lanelet_map.laneletLayer[lanelet_id].centerline
        ]
        if points and math.dist(points[-1], current[0]) < 0.5:
            current = current[1:]
        points.extend(current)
    return resample(points)[skip_points:]


def load_route(map_path):
    return load_lanelet_route(map_path, ROUTE_IDS)


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
        self.latest_lanelet_grid = None
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
        self.route_hold_seen = False
        self.max_output_mps = 0.0
        self.max_recovery_output_mps = 0.0
        self.minimum_recovery_linear_x = 0.0
        self.maximum_recovery_abs_linear_y = 0.0
        self.maximum_recovery_abs_angular_z = 0.0

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
        self.raw_command_publisher = self.create_publisher(
            AvgTwist, "/control/cmd_vel_raw", 10
        )
        self.navigation_command_publisher = self.create_publisher(
            Twist, "/control/nav2_cmd_vel_ros", 10
        )
        self.create_subscription(
            AvgPoseStamped, "/localization/pose", self.on_pose, 20
        )
        # HH_260807 - Classify evidence against the same 0.10 m robot-centred
        # safety raster consumed by the final gate. The 0.25 m map display grid
        # is intentionally not authoritative for physical/planning contact.
        lanelet_grid_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            AvgOccupancyGrid,
            "/map/cost_grid/lanelet_safety",
            self.on_lanelet_grid,
            lanelet_grid_qos,
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

    def on_lanelet_grid(self, message):
        self.latest_lanelet_grid = message

    @staticmethod
    def point_in_polygon(x, y, polygon):
        inside = False
        previous = polygon[-1]
        for current in polygon:
            crosses = (current[1] > y) != (previous[1] > y)
            if crosses:
                edge_x = (
                    (previous[0] - current[0]) * (y - current[1])
                    / (previous[1] - current[1])
                    + current[0]
                )
                if x < edge_x:
                    inside = not inside
            previous = current
        return inside

    @staticmethod
    def world_to_grid(grid, world_x, world_y):
        origin_yaw = yaw_from_quaternion(grid.info.origin.orientation)
        dx = world_x - grid.info.origin.position.x
        dy = world_y - grid.info.origin.position.y
        local_x = math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy
        local_y = -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy
        grid_x = math.floor(local_x / grid.info.resolution)
        grid_y = math.floor(local_y / grid.info.resolution)
        if grid_x < 0 or grid_y < 0 or grid_x >= grid.info.width or grid_y >= grid.info.height:
            return None
        return int(grid_x), int(grid_y)

    @staticmethod
    def grid_to_world(grid, grid_x, grid_y):
        origin_yaw = yaw_from_quaternion(grid.info.origin.orientation)
        local_x = (grid_x + 0.5) * grid.info.resolution
        local_y = (grid_y + 0.5) * grid.info.resolution
        return (
            grid.info.origin.position.x
            + math.cos(origin_yaw) * local_x
            - math.sin(origin_yaw) * local_y,
            grid.info.origin.position.y
            + math.sin(origin_yaw) * local_x
            + math.cos(origin_yaw) * local_y,
        )

    def classify_boundary(self, pose, extents):
        grid = self.latest_lanelet_grid
        if grid is None or not grid.data:
            return {"available": False, "cost_100_contact": None}
        front, rear, left, right = extents
        local = ((front, left), (front, -right), (-rear, -right), (-rear, left))
        cosine = math.cos(pose["yaw_rad"])
        sine = math.sin(pose["yaw_rad"])
        polygon = [
            (
                pose["x"] + cosine * x - sine * y,
                pose["y"] + sine * x + cosine * y,
            )
            for x, y in local
        ]
        sampled_cells = set()
        edge_step = max(0.01, grid.info.resolution * 0.5)
        for start, end in zip(polygon, polygon[1:] + polygon[:1]):
            length = math.dist(start, end)
            steps = max(1, math.ceil(length / edge_step))
            for index in range(steps + 1):
                ratio = index / steps
                cell = self.world_to_grid(
                    grid,
                    start[0] + ratio * (end[0] - start[0]),
                    start[1] + ratio * (end[1] - start[1]),
                )
                if cell is not None:
                    sampled_cells.add(cell)

        grid_vertices = [self.world_to_grid(grid, x, y) for x, y in polygon]
        grid_vertices = [cell for cell in grid_vertices if cell is not None]
        if grid_vertices:
            min_x = max(0, min(cell[0] for cell in grid_vertices) - 1)
            max_x = min(grid.info.width - 1, max(cell[0] for cell in grid_vertices) + 1)
            min_y = max(0, min(cell[1] for cell in grid_vertices) - 1)
            max_y = min(grid.info.height - 1, max(cell[1] for cell in grid_vertices) + 1)
            for grid_y in range(min_y, max_y + 1):
                for grid_x in range(min_x, max_x + 1):
                    world = self.grid_to_world(grid, grid_x, grid_y)
                    if self.point_in_polygon(world[0], world[1], polygon):
                        sampled_cells.add((grid_x, grid_y))

        costs = [
            int(grid.data[grid_y * grid.info.width + grid_x])
            for grid_x, grid_y in sampled_cells
        ]
        return {
            "available": True,
            "cost_100_contact": any(cost >= 100 for cost in costs),
            "maximum_cost": max(costs, default=-1),
            "sampled_cells": len(costs),
            "extent_front_rear_left_right_m": list(extents),
        }

    def classify_physical_and_planning_boundaries(self, pose):
        return {
            "physical_body": self.classify_boundary(pose, BODY_EXTENTS),
            "planning_boundary": self.classify_boundary(pose, PLANNING_EXTENTS),
        }

    def scan_static_contact_pose(self, expected_physical_contact):
        """Select a robust contact pose from the currently published lanelet grid."""
        base_x, base_y, yaw = STATIC_CONTACT_BASE_POSE
        samples = []
        for index in range(-160, 161):
            lateral_offset = index * 0.01
            pose = {
                "x": base_x - math.sin(yaw) * lateral_offset,
                "y": base_y + math.cos(yaw) * lateral_offset,
                "yaw_rad": yaw,
            }
            classification = self.classify_physical_and_planning_boundaries(pose)
            samples.append({
                "offset_m": lateral_offset,
                "pose": pose,
                "physical_contact": classification["physical_body"][
                    "cost_100_contact"
                ],
                "planning_contact": classification["planning_boundary"][
                    "cost_100_contact"
                ],
            })

        margin_indices = [
            index
            for index, sample in enumerate(samples)
            if sample["physical_contact"] is False
            and sample["planning_contact"] is True
        ]
        margin_groups = []
        for index in margin_indices:
            if not margin_groups or index != margin_groups[-1][-1] + 1:
                margin_groups.append([index])
            else:
                margin_groups[-1].append(index)
        if not margin_groups:
            raise RuntimeError("live lanelet grid has no planning-margin-only band")

        positive_groups = [
            group for group in margin_groups if samples[group[len(group) // 2]]["offset_m"] > 0
        ]
        candidate_groups = positive_groups or margin_groups
        margin_group = min(
            candidate_groups,
            key=lambda group: abs(samples[group[len(group) // 2]]["offset_m"]),
        )
        if expected_physical_contact:
            margin_outer_index = margin_group[-1]
            physical_indices = [
                index
                for index in range(margin_outer_index + 1, len(samples))
                if samples[index]["physical_contact"] is True
                and samples[index]["planning_contact"] is True
            ]
            if not physical_indices:
                raise RuntimeError("live lanelet grid has no body-contact band")
            # Enter 5 cm into the body-contact band to avoid a cell-edge-only result.
            selected_index = physical_indices[min(5, len(physical_indices) - 1)]
        else:
            selected_index = margin_group[len(margin_group) // 2]

        selected = samples[selected_index]
        intervals = [
            {
                "start_offset_m": samples[group[0]]["offset_m"],
                "end_offset_m": samples[group[-1]]["offset_m"],
            }
            for group in margin_groups
        ]
        return selected["pose"], {
            "base_pose": {
                "x": base_x,
                "y": base_y,
                "yaw_rad": yaw,
            },
            "step_m": 0.01,
            "range_m": [-1.6, 1.6],
            "margin_only_intervals": intervals,
            "selected_lateral_offset_m": selected["offset_m"],
        }

    def on_gate(self, message):
        self.latest_gate = message
        if message.operating_state == "ROUTE_SAFETY_HOLD":
            self.route_hold_seen = True
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
            self.maximum_recovery_abs_angular_z = max(
                self.maximum_recovery_abs_angular_z, abs(message.angular.z)
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

    def wait_for_pose_settle(self, timeout_s=6.0, stable_s=0.6, tolerance_m=0.003):
        """Wait until the simulated localization pose stops carrying reset momentum."""
        deadline = time.monotonic() + timeout_s
        stable_since = None
        stable_anchor = None
        while rclpy.ok() and time.monotonic() < deadline:
            self.spin(0.10)
            if self.latest_pose is None:
                continue
            current = self.pose_snapshot()
            point = (current["x"], current["y"])
            if stable_anchor is None or math.dist(point, stable_anchor) > tolerance_m:
                stable_anchor = point
                stable_since = time.monotonic()
                continue
            if stable_since is not None and time.monotonic() - stable_since >= stable_s:
                return current
        raise RuntimeError("simulated pose did not settle after initialpose reset")

    def place_static_contact_pose(self):
        """Compensate deterministic simulator settling and reach the measured test pose."""
        desired = self.initial_pose
        commanded = desired
        attempts = []
        for attempt in range(3):
            self.initial_pose = commanded
            self.reset_pose()
            self.authorize(False)
            observed = self.wait_for_pose_settle()
            error_x = desired[0] - observed["x"]
            error_y = desired[1] - observed["y"]
            error_m = math.hypot(error_x, error_y)
            attempts.append({
                "attempt": attempt + 1,
                "commanded_pose": {
                    "x": commanded[0],
                    "y": commanded[1],
                    "yaw_rad": commanded[2],
                },
                "settled_pose": observed,
                "position_error_m": error_m,
            })
            self.append(
                "stage",
                name="static_pose_settled",
                attempt=attempt + 1,
                position_error_m=error_m,
                **observed,
            )
            if error_m <= 0.02:
                self.initial_pose = desired
                return observed, attempts
            # HH_260806 - Correct only the measured translation error. The
            # simulator preserves the requested yaw exactly during this reset.
            commanded = (
                commanded[0] + error_x,
                commanded[1] + error_y,
                desired[2],
            )
        self.initial_pose = desired
        raise RuntimeError(
            "static contact pose correction did not converge within 0.02 m"
        )

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
            # HH_260805 - A projected reverse-yaw arc is an active recovery
            # stage, not a stationary hold. Accept it and record its bounded
            # angular command so map-v15 runs can prove yaw actually changed.
            in {
                "CRAB_LEFT",
                "CRAB_RIGHT",
                "REVERSE",
                "REVERSE_YAW_LEFT",
                "REVERSE_YAW_RIGHT",
            }
        )

    def run(self, observe_retry=True):
        if not self.action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("FollowPath action unavailable")
        self.reset_pose()
        handle = self.start_goal()
        result_future = handle.get_result_async()
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
            # HH_260807 - A corrected route may finish before a second contact.
            # Preserve that as positive evidence instead of throwing away the
            # first-hold/recovery timeline because retry was not observed.
            deadline = time.monotonic() + 25.0
            while rclpy.ok() and time.monotonic() < deadline:
                self.spin(0.05, authorize=True)
                if self.gate_in_hold():
                    second_hold = self.pose_snapshot()
                    self.append("milestone", name="second_hold", **second_hold)
                    break
                if result_future.done():
                    self.append("milestone", name="route_completed_after_recovery")
                    break
        else:
            self.spin(1.0, authorize=True)

        if not result_future.done():
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
            "first_hold_boundary_classification": (
                self.classify_physical_and_planning_boundaries(first_hold)
            ),
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
            "maximum_recovery_abs_angular_z_radps": round(
                self.maximum_recovery_abs_angular_z, 4
            ),
            "final_output": {
                "linear_x": self.latest_output.linear.x,
                "linear_y": self.latest_output.linear.y,
                "angular_z": self.latest_output.angular.z,
            },
            "mission_completed": bool(
                result_future.done()
                and int(result_future.result().status)
                == GoalStatus.STATUS_SUCCEEDED
            ),
            "timeline": self.timeline,
        }

    def run_clear(self, timeout_s=120.0):
        """Verify that the production controller completes a known route without a hold."""
        if not self.action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("FollowPath action unavailable")
        self.reset_pose()
        start = self.pose_snapshot()
        handle = self.start_goal()
        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            self.spin(0.05, authorize=True)

        timed_out = not result_future.done()
        status = GoalStatus.STATUS_UNKNOWN
        if timed_out:
            cancel = handle.cancel_goal_async()
            while rclpy.ok() and not cancel.done():
                self.spin(0.05, authorize=True)
        else:
            status = int(result_future.result().status)

        end = self.pose_snapshot()
        self.authorize(False)
        self.spin(0.4)
        goal_x, goal_y = self.path_points[-1]
        displacement = math.hypot(end["x"] - start["x"], end["y"] - start["y"])
        goal_error = math.hypot(end["x"] - goal_x, end["y"] - goal_y)
        final_speed = math.hypot(
            self.latest_output.linear.x, self.latest_output.linear.y
        )
        passed = bool(
            not timed_out
            and status == GoalStatus.STATUS_SUCCEEDED
            and not self.route_hold_seen
            and displacement > 1.0
            and goal_error <= 0.5
            and final_speed <= 0.001
            and abs(self.latest_output.angular.z) <= 0.001
        )
        self.append(
            "milestone",
            name="route_clear_complete",
            passed=passed,
            status=status,
            timed_out=timed_out,
            route_hold_seen=self.route_hold_seen,
            displacement_m=displacement,
            goal_error_m=goal_error,
        )
        return {
            "route_lanelet_ids": list(self.lanelet_ids),
            "passed": passed,
            "timed_out": timed_out,
            "action_status": status,
            "route_safety_hold_seen": self.route_hold_seen,
            "start_pose": start,
            "end_pose": end,
            "displacement_m": round(displacement, 4),
            "goal_error_m": round(goal_error, 4),
            "maximum_final_output_mps": round(self.max_output_mps, 4),
            "final_output": {
                "linear_x": self.latest_output.linear.x,
                "linear_y": self.latest_output.linear.y,
                "angular_z": self.latest_output.angular.z,
            },
            "timeline": self.timeline,
        }

    def prepare_static_contact(self, expected_physical_contact):
        """Find, place, and classify one contact pose against the live grid."""
        self.wait_until(
            lambda: self.latest_lanelet_grid is not None,
            "lanelet_grid_available_before_scan",
            20.0,
        )
        selected_pose, live_sweep = self.scan_static_contact_pose(
            expected_physical_contact
        )
        self.initial_pose = (
            selected_pose["x"],
            selected_pose["y"],
            selected_pose["yaw_rad"],
        )
        pose, placement_attempts = self.place_static_contact_pose()
        pose = self.pose_snapshot()
        classification = self.classify_physical_and_planning_boundaries(pose)
        return pose, placement_attempts, live_sweep, classification

    def run_static_contact(self, expected_physical_contact):
        """Verify that a classified static contact cannot pass a drive command."""
        pose, placement_attempts, live_sweep, classification = (
            self.prepare_static_contact(expected_physical_contact)
        )

        # HH_260806 - Reset after pose convergence so the peak represents only
        # the challenged command window, not localization reset transients.
        self.max_output_mps = 0.0
        self.route_hold_seen = False
        command = AvgTwist()
        command.linear.x = 0.10
        deadline = time.monotonic() + 1.5
        while rclpy.ok() and time.monotonic() < deadline:
            self.authorize(True)
            self.raw_command_publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.04)

        challenged_gate = {
            "state": self.latest_gate.operating_state if self.latest_gate else None,
            "message": self.latest_gate.message if self.latest_gate else None,
        }
        peak_output = self.max_output_mps
        self.authorize(False)
        self.spin(0.4)
        final_speed = math.hypot(
            self.latest_output.linear.x, self.latest_output.linear.y
        )
        physical_contact = classification["physical_body"]["cost_100_contact"]
        planning_contact = classification["planning_boundary"]["cost_100_contact"]
        reason_is_footprint = "lanelet_footprint_cost" in (
            challenged_gate["message"] or ""
        )
        passed = bool(
            physical_contact is expected_physical_contact
            and planning_contact is True
            and self.route_hold_seen
            and challenged_gate["state"] == "ROUTE_SAFETY_HOLD"
            and reason_is_footprint
            and peak_output <= 0.001
            and final_speed <= 0.001
            and abs(self.latest_output.angular.z) <= 0.001
        )
        self.append(
            "milestone",
            name="static_contact_complete",
            passed=passed,
            expected_physical_contact=expected_physical_contact,
            peak_output_mps=peak_output,
        )
        return {
            "passed": passed,
            "expected_physical_contact": expected_physical_contact,
            "live_lateral_sweep": live_sweep,
            "placement_attempts": placement_attempts,
            "tested_pose": pose,
            "boundary_classification": classification,
            "challenged_command": {
                "linear_x": command.linear.x,
                "linear_y": command.linear.y,
                "angular_z": command.angular.z,
            },
            "route_safety_hold_seen": self.route_hold_seen,
            "challenged_gate": challenged_gate,
            "maximum_final_output_mps_during_challenge": round(peak_output, 4),
            "final_output": {
                "linear_x": self.latest_output.linear.x,
                "linear_y": self.latest_output.linear.y,
                "angular_z": self.latest_output.angular.z,
            },
            "timeline": self.timeline,
        }

    def run_margin_recovery(self):
        """Verify that a navigation-triggered margin contact moves and releases."""
        pose, placement_attempts, live_sweep, classification = (
            self.prepare_static_contact(expected_physical_contact=False)
        )
        self.route_hold_seen = False
        self.max_recovery_output_mps = 0.0
        self.minimum_recovery_linear_x = 0.0
        self.maximum_recovery_abs_linear_y = 0.0
        self.maximum_recovery_abs_angular_z = 0.0

        command = Twist()
        command.linear.x = 0.10
        deadline = time.monotonic() + 5.0
        first_hold = None
        while rclpy.ok() and time.monotonic() < deadline:
            self.authorize(True)
            self.navigation_command_publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.04)
            if self.gate_in_hold():
                first_hold = self.pose_snapshot()
                self.append("milestone", name="first_hold", **first_hold)
                break
        if first_hold is None:
            raise RuntimeError("margin contact did not trigger route safety hold")
        first_hold_gate = {
            "state": self.latest_gate.operating_state if self.latest_gate else None,
            "message": self.latest_gate.message if self.latest_gate else None,
        }

        recovery_start = self.wait_until(
            self.owner_moving, "automatic_recovery_started", 3.0
        )
        recovery_motion = self.latest_owner.operating_state
        release = self.wait_until(
            lambda: not self.gate_in_hold(), "hold_released", 12.0
        )
        release_classification = self.classify_physical_and_planning_boundaries(
            release
        )
        self.authorize(False)
        self.spin(0.4)
        displacement = math.hypot(
            release["x"] - first_hold["x"],
            release["y"] - first_hold["y"],
        )
        final_speed = math.hypot(
            self.latest_output.linear.x, self.latest_output.linear.y
        )
        passed = bool(
            classification["physical_body"]["cost_100_contact"] is False
            and classification["planning_boundary"]["cost_100_contact"] is True
            and self.route_hold_seen
            and recovery_motion
            in {
                "CRAB_LEFT",
                "CRAB_RIGHT",
                "REVERSE",
                "REVERSE_YAW_LEFT",
                "REVERSE_YAW_RIGHT",
            }
            and displacement >= 0.02
            and self.max_recovery_output_mps <= 0.101
            and final_speed <= 0.001
            and abs(self.latest_output.angular.z) <= 0.001
        )
        return {
            "passed": passed,
            "live_lateral_sweep": live_sweep,
            "placement_attempts": placement_attempts,
            "tested_pose": pose,
            "first_hold_boundary_classification": classification,
            "first_hold_gate": first_hold_gate,
            "challenged_command": {
                "linear_x": command.linear.x,
                "linear_y": command.linear.y,
                "angular_z": command.angular.z,
            },
            "automatic_recovery_motion": recovery_motion,
            "recovery_start": recovery_start,
            "hold_release": release,
            "hold_release_boundary_classification": release_classification,
            "recovery_displacement_m": round(displacement, 4),
            "maximum_recovery_output_mps": round(
                self.max_recovery_output_mps, 4
            ),
            "minimum_recovery_linear_x_mps": round(
                self.minimum_recovery_linear_x, 4
            ),
            "maximum_recovery_abs_linear_y_mps": round(
                self.maximum_recovery_abs_linear_y, 4
            ),
            "maximum_recovery_abs_angular_z_radps": round(
                self.maximum_recovery_abs_angular_z, 4
            ),
            "final_output": {
                "linear_x": self.latest_output.linear.x,
                "linear_y": self.latest_output.linear.y,
                "angular_z": self.latest_output.angular.z,
            },
            "timeline": self.timeline,
        }

    def run_physical_hard_stop(self):
        """Verify that navigation cannot turn a physical-body contact into recovery."""
        pose, placement_attempts, live_sweep, classification = (
            self.prepare_static_contact(expected_physical_contact=True)
        )
        self.route_hold_seen = False
        self.max_recovery_output_mps = 0.0
        command = Twist()
        command.linear.x = 0.10
        deadline = time.monotonic() + 5.0
        first_hold = None
        while rclpy.ok() and time.monotonic() < deadline:
            self.authorize(True)
            self.navigation_command_publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.04)
            if self.gate_in_hold():
                first_hold = self.pose_snapshot()
                self.append("milestone", name="first_hold", **first_hold)
                break
        if first_hold is None:
            raise RuntimeError("physical contact did not trigger route safety hold")
        first_hold_gate = {
            "state": self.latest_gate.operating_state if self.latest_gate else None,
            "message": self.latest_gate.message if self.latest_gate else None,
        }

        observation_start = self.relative_time()
        self.spin(2.5, authorize=True)
        observed_pose = self.pose_snapshot()
        owner_motion_seen = any(
            event.get("event") == "owner"
            and event.get("t", 0.0) >= observation_start
            and event.get("state")
            in {
                "CRAB_LEFT",
                "CRAB_RIGHT",
                "REVERSE",
                "REVERSE_YAW_LEFT",
                "REVERSE_YAW_RIGHT",
            }
            for event in self.timeline
        )
        displacement = math.hypot(
            observed_pose["x"] - first_hold["x"],
            observed_pose["y"] - first_hold["y"],
        )
        hold_retained = self.gate_in_hold()
        self.authorize(False)
        self.spin(0.4)
        final_speed = math.hypot(
            self.latest_output.linear.x, self.latest_output.linear.y
        )
        passed = bool(
            classification["physical_body"]["cost_100_contact"] is True
            and classification["planning_boundary"]["cost_100_contact"] is True
            and self.route_hold_seen
            and not owner_motion_seen
            and self.max_recovery_output_mps <= 0.001
            and displacement <= 0.01
            and hold_retained
            and final_speed <= 0.001
            and abs(self.latest_output.angular.z) <= 0.001
        )
        return {
            "passed": passed,
            "live_lateral_sweep": live_sweep,
            "placement_attempts": placement_attempts,
            "tested_pose": pose,
            "first_hold_boundary_classification": classification,
            "first_hold_gate": first_hold_gate,
            "challenged_command": {
                "linear_x": command.linear.x,
                "linear_y": command.linear.y,
                "angular_z": command.angular.z,
            },
            "owner_motion_seen": owner_motion_seen,
            "hold_retained_during_observation": hold_retained,
            "observed_displacement_m": round(displacement, 4),
            "maximum_recovery_output_mps": round(
                self.max_recovery_output_mps, 4
            ),
            "final_output": {
                "linear_x": self.latest_output.linear.x,
                "linear_y": self.latest_output.linear.y,
                "angular_z": self.latest_output.angular.z,
            },
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
        choices=(
            "route_clear",
            "route_retry",
            "b2_recontact",
            "static_reverse_retry",
            "one_sided_crab",
            "margin_contact",
            "margin_recovery",
            "physical_contact",
            "physical_hard_stop",
        ),
        default="route_retry",
    )
    args = parser.parse_args()

    rclpy.init()
    initial_pose = None
    if args.scenario in STATIC_CONTACT_SCENARIOS:
        path_points = load_route(args.map)
        initial_pose = STATIC_CONTACT_BASE_POSE
    elif args.scenario in {
        "route_clear",
        "route_retry",
        "b2_recontact",
        "static_reverse_retry",
    }:
        if args.scenario == "b2_recontact":
            path_points = load_lanelet_route(
                args.map, B2_RECONTACT_ROUTE_IDS, skip_points=0
            )
        else:
            path_points = load_route(args.map)
        if args.scenario == "static_reverse_retry":
            # HH_260803 - Start from the observed route-contact pose with zero
            # platform velocity so the validated reverse output is observable,
            # rather than letting residual forward deceleration clear the hold.
            initial_pose = (3.93, 45.245, math.radians(-12.5))
    else:
        path_points, initial_pose = load_one_sided_crab_route(args.map)
    lanelet_ids = (
        B2_RECONTACT_ROUTE_IDS
        if args.scenario == "b2_recontact"
        else ROUTE_IDS
        if args.scenario != "one_sided_crab"
        else (4677,)
    )
    node = AutomaticRecoveryProbe(
        path_points,
        initial_pose=initial_pose,
        lanelet_ids=lanelet_ids,
    )
    try:
        try:
            if args.scenario == "route_clear":
                # HH_260806 - The reduced boundary must prove that the previously
                # blocked production route now completes without a safety hold.
                result = node.run_clear()
            elif args.scenario == "margin_recovery":
                result = node.run_margin_recovery()
            elif args.scenario == "physical_hard_stop":
                result = node.run_physical_hard_stop()
            elif args.scenario in STATIC_CONTACT_SCENARIOS:
                result = node.run_static_contact(
                    expected_physical_contact=args.scenario == "physical_contact"
                )
            else:
                result = node.run(observe_retry=args.scenario != "one_sided_crab")
        except Exception as error:
            # HH_260807 - Timeout and safety-latch runs are required evidence,
            # not disposable console output. Preserve the last gate/owner state
            # and timeline before returning a non-zero process status.
            result = {
                "passed": False,
                "error": str(error),
                "last_gate_state": (
                    node.latest_gate.operating_state if node.latest_gate else None
                ),
                "last_gate_message": (
                    node.latest_gate.message if node.latest_gate else None
                ),
                "last_owner_state": (
                    node.latest_owner.operating_state if node.latest_owner else None
                ),
                "last_owner_message": (
                    node.latest_owner.message if node.latest_owner else None
                ),
                "timeline": node.timeline,
            }
            result["scenario"] = args.scenario
            result["map"] = map_metadata(args.map)
            result["geometry"] = geometry_contract()
            result["captured_at_utc"] = datetime.now(timezone.utc).isoformat()
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("w", encoding="utf-8") as stream:
                json.dump(result, stream, indent=2)
            raise
        result["scenario"] = args.scenario
        # HH_260804 - Bind evidence to the exact user-provided map revision;
        # regenerated images must not silently reuse an older map result.
        result["map"] = map_metadata(args.map)
        result["geometry"] = geometry_contract()
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
