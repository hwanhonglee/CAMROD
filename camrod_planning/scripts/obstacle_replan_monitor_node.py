#!/usr/bin/env python3
"""Monitor dynamic obstacle blockage on the active path."""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from avg_msgs.msg import AvgOccupancyGrid, AvgPath, AvgPoseStamped, AvgString
from geometry_msgs.msg import PoseStamped as RosPoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from std_msgs.msg import String as RosString


@dataclass
class GridRecord:
    topic: str
    received_time: Time
    serialized_grid: Optional[bytes]
    grid: Optional[AvgOccupancyGrid] = None
    transform_ready: bool = False
    valid: bool = False
    width: int = 0
    height: int = 0
    resolution: float = 0.0
    origin_x: float = 0.0
    origin_y: float = 0.0
    inverse_cos: float = 1.0
    inverse_sin: float = 0.0


@dataclass
class BlockageSample:
    blocked: bool
    blocked_count: int
    total_count: int
    max_cost: int
    source_topic: str
    point_x: float
    point_y: float


@dataclass
class LaneWidthSample:
    allowed: bool
    total_width_m: float
    left_clearance_m: float
    right_clearance_m: float
    reason: str


class ObstacleReplanMonitor(Node):
    """Monitors dynamic obstacle grids and optionally preempts Nav2."""

    def __init__(self) -> None:
        super().__init__("obstacle_replan_monitor")

        self._enabled = bool(self.declare_parameter("enabled", True).value)
        # HH_260702 - Monitor the same active local path used by cmd_vel_gate.
        # Monitoring only the global route can miss dynamic costs on the path
        # that the controller/gate is actually evaluating.
        self._path_topic = str(
            self.declare_parameter("path_topic", "/planning/local_path").value
        ).strip() or "/planning/local_path"
        self._goal_topic = str(
            self.declare_parameter("goal_topic", "/planning/goal_pose_snapped").value
        ).strip()
        self._pose_topic = str(
            self.declare_parameter("pose_topic", "/localization/pose").value
        ).strip()
        self._navigate_action_name = str(
            self.declare_parameter("navigate_action_name", "/planning/navigate_to_pose").value
        ).strip()
        self._navigate_status_topic = str(
            self.declare_parameter(
                "navigate_status_topic", "/planning/navigate_to_pose/_action/status"
            ).value
        ).strip()
        self._planner_selector_topic = str(
            # HH_260720 - Publish std_msgs only to the explicit Nav2 selector boundary.
            self.declare_parameter(
                "planner_selector_topic", "/planning/planner_selector_ros"
            ).value
        ).strip()
        self._status_topic = str(
            self.declare_parameter("status_topic", "/planning/obstacle_replan/status").value
        ).strip()
        self._lanelet_grid_topic = str(
            self.declare_parameter(
                "lanelet_grid_topic", "/map/cost_grid/lanelet"
            ).value
        ).strip()

        self._dynamic_grid_topics = [
            str(topic).strip()
            for topic in self.declare_parameter(
                "dynamic_cost_grid_topics",
                ["/sensing/cost_grid/lidar", "/sensing/cost_grid/radar"],
            ).value
            if str(topic).strip()
        ]

        self._fallback_planner_id = str(
            self.declare_parameter("fallback_planner_id", "SmacLattice").value
        ).strip()
        # HH_260702 - Keep global route geometry stable by default. Dynamic
        # obstacle blockage is reported for diagnostics/gates, while explicit
        # fallback preemption is opt-in for controlled experiments.
        self._preempt_enabled = bool(self.declare_parameter("preempt_enabled", False).value)
        # HH_260805 - Free-space fallback is permitted only where the current
        # lanelet grid proves enough lateral room. Missing/stale geometry denies
        # preemption and leaves the existing safety stop in authority.
        self._wide_lane_replan_enabled = bool(
            self.declare_parameter("wide_lane_replan_enabled", True).value
        )
        self._lanelet_grid_max_age_s = float(
            self.declare_parameter("lanelet_grid_max_age_s", 2.5).value
        )
        self._lanelet_blocked_cost_threshold = int(
            self.declare_parameter("lanelet_blocked_cost_threshold", 100).value
        )
        self._minimum_replan_lane_width_m = float(
            self.declare_parameter("minimum_replan_lane_width_m", 2.5).value
        )
        self._minimum_side_clearance_m = float(
            self.declare_parameter("minimum_side_clearance_m", 0.60).value
        )
        self._lane_width_max_search_m = float(
            self.declare_parameter("lane_width_max_search_m", 6.0).value
        )
        self._restore_planner_id = str(
            self.declare_parameter("restore_planner_id", "LaneletRoute").value
        ).strip()
        self._monitor_rate_hz = float(self.declare_parameter("monitor_rate_hz", 5.0).value)
        self._grid_max_age_s = float(self.declare_parameter("grid_max_age_s", 0.75).value)
        self._lookahead_m = float(self.declare_parameter("lookahead_m", 8.0).value)
        self._sample_start_distance_m = float(
            self.declare_parameter("sample_start_distance_m", 0.8).value
        )
        self._sample_step_m = float(self.declare_parameter("sample_step_m", 0.25).value)
        self._corridor_half_width_m = float(
            self.declare_parameter("corridor_half_width_m", 0.55).value
        )
        self._lateral_step_m = float(self.declare_parameter("lateral_step_m", 0.25).value)
        self._obstacle_cost_threshold = int(
            self.declare_parameter("obstacle_cost_threshold", 60).value
        )
        self._min_blocked_samples = int(
            self.declare_parameter("min_blocked_samples", 5).value
        )
        self._blocked_sample_ratio = float(
            self.declare_parameter("blocked_sample_ratio", 0.08).value
        )
        # HH_260805 - Require a sustained blockage before planner preemption;
        # command-level obstacle stopping remains owned by cmd_vel_safety_gate.
        self._block_hold_s = float(self.declare_parameter("block_hold_s", 20.0).value)
        # HH_260619 - Dynamic grids can alternate between empty frames and an
        # obstacle frame on the same topic in sim/test. Keep the blockage latched
        # briefly so a single empty grid cannot break the persistent-block timer.
        self._clear_hold_s = float(self.declare_parameter("clear_hold_s", 1.0).value)
        self._replan_cooldown_s = float(
            self.declare_parameter("replan_cooldown_s", 10.0).value
        )
        self._selector_override_s = float(
            self.declare_parameter("selector_override_s", 3.0).value
        )
        self._selector_publish_hz = float(
            self.declare_parameter("selector_publish_hz", 10.0).value
        )
        self._require_navigation_active = bool(
            self.declare_parameter("require_navigation_active", True).value
        )
        self._require_goal = bool(self.declare_parameter("require_goal", True).value)
        self._ignore_unknown_cells = bool(
            self.declare_parameter("ignore_unknown_cells", True).value
        )
        self._lateral_sample_offsets = tuple(self._lateral_offsets())

        self._latest_path: Optional[AvgPath] = None
        self._latest_path_serialized: Optional[bytes] = None
        self._latest_goal: Optional[AvgPoseStamped] = None
        self._latest_pose: Optional[AvgPoseStamped] = None
        self._grids: Dict[str, GridRecord] = {}
        self._lanelet_grid: Optional[GridRecord] = None
        self._navigation_active = False
        self._blocked_since: Optional[Time] = None
        self._last_blocked_sample_time: Optional[Time] = None
        self._last_blockage_sample: Optional[BlockageSample] = None
        self._last_replan_time: Optional[Time] = None
        self._selector_override_until: Optional[Time] = None

        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        selector_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # HH_260720 - Use generated CAMROD path, pose, grid, and status contracts.
        # The local-path extractor publishes on both pose updates and its timer.
        # Keep the newest serialized path and decode it only at monitor cadence.
        self.create_subscription(
            AvgPath, self._path_topic, self._on_path, path_qos, raw=True
        )
        self.create_subscription(AvgPoseStamped, self._goal_topic, self._on_goal, 1)
        self.create_subscription(AvgPoseStamped, self._pose_topic, self._on_pose, 1)
        self.create_subscription(
            GoalStatusArray, self._navigate_status_topic, self._on_nav_status, 1
        )
        for topic in self._dynamic_grid_topics:
            self.create_subscription(
                AvgOccupancyGrid,
                topic,
                lambda msg, topic_name=topic: self._on_grid(topic_name, msg),
                1,
                raw=True,
            )
        lanelet_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            AvgOccupancyGrid,
            self._lanelet_grid_topic,
            self._on_lanelet_grid,
            lanelet_qos,
            raw=True,
        )

        self._planner_selector_pub = self.create_publisher(
            RosString, self._planner_selector_topic, selector_qos
        )
        self._status_pub = self.create_publisher(AvgString, self._status_topic, 10)
        self._navigate_client = ActionClient(self, NavigateToPose, self._navigate_action_name)

        monitor_period = 1.0 / max(0.5, self._monitor_rate_hz)
        selector_period = 1.0 / max(1.0, self._selector_publish_hz)
        self.create_timer(monitor_period, self._on_monitor_timer)
        self._selector_timer = self.create_timer(
            selector_period, self._on_selector_timer
        )
        self._selector_timer.cancel()

        self.get_logger().info(
            "obstacle_replan_monitor: "
            f"enabled={str(self._enabled).lower()} "
            f"grids={','.join(self._dynamic_grid_topics)} "
            f"fallback={self._fallback_planner_id} "
            f"preempt={str(self._preempt_enabled).lower()} "
            f"wide_lane_min={self._minimum_replan_lane_width_m:.2f}m "
            f"lookahead={self._lookahead_m:.1f}m hold={self._block_hold_s:.1f}s"
        )

    def _on_path(self, msg: bytes) -> None:
        self._latest_path_serialized = msg

    def _on_goal(self, msg: AvgPoseStamped) -> None:
        self._latest_goal = msg
        self._blocked_since = None
        self._last_blocked_sample_time = None
        self._last_blockage_sample = None

    def _on_pose(self, msg: AvgPoseStamped) -> None:
        self._latest_pose = msg

    def _on_grid(self, topic: str, msg: bytes) -> None:
        self._grids[topic] = GridRecord(
            topic=topic,
            received_time=self.get_clock().now(),
            serialized_grid=msg,
        )

    def _on_lanelet_grid(self, msg: bytes) -> None:
        self._lanelet_grid = GridRecord(
            topic=self._lanelet_grid_topic,
            received_time=self.get_clock().now(),
            serialized_grid=msg,
        )

    def _on_nav_status(self, msg: GoalStatusArray) -> None:
        active_states = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        self._navigation_active = any(status.status in active_states for status in msg.status_list)

    def _on_monitor_timer(self) -> None:
        if not self._enabled:
            return
        if self._require_navigation_active and not self._navigation_active:
            self._blocked_since = None
            self._publish_status("IDLE: navigation inactive")
            return
        if self._require_goal and self._latest_goal is None:
            self._blocked_since = None
            self._publish_status("IDLE: no active snapped goal")
            return
        if (
            self._latest_path is None
            and self._latest_path_serialized is None
        ) or self._latest_pose is None:
            self._blocked_since = None
            self._publish_status("IDLE: waiting for path/pose")
            return

        blockage = self._sample_dynamic_blockage()
        now_time = self.get_clock().now()
        if not blockage.blocked:
            if self._blocked_since is not None and self._last_blocked_sample_time is not None:
                clear_age = (now_time - self._last_blocked_sample_time).nanoseconds / 1.0e9
                if clear_age < self._clear_hold_s and self._last_blockage_sample is not None:
                    blocked_duration = (now_time - self._blocked_since).nanoseconds / 1.0e9
                    self._publish_status(
                        "BLOCKED_HOLD: "
                        f"{blocked_duration:.1f}s clear_age={clear_age:.1f}s "
                        f"last_max={self._last_blockage_sample.max_cost} "
                        f"source={self._last_blockage_sample.source_topic}"
                    )
                    if blocked_duration >= self._block_hold_s:
                        self._maybe_trigger_fallback_replan(
                            self._last_blockage_sample,
                            now_time,
                            blocked_duration,
                            "BLOCKED_HOLD_NO_PREEMPT",
                        )
                    return
            self._blocked_since = None
            self._last_blocked_sample_time = None
            self._last_blockage_sample = None
            self._publish_status(
                f"CLEAR: blocked={blockage.blocked_count}/{blockage.total_count} max={blockage.max_cost}"
            )
            return

        if self._blocked_since is None:
            self._blocked_since = now_time
        self._last_blocked_sample_time = now_time
        self._last_blockage_sample = blockage
        blocked_duration = (now_time - self._blocked_since).nanoseconds / 1.0e9
        self._publish_status(
            "BLOCKED: "
            f"{blocked_duration:.1f}s samples={blockage.blocked_count}/{blockage.total_count} "
            f"max={blockage.max_cost} source={blockage.source_topic}"
        )
        if blocked_duration < self._block_hold_s:
            return
        self._maybe_trigger_fallback_replan(
            blockage,
            now_time,
            blocked_duration,
            "BLOCKED_NO_PREEMPT",
        )

    def _on_selector_timer(self) -> None:
        if self._selector_override_until is None:
            return
        now_time = self.get_clock().now()
        if now_time > self._selector_override_until:
            self._selector_override_until = None
            self._selector_timer.cancel()
            if self._restore_planner_id:
                self._publish_planner_selector(self._restore_planner_id)
            return
        self._publish_planner_selector()

    def _maybe_trigger_fallback_replan(
        self,
        blockage: BlockageSample,
        now_time: Time,
        blocked_duration: float,
        no_preempt_status: str,
    ) -> None:
        if not self._preempt_enabled:
            self._publish_status(
                f"{no_preempt_status}: {blocked_duration:.1f}s "
                f"samples={blockage.blocked_count}/{blockage.total_count} "
                f"max={blockage.max_cost} source={blockage.source_topic}"
            )
            return
        if self._last_replan_time is not None:
            cooldown = (now_time - self._last_replan_time).nanoseconds / 1.0e9
            if cooldown < self._replan_cooldown_s:
                return
        lane_width = self._lane_width_for_blockage(blockage)
        if not lane_width.allowed:
            self._publish_status(
                "BLOCKED_REPLAN_DENIED: "
                f"reason={lane_width.reason} "
                f"width={lane_width.total_width_m:.2f}m "
                f"left={lane_width.left_clearance_m:.2f}m "
                f"right={lane_width.right_clearance_m:.2f}m"
            )
            return
        self._trigger_fallback_replan(blockage, lane_width)

    def _trigger_fallback_replan(
        self, blockage: BlockageSample, lane_width: LaneWidthSample
    ) -> None:
        if self._latest_goal is None:
            return
        now_time = self.get_clock().now()
        if not self._navigate_client.wait_for_server(timeout_sec=0.05):
            self.get_logger().warn(
                "obstacle_replan_monitor: "
                f"navigate action unavailable: {self._navigate_action_name}"
            )
            return

        self._last_replan_time = now_time
        self._selector_override_until = now_time + Duration(seconds=max(0.2, self._selector_override_s))
        self._selector_timer.reset()
        # HH_260702 - Publish the fallback selector before sending the action so
        # Nav2 PlannerSelector reads the opt-in fallback planner for this
        # preempted goal instead of the normal LaneletRoute latch.
        self._publish_planner_selector()

        goal_msg = NavigateToPose.Goal()
        # HH_260720 - Convert the generated snapped goal only at the Nav2 action boundary.
        goal_msg.pose = self._goal_pose_to_ros(self._latest_goal)
        goal_msg.pose.header.stamp = now_time.to_msg()
        goal_msg.behavior_tree = ""
        send_future = self._navigate_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)
        self.get_logger().warn(
            "obstacle_replan_monitor: persistent dynamic blockage "
            f"at ({blockage.point_x:.2f}, {blockage.point_y:.2f}) "
            f"source={blockage.source_topic} cost={blockage.max_cost}; "
            f"lane_width={lane_width.total_width_m:.2f}m; "
            f"preempting with planner={self._fallback_planner_id}"
        )

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("obstacle_replan_monitor: fallback NavigateToPose rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_replan_result)

    def _on_replan_result(self, future) -> None:
        result = future.result()
        self.get_logger().info(
            "obstacle_replan_monitor: "
            f"fallback NavigateToPose finished status={int(result.status)}"
        )

    def _publish_planner_selector(self, planner_id: Optional[str] = None) -> None:
        # HH_260720 - PlannerSelector is a Nav2 ROS boundary.
        msg = RosString()
        msg.data = planner_id if planner_id is not None else self._fallback_planner_id
        self._planner_selector_pub.publish(msg)

    @staticmethod
    def _goal_pose_to_ros(message: AvgPoseStamped) -> RosPoseStamped:
        # HH_260720 - Explicit field copies prevent standard ROS types from leaking internally.
        output = RosPoseStamped()
        output.header.stamp = message.header.stamp
        output.header.frame_id = message.header.frame_id
        output.pose.position.x = message.pose.position.x
        output.pose.position.y = message.pose.position.y
        output.pose.position.z = message.pose.position.z
        output.pose.orientation.x = message.pose.orientation.x
        output.pose.orientation.y = message.pose.orientation.y
        output.pose.orientation.z = message.pose.orientation.z
        output.pose.orientation.w = message.pose.orientation.w
        return output

    def _publish_status(self, text: str) -> None:
        msg = AvgString()
        msg.data = text
        self._status_pub.publish(msg)

    def _fresh_grids(self) -> List[GridRecord]:
        now_time = self.get_clock().now()
        fresh: List[GridRecord] = []
        for record in self._grids.values():
            age_s = (now_time - record.received_time).nanoseconds / 1.0e9
            if age_s <= self._grid_max_age_s:
                if record.grid is None and record.serialized_grid is not None:
                    record.grid = deserialize_message(
                        record.serialized_grid, AvgOccupancyGrid
                    )
                    record.serialized_grid = None
                if not record.transform_ready:
                    self._prepare_grid_record(record)
                fresh.append(record)
        return fresh

    def _fresh_lanelet_grid(self) -> Optional[GridRecord]:
        record = self._lanelet_grid
        if record is None:
            return None
        age_s = (self.get_clock().now() - record.received_time).nanoseconds / 1.0e9
        if age_s > self._lanelet_grid_max_age_s:
            return None
        if record.grid is None and record.serialized_grid is not None:
            record.grid = deserialize_message(
                record.serialized_grid, AvgOccupancyGrid
            )
            record.serialized_grid = None
        if not record.transform_ready:
            self._prepare_grid_record(record)
        return record if record.valid else None

    def _lane_width_for_blockage(
        self, blockage: BlockageSample
    ) -> LaneWidthSample:
        if not self._wide_lane_replan_enabled:
            return LaneWidthSample(True, 0.0, 0.0, 0.0, "gate_disabled")
        record = self._fresh_lanelet_grid()
        if record is None:
            return LaneWidthSample(False, 0.0, 0.0, 0.0, "lanelet_grid_unavailable")
        path = self._latest_path
        if path is None or len(path.poses) < 2:
            return LaneWidthSample(False, 0.0, 0.0, 0.0, "path_unavailable")
        reference = self._nearest_path_reference(
            path, blockage.point_x, blockage.point_y
        )
        if reference is None:
            return LaneWidthSample(False, 0.0, 0.0, 0.0, "path_heading_unavailable")
        center_x, center_y, yaw = reference
        return self._measure_lane_width(record, center_x, center_y, yaw)

    def _measure_lane_width(
        self,
        record: GridRecord,
        center_x: float,
        center_y: float,
        path_yaw: float,
    ) -> LaneWidthSample:
        """Measure the contiguous non-lethal lane width normal to the path."""
        center_cost = self._grid_cost_at(record, center_x, center_y)
        if center_cost is None:
            return LaneWidthSample(False, 0.0, 0.0, 0.0, "center_unknown")
        if center_cost >= self._lanelet_blocked_cost_threshold:
            return LaneWidthSample(False, 0.0, 0.0, 0.0, "center_outside_lane")

        normal_x = -math.sin(path_yaw)
        normal_y = math.cos(path_yaw)
        left = self._scan_lane_clearance(
            record, center_x, center_y, normal_x, normal_y
        )
        right = self._scan_lane_clearance(
            record, center_x, center_y, -normal_x, -normal_y
        )
        total = left + right
        allowed = (
            total >= self._minimum_replan_lane_width_m
            and left >= self._minimum_side_clearance_m
            and right >= self._minimum_side_clearance_m
        )
        reason = "wide_lane" if allowed else "lane_too_narrow"
        return LaneWidthSample(allowed, total, left, right, reason)

    def _scan_lane_clearance(
        self,
        record: GridRecord,
        center_x: float,
        center_y: float,
        direction_x: float,
        direction_y: float,
    ) -> float:
        # Half-cell sampling avoids skipping a lethal raster cell while keeping
        # this 5 Hz check bounded on the 0.25 m deployed lanelet grid.
        step_m = min(0.10, max(0.025, 0.5 * record.resolution))
        max_search_m = max(step_m, self._lane_width_max_search_m)
        distance_m = step_m
        while distance_m <= max_search_m + 1.0e-9:
            cost = self._grid_cost_at(
                record,
                center_x + direction_x * distance_m,
                center_y + direction_y * distance_m,
            )
            if cost is None or cost >= self._lanelet_blocked_cost_threshold:
                return max(0.0, distance_m - 0.5 * step_m)
            distance_m += step_m
        return max_search_m

    @staticmethod
    def _nearest_path_reference(
        path: AvgPath, point_x: float, point_y: float
    ) -> Optional[Tuple[float, float, float]]:
        best_reference: Optional[Tuple[float, float, float]] = None
        best_distance_sq = float("inf")
        for start_pose, end_pose in zip(path.poses, path.poses[1:]):
            start = start_pose.pose.position
            end = end_pose.pose.position
            dx = end.x - start.x
            dy = end.y - start.y
            length_sq = dx * dx + dy * dy
            if length_sq < 1.0e-12:
                continue
            ratio = ((point_x - start.x) * dx + (point_y - start.y) * dy) / length_sq
            ratio = min(1.0, max(0.0, ratio))
            reference_x = start.x + ratio * dx
            reference_y = start.y + ratio * dy
            distance_sq = (
                (point_x - reference_x) ** 2 + (point_y - reference_y) ** 2
            )
            if distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_reference = (
                    reference_x,
                    reference_y,
                    math.atan2(dy, dx),
                )
        return best_reference

    def _prepare_grid_record(self, record: GridRecord) -> None:
        record.transform_ready = True
        grid = record.grid
        if grid is None:
            return
        info = grid.info
        record.width = int(info.width)
        record.height = int(info.height)
        record.resolution = float(info.resolution)
        record.valid = (
            record.width > 0
            and record.height > 0
            and record.resolution > 0.0
        )
        if not record.valid:
            return
        record.origin_x = info.origin.position.x
        record.origin_y = info.origin.position.y
        yaw = self._yaw_from_quaternion(info.origin.orientation)
        record.inverse_cos = math.cos(-yaw)
        record.inverse_sin = math.sin(-yaw)

    def _sample_dynamic_blockage(self) -> BlockageSample:
        if self._latest_path_serialized is not None:
            self._latest_path = deserialize_message(
                self._latest_path_serialized, AvgPath
            )
            self._latest_path_serialized = None
        path = self._latest_path
        pose = self._latest_pose
        if path is None or pose is None or len(path.poses) < 2:
            return BlockageSample(False, 0, 0, -1, "", 0.0, 0.0)
        fresh_grids = self._fresh_grids()
        if not fresh_grids:
            return BlockageSample(False, 0, 0, -1, "", 0.0, 0.0)

        closest_index = self._closest_path_index(path, pose)
        lateral_offsets = self._lateral_sample_offsets
        total_count = 0
        blocked_count = 0
        max_cost = -1
        max_source = ""
        max_point_x = 0.0
        max_point_y = 0.0
        traveled_m = 0.0

        for segment_index in range(closest_index, len(path.poses) - 1):
            start = path.poses[segment_index].pose.position
            end = path.poses[segment_index + 1].pose.position
            segment_dx = end.x - start.x
            segment_dy = end.y - start.y
            segment_length = math.hypot(segment_dx, segment_dy)
            if segment_length < 1.0e-6:
                continue
            yaw = math.atan2(segment_dy, segment_dx)
            normal_x = -math.sin(yaw)
            normal_y = math.cos(yaw)
            steps = max(1, int(math.ceil(segment_length / max(0.05, self._sample_step_m))))
            for step_index in range(1, steps + 1):
                ratio = float(step_index) / float(steps)
                route_distance = traveled_m + segment_length * ratio
                if route_distance < self._sample_start_distance_m:
                    continue
                if route_distance > self._lookahead_m:
                    return self._make_blockage_result(
                        blocked_count, total_count, max_cost, max_source, max_point_x, max_point_y
                    )
                center_x = start.x + segment_dx * ratio
                center_y = start.y + segment_dy * ratio
                for offset in lateral_offsets:
                    sample_x = center_x + normal_x * offset
                    sample_y = center_y + normal_y * offset
                    total_count += 1
                    blocked, cost, topic = self._sample_grids(fresh_grids, sample_x, sample_y)
                    if cost > max_cost:
                        max_cost = cost
                        max_source = topic
                        max_point_x = sample_x
                        max_point_y = sample_y
                    if blocked:
                        blocked_count += 1
            traveled_m += segment_length
            if traveled_m >= self._lookahead_m:
                break

        return self._make_blockage_result(
            blocked_count, total_count, max_cost, max_source, max_point_x, max_point_y
        )

    def _make_blockage_result(
        self,
        blocked_count: int,
        total_count: int,
        max_cost: int,
        source_topic: str,
        point_x: float,
        point_y: float,
    ) -> BlockageSample:
        ratio = float(blocked_count) / float(max(1, total_count))
        blocked = (
            blocked_count >= max(1, self._min_blocked_samples)
            and ratio >= max(0.0, self._blocked_sample_ratio)
        )
        return BlockageSample(
            blocked=blocked,
            blocked_count=blocked_count,
            total_count=total_count,
            max_cost=max_cost,
            source_topic=source_topic,
            point_x=point_x,
            point_y=point_y,
        )

    def _sample_grids(
        self, grids: List[GridRecord], point_x: float, point_y: float
    ) -> Tuple[bool, int, str]:
        max_cost = -1
        max_topic = ""
        blocked = False
        for record in grids:
            cost = self._grid_cost_at(record, point_x, point_y)
            if cost is None:
                continue
            if cost > max_cost:
                max_cost = cost
                max_topic = record.topic
            if cost >= self._obstacle_cost_threshold:
                blocked = True
        return blocked, max_cost, max_topic

    def _grid_cost_at(
        self, record: GridRecord, point_x: float, point_y: float
    ) -> Optional[int]:
        grid = record.grid
        if grid is None or not record.valid:
            return None
        dx = point_x - record.origin_x
        dy = point_y - record.origin_y
        local_x = record.inverse_cos * dx - record.inverse_sin * dy
        local_y = record.inverse_sin * dx + record.inverse_cos * dy
        cell_x = int(math.floor(local_x / record.resolution))
        cell_y = int(math.floor(local_y / record.resolution))
        if (
            cell_x < 0
            or cell_y < 0
            or cell_x >= record.width
            or cell_y >= record.height
        ):
            return None
        index = cell_y * record.width + cell_x
        if index < 0 or index >= len(grid.data):
            return None
        cost = int(grid.data[index])
        if cost < 0 and self._ignore_unknown_cells:
            return None
        return cost

    def _closest_path_index(self, path: AvgPath, pose: AvgPoseStamped) -> int:
        pose_x = pose.pose.position.x
        pose_y = pose.pose.position.y
        best_index = 0
        best_distance = float("inf")
        for index, path_pose in enumerate(path.poses):
            position = path_pose.pose.position
            distance = (position.x - pose_x) ** 2 + (position.y - pose_y) ** 2
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return min(best_index, max(0, len(path.poses) - 2))

    def _lateral_offsets(self) -> List[float]:
        half_width = max(0.0, self._corridor_half_width_m)
        step = max(0.05, self._lateral_step_m)
        count = max(0, int(math.floor(half_width / step)))
        offsets = [0.0]
        for index in range(1, count + 1):
            offset = step * index
            offsets.append(offset)
            offsets.append(-offset)
        if half_width > 0.0 and all(abs(abs(offset) - half_width) > 1.0e-6 for offset in offsets):
            offsets.append(half_width)
            offsets.append(-half_width)
        return offsets

    @staticmethod
    def _yaw_from_quaternion(quaternion) -> float:
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )


def main() -> None:
    rclpy.init()
    node = ObstacleReplanMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        # HH_260707 - Suppress rclpy teardown races during launch shutdown only.
        if rclpy.ok() and "Unable to convert call argument" not in str(exc):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
