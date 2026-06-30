#!/usr/bin/env python3
"""Trigger a Nav2 global replan when dynamic obstacles persistently block the route."""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String


@dataclass
class GridRecord:
    topic: str
    grid: OccupancyGrid
    received_time: Time


@dataclass
class BlockageSample:
    blocked: bool
    blocked_count: int
    total_count: int
    max_cost: int
    source_topic: str
    point_x: float
    point_y: float


class ObstacleReplanMonitor(Node):
    """Monitors dynamic obstacle grids and preempts Nav2 with a fallback planner."""

    def __init__(self) -> None:
        super().__init__("obstacle_replan_monitor")

        self._enabled = bool(self.declare_parameter("enabled", True).value)
        self._path_topic = str(
            self.declare_parameter("global_path_topic", "/planning/global_path").value
        ).strip()
        self._goal_topic = str(
            self.declare_parameter("goal_topic", "/planning/goal_pose_snapped_ros").value
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
            self.declare_parameter("planner_selector_topic", "/planning/planner_selector").value
        ).strip()
        self._status_topic = str(
            self.declare_parameter("status_topic", "/planning/obstacle_replan/status").value
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
            self.declare_parameter("fallback_planner_id", "Smac2D").value
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
        self._block_hold_s = float(self.declare_parameter("block_hold_s", 2.0).value)
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

        self._latest_path: Optional[Path] = None
        self._latest_goal: Optional[PoseStamped] = None
        self._latest_pose: Optional[PoseStamped] = None
        self._grids: Dict[str, GridRecord] = {}
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

        self.create_subscription(Path, self._path_topic, self._on_path, path_qos)
        self.create_subscription(PoseStamped, self._goal_topic, self._on_goal, 10)
        self.create_subscription(PoseStamped, self._pose_topic, self._on_pose, 10)
        self.create_subscription(
            GoalStatusArray, self._navigate_status_topic, self._on_nav_status, 10
        )
        for topic in self._dynamic_grid_topics:
            self.create_subscription(
                OccupancyGrid,
                topic,
                lambda msg, topic_name=topic: self._on_grid(topic_name, msg),
                10,
            )

        self._planner_selector_pub = self.create_publisher(
            String, self._planner_selector_topic, selector_qos
        )
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._navigate_client = ActionClient(self, NavigateToPose, self._navigate_action_name)

        monitor_period = 1.0 / max(0.5, self._monitor_rate_hz)
        selector_period = 1.0 / max(1.0, self._selector_publish_hz)
        self.create_timer(monitor_period, self._on_monitor_timer)
        self.create_timer(selector_period, self._on_selector_timer)

        self.get_logger().info(
            "obstacle_replan_monitor: "
            f"enabled={str(self._enabled).lower()} "
            f"grids={','.join(self._dynamic_grid_topics)} "
            f"fallback={self._fallback_planner_id} "
            f"lookahead={self._lookahead_m:.1f}m hold={self._block_hold_s:.1f}s"
        )

    def _on_path(self, msg: Path) -> None:
        self._latest_path = msg

    def _on_goal(self, msg: PoseStamped) -> None:
        self._latest_goal = msg
        self._blocked_since = None
        self._last_blocked_sample_time = None
        self._last_blockage_sample = None

    def _on_pose(self, msg: PoseStamped) -> None:
        self._latest_pose = msg

    def _on_grid(self, topic: str, msg: OccupancyGrid) -> None:
        self._grids[topic] = GridRecord(topic=topic, grid=msg, received_time=self.get_clock().now())

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
        if self._latest_path is None or self._latest_pose is None:
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
                        if self._last_replan_time is not None:
                            cooldown = (now_time - self._last_replan_time).nanoseconds / 1.0e9
                            if cooldown < self._replan_cooldown_s:
                                return
                        self._trigger_fallback_replan(self._last_blockage_sample)
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
        if self._last_replan_time is not None:
            cooldown = (now_time - self._last_replan_time).nanoseconds / 1.0e9
            if cooldown < self._replan_cooldown_s:
                return
        self._trigger_fallback_replan(blockage)

    def _on_selector_timer(self) -> None:
        if self._selector_override_until is None:
            return
        now_time = self.get_clock().now()
        if now_time > self._selector_override_until:
            self._selector_override_until = None
            return
        self._publish_planner_selector()

    def _trigger_fallback_replan(self, blockage: BlockageSample) -> None:
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
        # HH_260619 - Publish the fallback selector before sending the action so
        # Nav2 PlannerSelector reads Smac2D for this preempted goal instead of the
        # normal LaneletRoute latch.
        self._publish_planner_selector()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._latest_goal
        goal_msg.pose.header.stamp = now_time.to_msg()
        goal_msg.behavior_tree = ""
        send_future = self._navigate_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)
        self.get_logger().warn(
            "obstacle_replan_monitor: persistent dynamic blockage "
            f"at ({blockage.point_x:.2f}, {blockage.point_y:.2f}) "
            f"source={blockage.source_topic} cost={blockage.max_cost}; "
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

    def _publish_planner_selector(self) -> None:
        msg = String()
        msg.data = self._fallback_planner_id
        self._planner_selector_pub.publish(msg)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _fresh_grids(self) -> List[GridRecord]:
        now_time = self.get_clock().now()
        fresh: List[GridRecord] = []
        for record in self._grids.values():
            age_s = (now_time - record.received_time).nanoseconds / 1.0e9
            if age_s <= self._grid_max_age_s:
                fresh.append(record)
        return fresh

    def _sample_dynamic_blockage(self) -> BlockageSample:
        path = self._latest_path
        pose = self._latest_pose
        if path is None or pose is None or len(path.poses) < 2:
            return BlockageSample(False, 0, 0, -1, "", 0.0, 0.0)
        fresh_grids = self._fresh_grids()
        if not fresh_grids:
            return BlockageSample(False, 0, 0, -1, "", 0.0, 0.0)

        closest_index = self._closest_path_index(path, pose)
        lateral_offsets = self._lateral_offsets()
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
            cost = self._grid_cost_at(record.grid, point_x, point_y)
            if cost is None:
                continue
            if cost > max_cost:
                max_cost = cost
                max_topic = record.topic
            if cost >= self._obstacle_cost_threshold:
                blocked = True
        return blocked, max_cost, max_topic

    def _grid_cost_at(self, grid: OccupancyGrid, point_x: float, point_y: float) -> Optional[int]:
        info = grid.info
        if info.width == 0 or info.height == 0 or info.resolution <= 0.0:
            return None
        origin = info.origin
        yaw = self._yaw_from_quaternion(origin.orientation)
        dx = point_x - origin.position.x
        dy = point_y - origin.position.y
        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        local_x = cos_yaw * dx - sin_yaw * dy
        local_y = sin_yaw * dx + cos_yaw * dy
        cell_x = int(math.floor(local_x / info.resolution))
        cell_y = int(math.floor(local_y / info.resolution))
        if cell_x < 0 or cell_y < 0 or cell_x >= int(info.width) or cell_y >= int(info.height):
            return None
        index = cell_y * int(info.width) + cell_x
        if index < 0 or index >= len(grid.data):
            return None
        cost = int(grid.data[index])
        if cost < 0 and self._ignore_unknown_cells:
            return None
        return cost

    def _closest_path_index(self, path: Path, pose: PoseStamped) -> int:
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
