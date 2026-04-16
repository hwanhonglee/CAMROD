#!/usr/bin/env python3
# HH_260331: Gate Nav2 controller cmd_vel with explicit planning engage trigger.
# HH_260409: Use a single trigger topic (/planning/engage) for operator UX consistency.

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


class PlanningCmdVelGateNode(Node):
    # Initializes command gate interfaces and optional cost-stop watchdog.
    def __init__(self) -> None:
        super().__init__("planning_cmd_vel_gate")

        self.input_topic = str(
            self.declare_parameter("input_topic", "/planning/cmd_vel_raw").value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/planning/cmd_vel").value
        )
        self.engage_topic = str(
            self.declare_parameter("engage_topic", "/planning/engage").value
        )
        self.state_topic = str(
            self.declare_parameter("state_topic", "/planning/engaged").value
        )
        self.use_estop_topic = bool(
            self.declare_parameter("use_estop_topic", True).value
        )
        self.estop_topic = str(
            # HH_260409: Use platform status e-stop as default shared source.
            self.declare_parameter("estop_topic", "/platform/status/estop").value
        )
        self.allow_on_start = bool(
            self.declare_parameter("allow_on_start", False).value
        )
        self.publish_zero_when_blocked = bool(
            self.declare_parameter("publish_zero_when_blocked", True).value
        )

        # Costmap-based stop options.
        self.enable_cost_stop = bool(
            self.declare_parameter("enable_cost_stop", True).value
        )
        self.cost_grid_topic = str(
            self.declare_parameter(
                "cost_grid_topic", "/planning/local_costmap/costmap"
            ).value
        )
        self.pose_topic = str(
            self.declare_parameter("pose_topic", "/localization/pose").value
        )
        self.robot_base_frame = str(
            self.declare_parameter("robot_base_frame", "robot_base_link").value
        )
        self.cost_stop_threshold = int(
            self.declare_parameter("cost_stop_threshold", 200).value
        )
        self.cost_stop_lookahead_m = float(
            self.declare_parameter("cost_stop_lookahead_m", 2.0).value
        )
        self.cost_stop_width_m = float(
            self.declare_parameter("cost_stop_width_m", 1.0).value
        )
        self.cost_stop_hold_sec = float(
            self.declare_parameter("cost_stop_hold_sec", 1.0).value
        )

        # Unavoidable-cluster stop options.
        self.enable_unavoidable_stop = bool(
            self.declare_parameter("enable_unavoidable_stop", True).value
        )
        self.unavoidable_lethal_threshold = int(
            self.declare_parameter("unavoidable_lethal_threshold", 253).value
        )
        self.unavoidable_cluster_min_cells = int(
            self.declare_parameter("unavoidable_cluster_min_cells", 25).value
        )
        self.unavoidable_cluster_min_ratio = float(
            self.declare_parameter("unavoidable_cluster_min_ratio", 0.25).value
        )

        self._enabled = self.allow_on_start
        self._estop = False
        self._cost_blocked_until = 0.0
        self._last_unavoidable_cluster_cells = 0
        self._last_unavoidable_cluster_ratio = 0.0
        self._last_tf_warn_sec = 0.0
        self._last_empty_corridor_warn_sec = 0.0

        self._last_grid = None
        self._last_pose = None

        self.pub_cmd = self.create_publisher(Twist, self.output_topic, 10)
        self.pub_state = self.create_publisher(Bool, self.state_topic, 10)

        # HH_260415: Resolve robot pose in costmap frame through TF first.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.sub_cmd = self.create_subscription(
            Twist, self.input_topic, self._on_cmd, 10
        )
        self.sub_engage = self.create_subscription(
            Bool, self.engage_topic, self._on_engage, 10
        )

        self.sub_estop = None
        if self.use_estop_topic:
            self.sub_estop = self.create_subscription(
                Bool, self.estop_topic, self._on_estop, 10
            )

        self.sub_cost_grid = None
        self.sub_pose = None
        if self.enable_cost_stop:
            # HH_260415: Match local_costmap durability to avoid missed updates.
            cost_qos = QoSProfile(depth=10)
            cost_qos.reliability = QoSReliabilityPolicy.RELIABLE
            cost_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.sub_cost_grid = self.create_subscription(
                OccupancyGrid, self.cost_grid_topic, self._on_cost_grid, cost_qos
            )
            # Pose topic is kept as fallback when TF lookup is temporarily unavailable.
            self.sub_pose = self.create_subscription(
                PoseStamped, self.pose_topic, self._on_pose, 10
            )

        self._state_timer = self.create_timer(0.5, self._publish_state)
        # HH_260415: Allow runtime threshold/profile tuning via ros2 param set.
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._publish_state()
        self.get_logger().info(
            "planning_cmd_vel_gate ready: "
            f"in={self.input_topic} out={self.output_topic} "
            f"engage_topic={self.engage_topic} "
            f"estop_topic={self.estop_topic if self.use_estop_topic else '(disabled)'} "
            f"allow_on_start={'true' if self.allow_on_start else 'false'} "
            f"cost_stop={'true' if self.enable_cost_stop else 'false'} "
            f"cost_grid={self.cost_grid_topic} pose_topic={self.pose_topic} "
            f"robot_base_frame={self.robot_base_frame} "
            f"unavoidable_stop={'true' if self.enable_unavoidable_stop else 'false'}"
        )

    # Applies runtime parameter updates for gate/cost-stop tuning.
    def _on_set_parameters(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "enable_cost_stop":
                self.enable_cost_stop = bool(p.value)
            elif p.name == "cost_stop_threshold":
                self.cost_stop_threshold = int(p.value)
            elif p.name == "cost_stop_hold_sec":
                self.cost_stop_hold_sec = float(p.value)
            elif p.name == "cost_stop_lookahead_m":
                self.cost_stop_lookahead_m = float(p.value)
            elif p.name == "cost_stop_width_m":
                self.cost_stop_width_m = float(p.value)
            elif p.name == "enable_unavoidable_stop":
                self.enable_unavoidable_stop = bool(p.value)
            elif p.name == "unavoidable_lethal_threshold":
                self.unavoidable_lethal_threshold = int(p.value)
            elif p.name == "unavoidable_cluster_min_cells":
                self.unavoidable_cluster_min_cells = int(p.value)
            elif p.name == "unavoidable_cluster_min_ratio":
                self.unavoidable_cluster_min_ratio = float(p.value)
            elif p.name == "allow_on_start":
                self.allow_on_start = bool(p.value)
            elif p.name == "publish_zero_when_blocked":
                self.publish_zero_when_blocked = bool(p.value)
        return SetParametersResult(successful=True)

    # Returns whether cmd passthrough is currently allowed.
    def _effective_enabled(self) -> bool:
        if not (self._enabled and not self._estop):
            return False
        if self._cost_blocked_until > self.get_clock().now().nanoseconds * 1e-9:
            return False
        return True

    # Publishes current effective engage state.
    def _publish_state(self) -> None:
        msg = Bool()
        msg.data = self._effective_enabled()
        self.pub_state.publish(msg)

    # Publishes zero Twist when blocked/disabled.
    def _publish_zero(self) -> None:
        self.pub_cmd.publish(Twist())

    # Handles raw cmd_vel input and applies engage/estop/cost-stop rules.
    def _on_cmd(self, msg: Twist) -> None:
        if self.enable_cost_stop and self._should_stop_for_cost():
            if self.publish_zero_when_blocked:
                self._publish_zero()
            return
        if self._effective_enabled():
            self.pub_cmd.publish(msg)
            return
        if self.publish_zero_when_blocked:
            self._publish_zero()

    # Updates engage latch from /planning/engage.
    def _set_enabled(self, enabled: bool) -> None:
        new_enabled = bool(enabled)
        if new_enabled == self._enabled:
            return
        self._enabled = new_enabled
        self._publish_state()
        if not self._effective_enabled() and self.publish_zero_when_blocked:
            self._publish_zero()
        self.get_logger().info(
            "planning engage update: "
            f"enabled={'true' if self._enabled else 'false'} "
            f"estop={'true' if self._estop else 'false'} "
            f"effective={'true' if self._effective_enabled() else 'false'}"
        )

    # Handles engage messages.
    def _on_engage(self, msg: Bool) -> None:
        self._set_enabled(msg.data)

    # Handles e-stop messages.
    def _on_estop(self, msg: Bool) -> None:
        new_estop = bool(msg.data)
        if new_estop == self._estop:
            return
        self._estop = new_estop
        self._publish_state()
        if self._estop and self.publish_zero_when_blocked:
            self._publish_zero()
        self.get_logger().warn(
            "planning estop update: "
            f"estop={'true' if self._estop else 'false'} "
            f"effective={'true' if self._effective_enabled() else 'false'}"
        )

    # Stores latest local costmap.
    def _on_cost_grid(self, msg: OccupancyGrid) -> None:
        self._last_grid = msg

    # Stores latest pose fallback.
    def _on_pose(self, msg: PoseStamped) -> None:
        self._last_pose = msg

    # Checks forward corridor in cost grid and triggers stop when blocked.
    def _should_stop_for_cost(self) -> bool:
        grid = self._last_grid
        if grid is None:
            return False
        if not grid.data or grid.info.resolution <= 0.0:
            return False

        target_frame = str(grid.header.frame_id).strip()
        pose_candidates = self._resolve_pose_candidates(target_frame)
        if not pose_candidates:
            return False

        best_total_cells = -1
        best_label = ""
        best_lethal_cells: list[tuple[int, int]] = []
        for label, pose in pose_candidates:
            blocked, total_cells, lethal_cells = self._sample_cost_corridor(grid, pose)
            if blocked:
                self._cost_blocked_until = (
                    self.get_clock().now().nanoseconds * 1e-9 + self.cost_stop_hold_sec
                )
                self.get_logger().warn(
                    # HH_260415: rclpy logger does not support printf-style variadic args.
                    f"planning cost-stop: source={label} "
                    f"threshold={self.cost_stop_threshold} hold={self.cost_stop_hold_sec:.2f}s"
                )
                return True
            if total_cells > best_total_cells:
                best_total_cells = total_cells
                best_label = label
                best_lethal_cells = lethal_cells

        if best_total_cells <= 0:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if (now_sec - self._last_empty_corridor_warn_sec) >= 2.0:
                self._last_empty_corridor_warn_sec = now_sec
                self.get_logger().warn(
                    "planning cost-stop: sampled corridor outside costmap; "
                    "check pose/costmap frame alignment"
                )
            return False

        if self.enable_unavoidable_stop and best_lethal_cells:
            if self._is_unavoidable_cluster(best_lethal_cells, best_total_cells):
                self._cost_blocked_until = (
                    self.get_clock().now().nanoseconds * 1e-9
                    + self.cost_stop_hold_sec
                )
                self.get_logger().warn(
                    # HH_260415: Include selected pose source to debug frame alignment.
                    f"planning cost-stop (unavoidable): source={best_label} "
                    f"lethal={self._last_unavoidable_cluster_cells} "
                    f"ratio={self._last_unavoidable_cluster_ratio:.2f} "
                    f"hold={self.cost_stop_hold_sec:.2f}s"
                )
                return True

        return False

    # Samples forward corridor for one pose candidate and returns stop decision/statistics.
    def _sample_cost_corridor(
        self, grid: OccupancyGrid, pose: tuple[float, float, float]
    ) -> tuple[bool, int, list[tuple[int, int]]]:
        pose_x, pose_y, yaw = pose
        lookahead = max(0.1, self.cost_stop_lookahead_m)
        width = max(0.1, self.cost_stop_width_m)
        res = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        w = int(grid.info.width)
        h = int(grid.info.height)

        step = res
        x = 0.0
        half_w = width * 0.5
        total_cells = 0
        lethal_cells: list[tuple[int, int]] = []

        while x <= lookahead:
            y = -half_w
            while y <= half_w:
                wx = pose_x + x * self._cos(yaw) - y * self._sin(yaw)
                wy = pose_y + x * self._sin(yaw) + y * self._cos(yaw)
                mx = int((wx - origin_x) / res)
                my = int((wy - origin_y) / res)
                if 0 <= mx < w and 0 <= my < h:
                    idx = my * w + mx
                    cost = int(grid.data[idx])
                    total_cells += 1
                    if cost >= self.cost_stop_threshold:
                        return True, total_cells, lethal_cells
                    if cost >= self.unavoidable_lethal_threshold:
                        lethal_cells.append((mx, my))
                y += step
            x += step

        return False, total_cells, lethal_cells

    # Resolves robot pose candidates as (x, y, yaw) in target_frame.
    def _resolve_pose_candidates(
        self, target_frame: str
    ) -> list[tuple[str, tuple[float, float, float]]]:
        if not target_frame:
            return []

        candidates: list[tuple[str, tuple[float, float, float]]] = []

        # HH_260415: Primary path: TF(target <- robot_base_frame).
        tf_pose = self._lookup_tf_pose(target_frame, self.robot_base_frame)
        if tf_pose is not None:
            candidates.append(("tf_robot_base", tf_pose))

        # Fallback path: transform pose_topic data if available.
        pose_msg = self._last_pose
        if pose_msg is None:
            return candidates

        source_frame = str(pose_msg.header.frame_id).strip()
        px = float(pose_msg.pose.position.x)
        py = float(pose_msg.pose.position.y)
        pyaw = self._yaw_from_quat(pose_msg.pose.orientation)

        if not source_frame or source_frame == target_frame:
            candidates.append(("pose_raw", (px, py, pyaw)))
            return candidates

        tf_msg = self._lookup_transform(target_frame, source_frame)
        if tf_msg is not None:
            tx = float(tf_msg.transform.translation.x)
            ty = float(tf_msg.transform.translation.y)
            tyaw = self._yaw_from_quat(tf_msg.transform.rotation)
            rx, ry = self._rotate_xy(px, py, tyaw)
            candidates.append(
                ("pose_tf", (tx + rx, ty + ry, self._normalize_yaw(tyaw + pyaw)))
            )

        # HH_260415: Keep raw pose as last fallback when TF frame alignment is unstable.
        candidates.append(("pose_raw", (px, py, pyaw)))
        return candidates

    # Looks up transform target<-source at latest available time.
    def _lookup_transform(self, target_frame: str, source_frame: str):
        try:
            return self._tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except TransformException as exc:
            self._throttled_tf_warn(
                f"cost-stop TF unavailable ({source_frame} -> {target_frame}): {exc}"
            )
            return None

    # Returns robot pose from TF transform target<-source.
    def _lookup_tf_pose(
        self, target_frame: str, source_frame: str
    ) -> tuple[float, float, float] | None:
        tf_msg = self._lookup_transform(target_frame, source_frame)
        if tf_msg is None:
            return None
        x = float(tf_msg.transform.translation.x)
        y = float(tf_msg.transform.translation.y)
        yaw = self._yaw_from_quat(tf_msg.transform.rotation)
        return (x, y, yaw)

    # Emits TF warning at low rate to prevent log flooding.
    def _throttled_tf_warn(self, text: str) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (now_sec - self._last_tf_warn_sec) < 2.0:
            return
        self._last_tf_warn_sec = now_sec
        self.get_logger().warn(text)

    # Detects lethal-cell clusters that are too large to bypass within corridor.
    def _is_unavoidable_cluster(
        self, lethal_cells: list[tuple[int, int]], total_cells: int
    ) -> bool:
        # HH_260413: Basic connected-component scan for lethal cells.
        lethal = set(lethal_cells)
        visited: set[tuple[int, int]] = set()
        max_cluster = 0
        for cell in lethal_cells:
            if cell in visited:
                continue
            stack = [cell]
            visited.add(cell)
            count = 0
            while stack:
                cx, cy = stack.pop()
                count += 1
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nxt = (cx + dx, cy + dy)
                    if nxt in lethal and nxt not in visited:
                        visited.add(nxt)
                        stack.append(nxt)
            max_cluster = max(max_cluster, count)
        ratio = float(max_cluster) / float(max(1, total_cells))
        self._last_unavoidable_cluster_cells = max_cluster
        self._last_unavoidable_cluster_ratio = ratio
        return (
            max_cluster >= self.unavoidable_cluster_min_cells
            and ratio >= self.unavoidable_cluster_min_ratio
        )

    def _yaw_from_quat(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _sin(self, v: float) -> float:
        return math.sin(v)

    def _cos(self, v: float) -> float:
        return math.cos(v)

    def _rotate_xy(self, x: float, y: float, yaw: float) -> tuple[float, float]:
        c = self._cos(yaw)
        s = self._sin(yaw)
        return (x * c - y * s, x * s + y * c)

    def _normalize_yaw(self, yaw: float) -> float:
        while yaw > math.pi:
            yaw -= 2.0 * math.pi
        while yaw < -math.pi:
            yaw += 2.0 * math.pi
        return yaw


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanningCmdVelGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
