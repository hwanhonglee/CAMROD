#!/usr/bin/env python3
# HH_260331: Gate Nav2 controller cmd_vel with explicit planning engage trigger.
# HH_260409: Use a single trigger topic (/planning/engage) for operator UX consistency.

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


def _declare_with_alias(node: Node, canonical_name: str, default_value, *legacy_names):
    """Read canonical parameter with optional legacy-name fallback."""
    value = node.declare_parameter(canonical_name, default_value).value
    canonical_is_default = value == default_value
    for legacy_name in legacy_names:
        legacy_value = node.declare_parameter(legacy_name, default_value).value
        if legacy_value == default_value:
            continue
        if canonical_is_default:
            node.get_logger().warn(
                f"Parameter '{legacy_name}' is deprecated. Use '{canonical_name}' instead."
            )
            value = legacy_value
            canonical_is_default = False
            continue
        if legacy_value != value:
            node.get_logger().warn(
                f"Both '{canonical_name}' and deprecated '{legacy_name}' are set with "
                f"different values. Using '{canonical_name}'."
            )
    return value


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
        # HH_260424: Unified inflation cost grid (lanelet + LiDAR + Radar + global_path).
        self.cost_grid_topic = str(
            self.declare_parameter(
                "cost_grid_topic", "/planning/cost_grid/inflation"
            ).value
        )
        self.pose_topic = str(
            self.declare_parameter("pose_topic", "/localization/pose").value
        )
        self.odometry_topic = str(
            self.declare_parameter("odometry_topic", "/localization/vio/odometry").value
        )
        self.pose_source_preference = str(
            self.declare_parameter("pose_source_preference", "odometry").value
        )
        self.enable_pose_raw_fallback = bool(
            self.declare_parameter("enable_pose_raw_fallback", False).value
        )
        self.robot_base_frame = str(
            self.declare_parameter("robot_base_frame", "robot_base_link").value
        )
        self.cost_stop_threshold = int(
            self.declare_parameter("cost_stop_threshold", 85).value
        )
        self.cost_stop_lookahead_m = float(
            # HH_260422: Legacy fixed lookahead; used only when enable_speed_dependent_lookahead=false.
            self.declare_parameter("cost_stop_lookahead_m", 2.0).value
        )
        self.cost_stop_width_m = float(
            self.declare_parameter("cost_stop_width_m", 1.0).value
        )
        self.cost_stop_hold_s = float(
            _declare_with_alias(self, "cost_stop_hold_s", 1.0, "cost_stop_hold_sec")
        )

        # HH_260422: Speed-dependent front lookahead.
        #   lookahead = clamp(v²/(2·g·mu) + reaction_time·v + margin, min, max)
        #   where v = |forward speed| from odometry, g=9.8, mu=front_lookahead_friction.
        self.enable_speed_dependent_lookahead = bool(
            self.declare_parameter("enable_speed_dependent_lookahead", True).value
        )
        self.front_lookahead_min_m = float(
            self.declare_parameter("front_lookahead_min_m", 0.4).value
        )
        self.front_lookahead_max_m = float(
            self.declare_parameter("front_lookahead_max_m", 3.0).value
        )
        self.front_lookahead_friction = float(
            # Wet road kinetic friction coefficient for braking distance calc.
            self.declare_parameter("front_lookahead_friction", 0.4).value
        )
        self.front_reaction_time_s = float(
            # Sensor/compute latency budget added to braking distance.
            self.declare_parameter("front_reaction_time_s", 0.15).value
        )
        self.front_lookahead_margin_m = float(
            # Static safety buffer added on top of braking+reaction distance.
            self.declare_parameter("front_lookahead_margin_m", 0.3).value
        )

        # HH_260422: Side and rear cost-stop — uses the same merged cost grid as front.
        #   enable_side_rear_cost_stop=false keeps only the forward corridor check active.
        self.enable_side_rear_cost_stop = bool(
            self.declare_parameter("enable_side_rear_cost_stop", True).value
        )
        self.side_cost_threshold = int(
            self.declare_parameter("side_cost_threshold", 85).value
        )
        self.side_lookahead_m = float(
            self.declare_parameter("side_lookahead_m", 1.2).value
        )
        self.side_corridor_width_m = float(
            self.declare_parameter("side_corridor_width_m", 0.6).value
        )
        self.rear_cost_threshold = int(
            self.declare_parameter("rear_cost_threshold", 85).value
        )
        self.rear_lookahead_m = float(
            self.declare_parameter("rear_lookahead_m", 0.8).value
        )
        self.rear_corridor_width_m = float(
            self.declare_parameter("rear_corridor_width_m", 0.9).value
        )

        # Unavoidable-cluster stop options (front only).
        self.enable_unavoidable_stop = bool(
            self.declare_parameter("enable_unavoidable_stop", True).value
        )
        self.unavoidable_lethal_threshold = int(
            self.declare_parameter("unavoidable_lethal_threshold", 90).value
        )
        self.unavoidable_cluster_min_cells = int(
            self.declare_parameter("unavoidable_cluster_min_cells", 25).value
        )
        self.unavoidable_cluster_min_ratio = float(
            self.declare_parameter("unavoidable_cluster_min_ratio", 0.25).value
        )

        # HH_260422: _enabled becomes True when /planning/engage publishes True.
        #   True + _estop==False -> passes /planning/cmd_vel_raw through to /planning/cmd_vel.
        #   False -> blocks nav2 cmd_vel; publish_zero_when_blocked=True sends zero Twist downstream.
        #   allow_on_start=True enables gate at startup without an explicit engage signal (default: False).
        self._enabled = self.allow_on_start

        # HH_260422: _estop becomes True when /platform/status/estop publishes True.
        #   True -> blocks cmd_vel regardless of _enabled state; zero Twist is sent immediately.
        self._estop = False

        # HH_260422: _cost_blocked_until is the monotonic timestamp until which cost-stop keeps the gate closed.
        #   Gate remains blocked while time.monotonic() < _cost_blocked_until (cost_stop_hold_s duration).
        self._cost_blocked_until = 0.0
        self._last_unavoidable_cluster_cells = 0
        self._last_unavoidable_cluster_ratio = 0.0
        self._last_tf_warn_sec = 0.0
        self._last_empty_corridor_warn_sec = 0.0

        # HH_260422: _current_speed holds the latest forward body velocity (m/s) from odometry.
        #   Used to compute speed-dependent front lookahead. Stays 0.0 until first odometry arrives.
        self._current_speed = 0.0

        # HH_260422: Single merged cost grid (LiDAR + Radar) used for all directional checks.
        self._last_grid = None
        self._last_pose = None
        self._last_odom = None

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
        self.sub_odom = None
        if self.enable_cost_stop:
            # HH_260422: Merged cost grid — single subscription for all directional checks.
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
            self.sub_odom = self.create_subscription(
                Odometry, self.odometry_topic, self._on_odom, 10
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
            f"inflation_grid={self.cost_grid_topic} "
            f"speed_dependent={'true' if self.enable_speed_dependent_lookahead else 'false'} "
            f"front_lookahead=[{self.front_lookahead_min_m:.1f},{self.front_lookahead_max_m:.1f}]m "
            f"side_rear={'true' if self.enable_side_rear_cost_stop else 'false'} "
            f"unavoidable_stop={'true' if self.enable_unavoidable_stop else 'false'}"
        )

    # Applies runtime parameter updates for gate/cost-stop tuning.
    def _on_set_parameters(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "enable_cost_stop":
                self.enable_cost_stop = bool(p.value)
            elif p.name == "cost_stop_threshold":
                self.cost_stop_threshold = int(p.value)
            elif p.name == "cost_stop_hold_s":
                self.cost_stop_hold_s = float(p.value)
            elif p.name == "cost_stop_hold_sec":
                # Legacy runtime parameter alias.
                self.cost_stop_hold_s = float(p.value)
            elif p.name == "cost_stop_lookahead_m":
                self.cost_stop_lookahead_m = float(p.value)
            elif p.name == "cost_stop_width_m":
                self.cost_stop_width_m = float(p.value)
            elif p.name == "enable_speed_dependent_lookahead":
                self.enable_speed_dependent_lookahead = bool(p.value)
            elif p.name == "front_lookahead_min_m":
                self.front_lookahead_min_m = float(p.value)
            elif p.name == "front_lookahead_max_m":
                self.front_lookahead_max_m = float(p.value)
            elif p.name == "front_lookahead_friction":
                self.front_lookahead_friction = float(p.value)
            elif p.name == "front_reaction_time_s":
                self.front_reaction_time_s = float(p.value)
            elif p.name == "front_lookahead_margin_m":
                self.front_lookahead_margin_m = float(p.value)
            elif p.name == "enable_side_rear_cost_stop":
                self.enable_side_rear_cost_stop = bool(p.value)
            elif p.name == "side_cost_threshold":
                self.side_cost_threshold = int(p.value)
            elif p.name == "side_lookahead_m":
                self.side_lookahead_m = float(p.value)
            elif p.name == "side_corridor_width_m":
                self.side_corridor_width_m = float(p.value)
            elif p.name == "rear_cost_threshold":
                self.rear_cost_threshold = int(p.value)
            elif p.name == "rear_lookahead_m":
                self.rear_lookahead_m = float(p.value)
            elif p.name == "rear_corridor_width_m":
                self.rear_corridor_width_m = float(p.value)
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

    # Stores latest merged near_cost_grid (all directions).
    def _on_cost_grid(self, msg: OccupancyGrid) -> None:
        self._last_grid = msg

    # Stores latest pose fallback.
    def _on_pose(self, msg: PoseStamped) -> None:
        self._last_pose = msg

    # Stores latest odometry; extracts forward speed for lookahead computation.
    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg
        # HH_260422: twist.twist.linear.x is robot-frame forward speed (m/s).
        self._current_speed = float(msg.twist.twist.linear.x)

    # Computes speed-dependent front lookahead distance using wet-road braking physics.
    def _compute_front_lookahead(self) -> float:
        speed = abs(self._current_speed)
        if not self.enable_speed_dependent_lookahead:
            return self.cost_stop_lookahead_m
        mu = max(0.05, self.front_lookahead_friction)
        braking = speed * speed / (2.0 * 9.8 * mu)
        reaction = self.front_reaction_time_s * speed
        raw = braking + reaction + self.front_lookahead_margin_m
        return max(self.front_lookahead_min_m, min(self.front_lookahead_max_m, raw))

    # Checks all directional corridors and triggers stop when any is blocked.
    def _should_stop_for_cost(self) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        now_sec = now_ns * 1e-9

        # --- Front stop (LiDAR grid, speed-dependent lookahead) ---
        lidar_grid = self._last_grid
        if lidar_grid is not None and lidar_grid.data and lidar_grid.info.resolution > 0.0:
            front_lookahead = self._compute_front_lookahead()
            target_frame = str(lidar_grid.header.frame_id).strip()
            pose_candidates = self._resolve_pose_candidates(target_frame)

            if not pose_candidates:
                if (now_sec - self._last_empty_corridor_warn_sec) >= 2.0:
                    self._last_empty_corridor_warn_sec = now_sec
                    self.get_logger().warn(
                        "cost-stop front: no pose candidates; "
                        "check pose/costmap frame alignment"
                    )
            else:
                best_total = -1
                best_label = ""
                best_lethal: list[tuple[int, int]] = []
                for label, pose in pose_candidates:
                    blocked, total_cells, lethal_cells = self._sample_cost_corridor(
                        lidar_grid, pose,
                        yaw_offset=0.0,
                        lookahead=front_lookahead,
                        width=self.cost_stop_width_m,
                        threshold=self.cost_stop_threshold,
                    )
                    if blocked:
                        self._cost_blocked_until = now_sec + self.cost_stop_hold_s
                        self.get_logger().warn(
                            f"cost-stop FRONT: source={label} "
                            f"lookahead={front_lookahead:.2f}m "
                            f"speed={self._current_speed:.2f}m/s "
                            f"hold={self.cost_stop_hold_s:.2f}s"
                        )
                        return True
                    if total_cells > best_total:
                        best_total = total_cells
                        best_label = label
                        best_lethal = lethal_cells

                if self.enable_unavoidable_stop and best_lethal:
                    if self._is_unavoidable_cluster(best_lethal, max(1, best_total)):
                        self._cost_blocked_until = now_sec + self.cost_stop_hold_s
                        self.get_logger().warn(
                            f"cost-stop FRONT (unavoidable): source={best_label} "
                            f"lethal={self._last_unavoidable_cluster_cells} "
                            f"ratio={self._last_unavoidable_cluster_ratio:.2f} "
                            f"hold={self.cost_stop_hold_s:.2f}s"
                        )
                        return True

        # --- Side / Rear stop (same merged grid, fixed short lookaheads) ---
        if self.enable_side_rear_cost_stop:
            merged_grid = self._last_grid
            if merged_grid is not None and merged_grid.data and merged_grid.info.resolution > 0.0:
                target_frame = str(merged_grid.header.frame_id).strip()
                pose_candidates = self._resolve_pose_candidates(target_frame)
                if pose_candidates:
                    label, pose = pose_candidates[0]
                    for direction, yaw_off, la, wd, thr in (
                        ("LEFT",  math.pi / 2,  self.side_lookahead_m, self.side_corridor_width_m, self.side_cost_threshold),
                        ("RIGHT", -math.pi / 2, self.side_lookahead_m, self.side_corridor_width_m, self.side_cost_threshold),
                        ("REAR",  math.pi,      self.rear_lookahead_m, self.rear_corridor_width_m, self.rear_cost_threshold),
                    ):
                        blocked, _, _ = self._sample_cost_corridor(
                            merged_grid, pose,
                            yaw_offset=yaw_off,
                            lookahead=la,
                            width=wd,
                            threshold=thr,
                        )
                        if blocked:
                            self._cost_blocked_until = now_sec + self.cost_stop_hold_s
                            self.get_logger().warn(
                                f"cost-stop {direction}: source={label} "
                                f"lookahead={la:.2f}m "
                                f"hold={self.cost_stop_hold_s:.2f}s"
                            )
                            return True

        return False

    # Samples a corridor in the direction (yaw + yaw_offset) for one pose candidate.
    def _sample_cost_corridor(
        self,
        grid: OccupancyGrid,
        pose: tuple[float, float, float],
        *,
        yaw_offset: float = 0.0,
        lookahead: float | None = None,
        width: float | None = None,
        threshold: int | None = None,
    ) -> tuple[bool, int, list[tuple[int, int]]]:
        pose_x, pose_y, yaw = pose
        effective_yaw = yaw + yaw_offset
        scan_lookahead = max(0.05, lookahead if lookahead is not None else self.cost_stop_lookahead_m)
        scan_width = max(0.05, width if width is not None else self.cost_stop_width_m)
        stop_threshold = threshold if threshold is not None else self.cost_stop_threshold
        res = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        w = int(grid.info.width)
        h = int(grid.info.height)
        cos_y = self._cos(effective_yaw)
        sin_y = self._sin(effective_yaw)

        step = res
        x = 0.0
        half_w = scan_width * 0.5
        total_cells = 0
        lethal_cells: list[tuple[int, int]] = []

        while x <= scan_lookahead:
            y = -half_w
            while y <= half_w:
                wx = pose_x + x * cos_y - y * sin_y
                wy = pose_y + x * sin_y + y * cos_y
                mx = int((wx - origin_x) / res)
                my = int((wy - origin_y) / res)
                if 0 <= mx < w and 0 <= my < h:
                    idx = my * w + mx
                    cost = int(grid.data[idx])
                    total_cells += 1
                    if cost >= stop_threshold:
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

        source_candidates: dict[str, list[tuple[str, tuple[float, float, float]]]] = {
            "tf_robot_base": [],
            "odometry": [],
            "pose_topic": [],
        }

        tf_pose = self._lookup_tf_pose(target_frame, self.robot_base_frame)
        if tf_pose is not None:
            source_candidates["tf_robot_base"].append(("tf_robot_base", tf_pose))

        source_candidates["odometry"] = self._resolve_odometry_candidates(target_frame)
        source_candidates["pose_topic"] = self._resolve_pose_topic_candidates(target_frame)

        preference = self.pose_source_preference.strip().lower()
        ordered_sources = {
            "tf_robot_base": ["tf_robot_base", "odometry", "pose_topic"],
            "odometry": ["odometry", "tf_robot_base", "pose_topic"],
            "pose_topic": ["pose_topic", "tf_robot_base", "odometry"],
        }.get(preference, ["odometry", "tf_robot_base", "pose_topic"])

        candidates: list[tuple[str, tuple[float, float, float]]] = []
        for source in ordered_sources:
            candidates.extend(source_candidates.get(source, []))
        return candidates

    # Resolves pose-topic candidates in target frame.
    def _resolve_pose_topic_candidates(
        self, target_frame: str
    ) -> list[tuple[str, tuple[float, float, float]]]:
        msg = self._last_pose
        if msg is None:
            return []
        return self._resolve_stamped_pose_candidates(
            label_prefix="pose",
            source_frame=str(msg.header.frame_id).strip(),
            x=float(msg.pose.position.x),
            y=float(msg.pose.position.y),
            yaw=self._yaw_from_quat(msg.pose.orientation),
            target_frame=target_frame,
        )

    # Resolves odometry candidates in target frame.
    def _resolve_odometry_candidates(
        self, target_frame: str
    ) -> list[tuple[str, tuple[float, float, float]]]:
        msg = self._last_odom
        if msg is None:
            return []
        return self._resolve_stamped_pose_candidates(
            label_prefix="odom",
            source_frame=str(msg.header.frame_id).strip(),
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=self._yaw_from_quat(msg.pose.pose.orientation),
            target_frame=target_frame,
        )

    # Converts a stamped 2D pose candidate to target frame.
    def _resolve_stamped_pose_candidates(
        self,
        *,
        label_prefix: str,
        source_frame: str,
        x: float,
        y: float,
        yaw: float,
        target_frame: str,
    ) -> list[tuple[str, tuple[float, float, float]]]:
        candidates: list[tuple[str, tuple[float, float, float]]] = []
        if not source_frame or source_frame == target_frame:
            candidates.append((f"{label_prefix}_raw", (x, y, yaw)))
            return candidates

        tf_msg = self._lookup_transform(target_frame, source_frame)
        if tf_msg is not None:
            tx = float(tf_msg.transform.translation.x)
            ty = float(tf_msg.transform.translation.y)
            tyaw = self._yaw_from_quat(tf_msg.transform.rotation)
            rx, ry = self._rotate_xy(x, y, tyaw)
            candidates.append(
                (f"{label_prefix}_tf", (tx + rx, ty + ry, self._normalize_yaw(tyaw + yaw)))
            )
            return candidates

        if self.enable_pose_raw_fallback:
            candidates.append((f"{label_prefix}_raw_fallback", (x, y, yaw)))
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
