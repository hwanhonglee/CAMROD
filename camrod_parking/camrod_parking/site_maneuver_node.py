#!/usr/bin/env python3
# HH_260617: Rule-based campsite crab entry/exit and 180-degree body rotation.

from __future__ import annotations

import math
import os
import re
from enum import Enum

import rclpy
import yaml
from avg_msgs.msg import AvgAmrServiceState, ModuleState, PlanningScenario, PlanningState
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .utils import (
    clamp,
    make_diagnostics,
    make_module_state,
    normalize_angle,
    relative_xy,
    yaw_from_pose,
)


class Phase(str, Enum):
    IDLE = "IDLE"
    ALIGN_ENTRY_YAW = "ALIGN_ENTRY_YAW"
    REVERSE_IN = "REVERSE_IN"
    CRAB_IN = "CRAB_IN"
    ROTATE_180 = "ROTATE_180"
    UNLOAD_WAIT = "UNLOAD_WAIT"
    WAIT_RETURN = "WAIT_RETURN"
    REVERSE_OUT = "REVERSE_OUT"
    CRAB_OUT = "CRAB_OUT"
    DONE = "DONE"
    ERROR = "ERROR"


def quat_from_yaw(yaw: float):
    q = PoseStamped().pose.orientation
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class SiteManeuverNode(Node):
    def __init__(self) -> None:
        super().__init__("site_maneuver")

        self.cmd_vel_topic = str(self.declare_parameter("cmd_vel_topic", "/planning/cmd_vel_raw").value)
        self.pose_topic = str(self.declare_parameter("pose_topic", "/localization/pose").value)
        # HH_260618: Site center is the raw operator/UI goal on /goal_pose.
        # /planning/goal_pose_snapped_ros is the ROS-native lanelet route goal;
        # /planning/goal_pose_snapped carries avg_msgs for C++ planning nodes.
        self.site_goal_topic = str(self.declare_parameter("site_goal_topic", "/goal_pose").value)
        self.route_goal_topic = str(
            self.declare_parameter("route_goal_topic", "/planning/goal_pose_snapped_ros").value
        )
        self.planning_state_topic = str(
            self.declare_parameter("planning_state_topic", "/planning/state_machine/state").value
        )
        self.start_topic = str(
            self.declare_parameter("start_topic", "/parking/site_maneuver/start").value
        )
        self.return_topic = str(
            self.declare_parameter("return_topic", "/parking/site_maneuver/return").value
        )
        self.cancel_topic = str(
            self.declare_parameter("cancel_topic", "/parking/site_maneuver/cancel").value
        )
        self.status_topic = str(
            self.declare_parameter("status_topic", "/parking/site_maneuver/status").value
        )
        self.diagnostics_topic = str(self.declare_parameter("diagnostics_topic", "/system/diagnostics").value)
        # HHL_260622 - Parking phase updates the UI/system service state during non-Nav2 maneuvers.
        self.amr_service_state_topic = str(
            self.declare_parameter("amr_service_state_topic", "/AMR_service_state").value
        )
        # HH_260619 - Publish the planned campsite reverse-in/reverse-out path for RViz verification.
        self.reverse_path_topic = str(
            self.declare_parameter("reverse_path_topic", "/parking/site_maneuver/reverse_path").value
        )

        # HH_260617: Auto mode is based on PlanningState.GOAL_REACHED so Nav2
        # remains responsible for lanelet-snap approach before parking motion.
        self.enable_auto_start_from_planning_state = bool(
            self.declare_parameter("enable_auto_start_from_planning_state", True).value
        )
        self.site_mission_key_prefix = str(
            self.declare_parameter("site_mission_key_prefix", "camping_site_").value
        )
        self.use_goal_pair_for_lateral_offset = bool(
            self.declare_parameter("use_goal_pair_for_lateral_offset", True).value
        )
        # HH_260618: Auto-started campsite parking must use the actual
        # route-goal/site-goal pair. Falling back silently made the maneuver
        # look successful even when the site center was never received.
        self.require_goal_pair_for_auto_start = bool(
            self.declare_parameter("require_goal_pair_for_auto_start", True).value
        )
        # HHL_260622 - Campsite entry defaults to wheel-crab lateral motion:
        # lanelet-snap approach -> crab into the site -> body rotate 180deg
        # only after the robot is inside the selected camping site.
        self.site_entry_mode = str(self.declare_parameter("site_entry_mode", "crab").value).strip().lower()
        if self.site_entry_mode not in {"reverse", "crab"}:
            self.get_logger().warn(
                f"unknown site_entry_mode='{self.site_entry_mode}'; using crab"
            )
            self.site_entry_mode = "crab"
        self.enable_manual_site_goal_auto_start = bool(
            self.declare_parameter("enable_manual_site_goal_auto_start", True).value
        )
        self.manual_site_min_offset_m = abs(
            float(self.declare_parameter("manual_site_min_offset_m", 1.0).value)
        )
        self.camping_sites_yaml = str(self.declare_parameter("camping_sites_yaml", "").value)
        self.site_goal_frame_id = str(self.declare_parameter("site_goal_frame_id", "map").value)
        self.default_lateral_offset_m = float(
            self.declare_parameter("default_lateral_offset_m", 1.2).value
        )
        self.min_lateral_offset_m = float(
            self.declare_parameter("min_lateral_offset_m", 0.2).value
        )
        self.max_lateral_offset_m = float(
            self.declare_parameter("max_lateral_offset_m", 7.0).value
        )
        self.default_lateral_direction = str(
            self.declare_parameter("default_lateral_direction", "left").value
        ).strip().lower()
        self.crab_speed_mps = abs(float(self.declare_parameter("crab_speed_mps", 0.18).value))
        self.entry_position_tolerance_m = abs(
            float(self.declare_parameter("entry_position_tolerance_m", 0.15).value)
        )
        self.return_position_tolerance_m = abs(
            float(self.declare_parameter("return_position_tolerance_m", 0.18).value)
        )
        self.crab_timeout_margin_s = abs(
            float(self.declare_parameter("crab_timeout_margin_s", 3.0).value)
        )
        # HH_260619 - Reverse-out follows a curved nonholonomic path back to the
        # lanelet snap pose, so it needs a separate margin from crab/reverse-in
        # timeout prediction.
        self.reverse_return_timeout_margin_s = abs(
            float(self.declare_parameter("reverse_return_timeout_margin_s", 45.0).value)
        )
        # HH_260619 - Reverse-out completion uses progress along the return
        # travel axis plus lateral error to the lanelet-snap line. These
        # tolerances avoid overshooting forever when the robot has crossed the
        # snap pose between low-speed control ticks.
        self.reverse_return_progress_tolerance_m = abs(
            float(self.declare_parameter("reverse_return_progress_tolerance_m", 0.45).value)
        )
        self.reverse_return_lateral_tolerance_m = abs(
            float(self.declare_parameter("reverse_return_lateral_tolerance_m", 0.40).value)
        )
        # HH_260618: The planning cmd_vel gate can scale `/planning/cmd_vel_raw`
        # before the simulator/platform integrates motion. Use the expected
        # effective scale for timeout prediction so long campsite crab entries
        # do not fail while the robot is still moving correctly.
        self.crab_timeout_speed_scale = max(
            0.05,
            abs(float(self.declare_parameter("crab_timeout_speed_scale", 1.0).value)),
        )
        self.max_forward_residual_m = abs(
            float(self.declare_parameter("max_forward_residual_m", 0.8).value)
        )
        self.rotate_kp = float(self.declare_parameter("rotate_kp", 1.2).value)
        self.max_angular_speed_radps = abs(
            float(self.declare_parameter("max_angular_speed_radps", 0.35).value)
        )
        self.rotate_yaw_tolerance_deg = abs(
            float(self.declare_parameter("rotate_yaw_tolerance_deg", 4.0).value)
        )
        # HHL_260624 - Force campsite 180deg body rotation toward the lanelet side.
        self.site_rotate_direction_policy = str(
            self.declare_parameter("site_rotate_direction_policy", "site_index_lanelet_side").value
        ).strip().lower()
        if self.site_rotate_direction_policy not in {"site_index_lanelet_side", "shortest"}:
            self.get_logger().warn(
                f"unknown site_rotate_direction_policy='{self.site_rotate_direction_policy}'; using shortest"
            )
            self.site_rotate_direction_policy = "shortest"
        self.right_lanelet_site_indices = self._int_set(
            self.declare_parameter("right_lanelet_site_indices", [1, 3, 5, 7, 9, 11]).value
        )
        self.left_lanelet_site_indices = self._int_set(
            self.declare_parameter("left_lanelet_site_indices", [2, 4, 6, 8, 10, 12, 13]).value
        )
        # HHL_260622 - Match the package/config fallback reverse speed.
        self.reverse_entry_speed_mps = abs(
            float(self.declare_parameter("reverse_entry_speed_mps", 0.16).value)
        )
        self.reverse_entry_yaw_kp = float(self.declare_parameter("reverse_entry_yaw_kp", 1.0).value)
        self.reverse_entry_lateral_kp = float(
            self.declare_parameter("reverse_entry_lateral_kp", -0.25).value
        )
        self.reverse_entry_max_angular_speed_radps = abs(
            float(self.declare_parameter("reverse_entry_max_angular_speed_radps", 0.25).value)
        )
        # HH_260619 - Campsite reverse entry should track the semantic site yaw,
        # not only the straight vector from the lanelet snap pose to the site
        # center. The default treats site yaw as the desired reverse travel
        # axis; the robot body target is therefore site_yaw + 180deg.
        self.reverse_entry_use_site_goal_yaw = bool(
            self.declare_parameter("reverse_entry_use_site_goal_yaw", True).value
        )
        self.reverse_entry_site_yaw_mode = str(
            self.declare_parameter("reverse_entry_site_yaw_mode", "reverse_axis").value
        ).strip().lower()
        if self.reverse_entry_site_yaw_mode not in {"reverse_axis", "robot_yaw"}:
            self.get_logger().warn(
                f"unknown reverse_entry_site_yaw_mode='{self.reverse_entry_site_yaw_mode}'; using reverse_axis"
            )
            self.reverse_entry_site_yaw_mode = "reverse_axis"
        self.reverse_entry_auto_select_yaw_equivalent = bool(
            self.declare_parameter("reverse_entry_auto_select_yaw_equivalent", True).value
        )
        self.reverse_entry_lateral_tolerance_m = abs(
            float(self.declare_parameter("reverse_entry_lateral_tolerance_m", 0.35).value)
        )
        self.reverse_entry_debug_period_s = float(
            self.declare_parameter("reverse_entry_debug_period_s", 1.0).value
        )
        self.unload_wait_s = float(self.declare_parameter("unload_wait_s", 5.0).value)
        # HH_260618: Default campsite behavior is to remain parked inside the
        # selected site after unload. Automatic return is opt-in for scripted
        # demos only; otherwise the robot leaves the site immediately.
        self.auto_return_after_unload_wait = bool(
            self.declare_parameter("auto_return_after_unload_wait", False).value
        )
        # HH_260619 - Re-selecting a campsite while the node waits for an
        # external return command is an explicit new mission/test request. Without
        # this, selecting the same site again keeps WAIT_RETURN because the raw
        # site-goal coordinates are unchanged.
        self.reset_wait_return_on_site_goal = bool(
            self.declare_parameter("reset_wait_return_on_site_goal", True).value
        )
        self.request_return_to_drop_zone_on_done = bool(
            self.declare_parameter("request_return_to_drop_zone_on_done", True).value
        )
        # HH_260618: Return request is a mission command, not a telemetry sample.
        # Retry until the planning state machine acknowledges RETURN_TO_DROP_ZONE.
        self.return_request_retry_period_s = float(
            self.declare_parameter("return_request_retry_period_s", 1.0).value
        )
        self.return_to_drop_zone_topic = str(
            self.declare_parameter(
                "return_to_drop_zone_topic", "/planning/state_machine/return_to_drop_zone"
            ).value
        )
        self.pose_timeout_s = float(self.declare_parameter("pose_timeout_s", 2.0).value)
        # HH_260618: Guard automatic campsite entry with the actual latest
        # route-goal distance. This prevents stale GOAL_REACHED samples from
        # starting crab motion before Nav2 reaches the lanelet-snap pose.
        self.route_goal_reached_distance_m = abs(
            float(self.declare_parameter("route_goal_reached_distance_m", 0.9).value)
        )
        # HH_260618: Keep the route/site pair valid for the whole Nav2 approach.
        # This is command context, not sensor data; a long drive can exceed 60 s.
        # New /goal_pose or route_goal messages replace/cancel the old pair.
        # Set >0 only for short scripted tests that require time-based expiry.
        self.goal_pair_max_age_s = float(self.declare_parameter("goal_pair_max_age_s", 0.0).value)
        # HH_260618: Low-speed crab/rotation maneuver does not need a 20 Hz
        # Python timer while idle; 10 Hz reduces baseline CPU.
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 10.0).value)
        # HH_260618: Use a slow idle timer and switch to control_rate_hz only
        # while a parking maneuver is active.
        self.idle_tick_rate_hz = float(self.declare_parameter("idle_tick_rate_hz", 1.0).value)
        # HH_260618: Status/diagnostics do not need control-rate publishing.
        self.status_publish_rate_hz = float(
            self.declare_parameter("status_publish_rate_hz", 1.0).value
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(ModuleState, self.status_topic, 10)
        self.diag_pub = self.create_publisher(type(make_diagnostics(self, "", "", 0, "")), self.diagnostics_topic, 10)
        self.return_request_pub = self.create_publisher(Bool, self.return_to_drop_zone_topic, 10)
        self.amr_service_pub = self.create_publisher(
            AvgAmrServiceState, self.amr_service_state_topic, 10
        )
        self.reverse_path_pub = self.create_publisher(Path, self.reverse_path_topic, 10)

        self.create_subscription(PoseStamped, self.pose_topic, self._on_pose, 10)
        self.create_subscription(PoseStamped, self.site_goal_topic, self._on_site_goal, 10)
        self.create_subscription(PoseStamped, self.route_goal_topic, self._on_route_goal, 10)
        self.create_subscription(PlanningState, self.planning_state_topic, self._on_planning_state, 10)
        self.create_subscription(Bool, self.start_topic, self._on_start_bool, 10)
        self.create_subscription(Bool, self.return_topic, self._on_return_bool, 10)
        self.create_subscription(Bool, self.cancel_topic, self._on_cancel_bool, 10)
        self.create_service(Trigger, "/parking/site_maneuver/start_service", self._on_start_service)
        self.create_service(Trigger, "/parking/site_maneuver/return_service", self._on_return_service)
        self.create_service(Trigger, "/parking/site_maneuver/cancel_service", self._on_cancel_service)

        self.phase = Phase.IDLE
        self.last_pose: PoseStamped | None = None
        self.last_pose_time_s = 0.0
        self.site_goal: PoseStamped | None = None
        self.route_goal: PoseStamped | None = None
        self.site_goal_time_s = 0.0
        self.route_goal_time_s = 0.0
        self.site_goal_key = ""
        self.phase_start_s = self._now_s()
        self.start_yaw = 0.0
        self.target_yaw = 0.0
        self.rotate_direction_sign = 0.0
        self.rotate_direction_label = "shortest"
        self.entry_target_yaw = 0.0
        self.entry_reverse_axis_yaw = 0.0
        self.entry_line_origin_x = 0.0
        self.entry_line_origin_y = 0.0
        self.entry_target_progress_m = 0.0
        self.entry_target_x = 0.0
        self.entry_target_y = 0.0
        self.entry_reverse_distance_m = 0.0
        self.return_start_x = 0.0
        self.return_start_y = 0.0
        self.crab_direction = 1.0
        self.crab_duration_s = 0.0
        self.crab_offset_m = 0.0
        self.crab_source = "default"
        self.goal_pair_forward_m = 0.0
        self.entry_start_x = 0.0
        self.entry_start_y = 0.0
        self.entry_reference_yaw = 0.0
        self.last_auto_key = ""
        self.return_requested = False
        self.return_published = False
        self.return_acknowledged = False
        self.last_return_request_publish_s = 0.0
        self.last_reverse_entry_debug_s = 0.0
        self.last_status_publish_s = 0.0
        self.camping_site_goals: dict[str, PoseStamped] = {}
        self._load_camping_sites()

        self.timer = self.create_timer(self._timer_period_s(), self._tick)
        self.get_logger().info(
            "site_maneuver ready: "
            f"cmd={self.cmd_vel_topic} pose={self.pose_topic} "
            f"auto_start={self.enable_auto_start_from_planning_state} "
            f"entry_mode={self.site_entry_mode}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # HHL_260624 - Normalize YAML list/string campsite index parameters.
    @staticmethod
    def _int_set(value: object) -> set[int]:
        if isinstance(value, str):
            raw_items = [item.strip() for item in value.split(",")]
        elif isinstance(value, (list, tuple, set)):
            raw_items = list(value)
        else:
            raw_items = [value]
        result: set[int] = set()
        for item in raw_items:
            try:
                result.add(int(item))
            except (TypeError, ValueError):
                continue
        return result

    def _is_active_phase(self) -> bool:
        return self.phase not in {Phase.IDLE, Phase.DONE, Phase.ERROR}

    def _is_site_internal_phase(self) -> bool:
        # HHL_260622 - Once the robot is inside or leaving a campsite, a new
        # campsite goal must not overwrite the original site/route pair. The
        # only safe transition is to finish crab-out first, then let planning
        # route from the lanelet snap pose.
        return self.phase in {
            Phase.ALIGN_ENTRY_YAW,
            Phase.REVERSE_IN,
            Phase.CRAB_IN,
            Phase.ROTATE_180,
            Phase.UNLOAD_WAIT,
            Phase.WAIT_RETURN,
            Phase.REVERSE_OUT,
            Phase.CRAB_OUT,
        }

    def _timer_period_s(self) -> float:
        rate_hz = self.control_rate_hz if self._is_active_phase() else self.idle_tick_rate_hz
        return 1.0 / max(rate_hz, 0.1)

    def _update_timer_period(self) -> None:
        if hasattr(self, "timer"):
            self.timer.timer_period_ns = int(self._timer_period_s() * 1e9)

    def _make_pose(self, frame_id: str, x: float, y: float, z: float, yaw_deg: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = frame_id or self.site_goal_frame_id or "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation = quat_from_yaw(math.radians(float(yaw_deg)))
        return pose

    def _copy_pose(self, msg: PoseStamped) -> PoseStamped:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose
        return pose

    def _publish_reverse_path(
        self,
        start_x: float,
        start_y: float,
        end_x: float,
        end_y: float,
        robot_yaw: float,
    ) -> None:
        # HH_260619 - Visualize the low-speed rule-based reverse path separately from Nav2 paths.
        path = Path()
        stamp = self.get_clock().now().to_msg()
        frame_id = self.site_goal_frame_id or "map"
        if self.last_pose is not None and self.last_pose.header.frame_id:
            frame_id = self.last_pose.header.frame_id
        path.header.frame_id = frame_id
        path.header.stamp = stamp

        distance = math.hypot(end_x - start_x, end_y - start_y)
        steps = max(1, int(distance / 0.25))
        for index in range(steps + 1):
            ratio = index / steps
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = stamp
            pose.pose.position.x = start_x + (end_x - start_x) * ratio
            pose.pose.position.y = start_y + (end_y - start_y) * ratio
            pose.pose.orientation = quat_from_yaw(robot_yaw)
            path.poses.append(pose)
        self.reverse_path_pub.publish(path)

    def _stamp_pose_now(self, pose: PoseStamped) -> PoseStamped:
        pose.header.stamp = self.get_clock().now().to_msg()
        if not pose.header.frame_id:
            pose.header.frame_id = self.site_goal_frame_id or "map"
        return pose

    def _load_camping_sites(self) -> None:
        self.camping_site_goals.clear()
        path = self.camping_sites_yaml.strip()
        if not path:
            return
        if not os.path.isfile(path):
            self.get_logger().warn(f"camping_sites_yaml not found: {path}")
            return
        try:
            with open(path, "r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
        except Exception as exc:
            self.get_logger().warn(f"failed to load camping_sites_yaml ({path}): {exc}")
            return
        sites = data.get("camping_sites", data if isinstance(data, list) else [])
        if not isinstance(sites, list):
            self.get_logger().warn(f"invalid camping_sites_yaml format: {path}")
            return
        for item in sites:
            if not isinstance(item, dict):
                continue
            try:
                pose = self._make_pose(
                    str(item.get("frame_id", self.site_goal_frame_id or "map")),
                    float(item["x"]),
                    float(item["y"]),
                    float(item.get("z", 0.0)),
                    float(item.get("yaw_deg", 0.0)),
                )
            except Exception as exc:
                self.get_logger().warn(f"skipped invalid camping site entry in {path}: {exc}")
                continue
            for key_field in ("type", "id", "name"):
                key = str(item.get(key_field, "")).strip()
                if key:
                    self.camping_site_goals[key] = self._copy_pose(pose)
        self.get_logger().info(
            f"loaded {len(self.camping_site_goals)} campsite goal aliases from {path}"
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        self.last_pose = msg
        self.last_pose_time_s = self._now_s()

    def _on_site_goal(self, msg: PoseStamped) -> None:
        if self._is_site_internal_phase():
            ok, message = self._request_return("site_goal_blocked_inside_site")
            self.get_logger().warn(
                "site_maneuver ignored new site_goal while site maneuver is internal: "
                f"phase={self.phase.value} x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f} "
                f"return_request={str(ok).lower()} message='{message}'"
            )
            return

        old_goal = self.site_goal
        self.site_goal = msg
        self.site_goal_time_s = self._now_s()
        self.site_goal_key = ""
        goal_changed = old_goal is not None and self._pose_distance(old_goal, msg) > 0.2
        waiting_return_reselected = (
            self.reset_wait_return_on_site_goal
            and self.phase == Phase.WAIT_RETURN
        )
        if old_goal is None or goal_changed or self.phase in {Phase.IDLE, Phase.DONE, Phase.ERROR}:
            # HH_260619 - Treat every explicit idle/done/error campsite command
            # as a retryable command so P-reset or same-site reselection can
            # restart the entry sequence instead of being filtered as stale.
            self.last_auto_key = ""
        if self._is_active_phase() and (goal_changed or waiting_return_reselected):
            self._publish_zero()
            self.last_auto_key = ""
            self.return_requested = False
            self.return_published = False
            self.return_acknowledged = False
            reason = (
                "new site_goal received; stale maneuver cancelled"
                if goal_changed
                else "site_goal reselected during WAIT_RETURN; maneuver reset"
            )
            self._set_phase(Phase.IDLE, reason)
        self.get_logger().info(
            "site_maneuver site_goal updated: "
            f"topic={self.site_goal_topic} x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}"
        )

    def _on_route_goal(self, msg: PoseStamped) -> None:
        self.route_goal = msg
        self.route_goal_time_s = self._now_s()
        self.last_auto_key = ""

    def _site_goal_matches_key(self, key: str) -> bool:
        if not key or self.site_goal is None:
            return False
        expected = self.camping_site_goals.get(key)
        if expected is None:
            return True
        return self._pose_distance(self.site_goal, expected) <= 0.5

    def _ensure_goal_pair_for_auto_start(self, key: str) -> None:
        # HH_260618: Auto site maneuver is triggered after Nav2 reaches the
        # lanelet-snap route goal. If the transient raw/site or snapped/route
        # topic was missed, rebuild the pair from mission config and current pose
        # instead of failing with "site/route goal pair unavailable".
        now_s = self._now_s()
        site_from_config = self.camping_site_goals.get(key)
        if site_from_config is not None and (
            self.site_goal is None or not self._site_goal_matches_key(key)
        ):
            self.site_goal = self._stamp_pose_now(self._copy_pose(site_from_config))
            self.site_goal_time_s = now_s
            self.site_goal_key = key
            self.get_logger().info(
                "site_maneuver restored site_goal from camping_sites_yaml: "
                f"key={key} x={self.site_goal.pose.position.x:.2f} "
                f"y={self.site_goal.pose.position.y:.2f}"
            )
        if self.route_goal is None and self._pose_is_fresh():
            self.route_goal = self._stamp_pose_now(self._copy_pose(self.last_pose))  # type: ignore[arg-type]
            self.route_goal_time_s = now_s
            self.get_logger().info(
                "site_maneuver restored route_goal from current pose at Nav2 GOAL_REACHED"
            )

    # HHL_260624 - Extract campsite index from semantic mission strings.
    def _site_index_from_text(self, text: str) -> int | None:
        match = re.search(r"camping_site_(\d+)", text or "")
        if match is None:
            return None
        return int(match.group(1))

    # HHL_260624 - Map odd/even park campsite groups to lanelet-side rotation direction.
    def _rotation_direction_for_source(self, source: str) -> tuple[float, str]:
        if self.site_rotate_direction_policy != "site_index_lanelet_side":
            return 0.0, "shortest"
        site_index = (
            self._site_index_from_text(source)
            or self._site_index_from_text(self.site_goal_key)
            or self._site_index_from_text(self.last_auto_key)
        )
        if site_index is None:
            return 0.0, "shortest"
        if site_index in self.right_lanelet_site_indices:
            return -1.0, f"right_cw_site_{site_index}"
        if site_index in self.left_lanelet_site_indices:
            return 1.0, f"left_ccw_site_{site_index}"
        return 0.0, f"shortest_site_{site_index}"

    def _on_planning_state(self, msg: PlanningState) -> None:
        if (
            msg.scenario_id == int(PlanningScenario.RETURN_TO_DROP_ZONE)
            or msg.active_mission_key.strip() == "drop_zone"
        ):
            self.return_acknowledged = True
        if not self.enable_auto_start_from_planning_state:
            return
        if msg.state != PlanningState.GOAL_REACHED:
            if self.phase in {Phase.IDLE, Phase.DONE, Phase.ERROR}:
                self.last_auto_key = ""
            return
        if msg.scenario_id not in {
            int(PlanningScenario.DELIVERY_TO_SITE),
            int(PlanningScenario.RECALL_TO_SITE),
        }:
            return
        key = msg.active_mission_key.strip()
        auto_key = ""
        if key.startswith(self.site_mission_key_prefix):
            self._ensure_goal_pair_for_auto_start(key)
            auto_key = key
        elif self._manual_site_goal_is_ready(msg):
            # HH_260618: RViz/manual off-lane goals are allowed to behave like a
            # campsite request when the raw goal and snapped route goal differ
            # enough to describe a parking-site entry.
            auto_key = f"manual_site:{self.site_goal_time_s:.3f}:{self.route_goal_time_s:.3f}"
        elif self._site_goal_pair_is_ready():
            # HH_260619 - Separate parking entry readiness from fragile semantic
            # key matching. A valid raw campsite goal plus reached route goal is
            # sufficient when PlanningState is already in a site scenario.
            auto_key = f"site_pair:{self.site_goal_time_s:.3f}:{self.route_goal_time_s:.3f}"
        else:
            return
        if auto_key == self.last_auto_key and self.phase not in {Phase.IDLE, Phase.DONE}:
            return
        if auto_key == self.last_auto_key and self.phase == Phase.DONE:
            return
        if not self._route_goal_reached_for_auto_start():
            return
        ok, _ = self._start_sequence(f"planning_state:{auto_key}")
        if ok:
            self.last_auto_key = auto_key

    def _on_start_bool(self, msg: Bool) -> None:
        if msg.data:
            self._start_sequence("topic")

    def _on_return_bool(self, msg: Bool) -> None:
        if msg.data:
            self._request_return("topic")

    def _on_cancel_bool(self, msg: Bool) -> None:
        if msg.data:
            self._cancel("topic")

    def _on_start_service(self, _request, response):
        ok, message = self._start_sequence("service")
        response.success = ok
        response.message = message
        return response

    def _on_return_service(self, _request, response):
        ok, message = self._request_return("service")
        response.success = ok
        response.message = message
        return response

    def _on_cancel_service(self, _request, response):
        self._cancel("service")
        response.success = True
        response.message = "site maneuver cancelled"
        return response

    def _start_sequence(self, source: str) -> tuple[bool, str]:
        if self.phase not in {Phase.IDLE, Phase.DONE, Phase.ERROR}:
            return False, f"site maneuver already active: {self.phase.value}"
        if not self._pose_is_fresh():
            self._set_error("fresh pose unavailable")
            return False, "fresh pose unavailable"

        lateral_offset, direction, source_name, forward_residual = self._resolve_lateral_motion()
        if (
            source.startswith("planning_state:")
            and self.require_goal_pair_for_auto_start
            and source_name != "goal_pair"
        ):
            self._set_error("site/route goal pair unavailable for auto site maneuver")
            return False, "site/route goal pair unavailable for auto site maneuver"
        if self.site_entry_mode == "reverse":
            return self._start_reverse_entry_sequence(source, source_name, forward_residual)
        if lateral_offset <= 0.0 or self.crab_speed_mps <= 0.0:
            self._set_error("invalid lateral motion parameters")
            return False, "invalid lateral motion parameters"

        self.start_yaw = yaw_from_pose(self.last_pose)  # type: ignore[arg-type]
        self.rotate_direction_sign, self.rotate_direction_label = self._rotation_direction_for_source(source)
        rotation_delta = self.rotate_direction_sign * math.pi if self.rotate_direction_sign else math.pi
        self.target_yaw = normalize_angle(self.start_yaw + rotation_delta)
        self.crab_direction = direction
        self.crab_offset_m = lateral_offset
        self.crab_source = source_name
        self.goal_pair_forward_m = forward_residual
        self.entry_start_x = self.last_pose.pose.position.x  # type: ignore[union-attr]
        self.entry_start_y = self.last_pose.pose.position.y  # type: ignore[union-attr]
        self.entry_reference_yaw = self.start_yaw
        effective_crab_speed = max(0.01, self.crab_speed_mps * self.crab_timeout_speed_scale)
        self.crab_duration_s = lateral_offset / effective_crab_speed
        self.return_requested = False
        self.return_published = False
        self.return_acknowledged = False
        self.last_return_request_publish_s = 0.0
        if source_name == "goal_pair" and abs(forward_residual) > self.max_forward_residual_m:
            # HHL_260622 - rclpy logger does not accept printf-style extra args.
            self.get_logger().warn(
                f"site_maneuver goal pair has forward residual {forward_residual:.2f}m. "
                "Crab-only parking corrects lateral offset; check lanelet snap if the site is not perpendicular."
            )
        self._set_phase(
            Phase.CRAB_IN,
            f"start={source} offset={lateral_offset:.2f}m direction={direction:+.0f} "
            f"rotate={self.rotate_direction_label} "
            f"source={source_name} forward={forward_residual:.2f}m "
            f"timeout_speed_scale={self.crab_timeout_speed_scale:.2f} "
            f"duration={self.crab_duration_s:.1f}s",
        )
        return True, "site maneuver started"

    def _start_reverse_entry_sequence(
        self, source: str, source_name: str, forward_residual: float
    ) -> tuple[bool, str]:
        if self.site_goal is None:
            self._set_error("site goal unavailable for reverse site maneuver")
            return False, "site goal unavailable for reverse site maneuver"
        if self.reverse_entry_speed_mps <= 0.0:
            self._set_error("invalid reverse entry speed")
            return False, "invalid reverse entry speed"

        self.entry_start_x = self.last_pose.pose.position.x  # type: ignore[union-attr]
        self.entry_start_y = self.last_pose.pose.position.y  # type: ignore[union-attr]
        self.entry_target_x = self.site_goal.pose.position.x
        self.entry_target_y = self.site_goal.pose.position.y
        dx = self.entry_target_x - self.entry_start_x
        dy = self.entry_target_y - self.entry_start_y
        distance = math.hypot(dx, dy)
        if distance < self.min_lateral_offset_m:
            self._set_error("reverse site target is too close to lanelet snap pose")
            return False, "reverse site target is too close to lanelet snap pose"
        if distance > self.max_lateral_offset_m:
            self._set_error(
                f"reverse site target distance {distance:.2f}m exceeds max {self.max_lateral_offset_m:.2f}m"
            )
            return False, "reverse site target exceeds max distance"

        self.start_yaw = yaw_from_pose(self.last_pose)  # type: ignore[arg-type]
        direct_axis_yaw = math.atan2(dy, dx)
        self.entry_reverse_axis_yaw, reverse_axis_source = self._select_reverse_entry_axis(
            direct_axis_yaw, dx, dy
        )
        self.entry_target_yaw = normalize_angle(self.entry_reverse_axis_yaw + math.pi)
        self.target_yaw = self.entry_target_yaw
        self.rotate_direction_sign = 0.0
        self.rotate_direction_label = "shortest"
        self.entry_line_origin_x = self.entry_target_x
        self.entry_line_origin_y = self.entry_target_y
        self.entry_target_progress_m = (
            math.cos(self.entry_reverse_axis_yaw) * dx
            + math.sin(self.entry_reverse_axis_yaw) * dy
        )
        self.entry_reverse_distance_m = distance
        self.crab_offset_m = distance
        self.crab_source = source_name
        self.goal_pair_forward_m = forward_residual
        # HH_260618: Reverse entry also passes through the planning cmd_vel
        # gate, so timeout prediction must use the same effective speed scale
        # as crab entry. Without this, long campsite reverse maneuvers can
        # error while the robot is still moving correctly.
        effective_reverse_speed = max(
            0.01,
            self.reverse_entry_speed_mps * self.crab_timeout_speed_scale,
        )
        self.crab_duration_s = distance / effective_reverse_speed
        self.return_requested = False
        self.return_published = False
        self.return_acknowledged = False
        self.last_return_request_publish_s = 0.0
        self._publish_reverse_path(
            self.entry_start_x,
            self.entry_start_y,
            self.entry_target_x,
            self.entry_target_y,
            self.entry_target_yaw,
        )
        self._set_phase(
            Phase.ALIGN_ENTRY_YAW,
            f"start={source} reverse_distance={distance:.2f}m "
            f"target=({self.entry_target_x:.2f},{self.entry_target_y:.2f}) "
            f"source={source_name} forward={forward_residual:.2f}m "
            f"axis={reverse_axis_source} axis_yaw={math.degrees(self.entry_reverse_axis_yaw):.1f}deg "
            f"body_yaw={math.degrees(self.entry_target_yaw):.1f}deg",
        )
        return True, "site reverse maneuver started"

    def _request_return(self, source: str) -> tuple[bool, str]:
        if self.phase == Phase.WAIT_RETURN:
            self.return_requested = True
            next_phase = Phase.REVERSE_OUT if self.site_entry_mode == "reverse" else Phase.CRAB_OUT
            self._set_phase(next_phase, f"return={source}")
            return True, "site maneuver return started"
        if self.phase in {Phase.IDLE, Phase.DONE}:
            return False, f"site maneuver is not waiting for return: {self.phase.value}"
        self.return_requested = True
        return True, f"return request latched during {self.phase.value}"

    def _cancel(self, source: str) -> None:
        self._publish_zero()
        self._set_phase(Phase.IDLE, f"cancel={source}")

    def _resolve_lateral_motion(self) -> tuple[float, float, str, float]:
        offset = self.default_lateral_offset_m
        direction = 1.0 if self.default_lateral_direction != "right" else -1.0
        source_name = "default"
        forward = 0.0
        if (
            self.use_goal_pair_for_lateral_offset
            and self.route_goal is not None
            and self.site_goal is not None
            and self._goal_pair_is_fresh()
        ):
            forward, lateral = relative_xy(self.route_goal, self.site_goal)
            if abs(lateral) >= self.min_lateral_offset_m:
                offset = abs(lateral)
                direction = 1.0 if lateral >= 0.0 else -1.0
                source_name = "goal_pair"
        offset = clamp(offset, self.min_lateral_offset_m, self.max_lateral_offset_m)
        return offset, direction, source_name, forward

    def _select_reverse_entry_axis(self, direct_axis_yaw: float, dx: float, dy: float) -> tuple[float, str]:
        if not self.reverse_entry_use_site_goal_yaw or self.site_goal is None:
            return direct_axis_yaw, "snap_to_site"

        site_yaw = yaw_from_pose(self.site_goal)
        if self.reverse_entry_site_yaw_mode == "robot_yaw":
            axis_yaw = normalize_angle(site_yaw + math.pi)
            source = "site_robot_yaw"
        else:
            axis_yaw = site_yaw
            source = "site_reverse_axis"

        if self.reverse_entry_auto_select_yaw_equivalent:
            forward_projection = math.cos(axis_yaw) * dx + math.sin(axis_yaw) * dy
            flipped_axis = normalize_angle(axis_yaw + math.pi)
            flipped_projection = math.cos(flipped_axis) * dx + math.sin(flipped_axis) * dy
            if flipped_projection > forward_projection:
                axis_yaw = flipped_axis
                source = f"{source}_flipped"

        return normalize_angle(axis_yaw), source

    def _pose_distance(self, a: PoseStamped, b: PoseStamped) -> float:
        return math.hypot(
            float(a.pose.position.x) - float(b.pose.position.x),
            float(a.pose.position.y) - float(b.pose.position.y),
        )

    def _goal_pair_is_fresh(self) -> bool:
        if self.goal_pair_max_age_s <= 0.0:
            return True
        now_s = self._now_s()
        return (
            (now_s - self.site_goal_time_s) <= self.goal_pair_max_age_s
            and (now_s - self.route_goal_time_s) <= self.goal_pair_max_age_s
        )

    def _goal_pair_distance(self) -> float:
        if self.site_goal is None or self.route_goal is None:
            return 0.0
        return self._pose_distance(self.site_goal, self.route_goal)

    def _manual_site_goal_is_ready(self, msg: PlanningState) -> bool:
        if not self.enable_manual_site_goal_auto_start:
            return False
        if msg.active_goal_source.strip() != "manual":
            return False
        return self._site_goal_pair_is_ready()

    def _site_goal_pair_is_ready(self) -> bool:
        if self.site_goal is None or self.route_goal is None:
            return False
        if not self._goal_pair_is_fresh():
            return False
        return self._goal_pair_distance() >= self.manual_site_min_offset_m

    def _route_goal_reached_for_auto_start(self) -> bool:
        if self.route_goal is None or not self._pose_is_fresh():
            return False
        distance = self._pose_distance(self.last_pose, self.route_goal)  # type: ignore[arg-type]
        return distance <= self.route_goal_reached_distance_m

    def _relative_lateral_from_entry(self) -> float:
        if self.last_pose is None:
            return 0.0
        dx = self.last_pose.pose.position.x - self.entry_start_x
        dy = self.last_pose.pose.position.y - self.entry_start_y
        return -math.sin(self.entry_reference_yaw) * dx + math.cos(self.entry_reference_yaw) * dy

    def _entry_reached(self) -> bool:
        if not self._pose_is_fresh():
            return False
        signed_lateral = self.crab_direction * self._relative_lateral_from_entry()
        return signed_lateral >= max(0.0, self.crab_offset_m - self.entry_position_tolerance_m)

    def _return_reached(self) -> bool:
        if not self._pose_is_fresh():
            return False
        # HH_260618: Use signed lateral progress so crab-out completion is
        # robust to one control tick of overshoot around the original snap pose.
        signed_lateral = self.crab_direction * self._relative_lateral_from_entry()
        if signed_lateral <= self.return_position_tolerance_m:
            return True
        dx = self.last_pose.pose.position.x - self.entry_start_x  # type: ignore[union-attr]
        dy = self.last_pose.pose.position.y - self.entry_start_y  # type: ignore[union-attr]
        return math.hypot(dx, dy) <= self.return_position_tolerance_m

    def _crab_timed_out(self, elapsed: float) -> bool:
        return elapsed > (self.crab_duration_s + self.crab_timeout_margin_s)

    def _reverse_return_timed_out(self, elapsed: float) -> bool:
        # HH_260619 - Returning from the campsite usually includes yaw and
        # lateral correction, so use a longer dedicated margin than entry.
        return elapsed > (self.crab_duration_s + self.reverse_return_timeout_margin_s)

    def _pose_is_fresh(self) -> bool:
        return self.last_pose is not None and (self._now_s() - self.last_pose_time_s) <= self.pose_timeout_s

    def _set_phase(self, phase: Phase, message: str) -> None:
        self.phase = phase
        self.phase_start_s = self._now_s()
        if phase == Phase.REVERSE_OUT and self.last_pose is not None:
            self.return_start_x = self.last_pose.pose.position.x
            self.return_start_y = self.last_pose.pose.position.y
            self._publish_reverse_path(
                self.return_start_x,
                self.return_start_y,
                self.entry_start_x,
                self.entry_start_y,
                normalize_angle(self.entry_target_yaw + math.pi),
            )
        self._update_timer_period()
        self.get_logger().info(f"site_maneuver {phase.value}: {message}")
        self._publish_amr_service_state_for_phase(phase, message)
        self._publish_status(force=True)

    def _set_error(self, message: str) -> None:
        self._publish_zero()
        self._set_phase(Phase.ERROR, message)

    def _publish_amr_service_state_for_phase(self, phase: Phase, source_message: str) -> None:
        # HHL_260622 - UI needs campsite-internal phases, not only Nav2 lanelet arrival state.
        service_state = None
        if phase in {Phase.ALIGN_ENTRY_YAW, Phase.REVERSE_IN, Phase.CRAB_IN, Phase.ROTATE_180}:
            service_state = AvgAmrServiceState.SITE_ENTRY
        elif phase in {Phase.UNLOAD_WAIT, Phase.WAIT_RETURN}:
            service_state = AvgAmrServiceState.UNLOAD_WAIT
        elif phase in {Phase.REVERSE_OUT, Phase.CRAB_OUT, Phase.DONE}:
            service_state = AvgAmrServiceState.RETURN_WITH_CARGO

        if service_state is None:
            return

        msg = AvgAmrServiceState()
        msg.state = int(service_state)
        msg.description = f"site_maneuver:{phase.value}:{source_message}"
        self.amr_service_pub.publish(msg)

    def _publish_zero(self) -> None:
        self.cmd_pub.publish(Twist())

    def _publish_crab(self, direction: float) -> None:
        cmd = Twist()
        cmd.linear.y = direction * self.crab_speed_mps
        self.cmd_pub.publish(cmd)

    def _axis_progress_lateral(self, origin_x: float, origin_y: float, axis_yaw: float) -> tuple[float, float]:
        if self.last_pose is None:
            return 0.0, 0.0
        dx = self.last_pose.pose.position.x - origin_x
        dy = self.last_pose.pose.position.y - origin_y
        progress = math.cos(axis_yaw) * dx + math.sin(axis_yaw) * dy
        lateral = -math.sin(axis_yaw) * dx + math.cos(axis_yaw) * dy
        return progress, lateral

    def _distance_to_xy(self, x: float, y: float) -> float:
        if self.last_pose is None:
            return float("inf")
        return math.hypot(self.last_pose.pose.position.x - x, self.last_pose.pose.position.y - y)

    def _reverse_in_reached(self) -> bool:
        if not self._pose_is_fresh():
            return False
        progress, _ = self._axis_progress_lateral(
            self.entry_start_x, self.entry_start_y, self.entry_reverse_axis_yaw
        )
        _, target_line_lateral = self._axis_progress_lateral(
            self.entry_line_origin_x, self.entry_line_origin_y, self.entry_reverse_axis_yaw
        )
        return (
            self._distance_to_xy(self.entry_target_x, self.entry_target_y)
            <= self.entry_position_tolerance_m
            or (
                progress >= max(0.0, self.entry_target_progress_m - self.entry_position_tolerance_m)
                and abs(target_line_lateral) <= self.reverse_entry_lateral_tolerance_m
            )
        )

    def _reverse_out_reached(self) -> bool:
        if not self._pose_is_fresh():
            return False
        if self._distance_to_xy(self.entry_start_x, self.entry_start_y) <= self.return_position_tolerance_m:
            return True
        # HH_260619 - Reverse-out control drives backward with the robot body
        # yaw aligned to entry_reverse_axis_yaw, so the actual travel axis is
        # entry_reverse_axis_yaw + 180deg. Use the same snap-line origin as the
        # controller; using the campsite origin made off-axis sites fail even
        # after crossing the lanelet snap pose.
        return_axis_yaw = normalize_angle(self.entry_reverse_axis_yaw + math.pi)
        target_dx = self.entry_start_x - self.return_start_x
        target_dy = self.entry_start_y - self.return_start_y
        target_progress = math.cos(return_axis_yaw) * target_dx + math.sin(return_axis_yaw) * target_dy
        current_progress, _ = self._axis_progress_lateral(
            self.return_start_x, self.return_start_y, return_axis_yaw
        )
        _, current_lateral_to_snap_line = self._axis_progress_lateral(
            self.entry_start_x, self.entry_start_y, return_axis_yaw
        )
        progress_tolerance = max(
            self.return_position_tolerance_m,
            self.reverse_return_progress_tolerance_m,
        )
        lateral_tolerance = max(
            self.return_position_tolerance_m,
            self.reverse_entry_lateral_tolerance_m,
            self.reverse_return_lateral_tolerance_m,
        )
        if abs(current_lateral_to_snap_line) > lateral_tolerance:
            return False
        if target_progress >= 0.0:
            return current_progress >= target_progress - progress_tolerance
        return current_progress <= target_progress + progress_tolerance

    def _publish_reverse_along_axis(self, target_yaw: float, origin_x: float, origin_y: float) -> None:
        if not self._pose_is_fresh():
            self._set_error(f"pose timeout during {self.phase.value.lower()}")
            return
        yaw = yaw_from_pose(self.last_pose)  # type: ignore[arg-type]
        heading_error = normalize_angle(target_yaw - yaw)
        reverse_axis_yaw = normalize_angle(target_yaw + math.pi)
        _, lateral_error = self._axis_progress_lateral(origin_x, origin_y, reverse_axis_yaw)
        progress_text = ""
        if self.phase == Phase.REVERSE_OUT:
            target_dx = self.entry_start_x - self.return_start_x
            target_dy = self.entry_start_y - self.return_start_y
            target_progress = math.cos(reverse_axis_yaw) * target_dx + math.sin(reverse_axis_yaw) * target_dy
            current_progress, _ = self._axis_progress_lateral(
                self.return_start_x, self.return_start_y, reverse_axis_yaw
            )
            progress_text = f" progress={current_progress:.2f}/{target_progress:.2f}m"
        cmd = Twist()
        cmd.linear.x = -self.reverse_entry_speed_mps
        cmd.angular.z = clamp(
            self.reverse_entry_yaw_kp * heading_error + self.reverse_entry_lateral_kp * lateral_error,
            -self.reverse_entry_max_angular_speed_radps,
            self.reverse_entry_max_angular_speed_radps,
        )
        self.cmd_pub.publish(cmd)
        now_s = self._now_s()
        if self.reverse_entry_debug_period_s > 0.0 and (
            now_s - self.last_reverse_entry_debug_s
        ) >= self.reverse_entry_debug_period_s:
            self.last_reverse_entry_debug_s = now_s
            self.get_logger().info(
                "site_maneuver reverse control: "
                f"phase={self.phase.value} target_yaw={math.degrees(target_yaw):.1f}deg "
                f"reverse_axis={math.degrees(reverse_axis_yaw):.1f}deg "
                f"heading_err={math.degrees(heading_error):.1f}deg "
                f"lateral_err={lateral_error:.2f}m{progress_text} angular_z={cmd.angular.z:.2f}"
            )

    def _publish_reverse_in(self) -> None:
        self._publish_reverse_along_axis(
            self.entry_target_yaw, self.entry_line_origin_x, self.entry_line_origin_y
        )

    def _publish_reverse_out(self) -> None:
        self._publish_reverse_along_axis(
            normalize_angle(self.entry_target_yaw + math.pi),
            self.entry_start_x,
            self.entry_start_y,
        )

    def _publish_rotate(self) -> bool:
        if not self._pose_is_fresh():
            self._set_error(f"pose timeout during {self.phase.value.lower()} yaw control")
            return False
        yaw = yaw_from_pose(self.last_pose)  # type: ignore[arg-type]
        if self.phase == Phase.ROTATE_180 and self.rotate_direction_sign != 0.0:
            target_delta = self.rotate_direction_sign * math.pi
            traveled = normalize_angle(yaw - self.start_yaw)
            if self.rotate_direction_sign > 0.0 and traveled < 0.0:
                traveled += 2.0 * math.pi
            elif self.rotate_direction_sign < 0.0 and traveled > 0.0:
                traveled -= 2.0 * math.pi
            remaining = target_delta - traveled
            if abs(math.degrees(remaining)) <= self.rotate_yaw_tolerance_deg:
                self._publish_zero()
                return True
            cmd = Twist()
            if self.rotate_direction_sign * remaining > 0.0:
                angular_speed = min(
                    self.max_angular_speed_radps,
                    max(0.05, abs(self.rotate_kp * remaining)),
                )
                cmd.angular.z = self.rotate_direction_sign * angular_speed
            else:
                err = normalize_angle(self.target_yaw - yaw)
                cmd.angular.z = clamp(
                    self.rotate_kp * err,
                    -self.max_angular_speed_radps,
                    self.max_angular_speed_radps,
                )
            self.cmd_pub.publish(cmd)
            return False
        err = normalize_angle(self.target_yaw - yaw)
        if abs(math.degrees(err)) <= self.rotate_yaw_tolerance_deg:
            self._publish_zero()
            return True
        cmd = Twist()
        cmd.angular.z = clamp(self.rotate_kp * err, -self.max_angular_speed_radps, self.max_angular_speed_radps)
        self.cmd_pub.publish(cmd)
        return False

    def _publish_return_request(self, reason: str) -> None:
        if not self.request_return_to_drop_zone_on_done or self.return_acknowledged:
            return
        now_s = self._now_s()
        if (
            self.return_published
            and self.return_request_retry_period_s > 0.0
            and (now_s - self.last_return_request_publish_s) < self.return_request_retry_period_s
        ):
            return
        msg = Bool()
        msg.data = True
        self.return_request_pub.publish(msg)
        self.return_published = True
        self.last_return_request_publish_s = now_s
        self.get_logger().info(f"site_maneuver return_to_drop_zone requested: {reason}")

    def _tick(self) -> None:
        elapsed = self._now_s() - self.phase_start_s

        if self.phase == Phase.ALIGN_ENTRY_YAW:
            if self._publish_rotate():
                self._set_phase(Phase.REVERSE_IN, "entry yaw aligned for reverse site approach")
        elif self.phase == Phase.REVERSE_IN:
            if self._reverse_in_reached():
                self._publish_zero()
                self.target_yaw = normalize_angle(self.entry_target_yaw + math.pi)
                self._set_phase(Phase.ROTATE_180, "reverse entry pose reached")
            elif self._crab_timed_out(elapsed):
                self._set_error("reverse entry timeout before reaching site target")
            else:
                self._publish_reverse_in()
        elif self.phase == Phase.CRAB_IN:
            if self._entry_reached():
                self._publish_zero()
                self._set_phase(Phase.ROTATE_180, "crab entry pose reached")
            elif self._crab_timed_out(elapsed):
                self._set_error("crab entry timeout before reaching site offset")
            else:
                self._publish_crab(self.crab_direction)
        elif self.phase == Phase.ROTATE_180:
            if self._publish_rotate():
                self._set_phase(Phase.UNLOAD_WAIT, "robot yaw rotated 180deg")
        elif self.phase == Phase.UNLOAD_WAIT:
            self._publish_zero()
            if elapsed >= self.unload_wait_s:
                if self.auto_return_after_unload_wait or self.return_requested:
                    next_phase = Phase.REVERSE_OUT if self.site_entry_mode == "reverse" else Phase.CRAB_OUT
                    self._set_phase(next_phase, "unload wait complete")
                else:
                    self._set_phase(Phase.WAIT_RETURN, "waiting external return command")
        elif self.phase == Phase.WAIT_RETURN:
            self._publish_zero()
        elif self.phase == Phase.REVERSE_OUT:
            if self._reverse_out_reached():
                self._publish_zero()
                self._set_phase(Phase.DONE, "returned to lanelet snap pose")
                self._publish_return_request("done")
            elif self._reverse_return_timed_out(elapsed):
                self._set_error("reverse return timeout before reaching lanelet snap pose")
            else:
                self._publish_reverse_out()
        elif self.phase == Phase.CRAB_OUT:
            if self._return_reached():
                self._publish_zero()
                self._set_phase(Phase.DONE, "returned to lanelet snap pose")
                self._publish_return_request("done")
            elif self._crab_timed_out(elapsed):
                self._set_error("crab return timeout before reaching lanelet snap pose")
            else:
                # HH_260617: After the 180deg body rotation, the same body-left/right
                # command moves back toward the original lanelet snap pose.
                self._publish_crab(self.crab_direction)
        elif self.phase == Phase.DONE:
            # HH_260618: Keep retrying the return command while DONE until the
            # state machine switches to RETURN_TO_DROP_ZONE/drop_zone.
            self._publish_return_request("done_retry")
        elif self.phase in {Phase.IDLE, Phase.DONE, Phase.ERROR}:
            # HH_260618: Do not publish zero Twist while inactive. Nav2 also
            # publishes /planning/cmd_vel_raw, and idle parking zeros can race
            # the controller output and stop normal driving.
            pass

        self._publish_status()

    def _publish_status(self, force: bool = False) -> None:
        now_s = self._now_s()
        period_s = 1.0 / max(self.status_publish_rate_hz, 0.1)
        if not force and self.last_status_publish_s > 0.0 and (now_s - self.last_status_publish_s) < period_s:
            return
        if self.phase == Phase.ERROR:
            level = ModuleState.ERROR
        elif self.phase in {Phase.IDLE, Phase.DONE}:
            level = ModuleState.OK
        else:
            level = ModuleState.WARN
        message = (
            f"phase={self.phase.value} "
            f"return_published={self.return_published} "
            f"return_ack={self.return_acknowledged}"
        )
        self.status_pub.publish(make_module_state(self, "parking", level, message))
        diag_level = DiagnosticStatus.ERROR if level == ModuleState.ERROR else (
            DiagnosticStatus.WARN if level == ModuleState.WARN else DiagnosticStatus.OK
        )
        self.diag_pub.publish(
            make_diagnostics(
                self,
                "parking/site_maneuver",
                "parking",
                diag_level,
                message,
                (
                    ("phase", self.phase.value),
                    ("cmd_vel_topic", self.cmd_vel_topic),
                    ("crab_duration_s", f"{self.crab_duration_s:.2f}"),
                    ("crab_offset_m", f"{self.crab_offset_m:.2f}"),
                    ("crab_source", self.crab_source),
                ),
            )
        )
        self.last_status_publish_s = now_s


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SiteManeuverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
