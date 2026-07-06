#!/usr/bin/env python3
# HH_260331: Gate Nav2 controller cmd_vel with explicit planning engage trigger.
# HH_260624 - Split manual /planning/engage from UI /planning/mission_engage
# while publishing one effective /planning/engaged state for downstream gates.

from __future__ import annotations

import math
import os
from dataclasses import dataclass

import rclpy
from avg_msgs.msg import AvgLocalizationMode, ModuleState
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


@dataclass
class YawAlignmentZone:
    """Per-zone yaw alignment policy."""

    zone_id: str
    x: float
    y: float
    z: float
    target_yaw_rad: float
    activation_radius_m: float
    lock_radius_m: float
    position_tolerance_m: float
    yaw_tolerance_deg: float
    yaw_tolerance_per_meter_deg: float
    hold_s: float
    angular_kp: float
    max_angular_z: float
    max_approach_linear_x: float


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
        self.mission_engage_topic = str(
            self.declare_parameter("mission_engage_topic", "/planning/mission_engage").value
        )
        self.state_topic = str(
            self.declare_parameter("state_topic", "/planning/engaged").value
        )
        if self.engage_topic == self.state_topic:
            # HH_260625: Never subscribe manual engage from this node's own effective-state output.
            # A stale/miswired launch can otherwise create a self-loop and ignore /planning/engage.
            fallback_engage_topic = "/planning/engage"
            self.get_logger().warn(
                "manual engage_topic matched state_topic (%s); using %s instead"
                % (self.state_topic, fallback_engage_topic)
            )
            self.engage_topic = fallback_engage_topic
        # HH_260522: unified source selector for e-stop input.
        #   platform_status/topic/enabled/on -> subscribe
        #   disabled/off/none -> ignore
        self.estop_source_mode = str(
            self.declare_parameter("estop_source_mode", "platform_status").value
        ).strip().lower()
        if self.estop_source_mode in {"disabled", "off", "none"}:
            self.estop_topic_enabled = False
        elif self.estop_source_mode in {"platform_status", "topic", "enabled", "on"}:
            self.estop_topic_enabled = True
        else:
            self.estop_topic_enabled = True
            self.get_logger().warn(
                "Unknown estop_source_mode='%s'. Using default platform_status mode."
                % self.estop_source_mode
            )
        self.estop_topic = str(
            # HH_260409: Use platform status e-stop as default shared source.
            self.declare_parameter("estop_topic", "/platform/status/estop").value
        )
        # HH_260701 - OR additional soft-estop topics into the planning gate.
        # The state-machine estop is mission/diagnostic-owned, so parking and
        # Nav2 cmd_vel now close on the same ERROR_STOP condition.
        self.additional_estop_topics = self._parse_topic_list(
            self.declare_parameter(
                "additional_estop_topics",
                "/planning/state_machine/estop",
            ).value
        )
        # HH_260507: Block cmd_vel when DR timeout published by localization_monitor.
        # HH_260522: unified source selector for DR-timeout trigger.
        #   localization_monitor/topic/enabled/on -> subscribe
        #   disabled/off/none -> ignore
        self.dr_timeout_source_mode = str(
            self.declare_parameter("dr_timeout_source_mode", "localization_monitor").value
        ).strip().lower()
        if self.dr_timeout_source_mode in {"disabled", "off", "none"}:
            self.dr_timeout_topic_enabled = False
        elif self.dr_timeout_source_mode in {"localization_monitor", "topic", "enabled", "on"}:
            self.dr_timeout_topic_enabled = True
        else:
            self.dr_timeout_topic_enabled = True
            self.get_logger().warn(
                "Unknown dr_timeout_source_mode='%s'. Using default localization_monitor mode."
                % self.dr_timeout_source_mode
            )
        self.dr_timeout_topic = str(
            self.declare_parameter("dr_timeout_topic", "/localization/state/dr_timeout").value
        )
        self.allow_on_start = bool(
            self.declare_parameter("allow_on_start", False).value
        )
        self.publish_zero_when_blocked = bool(
            self.declare_parameter("publish_zero_when_blocked", True).value
        )
        # HH_260507: Speed scale applied to all cmd_vel output before publishing.
        self.speed_scale = float(
            self.declare_parameter("speed_scale", 1.0).value
        )
        # HH_260626: If Nav2 stops publishing raw cmd_vel while gates are open,
        # keep sending zero so downstream controllers never reuse a stale command.
        self.input_timeout_s = float(
            self.declare_parameter("input_timeout_s", 0.35).value
        )
        self.zero_publish_rate_hz = float(
            self.declare_parameter("zero_publish_rate_hz", 10.0).value
        )
        # HH_260427: When localization recovers from DR_ONLY to NORMAL, force
        # a short stop-hold window so downstream stack can settle the recovered pose.
        self.enable_gnss_recovery_hold = bool(
            self.declare_parameter("enable_gnss_recovery_hold", True).value
        )
        self.localization_mode_topic = str(
            self.declare_parameter("localization_mode_topic", "/localization/mode").value
        )
        self.gnss_recovery_hold_s = float(
            self.declare_parameter("gnss_recovery_hold_s", 2.0).value
        )
        self.gnss_recovery_min_source_s = float(
            self.declare_parameter("gnss_recovery_min_source_s", 0.5).value
        )
        self.gnss_recovery_hold_cooldown_s = float(
            self.declare_parameter("gnss_recovery_hold_cooldown_s", 5.0).value
        )
        # Default transition condition: DR_ONLY(2)+ -> NORMAL(0).
        self.gnss_recovery_source_mode_min = int(
            self.declare_parameter(
                "gnss_recovery_source_mode_min",
                int(AvgLocalizationMode.DR_ONLY),
            ).value
        )
        self.gnss_recovery_target_mode = int(
            self.declare_parameter(
                "gnss_recovery_target_mode",
                int(AvgLocalizationMode.NORMAL),
            ).value
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
            # HH_260426: VIO stack is disabled; use localization fallback odometry.
            self.declare_parameter("odometry_topic", "/localization/fallback/odometry").value
        )
        # HH_260522: pose source preference options:
        #   odometry | tf_robot_base | pose_topic
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
            # HH_260623 - Full robot width plus 0.10 m margin per side.
            self.declare_parameter("cost_stop_width_m", 1.27).value
        )
        self.cost_stop_hold_s = float(
            self.declare_parameter("cost_stop_hold_s", 1.0).value
        )
        # HH_260703: Dynamic obstacle stops must remain latched until sensors
        # report a continuous clear window; this prevents stop/go oscillation
        # when lidar/radar cost cells briefly flicker below threshold.
        self.cost_stop_latch_enable = bool(
            self.declare_parameter("cost_stop_latch_enable", True).value
        )
        self.cost_stop_clear_required_s = float(
            self.declare_parameter("cost_stop_clear_required_s", 2.0).value
        )
        self.cost_stop_latch_log_interval_s = float(
            self.declare_parameter("cost_stop_latch_log_interval_s", 1.0).value
        )
        # HH_260703 - Missing/stale merged cost grid is a fail-safe motion stop.
        # Diagnostics reports the fault, but cmd_vel safety should not wait for
        # state-machine e-stop propagation.
        self.cost_grid_stale_stop_enable = bool(
            self.declare_parameter("cost_grid_stale_stop_enable", True).value
        )
        self.cost_grid_stale_timeout_s = float(
            self.declare_parameter("cost_grid_stale_timeout_s", 1.0).value
        )
        self.cost_grid_stale_log_interval_s = float(
            self.declare_parameter("cost_grid_stale_log_interval_s", 1.0).value
        )
        # HH_260618: Hard lanelet safety uses the raw lanelet grid before
        # inflation ego-clear. The merged inflation grid intentionally clears
        # the robot footprint, so it cannot be the only source that decides
        # whether translational cmd_vel may leave the drivable lanelet area.
        self.lanelet_safety_enable = bool(
            self.declare_parameter("lanelet_safety_enable", True).value
        )
        self.lanelet_safety_grid_topic = str(
            self.declare_parameter(
                "lanelet_safety_grid_topic", "/map/cost_grid/lanelet"
            ).value
        )
        self.lanelet_safety_threshold = int(
            self.declare_parameter("lanelet_safety_threshold", 85).value
        )
        self.lanelet_safety_current_threshold = int(
            self.declare_parameter("lanelet_safety_current_threshold", 85).value
        )
        self.lanelet_safety_lookahead_m = float(
            self.declare_parameter("lanelet_safety_lookahead_m", 1.0).value
        )
        self.lanelet_safety_width_m = float(
            self.declare_parameter("lanelet_safety_width_m", 0.8).value
        )
        self.lanelet_safety_stop_on_unknown = bool(
            self.declare_parameter("lanelet_safety_stop_on_unknown", True).value
        )
        self.lanelet_safety_allow_rotation_in_place = bool(
            self.declare_parameter(
                "lanelet_safety_allow_rotation_in_place", True
            ).value
        )
        self.lanelet_safety_check_reverse = bool(
            self.declare_parameter("lanelet_safety_check_reverse", False).value
        )
        self.lanelet_safety_check_lateral = bool(
            self.declare_parameter("lanelet_safety_check_lateral", False).value
        )
        self.lanelet_safety_min_translation_mps = float(
            self.declare_parameter("lanelet_safety_min_translation_mps", 0.02).value
        )
        # HH_260619 - Sample forward lanelet safety along the active local path
        # instead of the raw robot-yaw rectangle when possible. Merge lanes and
        # route starts can place static lane boundaries inside the robot-forward
        # rectangle even though the selected local path is centered and valid.
        self.lanelet_safety_front_use_local_path = bool(
            self.declare_parameter("lanelet_safety_front_use_local_path", True).value
        )
        self.lanelet_safety_front_path_max_start_distance_m = float(
            self.declare_parameter("lanelet_safety_front_path_max_start_distance_m", 1.5).value
        )
        # HH_260622 - Path-based lanelet safety should validate the selected
        # route center corridor, not the full robot-width boundary area. Merge
        # lanes and lane-change connections can legitimately place raw lanelet
        # boundary cost near the local path.
        self.lanelet_safety_front_path_width_m = float(
            self.declare_parameter("lanelet_safety_front_path_width_m", 0.25).value
        )
        # HH_260624 - When the robot is leaving a drop-zone/site back to the
        # lanelet route, the local path can start in static off-lane cost. Allow
        # only that bounded route-reentry segment; normal FRONT_PATH blocking
        # still applies once the robot is no longer close to the active path.
        self.lanelet_safety_front_path_allow_route_reentry = bool(
            self.declare_parameter("lanelet_safety_front_path_allow_route_reentry", True).value
        )
        # HH_260622 - When a map/profile starts from a manually placed pose
        # just outside the drivable lanelet, allow one controlled forward
        # re-entry if the active local path is already close. This is not a
        # general lanelet bypass: FRONT_PATH and dynamic obstacle checks still
        # run after the current-cell guard is skipped.
        self.lanelet_safety_current_allow_route_reentry = bool(
            self.declare_parameter(
                "lanelet_safety_current_allow_route_reentry", True
            ).value
        )
        self.lanelet_safety_current_route_reentry_max_distance_m = float(
            self.declare_parameter(
                "lanelet_safety_current_route_reentry_max_distance_m", 4.0
            ).value
        )
        self.lanelet_safety_current_route_reentry_require_front_cmd = bool(
            self.declare_parameter(
                "lanelet_safety_current_route_reentry_require_front_cmd", True
            ).value
        )
        # HH_260624 - Drop-zone straight exit is mission-owned parking motion:
        # it intentionally starts inside static lanelet/drop-zone boundary cost,
        # so raw lanelet safety must not block it before Nav2 campsite routing.
        self.parking_drop_zone_status_topic = str(
            self.declare_parameter(
                "parking_drop_zone_status_topic", "/parking/drop_zone/status"
            ).value
        )
        self.parking_drop_zone_static_bypass_phases = self._parse_source_label_set(
            self.declare_parameter(
                "parking_drop_zone_static_bypass_phases",
                "EXIT_STRAIGHT,ALIGN_EXIT_YAW",
            ).value,
            {"exit_straight", "align_exit_yaw"},
        )
        # HH_260701 - Campsite internal motion is owned by camrod_parking.
        # Bypass only static lanelet/global-path cost during these phases;
        # live LiDAR/Radar source cost remains blocking.
        self.parking_site_status_topic = str(
            self.declare_parameter(
                "parking_site_status_topic", "/parking/site_maneuver/status"
            ).value
        )
        self.parking_site_static_bypass_phases = self._parse_source_label_set(
            self.declare_parameter(
                "parking_site_static_bypass_phases",
                "ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,ALIGN_RETURN_YAW,REVERSE_OUT,CRAB_OUT",
            ).value,
            {
                "align_entry_yaw",
                "reverse_in",
                "crab_in",
                "rotate_180",
                "align_return_yaw",
                "reverse_out",
                "crab_out",
            },
        )
        # HH_260618: Attribute cost-stop events to original cost-grid sources
        # without publishing another large debug grid. Source grids are sampled
        # only when a merged-grid stop actually occurs.
        self.cost_source_debug_enable = bool(
            self.declare_parameter("cost_source_debug_enable", True).value
        )
        self.cost_source_debug_max_age_s = float(
            self.declare_parameter("cost_source_debug_max_age_s", 1.0).value
        )
        self.cost_source_debug_topics = list(
            self.declare_parameter(
                "cost_source_debug_topics",
                [
                    "/map/cost_grid/lanelet",
                    "/sensing/cost_grid/lidar",
                    "/sensing/cost_grid/radar",
                    "/planning/cost_grid/global_path",
                ],
            ).value
        )
        self.cost_source_debug_labels = list(
            self.declare_parameter(
                "cost_source_debug_labels",
                ["lanelet", "lidar", "radar", "global_path"],
            ).value
        )
        # HH_260622 - Treat the merged inflation grid as an attribution surface:
        # lanelet/global-path costs guide planning and RViz, while only live
        # dynamic sources may close cmd_vel. Raw lanelet safety stays separate.
        self.cost_stop_require_dynamic_source = bool(
            self.declare_parameter("cost_stop_require_dynamic_source", True).value
        )
        self.cost_stop_dynamic_source_labels = self._parse_source_label_set(
            self.declare_parameter(
                "cost_stop_dynamic_source_labels", "lidar,radar"
            ).value,
            {"lidar", "radar"},
        )
        # HH_260702 - If Nav2 has already produced an avoidance local path, the
        # dynamic stop gate should validate that path corridor instead of holding
        # a stale body-forward rectangle on the original obstacle.
        self.front_dynamic_stop_use_local_path = bool(
            self.declare_parameter("front_dynamic_stop_use_local_path", True).value
        )
        self.front_dynamic_path_width_m = float(
            self.declare_parameter(
                "front_dynamic_path_width_m",
                self.cost_stop_width_m,
            ).value
        )
        self.front_dynamic_path_max_start_distance_m = float(
            self.declare_parameter(
                "front_dynamic_path_max_start_distance_m",
                self.lanelet_safety_front_path_max_start_distance_m,
            ).value
        )

        # HH_260422: Speed-dependent front lookahead.
        #   lookahead = clamp(v²/(2·g·mu) + reaction_time·v + margin, min, max)
        #   where v = |forward speed| from odometry, g=9.8, mu=front_lookahead_friction.
        self.enable_speed_dependent_lookahead = bool(
            self.declare_parameter("enable_speed_dependent_lookahead", True).value
        )
        self.front_lookahead_min_m = float(
            # HH_260630 - Include front radar mount plus roughly 1m sensor-forward clearance.
            self.declare_parameter("front_lookahead_min_m", 2.10).value
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
        self.enable_body_near_dynamic_stop = bool(
            # HH_260706 - Keep short side/rear dynamic guards active during
            # forward driving so body-adjacent radar hits still stop motion.
            self.declare_parameter("enable_body_near_dynamic_stop", True).value
        )
        self.body_near_side_lookahead_m = float(
            self.declare_parameter("body_near_side_lookahead_m", 0.75).value
        )
        self.body_near_rear_lookahead_m = float(
            self.declare_parameter("body_near_rear_lookahead_m", 0.55).value
        )
        self.body_near_maneuver_side_lookahead_m = float(
            self.declare_parameter("body_near_maneuver_side_lookahead_m", 0.55).value
        )
        self.body_near_maneuver_rear_lookahead_m = float(
            self.declare_parameter("body_near_maneuver_rear_lookahead_m", 0.45).value
        )
        self.side_cost_threshold = int(
            self.declare_parameter("side_cost_threshold", 85).value
        )
        self.side_lookahead_m = float(
            self.declare_parameter("side_lookahead_m", 1.2).value
        )
        self.side_corridor_width_m = float(
            # HH_260623 - Side scans need full body length plus 0.10 m margin front/rear.
            self.declare_parameter("side_corridor_width_m", 1.69160).value
        )
        self.rear_cost_threshold = int(
            self.declare_parameter("rear_cost_threshold", 85).value
        )
        self.rear_lookahead_m = float(
            # HH_260630: Rear dynamic cost cells can land past 0.9 m from the
            # robot center after grid quantization; keep reverse stop outside it.
            self.declare_parameter("rear_lookahead_m", 1.2).value
        )
        self.rear_corridor_width_m = float(
            # HH_260623 - Rear scans need full body width plus 0.10 m margin per side.
            self.declare_parameter("rear_corridor_width_m", 1.27).value
        )
        # HH_260618: Site parking enters campsites with explicit lateral or
        # reverse cmd_vel. That motion is mission-owned and intentionally
        # leaves the lanelet corridor, so static lanelet/global-path
        # front/side/rear cost must not block it. Dynamic LiDAR/Radar source
        # cost still blocks site maneuver motion.
        self.lateral_cmd_bypass_static_cost_stop = bool(
            self.declare_parameter(
                "lateral_cmd_bypass_static_cost_stop", True
            ).value
        )
        self.lateral_cmd_bypass_min_mps = float(
            self.declare_parameter("lateral_cmd_bypass_min_mps", 0.02).value
        )
        self.reverse_cmd_bypass_static_cost_stop = bool(
            self.declare_parameter(
                "reverse_cmd_bypass_static_cost_stop", True
            ).value
        )
        self.reverse_cmd_bypass_min_mps = float(
            self.declare_parameter("reverse_cmd_bypass_min_mps", 0.02).value
        )
        self.lateral_cmd_dynamic_obstacle_threshold = int(
            self.declare_parameter("lateral_cmd_dynamic_obstacle_threshold", 85).value
        )
        # HH_260624 - In-place 180deg site rotations may ignore static lanelet
        # cost, but live LiDAR/Radar cost around the body must still stop motion.
        self.rotation_cmd_dynamic_obstacle_stop = bool(
            self.declare_parameter("rotation_cmd_dynamic_obstacle_stop", True).value
        )
        self.rotation_cmd_dynamic_obstacle_radius_m = float(
            self.declare_parameter("rotation_cmd_dynamic_obstacle_radius_m", 1.5).value
        )
        self.rotation_cmd_dynamic_obstacle_threshold = int(
            self.declare_parameter("rotation_cmd_dynamic_obstacle_threshold", 85).value
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

        # Yaw-alignment zone options.
        # When enabled, robot entering configured map zone must align heading
        # before full cmd_vel passthrough. Zone coordinates/yaw should be
        # lanelet2-derived keypoints for deterministic behavior.
        self.enable_yaw_alignment_zone = bool(
            self.declare_parameter("enable_yaw_alignment_zone", False).value
        )
        self.yaw_alignment_frame_id = str(
            self.declare_parameter("yaw_alignment_frame_id", "map").value
        )
        self.yaw_alignment_zones_file = str(
            self.declare_parameter("yaw_alignment_zones_file", "").value
        )
        self.yaw_alignment_exit_margin_m = float(
            self.declare_parameter("yaw_alignment_exit_margin_m", 0.3).value
        )
        # HH_260706: Route-heading alignment guard uses damped field defaults.
        # It holds linear motion and rotates first when the robot faces away
        # from the local path, but releases before delayed localization causes
        # over-correction and left-right startup oscillation.
        self.enable_route_heading_alignment = bool(
            self.declare_parameter("enable_route_heading_alignment", True).value
        )
        self.route_heading_path_topic = str(
            self.declare_parameter(
                "route_heading_path_topic", "/planning/local_path"
            ).value
        )
        self.route_heading_frame_id = str(
            self.declare_parameter("route_heading_frame_id", "map").value
        )
        self.route_heading_min_cmd_x_mps = float(
            self.declare_parameter("route_heading_min_cmd_x_mps", 0.03).value
        )
        self.route_heading_lateral_cmd_epsilon_mps = float(
            self.declare_parameter("route_heading_lateral_cmd_epsilon_mps", 0.02).value
        )
        self.route_heading_lookahead_m = float(
            self.declare_parameter("route_heading_lookahead_m", 2.0).value
        )
        self.route_heading_error_enter_deg = float(
            self.declare_parameter("route_heading_error_enter_deg", 75.0).value
        )
        self.route_heading_error_exit_deg = float(
            self.declare_parameter("route_heading_error_exit_deg", 35.0).value
        )
        self.route_heading_angular_kp = float(
            self.declare_parameter("route_heading_angular_kp", 0.8).value
        )
        self.route_heading_max_angular_z = float(
            self.declare_parameter("route_heading_max_angular_z", 0.35).value
        )
        self.route_heading_max_linear_x = float(
            self.declare_parameter("route_heading_max_linear_x", 0.0).value
        )
        self.route_heading_min_path_points = int(
            self.declare_parameter("route_heading_min_path_points", 2).value
        )

        # HH_260623 - Split manual 2D-goal engage from UI mission engage.
        #   /planning/engage is only the manual operator latch.
        #   /planning/mission_engage is the scenario latch for camping-site/drop-zone missions.
        #   The final gate opens when either latch is true; turning manual engage off must not
        #   stop an active UI mission.
        self._manual_enabled = self.allow_on_start
        self._mission_enabled = False
        self._enabled = self._manual_enabled or self._mission_enabled

        # HH_260422: _estop becomes True when any configured estop source publishes True.
        #   True -> blocks cmd_vel regardless of _enabled state; zero Twist is sent immediately.
        self._estop = False
        self._estop_sources: dict[str, bool] = {}
        # HH_260507: _dr_timeout becomes True when DR exceeds time/covariance limit.
        self._dr_timeout = False

        # HH_260422: _cost_blocked_until is the monotonic timestamp until which cost-stop keeps the gate closed.
        #   Gate remains blocked while time.monotonic() < _cost_blocked_until (cost_stop_hold_s duration).
        self._cost_blocked_until = 0.0
        self._cost_stop_latched = False
        self._cost_stop_clear_since_sec: float | None = None
        self._cost_stop_latch_reason = ""
        self._last_cost_stop_latch_log_sec = 0.0
        self._last_cost_grid_stale_log_sec = 0.0
        # HH_260427: Recovery hold timestamp after DR_ONLY -> NORMAL transition.
        self._gnss_recovery_blocked_until = 0.0
        self._gnss_recovery_source_enter_sec: float | None = None
        self._gnss_recovery_last_hold_sec = -1.0e9
        self._last_localization_mode_value: int | None = None
        self._last_unavoidable_cluster_cells = 0
        self._last_unavoidable_cluster_ratio = 0.0
        self._last_tf_warn_sec = 0.0
        self._last_empty_corridor_warn_sec = 0.0
        self._last_yaw_align_log_sec = 0.0
        self._last_route_heading_log_sec = 0.0
        self._last_lateral_static_bypass_log_sec = 0.0
        self._last_static_cost_ignored_log_sec = 0.0
        self._last_lanelet_current_reentry_bypass_log_sec = 0.0
        self._last_lanelet_front_path_reentry_bypass_log_sec = 0.0
        self._last_drop_zone_static_bypass_log_sec = 0.0
        self._last_site_static_phase_bypass_log_sec = 0.0
        self._last_front_path_dynamic_clear_log_sec = 0.0
        self._parking_drop_zone_phase = ""
        self._parking_site_phase = ""

        # HH_260422: _current_speed holds the latest forward body velocity (m/s) from odometry.
        #   Used to compute speed-dependent front lookahead. Stays 0.0 until first odometry arrives.
        self._current_speed = 0.0

        # HH_260422: Single merged cost grid (LiDAR + Radar) used for all directional checks.
        self._last_grid = None
        self._last_grid_recv_sec: float | None = None
        self._last_lanelet_safety_grid = None
        self._last_pose = None
        self._last_odom = None
        self._last_route_heading_path: Path | None = None
        self._route_heading_align_active = False
        self._last_block_reason_log_sec = 0.0
        self._last_input_cmd_time: Time | None = None
        self._cmd_input_stale = False
        self._last_cmd_input_stale_log_sec = 0.0
        self._cost_source_grids: dict[str, OccupancyGrid] = {}
        self._cost_source_recv_sec: dict[str, float] = {}

        # Runtime state for yaw-alignment gate.
        self._yaw_alignment_zones: list[YawAlignmentZone] = []
        self._active_yaw_zone_id: str | None = None
        self._active_yaw_zone_hold_start_sec: float | None = None
        self._active_yaw_zone_locked = False
        self._load_yaw_alignment_zones()

        self.pub_cmd = self.create_publisher(Twist, self.output_topic, 10)
        # HH_260625: Nav2 progress checker subscribes with transient_local QoS.
        # Publish effective engage state latched so late lifecycle subscribers receive it.
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = QoSReliabilityPolicy.RELIABLE
        state_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub_state = self.create_publisher(Bool, self.state_topic, state_qos)

        # HH_260415: Resolve robot pose in costmap frame through TF first.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.sub_cmd = self.create_subscription(
            Twist, self.input_topic, self._on_cmd, 10
        )
        self.sub_engage = self.create_subscription(
            Bool, self.engage_topic, self._on_engage, 10
        )
        self.sub_mission_engage = self.create_subscription(
            Bool, self.mission_engage_topic, self._on_mission_engage, 10
        )
        self.sub_parking_drop_zone_status = self.create_subscription(
            ModuleState,
            self.parking_drop_zone_status_topic,
            self._on_parking_drop_zone_status,
            10,
        )
        self.sub_parking_site_status = self.create_subscription(
            ModuleState,
            self.parking_site_status_topic,
            self._on_parking_site_status,
            10,
        )
        self.sub_localization_mode = None
        if self.enable_gnss_recovery_hold:
            self.sub_localization_mode = self.create_subscription(
                AvgLocalizationMode,
                self.localization_mode_topic,
                self._on_localization_mode,
                20,
            )

        self.sub_estop = None
        self.sub_additional_estops = []
        if self.estop_topic_enabled:
            self._estop_sources[self.estop_topic] = False
            self.sub_estop = self.create_subscription(
                Bool,
                self.estop_topic,
                lambda msg, topic=self.estop_topic: self._on_estop(msg, topic),
                10,
            )
            for topic in self.additional_estop_topics:
                if not topic or topic == self.estop_topic:
                    continue
                self._estop_sources[topic] = False
                self.sub_additional_estops.append(
                    self.create_subscription(
                        Bool,
                        topic,
                        lambda msg, topic=topic: self._on_estop(msg, topic),
                        10,
                    )
                )
        self.sub_dr_timeout = None
        if self.dr_timeout_topic_enabled:
            self.sub_dr_timeout = self.create_subscription(
                Bool, self.dr_timeout_topic, self._on_dr_timeout, 10
            )

        self.sub_cost_grid = None
        self.sub_lanelet_safety_grid = None
        self.sub_pose = None
        self.sub_odom = None
        if self.enable_cost_stop:
            # Merged cost grid for front/side/rear cost-stop checks.
            cost_qos = QoSProfile(depth=10)
            cost_qos.reliability = QoSReliabilityPolicy.RELIABLE
            cost_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.sub_cost_grid = self.create_subscription(
                OccupancyGrid, self.cost_grid_topic, self._on_cost_grid, cost_qos
            )
            if self.lanelet_safety_enable:
                self.sub_lanelet_safety_grid = self.create_subscription(
                    OccupancyGrid,
                    self.lanelet_safety_grid_topic,
                    self._on_lanelet_safety_grid,
                    cost_qos,
                )
            self.sub_cost_source_grids = []
            if self.cost_source_debug_enable or self.cost_stop_require_dynamic_source:
                for idx, topic in enumerate(self.cost_source_debug_topics):
                    label = (
                        str(self.cost_source_debug_labels[idx])
                        if idx < len(self.cost_source_debug_labels)
                        else str(topic)
                    )
                    self.sub_cost_source_grids.append(
                        self.create_subscription(
                            OccupancyGrid,
                            str(topic),
                            lambda msg, source_label=label: self._on_cost_source_grid(
                                source_label, msg
                            ),
                            cost_qos,
                        )
                    )
        self.sub_route_heading_path = None
        if self.enable_route_heading_alignment:
            self.sub_route_heading_path = self.create_subscription(
                Path,
                self.route_heading_path_topic,
                self._on_route_heading_path,
                10,
            )
        if (
            self.enable_cost_stop
            or self.enable_yaw_alignment_zone
            or self.enable_route_heading_alignment
        ):
            # Pose/odometry are also required by yaw-alignment and route-heading logic.
            # Keep pose topic as fallback when TF lookup is temporarily unavailable.
            self.sub_pose = self.create_subscription(
                PoseStamped, self.pose_topic, self._on_pose, 10
            )
            self.sub_odom = self.create_subscription(
                Odometry, self.odometry_topic, self._on_odom, 10
            )

        self._state_timer = self.create_timer(0.5, self._publish_state)
        self._cmd_timeout_timer = self.create_timer(
            1.0 / max(1.0, self.zero_publish_rate_hz),
            self._on_cmd_timeout_timer,
        )
        # HH_260415: Allow runtime threshold/profile tuning via ros2 param set.
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._publish_state()
        self.get_logger().info(
            "planning_cmd_vel_gate ready: "
            f"in={self.input_topic} out={self.output_topic} "
            f"manual_engage_topic={self.engage_topic} "
            f"mission_engage_topic={self.mission_engage_topic} "
            f"estop_topic={self.estop_topic if self.estop_topic_enabled else '(disabled)'} "
            f"allow_on_start={'true' if self.allow_on_start else 'false'} "
            f"speed_scale={self.speed_scale:.2f} "
            f"input_timeout_s={self.input_timeout_s:.2f} "
            f"zero_rate_hz={self.zero_publish_rate_hz:.1f} "
            f"gnss_recovery_hold={'true' if self.enable_gnss_recovery_hold else 'false'} "
            f"gnss_recovery_hold_s={self.gnss_recovery_hold_s:.2f}s "
            f"gnss_recovery_min_source_s={self.gnss_recovery_min_source_s:.2f}s "
            f"gnss_recovery_cooldown_s={self.gnss_recovery_hold_cooldown_s:.2f}s "
            f"cost_stop={'true' if self.enable_cost_stop else 'false'} "
            f"inflation_grid={self.cost_grid_topic} "
            f"lanelet_safety={'true' if self.lanelet_safety_enable else 'false'} "
            f"lanelet_grid={self.lanelet_safety_grid_topic} "
            f"speed_dependent={'true' if self.enable_speed_dependent_lookahead else 'false'} "
            f"front_lookahead=[{self.front_lookahead_min_m:.1f},{self.front_lookahead_max_m:.1f}]m "
            f"cost_source_debug={'true' if self.cost_source_debug_enable else 'false'} "
            f"dynamic_cost_stop={'true' if self.cost_stop_require_dynamic_source else 'false'} "
            f"dynamic_sources={sorted(self.cost_stop_dynamic_source_labels)} "
            f"side_rear={'true' if self.enable_side_rear_cost_stop else 'false'} "
            f"cost_stop_latch={'true' if self.cost_stop_latch_enable else 'false'} "
            f"clear_required={self.cost_stop_clear_required_s:.2f}s "
            f"cost_grid_stale_stop={'true' if self.cost_grid_stale_stop_enable else 'false'} "
            f"stale_timeout={self.cost_grid_stale_timeout_s:.2f}s "
            f"lateral_static_bypass={'true' if self.lateral_cmd_bypass_static_cost_stop else 'false'} "
            f"reverse_static_bypass={'true' if self.reverse_cmd_bypass_static_cost_stop else 'false'} "
            f"site_static_phases={sorted(self.parking_site_static_bypass_phases)} "
            f"rotation_dynamic_stop={'true' if self.rotation_cmd_dynamic_obstacle_stop else 'false'} "
            f"yaw_zone_align={'true' if self.enable_yaw_alignment_zone else 'false'} "
            f"route_heading_align={'true' if self.enable_route_heading_alignment else 'false'} "
            f"unavoidable_stop={'true' if self.enable_unavoidable_stop else 'false'}"
        )

    # Applies runtime parameter updates for gate/cost-stop tuning.
    def _on_set_parameters(self, params) -> SetParametersResult:
        reload_yaw_zone_cfg = False
        for p in params:
            if p.name == "enable_cost_stop":
                self.enable_cost_stop = bool(p.value)
            elif p.name == "cost_stop_threshold":
                self.cost_stop_threshold = int(p.value)
            elif p.name == "cost_stop_hold_s":
                self.cost_stop_hold_s = float(p.value)
            elif p.name == "cost_stop_latch_enable":
                self.cost_stop_latch_enable = bool(p.value)
                if not self.cost_stop_latch_enable:
                    self._cost_stop_latched = False
                    self._cost_stop_clear_since_sec = None
                    self._cost_stop_latch_reason = ""
            elif p.name == "cost_stop_clear_required_s":
                self.cost_stop_clear_required_s = float(p.value)
            elif p.name == "cost_stop_latch_log_interval_s":
                self.cost_stop_latch_log_interval_s = float(p.value)
            elif p.name == "cost_grid_stale_stop_enable":
                self.cost_grid_stale_stop_enable = bool(p.value)
            elif p.name == "cost_grid_stale_timeout_s":
                self.cost_grid_stale_timeout_s = float(p.value)
            elif p.name == "cost_grid_stale_log_interval_s":
                self.cost_grid_stale_log_interval_s = float(p.value)
            elif p.name == "cost_stop_lookahead_m":
                self.cost_stop_lookahead_m = float(p.value)
            elif p.name == "cost_stop_width_m":
                self.cost_stop_width_m = float(p.value)
            elif p.name == "cost_source_debug_enable":
                self.cost_source_debug_enable = bool(p.value)
            elif p.name == "cost_source_debug_max_age_s":
                self.cost_source_debug_max_age_s = float(p.value)
            elif p.name == "cost_stop_require_dynamic_source":
                self.cost_stop_require_dynamic_source = bool(p.value)
            elif p.name == "cost_stop_dynamic_source_labels":
                self.cost_stop_dynamic_source_labels = self._parse_source_label_set(
                    p.value,
                    {"lidar", "radar"},
                )
            elif p.name == "front_dynamic_stop_use_local_path":
                self.front_dynamic_stop_use_local_path = bool(p.value)
            elif p.name == "front_dynamic_path_width_m":
                self.front_dynamic_path_width_m = float(p.value)
            elif p.name == "front_dynamic_path_max_start_distance_m":
                self.front_dynamic_path_max_start_distance_m = float(p.value)
            elif p.name == "lanelet_safety_enable":
                self.lanelet_safety_enable = bool(p.value)
            elif p.name == "lanelet_safety_threshold":
                self.lanelet_safety_threshold = int(p.value)
            elif p.name == "lanelet_safety_current_threshold":
                self.lanelet_safety_current_threshold = int(p.value)
            elif p.name == "lanelet_safety_lookahead_m":
                self.lanelet_safety_lookahead_m = float(p.value)
            elif p.name == "lanelet_safety_width_m":
                self.lanelet_safety_width_m = float(p.value)
            elif p.name == "lanelet_safety_stop_on_unknown":
                self.lanelet_safety_stop_on_unknown = bool(p.value)
            elif p.name == "lanelet_safety_allow_rotation_in_place":
                self.lanelet_safety_allow_rotation_in_place = bool(p.value)
            elif p.name == "lanelet_safety_check_reverse":
                self.lanelet_safety_check_reverse = bool(p.value)
            elif p.name == "lanelet_safety_check_lateral":
                self.lanelet_safety_check_lateral = bool(p.value)
            elif p.name == "lanelet_safety_min_translation_mps":
                self.lanelet_safety_min_translation_mps = float(p.value)
            elif p.name == "lanelet_safety_front_use_local_path":
                self.lanelet_safety_front_use_local_path = bool(p.value)
            elif p.name == "lanelet_safety_front_path_max_start_distance_m":
                self.lanelet_safety_front_path_max_start_distance_m = float(p.value)
            elif p.name == "lanelet_safety_front_path_width_m":
                self.lanelet_safety_front_path_width_m = float(p.value)
            elif p.name == "lanelet_safety_front_path_allow_route_reentry":
                self.lanelet_safety_front_path_allow_route_reentry = bool(p.value)
            elif p.name == "lanelet_safety_current_allow_route_reentry":
                self.lanelet_safety_current_allow_route_reentry = bool(p.value)
            elif p.name == "lanelet_safety_current_route_reentry_max_distance_m":
                self.lanelet_safety_current_route_reentry_max_distance_m = float(p.value)
            elif p.name == "lanelet_safety_current_route_reentry_require_front_cmd":
                self.lanelet_safety_current_route_reentry_require_front_cmd = bool(p.value)
            elif p.name == "parking_site_static_bypass_phases":
                self.parking_site_static_bypass_phases = self._parse_source_label_set(
                    p.value,
                    {
                        "align_entry_yaw",
                        "reverse_in",
                        "crab_in",
                        "rotate_180",
                        "align_return_yaw",
                        "reverse_out",
                        "crab_out",
                    },
                )
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
            elif p.name == "enable_body_near_dynamic_stop":
                self.enable_body_near_dynamic_stop = bool(p.value)
            elif p.name == "body_near_side_lookahead_m":
                self.body_near_side_lookahead_m = float(p.value)
            elif p.name == "body_near_rear_lookahead_m":
                self.body_near_rear_lookahead_m = float(p.value)
            elif p.name == "body_near_maneuver_side_lookahead_m":
                self.body_near_maneuver_side_lookahead_m = float(p.value)
            elif p.name == "body_near_maneuver_rear_lookahead_m":
                self.body_near_maneuver_rear_lookahead_m = float(p.value)
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
            elif p.name == "lateral_cmd_bypass_static_cost_stop":
                self.lateral_cmd_bypass_static_cost_stop = bool(p.value)
            elif p.name == "lateral_cmd_bypass_min_mps":
                self.lateral_cmd_bypass_min_mps = float(p.value)
            elif p.name == "reverse_cmd_bypass_static_cost_stop":
                self.reverse_cmd_bypass_static_cost_stop = bool(p.value)
            elif p.name == "reverse_cmd_bypass_min_mps":
                self.reverse_cmd_bypass_min_mps = float(p.value)
            elif p.name == "lateral_cmd_dynamic_obstacle_threshold":
                self.lateral_cmd_dynamic_obstacle_threshold = int(p.value)
            elif p.name == "rotation_cmd_dynamic_obstacle_stop":
                self.rotation_cmd_dynamic_obstacle_stop = bool(p.value)
            elif p.name == "rotation_cmd_dynamic_obstacle_radius_m":
                self.rotation_cmd_dynamic_obstacle_radius_m = float(p.value)
            elif p.name == "rotation_cmd_dynamic_obstacle_threshold":
                self.rotation_cmd_dynamic_obstacle_threshold = int(p.value)
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
            elif p.name == "speed_scale":
                self.speed_scale = float(p.value)
            elif p.name == "input_timeout_s":
                self.input_timeout_s = float(p.value)
            elif p.name == "zero_publish_rate_hz":
                self.zero_publish_rate_hz = float(p.value)
                if hasattr(self, "_cmd_timeout_timer"):
                    self._cmd_timeout_timer.cancel()
                    self._cmd_timeout_timer = self.create_timer(
                        1.0 / max(1.0, self.zero_publish_rate_hz),
                        self._on_cmd_timeout_timer,
                    )
            elif p.name == "enable_gnss_recovery_hold":
                self.enable_gnss_recovery_hold = bool(p.value)
            elif p.name == "gnss_recovery_hold_s":
                self.gnss_recovery_hold_s = float(p.value)
            elif p.name == "gnss_recovery_min_source_s":
                self.gnss_recovery_min_source_s = float(p.value)
            elif p.name == "gnss_recovery_hold_cooldown_s":
                self.gnss_recovery_hold_cooldown_s = float(p.value)
            elif p.name == "gnss_recovery_source_mode_min":
                self.gnss_recovery_source_mode_min = int(p.value)
            elif p.name == "gnss_recovery_target_mode":
                self.gnss_recovery_target_mode = int(p.value)
            elif p.name == "enable_yaw_alignment_zone":
                self.enable_yaw_alignment_zone = bool(p.value)
                reload_yaw_zone_cfg = True
            elif p.name == "yaw_alignment_frame_id":
                self.yaw_alignment_frame_id = str(p.value)
            elif p.name == "yaw_alignment_zones_file":
                self.yaw_alignment_zones_file = str(p.value)
                reload_yaw_zone_cfg = True
            elif p.name == "yaw_alignment_exit_margin_m":
                self.yaw_alignment_exit_margin_m = float(p.value)
            elif p.name == "enable_route_heading_alignment":
                self.enable_route_heading_alignment = bool(p.value)
                if not self.enable_route_heading_alignment:
                    self._route_heading_align_active = False
            elif p.name == "route_heading_min_cmd_x_mps":
                self.route_heading_min_cmd_x_mps = float(p.value)
            elif p.name == "route_heading_lateral_cmd_epsilon_mps":
                self.route_heading_lateral_cmd_epsilon_mps = float(p.value)
            elif p.name == "route_heading_lookahead_m":
                self.route_heading_lookahead_m = float(p.value)
            elif p.name == "route_heading_error_enter_deg":
                self.route_heading_error_enter_deg = float(p.value)
            elif p.name == "route_heading_error_exit_deg":
                self.route_heading_error_exit_deg = float(p.value)
            elif p.name == "route_heading_angular_kp":
                self.route_heading_angular_kp = float(p.value)
            elif p.name == "route_heading_max_angular_z":
                self.route_heading_max_angular_z = float(p.value)
            elif p.name == "route_heading_max_linear_x":
                self.route_heading_max_linear_x = float(p.value)
            elif p.name == "route_heading_min_path_points":
                self.route_heading_min_path_points = int(p.value)
        if reload_yaw_zone_cfg:
            self._load_yaw_alignment_zones()
        return SetParametersResult(successful=True)

    # Returns whether cmd passthrough is currently allowed.
    def _effective_enabled(self) -> bool:
        if not (self._enabled and not self._estop):
            return False
        # HH_260507: Block when DR timeout (accumulated error too large).
        if self._dr_timeout:
            return False
        if self._cost_blocked_until > self.get_clock().now().nanoseconds * 1e-9:
            return False
        if self._gnss_recovery_blocked_until > self.get_clock().now().nanoseconds * 1e-9:
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

    # Publishes zero if raw controller commands stop arriving after motion began.
    def _on_cmd_timeout_timer(self) -> None:
        if self.input_timeout_s <= 0.0:
            return
        if self._last_input_cmd_time is None:
            return
        now = self.get_clock().now()
        age_s = (now - self._last_input_cmd_time).nanoseconds * 1e-9
        if age_s <= self.input_timeout_s:
            return
        if self.publish_zero_when_blocked:
            self._publish_zero()
        if self._cmd_input_stale:
            return
        self._cmd_input_stale = True
        now_sec = now.nanoseconds * 1e-9
        if (now_sec - self._last_cmd_input_stale_log_sec) >= 2.0:
            self._last_cmd_input_stale_log_sec = now_sec
            self.get_logger().warn(
                "cmd_vel input stale: "
                f"last_raw_cmd_age={age_s:.2f}s timeout={self.input_timeout_s:.2f}s; "
                "publishing zero"
            )

    # Handles raw cmd_vel input and applies engage/estop/cost-stop rules.
    def _on_cmd(self, msg: Twist) -> None:
        self._last_input_cmd_time = self.get_clock().now()
        self._cmd_input_stale = False
        if self._effective_enabled():
            cmd_to_publish = msg
            # Keep callback robust for lightweight unit-test doubles that may
            # bypass __init__ and not populate newly added yaw-zone fields.
            yaw_alignment_enabled = bool(
                getattr(self, "enable_yaw_alignment_zone", False)
            )
            yaw_alignment_zones = getattr(self, "_yaw_alignment_zones", [])
            if yaw_alignment_enabled and yaw_alignment_zones:
                override_cmd = self._apply_yaw_alignment_gate(msg)
                if override_cmd is not None:
                    cmd_to_publish = override_cmd
            route_heading_enabled = bool(getattr(self, "enable_route_heading_alignment", False))
            if cmd_to_publish is msg and route_heading_enabled:
                override_cmd = self._apply_route_heading_alignment_gate(msg)
                if override_cmd is not None:
                    cmd_to_publish = override_cmd
            # HH_260630: Heading/yaw alignment may rewrite controller commands,
            # but dynamic obstacle stop must still arbitrate the final motion.
            if self.enable_cost_stop and self._should_stop_for_cost(cmd_to_publish):
                if self.publish_zero_when_blocked:
                    self._publish_zero()
                return
            self.pub_cmd.publish(self._scale_twist(cmd_to_publish))
            return
        if self.publish_zero_when_blocked:
            self._publish_zero()
        self._log_block_reason()

    # Emits a throttled log identifying which gate condition is blocking cmd_vel.
    def _log_block_reason(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (now_sec - self._last_block_reason_log_sec) < 2.0:
            return
        self._last_block_reason_log_sec = now_sec
        now_ns = self.get_clock().now().nanoseconds
        reasons = []
        if not self._enabled:
            reasons.append(
                "engage=False("
                f"manual={str(getattr(self, '_manual_enabled', False)).lower()},"
                f"mission={str(getattr(self, '_mission_enabled', False)).lower()})"
            )
        if self._estop:
            reasons.append("estop=True")
        if self._dr_timeout:
            reasons.append("dr_timeout=True")
        now_sec = now_ns * 1e-9
        if self._cost_stop_latched:
            clear_elapsed = 0.0
            if self._cost_stop_clear_since_sec is not None:
                clear_elapsed = max(0.0, now_sec - self._cost_stop_clear_since_sec)
            reasons.append(
                "cost_latch("
                f"clear={clear_elapsed:.1f}/{max(0.0, self.cost_stop_clear_required_s):.1f}s"
                ")"
            )
        elif self._cost_blocked_until > now_sec:
            reasons.append(f"cost_hold({self._cost_blocked_until - now_sec:.1f}s left)")
        if self._gnss_recovery_blocked_until > now_ns * 1e-9:
            reasons.append(f"gnss_recovery_hold({self._gnss_recovery_blocked_until - now_ns * 1e-9:.1f}s left)")
        self.get_logger().warn(
            "cmd_vel BLOCKED: " + (", ".join(reasons) if reasons else "unknown")
        )

    # HH_260623 - Updates one engage latch without collapsing manual and mission intent.
    def _set_enabled(self, enabled: bool, source: str = "manual") -> None:
        source_key = str(source).strip().lower()
        new_latch = bool(enabled)
        prev_enabled = bool(self._enabled)
        prev_latch = (
            bool(getattr(self, "_mission_enabled", False))
            if source_key == "mission"
            else bool(getattr(self, "_manual_enabled", False))
        )

        if source_key == "mission":
            self._mission_enabled = new_latch
        else:
            self._manual_enabled = new_latch

        self._enabled = bool(
            getattr(self, "_manual_enabled", False)
            or getattr(self, "_mission_enabled", False)
        )
        if new_latch == prev_latch and self._enabled == prev_enabled:
            return

        self._publish_state()
        if not self._effective_enabled() and self.publish_zero_when_blocked:
            self._publish_zero()
        self.get_logger().info(
            "planning engage update: "
            f"source={source_key} "
            f"manual={'true' if getattr(self, '_manual_enabled', False) else 'false'} "
            f"mission={'true' if getattr(self, '_mission_enabled', False) else 'false'} "
            f"enabled={'true' if self._enabled else 'false'} "
            f"estop={'true' if self._estop else 'false'} "
            f"effective={'true' if self._effective_enabled() else 'false'}"
        )

    # Handles manual 2D-goal engage messages.
    def _on_engage(self, msg: Bool) -> None:
        self._set_enabled(msg.data, source="manual")

    # Handles UI mission engage messages.
    def _on_mission_engage(self, msg: Bool) -> None:
        self._set_enabled(msg.data, source="mission")

    # HH_260624 - Track parking phase so drop-zone exit can cross static
    # lanelet/drop-zone cost without disabling live obstacle protection.
    def _on_parking_drop_zone_status(self, msg: ModuleState) -> None:
        self._parking_drop_zone_phase = self._extract_phase_from_status(msg.message)

    def _on_parking_site_status(self, msg: ModuleState) -> None:
        self._parking_site_phase = self._extract_phase_from_status(msg.message)

    # Handles localization mode updates and applies DR->NORMAL recovery hold.
    def _on_localization_mode(self, msg: AvgLocalizationMode) -> None:
        current_mode = int(msg.value)
        prev_mode = self._last_localization_mode_value
        self._last_localization_mode_value = current_mode
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        prev_source = (
            prev_mode is not None
            and prev_mode >= int(self.gnss_recovery_source_mode_min)
        )
        current_source = current_mode >= int(self.gnss_recovery_source_mode_min)
        if current_source and not prev_source:
            self._gnss_recovery_source_enter_sec = now_sec

        if prev_mode is None or not self.enable_gnss_recovery_hold:
            return
        if self.gnss_recovery_hold_s <= 0.0:
            return

        recovered = (
            prev_source
            and current_mode == int(self.gnss_recovery_target_mode)
        )
        if not recovered:
            return

        source_enter_sec = self._gnss_recovery_source_enter_sec
        source_duration_s = (
            now_sec - source_enter_sec
            if source_enter_sec is not None
            else self.gnss_recovery_min_source_s
        )
        self._gnss_recovery_source_enter_sec = None
        if source_duration_s < self.gnss_recovery_min_source_s:
            self.get_logger().info(
                "localization recovery hold skipped: "
                f"mode {prev_mode}->{current_mode}, "
                f"source_duration={source_duration_s:.2f}s "
                f"< min={self.gnss_recovery_min_source_s:.2f}s"
            )
            return

        since_last_hold_s = now_sec - self._gnss_recovery_last_hold_sec
        if since_last_hold_s < self.gnss_recovery_hold_cooldown_s:
            self.get_logger().info(
                "localization recovery hold skipped: "
                f"mode {prev_mode}->{current_mode}, "
                f"cooldown={self.gnss_recovery_hold_cooldown_s - since_last_hold_s:.2f}s left"
            )
            return

        self._gnss_recovery_last_hold_sec = now_sec
        self._gnss_recovery_blocked_until = max(
            self._gnss_recovery_blocked_until,
            now_sec + self.gnss_recovery_hold_s,
        )
        self.get_logger().warn(
            "cmd_vel hold after localization recovery: "
            f"mode {prev_mode}->{current_mode}, "
            f"hold={self.gnss_recovery_hold_s:.2f}s"
        )
        self._publish_state()
        if self.publish_zero_when_blocked:
            self._publish_zero()

    # HH_260507: Blocks cmd_vel when DR timeout (covariance or time limit exceeded).
    # Clears automatically when localization_monitor resets (GNSS recovered).
    def _on_dr_timeout(self, msg: Bool) -> None:
        new_timeout = bool(msg.data)
        if new_timeout == self._dr_timeout:
            return
        self._dr_timeout = new_timeout
        self._publish_state()
        if self._dr_timeout and self.publish_zero_when_blocked:
            self._publish_zero()
        # HH_260629: rclpy binds severity to a call site. Keep WARN/INFO on
        # separate lines so DR timeout flaps do not crash the gate.
        if self._dr_timeout:
            self.get_logger().warn(
                f"DR timeout update: dr_timeout=true "
                f"effective={'true' if self._effective_enabled() else 'false'}"
            )
        else:
            self.get_logger().info(
                f"DR timeout update: dr_timeout=false "
                f"effective={'true' if self._effective_enabled() else 'false'}"
            )

    # Handles e-stop messages.
    def _on_estop(self, msg: Bool, topic: str | None = None) -> None:
        source_topic = str(topic or self.estop_topic)
        self._estop_sources[source_topic] = bool(msg.data)
        new_estop = any(self._estop_sources.values())
        if new_estop == self._estop:
            return
        self._estop = new_estop
        self._publish_state()
        if self._estop and self.publish_zero_when_blocked:
            self._publish_zero()
        self.get_logger().warn(
            "planning estop update: "
            f"source={source_topic} "
            f"estop={'true' if self._estop else 'false'} "
            f"sources={self._format_estop_sources()} "
            f"effective={'true' if self._effective_enabled() else 'false'}"
        )

    # Stores latest merged near_cost_grid (all directions).
    def _on_cost_grid(self, msg: OccupancyGrid) -> None:
        self._last_grid = msg
        self._last_grid_recv_sec = self.get_clock().now().nanoseconds * 1e-9

    # HH_260618: Stores raw lanelet safety grid before inflation ego-clear.
    def _on_lanelet_safety_grid(self, msg: OccupancyGrid) -> None:
        self._last_lanelet_safety_grid = msg

    # HH_260618: Stores original cost-grid sources for event-time attribution.
    def _on_cost_source_grid(self, label: str, msg: OccupancyGrid) -> None:
        self._cost_source_grids[label] = msg
        self._cost_source_recv_sec[label] = self.get_clock().now().nanoseconds * 1e-9

    # Stores latest pose fallback.
    def _on_pose(self, msg: PoseStamped) -> None:
        self._last_pose = msg

    # Stores latest odometry; extracts forward speed for lookahead computation.
    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg
        # HH_260422: twist.twist.linear.x is robot-frame forward speed (m/s).
        self._current_speed = float(msg.twist.twist.linear.x)

    # HH_260618: Stores the active local route used by route-heading alignment.
    def _on_route_heading_path(self, msg: Path) -> None:
        self._last_route_heading_path = msg

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

    # HH_260630: Build dynamic obstacle corridors from the commanded motion
    # direction. Sensor names stay body-fixed, but "front" for safety must follow
    # cmd_vel during reverse/crab parking maneuvers.
    def _cost_stop_corridors_for_cmd(
        self,
        cmd_in: Twist | None,
        front_lookahead: float,
    ) -> list[tuple[str, float, float, float, int, bool]]:
        front = (
            "FRONT",
            0.0,
            front_lookahead,
            self.cost_stop_width_m,
            self.cost_stop_threshold,
            True,
        )
        def side_rear_corridors(
            side_lookahead_m: float,
            rear_lookahead_m: float,
        ) -> list[tuple[str, float, float, float, int, bool]]:
            return [
                (
                    "LEFT",
                    math.pi / 2,
                    side_lookahead_m,
                    self.side_corridor_width_m,
                    self.side_cost_threshold,
                    False,
                ),
                (
                    "RIGHT",
                    -math.pi / 2,
                    side_lookahead_m,
                    self.side_corridor_width_m,
                    self.side_cost_threshold,
                    False,
                ),
                (
                    "REAR",
                    math.pi,
                    rear_lookahead_m,
                    self.rear_corridor_width_m,
                    self.rear_cost_threshold,
                    False,
                ),
            ]

        side_rear = side_rear_corridors(self.side_lookahead_m, self.rear_lookahead_m)

        if cmd_in is None:
            return [front] + (side_rear if self.enable_side_rear_cost_stop else [])

        min_cmd = max(0.0, float(self.lanelet_safety_min_translation_mps))
        vx = float(cmd_in.linear.x)
        vy = float(cmd_in.linear.y)
        corridors: list[tuple[str, float, float, float, int, bool]] = []
        maneuver_mode = vx < -min_cmd or abs(vy) > min_cmd
        active_side_near = (
            self.body_near_maneuver_side_lookahead_m
            if maneuver_mode
            else self.body_near_side_lookahead_m
        )
        active_rear_near = (
            self.body_near_maneuver_rear_lookahead_m
            if maneuver_mode
            else self.body_near_rear_lookahead_m
        )
        if maneuver_mode:
            # HH_260706 - Crab/reverse enters tight spaces intentionally, so
            # use shorter dynamic corridors than normal forward path-following.
            side_rear = side_rear_corridors(active_side_near, active_rear_near)

        if vx > min_cmd:
            corridors.append(front)
        elif self.enable_side_rear_cost_stop and vx < -min_cmd:
            corridors.append(side_rear[2])

        if self.enable_side_rear_cost_stop:
            if vy > min_cmd:
                corridors.append(side_rear[0])
            elif vy < -min_cmd:
                corridors.append(side_rear[1])

        if (
            self.enable_side_rear_cost_stop
            and self.enable_body_near_dynamic_stop
            and (abs(vx) > min_cmd or abs(vy) > min_cmd)
        ):
            # HH_260706 - Travel-direction checks are path-following friendly,
            # but close side/rear dynamic hits must still stop the body.
            # Forward driving uses the normal near-body guard. Crab/reverse
            # maneuvers use shorter adaptive distances.
            near_dynamic = [
                (
                    "LEFT_NEAR",
                    math.pi / 2,
                    max(0.0, float(active_side_near)),
                    self.side_corridor_width_m,
                    self.side_cost_threshold,
                    False,
                ),
                (
                    "RIGHT_NEAR",
                    -math.pi / 2,
                    max(0.0, float(active_side_near)),
                    self.side_corridor_width_m,
                    self.side_cost_threshold,
                    False,
                ),
                (
                    "REAR_NEAR",
                    math.pi,
                    max(0.0, float(active_rear_near)),
                    self.rear_corridor_width_m,
                    self.rear_cost_threshold,
                    False,
                ),
            ]
            existing_angles = {round(float(item[1]), 6) for item in corridors}
            for item in near_dynamic:
                if round(float(item[1]), 6) not in existing_angles:
                    corridors.append(item)

        return corridors

    # HH_260703: Latch dynamic cost-stop blocks until the obstacle corridor has
    # been continuously clear. Static lanelet safety can still use short holds.
    def _mark_cost_stop_blocked(
        self,
        now_sec: float,
        reason: str,
        *,
        latch: bool = True,
    ) -> None:
        self._cost_blocked_until = max(
            self._cost_blocked_until,
            now_sec + max(0.0, float(self.cost_stop_hold_s)),
        )
        if not (latch and self.cost_stop_latch_enable):
            return
        self._cost_stop_latched = True
        self._cost_stop_clear_since_sec = None
        self._cost_stop_latch_reason = reason

    def _cost_stop_latch_blocks(self, now_sec: float) -> bool:
        if not self._cost_stop_latched:
            return False
        if not self.cost_stop_latch_enable:
            self._cost_stop_latched = False
            self._cost_stop_clear_since_sec = None
            self._cost_stop_latch_reason = ""
            return False

        required_s = max(0.0, float(self.cost_stop_clear_required_s))
        if required_s <= 0.0:
            self._cost_stop_latched = False
            self._cost_stop_clear_since_sec = None
            self._cost_stop_latch_reason = ""
            return False

        if self._cost_stop_clear_since_sec is None:
            self._cost_stop_clear_since_sec = now_sec

        clear_elapsed = max(0.0, now_sec - self._cost_stop_clear_since_sec)
        if clear_elapsed >= required_s:
            reason = self._cost_stop_latch_reason or "dynamic_obstacle"
            self._cost_stop_latched = False
            self._cost_stop_clear_since_sec = None
            self._cost_stop_latch_reason = ""
            self.get_logger().info(
                f"cost-stop latch released: clear={clear_elapsed:.2f}s "
                f"required={required_s:.2f}s previous={reason}"
            )
            return False

        self._cost_blocked_until = max(
            self._cost_blocked_until,
            now_sec + max(0.0, float(self.cost_stop_hold_s)),
        )
        log_interval_s = max(0.0, float(self.cost_stop_latch_log_interval_s))
        if log_interval_s <= 0.0 or (
            now_sec - self._last_cost_stop_latch_log_sec
        ) >= log_interval_s:
            self._last_cost_stop_latch_log_sec = now_sec
            reason = self._cost_stop_latch_reason or "dynamic_obstacle"
            self.get_logger().warn(
                f"cost-stop latched: waiting_clear={clear_elapsed:.2f}/"
                f"{required_s:.2f}s previous={reason}"
        )
        return True

    def _cost_grid_stale_blocks(self, now_sec: float) -> bool:
        if not self.cost_grid_stale_stop_enable:
            return False
        timeout_s = max(0.0, float(self.cost_grid_stale_timeout_s))
        if timeout_s <= 0.0:
            return False

        reason = ""
        grid = self._last_grid
        if grid is None or not grid.data or grid.info.resolution <= 0.0:
            reason = "no valid merged cost grid"
        elif self._last_grid_recv_sec is None:
            reason = "merged cost grid timestamp missing"
        else:
            age_s = max(0.0, now_sec - self._last_grid_recv_sec)
            if age_s <= timeout_s:
                return False
            reason = (
                f"merged cost grid stale age={age_s:.2f}s "
                f"timeout={timeout_s:.2f}s"
            )

        self._mark_cost_stop_blocked(now_sec, reason, latch=False)
        log_interval_s = max(0.1, float(self.cost_grid_stale_log_interval_s))
        if (now_sec - self._last_cost_grid_stale_log_sec) >= log_interval_s:
            self._last_cost_grid_stale_log_sec = now_sec
            self.get_logger().warn(f"cmd_vel blocked: {reason}")
        return True

    # Checks all directional corridors and triggers stop when any is blocked.
    def _should_stop_for_cost(self, cmd_in: Twist | None = None) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        now_sec = now_ns * 1e-9

        if self._cost_grid_stale_blocks(now_sec):
            return True

        # HH_260618: Allow pure in-place rotation to re-align with the latest
        # route even when the current cell is near a lanelet boundary. Any
        # translational component still goes through lanelet/inflation checks.
        if (
            cmd_in is not None
            and self.lanelet_safety_allow_rotation_in_place
            and not self._is_translational_cmd(cmd_in)
        ):
            if self._should_stop_for_rotation_dynamic_obstacle(now_sec):
                return True
            return self._cost_stop_latch_blocks(now_sec)

        if self._should_stop_for_lanelet_safety(cmd_in, now_sec):
            return True
        site_static_bypass = self._should_bypass_static_cost_for_site_maneuver(cmd_in)

        front_lookahead = self._compute_front_lookahead()
        corridors = self._cost_stop_corridors_for_cmd(cmd_in, front_lookahead)
        if not corridors:
            return self._cost_stop_latch_blocks(now_sec)

        if self._should_stop_for_dynamic_source_corridors(corridors, now_sec):
            return True

        merged_grid = self._last_grid
        if merged_grid is None or not merged_grid.data or merged_grid.info.resolution <= 0.0:
            return self._cost_stop_latch_blocks(now_sec)

        target_frame = str(merged_grid.header.frame_id).strip()
        pose_candidates = self._resolve_pose_candidates(target_frame)
        if not pose_candidates:
            if (now_sec - self._last_empty_corridor_warn_sec) >= 2.0:
                self._last_empty_corridor_warn_sec = now_sec
                self.get_logger().warn(
                    "cost-stop: no pose candidates; "
                    "check pose/costmap frame alignment"
                )
            return self._cost_stop_latch_blocks(now_sec)

        best_total = -1
        best_label = ""
        best_lethal: list[tuple[int, int]] = []
        for label, pose in pose_candidates:
            for direction, yaw_off, la, wd, thr, check_unavoidable in corridors:
                if direction == "FRONT" and self.front_dynamic_stop_use_local_path:
                    path_valid, path_blocked, path_detail = (
                        self._sample_dynamic_local_path_corridor(
                            merged_grid,
                            pose,
                            lookahead=la,
                            width=self.front_dynamic_path_width_m,
                            threshold=thr,
                            max_start_distance=self.front_dynamic_path_max_start_distance_m,
                        )
                    )
                    if path_valid:
                        if not path_blocked:
                            self._log_front_path_dynamic_clear(
                                label,
                                "merged",
                                la,
                                path_detail,
                            )
                            continue
                        wx, wy, cost, _ = path_detail
                        blocked = True
                        total_cells = 0
                        lethal_cells = []
                        blocked_detail = (wx, wy, cost)
                    else:
                        blocked, total_cells, lethal_cells, blocked_detail = self._sample_cost_corridor(
                            merged_grid,
                            pose,
                            yaw_offset=yaw_off,
                            lookahead=la,
                            width=wd,
                            threshold=thr,
                        )
                else:
                    blocked, total_cells, lethal_cells, blocked_detail = self._sample_cost_corridor(
                        merged_grid,
                        pose,
                        yaw_offset=yaw_off,
                        lookahead=la,
                        width=wd,
                        threshold=thr,
                    )
                if blocked:
                    if not self._merged_cost_should_block(
                        blocked_detail,
                        threshold=thr,
                        direction=direction,
                        source_label=label,
                        lookahead_m=la,
                    ):
                        continue
                    if site_static_bypass and not self._dynamic_obstacle_source_blocks(
                        blocked_detail,
                        threshold=thr,
                    ):
                        self._log_site_static_cost_bypass(
                            direction, label, la, blocked_detail
                        )
                        continue
                    cause = self._format_cost_source_debug(blocked_detail)
                    reason = f"{direction}:{label}:{cause}"
                    self._mark_cost_stop_blocked(now_sec, reason, latch=True)
                    self.get_logger().warn(
                        f"cost-stop {direction}: source={label} "
                        f"lookahead={la:.2f}m "
                        f"speed={self._current_speed:.2f}m/s "
                        f"cause={cause} "
                        f"hold={self.cost_stop_hold_s:.2f}s"
                    )
                    return True
                if check_unavoidable and total_cells > best_total:
                    best_total = total_cells
                    best_label = label
                    best_lethal = lethal_cells

        if self.enable_unavoidable_stop and best_lethal:
            if self._is_unavoidable_cluster(best_lethal, max(1, best_total)):
                self._mark_cost_stop_blocked(
                    now_sec,
                    f"FRONT:unavoidable:{best_label}",
                    latch=True,
                )
                self.get_logger().warn(
                    f"cost-stop FRONT (unavoidable): source={best_label} "
                    f"lethal={self._last_unavoidable_cluster_cells} "
                    f"ratio={self._last_unavoidable_cluster_ratio:.2f} "
                    f"hold={self.cost_stop_hold_s:.2f}s"
                )
                return True

        return self._cost_stop_latch_blocks(now_sec)

    # HH_260630: Sample live dynamic source grids directly before consulting
    # the merged inflation grid. A static lanelet/global-path cell can be the
    # first high merged-grid hit in a side/rear corridor; ignoring that static
    # hit must not hide a live LiDAR/Radar obstacle farther along the same corridor.
    def _should_stop_for_dynamic_source_corridors(
        self,
        corridors: list[tuple[str, float, float, float, int, bool]],
        now_sec: float,
    ) -> bool:
        for source_label, grid in self._cost_source_grids.items():
            if not self._source_label_matches(source_label, self.cost_stop_dynamic_source_labels):
                continue
            recv_sec = self._cost_source_recv_sec.get(source_label, 0.0)
            if self.cost_source_debug_max_age_s > 0.0 and (
                now_sec - recv_sec
            ) > self.cost_source_debug_max_age_s:
                continue
            if grid is None or not grid.data or grid.info.resolution <= 0.0:
                continue

            target_frame = str(grid.header.frame_id).strip()
            for pose_label, pose in self._resolve_pose_candidates(target_frame):
                for direction, yaw_off, lookahead, width, threshold, _ in corridors:
                    if direction == "FRONT" and self.front_dynamic_stop_use_local_path:
                        path_valid, path_blocked, path_detail = (
                            self._sample_dynamic_local_path_corridor(
                                grid,
                                pose,
                                lookahead=lookahead,
                                width=self.front_dynamic_path_width_m,
                                threshold=threshold,
                                max_start_distance=self.front_dynamic_path_max_start_distance_m,
                            )
                        )
                        if path_valid:
                            if not path_blocked:
                                self._log_front_path_dynamic_clear(
                                    pose_label,
                                    source_label,
                                    lookahead,
                                    path_detail,
                                )
                                continue
                            wx, wy, cost, _ = path_detail
                            self._mark_cost_stop_blocked(
                                now_sec,
                                f"FRONT_PATH:{pose_label}:{source_label}:{cost}",
                                latch=True,
                            )
                            self.get_logger().warn(
                                f"cost-stop FRONT_PATH: source={pose_label} "
                                f"dynamic={source_label}:{cost}@({wx:.2f},{wy:.2f}) "
                                f"lookahead={lookahead:.2f}m "
                                f"hold={self.cost_stop_hold_s:.2f}s"
                            )
                            return True

                    blocked, _, _, detail = self._sample_cost_corridor(
                        grid,
                        pose,
                        yaw_offset=yaw_off,
                        lookahead=lookahead,
                        width=width,
                        threshold=threshold,
                    )
                    if not blocked:
                        continue
                    wx, wy, cost = detail if detail is not None else (pose[0], pose[1], -1)
                    self._mark_cost_stop_blocked(
                        now_sec,
                        f"{direction}:{pose_label}:{source_label}:{cost}",
                        latch=True,
                    )
                    self.get_logger().warn(
                        f"cost-stop {direction}: source={pose_label} "
                        f"dynamic={source_label}:{cost}@({wx:.2f},{wy:.2f}) "
                        f"lookahead={lookahead:.2f}m "
                        f"hold={self.cost_stop_hold_s:.2f}s"
                    )
                    return True
        return False

    def _log_front_path_dynamic_clear(
        self,
        pose_label: str,
        source_label: str,
        lookahead_m: float,
        detail: tuple[float, float, int, str],
    ) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (now_sec - self._last_front_path_dynamic_clear_log_sec) < 2.0:
            return
        self._last_front_path_dynamic_clear_log_sec = now_sec
        _, _, _, reason = detail
        self.get_logger().info(
            f"cost-stop FRONT_PATH clear: source={pose_label} dynamic={source_label} "
            f"lookahead={lookahead_m:.2f}m reason={reason}"
        )

    # HH_260618: Returns true only for mission-owned campsite parking commands.
    # Mixed x/y motion remains under normal cost-stop behavior.
    def _should_bypass_static_cost_for_site_maneuver(
        self, cmd_in: Twist | None
    ) -> bool:
        if cmd_in is None:
            return False
        if self._is_site_static_bypass_phase():
            return True
        min_lateral = max(0.0, float(self.lateral_cmd_bypass_min_mps))
        lateral_site_motion = (
            self.lateral_cmd_bypass_static_cost_stop
            and
            abs(float(cmd_in.linear.y)) > min_lateral
            and abs(float(cmd_in.linear.x)) <= min_lateral
        )
        min_reverse = max(0.0, float(self.reverse_cmd_bypass_min_mps))
        reverse_site_motion = (
            self.reverse_cmd_bypass_static_cost_stop
            and float(cmd_in.linear.x) < -min_reverse
            and abs(float(cmd_in.linear.y)) <= min_lateral
        )
        return lateral_site_motion or reverse_site_motion

    # HH_260622 - Normal driving must not stop on route/lanelet visualization
    # costs embedded in the merged grid. Only configured dynamic source grids
    # are allowed to turn a merged-grid hit into a cmd_vel block.
    def _merged_cost_should_block(
        self,
        blocked_detail: tuple[float, float, int] | None,
        *,
        threshold: int,
        direction: str,
        source_label: str,
        lookahead_m: float,
    ) -> bool:
        if not self.cost_stop_require_dynamic_source:
            return True
        if self._dynamic_obstacle_source_blocks(
            blocked_detail,
            threshold=threshold,
        ):
            return True
        self._log_static_guide_cost_ignored(
            direction,
            source_label,
            lookahead_m,
            blocked_detail,
        )
        return False

    # HH_260618: Static lanelet/global-path cost is allowed during explicit
    # site maneuver, but live obstacle sources must still stop parking motion.
    def _dynamic_obstacle_source_blocks(
        self,
        blocked_detail: tuple[float, float, int] | None,
        *,
        threshold: int | None = None,
    ) -> bool:
        if blocked_detail is None:
            return False
        wx, wy, _ = blocked_detail
        stop_threshold = int(
            threshold
            if threshold is not None
            else self.lateral_cmd_dynamic_obstacle_threshold
        )
        for label, grid in self._cost_source_grids.items():
            if not self._source_label_matches(label, self.cost_stop_dynamic_source_labels):
                continue
            recv_sec = self._cost_source_recv_sec.get(label, 0.0)
            if self.cost_source_debug_max_age_s > 0.0:
                now_sec = self.get_clock().now().nanoseconds * 1e-9
                if (now_sec - recv_sec) > self.cost_source_debug_max_age_s:
                    continue
            if self._sample_grid_cost(grid, wx, wy) >= stop_threshold:
                return True
        return False

    # HH_260624 - Pure yaw rotation has no front/rear direction, so sample a
    # body-centered disk from live dynamic sources before allowing static-cost bypass.
    def _should_stop_for_rotation_dynamic_obstacle(self, now_sec: float) -> bool:
        if not self.rotation_cmd_dynamic_obstacle_stop:
            return False
        radius = max(0.05, float(self.rotation_cmd_dynamic_obstacle_radius_m))
        threshold = int(self.rotation_cmd_dynamic_obstacle_threshold)
        for label, grid in self._cost_source_grids.items():
            if not self._source_label_matches(label, self.cost_stop_dynamic_source_labels):
                continue
            recv_sec = self._cost_source_recv_sec.get(label, 0.0)
            if self.cost_source_debug_max_age_s > 0.0 and (
                now_sec - recv_sec
            ) > self.cost_source_debug_max_age_s:
                continue
            if grid is None or not grid.data or grid.info.resolution <= 0.0:
                continue
            target_frame = str(grid.header.frame_id).strip()
            for pose_label, pose in self._resolve_pose_candidates(target_frame):
                blocked, detail = self._sample_dynamic_cost_disk(
                    grid,
                    pose,
                    radius=radius,
                    threshold=threshold,
                )
                if not blocked:
                    continue
                wx, wy, cost = detail
                self._mark_cost_stop_blocked(
                    now_sec,
                    f"ROTATE:{pose_label}:{label}:{cost}",
                    latch=True,
                )
                self.get_logger().warn(
                    f"cost-stop ROTATE: source={pose_label} dynamic={label} "
                    f"cost={cost} at=({wx:.2f},{wy:.2f}) radius={radius:.2f}m "
                    f"hold={self.cost_stop_hold_s:.2f}s"
                )
                return True
        return False

    def _sample_dynamic_cost_disk(
        self,
        grid: OccupancyGrid,
        pose: tuple[float, float, float],
        *,
        radius: float,
        threshold: int,
    ) -> tuple[bool, tuple[float, float, int]]:
        resolution = float(grid.info.resolution)
        if resolution <= 0.0:
            return False, (pose[0], pose[1], -1)
        origin_x = float(grid.info.origin.position.x)
        origin_y = float(grid.info.origin.position.y)
        center_x = int((pose[0] - origin_x) / resolution)
        center_y = int((pose[1] - origin_y) / resolution)
        width = int(grid.info.width)
        height = int(grid.info.height)
        cell_radius = max(1, int(math.ceil(radius / resolution)))
        best_cost = -1
        best_detail = (pose[0], pose[1], -1)

        for dy in range(-cell_radius, cell_radius + 1):
            gy = center_y + dy
            if gy < 0 or gy >= height:
                continue
            for dx in range(-cell_radius, cell_radius + 1):
                if (dx * resolution) ** 2 + (dy * resolution) ** 2 > radius * radius:
                    continue
                gx = center_x + dx
                if gx < 0 or gx >= width:
                    continue
                cost = int(grid.data[gy * width + gx])
                if cost > best_cost:
                    best_cost = cost
                    best_detail = (
                        origin_x + (gx + 0.5) * resolution,
                        origin_y + (gy + 0.5) * resolution,
                        cost,
                    )
                if cost >= threshold:
                    return True, (
                        origin_x + (gx + 0.5) * resolution,
                        origin_y + (gy + 0.5) * resolution,
                        cost,
                    )
        return False, best_detail

    # HH_260622 - Keep a visible breadcrumb when the merged grid is high only
    # because static route/lanelet layers are present, without spamming logs.
    def _log_static_guide_cost_ignored(
        self,
        direction: str,
        source_label: str,
        lookahead_m: float,
        blocked_detail: tuple[float, float, int] | None,
    ) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (now_sec - self._last_static_cost_ignored_log_sec) < 2.0:
            return
        self._last_static_cost_ignored_log_sec = now_sec
        cause = self._format_cost_source_debug(blocked_detail)
        self.get_logger().info(
            f"cost-stop static-guide ignored {direction}: source={source_label} "
            f"lookahead={lookahead_m:.2f}m cause={cause}"
        )

    # HH_260618: Throttled visibility for campsite maneuver static-cost bypass.
    def _log_site_static_cost_bypass(
        self,
        direction: str,
        source_label: str,
        lookahead_m: float,
        blocked_detail: tuple[float, float, int] | None,
    ) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (now_sec - self._last_lateral_static_bypass_log_sec) < 2.0:
            return
        self._last_lateral_static_bypass_log_sec = now_sec
        cause = self._format_cost_source_debug(blocked_detail)
        self.get_logger().info(
            f"site-maneuver static-cost bypass {direction}: source={source_label} "
            f"lookahead={lookahead_m:.2f}m cause={cause}"
        )

    # HH_260618: Returns true when cmd_vel has forward/rear/lateral translation.
    def _is_translational_cmd(self, cmd_in: Twist) -> bool:
        eps = max(0.0, float(self.lanelet_safety_min_translation_mps))
        return abs(float(cmd_in.linear.x)) > eps or abs(float(cmd_in.linear.y)) > eps

    def _parse_source_label_set(
        self,
        value,
        default_labels: set[str],
    ) -> set[str]:
        if value is None:
            return set(default_labels)
        if isinstance(value, str):
            raw_items = value.split(",")
        else:
            try:
                raw_items = list(value)
            except TypeError:
                raw_items = [value]
        labels = set()
        for item in raw_items:
            normalized = self._normalize_source_label(item)
            if normalized:
                labels.add(normalized)
        return labels or set(default_labels)

    def _parse_topic_list(self, value) -> list[str]:
        if value is None:
            return []
        if isinstance(value, str):
            raw_items = value.split(",")
        else:
            try:
                raw_items = list(value)
            except TypeError:
                raw_items = [value]
        topics: list[str] = []
        seen: set[str] = set()
        for item in raw_items:
            topic = str(item).strip()
            if not topic or topic in seen:
                continue
            topics.append(topic)
            seen.add(topic)
        return topics

    def _format_estop_sources(self) -> str:
        if not self._estop_sources:
            return "none"
        return ",".join(
            f"{topic}={'true' if active else 'false'}"
            for topic, active in sorted(self._estop_sources.items())
        )

    def _normalize_source_label(self, label) -> str:
        return str(label).strip().lower().replace("-", "_").replace(" ", "_")

    def _source_label_matches(self, label, accepted_labels: set[str]) -> bool:
        normalized = self._normalize_source_label(label)
        return normalized in accepted_labels or any(
            token and token in normalized for token in accepted_labels
        )

    # HH_260618: Blocks translation based on the raw lanelet cost grid. This
    # catches off-lane and lane-boundary penetration that can be hidden by the
    # merged inflation grid's ego-clear footprint.
    def _should_stop_for_lanelet_safety(
        self, cmd_in: Twist | None, now_sec: float
    ) -> bool:
        if not self.lanelet_safety_enable:
            return False
        if cmd_in is None:
            return False
        if not self._is_translational_cmd(cmd_in):
            return False

        grid = self._last_lanelet_safety_grid
        if grid is None or not grid.data or grid.info.resolution <= 0.0:
            return False

        target_frame = str(grid.header.frame_id).strip()
        pose_candidates = self._resolve_pose_candidates(target_frame)
        if not pose_candidates:
            if (now_sec - self._last_empty_corridor_warn_sec) >= 2.0:
                self._last_empty_corridor_warn_sec = now_sec
                self.get_logger().warn(
                    "lanelet-safety: no pose candidates; "
                    "check pose/lanelet costmap frame alignment"
                )
            return False

        directions: list[tuple[str, float, float]] = []
        min_cmd = max(0.0, float(self.lanelet_safety_min_translation_mps))
        if float(cmd_in.linear.x) > min_cmd:
            directions.append(("FRONT", 0.0, self.lanelet_safety_lookahead_m))
        if self.lanelet_safety_check_reverse and float(cmd_in.linear.x) < -min_cmd:
            directions.append(("REAR", math.pi, self.lanelet_safety_lookahead_m))
        if self.lanelet_safety_check_lateral and float(cmd_in.linear.y) > min_cmd:
            directions.append(("LEFT", math.pi / 2.0, self.lanelet_safety_lookahead_m))
        if self.lanelet_safety_check_lateral and float(cmd_in.linear.y) < -min_cmd:
            directions.append(("RIGHT", -math.pi / 2.0, self.lanelet_safety_lookahead_m))

        if not directions:
            return False
        if self._should_bypass_static_lanelet_for_drop_zone_exit(cmd_in, now_sec):
            return False
        if self._should_bypass_static_lanelet_for_site_maneuver(cmd_in, now_sec):
            return False

        for label, pose in pose_candidates:
            pose_x, pose_y, _ = pose
            current_reentry_bypass = False
            inside, current_cost = self._sample_grid_cost_detail(grid, pose_x, pose_y)
            if not inside and self.lanelet_safety_stop_on_unknown:
                self._cost_blocked_until = now_sec + self.cost_stop_hold_s
                self.get_logger().warn(
                    f"lanelet-safety CURRENT: source={label} out_of_grid "
                    f"hold={self.cost_stop_hold_s:.2f}s"
                )
                return True
            if inside and current_cost >= self.lanelet_safety_current_threshold:
                current_reentry_bypass = self._can_bypass_lanelet_current_for_route_reentry(
                    cmd_in,
                    target_frame,
                    pose,
                    current_cost,
                    label,
                    now_sec,
                )
                if not current_reentry_bypass:
                    self._cost_blocked_until = now_sec + self.cost_stop_hold_s
                    self.get_logger().warn(
                        f"lanelet-safety CURRENT: source={label} "
                        f"cost={current_cost} threshold={self.lanelet_safety_current_threshold} "
                        f"hold={self.cost_stop_hold_s:.2f}s"
                    )
                    return True

            for direction, yaw_offset, lookahead in directions:
                if direction == "FRONT" and self.lanelet_safety_front_use_local_path:
                    # HH_260622 - Keep using the selected local path during
                    # route re-entry even after the current cell drops below
                    # the hard threshold. Otherwise the raw robot-yaw rectangle
                    # can re-block near lane boundaries before the robot has
                    # fully converged to the centerline.
                    route_reentry_max_start = (
                        self.lanelet_safety_current_route_reentry_max_distance_m
                        if self.lanelet_safety_current_allow_route_reentry
                        else None
                    )
                    path_available, path_blocked, path_detail = (
                        self._sample_lanelet_safety_local_path_corridor(
                            grid,
                            pose,
                            lookahead=lookahead,
                            width=self.lanelet_safety_front_path_width_m,
                            threshold=self.lanelet_safety_threshold,
                            max_start_distance=(
                                route_reentry_max_start
                                if current_reentry_bypass or route_reentry_max_start is not None
                                else None
                            ),
                        )
                    )
                    if path_available:
                        if path_blocked:
                            if self._can_bypass_lanelet_front_path_for_route_reentry(
                                cmd_in,
                                target_frame,
                                pose,
                                path_detail,
                                label,
                                now_sec,
                            ):
                                continue
                            self._cost_blocked_until = now_sec + self.cost_stop_hold_s
                            wx, wy, cost, reason = path_detail
                            self.get_logger().warn(
                                f"lanelet-safety FRONT_PATH: source={label} "
                                f"cost={cost} reason={reason} at=({wx:.2f},{wy:.2f}) "
                                f"lookahead={lookahead:.2f}m width={self.lanelet_safety_front_path_width_m:.2f}m "
                                f"hold={self.cost_stop_hold_s:.2f}s"
                            )
                            return True
                        continue
                blocked, detail = self._sample_lanelet_safety_corridor(
                    grid,
                    pose,
                    yaw_offset=yaw_offset,
                    lookahead=lookahead,
                    width=self.lanelet_safety_width_m,
                    threshold=self.lanelet_safety_threshold,
                )
                if blocked:
                    self._cost_blocked_until = now_sec + self.cost_stop_hold_s
                    wx, wy, cost, reason = detail
                    self.get_logger().warn(
                        f"lanelet-safety {direction}: source={label} "
                        f"cost={cost} reason={reason} at=({wx:.2f},{wy:.2f}) "
                        f"lookahead={lookahead:.2f}m width={self.lanelet_safety_width_m:.2f}m "
                        f"hold={self.cost_stop_hold_s:.2f}s"
                    )
                    return True

        return False

    # HH_260622 - Route re-entry guard for map-agnostic simulation starts:
    # a raw pose can be outside lanelet while the selected local path is valid
    # and close. Let the robot move toward that path, but only for forward
    # commands and only within a bounded distance.
    def _can_bypass_lanelet_current_for_route_reentry(
        self,
        cmd_in: Twist,
        frame_id: str,
        pose: tuple[float, float, float],
        current_cost: int,
        source_label: str,
        now_sec: float,
    ) -> bool:
        if not self.lanelet_safety_current_allow_route_reentry:
            return False
        min_cmd = max(0.0, float(self.lanelet_safety_min_translation_mps))
        if (
            self.lanelet_safety_current_route_reentry_require_front_cmd
            and float(cmd_in.linear.x) <= min_cmd
        ):
            return False
        max_dist = max(
            0.0,
            float(self.lanelet_safety_current_route_reentry_max_distance_m),
        )
        if max_dist <= 0.0:
            return False
        distance_m = self._closest_route_path_distance_m(frame_id, pose[0], pose[1])
        if distance_m is None or distance_m > max_dist:
            return False
        if (now_sec - self._last_lanelet_current_reentry_bypass_log_sec) >= 1.0:
            self._last_lanelet_current_reentry_bypass_log_sec = now_sec
            self.get_logger().warn(
                "lanelet-safety CURRENT route-reentry bypass: "
                f"source={source_label} cost={current_cost} "
                f"path_dist={distance_m:.2f}m max={max_dist:.2f}m"
            )
        return True

    # HH_260624 - FRONT_PATH is a static map safety layer. During controlled
    # route re-entry from a drop-zone/site, block dynamic obstacles elsewhere
    # but do not let the first off-lane static cells prevent reaching the lanelet.
    def _can_bypass_lanelet_front_path_for_route_reentry(
        self,
        cmd_in: Twist,
        frame_id: str,
        pose: tuple[float, float, float],
        path_detail: tuple[float, float, int, str],
        source_label: str,
        now_sec: float,
    ) -> bool:
        if not self.lanelet_safety_front_path_allow_route_reentry:
            return False
        if not self.lanelet_safety_current_allow_route_reentry:
            return False
        min_cmd = max(0.0, float(self.lanelet_safety_min_translation_mps))
        if (
            self.lanelet_safety_current_route_reentry_require_front_cmd
            and float(cmd_in.linear.x) <= min_cmd
        ):
            return False
        max_dist = max(
            0.0,
            float(self.lanelet_safety_current_route_reentry_max_distance_m),
        )
        if max_dist <= 0.0:
            return False
        path_dist_m = self._closest_route_path_distance_m(frame_id, pose[0], pose[1])
        if path_dist_m is None or path_dist_m > max_dist:
            return False
        wx, wy, cost, reason = path_detail
        blocked_point_dist_m = math.hypot(float(wx) - pose[0], float(wy) - pose[1])
        if blocked_point_dist_m > max(0.05, float(self.lanelet_safety_lookahead_m)):
            return False
        if (now_sec - self._last_lanelet_front_path_reentry_bypass_log_sec) >= 1.0:
            self._last_lanelet_front_path_reentry_bypass_log_sec = now_sec
            self.get_logger().warn(
                "lanelet-safety FRONT_PATH route-reentry bypass: "
                f"source={source_label} cost={cost} reason={reason} "
                f"path_dist={path_dist_m:.2f}m blocked_dist={blocked_point_dist_m:.2f}m "
                f"max={max_dist:.2f}m"
            )
        return True

    def _extract_phase_from_status(self, message: str) -> str:
        # HH_260624 - ModuleState.message uses "phase=<PHASE> ..." from
        # camrod_parking; keep parsing local to avoid adding a new message type.
        for token in str(message).split():
            if token.startswith("phase="):
                return token.split("=", 1)[1].strip()
        return ""

    def _should_bypass_static_lanelet_for_drop_zone_exit(
        self, cmd_in: Twist, now_sec: float
    ) -> bool:
        if not self._is_drop_zone_exit_phase():
            return False
        min_cmd = max(0.0, float(self.lanelet_safety_min_translation_mps))
        if float(cmd_in.linear.x) <= min_cmd:
            return False
        if abs(float(cmd_in.linear.y)) > min_cmd:
            return False
        if (now_sec - self._last_drop_zone_static_bypass_log_sec) >= 1.0:
            self._last_drop_zone_static_bypass_log_sec = now_sec
            self.get_logger().warn(
                "lanelet-safety drop-zone exit static bypass: "
                f"phase={self._parking_drop_zone_phase} "
                f"cmd_x={float(cmd_in.linear.x):.2f}m/s"
            )
        return True

    def _should_bypass_static_lanelet_for_site_maneuver(
        self, cmd_in: Twist, now_sec: float
    ) -> bool:
        if not self._is_site_static_bypass_phase():
            return False
        if not self._is_translational_cmd(cmd_in):
            return False
        if (now_sec - self._last_site_static_phase_bypass_log_sec) >= 1.0:
            self._last_site_static_phase_bypass_log_sec = now_sec
            self.get_logger().warn(
                "lanelet-safety site-maneuver static bypass: "
                f"phase={self._parking_site_phase} "
                f"cmd_x={float(cmd_in.linear.x):.2f}m/s "
                f"cmd_y={float(cmd_in.linear.y):.2f}m/s"
            )
        return True

    def _log_site_lanelet_static_bypass(self, cmd_in: Twist, now_sec: float) -> None:
        if (now_sec - self._last_lateral_static_bypass_log_sec) < 2.0:
            return
        self._last_lateral_static_bypass_log_sec = now_sec
        self.get_logger().info(
            "lanelet-safety site-maneuver static bypass: "
            f"cmd_x={float(cmd_in.linear.x):.2f}m/s "
            f"cmd_y={float(cmd_in.linear.y):.2f}m/s"
        )

    def _is_drop_zone_exit_phase(self) -> bool:
        # HH_260624 - Keep drop-zone exit phase checks centralized so route
        # heading and static lanelet safety agree on parking-owned motion.
        phase = self._normalize_source_label(self._parking_drop_zone_phase)
        return bool(phase and phase in self.parking_drop_zone_static_bypass_phases)

    def _is_site_static_bypass_phase(self) -> bool:
        phase = self._normalize_source_label(self._parking_site_phase)
        return bool(phase and phase in self.parking_site_static_bypass_phases)

    def _closest_route_path_distance_m(
        self, frame_id: str, pose_x: float, pose_y: float
    ) -> float | None:
        path = self._last_route_heading_path
        if path is None or len(path.poses) < 2:
            return None
        path_frame = str(path.header.frame_id).strip()
        target_frame = str(frame_id).strip()
        if path_frame and target_frame and path_frame != target_frame:
            return None

        points: list[tuple[float, float]] = []
        for pose_stamped in path.poses:
            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)
            if math.isfinite(x) and math.isfinite(y):
                points.append((x, y))
        if len(points) < 2:
            return None

        best_dist_sq = float("inf")
        for idx in range(len(points) - 1):
            ax, ay = points[idx]
            bx, by = points[idx + 1]
            dx = bx - ax
            dy = by - ay
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq <= 1e-9:
                dist_sq = (pose_x - ax) * (pose_x - ax) + (pose_y - ay) * (pose_y - ay)
            else:
                t = ((pose_x - ax) * dx + (pose_y - ay) * dy) / seg_len_sq
                t = max(0.0, min(1.0, t))
                proj_x = ax + t * dx
                proj_y = ay + t * dy
                dist_sq = (
                    (pose_x - proj_x) * (pose_x - proj_x)
                    + (pose_y - proj_y) * (pose_y - proj_y)
                )
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq

        if not math.isfinite(best_dist_sq):
            return None
        return math.sqrt(best_dist_sq)

    # Loads yaw-alignment zones from YAML (if enabled).
    def _load_yaw_alignment_zones(self) -> None:
        self._yaw_alignment_zones = []
        self._active_yaw_zone_id = None
        self._active_yaw_zone_hold_start_sec = None
        self._active_yaw_zone_locked = False

        if not self.enable_yaw_alignment_zone:
            return

        cfg = self.yaw_alignment_zones_file.strip()
        if not cfg:
            self.get_logger().warn(
                "yaw alignment zone is enabled but yaw_alignment_zones_file is empty; "
                "feature remains inactive"
            )
            return
        if not os.path.isfile(cfg):
            self.get_logger().error(f"yaw alignment zones file not found: {cfg}")
            return

        try:
            with open(cfg, "r", encoding="utf-8") as f:
                raw = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error(
                f"failed to load yaw alignment zones yaml ({cfg}): {exc}"
            )
            return

        container = raw.get("yaw_alignment_zones", raw)
        if not isinstance(container, dict):
            self.get_logger().error(f"invalid yaw alignment zones format: {cfg}")
            return

        frame_override = str(container.get("frame_id", "")).strip()
        if frame_override:
            self.yaw_alignment_frame_id = frame_override
        zones = container.get("zones", [])
        if not isinstance(zones, list):
            self.get_logger().error(f"invalid zones list in {cfg}")
            return

        loaded: list[YawAlignmentZone] = []
        for idx, zone_cfg in enumerate(zones):
            if not isinstance(zone_cfg, dict):
                continue
            try:
                zone_id = str(zone_cfg.get("id", f"zone_{idx+1}")).strip()
                if not zone_id:
                    zone_id = f"zone_{idx+1}"
                x = float(zone_cfg.get("x", 0.0))
                y = float(zone_cfg.get("y", 0.0))
                z = float(zone_cfg.get("z", 0.0))
                if "yaw_deg" in zone_cfg:
                    yaw_rad = math.radians(float(zone_cfg.get("yaw_deg", 0.0)))
                elif "next_x" in zone_cfg and "next_y" in zone_cfg:
                    # Optional tangent-style yaw: from zone center to next waypoint.
                    yaw_rad = math.atan2(
                        float(zone_cfg["next_y"]) - y,
                        float(zone_cfg["next_x"]) - x,
                    )
                else:
                    self.get_logger().warn(
                        f"yaw zone '{zone_id}' skipped: yaw_deg or next_x/next_y required"
                    )
                    continue

                position_tolerance_m = max(
                    0.05, float(zone_cfg.get("position_tolerance_m", 0.10))
                )
                activation_radius_m = max(
                    position_tolerance_m + 0.05,
                    float(zone_cfg.get("activation_radius_m", 1.2)),
                )
                lock_radius_default = max(
                    position_tolerance_m + 0.15,
                    activation_radius_m * 0.6,
                )
                lock_radius_m = max(
                    position_tolerance_m,
                    float(zone_cfg.get("lock_radius_m", lock_radius_default)),
                )

                loaded.append(
                    YawAlignmentZone(
                        zone_id=zone_id,
                        x=x,
                        y=y,
                        z=z,
                        target_yaw_rad=self._normalize_yaw(yaw_rad),
                        activation_radius_m=activation_radius_m,
                        lock_radius_m=lock_radius_m,
                        position_tolerance_m=position_tolerance_m,
                        yaw_tolerance_deg=max(
                            1.0, float(zone_cfg.get("yaw_tolerance_deg", 8.0))
                        ),
                        yaw_tolerance_per_meter_deg=max(
                            0.0,
                            float(zone_cfg.get("yaw_tolerance_per_meter_deg", 4.0)),
                        ),
                        hold_s=max(0.0, float(zone_cfg.get("hold_s", 0.5))),
                        angular_kp=max(0.1, float(zone_cfg.get("angular_kp", 1.8))),
                        max_angular_z=max(
                            0.05, float(zone_cfg.get("max_angular_z", 0.8))
                        ),
                        max_approach_linear_x=max(
                            0.0,
                            float(zone_cfg.get("max_approach_linear_x", 0.25)),
                        ),
                    )
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"failed to parse yaw zone index={idx}: {exc}"
                )

        self._yaw_alignment_zones = loaded
        self.get_logger().info(
            "yaw alignment zones loaded: "
            f"count={len(self._yaw_alignment_zones)} "
            f"frame={self.yaw_alignment_frame_id} file={cfg}"
        )

    # Applies yaw-alignment gate and returns override cmd when lock is active.
    def _apply_yaw_alignment_gate(self, cmd_in: Twist) -> Twist | None:
        pose_candidates = self._resolve_pose_candidates(self.yaw_alignment_frame_id)
        if not pose_candidates:
            return None
        source_label, pose = pose_candidates[0]
        pose_x, pose_y, pose_yaw = pose
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        zone = self._select_active_yaw_zone(pose_x, pose_y)
        if zone is None:
            self._active_yaw_zone_id = None
            self._active_yaw_zone_hold_start_sec = None
            self._active_yaw_zone_locked = False
            return None

        if zone.zone_id != self._active_yaw_zone_id:
            self._active_yaw_zone_id = zone.zone_id
            self._active_yaw_zone_hold_start_sec = None
            self._active_yaw_zone_locked = False

        # Once zone requirement is satisfied, keep unlocked passthrough until
        # robot exits the zone + margin to avoid re-lock oscillation.
        if self._active_yaw_zone_locked:
            return None

        dist_m = math.hypot(pose_x - zone.x, pose_y - zone.y)
        if dist_m > zone.lock_radius_m:
            # Track zone entry in activation ring but only enforce inside lock radius.
            self._active_yaw_zone_hold_start_sec = None
            return None

        yaw_error_rad = self._normalize_yaw(zone.target_yaw_rad - pose_yaw)
        yaw_error_deg = abs(math.degrees(yaw_error_rad))
        pos_error_m = max(0.0, dist_m - zone.position_tolerance_m)
        yaw_tol_deg_eff = (
            zone.yaw_tolerance_deg
            + zone.yaw_tolerance_per_meter_deg * pos_error_m
        )

        yaw_ok = yaw_error_deg <= yaw_tol_deg_eff
        pos_ok = dist_m <= zone.position_tolerance_m
        if yaw_ok and pos_ok:
            if self._active_yaw_zone_hold_start_sec is None:
                self._active_yaw_zone_hold_start_sec = now_sec
            hold_elapsed = now_sec - self._active_yaw_zone_hold_start_sec
            if hold_elapsed >= zone.hold_s:
                self._active_yaw_zone_locked = True
                self.get_logger().info(
                    f"yaw alignment satisfied: zone={zone.zone_id} source={source_label} "
                    f"dist={dist_m:.2f}m yaw_err={yaw_error_deg:.2f}deg hold={hold_elapsed:.2f}s"
                )
                return None
        else:
            self._active_yaw_zone_hold_start_sec = None

        cmd = Twist()
        gain_scale = 1.0 + min(1.0, pos_error_m)
        cmd.angular.z = self._clamp(
            zone.angular_kp * gain_scale * yaw_error_rad,
            -zone.max_angular_z,
            zone.max_angular_z,
        )
        if dist_m > zone.position_tolerance_m:
            # Position-aware coupling: large yaw error suppresses approach speed.
            yaw_stop_deg = max(1.0, zone.yaw_tolerance_deg * 2.5)
            yaw_scale = max(0.0, 1.0 - (yaw_error_deg / yaw_stop_deg))
            cmd.linear.x = self._clamp(
                float(cmd_in.linear.x),
                -zone.max_approach_linear_x,
                zone.max_approach_linear_x,
            ) * yaw_scale
        else:
            cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        if (now_sec - self._last_yaw_align_log_sec) >= 1.0:
            self._last_yaw_align_log_sec = now_sec
            self.get_logger().warn(
                f"yaw alignment active: zone={zone.zone_id} source={source_label} "
                f"dist={dist_m:.2f}m yaw_err={yaw_error_deg:.2f}deg "
                f"yaw_tol={yaw_tol_deg_eff:.2f}deg"
            )
        return cmd

    # HH_260618: Holds forward motion until robot yaw agrees with route tangent.
    def _apply_route_heading_alignment_gate(self, cmd_in: Twist) -> Twist | None:
        if not self.enable_route_heading_alignment:
            self._route_heading_align_active = False
            return None
        if self._is_drop_zone_exit_phase():
            # HH_260624 - During explicit drop-zone departure, the parking
            # node owns straight exit/alignment before any Nav2 local path is valid.
            self._route_heading_align_active = False
            return None
        if self._is_site_static_bypass_phase():
            # HH_260701 - During campsite entry/return, camrod_parking owns
            # the body-frame command and Nav2 route tangent should not rewrite it.
            self._route_heading_align_active = False
            return None
        if abs(float(cmd_in.linear.y)) > self.route_heading_lateral_cmd_epsilon_mps:
            # Crab/site maneuvers are explicit non-Nav2 phases; do not override them.
            self._route_heading_align_active = False
            return None
        if float(cmd_in.linear.x) < -self.route_heading_min_cmd_x_mps:
            # Reverse motion is reserved for parking/recovery nodes and must not
            # be converted into a route-facing turn by the normal driving gate.
            self._route_heading_align_active = False
            return None
        if (
            not self._route_heading_align_active
            and float(cmd_in.linear.x) <= self.route_heading_min_cmd_x_mps
        ):
            return None

        path = self._last_route_heading_path
        if path is None:
            self._route_heading_align_active = False
            return None
        path_frame = str(path.header.frame_id).strip() or self.route_heading_frame_id
        pose_candidates = self._resolve_pose_candidates(path_frame)
        if not pose_candidates:
            self._route_heading_align_active = False
            return None
        source_label, pose = pose_candidates[0]
        pose_x, pose_y, pose_yaw = pose

        heading = self._route_heading_from_path(path, pose_x, pose_y, pose_yaw)
        if heading is None:
            self._route_heading_align_active = False
            return None
        desired_yaw, closest_dist_m, start_idx, target_idx = heading
        yaw_error_rad = self._normalize_yaw(desired_yaw - pose_yaw)
        yaw_error_deg = abs(math.degrees(yaw_error_rad))

        enter_deg = max(1.0, self.route_heading_error_enter_deg)
        exit_deg = max(1.0, min(self.route_heading_error_exit_deg, enter_deg))
        if self._route_heading_align_active:
            if yaw_error_deg <= exit_deg:
                self._route_heading_align_active = False
                return None
        elif yaw_error_deg >= enter_deg:
            self._route_heading_align_active = True
        else:
            return None

        cmd = Twist()
        cmd.linear.x = self._clamp(
            float(cmd_in.linear.x),
            0.0,
            max(0.0, self.route_heading_max_linear_x),
        )
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.z = self._clamp(
            self.route_heading_angular_kp * yaw_error_rad,
            -abs(self.route_heading_max_angular_z),
            abs(self.route_heading_max_angular_z),
        )

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (now_sec - self._last_route_heading_log_sec) >= 1.0:
            self._last_route_heading_log_sec = now_sec
            self.get_logger().warn(
                "route-heading align active: "
                f"source={source_label} yaw_err={yaw_error_deg:.1f}deg "
                f"path_idx={start_idx}->{target_idx} nearest={closest_dist_m:.2f}m "
                f"cmd_x={cmd_in.linear.x:.2f}"
            )
        return cmd

    # HH_260618: Computes active path tangent near the robot for heading guard.
    def _route_heading_from_path(
        self, path: Path, pose_x: float, pose_y: float, pose_yaw: float
    ) -> tuple[float, float, int, int] | None:
        points: list[tuple[float, float]] = []
        for pose_stamped in path.poses:
            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)
            if math.isfinite(x) and math.isfinite(y):
                points.append((x, y))
        min_points = max(2, int(self.route_heading_min_path_points))
        if len(points) < min_points:
            return None

        closest_idx = 0
        closest_dist_sq = float("inf")
        forward_candidates: list[tuple[float, int]] = []
        forward_x = math.cos(pose_yaw)
        forward_y = math.sin(pose_yaw)
        for idx, (x, y) in enumerate(points):
            dist_sq = (x - pose_x) * (x - pose_x) + (y - pose_y) * (y - pose_y)
            if dist_sq < closest_dist_sq:
                closest_dist_sq = dist_sq
                closest_idx = idx
            projection = forward_x * (x - pose_x) + forward_y * (y - pose_y)
            if projection >= -0.30:
                forward_candidates.append((dist_sq, idx))

        if forward_candidates:
            # HH_260706 - Prefer path points not behind the robot so restart
            # alignment does not turn toward a stale/behind local-path segment.
            closest_dist_sq, closest_idx = min(forward_candidates, key=lambda item: item[0])

        lookahead = max(0.10, float(self.route_heading_lookahead_m))
        target_idx = closest_idx
        accumulated = 0.0
        prev_x, prev_y = points[closest_idx]
        for idx in range(closest_idx + 1, len(points)):
            x, y = points[idx]
            step = math.hypot(x - prev_x, y - prev_y)
            if step <= 1e-4:
                continue
            accumulated += step
            prev_x, prev_y = x, y
            target_idx = idx
            if accumulated >= lookahead:
                break

        start_idx = closest_idx
        if target_idx == closest_idx:
            if closest_idx <= 0:
                return None
            start_idx = closest_idx - 1
            target_idx = closest_idx

        start_x, start_y = points[start_idx]
        target_x, target_y = points[target_idx]
        if math.hypot(target_x - start_x, target_y - start_y) <= 1e-4:
            return None

        heading = math.atan2(target_y - start_y, target_x - start_x)
        return heading, math.sqrt(closest_dist_sq), start_idx, target_idx

    # Selects currently active zone by nearest-distance rule with exit hysteresis.
    def _select_active_yaw_zone(self, x: float, y: float) -> YawAlignmentZone | None:
        if not self._yaw_alignment_zones:
            return None

        zones_by_id = {z.zone_id: z for z in self._yaw_alignment_zones}
        if self._active_yaw_zone_id in zones_by_id:
            active = zones_by_id[self._active_yaw_zone_id]
            active_dist = math.hypot(x - active.x, y - active.y)
            if active_dist <= (active.activation_radius_m + self.yaw_alignment_exit_margin_m):
                return active

        nearest: YawAlignmentZone | None = None
        nearest_dist = float("inf")
        for zone in self._yaw_alignment_zones:
            dist = math.hypot(x - zone.x, y - zone.y)
            if dist > zone.activation_radius_m:
                continue
            if dist < nearest_dist:
                nearest = zone
                nearest_dist = dist
        return nearest

    # HH_260619 - Path-based lanelet safety sampler for forward Nav2 driving.
    # The raw yaw rectangle is still used as fallback, but a valid local path is
    # the correct nominal corridor after route selection and lanelet snapping.
    def _sample_lanelet_safety_local_path_corridor(
        self,
        grid: OccupancyGrid,
        pose: tuple[float, float, float],
        *,
        lookahead: float,
        width: float,
        threshold: int,
        max_start_distance: float | None = None,
    ) -> tuple[bool, bool, tuple[float, float, int, str]]:
        path = self._last_route_heading_path
        if path is None or len(path.poses) < 2:
            return False, False, (pose[0], pose[1], -1, "no_path")

        grid_frame = str(grid.header.frame_id).strip()
        path_frame = str(path.header.frame_id).strip()
        if grid_frame and path_frame and grid_frame != path_frame:
            return False, False, (pose[0], pose[1], -1, "frame_mismatch")

        points: list[tuple[float, float]] = []
        for pose_stamped in path.poses:
            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)
            if math.isfinite(x) and math.isfinite(y):
                points.append((x, y))
        if len(points) < 2:
            return False, False, (pose[0], pose[1], -1, "empty_path")

        pose_x, pose_y, _ = pose
        closest_idx = 0
        closest_dist_sq = float("inf")
        for idx, (x, y) in enumerate(points):
            dist_sq = (x - pose_x) * (x - pose_x) + (y - pose_y) * (y - pose_y)
            if dist_sq < closest_dist_sq:
                closest_idx = idx
                closest_dist_sq = dist_sq

        max_start_cfg = (
            self.lanelet_safety_front_path_max_start_distance_m
            if max_start_distance is None
            else max(
                self.lanelet_safety_front_path_max_start_distance_m,
                float(max_start_distance),
            )
        )
        max_start = max(0.05, float(max_start_cfg))
        if math.sqrt(closest_dist_sq) > max_start:
            return False, False, (pose_x, pose_y, -1, "path_far")

        res = float(grid.info.resolution)
        if res <= 0.0:
            return False, False, (pose_x, pose_y, -1, "bad_resolution")
        origin_x = float(grid.info.origin.position.x)
        origin_y = float(grid.info.origin.position.y)
        grid_width = int(grid.info.width)
        grid_height = int(grid.info.height)
        stop_threshold = int(threshold)
        scan_lookahead = max(0.05, float(lookahead))
        scan_width = max(0.05, float(width))
        half_w = scan_width * 0.5
        # HH_260622 - Always test the route centerline first, then optional
        # narrow lateral offsets. The previous full-width sweep made raw
        # lanelet boundary pixels at merge/branch connections stop valid paths.
        lateral_offsets = [0.0]
        lateral_step_count = int(math.floor(half_w / res + 1e-9))
        for step in range(1, lateral_step_count + 1):
            offset = step * res
            lateral_offsets.extend((-offset, offset))

        accumulated = 0.0
        for idx in range(closest_idx, len(points) - 1):
            start_x, start_y = points[idx]
            end_x, end_y = points[idx + 1]
            seg_dx = end_x - start_x
            seg_dy = end_y - start_y
            seg_len = math.hypot(seg_dx, seg_dy)
            if seg_len <= 1e-4:
                continue
            heading = math.atan2(seg_dy, seg_dx)
            cos_y = self._cos(heading)
            sin_y = self._sin(heading)
            step_count = max(1, int(math.ceil(seg_len / res)))
            for step in range(step_count + 1):
                along = min(seg_len, step * res)
                total_along = accumulated + along
                if total_along > scan_lookahead:
                    return True, False, (pose_x, pose_y, -1, "clear")
                center_x = start_x + cos_y * along
                center_y = start_y + sin_y * along
                for lateral in lateral_offsets:
                    wx = center_x - lateral * sin_y
                    wy = center_y + lateral * cos_y
                    mx = int(round((wx - origin_x) / res))
                    my = int(round((wy - origin_y) / res))
                    if mx < 0 or my < 0 or mx >= grid_width or my >= grid_height:
                        if self.lanelet_safety_stop_on_unknown:
                            return True, True, (wx, wy, -1, "path_out_of_grid")
                        continue
                    cost = int(grid.data[my * grid_width + mx])
                    if cost >= stop_threshold:
                        return True, True, (wx, wy, cost, "path_cost")
            accumulated += seg_len
            if accumulated >= scan_lookahead:
                break

        return True, False, (pose_x, pose_y, -1, "clear")

    # HH_260702 - Path-based dynamic obstacle sampler for forward avoidance.
    # Unlike lanelet safety, out-of-grid cells are ignored here because live
    # LiDAR/Radar grids are rolling dynamic surfaces, not static map authority.
    def _sample_dynamic_local_path_corridor(
        self,
        grid: OccupancyGrid,
        pose: tuple[float, float, float],
        *,
        lookahead: float,
        width: float,
        threshold: int,
        max_start_distance: float | None = None,
    ) -> tuple[bool, bool, tuple[float, float, int, str]]:
        path = self._last_route_heading_path
        if path is None or len(path.poses) < 2:
            return False, False, (pose[0], pose[1], -1, "no_path")

        grid_frame = str(grid.header.frame_id).strip()
        path_frame = str(path.header.frame_id).strip()
        if grid_frame and path_frame and grid_frame != path_frame:
            return False, False, (pose[0], pose[1], -1, "frame_mismatch")

        points: list[tuple[float, float]] = []
        for pose_stamped in path.poses:
            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)
            if math.isfinite(x) and math.isfinite(y):
                points.append((x, y))
        if len(points) < 2:
            return False, False, (pose[0], pose[1], -1, "empty_path")

        pose_x, pose_y, _ = pose
        closest_idx = 0
        closest_dist_sq = float("inf")
        for idx, (x, y) in enumerate(points):
            dist_sq = (x - pose_x) * (x - pose_x) + (y - pose_y) * (y - pose_y)
            if dist_sq < closest_dist_sq:
                closest_idx = idx
                closest_dist_sq = dist_sq

        max_start = max(
            0.05,
            float(
                self.front_dynamic_path_max_start_distance_m
                if max_start_distance is None
                else max_start_distance
            ),
        )
        if math.sqrt(closest_dist_sq) > max_start:
            return False, False, (pose_x, pose_y, -1, "path_far")

        res = float(grid.info.resolution)
        if res <= 0.0:
            return False, False, (pose_x, pose_y, -1, "bad_resolution")
        origin_x = float(grid.info.origin.position.x)
        origin_y = float(grid.info.origin.position.y)
        grid_width = int(grid.info.width)
        grid_height = int(grid.info.height)
        stop_threshold = int(threshold)
        scan_lookahead = max(0.05, float(lookahead))
        scan_width = max(0.05, float(width))
        half_w = scan_width * 0.5
        lateral_offsets = [0.0]
        lateral_step_count = int(math.floor(half_w / res + 1e-9))
        for step in range(1, lateral_step_count + 1):
            offset = step * res
            lateral_offsets.extend((-offset, offset))

        accumulated = 0.0
        for idx in range(closest_idx, len(points) - 1):
            start_x, start_y = points[idx]
            end_x, end_y = points[idx + 1]
            seg_dx = end_x - start_x
            seg_dy = end_y - start_y
            seg_len = math.hypot(seg_dx, seg_dy)
            if seg_len <= 1e-4:
                continue
            heading = math.atan2(seg_dy, seg_dx)
            cos_y = self._cos(heading)
            sin_y = self._sin(heading)
            step_count = max(1, int(math.ceil(seg_len / res)))
            for step in range(step_count + 1):
                along = min(seg_len, step * res)
                total_along = accumulated + along
                if total_along > scan_lookahead:
                    return True, False, (pose_x, pose_y, -1, "clear")
                center_x = start_x + cos_y * along
                center_y = start_y + sin_y * along
                for lateral in lateral_offsets:
                    wx = center_x - lateral * sin_y
                    wy = center_y + lateral * cos_y
                    mx = int(round((wx - origin_x) / res))
                    my = int(round((wy - origin_y) / res))
                    if mx < 0 or my < 0 or mx >= grid_width or my >= grid_height:
                        continue
                    cost = int(grid.data[my * grid_width + mx])
                    if cost >= stop_threshold:
                        return True, True, (wx, wy, cost, "path_cost")
            accumulated += seg_len
            if accumulated >= scan_lookahead:
                break

        return True, False, (pose_x, pose_y, -1, "clear")

    # HH_260618: Strict lanelet-grid corridor sampler. Unlike the merged-grid
    # sampler, out-of-grid can be treated as blocked because leaving the
    # lanelet map is a route-safety violation, not an obstacle-grid miss.
    def _sample_lanelet_safety_corridor(
        self,
        grid: OccupancyGrid,
        pose: tuple[float, float, float],
        *,
        yaw_offset: float,
        lookahead: float,
        width: float,
        threshold: int,
    ) -> tuple[bool, tuple[float, float, int, str]]:
        pose_x, pose_y, yaw = pose
        effective_yaw = yaw + yaw_offset
        scan_lookahead = max(0.05, float(lookahead))
        scan_width = max(0.05, float(width))
        stop_threshold = int(threshold)
        res = float(grid.info.resolution)
        origin_x = float(grid.info.origin.position.x)
        origin_y = float(grid.info.origin.position.y)
        grid_width = int(grid.info.width)
        grid_height = int(grid.info.height)
        cos_y = self._cos(effective_yaw)
        sin_y = self._sin(effective_yaw)
        half_w = scan_width * 0.5

        n_x = int(math.floor(scan_lookahead / res + 1e-9)) + 1
        n_y = int(math.floor(scan_width / res + 1e-9)) + 1

        for ix in range(n_x):
            x = ix * res
            for iy in range(n_y):
                y = -half_w + iy * res
                wx = pose_x + x * cos_y - y * sin_y
                wy = pose_y + x * sin_y + y * cos_y
                mx = int(round((wx - origin_x) / res))
                my = int(round((wy - origin_y) / res))
                if mx < 0 or my < 0 or mx >= grid_width or my >= grid_height:
                    if self.lanelet_safety_stop_on_unknown:
                        return True, (wx, wy, -1, "out_of_grid")
                    continue
                cost = int(grid.data[my * grid_width + mx])
                if cost >= stop_threshold:
                    return True, (wx, wy, cost, "cost")

        return False, (pose_x, pose_y, -1, "clear")

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
    ) -> tuple[bool, int, list[tuple[int, int]], tuple[float, float, int] | None]:
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

        half_w = scan_width * 0.5
        # Use integer-counted steps (ix * res / iy * res) to avoid floating-point
        # accumulation drift from repeated addition; drift can cause systematic
        # cell misses that break 4-connectivity in the BFS cluster check.
        n_x = int(math.floor(scan_lookahead / res + 1e-9)) + 1
        n_y = int(math.floor(scan_width / res + 1e-9)) + 1
        total_cells = 0
        lethal_cells: list[tuple[int, int]] = []

        for ix in range(n_x):
            x = ix * res
            for iy in range(n_y):
                y = -half_w + iy * res
                wx = pose_x + x * cos_y - y * sin_y
                wy = pose_y + x * sin_y + y * cos_y
                mx = int(round((wx - origin_x) / res))
                my = int(round((wy - origin_y) / res))
                if 0 <= mx < w and 0 <= my < h:
                    idx = my * w + mx
                    cost = int(grid.data[idx])
                    total_cells += 1
                    if cost >= stop_threshold:
                        # HH_260630: Return the actual occupied cell center so
                        # source-grid attribution samples the same cell instead
                        # of a neighboring floor/round boundary case.
                        cell_wx = origin_x + (mx + 0.5) * res
                        cell_wy = origin_y + (my + 0.5) * res
                        return True, total_cells, lethal_cells, (cell_wx, cell_wy, cost)
                    if cost >= self.unavoidable_lethal_threshold:
                        lethal_cells.append((mx, my))

        return False, total_cells, lethal_cells, None

    # HH_260618: Samples original source grids at a blocked merged-grid cell.
    def _format_cost_source_debug(
        self, blocked_detail: tuple[float, float, int] | None
    ) -> str:
        if blocked_detail is None:
            return "merged:unknown"
        wx, wy, merged_cost = blocked_detail
        if not self.cost_source_debug_enable:
            return f"merged:{merged_cost}@({wx:.2f},{wy:.2f})"

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        best_label = "source"
        best_cost = -1
        source_parts = []
        for label, grid in self._cost_source_grids.items():
            recv_sec = self._cost_source_recv_sec.get(label, 0.0)
            if self.cost_source_debug_max_age_s > 0.0:
                if (now_sec - recv_sec) > self.cost_source_debug_max_age_s:
                    continue
            cost = self._sample_grid_cost(grid, wx, wy)
            source_parts.append(f"{label}:{cost}")
            if cost > best_cost:
                best_label = label
                best_cost = cost

        if source_parts:
            return (
                f"{best_label}:{best_cost}@({wx:.2f},{wy:.2f}) "
                f"merged={merged_cost} sources=[{', '.join(source_parts)}]"
            )
        return f"merged:{merged_cost}@({wx:.2f},{wy:.2f}) sources=unavailable"

    # HH_260618: Samples one occupancy grid and reports whether the world point
    # is inside the grid. Raw lanelet safety uses this to distinguish free from
    # unknown/outside-map space.
    def _sample_grid_cost_detail(
        self, grid: OccupancyGrid, wx: float, wy: float
    ) -> tuple[bool, int]:
        res = float(grid.info.resolution)
        if res <= 0.0:
            return False, -1
        origin_x = float(grid.info.origin.position.x)
        origin_y = float(grid.info.origin.position.y)
        mx = int((wx - origin_x) / res)
        my = int((wy - origin_y) / res)
        width = int(grid.info.width)
        height = int(grid.info.height)
        if mx < 0 or my < 0 or mx >= width or my >= height:
            return False, -1
        return True, int(grid.data[my * width + mx])

    # HH_260618: Samples one occupancy grid at map/world position.
    def _sample_grid_cost(self, grid: OccupancyGrid, wx: float, wy: float) -> int:
        res = float(grid.info.resolution)
        if res <= 0.0:
            return -1
        origin_x = float(grid.info.origin.position.x)
        origin_y = float(grid.info.origin.position.y)
        mx = int((wx - origin_x) / res)
        my = int((wy - origin_y) / res)
        width = int(grid.info.width)
        height = int(grid.info.height)
        if mx < 0 or my < 0 or mx >= width or my >= height:
            return -1
        return int(grid.data[my * width + mx])

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

        source_candidates["odometry"] = self._resolve_odometry_candidates(target_frame)
        source_candidates["pose_topic"] = self._resolve_pose_topic_candidates(target_frame)

        preference = self.pose_source_preference.strip().lower()
        # Avoid unnecessary TF lookup warnings when odom/pose candidates are already available.
        need_tf_lookup = (
            preference == "tf_robot_base"
            or (
                not source_candidates["odometry"]
                and not source_candidates["pose_topic"]
            )
        )
        if need_tf_lookup:
            tf_pose = self._lookup_tf_pose(target_frame, self.robot_base_frame)
            if tf_pose is not None:
                source_candidates["tf_robot_base"].append(("tf_robot_base", tf_pose))

        ordered_sources = {
            "tf_robot_base": ["tf_robot_base", "odometry", "pose_topic"],
            "odometry": ["odometry", "tf_robot_base", "pose_topic"],
            "pose_topic": ["pose_topic", "tf_robot_base", "odometry"],
        }.get(preference, ["odometry", "tf_robot_base", "pose_topic"])

        # HH_260617: Treat pose_source_preference as an exclusive priority list, not a union.
        # A valid lanelet-snapped pose must not be vetoed by raw odometry/TF that can sit on lane-boundary cost.
        for source in ordered_sources:
            candidates = source_candidates.get(source, [])
            if candidates:
                return candidates
        return []

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

    # HH_260507: Apply speed_scale to linear and angular components.
    def _scale_twist(self, msg: Twist) -> Twist:
        if self.speed_scale == 1.0:
            return msg
        scaled = Twist()
        scaled.linear.x = msg.linear.x * self.speed_scale
        scaled.linear.y = msg.linear.y * self.speed_scale
        scaled.linear.z = msg.linear.z * self.speed_scale
        scaled.angular.z = msg.angular.z * self.speed_scale
        return scaled

    def _clamp(self, value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
