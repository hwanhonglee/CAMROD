"""Opt-in Woraksan mission tuning layered over the develop-parity CARLA launch.

The default ``camrod_carla_full.launch.py`` intentionally preserves CAMROD
develop control, planning, perception, and UI defaults.  This wrapper records
the plant- and map-specific values used by the historical B1/B12 rendered runs
without allowing them to become the default CARLA contract.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    adapter_share = get_package_share_directory("camrod_carla_adapter")
    planning_share = get_package_share_directory("camrod_planning")

    full_launch = os.path.join(
        adapter_share, "launch", "camrod_carla_full.launch.py"
    )
    tuned_input_adapter = os.path.join(
        adapter_share, "config", "camrod_input_adapter_carla.yaml"
    )
    tuned_lanelet_map = os.path.join(
        adapter_share, "config", "woraksan_carla_lanelet2.osm"
    )
    tuned_lidar_cost_grid = os.path.join(
        adapter_share, "config", "carla_lidar_cost_grid.yaml"
    )
    tuned_nav2 = os.path.join(
        adapter_share, "config", "nav2_carla_reverse_return.yaml"
    )
    tuned_parking = os.path.join(
        adapter_share, "config", "parking_carla.yaml"
    )
    tuned_nav_to_pose = os.path.join(
        adapter_share, "config", "navigate_to_pose_carla.xml"
    )
    tuned_nav_through_poses = os.path.join(
        planning_share,
        "config",
        "bt",
        "navigate_through_poses_w_planner_selector.xml",
    )

    tuned_arguments = {
        "launch_charging_contact_emulator": "true",
        "recovery_breakaway_enable": "true",
        "camrod_map_path": tuned_lanelet_map,
        "operator_telemetry_tf_transform_enabled": "true",
        "operator_telemetry_tf_latest_fallback_tolerance_s": "0.075",
        "operator_telemetry_camera_raw_fallback_enabled": "false",
        "operator_telemetry_docking_rear_camera_fallback_enabled": "true",
        "return_site_exit_rearm_enabled": "true",
        "use_sim_planning_profile": "true",
        "use_sim_localization_profile": "true",
        "use_sim_parking_method": "true",
        "manual_drive_linear_limit_mps": "1.40",
        "manual_drive_lateral_limit_mps": "1.00",
        "manual_drive_angular_limit_radps": "0.7853",
        "manual_drive_deadman_timeout_s": "0.75",
        "carla_cmd_vel_gate_speed_scale": "1.0",
        "carla_navigation_minimum_ackermann_turn_radius_m": "0.82",
        "carla_cost_stop_threshold": "100",
        "carla_lanelet_safety_threshold": "100",
        "carla_lanelet_safety_current_threshold": "100",
        "carla_lanelet_safety_check_reverse": "true",
        "carla_roadside_reverse_return_enable": "true",
        "carla_roadside_reverse_handoff_distance_m": "0.10",
        "carla_crab_approach_slowdown_distance_m": "1.0",
        "carla_crab_approach_min_speed_mps": "0.12",
        "carla_rotate_180_timeout_s": "60.0",
        "carla_entry_position_tolerance_m": "0.05",
        "carla_rotate_entry_max_position_error_m": "0.05",
        "carla_rotate_entry_centering_max_initial_error_m": "0.50",
        # Follow develop's live-lanelet/current-pose return handoff instead of
        # recovering the historical inbound snap before campsite entry.
        "carla_entry_anchor_centering_max_initial_error_m": "0.0",
        "carla_entry_anchor_centering_max_speed_mps": "0.12",
        "carla_entry_anchor_centering_timeout_s": "15",
        "carla_entry_anchor_centering_tolerance_m": "0.05",
        "carla_crab_entry_max_heading_drift_deg": "5.0",
        "carla_crab_entry_max_cross_track_error_m": "0.10",
        "carla_crab_entry_body_yaw_compensation_deg": "2.0",
        "carla_crab_entry_body_yaw_alignment_tolerance_deg": "0.5",
        "carla_crab_entry_body_yaw_alignment_timeout_s": "15",
        "carla_nav2_reverse_controller": "RPPReverse",
        "carla_reverse_goal_topic": "/planning/auto_reverse_goal_raw",
        "carla_goal_snapper_pose_jump_check_topic": "/localization/pose",
        "carla_route_safety_path_relative_recovery_enable": "true",
        "carla_route_safety_zero_hold_pauses_limits": "true",
        "carla_route_safety_allow_corrective_yaw_beyond_limit": "true",
        "carla_goal_reissue_while_nav_active": "true",
        "carla_nav2_reverse_return_param_file": tuned_nav2,
        "carla_parking_runtime_override_param_file": tuned_parking,
        "camrod_input_adapter_config": tuned_input_adapter,
        "carla_lidar_cost_grid_param_file": tuned_lidar_cost_grid,
        "nav2_bt_xml_nav_to_pose": tuned_nav_to_pose,
        "nav2_bt_xml_nav_through_poses": tuned_nav_through_poses,
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_launch),
            launch_arguments=tuned_arguments.items(),
        )
    ])


if __name__ == "__main__":
    generate_launch_description()
