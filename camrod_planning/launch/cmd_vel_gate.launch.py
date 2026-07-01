import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launches cmd_vel gate with optional cost-based stop parameters.
    default_yaw_zone_file = os.path.join(
        get_package_share_directory("camrod_planning"),
        "config",
        "yaw_alignment_zones.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument('module_namespace', default_value='planning'),
        DeclareLaunchArgument('cmd_vel_gate_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_raw_topic', default_value='/planning/cmd_vel_raw'),
        DeclareLaunchArgument('cmd_vel_output_topic', default_value='/planning/cmd_vel'),
        DeclareLaunchArgument('planning_engage_topic', default_value='/planning/engage'),
        # HH_260623 - UI mission engage is independent from manual 2D-goal engage.
        DeclareLaunchArgument('planning_mission_engage_topic', default_value='/planning/mission_engage'),
        DeclareLaunchArgument('planning_engaged_state_topic', default_value='/planning/engaged'),
        # HH_260522: unified source selector for planning e-stop input.
        #   platform_status/topic/enabled/on: subscribe /platform/status/estop
        #   disabled/off/none: ignore e-stop topic
        DeclareLaunchArgument('cmd_vel_gate_estop_source_mode', default_value='platform_status'),
        # HH_260409: Use platform-originated e-stop by default.
        DeclareLaunchArgument('cmd_vel_gate_estop_topic', default_value='/platform/status/estop'),
        # HH_260522: unified source selector for DR-timeout trigger.
        #   localization_monitor/topic/enabled/on: subscribe /localization/state/dr_timeout
        #   disabled/off/none: ignore DR-timeout trigger
        DeclareLaunchArgument('cmd_vel_gate_dr_timeout_source_mode', default_value='localization_monitor'),
        DeclareLaunchArgument('cmd_vel_gate_allow_on_start', default_value='false'),
        # HH_260427: Short hold window after DR_ONLY->NORMAL localization recovery.
        DeclareLaunchArgument('cmd_vel_gate_enable_gnss_recovery_hold', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_localization_mode_topic', default_value='/localization/mode'),
        DeclareLaunchArgument('cmd_vel_gate_gnss_recovery_hold_s', default_value='2.0'),
        DeclareLaunchArgument('cmd_vel_gate_gnss_recovery_min_source_s', default_value='0.5'),
        DeclareLaunchArgument('cmd_vel_gate_gnss_recovery_hold_cooldown_s', default_value='5.0'),
        DeclareLaunchArgument('cmd_vel_gate_gnss_recovery_source_mode_min', default_value='2'),
        DeclareLaunchArgument('cmd_vel_gate_gnss_recovery_target_mode', default_value='0'),
        # HH_260413: Optional cost-based stop in front of the platform.
        DeclareLaunchArgument('cmd_vel_gate_cost_stop_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_cost_grid_topic', default_value='/planning/cost_grid/inflation'),
        # HH_260618: Safety gates must use the real robot pose, not lanelet-snapped pose.
        # Snapped pose can stay on a lane while the robot is physically outside it.
        DeclareLaunchArgument('cmd_vel_gate_cost_pose_topic', default_value='/localization/pose'),
        # HH_260426: VIO stack is disabled; use localization fallback odometry.
        DeclareLaunchArgument('cmd_vel_gate_cost_odometry_topic', default_value='/localization/fallback/odometry'),
        # HH_260522: pose source preference options:
        #   odometry | tf_robot_base | pose_topic
        # HH_260618: Prefer the configured raw localization pose for safety checks.
        DeclareLaunchArgument('cmd_vel_gate_pose_source_preference', default_value='pose_topic'),
        DeclareLaunchArgument('cmd_vel_gate_enable_pose_raw_fallback', default_value='false'),
        DeclareLaunchArgument('cmd_vel_gate_cost_threshold', default_value='85'),
        DeclareLaunchArgument('cmd_vel_gate_cost_lookahead_m', default_value='2.0'),
        # HH_260623 - Measured body width plus 0.10 m planning margin per side.
        DeclareLaunchArgument('cmd_vel_gate_cost_width_m', default_value='1.27'),
        DeclareLaunchArgument('cmd_vel_gate_cost_hold_s', default_value='1.0'),
        # HH_260622: Merged inflation cost is only a stop source when one of
        # these dynamic source grids owns the high-cost cell.
        DeclareLaunchArgument('cmd_vel_gate_cost_stop_require_dynamic_source', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_cost_stop_dynamic_source_labels', default_value='lidar,radar'),
        # HH_260618: Raw lanelet grid hard-stop before inflation ego-clear.
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_grid_topic', default_value='/map/cost_grid/lanelet'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_threshold', default_value='85'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_current_threshold', default_value='85'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_lookahead_m', default_value='1.0'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_width_m', default_value='0.8'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_stop_on_unknown', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_allow_rotation_in_place', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_check_reverse', default_value='false'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_check_lateral', default_value='false'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_min_translation_mps', default_value='0.02'),
        # HH_260619 - Prefer active local-path corridor for forward lanelet safety.
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_front_use_local_path', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_front_path_max_start_distance_m', default_value='1.5'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_front_path_width_m', default_value='0.25'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_front_path_allow_route_reentry', default_value='true'),
        # HH_260622: Allow bounded re-entry when a manually placed/sim pose is
        # outside lanelet but the selected local path is close and valid.
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_current_allow_route_reentry', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_current_route_reentry_max_distance_m', default_value='4.0'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_current_route_reentry_require_front_cmd', default_value='true'),
        # HH_260624 - Drop-zone exit is an explicit parking phase that may
        # cross static lanelet/drop-zone cost before Nav2 campsite routing.
        DeclareLaunchArgument('cmd_vel_gate_parking_drop_zone_status_topic', default_value='/parking/drop_zone/status'),
        DeclareLaunchArgument('cmd_vel_gate_parking_drop_zone_static_bypass_phases', default_value='EXIT_STRAIGHT,ALIGN_EXIT_YAW'),
        # HH_260701 - Campsite maneuver phases may cross static lanelet cost,
        # while live LiDAR/Radar source costs stay blocking.
        DeclareLaunchArgument('cmd_vel_gate_parking_site_status_topic', default_value='/parking/site_maneuver/status'),
        DeclareLaunchArgument(
            'cmd_vel_gate_parking_site_static_bypass_phases',
            default_value='ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,ALIGN_RETURN_YAW,REVERSE_OUT,CRAB_OUT',
        ),
        # HH_260422: Speed-dependent front lookahead parameters.
        DeclareLaunchArgument('cmd_vel_gate_speed_dependent_lookahead', default_value='true'),
        # HH_260630 - Minimum front scan reaches the front radar mount plus about 1m clearance.
        DeclareLaunchArgument('cmd_vel_gate_front_lookahead_min_m', default_value='2.10'),
        DeclareLaunchArgument('cmd_vel_gate_front_lookahead_max_m', default_value='3.0'),
        DeclareLaunchArgument('cmd_vel_gate_front_lookahead_friction', default_value='0.4'),
        DeclareLaunchArgument('cmd_vel_gate_front_reaction_time_s', default_value='0.15'),
        DeclareLaunchArgument('cmd_vel_gate_front_lookahead_margin_m', default_value='0.3'),
        # HH_260622: Side/rear cost-stop samples the merged grid, but blocks
        # only when dynamic source attribution owns the high-cost cell.
        DeclareLaunchArgument('cmd_vel_gate_side_rear_cost_stop', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_side_cost_threshold', default_value='85'),
        DeclareLaunchArgument('cmd_vel_gate_side_lookahead_m', default_value='1.2'),
        # HH_260623 - Side scan width covers full body length plus front/rear margins.
        DeclareLaunchArgument('cmd_vel_gate_side_corridor_width_m', default_value='1.69160'),
        DeclareLaunchArgument('cmd_vel_gate_rear_cost_threshold', default_value='85'),
        DeclareLaunchArgument('cmd_vel_gate_rear_lookahead_m', default_value='1.2'),
        # HH_260623 - Rear scan width covers full body width plus left/right margins.
        DeclareLaunchArgument('cmd_vel_gate_rear_corridor_width_m', default_value='1.27'),
        # HH_260618: Allow explicit parking/site crab to cross static
        # lanelet/global-path front/side/rear cost while keeping LiDAR/Radar stops active.
        DeclareLaunchArgument('cmd_vel_gate_lateral_cmd_bypass_static_cost_stop', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_lateral_cmd_bypass_min_mps', default_value='0.02'),
        DeclareLaunchArgument('cmd_vel_gate_reverse_cmd_bypass_static_cost_stop', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_reverse_cmd_bypass_min_mps', default_value='0.02'),
        DeclareLaunchArgument('cmd_vel_gate_lateral_cmd_dynamic_obstacle_threshold', default_value='85'),
        # HH_260624 - Pure in-place parking rotation bypasses only static
        # lanelet cost; live LiDAR/Radar disk cost still blocks rotation.
        DeclareLaunchArgument('cmd_vel_gate_rotation_cmd_dynamic_obstacle_stop', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_rotation_cmd_dynamic_obstacle_radius_m', default_value='1.5'),
        DeclareLaunchArgument('cmd_vel_gate_rotation_cmd_dynamic_obstacle_threshold', default_value='85'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_stop_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_lethal_threshold', default_value='90'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_cluster_min_cells', default_value='25'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_cluster_min_ratio', default_value='0.25'),
        # Optional yaw-alignment zone gate (map-keypoint based).
        DeclareLaunchArgument('cmd_vel_gate_yaw_alignment_enable', default_value='false'),
        DeclareLaunchArgument('cmd_vel_gate_yaw_alignment_frame_id', default_value='map'),
        DeclareLaunchArgument('cmd_vel_gate_yaw_alignment_zones_file', default_value=default_yaw_zone_file),
        DeclareLaunchArgument('cmd_vel_gate_yaw_alignment_exit_margin_m', default_value='0.3'),
        # HH_260618: Route-heading guard prevents forward motion while the
        # robot is facing opposite to the active local path tangent.
        DeclareLaunchArgument('cmd_vel_gate_route_heading_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_path_topic', default_value='/planning/local_path'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_frame_id', default_value='map'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_min_cmd_x_mps', default_value='0.03'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_lateral_cmd_epsilon_mps', default_value='0.02'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_lookahead_m', default_value='1.5'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_error_enter_deg', default_value='75.0'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_error_exit_deg', default_value='20.0'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_angular_kp', default_value='1.4'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_max_angular_z', default_value='0.6'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_max_linear_x', default_value='0.0'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_min_path_points', default_value='2'),
        # HH_260507: Speed scale for cmd_vel output (applied in planning_cmd_vel_gate_node).
        DeclareLaunchArgument('cmd_vel_gate_speed_scale', default_value='1.0'),
        # HH_260626: Stop downstream motion if Nav2 raw cmd_vel stops publishing.
        DeclareLaunchArgument('cmd_vel_gate_input_timeout_s', default_value='0.35'),
        DeclareLaunchArgument('cmd_vel_gate_zero_publish_rate_hz', default_value='10.0'),

        Node(
            package='camrod_planning',
            executable='planning_cmd_vel_gate_node.py',
            name='cmd_vel_gate',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('cmd_vel_raw_topic'),
                'output_topic': LaunchConfiguration('cmd_vel_output_topic'),
                'engage_topic': LaunchConfiguration('planning_engage_topic'),
                'mission_engage_topic': LaunchConfiguration('planning_mission_engage_topic'),
                'state_topic': LaunchConfiguration('planning_engaged_state_topic'),
                'estop_source_mode': LaunchConfiguration('cmd_vel_gate_estop_source_mode'),
                'estop_topic': LaunchConfiguration('cmd_vel_gate_estop_topic'),
                'dr_timeout_source_mode': LaunchConfiguration('cmd_vel_gate_dr_timeout_source_mode'),
                'allow_on_start': LaunchConfiguration('cmd_vel_gate_allow_on_start'),
                'publish_zero_when_blocked': True,
                'enable_gnss_recovery_hold': LaunchConfiguration('cmd_vel_gate_enable_gnss_recovery_hold'),
                'localization_mode_topic': LaunchConfiguration('cmd_vel_gate_localization_mode_topic'),
                'gnss_recovery_hold_s': LaunchConfiguration('cmd_vel_gate_gnss_recovery_hold_s'),
                'gnss_recovery_min_source_s': LaunchConfiguration('cmd_vel_gate_gnss_recovery_min_source_s'),
                'gnss_recovery_hold_cooldown_s': LaunchConfiguration('cmd_vel_gate_gnss_recovery_hold_cooldown_s'),
                'gnss_recovery_source_mode_min': LaunchConfiguration('cmd_vel_gate_gnss_recovery_source_mode_min'),
                'gnss_recovery_target_mode': LaunchConfiguration('cmd_vel_gate_gnss_recovery_target_mode'),
                'enable_cost_stop': LaunchConfiguration('cmd_vel_gate_cost_stop_enable'),
                'cost_grid_topic': LaunchConfiguration('cmd_vel_gate_cost_grid_topic'),
                'pose_topic': LaunchConfiguration('cmd_vel_gate_cost_pose_topic'),
                'odometry_topic': LaunchConfiguration('cmd_vel_gate_cost_odometry_topic'),
                'pose_source_preference': LaunchConfiguration('cmd_vel_gate_pose_source_preference'),
                'enable_pose_raw_fallback': LaunchConfiguration('cmd_vel_gate_enable_pose_raw_fallback'),
                'cost_stop_threshold': LaunchConfiguration('cmd_vel_gate_cost_threshold'),
                'cost_stop_lookahead_m': LaunchConfiguration('cmd_vel_gate_cost_lookahead_m'),
                'cost_stop_width_m': LaunchConfiguration('cmd_vel_gate_cost_width_m'),
                'cost_stop_hold_s': LaunchConfiguration('cmd_vel_gate_cost_hold_s'),
                'cost_stop_require_dynamic_source': LaunchConfiguration('cmd_vel_gate_cost_stop_require_dynamic_source'),
                'cost_stop_dynamic_source_labels': LaunchConfiguration('cmd_vel_gate_cost_stop_dynamic_source_labels'),
                'lanelet_safety_enable': LaunchConfiguration('cmd_vel_gate_lanelet_safety_enable'),
                'lanelet_safety_grid_topic': LaunchConfiguration('cmd_vel_gate_lanelet_safety_grid_topic'),
                'lanelet_safety_threshold': LaunchConfiguration('cmd_vel_gate_lanelet_safety_threshold'),
                'lanelet_safety_current_threshold': LaunchConfiguration('cmd_vel_gate_lanelet_safety_current_threshold'),
                'lanelet_safety_lookahead_m': LaunchConfiguration('cmd_vel_gate_lanelet_safety_lookahead_m'),
                'lanelet_safety_width_m': LaunchConfiguration('cmd_vel_gate_lanelet_safety_width_m'),
                'lanelet_safety_stop_on_unknown': LaunchConfiguration('cmd_vel_gate_lanelet_safety_stop_on_unknown'),
                'lanelet_safety_allow_rotation_in_place': LaunchConfiguration('cmd_vel_gate_lanelet_safety_allow_rotation_in_place'),
                'lanelet_safety_check_reverse': LaunchConfiguration('cmd_vel_gate_lanelet_safety_check_reverse'),
                'lanelet_safety_check_lateral': LaunchConfiguration('cmd_vel_gate_lanelet_safety_check_lateral'),
                'lanelet_safety_min_translation_mps': LaunchConfiguration('cmd_vel_gate_lanelet_safety_min_translation_mps'),
                'lanelet_safety_front_use_local_path': LaunchConfiguration('cmd_vel_gate_lanelet_safety_front_use_local_path'),
                'lanelet_safety_front_path_max_start_distance_m': LaunchConfiguration('cmd_vel_gate_lanelet_safety_front_path_max_start_distance_m'),
                'lanelet_safety_front_path_width_m': LaunchConfiguration('cmd_vel_gate_lanelet_safety_front_path_width_m'),
                'lanelet_safety_front_path_allow_route_reentry': LaunchConfiguration('cmd_vel_gate_lanelet_safety_front_path_allow_route_reentry'),
                'lanelet_safety_current_allow_route_reentry': LaunchConfiguration('cmd_vel_gate_lanelet_safety_current_allow_route_reentry'),
                'lanelet_safety_current_route_reentry_max_distance_m': LaunchConfiguration('cmd_vel_gate_lanelet_safety_current_route_reentry_max_distance_m'),
                'lanelet_safety_current_route_reentry_require_front_cmd': LaunchConfiguration('cmd_vel_gate_lanelet_safety_current_route_reentry_require_front_cmd'),
                'parking_drop_zone_status_topic': LaunchConfiguration('cmd_vel_gate_parking_drop_zone_status_topic'),
                'parking_drop_zone_static_bypass_phases': LaunchConfiguration('cmd_vel_gate_parking_drop_zone_static_bypass_phases'),
                'parking_site_status_topic': LaunchConfiguration('cmd_vel_gate_parking_site_status_topic'),
                'parking_site_static_bypass_phases': LaunchConfiguration('cmd_vel_gate_parking_site_static_bypass_phases'),
                'enable_speed_dependent_lookahead': LaunchConfiguration('cmd_vel_gate_speed_dependent_lookahead'),
                'front_lookahead_min_m': LaunchConfiguration('cmd_vel_gate_front_lookahead_min_m'),
                'front_lookahead_max_m': LaunchConfiguration('cmd_vel_gate_front_lookahead_max_m'),
                'front_lookahead_friction': LaunchConfiguration('cmd_vel_gate_front_lookahead_friction'),
                'front_reaction_time_s': LaunchConfiguration('cmd_vel_gate_front_reaction_time_s'),
                'front_lookahead_margin_m': LaunchConfiguration('cmd_vel_gate_front_lookahead_margin_m'),
                'enable_side_rear_cost_stop': LaunchConfiguration('cmd_vel_gate_side_rear_cost_stop'),
                'side_cost_threshold': LaunchConfiguration('cmd_vel_gate_side_cost_threshold'),
                'side_lookahead_m': LaunchConfiguration('cmd_vel_gate_side_lookahead_m'),
                'side_corridor_width_m': LaunchConfiguration('cmd_vel_gate_side_corridor_width_m'),
                'rear_cost_threshold': LaunchConfiguration('cmd_vel_gate_rear_cost_threshold'),
                'rear_lookahead_m': LaunchConfiguration('cmd_vel_gate_rear_lookahead_m'),
                'rear_corridor_width_m': LaunchConfiguration('cmd_vel_gate_rear_corridor_width_m'),
                'lateral_cmd_bypass_static_cost_stop': LaunchConfiguration('cmd_vel_gate_lateral_cmd_bypass_static_cost_stop'),
                'lateral_cmd_bypass_min_mps': LaunchConfiguration('cmd_vel_gate_lateral_cmd_bypass_min_mps'),
                'reverse_cmd_bypass_static_cost_stop': LaunchConfiguration('cmd_vel_gate_reverse_cmd_bypass_static_cost_stop'),
                'reverse_cmd_bypass_min_mps': LaunchConfiguration('cmd_vel_gate_reverse_cmd_bypass_min_mps'),
                'lateral_cmd_dynamic_obstacle_threshold': LaunchConfiguration('cmd_vel_gate_lateral_cmd_dynamic_obstacle_threshold'),
                'rotation_cmd_dynamic_obstacle_stop': LaunchConfiguration('cmd_vel_gate_rotation_cmd_dynamic_obstacle_stop'),
                'rotation_cmd_dynamic_obstacle_radius_m': LaunchConfiguration('cmd_vel_gate_rotation_cmd_dynamic_obstacle_radius_m'),
                'rotation_cmd_dynamic_obstacle_threshold': LaunchConfiguration('cmd_vel_gate_rotation_cmd_dynamic_obstacle_threshold'),
                'enable_unavoidable_stop': LaunchConfiguration('cmd_vel_gate_unavoidable_stop_enable'),
                'unavoidable_lethal_threshold': LaunchConfiguration('cmd_vel_gate_unavoidable_lethal_threshold'),
                'unavoidable_cluster_min_cells': LaunchConfiguration('cmd_vel_gate_unavoidable_cluster_min_cells'),
                'unavoidable_cluster_min_ratio': LaunchConfiguration('cmd_vel_gate_unavoidable_cluster_min_ratio'),
                'enable_yaw_alignment_zone': LaunchConfiguration('cmd_vel_gate_yaw_alignment_enable'),
                'yaw_alignment_frame_id': LaunchConfiguration('cmd_vel_gate_yaw_alignment_frame_id'),
                'yaw_alignment_zones_file': LaunchConfiguration('cmd_vel_gate_yaw_alignment_zones_file'),
                'yaw_alignment_exit_margin_m': LaunchConfiguration('cmd_vel_gate_yaw_alignment_exit_margin_m'),
                'enable_route_heading_alignment': LaunchConfiguration('cmd_vel_gate_route_heading_enable'),
                'route_heading_path_topic': LaunchConfiguration('cmd_vel_gate_route_heading_path_topic'),
                'route_heading_frame_id': LaunchConfiguration('cmd_vel_gate_route_heading_frame_id'),
                'route_heading_min_cmd_x_mps': LaunchConfiguration('cmd_vel_gate_route_heading_min_cmd_x_mps'),
                'route_heading_lateral_cmd_epsilon_mps': LaunchConfiguration('cmd_vel_gate_route_heading_lateral_cmd_epsilon_mps'),
                'route_heading_lookahead_m': LaunchConfiguration('cmd_vel_gate_route_heading_lookahead_m'),
                'route_heading_error_enter_deg': LaunchConfiguration('cmd_vel_gate_route_heading_error_enter_deg'),
                'route_heading_error_exit_deg': LaunchConfiguration('cmd_vel_gate_route_heading_error_exit_deg'),
                'route_heading_angular_kp': LaunchConfiguration('cmd_vel_gate_route_heading_angular_kp'),
                'route_heading_max_angular_z': LaunchConfiguration('cmd_vel_gate_route_heading_max_angular_z'),
                'route_heading_max_linear_x': LaunchConfiguration('cmd_vel_gate_route_heading_max_linear_x'),
                'route_heading_min_path_points': LaunchConfiguration('cmd_vel_gate_route_heading_min_path_points'),
                'speed_scale': LaunchConfiguration('cmd_vel_gate_speed_scale'),
                'input_timeout_s': LaunchConfiguration('cmd_vel_gate_input_timeout_s'),
                'zero_publish_rate_hz': LaunchConfiguration('cmd_vel_gate_zero_publish_rate_hz'),
            }],
            condition=IfCondition(LaunchConfiguration('cmd_vel_gate_enable')),
        ),
    ])
