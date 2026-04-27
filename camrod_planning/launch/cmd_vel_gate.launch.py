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
        DeclareLaunchArgument('planning_engaged_state_topic', default_value='/planning/engaged'),
        DeclareLaunchArgument('cmd_vel_gate_use_estop_topic', default_value='true'),
        # HH_260409: Use platform-originated e-stop by default.
        DeclareLaunchArgument('cmd_vel_gate_estop_topic', default_value='/platform/status/estop'),
        DeclareLaunchArgument('cmd_vel_gate_allow_on_start', default_value='false'),
        # HH_260427: Short hold window after DR_ONLY->NORMAL localization recovery.
        DeclareLaunchArgument('cmd_vel_gate_enable_gnss_recovery_hold', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_localization_mode_topic', default_value='/localization/mode'),
        DeclareLaunchArgument('cmd_vel_gate_gnss_recovery_hold_s', default_value='2.0'),
        DeclareLaunchArgument('cmd_vel_gate_gnss_recovery_source_mode_min', default_value='2'),
        DeclareLaunchArgument('cmd_vel_gate_gnss_recovery_target_mode', default_value='0'),
        # HH_260413: Optional cost-based stop in front of the platform.
        DeclareLaunchArgument('cmd_vel_gate_cost_stop_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_cost_grid_topic', default_value='/planning/cost_grid/inflation'),
        DeclareLaunchArgument('cmd_vel_gate_cost_pose_topic', default_value='/localization/pose'),
        # HH_260426: VIO stack is disabled; use localization fallback odometry.
        DeclareLaunchArgument('cmd_vel_gate_cost_odometry_topic', default_value='/localization/fallback/odometry'),
        # HH_260425: Keep odometry as default pose source for cost-stop corridor heading.
        DeclareLaunchArgument('cmd_vel_gate_pose_source_preference', default_value='odometry'),
        DeclareLaunchArgument('cmd_vel_gate_enable_pose_raw_fallback', default_value='false'),
        DeclareLaunchArgument('cmd_vel_gate_cost_threshold', default_value='85'),
        DeclareLaunchArgument('cmd_vel_gate_cost_lookahead_m', default_value='2.0'),
        DeclareLaunchArgument('cmd_vel_gate_cost_width_m', default_value='1.0'),
        DeclareLaunchArgument('cmd_vel_gate_cost_hold_s', default_value='1.0'),
        # HH_260422: Speed-dependent front lookahead parameters.
        DeclareLaunchArgument('cmd_vel_gate_speed_dependent_lookahead', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_front_lookahead_min_m', default_value='0.4'),
        DeclareLaunchArgument('cmd_vel_gate_front_lookahead_max_m', default_value='3.0'),
        DeclareLaunchArgument('cmd_vel_gate_front_lookahead_friction', default_value='0.4'),
        DeclareLaunchArgument('cmd_vel_gate_front_reaction_time_s', default_value='0.15'),
        DeclareLaunchArgument('cmd_vel_gate_front_lookahead_margin_m', default_value='0.3'),
        # HH_260422: Side/rear cost-stop — uses same merged grid as front.
        DeclareLaunchArgument('cmd_vel_gate_side_rear_cost_stop', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_side_cost_threshold', default_value='92'),
        DeclareLaunchArgument('cmd_vel_gate_side_lookahead_m', default_value='0.8'),
        DeclareLaunchArgument('cmd_vel_gate_side_corridor_width_m', default_value='0.45'),
        DeclareLaunchArgument('cmd_vel_gate_rear_cost_threshold', default_value='92'),
        DeclareLaunchArgument('cmd_vel_gate_rear_lookahead_m', default_value='0.6'),
        DeclareLaunchArgument('cmd_vel_gate_rear_corridor_width_m', default_value='0.6'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_stop_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_lethal_threshold', default_value='90'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_cluster_min_cells', default_value='25'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_cluster_min_ratio', default_value='0.25'),
        # Optional yaw-alignment zone gate (map-keypoint based).
        DeclareLaunchArgument('cmd_vel_gate_yaw_alignment_enable', default_value='false'),
        DeclareLaunchArgument('cmd_vel_gate_yaw_alignment_frame_id', default_value='map'),
        DeclareLaunchArgument('cmd_vel_gate_yaw_alignment_zones_file', default_value=default_yaw_zone_file),
        DeclareLaunchArgument('cmd_vel_gate_yaw_alignment_exit_margin_m', default_value='0.3'),

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
                'state_topic': LaunchConfiguration('planning_engaged_state_topic'),
                'use_estop_topic': LaunchConfiguration('cmd_vel_gate_use_estop_topic'),
                'estop_topic': LaunchConfiguration('cmd_vel_gate_estop_topic'),
                'allow_on_start': LaunchConfiguration('cmd_vel_gate_allow_on_start'),
                'publish_zero_when_blocked': True,
                'enable_gnss_recovery_hold': LaunchConfiguration('cmd_vel_gate_enable_gnss_recovery_hold'),
                'localization_mode_topic': LaunchConfiguration('cmd_vel_gate_localization_mode_topic'),
                'gnss_recovery_hold_s': LaunchConfiguration('cmd_vel_gate_gnss_recovery_hold_s'),
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
                'enable_unavoidable_stop': LaunchConfiguration('cmd_vel_gate_unavoidable_stop_enable'),
                'unavoidable_lethal_threshold': LaunchConfiguration('cmd_vel_gate_unavoidable_lethal_threshold'),
                'unavoidable_cluster_min_cells': LaunchConfiguration('cmd_vel_gate_unavoidable_cluster_min_cells'),
                'unavoidable_cluster_min_ratio': LaunchConfiguration('cmd_vel_gate_unavoidable_cluster_min_ratio'),
                'enable_yaw_alignment_zone': LaunchConfiguration('cmd_vel_gate_yaw_alignment_enable'),
                'yaw_alignment_frame_id': LaunchConfiguration('cmd_vel_gate_yaw_alignment_frame_id'),
                'yaw_alignment_zones_file': LaunchConfiguration('cmd_vel_gate_yaw_alignment_zones_file'),
                'yaw_alignment_exit_margin_m': LaunchConfiguration('cmd_vel_gate_yaw_alignment_exit_margin_m'),
            }],
            condition=IfCondition(LaunchConfiguration('cmd_vel_gate_enable')),
        ),
    ])
