from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launches cmd_vel gate with optional cost-based stop parameters.
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
        # HH_260413: Optional cost-based stop in front of the platform.
        DeclareLaunchArgument('cmd_vel_gate_cost_stop_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_cost_grid_topic', default_value='/planning/local_costmap/costmap'),
        DeclareLaunchArgument('cmd_vel_gate_cost_pose_topic', default_value='/localization/pose'),
        DeclareLaunchArgument('cmd_vel_gate_cost_threshold', default_value='200'),
        DeclareLaunchArgument('cmd_vel_gate_cost_lookahead_m', default_value='2.0'),
        DeclareLaunchArgument('cmd_vel_gate_cost_width_m', default_value='1.0'),
        DeclareLaunchArgument('cmd_vel_gate_cost_hold_sec', default_value='1.0'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_stop_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_lethal_threshold', default_value='253'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_cluster_min_cells', default_value='25'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_cluster_min_ratio', default_value='0.25'),

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
                'enable_cost_stop': LaunchConfiguration('cmd_vel_gate_cost_stop_enable'),
                'cost_grid_topic': LaunchConfiguration('cmd_vel_gate_cost_grid_topic'),
                'pose_topic': LaunchConfiguration('cmd_vel_gate_cost_pose_topic'),
                'cost_stop_threshold': LaunchConfiguration('cmd_vel_gate_cost_threshold'),
                'cost_stop_lookahead_m': LaunchConfiguration('cmd_vel_gate_cost_lookahead_m'),
                'cost_stop_width_m': LaunchConfiguration('cmd_vel_gate_cost_width_m'),
                'cost_stop_hold_sec': LaunchConfiguration('cmd_vel_gate_cost_hold_sec'),
                'enable_unavoidable_stop': LaunchConfiguration('cmd_vel_gate_unavoidable_stop_enable'),
                'unavoidable_lethal_threshold': LaunchConfiguration('cmd_vel_gate_unavoidable_lethal_threshold'),
                'unavoidable_cluster_min_cells': LaunchConfiguration('cmd_vel_gate_unavoidable_cluster_min_cells'),
                'unavoidable_cluster_min_ratio': LaunchConfiguration('cmd_vel_gate_unavoidable_cluster_min_ratio'),
            }],
            condition=IfCondition(LaunchConfiguration('cmd_vel_gate_enable')),
        ),
    ])
