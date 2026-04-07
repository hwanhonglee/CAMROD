from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('module_namespace', default_value='platform'),
        DeclareLaunchArgument('cmd_vel_gate_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_in_topic', default_value='/planning/cmd_vel'),
        DeclareLaunchArgument('cmd_vel_out_topic', default_value='/platform/cmd_vel'),
        DeclareLaunchArgument('drive_enable_topic', default_value='/platform/drive_enable'),
        DeclareLaunchArgument('planning_engage_topic', default_value='/planning/engage'),
        DeclareLaunchArgument('use_planning_engage_topic', default_value='true'),
        DeclareLaunchArgument('drive_state_topic', default_value='/platform/drive_enabled'),
        DeclareLaunchArgument('use_estop_topic', default_value='true'),
        DeclareLaunchArgument('estop_topic', default_value='/planning/state_machine/estop'),
        DeclareLaunchArgument('drive_allow_on_start', default_value='false'),

        Node(
            package='camrod_platform',
            executable='cmd_vel_gate_node.py',
            name='cmd_vel_gate',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            parameters=[{
                'input_cmd_vel_topic': LaunchConfiguration('cmd_vel_in_topic'),
                'output_cmd_vel_topic': LaunchConfiguration('cmd_vel_out_topic'),
                'enable_topic': LaunchConfiguration('drive_enable_topic'),
                'engage_topic': LaunchConfiguration('planning_engage_topic'),
                'use_engage_topic': LaunchConfiguration('use_planning_engage_topic'),
                'state_topic': LaunchConfiguration('drive_state_topic'),
                'use_estop_topic': LaunchConfiguration('use_estop_topic'),
                'estop_topic': LaunchConfiguration('estop_topic'),
                'allow_on_start': LaunchConfiguration('drive_allow_on_start'),
                'publish_zero_when_blocked': True,
            }],
            condition=IfCondition(LaunchConfiguration('cmd_vel_gate_enable')),
        ),
    ])
