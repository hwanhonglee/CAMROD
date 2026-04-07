from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('module_namespace', default_value='planning'),
        DeclareLaunchArgument('cmd_vel_gate_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_raw_topic', default_value='/planning/cmd_vel_raw'),
        DeclareLaunchArgument('cmd_vel_output_topic', default_value='/planning/cmd_vel'),
        DeclareLaunchArgument('planning_engage_topic', default_value='/planning/engage'),
        DeclareLaunchArgument('planning_engaged_state_topic', default_value='/planning/engaged'),
        DeclareLaunchArgument('cmd_vel_gate_use_estop_topic', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_estop_topic', default_value='/planning/state_machine/estop'),
        DeclareLaunchArgument('cmd_vel_gate_allow_on_start', default_value='false'),

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
            }],
            condition=IfCondition(LaunchConfiguration('cmd_vel_gate_enable')),
        ),
    ])
