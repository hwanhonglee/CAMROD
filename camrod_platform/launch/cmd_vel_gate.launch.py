from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Declares cmd_vel gate arguments and launches the platform velocity gate node.
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('module_namespace', default_value='platform'),
        DeclareLaunchArgument('cmd_vel_gate_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_in_topic', default_value='/planning/cmd_vel'),
        DeclareLaunchArgument('cmd_vel_out_topic', default_value='/platform/cmd_vel'),
        DeclareLaunchArgument('drive_enable_topic', default_value='/platform/drive_enable'),
        # HHL_260624 - Subscribe to the effective planning gate state by default.
        # This keeps manual /planning/engage and UI /planning/mission_engage independent.
        DeclareLaunchArgument('planning_engage_topic', default_value='/planning/engaged'),
        # HH_260522: unified source selector for engage signal.
        #   planning_engage/planning_engaged/topic/enabled/on: subscribe configured topic
        #   disabled/off/none: ignore engage topic
        DeclareLaunchArgument('engage_source_mode', default_value='planning_engage'),
        DeclareLaunchArgument('drive_state_topic', default_value='/platform/drive_enabled'),
        # HH_260522: unified source selector for e-stop signal.
        #   platform_status/topic/enabled/on: subscribe /platform/status/estop
        #   disabled/off/none: ignore e-stop topic
        DeclareLaunchArgument('estop_source_mode', default_value='platform_status'),
        # HH_260409: Default e-stop source is platform status bridge from CAN/system_state.
        DeclareLaunchArgument('estop_topic', default_value='/platform/status/estop'),
        DeclareLaunchArgument('drive_allow_on_start', default_value='false'),

        Node(
            package='camrod_platform',
            executable='cmd_vel_gate_node',
            name='cmd_vel_gate',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            parameters=[{
                'input_cmd_vel_topic': LaunchConfiguration('cmd_vel_in_topic'),
                'output_cmd_vel_topic': LaunchConfiguration('cmd_vel_out_topic'),
                'enable_topic': LaunchConfiguration('drive_enable_topic'),
                'engage_topic': LaunchConfiguration('planning_engage_topic'),
                'engage_source_mode': LaunchConfiguration('engage_source_mode'),
                'state_topic': LaunchConfiguration('drive_state_topic'),
                'estop_source_mode': LaunchConfiguration('estop_source_mode'),
                'estop_topic': LaunchConfiguration('estop_topic'),
                'allow_on_start': LaunchConfiguration('drive_allow_on_start'),
                'publish_zero_when_blocked': True,
            }],
            condition=IfCondition(LaunchConfiguration('cmd_vel_gate_enable')),
        ),
    ])
