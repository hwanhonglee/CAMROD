# HJ_260601: Launch file for guest UI node (WiFi-accessible recall site).
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_arg = DeclareLaunchArgument(
        'enable_ui_guest',
        default_value='true',
        description='Enable guest UI backend',
    )
    host_arg = DeclareLaunchArgument(
        'guest_host',
        default_value='0.0.0.0',
        description='Guest UI bind host (0.0.0.0 = all interfaces, accessible on WiFi)',
    )
    port_arg = DeclareLaunchArgument(
        'guest_port',
        default_value='8012',
        description='Guest UI bind port',
    )
    grace_period_arg = DeclareLaunchArgument(
        'grace_period_sec',
        default_value='60',
        description='Seconds to hold lock after disconnect before releasing to others',
    )

    guest_node = Node(
        package='camrod_ui',
        executable='ui_guest_node',
        name='ui_guest',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_ui_guest')),
        parameters=[{
            'host': LaunchConfiguration('guest_host'),
            'port': LaunchConfiguration('guest_port'),
            'amr_service_state_topic': '/AMR_service_state',
            'battery_topic': '/battery_percentage',
            'grace_period_sec': LaunchConfiguration('grace_period_sec'),
        }],
    )

    return LaunchDescription([
        enable_arg,
        host_arg,
        port_arg,
        grace_period_arg,
        guest_node,
    ])
