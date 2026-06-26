# HJ_260601: Launch file for guest UI node (WiFi-accessible recall site).
# (확장) ui_destination_topic / site_names 파라미터 추가 (사이트 선택 호출 지원)
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
    # HH_260617: Use canonical `_s` suffix for duration launch arguments.
    grace_period_arg = DeclareLaunchArgument(
        'grace_period_s',
        default_value='60',
        description='Seconds to hold lock after disconnect before releasing to others',
    )
    # (확장) 사이트 선택 호출 — ui_backend_node가 구독하는 토픽과 동일하게 설정
    ui_destination_topic_arg = DeclareLaunchArgument(
        'ui_destination_topic',
        default_value='/ui/selected_destination',
        description='Topic to publish selected site destination (consumed by ui_backend_node)',
    )
    # Note: site_names is a string_array parameter — override via a params YAML file
    #       if the default B1~B13 list needs to change.

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
            'grace_period_s': LaunchConfiguration('grace_period_s'),
            'ui_destination_topic': LaunchConfiguration('ui_destination_topic'),
        }],
    )

    return LaunchDescription([
        enable_arg,
        host_arg,
        port_arg,
        grace_period_arg,
        ui_destination_topic_arg,
        guest_node,
    ])
