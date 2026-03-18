from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    # 2026-02-02 10:32: Use centralized bringup config (synced from package config).
    bringup_share = get_package_share_directory('camrod_bringup')
    default_param = os.path.join(bringup_share, 'config', 'system', 'system_checker.yaml')

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param,
        description='System checker parameter file',
    )
    system_namespace_arg = DeclareLaunchArgument(
        'system_namespace',
        default_value='system',
        description='Namespace for system checker/diagnostic nodes',
    )
    param_file = LaunchConfiguration('param_file')
    system_namespace = LaunchConfiguration('system_namespace')

    checker_node = Node(
        package='camrod_system',
        executable='system_checker_node.py',
        name='system_checker',
        namespace=system_namespace,
        output='screen',
        parameters=[param_file],
    )

    diagnostic_node = Node(
        package='camrod_system',
        executable='system_diagnostic_node.py',
        name='system_diagnostic',
        namespace=system_namespace,
        output='screen',
        parameters=[{
            'diagnostic_topic': '/diagnostics',
            'source_diagnostic_topic': '/diagnostics',
            'publish_period_s': 0.5,
        }],
    )

    return LaunchDescription([
        param_file_arg,
        system_namespace_arg,
        checker_node,
        diagnostic_node,
    ])
