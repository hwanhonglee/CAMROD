from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 2026-02-02 10:32: Use centralized bringup config (synced from package config).
    bringup_share = get_package_share_directory('camping_cart_bringup')
    default_param = os.path.join(bringup_share, 'config', 'system', 'system_checker.yaml')

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param,
        description='System checker parameter file',
    )
    param_file = LaunchConfiguration('param_file')

    checker_node = Node(
        package='camping_cart_system',
        executable='system_checker_node.py',
        name='system_checker',
        namespace='system',
        output='screen',
        parameters=[param_file],
    )

    return LaunchDescription([
        param_file_arg,
        checker_node,
    ])
