# HH_260109 Launch perception obstacle fusion pipeline.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 2026-02-02 10:32: Use centralized bringup config (synced from package config).
    bringup_share = get_package_share_directory('camping_cart_bringup')
    default_param = os.path.join(bringup_share, 'config', 'perception', 'perception_params.yaml')

    # HH_260114 Unique param arg to avoid collisions across includes.
    param_file_arg = DeclareLaunchArgument(
        'perception_param_file',
        default_value=default_param,
        description='Perception parameter file (obstacle fusion)',
    )
    param_file = LaunchConfiguration('perception_param_file')

    # HH_260112 Namespace perception nodes under /perception with short names.
    obstacle_fusion = Node(
        package='camping_cart_perception',
        executable='obstacle_fusion_node',
        name='obstacle_fusion',
        namespace='perception',
        output='screen',
        parameters=[param_file],
    )

    return LaunchDescription([
        param_file_arg,
        obstacle_fusion,
    ])
