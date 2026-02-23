from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 2026-02-02 10:32: Use centralized bringup config (synced from package config).
    config_dir = FindPackageShare('camping_cart_bringup').find('camping_cart_bringup')
    default_map_info = os.path.join(config_dir, 'config', 'map', 'map_info.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_param_file',
            default_value=default_map_info,
            description='Map info YAML file for lanelet loader (unique name to avoid collisions)'),  # HH_260114 Avoid arg name clashes.

        # HH_260112 Namespace map node under /map with short name.
        Node(
            package='camping_cart_map',
            executable='lanelet2_map_node',
            name='lanelet2_map',
            namespace='map',
            parameters=[LaunchConfiguration('map_param_file')],
            output='screen'
        )
    ])
