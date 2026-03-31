from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    # HH_260330: Standalone map launch uses package-local config by default.
    config_dir = FindPackageShare('camrod_map').find('camrod_map')
    default_map_info = os.path.join(config_dir, 'config', 'map_info.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_param_file',
            default_value=default_map_info,
            description='Map info YAML file for lanelet loader (unique name to avoid collisions)'),  # HH_260114 Avoid arg name clashes.
        DeclareLaunchArgument(
            'module_namespace',
            default_value='map',
            description='Namespace for map module nodes',
        ),

        # HH_260112 Namespace map node under /map with short name.
        Node(
            package='camrod_map',
            executable='lanelet2_map_node',
            name='lanelet2_map',
            namespace=LaunchConfiguration('module_namespace'),
            parameters=[LaunchConfiguration('map_param_file')],
            output='screen'
        )
    ])
