import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('module_namespace', default_value='platform'),
        DeclareLaunchArgument('map_frame_id', default_value='map'),
        DeclareLaunchArgument('base_frame_id', default_value='robot_base_link'),
        DeclareLaunchArgument(
            'params_file',
            default_value=pkg_share('camrod_sensor_kit', os.path.join('config', 'robot_params.yaml')),
        ),
        DeclareLaunchArgument(
            'robot_visualization_param_file',
            default_value=pkg_share('camrod_platform', os.path.join('config', 'robot_visualization.yaml')),
        ),

        Node(
            package='camrod_platform',
            executable='robot_visualization_node',
            name='robot_visualization',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                LaunchConfiguration('robot_visualization_param_file'),
                {
                    'map_frame_id': LaunchConfiguration('map_frame_id'),
                    'base_frame_id': LaunchConfiguration('base_frame_id'),
                },
            ],
        ),
    ])
