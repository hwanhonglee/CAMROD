import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def generate_launch_description():
    default_param = pkg_share('camrod_perception', os.path.join('config', 'perception_params.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument('module_namespace', default_value='perception'),
        DeclareLaunchArgument('perception_param_file', default_value=default_param),
        DeclareLaunchArgument('enable_lidar_obstacle', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_perception', os.path.join('launch', 'obstacle_fusion.launch.py'))
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_perception', os.path.join('launch', 'obstacle_lidar.launch.py'))
            ),
        ),
    ])
