import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def _inc(path, *through, **overrides):
    args = {k: LaunchConfiguration(k) for k in through}
    args.update(overrides)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=args.items(),
    )


def generate_launch_description():
    default_param = pkg_share('camrod_perception', os.path.join('config', 'perception_params.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument('module_namespace',       default_value='perception'),
        DeclareLaunchArgument('perception_param_file',  default_value=default_param),
        DeclareLaunchArgument('enable_lidar_obstacle',  default_value='true'),

        _inc(pkg_share('camrod_perception', os.path.join('launch', 'obstacle_fusion.launch.py')),
             'module_namespace', 'perception_param_file'),

        _inc(pkg_share('camrod_perception', os.path.join('launch', 'obstacle_lidar.launch.py')),
             'module_namespace', 'perception_param_file', 'enable_lidar_obstacle'),
    ])
