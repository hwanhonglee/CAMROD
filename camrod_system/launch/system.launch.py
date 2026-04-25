import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('module_namespace', default_value='system'),
        DeclareLaunchArgument('config_profile', default_value='default'),
        DeclareLaunchArgument('enable_checkers', default_value='true'),
        DeclareLaunchArgument('enable_platform', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_system', os.path.join('launch', 'system_diagnostics.launch.py'))
            ),
            launch_arguments={
                'module_namespace': LaunchConfiguration('module_namespace'),
                'config_profile': LaunchConfiguration('config_profile'),
                'enable_checkers': LaunchConfiguration('enable_checkers'),
                'enable_platform': LaunchConfiguration('enable_platform'),
            }.items(),
        ),
    ])
