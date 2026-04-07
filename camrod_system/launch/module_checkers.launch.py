from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# HH_260407: Backward-compat wrapper.
# Old module_checkers entrypoint now forwards to system.launch.py.
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_checkers', default_value='true'),
        DeclareLaunchArgument('diagnostics_profile', default_value='default'),
        DeclareLaunchArgument('enable_platform_checker', default_value='false'),
        DeclareLaunchArgument('system_namespace', default_value='system'),

        # legacy no-op args
        DeclareLaunchArgument('enable_diagnostics_aggregator', default_value='true'),
        DeclareLaunchArgument('enable_system_diagnostic', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('camrod_system'),
                    'launch',
                    'system.launch.py',
                )
            ),
            launch_arguments={
                'module_namespace': LaunchConfiguration('system_namespace'),
                'config_profile': LaunchConfiguration('diagnostics_profile'),
                'enable_checkers': LaunchConfiguration('enable_checkers'),
                'enable_platform': LaunchConfiguration('enable_platform_checker'),
            }.items(),
        ),
    ])
