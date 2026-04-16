#!/usr/bin/env python3

# HH_260410: Keep platform launch tree flat.
# This wrapper keeps Ranger entrypoint under camrod_platform/launch and forwards
# the selected params file to ranger_bringup.

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
        DeclareLaunchArgument(
            'params_file',
            default_value=pkg_share('camrod_platform', os.path.join('config', 'ranger_params.yaml')),
            description='YAML parameter file forwarded to ranger_bringup/ranger.launch.py',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('ranger_bringup', os.path.join('launch', 'ranger.launch.py'))
            ),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
            }.items(),
        ),
    ])
