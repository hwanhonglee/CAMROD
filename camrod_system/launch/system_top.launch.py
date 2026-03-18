#!/usr/bin/env python3
# HH_260317-00:00 Minimal system entrypoint.
# Keep top-level launch short; delegate to module_checkers.launch.py.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    system_namespace_arg = DeclareLaunchArgument(
        "system_namespace",
        default_value="system",
        description="System checker namespace",
    )
    enable_checkers_arg = DeclareLaunchArgument(
        "enable_checkers",
        default_value="true",
        description="Enable per-module checkers + system diagnostic",
    )

    include_checkers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("camrod_system"), "launch", "module_checkers.launch.py")
        ),
        launch_arguments={
            "system_namespace": LaunchConfiguration("system_namespace"),
            "enable_checkers": LaunchConfiguration("enable_checkers"),
        }.items(),
    )

    return LaunchDescription(
        [
            system_namespace_arg,
            enable_checkers_arg,
            include_checkers,
        ]
    )

