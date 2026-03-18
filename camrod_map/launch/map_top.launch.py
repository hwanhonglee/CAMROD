#!/usr/bin/env python3
# HH_260317-00:00 Minimal map entrypoint.
# Keep top-level launch short; delegate detailed args/params to map.launch.py.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    module_namespace_arg = DeclareLaunchArgument(
        "module_namespace",
        default_value="map",
        description="Map module namespace",
    )
    system_namespace_arg = DeclareLaunchArgument(
        "system_namespace",
        default_value="system",
        description="System checker namespace",
    )
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value="",
        description="Lanelet2 map path override (empty: use defaults from config/map/map_info.yaml)",
    )
    enable_module_checker_arg = DeclareLaunchArgument(
        "enable_module_checker",
        default_value="true",
        description="Enable map module checker",
    )

    include_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("camrod_map"), "launch", "map.launch.py")
        ),
        launch_arguments={
            "module_namespace": LaunchConfiguration("module_namespace"),
            "system_namespace": LaunchConfiguration("system_namespace"),
            "map_path": LaunchConfiguration("map_path"),
            "enable_module_checker": LaunchConfiguration("enable_module_checker"),
        }.items(),
    )

    return LaunchDescription(
        [
            module_namespace_arg,
            system_namespace_arg,
            map_path_arg,
            enable_module_checker_arg,
            include_full,
        ]
    )

