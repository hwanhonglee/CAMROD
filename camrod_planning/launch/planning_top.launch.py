#!/usr/bin/env python3
# HH_260317-00:00 Minimal planning entrypoint.
# Keep top-level launch short; delegate detailed args/params to planning.launch.py.

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
        default_value="planning",
        description="Planning module namespace",
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
    enable_goal_replanner_arg = DeclareLaunchArgument(
        "enable_goal_replanner",
        default_value="true",
        description="Enable goal_replanner helper",
    )
    enable_state_machine_arg = DeclareLaunchArgument(
        "enable_state_machine",
        default_value="false",
        description="Enable planning state machine",
    )
    enable_module_checker_arg = DeclareLaunchArgument(
        "enable_module_checker",
        default_value="true",
        description="Enable planning module checker",
    )

    include_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("camrod_planning"), "launch", "planning.launch.py")
        ),
        launch_arguments={
            "module_namespace": LaunchConfiguration("module_namespace"),
            "system_namespace": LaunchConfiguration("system_namespace"),
            "map_path": LaunchConfiguration("map_path"),
            "enable_goal_replanner": LaunchConfiguration("enable_goal_replanner"),
            "enable_state_machine": LaunchConfiguration("enable_state_machine"),
            "enable_module_checker": LaunchConfiguration("enable_module_checker"),
        }.items(),
    )

    return LaunchDescription(
        [
            module_namespace_arg,
            system_namespace_arg,
            map_path_arg,
            enable_goal_replanner_arg,
            enable_state_machine_arg,
            enable_module_checker_arg,
            include_full,
        ]
    )

