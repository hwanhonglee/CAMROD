#!/usr/bin/env python3
# HH_260317-00:00 Minimal localization entrypoint.
# Keep top-level launch short; delegate detailed args/params to localization.launch.py.

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
        default_value="localization",
        description="Localization module namespace",
    )
    platform_namespace_arg = DeclareLaunchArgument(
        "platform_namespace",
        default_value="platform",
        description="Platform interface namespace",
    )
    system_namespace_arg = DeclareLaunchArgument(
        "system_namespace",
        default_value="system",
        description="System checker namespace",
    )
    navsat_topic_arg = DeclareLaunchArgument(
        "navsat_topic",
        default_value="/sensing/gnss/navsatfix",
        description="GNSS NavSatFix topic",
    )
    use_eskf_arg = DeclareLaunchArgument(
        "use_eskf",
        default_value="true",
        description="Use ESKF or legacy EKF",
    )
    enable_module_checker_arg = DeclareLaunchArgument(
        "enable_module_checker",
        default_value="true",
        description="Enable localization module checker",
    )

    include_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("camrod_localization"), "launch", "localization.launch.py")
        ),
        launch_arguments={
            "module_namespace": LaunchConfiguration("module_namespace"),
            "platform_namespace": LaunchConfiguration("platform_namespace"),
            "system_namespace": LaunchConfiguration("system_namespace"),
            "navsat_topic": LaunchConfiguration("navsat_topic"),
            "use_eskf": LaunchConfiguration("use_eskf"),
            "enable_module_checker": LaunchConfiguration("enable_module_checker"),
        }.items(),
    )

    return LaunchDescription(
        [
            module_namespace_arg,
            platform_namespace_arg,
            system_namespace_arg,
            navsat_topic_arg,
            use_eskf_arg,
            enable_module_checker_arg,
            include_full,
        ]
    )

