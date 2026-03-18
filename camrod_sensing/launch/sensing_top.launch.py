#!/usr/bin/env python3
# HH_260317-00:00 Minimal sensing entrypoint.
# Keep top-level launch short; delegate detailed args/params to sensing.launch.py.

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
        default_value="sensing",
        description="Sensing module namespace",
    )
    system_namespace_arg = DeclareLaunchArgument(
        "system_namespace",
        default_value="system",
        description="System checker namespace",
    )
    gnss_namespace_arg = DeclareLaunchArgument(
        "gnss_namespace",
        default_value="sensing/gnss",
        description="GNSS stack namespace",
    )
    enable_lidar_driver_arg = DeclareLaunchArgument(
        "enable_lidar_driver",
        default_value="false",
        description="Enable Vanjee LiDAR SDK driver",
    )
    enable_radar_arg = DeclareLaunchArgument(
        "enable_radar",
        default_value="false",
        description="Enable serial radar node",
    )
    enable_gnss_arg = DeclareLaunchArgument(
        "enable_gnss",
        default_value="true",
        description="Enable ublox GNSS stack",
    )
    enable_ntrip_arg = DeclareLaunchArgument(
        "enable_ntrip",
        default_value="true",
        description="Enable NTRIP client",
    )
    enable_module_checker_arg = DeclareLaunchArgument(
        "enable_module_checker",
        default_value="true",
        description="Enable sensing module checker",
    )

    include_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("camrod_sensing"), "launch", "sensing.launch.py")
        ),
        launch_arguments={
            "module_namespace": LaunchConfiguration("module_namespace"),
            "system_namespace": LaunchConfiguration("system_namespace"),
            "gnss_namespace": LaunchConfiguration("gnss_namespace"),
            "enable_lidar_driver": LaunchConfiguration("enable_lidar_driver"),
            "enable_radar": LaunchConfiguration("enable_radar"),
            "enable_gnss": LaunchConfiguration("enable_gnss"),
            "enable_ntrip": LaunchConfiguration("enable_ntrip"),
            "enable_module_checker": LaunchConfiguration("enable_module_checker"),
        }.items(),
    )

    return LaunchDescription(
        [
            module_namespace_arg,
            system_namespace_arg,
            gnss_namespace_arg,
            enable_lidar_driver_arg,
            enable_radar_arg,
            enable_gnss_arg,
            enable_ntrip_arg,
            enable_module_checker_arg,
            include_full,
        ]
    )

