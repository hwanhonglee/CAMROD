#!/usr/bin/env python3
"""
Placeholder launch for LiDAR sensing pipeline.

TODO:
- Replace placeholder with actual LiDAR driver/preprocessor nodes
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory("camping_cart_bringup")
    default_param = os.path.join(bringup_share, "config", "sensing", "sensing_params.yaml")

    sensing_param_file = LaunchConfiguration("sensing_param_file")

    declare_args = [
        DeclareLaunchArgument(
            "sensing_param_file",
            default_value=default_param,
            description="Sensing parameter file (LiDAR placeholder)",
        ),
    ]

    return LaunchDescription(declare_args + [
        LogInfo(msg=["[sensing/lidar] Placeholder launch active. param=", sensing_param_file]),
    ])
