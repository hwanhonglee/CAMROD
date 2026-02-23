#!/usr/bin/env python3
"""
imu_only.launch.py

Purpose:
  - Launch the upstream MicroStrain ROS 2 driver from this package.
  - Use a single YAML file (microstrain_cv7.yaml) for all CV7 settings.
  - Expose only simple launch arguments (port, params_file).

Usage:
  ros2 launch camping_cart_sensing imu_only.launch.py

Examples:
  ros2 launch camping_cart_sensing imu_only.launch.py port:=/dev/ttyACM0
  ros2 launch camping_cart_sensing imu_only.launch.py params_file:=/absolute/path/to/microstrain_cv7.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Package share directories
    sensing_share = get_package_share_directory("camping_cart_sensing")
    microstrain_share = get_package_share_directory("microstrain_inertial_driver")

    # Default parameter YAML (CV7 / IMU-only)
    default_params_file = os.path.join(sensing_share, "config", "microstrain_cv7.yaml")

    # Upstream MicroStrain launch file
    microstrain_launch_file = os.path.join(microstrain_share, "launch", "microstrain_launch.py")

    # Launch arguments
    declare_port = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyACM0",
        description="Main serial port for the CV7 device (e.g., /dev/ttyACM0).",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Absolute path to the MicroStrain parameter YAML.",
    )

    # Include upstream MicroStrain launch and pass arguments through
    include_microstrain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(microstrain_launch_file),
        launch_arguments={
            "port": LaunchConfiguration("port"),
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )

    return LaunchDescription([
        declare_port,
        declare_params_file,
        include_microstrain,
    ])
