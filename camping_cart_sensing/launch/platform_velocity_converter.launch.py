#!/usr/bin/env python3
"""
platform_velocity_converter.launch.py

Purpose:
  - Launch platform_velocity_converter_node as a standalone node.
  - Convert platform velocity + IMU inputs to twist_with_covariance (or related output).

Usage:
  ros2 launch camping_cart_sensing platform_velocity_converter.launch.py

Examples:
  ros2 launch camping_cart_sensing platform_velocity_converter.launch.py
  ros2 launch camping_cart_sensing platform_velocity_converter.launch.py params_file:=/absolute/path/to/sensing_params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # NOTE:
    # If your sensing_params.yaml is stored in camping_cart_bringup,
    # use camping_cart_bringup package share instead.
    #
    # Example:
    # bringup_share = get_package_share_directory("camping_cart_bringup")
    # default_params_file = os.path.join(bringup_share, "config", "sensing", "sensing_params.yaml")

    sensing_share = get_package_share_directory("camping_cart_sensing")
    default_params_file = os.path.join(sensing_share, "config", "sensing_params.yaml")

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Parameter YAML for platform_velocity_converter_node.",
    )

    velocity_converter_node = Node(
        package="camping_cart_sensing",
        executable="platform_velocity_converter_node",
        name="platform_velocity_converter",
        namespace="sensing",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        declare_params_file,
        velocity_converter_node,
    ])
