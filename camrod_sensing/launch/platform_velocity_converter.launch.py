#!/usr/bin/env python3
"""
platform_velocity_converter.launch.py

Purpose:
  - Launch platform_velocity_converter_node as a standalone node.
  - Convert platform velocity + IMU inputs to twist_with_covariance (or related output).

Usage:
  ros2 launch camrod_sensing platform_velocity_converter.launch.py

Examples:
  ros2 launch camrod_sensing platform_velocity_converter.launch.py
  ros2 launch camrod_sensing platform_velocity_converter.launch.py params_file:=/absolute/path/to/sensing_params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    # NOTE:
    # If your sensing_params.yaml is stored in camrod_bringup,
    # use camrod_bringup package share instead.
    #
    # Example:
    # bringup_share = get_package_share_directory("camrod_bringup")
    # default_params_file = os.path.join(bringup_share, "config", "sensing", "sensing_params.yaml")

    sensing_share = get_package_share_directory("camrod_sensing")
    default_params_file = os.path.join(
        sensing_share, "config", "imu", "platform_velocity_converter.yaml"
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Parameter YAML for platform_velocity_converter_node.",
    )
    declare_namespace = DeclareLaunchArgument(
        "module_namespace",
        default_value="imu",
        description="Namespace for platform velocity converter standalone launch",
    )
    declare_velocity_topic = DeclareLaunchArgument(
        "velocity_topic",
        # HH_260317-00:00 Cross-module input from platform stack stays absolute.
        default_value="/platform/status/velocity",
        description="Platform velocity input topic",
    )
    declare_imu_topic = DeclareLaunchArgument(
        "imu_topic",
        default_value="data",
        description="IMU input topic (relative to module namespace)",
    )
    declare_output_topic = DeclareLaunchArgument(
        "output_topic",
        default_value="twist_with_covariance",
        description="Output twist_with_covariance topic (relative to module namespace)",
    )
    declare_diag_topic = DeclareLaunchArgument(
        "imu_diagnostic_topic",
        default_value="diagnostic",
        description="IMU diagnostic topic (relative to module namespace)",
    )

    velocity_converter_node = Node(
        package="camrod_sensing",
        executable="platform_velocity_converter_node",
        name="platform_velocity_converter",
        namespace=LaunchConfiguration("module_namespace"),
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "velocity_topic": LaunchConfiguration("velocity_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "imu_diagnostic_topic": LaunchConfiguration("imu_diagnostic_topic"),
            },
        ],
    )

    return LaunchDescription([
        declare_params_file,
        declare_namespace,
        declare_velocity_topic,
        declare_imu_topic,
        declare_output_topic,
        declare_diag_topic,
        velocity_converter_node,
    ])
