#!/usr/bin/env python3
"""
Launch SEN0592 ultrasonic radar node (7x serial sensors, Modbus RTU).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    pkg_share = get_package_share_directory("camrod_sensing")

    radar_params = LaunchConfiguration("radar_params")
    module_namespace = LaunchConfiguration("module_namespace")
    radar_log_status = LaunchConfiguration("radar_log_status")

    return LaunchDescription([
        DeclareLaunchArgument(
            "radar_params",
            # HH_260330: Standalone sensing launch uses package-local config by default.
            default_value=os.path.join(pkg_share, "config", "radar", "sen0592_radar.yaml"),
            description="ROS2 params YAML for SEN0592 radar node",
        ),
        DeclareLaunchArgument(
            "module_namespace",
            # HH_260623 - Direct radar_sensor.launch should publish the same
            # /sensing/radar/* topics consumed by radar cost grid and Nav2.
            default_value="sensing/radar",
            description="Namespace for radar standalone launch",
        ),
        DeclareLaunchArgument(
            "radar_log_status",
            default_value="false",
            description="Print periodic per-port radar range status lines",
        ),

        Node(
            package="camrod_sensing",
            executable="sen0592_radar_node",
            name="sen0592_radar_node",
            namespace=module_namespace,
            output="screen",
            # HH_260623 - Keep the latest seven-radar mapping in YAML so bench overrides
            # can still replace the sensor count without launch-time topic mismatch.
            parameters=[
                radar_params,
                {
                    "radar_status_topic": "status",
                    "log_status": radar_log_status,
                },
            ],
        ),
    ])
