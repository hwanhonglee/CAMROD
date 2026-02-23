#!/usr/bin/env python3
"""
Launch SEN0592 ultrasonic radar node (6x serial sensors, Modbus RTU).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("camping_cart_sensing")

    radar_params = LaunchConfiguration("radar_params")

    return LaunchDescription([
        DeclareLaunchArgument(
            "radar_params",
            default_value=os.path.join(pkg_share, "config", "sen0592_radar.yaml"),
            description="ROS2 params YAML for SEN0592 radar node",
        ),

        Node(
            package="camping_cart_sensing",
            executable="sen0592_radar_node",
            name="sen0592_radar_node",
            output="screen",
            parameters=[radar_params],
        ),
    ])
