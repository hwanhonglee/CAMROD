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


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    pkg_share = get_package_share_directory("camrod_sensing")

    radar_params = LaunchConfiguration("radar_params")
    module_namespace = LaunchConfiguration("module_namespace")

    return LaunchDescription([
        DeclareLaunchArgument(
            "radar_params",
            default_value=os.path.join(pkg_share, "config", "radar", "sen0592_radar.yaml"),
            description="ROS2 params YAML for SEN0592 radar node",
        ),
        DeclareLaunchArgument(
            "module_namespace",
            default_value="radar",
            description="Namespace for radar standalone launch",
        ),

        Node(
            package="camrod_sensing",
            executable="sen0592_radar_node",
            name="sen0592_radar_node",
            namespace=module_namespace,
            output="screen",
            parameters=[
                radar_params,
                {
                    "topics": [
                        "rear/range",
                        "left2/range",
                        "left1/range",
                        "right2/range",
                        "right1/range",
                        "front/range",
                    ],
                    "radar_diagnostic_topic": "diagnostic",
                },
            ],
        ),
    ])
