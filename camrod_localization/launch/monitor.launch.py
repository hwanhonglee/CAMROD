#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    localization_share = get_package_share_directory("camrod_localization")

    monitor_param_file = os.path.join(
        localization_share, "config", "filter", "monitor.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=monitor_param_file,
            description="Parameter file for localization_monitor_node",
        ),
        DeclareLaunchArgument(
            "module_namespace",
            default_value="localization",
            description="Namespace for localization monitor node",
        ),

        Node(
            package="camrod_localization",
            executable="localization_monitor_node",
            name="monitor",
            namespace=LaunchConfiguration("module_namespace"),
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
