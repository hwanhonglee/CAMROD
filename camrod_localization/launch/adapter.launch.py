#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    localization_share = get_package_share_directory("camrod_localization")
    map_share = get_package_share_directory("camrod_map")

    input_adapter_param_file = os.path.join(
        localization_share, "config", "source", "input_adapter.yaml"
    )
    map_info_file = os.path.join(
        map_share, "config", "map_info.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=input_adapter_param_file,
            description="Parameter file for localization_input_adapter_node",
        ),
        DeclareLaunchArgument(
            "map_info_file",
            default_value=map_info_file,
            description="Shared map/localization reference parameter file",
        ),
        DeclareLaunchArgument(
            "module_namespace",
            default_value="localization",
            description="Namespace for localization adapter node",
        ),

        Node(
            package="camrod_localization",
            executable="localization_input_adapter_node",
            name="input_adapter",
            namespace=LaunchConfiguration("module_namespace"),
            output="screen",
            parameters=[
                LaunchConfiguration("map_info_file"),
                LaunchConfiguration("params_file"),
            ],
        ),
    ])
