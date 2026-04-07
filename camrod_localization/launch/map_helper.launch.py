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

    map_helper_param_file = os.path.join(
        localization_share, "config", "reference", "map_helper.yaml"
    )
    map_info_file = os.path.join(
        map_share, "config", "map_info.yaml"
    )
    drop_zones_yaml = os.path.join(
        localization_share, "config", "drop_zones.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=map_helper_param_file,
            description="Parameter file for localization_map_helper_node",
        ),
        DeclareLaunchArgument(
            "map_info_file",
            default_value=map_info_file,
            description="Shared map/localization reference parameter file",
        ),
        DeclareLaunchArgument(
            "drop_zones_yaml",
            default_value=drop_zones_yaml,
            description="Drop zones YAML resolved from package share",
        ),
        DeclareLaunchArgument(
            "map_path",
            default_value="",
            description="Lanelet map path. Leave empty to use map_info.yaml value.",
        ),
        DeclareLaunchArgument(
            "module_namespace",
            default_value="localization",
            description="Namespace for localization map helper node",
        ),

        Node(
            package="camrod_localization",
            executable="localization_map_helper_node",
            name="map_helper",
            namespace=LaunchConfiguration("module_namespace"),
            output="screen",
            parameters=[
                LaunchConfiguration("map_info_file"),
                LaunchConfiguration("params_file"),
                {
                    "drop_zones_yaml": LaunchConfiguration("drop_zones_yaml"),
                    "map_path": LaunchConfiguration("map_path"),
                },
            ],
        ),
    ])
