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
        # HH_260409: Expose wheel bridge wiring at launch level so bringup overrides are applied.
        DeclareLaunchArgument(
            "wheel_bridge_enable",
            default_value="true",
            description="Enable wheel odometry bridge path",
        ),
        DeclareLaunchArgument(
            "wheel_input_topic",
            # HH_260410: Prefer platform status odometry as primary wheel source.
            default_value="/platform/status/odometry",
            description="Wheel bridge input topic",
        ),
        DeclareLaunchArgument(
            "wheel_input_type",
            default_value="nav_odom",
            description="Wheel bridge input type: twist|avg_odom|nav_odom",
        ),
        DeclareLaunchArgument(
            "wheel_fallback_input_topic",
            # HH_260410: Use /rmp401/odom only as fallback when status topic is stale/missing.
            default_value="/rmp401/odom",
            description="Wheel bridge fallback input topic",
        ),
        DeclareLaunchArgument(
            "wheel_fallback_input_type",
            default_value="nav_odom",
            description="Wheel bridge fallback input type: twist|avg_odom|nav_odom",
        ),
        DeclareLaunchArgument(
            "wheel_primary_timeout_sec",
            default_value="0.7",
            description="Primary wheel input timeout before fallback activation (sec)",
        ),
        DeclareLaunchArgument(
            "wheel_output_topic",
            # HH_260410: Use status namespace for unified wheel odometry output.
            default_value="/platform/status/wheel_odometry",
            description="Unified wheel odometry output topic (avg_msgs/msg/Odometry alias)",
        ),
        DeclareLaunchArgument(
            "wheel_nav_output_topic",
            default_value="/platform/wheel/nav_odometry",
            description="Unified wheel odometry output topic (nav_msgs/msg/Odometry)",
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
                {
                    "enable_wheel_odometry_bridge": LaunchConfiguration("wheel_bridge_enable"),
                    "wheel_input_topic": LaunchConfiguration("wheel_input_topic"),
                    "wheel_input_type": LaunchConfiguration("wheel_input_type"),
                    "wheel_fallback_input_topic": LaunchConfiguration("wheel_fallback_input_topic"),
                    "wheel_fallback_input_type": LaunchConfiguration("wheel_fallback_input_type"),
                    "wheel_primary_timeout_sec": LaunchConfiguration("wheel_primary_timeout_sec"),
                    "wheel_output_topic": LaunchConfiguration("wheel_output_topic"),
                    "wheel_nav_output_topic": LaunchConfiguration("wheel_nav_output_topic"),
                },
            ],
        ),
    ])
