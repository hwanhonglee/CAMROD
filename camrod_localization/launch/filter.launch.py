#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    localization_share = get_package_share_directory("camrod_localization")

    eskf_param_file = os.path.join(
        localization_share, "config", "filter", "eskf.yaml"
    )
    pose_selector_param_file = os.path.join(
        localization_share, "config", "filter", "pose_selector.yaml"
    )
    kimera_param_file = os.path.join(
        localization_share, "config", "source", "kimera_bridge.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "module_namespace",
            default_value="localization",
            description="Namespace for localization filter nodes",
        ),
        DeclareLaunchArgument(
            "enable_kimera_bridge",
            default_value="false",
            description="Enable kimera csv bridge node",
        ),
        DeclareLaunchArgument(
            "eskf_params_file",
            default_value=eskf_param_file,
            description="ESKF parameter file",
        ),
        DeclareLaunchArgument(
            "pose_selector_params_file",
            default_value=pose_selector_param_file,
            description="Pose selector parameter file",
        ),
        DeclareLaunchArgument(
            "kimera_params_file",
            default_value=kimera_param_file,
            description="Kimera bridge parameter file",
        ),

        Node(
            package="camrod_localization",
            executable="localization_eskf_node",
            name="eskf_filter",
            namespace=LaunchConfiguration("module_namespace"),
            output="screen",
            parameters=[LaunchConfiguration("eskf_params_file")],
        ),

        Node(
            package="camrod_localization",
            executable="localization_pose_selector_node",
            name="pose_selector",
            namespace=LaunchConfiguration("module_namespace"),
            output="screen",
            parameters=[LaunchConfiguration("pose_selector_params_file")],
        ),

        Node(
            package="camrod_localization",
            executable="kimera_csv_bridge_node",
            name="kimera_csv_bridge",
            namespace=LaunchConfiguration("module_namespace"),
            output="screen",
            parameters=[LaunchConfiguration("kimera_params_file")],
            condition=IfCondition(LaunchConfiguration("enable_kimera_bridge")),
        ),
    ])
