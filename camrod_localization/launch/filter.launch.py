#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    localization_share = get_package_share_directory("camrod_localization")

    ekf_param_file = os.path.join(localization_share, "config", "filter", "ekf.yaml")
    pose_selector_param_file = os.path.join(
        localization_share, "config", "filter", "pose_selector.yaml"
    )
    gnss_reattach_param_file = os.path.join(
        localization_share, "config", "filter", "gnss_reattach.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "module_namespace",
                default_value="localization",
                description="Namespace for localization filter nodes",
            ),
            # HH_260721 - Run robot_localization EKF as the only localization filter backend.
            DeclareLaunchArgument(
                "ekf_params_file",
                default_value=ekf_param_file,
                description="EKF parameter file",
            ),
            DeclareLaunchArgument(
                "pose_selector_params_file",
                default_value=pose_selector_param_file,
                description="Pose selector parameter file",
            ),
            DeclareLaunchArgument(
                "gnss_reattach_params_file",
                default_value=gnss_reattach_param_file,
                description="GNSS reattach helper parameter file",
            ),
            # HH_260721 - Publish EKF odometry on the explicit standard ROS boundary.
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter",
                namespace=LaunchConfiguration("module_namespace"),
                output="screen",
                parameters=[LaunchConfiguration("ekf_params_file")],
                # HH_260528: suppress verbose INFO logs from robot_localization
                # (e.g. "Received set_pose request" printed on every GNSS-reattach cycle).
                ros_arguments=["--log-level", "localization.ekf_filter:=warn"],
                # HH_260720 - Keep robot_localization output on an explicit standard boundary;
                # localization_input_adapter converts it to generated primary topics.
                remappings=[
                    ("odometry/filtered", "/localization/primary/odometry_ros"),
                ],
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
                executable="localization_gnss_reattach_node",
                name="gnss_reattach",
                namespace=LaunchConfiguration("module_namespace"),
                output="screen",
                parameters=[LaunchConfiguration("gnss_reattach_params_file")],
            ),
        ]
    )
