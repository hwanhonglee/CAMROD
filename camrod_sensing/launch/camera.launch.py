#!/usr/bin/env python3
"""Launch camera preprocessor with module-level and camera-specific param overlays."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    sensing_share = get_package_share_directory("camrod_sensing")

    default_sensing_param = os.path.join(sensing_share, "config", "sensing_params.yaml")
    default_camera_param = os.path.join(sensing_share, "config", "camera", "preprocessor.yaml")

    sensing_param_file = LaunchConfiguration("sensing_param_file")
    camera_preprocess_param_file = LaunchConfiguration("camera_preprocess_param_file")
    module_namespace = LaunchConfiguration("module_namespace")
    input_image_topic = LaunchConfiguration("input_image_topic")
    input_camera_info_topic = LaunchConfiguration("input_camera_info_topic")
    output_image_topic = LaunchConfiguration("output_image_topic")
    output_camera_info_topic = LaunchConfiguration("output_camera_info_topic")
    camera_diagnostic_topic = LaunchConfiguration("camera_diagnostic_topic")

    declare_args = [
        DeclareLaunchArgument(
            "sensing_param_file",
            default_value=default_sensing_param,
            description="Monolithic sensing parameter file (compatibility overlay)",
        ),
        DeclareLaunchArgument(
            "camera_preprocess_param_file",
            default_value=default_camera_param,
            description="Camera preprocessor algorithm parameters",
        ),
        DeclareLaunchArgument(
            "module_namespace",
            default_value="camera",
            description="Namespace for camera preprocessor node",
        ),
        DeclareLaunchArgument(
            "input_image_topic",
            default_value="image_raw",
            description="Input image topic (relative to module namespace)",
        ),
        DeclareLaunchArgument(
            "input_camera_info_topic",
            default_value="camera_info",
            description="Input camera_info topic (relative to module namespace)",
        ),
        DeclareLaunchArgument(
            "output_image_topic",
            default_value="processed/image",
            description="Output processed image topic (relative to module namespace)",
        ),
        DeclareLaunchArgument(
            "output_camera_info_topic",
            default_value="processed/camera_info",
            description="Output processed camera_info topic (relative to module namespace)",
        ),
        DeclareLaunchArgument(
            "camera_diagnostic_topic",
            default_value="diagnostic",
            description="Camera diagnostic topic (relative to module namespace)",
        ),
    ]

    camera_preprocessor = Node(
        package="camrod_sensing",
        executable="camera_preprocessor_node",
        name="camera_preprocessor",
        namespace=module_namespace,
        output="screen",
        # HH_260315-00:00 Keep deterministic override order:
        #   1) module default
        #   2) camera algorithm-specific parameters
        parameters=[
            sensing_param_file,
            camera_preprocess_param_file,
            {
                "input_image_topic": input_image_topic,
                "input_camera_info_topic": input_camera_info_topic,
                "output_image_topic": output_image_topic,
                "output_camera_info_topic": output_camera_info_topic,
                "camera_diagnostic_topic": camera_diagnostic_topic,
            },
        ],
    )

    return LaunchDescription(declare_args + [camera_preprocessor])
