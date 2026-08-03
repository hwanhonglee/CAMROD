#!/usr/bin/env python3
"""Rectify the rear image and detect the configured AprilTag parking target.

HH_260720 - image_proc owns raw-image rectification. The installed
camrod_sensor_kit URDF owns robot_center_link-to-camera_rear TF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # HH_260720 - Launch perception-only nodes with canonical CAMROD names and topics.
    params = os.path.join(
        get_package_share_directory('camrod_perception'),
        'config', 'apriltag_parking_detector.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rear_image_topic', default_value='/sensing/camera/econ_rear/image_raw',
        ),
        DeclareLaunchArgument(
            'rear_camera_info_topic', default_value='/sensing/camera/econ_rear/camera_info',
        ),
        DeclareLaunchArgument(
            'rear_image_rect_topic', default_value='/sensing/camera/econ_rear/image_rect',
        ),

        # HH_260720 - Keep rectification as a standard image_proc perception component.
        ComposableNodeContainer(
            name='apriltag_parking_image_proc',
            namespace='perception',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='econ_rear_rectify',
                    remappings=[
                        ('image', LaunchConfiguration('rear_image_topic')),
                        ('camera_info', LaunchConfiguration('rear_camera_info_topic')),
                        ('image_rect', LaunchConfiguration('rear_image_rect_topic')),
                    ],
                ),
            ],
        ),
        Node(
            package='camrod_perception',
            executable='apriltag_parking_detector_node',
            namespace='perception',
            name='apriltag_parking_detector',
            parameters=[params, {
                # HH_260720 - The detector consumes the image_proc rectified stream.
                'image_topic': LaunchConfiguration('rear_image_rect_topic'),
                'camera_info_topic': LaunchConfiguration('rear_camera_info_topic'),
            }],
            output='screen',
        ),
    ])
