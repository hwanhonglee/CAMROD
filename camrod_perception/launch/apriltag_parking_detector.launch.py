#!/usr/bin/env python3
"""Rectify the rear image and detect the configured AprilTag parking target.

HH_260720 - image_proc owns raw-image rectification. The installed
camrod_sensor_kit URDF owns robot_center_link-to-camera_rear TF.
HH_260814 - camrod_sensing now rectifies next to the rear capture node, so this
launch rectifies only when it is started without that camera pipeline. Callers
that already own the rear camera pass launch_rectify:=false to keep exactly one
publisher on image_rect.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        DeclareLaunchArgument('parameter_file', default_value=params),
        # HH_260814 - camera_rear_publisher_node publishes image_rect itself, so this
        # image_proc fallback stays off. Enable it only to rectify a raw source that
        # has no rectifier, such as a bag of image_raw, and expect the opencv4_vendor
        # vs apt OpenCV conflict documented in camera.launch.py.
        DeclareLaunchArgument(
            'launch_rectify',
            default_value='false',
            description=(
                'Rectify image_raw with image_proc. Leave false whenever the rear '
                'camera node is running, since it already publishes image_rect.'
            ),
        ),

        # HH_260720 - Keep rectification as a standard image_proc perception component.
        ComposableNodeContainer(
            name='apriltag_parking_image_proc',
            namespace='perception',
            package='rclcpp_components',
            executable='component_container',
            condition=IfCondition(LaunchConfiguration('launch_rectify')),
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
            parameters=[LaunchConfiguration('parameter_file'), {
                # HH_260720 - The detector consumes the image_proc rectified stream.
                'image_topic': LaunchConfiguration('rear_image_rect_topic'),
                'camera_info_topic': LaunchConfiguration('rear_camera_info_topic'),
            }],
            output='screen',
        ),
    ])
