"""Compose the physical rear-camera parking perception hot path."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _pkg_share(package: str, relative_path: str) -> str:
    return os.path.join(get_package_share_directory(package), relative_path)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("enable_container", default_value="false"),
        DeclareLaunchArgument(
            "camera_params_file",
            default_value=_pkg_share(
                "camrod_sensing", "config/camera/camera_params.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "apriltag_params_file",
            default_value=_pkg_share(
                "camrod_perception", "config/apriltag_parking_detector.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "rear_camera_namespace",
            default_value="/sensing/camera/econ_rear",
        ),
        DeclareLaunchArgument(
            "perception_namespace",
            default_value="/perception",
        ),

        # HH_260805 - Keep capture, rectification, and tag detection in one
        # multithreaded process. Public topics and node names match the fallback
        # launch, while raw/rectified image transport stays in-process.
        ComposableNodeContainer(
            package="camrod_runtime",
            executable="scoped_component_container_mt",
            name="rear_camera_apriltag_container",
            namespace="",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_container")),
            composable_node_descriptions=[
                ComposableNode(
                    package="camrod_sensing",
                    plugin="camrod::sensing::CameraRearPublisherNode",
                    name="camera_rear_publisher",
                    namespace=LaunchConfiguration("rear_camera_namespace"),
                    parameters=[LaunchConfiguration("camera_params_file")],
                    remappings=[
                        ("~/image_raw", "image_raw"),
                        ("~/image_raw/compressed", "image_raw/compressed"),
                        ("~/camera_info", "camera_info"),
                        ("~/image_rect", "image_rect"),
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                # HH_260814 - image_proc::RectifyNode was removed here. It pulled a
                # workspace image_geometry (opencv4_vendor) into a process holding
                # the apt OpenCV and segfaulted inside initUndistortRectifyMap, so
                # image_rect never reached the detector. camera_rear_publisher_node
                # now rectifies with the same CameraInfo it publishes.
                ComposableNode(
                    package="camrod_perception",
                    plugin="camrod::perception::AprilTagParkingDetectorNode",
                    name="apriltag_parking_detector",
                    namespace=LaunchConfiguration("perception_namespace"),
                    parameters=[
                        LaunchConfiguration("apriltag_params_file"),
                        {
                            "image_topic": "/sensing/camera/econ_rear/image_rect",
                            "camera_info_topic": "/sensing/camera/econ_rear/camera_info",
                        },
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
        ),
    ])
