"""HH_261006 - Measure two isolated detector profiles without changing defaults.

The physical Jetson capture component is deliberately not loaded on x86.
Both containers receive the same live CARLA camera; no vehicle commands exist.
"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    source = Path(get_package_share_directory("camrod_perception")) / "config/apriltag_parking_detector.yaml"
    production = yaml.safe_load(source.read_text())["/perception/apriltag_parking_detector"]["ros__parameters"]
    containers = []
    for decimate in (1, 2):
        parameters = dict(production)
        parameters.update({
            "image_topic": "/carla/ego_vehicle/rgb_rear/image",
            "camera_info_topic": "/carla/ego_vehicle/rgb_rear/camera_info",
            "quad_decimate": float(decimate),
            "publish_tf": False,
        })
        namespace = f"/rear_perf_decimate_{decimate}"
        containers.append(ComposableNodeContainer(
            package="camrod_runtime", executable="scoped_component_container_mt",
            name=f"rear_perf_decimate_{decimate}_container", namespace="",
            output="screen",
            composable_node_descriptions=[ComposableNode(
                package="camrod_perception",
                plugin="camrod::perception::AprilTagParkingDetectorNode",
                name="apriltag_parking_detector", namespace=namespace,
                parameters=[parameters],
                extra_arguments=[{"use_intra_process_comms": True}],
            )],
        ))
    return LaunchDescription(containers)
