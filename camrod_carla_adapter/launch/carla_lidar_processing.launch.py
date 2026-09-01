"""Process actual CARLA ray-cast LiDAR into CAMROD perception contracts."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    adapter_share = get_package_share_directory("camrod_carla_adapter")
    perception_share = get_package_share_directory("camrod_perception")
    filter_config = os.path.join(
        adapter_share, "config", "carla_lidar_filter.yaml"
    )
    perception_config = os.path.join(
        perception_share, "config", "perception_params.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("enable", default_value="true"),
        DeclareLaunchArgument(
            "lidar_filter_param_file", default_value=filter_config
        ),
        DeclareLaunchArgument(
            "perception_param_file", default_value=perception_config
        ),
        Node(
            package="camrod_carla_adapter",
            executable="carla_lidar_filter",
            name="carla_lidar_filter",
            namespace="sensing/lidar",
            output="screen",
            parameters=[LaunchConfiguration("lidar_filter_param_file")],
            condition=IfCondition(LaunchConfiguration("enable")),
        ),
        Node(
            package="camrod_perception",
            executable="obstacle_lidar_node",
            name="obstacle_lidar",
            namespace="perception",
            output="screen",
            parameters=[
                LaunchConfiguration("perception_param_file"),
                {
                    # Euclidean clusters remain visualization-only. The
                    # production camera-LiDAR fusion node is the sole owner of
                    # the semantic /perception/obstacles topic in CARLA too.
                    "publish_cluster_cloud": True,
                    "cluster_cloud_topic": (
                        "/perception/lidar/cluster_points"
                    ),
                },
            ],
            condition=IfCondition(LaunchConfiguration("enable")),
        ),
    ])


if __name__ == "__main__":
    generate_launch_description()
