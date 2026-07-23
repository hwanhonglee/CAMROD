"""Launch semantic campsite occupancy detection."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("module_namespace", default_value="perception"),
        DeclareLaunchArgument("perception_param_file", default_value=""),
        DeclareLaunchArgument("camping_sites_yaml", default_value=""),
        Node(
            package="camrod_perception",
            executable="campsite_occupancy_node",
            name="campsite_occupancy",
            namespace=LaunchConfiguration("module_namespace"),
            output="screen",
            parameters=[
                LaunchConfiguration("perception_param_file"),
                {"camping_sites_yaml": LaunchConfiguration("camping_sites_yaml")},
            ],
        ),
    ])
