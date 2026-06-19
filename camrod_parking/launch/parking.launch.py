import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def pkg_path(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def generate_launch_description():
    # HH_260617: Keep parking controllers behind planning/platform cmd_vel gates by default.
    param_file_default = pkg_path("camrod_parking", os.path.join("config", "parking.yaml"))

    return LaunchDescription([
        DeclareLaunchArgument("parking_namespace", default_value="parking"),
        DeclareLaunchArgument("param_file", default_value=param_file_default),
        DeclareLaunchArgument("enable_site_maneuver", default_value="true"),
        DeclareLaunchArgument("enable_drop_zone_parking", default_value="true"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/planning/cmd_vel_raw"),
        DeclareLaunchArgument("pose_topic", default_value="/localization/pose"),
        DeclareLaunchArgument("drop_zones_yaml", default_value=""),
        DeclareLaunchArgument("camping_sites_yaml", default_value=""),

        Node(
            package="camrod_parking",
            executable="site_maneuver_node",
            namespace=LaunchConfiguration("parking_namespace"),
            name="site_maneuver",
            output="screen",
            parameters=[
                LaunchConfiguration("param_file"),
                {
                    "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    "pose_topic": LaunchConfiguration("pose_topic"),
                    # HH_260618: Pass campsite coordinates for site-goal fallback
                    # when /goal_pose was missed before Nav2 reports GOAL_REACHED.
                    "camping_sites_yaml": LaunchConfiguration("camping_sites_yaml"),
                },
            ],
            condition=IfCondition(LaunchConfiguration("enable_site_maneuver")),
        ),
        Node(
            package="camrod_parking",
            executable="drop_zone_parking_node",
            namespace=LaunchConfiguration("parking_namespace"),
            name="drop_zone_parking",
            output="screen",
            parameters=[
                LaunchConfiguration("param_file"),
                {
                    "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    "pose_topic": LaunchConfiguration("pose_topic"),
                    "drop_zones_yaml": LaunchConfiguration("drop_zones_yaml"),
                },
            ],
            condition=IfCondition(LaunchConfiguration("enable_drop_zone_parking")),
        ),
    ])
