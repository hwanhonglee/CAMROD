import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def package_path(package_name: str, relative_path: str) -> str:
    return os.path.join(get_package_share_directory(package_name), relative_path)


def generate_launch_description():
    # HH_260720 - Group non-Nav2 motion controllers under the control namespace.
    default_parameter_file = package_path(
        "camrod_control", os.path.join("config", "control.yaml")
    )

    return LaunchDescription([
        DeclareLaunchArgument("control_namespace", default_value="control"),
        DeclareLaunchArgument("parameter_file", default_value=default_parameter_file),
        DeclareLaunchArgument("enable_camping_site_maneuver_controller", default_value="true"),
        DeclareLaunchArgument("enable_drop_zone_maneuver_controller", default_value="true"),
        DeclareLaunchArgument("command_topic", default_value="/control/cmd_vel_raw"),
        DeclareLaunchArgument("vehicle_pose_topic", default_value="/localization/pose"),
        DeclareLaunchArgument("drop_zones_yaml", default_value=""),
        DeclareLaunchArgument("camping_sites_yaml", default_value=""),

        Node(
            package="camrod_control",
            # HH_260720 - Launch the explicitly named camping-site controller executable.
            executable="camping_site_maneuver_controller_node",
            namespace=LaunchConfiguration("control_namespace"),
            name="camping_site_maneuver_controller",
            output="screen",
            parameters=[
                LaunchConfiguration("parameter_file"),
                {
                    "cmd_vel_topic": LaunchConfiguration("command_topic"),
                    "pose_topic": LaunchConfiguration("vehicle_pose_topic"),
                    "camping_sites_yaml": LaunchConfiguration("camping_sites_yaml"),
                },
            ],
            condition=IfCondition(LaunchConfiguration("enable_camping_site_maneuver_controller")),
        ),
        Node(
            package="camrod_control",
            # HH_260720 - Launch the explicitly named drop-zone controller executable.
            executable="drop_zone_maneuver_controller_node",
            namespace=LaunchConfiguration("control_namespace"),
            name="drop_zone_maneuver_controller",
            output="screen",
            parameters=[
                LaunchConfiguration("parameter_file"),
                {
                    "command_topic": LaunchConfiguration("command_topic"),
                    "vehicle_pose_topic": LaunchConfiguration("vehicle_pose_topic"),
                    "drop_zones_yaml": LaunchConfiguration("drop_zones_yaml"),
                },
            ],
            condition=IfCondition(LaunchConfiguration("enable_drop_zone_maneuver_controller")),
        ),
    ])
