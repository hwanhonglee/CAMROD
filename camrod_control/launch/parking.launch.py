import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def package_path(package_name: str, relative_path: str) -> str:
    return os.path.join(get_package_share_directory(package_name), relative_path)


def generate_launch_description():
    # HH_260720 - Select exactly one parking controller inside camrod_control.
    default_parameter_file = package_path(
        "camrod_control", os.path.join("config", "parking.yaml")
    )

    return LaunchDescription([
        DeclareLaunchArgument("parking_namespace", default_value="parking"),
        DeclareLaunchArgument("parameter_file", default_value=default_parameter_file),
        DeclareLaunchArgument("parking_method", default_value="reverse"),
        DeclareLaunchArgument("command_topic", default_value="/control/cmd_vel_raw"),
        DeclareLaunchArgument("vehicle_pose_topic", default_value="/localization/pose"),
        DeclareLaunchArgument("drop_zones_yaml", default_value=""),
        # HH_260720 - AprilTag parking starts its perception input only when selected.
        DeclareLaunchArgument("launch_apriltag_detector", default_value="true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_path(
                "camrod_perception",
                os.path.join("launch", "apriltag_parking_detector.launch.py"),
            )),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("parking_method"),
                "'.strip().lower() == 'apriltag' and '",
                LaunchConfiguration("launch_apriltag_detector"),
                "'.strip().lower() in ['1', 'true', 'yes', 'on']",
            ])),
        ),

        Node(
            package="camrod_control",
            # HH_260720 - Launch the concrete reverse-parking controller without a legacy alias.
            executable="reverse_parking_controller_node",
            namespace=LaunchConfiguration("parking_namespace"),
            name="reverse_parking_controller",
            output="screen",
            parameters=[
                LaunchConfiguration("parameter_file"),
                {
                    "command_topic": LaunchConfiguration("command_topic"),
                    "vehicle_pose_topic": LaunchConfiguration("vehicle_pose_topic"),
                    "drop_zones_yaml": LaunchConfiguration("drop_zones_yaml"),
                },
            ],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("parking_method"), "'.strip().lower() == 'reverse'"
            ])),
        ),
        Node(
            package="camrod_control",
            executable="apriltag_parking_controller_node",
            namespace=LaunchConfiguration("parking_namespace"),
            name="apriltag_parking_controller",
            output="screen",
            parameters=[
                LaunchConfiguration("parameter_file"),
                {
                    "command_topic": LaunchConfiguration("command_topic"),
                },
            ],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("parking_method"), "'.strip().lower() == 'apriltag'"
            ])),
        ),
    ])
