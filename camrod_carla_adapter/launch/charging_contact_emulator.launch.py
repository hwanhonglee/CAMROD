"""Launch the fail-closed CARLA Drop Zone charging-contact emulator."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_drop_zones = os.path.join(
        get_package_share_directory("camrod_map"), "config", "drop_zones.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("enable", default_value="false"),
        DeclareLaunchArgument("drop_zones_yaml", default_value=default_drop_zones),
        DeclareLaunchArgument("drop_zone_id", default_value="drop_zone"),
        DeclareLaunchArgument("pose_topic", default_value="/localization/pose"),
        DeclareLaunchArgument("odometry_topic", default_value="/odom"),
        DeclareLaunchArgument(
            "parking_status_topic",
            default_value="/parking/reverse_parking_controller/status",
        ),
        DeclareLaunchArgument(
            "planning_state_topic",
            default_value="/planning/state_machine/state",
        ),
        DeclareLaunchArgument(
            "charging_topic",
            default_value="/camrod_carla/platform_heartbeat/charging",
        ),
        DeclareLaunchArgument("position_tolerance_m", default_value="0.35"),
        DeclareLaunchArgument("speed_tolerance_mps", default_value="0.05"),
        DeclareLaunchArgument("pose_timeout_s", default_value="0.5"),
        DeclareLaunchArgument("odometry_timeout_s", default_value="0.5"),
        # reverse_parking_controller publishes IDLE at 1 Hz.  Two seconds is
        # bounded but tolerates one scheduling interval during stack startup.
        DeclareLaunchArgument("state_timeout_s", default_value="2.0"),
        DeclareLaunchArgument("dwell_s", default_value="1.0"),
        # IncludeLaunchDescription launch arguments share one context.  A
        # generic ``publish_rate_hz`` therefore inherits the intentionally
        # empty legacy alias declared by platform_heartbeat.launch.py when the
        # two nodes are composed by camrod_carla_full.launch.py.  Keep the ROS
        # parameter name generic, but make this launch boundary unambiguous.
        DeclareLaunchArgument(
            "charging_contact_publish_rate_hz", default_value="10.0"
        ),
        Node(
            package="camrod_carla_adapter",
            executable="carla_charging_contact_emulator",
            name="carla_charging_contact_emulator",
            output="screen",
            parameters=[{
                "drop_zones_yaml": LaunchConfiguration("drop_zones_yaml"),
                "drop_zone_id": LaunchConfiguration("drop_zone_id"),
                "pose_topic": LaunchConfiguration("pose_topic"),
                "odometry_topic": LaunchConfiguration("odometry_topic"),
                "parking_status_topic": LaunchConfiguration(
                    "parking_status_topic"
                ),
                "planning_state_topic": LaunchConfiguration(
                    "planning_state_topic"
                ),
                "charging_topic": LaunchConfiguration("charging_topic"),
                **{
                    name: ParameterValue(
                        LaunchConfiguration(name), value_type=float
                    )
                    for name in (
                        "position_tolerance_m",
                        "speed_tolerance_mps",
                        "pose_timeout_s",
                        "odometry_timeout_s",
                        "state_timeout_s",
                        "dwell_s",
                    )
                },
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration(
                        "charging_contact_publish_rate_hz"
                    ),
                    value_type=float,
                ),
            }],
            condition=IfCondition(LaunchConfiguration("enable")),
        ),
    ])
