"""Launch the command and feedback boundaries between CAMROD and CARLA."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _role_topic(suffix):
    """Build a CARLA topic from the configured ego role at launch time."""

    return [
        TextSubstitution(text="/carla/"),
        LaunchConfiguration("role_name"),
        TextSubstitution(text=suffix),
    ]


def generate_launch_description():
    share = get_package_share_directory("camrod_carla_adapter")
    command_config = os.path.join(share, "config", "command_adapter.yaml")
    feedback_config = os.path.join(share, "config", "feedback_bridge.yaml")
    alignment_config = os.path.join(
        share, "config", "woraksan_lane_anchor_alignment.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("role_name", default_value="ego_vehicle"),
        DeclareLaunchArgument(
            "command_config_file", default_value=command_config),
        DeclareLaunchArgument(
            "feedback_config_file", default_value=feedback_config),
        DeclareLaunchArgument(
            "map_alignment_file", default_value=alignment_config),
        DeclareLaunchArgument(
            "input_twist_topic", default_value="/control/cmd_vel_ros"),
        DeclareLaunchArgument(
            "recovery_breakaway_enable",
            default_value="false",
            description=(
                "Authorize the historical low-speed route-recovery torque "
                "lease; disabled for develop-parity command translation"
            ),
        ),
        DeclareLaunchArgument(
            "rotation_recovery_breakaway_enable",
            default_value="false",
            description=(
                "Authorize only a fresh active CRAB_OUT yaw-recovery lease; "
                "disabled for ordinary/develop-parity rotation"
            ),
        ),
        DeclareLaunchArgument(
            "rotation_recovery_breakaway_status_timeout_sec",
            default_value="0.30",
            description=(
                "Freshness lease for the exact campsite CRAB_OUT status; "
                "the CARLA site wrapper alone raises this for the existing "
                "1 Hz controller heartbeat"
            ),
        ),
        DeclareLaunchArgument(
            "extended_command_topic",
            default_value=_role_topic("/extended_ackermann_cmd"),
        ),
        DeclareLaunchArgument(
            "carla_odometry_topic",
            default_value=_role_topic("/odometry"),
        ),
        DeclareLaunchArgument(
            "carla_imu_topic", default_value=_role_topic("/imu")),
        DeclareLaunchArgument(
            "carla_gnss_topic", default_value=_role_topic("/gnss")),
        DeclareLaunchArgument(
            "carla_gnss_right_topic",
            default_value=_role_topic("/gnss_right"),
        ),
        DeclareLaunchArgument(
            "platform_odometry_topic",
            default_value="/platform/status/odometry",
            description=(
                "Standard nav_msgs/Odometry output. Use /odom when the "
                "CAMROD ranger_platform_bridge owns the generated "
                "/platform/status/odometry topic."
            ),
        ),
        DeclareLaunchArgument(
            "publish_platform_odometry", default_value="true"),
        DeclareLaunchArgument("publish_metric_pose", default_value="true"),
        DeclareLaunchArgument("relay_imu", default_value="true"),
        DeclareLaunchArgument("relay_gnss", default_value="true"),
        DeclareLaunchArgument(
            "publish_ground_truth_localization",
            default_value="false",
            description=(
                "Publish standard /localization/*_ros directly. Keep false "
                "when CAMROD localization/EKF is running."
            ),
        ),
        DeclareLaunchArgument(
            "publish_ground_truth_tf",
            default_value="false",
            description=(
                "Publish map->odom and odom->robot_center_link. Keep false "
                "when CAMROD localization/EKF owns TF."
            ),
        ),

        Node(
            package="camrod_carla_adapter",
            executable="twist_to_4ws",
            name="camrod_twist_to_4ws",
            output="screen",
            parameters=[
                LaunchConfiguration("command_config_file"),
                {
                    "input_topic": LaunchConfiguration("input_twist_topic"),
                    "output_topic": LaunchConfiguration(
                        "extended_command_topic"),
                    "recovery_breakaway_enable": ParameterValue(
                        LaunchConfiguration("recovery_breakaway_enable"),
                        value_type=bool,
                    ),
                    "rotation_recovery_breakaway_enable": ParameterValue(
                        LaunchConfiguration(
                            "rotation_recovery_breakaway_enable"
                        ),
                        value_type=bool,
                    ),
                    "rotation_recovery_breakaway_status_timeout_sec": (
                        ParameterValue(
                            LaunchConfiguration(
                                "rotation_recovery_breakaway_status_timeout_sec"
                            ),
                            value_type=float,
                        )
                    ),
                    "use_sim_time": False,
                },
            ],
        ),
        Node(
            package="camrod_carla_adapter",
            executable="carla_feedback_bridge",
            name="carla_feedback_bridge",
            output="screen",
            parameters=[
                LaunchConfiguration("feedback_config_file"),
                LaunchConfiguration("map_alignment_file"),
                {
                    "input_odometry_topic": LaunchConfiguration(
                        "carla_odometry_topic"),
                    "input_imu_topic": LaunchConfiguration("carla_imu_topic"),
                    "input_gnss_topic": LaunchConfiguration(
                        "carla_gnss_topic"),
                    "input_gnss_right_topic": LaunchConfiguration(
                        "carla_gnss_right_topic"
                    ),
                    "platform_odometry_topic": LaunchConfiguration(
                        "platform_odometry_topic"),
                    "publish_platform_odometry": ParameterValue(
                        LaunchConfiguration("publish_platform_odometry"),
                        value_type=bool,
                    ),
                    "publish_metric_pose": ParameterValue(
                        LaunchConfiguration("publish_metric_pose"),
                        value_type=bool,
                    ),
                    "relay_imu": ParameterValue(
                        LaunchConfiguration("relay_imu"), value_type=bool
                    ),
                    "relay_gnss": ParameterValue(
                        LaunchConfiguration("relay_gnss"), value_type=bool
                    ),
                    "publish_ground_truth_localization": ParameterValue(
                        LaunchConfiguration(
                            "publish_ground_truth_localization"),
                        value_type=bool,
                    ),
                    "publish_ground_truth_tf": ParameterValue(
                        LaunchConfiguration("publish_ground_truth_tf"),
                        value_type=bool,
                    ),
                    "use_sim_time": False,
                },
            ],
        ),
    ])


if __name__ == "__main__":
    generate_launch_description()
