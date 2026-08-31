import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def package_path(package_name: str, relative_path: str) -> str:
    return os.path.join(get_package_share_directory(package_name), relative_path)


def generate_launch_description():
    # HH_260720 - Group non-Nav2 motion controllers under the control namespace.
    default_parameter_file = package_path(
        "camrod_control", os.path.join("config", "control.yaml")
    )
    # HH_260807 - Standalone controllers need the same semantic areas that
    # full bringup injects; empty defaults start nodes that cannot serve sites.
    default_drop_zones_yaml = package_path(
        "camrod_map", os.path.join("config", "drop_zones.yaml")
    )
    default_camping_sites_yaml = package_path(
        "camrod_planning", os.path.join("config", "camping_sites.yaml")
    )

    return LaunchDescription([
        DeclareLaunchArgument("control_namespace", default_value="control"),
        DeclareLaunchArgument("parameter_file", default_value=default_parameter_file),
        DeclareLaunchArgument("enable_camping_site_maneuver_controller", default_value="true"),
        DeclareLaunchArgument("enable_campsite_occupancy_guard", default_value="false"),
        DeclareLaunchArgument(
            "crab_approach_slowdown_distance_m", default_value="0"
        ),
        DeclareLaunchArgument(
            "crab_approach_min_speed_mps", default_value="0"
        ),
        DeclareLaunchArgument("rotate_180_timeout_s", default_value="0"),
        DeclareLaunchArgument(
            "camping_site_entry_position_tolerance_m", default_value="0.15"
        ),
        DeclareLaunchArgument(
            "camping_site_rotate_entry_max_position_error_m",
            default_value="0",
        ),
        DeclareLaunchArgument(
            "entry_anchor_centering_max_initial_error_m", default_value="0"
        ),
        DeclareLaunchArgument(
            "entry_anchor_centering_max_speed_mps", default_value="0.12"
        ),
        DeclareLaunchArgument(
            "entry_anchor_centering_timeout_s", default_value="15"
        ),
        DeclareLaunchArgument(
            "entry_anchor_centering_tolerance_m", default_value="0.05"
        ),
        DeclareLaunchArgument(
            "crab_entry_max_heading_drift_deg", default_value="0"
        ),
        DeclareLaunchArgument(
            "crab_entry_max_cross_track_error_m", default_value="0"
        ),
        DeclareLaunchArgument(
            "crab_entry_body_yaw_compensation_deg", default_value="0"
        ),
        DeclareLaunchArgument(
            "crab_entry_body_yaw_alignment_tolerance_deg", default_value="0.5"
        ),
        DeclareLaunchArgument(
            "crab_entry_body_yaw_alignment_timeout_s", default_value="15"
        ),
        DeclareLaunchArgument(
            "roadside_reverse_return_enable", default_value="false"
        ),
        DeclareLaunchArgument(
            "roadside_reverse_handoff_distance_m", default_value="0.03"
        ),
        DeclareLaunchArgument("enable_drop_zone_maneuver_controller", default_value="true"),
        DeclareLaunchArgument("enable_route_safety_recovery_controller", default_value="true"),
        DeclareLaunchArgument("command_topic", default_value="/control/cmd_vel_raw"),
        DeclareLaunchArgument("vehicle_pose_topic", default_value="/localization/pose"),
        DeclareLaunchArgument("drop_zones_yaml", default_value=default_drop_zones_yaml),
        DeclareLaunchArgument("camping_sites_yaml", default_value=default_camping_sites_yaml),

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
                    # HH_260818 - Share the same semantic occupancy policy with
                    # UI mission admission in full bringup.
                    "enable_campsite_occupancy_guard": ParameterValue(
                        LaunchConfiguration("enable_campsite_occupancy_guard"),
                        value_type=bool,
                    ),
                    "crab_approach_slowdown_distance_m": ParameterValue(
                        LaunchConfiguration(
                            "crab_approach_slowdown_distance_m"
                        ),
                        value_type=float,
                    ),
                    "crab_approach_min_speed_mps": ParameterValue(
                        LaunchConfiguration("crab_approach_min_speed_mps"),
                        value_type=float,
                    ),
                    "rotate_180_timeout_s": ParameterValue(
                        LaunchConfiguration("rotate_180_timeout_s"),
                        value_type=float,
                    ),
                    "entry_position_tolerance_m": ParameterValue(
                        LaunchConfiguration(
                            "camping_site_entry_position_tolerance_m"
                        ),
                        value_type=float,
                    ),
                    "rotate_entry_max_position_error_m": ParameterValue(
                        LaunchConfiguration(
                            "camping_site_rotate_entry_max_position_error_m"
                        ),
                        value_type=float,
                    ),
                    "entry_anchor_centering_max_initial_error_m": ParameterValue(
                        LaunchConfiguration(
                            "entry_anchor_centering_max_initial_error_m"
                        ),
                        value_type=float,
                    ),
                    "entry_anchor_centering_max_speed_mps": ParameterValue(
                        LaunchConfiguration(
                            "entry_anchor_centering_max_speed_mps"
                        ),
                        value_type=float,
                    ),
                    "entry_anchor_centering_timeout_s": ParameterValue(
                        LaunchConfiguration("entry_anchor_centering_timeout_s"),
                        value_type=float,
                    ),
                    "entry_anchor_centering_tolerance_m": ParameterValue(
                        LaunchConfiguration(
                            "entry_anchor_centering_tolerance_m"
                        ),
                        value_type=float,
                    ),
                    "crab_entry_max_heading_drift_deg": ParameterValue(
                        LaunchConfiguration("crab_entry_max_heading_drift_deg"),
                        value_type=float,
                    ),
                    "crab_entry_max_cross_track_error_m": ParameterValue(
                        LaunchConfiguration(
                            "crab_entry_max_cross_track_error_m"
                        ),
                        value_type=float,
                    ),
                    "crab_entry_body_yaw_compensation_deg": ParameterValue(
                        LaunchConfiguration(
                            "crab_entry_body_yaw_compensation_deg"
                        ),
                        value_type=float,
                    ),
                    "crab_entry_body_yaw_alignment_tolerance_deg": ParameterValue(
                        LaunchConfiguration(
                            "crab_entry_body_yaw_alignment_tolerance_deg"
                        ),
                        value_type=float,
                    ),
                    "crab_entry_body_yaw_alignment_timeout_s": ParameterValue(
                        LaunchConfiguration(
                            "crab_entry_body_yaw_alignment_timeout_s"
                        ),
                        value_type=float,
                    ),
                    "roadside_reverse_return_enable": ParameterValue(
                        LaunchConfiguration(
                            "roadside_reverse_return_enable"
                        ),
                        value_type=bool,
                    ),
                    # HH_260830 - Keep the hardware 3 cm centerline contract,
                    # while allowing CARLA to stop at its measured 10 cm
                    # brake-safe handoff before a Terrain bank.
                    "roadside_reverse_handoff_distance_m": ParameterValue(
                        LaunchConfiguration(
                            "roadside_reverse_handoff_distance_m"
                        ),
                        value_type=float,
                    ),
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
        Node(
            package="camrod_control",
            executable="route_safety_recovery_controller_node",
            namespace=LaunchConfiguration("control_namespace"),
            name="route_safety_recovery_controller",
            output="screen",
            parameters=[
                LaunchConfiguration("parameter_file"),
                {
                    "command_topic": LaunchConfiguration("command_topic"),
                    "pose_topic": LaunchConfiguration("vehicle_pose_topic"),
                },
            ],
            condition=IfCondition(
                LaunchConfiguration("enable_route_safety_recovery_controller")
            ),
        ),
    ])
