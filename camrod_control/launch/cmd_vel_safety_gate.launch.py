import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# HH_260720 - Only names that differ between bringup arguments and node
# parameters need an explicit mapping. Every other cmd_vel_gate_* override maps
# directly after removing that prefix.
PARAMETER_NAME_OVERRIDES = {
    "cost_stop_enable": "enable_cost_stop",
    "cost_pose_topic": "pose_topic",
    "cost_odometry_topic": "odometry_topic",
    "cost_threshold": "cost_stop_threshold",
    "cost_lookahead_m": "cost_stop_lookahead_m",
    "cost_width_m": "cost_stop_width_m",
    "cost_hold_s": "cost_stop_hold_s",
    "speed_dependent_lookahead": "enable_speed_dependent_lookahead",
    "side_rear_cost_stop": "enable_side_rear_cost_stop",
    "body_near_dynamic_stop": "enable_body_near_dynamic_stop",
    "unavoidable_stop_enable": "enable_unavoidable_stop",
    "yaw_alignment_enable": "enable_yaw_alignment_zone",
    "route_heading_enable": "enable_route_heading_alignment",
}


def _create_gate_node(context):
    # HH_260720 - Preserve bringup-level field overrides without declaring the
    # same 100+ defaults a second time in this launch file.
    parameter_overrides = {
        "input_topic": LaunchConfiguration("cmd_vel_raw_topic"),
        "navigation_input_topic": LaunchConfiguration("navigation_cmd_vel_ros_topic"),
        "manual_input_topic": LaunchConfiguration("manual_cmd_vel_ros_topic"),
        "output_topic": LaunchConfiguration("cmd_vel_output_topic"),
        "ros_output_topic": LaunchConfiguration("cmd_vel_ros_output_topic"),
        "engage_topic": LaunchConfiguration("planning_engage_topic"),
        "mission_engage_topic": LaunchConfiguration("planning_mission_engage_topic"),
        "state_topic": LaunchConfiguration("command_enabled_topic"),
        "platform_drive_enable_topic": LaunchConfiguration("platform_drive_enable_topic"),
        "publish_zero_when_blocked": True,
    }

    for argument_name in sorted(context.launch_configurations):
        if not argument_name.startswith("cmd_vel_gate_"):
            continue
        if argument_name == "cmd_vel_gate_enable":
            continue
        suffix = argument_name[len("cmd_vel_gate_") :]
        parameter_name = PARAMETER_NAME_OVERRIDES.get(suffix, suffix)
        parameter_overrides[parameter_name] = LaunchConfiguration(argument_name)

    return [
        Node(
            package="camrod_control",
            executable="cmd_vel_safety_gate_node",
            name="cmd_vel_safety_gate",
            namespace=LaunchConfiguration("module_namespace"),
            output="screen",
            # HH_260720 - Respawn restores blocked zero-command publication
            # after a transient process failure.
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                LaunchConfiguration("parameter_file"),
                parameter_overrides,
            ],
            condition=IfCondition(LaunchConfiguration("cmd_vel_gate_enable")),
        )
    ]


def generate_launch_description():
    control_share = get_package_share_directory("camrod_control")
    default_parameter_file = os.path.join(
        control_share, "config", "cmd_vel_safety_gate.yaml"
    )
    default_yaw_zone_file = os.path.join(
        control_share, "config", "yaw_alignment_zones.yaml"
    )

    # HH_260720 - Launch arguments now describe topology and file selection.
    # Safety tuning belongs to the dedicated YAML file.
    return LaunchDescription(
        [
            DeclareLaunchArgument("module_namespace", default_value="control"),
            DeclareLaunchArgument("parameter_file", default_value=default_parameter_file),
            DeclareLaunchArgument("cmd_vel_gate_enable", default_value="true"),
            DeclareLaunchArgument("cmd_vel_raw_topic", default_value="/control/cmd_vel_raw"),
            DeclareLaunchArgument(
                "navigation_cmd_vel_ros_topic",
                default_value="/control/nav2_cmd_vel_ros",
            ),
            # Empty by default: ordinary CAMROD retains its existing Nav2/raw
            # command ownership. External simulators may opt into a dedicated
            # operator Twist boundary.
            DeclareLaunchArgument("manual_cmd_vel_ros_topic", default_value=""),
            DeclareLaunchArgument("cmd_vel_output_topic", default_value="/control/cmd_vel"),
            DeclareLaunchArgument(
                "cmd_vel_ros_output_topic", default_value="/control/cmd_vel_ros"
            ),
            DeclareLaunchArgument("planning_engage_topic", default_value="/planning/engage"),
            DeclareLaunchArgument(
                "planning_mission_engage_topic",
                default_value="/planning/mission_engage",
            ),
            DeclareLaunchArgument(
                "command_enabled_topic", default_value="/control/command_enabled"
            ),
            DeclareLaunchArgument(
                "platform_drive_enable_topic", default_value="/platform/drive_enable"
            ),
            DeclareLaunchArgument(
                "cmd_vel_gate_yaw_alignment_zones_file",
                default_value=default_yaw_zone_file,
            ),
            OpaqueFunction(function=_create_gate_node),
        ]
    )
