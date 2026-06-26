import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def _inc(path, *through, condition=None, **overrides):
    args = {k: LaunchConfiguration(k) for k in through}
    args.update(overrides)
    kw = {"launch_arguments": args.items()}
    if condition is not None:
        kw["condition"] = condition
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), **kw)


def generate_launch_description():
    def plat(rel): return pkg_share("camrod_platform", rel)

    return LaunchDescription([
        DeclareLaunchArgument("map_frame_id",              default_value="map"),
        DeclareLaunchArgument("base_frame_id",             default_value="robot_base_link"),
        DeclareLaunchArgument("sensor_kit_base_frame_id",  default_value="sensor_kit_base_link"),
        DeclareLaunchArgument("params_file",               default_value=pkg_share("camrod_sensor_kit", os.path.join("config", "robot_params.yaml"))),
        DeclareLaunchArgument("robot_visualization_param_file", default_value=plat(os.path.join("config", "robot_visualization.yaml"))),
        DeclareLaunchArgument("module_namespace",          default_value="platform"),
        DeclareLaunchArgument("sensor_kit_namespace",      default_value="sensor_kit"),
        # HH_260527: Removed unused compatibility args
        # (enable_module_validator, system_namespace).
        DeclareLaunchArgument("cmd_vel_gate_enable",       default_value="true"),
        DeclareLaunchArgument("cmd_vel_in_topic",          default_value="/planning/cmd_vel"),
        DeclareLaunchArgument("cmd_vel_out_topic",         default_value="/platform/cmd_vel"),
        DeclareLaunchArgument("drive_enable_topic",        default_value="/platform/drive_enable"),
        # HH_260625 - Platform listens to the effective planning engage state.
        # Raw /planning/engage remains manual-only; UI missions use /planning/mission_engage.
        DeclareLaunchArgument("platform_planning_engage_topic", default_value="/planning/engaged"),
        # HH_260522: unified source selector for engage signal.
        #   planning_engage/planning_engaged/topic/enabled/on: subscribe configured topic
        #   disabled/off/none: ignore engage topic
        DeclareLaunchArgument("engage_source_mode",        default_value="planning_engaged"),
        DeclareLaunchArgument("drive_state_topic",         default_value="/platform/drive_enabled"),
        # HH_260522: unified source selector for e-stop signal.
        #   platform_status/topic/enabled/on: subscribe /platform/status/estop
        #   disabled/off/none: ignore e-stop topic
        DeclareLaunchArgument("estop_source_mode",         default_value="platform_status"),
        # HH_260410: Use Ranger CAN derived /platform/status/estop as the default gate source.
        DeclareLaunchArgument("estop_topic",               default_value="/platform/status/estop"),
        DeclareLaunchArgument("drive_allow_on_start",      default_value="false"),
        DeclareLaunchArgument("cmd_vel_input_timeout_s",   default_value="0.50"),
        DeclareLaunchArgument("cmd_vel_zero_publish_rate_hz", default_value="10.0"),
        # HH_260528: Select platform type profile.
        #   ranger: launch Ranger CAN driver path
        #   rmp401: skip Ranger CAN path, keep external /rmp401 topics
        DeclareLaunchArgument("platform_type",             default_value="ranger"),
        DeclareLaunchArgument("ranger_driver_enable",      default_value="true"),
        # HH_260528: Toggle Ranger status bridge independently from CAN driver.
        DeclareLaunchArgument("ranger_bridge_enable",      default_value="true"),
        DeclareLaunchArgument("ranger_params_file",        default_value=plat(os.path.join("config", "ranger_driver.yaml"))),
        # HH_260528: Keep sensor_kit bridge optional for debug.
        DeclareLaunchArgument("sensor_kit_bridge_enable",  default_value="true"),

        _inc(plat(os.path.join("launch", "robot_visualization.launch.py")),
             "module_namespace", "map_frame_id", "base_frame_id",
             "params_file", "robot_visualization_param_file"),

        _inc(plat(os.path.join("launch", "cmd_vel_gate.launch.py")),
             "module_namespace", "cmd_vel_gate_enable",
             "cmd_vel_in_topic", "cmd_vel_out_topic",
             "drive_enable_topic", "platform_planning_engage_topic", "engage_source_mode",
             "drive_state_topic", "estop_source_mode", "estop_topic", "drive_allow_on_start",
             "cmd_vel_input_timeout_s", "cmd_vel_zero_publish_rate_hz"),

        _inc(plat(os.path.join("launch", "ranger.launch.py")),
             "platform_type",
             enable_ranger_base_node=LaunchConfiguration("ranger_driver_enable"),
             enable_ranger_bridge_node=LaunchConfiguration("ranger_bridge_enable"),
             params_file=LaunchConfiguration("ranger_params_file")),

        _inc(plat(os.path.join("launch", "sensor_kit_bridge.launch.py")),
             "base_frame_id", "sensor_kit_base_frame_id",
             "params_file", "sensor_kit_namespace",
             condition=IfCondition(LaunchConfiguration("sensor_kit_bridge_enable"))),
    ])
