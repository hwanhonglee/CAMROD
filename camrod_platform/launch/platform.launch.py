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
        DeclareLaunchArgument("base_frame_id",             default_value="robot_center_link"),
        DeclareLaunchArgument("rear_axle_frame_id",        default_value="robot_base_link"),
        DeclareLaunchArgument("sensor_kit_base_frame_id",  default_value="sensor_kit_base_link"),
        DeclareLaunchArgument("params_file",               default_value=pkg_share("camrod_sensor_kit", os.path.join("config", "robot_params.yaml"))),
        DeclareLaunchArgument("robot_visualization_param_file", default_value=plat(os.path.join("config", "robot_visualization.yaml"))),
        # HH_260720 - Keep the platform namespace independent from nested module arguments.
        DeclareLaunchArgument("platform_namespace",        default_value="platform"),
        DeclareLaunchArgument("sensor_kit_namespace",      default_value="sensor_kit"),
        # HH_260527: Removed unused compatibility args
        # (enable_module_validator, system_namespace).
        # HH_260528: Select platform type profile.
        #   ranger: launch Ranger CAN driver path
        #   rmp401: skip Ranger CAN path, keep external /rmp401 topics
        DeclareLaunchArgument("platform_type",             default_value="ranger"),
        DeclareLaunchArgument("ranger_driver_enable",      default_value="true"),
        # HH_260729 - Keep raw platform topic contracts available for
        # hardware-disabled tests without ever presenting a drivable state.
        # Set false when another simulator already owns the raw Ranger topics.
        DeclareLaunchArgument(
            "ranger_dummy_when_disabled", default_value="true"
        ),
        DeclareLaunchArgument(
            "ranger_dummy_publish_rate_hz", default_value="5.0"
        ),
        # HH_260528: Toggle Ranger status bridge independently from CAN driver.
        DeclareLaunchArgument("ranger_bridge_enable",      default_value="true"),
        DeclareLaunchArgument("ranger_auto_setup_can",     default_value="true"),
        DeclareLaunchArgument("ranger_can_bitrate",        default_value="500000"),
        DeclareLaunchArgument("ranger_can_restart_ms",     default_value="100"),
        DeclareLaunchArgument("ranger_params_file",        default_value=plat(os.path.join("config", "ranger_driver.yaml"))),
        # HH_260528: Keep sensor_kit bridge optional for debug.
        DeclareLaunchArgument("sensor_kit_bridge_enable",  default_value="true"),
        # 260708: Exterior lights (headlight relay + WS2815 indicators via MCU).
        DeclareLaunchArgument("lights_enable",             default_value="true"),
        DeclareLaunchArgument("lights_mcu_bridge_enable",  default_value="true"),
        DeclareLaunchArgument("lights_param_file",         default_value=plat(os.path.join("config", "lights.yaml"))),

        _inc(plat(os.path.join("launch", "robot_visualization.launch.py")),
             "map_frame_id", "base_frame_id", "params_file", "robot_visualization_param_file",
             # HH_260720 - Pass the platform namespace explicitly to the visualization child.
             module_namespace=LaunchConfiguration("platform_namespace")),

        _inc(plat(os.path.join("launch", "ranger.launch.py")),
             "platform_type",
             enable_ranger_base_node=LaunchConfiguration("ranger_driver_enable"),
             enable_ranger_dummy_when_disabled=LaunchConfiguration(
                 "ranger_dummy_when_disabled"
             ),
             dummy_publish_rate_hz=LaunchConfiguration(
                 "ranger_dummy_publish_rate_hz"
             ),
             enable_ranger_bridge_node=LaunchConfiguration("ranger_bridge_enable"),
             auto_setup_can=LaunchConfiguration("ranger_auto_setup_can"),
             can_bitrate=LaunchConfiguration("ranger_can_bitrate"),
             can_restart_ms=LaunchConfiguration("ranger_can_restart_ms"),
             params_file=LaunchConfiguration("ranger_params_file")),

        _inc(plat(os.path.join("launch", "sensor_kit_bridge.launch.py")),
             "base_frame_id", "rear_axle_frame_id", "sensor_kit_base_frame_id",
             "params_file", "sensor_kit_namespace",
             condition=IfCondition(LaunchConfiguration("sensor_kit_bridge_enable"))),

        _inc(plat(os.path.join("launch", "lights.launch.py")),
             "lights_enable", "lights_mcu_bridge_enable", "lights_param_file",
             # HH_260720 - Isolate the platform light namespace from sensor-kit launch arguments.
             lights_namespace=LaunchConfiguration("platform_namespace")),
    ])
