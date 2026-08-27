"""Run the complete CAMROD algorithms and production UI on a CARLA plant.

CARLA server, carla_ros_bridge, and actor spawning remain explicit lifecycle
owners. This launch owns only CAMROD, the physical Ranger controller, and the
conversion boundaries, so stopping it cannot destroy a shared CARLA world.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _environment_path(name, ranger_relative_path=None, default=""):
    """Resolve a portable launch default without embedding a host home path."""
    configured = os.environ.get(name, "").strip()
    if configured:
        return os.path.abspath(os.path.expanduser(configured))

    ranger_root = os.environ.get("RANGER_CARLA_ROOT", "").strip()
    if ranger_root and ranger_relative_path:
        return os.path.abspath(
            os.path.join(
                os.path.expanduser(ranger_root), ranger_relative_path
            )
        )
    return default


def _include(path, arguments, condition=None):
    kwargs = {"launch_arguments": arguments.items()}
    if condition is not None:
        kwargs["condition"] = condition
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), **kwargs
    )


def generate_launch_description():
    adapter_share = get_package_share_directory("camrod_carla_adapter")
    controller_share = get_package_share_directory(
        "carla_extended_ackermann_control"
    )
    bringup_share = get_package_share_directory("camrod_bringup")
    planning_share = get_package_share_directory("camrod_planning")

    adapter_launch = os.path.join(
        adapter_share, "launch", "adapter.launch.py"
    )
    sensor_relay_launch = os.path.join(
        adapter_share, "launch", "sensor_relay.launch.py"
    )
    platform_heartbeat_launch = os.path.join(
        adapter_share, "launch", "platform_heartbeat.launch.py"
    )
    # carla_extended_ackermann_control follows the upstream package layout
    # and installs launch files directly under share/<package> (without a
    # launch/ subdirectory).
    controller_launch = os.path.join(
        controller_share, "full_stack.launch.py"
    )
    bringup_launch = os.path.join(
        bringup_share, "launch", "bringup.launch.py"
    )
    launch_defaults = os.path.join(
        bringup_share, "config", "bringup", "launch_defaults.yaml"
    )
    alignment_config = os.path.join(
        adapter_share, "config", "woraksan_lane_anchor_alignment.yaml"
    )
    nav_through_poses_bt = os.path.join(
        planning_share,
        "config",
        "bt",
        "navigate_through_poses_w_planner_selector.xml",
    )

    baseline_manifest = _environment_path(
        "RANGER_BASELINE_MANIFEST",
        os.path.join(".work", "evidence", "ranger_ros_backend_gate.json"),
    )
    physical_manifest = _environment_path(
        "RANGER_PHYSICAL_MANIFEST",
        os.path.join(
            ".work", "evidence", "ranger_physical_4ws_acceptance_gate.json"
        ),
    )
    accepted_carla_python_egg = _environment_path(
        "CARLA_PYTHON_EGG"
    ) or _environment_path("RANGER_CARLA_PYTHON_EGG")
    python_egg_cache = _environment_path(
        "CARLA_PYTHON_EGG_CACHE"
    ) or _environment_path("RANGER_PYTHON_EGG_CACHE")
    camrod_map_path = _environment_path("CAMROD_LANELET_MAP")
    alignment_config = _environment_path(
        "CAMROD_MAP_ALIGNMENT_FILE", default=alignment_config
    )
    launch_defaults = _environment_path(
        "CAMROD_LAUNCH_DEFAULTS_FILE", default=launch_defaults
    )

    declarations = [
        DeclareLaunchArgument(
            "role_name",
            default_value=os.environ.get("CARLA_ROLE_NAME", "ego_vehicle"),
        ),
        DeclareLaunchArgument(
            "host", default_value=os.environ.get("CARLA_HOST", "127.0.0.1")
        ),
        DeclareLaunchArgument(
            "port", default_value=os.environ.get("CARLA_PORT", "2000")
        ),
        DeclareLaunchArgument("launch_vehicle_control", default_value="true"),
        DeclareLaunchArgument("launch_sensor_relay", default_value="true"),
        DeclareLaunchArgument(
            "launch_platform_heartbeat", default_value="true"
        ),
        DeclareLaunchArgument(
            "map_alignment_file", default_value=alignment_config
        ),
        DeclareLaunchArgument(
            "nav2_bt_xml_nav_through_poses",
            default_value=nav_through_poses_bt,
        ),
        DeclareLaunchArgument(
            "camrod_launch_defaults_file", default_value=launch_defaults
        ),
        DeclareLaunchArgument(
            "camrod_map_path", default_value=camrod_map_path
        ),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("enable_plugin_api", default_value="true"),
        DeclareLaunchArgument("enable_api_ui", default_value="true"),
        DeclareLaunchArgument(
            "enable_operator_ui_window", default_value="true"
        ),
        DeclareLaunchArgument("api_ui_port", default_value="8010"),
        DeclareLaunchArgument(
            "operator_ui_window_url", default_value="http://127.0.0.1:8010"
        ),
        DeclareLaunchArgument("enable_voice", default_value="false"),
        DeclareLaunchArgument("initial_soc", default_value="0.8"),
        DeclareLaunchArgument("initial_charging", default_value="false"),
        DeclareLaunchArgument("initial_estop", default_value="false"),
        DeclareLaunchArgument("initial_error_code", default_value="0"),
        DeclareLaunchArgument("module_launch_gap_s", default_value="0.25"),
        DeclareLaunchArgument(
            "verified_baseline_manifest", default_value=baseline_manifest
        ),
        DeclareLaunchArgument(
            "verified_physical_four_wheel_manifest",
            default_value=physical_manifest,
        ),
        DeclareLaunchArgument(
            "extended_mode_backend",
            default_value="PHYSX_FOUR_WHEEL_STEERING",
        ),
        DeclareLaunchArgument(
            "accepted_carla_python_egg",
            default_value=accepted_carla_python_egg,
        ),
        DeclareLaunchArgument(
            "python_egg_cache", default_value=python_egg_cache
        ),
    ]

    actions = [
        _include(
            controller_launch,
            {
                "role_name": LaunchConfiguration("role_name"),
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "verified_baseline_manifest": LaunchConfiguration(
                    "verified_baseline_manifest"
                ),
                "verified_physical_four_wheel_manifest": LaunchConfiguration(
                    "verified_physical_four_wheel_manifest"
                ),
                "extended_mode_backend": LaunchConfiguration(
                    "extended_mode_backend"
                ),
                "accepted_carla_python_egg": LaunchConfiguration(
                    "accepted_carla_python_egg"
                ),
                "python_egg_cache": LaunchConfiguration("python_egg_cache"),
            },
            condition=IfCondition(
                LaunchConfiguration("launch_vehicle_control")
            ),
        ),
        _include(
            adapter_launch,
            {
                "role_name": LaunchConfiguration("role_name"),
                "map_alignment_file": LaunchConfiguration(
                    "map_alignment_file"
                ),
                # The standard /odom boundary is consumed by the unmodified
                # ranger_platform_bridge, which remains the sole owner of the
                # generated AvgOdometry /platform/status/odometry topic.
                "platform_odometry_topic": "/odom",
                "publish_platform_odometry": "true",
                "publish_metric_pose": "true",
                "publish_ground_truth_localization": "false",
                "publish_ground_truth_tf": "false",
                # CAMROD's external-simulator fake-sensor contract derives
                # GNSS/IMU from the same mapped CARLA pose; avoid duplicates.
                "relay_imu": "false",
                "relay_gnss": "false",
            },
        ),
        _include(
            sensor_relay_launch,
            {"role_name": LaunchConfiguration("role_name")},
            condition=IfCondition(LaunchConfiguration("launch_sensor_relay")),
        ),
        _include(
            platform_heartbeat_launch,
            {
                "initial_soc": LaunchConfiguration("initial_soc"),
                "initial_charging": LaunchConfiguration("initial_charging"),
                "initial_estop": LaunchConfiguration("initial_estop"),
                "initial_error_code": LaunchConfiguration(
                    "initial_error_code"
                ),
            },
            condition=IfCondition(
                LaunchConfiguration("launch_platform_heartbeat")
            ),
        ),
        _include(
            bringup_launch,
            {
                "launch_defaults_file": LaunchConfiguration(
                    "camrod_launch_defaults_file"
                ),
                "map_path": LaunchConfiguration("camrod_map_path"),
                "clean_before_launch": "false",
                "clean_on_shutdown": "false",
                "sim": "true",
                "external_simulator": "true",
                "external_simulator_odometry_topic": "/odom",
                "external_simulator_odometry_timeout_s": "0.5",
                # Keep fake GNSS/lidar/perception aligned with CAMROD's 10 Hz
                # readiness contracts. This unique argument cannot inherit the
                # platform heartbeat include's generic 5 Hz publish rate.
                "sim_fake_sensor_publish_rate_hz": "10.0",
                "sim_platform_status_enable": "true",
                "sim_publish_platform_status": "false",
                "platform_ranger_driver_enable": "false",
                "platform_ranger_bridge_enable": "true",
                "planning_nav2_bt_xml_nav_through_poses": LaunchConfiguration(
                    "nav2_bt_xml_nav_through_poses"
                ),
                "rviz": LaunchConfiguration("rviz"),
                "enable_plugin_api": LaunchConfiguration("enable_plugin_api"),
                "enable_api_ui": LaunchConfiguration("enable_api_ui"),
                "enable_operator_ui_window": LaunchConfiguration(
                    "enable_operator_ui_window"
                ),
                "api_ui_port": LaunchConfiguration("api_ui_port"),
                "operator_ui_window_url": LaunchConfiguration(
                    "operator_ui_window_url"
                ),
                "enable_voice": LaunchConfiguration("enable_voice"),
                "module_launch_gap_s": LaunchConfiguration(
                    "module_launch_gap_s"
                ),
            },
        ),
    ]

    return LaunchDescription([*declarations, *actions])


if __name__ == "__main__":
    generate_launch_description()
