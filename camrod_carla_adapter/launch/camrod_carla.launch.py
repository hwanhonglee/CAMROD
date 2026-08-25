"""Compose CAMROD map/localization/planning with the accepted CARLA 4WS stack.

The CARLA server, carla_ros_bridge and ego spawn are intentionally external
lifecycle owners. This launch starts only the algorithms, adapters and vehicle
controller, so stopping it cannot silently destroy a shared CARLA world.
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
        PythonLaunchDescriptionSource(path), **kwargs)


def generate_launch_description():
    adapter_share = get_package_share_directory("camrod_carla_adapter")
    controller_share = get_package_share_directory(
        "carla_extended_ackermann_control")
    localization_share = get_package_share_directory("camrod_localization")
    map_share = get_package_share_directory("camrod_map")
    planning_share = get_package_share_directory("camrod_planning")
    control_share = get_package_share_directory("camrod_control")

    adapter_launch = os.path.join(adapter_share, "launch", "adapter.launch.py")
    controller_launch = os.path.join(
        controller_share, "full_stack.launch.py")
    localization_launch = os.path.join(
        localization_share, "launch", "localization.launch.py")
    map_launch = os.path.join(map_share, "launch", "map.launch.py")
    planning_launch = os.path.join(
        planning_share, "launch", "planning.launch.py")
    safety_launch = os.path.join(
        control_share, "launch", "cmd_vel_safety_gate.launch.py")

    input_adapter_config = os.path.join(
        adapter_share, "config", "camrod_input_adapter_carla.yaml")
    safety_config = os.path.join(
        adapter_share, "config", "camrod_safety_gate_carla.yaml")
    alignment_config = os.path.join(
        adapter_share, "config", "woraksan_lane_anchor_alignment.yaml")
    ekf_sim_config = os.path.join(
        localization_share, "config", "filter", "ekf_sim.yaml")
    gnss_reattach_sim_config = os.path.join(
        localization_share, "config", "filter", "gnss_reattach_sim.yaml")
    nav_through_poses_bt = os.path.join(
        planning_share,
        "config",
        "bt",
        "navigate_through_poses_w_planner_selector.xml",
    )

    default_baseline = _environment_path(
        "RANGER_BASELINE_MANIFEST",
        os.path.join(".work", "evidence", "ranger_ros_backend_gate.json"),
    )
    default_physical_gate = _environment_path(
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

    launch_vehicle_control = LaunchConfiguration("launch_vehicle_control")
    launch_camrod_map = LaunchConfiguration("launch_camrod_map")
    launch_camrod_localization = LaunchConfiguration(
        "launch_camrod_localization")
    launch_camrod_planning = LaunchConfiguration("launch_camrod_planning")
    launch_camrod_safety_gate = LaunchConfiguration(
        "launch_camrod_safety_gate")

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
        DeclareLaunchArgument("launch_camrod_map", default_value="true"),
        DeclareLaunchArgument(
            "launch_camrod_localization", default_value="true"),
        DeclareLaunchArgument("launch_camrod_planning", default_value="true"),
        DeclareLaunchArgument(
            "launch_camrod_safety_gate", default_value="true"),
        DeclareLaunchArgument(
            "nav2_selected_planner", default_value="LaneletRoute"),
        DeclareLaunchArgument(
            "nav2_selected_controller", default_value="RPP"),
        # nav2_lanelet.launch.py consumes this inherited launch configuration.
        # Pin a BT whose plugin IDs match the production planner/controller
        # set.
        DeclareLaunchArgument(
            "nav2_bt_xml_nav_through_poses",
            default_value=nav_through_poses_bt,
        ),
        DeclareLaunchArgument(
            "map_alignment_file", default_value=alignment_config),
        DeclareLaunchArgument(
            "camrod_map_path", default_value=camrod_map_path),
        DeclareLaunchArgument(
            "camrod_input_adapter_config", default_value=input_adapter_config),
        DeclareLaunchArgument(
            "camrod_safety_gate_config", default_value=safety_config),
        DeclareLaunchArgument(
            "camrod_ekf_config", default_value=ekf_sim_config),
        DeclareLaunchArgument(
            "verified_baseline_manifest", default_value=default_baseline),
        DeclareLaunchArgument(
            "verified_physical_four_wheel_manifest",
            default_value=default_physical_gate,
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
                    "verified_baseline_manifest"),
                "verified_physical_four_wheel_manifest": LaunchConfiguration(
                    "verified_physical_four_wheel_manifest"),
                "extended_mode_backend": LaunchConfiguration(
                    "extended_mode_backend"),
                "accepted_carla_python_egg": LaunchConfiguration(
                    "accepted_carla_python_egg"),
                "python_egg_cache": LaunchConfiguration("python_egg_cache"),
            },
            condition=IfCondition(launch_vehicle_control),
        ),
        _include(
            adapter_launch,
            {
                "role_name": LaunchConfiguration("role_name"),
                "map_alignment_file": LaunchConfiguration(
                    "map_alignment_file"),
                "publish_ground_truth_localization": "false",
                "publish_ground_truth_tf": "false",
            },
        ),
        _include(
            map_launch,
            {
                "module_namespace": "map",
                "map_path": LaunchConfiguration("camrod_map_path"),
                "enable_cost_grids": "true",
                "enable_cost_field": "false",
                "enable_inflation_markers": "false",
                "enable_map_cost_markers": "false",
            },
            condition=IfCondition(launch_camrod_map),
        ),
        _include(
            localization_launch,
            {
                "module_namespace": "localization",
                "enable_adapter": "true",
                "enable_filter": "true",
                "enable_monitor": "false",
                "enable_map_helper": "false",
                "adapter_param_file": LaunchConfiguration(
                    "camrod_input_adapter_config"),
                "filter_ekf_param_file": LaunchConfiguration(
                    "camrod_ekf_config"),
                "filter_gnss_reattach_param_file": gnss_reattach_sim_config,
                "wheel_bridge_enable": "true",
                "wheel_input_topic": "/platform/status/odometry",
                "wheel_input_type": "nav_odom",
                "wheel_fallback_input_topic": "",
                "wheel_fallback_input_type": "nav_odom",
                "ekf_publish_map_to_odom_static_tf": "true",
            },
            condition=IfCondition(launch_camrod_localization),
        ),
        _include(
            planning_launch,
            {
                "module_namespace": "planning",
                "map_path": LaunchConfiguration("camrod_map_path"),
                "enable_path_cost_grids": "false",
                "enable_goal_replanner": "false",
                "enable_nav2_lifecycle_retry": "false",
                "require_localization_ready": "false",
                "enable_state_machine": "false",
                "enable_obstacle_replan_monitor": "false",
                "navigation_cmd_vel_topic": "/control/nav2_cmd_vel_ros",
                # Pin the production selector IDs explicitly so CARLA behavior
                # remains deterministic even when standalone planning defaults
                # or opt-in combo profiles change.
                "nav2_selected_planner": LaunchConfiguration(
                    "nav2_selected_planner"),
                "nav2_selected_controller": LaunchConfiguration(
                    "nav2_selected_controller"),
                "nav2_bt_xml_nav_through_poses": LaunchConfiguration(
                    "nav2_bt_xml_nav_through_poses"),
            },
            condition=IfCondition(launch_camrod_planning),
        ),
        _include(
            safety_launch,
            {
                "module_namespace": "control",
                "parameter_file": LaunchConfiguration(
                    "camrod_safety_gate_config"),
                "cmd_vel_gate_enable": "true",
                "navigation_cmd_vel_ros_topic": "/control/nav2_cmd_vel_ros",
                "cmd_vel_ros_output_topic": "/control/cmd_vel_ros",
            },
            condition=IfCondition(launch_camrod_safety_gate),
        ),
    ]

    return LaunchDescription(declarations + actions)


if __name__ == "__main__":
    generate_launch_description()
