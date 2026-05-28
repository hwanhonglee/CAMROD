import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


CHECKER_NODE_SPECS = (
    # category, executable, name, param_file
    ("hw", "hw_checker_node", "hw_checker", "hw_gpu_checker.yaml"),
    ("hw", "gpu_checker_node", "gpu_checker", "hw_gpu_checker.yaml"),
    ("hw", "network_checker_node", "network_checker", "network_checker.yaml"),
    ("sensing", "gnss_checker_node", "gnss_checker", "gnss_checker.yaml"),
    ("sensing", "imu_checker_node", "imu_checker", "imu_checker.yaml"),
    ("sensing", "lidar_checker_node", "lidar_checker", "lidar_checker.yaml"),
    ("sensing", "radar_checker_node", "radar_checker", "radar_checker.yaml"),
    ("sensing", "camera_checker_node", "camera_checker", "camera_checker.yaml"),
    ("sensing", "wheel_odometry_checker_node", "wheel_odometry_checker", "wheel_odometry_checker.yaml"),
    ("sensing", "cost_grid_checker_node", "cost_grid_checker", "cost_grid_checker.yaml"),
    ("sensing", "velocity_converter_checker_node", "velocity_converter_checker", "velocity_converter_checker.yaml"),
    ("localization", "localization_gnss_checker_node", "localization_gnss_checker", "localization_gnss_checker.yaml"),
    ("localization", "localization_mode_checker_node", "localization_mode_checker", "localization_mode_checker.yaml"),
    ("localization", "localization_pose_checker_node", "localization_pose_checker", "localization_pose_checker.yaml"),
    ("localization", "localization_init_checker_node", "localization_init_checker", "localization_init_checker.yaml"),
    ("localization", "localization_source_checker_node", "localization_source_checker", "localization_source_checker.yaml"),
    ("localization", "localization_lanelet_checker_node", "localization_lanelet_checker", "localization_lanelet_checker.yaml"),
    ("map", "map_cost_grid_checker_node", "map_cost_grid_checker", "map_cost_grid_checker.yaml"),
    ("perception", "perception_obstacle_checker_node", "perception_obstacle_checker", "perception_obstacle_checker.yaml"),
    ("planning", "planning_lifecycle_checker_node", "planning_lifecycle_checker", "planning_lifecycle_checker.yaml"),
    ("planning", "planning_costmap_checker_node", "planning_costmap_checker", "planning_costmap_checker.yaml"),
    ("planning", "planning_nav_status_checker_node", "planning_nav_status_checker", "planning_nav_status_checker.yaml"),
    ("planning", "planning_path_checker_node", "planning_path_checker", "planning_path_checker.yaml"),
)


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _checker_node(config_dir: str, category: str, executable: str, name: str, param_file: str) -> Node:
    return Node(
        package="camrod_system",
        executable=executable,
        name=name,
        output="screen",
        parameters=[os.path.join(config_dir, category, param_file)],
    )


def _build_checker_nodes(config_dir: str) -> list:
    nodes = [
        # Main diagnostics aggregator used by the system health state machine.
        Node(
            package="camrod_system",
            executable="aggregator_node",
            name="diagnostics_agg",
            output="screen",
            parameters=[{"config_file": os.path.join(config_dir, "aggregator", "diagnostics_config.yaml")}],
        ),
    ]
    nodes.extend(_checker_node(config_dir, *spec) for spec in CHECKER_NODE_SPECS)
    return nodes


def _build_diagnostics_inline(context, *_args, **_kwargs):
    pkg_share_dir = get_package_share_directory("camrod_system")
    pkg_prefix = get_package_prefix("camrod_system")
    module_namespace = LaunchConfiguration("module_namespace").perform(context)
    config_profile = LaunchConfiguration("config_profile").perform(context)
    enable_checkers = _as_bool(LaunchConfiguration("enable_checkers").perform(context))
    enable_platform = _as_bool(LaunchConfiguration("enable_platform").perform(context))

    config_root = os.path.join(pkg_share_dir, "config", "diagnostics")
    profile_dir = os.path.join(config_root, config_profile)
    config_dir = profile_dir if os.path.isdir(profile_dir) else os.path.join(config_root, "default")

    if not enable_checkers:
        return [
            LogInfo(
                msg="[camrod_system] diagnostics checkers disabled (enable_checkers:=false)"
            )
        ]

    diagnostics_nodes = _build_checker_nodes(config_dir)

    ranger_checker_exec = os.path.join(
        pkg_prefix, "lib", "camrod_system", "ranger_platform_checker_node"
    )
    if enable_platform and os.path.exists(ranger_checker_exec):
        diagnostics_nodes.append(
            Node(
                package="camrod_system",
                executable="ranger_platform_checker_node",
                name="ranger_platform_checker",
                output="screen",
                parameters=[os.path.join(config_dir, "platform", "ranger_platform_checker.yaml")],
            )
        )
    elif enable_platform:
        diagnostics_nodes.append(
            LogInfo(
                msg=(
                    "[camrod_system] enable_platform:=true but "
                    "ranger_platform_checker_node is not installed. Skip platform checker."
                )
            )
        )
    else:
        diagnostics_nodes.append(
            LogInfo(
                msg="[camrod_system] platform checker disabled (enable_platform:=false)"
            )
        )

    return [
        GroupAction(
            actions=[
                PushRosNamespace(module_namespace),
                SetRemap(src="/diagnostics", dst="diagnostics"),
                SetRemap(src="/diagnostics_agg", dst="diagnostics_agg"),
                *diagnostics_nodes,
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('module_namespace', default_value='system'),
        DeclareLaunchArgument('config_profile', default_value='default'),
        DeclareLaunchArgument('enable_checkers', default_value='true'),
        DeclareLaunchArgument('enable_platform', default_value='false'),
        # system_checker + system_diagnostic + diagnostics_aggregator (tools channel)
        DeclareLaunchArgument('enable_system_tools', default_value='true'),
        DeclareLaunchArgument(
            'system_checker_param_file',
            default_value=pkg_share('camrod_system', os.path.join('config', 'system_checker.yaml')),
        ),

        # HH_260527: Main diagnostics stack is fully inline
        # (legacy system_diagnostics/component launch files removed).
        OpaqueFunction(function=_build_diagnostics_inline),

        # ── System tools: node/topic liveness check + lightweight aggregator ────
        GroupAction(
            condition=IfCondition(LaunchConfiguration('enable_system_tools')),
            actions=[
                PushRosNamespace(LaunchConfiguration('module_namespace')),
                Node(
                    package='camrod_system',
                    executable='system_checker_node',
                    name='system_checker',
                    output='screen',
                    parameters=[
                        LaunchConfiguration('system_checker_param_file'),
                        {'diagnostic_topic': 'diagnostics'},
                    ],
                ),
                Node(
                    package='camrod_system',
                    executable='system_diagnostic_node',
                    name='system_diagnostic',
                    output='screen',
                    parameters=[{
                        'diagnostic_topic': 'diagnostics',
                        'source_diagnostic_topic': 'diagnostics',
                        'publish_period_s': 0.5,
                        'stale_timeout_s': 2.0,
                    }],
                ),
                Node(
                    package='camrod_system',
                    executable='diagnostics_aggregator_node',
                    name='diagnostics_aggregator',
                    output='screen',
                    parameters=[{
                        'source_topic': 'diagnostics',
                        'output_topic': 'diagnostics_agg_tools',
                        'publish_period_s': 1.0,
                        'stale_timeout_s': 3.0,
                    }],
                ),
            ],
        ),
    ])
