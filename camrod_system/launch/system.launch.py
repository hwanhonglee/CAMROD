import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ComposableNode
import yaml


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
    # HH_260617: PlanningState semantic health is checked as part of system readiness.
    ("planning", "planning_state_checker_node", "planning_state_checker", "planning_state_checker.yaml"),
)

CHECKER_COMPONENT_PLUGINS = {
    "hw_checker": "HwCheckerNode",
    "gpu_checker": "GpuCheckerNode",
    "network_checker": "NetworkCheckerNode",
    "gnss_checker": "GnssCheckerNode",
    "imu_checker": "ImuCheckerNode",
    "lidar_checker": "LidarCheckerNode",
    "radar_checker": "RadarCheckerNode",
    "camera_checker": "CameraCheckerNode",
    "wheel_odometry_checker": "WheelOdometryCheckerNode",
    "cost_grid_checker": "CostGridCheckerNode",
    "velocity_converter_checker": "VelocityConverterCheckerNode",
    "localization_gnss_checker": "LocalizationGnssCheckerNode",
    "localization_mode_checker": "LocalizationModeCheckerNode",
    "localization_pose_checker": "LocalizationPoseCheckerNode",
    "localization_init_checker": "LocalizationInitCheckerNode",
    "localization_source_checker": "LocalizationSourceCheckerNode",
    "localization_lanelet_checker": "LocalizationLaneletCheckerNode",
    "map_cost_grid_checker": "MapCostGridCheckerNode",
    "perception_obstacle_checker": "PerceptionObstacleCheckerNode",
    "planning_lifecycle_checker": "PlanningLifecycleCheckerNode",
    "planning_costmap_checker": "PlanningCostmapCheckerNode",
    "planning_nav_status_checker": "PlanningNavStatusCheckerNode",
    "planning_path_checker": "PlanningPathCheckerNode",
    "planning_state_checker": "camrod_system::PlanningStateChecker",
}

CHECKER_COMPONENT_GROUPS = {
    "hardware_sensing": {"hw", "sensing"},
    "localization": {"localization"},
    "autonomy": {"map", "perception", "planning"},
}


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _profile_param_file(config_dir: str, default_dir: str, category: str, param_file: str) -> str:
    # HH_260617: Diagnostics profiles may override only the files they need.
    # Missing profile files fall back to `default` so sim can relax hardware-only
    # checks without duplicating every checker configuration.
    profile_path = os.path.join(config_dir, category, param_file)
    if os.path.exists(profile_path):
        return profile_path
    return os.path.join(default_dir, category, param_file)


def _flatten_ros_parameters(params: dict, prefix: str = "") -> dict:
    flattened = {}
    for key, value in params.items():
        full_key = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(value, dict):
            flattened.update(_flatten_ros_parameters(value, full_key))
        else:
            flattened[full_key] = value
    return flattened


def _checker_parameters(
    config_dir: str,
    default_dir: str,
    category: str,
    node_name: str,
    param_file: str,
) -> dict:
    # HH_260630 - Checker nodes run under /system, while profiles may be keyed by
    # basename or absolute node name. Flatten parameters directly so namespace
    # does not make camera_names/lidar_names/radar_names/grid_names use defaults.
    path = _profile_param_file(config_dir, default_dir, category, param_file)
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    key_candidates = (
        node_name,
        f"/system/{node_name}",
        f"system/{node_name}",
        f"/{node_name}",
    )
    node_section = next(
        (data.get(key) for key in key_candidates if isinstance(data.get(key), dict)),
        None,
    )
    if not isinstance(node_section, dict):
        return {}

    ros_params = node_section.get("ros__parameters", {})
    if not isinstance(ros_params, dict):
        return {}
    return _flatten_ros_parameters(ros_params)


def _checker_node(
    config_dir: str,
    default_dir: str,
    category: str,
    executable: str,
    name: str,
    param_file: str,
) -> Node:
    return Node(
        package="camrod_system",
        executable=executable,
        name=name,
        output="log",
        parameters=[_checker_parameters(config_dir, default_dir, category, name, param_file)],
    )


def _checker_component(
    config_dir: str,
    default_dir: str,
    module_namespace: str,
    category: str,
    _executable: str,
    name: str,
    param_file: str,
) -> ComposableNode:
    namespace = f"/{module_namespace.strip('/')}" if module_namespace.strip('/') else "/"
    return ComposableNode(
        package="camrod_system",
        plugin=CHECKER_COMPONENT_PLUGINS[name],
        name=name,
        namespace=namespace,
        parameters=[_checker_parameters(
            config_dir, default_dir, category, name, param_file
        )],
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("/diagnostics_agg", "diagnostics_agg"),
        ],
    )


def _checker_containers(
    config_dir: str,
    default_dir: str,
    module_namespace: str,
    _thread_count: int,
) -> list:
    containers = []
    for group_name, categories in CHECKER_COMPONENT_GROUPS.items():
        components = [
            _checker_component(
                config_dir, default_dir, module_namespace, *spec
            )
            for spec in CHECKER_NODE_SPECS
            if spec[0] in categories
        ]
        containers.append(
            ComposableNodeContainer(
                package="rclcpp_components",
                # HH_260805 - Serialize low-rate checker callbacks so executor
                # workers are gone before Cyclone DDS unloads at process exit.
                executable="component_container",
                name=f"{group_name}_checker_container",
                namespace="",
                output="log",
                composable_node_descriptions=components,
            )
        )
    return containers


def _build_checker_nodes(
    config_dir: str,
    default_dir: str,
    module_namespace: str,
    use_components: bool,
    component_threads: int,
) -> list:
    nodes = [
        # Main diagnostics aggregator used by the system health state machine.
        Node(
            package="camrod_system",
            executable="aggregator_node",
            name="diagnostics_agg",
            output="log",
            parameters=[{
                "config_file": _profile_param_file(
                    config_dir, default_dir, "aggregator", "diagnostics_config.yaml"
                )
            }],
        ),
    ]
    if use_components:
        # HH_260805 - Three process boundaries retain category fault isolation
        # while replacing one executor/process per checker on the Jetson.
        nodes.extend(_checker_containers(
            config_dir,
            default_dir,
            module_namespace,
            component_threads,
        ))
    else:
        nodes.extend(
            _checker_node(config_dir, default_dir, *spec)
            for spec in CHECKER_NODE_SPECS
        )
    return nodes


def _build_diagnostics_inline(context, *_args, **_kwargs):
    pkg_share_dir = get_package_share_directory("camrod_system")
    pkg_prefix = get_package_prefix("camrod_system")
    module_namespace = LaunchConfiguration("module_namespace").perform(context)
    config_profile = LaunchConfiguration("config_profile").perform(context)
    enable_checkers = _as_bool(LaunchConfiguration("enable_checkers").perform(context))
    use_checker_components = _as_bool(
        LaunchConfiguration("use_checker_components").perform(context)
    )
    try:
        checker_component_threads = max(
            1,
            int(LaunchConfiguration("checker_component_threads").perform(context)),
        )
    except ValueError:
        checker_component_threads = 2
    enable_platform = _as_bool(LaunchConfiguration("enable_platform").perform(context))
    diagnostics_config_root = LaunchConfiguration("diagnostics_config_root").perform(context)

    # HH_260630 - Standalone system.launch.py uses camrod_system configs by
    # default; bringup may pass its synchronized deployment config root.
    config_root = diagnostics_config_root.strip() or os.path.join(
        pkg_share_dir, "config", "diagnostics")
    if not os.path.isdir(config_root):
        config_root = os.path.join(pkg_share_dir, "config", "diagnostics")
    profile_dir = os.path.join(config_root, config_profile)
    default_dir = os.path.join(config_root, "default")
    config_dir = profile_dir if os.path.isdir(profile_dir) else default_dir

    if not enable_checkers:
        return [
            LogInfo(
                msg="[camrod_system] diagnostics checkers disabled (enable_checkers:=false)"
            )
        ]

    diagnostics_nodes = _build_checker_nodes(
        config_dir,
        default_dir,
        module_namespace,
        use_checker_components,
        checker_component_threads,
    )

    ranger_checker_exec = os.path.join(
        pkg_prefix, "lib", "camrod_system", "ranger_platform_checker_node"
    )
    if enable_platform and os.path.exists(ranger_checker_exec):
        diagnostics_nodes.append(
            Node(
                package="camrod_system",
                executable="ranger_platform_checker_node",
                name="ranger_platform_checker",
                output="log",
                parameters=[_checker_parameters(
                    config_dir, default_dir, "platform",
                    "ranger_platform_checker", "ranger_platform_checker.yaml"
                )],
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
        DeclareLaunchArgument(
            'diagnostics_config_root',
            default_value=pkg_share('camrod_system', os.path.join('config', 'diagnostics')),
            description='Root directory containing diagnostics/<profile> checker YAML files',
        ),
        DeclareLaunchArgument('enable_checkers', default_value='true'),
        DeclareLaunchArgument(
            'use_checker_components',
            default_value='true',
            description='Compose 24 diagnostic checkers into three category containers',
        ),
        DeclareLaunchArgument(
            'checker_component_threads',
            default_value='1',
            description=(
                'Compatibility setting; production checker containers use one '
                'serialized executor for deterministic DDS shutdown'
            ),
        ),
        DeclareLaunchArgument('enable_platform', default_value='false'),
        # system_checker + system_diagnostic + diagnostics_aggregator (tools channel)
        DeclareLaunchArgument('enable_system_tools', default_value='true'),
        DeclareLaunchArgument(
            'system_checker_param_file',
            default_value=pkg_share('camrod_system', os.path.join('config', 'system_checker.yaml')),
        ),
        DeclareLaunchArgument(
            'system_checker_disabled_modules',
            default_value='',
            description='Comma-separated module names excluded from system_checker graph readiness for debug',
        ),

        # HH_260527: Main diagnostics stack is fully inline
        # Retired system_diagnostics/component launch files were removed.
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
                    output='log',
                    parameters=[
                        LaunchConfiguration('system_checker_param_file'),
                        {
                            'diagnostic_topic': 'diagnostics',
                            # HH_260618: Keep a debug-only exclusion hook, while
                            # normal bringup requires exactly one final-parking graph.
                            'disabled_modules_csv': LaunchConfiguration('system_checker_disabled_modules'),
                        },
                    ],
                ),
                Node(
                    package='camrod_system',
                    executable='system_diagnostic_node',
                    name='system_diagnostic',
                    output='screen',
                    parameters=[{
                        'diagnostic_topic': 'diagnostics',
                        # HH_260707 - Operator-facing status must consume the
                        # filtered aggregate stream so profile ignored_names
                        # do not reappear as SYSTEM errors.
                        'source_diagnostic_topic': 'diagnostics_agg',
                        # HH_260617: Publish stable system status topics consumed by UI/voice/diagnostics.
                        'system_status_topic': 'status',
                        'avg_system_msgs_topic': 'msgs',
                        'publish_period_s': 0.5,
                        'stale_timeout_s': 2.0,
                        # HH_260721 - Keep startup normal, then fail missing required diagnostics.
                        'startup_grace_s': 10.0,
                        'log_status_summary': True,
                        'log_status_summary_period_s': 5.0,
                        # HH_260728 - Keep simultaneous per-sensor location
                        # details visible while bounding repeated console output.
                        'max_status_detail_lines': 24,
                    }],
                ),
                Node(
                    package='camrod_system',
                    executable='diagnostics_aggregator_node',
                    name='diagnostics_aggregator',
                    output='log',
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
