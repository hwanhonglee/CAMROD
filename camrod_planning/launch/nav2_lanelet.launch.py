#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def extract_map_ros_params(map_info_cfg: dict) -> dict:
    # HH_260406: Robust map_info parser for key-layout differences.
    if not isinstance(map_info_cfg, dict):
        return {}
    wildcard = map_info_cfg.get('/**')
    if isinstance(wildcard, dict):
        params = wildcard.get('ros__parameters')
        if isinstance(params, dict):
            return params
    for key in (
        '/map/lanelet2_map',
        'map/lanelet2_map',
        'lanelet2_map',
        '/lanelet2_map',
    ):
        val = map_info_cfg.get(key)
        if isinstance(val, dict):
            params = val.get('ros__parameters')
            if isinstance(params, dict):
                return params
    for val in map_info_cfg.values():
        if isinstance(val, dict):
            params = val.get('ros__parameters')
            if isinstance(params, dict):
                return params
    return {}


def infer_nav2_combo_ids(combo_param_file: str) -> tuple[str, str]:
    """Infer planner/controller IDs from combo profile filename."""
    name = os.path.basename(str(combo_param_file)).lower()
    planner_id = 'NavFn'
    controller_id = 'RPP'

    # HH_260528: Keep mapping explicit so one combo file controls both selectors.
    planner_tokens = (
        ('smachybrid', 'SmacHybrid'),
        ('smaclattice', 'SmacLattice'),
        ('smac2d', 'Smac2D'),
        ('thetastar', 'ThetaStar'),
        ('navfn', 'NavFn'),
    )
    controller_tokens = (
        ('rotationshim', 'RotationShim'),
        ('graceful', 'Graceful'),
        ('mppi', 'MPPI'),
        ('dwb', 'DWB'),
        ('rpp', 'RPP'),
    )

    for token, planner in planner_tokens:
        if token in name:
            planner_id = planner
            break
    for token, controller in controller_tokens:
        if token in name:
            controller_id = controller
            break
    return planner_id, controller_id


def resolve_selector_choice(requested: str, inferred: str) -> str:
    value = str(requested).strip()
    if not value or value in ('__auto__', 'auto'):
        return inferred
    return value


def build_nav2_selector_latch_node(context, *args, **kwargs):
    del args, kwargs
    combo_file = LaunchConfiguration('nav2_combo_param_file').perform(context)
    requested_planner = LaunchConfiguration('nav2_selected_planner').perform(context)
    requested_controller = LaunchConfiguration('nav2_selected_controller').perform(context)
    module_ns = LaunchConfiguration('module_namespace').perform(context).strip('/')

    inferred_planner, inferred_controller = infer_nav2_combo_ids(combo_file)
    planner_id = resolve_selector_choice(requested_planner, inferred_planner)
    controller_id = resolve_selector_choice(requested_controller, inferred_controller)

    planner_topic = f"/{module_ns}/planner_selector" if module_ns else "/planner_selector"
    controller_topic = f"/{module_ns}/controller_selector" if module_ns else "/controller_selector"

    return [
        Node(
            package='camrod_planning',
            executable='nav2_selector_latch_node.py',
            name='nav2_selector_latch',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            parameters=[{
                'planner_id': planner_id,
                'controller_id': controller_id,
                'planner_topic': planner_topic,
                'controller_topic': controller_topic,
                'repeat_hz': 1.0,
            }],
        )
    ]


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    pkg_share = get_package_share_directory('camrod_planning')
    nav2_bt_share = get_package_share_directory('nav2_bt_navigator')
    map_info_path = os.path.join(
        get_package_share_directory('camrod_map'),
        'config',
        'map_info.yaml',
    )
    map_path_default = ''
    origin_lat_default = '0.0'
    origin_lon_default = '0.0'
    origin_alt_default = '0.0'
    try:
        with open(map_info_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        params = extract_map_ros_params(data)
        map_path_default = str(params.get('map_path', map_path_default))
        origin_lat_default = str(params.get('offset_lat', origin_lat_default))
        origin_lon_default = str(params.get('offset_lon', origin_lon_default))
        origin_alt_default = str(params.get('offset_alt', origin_alt_default))
    except Exception:
        pass
    # HH_260409: Keep nav2_lanelet launch resilient when map_info map_path is empty.
    if not str(map_path_default).strip():
        for candidate in (
            os.path.join(os.path.expanduser('~'), 'camrod_ws', 'src', 'lanelet2_maps.osm'),
            os.path.join(os.getcwd(), 'lanelet2_maps.osm'),
            os.path.join(os.getcwd(), 'src', 'lanelet2_maps.osm'),
        ):
            if os.path.isfile(candidate):
                map_path_default = os.path.abspath(candidate)
                break

    # -------------------------------------------------------------------------
    # Default config file paths (4-stage overlay)
    # -------------------------------------------------------------------------
    # HH_260330: Standalone planning launch uses package-local config by default.
    default_base_param = os.path.join(pkg_share, 'config', 'nav2_base.yaml')
    default_vehicle_param = os.path.join(pkg_share, 'config', 'nav2_vehicle.yaml')
    default_lanelet_param = os.path.join(pkg_share, 'config', 'nav2_lanelet_overlay.yaml')
    default_behavior_param = os.path.join(pkg_share, 'config', 'nav2_behavior.yaml')
    default_combo_param = os.path.join(
        pkg_share, 'config', 'nav2_combo_profiles', 'disabled.yaml'
    )
    default_path_cost_grids_param = os.path.join(pkg_share, 'config', 'path_cost_grids.yaml')
    # HH_260513: Switched default BT to Smac2D profile (was _grid.xml / GridBased default).
    # _grid.xml remains available for runtime override via nav2_bt_xml_nav_to_pose launch arg.
    default_nav_to_pose_bt_xml = os.path.join(
        pkg_share, 'config', 'bt', 'navigate_to_pose_w_planner_selector.xml'
    )
    default_nav_through_poses_bt_xml = os.path.join(
        nav2_bt_share,
        'behavior_trees',
        'navigate_through_poses_w_replanning_and_recovery.xml',
    )

    # -------------------------------------------------------------------------
    # Launch Arguments
    # -------------------------------------------------------------------------
    nav2_base_param_arg = DeclareLaunchArgument(
        'nav2_base_param_file',
        default_value=default_base_param,
        description='Nav2 base profile (common planner/controller/costmap defaults)',
    )
    nav2_vehicle_param_arg = DeclareLaunchArgument(
        'nav2_vehicle_param_file',
        default_value=default_vehicle_param,
        description='Nav2 vehicle profile (footprint/dynamics/controller tuning)',
    )
    nav2_lanelet_param_arg = DeclareLaunchArgument(
        'nav2_lanelet_param_file',
        default_value=default_lanelet_param,
        description='Nav2 lanelet profile (lanelet/radar/inflation overlays)',
    )
    nav2_behavior_param_arg = DeclareLaunchArgument(
        'nav2_behavior_param_file',
        default_value=default_behavior_param,
        description='Nav2 behavior profile (BT XML/plugins)',
    )
    nav2_combo_param_arg = DeclareLaunchArgument(
        'nav2_combo_param_file',
        default_value=default_combo_param,
        description='Optional planner/controller combo override profile',
    )
    nav2_selected_planner_arg = DeclareLaunchArgument(
        'nav2_selected_planner',
        default_value='__auto__',
        description='Planner selector ID (auto resolves from nav2_combo_param_file)',
    )
    nav2_selected_controller_arg = DeclareLaunchArgument(
        'nav2_selected_controller',
        default_value='__auto__',
        description='Controller selector ID (auto resolves from nav2_combo_param_file)',
    )
    enable_path_cost_grids_arg = DeclareLaunchArgument(
        'enable_path_cost_grids',
        default_value='true',
        description='Enable planning path-cost-grid helper nodes (/planning/cost_grid/*)',
    )
    path_cost_grids_param_arg = DeclareLaunchArgument(
        'path_cost_grids_param_file',
        default_value=default_path_cost_grids_param,
        description='Parameter file for planning path-cost-grid helper nodes',
    )
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=map_path_default,
        description='Lanelet2 map path for path-cost-grid helpers',
    )
    origin_lat_arg = DeclareLaunchArgument('origin_lat', default_value=origin_lat_default)
    origin_lon_arg = DeclareLaunchArgument('origin_lon', default_value=origin_lon_default)
    origin_alt_arg = DeclareLaunchArgument('origin_alt', default_value=origin_alt_default)
    nav2_robot_base_frame_arg = DeclareLaunchArgument(
        'nav2_robot_base_frame',
        # Keep Nav2 base frame aligned with platform/localization TF.
        default_value='robot_base_link',
        description='Robot base frame used by Nav2 costmaps/BT/behaviors',
    )
    module_namespace_arg = DeclareLaunchArgument(
        'module_namespace',
        default_value='planning',
        description='Namespace for planning/Nav2 nodes',
    )
    nav2_autostart_arg = DeclareLaunchArgument(
        'nav2_autostart',
        default_value='true',
        description='Nav2 lifecycle_manager autostart flag',
    )
    nav2_bt_xml_nav_to_pose_arg = DeclareLaunchArgument(
        'nav2_bt_xml_nav_to_pose',
        default_value=default_nav_to_pose_bt_xml,
        description='BT XML path for NavigateToPose',
    )
    nav2_bt_xml_nav_through_poses_arg = DeclareLaunchArgument(
        'nav2_bt_xml_nav_through_poses',
        default_value=default_nav_through_poses_bt_xml,
        description='BT XML path for NavigateThroughPoses',
    )

    # -------------------------------------------------------------------------
    # LaunchConfigurations
    # -------------------------------------------------------------------------
    nav2_base_param_file = LaunchConfiguration('nav2_base_param_file')
    nav2_vehicle_param_file = LaunchConfiguration('nav2_vehicle_param_file')
    nav2_lanelet_param_file = LaunchConfiguration('nav2_lanelet_param_file')
    nav2_behavior_param_file = LaunchConfiguration('nav2_behavior_param_file')
    nav2_combo_param_file = LaunchConfiguration('nav2_combo_param_file')
    enable_path_cost_grids = LaunchConfiguration('enable_path_cost_grids')
    path_cost_grids_param_file = LaunchConfiguration('path_cost_grids_param_file')
    map_path = LaunchConfiguration('map_path')
    origin_lat = LaunchConfiguration('origin_lat')
    origin_lon = LaunchConfiguration('origin_lon')
    origin_alt = LaunchConfiguration('origin_alt')
    nav2_robot_base_frame = LaunchConfiguration('nav2_robot_base_frame')
    module_namespace = LaunchConfiguration('module_namespace')
    nav2_autostart = LaunchConfiguration('nav2_autostart')
    nav2_bt_xml_nav_to_pose = LaunchConfiguration('nav2_bt_xml_nav_to_pose')
    nav2_bt_xml_nav_through_poses = LaunchConfiguration('nav2_bt_xml_nav_through_poses')
    selector_latch = OpaqueFunction(function=build_nav2_selector_latch_node)
    # 2026-02-25: Apply Nav2 params in deterministic overlay order:
    # base -> vehicle -> lanelet -> behavior.
    nav2_base_params = RewrittenYaml(
        source_file=nav2_base_param_file,
        root_key='planning',
        param_rewrites={},
        convert_types=True,
    )
    nav2_vehicle_params = RewrittenYaml(
        source_file=nav2_vehicle_param_file,
        root_key='planning',
        param_rewrites={},
        convert_types=True,
    )
    nav2_lanelet_params = RewrittenYaml(
        source_file=nav2_lanelet_param_file,
        root_key='planning',
        param_rewrites={},
        convert_types=True,
    )
    nav2_behavior_params = RewrittenYaml(
        source_file=nav2_behavior_param_file,
        root_key='planning',
        param_rewrites={
            # HH_260421: Replace host-specific absolute BT XML paths from YAML.
            'default_nav_to_pose_bt_xml': nav2_bt_xml_nav_to_pose,
            'default_nav_through_poses_bt_xml': nav2_bt_xml_nav_through_poses,
        },
        convert_types=True,
    )
    nav2_combo_params = RewrittenYaml(
        source_file=nav2_combo_param_file,
        root_key='planning',
        param_rewrites={},
        convert_types=True,
    )

    # -------------------------------------------------------------------------
    # Critical safety guard:
    # Force robot_base_frame in costmap/behavior/smoother using canonical
    # ROS2 parameter-tree structure. This prevents silent fallback to robot_base_link.
    # -------------------------------------------------------------------------
    force_base_link_overrides = {
        'global_costmap': {
            'global_costmap': {
                'ros__parameters': {
                    'robot_base_frame': nav2_robot_base_frame,
                }
            }
        },
        'local_costmap': {
            'local_costmap': {
                'ros__parameters': {
                    'robot_base_frame': nav2_robot_base_frame,
                }
            }
        },
        'behavior_server': {
            'ros__parameters': {
                'global_frame': 'map',
                'local_frame': 'odom',
                'robot_base_frame': nav2_robot_base_frame,
                # HH_260330: Keep TF tolerance aligned with nav2_base/behavior profiles.
                'transform_tolerance': 0.5,
            }
        },
        'bt_navigator': {
            'ros__parameters': {
                'global_frame': 'map',
                'robot_base_frame': nav2_robot_base_frame,
                'odom_topic': '/localization/odometry/filtered',
                # HH_260421: Force package-share-resolved BT XML paths to avoid
                # host-specific absolute-path breakage.
                'default_nav_to_pose_bt_xml': nav2_bt_xml_nav_to_pose,
                'default_nav_through_poses_bt_xml': nav2_bt_xml_nav_through_poses,
                # HH_260330: Keep TF tolerance aligned with nav2_base/behavior profiles.
                'transform_tolerance': 0.5,
            }
        },
        'nav2_velocity_smoother': {
            'ros__parameters': {
                'robot_base_frame': nav2_robot_base_frame,
            }
        },
    }
    nav2_param_chain = [
        nav2_base_params,
        nav2_vehicle_params,
        nav2_lanelet_params,
        nav2_behavior_params,
        nav2_combo_params,
        force_base_link_overrides,
    ]

    # -------------------------------------------------------------------------
    # Nav2 nodes under /planning namespace
    # -------------------------------------------------------------------------
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace=module_namespace,
        output='screen',
        parameters=nav2_param_chain,
        # Keep canonical global route topic stable for downstream modules.
        remappings=[
            ('plan', '/planning/global_path'),
        ],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace=module_namespace,
        output='screen',
        parameters=nav2_param_chain,
        # HH_260304-00:00 Keep only controller-internal debug topics that are still useful.
        # HH_260304-00:00 Do not expose /planning/local_plan_raw: legacy RViz sessions may
        # HH_260304-00:00 overlay it on top of /planning/local_path and make the local plan
        # HH_260304-00:00 look duplicated or branch to a wrong loop segment.
        remappings=[
            # HH_260331: Route controller output to raw topic.
            # Final /planning/cmd_vel is published by planning cmd_vel gate after engage trigger.
            ('cmd_vel', '/planning/cmd_vel_raw'),
            ('received_global_plan', '/planning/local_path_controller'),
            ('transformed_global_plan', '/planning/local_path_dwb'),
        ],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace=module_namespace,
        output='screen',
        parameters=nav2_param_chain + [{
            # HH_260306-00:00 Hard-override to block fallback to default "robot_base_link"
            # during recovery behavior pose transforms.
            'global_frame': 'map',
            'local_frame': 'odom',
            'robot_base_frame': nav2_robot_base_frame,
            # HH_260330: Keep TF tolerance aligned with nav2_base/behavior profiles.
            'transform_tolerance': 0.5,
        }],
        remappings=[
            # HH_260331: Keep /planning/cmd_vel as controller-only stream.
            # Recovery behaviors publish to a separate topic for clearer runtime control/debug.
            ('cmd_vel', '/planning/cmd_vel_recovery'),
        ],
    )

    # HH_260513: SimpleSmoother server for BT SmoothPath node.
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        namespace=module_namespace,
        output='screen',
        parameters=nav2_param_chain,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace=module_namespace,
        output='screen',
        parameters=nav2_param_chain + [{
            # HH_260306-00:00 Keep BT transform helpers pinned to configured Nav2 base frame.
            'global_frame': 'map',
            'robot_base_frame': nav2_robot_base_frame,
            # HH_260421: Override host-specific absolute BT XML paths from YAML.
            'default_nav_to_pose_bt_xml': nav2_bt_xml_nav_to_pose,
            'default_nav_through_poses_bt_xml': nav2_bt_xml_nav_through_poses,
            # HH_260330: Keep TF tolerance aligned with nav2_base/behavior profiles.
            'transform_tolerance': 0.5,
        }],
        remappings=[
            # HH_260316-00:00 Consume snapped goal topic only.
            # Prevent raw-goal bypass when external tools publish to /planning/goal_pose directly.
            # HH_260317-00:00 Apply both relative/absolute forms for compatibility.
            ('goal_pose', '/planning/goal_pose_snapped_ros'),
            ('/goal_pose', '/planning/goal_pose_snapped_ros'),
        ],
    )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planning',
        namespace=module_namespace,
        output='screen',
        parameters=[{
            'use_sim_time': False,
            # HH_260327: allow launch-level localization gate to control activation timing.
            'autostart': nav2_autostart,
            # HH_260410: Increase bond/service windows to avoid false startup abort
            # on heavy bringup (planner_server may respond later under load).
            'bond_timeout': 20.0,
            'service_timeout': 10000,
            'attempt_respawn_reconnection': True,
            'bond_respawn_max_duration': 30.0,
            'node_names': [
                'planner_server',
                'controller_server',
                # HH_260513: smoother_server must be activated before bt_navigator to avoid
                # SmoothPath action server not-found errors at BT startup.
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                # 2026-01-30 14:32: Let planner/controller manage internal costmap lifecycles (avoid duplicate configure).
            ],
        }],
        remappings=[
        ],
    )

    path_cost_grids = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'path_cost_grids.launch.py')),
        launch_arguments={
            'path_cost_grids_param_file': path_cost_grids_param_file,
            'map_path': map_path,
            'origin_lat': origin_lat,
            'origin_lon': origin_lon,
            'origin_alt': origin_alt,
            'module_namespace': module_namespace,
        }.items(),
        condition=IfCondition(enable_path_cost_grids),
    )

    return LaunchDescription([
        nav2_base_param_arg,
        nav2_vehicle_param_arg,
        nav2_lanelet_param_arg,
        nav2_behavior_param_arg,
        nav2_combo_param_arg,
        nav2_selected_planner_arg,
        nav2_selected_controller_arg,
        enable_path_cost_grids_arg,
        path_cost_grids_param_arg,
        map_path_arg,
        origin_lat_arg,
        origin_lon_arg,
        origin_alt_arg,
        nav2_robot_base_frame_arg,
        module_namespace_arg,
        nav2_autostart_arg,
        nav2_bt_xml_nav_to_pose_arg,
        nav2_bt_xml_nav_through_poses_arg,

        selector_latch,
        planner_server,
        controller_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        lifecycle_mgr,
        path_cost_grids,
    ])
