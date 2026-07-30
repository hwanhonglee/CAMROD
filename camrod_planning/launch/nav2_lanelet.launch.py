#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
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
    # HH_260626: Default to the connected lanelet route when no combo profile
    # is selected; grid planners remain fallback/selectable.
    planner_id = 'LaneletRoute'
    # HH_260618: Default controller is MPPI. Global path remains the reference,
    # but local trajectory sampling/cost critics decide the actual cmd_vel.
    controller_id = 'MPPI'

    # HH_260528: Keep mapping explicit so one combo file controls both selectors.
    planner_tokens = (
        ('laneletroute', 'LaneletRoute'),
        ('lanelet_route', 'LaneletRoute'),
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


def package_available(package_name: str) -> bool:
    try:
        get_package_share_directory(package_name)
        return True
    except PackageNotFoundError:
        return False


def build_nav2_selector_latch_node(context, *args, **kwargs):
    del args, kwargs
    combo_file = LaunchConfiguration('nav2_combo_param_file').perform(context)
    requested_planner = LaunchConfiguration('nav2_selected_planner').perform(context)
    requested_controller = LaunchConfiguration('nav2_selected_controller').perform(context)
    regulated_goal_checker = LaunchConfiguration(
        'nav2_regulated_goal_checker'
    ).perform(context)
    manual_planner = LaunchConfiguration('nav2_manual_planner').perform(context)
    manual_controller = LaunchConfiguration('nav2_manual_controller').perform(context)
    manual_goal_checker = LaunchConfiguration('nav2_manual_goal_checker').perform(context)
    goal_source_topic = LaunchConfiguration('nav2_goal_source_topic').perform(context)
    module_ns = LaunchConfiguration('module_namespace').perform(context).strip('/')

    inferred_planner, inferred_controller = infer_nav2_combo_ids(combo_file)
    planner_id = resolve_selector_choice(requested_planner, inferred_planner)
    controller_id = resolve_selector_choice(requested_controller, inferred_controller)

    # HH_260720 - Keep Nav2's std_msgs selector endpoints on explicit ROS boundary topics.
    planner_topic = f'/{module_ns}/planner_selector_ros' if module_ns else '/planner_selector_ros'
    controller_topic = (
        f'/{module_ns}/controller_selector_ros' if module_ns else '/controller_selector_ros'
    )
    # HH_260727 - Goal source also selects the matching arrival/yaw policy.
    goal_checker_topic = (
        f'/{module_ns}/goal_checker_selector_ros'
        if module_ns else '/goal_checker_selector_ros'
    )

    return [
        Node(
            package='camrod_planning',
            executable='nav2_selector_latch_node.py',
            name='nav2_selector_latch',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            # HH_260528: Keep selector publisher alive even if transient startup errors occur.
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'planner_id': planner_id,
                'controller_id': controller_id,
                'regulated_goal_checker_id': regulated_goal_checker,
                'manual_planner_id': manual_planner,
                'manual_controller_id': manual_controller,
                'manual_goal_checker_id': manual_goal_checker,
                'goal_source_topic': goal_source_topic,
                'planner_topic': planner_topic,
                'controller_topic': controller_topic,
                'goal_checker_topic': goal_checker_topic,
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
    # HH_260626: Default BT uses PlannerSelector; selector default is LaneletRoute.
    # _grid.xml remains available for runtime override via nav2_bt_xml_nav_to_pose launch arg.
    default_nav_to_pose_bt_xml = os.path.join(
        pkg_share, 'config', 'bt', 'navigate_to_pose_w_planner_selector.xml'
    )
    default_nav_through_poses_bt_xml = os.path.join(
        nav2_bt_share,
        'behavior_trees',
        'navigate_through_poses_w_replanning_and_recovery.xml',
    )
    # HH_260720 - Resolve the installed SmacLattice primitive without a host-specific path.
    default_lattice_filepath = os.path.join(
        pkg_share,
        'lattice_primitives',
        'sample_primitives',
        '5cm_resolution',
        '1m_turning_radius',
        'ackermann',
        'output.json',
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
    # HH_260727 - Keep regulated mission defaults configurable while giving
    # manual RViz goals an explicit route/yaw-aware selector policy.
    nav2_regulated_goal_checker_arg = DeclareLaunchArgument(
        'nav2_regulated_goal_checker',
        default_value='goal_checker',
        description='Goal checker selector ID for regulated UI/mission goals',
    )
    nav2_manual_planner_arg = DeclareLaunchArgument(
        'nav2_manual_planner',
        # HH_260730 / TODOLIST 7 - Fresh sim calls with the same 50.29 m
        # start/goal produced zero poses from NavFn, ThetaStar, SmacHybrid, and
        # SmacLattice; LaneletRoute returned 256 poses in 130 ms and retained
        # the operator's 107.8 deg final yaw. Project manual x/y onto a routable
        # lane centerline before routing on the connected graph, while keeping
        # the clicked arrival orientation. UI goals additionally use lane yaw.
        # Costmaps, full footprint, and the final gate still apply.
        default_value='LaneletRoute',
        description='Planner selector ID for manual RViz goals (default: LaneletRoute)',
    )
    nav2_manual_controller_arg = DeclareLaunchArgument(
        'nav2_manual_controller',
        default_value='RotationShim',
        description='Controller selector ID for manual RViz goals',
    )
    nav2_manual_goal_checker_arg = DeclareLaunchArgument(
        'nav2_manual_goal_checker',
        default_value='manual_goal_checker',
        description='Goal checker selector ID for manual RViz goals',
    )
    nav2_goal_source_topic_arg = DeclareLaunchArgument(
        'nav2_goal_source_topic',
        default_value='/planning/goal_source',
        description='Goal source latch topic (regulated or manual)',
    )
    enable_path_cost_grids_arg = DeclareLaunchArgument(
        'enable_path_cost_grids',
        default_value='false',
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
    # HH_260720 - Route Nav2 output into the control-owned raw command channel.
    navigation_cmd_vel_topic_arg = DeclareLaunchArgument(
        'navigation_cmd_vel_topic',
        # HH_260720 - Nav2's standard Twist enters control through an explicit ROS boundary.
        default_value='/control/nav2_cmd_vel_ros',
        description='Raw navigation command consumed by camrod_control',
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
    navigation_cmd_vel_topic = LaunchConfiguration('navigation_cmd_vel_topic')
    nav2_autostart = LaunchConfiguration('nav2_autostart')
    nav2_bt_xml_nav_to_pose = LaunchConfiguration('nav2_bt_xml_nav_to_pose')
    nav2_bt_xml_nav_through_poses = LaunchConfiguration('nav2_bt_xml_nav_through_poses')
    selector_latch = OpaqueFunction(function=build_nav2_selector_latch_node)
    # 2026-02-25: Apply Nav2 params in deterministic overlay order:
    # base -> vehicle -> lanelet -> behavior.
    nav2_base_params = RewrittenYaml(
        source_file=nav2_base_param_file,
        root_key='planning',
        # HH_260619: Inject lanelet map launch arguments into the Nav2
        # LaneletRoute planner plugin. RewrittenYaml rewrites parameter leaves;
        # a raw nested dict in Node(parameters=...) is not a node-scoped YAML.
        param_rewrites={
            'map_path': map_path,
            'offset_lat': origin_lat,
            'offset_lon': origin_lon,
            'offset_alt': origin_alt,
            # HH_260720 - Override the YAML marker with the package-share primitive path.
            'lattice_filepath': default_lattice_filepath,
        },
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
        # HH_260720 - Keep optional SmacLattice combo profiles portable as well.
        param_rewrites={'lattice_filepath': default_lattice_filepath},
        convert_types=True,
    )

    # -------------------------------------------------------------------------
    # Critical safety guard:
    # Force robot_base_frame in costmap/behavior/smoother using canonical
    # ROS2 parameter-tree structure. This prevents silent fallback to base_link.
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
        'smoother_server': {
            'ros__parameters': {
                # HH_260629: SimpleSmoother collision checks otherwise default
                # to base_link, which this robot does not publish.
                'global_frame': 'map',
                'robot_base_frame': nav2_robot_base_frame,
                'transform_tolerance': 0.5,
            }
        },
        'bt_navigator': {
            'ros__parameters': {
                'global_frame': 'map',
                'robot_base_frame': nav2_robot_base_frame,
                # HH_260720 - Nav2 requires the explicit nav_msgs odometry mirror.
                'odom_topic': '/localization/odometry_ros',
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
    has_nav2_behaviors = package_available('nav2_behaviors')
    has_nav2_smoother = package_available('nav2_smoother')

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace=module_namespace,
        output='screen',
        # HH_260702 - Nav2 lifecycle_manager can reconnect to respawned
        # lifecycle nodes, but launch must first recreate the crashed process.
        respawn=True,
        respawn_delay=2.0,
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
        # HH_260702 - Keep the controller process alive after transient Nav2 crashes.
        respawn=True,
        respawn_delay=2.0,
        parameters=nav2_param_chain,
        # HH_260304-00:00 Keep only controller-internal debug topics that are still useful.
        # HH_260304-00:00 Do not expose /planning/local_plan_raw: older RViz sessions may
        # HH_260304-00:00 overlay it on top of /planning/local_path and make the local plan
        # HH_260304-00:00 look duplicated or branch to a wrong loop segment.
        remappings=[
            # HH_260720 - Nav2 produces an unapproved candidate for the control safety gate.
            ('cmd_vel', navigation_cmd_vel_topic),
            ('received_global_plan', '/planning/local_path_controller'),
            ('transformed_global_plan', '/planning/local_path_dwb'),
        ],
    )

    if has_nav2_behaviors:
        behavior_server = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=module_namespace,
            output='screen',
            # HH_260702 - Recovery behaviors are lifecycle-managed and may be
            # reconnected by lifecycle_manager after launch respawns the process.
            respawn=True,
            respawn_delay=2.0,
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
                # HH_260720 - Route recovery commands through the same control safety gate.
                ('cmd_vel', navigation_cmd_vel_topic),
            ],
        )
    else:
        # HH_260604: Do not abort bringup when optional Nav2 behavior package is absent.
        behavior_server = LogInfo(msg='[nav2_lanelet] nav2_behaviors not found; skipping behavior_server')

    # HH_260513: SimpleSmoother server for BT SmoothPath node.
    if has_nav2_smoother:
        smoother_server = Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            namespace=module_namespace,
            output='screen',
            # HH_260702 - SmoothPath is in the BT path; respawn keeps planner
            # recovery from staying broken after a one-off smoother crash.
            respawn=True,
            respawn_delay=2.0,
            parameters=nav2_param_chain + [{
                # HH_260629: Hard-override to block smoother collision checks
                # from using Nav2's default base_link frame.
                'global_frame': 'map',
                'robot_base_frame': nav2_robot_base_frame,
                'transform_tolerance': 0.5,
            }],
        )
    else:
        # HH_260604: Do not abort bringup when optional Nav2 smoother package is absent.
        smoother_server = LogInfo(msg='[nav2_lanelet] nav2_smoother not found; skipping smoother_server')

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace=module_namespace,
        output='screen',
        # HH_260702 - Keep the action server available if BT navigator exits.
        respawn=True,
        respawn_delay=2.0,
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
            # HH_260317-00:00 Apply both relative/absolute remaps for external tools.
            ('goal_pose', '/planning/goal_pose_snapped_ros'),
            ('/goal_pose', '/planning/goal_pose_snapped_ros'),
        ],
    )

    lifecycle_node_names = [
        'planner_server',
        'controller_server',
    ]
    if has_nav2_smoother:
        # HH_260604: Activate smoother only when its package is installed.
        lifecycle_node_names.append('smoother_server')
    if has_nav2_behaviors:
        # HH_260604: Activate behavior server only when its package is installed.
        lifecycle_node_names.append('behavior_server')
    lifecycle_node_names.extend([
        'bt_navigator',
        # 2026-01-30 14:32: Let planner/controller manage internal costmap lifecycles (avoid duplicate configure).
    ])

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planning',
        namespace=module_namespace,
        output='screen',
        # HH_260702 - If the manager itself exits, bring it back so it can
        # reconnect respawned Nav2 lifecycle nodes instead of leaving checker
        # missing-node warnings as the only recovery signal.
        respawn=True,
        respawn_delay=2.0,
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
            'node_names': lifecycle_node_names,
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
        nav2_regulated_goal_checker_arg,
        nav2_manual_planner_arg,
        nav2_manual_controller_arg,
        nav2_manual_goal_checker_arg,
        nav2_goal_source_topic_arg,
        enable_path_cost_grids_arg,
        path_cost_grids_param_arg,
        map_path_arg,
        origin_lat_arg,
        origin_lon_arg,
        origin_alt_arg,
        nav2_robot_base_frame_arg,
        module_namespace_arg,
        navigation_cmd_vel_topic_arg,
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
