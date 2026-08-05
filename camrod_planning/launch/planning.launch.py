import os
import re
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def lc_dict(*names: str) -> dict:
    return {name: LaunchConfiguration(name) for name in names}


def extract_map_ros_params(map_info_cfg: dict) -> dict:
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


def _first_existing_path(candidates: list[str]) -> str:
    seen = set()
    for candidate in candidates:
        normalized = os.path.abspath(candidate)
        if normalized in seen:
            continue
        seen.add(normalized)
        if os.path.isfile(normalized):
            return normalized
    return ''


def _normalize_profile_name(value) -> str:
    text = str(value or '').strip()
    if not text:
        return ''
    text = text.replace('(', '_').replace(')', '_')
    text = re.sub(r'[^A-Za-z0-9_]+', '_', text)
    text = re.sub(r'_+', '_', text).strip('_').lower()
    return text


def _profile_file_variants(default_path: str, profile: str) -> list[str]:
    normalized = _normalize_profile_name(profile)
    if not normalized:
        return []
    directory = os.path.dirname(default_path)
    stem, ext = os.path.splitext(os.path.basename(default_path))
    return [
        os.path.join(directory, f'{stem} ({normalized}){ext}'),
        os.path.join(directory, f'{stem}_{normalized}{ext}'),
        os.path.join(directory, f'{stem}-{normalized}{ext}'),
        os.path.join(directory, normalized, f'{stem}{ext}'),
    ]


def resolve_profile_file(default_path: str, profile: str) -> str:
    # HH_260622 - Standalone planning follows the map semantic profile used by bringup.
    selected = _first_existing_path(_profile_file_variants(default_path, profile))
    return selected if selected else default_path


def _map_filename_candidates(configured: str, profile: str) -> list[str]:
    filenames = []
    configured_name = os.path.basename(str(configured or '').strip())
    if configured_name:
        filenames.append(configured_name)
    normalized = _normalize_profile_name(profile)
    if normalized:
        filenames.extend([
            f'lanelet2_maps_({normalized}).osm',
            f'lanelet2_maps_{normalized}.osm',
            f'lanelet2_maps-{normalized}.osm',
        ])
    filenames.append('lanelet2_maps.osm')

    ordered = []
    for filename in filenames:
        if filename and filename not in ordered:
            ordered.append(filename)
    return ordered


def resolve_map_path_default(map_info_path: str, map_path_from_info: str, map_profile: str) -> str:
    configured = str(map_path_from_info or '').strip()
    if configured:
        configured_path = (
            configured
            if os.path.isabs(configured)
            else os.path.abspath(os.path.join(os.path.dirname(map_info_path), configured))
        )
        if os.path.isfile(configured_path):
            return configured_path

    candidates = []
    filenames = _map_filename_candidates(configured, map_profile)
    anchors = [
        os.path.join(os.path.expanduser('~'), 'camrod_ws', 'src'),
        os.getcwd(),
        os.path.join(os.getcwd(), 'src'),
        os.path.dirname(map_info_path),
        pkg_share('camrod_map', ''),
    ]
    for anchor in anchors:
        cur = os.path.abspath(anchor)
        for _ in range(8):
            for filename in filenames:
                candidates.append(os.path.join(cur, filename))
                candidates.append(os.path.join(cur, 'src', filename))
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent

    discovered = _first_existing_path(candidates)
    return discovered if discovered else configured


def generate_launch_description():
    # Top-level planning launch (Nav2 + lanelet tools + cmd_vel gate).
    map_info_path = pkg_share('camrod_map', os.path.join('config', 'map_info.yaml'))
    map_path_default = ''
    origin_lat_default = '0.0'
    origin_lon_default = '0.0'
    origin_alt_default = '0.0'
    map_profile_default = ''
    try:
        with open(map_info_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        params = extract_map_ros_params(data)
        map_path_default = str(params.get('map_path', map_path_default))
        map_profile_default = str(params.get('map_profile', params.get('profile', '')))
        origin_lat_default = str(params.get('offset_lat', origin_lat_default))
        origin_lon_default = str(params.get('offset_lon', origin_lon_default))
        origin_alt_default = str(params.get('offset_alt', origin_alt_default))
    except Exception:
        pass
    # HH_260622 - Resolve relative/profile map paths so standalone planning is map-file agnostic.
    map_path_default = resolve_map_path_default(map_info_path, map_path_default, map_profile_default)
    planning_state_machine_camping_sites_default = resolve_profile_file(
        pkg_share('camrod_planning', os.path.join('config', 'camping_sites.yaml')),
        map_profile_default,
    )
    planning_state_machine_keypoints_default = resolve_profile_file(
        pkg_share('camrod_map', os.path.join('config', 'drop_zones.yaml')),
        map_profile_default,
    )

    return LaunchDescription([
        DeclareLaunchArgument('module_namespace', default_value='planning'),

        DeclareLaunchArgument('map_path', default_value=map_path_default),
        DeclareLaunchArgument('origin_lat', default_value=origin_lat_default),
        DeclareLaunchArgument('origin_lon', default_value=origin_lon_default),
        DeclareLaunchArgument('origin_alt', default_value=origin_alt_default),

        DeclareLaunchArgument('enable_path_cost_grids', default_value='false'),
        DeclareLaunchArgument('enable_goal_replanner', default_value='false'),
        DeclareLaunchArgument('enable_nav2_lifecycle_retry', default_value='false'),
        DeclareLaunchArgument('require_localization_ready', default_value='false'),
        DeclareLaunchArgument('enable_state_machine', default_value='false'),
        DeclareLaunchArgument('enable_progress', default_value='true'),
        DeclareLaunchArgument('enable_tracking_error', default_value='true'),
        DeclareLaunchArgument('enable_path_visualization', default_value='true'),
        DeclareLaunchArgument('enable_obstacle_replan_monitor', default_value='false'),
        # HH_260805 - The scoped container owns and finalizes its ROS context;
        # standalone mode remains available as a field-isolation fallback.
        DeclareLaunchArgument('use_nav2_container', default_value='true'),

        DeclareLaunchArgument('centerline_input_pose_topic', default_value='/localization/pose'),
        DeclareLaunchArgument('local_path_pose_topic', default_value='/localization/pose'),
        DeclareLaunchArgument('local_path_global_path_topic', default_value='/planning/global_path'),
        DeclareLaunchArgument('local_path_fallback_global_path_topic', default_value=''),
        # HH_260720 - Correct the tracking error topic typo at the launch boundary.
        DeclareLaunchArgument('tracking_error_topic', default_value='/planning/tracking_error'),

        # HH_260720 - Planning publishes only a navigation candidate; control owns safety gating.
        DeclareLaunchArgument(
            # HH_260720 - Nav2 publishes standard Twist only on the explicit ROS boundary.
            'navigation_cmd_vel_topic', default_value='/control/nav2_cmd_vel_ros'
        ),

        # HH_260805 - Production defaults load only the route planner and the
        # policy-reachable wide-lane lattice fallback profile.
        *[DeclareLaunchArgument(k, default_value=pkg_share('camrod_planning', os.path.join('config', v)))
          for k, v in {
              'nav2_base_param_file':              'nav2_base.yaml',
              'nav2_vehicle_param_file':           'nav2_vehicle.yaml',
              'nav2_lanelet_param_file':           'nav2_lanelet_overlay.yaml',
              'nav2_behavior_param_file':          'nav2_behavior.yaml',
              'nav2_combo_param_file':             'nav2_combo_profiles/disabled.yaml',
              'nav2_planner_plugins_param_file':   'nav2_planner_profiles/production.yaml',
              'nav2_controller_plugins_param_file': 'nav2_controller_profiles/production.yaml',
              'local_path_extractor_param_file':   'local_path_extractor.yaml',
              'path_cost_grids_param_file':        'path_cost_grids.yaml',
              'goal_snapper_param_file':           'goal_snapper.yaml',
              'centerline_snapper_param_file':     'centerline_snapper.yaml',
              'goal_replanner_param_file':         'goal_replanner.yaml',
              'obstacle_replan_monitor_param_file': 'obstacle_replan_monitor.yaml',
              'planning_state_machine_param_file': 'planning_state_machine.yaml',
          }.items()],
        DeclareLaunchArgument(
            'planning_state_machine_camping_sites_yaml',
            default_value=planning_state_machine_camping_sites_default,
        ),
        DeclareLaunchArgument('nav2_robot_base_frame', default_value='robot_center_link'),
        DeclareLaunchArgument(
            'nav2_selected_planner',
            default_value='__auto__',
        ),
        DeclareLaunchArgument(
            'nav2_selected_controller',
            default_value='__auto__',
        ),
        DeclareLaunchArgument(
            'planning_state_machine_keypoints_yaml',
            default_value=planning_state_machine_keypoints_default,
        ),

        # HH_260527: Removed unused map-origin launch args.
        # (system_namespace, enable_module_validator).

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_planning', os.path.join('launch', 'nav2_lanelet.launch.py'))
            ),
            launch_arguments=({
                **lc_dict(
                    'nav2_base_param_file',
                    'nav2_vehicle_param_file',
                    'nav2_lanelet_param_file',
                    'nav2_behavior_param_file',
                    'nav2_combo_param_file',
                    'nav2_planner_plugins_param_file',
                    'nav2_controller_plugins_param_file',
                    'enable_path_cost_grids',
                    'path_cost_grids_param_file',
                    'map_path',
                    'origin_lat',
                    'origin_lon',
                    'origin_alt',
                    'nav2_robot_base_frame',
                    'nav2_selected_planner',
                    'nav2_selected_controller',
                    'use_nav2_container',
                    'module_namespace',
                    'navigation_cmd_vel_topic',
                ),
                'nav2_autostart': PythonExpression([
                    "'false' if ('", LaunchConfiguration('require_localization_ready'),
                    "' in ['true', 'True', '1']) else 'true'"
                ]),
            }).items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_planning', os.path.join('launch', 'lanelet_tools.launch.py'))
            ),
            launch_arguments=lc_dict(
                'module_namespace',
                'map_path',
                'origin_lat',
                'origin_lon',
                'origin_alt',
                'centerline_input_pose_topic',
                'goal_snapper_param_file',
                'centerline_snapper_param_file',
            ).items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_planning', os.path.join('launch', 'local_path.launch.py'))
            ),
            # HH_260409: Propagate local-path args to child launch so bringup overrides
            # are applied consistently (fixes hidden default fallback behavior).
            launch_arguments=lc_dict(
                'module_namespace',
                'local_path_extractor_param_file',
                'local_path_pose_topic',
                'local_path_global_path_topic',
                'local_path_fallback_global_path_topic',
                'enable_tracking_error',
                'tracking_error_topic',
            ).items(),
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_planning', os.path.join('launch', 'goal_replanner.launch.py'))
            ),
            launch_arguments=lc_dict(
                'module_namespace',
                'enable_goal_replanner',
                'goal_replanner_param_file',
            ).items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_planning', os.path.join('launch', 'state_machine.launch.py'))
            ),
            launch_arguments=lc_dict(
                'module_namespace',
                'enable_state_machine',
                'planning_state_machine_param_file',
                'planning_state_machine_keypoints_yaml',
                'planning_state_machine_camping_sites_yaml',
            ).items(),
        ),

        Node(
            package='camrod_planning',
            executable='planning_progress_node.py',
            name='planning_progress',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            # HH_260702 - Keep UI progress feedback alive after transient exits.
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration('enable_progress')),
        ),

        Node(
            package='camrod_planning',
            executable='path_visualizer_node.py',
            name='path_visualizer',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            # HH_260702 - RViz/operator path markers are non-safety critical
            # but should recover without a full bringup restart.
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration('enable_path_visualization')),
            parameters=[{
                # HH_260720 - Visualize the generated mirror, not Nav2's nav_msgs boundary topic.
                'global_path_topic': '/planning/global_path_avg',
                'local_path_topic': '/planning/local_path',
                'marker_topic': '/planning/path_markers',
                # HH_260707 - RViz-only marker keepalive; control/safety path
                # topics still publish at their normal rates. Marker rebuilds
                # are limited to 2 Hz because they were a visible CPU source
                # in full real-sensor bringup with RViz enabled.
                'republish_period_s': 1.00,
                'min_publish_period_s': 0.50,
                # HH_260619 - Drop cached global markers if the local route has
                # clearly moved to a newer goal while the global marker source is stale.
                'global_path_stale_timeout_s': 1.0,
                'route_endpoint_mismatch_m': 1.0,
                # HH_260623 - Keep path visualization on the same 2D ground plane as Lanelet2 markers.
                'flatten_path_z': True,
                'path_ground_z': 0.0,
            }],
        ),

        Node(
            package='camrod_planning',
            executable='obstacle_replan_monitor_node.py',
            name='obstacle_replan_monitor',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            # HH_260702 - Dynamic obstacle replan monitor must recover if it
            # exits while the robot is navigating around blocked routes.
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration('enable_obstacle_replan_monitor')),
            parameters=[
                LaunchConfiguration('obstacle_replan_monitor_param_file'),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_planning', os.path.join('launch', 'lifecycle_retry.launch.py'))
            ),
            launch_arguments=lc_dict(
                'module_namespace',
                'enable_nav2_lifecycle_retry',
                'require_localization_ready',
            ).items(),
        ),
    ])
