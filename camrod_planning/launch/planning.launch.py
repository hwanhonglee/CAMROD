import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


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


def generate_launch_description():
    # Top-level planning launch (Nav2 + lanelet tools + cmd_vel gate).
    map_info_path = pkg_share('camrod_map', os.path.join('config', 'map_info.yaml'))
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
    # HH_260409: Keep planning standalone launch usable even when map_info.yaml
    # leaves map_path empty (discover common workspace map path candidates).
    if not str(map_path_default).strip():
        for candidate in (
            os.path.join(os.path.expanduser('~'), 'camrod_ws', 'src', 'lanelet2_maps.osm'),
            os.path.join(os.getcwd(), 'lanelet2_maps.osm'),
            os.path.join(os.getcwd(), 'src', 'lanelet2_maps.osm'),
        ):
            if os.path.isfile(candidate):
                map_path_default = os.path.abspath(candidate)
                break

    return LaunchDescription([
        DeclareLaunchArgument('module_namespace', default_value='planning'),

        DeclareLaunchArgument('map_path', default_value=map_path_default),
        DeclareLaunchArgument('origin_lat', default_value=origin_lat_default),
        DeclareLaunchArgument('origin_lon', default_value=origin_lon_default),
        DeclareLaunchArgument('origin_alt', default_value=origin_alt_default),

        DeclareLaunchArgument('enable_path_cost_grids', default_value='true'),
        DeclareLaunchArgument('enable_goal_replanner', default_value='false'),
        DeclareLaunchArgument('enable_nav2_lifecycle_retry', default_value='false'),
        DeclareLaunchArgument('require_localization_ready', default_value='false'),
        DeclareLaunchArgument('enable_state_machine', default_value='false'),
        DeclareLaunchArgument('enable_tracking_error', default_value='true'),

        DeclareLaunchArgument('centerline_input_pose_topic', default_value='/localization/pose'),
        DeclareLaunchArgument('local_path_pose_topic', default_value='/localization/pose'),
        DeclareLaunchArgument('local_path_source', default_value='controller_then_slice'),
        DeclareLaunchArgument('tracking_error_topic', default_value='/planning/ltracking_error'),

        DeclareLaunchArgument('cmd_vel_gate_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_raw_topic', default_value='/planning/cmd_vel_raw'),
        DeclareLaunchArgument('cmd_vel_output_topic', default_value='/planning/cmd_vel'),
        DeclareLaunchArgument('planning_engage_topic', default_value='/planning/engage'),
        DeclareLaunchArgument('planning_engaged_state_topic', default_value='/planning/engaged'),
        DeclareLaunchArgument('cmd_vel_gate_use_estop_topic', default_value='true'),
        # HH_260409: Use platform-originated e-stop by default.
        DeclareLaunchArgument('cmd_vel_gate_estop_topic', default_value='/platform/status/estop'),
        DeclareLaunchArgument('cmd_vel_gate_allow_on_start', default_value='false'),
        # HH_260413: Optional cost-based stop in front of the platform.
        DeclareLaunchArgument('cmd_vel_gate_cost_stop_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_cost_grid_topic', default_value='/planning/local_costmap/costmap'),
        DeclareLaunchArgument('cmd_vel_gate_cost_pose_topic', default_value='/localization/pose'),
        DeclareLaunchArgument('cmd_vel_gate_cost_threshold', default_value='200'),
        DeclareLaunchArgument('cmd_vel_gate_cost_lookahead_m', default_value='2.0'),
        DeclareLaunchArgument('cmd_vel_gate_cost_width_m', default_value='1.0'),
        DeclareLaunchArgument('cmd_vel_gate_cost_hold_sec', default_value='1.0'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_stop_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_lethal_threshold', default_value='253'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_cluster_min_cells', default_value='25'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_cluster_min_ratio', default_value='0.25'),

        DeclareLaunchArgument(
            'nav2_base_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'nav2_base.yaml')),
        ),
        DeclareLaunchArgument(
            'nav2_vehicle_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'nav2_vehicle.yaml')),
        ),
        DeclareLaunchArgument(
            'nav2_lanelet_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'nav2_lanelet_overlay.yaml')),
        ),
        DeclareLaunchArgument(
            'nav2_behavior_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'nav2_behavior.yaml')),
        ),
        DeclareLaunchArgument('nav2_robot_base_frame', default_value='robot_base_link'),

        DeclareLaunchArgument(
            'local_path_extractor_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'local_path_extractor.yaml')),
        ),
        DeclareLaunchArgument(
            'path_cost_grids_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'path_cost_grids.yaml')),
        ),
        DeclareLaunchArgument(
            'goal_snapper_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'goal_snapper.yaml')),
        ),
        DeclareLaunchArgument(
            'centerline_snapper_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'centerline_snapper.yaml')),
        ),
        DeclareLaunchArgument(
            'goal_replanner_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'goal_replanner.yaml')),
        ),
        DeclareLaunchArgument(
            'planning_state_machine_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'planning_state_machine.yaml')),
        ),
        DeclareLaunchArgument(
            'planning_state_machine_keypoints_yaml',
            default_value=pkg_share('camrod_map', os.path.join('config', 'drop_zones.yaml')),
        ),
        DeclareLaunchArgument(
            'planning_state_machine_camping_sites_yaml',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'camping_sites.yaml')),
        ),

        # HH_260407: backward compatibility (no-op)
        DeclareLaunchArgument('system_namespace', default_value='system'),
        DeclareLaunchArgument('enable_module_validator', default_value='false'),

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
                    'enable_path_cost_grids',
                    'path_cost_grids_param_file',
                    'map_path',
                    'origin_lat',
                    'origin_lon',
                    'origin_alt',
                    'nav2_robot_base_frame',
                    'module_namespace',
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
                'local_path_source',
                'enable_tracking_error',
                'tracking_error_topic',
            ).items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_planning', os.path.join('launch', 'cmd_vel_gate.launch.py'))
            ),
            launch_arguments=lc_dict(
                'module_namespace',
                'cmd_vel_gate_enable',
                'cmd_vel_raw_topic',
                'cmd_vel_output_topic',
                'planning_engage_topic',
                'planning_engaged_state_topic',
                'cmd_vel_gate_use_estop_topic',
                'cmd_vel_gate_estop_topic',
                'cmd_vel_gate_allow_on_start',
                'cmd_vel_gate_cost_stop_enable',
                'cmd_vel_gate_cost_grid_topic',
                'cmd_vel_gate_cost_pose_topic',
                'cmd_vel_gate_cost_threshold',
                'cmd_vel_gate_cost_lookahead_m',
                'cmd_vel_gate_cost_width_m',
                'cmd_vel_gate_cost_hold_sec',
                'cmd_vel_gate_unavoidable_stop_enable',
                'cmd_vel_gate_unavoidable_lethal_threshold',
                'cmd_vel_gate_unavoidable_cluster_min_cells',
                'cmd_vel_gate_unavoidable_cluster_min_ratio',
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
