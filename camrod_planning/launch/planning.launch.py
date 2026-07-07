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

        DeclareLaunchArgument('centerline_input_pose_topic', default_value='/localization/pose'),
        DeclareLaunchArgument('local_path_pose_topic', default_value='/localization/pose'),
        DeclareLaunchArgument('local_path_global_path_topic', default_value='/planning/global_path'),
        DeclareLaunchArgument('local_path_fallback_global_path_topic', default_value=''),
        DeclareLaunchArgument('tracking_error_topic', default_value='/planning/ltracking_error'),

        DeclareLaunchArgument('cmd_vel_gate_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_raw_topic', default_value='/planning/cmd_vel_raw'),
        DeclareLaunchArgument('cmd_vel_output_topic', default_value='/planning/cmd_vel'),
        DeclareLaunchArgument('planning_engage_topic', default_value='/planning/engage'),
        # HH_260623 - Mission engage keeps UI scenarios independent from manual engage.
        DeclareLaunchArgument('planning_mission_engage_topic', default_value='/planning/mission_engage'),
        DeclareLaunchArgument('planning_engaged_state_topic', default_value='/planning/engaged'),
        DeclareLaunchArgument('cmd_vel_gate_estop_source_mode', default_value='platform_status'),
        # HH_260409: Use platform-originated e-stop by default.
        DeclareLaunchArgument('cmd_vel_gate_estop_topic', default_value='/platform/status/estop'),
        # HH_260701 - OR state-machine ERROR_STOP into the planning cmd_vel gate.
        DeclareLaunchArgument(
            'cmd_vel_gate_additional_estop_topics',
            default_value='/planning/state_machine/estop',
        ),
        DeclareLaunchArgument('cmd_vel_gate_dr_timeout_source_mode', default_value='localization_monitor'),
        DeclareLaunchArgument('cmd_vel_gate_allow_on_start', default_value='false'),
        # HH_260707 - Hold after sustained DR_ONLY->NORMAL recovery, but ignore short GNSS flaps.
        DeclareLaunchArgument('cmd_vel_gate_enable_gnss_recovery_hold', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_localization_mode_topic', default_value='/localization/mode'),
        DeclareLaunchArgument('cmd_vel_gate_gnss_recovery_hold_s', default_value='2.0'),
        DeclareLaunchArgument('cmd_vel_gate_gnss_recovery_min_source_s', default_value='1.5'),
        DeclareLaunchArgument('cmd_vel_gate_gnss_recovery_hold_cooldown_s', default_value='10.0'),
        DeclareLaunchArgument('cmd_vel_gate_gnss_recovery_source_mode_min', default_value='2'),
        DeclareLaunchArgument('cmd_vel_gate_gnss_recovery_target_mode', default_value='0'),
        # HH_260413: Optional cost-based stop in front of the platform.
        DeclareLaunchArgument('cmd_vel_gate_cost_stop_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_cost_grid_topic', default_value='/planning/cost_grid/inflation'),
        # HH_260618: Safety gates must use real localization pose, not lanelet-snapped pose.
        DeclareLaunchArgument('cmd_vel_gate_cost_pose_topic', default_value='/localization/pose'),
        # HH_260426: VIO stack is disabled; use localization fallback odometry.
        DeclareLaunchArgument('cmd_vel_gate_cost_odometry_topic', default_value='/localization/fallback/odometry'),
        # HH_260618: Prefer configured raw localization pose for safety checks.
        DeclareLaunchArgument('cmd_vel_gate_pose_source_preference', default_value='pose_topic'),
        DeclareLaunchArgument('cmd_vel_gate_enable_pose_raw_fallback', default_value='false'),
        DeclareLaunchArgument('cmd_vel_gate_cost_threshold', default_value='85'),
        DeclareLaunchArgument('cmd_vel_gate_cost_lookahead_m', default_value='2.0'),
        # HH_260623 - Measured body width plus 0.10 m planning margin per side.
        DeclareLaunchArgument('cmd_vel_gate_cost_width_m', default_value='1.27'),
        DeclareLaunchArgument('cmd_vel_gate_cost_hold_s', default_value='1.0'),
        # HH_260703 - Dynamic obstacle stops stay latched until clear; stale
        # merged cost grid blocks cmd_vel as a fail-safe.
        DeclareLaunchArgument('cmd_vel_gate_cost_stop_latch_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_cost_stop_clear_required_s', default_value='2.0'),
        DeclareLaunchArgument('cmd_vel_gate_cost_stop_latch_log_interval_s', default_value='1.0'),
        DeclareLaunchArgument('cmd_vel_gate_cost_grid_stale_stop_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_cost_grid_stale_timeout_s', default_value='1.0'),
        DeclareLaunchArgument('cmd_vel_gate_cost_grid_stale_log_interval_s', default_value='1.0'),
        # HH_260622: Merged inflation grid contains route/lanelet guidance;
        # cmd_vel blocking must be owned by live dynamic sources.
        DeclareLaunchArgument('cmd_vel_gate_cost_stop_require_dynamic_source', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_cost_stop_dynamic_source_labels', default_value='lidar,radar'),
        # HH_260702 - Use the selected local path corridor for front dynamic
        # obstacle release; fallback to body-front rectangle when no path exists.
        DeclareLaunchArgument('cmd_vel_gate_front_dynamic_stop_use_local_path', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_front_dynamic_path_width_m', default_value='1.27'),
        DeclareLaunchArgument('cmd_vel_gate_front_dynamic_path_max_start_distance_m', default_value='1.5'),
        # HH_260618: Raw lanelet hard-stop uses /map/cost_grid/lanelet before
        # the merged inflation grid clears the ego footprint.
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_grid_topic', default_value='/map/cost_grid/lanelet'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_threshold', default_value='85'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_current_threshold', default_value='85'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_lookahead_m', default_value='1.0'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_width_m', default_value='0.8'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_stop_on_unknown', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_allow_rotation_in_place', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_check_reverse', default_value='false'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_check_lateral', default_value='false'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_min_translation_mps', default_value='0.02'),
        # HH_260619 - Prefer active local-path corridor for forward lanelet safety.
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_front_use_local_path', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_front_path_max_start_distance_m', default_value='1.5'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_front_path_width_m', default_value='0.25'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_front_path_allow_route_reentry', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_current_allow_route_reentry', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_current_route_reentry_max_distance_m', default_value='4.0'),
        DeclareLaunchArgument('cmd_vel_gate_lanelet_safety_current_route_reentry_require_front_cmd', default_value='true'),
        # HH_260624 - Forward drop-zone departure is parking-owned motion and
        # must not be blocked by static lanelet/drop-zone cells before routing.
        DeclareLaunchArgument('cmd_vel_gate_parking_drop_zone_status_topic', default_value='/parking/drop_zone/status'),
        DeclareLaunchArgument('cmd_vel_gate_parking_drop_zone_static_bypass_phases', default_value='EXIT_STRAIGHT,ALIGN_EXIT_YAW'),
        # HH_260701 - Campsite maneuver phases may cross static lanelet cost,
        # while live LiDAR/Radar source costs stay blocking.
        DeclareLaunchArgument('cmd_vel_gate_parking_site_status_topic', default_value='/parking/site_maneuver/status'),
        DeclareLaunchArgument(
            'cmd_vel_gate_parking_site_static_bypass_phases',
            default_value='ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,ALIGN_RETURN_YAW,REVERSE_OUT,CRAB_OUT',
        ),
        # HH_260422: Speed-dependent front lookahead.
        DeclareLaunchArgument('cmd_vel_gate_speed_dependent_lookahead', default_value='true'),
        # HH_260630 - Minimum front scan reaches the front radar mount plus about 1m clearance.
        DeclareLaunchArgument('cmd_vel_gate_front_lookahead_min_m', default_value='2.60'),
        DeclareLaunchArgument('cmd_vel_gate_front_lookahead_max_m', default_value='3.5'),
        DeclareLaunchArgument('cmd_vel_gate_front_lookahead_friction', default_value='0.4'),
        DeclareLaunchArgument('cmd_vel_gate_front_reaction_time_s', default_value='0.20'),
        DeclareLaunchArgument('cmd_vel_gate_front_lookahead_margin_m', default_value='0.45'),
        # HH_260622: Side/rear cost-stop samples the merged grid, but blocks
        # only when dynamic source attribution owns the high-cost cell.
        DeclareLaunchArgument('cmd_vel_gate_side_rear_cost_stop', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_body_near_dynamic_stop', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_body_near_side_lookahead_m', default_value='0.75'),
        DeclareLaunchArgument('cmd_vel_gate_body_near_rear_lookahead_m', default_value='0.55'),
        DeclareLaunchArgument('cmd_vel_gate_body_near_maneuver_side_lookahead_m', default_value='0.55'),
        DeclareLaunchArgument('cmd_vel_gate_body_near_maneuver_rear_lookahead_m', default_value='0.45'),
        DeclareLaunchArgument('cmd_vel_gate_side_cost_threshold', default_value='85'),
        DeclareLaunchArgument('cmd_vel_gate_side_lookahead_m', default_value='1.2'),
        # HH_260623 - Side scan width covers full body length plus front/rear margins.
        DeclareLaunchArgument('cmd_vel_gate_side_corridor_width_m', default_value='1.69160'),
        DeclareLaunchArgument('cmd_vel_gate_rear_cost_threshold', default_value='85'),
        DeclareLaunchArgument('cmd_vel_gate_rear_lookahead_m', default_value='1.2'),
        # HH_260623 - Rear scan width covers full body width plus left/right margins.
        DeclareLaunchArgument('cmd_vel_gate_rear_corridor_width_m', default_value='1.27'),
        # HH_260618: Allow explicit parking/site crab to cross static
        # lanelet/global-path front/side/rear cost while keeping LiDAR/Radar stops active.
        DeclareLaunchArgument('cmd_vel_gate_lateral_cmd_bypass_static_cost_stop', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_lateral_cmd_bypass_min_mps', default_value='0.02'),
        DeclareLaunchArgument('cmd_vel_gate_reverse_cmd_bypass_static_cost_stop', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_reverse_cmd_bypass_min_mps', default_value='0.02'),
        DeclareLaunchArgument('cmd_vel_gate_lateral_cmd_dynamic_obstacle_threshold', default_value='85'),
        # HH_260624 - Pure in-place parking rotation bypasses only static
        # lanelet cost; live LiDAR/Radar disk cost still blocks rotation.
        DeclareLaunchArgument('cmd_vel_gate_rotation_cmd_dynamic_obstacle_stop', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_rotation_cmd_dynamic_obstacle_radius_m', default_value='1.5'),
        DeclareLaunchArgument('cmd_vel_gate_rotation_cmd_dynamic_obstacle_threshold', default_value='85'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_stop_enable', default_value='true'),
        # HH_260422: Fixed from 253 (OccupancyGrid max is 100; 253 never triggers).
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_lethal_threshold', default_value='90'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_cluster_min_cells', default_value='25'),
        DeclareLaunchArgument('cmd_vel_gate_unavoidable_cluster_min_ratio', default_value='0.25'),
        # Optional yaw-alignment zone gate based on map keypoints.
        DeclareLaunchArgument('cmd_vel_gate_yaw_alignment_enable', default_value='false'),
        DeclareLaunchArgument('cmd_vel_gate_yaw_alignment_frame_id', default_value='map'),
        DeclareLaunchArgument(
            'cmd_vel_gate_yaw_alignment_zones_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'yaw_alignment_zones.yaml')),
        ),
        DeclareLaunchArgument('cmd_vel_gate_yaw_alignment_exit_margin_m', default_value='0.3'),
        # HH_260706: Damped field defaults reduce startup yaw oscillation while
        # still blocking forward motion when the robot faces away from the path.
        DeclareLaunchArgument('cmd_vel_gate_route_heading_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_path_topic', default_value='/planning/local_path'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_frame_id', default_value='map'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_min_cmd_x_mps', default_value='0.03'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_lateral_cmd_epsilon_mps', default_value='0.02'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_lookahead_m', default_value='2.0'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_error_enter_deg', default_value='75.0'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_error_exit_deg', default_value='35.0'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_angular_kp', default_value='0.8'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_max_angular_z', default_value='0.35'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_max_linear_x', default_value='0.0'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_min_path_points', default_value='2'),
        # HH_260507: Speed scale for cmd_vel output (applied in planning gate).
        DeclareLaunchArgument('cmd_vel_gate_speed_scale', default_value='1.0'),
        # HH_260626: Force zero if controller cmd_vel stream goes stale.
        DeclareLaunchArgument('cmd_vel_gate_input_timeout_s', default_value='0.35'),
        DeclareLaunchArgument('cmd_vel_gate_zero_publish_rate_hz', default_value='10.0'),

        *[DeclareLaunchArgument(k, default_value=pkg_share('camrod_planning', os.path.join('config', v)))
          for k, v in {
              'nav2_base_param_file':              'nav2_base.yaml',
              'nav2_vehicle_param_file':           'nav2_vehicle.yaml',
              'nav2_lanelet_param_file':           'nav2_lanelet_overlay.yaml',
              'nav2_behavior_param_file':          'nav2_behavior.yaml',
              'nav2_combo_param_file':             'nav2_combo_profiles/disabled.yaml',
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
        DeclareLaunchArgument('nav2_robot_base_frame', default_value='robot_base_link'),
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
                    'enable_path_cost_grids',
                    'path_cost_grids_param_file',
                    'map_path',
                    'origin_lat',
                    'origin_lon',
                    'origin_alt',
                    'nav2_robot_base_frame',
                    'nav2_selected_planner',
                    'nav2_selected_controller',
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
                'local_path_global_path_topic',
                'local_path_fallback_global_path_topic',
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
                'planning_mission_engage_topic',
                'planning_engaged_state_topic',
                'cmd_vel_gate_estop_source_mode',
                'cmd_vel_gate_estop_topic',
                'cmd_vel_gate_additional_estop_topics',
                'cmd_vel_gate_dr_timeout_source_mode',
                'cmd_vel_gate_allow_on_start',
                'cmd_vel_gate_enable_gnss_recovery_hold',
                'cmd_vel_gate_localization_mode_topic',
                'cmd_vel_gate_gnss_recovery_hold_s',
                'cmd_vel_gate_gnss_recovery_min_source_s',
                'cmd_vel_gate_gnss_recovery_hold_cooldown_s',
                'cmd_vel_gate_gnss_recovery_source_mode_min',
                'cmd_vel_gate_gnss_recovery_target_mode',
                'cmd_vel_gate_cost_stop_enable',
                'cmd_vel_gate_cost_grid_topic',
                'cmd_vel_gate_cost_pose_topic',
                'cmd_vel_gate_cost_odometry_topic',
                'cmd_vel_gate_pose_source_preference',
                'cmd_vel_gate_enable_pose_raw_fallback',
                'cmd_vel_gate_cost_threshold',
                'cmd_vel_gate_cost_lookahead_m',
                'cmd_vel_gate_cost_width_m',
                'cmd_vel_gate_cost_hold_s',
                'cmd_vel_gate_cost_stop_latch_enable',
                'cmd_vel_gate_cost_stop_clear_required_s',
                'cmd_vel_gate_cost_stop_latch_log_interval_s',
                'cmd_vel_gate_cost_grid_stale_stop_enable',
                'cmd_vel_gate_cost_grid_stale_timeout_s',
                'cmd_vel_gate_cost_grid_stale_log_interval_s',
                'cmd_vel_gate_cost_stop_require_dynamic_source',
                'cmd_vel_gate_cost_stop_dynamic_source_labels',
                'cmd_vel_gate_front_dynamic_stop_use_local_path',
                'cmd_vel_gate_front_dynamic_path_width_m',
                'cmd_vel_gate_front_dynamic_path_max_start_distance_m',
                'cmd_vel_gate_lanelet_safety_enable',
                'cmd_vel_gate_lanelet_safety_grid_topic',
                'cmd_vel_gate_lanelet_safety_threshold',
                'cmd_vel_gate_lanelet_safety_current_threshold',
                'cmd_vel_gate_lanelet_safety_lookahead_m',
                'cmd_vel_gate_lanelet_safety_width_m',
                'cmd_vel_gate_lanelet_safety_stop_on_unknown',
                'cmd_vel_gate_lanelet_safety_allow_rotation_in_place',
                'cmd_vel_gate_lanelet_safety_check_reverse',
                'cmd_vel_gate_lanelet_safety_check_lateral',
                'cmd_vel_gate_lanelet_safety_min_translation_mps',
                'cmd_vel_gate_lanelet_safety_front_use_local_path',
                'cmd_vel_gate_lanelet_safety_front_path_max_start_distance_m',
                'cmd_vel_gate_lanelet_safety_front_path_width_m',
                'cmd_vel_gate_lanelet_safety_front_path_allow_route_reentry',
                'cmd_vel_gate_lanelet_safety_current_allow_route_reentry',
                'cmd_vel_gate_lanelet_safety_current_route_reentry_max_distance_m',
                'cmd_vel_gate_lanelet_safety_current_route_reentry_require_front_cmd',
                'cmd_vel_gate_parking_drop_zone_status_topic',
                'cmd_vel_gate_parking_drop_zone_static_bypass_phases',
                'cmd_vel_gate_parking_site_status_topic',
                'cmd_vel_gate_parking_site_static_bypass_phases',
                'cmd_vel_gate_speed_dependent_lookahead',
                'cmd_vel_gate_front_lookahead_min_m',
                'cmd_vel_gate_front_lookahead_max_m',
                'cmd_vel_gate_front_lookahead_friction',
                'cmd_vel_gate_front_reaction_time_s',
                'cmd_vel_gate_front_lookahead_margin_m',
                'cmd_vel_gate_side_rear_cost_stop',
                'cmd_vel_gate_body_near_dynamic_stop',
                'cmd_vel_gate_body_near_side_lookahead_m',
                'cmd_vel_gate_body_near_rear_lookahead_m',
                'cmd_vel_gate_body_near_maneuver_side_lookahead_m',
                'cmd_vel_gate_body_near_maneuver_rear_lookahead_m',
                'cmd_vel_gate_side_cost_threshold',
                'cmd_vel_gate_side_lookahead_m',
                'cmd_vel_gate_side_corridor_width_m',
                'cmd_vel_gate_rear_cost_threshold',
                'cmd_vel_gate_rear_lookahead_m',
                'cmd_vel_gate_rear_corridor_width_m',
                'cmd_vel_gate_lateral_cmd_bypass_static_cost_stop',
                'cmd_vel_gate_lateral_cmd_bypass_min_mps',
                'cmd_vel_gate_reverse_cmd_bypass_static_cost_stop',
                'cmd_vel_gate_reverse_cmd_bypass_min_mps',
                'cmd_vel_gate_lateral_cmd_dynamic_obstacle_threshold',
                'cmd_vel_gate_rotation_cmd_dynamic_obstacle_stop',
                'cmd_vel_gate_rotation_cmd_dynamic_obstacle_radius_m',
                'cmd_vel_gate_rotation_cmd_dynamic_obstacle_threshold',
                'cmd_vel_gate_unavoidable_stop_enable',
                'cmd_vel_gate_unavoidable_lethal_threshold',
                'cmd_vel_gate_unavoidable_cluster_min_cells',
                'cmd_vel_gate_unavoidable_cluster_min_ratio',
                'cmd_vel_gate_yaw_alignment_enable',
                'cmd_vel_gate_yaw_alignment_frame_id',
                'cmd_vel_gate_yaw_alignment_zones_file',
                'cmd_vel_gate_yaw_alignment_exit_margin_m',
                'cmd_vel_gate_route_heading_enable',
                'cmd_vel_gate_route_heading_path_topic',
                'cmd_vel_gate_route_heading_frame_id',
                'cmd_vel_gate_route_heading_min_cmd_x_mps',
                'cmd_vel_gate_route_heading_lateral_cmd_epsilon_mps',
                'cmd_vel_gate_route_heading_lookahead_m',
                'cmd_vel_gate_route_heading_error_enter_deg',
                'cmd_vel_gate_route_heading_error_exit_deg',
                'cmd_vel_gate_route_heading_angular_kp',
                'cmd_vel_gate_route_heading_max_angular_z',
                'cmd_vel_gate_route_heading_max_linear_x',
                'cmd_vel_gate_route_heading_min_path_points',
                'cmd_vel_gate_speed_scale',
                'cmd_vel_gate_input_timeout_s',
                'cmd_vel_gate_zero_publish_rate_hz',
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
                # HH_260619 - Show the same published route that local-path and
                # path-cost consumers use; /planning/plan_smoothed is not stable.
                'global_path_topic': LaunchConfiguration('local_path_global_path_topic'),
                'local_path_topic': '/planning/local_path',
                'marker_topic': '/planning/path_markers',
                'republish_period_s': 0.20,
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
