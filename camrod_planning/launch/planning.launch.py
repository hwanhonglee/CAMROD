import os
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
        DeclareLaunchArgument('enable_progress', default_value='true'),
        DeclareLaunchArgument('enable_tracking_error', default_value='true'),
        DeclareLaunchArgument('enable_path_visualization', default_value='true'),

        DeclareLaunchArgument('centerline_input_pose_topic', default_value='/localization/pose'),
        DeclareLaunchArgument('local_path_pose_topic', default_value='/localization/pose'),
        DeclareLaunchArgument('local_path_global_path_topic', default_value='/planning/global_path'),
        DeclareLaunchArgument('local_path_fallback_global_path_topic', default_value=''),
        DeclareLaunchArgument('local_path_source', default_value='slice_only'),
        DeclareLaunchArgument('tracking_error_topic', default_value='/planning/ltracking_error'),

        DeclareLaunchArgument('cmd_vel_gate_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_raw_topic', default_value='/planning/cmd_vel_raw'),
        DeclareLaunchArgument('cmd_vel_output_topic', default_value='/planning/cmd_vel'),
        DeclareLaunchArgument('planning_engage_topic', default_value='/planning/engage'),
        DeclareLaunchArgument('planning_engaged_state_topic', default_value='/planning/engaged'),
        DeclareLaunchArgument('cmd_vel_gate_estop_source_mode', default_value='platform_status'),
        # HH_260409: Use platform-originated e-stop by default.
        DeclareLaunchArgument('cmd_vel_gate_estop_topic', default_value='/platform/status/estop'),
        DeclareLaunchArgument('cmd_vel_gate_dr_timeout_source_mode', default_value='localization_monitor'),
        DeclareLaunchArgument('cmd_vel_gate_allow_on_start', default_value='false'),
        # HH_260427: Short hold window after DR_ONLY->NORMAL localization recovery.
        DeclareLaunchArgument('cmd_vel_gate_enable_gnss_recovery_hold', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_localization_mode_topic', default_value='/localization/mode'),
        DeclareLaunchArgument('cmd_vel_gate_gnss_recovery_hold_s', default_value='2.0'),
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
        DeclareLaunchArgument('cmd_vel_gate_cost_width_m', default_value='1.0'),
        DeclareLaunchArgument('cmd_vel_gate_cost_hold_s', default_value='1.0'),
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
        # HH_260422: Speed-dependent front lookahead.
        DeclareLaunchArgument('cmd_vel_gate_speed_dependent_lookahead', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_front_lookahead_min_m', default_value='0.4'),
        DeclareLaunchArgument('cmd_vel_gate_front_lookahead_max_m', default_value='3.0'),
        DeclareLaunchArgument('cmd_vel_gate_front_lookahead_friction', default_value='0.4'),
        DeclareLaunchArgument('cmd_vel_gate_front_reaction_time_s', default_value='0.15'),
        DeclareLaunchArgument('cmd_vel_gate_front_lookahead_margin_m', default_value='0.3'),
        # HH_260422: Side/rear cost-stop — uses same merged grid as front.
        DeclareLaunchArgument('cmd_vel_gate_side_rear_cost_stop', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_side_cost_threshold', default_value='92'),
        DeclareLaunchArgument('cmd_vel_gate_side_lookahead_m', default_value='0.8'),
        DeclareLaunchArgument('cmd_vel_gate_side_corridor_width_m', default_value='0.45'),
        DeclareLaunchArgument('cmd_vel_gate_rear_cost_threshold', default_value='92'),
        DeclareLaunchArgument('cmd_vel_gate_rear_lookahead_m', default_value='0.6'),
        DeclareLaunchArgument('cmd_vel_gate_rear_corridor_width_m', default_value='0.6'),
        # HH_260618: Allow explicit parking/site crab to cross static
        # lanelet/global-path front/side/rear cost while keeping LiDAR/Radar stops active.
        DeclareLaunchArgument('cmd_vel_gate_lateral_cmd_bypass_static_cost_stop', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_lateral_cmd_bypass_min_mps', default_value='0.02'),
        DeclareLaunchArgument('cmd_vel_gate_reverse_cmd_bypass_static_cost_stop', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_reverse_cmd_bypass_min_mps', default_value='0.02'),
        DeclareLaunchArgument('cmd_vel_gate_lateral_cmd_dynamic_obstacle_threshold', default_value='85'),
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
        # HH_260618: Route-heading guard holds forward cmd until the robot
        # yaw matches the active local path tangent.
        DeclareLaunchArgument('cmd_vel_gate_route_heading_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_path_topic', default_value='/planning/local_path'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_frame_id', default_value='map'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_min_cmd_x_mps', default_value='0.03'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_lateral_cmd_epsilon_mps', default_value='0.02'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_lookahead_m', default_value='1.5'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_error_enter_deg', default_value='75.0'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_error_exit_deg', default_value='20.0'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_angular_kp', default_value='1.4'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_max_angular_z', default_value='0.6'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_max_linear_x', default_value='0.0'),
        DeclareLaunchArgument('cmd_vel_gate_route_heading_min_path_points', default_value='2'),

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
              'planning_state_machine_param_file': 'planning_state_machine.yaml',
              'planning_state_machine_camping_sites_yaml': 'camping_sites.yaml',
          }.items()],
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
            default_value=pkg_share('camrod_map', os.path.join('config', 'drop_zones.yaml')),
        ),

        # HH_260527: Removed unused compatibility args
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
                'cmd_vel_gate_estop_source_mode',
                'cmd_vel_gate_estop_topic',
                'cmd_vel_gate_dr_timeout_source_mode',
                'cmd_vel_gate_allow_on_start',
                'cmd_vel_gate_enable_gnss_recovery_hold',
                'cmd_vel_gate_localization_mode_topic',
                'cmd_vel_gate_gnss_recovery_hold_s',
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
                'cmd_vel_gate_speed_dependent_lookahead',
                'cmd_vel_gate_front_lookahead_min_m',
                'cmd_vel_gate_front_lookahead_max_m',
                'cmd_vel_gate_front_lookahead_friction',
                'cmd_vel_gate_front_reaction_time_s',
                'cmd_vel_gate_front_lookahead_margin_m',
                'cmd_vel_gate_side_rear_cost_stop',
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
            condition=IfCondition(LaunchConfiguration('enable_progress')),
        ),

        Node(
            package='camrod_planning',
            executable='path_visualizer_node.py',
            name='path_visualizer',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
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
            }],
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
