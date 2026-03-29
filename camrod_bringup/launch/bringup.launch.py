"""Top-level orchestrator for camrod runtime.

HH_260317-00:00
- Keep only cross-package wiring + runtime toggles in bringup.
- Delegate detailed node params/composition to each module launch.
"""

import os
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# Resolves package-relative path.
def pkg_path(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


# Loads YAML as dict/list (returns {} on failure).
def read_yaml(path: str) -> Any:
    try:
        with open(path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


# Gets nested dict value by slash-separated key path.
def cfg_get(cfg: dict, key_path: str, default: Any) -> Any:
    cur: Any = cfg
    for key in key_path.split('/'):
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


# Converts bool/number to launch-default string.
def as_launch_default(value: Any) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


# Makes a pkill-safe regex prefix so cleanup command does not kill itself.
def pkill_safe_pattern(raw: str) -> str:
    if not raw:
        return raw
    first = raw[0]
    if first.isalnum():
        return f'[{first}]{raw[1:]}'
    return raw


# Builds a shell command for cleanup from process patterns.
def build_cleanup_cmd(patterns: list[str]) -> str:
    parts = []
    for p in patterns:
        safe = pkill_safe_pattern(str(p))
        parts.append(
            f'for _pid in $(pgrep -f "{safe}" || true); do '
            f'[ "$_pid" = "$$" ] && continue; '
            f'[ "$_pid" = "$PPID" ] && continue; '
            f'kill "$_pid" 2>/dev/null || true; '
            f'done'
        )
    return '; '.join(parts)


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    bringup_cfg = lambda rel: pkg_path('camrod_bringup', os.path.join('config', rel))

    launch_cfg = read_yaml(bringup_cfg('bringup/launch_defaults.yaml')).get('bringup', {})
    map_info = read_yaml(bringup_cfg('map/map_info.yaml'))
    map_params = map_info.get('/map/lanelet2_map', {}).get('ros__parameters', {})
    # HH_260327: Read unified map/localization reference from ros__parameters
    # to keep map_info.yaml ROS2-param-parser compatible.
    map_ref_llh = {
        'lat': map_params.get('reference_lat', map_params.get('offset_lat', 0.0)),
        'lon': map_params.get('reference_lon', map_params.get('offset_lon', 0.0)),
        'alt': map_params.get('reference_alt', map_params.get('offset_alt', 0.0)),
    }
    map_ref_utm = {
        'easting': map_params.get('reference_utm_easting', 0.0),
        'northing': map_params.get('reference_utm_northing', 0.0),
        'alt': map_params.get('reference_utm_alt', map_ref_llh['alt']),
    }
    yaw_default = float(
        map_params.get(
            'yaw_offset_deg',
            read_yaml(bringup_cfg('localization/localization_origin.yaml')).get('origin', {}).get('yaw_offset_deg', 0.0),
        )
    )

    # High-level arguments only.
    arg_specs = [
        ('clean_before_launch', cfg_get(launch_cfg, 'runtime/clean_before_launch', True), 'Kill stale processes first'),
        ('sim', cfg_get(launch_cfg, 'runtime/sim', True), 'Simulation mode'),
        ('rviz', cfg_get(launch_cfg, 'runtime/rviz', True), 'Enable RViz'),
        # HH_260327: Stagger module includes to reduce startup CPU/memory spikes.
        ('module_launch_gap_sec', cfg_get(launch_cfg, 'runtime/module_launch_gap_sec', 0.8), 'Gap (seconds) between module launch includes'),

        ('use_eskf', cfg_get(launch_cfg, 'localization/use_eskf', True), 'Localization filter selector'),
        # HH_260326: New explicit selector for localization.launch.py (ekf/eskf).
        # Keep 'auto' to preserve backward compatibility with legacy use_eskf toggle.
        ('filter_type', cfg_get(launch_cfg, 'localization/filter_type', 'auto'), 'Localization filter type: auto|ekf|eskf'),
        ('wheel_bridge_enable', cfg_get(launch_cfg, 'localization/wheel_bridge_enable', True), 'Enable wheel bridge'),
        # HH_260326: Bringup-level wheel source wiring for unified wheel bridge.
        ('wheel_input_topic', cfg_get(launch_cfg, 'localization/wheel_input_topic', '/platform/status/wheel'), 'Wheel bridge input topic'),
        ('wheel_input_type', cfg_get(launch_cfg, 'localization/wheel_input_type', 'twist'), 'Wheel bridge input type: twist|avg_odom|nav_odom'),
        ('wheel_output_topic', cfg_get(launch_cfg, 'localization/wheel_output_topic', '/platform/wheel/odometry'), 'Unified wheel odometry topic (avg_msgs/Odometry)'),
        ('wheel_nav_output_topic', cfg_get(launch_cfg, 'localization/wheel_nav_output_topic', '/platform/wheel/nav_odometry'), 'Unified wheel odometry topic (nav_msgs/Odometry)'),
        ('eskf_force_rmp401_odom', cfg_get(launch_cfg, 'localization/eskf_force_rmp401_odom', True), 'Force ESKF wheel source to /rmp401/odom'),
        ('eskf_rmp401_odom_topic', cfg_get(launch_cfg, 'localization/eskf_rmp401_odom_topic', '/rmp401/odom'), 'Temporary ESKF wheel source topic'),
        ('kimera_bridge_enable', cfg_get(launch_cfg, 'localization/kimera_bridge_enable', False), 'Enable Kimera bridge'),
        ('pose_selector_enable', cfg_get(launch_cfg, 'localization/pose_selector_enable', False), 'Enable pose selector'),

        ('enable_path_cost_grids', cfg_get(launch_cfg, 'planning/enable_path_cost_grids', True), 'Enable path cost-grid helpers'),
        ('enable_goal_replanner', cfg_get(launch_cfg, 'planning/enable_goal_replanner', True), 'Enable goal replanner'),
        ('enable_nav2_lifecycle_retry', cfg_get(launch_cfg, 'planning/enable_nav2_lifecycle_retry', True), 'Enable Nav2 lifecycle retry'),
        # HH_260327: Hold Nav2 STARTUP until localization reports ready.
        ('require_localization_ready', cfg_get(launch_cfg, 'planning/require_localization_ready', True), 'Gate Nav2 STARTUP on localization readiness'),
        ('enable_state_machine', cfg_get(launch_cfg, 'planning/enable_state_machine', False), 'Enable planning state machine'),
        ('local_path_pose_topic', cfg_get(launch_cfg, 'planning/local_path_pose_topic', '/planning/lanelet_pose'), 'Pose topic for local_path_extractor'),
        ('local_path_source', cfg_get(launch_cfg, 'planning/local_path_source', 'slice_only'), 'Local path source policy: controller_then_slice|controller_only|slice_only'),
        ('use_dwb_controller', cfg_get(launch_cfg, 'planning/use_dwb_controller', False), 'Use Nav2 DWB profile'),
        # HH_260326: Declare explicitly so include-time LaunchConfiguration always exists.
        ('nav2_vehicle_param_file', cfg_get(
            launch_cfg,
            'planning/nav2_vehicle_param_file',
            bringup_cfg('planning/nav2_vehicle.yaml'),
        ), 'Nav2 vehicle profile param file'),

        ('enable_module_validators', cfg_get(launch_cfg, 'system/enable_module_validators', True), 'Enable module validators'),
        (
            'enable_plugin_api',
            cfg_get(launch_cfg, 'system/enable_plugin_api', True),
            'Enable plugin API bridge',
        ),
        ('enable_api_ui', cfg_get(launch_cfg, 'system/enable_api_ui', True), 'Enable API UI backend node'),
        ('api_ui_host', cfg_get(launch_cfg, 'system/api_ui_host', '0.0.0.0'), 'API UI backend bind host'),
        ('api_ui_port', cfg_get(launch_cfg, 'system/api_ui_port', 8010), 'API UI backend bind port'),

        ('enable_radar', cfg_get(launch_cfg, 'sensing/enable_radar', False), 'Enable serial radar'),
        ('enable_radar_cost_grid', cfg_get(launch_cfg, 'sensing/enable_radar_cost_grid', True), 'Enable radar cost-grid'),
        ('enable_lidar_cost_grid', cfg_get(launch_cfg, 'sensing/enable_lidar_cost_grid', True), 'Enable lidar cost-grid'),
        ('enable_lidar_driver', cfg_get(launch_cfg, 'sensing/enable_lidar_driver', False), 'Enable lidar driver'),
        ('enable_imu', cfg_get(launch_cfg, 'sensing/enable_imu', True), 'Enable IMU driver + velocity converter'),
        ('enable_gnss', cfg_get(launch_cfg, 'sensing/enable_gnss', False), 'Enable GNSS driver stack'),
        ('enable_ntrip', cfg_get(launch_cfg, 'sensing/enable_ntrip', False), 'Enable GNSS NTRIP client'),

        ('map_namespace', cfg_get(launch_cfg, 'namespaces/map', 'map'), 'Map namespace'),
        ('sensing_namespace', cfg_get(launch_cfg, 'namespaces/sensing', 'sensing'), 'Sensing namespace'),
        ('localization_namespace', cfg_get(launch_cfg, 'namespaces/localization', 'localization'), 'Localization namespace'),
        ('planning_namespace', cfg_get(launch_cfg, 'namespaces/planning', 'planning'), 'Planning namespace'),
        ('platform_namespace', cfg_get(launch_cfg, 'namespaces/platform', 'platform'), 'Platform namespace'),
        ('perception_namespace', cfg_get(launch_cfg, 'namespaces/perception', 'perception'), 'Perception namespace'),
        ('sensor_kit_namespace', cfg_get(launch_cfg, 'namespaces/sensor_kit', 'sensor_kit'), 'Sensor-kit namespace'),
        ('bringup_namespace', cfg_get(launch_cfg, 'namespaces/bringup', 'bringup'), 'Bringup namespace'),
        ('system_namespace', cfg_get(launch_cfg, 'namespaces/system', 'system'), 'System namespace'),
        ('gnss_namespace', cfg_get(launch_cfg, 'namespaces/gnss', 'sensing/gnss'), 'GNSS namespace'),

        ('gnss_navsatfix_topic', cfg_get(launch_cfg, 'topics/gnss_navsatfix', '/sensing/gnss/ublox_gps_node/fix'), 'GNSS navsatfix topic'),
        ('gnss_pose_topic', cfg_get(launch_cfg, 'topics/gnss_pose', '/sensing/gnss/pose'), 'GNSS pose topic'),
        ('gnss_pose_cov_topic', cfg_get(launch_cfg, 'topics/gnss_pose_cov', '/sensing/gnss/pose_with_covariance'), 'GNSS pose-with-cov topic'),
        ('gnss_rtcm_topic', cfg_get(launch_cfg, 'topics/gnss_rtcm', '/sensing/gnss/rtcm'), 'GNSS RTCM topic'),

        ('map_path', cfg_get(launch_cfg, 'map/map_path', str(map_params.get('map_path', ''))), 'Lanelet2 map path'),
        (
            'origin_lat',
            cfg_get(
                launch_cfg,
                'map/origin_lat',
                float(map_ref_llh.get('lat', map_params.get('offset_lat', 0.0))),
            ),
            'Map origin latitude',
        ),
        (
            'origin_lon',
            cfg_get(
                launch_cfg,
                'map/origin_lon',
                float(map_ref_llh.get('lon', map_params.get('offset_lon', 0.0))),
            ),
            'Map origin longitude',
        ),
        (
            'origin_alt',
            cfg_get(
                launch_cfg,
                'map/origin_alt',
                float(map_ref_llh.get('alt', map_params.get('offset_alt', 0.0))),
            ),
            'Map origin altitude',
        ),
        ('yaw_offset_deg', cfg_get(launch_cfg, 'map/yaw_offset_deg', yaw_default), 'GNSS->map yaw offset deg'),
        (
            'utm_origin_easting',
            cfg_get(launch_cfg, 'map/utm_origin_easting', float(map_ref_utm.get('easting', 0.0))),
            'UTM origin easting [m] for localization',
        ),
        (
            'utm_origin_northing',
            cfg_get(launch_cfg, 'map/utm_origin_northing', float(map_ref_utm.get('northing', 0.0))),
            'UTM origin northing [m] for localization',
        ),
        (
            'utm_origin_alt',
            cfg_get(launch_cfg, 'map/utm_origin_alt', float(map_ref_utm.get('alt', map_ref_llh.get('alt', 0.0)))),
            'UTM origin altitude [m] for localization',
        ),
        (
            'rotate_latlon_xy_by_yaw_offset',
            cfg_get(
                launch_cfg,
                'map/rotate_latlon_xy_by_yaw_offset',
                bool(map_params.get('rotate_latlon_xy_by_yaw_offset', True)),
            ),
            'Rotate LLH ENU XY by yaw_offset_deg',
        ),

        ('lanelet_id', cfg_get(launch_cfg, 'sim/lanelet_id', -1), 'Fake sensor lanelet id'),
        ('fake_lanelet_id', cfg_get(launch_cfg, 'sim/fake_lanelet_id', -1), 'Legacy fake lanelet id alias'),
        (
            'fake_sensors_param_file',
            cfg_get(
                launch_cfg,
                'sim/fake_sensors_param_file',
                bringup_cfg('sim/fake_sensors.yaml'),
            ),
            'Fake sensor parameter file path',
        ),
    ]

    args = [
        DeclareLaunchArgument(name, default_value=as_launch_default(default), description=desc)
        for name, default, desc in arg_specs
    ]

    lc = {name: LaunchConfiguration(name) for name, _, _ in arg_specs}

    # Keep parameter-file paths internal to bringup/module defaults.
    nav2_base = pkg_path('camrod_bringup', os.path.join('config', 'planning', 'nav2_base.yaml'))
    nav2_vehicle = pkg_path('camrod_bringup', os.path.join('config', 'planning', 'nav2_vehicle.yaml'))
    nav2_vehicle_dwb = pkg_path('camrod_bringup', os.path.join('config', 'planning', 'nav2_vehicle_dwb.yaml'))
    nav2_lanelet = pkg_path('camrod_bringup', os.path.join('config', 'planning', 'nav2_lanelet_overlay.yaml'))
    nav2_behavior = pkg_path('camrod_bringup', os.path.join('config', 'planning', 'nav2_behavior.yaml'))
    eskf_param = pkg_path('camrod_bringup', os.path.join('config', 'localization', 'eskf.yaml'))
    kimera_param = pkg_path('camrod_bringup', os.path.join('config', 'localization', 'kimera_bridge.yaml'))
    selector_param = pkg_path('camrod_bringup', os.path.join('config', 'localization', 'pose_selector.yaml'))
    # HH_260326: drop_zone_matcher node expects raw zone list YAML, not matcher param YAML.
    drop_zone_param = pkg_path('camrod_bringup', os.path.join('config', 'localization', 'drop_zones.yaml'))

    # HH_260326: Resolve filter_type with runtime backward compatibility.
    # Priority:
    # 1) filter_type=ekf|eskf
    # 2) filter_type=auto(or invalid) -> legacy use_eskf toggle
    resolved_filter_type = PythonExpression([
        "'ekf' if '", lc['filter_type'], "' == 'ekf' else "
        "('eskf' if '", lc['filter_type'], "' == 'eskf' else "
        "('eskf' if '", lc['use_eskf'], "' == 'true' else 'ekf'))"
    ])

    def include(pkg: str, launch_file: str, launch_args: dict, condition=None):
        kwargs = {
            'launch_description_source': PythonLaunchDescriptionSource(pkg_path(pkg, os.path.join('launch', launch_file))),
            'launch_arguments': launch_args.items(),
        }
        if condition is not None:
            kwargs['condition'] = condition
        return IncludeLaunchDescription(**kwargs)

    wheel_bridge_enable_sim_safe = PythonExpression([
        "'", lc['wheel_bridge_enable'], "' == 'true' and '", lc['sim'], "' == 'false'"
    ])

    modules = [
        include('camrod_platform', 'platform.launch.py', {
            'module_namespace': lc['platform_namespace'],
            'system_namespace': lc['system_namespace'],
            'sensor_kit_namespace': lc['sensor_kit_namespace'],
            'enable_module_validator': 'false',
            'publish_base_link_alias': 'true',
        }),
        include('camrod_map', 'map.launch.py', {
            'map_path': lc['map_path'],
            'origin_lat': lc['origin_lat'],
            'origin_lon': lc['origin_lon'],
            'origin_alt': lc['origin_alt'],
            'module_namespace': lc['map_namespace'],
            'system_namespace': lc['system_namespace'],
            'enable_module_validator': 'false',
        }),
        include('camrod_bringup', 'fake_sensors.launch.py', {
            'bringup_namespace': lc['bringup_namespace'],
            'sensing_namespace': lc['sensing_namespace'],
            # Keep fake-sensor profile selectable from top-level bringup.
            'fake_sensors_param_file': lc['fake_sensors_param_file'],
            'fake_enable_cost_grids': 'false',
            'map_path': lc['map_path'],
            'origin_lat': lc['origin_lat'],
            'origin_lon': lc['origin_lon'],
            'origin_alt': lc['origin_alt'],
            'lanelet_id': lc['lanelet_id'],
            'fake_lanelet_id': lc['fake_lanelet_id'],
        }, condition=IfCondition(lc['sim'])),
        include('camrod_sensing', 'sensing.launch.py', {
            # HH_260326: sensing.launch.py declares `sensing_namespace` (not `module_namespace`).
            # Keep key aligned so bringup namespace override is actually applied.
            'sensing_namespace': lc['sensing_namespace'],
            'system_namespace': lc['system_namespace'],
            'gnss_namespace': lc['gnss_namespace'],
            'gnss_navsatfix_topic': lc['gnss_navsatfix_topic'],
            'gnss_rtcm_topic': lc['gnss_rtcm_topic'],
            'enable_ntrip': lc['enable_ntrip'],
            'enable_radar': lc['enable_radar'],
            'enable_radar_cost_grid': lc['enable_radar_cost_grid'],
            'enable_lidar_cost_grid': lc['enable_lidar_cost_grid'],
            'enable_lidar_driver': lc['enable_lidar_driver'],
            # HH_260327: In sim mode, disable hardware IMU driver to avoid serial-port contention.
            'enable_imu': PythonExpression([
                "'false' if '", lc['sim'], "' == 'true' else '", lc['enable_imu'], "'"
            ]),
            'enable_gnss': lc['enable_gnss'],
            'enable_module_validator': 'false',
        }),
        include('camrod_perception', 'perception.launch.py', {
            'module_namespace': lc['perception_namespace'],
            'system_namespace': lc['system_namespace'],
            'enable_module_validator': 'false',
        }),
        include('camrod_localization', 'localization.launch.py', {
            'module_namespace': lc['localization_namespace'],
            'platform_namespace': lc['platform_namespace'],
            'system_namespace': lc['system_namespace'],
            'navsat_topic': lc['gnss_navsatfix_topic'],
            'gnss_pose_topic': lc['gnss_pose_topic'],
            'gnss_pose_cov_topic': lc['gnss_pose_cov_topic'],
            'origin_lat': lc['origin_lat'],
            'origin_lon': lc['origin_lon'],
            'origin_alt': lc['origin_alt'],
            'yaw_offset_deg': lc['yaw_offset_deg'],
            'utm_origin_easting': lc['utm_origin_easting'],
            'utm_origin_northing': lc['utm_origin_northing'],
            'utm_origin_alt': lc['utm_origin_alt'],
            'rotate_latlon_xy_by_yaw_offset': lc['rotate_latlon_xy_by_yaw_offset'],
            # HH_260326: Pass concrete filter type expected by localization.launch.py.
            'filter_type': resolved_filter_type,
            'eskf_param_file': eskf_param,
            'wheel_bridge_enable': wheel_bridge_enable_sim_safe,
            'wheel_input_topic': lc['wheel_input_topic'],
            'wheel_input_type': lc['wheel_input_type'],
            'wheel_output_topic': lc['wheel_output_topic'],
            'wheel_nav_output_topic': lc['wheel_nav_output_topic'],
            'eskf_force_rmp401_odom': lc['eskf_force_rmp401_odom'],
            'eskf_rmp401_odom_topic': lc['eskf_rmp401_odom_topic'],
            # HH_260326: localization.launch.py expects drop_zones_yaml.
            'drop_zones_yaml': drop_zone_param,
            'kimera_bridge_enable': lc['kimera_bridge_enable'],
            'kimera_bridge_param_file': kimera_param,
            'pose_selector_enable': lc['pose_selector_enable'],
            'pose_selector_param_file': selector_param,
            'enable_module_validator': 'false',
        }),
        include('camrod_planning', 'planning.launch.py', {
            'nav2_base_param_file': nav2_base,
            'nav2_vehicle_param_file': lc['nav2_vehicle_param_file'],
            'nav2_lanelet_param_file': nav2_lanelet,
            'nav2_behavior_param_file': nav2_behavior,
            'enable_path_cost_grids': lc['enable_path_cost_grids'],
            'enable_goal_replanner': lc['enable_goal_replanner'],
            'enable_nav2_lifecycle_retry': lc['enable_nav2_lifecycle_retry'],
            'require_localization_ready': lc['require_localization_ready'],
            'enable_state_machine': lc['enable_state_machine'],
            'map_path': lc['map_path'],
            'origin_lat': lc['origin_lat'],
            'origin_lon': lc['origin_lon'],
            'origin_alt': lc['origin_alt'],
            # In sim, use yaw-aligned pose generated by fake_sensor_publisher.
            # In real mode, keep native localization pose.
            'centerline_input_pose_topic': PythonExpression([
                "'/localization/pose_yaw_aligned' if '", lc['sim'],
                "' == 'true' else '/localization/pose'"
            ]),
            'local_path_pose_topic': lc['local_path_pose_topic'],
            'local_path_source': lc['local_path_source'],
            'module_namespace': lc['planning_namespace'],
            'system_namespace': lc['system_namespace'],
            'enable_module_validator': 'false',
        }),
        # HH_260326: camrod_system provides module_checkers.launch.py (not module_validators.launch.py).
        # HH_260327: Keep include always-on so diagnostics_agg can run even when checkers are disabled.
        include('camrod_system', 'module_checkers.launch.py', {
            'enable_checkers': lc['enable_module_validators'],
            'enable_diagnostics_aggregator': 'true',
            'enable_system_diagnostic': 'false',
            'system_namespace': lc['system_namespace'],
        }),
        include('camrod_api', 'api.launch.py', {
            'enable_plugin_api': lc['enable_plugin_api'],
            'enable_ui_backend': lc['enable_api_ui'],
            'ui_host': lc['api_ui_host'],
            'ui_port': lc['api_ui_port'],
        }),
    ]

    # HH_260326: Removed bringup_status runtime node from default launch path.

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', pkg_path('camrod_map', 'rviz/camrod_dark.rviz'),
            '-stylesheet', pkg_path('camrod_map', 'rviz/dark_theme.qss'),
        ],
        output='screen',
        additional_env={'QT_STYLE_OVERRIDE': 'Fusion'},
        condition=IfCondition(lc['rviz']),
    )

    cleanup_patterns = read_yaml(bringup_cfg('bringup/cleanup_patterns.yaml')).get('patterns', [])
    clean_action = ExecuteProcess(
        cmd=['bash', '-lc', build_cleanup_cmd(cleanup_patterns)],
        output='screen',
        condition=IfCondition(lc['clean_before_launch']),
    )

    # HH_260319-01: Sequence launch deterministically.
    # Do not start module stack until cleanup process exits; otherwise cleanup
    # can kill freshly launched nodes (race between pkill and stack bringup).
    init_nav2_vehicle_profile = SetLaunchConfiguration('nav2_vehicle_param_file', nav2_vehicle)
    apply_dwb_profile = SetLaunchConfiguration(
        'nav2_vehicle_param_file', nav2_vehicle_dwb, condition=IfCondition(lc['use_dwb_controller'])
    )

    # HH_260327: Start includes sequentially with configurable gap.
    staged_modules = [
        TimerAction(
            period=PythonExpression([str(idx), " * ", lc['module_launch_gap_sec']]),
            actions=[module_action],
        )
        for idx, module_action in enumerate(modules)
    ]
    launch_stack = GroupAction([*staged_modules, rviz_node])
    delayed_stack = TimerAction(period=1.0, actions=[launch_stack])
    start_stack_actions = [init_nav2_vehicle_profile, apply_dwb_profile, delayed_stack]

    start_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=clean_action,
            on_exit=start_stack_actions,
        ),
        condition=IfCondition(lc['clean_before_launch']),
    )

    start_without_cleanup = GroupAction(
        actions=start_stack_actions,
        condition=UnlessCondition(lc['clean_before_launch']),
    )

    return LaunchDescription([
        *args,
        clean_action,
        start_after_cleanup,
        start_without_cleanup,
    ])
