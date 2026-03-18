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
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
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
    map_params = read_yaml(bringup_cfg('map/map_info.yaml')).get('/map/lanelet2_map', {}).get('ros__parameters', {})
    yaw_default = float(
        read_yaml(bringup_cfg('localization/localization_origin.yaml')).get('origin', {}).get('yaw_offset_deg', 0.0)
    )

    # High-level arguments only.
    arg_specs = [
        ('clean_before_launch', cfg_get(launch_cfg, 'runtime/clean_before_launch', True), 'Kill stale processes first'),
        ('sim', cfg_get(launch_cfg, 'runtime/sim', True), 'Simulation mode'),
        ('rviz', cfg_get(launch_cfg, 'runtime/rviz', True), 'Enable RViz'),

        ('use_eskf', cfg_get(launch_cfg, 'localization/use_eskf', True), 'Localization filter selector'),
        ('wheel_bridge_enable', cfg_get(launch_cfg, 'localization/wheel_bridge_enable', True), 'Enable wheel bridge'),
        ('kimera_bridge_enable', cfg_get(launch_cfg, 'localization/kimera_bridge_enable', False), 'Enable Kimera bridge'),
        ('pose_selector_enable', cfg_get(launch_cfg, 'localization/pose_selector_enable', False), 'Enable pose selector'),

        ('enable_path_cost_grids', cfg_get(launch_cfg, 'planning/enable_path_cost_grids', True), 'Enable path cost-grid helpers'),
        ('enable_goal_replanner', cfg_get(launch_cfg, 'planning/enable_goal_replanner', True), 'Enable goal replanner'),
        ('enable_nav2_lifecycle_retry', cfg_get(launch_cfg, 'planning/enable_nav2_lifecycle_retry', True), 'Enable Nav2 lifecycle retry'),
        ('enable_state_machine', cfg_get(launch_cfg, 'planning/enable_state_machine', False), 'Enable planning state machine'),
        ('use_dwb_controller', cfg_get(launch_cfg, 'planning/use_dwb_controller', False), 'Use Nav2 DWB profile'),

        ('enable_module_checkers', cfg_get(launch_cfg, 'system/enable_module_checkers', True), 'Enable module checkers'),

        ('enable_radar', cfg_get(launch_cfg, 'sensing/enable_radar', False), 'Enable serial radar'),
        ('enable_radar_cost_grid', cfg_get(launch_cfg, 'sensing/enable_radar_cost_grid', True), 'Enable radar cost-grid'),
        ('enable_lidar_cost_grid', cfg_get(launch_cfg, 'sensing/enable_lidar_cost_grid', True), 'Enable lidar cost-grid'),
        ('enable_lidar_driver', cfg_get(launch_cfg, 'sensing/enable_lidar_driver', False), 'Enable lidar driver'),
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

        ('gnss_navsatfix_topic', cfg_get(launch_cfg, 'topics/gnss_navsatfix', '/sensing/gnss/navsatfix'), 'GNSS navsatfix topic'),
        ('gnss_pose_topic', cfg_get(launch_cfg, 'topics/gnss_pose', '/sensing/gnss/pose'), 'GNSS pose topic'),
        ('gnss_pose_cov_topic', cfg_get(launch_cfg, 'topics/gnss_pose_cov', '/sensing/gnss/pose_with_covariance'), 'GNSS pose-with-cov topic'),
        ('gnss_rtcm_topic', cfg_get(launch_cfg, 'topics/gnss_rtcm', '/sensing/gnss/rtcm'), 'GNSS RTCM topic'),

        ('map_path', cfg_get(launch_cfg, 'map/map_path', str(map_params.get('map_path', ''))), 'Lanelet2 map path'),
        ('origin_lat', cfg_get(launch_cfg, 'map/origin_lat', float(map_params.get('offset_lat', 0.0))), 'Map origin latitude'),
        ('origin_lon', cfg_get(launch_cfg, 'map/origin_lon', float(map_params.get('offset_lon', 0.0))), 'Map origin longitude'),
        ('origin_alt', cfg_get(launch_cfg, 'map/origin_alt', float(map_params.get('offset_alt', 0.0))), 'Map origin altitude'),
        ('yaw_offset_deg', cfg_get(launch_cfg, 'map/yaw_offset_deg', yaw_default), 'GNSS->map yaw offset deg'),

        ('lanelet_id', cfg_get(launch_cfg, 'sim/lanelet_id', -1), 'Fake sensor lanelet id'),
        ('fake_lanelet_id', cfg_get(launch_cfg, 'sim/fake_lanelet_id', -1), 'Legacy fake lanelet id alias'),
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
    fake_sensors_param = pkg_path('camrod_bringup', os.path.join('config', 'sim', 'fake_sensors.yaml'))
    eskf_param = pkg_path('camrod_bringup', os.path.join('config', 'localization', 'eskf.yaml'))
    supervisor_param = pkg_path('camrod_bringup', os.path.join('config', 'localization', 'supervisor.yaml'))
    kimera_param = pkg_path('camrod_bringup', os.path.join('config', 'localization', 'kimera_bridge.yaml'))
    selector_param = pkg_path('camrod_bringup', os.path.join('config', 'localization', 'pose_selector.yaml'))
    drop_zone_param = pkg_path('camrod_bringup', os.path.join('config', 'localization', 'drop_zone_matcher.yaml'))

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
            'enable_module_checker': 'false',
            'publish_base_link_alias': 'true',
        }),
        include('camrod_map', 'map.launch.py', {
            'map_path': lc['map_path'],
            'origin_lat': lc['origin_lat'],
            'origin_lon': lc['origin_lon'],
            'origin_alt': lc['origin_alt'],
            'module_namespace': lc['map_namespace'],
            'system_namespace': lc['system_namespace'],
            'enable_module_checker': 'false',
        }),
        include('camrod_bringup', 'fake_sensors.launch.py', {
            'bringup_namespace': lc['bringup_namespace'],
            'sensing_namespace': lc['sensing_namespace'],
            'fake_sensors_param_file': fake_sensors_param,
            'fake_enable_cost_grids': 'false',
            'map_path': lc['map_path'],
            'origin_lat': lc['origin_lat'],
            'origin_lon': lc['origin_lon'],
            'origin_alt': lc['origin_alt'],
            'lanelet_id': lc['lanelet_id'],
            'fake_lanelet_id': lc['fake_lanelet_id'],
        }, condition=IfCondition(lc['sim'])),
        include('camrod_sensing', 'sensing.launch.py', {
            'module_namespace': lc['sensing_namespace'],
            'system_namespace': lc['system_namespace'],
            'gnss_namespace': lc['gnss_namespace'],
            'gnss_navsatfix_topic': lc['gnss_navsatfix_topic'],
            'gnss_rtcm_topic': lc['gnss_rtcm_topic'],
            'enable_ntrip': lc['enable_ntrip'],
            'enable_radar': lc['enable_radar'],
            'enable_radar_cost_grid': lc['enable_radar_cost_grid'],
            'enable_lidar_cost_grid': lc['enable_lidar_cost_grid'],
            'enable_lidar_driver': lc['enable_lidar_driver'],
            'enable_gnss': lc['enable_gnss'],
            'enable_module_checker': 'false',
        }),
        include('camrod_perception', 'perception.launch.py', {
            'module_namespace': lc['perception_namespace'],
            'system_namespace': lc['system_namespace'],
            'enable_module_checker': 'false',
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
            'use_eskf': lc['use_eskf'],
            'eskf_param_file': eskf_param,
            'supervisor_param_file': supervisor_param,
            'wheel_bridge_enable': wheel_bridge_enable_sim_safe,
            'drop_zone_param_file': drop_zone_param,
            'kimera_bridge_enable': lc['kimera_bridge_enable'],
            'kimera_bridge_param_file': kimera_param,
            'pose_selector_enable': lc['pose_selector_enable'],
            'pose_selector_param_file': selector_param,
            'enable_module_checker': 'false',
        }),
        include('camrod_planning', 'planning.launch.py', {
            'nav2_base_param_file': nav2_base,
            'nav2_vehicle_param_file': LaunchConfiguration('nav2_vehicle_param_file'),
            'nav2_lanelet_param_file': nav2_lanelet,
            'nav2_behavior_param_file': nav2_behavior,
            'enable_path_cost_grids': lc['enable_path_cost_grids'],
            'enable_goal_replanner': lc['enable_goal_replanner'],
            'enable_nav2_lifecycle_retry': lc['enable_nav2_lifecycle_retry'],
            'enable_state_machine': lc['enable_state_machine'],
            'map_path': lc['map_path'],
            'origin_lat': lc['origin_lat'],
            'origin_lon': lc['origin_lon'],
            'origin_alt': lc['origin_alt'],
            'module_namespace': lc['planning_namespace'],
            'system_namespace': lc['system_namespace'],
            'enable_module_checker': 'false',
        }),
        include('camrod_system', 'module_checkers.launch.py', {
            'enable_checkers': 'true',
            'system_namespace': lc['system_namespace'],
        }, condition=IfCondition(lc['enable_module_checkers'])),
    ]

    bringup_diagnostic = Node(
        package='camrod_bringup',
        executable='bringup_diagnostic_node.py',
        name='bringup_diagnostic',
        namespace=lc['bringup_namespace'],
        output='screen',
        parameters=[{
            # HH_260318-00:00 Module-local diagnostic topic (namespaced).
            'diagnostic_topic': 'diagnostic',
            # Keep consolidated checker/system stream on /diagnostics.
            'source_diagnostic_topic': '/diagnostics',
            'publish_period_s': 0.5,
            'stale_timeout_s': 2.5,
            'sim': lc['sim'],
            'rviz': lc['rviz'],
            'use_eskf': lc['use_eskf'],
            'enable_goal_replanner': lc['enable_goal_replanner'],
            'enable_nav2_lifecycle_retry': lc['enable_nav2_lifecycle_retry'],
            'enable_state_machine': lc['enable_state_machine'],
            'enable_module_checkers': lc['enable_module_checkers'],
            'map_path': lc['map_path'],
            'origin_lat': lc['origin_lat'],
            'origin_lon': lc['origin_lon'],
            'origin_alt': lc['origin_alt'],
        }],
    )

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

    init_nav2_vehicle_profile = SetLaunchConfiguration('nav2_vehicle_param_file', nav2_vehicle)
    apply_dwb_profile = SetLaunchConfiguration(
        'nav2_vehicle_param_file', nav2_vehicle_dwb, condition=IfCondition(lc['use_dwb_controller'])
    )

    launch_stack = GroupAction([*modules, bringup_diagnostic, rviz_node])
    delayed_stack = TimerAction(period=1.0, actions=[launch_stack])

    return LaunchDescription([
        *args,
        clean_action,
        init_nav2_vehicle_profile,
        apply_dwb_profile,
        delayed_stack,
    ])
