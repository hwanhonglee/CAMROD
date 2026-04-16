"""Top-level orchestrator for camrod runtime.

HH_260317-00:00
- Keep only cross-package wiring + runtime toggles in bringup.
- Delegate detailed node params/composition to each module launch.
"""

import os
import subprocess
import sys
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit, OnShutdown
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


# HH_260330: Robustly extract lanelet map ros__parameters even when top-level key
# style changes ("/map/lanelet2_map", "map/lanelet2_map", nested map node, etc.).
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
    # Fallback: first dict that contains ros__parameters.
    for val in map_info_cfg.values():
        if isinstance(val, dict):
            params = val.get('ros__parameters')
            if isinstance(params, dict):
                return params
    return {}


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


# Runs cleanup command synchronously in launch context.
def run_cleanup_sync(cmd: str) -> None:
    if not isinstance(cmd, str) or not cmd.strip():
        return
    try:
        print('[HH_260408] bringup cleanup: terminating stale matching processes')
        subprocess.run(['bash', '-lc', cmd], check=False)
    except Exception:
        pass


# HH_260408: On Ctrl+C, execute one more cleanup pass to avoid stale duplicate nodes.
def run_cleanup_on_shutdown(_context: Any, cmd: str):
    run_cleanup_sync(cmd)
    return []


# Reads CLI launch arg value from `name:=value` form.
def cli_launch_arg(name: str) -> str:
    prefix = f'{name}:='
    for arg in sys.argv[1:]:
        if arg.startswith(prefix):
            return arg[len(prefix):]
    return ''


# Resolves config file path from absolute or config-root-relative input.
def resolve_cfg_file(config_root: str, raw_value: Any, default_rel: str) -> str:
    candidate = str(raw_value).strip() if raw_value is not None else ''
    if not candidate:
        candidate = default_rel
    if os.path.isabs(candidate):
        return candidate
    return os.path.join(config_root, candidate)


# Resolves config override path; returns empty string when not overridden.
def resolve_cfg_override(config_root: str, raw_value: Any) -> str:
    candidate = str(raw_value).strip() if raw_value is not None else ''
    if not candidate or candidate in ('__module_default__', 'module_default', 'default'):
        return ''
    if os.path.isabs(candidate):
        return candidate
    return os.path.join(config_root, candidate)


# Adds launch argument key/value only when value is non-empty.
def set_if_not_empty(args: dict, key: str, value: str) -> None:
    if isinstance(value, str) and value.strip():
        args[key] = value


def _first_existing_path(candidates: list[str]) -> str:
    seen = set()
    for candidate in candidates:
        if not isinstance(candidate, str):
            continue
        normalized = os.path.abspath(candidate)
        if normalized in seen:
            continue
        seen.add(normalized)
        if os.path.isfile(normalized):
            return normalized
    return ''


def resolve_map_path_default(
    launch_cfg_map_path: Any,
    map_info_map_path: Any,
    map_info_path: str,
    config_root: str,
) -> str:
    launch_cfg_value = str(launch_cfg_map_path).strip() if launch_cfg_map_path is not None else ''
    configured_path = ''
    if launch_cfg_value:
        configured_path = (
            launch_cfg_value
            if os.path.isabs(launch_cfg_value)
            else os.path.abspath(os.path.join(config_root, launch_cfg_value))
        )
    if configured_path and os.path.isfile(configured_path):
        return os.path.abspath(configured_path)

    map_info_value = str(map_info_map_path).strip() if map_info_map_path is not None else ''
    map_info_path_candidate = ''
    if map_info_value:
        if os.path.isabs(map_info_value):
            map_info_path_candidate = map_info_value
        else:
            map_info_path_candidate = os.path.abspath(
                os.path.join(os.path.dirname(map_info_path), map_info_value)
            )
        if os.path.isfile(map_info_path_candidate):
            return os.path.abspath(map_info_path_candidate)

    config_parent = os.path.abspath(os.path.join(config_root, '..'))
    auto_candidates = [
        os.path.join(config_parent, 'lanelet2_maps.osm'),
        os.path.join(os.path.dirname(config_parent), 'lanelet2_maps.osm'),
        os.path.join(os.path.expanduser('~'), 'camrod_ws', 'src', 'lanelet2_maps.osm'),
        os.path.join(os.getcwd(), 'lanelet2_maps.osm'),
        os.path.join(os.getcwd(), 'src', 'lanelet2_maps.osm'),
    ]
    discovered = _first_existing_path(auto_candidates)
    if discovered:
        return discovered

    if configured_path:
        return configured_path
    return map_info_path_candidate


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    pkg_config_root = pkg_path('camrod_bringup', 'config')
    cli_config_root = cli_launch_arg('config_root')
    env_config_root = os.environ.get('CAMROD_CONFIG_ROOT', '')
    src_config_root_guess = os.path.join(os.path.expanduser('~'), 'camrod_ws', 'src', 'camrod_bringup', 'config')
    # HH_260330: Prefer source-tree config root when provided so YAML edits apply
    # immediately without rebuild. Fallback to installed share path.
    config_root_default = cli_config_root or env_config_root or src_config_root_guess or pkg_config_root
    if not os.path.isdir(config_root_default):
        config_root_default = pkg_config_root
    config_root_default = os.path.abspath(config_root_default)

    bringup_cfg = lambda rel: os.path.join(config_root_default, rel)
    cli_launch_defaults_file = cli_launch_arg('launch_defaults_file')
    env_launch_defaults_file = os.environ.get('CAMROD_LAUNCH_DEFAULTS_FILE', '')
    launch_defaults_file_default = (
        cli_launch_defaults_file
        or env_launch_defaults_file
        or bringup_cfg('bringup/launch_defaults.yaml')
    )
    launch_cfg = read_yaml(launch_defaults_file_default).get('bringup', {})
    # HH_260407: Use camrod_map/config/map_info.yaml as the single source of truth.
    map_info_cfg_entry = cfg_get(launch_cfg, 'map/map_info_file', '__module_default__')
    if str(map_info_cfg_entry).strip() in ('', '__module_default__', 'module_default', 'default'):
        map_info_path = pkg_path('camrod_map', os.path.join('config', 'map_info.yaml'))
    else:
        map_info_path = resolve_cfg_file(config_root_default, map_info_cfg_entry, 'map/map_info.yaml')
    map_info = read_yaml(map_info_path)
    map_params = extract_map_ros_params(map_info)
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
    yaw_default = float(map_params.get('yaw_offset_deg', 0.0))
    planning_state_machine_keypoints_default = resolve_cfg_file(
        config_root_default,
        cfg_get(launch_cfg, 'planning/state_machine_keypoints_yaml', 'map/drop_zones.yaml'),
        'map/drop_zones.yaml',
    )
    planning_state_machine_camping_sites_default = resolve_cfg_file(
        config_root_default,
        cfg_get(
            launch_cfg,
            'planning/state_machine_camping_sites_yaml',
            'planning/camping_sites.yaml',
        ),
        'planning/camping_sites.yaml',
    )
    map_path_default = resolve_map_path_default(
        cfg_get(launch_cfg, 'map/map_path', ''),
        map_params.get('map_path', ''),
        map_info_path,
        config_root_default,
    )
    map_info_launch_default = (
        map_info_path
        if str(map_info_cfg_entry).strip() in ('', '__module_default__', 'module_default', 'default')
        else resolve_cfg_file(config_root_default, map_info_cfg_entry, 'map/map_info.yaml')
    )

    # High-level arguments only.
    arg_specs = [
        # HH_260330: Top-level YAML control points.
        ('config_root', config_root_default, 'Root directory for bringup YAML configs'),
        ('launch_defaults_file', launch_defaults_file_default, 'Top-level launch defaults YAML file'),
        ('clean_before_launch', cfg_get(launch_cfg, 'runtime/clean_before_launch', True), 'Kill stale processes first'),
        # HH_260408: Ensure stale duplicate processes are also cleaned on Ctrl+C shutdown.
        ('clean_on_shutdown', cfg_get(launch_cfg, 'runtime/clean_on_shutdown', True), 'Kill stale processes on bringup shutdown'),
        ('sim', cfg_get(launch_cfg, 'runtime/sim', True), 'Simulation mode'),
        ('rviz', cfg_get(launch_cfg, 'runtime/rviz', True), 'Enable RViz'),
        # HH_260327: Stagger module includes to reduce startup CPU/memory spikes.
        ('module_launch_gap_sec', cfg_get(launch_cfg, 'runtime/module_launch_gap_sec', 1.0), 'Gap (seconds) between module launch includes'),

        ('use_eskf', cfg_get(launch_cfg, 'localization/use_eskf', True), 'Localization filter selector'),
        # HH_260326: New explicit selector for localization.launch.py (ekf/eskf).
        # Keep 'auto' to preserve backward compatibility with legacy use_eskf toggle.
        ('filter_type', cfg_get(launch_cfg, 'localization/filter_type', 'auto'), 'Localization filter type: auto|ekf|eskf'),
        ('wheel_bridge_enable', cfg_get(launch_cfg, 'localization/wheel_bridge_enable', True), 'Enable wheel bridge'),
        # HH_260326: Bringup-level wheel source wiring for unified wheel bridge.
        ('wheel_input_topic', cfg_get(launch_cfg, 'localization/wheel_input_topic', '/platform/status/odometry'), 'Wheel bridge primary input topic'),
        ('wheel_input_type', cfg_get(launch_cfg, 'localization/wheel_input_type', 'nav_odom'), 'Wheel bridge primary input type: twist|avg_odom|nav_odom'),
        ('wheel_fallback_input_topic', cfg_get(launch_cfg, 'localization/wheel_fallback_input_topic', '/rmp401/odom'), 'Wheel bridge fallback input topic'),
        ('wheel_fallback_input_type', cfg_get(launch_cfg, 'localization/wheel_fallback_input_type', 'nav_odom'), 'Wheel bridge fallback input type: twist|avg_odom|nav_odom'),
        ('wheel_primary_timeout_sec', cfg_get(launch_cfg, 'localization/wheel_primary_timeout_sec', 0.7), 'Primary wheel timeout before fallback (sec)'),
        # HH_260410: Keep unified wheel output in /platform/status namespace.
        ('wheel_output_topic', cfg_get(launch_cfg, 'localization/wheel_output_topic', '/platform/status/wheel_odometry'), 'Unified wheel odometry topic (avg_msgs/Odometry)'),
        ('wheel_nav_output_topic', cfg_get(launch_cfg, 'localization/wheel_nav_output_topic', '/platform/wheel/nav_odometry'), 'Unified wheel odometry topic (nav_msgs/Odometry)'),
        ('eskf_force_rmp401_odom', cfg_get(launch_cfg, 'localization/eskf_force_rmp401_odom', False), 'Force ESKF wheel source to /rmp401/odom'),
        ('eskf_rmp401_odom_topic', cfg_get(launch_cfg, 'localization/eskf_rmp401_odom_topic', '/rmp401/odom'), 'Temporary ESKF wheel source topic'),
        ('kimera_bridge_enable', cfg_get(launch_cfg, 'localization/kimera_bridge_enable', False), 'Enable Kimera bridge'),
        ('pose_selector_enable', cfg_get(launch_cfg, 'localization/pose_selector_enable', False), 'Enable pose selector'),
        ('localization_enable_adapter', cfg_get(launch_cfg, 'localization/enable_adapter', True), 'Enable localization adapter launch'),
        ('localization_enable_filter', cfg_get(launch_cfg, 'localization/enable_filter', True), 'Enable localization filter launch'),
        ('localization_enable_monitor', cfg_get(launch_cfg, 'localization/enable_monitor', True), 'Enable localization monitor launch'),
        ('localization_enable_map_helper', cfg_get(launch_cfg, 'localization/enable_map_helper', True), 'Enable localization map_helper launch'),

        ('enable_path_cost_grids', cfg_get(launch_cfg, 'planning/enable_path_cost_grids', True), 'Enable path cost-grid helpers'),
        ('enable_goal_replanner', cfg_get(launch_cfg, 'planning/enable_goal_replanner', True), 'Enable goal replanner'),
        ('enable_nav2_lifecycle_retry', cfg_get(launch_cfg, 'planning/enable_nav2_lifecycle_retry', True), 'Enable Nav2 lifecycle retry'),
        # HH_260327: Hold Nav2 STARTUP until localization reports ready.
        ('require_localization_ready', cfg_get(launch_cfg, 'planning/require_localization_ready', True), 'Gate Nav2 STARTUP on localization readiness'),
        ('enable_state_machine', cfg_get(launch_cfg, 'planning/enable_state_machine', False), 'Enable planning state machine'),
        (
            'planning_state_machine_keypoints_yaml',
            planning_state_machine_keypoints_default,
            'Keypoints/drop-zone YAML for planning state machine',
        ),
        (
            'planning_state_machine_camping_sites_yaml',
            planning_state_machine_camping_sites_default,
            'Camping-sites YAML for planning state machine',
        ),
        ('local_path_pose_topic', cfg_get(launch_cfg, 'planning/local_path_pose_topic', '/localization/pose'), 'Pose topic for local_path_extractor'),
        ('local_path_source', cfg_get(launch_cfg, 'planning/local_path_source', 'controller_then_slice'), 'Local path source policy: controller_then_slice|controller_only|slice_only'),
        # HH_260331: Require explicit planning engage trigger before publishing /planning/cmd_vel.
        ('planning_cmd_vel_gate_enable', cfg_get(launch_cfg, 'planning/cmd_vel_gate_enable', True), 'Enable planning cmd_vel gate'),
        ('planning_cmd_vel_raw_topic', cfg_get(launch_cfg, 'planning/cmd_vel_raw_topic', '/planning/cmd_vel_raw'), 'Raw cmd_vel topic from Nav2 controller'),
        ('planning_cmd_vel_topic', cfg_get(launch_cfg, 'planning/cmd_vel_topic', '/planning/cmd_vel'), 'Final gated cmd_vel topic'),
        ('planning_engage_topic', cfg_get(launch_cfg, 'planning/engage_topic', '/planning/engage'), 'Planning engage trigger topic'),
        ('planning_engaged_state_topic', cfg_get(launch_cfg, 'planning/engaged_state_topic', '/planning/engaged'), 'Planning engage state topic'),
        ('planning_cmd_vel_gate_use_estop_topic', cfg_get(launch_cfg, 'planning/cmd_vel_gate_use_estop_topic', True), 'Use estop topic in planning cmd_vel gate'),
        # HH_260409: Default planning gate e-stop source is platform status topic.
        ('planning_cmd_vel_gate_estop_topic', cfg_get(launch_cfg, 'planning/cmd_vel_gate_estop_topic', '/platform/status/estop'), 'Planning estop topic'),
        ('planning_cmd_vel_gate_allow_on_start', cfg_get(launch_cfg, 'planning/cmd_vel_gate_allow_on_start', False), 'Allow planning cmd_vel on startup without engage'),
        # HH_260413: Cost-based stop parameters for planning cmd_vel gate.
        ('planning_cmd_vel_gate_cost_stop_enable', cfg_get(launch_cfg, 'planning/cmd_vel_gate_cost_stop_enable', True), 'Enable cost-based stop in cmd_vel gate'),
        ('planning_cmd_vel_gate_cost_grid_topic', cfg_get(launch_cfg, 'planning/cmd_vel_gate_cost_grid_topic', '/planning/local_costmap/costmap'), 'Cost grid topic for cost-stop'),
        ('planning_cmd_vel_gate_cost_pose_topic', cfg_get(launch_cfg, 'planning/cmd_vel_gate_cost_pose_topic', '/localization/pose'), 'Pose topic for cost-stop'),
        ('planning_cmd_vel_gate_cost_threshold', cfg_get(launch_cfg, 'planning/cmd_vel_gate_cost_threshold', 200), 'Cost threshold for stop'),
        ('planning_cmd_vel_gate_cost_lookahead_m', cfg_get(launch_cfg, 'planning/cmd_vel_gate_cost_lookahead_m', 2.0), 'Lookahead distance for cost-stop'),
        ('planning_cmd_vel_gate_cost_width_m', cfg_get(launch_cfg, 'planning/cmd_vel_gate_cost_width_m', 1.0), 'Corridor width for cost-stop'),
        ('planning_cmd_vel_gate_cost_hold_sec', cfg_get(launch_cfg, 'planning/cmd_vel_gate_cost_hold_sec', 1.0), 'Hold duration for cost-stop'),
        ('planning_cmd_vel_gate_unavoidable_stop_enable', cfg_get(launch_cfg, 'planning/cmd_vel_gate_unavoidable_stop_enable', True), 'Enable unavoidable stop cluster check'),
        ('planning_cmd_vel_gate_unavoidable_lethal_threshold', cfg_get(launch_cfg, 'planning/cmd_vel_gate_unavoidable_lethal_threshold', 253), 'Lethal cost threshold for unavoidable stop'),
        ('planning_cmd_vel_gate_unavoidable_cluster_min_cells', cfg_get(launch_cfg, 'planning/cmd_vel_gate_unavoidable_cluster_min_cells', 25), 'Min cluster cells for unavoidable stop'),
        ('planning_cmd_vel_gate_unavoidable_cluster_min_ratio', cfg_get(launch_cfg, 'planning/cmd_vel_gate_unavoidable_cluster_min_ratio', 0.25), 'Min cluster ratio for unavoidable stop'),

        ('enable_module_validators', cfg_get(launch_cfg, 'system/enable_module_validators', True), 'Enable module validators'),
        (
            'diagnostics_profile',
            cfg_get(launch_cfg, 'system/diagnostics_profile', cfg_get(launch_cfg, 'system/legacy_robot_profile', 'default')),
            'Diagnostics config profile name',
        ),
        (
            'enable_platform_checker',
            cfg_get(launch_cfg, 'system/enable_platform_checker', cfg_get(launch_cfg, 'system/legacy_enable_platform_checker', False)),
            'Enable diagnostics platform checker (requires ranger_msgs)',
        ),
        (
            'enable_plugin_api',
            cfg_get(launch_cfg, 'system/enable_plugin_api', True),
            'Enable plugin API bridge',
        ),
        ('enable_api_ui', cfg_get(launch_cfg, 'system/enable_api_ui', True), 'Enable API UI backend node'),
        ('api_ui_host', cfg_get(launch_cfg, 'system/api_ui_host', '127.0.0.1'), 'API UI backend bind host'),
        ('api_ui_port', cfg_get(launch_cfg, 'system/api_ui_port', 8010), 'API UI backend bind port'),

        ('enable_radar', cfg_get(launch_cfg, 'sensing/enable_radar', False), 'Enable serial radar'),
        ('enable_radar_cost_grid', cfg_get(launch_cfg, 'sensing/enable_radar_cost_grid', True), 'Enable radar cost-grid'),
        ('enable_lidar_cost_grid', cfg_get(launch_cfg, 'sensing/enable_lidar_cost_grid', True), 'Enable lidar cost-grid'),
        ('enable_lidar_driver', cfg_get(launch_cfg, 'sensing/enable_lidar_driver', False), 'Enable lidar driver'),
        ('enable_imu', cfg_get(launch_cfg, 'sensing/enable_imu', True), 'Enable IMU driver + velocity converter'),
        ('enable_gnss', cfg_get(launch_cfg, 'sensing/enable_gnss', False), 'Enable GNSS driver stack'),
        ('enable_ntrip', cfg_get(launch_cfg, 'sensing/enable_ntrip', False), 'Enable GNSS NTRIP client'),
        ('perception_enable_lidar_obstacle', cfg_get(launch_cfg, 'perception/enable_lidar_obstacle', True), 'Enable perception LiDAR obstacle node'),

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
        # HH_260331: Real-platform cmd_vel policy knobs.
        ('platform_cmd_vel_gate_enable', cfg_get(launch_cfg, 'platform/cmd_vel_gate_enable', False), 'Enable cmd_vel gate in platform module'),
        ('platform_cmd_vel_in_topic', cfg_get(launch_cfg, 'platform/cmd_vel_in_topic', '/planning/cmd_vel'), 'Platform cmd_vel gate input topic'),
        ('platform_cmd_vel_out_topic', cfg_get(launch_cfg, 'platform/cmd_vel_out_topic', '/platform/cmd_vel'), 'Platform cmd_vel gate output topic'),
        # HH_260410: Ranger base node is the single source for /platform/status/* outputs.
        ('platform_estop_topic', cfg_get(launch_cfg, 'platform/estop_topic', '/platform/status/estop'), 'Platform e-stop status topic'),
        # HH_260410: Keep platform top launch lean. Ranger detailed params are in ranger_params_file.
        ('platform_ranger_driver_enable', cfg_get(launch_cfg, 'platform/ranger_driver_enable', True), 'Enable camrod_platform ranger wrapper include'),
        (
            'platform_ranger_params_file',
            cfg_get(launch_cfg, 'platform/ranger_params_file', '__module_default__'),
            'Ranger parameter YAML passed to camrod_platform/launch/ranger.launch.py',
        ),

        ('map_path', map_path_default, 'Lanelet2 map path'),
        ('map_info_file', map_info_launch_default, 'Map info YAML path used by map/localization'),
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
    ]

    args = [
        DeclareLaunchArgument(name, default_value=as_launch_default(default), description=desc)
        for name, default, desc in arg_specs
    ]

    lc = {name: LaunchConfiguration(name) for name, _, _ in arg_specs}

    # HH_260331: Module config defaults-first policy.
    # Bringup passes file paths only when an explicit override is configured.
    localization_adapter_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'localization/adapter_param_file', ''))
    localization_filter_ekf_override = resolve_cfg_override(
        config_root_default,
        cfg_get(launch_cfg, 'localization/filter_ekf_param_file', cfg_get(launch_cfg, 'localization/ekf_param_file', '')),
    )
    localization_filter_eskf_override = resolve_cfg_override(
        config_root_default,
        cfg_get(launch_cfg, 'localization/filter_eskf_param_file', cfg_get(launch_cfg, 'localization/eskf_param_file', '')),
    )
    localization_drop_zones_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'localization/drop_zones_yaml', ''))
    localization_filter_kimera_override = resolve_cfg_override(
        config_root_default,
        cfg_get(launch_cfg, 'localization/filter_kimera_param_file', cfg_get(launch_cfg, 'localization/kimera_bridge_param_file', '')),
    )
    localization_filter_pose_selector_override = resolve_cfg_override(
        config_root_default,
        cfg_get(launch_cfg, 'localization/filter_pose_selector_param_file', cfg_get(launch_cfg, 'localization/pose_selector_param_file', '')),
    )
    localization_monitor_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'localization/monitor_param_file', ''))
    localization_map_helper_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'localization/map_helper_param_file', ''))
    localization_map_info_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'localization/map_info_file', ''))

    planning_nav2_base_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'planning/nav2_base_param_file', ''))
    planning_nav2_vehicle_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'planning/nav2_vehicle_param_file', ''))
    planning_nav2_vehicle_dwb_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'planning/nav2_vehicle_dwb_param_file', ''))
    planning_nav2_lanelet_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'planning/nav2_lanelet_param_file', ''))
    planning_nav2_behavior_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'planning/nav2_behavior_param_file', ''))
    planning_local_path_extractor_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'planning/local_path_extractor_param_file', ''))
    planning_path_cost_grids_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'planning/path_cost_grids_param_file', ''))
    planning_goal_snapper_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'planning/goal_snapper_param_file', ''))
    planning_centerline_snapper_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'planning/centerline_snapper_param_file', ''))
    planning_goal_replanner_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'planning/goal_replanner_param_file', ''))
    planning_state_machine_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'planning/planning_state_machine_param_file', ''))
    use_dwb_profile = bool(cfg_get(launch_cfg, 'planning/use_dwb_controller', False))
    selected_nav2_vehicle_override = (
        planning_nav2_vehicle_dwb_override if use_dwb_profile and planning_nav2_vehicle_dwb_override
        else planning_nav2_vehicle_override
    )

    sensing_common_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'sensing/sensing_param_file', ''))
    sensing_lidar_preprocess_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'sensing/lidar_preprocess_param_file', ''))
    sensing_camera_preprocess_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'sensing/camera_preprocess_param_file', ''))
    sensing_imu_converter_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'sensing/imu_converter_param_file', ''))
    sensing_radar_sensor_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'sensing/radar_sensor_param_file', ''))
    sensing_radar_cost_grid_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'sensing/radar_cost_grid_param_file', ''))
    sensing_lidar_cost_grid_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'sensing/lidar_cost_grid_param_file', ''))
    sensing_gnss_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'sensing/gnss_param_file', ''))
    sensing_ntrip_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'sensing/ntrip_param_file', ''))
    sensing_cv7_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'sensing/cv7_param_file', ''))
    sensing_gq7_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'sensing/gq7_param_file', ''))
    sensing_vanjee_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'sensing/vanjee_config_path', ''))

    platform_params_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'platform/params_file', ''))
    platform_robot_viz_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'platform/robot_visualization_param_file', ''))
    platform_ranger_params_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'platform/ranger_params_file', ''))
    map_info_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'map/map_info_file', ''))
    map_param_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'map/map_param_file', ''))
    map_lanelet_cost_grid_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'map/lanelet_cost_grid_param_file', ''))
    map_visualization_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'map/map_visualization_param_file', ''))
    perception_param_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'perception/perception_param_file', ''))
    fake_sensors_param_override = resolve_cfg_override(
        config_root_default, cfg_get(launch_cfg, 'sim/fake_sensors_param_file', ''))

    def include(pkg: str, launch_file: str, launch_args: dict, condition=None):
        kwargs = {
            'launch_description_source': PythonLaunchDescriptionSource(pkg_path(pkg, os.path.join('launch', launch_file))),
            'launch_arguments': launch_args.items(),
        }
        if condition is not None:
            kwargs['condition'] = condition
        return IncludeLaunchDescription(**kwargs)

    platform_args = {
        'module_namespace': lc['platform_namespace'],
        'sensor_kit_namespace': lc['sensor_kit_namespace'],
        # HH_260331: Keep gate enabled in sim for fake-motion loop;
        # default to launch-default policy on real platform.
        'cmd_vel_gate_enable': PythonExpression([
            "'true' if '", lc['sim'], "' == 'true' else '", lc['platform_cmd_vel_gate_enable'], "'"
        ]),
        'cmd_vel_in_topic': lc['platform_cmd_vel_in_topic'],
        'cmd_vel_out_topic': lc['platform_cmd_vel_out_topic'],
        'estop_topic': lc['platform_estop_topic'],
        # HH_260410: In sim, disable external CAN driver by default.
        'ranger_driver_enable': PythonExpression([
            "'false' if '", lc['sim'], "' == 'true' else '", lc['platform_ranger_driver_enable'], "'"
        ]),
    }
    set_if_not_empty(platform_args, 'params_file', platform_params_override)
    set_if_not_empty(platform_args, 'robot_visualization_param_file', platform_robot_viz_override)
    set_if_not_empty(platform_args, 'ranger_params_file', platform_ranger_params_override)

    map_args = {
        'map_info_file': lc['map_info_file'],
        'map_path': lc['map_path'],
        'origin_lat': lc['origin_lat'],
        'origin_lon': lc['origin_lon'],
        'origin_alt': lc['origin_alt'],
        'module_namespace': lc['map_namespace'],
        'system_namespace': lc['system_namespace'],
        'enable_module_validator': 'false',
    }
    set_if_not_empty(map_args, 'map_info_file', map_info_override)
    set_if_not_empty(map_args, 'map_param_file', map_param_override)
    set_if_not_empty(map_args, 'lanelet_cost_grid_param_file', map_lanelet_cost_grid_override)
    set_if_not_empty(map_args, 'map_visualization_param_file', map_visualization_override)

    fake_sensors_args = {
        'bringup_namespace': lc['bringup_namespace'],
        'sensing_namespace': lc['sensing_namespace'],
        'fake_enable_cost_grids': 'false',
        'map_path': lc['map_path'],
        'origin_lat': lc['origin_lat'],
        'origin_lon': lc['origin_lon'],
        'origin_alt': lc['origin_alt'],
        'lanelet_id': lc['lanelet_id'],
        'fake_lanelet_id': lc['fake_lanelet_id'],
    }
    set_if_not_empty(fake_sensors_args, 'fake_sensors_param_file', fake_sensors_param_override)

    sensing_args = {
        # HH_260326: sensing.launch.py declares `sensing_namespace` (not `module_namespace`).
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
        'enable_imu': lc['enable_imu'],
        'enable_gnss': lc['enable_gnss'],
        'enable_module_validator': 'false',
    }
    set_if_not_empty(sensing_args, 'sensing_param_file', sensing_common_override)
    set_if_not_empty(sensing_args, 'lidar_preprocess_param_file', sensing_lidar_preprocess_override)
    set_if_not_empty(sensing_args, 'camera_preprocess_param_file', sensing_camera_preprocess_override)
    set_if_not_empty(sensing_args, 'imu_converter_param_file', sensing_imu_converter_override)
    set_if_not_empty(sensing_args, 'radar_sensor_param_file', sensing_radar_sensor_override)
    set_if_not_empty(sensing_args, 'radar_cost_grid_param_file', sensing_radar_cost_grid_override)
    set_if_not_empty(sensing_args, 'lidar_cost_grid_param_file', sensing_lidar_cost_grid_override)
    set_if_not_empty(sensing_args, 'gnss_param_file', sensing_gnss_override)
    set_if_not_empty(sensing_args, 'ntrip_param_file', sensing_ntrip_override)
    set_if_not_empty(sensing_args, 'cv7_param_file', sensing_cv7_override)
    set_if_not_empty(sensing_args, 'gq7_param_file', sensing_gq7_override)
    set_if_not_empty(sensing_args, 'vanjee_config_path', sensing_vanjee_override)

    perception_args = {
        'module_namespace': lc['perception_namespace'],
        'enable_lidar_obstacle': lc['perception_enable_lidar_obstacle'],
    }
    set_if_not_empty(perception_args, 'perception_param_file', perception_param_override)

    # HH_260409: Legacy escape hatch for forced ESKF source.
    # HH_260413: Also auto-force /rmp401/odom when ranger driver is disabled.
    # This keeps wheel input valid in no-platform(real CAN absent) bringup runs.
    eskf_force_rmp401_effective = PythonExpression([
        "'true' if (('",
        lc['eskf_force_rmp401_odom'],
        "' == 'true') or ('",
        lc['platform_ranger_driver_enable'],
        "' != 'true')) else 'false'"
    ])

    # Default path is primary /platform/status/* + runtime fallback /rmp401/odom.
    wheel_input_topic_for_filter = PythonExpression([
        "'",
        lc['eskf_rmp401_odom_topic'],
        "' if (('",
        eskf_force_rmp401_effective,
        "' == 'true') and (('",
        lc['filter_type'],
        "' == 'eskf') or (('",
        lc['filter_type'],
        "' == 'auto') and ('",
        lc['use_eskf'],
        "' == 'true')))) else '",
        lc['wheel_input_topic'],
        "'",
    ])
    wheel_input_type_for_filter = PythonExpression([
        "'nav_odom' if (('",
        eskf_force_rmp401_effective,
        "' == 'true') and (('",
        lc['filter_type'],
        "' == 'eskf') or (('",
        lc['filter_type'],
        "' == 'auto') and ('",
        lc['use_eskf'],
        "' == 'true')))) else '",
        lc['wheel_input_type'],
        "'",
    ])

    localization_args = {
        'module_namespace': lc['localization_namespace'],
        'enable_adapter': lc['localization_enable_adapter'],
        'enable_filter': lc['localization_enable_filter'],
        'enable_monitor': lc['localization_enable_monitor'],
        'enable_map_helper': lc['localization_enable_map_helper'],
        'enable_kimera_bridge': lc['kimera_bridge_enable'],
        'filter_type': lc['filter_type'],
        'use_eskf': lc['use_eskf'],
        'wheel_bridge_enable': lc['wheel_bridge_enable'],
        'wheel_input_topic': wheel_input_topic_for_filter,
        'wheel_input_type': wheel_input_type_for_filter,
        'wheel_fallback_input_topic': lc['wheel_fallback_input_topic'],
        'wheel_fallback_input_type': lc['wheel_fallback_input_type'],
        'wheel_primary_timeout_sec': lc['wheel_primary_timeout_sec'],
        'wheel_output_topic': lc['wheel_output_topic'],
        'wheel_nav_output_topic': lc['wheel_nav_output_topic'],
        'map_path': lc['map_path'],
    }
    set_if_not_empty(localization_args, 'adapter_param_file', localization_adapter_override)
    set_if_not_empty(localization_args, 'filter_ekf_param_file', localization_filter_ekf_override)
    set_if_not_empty(localization_args, 'filter_eskf_param_file', localization_filter_eskf_override)
    set_if_not_empty(localization_args, 'filter_pose_selector_param_file', localization_filter_pose_selector_override)
    set_if_not_empty(localization_args, 'filter_kimera_param_file', localization_filter_kimera_override)
    set_if_not_empty(localization_args, 'monitor_param_file', localization_monitor_override)
    set_if_not_empty(localization_args, 'map_helper_param_file', localization_map_helper_override)
    set_if_not_empty(localization_args, 'map_info_file', localization_map_info_override)
    set_if_not_empty(localization_args, 'drop_zones_yaml', localization_drop_zones_override)

    planning_args = {
        'enable_path_cost_grids': lc['enable_path_cost_grids'],
        'enable_goal_replanner': lc['enable_goal_replanner'],
        'enable_nav2_lifecycle_retry': lc['enable_nav2_lifecycle_retry'],
        'require_localization_ready': lc['require_localization_ready'],
        'enable_state_machine': lc['enable_state_machine'],
        'planning_state_machine_keypoints_yaml': lc['planning_state_machine_keypoints_yaml'],
        'planning_state_machine_camping_sites_yaml': lc['planning_state_machine_camping_sites_yaml'],
        'map_path': lc['map_path'],
        'origin_lat': lc['origin_lat'],
        'origin_lon': lc['origin_lon'],
        'origin_alt': lc['origin_alt'],
        # Keep centerline anchor on fused localization pose.
        'centerline_input_pose_topic': '/localization/pose',
        'local_path_pose_topic': lc['local_path_pose_topic'],
        'local_path_source': lc['local_path_source'],
        'cmd_vel_gate_enable': lc['planning_cmd_vel_gate_enable'],
        'cmd_vel_raw_topic': lc['planning_cmd_vel_raw_topic'],
        'cmd_vel_output_topic': lc['planning_cmd_vel_topic'],
        'planning_engage_topic': lc['planning_engage_topic'],
        'planning_engaged_state_topic': lc['planning_engaged_state_topic'],
        'cmd_vel_gate_use_estop_topic': lc['planning_cmd_vel_gate_use_estop_topic'],
        'cmd_vel_gate_estop_topic': lc['planning_cmd_vel_gate_estop_topic'],
        'cmd_vel_gate_allow_on_start': lc['planning_cmd_vel_gate_allow_on_start'],
        'cmd_vel_gate_cost_stop_enable': lc['planning_cmd_vel_gate_cost_stop_enable'],
        'cmd_vel_gate_cost_grid_topic': lc['planning_cmd_vel_gate_cost_grid_topic'],
        'cmd_vel_gate_cost_pose_topic': lc['planning_cmd_vel_gate_cost_pose_topic'],
        'cmd_vel_gate_cost_threshold': lc['planning_cmd_vel_gate_cost_threshold'],
        'cmd_vel_gate_cost_lookahead_m': lc['planning_cmd_vel_gate_cost_lookahead_m'],
        'cmd_vel_gate_cost_width_m': lc['planning_cmd_vel_gate_cost_width_m'],
        'cmd_vel_gate_cost_hold_sec': lc['planning_cmd_vel_gate_cost_hold_sec'],
        'cmd_vel_gate_unavoidable_stop_enable': lc['planning_cmd_vel_gate_unavoidable_stop_enable'],
        'cmd_vel_gate_unavoidable_lethal_threshold': lc['planning_cmd_vel_gate_unavoidable_lethal_threshold'],
        'cmd_vel_gate_unavoidable_cluster_min_cells': lc['planning_cmd_vel_gate_unavoidable_cluster_min_cells'],
        'cmd_vel_gate_unavoidable_cluster_min_ratio': lc['planning_cmd_vel_gate_unavoidable_cluster_min_ratio'],
        'module_namespace': lc['planning_namespace'],
    }
    set_if_not_empty(planning_args, 'nav2_base_param_file', planning_nav2_base_override)
    set_if_not_empty(planning_args, 'nav2_vehicle_param_file', selected_nav2_vehicle_override)
    set_if_not_empty(planning_args, 'nav2_lanelet_param_file', planning_nav2_lanelet_override)
    set_if_not_empty(planning_args, 'nav2_behavior_param_file', planning_nav2_behavior_override)
    set_if_not_empty(planning_args, 'path_cost_grids_param_file', planning_path_cost_grids_override)
    set_if_not_empty(planning_args, 'goal_snapper_param_file', planning_goal_snapper_override)
    set_if_not_empty(planning_args, 'centerline_snapper_param_file', planning_centerline_snapper_override)
    set_if_not_empty(planning_args, 'goal_replanner_param_file', planning_goal_replanner_override)
    set_if_not_empty(planning_args, 'local_path_extractor_param_file', planning_local_path_extractor_override)
    set_if_not_empty(planning_args, 'planning_state_machine_param_file', planning_state_machine_override)

    system_args = {
        'enable_checkers': lc['enable_module_validators'],
        'config_profile': lc['diagnostics_profile'],
        'enable_platform': lc['enable_platform_checker'],
        'module_namespace': lc['system_namespace'],
    }

    api_args = {
        'enable_plugin_api': lc['enable_plugin_api'],
        'enable_ui_backend': lc['enable_api_ui'],
        'ui_host': lc['api_ui_host'],
        'ui_port': lc['api_ui_port'],
    }

    modules = [
        include('camrod_platform', 'platform.launch.py', platform_args),
        include('camrod_map', 'map.launch.py', map_args),
        include('camrod_bringup', 'fake_sensors.launch.py', fake_sensors_args, condition=IfCondition(lc['sim'])),
        include('camrod_sensing', 'sensing.launch.py', sensing_args),
        include('camrod_perception', 'perception.launch.py', perception_args),
        include('camrod_localization', 'localization.launch.py', localization_args),
        include('camrod_planning', 'planning.launch.py', planning_args),
        # HH_260407: Launch unified diagnostics stack via top-level system.launch.py.
        include('camrod_system', 'system.launch.py', system_args),
        include('camrod_api', 'api.launch.py', api_args),
    ]

    # HH_260326: Removed bringup_status runtime node from default launch path.

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', pkg_path('camrod_map', 'rviz/camrod_operator.rviz'),
            '-stylesheet', pkg_path('camrod_map', 'rviz/operator_theme.qss'),
        ],
        output='screen',
        # HH_260415: Auto-restart RViz when plugin/TF race causes transient crash.
        respawn=True,
        respawn_delay=2.0,
        additional_env={'QT_STYLE_OVERRIDE': 'Fusion'},
        condition=IfCondition(lc['rviz']),
    )

    cleanup_patterns = read_yaml(bringup_cfg('bringup/cleanup_patterns.yaml')).get('patterns', [])
    cleanup_cmd = build_cleanup_cmd(cleanup_patterns)
    clean_action = ExecuteProcess(
        cmd=['bash', '-lc', cleanup_cmd],
        output='screen',
        condition=IfCondition(lc['clean_before_launch']),
    )

    # HH_260319-01: Sequence launch deterministically.
    # Do not start module stack until cleanup process exits; otherwise cleanup
    # can kill freshly launched nodes (race between pkill and stack bringup).
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
    start_stack_actions = [delayed_stack]

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

    shutdown_cleanup = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[OpaqueFunction(function=run_cleanup_on_shutdown, args=[cleanup_cmd])],
        ),
        condition=IfCondition(lc['clean_on_shutdown']),
    )

    return LaunchDescription([
        *args,
        clean_action,
        start_after_cleanup,
        start_without_cleanup,
        shutdown_cleanup,
    ])
