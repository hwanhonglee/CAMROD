"""Top-level orchestrator for camrod runtime.

- Keep only cross-package wiring + runtime toggles in bringup.
- Delegate detailed node params/composition to each module launch.
"""

import os
import re
import subprocess
import sys
from typing import Any

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

_MISSING = object()

# HH_260617: Optional modules are skipped on x86 when their launch files remain
# installed but their native/third-party executables are unavailable.
OPTIONAL_MODULE_EXECUTABLES = {
    'camrod_voice': (
        ('camrod_voice', 'voice_announcer_node'),
        ('camrod_voice', 'voice_event_adapter_node'),
    ),
}

# Override resolution specs:
# key   = launch argument key to pass to module launch.
# value = tuple of bringup config lookup paths, using missing-only fallback order.
OVERRIDE_SPECS = {
    'localization': {
        'adapter_param_file': ('localization/adapter_param_file',),
        'filter_ekf_param_file': ('localization/filter_ekf_param_file',),
        'filter_gnss_reattach_param_file': ('localization/filter_gnss_reattach_param_file',),
        'drop_zones_yaml': ('localization/drop_zones_yaml',),
        'filter_pose_selector_param_file': ('localization/filter_pose_selector_param_file',),
        'monitor_param_file': ('localization/monitor_param_file',),
        'map_helper_param_file': ('localization/map_helper_param_file',),
        'map_info_file': ('localization/map_info_file',),
    },
    'planning': {
        'nav2_base_param_file': ('planning/nav2_base_param_file',),
        'nav2_vehicle_param_file': ('planning/nav2_vehicle_param_file',),
        'nav2_vehicle_dwb_param_file': ('planning/nav2_vehicle_dwb_param_file',),
        'nav2_lanelet_param_file': ('planning/nav2_lanelet_param_file',),
        'nav2_behavior_param_file': ('planning/nav2_behavior_param_file',),
        'nav2_combo_param_file': ('planning/nav2_combo_param_file',),
        'nav2_planner_plugins_param_file': (
            'planning/nav2_planner_plugins_param_file',
        ),
        'nav2_controller_plugins_param_file': (
            'planning/nav2_controller_plugins_param_file',
        ),
        'local_path_extractor_param_file': ('planning/local_path_extractor_param_file',),
        'path_cost_grids_param_file': ('planning/path_cost_grids_param_file',),
        'goal_snapper_param_file': ('planning/goal_snapper_param_file',),
        'centerline_snapper_param_file': ('planning/centerline_snapper_param_file',),
        'goal_replanner_param_file': ('planning/goal_replanner_param_file',),
        'obstacle_replan_monitor_param_file': ('planning/obstacle_replan_monitor_param_file',),
    },
    # HH_260720 - Resolve safety-gate file overrides under the owning module.
    'control': {
        'cmd_vel_gate_yaw_alignment_zones_file': ('control/cmd_vel_gate_yaw_alignment_zones_file',),
    },
    'sensing': {
        'camera_params_file':    ('sensing/camera_params_file',),
        'camera_device_path':    ('sensing/camera_device_path',),
        'imu_converter_param_file': ('sensing/imu_converter_param_file',),
        'radar_sensor_param_file': ('sensing/radar_sensor_param_file',),
        'radar_cost_grid_param_file': ('sensing/radar_cost_grid_param_file',),
        'lidar_preprocessor_param_file': (
            'sensing/lidar_preprocessor_param_file',
        ),
        'lidar_cost_grid_param_file': ('sensing/lidar_cost_grid_param_file',),
        'inflation_cost_grid_param_file': ('sensing/inflation_cost_grid_param_file',),
        'gnss_param_file':        ('sensing/gnss_param_file',),
        'ntrip_param_file':       ('sensing/ntrip_param_file',),
        # HH_260528: imu_param_file resolves model-specific YAML via OVERRIDE_SPECS (file path only).
        'imu_param_file': ('sensing/imu_param_file',),
        'vanjee_config_path': ('sensing/vanjee_config_path',),
        'ground_seg_param_file': ('sensing/ground_seg_param_file',),
    },
    'platform': {
        'params_file': ('platform/params_file',),
        'robot_visualization_param_file': ('platform/robot_visualization_param_file',),
        'ranger_params_file': ('platform/ranger_params_file',),
        # 260708: exterior lights (light_controller + mcu_serial_bridge).
        'lights_param_file': ('platform/lights_param_file',),
    },
    'map': {
        'map_info_file': ('map/map_info_file',),
        'map_param_file': ('map/map_param_file',),
        'lanelet_cost_grid_param_file': ('map/lanelet_cost_grid_param_file',),
        'lanelet_safety_cost_grid_param_file': (
            'map/lanelet_safety_cost_grid_param_file',
        ),
        'map_visualization_param_file': ('map/map_visualization_param_file',),
    },
    'perception': {
        'perception_param_file': ('perception/perception_param_file',),
    },
    'sim': {
        'fake_sensors_param_file': ('sim/fake_sensors_param_file',),
    },
}


# Resolves package-relative path.
def pkg_path(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def optional_pkg_path(pkg: str, rel: str) -> str:
    try:
        return pkg_path(pkg, rel)
    except Exception:
        return ''


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


# Robustly extract lanelet map ros__parameters even when top-level key
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
    # HH_260618: Build an ancestor allow-list so cleanup can kill stale previous
    # bringup processes without terminating the current launch wrapper/shell.
    ancestor_scan = (
        '_ancestors=" $$ "; '
        '_pp="$PPID"; '
        'while [ -n "$_pp" ] && [ "$_pp" != "0" ]; do '
        '_ancestors="$_ancestors$_pp "; '
        '_pp="$(ps -o ppid= -p "$_pp" 2>/dev/null | tr -d "[:space:]")"; '
        'done'
    )
    parts = ['_cleanup_pids=""']
    for p in patterns:
        safe = pkill_safe_pattern(str(p))
        parts.append(
            f'for _pid in $(pgrep -f "{safe}" || true); do '
            f'case "$_ancestors" in *" $_pid "*) continue;; esac; '
            f'_cleanup_pids="$_cleanup_pids $_pid"; '
            f'kill "$_pid" 2>/dev/null || true; '
            f'done'
        )
    # HH_260618: Python launch children can survive SIGTERM long enough to
    # overload the next test run. Escalate only the previously matched stale
    # CAMROD/Nav2 process set after a short grace period.
    force_kill = (
        'sleep 0.5; '
        'for _pid in $_cleanup_pids; do '
        'case "$_ancestors" in *" $_pid "*) continue;; esac; '
        'kill -0 "$_pid" 2>/dev/null && kill -9 "$_pid" 2>/dev/null || true; '
        'done'
    )
    return '; '.join([ancestor_scan, *parts, force_kill])


# Runs cleanup command synchronously in launch context.
def run_cleanup_sync(cmd: str) -> None:
    if not isinstance(cmd, str) or not cmd.strip():
        return
    try:
        print('bringup cleanup: terminating stale matching processes')
        subprocess.run(['bash', '-lc', cmd], check=False)
    except Exception:
        pass


# On Ctrl+C, execute one more cleanup pass to avoid stale duplicate nodes.
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


def _normalize_profile_name(value: Any) -> str:
    text = str(value or '').strip()
    if not text:
        return ''
    text = text.replace('(', '_').replace(')', '_')
    text = re.sub(r'[^A-Za-z0-9_]+', '_', text)
    text = re.sub(r'_+', '_', text).strip('_').lower()
    return text


def infer_map_profile(map_params: dict, map_path: str) -> str:
    # HH_260622 - Map profile selects map-coupled semantic YAML without hardcoding C-track/Park files.
    for key in ('map_profile', 'profile', 'semantic_profile'):
        profile = _normalize_profile_name(map_params.get(key, ''))
        if profile:
            return profile

    basename = os.path.basename(str(map_path or ''))
    match = re.search(r'\(([^)]+)\)', basename)
    if match:
        return _normalize_profile_name(match.group(1))

    stem = os.path.splitext(basename)[0]
    for prefix in ('lanelet2_maps_', 'lanelet2_map_', 'lanelet_'):
        if stem.startswith(prefix):
            return _normalize_profile_name(stem[len(prefix):])
    return ''


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
    # HH_260623 - Resolve map-profile package configs without using stale bringup-local copies.
    selected = _first_existing_path(_profile_file_variants(default_path, profile))
    return selected if selected else default_path


def _is_default_cfg_value(raw_value: Any, default_rel: str) -> bool:
    text = str(raw_value or '').strip()
    if text in ('', '__module_default__', 'module_default', 'default'):
        return True
    return os.path.normpath(text) == os.path.normpath(default_rel)


def resolve_profile_cfg_file(
    config_root: str,
    raw_value: Any,
    default_rel: str,
    map_profile: str,
) -> str:
    if not _is_default_cfg_value(raw_value, default_rel):
        return resolve_cfg_file(config_root, raw_value, default_rel)

    profile = _normalize_profile_name(map_profile)
    if profile:
        default_abs = resolve_cfg_file(config_root, default_rel, default_rel)
        directory = os.path.dirname(default_abs)
        stem, ext = os.path.splitext(os.path.basename(default_abs))
        candidates = [
            os.path.join(directory, f'{stem} ({profile}){ext}'),
            os.path.join(directory, f'{stem}_{profile}{ext}'),
            os.path.join(directory, f'{stem}-{profile}{ext}'),
            os.path.join(directory, profile, f'{stem}{ext}'),
        ]
        selected = _first_existing_path(candidates)
        if selected:
            return selected

    return resolve_cfg_file(config_root, raw_value, default_rel)


# Adds launch argument key/value only when value is non-empty.
def set_if_not_empty(args: dict, key: str, value: str) -> None:
    if isinstance(value, str) and value.strip():
        args[key] = value


def build_cfg_override_map(
    config_root: str,
    launch_cfg: dict,
    override_specs: dict[str, tuple[str, ...]],
) -> dict[str, str]:
    resolved: dict[str, str] = {}
    for launch_arg, cfg_paths in override_specs.items():
        raw_value: Any = ''
        for cfg_path in cfg_paths:
            candidate = cfg_get(launch_cfg, cfg_path, _MISSING)
            if candidate is not _MISSING:
                raw_value = candidate
                break
        resolved[launch_arg] = resolve_cfg_override(config_root, raw_value)
    return resolved


def apply_cfg_overrides(args: dict, overrides: dict[str, str]) -> None:
    for key, value in overrides.items():
        set_if_not_empty(args, key, value)


def sim_switch(sim_cfg: Any, sim_value: str, real_value: Any) -> PythonExpression:
    # HH_260729 - Match launch IfCondition truthy spellings. This prevents
    # sim:=1/True/yes/on from starting hardware or auxiliary dummy publishers
    # alongside fake_sensors.launch.py.
    return PythonExpression([
        f"'{sim_value}' if str('", sim_cfg,
        "').lower() in ['1', 'true', 'yes', 'on'] else '", real_value, "'"
    ])


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


def _map_filename_candidates(configured: str, map_profile: str) -> list[str]:
    filenames = []
    configured_name = os.path.basename(str(configured or '').strip())
    if configured_name:
        filenames.append(configured_name)
    profile = _normalize_profile_name(map_profile)
    if profile:
        filenames.extend([
            f'lanelet2_maps_({profile}).osm',
            f'lanelet2_maps_{profile}.osm',
            f'lanelet2_maps-{profile}.osm',
        ])
    filenames.append('lanelet2_maps.osm')

    ordered = []
    for filename in filenames:
        if filename and filename not in ordered:
            ordered.append(filename)
    return ordered


def resolve_map_path_default(
    launch_cfg_map_path: Any,
    map_info_map_path: Any,
    map_info_path: str,
    config_root: str,
    map_profile: Any = '',
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

    # HH_260622 - Fallback discovery honors map_profile so bringup is not tied to C-track.
    auto_candidates = []
    filenames = _map_filename_candidates(map_info_value or launch_cfg_value, str(map_profile or ''))
    anchors = [
        os.path.abspath(os.path.join(config_root, '..')),
        os.path.abspath(os.path.join(config_root, '..', '..')),
        os.path.join(os.path.expanduser('~'), 'camrod_ws', 'src'),
        os.getcwd(),
        os.path.join(os.getcwd(), 'src'),
    ]
    for anchor in anchors:
        cur = os.path.abspath(anchor)
        for _ in range(8):
            for filename in filenames:
                auto_candidates.append(os.path.join(cur, filename))
                auto_candidates.append(os.path.join(cur, 'src', filename))
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent
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
    # Prefer source-tree config root when provided so YAML edits apply
    # immediately without rebuild. Fallback to installed share path.
    config_root_default = cli_config_root or env_config_root or src_config_root_guess or pkg_config_root
    if not os.path.isdir(config_root_default):
        config_root_default = pkg_config_root
    config_root_default = os.path.abspath(config_root_default)

    # HH_260721 - Use a named helper so config-root resolution remains traceable in diagnostics.
    def bringup_cfg(rel):
        return os.path.join(config_root_default, rel)

    cli_launch_defaults_file = cli_launch_arg('launch_defaults_file')
    env_launch_defaults_file = os.environ.get('CAMROD_LAUNCH_DEFAULTS_FILE', '')
    launch_defaults_file_default = (
        cli_launch_defaults_file
        or env_launch_defaults_file
        or bringup_cfg('bringup/launch_defaults.yaml')
    )
    launch_cfg = read_yaml(launch_defaults_file_default).get('bringup', {})
    # Use camrod_map/config/map_info.yaml as the single source of truth.
    map_info_cfg_entry = cfg_get(launch_cfg, 'map/map_info_file', '__module_default__')
    if str(map_info_cfg_entry).strip() in ('', '__module_default__', 'module_default', 'default'):
        map_info_path = pkg_path('camrod_map', os.path.join('config', 'map_info.yaml'))
    else:
        map_info_path = resolve_cfg_file(config_root_default, map_info_cfg_entry, 'map/map_info.yaml')
    map_info = read_yaml(map_info_path)
    map_params = extract_map_ros_params(map_info)
    # Read unified map/localization reference from ros__parameters
    # to keep map_info.yaml ROS2-param-parser compatible.
    map_ref_llh = {
        'lat': map_params.get('reference_lat', map_params.get('offset_lat', 0.0)),
        'lon': map_params.get('reference_lon', map_params.get('offset_lon', 0.0)),
        'alt': map_params.get('reference_alt', map_params.get('offset_alt', 0.0)),
    }
    map_path_default = resolve_map_path_default(
        cfg_get(launch_cfg, 'map/map_path', ''),
        map_params.get('map_path', ''),
        map_info_path,
        config_root_default,
        map_params.get('map_profile', map_params.get('profile', '')),
    )
    map_profile = infer_map_profile(map_params, map_path_default)
    state_machine_keypoints_cfg = cfg_get(
        launch_cfg, 'planning/state_machine_keypoints_yaml', 'map/drop_zones.yaml')
    state_machine_camping_sites_cfg = cfg_get(
        launch_cfg, 'planning/state_machine_camping_sites_yaml', 'planning/camping_sites.yaml')
    if _is_default_cfg_value(state_machine_keypoints_cfg, 'map/drop_zones.yaml'):
        # HH_260623 - Use camrod_map exporter output as the default drop-zone source.
        # Bringup-local profile copies easily go stale when the active Lanelet2 map changes.
        planning_state_machine_keypoints_default = resolve_profile_file(
            pkg_path('camrod_map', os.path.join('config', 'drop_zones.yaml')),
            map_profile,
        )
    else:
        planning_state_machine_keypoints_default = resolve_profile_cfg_file(
            config_root_default,
            state_machine_keypoints_cfg,
            'map/drop_zones.yaml',
            map_profile,
        )
    if _is_default_cfg_value(state_machine_camping_sites_cfg, 'planning/camping_sites.yaml'):
        # HH_260623 - Use camrod_planning exporter output as the default camping-site source.
        planning_state_machine_camping_sites_default = resolve_profile_file(
            pkg_path('camrod_planning', os.path.join('config', 'camping_sites.yaml')),
            map_profile,
        )
    else:
        planning_state_machine_camping_sites_default = resolve_profile_cfg_file(
            config_root_default,
            state_machine_camping_sites_cfg,
            'planning/camping_sites.yaml',
            map_profile,
        )
    planning_state_machine_cfg_entry = cfg_get(
        launch_cfg,
        'planning/planning_state_machine_param_file',
        'planning/planning_state_machine.yaml',
    )
    if str(planning_state_machine_cfg_entry).strip() in (
        '',
        '__module_default__',
        'module_default',
        'default',
    ):
        planning_state_machine_param_default = pkg_path(
            'camrod_planning', os.path.join('config', 'planning_state_machine.yaml')
        )
    else:
        planning_state_machine_param_default = resolve_cfg_file(
            config_root_default,
            planning_state_machine_cfg_entry,
            'planning/planning_state_machine.yaml',
        )
    # HH_260720 - Use one canonical parking method key; removed backend/mode aliases.
    parking_method_default = str(
        cfg_get(launch_cfg, 'parking/method', 'reverse')
    ).strip().lower()
    if parking_method_default not in ('reverse', 'apriltag'):
        parking_method_default = 'reverse'
    parking_cfg_entry = cfg_get(launch_cfg, 'parking/param_file', '__module_default__')
    if str(parking_cfg_entry).strip() in ('', '__module_default__', 'module_default', 'default'):
        parking_param_default = optional_pkg_path(
            'camrod_control', os.path.join('config', 'parking.yaml')
        )
    else:
        parking_param_default = resolve_cfg_file(
            config_root_default,
            parking_cfg_entry,
            'control/parking.yaml',
        )
    apriltag_cfg_entry = cfg_get(
        launch_cfg,
        'perception/apriltag_param_file',
        'perception/apriltag_parking_detector.yaml',
    )
    apriltag_param_default = resolve_cfg_file(
        config_root_default,
        apriltag_cfg_entry,
        'perception/apriltag_parking_detector.yaml',
    )
    # HH_260720 - Resolve motion-maneuver parameters independently from parking parameters.
    control_cfg_entry = cfg_get(launch_cfg, 'control/param_file', '__module_default__')
    if str(control_cfg_entry).strip() in ('', '__module_default__', 'module_default', 'default'):
        control_param_default = optional_pkg_path(
            'camrod_control', os.path.join('config', 'control.yaml')
        )
    else:
        control_param_default = resolve_cfg_file(
            config_root_default,
            control_cfg_entry,
            'control/control.yaml',
        )
    map_info_launch_default = (
        map_info_path
        if str(map_info_cfg_entry).strip() in ('', '__module_default__', 'module_default', 'default')
        else resolve_cfg_file(config_root_default, map_info_cfg_entry, 'map/map_info.yaml')
    )

    # High-level arguments only.
    arg_specs = [
        # Top-level YAML control points.
        ('config_root', config_root_default, 'Root directory for bringup YAML configs'),
        ('launch_defaults_file', launch_defaults_file_default, 'Top-level launch defaults YAML file'),
        ('clean_before_launch', cfg_get(launch_cfg, 'runtime/clean_before_launch', True), 'Kill stale processes first'),
        # HH_260805 - Normal launch shutdown owns live children. An optional
        # post-pass remains available, but running both paths races components.
        ('clean_on_shutdown', cfg_get(launch_cfg, 'runtime/clean_on_shutdown', False), 'Run an optional stale-process cleanup after bringup shutdown'),
        ('sim', cfg_get(launch_cfg, 'runtime/sim', True), 'Simulation mode'),
        # HH_260721 - Let charging tests opt into the hardware gate contract in simulation.
        (
            'sim_platform_status_enable',
            cfg_get(launch_cfg, 'runtime/sim_platform_status_enable', False),
            'Require simulated normalized platform status at the command gate',
        ),
        # HH_260810 - UI telemetry/manual-goal is the production surface; RViz
        # remains an explicit engineering override for deep visualization only.
        ('rviz', cfg_get(launch_cfg, 'runtime/rviz', False), 'Enable RViz'),
        # Stagger module includes to reduce startup CPU/memory spikes.
        ('module_launch_gap_s', cfg_get(launch_cfg, 'runtime/module_launch_gap_s', 1.0), 'Gap (seconds) between module launch includes'),
        # HH_260805 - Keep CycloneDDS/iceoryx explicitly opt-in and scope it to
        # the physical LiDAR driver/processing boundary. The complete Humble
        # graph exceeds iceoryx's static endpoint capacity.
        (
            'enable_dds_shared_memory',
            cfg_get(launch_cfg, 'runtime/enable_dds_shared_memory', False),
            'Enable managed CycloneDDS shared memory for the physical LiDAR path',
        ),
        (
            'dds_shared_memory_cyclonedds_config',
            bringup_cfg(cfg_get(
                launch_cfg,
                'runtime/dds_shared_memory_cyclonedds_config',
                'middleware/cyclonedds_shm.xml',
            )),
            'CycloneDDS shared-memory XML profile',
        ),
        (
            'dds_shared_memory_roudi_config',
            bringup_cfg(cfg_get(
                launch_cfg,
                'runtime/dds_shared_memory_roudi_config',
                'middleware/iceoryx_roudi.toml',
            )),
            'iceoryx RouDi memory-pool configuration',
        ),
        (
            'dds_shared_memory_roudi_executable',
            cfg_get(
                launch_cfg,
                'runtime/dds_shared_memory_roudi_executable',
                '/opt/ros/humble/bin/iox-roudi',
            ),
            'iceoryx RouDi executable',
        ),
        (
            'dds_shared_memory_roudi_log_level',
            cfg_get(
                launch_cfg,
                'runtime/dds_shared_memory_roudi_log_level',
                'warning',
            ),
            'iceoryx RouDi log level',
        ),

        # HH_260803 - Canonical axle-midpoint frame plus retained rear-axle alias.
        (
            'robot_center_frame_id',
            cfg_get(launch_cfg, 'platform/robot_center_frame_id', 'robot_center_link'),
            'Axle-midpoint frame used by localization, planning, and control',
        ),
        (
            'rear_axle_frame_id',
            cfg_get(launch_cfg, 'platform/rear_axle_frame_id', 'robot_base_link'),
            'Legacy rear-axle compatibility frame',
        ),

        # HH_260721 - robot_localization EKF is the only supported localization backend.
        ('wheel_bridge_enable', cfg_get(launch_cfg, 'localization/wheel_bridge_enable', True), 'Enable wheel bridge'),
        # Bringup-level wheel source wiring for unified wheel bridge.
        ('wheel_input_topic', cfg_get(launch_cfg, 'localization/wheel_input_topic', '/platform/status/odometry'), 'Wheel bridge primary input topic'),
        ('wheel_input_type', cfg_get(launch_cfg, 'localization/wheel_input_type', 'nav_odom'), 'Wheel bridge primary input type: twist|avg_odom|nav_odom'),
        ('wheel_fallback_input_topic', cfg_get(launch_cfg, 'localization/wheel_fallback_input_topic', '/rmp401/odom'), 'Wheel bridge fallback input topic'),
        ('wheel_fallback_input_type', cfg_get(launch_cfg, 'localization/wheel_fallback_input_type', 'nav_odom'), 'Wheel bridge fallback input type: twist|avg_odom|nav_odom'),
        (
            'wheel_primary_timeout_s',
            cfg_get(launch_cfg, 'localization/wheel_primary_timeout_s', 0.7),
            'Primary wheel timeout before fallback (seconds)',
        ),
        # Keep unified wheel output in /platform/status namespace.
        # HH_260720 - Generated wheel input plus an explicit robot_localization boundary.
        ('wheel_output_topic', cfg_get(launch_cfg, 'localization/wheel_output_topic', '/localization/input/wheel_odometry'), 'Generated AvgOdometry wheel input'),
        ('wheel_nav_output_topic', cfg_get(launch_cfg, 'localization/wheel_nav_output_topic', '/localization/input/wheel_odometry_ros'), 'Standard wheel odometry boundary'),
        ('localization_enable_adapter', cfg_get(launch_cfg, 'localization/enable_adapter', True), 'Enable localization adapter launch'),
        ('localization_enable_filter', cfg_get(launch_cfg, 'localization/enable_filter', True), 'Enable localization filter launch'),
        ('localization_enable_monitor', cfg_get(launch_cfg, 'localization/enable_monitor', True), 'Enable localization monitor launch'),
        ('localization_enable_map_helper', cfg_get(launch_cfg, 'localization/enable_map_helper', True), 'Enable localization map_helper launch'),

        # HH_260604: Allow GNSS/localization-only bringup tests without requiring Nav2 runtime packages.
        ('enable_planning', cfg_get(launch_cfg, 'planning/enable_planning', True), 'Enable planning launch module'),
        (
            'use_nav2_container',
            # HH_260805 - Scoped context ownership makes the planner/controller
            # hybrid component topology the validated production default.
            cfg_get(launch_cfg, 'planning/use_nav2_container', True),
            'Compose Nav2 planner/controller in the scoped runtime container',
        ),
        ('enable_path_cost_grids', cfg_get(launch_cfg, 'planning/enable_path_cost_grids', False), 'Enable path cost-grid helpers'),
        # HH_260618: Default off unless explicitly enabled; Nav2 planner_server
        # already owns /planning/global_path in the normal bringup path.
        ('enable_goal_replanner', cfg_get(launch_cfg, 'planning/enable_goal_replanner', False), 'Enable goal replanner'),
        # HH_260619 - Keep this separate from goal_replanner: it only preempts
        # active navigation after persistent LiDAR/Radar blockage, then lets Nav2
        # compute a Smac2D fallback route.
        ('enable_obstacle_replan_monitor', cfg_get(launch_cfg, 'planning/enable_obstacle_replan_monitor', True), 'Enable dynamic-obstacle replan monitor'),
        ('enable_nav2_lifecycle_retry', cfg_get(launch_cfg, 'planning/enable_nav2_lifecycle_retry', False), 'Enable Nav2 lifecycle retry'),
        # Hold Nav2 STARTUP until localization reports ready.
        ('require_localization_ready', cfg_get(launch_cfg, 'planning/require_localization_ready', True), 'Gate Nav2 STARTUP on localization readiness'),
        ('enable_state_machine', cfg_get(launch_cfg, 'planning/enable_state_machine', False), 'Enable planning state machine'),
        ('enable_progress', cfg_get(launch_cfg, 'planning/enable_progress', True), 'Enable remaining distance/time progress publisher'),
        ('enable_path_visualization', cfg_get(launch_cfg, 'planning/enable_path_visualization', True), 'Enable RViz global/local path marker publisher'),
        ('controller_profile', cfg_get(launch_cfg, 'planning/controller_profile', 'rpp'), 'Nav2 controller profile: rpp|dwb'),
        (
            'planning_nav2_selected_planner',
            cfg_get(launch_cfg, 'planning/nav2_selected_planner', '__auto__'),
            'Nav2 planner selector ID override (__auto__|LaneletRoute|NavFn|Smac2D|ThetaStar|SmacHybrid|SmacLattice)',
        ),
        (
            'planning_nav2_selected_controller',
            cfg_get(launch_cfg, 'planning/nav2_selected_controller', '__auto__'),
            'Nav2 controller selector ID override (__auto__|RPP|DWB|MPPI|Graceful|RotationShim)',
        ),
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
        (
            'planning_state_machine_param_file',
            planning_state_machine_param_default,
            'Planning state-machine parameter YAML',
        ),
        (
            'local_path_pose_topic',
            cfg_get(launch_cfg, 'planning/local_path_pose_topic', '/localization/pose'),
            'Pose topic for local_path_extractor',
        ),
        # HH_260720 - Require explicit mission/operator engage before control output.
        (
            'control_cmd_vel_gate_enable',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_enable', True),
            'Enable the control cmd_vel safety gate',
        ),
        (
            'control_cmd_vel_raw_topic',
            cfg_get(launch_cfg, 'control/cmd_vel_raw_topic', '/control/cmd_vel_raw'),
            'Raw motion-command candidate topic consumed by control',
        ),
        (
            'control_navigation_cmd_vel_ros_topic',
            cfg_get(
                launch_cfg,
                'control/navigation_cmd_vel_ros_topic',
                '/control/nav2_cmd_vel_ros',
            ),
            'Standard Nav2 command boundary consumed by control',
        ),
        (
            'control_cmd_vel_topic',
            cfg_get(launch_cfg, 'control/cmd_vel_topic', '/control/cmd_vel'),
            'Final cmd_vel topic published by the control safety gate',
        ),
        (
            'control_cmd_vel_ros_topic',
            cfg_get(launch_cfg, 'control/cmd_vel_ros_topic', '/control/cmd_vel_ros'),
            'Standard Ranger driver command boundary published by control',
        ),
        (
            'planning_engage_topic',
            cfg_get(launch_cfg, 'control/engage_topic', '/planning/engage'),
            'Planning manual engage trigger topic',
        ),
        (
            'planning_mission_engage_topic',
            cfg_get(launch_cfg, 'control/mission_engage_topic', '/planning/mission_engage'),
            'Planning UI mission engage trigger topic',
        ),
        (
            'control_command_enabled_topic',
            cfg_get(launch_cfg, 'control/command_enabled_topic', '/control/command_enabled'),
            'Effective command-enabled state from the control safety gate',
        ),
        (
            'platform_drive_enable_topic',
            # HH_260720 - The operator arm is consumed by the final control gate.
            cfg_get(launch_cfg, 'control/platform_drive_enable_topic', '/platform/drive_enable'),
            'Operator platform-arm topic consumed by the control safety gate',
        ),
        # HH_260720 - Hardware e-stop is carried by AvgPlatformStatus; configure only soft sources.
        (
            'control_cmd_vel_gate_additional_estop_topics',
            cfg_get(
                launch_cfg,
                'control/cmd_vel_gate_additional_estop_topics',
                '/planning/state_machine/estop',
            ),
            'Additional control gate estop topics, comma separated',
        ),
        (
            'control_cmd_vel_gate_dr_timeout_source_mode',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_dr_timeout_source_mode', 'localization_monitor'),
            'Planning DR-timeout source mode: localization_monitor|topic|enabled|on|disabled|off|none',
        ),
        (
            'control_cmd_vel_gate_allow_on_start',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_allow_on_start', False),
            'Allow planning cmd_vel on startup without engage',
        ),
        # HH_260707 - Hold only after sustained DR_ONLY recovery and debounce repeated GNSS flaps.
        (
            'control_cmd_vel_gate_enable_gnss_recovery_hold',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_enable_gnss_recovery_hold', True),
            'Enable GNSS recovery hold in the control cmd_vel gate',
        ),
        (
            'control_cmd_vel_gate_localization_mode_topic',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_localization_mode_topic', '/localization/mode'),
            'Localization mode topic for GNSS recovery hold',
        ),
        (
            'control_cmd_vel_gate_gnss_recovery_hold_s',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_gnss_recovery_hold_s', 2.0),
            'Hold duration after localization recovery (s)',
        ),
        (
            'control_cmd_vel_gate_gnss_recovery_min_source_s',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_gnss_recovery_min_source_s', 1.5),
            'Minimum DR/degraded duration before planning applies GNSS recovery hold',
        ),
        (
            'control_cmd_vel_gate_gnss_recovery_hold_cooldown_s',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_gnss_recovery_hold_cooldown_s', 10.0),
            'Cooldown before planning can apply another GNSS recovery hold',
        ),
        (
            'control_cmd_vel_gate_gnss_recovery_source_mode_min',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_gnss_recovery_source_mode_min', 2),
            'Minimum source localization mode value to trigger recovery hold',
        ),
        (
            'control_cmd_vel_gate_gnss_recovery_target_mode',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_gnss_recovery_target_mode', 0),
            'Target localization mode value for recovery hold trigger',
        ),
        # HH_260720 - Cost-based stop parameters belong to the control gate.
        (
            'control_cmd_vel_gate_cost_stop_enable',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_cost_stop_enable', True),
            'Enable cost-based stop in cmd_vel gate',
        ),
        (
            'control_cmd_vel_gate_cost_grid_topic',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_cost_grid_topic', '/planning/cost_grid/inflation'),
            'Cost grid topic for cost-stop',
        ),
        (
            'control_cmd_vel_gate_cost_pose_topic',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_cost_pose_topic', '/localization/pose'),
            'Pose topic for cost-stop',
        ),
        (
            'control_cmd_vel_gate_cost_odometry_topic',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_cost_odometry_topic', '/localization/odometry'),
            'Odometry topic for cost-stop pose source',
        ),
        (
            'control_cmd_vel_gate_robot_base_frame',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_robot_base_frame', 'robot_center_link'),
            'Axle-midpoint frame used by command safety geometry',
        ),
        # HH_260618: Default to real localization pose for safety/cmd_vel gates.
        (
            'control_cmd_vel_gate_pose_source_preference',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_pose_source_preference', 'pose_topic'),
            'Cost-stop pose source preference: odometry|tf_robot_base|pose_topic',
        ),
        (
            'control_cmd_vel_gate_enable_pose_raw_fallback',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_enable_pose_raw_fallback', False),
            'Allow raw pose fallback without TF frame transform',
        ),
        (
            'control_cmd_vel_gate_cost_threshold',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_cost_threshold', 85),
            'Cost threshold for stop',
        ),
        (
            'control_cmd_vel_gate_cost_lookahead_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_cost_lookahead_m', 2.0),
            'Lookahead distance for cost-stop',
        ),
        (
            'control_cmd_vel_gate_cost_width_m',
            # HH_260806 - Keep the 1.27 m dynamic-obstacle corridor independent
            # from the physical lanelet body and planning envelopes.
            cfg_get(launch_cfg, 'control/cmd_vel_gate_cost_width_m', 1.27),
            'Corridor width for front dynamic cost-stop',
        ),
        (
            'control_cmd_vel_gate_cost_hold_s',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_cost_hold_s', 1.0),
            'Hold duration for cost-stop',
        ),
        # HH_260703 - Dynamic obstacle stops should not release on single-frame
        # sensor flicker; require a continuous clear window before re-enabling.
        # HH_260728 - The gate retains the original trigger source/path/corridor,
        # so a stop-induced zero command cannot advance the clear window.
        (
            'control_cmd_vel_gate_cost_stop_latch_enable',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_cost_stop_latch_enable', True),
            'Latch dynamic cost-stop until the obstacle corridor is continuously clear',
        ),
        (
            'control_cmd_vel_gate_cost_stop_clear_required_s',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_cost_stop_clear_required_s', 2.0),
            'Continuous clear duration required to release dynamic cost-stop latch',
        ),
        (
            'control_cmd_vel_gate_cost_stop_latch_log_interval_s',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_cost_stop_latch_log_interval_s', 1.0),
            'Log interval while waiting for dynamic cost-stop latch release',
        ),
        # HH_260703 - Fail closed if the merged inflation cost grid is missing
        # or stale; this protects cmd_vel even when diagnostics are only WARN.
        (
            'control_cmd_vel_gate_cost_grid_stale_stop_enable',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_cost_grid_stale_stop_enable', True),
            'Block cmd_vel when the merged cost grid is missing or stale',
        ),
        (
            'control_cmd_vel_gate_cost_grid_stale_timeout_s',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_cost_grid_stale_timeout_s', 1.0),
            'Maximum merged cost-grid age before cmd_vel fail-safe stop',
        ),
        (
            'control_cmd_vel_gate_cost_grid_stale_log_interval_s',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_cost_grid_stale_log_interval_s', 1.0),
            'Log interval for merged cost-grid stale fail-safe blocks',
        ),
        # HH_260622: The merged inflation grid includes static route/lanelet
        # guidance. Only configured dynamic sources may trigger cost-stop.
        (
            'control_cmd_vel_gate_cost_stop_require_dynamic_source',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_cost_stop_require_dynamic_source', True),
            'Require dynamic source attribution before merged-grid cost-stop blocks cmd_vel',
        ),
        (
            'control_cmd_vel_gate_cost_stop_dynamic_source_labels',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_cost_stop_dynamic_source_labels', 'radar'),
            'Comma-separated source labels that can trigger dynamic cost-stop',
        ),
        (
            'control_cmd_vel_gate_front_dynamic_stop_use_local_path',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_front_dynamic_stop_use_local_path', True),
            'Use local path corridor for front dynamic obstacle release',
        ),
        (
            'control_cmd_vel_gate_front_dynamic_path_width_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_front_dynamic_path_width_m', 1.27),
            'Front dynamic local-path corridor width (m)',
        ),
        (
            'control_cmd_vel_gate_front_dynamic_path_max_start_distance_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_front_dynamic_path_max_start_distance_m', 1.5),
            'Max robot-to-local-path start distance for front dynamic release (m)',
        ),
        # HH_260618: Raw lanelet hard-stop parameters. This stays separate
        # from /planning/cost_grid/inflation because inflation clears the ego
        # footprint for planner startup and cannot be the final lanelet guard.
        (
            'control_cmd_vel_gate_lanelet_safety_enable',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_enable', True),
            'Enable raw lanelet-grid safety stop',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_grid_topic',
            cfg_get(
                launch_cfg,
                'control/cmd_vel_gate_lanelet_safety_grid_topic',
                '/map/cost_grid/lanelet_safety',
            ),
            'Raw lanelet cost grid topic for safety stop',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_threshold',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_threshold', 85),
            'Raw lanelet corridor threshold for safety stop',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_current_threshold',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_current_threshold', 85),
            'Raw lanelet current-cell threshold for safety stop',
        ),
        # HH_260807 - Physical contact stops ordinary motion. Only a monotonic,
        # projected-clear bounded escape may move back inside the lanelet.
        (
            'control_cmd_vel_gate_lanelet_safety_body_hard_stop_enable',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_body_hard_stop_enable', True),
            'Hard-stop ordinary motion on body cost 100; allow only verified inward escape',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_body_hard_stop_threshold',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_body_hard_stop_threshold', 100),
            'Physical body hard-stop lanelet threshold',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_body_front_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_body_front_m', 0.70837),
            'Physical body front extent (m)',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_body_rear_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_body_rear_m', 0.68323),
            'Physical body rear extent (m)',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_body_left_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_body_left_m', 0.53505),
            'Physical body left extent (m)',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_body_right_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_body_right_m', 0.53495),
            'Physical body right extent (m)',
        ),
        # HH_260809 - Keep bringup overrides synchronized with the canonical
        # tapered-front, rounded-corner collision polygon.
        (
            'control_cmd_vel_gate_lanelet_safety_tapered_rounded_boundary_enable',
            cfg_get(
                launch_cfg,
                'control/cmd_vel_gate_lanelet_safety_tapered_rounded_boundary_enable',
                True,
            ),
            'Enable tapered-front rounded robot boundaries',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_boundary_front_taper_m',
            cfg_get(
                launch_cfg,
                'control/cmd_vel_gate_lanelet_safety_boundary_front_taper_m',
                0.12,
            ),
            'Per-side front-face taper (m)',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_boundary_front_shoulder_depth_m',
            cfg_get(
                launch_cfg,
                'control/cmd_vel_gate_lanelet_safety_boundary_front_shoulder_depth_m',
                0.12,
            ),
            'Longitudinal tapered-shoulder depth (m)',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_boundary_corner_radius_m',
            cfg_get(
                launch_cfg,
                'control/cmd_vel_gate_lanelet_safety_boundary_corner_radius_m',
                0.05,
            ),
            'Physical collision-boundary corner radius (m)',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_boundary_corner_samples',
            cfg_get(
                launch_cfg,
                'control/cmd_vel_gate_lanelet_safety_boundary_corner_samples',
                4,
            ),
            'Line segments per rounded collision-boundary corner',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_footprint_enable',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_footprint_enable', True),
            'Check the complete robot planning footprint against raw lanelet cost',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_footprint_threshold',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_footprint_threshold', 100),
            'Whole-footprint hard boundary threshold (100=off-lane only)',
        ),
        (
            'control_cmd_vel_gate_robot_planning_boundary_topic',
            cfg_get(
                launch_cfg,
                'control/cmd_vel_gate_robot_planning_boundary_topic',
                '/platform/robot/planning_boundary',
            ),
            'Configured robot planning-boundary polygon topic',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_footprint_front_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_footprint_front_m', 0.80837),
            'Fallback planning footprint front extent (m)',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_footprint_rear_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_footprint_rear_m', 0.78323),
            'Fallback planning footprint rear extent (m)',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_footprint_left_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_footprint_left_m', 0.63505),
            'Fallback planning footprint left extent (m)',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_footprint_right_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_footprint_right_m', 0.63495),
            'Fallback planning footprint right extent (m)',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_lookahead_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_lookahead_m', 1.0),
            'Raw lanelet safety lookahead distance (m)',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_width_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_width_m', 0.8),
            'Raw lanelet safety corridor width (m)',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_stop_on_unknown',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_stop_on_unknown', True),
            'Treat unknown/out-of-grid lanelet safety cells as blocked',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_allow_rotation_in_place',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_allow_rotation_in_place', True),
            'Allow pure in-place rotation during lanelet safety stop',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_check_reverse',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_check_reverse', False),
            'Apply raw lanelet safety stop to reverse motion',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_check_lateral',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_check_lateral', False),
            'Apply raw lanelet safety stop to lateral crab motion',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_min_translation_mps',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_min_translation_mps', 0.02),
            'Minimum cmd_vel translation treated as lanelet-safety motion',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_front_use_local_path',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_front_use_local_path', True),
            'Use active local-path corridor for forward lanelet-safety sampling',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_front_path_max_start_distance_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_front_path_max_start_distance_m', 1.5),
            'Maximum pose-to-local-path distance for path-based lanelet safety',
        ),
        # HH_260622: Use a narrow center corridor for local-path lanelet safety;
        # full robot-width raw boundary checks falsely stop at merges.
        (
            'control_cmd_vel_gate_lanelet_safety_front_path_width_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_front_path_width_m', 0.25),
            'Center corridor width for path-based lanelet safety',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_front_path_allow_route_reentry',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_front_path_allow_route_reentry', True),
            'Allow FRONT_PATH static-cost bypass during bounded route re-entry',
        ),
        # HH_260622: Allow bounded route re-entry for manually placed/sim poses
        # that start slightly outside lanelet while a valid local path exists.
        (
            'control_cmd_vel_gate_lanelet_safety_current_allow_route_reentry',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_current_allow_route_reentry', True),
            'Allow current-cell lanelet bypass when close to active local path',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_current_route_reentry_max_distance_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_current_route_reentry_max_distance_m', 4.0),
            'Maximum distance to active local path for current-cell route re-entry',
        ),
        (
            'control_cmd_vel_gate_lanelet_safety_current_route_reentry_require_front_cmd',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lanelet_safety_current_route_reentry_require_front_cmd', True),
            'Require forward cmd_vel for current-cell route re-entry bypass',
        ),
        (
            'control_cmd_vel_gate_route_safety_recovery_max_auto_releases',
            cfg_get(
                launch_cfg,
                'control/cmd_vel_gate_route_safety_recovery_max_auto_releases',
                50,
            ),
            'Bounded Nav2 route-resume attempts within one contact region',
        ),
        (
            'control_cmd_vel_gate_route_safety_recovery_progress_reset_distance_m',
            cfg_get(
                launch_cfg,
                'control/cmd_vel_gate_route_safety_recovery_progress_reset_distance_m',
                0.75,
            ),
            'Signed forward progress required to reset the regional retry budget',
        ),
        # HH_260720 - Drop-zone exit is explicit control motion; bypass only
        # static lanelet cost during its bounded phases.
        (
            'control_cmd_vel_gate_drop_zone_maneuver_controller_status_topic',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_drop_zone_maneuver_controller_status_topic', '/control/drop_zone_maneuver_controller/status'),
            'Drop-zone maneuver status topic for bounded static lanelet bypass',
        ),
        (
            'control_cmd_vel_gate_drop_zone_maneuver_controller_static_bypass_phases',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_drop_zone_maneuver_controller_static_bypass_phases', 'EXIT_STRAIGHT,ALIGN_EXIT_YAW'),
            'Drop-zone maneuver phases allowed to cross static lanelet cost',
        ),
        # HH_260720 - Site maneuver owns campsite entry/return body motion;
        # forward the phase list so only static lanelet cost is bypassed there.
        (
            'control_cmd_vel_gate_camping_site_maneuver_controller_status_topic',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_camping_site_maneuver_controller_status_topic', '/control/camping_site_maneuver_controller/status'),
            'Site maneuver status topic for bounded static lanelet bypass',
        ),
        (
            'control_cmd_vel_gate_camping_site_maneuver_controller_static_bypass_phases',
            cfg_get(
                launch_cfg,
                'control/cmd_vel_gate_camping_site_maneuver_controller_static_bypass_phases',
                # HH_260721 - Match the campsite controller's explicit retrace-yaw phase.
                'ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,ALIGN_RETRACE_YAW,REVERSE_OUT,CRAB_OUT',
            ),
            'Site maneuver phases allowed to cross static lanelet cost',
        ),
        (
            'control_cmd_vel_gate_camping_site_maneuver_controller_lanelet_bypass_phases',
            cfg_get(
                launch_cfg,
                'control/cmd_vel_gate_camping_site_maneuver_controller_lanelet_bypass_phases',
                # HH_260806 - Only explicit site phases may leave the road lanelet.
                'ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,ALIGN_RETRACE_YAW,REVERSE_OUT,CRAB_OUT',
            ),
            'Site maneuver phases allowed outside road lanelet geometry',
        ),
        # Speed-dependent front lookahead.
        (
            'control_cmd_vel_gate_speed_dependent_lookahead',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_speed_dependent_lookahead', True),
            'Enable speed-dependent front lookahead',
        ),
        (
            'control_cmd_vel_gate_front_lookahead_min_m',
            # HH_260702 - Stop earlier so Nav2 has room to publish an avoidance path.
            cfg_get(launch_cfg, 'control/cmd_vel_gate_front_lookahead_min_m', 2.60),
            'Min front lookahead (m)',
        ),
        (
            'control_cmd_vel_gate_front_lookahead_max_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_front_lookahead_max_m', 3.5),
            'Max front lookahead (m)',
        ),
        (
            'control_cmd_vel_gate_front_lookahead_friction',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_front_lookahead_friction', 0.4),
            'Wet road friction for braking distance',
        ),
        (
            'control_cmd_vel_gate_front_reaction_time_s',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_front_reaction_time_s', 0.20),
            'Latency budget for front lookahead (s)',
        ),
        (
            'control_cmd_vel_gate_front_lookahead_margin_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_front_lookahead_margin_m', 0.45),
            'Static safety margin for front lookahead (m)',
        ),
        # HH_260622: Side/rear cost-stop samples the merged grid, but blocks
        # only when dynamic source attribution owns the high-cost cell.
        (
            'control_cmd_vel_gate_side_rear_cost_stop',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_side_rear_cost_stop', True),
            'Enable side/rear cost-stop',
        ),
        (
            'control_cmd_vel_gate_body_near_dynamic_stop',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_body_near_dynamic_stop', True),
            'Enable near-body dynamic side/rear stop during translation',
        ),
        (
            'control_cmd_vel_gate_body_near_side_lookahead_m',
            # HH_260728 - Match the inflation-aware source/bringup forward probe.
            cfg_get(launch_cfg, 'control/cmd_vel_gate_body_near_side_lookahead_m', 0.60),
            'Near-body side dynamic stop distance (m)',
        ),
        (
            'control_cmd_vel_gate_body_near_rear_lookahead_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_body_near_rear_lookahead_m', 0.80),
            'Near-body rear dynamic stop distance (m)',
        ),
        (
            'control_cmd_vel_gate_body_near_maneuver_side_lookahead_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_body_near_maneuver_side_lookahead_m', 1.20),
            'Adaptive maneuver side dynamic stop distance (m)',
        ),
        (
            'control_cmd_vel_gate_body_near_maneuver_rear_lookahead_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_body_near_maneuver_rear_lookahead_m', 0.80),
            'Adaptive maneuver rear dynamic stop distance (m)',
        ),
        (
            'control_cmd_vel_gate_side_cost_threshold',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_side_cost_threshold', 85),
            'Cost threshold for side stop',
        ),
        (
            'control_cmd_vel_gate_side_lookahead_m',
            # HH_260630: Sim and sensor cost grids place side hits near 1.0 m
            # from robot center; keep crab stop ahead of that cell center.
            cfg_get(launch_cfg, 'control/cmd_vel_gate_side_lookahead_m', 1.2),
            'Side lookahead distance (m)',
        ),
        (
            'control_cmd_vel_gate_side_corridor_width_m',
            # HH_260623 - Side corridor width covers full body length plus front/rear margins.
            cfg_get(launch_cfg, 'control/cmd_vel_gate_side_corridor_width_m', 1.69160),
            'Side corridor width (m)',
        ),
        (
            'control_cmd_vel_gate_rear_cost_threshold',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_rear_cost_threshold', 85),
            'Cost threshold for rear stop',
        ),
        (
            'control_cmd_vel_gate_rear_lookahead_m',
            # HH_260630: Rear obstacle cell centers can sit past 0.9 m after
            # grid quantization, so 0.8 m can leak reverse motion.
            cfg_get(launch_cfg, 'control/cmd_vel_gate_rear_lookahead_m', 1.2),
            'Rear lookahead distance (m)',
        ),
        (
            'control_cmd_vel_gate_rear_corridor_width_m',
            # HH_260623 - Rear corridor width covers full body width plus left/right margins.
            cfg_get(launch_cfg, 'control/cmd_vel_gate_rear_corridor_width_m', 1.27),
            'Rear corridor width (m)',
        ),
        # HH_260618: Site-crab lateral parking is mission-owned; let it cross
        # static lanelet/global-path front/side/rear cost while preserving live LiDAR/Radar stops.
        (
            'control_cmd_vel_gate_lateral_cmd_bypass_static_cost_stop',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lateral_cmd_bypass_static_cost_stop', True),
            'Bypass static front/side/rear cost for explicit lateral site-crab cmd_vel',
        ),
        (
            'control_cmd_vel_gate_lateral_cmd_bypass_min_mps',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_lateral_cmd_bypass_min_mps', 0.02),
            'Minimum lateral cmd_vel for site-crab static cost bypass',
        ),
        # HH_260618: Reverse campsite parking also leaves the lanelet corridor,
        # so it needs the same static-cost bypass while preserving live obstacle stops.
        (
            'control_cmd_vel_gate_reverse_cmd_bypass_static_cost_stop',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_reverse_cmd_bypass_static_cost_stop', True),
            'Bypass static front/side/rear cost for explicit reverse site-parking cmd_vel',
        ),
        (
            'control_cmd_vel_gate_reverse_cmd_bypass_min_mps',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_reverse_cmd_bypass_min_mps', 0.02),
            'Minimum reverse cmd_vel for site-parking static cost bypass',
        ),
        # HH_260721 - The gate now uses side_cost_threshold for all lateral obstacle sources.
        # HH_260624 - Keep dynamic LiDAR/Radar obstacle stops active for pure
        # in-place parking rotations even when static lanelet cost is bypassed.
        (
            'control_cmd_vel_gate_rotation_cmd_dynamic_obstacle_stop',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_rotation_cmd_dynamic_obstacle_stop', True),
            'Block pure rotation when live dynamic cost is near the robot body',
        ),
        (
            'control_cmd_vel_gate_rotation_cmd_dynamic_obstacle_radius_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_rotation_cmd_dynamic_obstacle_radius_m', 1.5),
            'Dynamic obstacle disk radius for pure rotation',
        ),
        (
            'control_cmd_vel_gate_rotation_cmd_dynamic_obstacle_threshold',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_rotation_cmd_dynamic_obstacle_threshold', 85),
            'LiDAR/Radar cost threshold that blocks pure rotation',
        ),
        (
            'control_cmd_vel_gate_unavoidable_stop_enable',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_unavoidable_stop_enable', True),
            'Enable unavoidable stop cluster check',
        ),
        # Fixed from 253 (OccupancyGrid max is 100; 253 never triggers).
        (
            'control_cmd_vel_gate_unavoidable_lethal_threshold',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_unavoidable_lethal_threshold', 90),
            'Lethal cost threshold for unavoidable stop',
        ),
        (
            'control_cmd_vel_gate_unavoidable_cluster_min_cells',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_unavoidable_cluster_min_cells', 25),
            'Min cluster cells for unavoidable stop',
        ),
        (
            'control_cmd_vel_gate_unavoidable_cluster_min_ratio',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_unavoidable_cluster_min_ratio', 0.25),
            'Min cluster ratio for unavoidable stop',
        ),
        # Optional map-keypoint yaw alignment gate.
        (
            'control_cmd_vel_gate_yaw_alignment_enable',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_yaw_alignment_enable', False),
            'Enable yaw alignment gate in cmd_vel gate',
        ),
        (
            'control_cmd_vel_gate_yaw_alignment_frame_id',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_yaw_alignment_frame_id', 'map'),
            'Frame id for yaw alignment zones',
        ),
        (
            'control_cmd_vel_gate_yaw_alignment_exit_margin_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_yaw_alignment_exit_margin_m', 0.3),
            'Exit hysteresis margin for yaw alignment zones (m)',
        ),
        # HH_260618: Route-heading guard for normal Nav2 driving.
        (
            'control_cmd_vel_gate_route_heading_enable',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_route_heading_enable', True),
            'Enable route-heading alignment guard in the control cmd_vel gate',
        ),
        (
            'control_cmd_vel_gate_route_heading_path_topic',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_route_heading_path_topic', '/planning/local_path'),
            'Path topic used by route-heading alignment guard',
        ),
        (
            'control_cmd_vel_gate_route_heading_frame_id',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_route_heading_frame_id', 'map'),
            'Fallback frame for route-heading alignment path',
        ),
        (
            'control_cmd_vel_gate_route_heading_min_cmd_x_mps',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_route_heading_min_cmd_x_mps', 0.03),
            'Minimum forward cmd_vel before route-heading alignment engages',
        ),
        (
            'control_cmd_vel_gate_route_heading_lateral_cmd_epsilon_mps',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_route_heading_lateral_cmd_epsilon_mps', 0.02),
            'Lateral cmd_vel threshold treated as explicit crab motion',
        ),
        (
            'control_cmd_vel_gate_route_heading_lookahead_m',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_route_heading_lookahead_m', 2.0),
            'Path tangent lookahead distance for route-heading alignment',
        ),
        (
            'control_cmd_vel_gate_route_heading_error_enter_deg',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_route_heading_error_enter_deg', 75.0),
            'Yaw error threshold to start route-heading alignment',
        ),
        (
            'control_cmd_vel_gate_route_heading_error_exit_deg',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_route_heading_error_exit_deg', 35.0),
            'Yaw error threshold to release route-heading alignment',
        ),
        (
            'control_cmd_vel_gate_route_heading_angular_kp',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_route_heading_angular_kp', 0.8),
            'Angular proportional gain for route-heading alignment',
        ),
        (
            'control_cmd_vel_gate_route_heading_max_angular_z',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_route_heading_max_angular_z', 0.35),
            'Max angular speed for route-heading alignment',
        ),
        (
            'control_cmd_vel_gate_route_heading_max_linear_x',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_route_heading_max_linear_x', 0.0),
            'Max forward speed while route-heading alignment is active',
        ),
        (
            'control_cmd_vel_gate_route_heading_min_path_points',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_route_heading_min_path_points', 2),
            'Minimum path points required for route-heading alignment',
        ),
        # HH_260720 - Scale all cmd_vel output in the control gate.
        (
            'control_cmd_vel_gate_speed_scale',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_speed_scale', 1.0),
            'Speed scale applied to all cmd_vel output (0.0-1.0)',
        ),
        (
            'control_cmd_vel_gate_input_timeout_s',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_input_timeout_s', 0.35),
            'Publish zero when raw planning cmd_vel input is stale for this many seconds',
        ),
        (
            'control_cmd_vel_gate_zero_publish_rate_hz',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_zero_publish_rate_hz', 10.0),
            'Zero Twist publish rate while raw planning cmd_vel input is stale',
        ),
        # HH_260806 - Keep explicit maneuver/Nav2 ownership handoff stationary.
        (
            'control_cmd_vel_gate_maneuver_command_release_hold_s',
            cfg_get(launch_cfg, 'control/cmd_vel_gate_maneuver_command_release_hold_s', 0.5),
            'Stationary hold after maneuver command ownership ends',
        ),

        (
            'enable_module_validators',
            cfg_get(launch_cfg, 'system/enable_module_validators', True),
            'Enable module validators',
        ),
        (
            'use_system_tools_container',
            cfg_get(launch_cfg, 'system/use_system_tools_container', True),
            'Compose the system aggregate/status tool chain',
        ),
        (
            'use_checker_components',
            cfg_get(launch_cfg, 'system/use_checker_components', True),
            'Compose selected system checker fault domains',
        ),
        (
            'checker_component_groups',
            cfg_get(
                launch_cfg,
                'system/checker_component_groups',
                'hardware_sensing,localization,autonomy_topics,planning_lifecycle',
            ),
            'Comma-separated system checker component groups',
        ),
        (
            'checker_component_threads',
            cfg_get(launch_cfg, 'system/checker_component_threads', 1),
            'Compatibility value for system checker executor experiments',
        ),
        (
            'diagnostics_profile',
            cfg_get(launch_cfg, 'system/diagnostics_profile', 'default'),
            'Diagnostics config profile name',
        ),
        (
            'system_checker_param_file',
            cfg_get(launch_cfg, 'system/system_checker_param_file', '__module_default__'),
            'System checker graph manifest YAML path (__module_default__ selects hardware/sim automatically)',
        ),
        (
            'enable_platform_checker',
            cfg_get(launch_cfg, 'system/enable_platform_checker', False),
            'Enable diagnostics platform checker (requires ranger_msgs)',
        ),
        (
            'enable_plugin_api',
            cfg_get(launch_cfg, 'system/enable_plugin_api', True),
            'Enable plugin API bridge',
        ),
        (
            'enable_api_ui',
            cfg_get(launch_cfg, 'system/enable_api_ui', True),
            'Enable API UI backend node',
        ),
        (
            'api_ui_host',
            cfg_get(launch_cfg, 'system/api_ui_host', '0.0.0.0'),
            'API UI backend bind host',
        ),
        (
            'api_ui_port',
            cfg_get(launch_cfg, 'system/api_ui_port', 8010),
            'API UI backend bind port',
        ),
        (
            'enable_operator_telemetry',
            cfg_get(launch_cfg, 'system/enable_operator_telemetry', True),
            'Enable leased operator sensor/map telemetry views',
        ),
        (
            'enable_guest_ui',
            cfg_get(launch_cfg, 'system/enable_guest_ui', True),
            'Enable the WiFi guest UI server',
        ),
        (
            'guest_ui_host',
            cfg_get(launch_cfg, 'system/guest_ui_host', '0.0.0.0'),
            'Guest UI bind host',
        ),
        (
            'guest_ui_port',
            cfg_get(launch_cfg, 'system/guest_ui_port', 8012),
            'Guest UI bind port',
        ),
        # HH_260724 - Keep UI mission-admission thresholds visible in bringup config.
        (
            'api_ui_require_battery_for_mission_dispatch',
            cfg_get(launch_cfg, 'system/api_ui_require_battery_for_mission_dispatch', True),
            'Require battery feedback before accepting new campsite dispatch',
        ),
        (
            'api_ui_minimum_mission_dispatch_battery_percent',
            cfg_get(launch_cfg, 'system/api_ui_minimum_mission_dispatch_battery_percent', 35.0),
            'Minimum SOC percent for new campsite dispatch',
        ),
        (
            'api_ui_low_battery_return_after_current_mission',
            cfg_get(launch_cfg, 'system/api_ui_low_battery_return_after_current_mission', True),
            'Latch low battery during a campsite mission and wait for user return',
        ),
        (
            'api_ui_low_battery_return_threshold_percent',
            cfg_get(launch_cfg, 'system/api_ui_low_battery_return_threshold_percent', 35.0),
            'SOC percent that starts the finish-current-mission return latch',
        ),
        # HH_260727 - Pass the lightweight local UI surface through bringup.
        (
            'enable_operator_ui_window',
            cfg_get(launch_cfg, 'system/enable_operator_ui_window', True),
            'Open the local managed operator UI browser window',
        ),
        (
            'operator_ui_window_url',
            cfg_get(
                launch_cfg,
                'system/operator_ui_window_url',
                'http://127.0.0.1:8010',
            ),
            'URL loaded by the local operator UI window',
        ),
        (
            'operator_ui_window_engine',
            # HH_260807 - Keep the fallback aligned with standalone UI launch
            # and the robot profile even if launch_defaults.yaml is unavailable.
            cfg_get(launch_cfg, 'system/operator_ui_window_engine', 'webkit'),
            'Operator UI renderer: chromium, webkit, or auto',
        ),
        (
            'operator_ui_window_width',
            cfg_get(launch_cfg, 'system/operator_ui_window_width', 1280),
            'Initial local operator UI window width in pixels',
        ),
        (
            'operator_ui_window_height',
            cfg_get(launch_cfg, 'system/operator_ui_window_height', 800),
            'Initial local operator UI window height in pixels',
        ),
        (
            'operator_ui_window_fullscreen',
            cfg_get(launch_cfg, 'system/operator_ui_window_fullscreen', True),
            'Open the local operator UI window fullscreen',
        ),

        (
            'enable_sensing_module',
            cfg_get(launch_cfg, 'sensing/enable_module', True),
            'Enable the aggregate sensing launch (disable when sensing is already running)',
        ),
        ('enable_radar', cfg_get(launch_cfg, 'sensing/enable_radar', False), 'Enable serial radar'),
        # HH_260729 - One master policy covers every hardware acquisition
        # enable flag without adding another per-sensor enable_* switch.
        (
            'publish_sensor_dummies_when_disabled',
            cfg_get(
                launch_cfg,
                'sensing/publish_sensor_dummies_when_disabled',
                True,
            ),
            'Publish explicit low-rate DUMMY/WARN data while hardware inputs are disabled',
        ),
        ('radar_log_status', cfg_get(launch_cfg, 'sensing/radar_log_status', False), 'Print per-port radar status lines'),
        ('enable_camera', cfg_get(launch_cfg, 'sensing/enable_camera', True), 'Enable camera publisher stack'),
        # HH_260528: Per-camera enable flags for dual econ camera setup.
        ('enable_front_camera', cfg_get(launch_cfg, 'sensing/enable_front_camera', True), 'Enable front econ camera node'),
        ('enable_rear_camera',  cfg_get(launch_cfg, 'sensing/enable_rear_camera',  True), 'Enable rear econ camera node'),
        ('enable_radar_cost_grid', cfg_get(launch_cfg, 'sensing/enable_radar_cost_grid', True), 'Enable radar cost-grid'),
        ('enable_lidar_cost_grid', cfg_get(launch_cfg, 'sensing/enable_lidar_cost_grid', False), 'Enable optional lidar cost-grid'),
        (
            'use_lidar_processing_container',
            cfg_get(launch_cfg, 'sensing/use_lidar_processing_container', True),
            'Compose LiDAR preprocessing, segmentation, and optional cost-grid nodes',
        ),
        ('enable_inflation_cost_grid', cfg_get(launch_cfg, 'sensing/enable_inflation_cost_grid', True), 'Enable inflation cost-grid (lanelet+radar+global_path merger)'),
        ('enable_lidar_driver', cfg_get(launch_cfg, 'sensing/enable_lidar_driver', False), 'Enable lidar driver'),
        ('enable_imu',      cfg_get(launch_cfg, 'sensing/enable_imu',      True),  'Enable physical IMU driver (converter remains available for dummy input)'),
        # HH_260528: Unified IMU model selector (imu_mode → imu_model).
        ('imu_model',       cfg_get(launch_cfg, 'sensing/imu_model',       'cv7'), 'IMU model: cv7 | gq7'),
        ('imu_param_file',  cfg_get(launch_cfg, 'sensing/imu_param_file',  '__module_default__'), 'IMU param file path (or __module_default__)'),
        ('enable_gnss', cfg_get(launch_cfg, 'sensing/enable_gnss', False), 'Enable GNSS driver stack'),
        ('enable_ntrip', cfg_get(launch_cfg, 'sensing/enable_ntrip', False), 'Enable GNSS NTRIP client'),
        # HH_260727 - Device/baud normally come from the writer-specific section
        # of gnss_param_file. These launch arguments are explicit overrides only.
        ('ublox_dual_antenna', cfg_get(launch_cfg, 'sensing/ublox_dual_antenna', True), 'Use ublox_gps for dual-antenna simpleRTK2B Heading'),
        ('ublox_dual_forward_ntrip_to_rover', cfg_get(launch_cfg, 'sensing/ublox_dual_forward_ntrip_to_rover', False), 'Diagnostic only: forward NTRIP directly to rover USB'),
        ('ublox_dual_warm_start_on_startup', cfg_get(launch_cfg, 'sensing/ublox_dual_warm_start_on_startup', False), 'One-shot heading-rover warm-start recovery'),
        (
            'ublox_dual_base_rtcm_device',
            cfg_get(
                launch_cfg,
                'sensing/ublox_dual_base_rtcm_device',
                '__config__',
            ),
            'Moving-base serial override (__config__ uses gnss_param_file)',
        ),
        ('ublox_dual_base_rtcm_baud', cfg_get(launch_cfg, 'sensing/ublox_dual_base_rtcm_baud', '__config__'), 'Moving-base baud override (__config__ uses gnss_param_file)'),
        ('perception_enable_lidar_obstacle', cfg_get(launch_cfg, 'perception/enable_lidar_obstacle', True), 'Enable perception LiDAR obstacle node'),
        ('perception_enable_yolo', cfg_get(launch_cfg, 'perception/enable_yolo', True), 'Enable perception YOLO node'),
        ('use_camera_yolo_container', cfg_get(launch_cfg, 'perception/use_camera_yolo_container', False), 'Run front camera and YOLO in one component container'),
        (
            'use_rear_camera_apriltag_container',
            cfg_get(
                launch_cfg,
                'perception/use_rear_camera_apriltag_container',
                True,
            ),
            'Compose physical rear camera, rectification, and AprilTag detection',
        ),
        (
            'apriltag_param_file',
            apriltag_param_default,
            'AprilTag parking detector parameter YAML',
        ),
        ('perception_mode', cfg_get(launch_cfg, 'perception/mode', 'auto'), 'Perception mode: auto|lidar_only|camera_lidar'),
        ('camera_device_path', cfg_get(launch_cfg, 'sensing/camera_device_path', '/dev/video0'), 'Camera device path'),

        ('map_namespace', cfg_get(launch_cfg, 'namespaces/map', 'map'), 'Map namespace'),
        ('sensing_namespace', cfg_get(launch_cfg, 'namespaces/sensing', 'sensing'), 'Sensing namespace'),
        ('localization_namespace', cfg_get(launch_cfg, 'namespaces/localization', 'localization'), 'Localization namespace'),
        ('planning_namespace', cfg_get(launch_cfg, 'namespaces/planning', 'planning'), 'Planning namespace'),
        # HH_260720 - Control has an explicit namespace independent from planning and parking.
        ('control_namespace', cfg_get(launch_cfg, 'namespaces/control', 'control'), 'Control namespace'),
        ('platform_namespace', cfg_get(launch_cfg, 'namespaces/platform', 'platform'), 'Platform namespace'),
        ('parking_namespace', cfg_get(launch_cfg, 'namespaces/parking', 'parking'), 'Parking namespace'),
        ('perception_namespace', cfg_get(launch_cfg, 'namespaces/perception', 'perception'), 'Perception namespace'),
        ('sensor_kit_namespace', cfg_get(launch_cfg, 'namespaces/sensor_kit', 'sensor_kit'), 'Sensor-kit namespace'),
        ('bringup_namespace', cfg_get(launch_cfg, 'namespaces/bringup', 'bringup'), 'Bringup namespace'),
        ('system_namespace', cfg_get(launch_cfg, 'namespaces/system', 'system'), 'System namespace'),
        # HH_260722 - sensing.launch.py already pushes /sensing; keep the GNSS
        # fallback relative so a missing config cannot create /sensing/sensing.
        ('gnss_namespace', cfg_get(launch_cfg, 'namespaces/gnss', 'gnss'), 'GNSS namespace'),

        ('gnss_rtcm_topic', cfg_get(launch_cfg, 'topics/gnss_rtcm', '/sensing/gnss/rtcm'), 'GNSS RTCM topic'),
        # HH_260528: Platform type selector.
        ('platform_type', cfg_get(launch_cfg, 'platform/type', 'ranger'), 'Platform type: ranger|rmp401'),
        # Keep platform top launch lean. Ranger detailed params are in ranger_params_file.
        ('platform_ranger_driver_enable', cfg_get(launch_cfg, 'platform/ranger_driver_enable', True), 'Enable Ranger base CAN node in platform launch'),
        # HH_260528: Ranger bridge toggle (independent from Ranger base node).
        ('platform_ranger_bridge_enable', cfg_get(launch_cfg, 'platform/ranger_bridge_enable', True), 'Enable ranger_platform_bridge_node in platform launch'),
        ('platform_ranger_auto_setup_can', cfg_get(launch_cfg, 'platform/ranger_auto_setup_can', True), 'Bring can0 up before Ranger CAN node starts'),
        ('platform_ranger_can_bitrate', cfg_get(launch_cfg, 'platform/ranger_can_bitrate', 500000), 'Ranger SocketCAN bitrate'),
        ('platform_ranger_can_restart_ms', cfg_get(launch_cfg, 'platform/ranger_can_restart_ms', 100), 'Ranger SocketCAN restart-ms'),
        # HH_260528: Keep sensor_kit bridge optional for debug.
        ('platform_sensor_kit_bridge_enable', cfg_get(launch_cfg, 'platform/sensor_kit_bridge_enable', True), 'Enable sensor_kit bridge include in platform launch'),
        # 260708: Exterior lights (headlight relay + WS2815 indicators via light MCU).
        ('platform_lights_enable', cfg_get(launch_cfg, 'platform/lights_enable', True), 'Enable light_controller node in platform launch'),
        ('platform_lights_mcu_bridge_enable', cfg_get(launch_cfg, 'platform/lights_mcu_bridge_enable', True), 'Enable light MCU serial bridge (real hardware only; sim forces false)'),

        # HH_260720 - Select reverse or AprilTag parking inside camrod_control.
        ('parking_method', parking_method_default, 'Parking implementation: reverse|apriltag'),
        ('enable_parking', cfg_get(launch_cfg, 'parking/enable_parking', True), 'Enable final parking control'),
        ('enable_camping_site_maneuver_controller', cfg_get(launch_cfg, 'control/enable_camping_site_maneuver_controller', True), 'Enable campsite crab/rotate control node'),
        ('enable_drop_zone_maneuver_controller', cfg_get(launch_cfg, 'control/enable_drop_zone_maneuver_controller', True), 'Enable drop-zone exit/yaw control node'),
        ('enable_route_safety_recovery_controller', cfg_get(launch_cfg, 'control/enable_route_safety_recovery_controller', True), 'Enable bounded route-safety reverse/crab owner'),
        ('control_param_file', control_param_default, 'Control maneuver parameter YAML path'),
        # HH_260720 - Keep CAN/charging gate settings under the owning control module.
        ('control_cmd_vel_gate_status_topic', cfg_get(launch_cfg, 'control/cmd_vel_gate_status_topic', '/control/cmd_vel_safety_gate/status'), 'Control safety-gate status topic'),
        ('control_cmd_vel_gate_platform_safety_source_mode', cfg_get(launch_cfg, 'control/cmd_vel_gate_platform_safety_source_mode', 'platform_status'), 'Platform safety source mode'),
        ('control_cmd_vel_gate_platform_status_topic', cfg_get(launch_cfg, 'control/cmd_vel_gate_platform_status_topic', '/platform/status'), 'Normalized platform status topic'),
        ('control_cmd_vel_gate_platform_status_timeout_s', cfg_get(launch_cfg, 'control/cmd_vel_gate_platform_status_timeout_s', 0.5), 'Platform status timeout'),
        ('control_cmd_vel_gate_block_on_platform_status_stale', cfg_get(launch_cfg, 'control/cmd_vel_gate_block_on_platform_status_stale', True), 'Block on missing or stale platform status'),
        ('control_cmd_vel_gate_block_on_charging', cfg_get(launch_cfg, 'control/cmd_vel_gate_block_on_charging', True), 'Block normal commands while charging'),
        ('control_cmd_vel_gate_allow_mission_departure_while_charging', cfg_get(launch_cfg, 'control/cmd_vel_gate_allow_mission_departure_while_charging', True), 'Allow a new mission to release from charger'),
        ('control_cmd_vel_gate_charging_departure_grace_s', cfg_get(launch_cfg, 'control/cmd_vel_gate_charging_departure_grace_s', 15.0), 'Bounded charger departure window'),
        # HH_260720 - A concrete campsite mission request authorizes charger departure.
        ('control_cmd_vel_gate_mission_request_topic', cfg_get(launch_cfg, 'control/cmd_vel_gate_mission_request_topic', '/planning/mission_key'), 'Mission request topic used for charger departure'),
        ('control_cmd_vel_gate_charger_departure_mission_prefixes', cfg_get(launch_cfg, 'control/cmd_vel_gate_charger_departure_mission_prefixes', 'camping_site_'), 'Mission prefixes allowed to depart the charger'),
        ('control_cmd_vel_gate_mission_request_dedup_s', cfg_get(launch_cfg, 'control/cmd_vel_gate_mission_request_dedup_s', 1.0), 'Repeated mission request deduplication window'),
        # HH_260724 - Match the package YAML charger-departure SOC gate in full bringup.
        ('control_cmd_vel_gate_require_battery_for_mission_departure', cfg_get(launch_cfg, 'control/cmd_vel_gate_require_battery_for_mission_departure', True), 'Require battery feedback before charger departure'),
        ('control_cmd_vel_gate_minimum_mission_departure_battery_percentage', cfg_get(launch_cfg, 'control/cmd_vel_gate_minimum_mission_departure_battery_percentage', 0.35), 'Minimum SOC ratio for charger departure'),
        ('control_cmd_vel_gate_block_on_platform_error_code', cfg_get(launch_cfg, 'control/cmd_vel_gate_block_on_platform_error_code', True), 'Block on non-zero platform CAN error mask'),
        ('control_cmd_vel_gate_require_can_control_mode', cfg_get(launch_cfg, 'control/cmd_vel_gate_require_can_control_mode', True), 'Require Ranger CAN command mode'),
        ('control_cmd_vel_gate_critical_battery_stop_enabled', cfg_get(launch_cfg, 'control/cmd_vel_gate_critical_battery_stop_enabled', True), 'Block at critical BMS SOC'),
        ('control_cmd_vel_gate_critical_battery_percentage', cfg_get(launch_cfg, 'control/cmd_vel_gate_critical_battery_percentage', 0.20), 'Critical BMS SOC ratio'),
        ('parking_param_file', parking_param_default, 'Parking parameter YAML path'),

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
        # HH_260527: Removed unused map-origin launch args.
        # (yaw_offset_deg, utm_origin_*, rotate_latlon_xy_by_yaw_offset).

        ('lanelet_id', cfg_get(launch_cfg, 'sim/lanelet_id', -1), 'Fake sensor lanelet id'),
        ('sim_obstacle_offset', cfg_get(launch_cfg, 'sim/obstacle_offset', 12.0), 'Fake obstacle offset distance (m)'),
        ('sim_obstacle_height', cfg_get(launch_cfg, 'sim/obstacle_height', 0.5), 'Fake obstacle height (m)'),
        ('sim_obstacle_direction', cfg_get(launch_cfg, 'sim/obstacle_direction', 'front'), 'Fake obstacle direction: front|left|right|rear'),
        ('sim_obstacle_lateral_offset', cfg_get(launch_cfg, 'sim/obstacle_lateral_offset', 0.0), 'Fake obstacle lateral offset in vehicle-left axis (m)'),

        ('enable_voice',         cfg_get(launch_cfg, 'voice/enable_voice',         True),  'Enable voice announcer module'),
        ('enable_voice_adapter', cfg_get(launch_cfg, 'voice/enable_voice_adapter', True),  'Enable voice event adapter node'),
        ('voice_namespace',      cfg_get(launch_cfg, 'namespaces/voice',           'voice'), 'Voice module namespace'),
    ]

    args = [
        DeclareLaunchArgument(name, default_value=as_launch_default(default), description=desc)
        for name, default, desc in arg_specs
    ]

    lc = {name: LaunchConfiguration(name) for name, _, _ in arg_specs}
    # HH_260720 - Both final parking implementations are selected inside camrod_control.
    parking_enabled_condition = IfCondition(lc['enable_parking'])
    # HH_260720 - Navigation and parking maneuvers share the same final command safety gate.
    control_command_gate_condition = IfCondition(PythonExpression([
        "'true' if '", lc['enable_planning'], "' == 'true' or '",
        lc['enable_parking'], "' == 'true' else 'false'",
    ]))

    # Module config defaults-first policy.
    # Bringup passes file paths only when an explicit override is configured.
    localization_overrides = build_cfg_override_map(config_root_default, launch_cfg, OVERRIDE_SPECS['localization'])
    planning_overrides = build_cfg_override_map(config_root_default, launch_cfg, OVERRIDE_SPECS['planning'])
    control_overrides = build_cfg_override_map(config_root_default, launch_cfg, OVERRIDE_SPECS['control'])
    sensing_overrides = build_cfg_override_map(config_root_default, launch_cfg, OVERRIDE_SPECS['sensing'])
    platform_overrides = build_cfg_override_map(config_root_default, launch_cfg, OVERRIDE_SPECS['platform'])
    map_overrides = build_cfg_override_map(config_root_default, launch_cfg, OVERRIDE_SPECS['map'])
    perception_overrides = build_cfg_override_map(config_root_default, launch_cfg, OVERRIDE_SPECS['perception'])
    sim_overrides = build_cfg_override_map(config_root_default, launch_cfg, OVERRIDE_SPECS['sim'])

    # HH_260522: Use unified controller_profile selector.
    controller_profile_cli = cli_launch_arg('controller_profile').strip().lower()
    controller_profile_cfg = str(cfg_get(launch_cfg, 'planning/controller_profile', '')).strip().lower()
    controller_profile = controller_profile_cli or controller_profile_cfg
    if controller_profile not in ('rpp', 'dwb'):
        controller_profile = 'rpp'
    selected_nav2_vehicle_override = (
        planning_overrides['nav2_vehicle_dwb_param_file']
        if controller_profile == 'dwb' and planning_overrides['nav2_vehicle_dwb_param_file']
        else planning_overrides['nav2_vehicle_param_file']
    )

    def include(pkg: str, launch_file: str, launch_args: dict, condition=None):
        kwargs = {
            'launch_description_source': PythonLaunchDescriptionSource(pkg_path(pkg, os.path.join('launch', launch_file))),
            'launch_arguments': launch_args.items(),
        }
        if condition is not None:
            kwargs['condition'] = condition
        # HH_260723 - Keep launch arguments local to each module. In particular, the sensing
        # include receives enable_front_camera=false when the composable
        # camera+YOLO path owns that device. Without a scoped group that child
        # value leaked into the following perception include and incorrectly
        # disabled camera fusion even though the camera and YOLO components were
        # running.
        return GroupAction(
            actions=[IncludeLaunchDescription(**kwargs)],
            scoped=True,
        )

    # HH_260611: Treat optional modules as optional at launch time so stale/missing
    # package artifacts do not block GNSS and sensing validation on this x86_64 PC.
    def has_launch_file(pkg: str, launch_file: str) -> tuple[bool, str]:
        try:
            launch_path = pkg_path(pkg, os.path.join('launch', launch_file))
        except Exception:
            return False, ''
        return os.path.isfile(launch_path), launch_path

    def has_executable(pkg: str, executable: str) -> tuple[bool, str]:
        try:
            executable_path = os.path.join(get_package_prefix(pkg), 'lib', pkg, executable)
        except Exception:
            return False, ''
        return os.path.isfile(executable_path) and os.access(executable_path, os.X_OK), executable_path

    def optional_module_available(pkg: str, launch_file: str) -> tuple[bool, str]:
        launch_exists, launch_path = has_launch_file(pkg, launch_file)
        if not launch_exists:
            return False, f'launch file not found{": " + launch_path if launch_path else ""}'

        missing = []
        for executable_pkg, executable_name in OPTIONAL_MODULE_EXECUTABLES.get(pkg, ()):
            executable_exists, _ = has_executable(executable_pkg, executable_name)
            if not executable_exists:
                missing.append(f'{executable_pkg}/{executable_name}')
        if missing:
            return False, 'missing executable(s): ' + ', '.join(missing)

        return True, ''

    platform_args = {
        # HH_260720 - Use the platform-specific launch argument to prevent nested namespace leaks.
        'platform_namespace': lc['platform_namespace'],
        'sensor_kit_namespace': lc['sensor_kit_namespace'],
        'base_frame_id': lc['robot_center_frame_id'],
        'rear_axle_frame_id': lc['rear_axle_frame_id'],
        'platform_type': sim_switch(
            lc['sim'], 'rmp401', lc['platform_type']
        ),
        # In sim, disable external CAN driver by default.
        'ranger_driver_enable': sim_switch(
            lc['sim'], 'false', lc['platform_ranger_driver_enable']
        ),
        # HH_260729 - The Ranger dummy is non-drivable/estop feedback, not a
        # healthy CAN simulation.  fake_sensors owns platform topics in sim.
        'ranger_dummy_when_disabled': sim_switch(
            lc['sim'], 'false', lc['publish_sensor_dummies_when_disabled']
        ),
        'ranger_bridge_enable': lc['platform_ranger_bridge_enable'],
        'ranger_auto_setup_can': lc['platform_ranger_auto_setup_can'],
        'ranger_can_bitrate': lc['platform_ranger_can_bitrate'],
        'ranger_can_restart_ms': lc['platform_ranger_can_restart_ms'],
        'sensor_kit_bridge_enable': lc['platform_sensor_kit_bridge_enable'],
        # 260708: light_controller runs in sim too (topic-only, no hardware);
        # the MCU serial bridge is hardware-facing so sim disables it.
        'lights_enable': lc['platform_lights_enable'],
        'lights_mcu_bridge_enable': sim_switch(
            lc['sim'], 'false', lc['platform_lights_mcu_bridge_enable']
        ),
    }
    apply_cfg_overrides(platform_args, platform_overrides)

    map_args = {
        'map_info_file': lc['map_info_file'],
        'map_path': lc['map_path'],
        'origin_lat': lc['origin_lat'],
        'origin_lon': lc['origin_lon'],
        'origin_alt': lc['origin_alt'],
        'module_namespace': lc['map_namespace'],
        # HH_260527: Removed unused pass-through args
        # (system_namespace, enable_module_validator).
    }
    apply_cfg_overrides(map_args, map_overrides)

    fake_sensors_args = {
        'bringup_namespace': lc['bringup_namespace'],
        'sensing_namespace': lc['sensing_namespace'],
        'base_frame_id': lc['robot_center_frame_id'],
        'fake_enable_cost_grids': 'false',
        # HH_260721 - Disable raw CAN/BMS simulation when the validator owns /platform/status.
        'publish_simulated_platform_status': PythonExpression([
            "'false' if str('", lc['sim_platform_status_enable'],
            "').lower() in ['1', 'true', 'yes', 'on'] else 'true'"
        ]),
        'map_path': lc['map_path'],
        'origin_lat': lc['origin_lat'],
        'origin_lon': lc['origin_lon'],
        'origin_alt': lc['origin_alt'],
        'lanelet_id': lc['lanelet_id'],
        'obstacle_offset': lc['sim_obstacle_offset'],
        'obstacle_height': lc['sim_obstacle_height'],
        'obstacle_direction': lc['sim_obstacle_direction'],
        'obstacle_lateral_offset': lc['sim_obstacle_lateral_offset'],
    }
    apply_cfg_overrides(fake_sensors_args, sim_overrides)

    # HH_260707: When the opt-in component path is enabled, the front camera is
    # started by camera_yolo_container.launch.py while the rear camera remains in
    # sensing.launch.py for future perception consumers. Sim disables hardware cameras.
    # HH_260720 - Camera launch does not carry an implicit AprilTag parking dependency.
    # HH_260716 - The container must also honor the high-level camera/front/YOLO
    # switches. Previously use_camera_yolo_container=true started both components
    # even when enable_camera=false or perception_enable_yolo=false.
    # HH_260729 - Use the same IfCondition-compatible truthy spellings as
    # sim_switch so sim:=1/yes/on cannot start the hardware camera container.
    camera_yolo_container_active_expression = PythonExpression([
        "'true' if str('", lc['sim'],
        "').lower() not in ['1', 'true', 'yes', 'on'] and str('",
        lc['use_camera_yolo_container'],
        "').lower() in ['1', 'true', 'yes', 'on'] and str('",
        lc['enable_camera'],
        "').lower() in ['1', 'true', 'yes', 'on'] and str('",
        lc['enable_front_camera'],
        "').lower() in ['1', 'true', 'yes', 'on'] and str('",
        lc['perception_enable_yolo'],
        "').lower() in ['1', 'true', 'yes', 'on'] else 'false'",
    ])
    # HH_260729 - Resolve component ownership once in the parent launch scope.
    # sensing.launch.py receives its own enable_front_camera=false override when
    # the component owns that device. Passing the deferred expression directly
    # let the child scope feed that false value back into the expression, which
    # incorrectly started a 1x1 front-camera dummy beside the live component
    # and could abort the camera+YOLO container on its first mixed-size frame.
    resolve_camera_yolo_container_active = SetLaunchConfiguration(
        'camera_yolo_container_active_resolved',
        camera_yolo_container_active_expression,
    )
    camera_yolo_container_active = LaunchConfiguration(
        'camera_yolo_container_active_resolved'
    )
    regular_front_camera_enable = PythonExpression([
        "'false' if str('", lc['sim'],
        "').lower() in ['1', 'true', 'yes', 'on'] or str('",
        camera_yolo_container_active,
        "').lower() in ['1', 'true', 'yes', 'on'] else '",
        lc['enable_front_camera'], "'",
    ])
    regular_yolo_enable = PythonExpression([
        "'false' if str('", lc['sim'],
        "').lower() in ['1', 'true', 'yes', 'on'] or str('",
        camera_yolo_container_active,
        "').lower() in ['1', 'true', 'yes', 'on'] else '",
        lc['perception_enable_yolo'], "'",
    ])
    camera_yolo_container_condition = IfCondition(camera_yolo_container_active)

    # HH_260805 - Resolve rear-camera ownership independently from the front
    # camera. The container is hardware-only and exists only for AprilTag parking.
    rear_camera_apriltag_container_active_expression = PythonExpression([
        "'true' if str('", lc['sim'],
        "').lower() not in ['1', 'true', 'yes', 'on'] and str('",
        lc['use_rear_camera_apriltag_container'],
        "').lower() in ['1', 'true', 'yes', 'on'] and str('",
        lc['enable_sensing_module'],
        "').lower() in ['1', 'true', 'yes', 'on'] and str('",
        lc['enable_camera'],
        "').lower() in ['1', 'true', 'yes', 'on'] and str('",
        lc['enable_rear_camera'],
        "').lower() in ['1', 'true', 'yes', 'on'] and str('",
        lc['enable_parking'],
        "').lower() in ['1', 'true', 'yes', 'on'] and str('",
        lc['parking_method'],
        "').strip().lower() == 'apriltag' else 'false'",
    ])
    resolve_rear_camera_apriltag_container_active = SetLaunchConfiguration(
        'rear_camera_apriltag_container_active_resolved',
        rear_camera_apriltag_container_active_expression,
    )
    rear_camera_apriltag_container_active = LaunchConfiguration(
        'rear_camera_apriltag_container_active_resolved'
    )
    regular_rear_camera_enable = PythonExpression([
        "'false' if str('", lc['sim'],
        "').lower() in ['1', 'true', 'yes', 'on'] or str('",
        rear_camera_apriltag_container_active,
        "').lower() in ['1', 'true', 'yes', 'on'] else '",
        lc['enable_rear_camera'], "'",
    ])
    regular_apriltag_detector_enable = PythonExpression([
        "'false' if str('", rear_camera_apriltag_container_active,
        "').lower() in ['1', 'true', 'yes', 'on'] else 'true'",
    ])
    rear_camera_apriltag_container_condition = IfCondition(
        rear_camera_apriltag_container_active
    )
    # HH_260805 - Resolve SHM before entering the sensing child scope. It is
    # inactive in simulation and never changes the RMW environment of the full
    # CAMROD graph; only the physical LiDAR launch receives this true value.
    lidar_dds_shared_memory_active = PythonExpression([
        "'true' if str('", lc['enable_dds_shared_memory'],
        "').lower() in ['1', 'true', 'yes', 'on'] and str('",
        lc['sim'], "').lower() not in ['1', 'true', 'yes', 'on'] and str('",
        lc['enable_sensing_module'],
        "').lower() in ['1', 'true', 'yes', 'on'] and str('",
        lc['enable_lidar_driver'],
        "').lower() in ['1', 'true', 'yes', 'on'] else 'false'",
    ])
    sensing_args = {
        # sensing.launch.py declares `sensing_namespace` (not `module_namespace`).
        'sensing_namespace': lc['sensing_namespace'],
        'gnss_namespace': lc['gnss_namespace'],
        'gnss_rtcm_topic': lc['gnss_rtcm_topic'],
        # In sim mode, fake_sensors.launch.py already publishes synthetic
        # GNSS/IMU/wheel data and obstacle cloud, so keep hardware drivers off.
        'enable_camera': sim_switch(lc['sim'], 'false', lc['enable_camera']),
        'enable_front_camera': regular_front_camera_enable,
        'enable_rear_camera': regular_rear_camera_enable,
        # The component container owns the physical front camera when active;
        # tell sensing.launch so it does not mistake the scoped regular-camera
        # false value for a disabled sensor and start a duplicate dummy.
        'front_camera_source_external': camera_yolo_container_active,
        'rear_camera_source_external': rear_camera_apriltag_container_active,
        'publish_sensor_dummies_when_disabled': sim_switch(
            lc['sim'],
            'false',
            lc['publish_sensor_dummies_when_disabled'],
        ),
        'enable_ntrip': sim_switch(lc['sim'], 'false', lc['enable_ntrip']),
        'enable_radar': sim_switch(lc['sim'], 'false', lc['enable_radar']),
        'radar_log_status': lc['radar_log_status'],
        'enable_radar_cost_grid': lc['enable_radar_cost_grid'],
        'enable_lidar_cost_grid': lc['enable_lidar_cost_grid'],
        'use_lidar_processing_container': lc['use_lidar_processing_container'],
        'enable_dds_shared_memory': lidar_dds_shared_memory_active,
        'dds_shared_memory_cyclonedds_config': lc[
            'dds_shared_memory_cyclonedds_config'
        ],
        'enable_inflation_cost_grid': lc['enable_inflation_cost_grid'],
        'enable_lidar_driver': sim_switch(lc['sim'], 'false', lc['enable_lidar_driver']),
        'enable_imu':      sim_switch(lc['sim'], 'false', lc['enable_imu']),
        'imu_model':       lc['imu_model'],
        'enable_gnss': sim_switch(lc['sim'], 'false', lc['enable_gnss']),
        # HH_260722 - Forward the complete verified GNSS cascade from bringup;
        # omitting any of these would let a child-launch default drift silently.
        'ublox_dual_antenna': lc['ublox_dual_antenna'],
        'ublox_dual_forward_ntrip_to_rover': lc['ublox_dual_forward_ntrip_to_rover'],
        'ublox_dual_warm_start_on_startup': lc['ublox_dual_warm_start_on_startup'],
        'ublox_dual_base_rtcm_device': lc['ublox_dual_base_rtcm_device'],
        'ublox_dual_base_rtcm_baud': lc['ublox_dual_base_rtcm_baud'],
        'camera_device_path': lc['camera_device_path'],
        # HH_260527: Removed unused pass-through args
        # (system_namespace, gnss_navsatfix_topic, enable_module_validator).
    }
    apply_cfg_overrides(sensing_args, sensing_overrides)

    perception_args = {
        'module_namespace': lc['perception_namespace'],
        'enable_lidar_obstacle': lc['perception_enable_lidar_obstacle'],
        # In sim mode, default to LiDAR-only perception to avoid GPU/TensorRT dependency.
        'enable_yolo': regular_yolo_enable,
        # HH_260723 - Pass raw parent switches under distinct child inputs. Building a
        # derived `enable_camera` substitution here made the child launch
        # resolve its own argument recursively and disabled obstacle_fusion
        # whenever the camera+YOLO component container was selected.
        'enable_camera': lc['enable_camera'],
        'enable_front_camera': lc['enable_front_camera'],
        'sim': lc['sim'],
        'camera_device_path': lc['camera_device_path'],
        'perception_mode': lc['perception_mode'],
        'camping_sites_yaml': lc['planning_state_machine_camping_sites_yaml'],
    }
    apply_cfg_overrides(perception_args, perception_overrides)

    camera_yolo_container_args = {
        'enable_container': camera_yolo_container_active,
        'camera_params_file': sensing_args.get('camera_params_file', pkg_path('camrod_sensing', 'config/camera/camera_params.yaml')),
        'perception_param_file': perception_args.get('perception_param_file', pkg_path('camrod_perception', 'config/perception_params.yaml')),
        'front_camera_namespace': '/sensing/camera/econ_front',
        'perception_namespace': lc['perception_namespace'],
    }
    rear_camera_apriltag_container_args = {
        'enable_container': rear_camera_apriltag_container_active,
        'camera_params_file': sensing_args.get(
            'camera_params_file',
            pkg_path('camrod_sensing', 'config/camera/camera_params.yaml'),
        ),
        'apriltag_params_file': lc['apriltag_param_file'],
        'rear_camera_namespace': '/sensing/camera/econ_rear',
        'perception_namespace': lc['perception_namespace'],
    }

    # HH_260721 - Keep the configured platform wheel source on the single EKF path.
    wheel_input_topic_for_filter = lc['wheel_input_topic']
    wheel_input_type_for_filter = lc['wheel_input_type']

    # HH_260618: In sim mode, relax EKF GNSS rejection so the fake GNSS start
    # pose becomes the planning/control truth instead of leaving EKF at map origin.
    _ekf_sim_cfg = os.path.join(config_root_default, 'localization', 'filter', 'ekf_sim.yaml')
    _ekf_real_cfg = localization_overrides.get('filter_ekf_param_file', '')
    _ekf_cfg = sim_switch(lc['sim'], _ekf_sim_cfg, _ekf_real_cfg or '')

    # HH_260617: In sim planning tests, automatic GNSS reattach can teleport the
    # EKF pose away from the active Nav2 path. Keep the node for manual
    # initialpose reset bridging, but use a sim parameter file that disables
    # automatic distance-based reattach.
    _gnss_reattach_sim_cfg = os.path.join(
        config_root_default, 'localization', 'filter', 'gnss_reattach_sim.yaml')
    _gnss_reattach_real_cfg = localization_overrides.get(
        'filter_gnss_reattach_param_file', '')
    _gnss_reattach_cfg = sim_switch(
        lc['sim'], _gnss_reattach_sim_cfg, _gnss_reattach_real_cfg or '')

    localization_args = {
        'module_namespace': lc['localization_namespace'],
        'enable_adapter': lc['localization_enable_adapter'],
        'enable_filter': lc['localization_enable_filter'],
        'enable_monitor': lc['localization_enable_monitor'],
        'enable_map_helper': lc['localization_enable_map_helper'],
        # HH_260713: Map and GNSS projection must consume the exact same map-info
        # file.  Falling back inside camrod_localization can silently select the
        # package copy while full bringup loads its bringup-local map copy.
        'map_info_file': lc['map_info_file'],
        'wheel_bridge_enable': lc['wheel_bridge_enable'],
        'wheel_input_topic': wheel_input_topic_for_filter,
        'wheel_input_type': wheel_input_type_for_filter,
        'wheel_fallback_input_topic': lc['wheel_fallback_input_topic'],
        'wheel_fallback_input_type': lc['wheel_fallback_input_type'],
        'wheel_primary_timeout_s': lc['wheel_primary_timeout_s'],
        'wheel_output_topic': lc['wheel_output_topic'],
        'wheel_nav_output_topic': lc['wheel_nav_output_topic'],
        'map_path': lc['map_path'],
    }
    apply_cfg_overrides(localization_args, localization_overrides)
    if 'drop_zones_yaml' not in localization_args:
        # HH_260623 - Keep localization map helper aligned with planning/parking drop-zone semantics.
        localization_args['drop_zones_yaml'] = lc['planning_state_machine_keypoints_yaml']
    # HH_260618: Apply sim EKF override after apply_cfg_overrides so it is not
    # overwritten by user-level localization/filter_ekf_param_file entries.
    localization_args['filter_ekf_param_file'] = _ekf_cfg
    localization_args['filter_gnss_reattach_param_file'] = _gnss_reattach_cfg

    planning_args = {
        'use_nav2_container': lc['use_nav2_container'],
        'enable_path_cost_grids': lc['enable_path_cost_grids'],
        'enable_goal_replanner': lc['enable_goal_replanner'],
        'enable_obstacle_replan_monitor': lc['enable_obstacle_replan_monitor'],
        'enable_nav2_lifecycle_retry': lc['enable_nav2_lifecycle_retry'],
        'require_localization_ready': lc['require_localization_ready'],
        'enable_state_machine': lc['enable_state_machine'],
        'enable_progress': lc['enable_progress'],
        'enable_path_visualization': lc['enable_path_visualization'],
        'planning_state_machine_keypoints_yaml': lc['planning_state_machine_keypoints_yaml'],
        'planning_state_machine_camping_sites_yaml': lc['planning_state_machine_camping_sites_yaml'],
        # HH_260617: In sim, avoid ERROR_STOP from intentionally missing/stale
        # hardware diagnostics so RViz/UI planning-control tests can move.
        'planning_state_machine_param_file': sim_switch(
            lc['sim'],
            os.path.join(config_root_default, 'planning', 'planning_state_machine_sim.yaml'),
            lc['planning_state_machine_param_file'],
        ),
        'map_path': lc['map_path'],
        'origin_lat': lc['origin_lat'],
        'origin_lon': lc['origin_lon'],
        'origin_alt': lc['origin_alt'],
        # Keep centerline anchor on fused localization pose.
        'centerline_input_pose_topic': '/localization/pose',
        'local_path_pose_topic': lc['local_path_pose_topic'],
        # HH_260720 - Planning emits an unapproved navigation command for camrod_control.
        # HH_260720 - Keep Nav2 standard Twist on the explicit ROS boundary.
        'navigation_cmd_vel_topic': lc['control_navigation_cmd_vel_ros_topic'],
        'module_namespace': lc['planning_namespace'],
        'nav2_robot_base_frame': lc['robot_center_frame_id'],
        # HH_260528: Keep selector IDs as raw values (not file-path overrides).
        'nav2_selected_planner': lc['planning_nav2_selected_planner'],
        'nav2_selected_controller': lc['planning_nav2_selected_controller'],
    }
    set_if_not_empty(planning_args, 'nav2_base_param_file', planning_overrides['nav2_base_param_file'])
    set_if_not_empty(planning_args, 'nav2_vehicle_param_file', selected_nav2_vehicle_override)
    set_if_not_empty(planning_args, 'nav2_lanelet_param_file', planning_overrides['nav2_lanelet_param_file'])
    set_if_not_empty(planning_args, 'nav2_behavior_param_file', planning_overrides['nav2_behavior_param_file'])
    set_if_not_empty(planning_args, 'nav2_combo_param_file', planning_overrides['nav2_combo_param_file'])
    set_if_not_empty(
        planning_args,
        'nav2_planner_plugins_param_file',
        planning_overrides['nav2_planner_plugins_param_file'],
    )
    set_if_not_empty(
        planning_args,
        'nav2_controller_plugins_param_file',
        planning_overrides['nav2_controller_plugins_param_file'],
    )
    set_if_not_empty(planning_args, 'path_cost_grids_param_file', planning_overrides['path_cost_grids_param_file'])
    set_if_not_empty(planning_args, 'goal_snapper_param_file', planning_overrides['goal_snapper_param_file'])
    set_if_not_empty(planning_args, 'centerline_snapper_param_file', planning_overrides['centerline_snapper_param_file'])
    set_if_not_empty(planning_args, 'goal_replanner_param_file', planning_overrides['goal_replanner_param_file'])
    set_if_not_empty(
        planning_args,
        'obstacle_replan_monitor_param_file',
        planning_overrides['obstacle_replan_monitor_param_file'],
    )
    set_if_not_empty(planning_args, 'local_path_extractor_param_file', planning_overrides['local_path_extractor_param_file'])
    # HH_260720 - Build the camrod_control safety-gate argument contract.
    safety_gate_args = {
        'module_namespace': lc['control_namespace'],
        'cmd_vel_gate_enable': lc['control_cmd_vel_gate_enable'],
        'cmd_vel_raw_topic': lc['control_cmd_vel_raw_topic'],
        'navigation_cmd_vel_ros_topic': lc['control_navigation_cmd_vel_ros_topic'],
        'cmd_vel_output_topic': lc['control_cmd_vel_topic'],
        'cmd_vel_ros_output_topic': lc['control_cmd_vel_ros_topic'],
        'planning_engage_topic': lc['planning_engage_topic'],
        'planning_mission_engage_topic': lc['planning_mission_engage_topic'],
        'command_enabled_topic': lc['control_command_enabled_topic'],
        # HH_260720 - Drive-enable and command timeout are enforced by this single gate.
        'platform_drive_enable_topic': lc['platform_drive_enable_topic'],
        **{k[len('control_'):]: lc[k] for k in lc if k.startswith('control_cmd_vel_gate_')},
    }
    # HH_260721 - Normal sim disables CAN gating; charging validation supplies a fake heartbeat.
    safety_gate_args['cmd_vel_gate_platform_safety_source_mode'] = PythonExpression([
        "'", lc['control_cmd_vel_gate_platform_safety_source_mode'], "' if ('",
        lc['sim'], "' != 'true' or '", lc['sim_platform_status_enable'],
        "' == 'true') else 'disabled'",
    ])
    set_if_not_empty(
        safety_gate_args,
        'cmd_vel_gate_yaw_alignment_zones_file',
        control_overrides['cmd_vel_gate_yaw_alignment_zones_file'],
    )

    diagnostics_profile_runtime = PythonExpression([
        "'sim' if str('", lc['sim'], "').lower() in ['1', 'true', 'yes', 'on'] "
        "and str('", lc['diagnostics_profile'], "') == 'default' "
        "else str('", lc['diagnostics_profile'], "')"
    ])
    _system_config_root = os.path.join(config_root_default, 'system')
    _system_checker_default_cfg = _first_existing_path([
        os.path.join(_system_config_root, 'system_checker.yaml'),
        pkg_path('camrod_system', os.path.join('config', 'system_checker.yaml')),
    ])
    _system_checker_sim_cfg = _first_existing_path([
        os.path.join(_system_config_root, 'system_checker_sim.yaml'),
        pkg_path('camrod_system', os.path.join('config', 'system_checker_sim.yaml')),
    ])
    _system_diagnostics_config_root = os.path.join(_system_config_root, 'diagnostics')
    if not os.path.isdir(_system_diagnostics_config_root):
        _system_diagnostics_config_root = pkg_path(
            'camrod_system', os.path.join('config', 'diagnostics'))
    system_checker_param_runtime = PythonExpression([
        "'", _system_checker_sim_cfg, "' if str('", lc['sim'],
        "').lower() in ('1', 'true', 'yes', 'on') "
        "and str('", lc['system_checker_param_file'],
        "') in ('', '__module_default__') else ('",
        _system_checker_default_cfg, "' if str('", lc['system_checker_param_file'],
        "') in ('', '__module_default__') else str('",
        lc['system_checker_param_file'], "'))",
    ])

    system_args = {
        'enable_checkers': lc['enable_module_validators'],
        'use_system_tools_container': lc['use_system_tools_container'],
        'use_checker_components': lc['use_checker_components'],
        'checker_component_groups': lc['checker_component_groups'],
        'checker_component_threads': lc['checker_component_threads'],
        # HH_260617: sim defaults to the diagnostics/sim profile so hardware-only
        # checks do not block planning/control validation.
        'config_profile': diagnostics_profile_runtime,
        # HH_260630 - Match graph-readiness manifest to the same hardware/sim mode
        # as diagnostics_profile; hardware driver nodes are not required in sim.
        'system_checker_param_file': system_checker_param_runtime,
        # HH_260630 - Use synchronized bringup/system diagnostics when present.
        'diagnostics_config_root': _system_diagnostics_config_root,
        'enable_platform': lc['enable_platform_checker'],
        # HH_260805 - Diagnostics remove the optional LiDAR grid contract when
        # the component is not loaded, instead of reporting a false system error.
        'enable_lidar_cost_grid': lc['enable_lidar_cost_grid'],
        'module_namespace': lc['system_namespace'],
    }

    # HH_260720 - Maneuver launch owns crab, zero-turn, and drop-zone alignment.
    maneuver_args = {
        'control_namespace': lc['control_namespace'],
        'enable_camping_site_maneuver_controller': lc['enable_camping_site_maneuver_controller'],
        'enable_drop_zone_maneuver_controller': lc['enable_drop_zone_maneuver_controller'],
        'enable_route_safety_recovery_controller': lc['enable_route_safety_recovery_controller'],
        'command_topic': lc['control_cmd_vel_raw_topic'],
        'vehicle_pose_topic': '/localization/pose',
        'drop_zones_yaml': lc['planning_state_machine_keypoints_yaml'],
        'camping_sites_yaml': lc['planning_state_machine_camping_sites_yaml'],
        'parameter_file': lc['control_param_file'],
    }

    # HH_260720 - Parking launch selects reverse or AprilTag implementation internally.
    parking_args = {
        'parking_namespace': lc['parking_namespace'],
        'parking_method': lc['parking_method'],
        'command_topic': lc['control_cmd_vel_raw_topic'],
        'vehicle_pose_topic': '/localization/pose',
        'drop_zones_yaml': lc['planning_state_machine_keypoints_yaml'],
        'parameter_file': lc['parking_param_file'],
        'apriltag_parameter_file': lc['apriltag_param_file'],
        'launch_apriltag_detector': regular_apriltag_detector_enable,
    }

    voice_args = {
        'voice_namespace':      lc['voice_namespace'],
        'enable_voice_adapter': lc['enable_voice_adapter'],
    }

    api_args = {
        'enable_plugin_api': lc['enable_plugin_api'],
        'enable_ui_backend': lc['enable_api_ui'],
        'ui_host': lc['api_ui_host'],
        'ui_port': lc['api_ui_port'],
        # HH_260810 - Keep the bounded RViz-replacement workspace explicit at
        # the top-level deployment boundary for constrained ARM64 targets.
        'enable_operator_telemetry': lc['enable_operator_telemetry'],
        'enable_ui_guest': lc['enable_guest_ui'],
        'guest_host': lc['guest_ui_host'],
        'guest_port': lc['guest_ui_port'],
        # HH_260724 - Pass the bringup-level SOC policy to camrod_ui.launch.py.
        'require_battery_for_mission_dispatch': lc['api_ui_require_battery_for_mission_dispatch'],
        'minimum_mission_dispatch_battery_percent': lc['api_ui_minimum_mission_dispatch_battery_percent'],
        'low_battery_return_after_current_mission': lc['api_ui_low_battery_return_after_current_mission'],
        'low_battery_return_threshold_percent': lc['api_ui_low_battery_return_threshold_percent'],
        # HH_260727 - Keep headless opt-out and window geometry explicit at top level.
        'enable_operator_ui_window': lc['enable_operator_ui_window'],
        'operator_ui_window_engine': lc['operator_ui_window_engine'],
        'operator_ui_window_url': lc['operator_ui_window_url'],
        'operator_ui_window_width': lc['operator_ui_window_width'],
        'operator_ui_window_height': lc['operator_ui_window_height'],
        'operator_ui_window_fullscreen': lc['operator_ui_window_fullscreen'],
        # Share bringup camping-sites YAML with UI backend so
        # /ui/selected_destination can dispatch exact goal_pose coordinates.
        'camping_sites_yaml': lc['planning_state_machine_camping_sites_yaml'],
        # HH_260623 - UI campsite missions publish a separate mission engage latch
        # so the manual ENGAGE button cannot stop an accepted scenario.
        'planning_engage_topic': lc['planning_engage_topic'],
        'planning_mission_engage_topic': lc['planning_mission_engage_topic'],
        'platform_drive_enable_topic': lc['platform_drive_enable_topic'],
        'camping_site_maneuver_controller_operation_topic': '/control/camping_site_maneuver_controller/operation',
        'camping_site_maneuver_controller_adopt_topic': '/control/camping_site_maneuver_controller/adopt',
        # HH_260721 - Let the UI release final parking before drop-zone departure.
        'parking_operation_topic': '/parking/operation',
        'arrival_pose_topic': '/localization/pose',
        'platform_status_topic': '/platform/status',
    }

    module_specs = [
        ('camrod_platform', 'platform.launch.py', platform_args, None),
        ('camrod_map', 'map.launch.py', map_args, None),
        ('camrod_bringup', 'fake_sensors.launch.py', fake_sensors_args, IfCondition(lc['sim'])),
        ('camrod_bringup', 'camera_yolo_container.launch.py', camera_yolo_container_args, camera_yolo_container_condition),
        (
            'camrod_bringup',
            'rear_camera_apriltag_container.launch.py',
            rear_camera_apriltag_container_args,
            rear_camera_apriltag_container_condition,
        ),
        (
            'camrod_sensing',
            'sensing.launch.py',
            sensing_args,
            IfCondition(lc['enable_sensing_module']),
        ),
        ('camrod_perception', 'perception.launch.py', perception_args, None),
        ('camrod_localization', 'localization.launch.py', localization_args, None),
        ('camrod_planning', 'planning.launch.py', planning_args, IfCondition(lc['enable_planning'])),
        ('camrod_control', 'cmd_vel_safety_gate.launch.py', safety_gate_args, control_command_gate_condition),
        ('camrod_control', 'maneuvers.launch.py', maneuver_args, parking_enabled_condition),
        ('camrod_control', 'parking.launch.py', parking_args, parking_enabled_condition),
        ('camrod_voice',    'voice.launch.py',    voice_args,   IfCondition(lc['enable_voice'])),
        # Launch unified diagnostics stack via top-level system.launch.py.
        ('camrod_system', 'system.launch.py', system_args, None),
    ]
    optional_modules = []
    ui_launch_exists, ui_launch_path = has_launch_file('camrod_ui', 'ui.launch.py')
    if ui_launch_exists:
        # HH_260611: Start UI only when an API/UI surface is requested and the launch file exists.
        ui_condition = IfCondition(
            PythonExpression([
                "'", lc['enable_plugin_api'], "' == 'true' or '",
                lc['enable_api_ui'], "' == 'true' or '",
                lc['enable_operator_ui_window'], "' == 'true'",
            ])
        )
        module_specs.append(('camrod_ui', 'ui.launch.py', api_args, ui_condition))
    else:
        optional_modules.append(
            LogInfo(
                msg=(
                    '[bringup] camrod_ui skipped: '
                    f'launch file not found{": " + ui_launch_path if ui_launch_path else ""}'
                )
            )
        )
    modules = []
    for pkg, launch_file, launch_args, condition in module_specs:
        # HH_260617: Optional feature packages may be intentionally unavailable on
        # development PCs. Check required executables as well as launch-file presence
        # so stale install/share artifacts do not make bringup fail at runtime.
        if pkg in OPTIONAL_MODULE_EXECUTABLES:
            available, reason = optional_module_available(pkg, launch_file)
            if not available:
                modules.append(LogInfo(msg=f'[bringup] {pkg} skipped: {reason}', condition=condition))
                continue
        modules.append(include(pkg, launch_file, launch_args, condition=condition))
    modules += optional_modules

    # Removed bringup_status runtime node from default launch path.

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', pkg_path('camrod_map', 'rviz/camrod_operator.rviz'),
            '-stylesheet', pkg_path('camrod_map', 'rviz/operator_theme.qss'),
        ],
        output='screen',
        # Auto-restart RViz when plugin/TF race causes transient crash.
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

    # Sequence module starts deterministically: cleanup must exit before the module
    # stack launches to prevent pkill from killing freshly started nodes.
    # Modules are staggered by module_launch_gap_s to reduce startup burst load.
    staged_modules = [
        TimerAction(
            period=PythonExpression([str(idx), " * ", lc['module_launch_gap_s']]),
            actions=[module_action],
        )
        for idx, module_action in enumerate(modules)
    ]
    launch_stack = GroupAction([*staged_modules, rviz_node])
    delayed_stack = TimerAction(period=1.0, actions=[launch_stack])
    # HH_260805 - RouDi is needed only by the scoped physical LiDAR participants.
    # Other ROS processes retain their host RMW and cannot exhaust iceoryx ports.
    roudi_process = ExecuteProcess(
        cmd=[
            lc['dds_shared_memory_roudi_executable'],
            '--config-file', lc['dds_shared_memory_roudi_config'],
            '--log-level', lc['dds_shared_memory_roudi_log_level'],
        ],
        output='screen',
        condition=IfCondition(lidar_dds_shared_memory_active),
    )
    start_stack_actions = [roudi_process, delayed_stack]

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
        resolve_camera_yolo_container_active,
        resolve_rear_camera_apriltag_container_active,
        clean_action,
        start_after_cleanup,
        start_without_cleanup,
        shutdown_cleanup,
    ])
