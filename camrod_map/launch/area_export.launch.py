#!/usr/bin/env python3

import os
import re
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def extract_map_ros_params(map_info_cfg: dict) -> dict:
    if not isinstance(map_info_cfg, dict):
        return {}
    wildcard = map_info_cfg.get("/**", None)
    if isinstance(wildcard, dict):
        params = wildcard.get("ros__parameters")
        if isinstance(params, dict):
            return params
    for val in map_info_cfg.values():
        if isinstance(val, dict):
            params = val.get("ros__parameters")
            if isinstance(params, dict):
                return params
    return {}


def _dedup_paths(paths: list[str]) -> list[str]:
    out = []
    seen = set()
    for path in paths:
        normalized = os.path.abspath(path)
        if normalized in seen:
            continue
        seen.add(normalized)
        out.append(normalized)
    return out


def _first_existing_file(candidates: list[str]) -> str:
    for candidate in _dedup_paths(candidates):
        if os.path.isfile(candidate):
            return candidate
    return ""


def _first_existing_dir(candidates: list[str]) -> str:
    for candidate in _dedup_paths(candidates):
        if os.path.isdir(candidate):
            return candidate
    return ""


def _workspace_src_candidates() -> list[str]:
    candidates = [
        os.path.join(os.path.expanduser("~"), "camrod_ws", "src"),
        os.path.join(os.getcwd(), "src"),
        os.getcwd(),
    ]
    cur = os.path.abspath(os.path.dirname(__file__))
    for _ in range(10):
        if os.path.isdir(os.path.join(cur, "camrod_map")):
            candidates.append(cur)
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    return _dedup_paths(candidates)


def _default_file_path(relative_path: str, package_share_path: str) -> str:
    src_candidates = []
    for src_root in _workspace_src_candidates():
        src_candidates.append(os.path.join(src_root, relative_path))
    discovered = _first_existing_file(src_candidates)
    if discovered:
        return discovered
    return os.path.join(package_share_path, os.path.basename(relative_path))


def _default_output_path(relative_path: str, package_share_path: str) -> str:
    src_candidates = [os.path.join(src_root, relative_path) for src_root in _workspace_src_candidates()]
    discovered = _first_existing_file(src_candidates)
    if discovered:
        return discovered
    src_parent_candidates = [os.path.dirname(path) for path in src_candidates]
    src_parent = _first_existing_dir(src_parent_candidates)
    if src_parent:
        return os.path.join(src_parent, os.path.basename(relative_path))
    return os.path.join(package_share_path, os.path.basename(relative_path))


def _normalize_profile_name(value: str) -> str:
    text = str(value or "").strip()
    if not text:
        return ""
    text = text.replace("(", "_").replace(")", "_")
    text = re.sub(r"[^A-Za-z0-9_]+", "_", text)
    return re.sub(r"_+", "_", text).strip("_").lower()


def _profiled_output_path(relative_path: str, map_profile: str) -> str:
    # HHL_260623 - Export active map-profile semantics to the same YAML selected by bringup/planning.
    profile = _normalize_profile_name(map_profile)
    if not profile:
        return relative_path
    directory, filename = os.path.split(relative_path)
    stem, ext = os.path.splitext(filename)
    return os.path.join(directory, f"{stem} ({profile}){ext}")


def _read_map_info_defaults(map_info_file: str) -> dict:
    defaults = {
        "map_path": "",
        "map_profile": "",
        "origin_lat": "",
        "origin_lon": "",
        "origin_alt": "",
    }
    if not map_info_file:
        return defaults
    try:
        with open(map_info_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        params = extract_map_ros_params(data)
        map_path = str(params.get("map_path", "")).strip()
        if map_path and not os.path.isabs(map_path):
            map_path = os.path.abspath(os.path.join(os.path.dirname(map_info_file), map_path))
        defaults["map_path"] = map_path
        defaults["map_profile"] = str(params.get("map_profile", params.get("profile", ""))).strip()
        defaults["origin_lat"] = str(params.get("offset_lat", "")).strip()
        defaults["origin_lon"] = str(params.get("offset_lon", "")).strip()
        defaults["origin_alt"] = str(params.get("offset_alt", "")).strip()
    except Exception:
        pass
    return defaults


def _resolve_default_map_path(configured_map_path: str, map_share: str, map_info_file: str) -> str:
    configured = str(configured_map_path or "").strip()
    if configured and os.path.isfile(configured):
        return configured
    candidates = []
    for src_root in _workspace_src_candidates():
        candidates.append(os.path.join(src_root, "lanelet2_maps.osm"))
    candidates.extend([
        os.path.join(map_share, "lanelet2_maps.osm"),
        os.path.join(os.getcwd(), "lanelet2_maps.osm"),
        os.path.join(os.getcwd(), "src", "lanelet2_maps.osm"),
    ])
    for anchor in (map_share, os.path.dirname(map_info_file)):
        cur = os.path.abspath(anchor)
        for _ in range(8):
            candidates.append(os.path.join(cur, "lanelet2_maps.osm"))
            candidates.append(os.path.join(cur, "src", "lanelet2_maps.osm"))
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent
    discovered = _first_existing_file(candidates)
    return discovered if discovered else configured


def _optional_float(value: str):
    if value is None:
        return None
    stripped = value.strip()
    if not stripped:
        return None
    try:
        return float(stripped)
    except (TypeError, ValueError):
        return None


def _launch_node(context, *_args, **_kwargs):
    map_info_file = LaunchConfiguration("map_info_file").perform(context).strip()
    map_info_defaults = _read_map_info_defaults(map_info_file)

    overrides = {}
    map_path = LaunchConfiguration("map_path").perform(context).strip()
    if not map_path:
        map_path = map_info_defaults["map_path"]
    if map_path:
        overrides["map_path"] = map_path

    origin_lat = _optional_float(LaunchConfiguration("origin_lat").perform(context))
    origin_lon = _optional_float(LaunchConfiguration("origin_lon").perform(context))
    origin_alt = _optional_float(LaunchConfiguration("origin_alt").perform(context))
    if origin_lat is None:
        origin_lat = _optional_float(map_info_defaults["origin_lat"])
    if origin_lon is None:
        origin_lon = _optional_float(map_info_defaults["origin_lon"])
    if origin_alt is None:
        origin_alt = _optional_float(map_info_defaults["origin_alt"])

    if origin_lat is not None:
        overrides["origin_lat"] = origin_lat
    if origin_lon is not None:
        overrides["origin_lon"] = origin_lon
    if origin_alt is not None:
        overrides["origin_alt"] = origin_alt

    default_yaw_deg = _optional_float(LaunchConfiguration("default_yaw_deg").perform(context))
    if default_yaw_deg is not None:
        overrides["default_yaw_deg"] = default_yaw_deg

    output_yaml_path = LaunchConfiguration("output_yaml_path").perform(context).strip()
    if output_yaml_path:
        overrides["output_yaml_path"] = output_yaml_path

    camping_sites_output_yaml_path = LaunchConfiguration(
        "camping_sites_output_yaml_path").perform(context).strip()
    if camping_sites_output_yaml_path:
        overrides["camping_sites_output_yaml_path"] = camping_sites_output_yaml_path

    return [
        Node(
            package="camrod_map",
            executable="area_exporter_node",
            name="area_exporter",
            output="screen",
            parameters=[overrides],
        )
    ]


def generate_launch_description():
    map_share = get_package_share_directory("camrod_map")
    planning_share = get_package_share_directory("camrod_planning")

    default_map_info = _default_file_path(
        os.path.join("camrod_map", "config", "map_info.yaml"),
        os.path.join(map_share, "config", "map_info.yaml"),
    )
    map_info_defaults = _read_map_info_defaults(default_map_info)
    default_map_path = _resolve_default_map_path(
        map_info_defaults["map_path"], map_share, default_map_info)

    default_drop_zones = _default_output_path(
        _profiled_output_path(
            os.path.join("camrod_map", "config", "drop_zones.yaml"),
            map_info_defaults["map_profile"],
        ),
        os.path.join(map_share, "config", "drop_zones.yaml"),
    )
    default_camping_sites = _default_output_path(
        _profiled_output_path(
            os.path.join("camrod_planning", "config", "camping_sites.yaml"),
            map_info_defaults["map_profile"],
        ),
        os.path.join(planning_share, "config", "camping_sites.yaml"),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_info_file",
            default_value=default_map_info,
            description="Shared map info YAML used for map/origin defaults",
        ),
        DeclareLaunchArgument(
            "map_path",
            default_value=default_map_path,
            description="Lanelet2 OSM map path for area export",
        ),
        DeclareLaunchArgument(
            "origin_lat",
            default_value=map_info_defaults["origin_lat"],
            description="Map origin latitude",
        ),
        DeclareLaunchArgument(
            "origin_lon",
            default_value=map_info_defaults["origin_lon"],
            description="Map origin longitude",
        ),
        DeclareLaunchArgument(
            "origin_alt",
            default_value=map_info_defaults["origin_alt"],
            description="Map origin altitude",
        ),
        DeclareLaunchArgument(
            "default_yaw_deg",
            default_value="0.0",
            description="Fallback yaw value when zone orientation cannot be inferred",
        ),
        DeclareLaunchArgument(
            "output_yaml_path",
            default_value=default_drop_zones,
            description="Output YAML path for drop zones",
        ),
        DeclareLaunchArgument(
            "camping_sites_output_yaml_path",
            default_value=default_camping_sites,
            description="Output YAML path for camping sites",
        ),
        OpaqueFunction(function=_launch_node),
    ])
