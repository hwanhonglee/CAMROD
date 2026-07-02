#!/usr/bin/env python3

import os
import re
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def _inc(path, *through, **overrides):
    args = {k: LaunchConfiguration(k) for k in through}
    args.update(overrides)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=args.items(),
    )


def extract_map_ros_params(map_info_cfg: dict) -> dict:
    if not isinstance(map_info_cfg, dict):
        return {}
    wildcard = map_info_cfg.get("/**", None)
    if isinstance(wildcard, dict):
        params = wildcard.get("ros__parameters")
        if isinstance(params, dict):
            return params
    for key in ("/map/lanelet2_map", "map/lanelet2_map", "lanelet2_map", "/lanelet2_map"):
        val = map_info_cfg.get(key)
        if isinstance(val, dict):
            params = val.get("ros__parameters")
            if isinstance(params, dict):
                return params
    for val in map_info_cfg.values():
        if isinstance(val, dict):
            params = val.get("ros__parameters")
            if isinstance(params, dict):
                return params
    return {}


def load_defaults(default_map_info: str) -> dict:
    defaults = {"map_path": "", "map_profile": "", "origin_lat": "", "origin_lon": "", "origin_alt": ""}
    try:
        with open(default_map_info, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        params = extract_map_ros_params(data)
        defaults["map_path"]    = str(params.get("map_path", "")).strip()
        defaults["map_profile"] = str(params.get("map_profile", params.get("profile", ""))).strip()
        defaults["origin_lat"]  = str(params.get("offset_lat", "")).strip()
        defaults["origin_lon"]  = str(params.get("offset_lon", "")).strip()
        defaults["origin_alt"]  = str(params.get("offset_alt", "")).strip()
    except Exception:
        pass
    return defaults


def _first_existing_path(candidates: list[str]) -> str:
    seen = set()
    for candidate in candidates:
        normalized = os.path.abspath(candidate)
        if normalized in seen:
            continue
        seen.add(normalized)
        if os.path.isfile(normalized):
            return normalized
    return ""


def _normalize_profile_name(value: str) -> str:
    text = str(value or "").strip()
    if not text:
        return ""
    text = text.replace("(", "_").replace(")", "_")
    text = re.sub(r"[^A-Za-z0-9_]+", "_", text)
    text = re.sub(r"_+", "_", text).strip("_").lower()
    return text


def _map_filename_candidates(configured: str, map_profile: str) -> list[str]:
    filenames = []
    configured_name = os.path.basename(str(configured or "").strip())
    if configured_name:
        filenames.append(configured_name)
    profile = _normalize_profile_name(map_profile)
    if profile:
        filenames.extend([
            f"lanelet2_maps_({profile}).osm",
            f"lanelet2_maps_{profile}.osm",
            f"lanelet2_maps-{profile}.osm",
        ])
    filenames.append("lanelet2_maps.osm")

    ordered = []
    for filename in filenames:
        if filename and filename not in ordered:
            ordered.append(filename)
    return ordered


def discover_map_path(map_share: str, map_info_file: str, map_path_from_info: str, map_profile: str = "") -> str:
    # HH_260622 - Map launch can select any lanelet2 map profile without hardcoding C-track.
    configured = str(map_path_from_info or "").strip()
    if configured:
        configured_path = (
            configured
            if os.path.isabs(configured)
            else os.path.abspath(os.path.join(os.path.dirname(map_info_file), configured))
        )
        if os.path.isfile(configured_path):
            return configured_path

    candidates = []
    filenames = _map_filename_candidates(configured, map_profile)
    anchors = [
        os.path.join(os.path.expanduser("~"), "camrod_ws", "src"),
        os.getcwd(),
        os.path.join(os.getcwd(), "src"),
    ]
    for anchor in anchors:
        for filename in filenames:
            candidates.append(os.path.join(anchor, filename))
    # HH_260407: Relative lookup around package share / config path.
    for anchor in (map_share, os.path.dirname(map_info_file)):
        cur = os.path.abspath(anchor)
        for _ in range(8):
            for filename in filenames:
                candidates.append(os.path.join(cur, filename))
                candidates.append(os.path.join(cur, "src", filename))
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent

    discovered = _first_existing_path(candidates)
    return discovered if discovered else configured


def generate_launch_description():
    map_share = get_package_share_directory("camrod_map")

    default_map_info = os.path.join(map_share, "config", "map_info.yaml")
    defaults = load_defaults(default_map_info)
    defaults["map_path"] = discover_map_path(
        map_share, default_map_info, defaults["map_path"], defaults["map_profile"]
    )

    lanelet2_map_launch  = os.path.join(map_share, "launch", "lanelet2_map.launch.py")
    cost_grid_launch     = os.path.join(map_share, "launch", "cost_grid.launch.py")
    visualization_launch = os.path.join(map_share, "launch", "visualization.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("map_info_file",        default_value=default_map_info),
        DeclareLaunchArgument("map_param_file",        default_value=LaunchConfiguration("map_info_file")),
        DeclareLaunchArgument("lanelet_cost_grid_param_file", default_value=pkg_share("camrod_map", os.path.join("config", "lanelet_cost_grid.yaml"))),
        DeclareLaunchArgument("map_visualization_param_file", default_value=pkg_share("camrod_map", os.path.join("config", "map_visualization.yaml"))),
        DeclareLaunchArgument("map_path",             default_value=defaults["map_path"]),
        DeclareLaunchArgument("origin_lat",           default_value=defaults["origin_lat"]),
        DeclareLaunchArgument("origin_lon",           default_value=defaults["origin_lon"]),
        DeclareLaunchArgument("origin_alt",           default_value=defaults["origin_alt"]),
        DeclareLaunchArgument("enable_nav2_inflation_debug_marker", default_value="false"),
        DeclareLaunchArgument("enable_inflation_markers",           default_value="false"),
        DeclareLaunchArgument("enable_map_cost_markers",            default_value="false"),
        DeclareLaunchArgument("enable_cost_field",                  default_value="false"),
        DeclareLaunchArgument("enable_cost_grids",                  default_value="true"),
        DeclareLaunchArgument("module_namespace",                   default_value="map"),
        # HH_260527: Removed unused map-origin launch args.
        # (enable_module_validator, system_namespace).

        _inc(lanelet2_map_launch,
             "map_param_file", "map_path", "origin_lat", "origin_lon", "origin_alt",
             "module_namespace"),

        _inc(cost_grid_launch,
             "module_namespace", "enable_cost_grids", "map_info_file",
             "lanelet_cost_grid_param_file", "map_path",
             "origin_lat", "origin_lon", "origin_alt"),

        _inc(visualization_launch,
             "module_namespace", "map_info_file", "map_visualization_param_file",
             "enable_nav2_inflation_debug_marker", "enable_inflation_markers",
             "enable_map_cost_markers", "enable_cost_field",
             "map_path", "origin_lat", "origin_lon", "origin_alt"),
    ])
