#!/usr/bin/env python3

import os
import re
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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


def _extract_map_ros_params(map_info_cfg: dict) -> dict:
    if not isinstance(map_info_cfg, dict):
        return {}
    wildcard = map_info_cfg.get("/**")
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


def _load_map_params(map_param_file: str) -> dict:
    if not map_param_file:
        return {}
    try:
        with open(map_param_file, "r", encoding="utf-8") as f:
            return _extract_map_ros_params(yaml.safe_load(f) or {})
    except Exception:
        return {}


def _first_existing_path(candidates: list[str]) -> str:
    seen = set()
    for candidate in candidates:
        if not candidate:
            continue
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


def _resolve_map_path(map_share: str, map_param_file: str, requested_map_path: str) -> str:
    params = _load_map_params(map_param_file)
    configured = str(requested_map_path or "").strip()
    if not configured:
        configured = str(params.get("map_path", "")).strip()
    map_profile = str(params.get("map_profile", params.get("profile", ""))).strip()

    candidates = []
    if configured:
        if os.path.isabs(configured):
            candidates.append(configured)
        else:
            candidates.append(os.path.abspath(configured))
            if map_param_file:
                # HH_260629: Standalone launch resolves map_info-relative OSM paths
                # before the C++ node checks filesystem existence.
                candidates.append(os.path.abspath(
                    os.path.join(os.path.dirname(map_param_file), configured)))

    filenames = _map_filename_candidates(configured, map_profile)
    anchors = [
        os.path.join(os.path.expanduser("~"), "camrod_ws", "src"),
        os.getcwd(),
        os.path.join(os.getcwd(), "src"),
        map_share,
        os.path.dirname(map_param_file) if map_param_file else "",
    ]
    for anchor in anchors:
        cur = os.path.abspath(anchor) if anchor else ""
        for _ in range(8):
            if not cur:
                break
            for filename in filenames:
                candidates.append(os.path.join(cur, filename))
                candidates.append(os.path.join(cur, "src", filename))
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent

    resolved = _first_existing_path(candidates)
    return resolved if resolved else configured


def _launch_node(context, *_args, **_kwargs):
    overrides = {}
    map_share = get_package_share_directory("camrod_map")
    map_param_file = LaunchConfiguration("map_param_file").perform(context).strip()

    map_path = LaunchConfiguration("map_path").perform(context).strip()
    resolved_map_path = _resolve_map_path(map_share, map_param_file, map_path)
    if resolved_map_path:
        overrides["map_path"] = resolved_map_path

    origin_lat = _optional_float(LaunchConfiguration("origin_lat").perform(context))
    origin_lon = _optional_float(LaunchConfiguration("origin_lon").perform(context))
    origin_alt = _optional_float(LaunchConfiguration("origin_alt").perform(context))
    if origin_lat is not None:
        overrides["offset_lat"] = origin_lat
    if origin_lon is not None:
        overrides["offset_lon"] = origin_lon
    if origin_alt is not None:
        overrides["offset_alt"] = origin_alt

    params = [map_param_file]
    if overrides:
        params.append(overrides)

    return [
        Node(
            package="camrod_map",
            executable="lanelet2_map_node",
            name="lanelet_map_provider",
            namespace=LaunchConfiguration("module_namespace"),
            parameters=params,
            output="screen",
        )
    ]


def generate_launch_description():
    config_dir = FindPackageShare("camrod_map").find("camrod_map")
    default_map_info = os.path.join(config_dir, "config", "map_info.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_param_file",
            default_value=default_map_info,
            description="Map info YAML file for lanelet2_map_node",
        ),
        DeclareLaunchArgument(
            "map_path",
            default_value="",
            description="Lanelet2 map path override (empty: keep map_param_file value)",
        ),
        DeclareLaunchArgument(
            "origin_lat",
            default_value="",
            description="Map origin latitude override",
        ),
        DeclareLaunchArgument(
            "origin_lon",
            default_value="",
            description="Map origin longitude override",
        ),
        DeclareLaunchArgument(
            "origin_alt",
            default_value="",
            description="Map origin altitude override",
        ),
        DeclareLaunchArgument(
            "module_namespace",
            default_value="map",
            description="Namespace for map module nodes",
        ),
        OpaqueFunction(function=_launch_node),
    ])
