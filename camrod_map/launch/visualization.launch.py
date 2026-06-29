#!/usr/bin/env python3

import os
import re
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


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


def _load_map_params(map_info_file: str) -> dict:
    if not map_info_file:
        return {}
    try:
        with open(map_info_file, "r", encoding="utf-8") as f:
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


def _resolve_map_path(map_share: str, map_info_file: str, requested_map_path: str) -> str:
    params = _load_map_params(map_info_file)
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
            if map_info_file:
                # HH_260629: Standalone visualization launch resolves
                # map_info-relative OSM paths.
                candidates.append(os.path.abspath(
                    os.path.join(os.path.dirname(map_info_file), configured)))

    filenames = _map_filename_candidates(configured, map_profile)
    anchors = [
        os.path.join(os.path.expanduser("~"), "camrod_ws", "src"),
        os.getcwd(),
        os.path.join(os.getcwd(), "src"),
        map_share,
        os.path.dirname(map_info_file) if map_info_file else "",
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


def _launch_nodes(context, *_args, **_kwargs):
    map_info_file = LaunchConfiguration("map_info_file").perform(context).strip()
    lanelet_cost_field_params = [
        map_info_file,
        LaunchConfiguration("map_visualization_param_file"),
        {
            # Keep consistent with lanelet2_map_loader and lanelet_cost_grid.
            "projector_type": "local_cartesian",
        },
    ]

    cost_field_overrides = {}
    map_path = LaunchConfiguration("map_path").perform(context).strip()
    resolved_map_path = _resolve_map_path(
        get_package_share_directory("camrod_map"), map_info_file, map_path)
    if resolved_map_path:
        cost_field_overrides["map_path"] = resolved_map_path

    origin_lat = _optional_float(LaunchConfiguration("origin_lat").perform(context))
    origin_lon = _optional_float(LaunchConfiguration("origin_lon").perform(context))
    origin_alt = _optional_float(LaunchConfiguration("origin_alt").perform(context))
    if origin_lat is not None:
        cost_field_overrides["offset_lat"] = origin_lat
    if origin_lon is not None:
        cost_field_overrides["offset_lon"] = origin_lon
    if origin_alt is not None:
        cost_field_overrides["offset_alt"] = origin_alt
    if cost_field_overrides:
        lanelet_cost_field_params.append(cost_field_overrides)

    return [
        Node(
            package="camrod_map",
            executable="cost_field_marker_node",
            name="nav2_costmap_debug_marker",
            namespace=LaunchConfiguration("module_namespace"),
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_nav2_inflation_debug_marker")),
            parameters=[LaunchConfiguration("map_visualization_param_file")],
        ),

        Node(
            package="camrod_map",
            executable="multi_cost_field_marker_node",
            name="cost_grid_multi_marker",
            namespace=LaunchConfiguration("module_namespace"),
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_map_cost_markers")),
            parameters=[LaunchConfiguration("map_visualization_param_file")],
        ),

        Node(
            package="camrod_map",
            executable="marker_array_aggregator_node",
            name="cost_grid_marker_aggregator",
            namespace=LaunchConfiguration("module_namespace"),
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_inflation_markers")),
            parameters=[{
                "output_topic": "/map/cost_grid/inflation_markers",
                "input_topics": [
                    "/map/cost_grid/lanelet_markers",
                    "/map/cost_grid/lidar_markers",
                    "/map/cost_grid/radar_markers",
                    "/planning/cost_grid/global_path_markers",
                    "/planning/cost_grid/local_path_markers",
                ],
                # HH_260618: Aggregated inflation markers are debug-only; avoid
                # a 20 Hz marker republish loop on resource-limited targets.
                "republish_period_s": 0.20,
                "min_publish_period_s": 0.10,
                "stale_timeout_s": 1.2,
                "stale_timeout_topics": [
                    "/map/cost_grid/lidar_markers",
                    "/map/cost_grid/radar_markers",
                ],
            }],
        ),

        Node(
            package="camrod_map",
            executable="cost_field_node",
            name="lanelet_cost_field_visualizer",
            namespace=LaunchConfiguration("module_namespace"),
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_cost_field")),
            parameters=lanelet_cost_field_params,
        ),
    ]


def generate_launch_description():
    module_namespace_arg = DeclareLaunchArgument(
        "module_namespace",
        default_value="map",
        description="Namespace for map module nodes",
    )
    map_info_file_arg = DeclareLaunchArgument(
        "map_info_file",
        default_value=pkg_share("camrod_map", os.path.join("config", "map_info.yaml")),
        description="Shared map info YAML (map_path/origin source of truth)",
    )
    map_viz_param_arg = DeclareLaunchArgument(
        "map_visualization_param_file",
        default_value=pkg_share("camrod_map", os.path.join("config", "map_visualization.yaml")),
        description="Map visualization parameters",
    )
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value="",
        description="Lanelet2 map path override for lanelet_cost_field",
    )
    origin_lat_arg = DeclareLaunchArgument(
        "origin_lat",
        default_value="",
        description="Map origin latitude override",
    )
    origin_lon_arg = DeclareLaunchArgument(
        "origin_lon",
        default_value="",
        description="Map origin longitude override",
    )
    origin_alt_arg = DeclareLaunchArgument(
        "origin_alt",
        default_value="",
        description="Map origin altitude override",
    )
    enable_nav2_inflation_debug_marker_arg = DeclareLaunchArgument(
        "enable_nav2_inflation_debug_marker",
        default_value="false",
        description="Enable debug marker from /planning/global_costmap/costmap",
    )
    enable_inflation_markers_arg = DeclareLaunchArgument(
        "enable_inflation_markers",
        default_value="false",
        description="Enable merged contributor inflation markers",
    )
    enable_map_cost_markers_arg = DeclareLaunchArgument(
        "enable_map_cost_markers",
        default_value="false",
        description="Enable RViz-only lanelet/lidar/radar cost marker publishers",
    )
    enable_cost_field_arg = DeclareLaunchArgument(
        "enable_cost_field",
        default_value="false",
        description="Enable lanelet cost field node",
    )

    return LaunchDescription([
        module_namespace_arg,
        map_info_file_arg,
        map_viz_param_arg,
        map_path_arg,
        origin_lat_arg,
        origin_lon_arg,
        origin_alt_arg,
        enable_nav2_inflation_debug_marker_arg,
        enable_inflation_markers_arg,
        enable_map_cost_markers_arg,
        enable_cost_field_arg,
        OpaqueFunction(function=_launch_nodes),
    ])
