#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def extract_map_ros_params(map_info_cfg: dict) -> dict:
    if not isinstance(map_info_cfg, dict):
        return {}
    wildcard = map_info_cfg.get("/**", None)
    if isinstance(wildcard, dict):
        params = wildcard.get("ros__parameters")
        if isinstance(params, dict):
            return params
    for key in (
        "/map/lanelet2_map",
        "map/lanelet2_map",
        "lanelet2_map",
        "/lanelet2_map",
    ):
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
    defaults = {
        "map_path": "",
        "origin_lat": "",
        "origin_lon": "",
        "origin_alt": "",
    }
    try:
        with open(default_map_info, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        params = extract_map_ros_params(data)
        defaults["map_path"] = str(params.get("map_path", "")).strip()
        defaults["origin_lat"] = str(params.get("offset_lat", "")).strip()
        defaults["origin_lon"] = str(params.get("offset_lon", "")).strip()
        defaults["origin_alt"] = str(params.get("offset_alt", "")).strip()
    except Exception:
        pass
    return defaults


def generate_launch_description():
    map_share = get_package_share_directory("camrod_map")

    default_map_info = os.path.join(map_share, "config", "map_info.yaml")
    defaults = load_defaults(default_map_info)

    lanelet2_map_launch = os.path.join(map_share, "launch", "lanelet2_map.launch.py")
    cost_grid_launch = os.path.join(map_share, "launch", "cost_grid.launch.py")
    visualization_launch = os.path.join(map_share, "launch", "visualization.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_info_file",
            default_value=default_map_info,
            description="Shared map info YAML (single source of map/origin defaults)",
        ),
        DeclareLaunchArgument(
            "map_param_file",
            default_value=LaunchConfiguration("map_info_file"),
            description="Alias for map_info_file (lanelet2_map.launch.py compatibility)",
        ),
        DeclareLaunchArgument(
            "lanelet_cost_grid_param_file",
            default_value=pkg_share("camrod_map", os.path.join("config", "lanelet_cost_grid.yaml")),
            description="Lanelet cost-grid node parameters",
        ),
        DeclareLaunchArgument(
            "map_visualization_param_file",
            default_value=pkg_share("camrod_map", os.path.join("config", "map_visualization.yaml")),
            description="Map visualization parameters",
        ),
        DeclareLaunchArgument(
            "map_path",
            default_value=defaults["map_path"],
            description="Lanelet2 map path override (empty: use map_info.yaml value)",
        ),
        DeclareLaunchArgument(
            "origin_lat",
            default_value=defaults["origin_lat"],
            description="Map origin latitude override (empty: use map_info.yaml value)",
        ),
        DeclareLaunchArgument(
            "origin_lon",
            default_value=defaults["origin_lon"],
            description="Map origin longitude override (empty: use map_info.yaml value)",
        ),
        DeclareLaunchArgument(
            "origin_alt",
            default_value=defaults["origin_alt"],
            description="Map origin altitude override (empty: use map_info.yaml value)",
        ),
        DeclareLaunchArgument(
            "enable_nav2_inflation_debug_marker",
            default_value="false",
            description="Enable debug marker stream from /planning/global_costmap/costmap",
        ),
        DeclareLaunchArgument(
            "enable_inflation_markers",
            default_value="false",
            description="Enable contributor-merged inflation markers",
        ),
        DeclareLaunchArgument(
            "enable_map_cost_markers",
            default_value="true",
            description="Enable lanelet/lidar/radar marker publishers",
        ),
        DeclareLaunchArgument(
            "enable_cost_field",
            default_value="false",
            description="Enable lanelet cost field node",
        ),
        DeclareLaunchArgument(
            "enable_cost_grids",
            default_value="true",
            description="Enable lanelet/planning base cost-grid publishers",
        ),
        DeclareLaunchArgument(
            "enable_module_validator",
            default_value="true",
            description="Reserved for bringup compatibility",
        ),
        DeclareLaunchArgument(
            "module_namespace",
            default_value="map",
            description="Namespace for map module nodes",
        ),
        DeclareLaunchArgument(
            "system_namespace",
            default_value="system",
            description="Reserved for bringup compatibility",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lanelet2_map_launch),
            launch_arguments={
                "map_param_file": LaunchConfiguration("map_param_file"),
                "map_path": LaunchConfiguration("map_path"),
                "origin_lat": LaunchConfiguration("origin_lat"),
                "origin_lon": LaunchConfiguration("origin_lon"),
                "origin_alt": LaunchConfiguration("origin_alt"),
                "module_namespace": LaunchConfiguration("module_namespace"),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cost_grid_launch),
            launch_arguments={
                "module_namespace": LaunchConfiguration("module_namespace"),
                "enable_cost_grids": LaunchConfiguration("enable_cost_grids"),
                "map_info_file": LaunchConfiguration("map_info_file"),
                "lanelet_cost_grid_param_file": LaunchConfiguration("lanelet_cost_grid_param_file"),
                "map_path": LaunchConfiguration("map_path"),
                "origin_lat": LaunchConfiguration("origin_lat"),
                "origin_lon": LaunchConfiguration("origin_lon"),
                "origin_alt": LaunchConfiguration("origin_alt"),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(visualization_launch),
            launch_arguments={
                "module_namespace": LaunchConfiguration("module_namespace"),
                "map_info_file": LaunchConfiguration("map_info_file"),
                "map_visualization_param_file": LaunchConfiguration("map_visualization_param_file"),
                "enable_nav2_inflation_debug_marker": LaunchConfiguration("enable_nav2_inflation_debug_marker"),
                "enable_inflation_markers": LaunchConfiguration("enable_inflation_markers"),
                "enable_map_cost_markers": LaunchConfiguration("enable_map_cost_markers"),
                "enable_cost_field": LaunchConfiguration("enable_cost_field"),
                "map_path": LaunchConfiguration("map_path"),
                "origin_lat": LaunchConfiguration("origin_lat"),
                "origin_lon": LaunchConfiguration("origin_lon"),
                "origin_alt": LaunchConfiguration("origin_alt"),
            }.items(),
        ),
    ])
