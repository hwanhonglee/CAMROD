#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Resolves a package-relative file path via package share directory.
def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


# Parses optional numeric launch arguments and returns None on empty/invalid input.
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


# Builds lanelet cost-grid node list with map/origin overrides from launch arguments.
def _launch_nodes(context, *_args, **_kwargs):
    params = [
        LaunchConfiguration("map_info_file"),
        LaunchConfiguration("lanelet_cost_grid_param_file"),
    ]
    overrides = {
        # Keep projector consistent across map/localization/planning helpers.
        "projector_type": "local_cartesian",
    }

    map_path = LaunchConfiguration("map_path").perform(context).strip()
    if map_path:
        overrides["map_path"] = map_path

    origin_lat = _optional_float(LaunchConfiguration("origin_lat").perform(context))
    origin_lon = _optional_float(LaunchConfiguration("origin_lon").perform(context))
    origin_alt = _optional_float(LaunchConfiguration("origin_alt").perform(context))
    if origin_lat is not None:
        overrides["offset_lat"] = origin_lat
    if origin_lon is not None:
        overrides["offset_lon"] = origin_lon
    if origin_alt is not None:
        overrides["offset_alt"] = origin_alt

    if overrides:
        params.append(overrides)

    return [
        Node(
            package="camrod_map",
            executable="lanelet_cost_grid_node",
            name="lanelet_boundary_cost_grid",
            namespace=LaunchConfiguration("module_namespace"),
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_cost_grids")),
            parameters=params,
        )
    ]


# Declares cost-grid launch arguments and defers node creation to runtime context.
def generate_launch_description():
    module_namespace_arg = DeclareLaunchArgument(
        "module_namespace",
        default_value="map",
        description="Namespace for map module nodes",
    )
    enable_cost_grids_arg = DeclareLaunchArgument(
        "enable_cost_grids",
        default_value="true",
        description="Enable lanelet/planning base cost-grid nodes",
    )
    map_info_file_arg = DeclareLaunchArgument(
        "map_info_file",
        default_value=pkg_share("camrod_map", os.path.join("config", "map_info.yaml")),
        description="Shared map info YAML (map_path/origin source of truth)",
    )
    lanelet_cost_grid_param_arg = DeclareLaunchArgument(
        "lanelet_cost_grid_param_file",
        default_value=pkg_share("camrod_map", os.path.join("config", "lanelet_cost_grid.yaml")),
        description="Lanelet cost-grid parameters",
    )
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value="",
        description="Lanelet2 map path override (empty: use map_info.yaml value)",
    )
    origin_lat_arg = DeclareLaunchArgument(
        "origin_lat",
        default_value="",
        description="Map origin latitude override (empty: use map_info.yaml value)",
    )
    origin_lon_arg = DeclareLaunchArgument(
        "origin_lon",
        default_value="",
        description="Map origin longitude override (empty: use map_info.yaml value)",
    )
    origin_alt_arg = DeclareLaunchArgument(
        "origin_alt",
        default_value="",
        description="Map origin altitude override (empty: use map_info.yaml value)",
    )

    return LaunchDescription([
        module_namespace_arg,
        enable_cost_grids_arg,
        map_info_file_arg,
        lanelet_cost_grid_param_arg,
        map_path_arg,
        origin_lat_arg,
        origin_lon_arg,
        origin_alt_arg,
        OpaqueFunction(function=_launch_nodes),
    ])
