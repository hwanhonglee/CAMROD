#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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


# Builds lanelet2_map_node with launch-time map/origin overrides.
def _launch_node(context, *_args, **_kwargs):
    overrides = {}

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

    params = [LaunchConfiguration("map_param_file")]
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


# Declares lanelet2 map launch arguments and dispatches node construction.
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
