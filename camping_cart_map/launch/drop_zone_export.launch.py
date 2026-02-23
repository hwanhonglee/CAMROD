import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 2026-02-02 10:32: Use centralized bringup config (synced from package config).
    bringup_share = get_package_share_directory("camping_cart_bringup")
    default_map_info = os.path.join(bringup_share, "config", "map", "map_info.yaml")

    map_info_arg = DeclareLaunchArgument(
        "map_info_file",
        default_value=default_map_info,
        description="Map info YAML (used for defaults)",
    )
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value="",
        description="Lanelet2 map path (optional; defaults to map_info.yaml)",
    )
    origin_lat_arg = DeclareLaunchArgument(
        "origin_lat",
        default_value="",
        description="Map origin latitude (optional; defaults to map_info.yaml)",
    )
    origin_lon_arg = DeclareLaunchArgument(
        "origin_lon",
        default_value="",
        description="Map origin longitude (optional; defaults to map_info.yaml)",
    )
    origin_alt_arg = DeclareLaunchArgument(
        "origin_alt",
        default_value="",
        description="Map origin altitude (optional; defaults to map_info.yaml)",
    )
    output_yaml_arg = DeclareLaunchArgument(
        "output_yaml_path",
        default_value=os.path.join(
            "/home/hong/cart_test_ws/src/camping_cart_bringup/config/localization",
            "drop_zones.yaml",
        ),
        description="Output YAML path for drop zones",
    )

    def _to_float(value, fallback):
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(fallback)

    def _launch_nodes(context, *_args, **_kwargs):
        map_info_path = LaunchConfiguration("map_info_file").perform(context)
        map_info = {}
        if os.path.exists(map_info_path):
            with open(map_info_path, "r") as f:
                map_info = yaml.safe_load(f) or {}
        map_cfg = map_info.get("/map/lanelet2_map", {}).get("ros__parameters", {})

        map_path = LaunchConfiguration("map_path").perform(context) or map_cfg.get(
            "map_path", "/home/hong/cart_test_ws/src/new_lanelet2_maps.osm"
        )
        origin_lat = _to_float(
            LaunchConfiguration("origin_lat").perform(context), map_cfg.get("offset_lat", 0.0)
        )
        origin_lon = _to_float(
            LaunchConfiguration("origin_lon").perform(context), map_cfg.get("offset_lon", 0.0)
        )
        origin_alt = _to_float(
            LaunchConfiguration("origin_alt").perform(context), map_cfg.get("offset_alt", 0.0)
        )

        return [
            Node(
                package="camping_cart_map",
                executable="drop_zone_exporter_node",
                name="drop_zone_exporter",
                output="screen",
                parameters=[
                    {
                        "map_path": map_path,
                        "origin_lat": origin_lat,
                        "origin_lon": origin_lon,
                        "origin_alt": origin_alt,
                        "output_yaml_path": LaunchConfiguration("output_yaml_path").perform(
                            context
                        ),
                    }
                ],
            )
        ]

    return LaunchDescription(
        [
            map_info_arg,
            map_path_arg,
            origin_lat_arg,
            origin_lon_arg,
            origin_alt_arg,
            output_yaml_arg,
            OpaqueFunction(function=_launch_nodes),
        ]
    )
