from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# HH_260109 Launch visualization-only nodes with /visualizer-prefixed topics.
def _launch_setup(context, *args, **kwargs):
    visualizer_params = LaunchConfiguration("visualizer_params").perform(context)
    robot_params = LaunchConfiguration("robot_params").perform(context)
    map_info = Path(LaunchConfiguration("map_info").perform(context))

    map_cfg = {}
    if map_info.exists():
        map_cfg = yaml.safe_load(map_info.read_text()) or {}
    map_params = map_cfg.get("/map/lanelet2_map", {}).get("ros__parameters", {})
    map_path = map_params.get("map_path", "")
    offset_lat = float(map_params.get("offset_lat", 0.0))
    offset_lon = float(map_params.get("offset_lon", 0.0))
    offset_alt = float(map_params.get("offset_alt", 0.0))

    # HH_260112 Namespace visualizer nodes under /visualizer with short names.
    robot_viz = Node(
        package="camping_cart_visualizer",
        executable="robot_visualization_node",
        name="robot_visualization",
        namespace="visualizer",
        output="screen",
        parameters=[robot_params, visualizer_params],
    )

    cost_marker = Node(
        package="camping_cart_visualizer",
        executable="cost_field_marker_node",
        name="cost_field_marker",
        namespace="visualizer",
        output="screen",
        parameters=[visualizer_params],
    )

    cost_field = Node(
        package="camping_cart_visualizer",
        executable="cost_field_node",
        name="cost_field",
        namespace="visualizer",
        output="screen",
        parameters=[visualizer_params, {
            # HH_260109 Pass map info for cost field visualization.
            "map_path": map_path,
            "offset_lat": offset_lat,
            "offset_lon": offset_lon,
            "offset_alt": offset_alt,
        }],
    )

    return [robot_viz, cost_marker, cost_field]


def generate_launch_description():
    # 2026-02-02 10:32: Use centralized bringup config (synced from package config).
    bringup_share = Path(get_package_share_directory("camping_cart_bringup"))
    default_visualizer = bringup_share / "config" / "visualizer" / "visualizer_params.yaml"
    default_robot_params = bringup_share / "config" / "sensor_kit" / "robot_params.yaml"
    default_map_info = bringup_share / "config" / "map" / "map_info.yaml"

    return LaunchDescription([
        DeclareLaunchArgument(
            "visualizer_params",
            default_value=str(default_visualizer),
            description="Visualizer parameter file",
        ),
        DeclareLaunchArgument(
            "robot_params",
            default_value=str(default_robot_params),
            description="Robot parameter file for visualization",
        ),
        DeclareLaunchArgument(
            "map_info",
            default_value=str(default_map_info),
            description="Map info YAML for visualization",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
