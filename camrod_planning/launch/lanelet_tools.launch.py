import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


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
    for val in map_info_cfg.values():
        if isinstance(val, dict):
            params = val.get('ros__parameters')
            if isinstance(params, dict):
                return params
    return {}


def generate_launch_description():
    map_info_path = pkg_share('camrod_map', os.path.join('config', 'map_info.yaml'))
    map_path_default = ''
    origin_lat_default = '0.0'
    origin_lon_default = '0.0'
    origin_alt_default = '0.0'
    try:
        with open(map_info_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        params = extract_map_ros_params(data)
        map_path_default = str(params.get('map_path', map_path_default))
        origin_lat_default = str(params.get('offset_lat', origin_lat_default))
        origin_lon_default = str(params.get('offset_lon', origin_lon_default))
        origin_alt_default = str(params.get('offset_alt', origin_alt_default))
    except Exception:
        pass
    # HH_260409: Standalone fallback for empty map_info map_path.
    if not str(map_path_default).strip():
        for candidate in (
            os.path.join(os.path.expanduser('~'), 'camrod_ws', 'src', 'lanelet2_maps.osm'),
            os.path.join(os.getcwd(), 'lanelet2_maps.osm'),
            os.path.join(os.getcwd(), 'src', 'lanelet2_maps.osm'),
        ):
            if os.path.isfile(candidate):
                map_path_default = os.path.abspath(candidate)
                break

    return LaunchDescription([
        DeclareLaunchArgument('module_namespace', default_value='planning'),
        # HH_260409: Use map_info defaults even when this launch runs standalone.
        # This prevents goal_snapper/centerline_snapper startup failures from empty map_path.
        DeclareLaunchArgument('map_path', default_value=map_path_default),
        DeclareLaunchArgument('origin_lat', default_value=origin_lat_default),
        DeclareLaunchArgument('origin_lon', default_value=origin_lon_default),
        DeclareLaunchArgument('origin_alt', default_value=origin_alt_default),
        DeclareLaunchArgument('centerline_input_pose_topic', default_value='/localization/pose'),
        DeclareLaunchArgument(
            'goal_snapper_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'goal_snapper.yaml')),
        ),
        DeclareLaunchArgument(
            'centerline_snapper_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'centerline_snapper.yaml')),
        ),

        Node(
            package='camrod_planning',
            executable='goal_snapper_node',
            name='goal_snapper',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            # HH_260702 - Goal snapping is required before Nav2 consumes goals;
            # respawn restores the input path after transient map/TF races.
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                LaunchConfiguration('goal_snapper_param_file'),
                {
                    'map_path': LaunchConfiguration('map_path'),
                    'offset_lat': LaunchConfiguration('origin_lat'),
                    'offset_lon': LaunchConfiguration('origin_lon'),
                    'offset_alt': LaunchConfiguration('origin_alt'),
                    # HH_260727 - Split regulated mission goals from manual RViz goals.
                    'input_goal_topic': '/planning/site_goal_pose_ros',
                    'manual_input_goal_topic': '/goal_pose',
                    'output_goal_topic': '/planning/goal_pose_snapped',
                    'output_goal_topic_ros': '/planning/goal_pose_snapped_ros',
                    'output_goal_source_topic': '/planning/goal_source',
                    'goal_source_settle_delay_s': 0.12,
                    'routable_lanelet_only': True,
                },
            ],
        ),

        Node(
            package='camrod_planning',
            executable='centerline_snapper_node',
            name='centerline_snapper',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            # HH_260702 - Centerline pose supports lanelet-aligned localization
            # and planning diagnostics; keep it alive after transient exits.
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                LaunchConfiguration('centerline_snapper_param_file'),
                {
                    'map_path': LaunchConfiguration('map_path'),
                    'offset_lat': LaunchConfiguration('origin_lat'),
                    'offset_lon': LaunchConfiguration('origin_lon'),
                    'offset_alt': LaunchConfiguration('origin_alt'),
                    'input_pose_topic': LaunchConfiguration('centerline_input_pose_topic'),
                    'output_pose_topic': '/planning/lanelet_pose',
                    'output_pose_topic_ros': '/planning/lanelet_pose_ros',
                },
            ],
        ),
    ])
