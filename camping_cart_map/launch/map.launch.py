import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def pkg_share(pkg, rel):
    return os.path.join(get_package_share_directory(pkg), rel)


def generate_launch_description():
    # HH_260128 Load defaults from map_info.yaml so map_path/offset are populated without extra args.
    default_map_info = pkg_share('camping_cart_bringup', os.path.join('config', 'map', 'map_info.yaml'))
    default_map_path = ''
    default_lat = '0.0'
    default_lon = '0.0'
    default_alt = '0.0'
    try:
        with open(default_map_info, 'r') as f:
            data = yaml.safe_load(f) or {}
        params = data.get('/map/lanelet2_map', {}).get('ros__parameters', {})
        default_map_path = str(params.get('map_path', default_map_path))
        default_lat = str(params.get('offset_lat', default_lat))
        default_lon = str(params.get('offset_lon', default_lon))
        default_alt = str(params.get('offset_alt', default_alt))
    except Exception:
        pass

    map_param_arg = DeclareLaunchArgument(
        'map_param_file',
        default_value=pkg_share('camping_cart_bringup', os.path.join('config', 'map', 'map_info.yaml')),
        description='Map info YAML for lanelet2_map_node',
    )
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=default_map_path,
        description='Lanelet2 map path override (empty uses map_info.yaml)',
    )
    origin_lat_arg = DeclareLaunchArgument(
        'origin_lat',
        default_value=default_lat,
        description='Map origin latitude',
    )
    origin_lon_arg = DeclareLaunchArgument(
        'origin_lon',
        default_value=default_lon,
        description='Map origin longitude',
    )
    origin_alt_arg = DeclareLaunchArgument(
        'origin_alt',
        default_value=default_alt,
        description='Map origin altitude',
    )
    map_viz_param_arg = DeclareLaunchArgument(
        'map_visualization_param_file',
        default_value=pkg_share('camping_cart_bringup', os.path.join('config', 'map', 'map_visualization.yaml')),
        description='Map visualization parameters (cost markers/field)',
    )

    map_param = LaunchConfiguration('map_param_file')
    map_path = LaunchConfiguration('map_path')
    origin_lat = LaunchConfiguration('origin_lat')
    origin_lon = LaunchConfiguration('origin_lon')
    origin_alt = LaunchConfiguration('origin_alt')
    map_viz_param = LaunchConfiguration('map_visualization_param_file')

    # HH_260121 Map server (Lanelet2) loads OSM and static TF world->map.
    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_share('camping_cart_map', 'launch/lanelet2_map.launch.py')),
        launch_arguments={
            'map_param_file': map_param,
        }.items(),
    )

    # HH_260121 Lanelet cost grid for Nav2 custom cost layer (/map/cost_grid/lanelet).
    cost_grid_param = pkg_share('camping_cart_bringup', os.path.join('config', 'map', 'lanelet_cost_grid.yaml'))
    lanelet_cost_grid = Node(
        package='camping_cart_map',
        executable='lanelet_cost_grid_node',
        name='cost_grid_map',
        namespace='map',
        output='screen',
        parameters=[
            cost_grid_param,
            {
                'map_path': map_path,
                'offset_lat': origin_lat,
                'offset_lon': origin_lon,
                'offset_alt': origin_alt,
                'map_frame_id': 'map',
                # 2026-02-23: Align base grid pose reference with planning/localization start frame.
                'pose_topic': '/localization/pose',
                'output_topic': '/map/cost_grid/lanelet',
            },
        ],
    )

    # 2026-02-24: Move map cost visualizers under map module ownership.
    inflation_cost_marker = Node(
        package='camping_cart_map',
        executable='cost_field_marker_node',
        name='inflation_cost_marker',
        namespace='map',
        output='screen',
        parameters=[map_viz_param],
    )

    lanelet_cost_field = Node(
        package='camping_cart_map',
        executable='cost_field_node',
        name='lanelet_cost_field',
        namespace='map',
        output='screen',
        parameters=[
            map_viz_param,
            {
                'map_path': map_path,
                'offset_lat': origin_lat,
                'offset_lon': origin_lon,
                'offset_alt': origin_alt,
            },
        ],
    )

    return LaunchDescription([
        map_param_arg,
        map_path_arg,
        origin_lat_arg,
        origin_lon_arg,
        origin_alt_arg,
        map_viz_param_arg,
        map_launch,
        lanelet_cost_grid,
        inflation_cost_marker,
        lanelet_cost_field,
    ])
