import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 2026-02-02 10:32: Use centralized bringup config (synced from package config).
    cost_grid_param = os.path.join(
        get_package_share_directory('camping_cart_bringup'),
        'config', 'planning', 'path_cost_grids.yaml')
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='/home/hong/cart_test_ws/src/lanelet2_maps.osm',
        description='Lanelet2 map path for path-cost-grid helpers',
    )
    origin_lat_arg = DeclareLaunchArgument('origin_lat', default_value='36.8436194')
    origin_lon_arg = DeclareLaunchArgument('origin_lon', default_value='128.0925490')
    origin_alt_arg = DeclareLaunchArgument('origin_alt', default_value='0.0')
    map_path = LaunchConfiguration('map_path')
    origin_lat = LaunchConfiguration('origin_lat')
    origin_lon = LaunchConfiguration('origin_lon')
    origin_alt = LaunchConfiguration('origin_alt')

    # HH_260123 Path-focused grid for global costmap (Nav2 planner).
    lanelet_cost_grid_global = Node(
        package='camping_cart_map',
        executable='lanelet_cost_grid_node',
        name='lanelet_cost_grid_global_path',
        namespace='planning',
        output='screen',
        parameters=[
            cost_grid_param,
            {
                'map_path': map_path,
                'offset_lat': origin_lat,
                'offset_lon': origin_lon,
                'offset_alt': origin_alt,
            },
        ],
    )

    # HH_260123 Path-focused grid for local costmap (Nav2 controller local plan).
    lanelet_cost_grid_local = Node(
        package='camping_cart_map',
        executable='lanelet_cost_grid_node',
        name='lanelet_cost_grid_local_path',
        namespace='planning',
        output='screen',
        parameters=[
            cost_grid_param,
            {
                'map_path': map_path,
                'offset_lat': origin_lat,
                'offset_lon': origin_lon,
                'offset_alt': origin_alt,
            },
        ],
    )

    # 2026-02-25: Marker views for path cost grids.
    # Keep OccupancyGrid for planner math, use marker topics for lanelet-shaped RViz display.
    global_path_cost_marker = Node(
        package='camping_cart_map',
        executable='cost_field_marker_node',
        name='global_path_cost_marker',
        namespace='planning',
        output='screen',
        parameters=[{
            'grid_topic': '/planning/cost_grid/global_path',
            'marker_topic': '/planning/cost_grid/global_path_markers',
            'min_value': 0,
            'max_value': 100,
            # HH_260311-00:00 Increase opacity so high-cost (red) segments are visible.
            'alpha': 0.60,
            'z_offset': 0.03,
            # HH_260305-00:00 Keep cells inside lane boundaries while preserving continuity.
            'cell_scale_ratio': 0.85,
            'palette': 'safety',
            'show_unknown': False,
            # HH_260311-00:00 Keep last valid marker set during transient empty frames (prevents blink).
            'clear_on_empty_grid': False,
            # HH_260311-00:00 Disable stale clear here; path refresh should replace markers naturally.
            'stale_timeout_sec': 0.0,
            # 2026-02-27: Path cost grids are transient_local; consume latched path immediately.
            'grid_qos_transient_local': True,
            # HH_260305-00:00 Keep full resolution to avoid sparse-looking global markers.
            'sample_stride': 1,
            # HH_260311-00:00 Faster marker conversion for smoother realtime color updates.
            'min_publish_period_sec': 0.01,
            # HH_260305-00:00 Publisher is transient_local; periodic republish not needed.
            # HH_260311-00:00 Match faster global-path grid heartbeat.
            'republish_period_sec': 0.06,
        }],
    )

    local_path_cost_marker = Node(
        package='camping_cart_map',
        executable='cost_field_marker_node',
        name='local_path_cost_marker',
        namespace='planning',
        output='screen',
        parameters=[{
            'grid_topic': '/planning/cost_grid/local_path',
            'marker_topic': '/planning/cost_grid/local_path_markers',
            'min_value': 0,
            'max_value': 100,
            # HH_260311-00:00 Increase opacity so local high-cost cells stand out.
            'alpha': 0.66,
            'z_offset': 0.05,
            # HH_260305-00:00 Keep cells inside lane boundaries while preserving continuity.
            'cell_scale_ratio': 0.85,
            'palette': 'safety',
            'show_unknown': False,
            # HH_260311-00:00 Keep last valid marker set during transient empty local windows.
            'clear_on_empty_grid': False,
            # HH_260311-00:00 Disable stale clear to avoid flicker from brief extractor gaps.
            'stale_timeout_sec': 0.0,
            # 2026-02-27: Path cost grids are transient_local; consume latched path immediately.
            'grid_qos_transient_local': True,
            # HH_260305-00:00 Keep curvature/edge continuity (no decimation holes).
            'sample_stride': 1,
            # HH_260311-00:00 Slightly faster local marker refresh.
            'min_publish_period_sec': 0.008,
            # HH_260305-00:00 Publisher is transient_local; periodic republish not needed.
            # HH_260311-00:00 Match faster local-path grid heartbeat.
            'republish_period_sec': 0.03,
        }],
    )

    return LaunchDescription([
        map_path_arg,
        origin_lat_arg,
        origin_lon_arg,
        origin_alt_arg,
        lanelet_cost_grid_global,
        lanelet_cost_grid_local,
        global_path_cost_marker,
        local_path_cost_marker,
    ])
