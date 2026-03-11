import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
    enable_nav2_inflation_debug_marker_arg = DeclareLaunchArgument(
        'enable_nav2_inflation_debug_marker',
        default_value='false',
        description='Enable heavy debug marker from /planning/global_costmap/costmap',
    )
    enable_module_checker_arg = DeclareLaunchArgument(
        'enable_module_checker',
        default_value='true',
        description='Enable map module checker/diagnostics publisher',
    )
    enable_diagnostic_arg = DeclareLaunchArgument(
        'map_enable_diagnostic',
        default_value='true',
        description='Enable /map/diagnostic publisher',
    )
    diagnostic_publish_period_arg = DeclareLaunchArgument(
        'map_diagnostic_publish_period_s',
        default_value='0.2',
        description='Publish period for /map/diagnostic',
    )

    map_param = LaunchConfiguration('map_param_file')
    map_path = LaunchConfiguration('map_path')
    origin_lat = LaunchConfiguration('origin_lat')
    origin_lon = LaunchConfiguration('origin_lon')
    origin_alt = LaunchConfiguration('origin_alt')
    map_viz_param = LaunchConfiguration('map_visualization_param_file')
    enable_nav2_inflation_debug_marker = LaunchConfiguration('enable_nav2_inflation_debug_marker')
    enable_module_checker = LaunchConfiguration('enable_module_checker')
    enable_diagnostic = LaunchConfiguration('map_enable_diagnostic')
    diagnostic_publish_period = LaunchConfiguration('map_diagnostic_publish_period_s')

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
                'pose_topic': '/planning/lanelet_pose',
                'output_topic': '/map/cost_grid/lanelet',
                # 2026-02-27: Static lanelet map grid should not rebuild on every pose callback.
                'rebuild_on_pose': False,
                'rebuild_on_path': False,
                'min_rebuild_period_sec': 0.2,
                # HH_260305-00:00 Keep static lanelet grid latched; avoid periodic full-grid republish load.
                'republish_period': 0.0,
            },
        ],
    )

    # 2026-02-24: Separate planner base grid with centerline gradient to keep global path centered
    # while preserving lanelet-shaped visualization grid above.
    lanelet_planning_grid = Node(
        package='camping_cart_map',
        executable='lanelet_cost_grid_node',
        name='cost_grid_planning_base',
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
                'pose_topic': '/planning/lanelet_pose',
                'output_topic': '/map/cost_grid/planning_base',
                # HH_260306-00:00 Planner-base must keep the whole lanelet interior drivable.
                # Using centerline-only strips can make start cells fall outside the strip and
                # trigger "Starting point in lethal space" on goal updates.
                'cost_mode': 'lanelet',
                # HH_260306-00:00 Keep planner-base pose-invariant.
                'centerline_use_distance_gradient': False,
                # HH_260306-00:00 Lanelet interior free, lane boundaries high, outside lethal.
                # HH_260309-00:00 Keep lane boundaries high-cost but non-lethal.
                # Outside lanelet remains lethal via outside_value=100.
                'lanelet_boundary_value': 95,
                'direction_penalty': 0,
                'backward_penalty': 0,
                'free_value': 0,
                'lethal_value': 90,
                'outside_value': 100,
                'use_path_bbox': False,
                'lock_window': True,
                'use_map_bbox': True,
                'resolution': 0.05,
                'rebuild_on_pose': False,
                'rebuild_on_path': False,
                'min_rebuild_period_sec': 0.2,
                # HH_260305-00:00 Keep static planning-base grid latched; avoid periodic full-grid republish load.
                'republish_period': 0.0,
            },
        ],
    )

    # 2026-02-24: Move map cost visualizers under map module ownership.
    inflation_cost_marker = Node(
        package='camping_cart_map',
        executable='cost_field_marker_node',
        name='inflation_nav2_cost_marker',
        namespace='map',
        output='screen',
        condition=IfCondition(enable_nav2_inflation_debug_marker),
        parameters=[
            map_viz_param,
            {
                # 2026-03-05: Keep nav2-master visualization as a lightweight debug-only stream.
                # Final contributor-combined visualization is published on /map/cost_grid/inflation_markers.
                'marker_topic': '/map/cost_grid/inflation_nav2_markers',
                'min_value': 40,
                'sample_stride': 6,
                'min_publish_period_sec': 1.00,
                'republish_period_sec': 0.0,
            },
        ],
    )

    # 2026-02-25: Marker-only visualization for lanelet base cost grid.
    lanelet_cost_marker = Node(
        package='camping_cart_map',
        executable='cost_field_marker_node',
        name='lanelet_cost_marker',
        namespace='map',
        output='screen',
        parameters=[
            {
                'grid_topic': '/map/cost_grid/lanelet',
                'marker_topic': '/map/cost_grid/lanelet_markers',
                'marker_scale': 0.08,
                'min_value': 0,
                'max_value': 100,
                'alpha': 0.12,
                'z_offset': 0.02,
                # HH_260305-00:00 Keep lanelet boundary markers visually inside lane bounds.
                'cell_scale_ratio': 0.85,
                'palette': 'safety',
                'show_unknown': False,
                'grid_qos_transient_local': True,
                # HH_260305-00:00 Preserve boundary detail (avoid corner underfill).
                'sample_stride': 1,
                'min_publish_period_sec': 0.02,
                # HH_260305-00:00 Publisher is transient_local; periodic republish not needed.
                'republish_period_sec': 0.20,
            },
        ],
    )

    radar_cost_marker = Node(
        package='camping_cart_map',
        executable='cost_field_marker_node',
        name='radar_cost_marker',
        namespace='map',
        output='screen',
        parameters=[
            {
                'grid_topic': '/sensing/radar/near_cost_grid',
                'marker_topic': '/map/cost_grid/radar_markers',
                'marker_scale': 0.10,
                'min_value': 0,
                'max_value': 100,
                'alpha': 0.42,
                'z_offset': 0.04,
                'cell_scale_ratio': 0.70,
                'palette': 'safety',
                'show_unknown': False,
                'grid_qos_transient_local': True,
                'sample_stride': 2,
                'min_publish_period_sec': 0.03,
                'republish_period_sec': 0.12,
            },
        ],
    )

    lidar_cost_marker = Node(
        package='camping_cart_map',
        executable='cost_field_marker_node',
        name='lidar_cost_marker',
        namespace='map',
        output='screen',
        parameters=[
            {
                'grid_topic': '/sensing/lidar/near_cost_grid',
                'marker_topic': '/map/cost_grid/lidar_markers',
                'marker_scale': 0.08,
                'min_value': 0,
                'max_value': 100,
                'alpha': 0.40,
                'z_offset': 0.035,
                'cell_scale_ratio': 0.70,
                'palette': 'safety',
                'show_unknown': False,
                'grid_qos_transient_local': True,
                'sample_stride': 2,
                'min_publish_period_sec': 0.03,
                'republish_period_sec': 0.12,
            },
        ],
    )

    inflation_marker_aggregator = Node(
        package='camping_cart_map',
        executable='marker_array_aggregator_node',
        name='inflation_marker_aggregator',
        namespace='map',
        output='screen',
        parameters=[{
            # 2026-03-05: User-facing final inflation marker = contributor-combined cost layers.
            'output_topic': '/map/cost_grid/inflation_markers',
            'input_topics': [
                '/map/cost_grid/lanelet_markers',
                '/map/cost_grid/lidar_markers',
                '/map/cost_grid/radar_markers',
                '/planning/cost_grid/global_path_markers',
                '/planning/cost_grid/local_path_markers',
            ],
            # HH_260305-00:00 Keep lightweight timer so stale contributor patches are purged quickly.
            'republish_period_sec': 0.05,
            # HH_260305-00:00 Low-latency merge with light throttling.
            'min_publish_period_sec': 0.01,
            # HH_260305-00:00 Purge only dynamic inputs when they stop updating.
            'stale_timeout_sec': 1.2,
            'stale_timeout_topics': [
                '/map/cost_grid/lidar_markers',
                '/map/cost_grid/radar_markers',
            ],
        }],
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

    map_checker = Node(
        package='camping_cart_system',
        executable='module_checker_node.py',
        name='map_checker',
        namespace='system',
        output='screen',
        condition=IfCondition(enable_module_checker),
        parameters=[{
            'module_name': 'map',
            'required_nodes': [
                '/map/lanelet2_map',
                '/map/cost_grid_map',
                '/map/cost_grid_planning_base',
            ],
            'required_topics': [
                '/map/cost_grid/lanelet',
                '/map/cost_grid/planning_base',
                '/map/cost_grid/inflation_markers',
            ],
            'health_topic': '/map/healthchecker',
            'check_period_s': 0.5,
            'warn_throttle_sec': 2.0,
            'publish_ok': True,
        }],
    )

    map_diagnostic = Node(
        package='camping_cart_map',
        executable='map_diagnostic_node.py',
        name='map_diagnostic',
        namespace='map',
        output='screen',
        condition=IfCondition(enable_diagnostic),
        parameters=[{
            'msgs_topic': '/map/messages',
            'diagnostic_topic': '/map/diagnostic',
            'publish_period_s': diagnostic_publish_period,
        }],
    )

    return LaunchDescription([
        map_param_arg,
        map_path_arg,
        origin_lat_arg,
        origin_lon_arg,
        origin_alt_arg,
        map_viz_param_arg,
        enable_nav2_inflation_debug_marker_arg,
        enable_module_checker_arg,
        enable_diagnostic_arg,
        diagnostic_publish_period_arg,
        map_launch,
        lanelet_cost_grid,
        lanelet_planning_grid,
        inflation_cost_marker,
        lanelet_cost_marker,
        lidar_cost_marker,
        radar_cost_marker,
        inflation_marker_aggregator,
        lanelet_cost_field,
        map_checker,
        map_diagnostic,
    ])
