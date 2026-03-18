import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# Implements `pkg_share` behavior.
def pkg_share(pkg, rel):
    return os.path.join(get_package_share_directory(pkg), rel)


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    # HH_260128 Load defaults from map_info.yaml so map_path/offset are populated without extra args.
    default_map_info = pkg_share('camrod_bringup', os.path.join('config', 'map', 'map_info.yaml'))
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
        default_value=pkg_share('camrod_bringup', os.path.join('config', 'map', 'map_info.yaml')),
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
        default_value=pkg_share('camrod_bringup', os.path.join('config', 'map', 'map_visualization.yaml')),
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
        description='Enable map module checker publisher',
    )
    module_namespace_arg = DeclareLaunchArgument(
        'module_namespace',
        default_value='map',
        description='Namespace for map module nodes',
    )
    system_namespace_arg = DeclareLaunchArgument(
        'system_namespace',
        default_value='system',
        description='Namespace for system checker nodes',
    )

    map_param = LaunchConfiguration('map_param_file')
    map_path = LaunchConfiguration('map_path')
    origin_lat = LaunchConfiguration('origin_lat')
    origin_lon = LaunchConfiguration('origin_lon')
    origin_alt = LaunchConfiguration('origin_alt')
    map_viz_param = LaunchConfiguration('map_visualization_param_file')
    enable_nav2_inflation_debug_marker = LaunchConfiguration('enable_nav2_inflation_debug_marker')
    enable_module_checker = LaunchConfiguration('enable_module_checker')
    module_namespace = LaunchConfiguration('module_namespace')
    system_namespace = LaunchConfiguration('system_namespace')

    # HH_260121 Map server (Lanelet2) loads OSM and static TF world->map.
    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_share('camrod_map', 'launch/lanelet2_map.launch.py')),
        launch_arguments={
            'map_param_file': map_param,
            'module_namespace': module_namespace,
        }.items(),
    )

    # HH_260121 Lanelet cost grid for Nav2 custom cost layer (/map/cost_grid/lanelet).
    cost_grid_param = pkg_share('camrod_bringup', os.path.join('config', 'map', 'lanelet_cost_grid.yaml'))
    lanelet_cost_grid = Node(
        package='camrod_map',
        executable='lanelet_cost_grid_node',
        name='cost_grid_map',
        namespace=module_namespace,
        output='screen',
        parameters=[
            cost_grid_param,
            {
                'map_path': map_path,
                'offset_lat': origin_lat,
                'offset_lon': origin_lon,
                'offset_alt': origin_alt,
                # HH_260316-00:00 Match projector with map loader/snapper nodes.
                'projector_type': 'local_cartesian',
                'map_frame_id': 'map',
                # 2026-02-23: Align base grid pose reference with planning/localization start frame.
                'pose_topic': '/planning/lanelet_pose',
                'output_topic': '/map/cost_grid/lanelet',
                # 2026-02-27: Static lanelet map grid should not rebuild on every pose callback.
                'rebuild_on_pose': False,
                'rebuild_on_path': False,
                'min_rebuild_period_sec': 0.2,
                # HH_260313-00:00 Boundary-coverage verifier.
                # Warn when known cells are outside lanelet polygons.
                'debug_coverage_stats': True,
                'debug_coverage_stride': 2,
                'debug_coverage_min_value': 0,
                # HH_260305-00:00 Keep static lanelet grid latched; avoid periodic full-grid republish load.
                'republish_period': 0.0,
            },
        ],
    )

    # 2026-02-24: Separate planner base grid with centerline gradient to keep global path centered
    # while preserving lanelet-shaped visualization grid above.
    lanelet_planning_grid = Node(
        package='camrod_map',
        executable='lanelet_cost_grid_node',
        name='cost_grid_planning_base',
        namespace=module_namespace,
        output='screen',
        parameters=[
            cost_grid_param,
            {
                'map_path': map_path,
                'offset_lat': origin_lat,
                'offset_lon': origin_lon,
                'offset_alt': origin_alt,
                # HH_260316-00:00 Match projector with map loader/snapper nodes.
                'projector_type': 'local_cartesian',
                'map_frame_id': 'map',
                'pose_topic': '/planning/lanelet_pose',
                'output_topic': '/map/cost_grid/planning_base',
                # HH_260316-00:00 Planner base policy:
                # 1) keep centerline as primary traversable corridor
                # 2) preserve lane-following preference
                # 3) avoid disconnected corner artifacts in lanelet-polygon raster.
                # This directly addresses intermittent Smac2D "no valid path found"
                # observed when lanelet polygon rasterization was too strict.
                'cost_mode': 'centerline',
                # Keep planner-base corridor tight so paths remain centerline-biased.
                'centerline_half_width': 0.8,
                'centerline_clip_to_lanelet': True,
                # Keep non-centerline lane interior higher cost than centerline strip.
                'centerline_lanelet_fill_value': 96,
                'lanelet_boundary_value': -1,
                'boundary_half_width': 0.05,
                'direction_penalty': 0,
                'backward_penalty': 0,
                'free_value': 0,
                'lethal_value': 100,
                # HH_260316-00:00 Keep outside unknown; local/global costmap base layer
                # treats unknown as non-traversable without forcing immediate lethal writes.
                'outside_value': -1,
                'use_path_bbox': False,
                'lock_window': True,
                'use_map_bbox': True,
                'resolution': 0.05,
                # HH_260316-00:00 Keep centerline strip sampling permissive
                # to avoid micro-gaps at sharp corners.
                'cell_inside_min_hits': 7,
                'cell_inside_min_hits_path': 6,
                'cell_inside_min_hits_boundary': 8,
                'rebuild_on_pose': False,
                'rebuild_on_path': False,
                'min_rebuild_period_sec': 0.2,
                # HH_260316-00:00 Disable heavy coverage logs on planner base.
                # Enable temporarily only for raster-debug sessions.
                'debug_coverage_stats': False,
                'debug_coverage_stride': 2,
                'debug_coverage_min_value': 0,
                # HH_260305-00:00 Keep static planning-base grid latched; avoid periodic full-grid republish load.
                'republish_period': 0.0,
            },
        ],
    )

    # 2026-02-24: Move map cost visualizers under map module ownership.
    inflation_cost_marker = Node(
        package='camrod_map',
        executable='cost_field_marker_node',
        name='inflation_nav2_cost_marker',
        namespace=module_namespace,
        output='screen',
        condition=IfCondition(enable_nav2_inflation_debug_marker),
        parameters=[
            map_viz_param,
            {
                # 2026-03-05: Keep nav2-master visualization as a lightweight debug-only stream.
                # Final contributor-combined visualization is published on /map/cost_grid/inflation_markers.
                'marker_topic': '/map/cost_grid/inflation_nav2_markers',
                'min_value': 40,
                'palette': 'safety',
                'sample_stride': 6,
                'min_publish_period_sec': 1.00,
                'republish_period_sec': 0.0,
            },
        ],
    )

    # 2026-02-25: Marker-only visualization for lanelet base cost grid.
    lanelet_cost_marker = Node(
        package='camrod_map',
        executable='cost_field_marker_node',
        name='lanelet_cost_marker',
        namespace=module_namespace,
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
                # HH_260313-00:00 Shrink cubes so boundary markers stay visibly inside lane bounds.
                'cell_scale_ratio': 0.70,
                'palette': 'pastel_blue_red',
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
        package='camrod_map',
        executable='cost_field_marker_node',
        name='radar_cost_marker',
        namespace=module_namespace,
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
                'palette': 'pastel_orange_red',
                'show_unknown': False,
                'grid_qos_transient_local': True,
                'sample_stride': 2,
                'min_publish_period_sec': 0.03,
                'republish_period_sec': 0.12,
            },
        ],
    )

    lidar_cost_marker = Node(
        package='camrod_map',
        executable='cost_field_marker_node',
        name='lidar_cost_marker',
        namespace=module_namespace,
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
                'palette': 'pastel_green_red',
                'show_unknown': False,
                'grid_qos_transient_local': True,
                'sample_stride': 2,
                'min_publish_period_sec': 0.03,
                'republish_period_sec': 0.12,
            },
        ],
    )

    inflation_marker_aggregator = Node(
        package='camrod_map',
        executable='marker_array_aggregator_node',
        name='inflation_marker_aggregator',
        namespace=module_namespace,
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
        package='camrod_map',
        executable='cost_field_node',
        name='lanelet_cost_field',
        namespace=module_namespace,
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
        package='camrod_system',
        executable='module_checker_node.py',
        name='map_checker',
        namespace=system_namespace,
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
            'diagnostic_topic': '/diagnostics',
            'status_name': 'map/checker',
            'check_period_s': 0.5,
            'warn_throttle_sec': 2.0,
            'publish_ok': True,
        }],
    )

    map_diagnostic = Node(
        package='camrod_map',
        executable='map_diagnostic_node.py',
        name='map_diagnostic',
        namespace=module_namespace,
        output='screen',
        parameters=[{
            # HH_260318-00:00 Module-local diagnostic topic (namespaced).
            'diagnostic_topic': 'diagnostic',
            'publish_period_s': 0.2,
            'stale_timeout_s': 2.0,
            'topic_lanelet_grid': '/map/cost_grid/lanelet',
            'topic_planning_base_grid': '/map/cost_grid/planning_base',
            'topic_lanelet_markers': '/map/cost_grid/lanelet_markers',
            'topic_lidar_markers': '/map/cost_grid/lidar_markers',
            'topic_radar_markers': '/map/cost_grid/radar_markers',
            'topic_inflation_markers': '/map/cost_grid/inflation_markers',
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
        module_namespace_arg,
        system_namespace_arg,
        map_launch,
        lanelet_cost_grid,
        lanelet_planning_grid,
        inflation_cost_marker,
        lanelet_cost_marker,
        lidar_cost_marker,
        radar_cost_marker,
        inflation_marker_aggregator,
        lanelet_cost_field,
        map_diagnostic,
        map_checker,
    ])
