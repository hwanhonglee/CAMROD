import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    # 2026-02-02 10:32: Use centralized bringup config (synced from package config).
    cost_grid_param = os.path.join(
        get_package_share_directory('camrod_bringup'),
        'config', 'planning', 'path_cost_grids.yaml')
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='/home/hong/camrod_ws/src/lanelet2_maps.osm',
        description='Lanelet2 map path for path-cost-grid helpers',
    )
    module_namespace_arg = DeclareLaunchArgument(
        'module_namespace',
        default_value='planning',
        description='Namespace for planning path-cost-grid helper nodes',
    )
    origin_lat_arg = DeclareLaunchArgument('origin_lat', default_value='36.8436194')
    origin_lon_arg = DeclareLaunchArgument('origin_lon', default_value='128.0925490')
    origin_alt_arg = DeclareLaunchArgument('origin_alt', default_value='0.0')
    map_path = LaunchConfiguration('map_path')
    origin_lat = LaunchConfiguration('origin_lat')
    origin_lon = LaunchConfiguration('origin_lon')
    origin_alt = LaunchConfiguration('origin_alt')
    module_namespace = LaunchConfiguration('module_namespace')

    # HH_260123 Path-focused grid for global costmap (Nav2 planner).
    lanelet_cost_grid_global = Node(
        package='camrod_map',
        executable='lanelet_cost_grid_node',
        name='lanelet_cost_grid_global_path',
        namespace=module_namespace,
        output='screen',
        parameters=[
            cost_grid_param,
            {
                'map_path': map_path,
                'offset_lat': origin_lat,
                'offset_lon': origin_lon,
                'offset_alt': origin_alt,
                # HH_260311-00:00 Hard overrides for stable realtime global path-cost refresh.
                # Keep this in launch so behavior is deterministic even with stale YAML installs.
                'pose_topic': '/planning/lanelet_pose',
                'path_topic': '/planning/global_path',
                'goal_topic': '/planning/goal_pose_snapped',
                # HH_260317-00:00 Switch global-path cost immediately on goal update.
                # Do not retain stale previous-route strips after a new target is selected.
                'clear_path_on_goal': True,
                'ignore_empty_path': True,
                # Do not build lane-wide fallback when no valid global path is available.
                'allow_build_without_path': False,
                'path_use_lateral_gradient': True,
                # HH_260316-00:00 Centerline-first tuning:
                # lateral(centerline) term must dominate pose-distance term.
                'path_pose_cost_weight': 0.05,
                'path_lateral_cost_weight': 0.95,
                # HH_260318-00:00 Slightly relax lanelet matching so global
                # path updates are accepted at curved endpoints after goal change.
                'path_lanelet_match_max_dist': 0.45,
                'path_lanelet_allow_nearest_fallback': True,
                'path_lanelet_sample_step': 0.05,
                'primary_path_timeout_sec': 6.0,
                # HH_260315-00:00 Problem fix:
                # Keep latest global route cost strip until a new route is published.
                'stale_path_timeout_sec': 0.0,
                # HH_260318-00:00 Robustness-first:
                # strict "fresh-path-after-goal" filtering can reject valid replans when
                # goal/path publishers have small timing/frame mismatches. Disabling the
                # filter prevents global-path marker freeze after a new goal.
                'drop_stale_path_after_goal': False,
                'path_goal_stamp_slack_sec': 0.20,
                'fresh_path_goal_match_tolerance_m': 4.0,
                'rebuild_on_pose': True,
                'rebuild_on_path': True,
                'rebuild_on_timer': True,
                'min_rebuild_period_sec': 0.02,
                'republish_period': 0.03,
                'debug_coverage_stats': False,
                'debug_coverage_stride': 2,
                'debug_coverage_min_value': 0,
            },
        ],
    )

    # HH_260123 Path-focused grid for local costmap (Nav2 controller local plan).
    lanelet_cost_grid_local = Node(
        package='camrod_map',
        executable='lanelet_cost_grid_node',
        name='lanelet_cost_grid_local_path',
        namespace=module_namespace,
        output='screen',
        parameters=[
            cost_grid_param,
            {
                'map_path': map_path,
                'offset_lat': origin_lat,
                'offset_lon': origin_lon,
                'offset_alt': origin_alt,
                # HH_260311-00:00 Hard overrides for low-latency local route refresh.
                # Keep previous local-path cost through transient extractor/controller gaps.
                'pose_topic': '/planning/lanelet_pose',
                'path_topic': '/planning/local_path',
                'goal_topic': '/planning/goal_pose_snapped',
                # HH_260316-00:00 Switch local-path strip immediately on goal handoff.
                # Prevent stale local-route cells from remaining after a new goal.
                'clear_path_on_goal': True,
                'ignore_empty_path': True,
                # HH_260316-00:00 Keep local grid publisher alive during transient
                # local-path extractor/controller gaps.
                'allow_build_without_path': True,
                'path_use_lateral_gradient': True,
                # HH_260316-00:00 Centerline-first tuning:
                # keep local path center-biased unless obstacle/risk costs force avoidance.
                'path_pose_cost_weight': 0.20,
                'path_lateral_cost_weight': 0.80,
                'path_lanelet_match_max_dist': 0.35,
                'path_lanelet_sample_step': 0.06,
                'primary_path_timeout_sec': 2.5,
                # HH_260315-00:00 Problem fix:
                # Keep short hold to avoid blink but still purge stale local route quickly.
                'stale_path_timeout_sec': 0.6,
                # Local path endpoint is near current robot pose (not goal endpoint),
                # so stale-goal geometric rejection must stay disabled.
                'drop_stale_path_after_goal': False,
                'rebuild_on_pose': True,
                'rebuild_on_path': True,
                'rebuild_on_timer': True,
                'min_rebuild_period_sec': 0.01,
                'republish_period': 0.015,
                'debug_coverage_stats': False,
                'debug_coverage_stride': 2,
                'debug_coverage_min_value': 0,
            },
        ],
    )

    # 2026-02-25: Marker views for path cost grids.
    # Keep OccupancyGrid for planner math, use marker topics for lanelet-shaped RViz display.
    global_path_cost_marker = Node(
        package='camrod_map',
        executable='cost_field_marker_node',
        name='global_path_cost_marker',
        namespace=module_namespace,
        output='screen',
        parameters=[{
            'grid_topic': '/planning/cost_grid/global_path',
            'marker_topic': '/planning/cost_grid/global_path_markers',
            'min_value': 0,
            'max_value': 100,
            # HH_260311-00:00 Increase opacity so high-cost (red) segments are visible.
                'alpha': 0.60,
                'z_offset': 0.03,
                # HH_260313-00:00 Shrink marker cubes to avoid visual spill over lane boundaries.
                'cell_scale_ratio': 0.70,
                'palette': 'pastel_purple_red',
                'show_unknown': False,
                # Clear immediately when global path grid is cleared on new goal.
                'clear_on_empty_grid': True,
                # Keep short stale timeout to prevent long-lived stale global strips.
                'stale_timeout_sec': 0.3,
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
        package='camrod_map',
        executable='cost_field_marker_node',
        name='local_path_cost_marker',
        namespace=module_namespace,
        output='screen',
        parameters=[{
            'grid_topic': '/planning/cost_grid/local_path',
            'marker_topic': '/planning/cost_grid/local_path_markers',
            'min_value': 0,
            'max_value': 100,
            # HH_260311-00:00 Increase opacity so local high-cost cells stand out.
                'alpha': 0.66,
                'z_offset': 0.05,
                # HH_260313-00:00 Shrink marker cubes to avoid visual spill over lane boundaries.
                'cell_scale_ratio': 0.70,
                'palette': 'pastel_cyan_red',
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
        module_namespace_arg,
        origin_lat_arg,
        origin_lon_arg,
        origin_alt_arg,
        lanelet_cost_grid_global,
        lanelet_cost_grid_local,
        global_path_cost_marker,
        local_path_cost_marker,
    ])
