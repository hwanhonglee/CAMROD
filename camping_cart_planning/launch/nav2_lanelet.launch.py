#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('camping_cart_planning')
    bringup_share = get_package_share_directory('camping_cart_bringup')

    # -------------------------------------------------------------------------
    # 기본 파일 경로 (4-stage overlay)
    # -------------------------------------------------------------------------
    default_base_param = os.path.join(bringup_share, 'config', 'planning', 'nav2_base.yaml')
    default_vehicle_param = os.path.join(bringup_share, 'config', 'planning', 'nav2_vehicle.yaml')
    default_lanelet_param = os.path.join(bringup_share, 'config', 'planning', 'nav2_lanelet_overlay.yaml')
    default_behavior_param = os.path.join(bringup_share, 'config', 'planning', 'nav2_behavior.yaml')

    # -------------------------------------------------------------------------
    # Launch Arguments
    # -------------------------------------------------------------------------
    nav2_base_param_arg = DeclareLaunchArgument(
        'nav2_base_param_file',
        default_value=default_base_param,
        description='Nav2 base profile (common planner/controller/costmap defaults)',
    )
    nav2_vehicle_param_arg = DeclareLaunchArgument(
        'nav2_vehicle_param_file',
        default_value=default_vehicle_param,
        description='Nav2 vehicle profile (footprint/dynamics/controller tuning)',
    )
    nav2_lanelet_param_arg = DeclareLaunchArgument(
        'nav2_lanelet_param_file',
        default_value=default_lanelet_param,
        description='Nav2 lanelet profile (lanelet/radar/inflation overlays)',
    )
    nav2_behavior_param_arg = DeclareLaunchArgument(
        'nav2_behavior_param_file',
        default_value=default_behavior_param,
        description='Nav2 behavior profile (BT XML/plugins)',
    )
    enable_path_cost_grids_arg = DeclareLaunchArgument(
        'enable_path_cost_grids',
        default_value='true',
        description='Enable planning path-cost-grid helper nodes (/planning/cost_grid/*)',
    )
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='/home/hong/cart_test_ws/src/lanelet2_maps.osm',
        description='Lanelet2 map path for path-cost-grid helpers',
    )
    origin_lat_arg = DeclareLaunchArgument('origin_lat', default_value='36.8436194')
    origin_lon_arg = DeclareLaunchArgument('origin_lon', default_value='128.0925490')
    origin_alt_arg = DeclareLaunchArgument('origin_alt', default_value='0.0')

    # -------------------------------------------------------------------------
    # LaunchConfigurations
    # -------------------------------------------------------------------------
    nav2_base_param_file = LaunchConfiguration('nav2_base_param_file')
    nav2_vehicle_param_file = LaunchConfiguration('nav2_vehicle_param_file')
    nav2_lanelet_param_file = LaunchConfiguration('nav2_lanelet_param_file')
    nav2_behavior_param_file = LaunchConfiguration('nav2_behavior_param_file')
    enable_path_cost_grids = LaunchConfiguration('enable_path_cost_grids')
    map_path = LaunchConfiguration('map_path')
    origin_lat = LaunchConfiguration('origin_lat')
    origin_lon = LaunchConfiguration('origin_lon')
    origin_alt = LaunchConfiguration('origin_alt')
    # 2026-02-25: Apply Nav2 params in deterministic overlay order:
    # base -> vehicle -> lanelet -> behavior.
    nav2_base_params = RewrittenYaml(
        source_file=nav2_base_param_file,
        root_key='planning',
        param_rewrites={},
        convert_types=True,
    )
    nav2_vehicle_params = RewrittenYaml(
        source_file=nav2_vehicle_param_file,
        root_key='planning',
        param_rewrites={},
        convert_types=True,
    )
    nav2_lanelet_params = RewrittenYaml(
        source_file=nav2_lanelet_param_file,
        root_key='planning',
        param_rewrites={},
        convert_types=True,
    )
    nav2_behavior_params = RewrittenYaml(
        source_file=nav2_behavior_param_file,
        root_key='planning',
        param_rewrites={},
        convert_types=True,
    )

    # -------------------------------------------------------------------------
    # ★가장 중요한 방어막: costmap/behavior/smoother의 robot_base_frame을 "구조 그대로" 강제
    # - 이 dict는 ROS2 파라미터 트리에 맞는 형태라서 깨질 일이 없음
    # - 여기서 base_link로 떨어지는 문제를 확실히 차단
    # -------------------------------------------------------------------------
    force_robot_base_link_overrides = {
        'global_costmap': {
            'global_costmap': {
                'ros__parameters': {
                    'robot_base_frame': 'robot_base_link',
                }
            }
        },
        'local_costmap': {
            'local_costmap': {
                'ros__parameters': {
                    'robot_base_frame': 'robot_base_link',
                }
            }
        },
        'behavior_server': {
            'ros__parameters': {
                'global_frame': 'map',
                'local_frame': 'odom',
                'robot_base_frame': 'robot_base_link',
                'transform_tolerance': 0.2,
            }
        },
        'bt_navigator': {
            'ros__parameters': {
                'global_frame': 'map',
                'robot_base_frame': 'robot_base_link',
                'odom_topic': '/localization/odometry/filtered',
                'transform_tolerance': 0.2,
            }
        },
        'nav2_velocity_smoother': {
            'ros__parameters': {
                'robot_base_frame': 'robot_base_link',
            }
        },
    }
    nav2_param_chain = [
        nav2_base_params,
        nav2_vehicle_params,
        nav2_lanelet_params,
        nav2_behavior_params,
        force_robot_base_link_overrides,
    ]

    # -------------------------------------------------------------------------
    # Nav2 nodes under /planning namespace
    # -------------------------------------------------------------------------
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='planning',
        output='screen',
        parameters=nav2_param_chain,
        # 2026-02-25: Publish canonical global path directly from planner_server.
        remappings=[
            ('plan', '/planning/global_path'),
        ],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='planning',
        output='screen',
        parameters=nav2_param_chain,
        # HH_260304-00:00 Keep only controller-internal debug topics that are still useful.
        # HH_260304-00:00 Do not expose /planning/local_plan_raw: legacy RViz sessions may
        # HH_260304-00:00 overlay it on top of /planning/local_path and make the local plan
        # HH_260304-00:00 look duplicated or branch to a wrong loop segment.
        remappings=[
            ('received_global_plan', '/planning/local_path_controller'),
            ('transformed_global_plan', '/planning/local_path_dwb'),
        ],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='planning',
        output='screen',
        parameters=nav2_param_chain + [{
            # HH_260306-00:00 Hard-override to block fallback to default "base_link"
            # during recovery behavior pose transforms.
            'global_frame': 'map',
            'local_frame': 'odom',
            'robot_base_frame': 'robot_base_link',
            'transform_tolerance': 0.2,
        }],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='planning',
        output='screen',
        parameters=nav2_param_chain + [{
            # HH_260306-00:00 Keep BT transform helpers pinned to robot_base_link.
            'global_frame': 'map',
            'robot_base_frame': 'robot_base_link',
            'transform_tolerance': 0.2,
        }],
        remappings=[
            # goal_snapper를 쓰는 구조라면 이게 맞음
            ('/goal_pose', '/planning/goal_pose'),
        ],
    )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planning',
        namespace='planning',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator',
                # 2026-01-30 14:32: Let planner/controller manage internal costmap lifecycles (avoid duplicate configure).
            ],
        }],
    )

    path_cost_grids = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'path_cost_grids.launch.py')),
        launch_arguments={
            'map_path': map_path,
            'origin_lat': origin_lat,
            'origin_lon': origin_lon,
            'origin_alt': origin_alt,
        }.items(),
        condition=IfCondition(enable_path_cost_grids),
    )

    return LaunchDescription([
        nav2_base_param_arg,
        nav2_vehicle_param_arg,
        nav2_lanelet_param_arg,
        nav2_behavior_param_arg,
        enable_path_cost_grids_arg,
        map_path_arg,
        origin_lat_arg,
        origin_lon_arg,
        origin_alt_arg,

        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        lifecycle_mgr,
        path_cost_grids,
    ])
