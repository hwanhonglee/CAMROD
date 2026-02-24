#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    # 기본 파일 경로
    # -------------------------------------------------------------------------
    # 2026-02-02 10:32: Use centralized bringup config (synced from package config).
    default_param = os.path.join(bringup_share, 'config', 'planning', 'nav2_lanelet.yaml')

    # -------------------------------------------------------------------------
    # Launch Arguments
    # -------------------------------------------------------------------------
    nav2_param_arg = DeclareLaunchArgument(
        'nav2_param_file',
        default_value=default_param,
        description='Nav2 parameter file (nav2_lanelet.yaml)',
    )
    controller_mode_arg = DeclareLaunchArgument(
        'controller_mode',
        default_value='rpp',
        description='Controller choice: rpp / dwb',
    )

    # -------------------------------------------------------------------------
    # LaunchConfigurations
    # -------------------------------------------------------------------------
    nav2_param_file = LaunchConfiguration('nav2_param_file')
    controller_mode = LaunchConfiguration('controller_mode')

    # HH_260129 Namespace 주입만 수행 (파라미터 값은 그대로 사용)
    nav2_params = RewrittenYaml(
        source_file=nav2_param_file,
        root_key='planning',          # 노드들이 /planning 네임스페이스에 있으므로
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
                'robot_base_frame': 'robot_base_link',
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

    # -------------------------------------------------------------------------
    # Nav2 nodes under /planning namespace
    # -------------------------------------------------------------------------
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='planning',
        output='screen',
        parameters=[nav2_params, force_robot_base_link_overrides],
        # 2026-02-05 14:37: Keep planner_server plan on /planning/plan; compute_path_bridge owns /planning/global_path.
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='planning',
        output='screen',
        parameters=[nav2_params, force_robot_base_link_overrides],
        # HH_260127 Align local path naming with global_path convention.
        remappings=[
            ('received_global_plan', '/planning/local_path'),
        ],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='planning',
        output='screen',
        parameters=[nav2_params, force_robot_base_link_overrides],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='planning',
        output='screen',
        parameters=[nav2_params, force_robot_base_link_overrides],
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
    )

    return LaunchDescription([
        nav2_param_arg,
        controller_mode_arg,

        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        lifecycle_mgr,
        path_cost_grids,
    ])
