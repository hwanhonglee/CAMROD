import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 2026-02-02 10:32: Use centralized bringup config (synced from package config).
    cost_grid_param = os.path.join(
        get_package_share_directory('camping_cart_bringup'),
        'config', 'planning', 'path_cost_grids.yaml')

    # HH_260123 Path-focused grid for global costmap (Nav2 planner).
    lanelet_cost_grid_global = Node(
        package='camping_cart_map',
        executable='lanelet_cost_grid_node',
        name='lanelet_cost_grid_global_path',
        namespace='planning',
        output='screen',
        parameters=[cost_grid_param],
    )

    # HH_260123 Path-focused grid for local costmap (Nav2 controller local plan).
    lanelet_cost_grid_local = Node(
        package='camping_cart_map',
        executable='lanelet_cost_grid_node',
        name='lanelet_cost_grid_local_path',
        namespace='planning',
        output='screen',
        parameters=[cost_grid_param],
    )

    return LaunchDescription([
        lanelet_cost_grid_global,
        lanelet_cost_grid_local,
    ])
