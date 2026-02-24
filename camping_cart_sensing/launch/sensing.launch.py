# HH_260109 Launch LiDAR/Camera preprocessing and velocity conversion pipeline.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 2026-02-02 10:32: Use centralized bringup config (synced from package config).
    bringup_share = get_package_share_directory('camping_cart_bringup')
    default_param = os.path.join(bringup_share, 'config', 'sensing', 'sensing_params.yaml')
    default_radar_param = os.path.join(bringup_share, 'config', 'sensing', 'sen0592_radar.yaml')
    default_radar_grid_param = os.path.join(bringup_share, 'config', 'sensing', 'radar_cost_grid.yaml')

    # HH_260114 Unique param arg to avoid global name collisions.
    param_file_arg = DeclareLaunchArgument(
        'sensing_param_file',
        default_value=default_param,
        description='Sensing parameter file (LiDAR/Camera preprocessing)',
    )
    radar_param_arg = DeclareLaunchArgument(
        'radar_param_file',
        default_value=default_radar_param,
        description='Radar serial sensor parameter file (SEN0592)',
    )
    radar_grid_param_arg = DeclareLaunchArgument(
        'radar_cost_grid_param_file',
        default_value=default_radar_grid_param,
        description='Near-range radar cost grid parameter file',
    )
    enable_radar_arg = DeclareLaunchArgument(
        'enable_radar',
        default_value='false',
        description='Enable SEN0592 serial radar node',
    )
    enable_radar_cost_grid_arg = DeclareLaunchArgument(
        'enable_radar_cost_grid',
        default_value='true',
        description='Enable near-range radar occupancy grid node',
    )
    param_file = LaunchConfiguration('sensing_param_file')
    radar_param_file = LaunchConfiguration('radar_param_file')
    radar_grid_param_file = LaunchConfiguration('radar_cost_grid_param_file')
    enable_radar = LaunchConfiguration('enable_radar')
    enable_radar_cost_grid = LaunchConfiguration('enable_radar_cost_grid')

    # HH_260112 Namespace sensing nodes under /sensing with short names.
    lidar_preprocessor = Node(
        package='camping_cart_sensing',
        executable='lidar_preprocessor_node',
        name='lidar_preprocessor',
        namespace='sensing',
        output='screen',
        parameters=[param_file],
    )

    # HH_260109 Camera preprocessing (raw -> processed).
    camera_preprocessor = Node(
        package='camping_cart_sensing',
        executable='camera_preprocessor_node',
        name='camera_preprocessor',
        namespace='sensing',
        output='screen',
        parameters=[param_file],
    )

    # HH_260109 Convert platform velocity + IMU to twist_with_covariance.
    velocity_converter = Node(
        package='camping_cart_sensing',
        executable='platform_velocity_converter_node',
        name='platform_velocity_converter',
        namespace='sensing',
        output='screen',
        parameters=[param_file],
    )

    # 2026-02-24: Optional ultrasonic radar input.
    radar_sensor = Node(
        package='camping_cart_sensing',
        executable='sen0592_radar_node',
        name='sen0592_radar',
        namespace='sensing',
        output='screen',
        parameters=[radar_param_file],
        condition=IfCondition(enable_radar),
    )

    # 2026-02-24: Radar near-range cost grid map for short-distance collision layer/visualization.
    radar_cost_grid = Node(
        package='camping_cart_sensing',
        executable='radar_cost_grid_node',
        name='radar_cost_grid',
        namespace='sensing',
        output='screen',
        parameters=[radar_grid_param_file],
        condition=IfCondition(enable_radar_cost_grid),
    )

    return LaunchDescription([
        param_file_arg,
        radar_param_arg,
        radar_grid_param_arg,
        enable_radar_arg,
        enable_radar_cost_grid_arg,
        lidar_preprocessor,
        camera_preprocessor,
        velocity_converter,
        radar_sensor,
        radar_cost_grid,
    ])
