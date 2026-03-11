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
    default_lidar_grid_param = os.path.join(bringup_share, 'config', 'sensing', 'lidar_cost_grid.yaml')

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
    lidar_grid_param_arg = DeclareLaunchArgument(
        'lidar_cost_grid_param_file',
        default_value=default_lidar_grid_param,
        description='Near-range lidar cost grid parameter file',
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
    enable_lidar_cost_grid_arg = DeclareLaunchArgument(
        'enable_lidar_cost_grid',
        default_value='true',
        description='Enable near-range lidar occupancy grid node',
    )
    enable_module_checker_arg = DeclareLaunchArgument(
        'enable_module_checker',
        default_value='true',
        description='Enable sensing module checker/diagnostics publisher',
    )
    enable_diagnostic_arg = DeclareLaunchArgument(
        'sensing_enable_diagnostic',
        default_value='true',
        description='Enable /sensing/diagnostic publisher',
    )
    diagnostic_publish_period_arg = DeclareLaunchArgument(
        'sensing_diagnostic_publish_period_s',
        default_value='0.2',
        description='Publish period for /sensing/diagnostic',
    )
    param_file = LaunchConfiguration('sensing_param_file')
    radar_param_file = LaunchConfiguration('radar_param_file')
    radar_grid_param_file = LaunchConfiguration('radar_cost_grid_param_file')
    lidar_grid_param_file = LaunchConfiguration('lidar_cost_grid_param_file')
    enable_radar = LaunchConfiguration('enable_radar')
    enable_radar_cost_grid = LaunchConfiguration('enable_radar_cost_grid')
    enable_lidar_cost_grid = LaunchConfiguration('enable_lidar_cost_grid')
    enable_module_checker = LaunchConfiguration('enable_module_checker')
    enable_diagnostic = LaunchConfiguration('sensing_enable_diagnostic')
    diagnostic_publish_period = LaunchConfiguration('sensing_diagnostic_publish_period_s')

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

    lidar_cost_grid = Node(
        package='camping_cart_sensing',
        executable='lidar_cost_grid_node',
        name='lidar_cost_grid',
        namespace='sensing',
        output='screen',
        parameters=[lidar_grid_param_file],
        condition=IfCondition(enable_lidar_cost_grid),
    )

    sensing_checker = Node(
        package='camping_cart_system',
        executable='module_checker_node.py',
        name='sensing_checker',
        namespace='system',
        output='screen',
        condition=IfCondition(enable_module_checker),
        parameters=[{
            'module_name': 'sensing',
            'required_nodes': [
                '/sensing/lidar_preprocessor',
                '/sensing/camera_preprocessor',
                '/sensing/platform_velocity_converter',
            ],
            'required_topics': [
                '/sensing/lidar/near_cost_grid',
                '/sensing/radar/near_cost_grid',
            ],
            'health_topic': '/sensing/healthchecker',
            'check_period_s': 0.5,
            'warn_throttle_sec': 2.0,
            'publish_ok': True,
        }],
    )

    sensing_diagnostic = Node(
        package='camping_cart_sensing',
        executable='sensing_diagnostic_node.py',
        name='sensing_diagnostic',
        namespace='sensing',
        output='screen',
        condition=IfCondition(enable_diagnostic),
        parameters=[{
            'msgs_topic': '/sensing/messages',
            'diagnostic_topic': '/sensing/diagnostic',
            'publish_period_s': diagnostic_publish_period,
        }],
    )

    return LaunchDescription([
        param_file_arg,
        radar_param_arg,
        radar_grid_param_arg,
        lidar_grid_param_arg,
        enable_radar_arg,
        enable_radar_cost_grid_arg,
        enable_lidar_cost_grid_arg,
        enable_module_checker_arg,
        enable_diagnostic_arg,
        diagnostic_publish_period_arg,
        lidar_preprocessor,
        camera_preprocessor,
        velocity_converter,
        radar_sensor,
        radar_cost_grid,
        lidar_cost_grid,
        sensing_checker,
        sensing_diagnostic,
    ])
