# HH_260109 Launch LiDAR/Camera preprocessing and velocity conversion pipeline.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 2026-02-02 10:32: Use centralized bringup config (synced from package config).
    bringup_share = get_package_share_directory('camping_cart_bringup')
    default_param = os.path.join(bringup_share, 'config', 'sensing', 'sensing_params.yaml')

    # HH_260114 Unique param arg to avoid global name collisions.
    param_file_arg = DeclareLaunchArgument(
        'sensing_param_file',
        default_value=default_param,
        description='Sensing parameter file (LiDAR/Camera preprocessing)',
    )
    param_file = LaunchConfiguration('sensing_param_file')

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

    return LaunchDescription([
        param_file_arg,
        lidar_preprocessor,
        camera_preprocessor,
        velocity_converter,
    ])
