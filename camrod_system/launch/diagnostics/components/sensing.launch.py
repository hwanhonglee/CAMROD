"""Sensing checker 노드들을 config_dir 기반으로 실행한다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = LaunchConfiguration('config_dir')

    def cfg(filename):
        return PathJoinSubstitution([config_dir, 'sensing', filename])

    nodes = [
        Node(
            package='camrod_system',
            executable='gnss_checker_node',
            name='gnss_checker',
            parameters=[cfg('gnss_checker.yaml')],
            output='screen',
        ),
        Node(
            package='camrod_system',
            executable='imu_checker_node',
            name='imu_checker',
            parameters=[cfg('imu_checker.yaml')],
            output='screen',
        ),
        Node(
            package='camrod_system',
            executable='lidar_checker_node',
            name='lidar_checker',
            parameters=[cfg('lidar_checker.yaml')],
            output='screen',
        ),
        Node(
            package='camrod_system',
            executable='radar_checker_node',
            name='radar_checker',
            parameters=[cfg('radar_checker.yaml')],
            output='screen',
        ),
        Node(
            package='camrod_system',
            executable='camera_checker_node',
            name='camera_checker',
            parameters=[cfg('camera_checker.yaml')],
            output='screen',
        ),
        Node(
            package='camrod_system',
            executable='wheel_odometry_checker_node',
            name='wheel_odometry_checker',
            parameters=[cfg('wheel_odometry_checker.yaml')],
            output='screen',
        ),
        Node(
            package='camrod_system',
            executable='cost_grid_checker_node',
            name='cost_grid_checker',
            parameters=[cfg('cost_grid_checker.yaml')],
            output='screen',
        ),
        Node(
            package='camrod_system',
            executable='velocity_converter_checker_node',
            name='velocity_converter_checker',
            parameters=[cfg('velocity_converter_checker.yaml')],
            output='screen',
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('config_dir', description='시스템 config 디렉토리 경로'),
        *nodes,
    ])
