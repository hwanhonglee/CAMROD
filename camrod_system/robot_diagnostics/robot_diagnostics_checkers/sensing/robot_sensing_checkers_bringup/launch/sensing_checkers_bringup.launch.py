"""
Sensing Checkers Bringup

센싱 진단 체커 노드를 한번에 실행한다.

사용법:
  ros2 launch robot_sensing_checkers_bringup sensing_checkers_bringup.launch.py

포함 노드
---------
  - gnss_checker_node                : /sensing/gnss/navsatfix
  - imu_checker_node                 : /sensing/imu/data
  - lidar_checker_node               : /sensing/lidar/points
  - radar_checker_node               : /sensing/radar/*/range (6채널)
  - camera_checker_node              : /sensing/camera/front/image_raw
  - wheel_odometry_checker_node      : /platform/wheel/odometry
  - cost_grid_checker_node           : /sensing/lidar/near_cost_grid
  - velocity_converter_checker_node  : /sensing/platform_velocity_converter/twist_with_covariance
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _config(pkg: str, filename: str):
    return PathJoinSubstitution([FindPackageShare(pkg), 'config', filename])


def generate_launch_description():
    nodes = [
        Node(
            package='robot_gnss_checker',
            executable='gnss_checker_node',
            name='gnss_checker',
            parameters=[_config('robot_gnss_checker', 'gnss_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_imu_checker',
            executable='imu_checker_node',
            name='imu_checker',
            parameters=[_config('robot_imu_checker', 'imu_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_lidar_checker',
            executable='lidar_checker_node',
            name='lidar_checker',
            parameters=[_config('robot_lidar_checker', 'lidar_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_radar_checker',
            executable='radar_checker_node',
            name='radar_checker',
            parameters=[_config('robot_radar_checker', 'radar_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_camera_checker',
            executable='camera_checker_node',
            name='camera_checker',
            parameters=[_config('robot_camera_checker', 'camera_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_wheel_odometry_checker',
            executable='wheel_odometry_checker_node',
            name='wheel_odometry_checker',
            parameters=[_config('robot_wheel_odometry_checker',
                                'wheel_odometry_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_cost_grid_checker',
            executable='cost_grid_checker_node',
            name='cost_grid_checker',
            parameters=[_config('robot_cost_grid_checker', 'cost_grid_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_velocity_converter_checker',
            executable='velocity_converter_checker_node',
            name='velocity_converter_checker',
            parameters=[_config('robot_velocity_converter_checker',
                                'velocity_converter_checker.yaml')],
            output='screen',
        ),
    ]

    return LaunchDescription(nodes)
