"""Localization checker 노드들을 config_dir 기반으로 실행한다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = LaunchConfiguration('config_dir')

    def cfg(filename):
        return PathJoinSubstitution([config_dir, 'localization', filename])

    nodes = [
        Node(
            package='robot_localization_gnss_checker',
            executable='localization_gnss_checker_node',
            name='localization_gnss_checker',
            parameters=[cfg('localization_gnss_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_localization_mode_checker',
            executable='localization_mode_checker_node',
            name='localization_mode_checker',
            parameters=[cfg('localization_mode_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_localization_pose_checker',
            executable='localization_pose_checker_node',
            name='localization_pose_checker',
            parameters=[cfg('localization_pose_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_localization_init_checker',
            executable='localization_init_checker_node',
            name='localization_init_checker',
            parameters=[cfg('localization_init_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_localization_source_checker',
            executable='localization_source_checker_node',
            name='localization_source_checker',
            parameters=[cfg('localization_source_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_localization_lanelet_checker',
            executable='localization_lanelet_checker_node',
            name='localization_lanelet_checker',
            parameters=[cfg('localization_lanelet_checker.yaml')],
            output='screen',
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('config_dir', description='시스템 config 디렉토리 경로'),
        *nodes,
    ])
