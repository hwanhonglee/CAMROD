"""Planning checker 노드들을 config_dir 기반으로 실행한다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = LaunchConfiguration('config_dir')

    def cfg(filename):
        return PathJoinSubstitution([config_dir, 'planning', filename])

    nodes = [
        Node(
            package='camrod_system',
            executable='planning_lifecycle_checker_node',
            name='planning_lifecycle_checker',
            parameters=[cfg('planning_lifecycle_checker.yaml')],
            output='screen',
        ),
        Node(
            package='camrod_system',
            executable='planning_costmap_checker_node',
            name='planning_costmap_checker',
            parameters=[cfg('planning_costmap_checker.yaml')],
            output='screen',
        ),
        Node(
            package='camrod_system',
            executable='planning_nav_status_checker_node',
            name='planning_nav_status_checker',
            parameters=[cfg('planning_nav_status_checker.yaml')],
            output='screen',
        ),
        Node(
            package='camrod_system',
            executable='planning_path_checker_node',
            name='planning_path_checker',
            parameters=[cfg('planning_path_checker.yaml')],
            output='screen',
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('config_dir', description='시스템 config 디렉토리 경로'),
        *nodes,
    ])
