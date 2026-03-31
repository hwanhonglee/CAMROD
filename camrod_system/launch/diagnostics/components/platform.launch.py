"""Platform checker 노드들을 config_dir 기반으로 실행한다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = LaunchConfiguration('config_dir')

    def cfg(filename):
        return PathJoinSubstitution([config_dir, 'platform', filename])

    nodes = [
        Node(
            package='camrod_system',
            executable='ranger_platform_checker_node',
            name='ranger_platform_checker',
            parameters=[cfg('ranger_platform_checker.yaml')],
            output='screen',
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('config_dir', description='시스템 config 디렉토리 경로'),
        *nodes,
    ])
