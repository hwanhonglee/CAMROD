"""Hardware checker 노드들을 config_dir 기반으로 실행한다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = LaunchConfiguration('config_dir')

    def cfg(filename):
        return PathJoinSubstitution([config_dir, 'hw', filename])

    nodes = [
        Node(
            package='robot_hw_gpu_checker',
            executable='hw_checker_node',
            name='hw_checker',
            parameters=[cfg('hw_gpu_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_hw_gpu_checker',
            executable='gpu_checker_node',
            name='gpu_checker',
            parameters=[cfg('hw_gpu_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_network_checker',
            executable='network_checker_node',
            name='network_checker',
            parameters=[cfg('network_checker.yaml')],
            output='screen',
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('config_dir', description='시스템 config 디렉토리 경로'),
        *nodes,
    ])
