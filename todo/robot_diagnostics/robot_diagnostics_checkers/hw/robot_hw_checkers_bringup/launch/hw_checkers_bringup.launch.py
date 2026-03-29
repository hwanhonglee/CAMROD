"""
HW Checkers Bringup

하드웨어 진단 체커 노드를 한번에 실행한다.

사용법:
  ros2 launch robot_hw_checkers_bringup hw_checkers_bringup.launch.py

포함 노드
---------
  - hw_checker_node      : CPU 사용률 / 메모리 / 디스크 / CPU 온도
  - gpu_checker_node     : GPU 사용률 / 온도
  - network_checker_node : WiFi 연결 / 신호 / 패킷 품질
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
            package='robot_hw_gpu_checker',
            executable='hw_checker_node',
            name='hw_checker',
            parameters=[_config('robot_hw_gpu_checker', 'hw_gpu_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_hw_gpu_checker',
            executable='gpu_checker_node',
            name='gpu_checker',
            parameters=[_config('robot_hw_gpu_checker', 'hw_gpu_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_network_checker',
            executable='network_checker_node',
            name='network_checker',
            parameters=[_config('robot_network_checker', 'network_checker.yaml')],
            output='screen',
        ),
    ]

    return LaunchDescription(nodes)
