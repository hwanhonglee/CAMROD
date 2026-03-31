"""
Platform Checkers Bringup

플랫폼 진단 체커 노드를 실행한다.

사용법:
  ros2 launch robot_platform_checkers_bringup platform_checkers_bringup.launch.py

포함 노드
---------
  - ranger_platform_checker_node : /system_state, /battery_state,
                                   /actuator_state, /odom
                                   (Ranger CAN 통신 / 배터리 / 액추에이터)
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
            package='robot_ranger_platform_checker',
            executable='ranger_platform_checker_node',
            name='ranger_platform_checker',
            parameters=[_config('robot_ranger_platform_checker',
                                'ranger_platform_checker.yaml')],
            output='screen',
        ),
    ]

    return LaunchDescription(nodes)
