"""
Perception Checkers Bringup

퍼셉션 진단 체커 노드를 실행한다.

사용법:
  ros2 launch robot_perception_checkers_bringup perception_checkers_bringup.launch.py

포함 노드
---------
  - perception_obstacle_checker_node : /perception/camera/detections_2d
                                       /perception/obstacles
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
            package='robot_perception_obstacle_checker',
            executable='perception_obstacle_checker_node',
            name='perception_obstacle_checker',
            parameters=[_config('robot_perception_obstacle_checker',
                                'perception_obstacle_checker.yaml')],
            output='screen',
        ),
    ]

    return LaunchDescription(nodes)
