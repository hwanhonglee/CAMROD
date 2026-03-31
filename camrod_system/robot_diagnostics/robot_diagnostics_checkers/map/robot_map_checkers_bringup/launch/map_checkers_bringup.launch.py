"""
Map Checkers Bringup

맵 진단 체커 노드를 실행한다.

사용법:
  ros2 launch robot_map_checkers_bringup map_checkers_bringup.launch.py

포함 노드
---------
  - map_cost_grid_checker_node : /map/cost_grid/lanelet (Lanelet 레인 제약 격자)
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
            package='robot_map_cost_grid_checker',
            executable='map_cost_grid_checker_node',
            name='map_cost_grid_checker',
            parameters=[_config('robot_map_cost_grid_checker',
                                'map_cost_grid_checker.yaml')],
            output='screen',
        ),
    ]

    return LaunchDescription(nodes)
