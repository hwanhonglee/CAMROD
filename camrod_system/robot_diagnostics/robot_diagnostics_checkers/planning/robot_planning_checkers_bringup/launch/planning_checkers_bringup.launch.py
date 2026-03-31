"""
Planning Checkers Bringup

플래닝 진단 체커 노드를 한번에 실행한다.

사용법:
  ros2 launch robot_planning_checkers_bringup planning_checkers_bringup.launch.py

포함 노드
---------
  - planning_lifecycle_checker_node  : Nav2 lifecycle 노드 상태
  - planning_costmap_checker_node    : /planning/global_costmap/costmap
                                       /planning/local_costmap/costmap
  - planning_nav_status_checker_node : /planning/navigate_to_pose/_action/status
  - planning_path_checker_node       : /planning/global_path, /planning/local_path
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
            package='robot_planning_lifecycle_checker',
            executable='planning_lifecycle_checker_node',
            name='planning_lifecycle_checker',
            parameters=[_config('robot_planning_lifecycle_checker',
                                'planning_lifecycle_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_planning_costmap_checker',
            executable='planning_costmap_checker_node',
            name='planning_costmap_checker',
            parameters=[_config('robot_planning_costmap_checker',
                                'planning_costmap_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_planning_nav_status_checker',
            executable='planning_nav_status_checker_node',
            name='planning_nav_status_checker',
            parameters=[_config('robot_planning_nav_status_checker',
                                'planning_nav_status_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_planning_path_checker',
            executable='planning_path_checker_node',
            name='planning_path_checker',
            parameters=[_config('robot_planning_path_checker',
                                'planning_path_checker.yaml')],
            output='screen',
        ),
    ]

    return LaunchDescription(nodes)
