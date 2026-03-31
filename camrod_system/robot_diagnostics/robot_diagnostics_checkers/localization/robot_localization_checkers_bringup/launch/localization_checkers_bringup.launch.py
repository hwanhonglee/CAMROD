"""
Localization Checkers Bringup

로컬라이제이션 진단 체커 노드를 한번에 실행한다.

사용법:
  ros2 launch robot_localization_checkers_bringup localization_checkers_bringup.launch.py

포함 노드
---------
  - localization_gnss_checker_node    : /sensing/gnss/pose_with_covariance
  - localization_mode_checker_node    : /localization/status
  - localization_pose_checker_node    : /localization/pose_with_covariance
  - localization_init_checker_node    : /localization/initial_match_*
  - localization_source_checker_node  : /localization/pose_source
  - localization_lanelet_checker_node : /localization/lanelet_pose

주의:
  localization_mode_checker 는 avg_msgs 에 의존합니다.
  실행 전 source ~/camload/install/setup.bash 를 실행하세요.
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
            package='robot_localization_gnss_checker',
            executable='localization_gnss_checker_node',
            name='localization_gnss_checker',
            parameters=[_config('robot_localization_gnss_checker',
                                'localization_gnss_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_localization_mode_checker',
            executable='localization_mode_checker_node',
            name='localization_mode_checker',
            parameters=[_config('robot_localization_mode_checker',
                                'localization_mode_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_localization_pose_checker',
            executable='localization_pose_checker_node',
            name='localization_pose_checker',
            parameters=[_config('robot_localization_pose_checker',
                                'localization_pose_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_localization_init_checker',
            executable='localization_init_checker_node',
            name='localization_init_checker',
            parameters=[_config('robot_localization_init_checker',
                                'localization_init_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_localization_source_checker',
            executable='localization_source_checker_node',
            name='localization_source_checker',
            parameters=[_config('robot_localization_source_checker',
                                'localization_source_checker.yaml')],
            output='screen',
        ),
        Node(
            package='robot_localization_lanelet_checker',
            executable='localization_lanelet_checker_node',
            name='localization_lanelet_checker',
            parameters=[_config('robot_localization_lanelet_checker',
                                'localization_lanelet_checker.yaml')],
            output='screen',
        ),
    ]

    return LaunchDescription(nodes)
