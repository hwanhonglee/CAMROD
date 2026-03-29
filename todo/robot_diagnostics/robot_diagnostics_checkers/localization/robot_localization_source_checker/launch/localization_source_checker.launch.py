"""
Localization Source Checker 운영용 launch 파일

사용법:
  ros2 launch robot_localization_source_checker localization_source_checker.launch.py
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_localization_source_checker'), 'config',
         'localization_source_checker.yaml']
    )

    checker_node = Node(
        package='robot_localization_source_checker',
        executable='localization_source_checker_node',
        name='localization_source_checker',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([checker_node])
