"""
Localization Init Checker 운영용 launch 파일

사용법:
  ros2 launch robot_localization_init_checker localization_init_checker.launch.py
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_localization_init_checker'), 'config',
         'localization_init_checker.yaml']
    )

    checker_node = Node(
        package='robot_localization_init_checker',
        executable='localization_init_checker_node',
        name='localization_init_checker',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([checker_node])
