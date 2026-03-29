from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_localization_mode_checker'), 'config',
         'localization_mode_checker.yaml']
    )

    return LaunchDescription([
        Node(
            package='robot_localization_mode_checker',
            executable='localization_mode_checker_node',
            name='localization_mode_checker',
            parameters=[config],
            output='screen',
        ),
    ])
