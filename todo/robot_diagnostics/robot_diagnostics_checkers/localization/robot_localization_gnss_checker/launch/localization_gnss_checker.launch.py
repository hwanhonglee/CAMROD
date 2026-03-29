from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_localization_gnss_checker'), 'config',
         'localization_gnss_checker.yaml']
    )

    return LaunchDescription([
        Node(
            package='robot_localization_gnss_checker',
            executable='localization_gnss_checker_node',
            name='localization_gnss_checker',
            parameters=[config],
            output='screen',
        ),
    ])
