from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_radar_checker'), 'config', 'radar_checker.yaml']
    )

    return LaunchDescription([
        Node(
            package='robot_radar_checker',
            executable='radar_checker_node',
            name='radar_checker',
            parameters=[config],
            output='screen',
        ),
    ])
