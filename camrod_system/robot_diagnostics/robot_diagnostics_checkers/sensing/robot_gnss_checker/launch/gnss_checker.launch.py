from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_gnss_checker'), 'config', 'gnss_checker.yaml']
    )

    return LaunchDescription([
        Node(
            package='robot_gnss_checker',
            executable='gnss_checker_node',
            name='gnss_checker',
            parameters=[config],
            output='screen',
        ),
    ])
