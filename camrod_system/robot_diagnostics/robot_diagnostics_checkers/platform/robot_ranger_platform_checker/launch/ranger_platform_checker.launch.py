from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('robot_ranger_platform_checker'),
        'config', 'ranger_platform_checker.yaml'
    ])

    return LaunchDescription([
        Node(
            package='robot_ranger_platform_checker',
            executable='ranger_platform_checker_node',
            name='ranger_platform_checker',
            parameters=[config],
            output='screen',
        ),
    ])
