from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_lidar_checker'), 'config', 'lidar_checker.yaml']
    )

    return LaunchDescription([
        Node(
            package='robot_lidar_checker',
            executable='lidar_checker_node',
            name='lidar_checker',
            parameters=[config],
            output='screen',
        ),
    ])
