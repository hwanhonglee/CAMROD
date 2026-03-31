from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_network_checker'), 'config', 'network_checker.yaml']
    )

    network_checker = Node(
        package='robot_network_checker',
        executable='network_checker_node',
        name='network_checker',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([network_checker])
