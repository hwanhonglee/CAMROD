from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_camera_checker'), 'config', 'camera_checker.yaml']
    )

    return LaunchDescription([
        Node(
            package='robot_camera_checker',
            executable='camera_checker_node',
            name='camera_checker',
            parameters=[config],
            output='screen',
        ),
    ])
