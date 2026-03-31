from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_velocity_converter_checker'),
         'config', 'velocity_converter_checker.yaml']
    )

    return LaunchDescription([
        Node(
            package='robot_velocity_converter_checker',
            executable='velocity_converter_checker_node',
            name='velocity_converter_checker',
            parameters=[config],
            output='screen',
        ),
    ])
