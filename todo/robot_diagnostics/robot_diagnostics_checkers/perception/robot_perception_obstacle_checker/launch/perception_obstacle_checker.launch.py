from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_perception_obstacle_checker'), 'config',
         'perception_obstacle_checker.yaml']
    )

    return LaunchDescription([
        Node(
            package='robot_perception_obstacle_checker',
            executable='perception_obstacle_checker_node',
            name='perception_obstacle_checker',
            parameters=[config],
            output='screen',
        ),
    ])
