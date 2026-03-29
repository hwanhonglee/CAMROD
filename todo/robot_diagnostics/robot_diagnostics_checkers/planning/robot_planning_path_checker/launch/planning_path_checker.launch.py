from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_planning_path_checker'), 'config',
         'planning_path_checker.yaml']
    )

    return LaunchDescription([
        Node(
            package='robot_planning_path_checker',
            executable='planning_path_checker_node',
            name='planning_path_checker',
            parameters=[config],
            output='screen',
        ),
    ])
