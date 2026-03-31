from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_planning_lifecycle_checker'), 'config',
         'planning_lifecycle_checker.yaml']
    )

    return LaunchDescription([
        Node(
            package='robot_planning_lifecycle_checker',
            executable='planning_lifecycle_checker_node',
            name='planning_lifecycle_checker',
            parameters=[config],
            output='screen',
        ),
    ])
