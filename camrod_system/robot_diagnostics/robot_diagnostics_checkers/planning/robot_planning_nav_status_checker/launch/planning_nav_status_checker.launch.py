from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_planning_nav_status_checker'), 'config',
         'planning_nav_status_checker.yaml']
    )

    return LaunchDescription([
        Node(
            package='robot_planning_nav_status_checker',
            executable='planning_nav_status_checker_node',
            name='planning_nav_status_checker',
            parameters=[config],
            output='screen',
        ),
    ])
