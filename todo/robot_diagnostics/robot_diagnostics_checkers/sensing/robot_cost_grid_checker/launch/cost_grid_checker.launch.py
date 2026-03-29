from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_cost_grid_checker'), 'config', 'cost_grid_checker.yaml']
    )

    return LaunchDescription([
        Node(
            package='robot_cost_grid_checker',
            executable='cost_grid_checker_node',
            name='cost_grid_checker',
            parameters=[config],
            output='screen',
        ),
    ])
