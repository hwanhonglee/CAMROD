from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_wheel_odometry_checker'), 'config',
         'wheel_odometry_checker.yaml']
    )

    return LaunchDescription([
        Node(
            package='robot_wheel_odometry_checker',
            executable='wheel_odometry_checker_node',
            name='wheel_odometry_checker',
            parameters=[config],
            output='screen',
        ),
    ])
