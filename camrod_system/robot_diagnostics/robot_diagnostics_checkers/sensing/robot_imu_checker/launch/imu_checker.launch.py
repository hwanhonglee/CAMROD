from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_imu_checker'), 'config', 'imu_checker.yaml']
    )

    return LaunchDescription([
        Node(
            package='robot_imu_checker',
            executable='imu_checker_node',
            name='imu_checker',
            parameters=[config],
            output='screen',
        ),
    ])
