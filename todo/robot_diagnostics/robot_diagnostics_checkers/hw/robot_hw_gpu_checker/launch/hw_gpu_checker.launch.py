from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('robot_hw_gpu_checker'), 'config', 'hw_gpu_checker.yaml']
    )

    hw_checker = Node(
        package='robot_hw_gpu_checker',
        executable='hw_checker_node',
        name='hw_checker',
        parameters=[config],
        output='screen',
    )

    gpu_checker = Node(
        package='robot_hw_gpu_checker',
        executable='gpu_checker_node',
        name='gpu_checker',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([hw_checker, gpu_checker])
