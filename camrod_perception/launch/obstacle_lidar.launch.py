import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_param = os.path.join(
        get_package_share_directory('camrod_perception'),
        'config',
        'perception_params.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('module_namespace', default_value='perception'),
        DeclareLaunchArgument('perception_param_file', default_value=default_param),
        DeclareLaunchArgument('enable_lidar_obstacle', default_value='true'),

        Node(
            package='camrod_perception',
            executable='obstacle_lidar_node',
            name='obstacle_lidar',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            parameters=[LaunchConfiguration('perception_param_file')],
            condition=IfCondition(LaunchConfiguration('enable_lidar_obstacle')),
        ),
    ])
