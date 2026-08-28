"""Relay seven standard CARLA radar clouds to CAMROD range topics."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('camrod_carla_adapter')
    default_params = os.path.join(share, 'config', 'radar_relay.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('role_name', default_value='ego_vehicle'),
        DeclareLaunchArgument(
            'radar_relay_param_file', default_value=default_params
        ),
        Node(
            package='camrod_carla_adapter',
            executable='carla_radar_relay',
            name='carla_radar_relay',
            output='screen',
            parameters=[
                LaunchConfiguration('radar_relay_param_file'),
                {
                    'role_name': LaunchConfiguration('role_name'),
                    'use_sim_time': False,
                },
            ],
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
