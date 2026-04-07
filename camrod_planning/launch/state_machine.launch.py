import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('module_namespace', default_value='planning'),
        DeclareLaunchArgument('enable_state_machine', default_value='false'),
        DeclareLaunchArgument(
            'planning_state_machine_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'planning_state_machine.yaml')),
        ),
        DeclareLaunchArgument(
            'planning_state_machine_keypoints_yaml',
            default_value=pkg_share('camrod_map', os.path.join('config', 'drop_zones.yaml')),
        ),
        DeclareLaunchArgument(
            'planning_state_machine_camping_sites_yaml',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'camping_sites.yaml')),
        ),

        Node(
            package='camrod_planning',
            executable='planning_state_machine_node.py',
            name='planning_state_machine',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            parameters=[
                LaunchConfiguration('planning_state_machine_param_file'),
                {
                    'keypoints_yaml': LaunchConfiguration('planning_state_machine_keypoints_yaml'),
                    'camping_sites_yaml': LaunchConfiguration('planning_state_machine_camping_sites_yaml'),
                },
            ],
            condition=IfCondition(LaunchConfiguration('enable_state_machine')),
        ),
    ])
