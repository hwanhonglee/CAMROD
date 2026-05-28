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
        DeclareLaunchArgument('enable_goal_replanner', default_value='false'),
        DeclareLaunchArgument(
            'goal_replanner_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'goal_replanner.yaml')),
        ),

        Node(
            package='camrod_planning',
            executable='goal_replanner_node',
            name='goal_replanner',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            parameters=[
                LaunchConfiguration('goal_replanner_param_file'),
                # HH_260528: Stop hard-overriding replanner params in launch.
                # Keep runtime behavior controlled by goal_replanner_param_file.
            ],
            condition=IfCondition(LaunchConfiguration('enable_goal_replanner')),
        ),
    ])
