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
                {
                    'replan_rate_hz': 0.0,
                    'request_timeout_sec': 0.0,
                    'retry_after_failure_sec': 0.8,
                    'immediate_replan_on_goal': True,
                    'immediate_replan_on_start': False,
                    'replan_on_start_change': False,
                    'stop_replan_after_goal_reached': True,
                    'goal_reached_distance_m': 0.8,
                    'pause_when_navigate_active': True,
                    'navigate_status_topic': '/planning/navigate_to_pose/_action/status',
                    'min_request_interval_sec': 0.25,
                    'enable_periodic_replan': False,
                    'publish_result_path': True,
                    'output_path_topic': '/planning/global_path_replanner',
                    'start_topic_fallback_to_tf': True,
                },
            ],
            condition=IfCondition(LaunchConfiguration('enable_goal_replanner')),
        ),
    ])
