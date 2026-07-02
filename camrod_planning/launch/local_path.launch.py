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
        DeclareLaunchArgument(
            'local_path_extractor_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'local_path_extractor.yaml')),
        ),
        DeclareLaunchArgument('local_path_pose_topic', default_value='/localization/pose'),
        DeclareLaunchArgument('local_path_global_path_topic', default_value='/planning/global_path'),
        DeclareLaunchArgument('local_path_fallback_global_path_topic', default_value=''),
        DeclareLaunchArgument('enable_tracking_error', default_value='true'),
        DeclareLaunchArgument('tracking_error_topic', default_value='/planning/ltracking_error'),

        Node(
            package='camrod_planning',
            executable='local_path_extractor_node',
            name='local_path_extractor',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            # HH_260702 - Local path feeds cmd_vel gate and RViz route state;
            # respawn prevents a single crash from hiding avoidance paths.
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                LaunchConfiguration('local_path_extractor_param_file'),
                {
                    # HH_260702 - Local path is always a map-fixed global-path slice.
                    'global_path_topic': LaunchConfiguration('local_path_global_path_topic'),
                    'fallback_global_path_topic': LaunchConfiguration('local_path_fallback_global_path_topic'),
                    'pose_topic': LaunchConfiguration('local_path_pose_topic'),
                    'output_topic': '/planning/local_path',
                },
            ],
        ),

        Node(
            package='camrod_planning',
            executable='path_tracking_error_node',
            name='path_tracking_error',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            # HH_260702 - Tracking error is diagnostic/control feedback; keep
            # it available after transient launch-time or runtime exits.
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'pose_topic': LaunchConfiguration('local_path_pose_topic'),
                'local_path_topic': '/planning/local_path',
                'global_path_topic': LaunchConfiguration('local_path_global_path_topic'),
                'output_topic': LaunchConfiguration('tracking_error_topic'),
                'prefer_local_path': True,
                'pose_timeout_s': 1.0,
                'publish_rate_hz': 15.0,
                'publish_on_input_update': True,
            }],
            condition=IfCondition(LaunchConfiguration('enable_tracking_error')),
        ),
    ])
