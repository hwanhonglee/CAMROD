import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('module_namespace', default_value='planning'),
        DeclareLaunchArgument('map_path', default_value=''),
        DeclareLaunchArgument('origin_lat', default_value='0.0'),
        DeclareLaunchArgument('origin_lon', default_value='0.0'),
        DeclareLaunchArgument('origin_alt', default_value='0.0'),
        DeclareLaunchArgument('centerline_input_pose_topic', default_value='/localization/pose'),
        DeclareLaunchArgument(
            'goal_snapper_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'goal_snapper.yaml')),
        ),
        DeclareLaunchArgument(
            'centerline_snapper_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'centerline_snapper.yaml')),
        ),

        Node(
            package='camrod_planning',
            executable='goal_snapper_node',
            name='goal_snapper',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            parameters=[
                LaunchConfiguration('goal_snapper_param_file'),
                {
                    'map_path': LaunchConfiguration('map_path'),
                    'offset_lat': LaunchConfiguration('origin_lat'),
                    'offset_lon': LaunchConfiguration('origin_lon'),
                    'offset_alt': LaunchConfiguration('origin_alt'),
                    'input_goal_topic': '/goal_pose',
                    'output_goal_topic': '/planning/goal_pose_snapped',
                    'output_goal_topic_ros': '/planning/goal_pose_snapped_ros',
                    'use_map_z': False,
                    'flatten_to_ground': False,
                },
            ],
        ),

        Node(
            package='camrod_planning',
            executable='centerline_snapper_node',
            name='centerline_snapper',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            parameters=[
                LaunchConfiguration('centerline_snapper_param_file'),
                {
                    'map_path': LaunchConfiguration('map_path'),
                    'offset_lat': LaunchConfiguration('origin_lat'),
                    'offset_lon': LaunchConfiguration('origin_lon'),
                    'offset_alt': LaunchConfiguration('origin_alt'),
                    'input_pose_topic': LaunchConfiguration('centerline_input_pose_topic'),
                    'output_pose_topic': '/planning/lanelet_pose',
                    'output_pose_topic_ros': '/planning/lanelet_pose_ros',
                },
            ],
        ),
    ])
