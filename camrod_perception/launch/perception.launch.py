# HH_260109 Launch perception obstacle fusion pipeline.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    # 2026-02-02 10:32: Use centralized bringup config (synced from package config).
    bringup_share = get_package_share_directory('camrod_bringup')
    default_param = os.path.join(bringup_share, 'config', 'perception', 'perception_params.yaml')

    # HH_260114 Unique param arg to avoid collisions across includes.
    param_file_arg = DeclareLaunchArgument(
        'perception_param_file',
        default_value=default_param,
        description='Perception parameter file (obstacle fusion)',
    )
    enable_module_checker_arg = DeclareLaunchArgument(
        'enable_module_checker',
        default_value='true',
        description='Enable perception module checker publisher',
    )
    enable_lidar_obstacle_arg = DeclareLaunchArgument(
        'enable_lidar_obstacle',
        default_value='true',
        description='Enable LiDAR DBSCAN obstacle marker node',
    )
    module_namespace_arg = DeclareLaunchArgument(
        'module_namespace',
        default_value='perception',
        description='Namespace for perception module nodes',
    )
    system_namespace_arg = DeclareLaunchArgument(
        'system_namespace',
        default_value='system',
        description='Namespace for system checker nodes',
    )
    param_file = LaunchConfiguration('perception_param_file')
    enable_module_checker = LaunchConfiguration('enable_module_checker')
    enable_lidar_obstacle = LaunchConfiguration('enable_lidar_obstacle')
    module_namespace = LaunchConfiguration('module_namespace')
    system_namespace = LaunchConfiguration('system_namespace')

    # HH_260112 Namespace perception nodes under /perception with short names.
    obstacle_fusion = Node(
        package='camrod_perception',
        executable='obstacle_fusion_node',
        name='obstacle_fusion',
        namespace=module_namespace,
        output='screen',
        parameters=[param_file],
    )

    obstacle_lidar = Node(
        package='camrod_perception',
        executable='obstacle_lidar_node',
        name='obstacle_lidar',
        namespace=module_namespace,
        output='screen',
        parameters=[param_file],
        condition=IfCondition(enable_lidar_obstacle),
    )

    perception_checker = Node(
        package='camrod_system',
        executable='module_checker_node.py',
        name='perception_checker',
        namespace=system_namespace,
        output='screen',
        condition=IfCondition(enable_module_checker),
        parameters=[{
            'module_name': 'perception',
            'required_nodes': ['/perception/obstacle_fusion', '/perception/obstacle_lidar'],
            'required_topics': ['/perception/obstacles', '/perception/lidar/bboxes'],
            'diagnostic_topic': '/diagnostics',
            'status_name': 'perception/checker',
            'check_period_s': 0.5,
            'warn_throttle_sec': 2.0,
            'publish_ok': True,
        }],
    )

    perception_diagnostic = Node(
        package='camrod_perception',
        executable='perception_diagnostic_node.py',
        name='perception_diagnostic',
        namespace=module_namespace,
        output='screen',
        parameters=[{
            # HH_260318-00:00 Module-local diagnostic topic (namespaced).
            'diagnostic_topic': 'diagnostic',
            'publish_period_s': 0.2,
            'stale_timeout_s': 2.0,
            'topic_obstacles': '/perception/obstacles',
            'topic_lidar_bboxes': '/perception/lidar/bboxes',
            'topic_detections': '/perception/camera/detections_2d',
        }],
    )

    return LaunchDescription([
        param_file_arg,
        enable_module_checker_arg,
        enable_lidar_obstacle_arg,
        module_namespace_arg,
        system_namespace_arg,
        obstacle_fusion,
        obstacle_lidar,
        perception_diagnostic,
        perception_checker,
    ])
