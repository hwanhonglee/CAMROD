# HH_260109 Launch perception obstacle fusion pipeline.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 2026-02-02 10:32: Use centralized bringup config (synced from package config).
    bringup_share = get_package_share_directory('camping_cart_bringup')
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
        description='Enable perception module checker/diagnostics publisher',
    )
    enable_diagnostic_arg = DeclareLaunchArgument(
        'perception_enable_diagnostic',
        default_value='true',
        description='Enable /perception/diagnostic publisher',
    )
    diagnostic_publish_period_arg = DeclareLaunchArgument(
        'perception_diagnostic_publish_period_s',
        default_value='0.2',
        description='Publish period for /perception/diagnostic',
    )
    param_file = LaunchConfiguration('perception_param_file')
    enable_module_checker = LaunchConfiguration('enable_module_checker')
    enable_diagnostic = LaunchConfiguration('perception_enable_diagnostic')
    diagnostic_publish_period = LaunchConfiguration('perception_diagnostic_publish_period_s')

    # HH_260112 Namespace perception nodes under /perception with short names.
    obstacle_fusion = Node(
        package='camping_cart_perception',
        executable='obstacle_fusion_node',
        name='obstacle_fusion',
        namespace='perception',
        output='screen',
        parameters=[param_file],
    )

    perception_checker = Node(
        package='camping_cart_system',
        executable='module_checker_node.py',
        name='perception_checker',
        namespace='system',
        output='screen',
        condition=IfCondition(enable_module_checker),
        parameters=[{
            'module_name': 'perception',
            'required_nodes': ['/perception/obstacle_fusion'],
            'required_topics': ['/perception/obstacles'],
            'health_topic': '/perception/healthchecker',
            'check_period_s': 0.5,
            'warn_throttle_sec': 2.0,
            'publish_ok': True,
        }],
    )

    perception_diagnostic = Node(
        package='camping_cart_perception',
        executable='perception_diagnostic_node.py',
        name='perception_diagnostic',
        namespace='perception',
        output='screen',
        condition=IfCondition(enable_diagnostic),
        parameters=[{
            'msgs_topic': '/perception/messages',
            'diagnostic_topic': '/perception/diagnostic',
            'publish_period_s': diagnostic_publish_period,
        }],
    )

    return LaunchDescription([
        param_file_arg,
        enable_module_checker_arg,
        enable_diagnostic_arg,
        diagnostic_publish_period_arg,
        obstacle_fusion,
        perception_checker,
        perception_diagnostic,
    ])
