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
    # HH_260330: Standalone perception launch uses package-local config by default.
    pkg_share = get_package_share_directory('camrod_perception')
    default_param = os.path.join(pkg_share, 'config', 'perception_params.yaml')

    # HH_260114 Unique param arg to avoid collisions across includes.
    param_file_arg = DeclareLaunchArgument(
        'perception_param_file',
        default_value=default_param,
        description='Perception parameter file (obstacle fusion)',
    )
    enable_module_validator_arg = DeclareLaunchArgument(
        'enable_module_validator',
        default_value='true',
        description='Enable perception module validator publisher',
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
        description='Namespace for system validator nodes',
    )
    param_file = LaunchConfiguration('perception_param_file')
    enable_module_validator = LaunchConfiguration('enable_module_validator')
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

    # HH_260326: Removed perception status/validator runtime nodes as requested.

    return LaunchDescription([
        param_file_arg,
        enable_module_validator_arg,
        enable_lidar_obstacle_arg,
        module_namespace_arg,
        system_namespace_arg,
        obstacle_fusion,
        obstacle_lidar,
    ])
