#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sensing_share = get_package_share_directory("camrod_sensing")
    lidar_driver_launch = os.path.join(sensing_share, "launch", "lidar_driver.launch.py")

    default_cost_grid_param = os.path.join(
        sensing_share, "config", "lidar", "cost_grid.yaml"
    )

    lidar_preprocess_param_file = LaunchConfiguration("lidar_preprocess_param_file")
    enable_lidar_driver = LaunchConfiguration("enable_lidar_driver")
    enable_lidar_cost_grid = LaunchConfiguration("enable_lidar_cost_grid")
    vanjee_config_path = LaunchConfiguration("vanjee_config_path")
    enable_vanjee_static_tf = LaunchConfiguration("enable_vanjee_static_tf")

    module_namespace = LaunchConfiguration("module_namespace")
    vanjee_driver_namespace = LaunchConfiguration("vanjee_driver_namespace")
    preprocessor_input_topic = LaunchConfiguration("preprocessor_input_topic")
    lidar_filtered_topic = LaunchConfiguration("lidar_filtered_topic")
    lidar_status_topic = LaunchConfiguration("lidar_status_topic")
    lidar_cost_grid_param_file = LaunchConfiguration("lidar_cost_grid_param_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_preprocess_param_file",
            default_value=os.path.join(sensing_share, "config", "lidar", "preprocessor.yaml"),
        ),
        DeclareLaunchArgument(
            "lidar_cost_grid_param_file",
            default_value=default_cost_grid_param,
        ),
        DeclareLaunchArgument(
            "vanjee_config_path",
            default_value=os.path.join(sensing_share, "config", "lidar", "vanjee", "config.yaml"),
        ),
        DeclareLaunchArgument("enable_lidar_driver", default_value="true"),
        DeclareLaunchArgument("enable_lidar_cost_grid", default_value="true"),
        DeclareLaunchArgument("enable_vanjee_static_tf", default_value="false"),

        DeclareLaunchArgument("module_namespace", default_value="lidar"),
        DeclareLaunchArgument("vanjee_driver_namespace", default_value="vanjee"),
        DeclareLaunchArgument("preprocessor_input_topic", default_value="vanjee/points_raw"),
        DeclareLaunchArgument("lidar_filtered_topic", default_value="points_filtered"),
        DeclareLaunchArgument("lidar_status_topic", default_value="status"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_driver_launch),
            launch_arguments={
                "lidar_preprocess_param_file": lidar_preprocess_param_file,
                "enable_lidar_driver": enable_lidar_driver,
                "vanjee_config_path": vanjee_config_path,
                "enable_vanjee_static_tf": enable_vanjee_static_tf,
                "module_namespace": module_namespace,
                "vanjee_driver_namespace": vanjee_driver_namespace,
                "preprocessor_input_topic": preprocessor_input_topic,
                "lidar_filtered_topic": lidar_filtered_topic,
                "lidar_status_topic": lidar_status_topic,
            }.items(),
        ),

        Node(
            package="camrod_sensing",
            executable="lidar_cost_grid_node",
            name="lidar_cost_grid",
            namespace=module_namespace,
            output="screen",
            parameters=[lidar_cost_grid_param_file],
            condition=IfCondition(enable_lidar_cost_grid),
        ),
    ])
