#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sensing_share = get_package_share_directory("camrod_sensing")
    radar_sensor_launch = os.path.join(sensing_share, "launch", "radar_sensor.launch.py")

    default_sensor_param = os.path.join(
        sensing_share, "config", "radar", "sen0592_radar.yaml"
    )
    default_cost_grid_param = os.path.join(
        sensing_share, "config", "radar", "cost_grid.yaml"
    )

    enable_radar = LaunchConfiguration("enable_radar")
    enable_radar_dummy_when_disabled = LaunchConfiguration(
        "enable_radar_dummy_when_disabled"
    )
    enable_radar_cost_grid = LaunchConfiguration("enable_radar_cost_grid")
    radar_sensor_param_file = LaunchConfiguration("radar_sensor_param_file")
    radar_cost_grid_param_file = LaunchConfiguration("radar_cost_grid_param_file")
    radar_log_status = LaunchConfiguration("radar_log_status")
    module_namespace = LaunchConfiguration("module_namespace")

    return LaunchDescription([
        DeclareLaunchArgument("enable_radar", default_value="true"),
        # HH_260729 - Keep canonical radar topics alive for hardware-free field
        # tests. The dummy is mutually exclusive with the serial driver and
        # publishes explicit dummy_active status for diagnostics.
        DeclareLaunchArgument(
            "enable_radar_dummy_when_disabled",
            default_value="true",
            description=(
                "Publish no-target radar heartbeat data while enable_radar is false"
            ),
        ),
        DeclareLaunchArgument("enable_radar_cost_grid", default_value="true"),
        # HH_260623 - Standalone radar.launch has no outer /sensing namespace,
        # so default directly to /sensing/radar for downstream consumers.
        DeclareLaunchArgument("module_namespace", default_value="sensing/radar"),
        DeclareLaunchArgument("radar_sensor_param_file", default_value=default_sensor_param),
        DeclareLaunchArgument("radar_cost_grid_param_file", default_value=default_cost_grid_param),
        DeclareLaunchArgument("radar_log_status", default_value="false"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(radar_sensor_launch),
            launch_arguments={
                "radar_params": radar_sensor_param_file,
                "module_namespace": module_namespace,
                "radar_log_status": radar_log_status,
            }.items(),
            condition=IfCondition(enable_radar),
        ),

        Node(
            package="camrod_sensing",
            executable="radar_dummy_publisher.py",
            name="radar_dummy_publisher",
            namespace=module_namespace,
            output="screen",
            parameters=[radar_sensor_param_file],
            # HH_260729 - PythonExpression is used instead of nested launch
            # conditions for ROS 2 Humble compatibility. Never let the dummy
            # and physical serial driver publish the same topics together.
            condition=IfCondition(PythonExpression([
                "'true' if str('", enable_radar,
                "').lower() not in ['1', 'true', 'yes', 'on'] and str('",
                enable_radar_dummy_when_disabled,
                "').lower() in ['1', 'true', 'yes', 'on'] else 'false'",
            ])),
        ),

        Node(
            package="camrod_sensing",
            executable="radar_cost_grid_node",
            name="radar_cost_grid",
            namespace=module_namespace,
            output="log",
            parameters=[radar_cost_grid_param_file],
            condition=IfCondition(enable_radar_cost_grid),
        ),
    ])
