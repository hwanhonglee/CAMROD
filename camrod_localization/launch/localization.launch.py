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
    localization_share = get_package_share_directory("camrod_localization")
    map_share = get_package_share_directory("camrod_map")

    adapter_launch = os.path.join(localization_share, "launch", "adapter.launch.py")
    filter_launch = os.path.join(localization_share, "launch", "filter.launch.py")
    monitor_launch = os.path.join(localization_share, "launch", "monitor.launch.py")
    map_helper_launch = os.path.join(localization_share, "launch", "map_helper.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument(
            "module_namespace",
            default_value="localization",
            description="Top-level namespace for localization package",
        ),
        DeclareLaunchArgument(
            "enable_adapter",
            default_value="true",
            description="Enable localization input adapter",
        ),
        DeclareLaunchArgument(
            "enable_filter",
            default_value="true",
            description="Enable localization filter nodes",
        ),
        DeclareLaunchArgument(
            "enable_monitor",
            default_value="true",
            description="Enable localization monitor node",
        ),
        DeclareLaunchArgument(
            "enable_map_helper",
            default_value="true",
            description="Enable localization map helper node",
        ),
        DeclareLaunchArgument(
            "enable_kimera_bridge",
            default_value="false",
            description="Enable kimera csv bridge node",
        ),
        DeclareLaunchArgument(
            "adapter_param_file",
            default_value=os.path.join(localization_share, "config", "source", "input_adapter.yaml"),
            description="Input adapter parameter file",
        ),
        DeclareLaunchArgument(
            "filter_type",
            default_value="auto",
            description="Localization filter type: auto|ekf|eskf",
        ),
        DeclareLaunchArgument(
            "use_eskf",
            default_value="true",
            description="Legacy selector used when filter_type=auto",
        ),
        DeclareLaunchArgument(
            "filter_ekf_param_file",
            default_value=os.path.join(localization_share, "config", "filter", "ekf.yaml"),
            description="EKF filter parameter file",
        ),
        DeclareLaunchArgument(
            "filter_eskf_param_file",
            default_value=os.path.join(localization_share, "config", "filter", "eskf.yaml"),
            description="ESKF filter parameter file",
        ),
        DeclareLaunchArgument(
            "filter_pose_selector_param_file",
            default_value=os.path.join(localization_share, "config", "filter", "pose_selector.yaml"),
            description="Pose selector parameter file",
        ),
        DeclareLaunchArgument(
            "filter_kimera_param_file",
            default_value=os.path.join(localization_share, "config", "source", "kimera_bridge.yaml"),
            description="Kimera bridge parameter file",
        ),
        DeclareLaunchArgument(
            "monitor_param_file",
            default_value=os.path.join(localization_share, "config", "filter", "monitor.yaml"),
            description="Monitor parameter file",
        ),
        DeclareLaunchArgument(
            "map_helper_param_file",
            default_value=os.path.join(localization_share, "config", "reference", "map_helper.yaml"),
            description="Map helper parameter file",
        ),
        DeclareLaunchArgument(
            "map_info_file",
            default_value=os.path.join(map_share, "config", "map_info.yaml"),
            description="Shared map/localization reference parameter file",
        ),
        DeclareLaunchArgument(
            "drop_zones_yaml",
            default_value=os.path.join(localization_share, "config", "drop_zones.yaml"),
            description="Drop zones YAML for map_helper",
        ),
        DeclareLaunchArgument(
            "map_path",
            default_value="",
            description="Lanelet map path for map helper",
        ),
        # HH_260409: Bringup-level wheel source overrides.
        DeclareLaunchArgument(
            "wheel_bridge_enable",
            default_value="true",
            description="Enable wheel odometry bridge path",
        ),
        DeclareLaunchArgument(
            "wheel_input_topic",
            # HH_260410: Prefer platform status odometry as primary wheel source.
            default_value="/platform/status/odometry",
            description="Wheel bridge input topic",
        ),
        DeclareLaunchArgument(
            "wheel_input_type",
            default_value="nav_odom",
            description="Wheel bridge input type: twist|avg_odom|nav_odom",
        ),
        DeclareLaunchArgument(
            "wheel_fallback_input_topic",
            # HH_260410: Use /rmp401/odom only as fallback when status topic is stale/missing.
            default_value="/rmp401/odom",
            description="Wheel bridge fallback input topic",
        ),
        DeclareLaunchArgument(
            "wheel_fallback_input_type",
            default_value="nav_odom",
            description="Wheel bridge fallback input type: twist|avg_odom|nav_odom",
        ),
        DeclareLaunchArgument(
            "wheel_primary_timeout_sec",
            default_value="0.7",
            description="Primary wheel input timeout before fallback activation (sec)",
        ),
        DeclareLaunchArgument(
            "wheel_output_topic",
            # HH_260410: Use status namespace for unified wheel odometry output.
            default_value="/platform/status/wheel_odometry",
            description="Unified wheel odometry output topic (avg_msgs/msg/Odometry alias)",
        ),
        DeclareLaunchArgument(
            "wheel_nav_output_topic",
            default_value="/platform/wheel/nav_odometry",
            description="Unified wheel odometry output topic (nav_msgs/msg/Odometry)",
        ),
        DeclareLaunchArgument(
            "ekf_publish_map_to_odom_static_tf",
            default_value="true",
            description="Publish static map->odom TF when EKF mode is selected",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(adapter_launch),
            launch_arguments={
                "module_namespace": LaunchConfiguration("module_namespace"),
                "params_file": LaunchConfiguration("adapter_param_file"),
                "map_info_file": LaunchConfiguration("map_info_file"),
                "wheel_bridge_enable": LaunchConfiguration("wheel_bridge_enable"),
                "wheel_input_topic": LaunchConfiguration("wheel_input_topic"),
                "wheel_input_type": LaunchConfiguration("wheel_input_type"),
                "wheel_fallback_input_topic": LaunchConfiguration("wheel_fallback_input_topic"),
                "wheel_fallback_input_type": LaunchConfiguration("wheel_fallback_input_type"),
                "wheel_primary_timeout_sec": LaunchConfiguration("wheel_primary_timeout_sec"),
                "wheel_output_topic": LaunchConfiguration("wheel_output_topic"),
                "wheel_nav_output_topic": LaunchConfiguration("wheel_nav_output_topic"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("enable_adapter")),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(filter_launch),
            launch_arguments={
                "module_namespace": LaunchConfiguration("module_namespace"),
                "enable_kimera_bridge": LaunchConfiguration("enable_kimera_bridge"),
                "filter_type": LaunchConfiguration("filter_type"),
                "use_eskf": LaunchConfiguration("use_eskf"),
                "ekf_params_file": LaunchConfiguration("filter_ekf_param_file"),
                "eskf_params_file": LaunchConfiguration("filter_eskf_param_file"),
                "pose_selector_params_file": LaunchConfiguration("filter_pose_selector_param_file"),
                "kimera_params_file": LaunchConfiguration("filter_kimera_param_file"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("enable_filter")),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(monitor_launch),
            launch_arguments={
                "module_namespace": LaunchConfiguration("module_namespace"),
                "params_file": LaunchConfiguration("monitor_param_file"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("enable_monitor")),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_helper_launch),
            launch_arguments={
                "module_namespace": LaunchConfiguration("module_namespace"),
                "map_path": LaunchConfiguration("map_path"),
                "params_file": LaunchConfiguration("map_helper_param_file"),
                "map_info_file": LaunchConfiguration("map_info_file"),
                "drop_zones_yaml": LaunchConfiguration("drop_zones_yaml"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("enable_map_helper")),
        ),

        # HH_260410: EKF mode only publishes odom->base by default, so we provide
        # a single map->odom static TF to keep map-based planning/costmap TF tree connected.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="ekf_map_to_odom_static_tf",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("ekf_publish_map_to_odom_static_tf"), "' == 'true' and ((",
                "'", LaunchConfiguration("filter_type"), "' == 'ekf') or ((",
                "'", LaunchConfiguration("filter_type"), "' == 'auto') and (",
                "'", LaunchConfiguration("use_eskf"), "' != 'true')))",
            ])),
        ),
    ])
