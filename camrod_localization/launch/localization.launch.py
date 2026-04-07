#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(adapter_launch),
            launch_arguments={
                "module_namespace": LaunchConfiguration("module_namespace"),
                "params_file": LaunchConfiguration("adapter_param_file"),
                "map_info_file": LaunchConfiguration("map_info_file"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("enable_adapter")),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(filter_launch),
            launch_arguments={
                "module_namespace": LaunchConfiguration("module_namespace"),
                "enable_kimera_bridge": LaunchConfiguration("enable_kimera_bridge"),
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
    ])
