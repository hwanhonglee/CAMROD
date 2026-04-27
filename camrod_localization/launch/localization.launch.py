#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _inc(path, *through, condition=None, **overrides):
    args = {k: LaunchConfiguration(k) for k in through}
    args.update(overrides)
    kw = {"launch_arguments": args.items()}
    if condition is not None:
        kw["condition"] = condition
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), **kw)


def generate_launch_description():
    localization_share = get_package_share_directory("camrod_localization")
    map_share = get_package_share_directory("camrod_map")

    def _lc(rel): return os.path.join(localization_share, rel)

    adapter_launch    = _lc("launch/adapter.launch.py")
    filter_launch     = _lc("launch/filter.launch.py")
    monitor_launch    = _lc("launch/monitor.launch.py")
    map_helper_launch = _lc("launch/map_helper.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("module_namespace",                 default_value="localization"),
        DeclareLaunchArgument("enable_adapter",                   default_value="true"),
        DeclareLaunchArgument("enable_filter",                    default_value="true"),
        DeclareLaunchArgument("enable_monitor",                   default_value="true"),
        DeclareLaunchArgument("enable_map_helper",                default_value="true"),
        DeclareLaunchArgument("adapter_param_file",               default_value=_lc("config/source/input_adapter.yaml")),
        DeclareLaunchArgument("filter_type",                      default_value="auto"),
        DeclareLaunchArgument("use_eskf",                         default_value="true"),
        DeclareLaunchArgument("filter_ekf_param_file",            default_value=_lc("config/filter/ekf.yaml")),
        DeclareLaunchArgument("filter_eskf_param_file",           default_value=_lc("config/filter/eskf.yaml")),
        DeclareLaunchArgument("filter_pose_selector_param_file",  default_value=_lc("config/filter/pose_selector.yaml")),
        DeclareLaunchArgument("monitor_param_file",               default_value=_lc("config/filter/monitor.yaml")),
        DeclareLaunchArgument("map_helper_param_file",            default_value=_lc("config/reference/map_helper.yaml")),
        DeclareLaunchArgument("map_info_file",                    default_value=os.path.join(map_share, "config", "map_info.yaml")),
        DeclareLaunchArgument("drop_zones_yaml",                  default_value=_lc("config/drop_zones.yaml")),
        DeclareLaunchArgument("map_path",                         default_value=""),
        # HH_260409: Bringup-level wheel source overrides.
        DeclareLaunchArgument("wheel_bridge_enable",              default_value="true"),
        # HH_260410: Prefer platform status odometry as primary wheel source.
        DeclareLaunchArgument("wheel_input_topic",                default_value="/platform/status/odometry"),
        DeclareLaunchArgument("wheel_input_type",                 default_value="nav_odom"),
        # HH_260410: Use /rmp401/odom only as fallback when status topic is stale/missing.
        DeclareLaunchArgument("wheel_fallback_input_topic",       default_value="/rmp401/odom"),
        DeclareLaunchArgument("wheel_fallback_input_type",        default_value="nav_odom"),
        DeclareLaunchArgument("wheel_primary_timeout_s",          default_value="0.7"),
        DeclareLaunchArgument("wheel_primary_timeout_sec",        default_value="0.7"),
        # HH_260410: Use status namespace for unified wheel odometry output.
        DeclareLaunchArgument("wheel_output_topic",               default_value="/platform/status/wheel_odometry"),
        DeclareLaunchArgument("wheel_nav_output_topic",           default_value="/platform/wheel/nav_odometry"),
        DeclareLaunchArgument("ekf_publish_map_to_odom_static_tf", default_value="true"),

        _inc(adapter_launch,
             "module_namespace",
             "wheel_bridge_enable", "wheel_input_topic", "wheel_input_type",
             "wheel_fallback_input_topic", "wheel_fallback_input_type",
             "wheel_primary_timeout_s", "wheel_primary_timeout_sec",
             "wheel_output_topic", "wheel_nav_output_topic",
             condition=IfCondition(LaunchConfiguration("enable_adapter")),
             params_file=LaunchConfiguration("adapter_param_file"),
             map_info_file=LaunchConfiguration("map_info_file"),
        ),

        _inc(filter_launch,
             "module_namespace", "filter_type", "use_eskf",
             condition=IfCondition(LaunchConfiguration("enable_filter")),
             ekf_params_file=LaunchConfiguration("filter_ekf_param_file"),
             eskf_params_file=LaunchConfiguration("filter_eskf_param_file"),
             pose_selector_params_file=LaunchConfiguration("filter_pose_selector_param_file"),
        ),

        _inc(monitor_launch,
             "module_namespace",
             condition=IfCondition(LaunchConfiguration("enable_monitor")),
             params_file=LaunchConfiguration("monitor_param_file"),
        ),

        _inc(map_helper_launch,
             "module_namespace", "map_path", "map_info_file", "drop_zones_yaml",
             condition=IfCondition(LaunchConfiguration("enable_map_helper")),
             params_file=LaunchConfiguration("map_helper_param_file"),
        ),

        # HH_260410: EKF mode only publishes odom->base; this static TF keeps the map TF tree connected.
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
