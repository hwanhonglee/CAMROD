#!/usr/bin/env python3

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    # HJ_261112 - LiDAR NDT localization (external/ndt_localization_cpp). Its pose enters the EKF as pose2
    # (/localization/ndt/pose_for_ekf, sent only while the NDT match is good, NVTL >= 1.2).
    # Optional package: when it is not built (Autoware NDT apt packages missing), only the NDT include is dropped.
    try:
        _ndt_share = get_package_share_directory("ndt_localization_cpp")
        ndt_launch = os.path.join(_ndt_share, "launch", "ndt_localization_camrod.launch.py")
        ndt_rviz_launch = os.path.join(_ndt_share, "launch", "ndt_rviz.launch.py")
    except PackageNotFoundError:
        ndt_launch = ndt_rviz_launch = None
    ndt_actions = [] if ndt_launch is None else [
        _inc(ndt_launch,
             condition=IfCondition(LaunchConfiguration("enable_ndt")),
             map_pcd=LaunchConfiguration("ndt_map_pcd"),
             camrod_map_info=LaunchConfiguration("map_info_file"),
             pose_source=LaunchConfiguration("ndt_pose_source"),
        ),
        # HJ_261112 - Separate RViz for the NDT match (PCD map + aligned scan); heavy, so its own process and opt-in.
        _inc(ndt_rviz_launch,
             condition=IfCondition(LaunchConfiguration("enable_ndt_rviz")),
             map_pcd=LaunchConfiguration("ndt_map_pcd"),
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("module_namespace",                 default_value="localization"),
        DeclareLaunchArgument("enable_adapter",                   default_value="true"),
        DeclareLaunchArgument("enable_filter",                    default_value="true"),
        DeclareLaunchArgument("enable_monitor",                   default_value="true"),
        DeclareLaunchArgument("enable_map_helper",                default_value="true"),
        # HJ_261112 - NDT is opt-in; ndt_map_pcd = NDT map in this map frame (map_info_file origin, ENU).
        DeclareLaunchArgument("enable_ndt",                       default_value="false"),
        DeclareLaunchArgument("ndt_map_pcd",                      default_value=""),
        # gnss_ndt (default): NDT initial guess from RTK-good GNSS, else the NDT's own prediction, so the NDT does not
        # depend on the EKF it feeds. odom (initial guess from this EKF) lost the NDT in a 148 s RTK shadow on
        # bag 16_53_00: one failed match -> EKF pulled by float GNSS -> wrong initial guess -> no recovery.
        DeclareLaunchArgument("ndt_pose_source",                  default_value="gnss_ndt"),
        # Separate NDT RViz (map + aligned LiDAR scan), independent of the operator RViz.
        DeclareLaunchArgument("enable_ndt_rviz",                  default_value="false"),
        DeclareLaunchArgument(
            "adapter_param_file", default_value=_lc("config/source/input_adapter.yaml")
        ),
        # HH_260721 - Expose only the robot_localization EKF parameter file.
        DeclareLaunchArgument("filter_ekf_param_file",            default_value=_lc("config/filter/ekf.yaml")),
        DeclareLaunchArgument(
            "filter_pose_selector_param_file",
            default_value=_lc("config/filter/pose_selector.yaml"),
        ),
        DeclareLaunchArgument(
            "filter_gnss_reattach_param_file",
            default_value=_lc("config/filter/gnss_reattach.yaml"),
        ),
        DeclareLaunchArgument("monitor_param_file",               default_value=_lc("config/filter/monitor.yaml")),
        DeclareLaunchArgument(
            "map_helper_param_file", default_value=_lc("config/reference/map_helper.yaml")
        ),
        DeclareLaunchArgument(
            "map_info_file",
            default_value=os.path.join(map_share, "config", "map_info.yaml"),
        ),
        DeclareLaunchArgument("drop_zones_yaml",                  default_value=_lc("config/drop_zones.yaml")),
        DeclareLaunchArgument("map_path",                         default_value=""),
        # HH_260409 - Bringup-level wheel source overrides.
        DeclareLaunchArgument("wheel_bridge_enable",              default_value="true"),
        # HH_260410 - Prefer platform status odometry as primary wheel source.
        DeclareLaunchArgument("wheel_input_topic",                default_value="/platform/status/odometry"),
        # HH_260720 - Use the generated AvgOdometry platform contract by default.
        DeclareLaunchArgument("wheel_input_type",                 default_value="avg_odom"),
        # HH_260410 - Use /rmp401/odom only as fallback when status topic is stale/missing.
        DeclareLaunchArgument("wheel_fallback_input_topic",       default_value="/rmp401/odom"),
        DeclareLaunchArgument("wheel_fallback_input_type",        default_value="nav_odom"),
        DeclareLaunchArgument("wheel_primary_timeout_s",          default_value="0.7"),
        # HH_260720 - Localization owns generated wheel output and its explicit EKF boundary.
        DeclareLaunchArgument("wheel_output_topic",               default_value="/localization/input/wheel_odometry"),
        DeclareLaunchArgument(
            "wheel_nav_output_topic",
            default_value="/localization/input/wheel_odometry_ros",
        ),
        DeclareLaunchArgument("ekf_publish_map_to_odom_static_tf", default_value="true"),

        _inc(adapter_launch,
             "module_namespace",
             "wheel_bridge_enable", "wheel_input_topic", "wheel_input_type",
             "wheel_fallback_input_topic", "wheel_fallback_input_type",
             "wheel_primary_timeout_s",
             "wheel_output_topic", "wheel_nav_output_topic",
             condition=IfCondition(LaunchConfiguration("enable_adapter")),
             params_file=LaunchConfiguration("adapter_param_file"),
             map_info_file=LaunchConfiguration("map_info_file"),
             # HH_260721 - Convert the single EKF odometry output into generated primary poses.
             enable_odometry_to_pose="true",
        ),

        _inc(filter_launch,
             "module_namespace",
             condition=IfCondition(LaunchConfiguration("enable_filter")),
             ekf_params_file=LaunchConfiguration("filter_ekf_param_file"),
             pose_selector_params_file=LaunchConfiguration("filter_pose_selector_param_file"),
             gnss_reattach_params_file=LaunchConfiguration("filter_gnss_reattach_param_file"),
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

        *ndt_actions,

        # HH_260720 - Keep the EKF map-to-odom link explicit and use current tf2 CLI options.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="ekf_map_to_odom_static_tf",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--yaw", "0", "--pitch", "0", "--roll", "0",
                "--frame-id", "map", "--child-frame-id", "odom",
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("ekf_publish_map_to_odom_static_tf")),
        ),
    ])
