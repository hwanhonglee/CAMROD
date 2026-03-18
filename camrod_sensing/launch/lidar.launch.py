#!/usr/bin/env python3
# HH_260311-00:00 Launch LiDAR raw driver (Vanjee SDK) + LiDAR preprocessor pipeline.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# Implements `_resolve_vanjee_config_and_presence` behavior.
def _resolve_vanjee_config_and_presence() -> tuple[str, bool]:
    # HH_260313-00:00 Always prefer project-owned driver config for reproducible behavior.
    sensing_share = get_package_share_directory("camrod_sensing")
    default_config = os.path.join(sensing_share, "config", "lidar", "vanjee", "config.yaml")
    # HH_260313-00:00 Detect only SDK package availability for executable presence.
    try:
        get_package_share_directory("vanjee_lidar_sdk")
        return default_config, True
    except Exception:
        return default_config, False


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    sensing_share = get_package_share_directory("camrod_sensing")
    default_sensing_param = os.path.join(
        sensing_share, "config", "sensing_params.yaml"
    )
    default_lidar_preprocess_param = os.path.join(
        sensing_share, "config", "lidar", "preprocessor.yaml"
    )
    default_vanjee_config, has_vanjee_driver_pkg = _resolve_vanjee_config_and_presence()
    enable_driver_default = "true" if has_vanjee_driver_pkg else "false"

    declare_args = [
        DeclareLaunchArgument(
            "sensing_param_file",
            default_value=default_sensing_param,
            description="Legacy monolithic sensing parameter file (kept for compatibility)",
        ),
        DeclareLaunchArgument(
            "lidar_preprocess_param_file",
            default_value=default_lidar_preprocess_param,
            description="LiDAR preprocessor node parameter file",
        ),
        DeclareLaunchArgument(
            "enable_lidar_driver",
            # HH_260311-00:00 If vanjee_lidar_sdk package is not installed in current overlay,
            # force disabled by default to avoid bringup hard failure.
            default_value=enable_driver_default,
            description="Enable Vanjee LiDAR raw driver node (auto false when vanjee_lidar_sdk is unavailable)",
        ),
        DeclareLaunchArgument(
            "vanjee_config_path",
            default_value=default_vanjee_config,
            description="Vanjee SDK config file path",
        ),
        DeclareLaunchArgument(
            "vanjee_driver_namespace",
            default_value="lidar/vanjee",
            description="Namespace for Vanjee SDK node",
        ),
        DeclareLaunchArgument(
            "module_namespace",
            # HH_260317-00:00 Standalone LiDAR stack defaults to /lidar/*.
            # sensing.launch.py overrides this to /sensing/*.
            default_value="lidar",
            description="Namespace for LiDAR preprocessing nodes",
        ),
        DeclareLaunchArgument(
            "lidar_raw_topic",
            # HH_260317-00:00 Relative default under module namespace:
            #   namespace=lidar + vanjee/points_raw -> /lidar/vanjee/points_raw
            default_value="vanjee/points_raw",
            description="Target raw LiDAR topic name (relative to module namespace)",
        ),
        DeclareLaunchArgument(
            "lidar_filtered_topic",
            default_value="points_filtered",
            description="Filtered LiDAR point cloud topic (relative to module namespace)",
        ),
        DeclareLaunchArgument(
            "lidar_imu_packets_topic",
            default_value="vanjee/imu_packets",
            description="Raw LiDAR IMU packet topic from driver (relative to module namespace)",
        ),
        DeclareLaunchArgument(
            "lidar_diagnostic_topic",
            default_value="diagnostic",
            description="LiDAR diagnostic topic (relative to module namespace)",
        ),
        DeclareLaunchArgument(
            "enable_vanjee_static_tf",
            default_value="false",
            description="Publish static TF robot_base_link -> vanjee_lidar",
        ),
        DeclareLaunchArgument("vanjee_tf_x", default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_y", default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_z", default_value="0.9"),
        DeclareLaunchArgument("vanjee_tf_roll", default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_pitch", default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_yaw", default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_parent", default_value="robot_base_link"),
        DeclareLaunchArgument("vanjee_tf_child", default_value="vanjee_lidar"),
    ]

    sensing_param_file = LaunchConfiguration("sensing_param_file")
    lidar_preprocess_param_file = LaunchConfiguration("lidar_preprocess_param_file")
    enable_lidar_driver = LaunchConfiguration("enable_lidar_driver")
    vanjee_config_path = LaunchConfiguration("vanjee_config_path")
    vanjee_driver_namespace = LaunchConfiguration("vanjee_driver_namespace")
    module_namespace = LaunchConfiguration("module_namespace")
    lidar_raw_topic = LaunchConfiguration("lidar_raw_topic")
    lidar_filtered_topic = LaunchConfiguration("lidar_filtered_topic")
    lidar_imu_packets_topic = LaunchConfiguration("lidar_imu_packets_topic")
    lidar_diagnostic_topic = LaunchConfiguration("lidar_diagnostic_topic")
    enable_vanjee_static_tf = LaunchConfiguration("enable_vanjee_static_tf")

    optional_driver_actions = []
    if has_vanjee_driver_pkg:
        vanjee_driver_node = Node(
            package="vanjee_lidar_sdk",
            executable="vanjee_lidar_sdk_node",
            name="vanjee_driver",
            namespace=vanjee_driver_namespace,
            output="screen",
            parameters=[{"config_path": vanjee_config_path}],
            remappings=[
                # HH_260313-00:00 Normalize raw cloud to requested convention.
                ("/vanjee_points722", lidar_raw_topic),
                ("vanjee_points722", lidar_raw_topic),
                ("/lidar/vanjee/points_raw", lidar_raw_topic),
                ("lidar/vanjee/points_raw", lidar_raw_topic),
                ("/vanjee_lidar_imu_packets", lidar_imu_packets_topic),
                ("vanjee_lidar_imu_packets", lidar_imu_packets_topic),
            ],
            condition=IfCondition(enable_lidar_driver),
        )
        optional_driver_actions.append(vanjee_driver_node)
    else:
        optional_driver_actions.append(
            LogInfo(
                msg=(
                    "[sensing.lidar.launch] package 'vanjee_lidar_sdk' not found; "
                    "skipping raw driver node and keeping lidar_preprocessor only."
                )
            )
        )

    lidar_preprocessor_node = Node(
        package="camrod_sensing",
        executable="lidar_preprocessor_node",
        name="lidar_preprocessor",
        namespace=module_namespace,
        output="screen",
        # HH_260314-00:00 Layered params: legacy monolithic file + lidar-specific override.
        parameters=[
            sensing_param_file,
            lidar_preprocess_param_file,
            {
                "input_topic": lidar_raw_topic,
                "filtered_topic": lidar_filtered_topic,
                "lidar_diagnostic_topic": lidar_diagnostic_topic,
            },
        ],
    )

    if has_vanjee_driver_pkg:
        vanjee_static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_to_vanjee",
            arguments=[
                "--x", LaunchConfiguration("vanjee_tf_x"),
                "--y", LaunchConfiguration("vanjee_tf_y"),
                "--z", LaunchConfiguration("vanjee_tf_z"),
                "--roll", LaunchConfiguration("vanjee_tf_roll"),
                "--pitch", LaunchConfiguration("vanjee_tf_pitch"),
                "--yaw", LaunchConfiguration("vanjee_tf_yaw"),
                "--frame-id", LaunchConfiguration("vanjee_tf_parent"),
                "--child-frame-id", LaunchConfiguration("vanjee_tf_child"),
            ],
            output="screen",
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        enable_lidar_driver,
                        "' == 'true' and '",
                        enable_vanjee_static_tf,
                        "' == 'true'",
                    ]
                )
            ),
        )
        optional_driver_actions.append(vanjee_static_tf)

    return LaunchDescription(
        declare_args
        + [
            lidar_preprocessor_node,
        ]
        + optional_driver_actions
    )
