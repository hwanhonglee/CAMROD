#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _resolve_vanjee_config_and_presence() -> tuple[str, bool]:
    sensing_share = get_package_share_directory("camrod_sensing")
    default_config = os.path.join(
        sensing_share, "config", "lidar", "vanjee", "config.yaml"
    )

    try:
        get_package_share_directory("vanjee_lidar_sdk")
        return default_config, True
    except Exception:
        return default_config, False


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
        DeclareLaunchArgument("sensing_param_file", default_value=default_sensing_param),
        DeclareLaunchArgument("lidar_preprocess_param_file", default_value=default_lidar_preprocess_param),
        DeclareLaunchArgument("vanjee_config_path", default_value=default_vanjee_config),

        DeclareLaunchArgument("enable_lidar_driver", default_value=enable_driver_default),
        DeclareLaunchArgument("enable_vanjee_static_tf", default_value="false"),

        # relative namespace structure
        DeclareLaunchArgument("module_namespace", default_value="lidar"),
        DeclareLaunchArgument("vanjee_driver_namespace", default_value="vanjee"),

        # preprocessor-relative topics
        DeclareLaunchArgument("preprocessor_input_topic", default_value="vanjee/points_raw"),
        DeclareLaunchArgument("lidar_filtered_topic", default_value="points_filtered"),
        DeclareLaunchArgument("lidar_status_topic", default_value="status"),

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

    module_namespace = LaunchConfiguration("module_namespace")
    vanjee_driver_namespace = LaunchConfiguration("vanjee_driver_namespace")

    preprocessor_input_topic = LaunchConfiguration("preprocessor_input_topic")
    lidar_filtered_topic = LaunchConfiguration("lidar_filtered_topic")
    lidar_status_topic = LaunchConfiguration("lidar_status_topic")
    enable_vanjee_static_tf = LaunchConfiguration("enable_vanjee_static_tf")

    optional_driver_actions = []

    if has_vanjee_driver_pkg:
        vanjee_driver_node = Node(
            package="vanjee_lidar_sdk",
            executable="vanjee_lidar_sdk_node",
            name="vanjee_driver",
            namespace=[module_namespace, "/", vanjee_driver_namespace],
            output="screen",
            parameters=[
                {"config_path": vanjee_config_path}
            ],
            remappings=[
                # SDK source names -> relative targets
                # target "points_raw" becomes:
                #   standalone -> /lidar/vanjee/points_raw
                #   sensing    -> /sensing/lidar/vanjee/points_raw
                ("/vanjee_points722", "points_raw"),
                ("vanjee_points722", "points_raw"),

                ("/vanjee_lidar_imu_packets", "imu_packets"),
                ("vanjee_lidar_imu_packets", "imu_packets"),

                # if SDK is actually publishing these absolute names internally
                ("/lidar/vanjee/points_raw", "points_raw"),
                ("/lidar/vanjee/imu_packets", "imu_packets"),
            ],
            condition=IfCondition(enable_lidar_driver),
        )
        optional_driver_actions.append(vanjee_driver_node)
    else:
        optional_driver_actions.append(
            LogInfo(
                msg="[lidar.launch] package 'vanjee_lidar_sdk' not found; skipping raw driver node."
            )
        )

    lidar_preprocessor_node = Node(
        package="camrod_sensing",
        executable="lidar_preprocessor_node",
        name="lidar_preprocessor",
        namespace=module_namespace,
        output="screen",
        parameters=[
            sensing_param_file,
            lidar_preprocess_param_file,
            {
                "input_topic": preprocessor_input_topic,
                "filtered_topic": lidar_filtered_topic,
                "lidar_status_topic": lidar_status_topic,
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
                PythonExpression([
                    "'",
                    enable_lidar_driver,
                    "' == 'true' and '",
                    enable_vanjee_static_tf,
                    "' == 'true'"
                ])
            ),
        )
        optional_driver_actions.append(vanjee_static_tf)

    return LaunchDescription(
        declare_args + [lidar_preprocessor_node] + optional_driver_actions
    )