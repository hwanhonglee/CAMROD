#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    sensing_share = get_package_share_directory("camrod_sensing")

    imu_cv7_launch = os.path.join(sensing_share, "launch", "imu_cv7.launch.py")
    imu_gq7_ntrip_launch = os.path.join(sensing_share, "launch", "imu_gq7_ntrip.launch.py")
    velocity_converter_launch = os.path.join(
        sensing_share, "launch", "platform_velocity_converter.launch.py"
    )

    imu_mode = LaunchConfiguration("imu_mode")
    module_namespace = LaunchConfiguration("module_namespace")
    enable_imu = LaunchConfiguration("enable_imu")

    cv7_params_file = LaunchConfiguration("cv7_params_file")
    cv7_port = LaunchConfiguration("cv7_port")

    gq7_params_file = LaunchConfiguration("gq7_params_file")
    gq7_port = LaunchConfiguration("gq7_port")
    use_ntrip = LaunchConfiguration("use_ntrip")
    ntrip_params_file = LaunchConfiguration("ntrip_params_file")

    velocity_converter_param_file = LaunchConfiguration("velocity_converter_param_file")
    velocity_topic = LaunchConfiguration("velocity_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    output_topic = LaunchConfiguration("output_topic")
    imu_status_topic = LaunchConfiguration("imu_status_topic")

    default_cv7_params = os.path.join(sensing_share, "config", "imu", "microstrain_cv7.yaml")
    default_gq7_params = os.path.join(sensing_share, "config", "imu", "microstrain_gq7.yaml")
    default_ntrip_params = os.path.join(sensing_share, "config", "gnss", "ntrip_client.yaml")
    default_converter_params = os.path.join(
        sensing_share, "config", "imu", "platform_velocity_converter.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("enable_imu", default_value="true"),
        DeclareLaunchArgument("imu_mode", default_value="cv7"),
        DeclareLaunchArgument("module_namespace", default_value="imu"),

        DeclareLaunchArgument("cv7_params_file", default_value=default_cv7_params),
        DeclareLaunchArgument(
            "cv7_port",
            default_value="/dev/serial/by-id/usb-Lord_Microstrain_Lord_Inertial_Sensor_0000_6286.226900-if00",
        ),

        DeclareLaunchArgument("gq7_params_file", default_value=default_gq7_params),
        DeclareLaunchArgument("gq7_port", default_value="/dev/ttyACM1"),
        DeclareLaunchArgument("use_ntrip", default_value="true"),
        DeclareLaunchArgument("ntrip_params_file", default_value=default_ntrip_params),

        DeclareLaunchArgument("velocity_converter_param_file", default_value=default_converter_params),
        DeclareLaunchArgument("velocity_topic", default_value="/platform/status/velocity"),
        DeclareLaunchArgument("imu_topic", default_value="data"),
        DeclareLaunchArgument("output_topic", default_value="twist_with_covariance"),
        DeclareLaunchArgument("imu_status_topic", default_value="status"),

        GroupAction([
            PushRosNamespace(module_namespace),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(imu_cv7_launch),
                launch_arguments={
                    "params_file": cv7_params_file,
                    "port": cv7_port,
                    "namespace": "",
                }.items(),
                condition=IfCondition(
                    PythonExpression([
                        "'",
                        enable_imu,
                        "' == 'true' and '",
                        imu_mode,
                        "' == 'cv7'"
                    ])
                ),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(imu_gq7_ntrip_launch),
                launch_arguments={
                    "microstrain_params": gq7_params_file,
                    "microstrain_port": gq7_port,
                    "use_ntrip": use_ntrip,
                    "ntrip_params": ntrip_params_file,
                }.items(),
                condition=IfCondition(
                    PythonExpression([
                        "'",
                        enable_imu,
                        "' == 'true' and '",
                        imu_mode,
                        "' == 'gq7_ntrip'"
                    ])
                ),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(velocity_converter_launch),
                launch_arguments={
                    "params_file": velocity_converter_param_file,
                    "module_namespace": "",
                    "velocity_topic": velocity_topic,
                    "imu_topic": imu_topic,
                    "output_topic": output_topic,
                    "imu_status_topic": imu_status_topic,
                }.items(),
                condition=IfCondition(enable_imu),
            ),
        ]),
    ])
