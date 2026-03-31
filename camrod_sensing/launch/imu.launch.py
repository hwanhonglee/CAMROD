#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    sensing_share = get_package_share_directory("camrod_sensing")
    microstrain_share = get_package_share_directory("microstrain_inertial_driver")

    default_params_file = os.path.join(
        sensing_share, "config", "imu", "microstrain_cv7.yaml"
    )

    microstrain_launch_file = os.path.join(
        microstrain_share, "launch", "microstrain_launch.py"
    )

    declare_port = DeclareLaunchArgument(
        "port",
        default_value="/dev/serial/by-id/usb-Lord_Microstrain_Lord_Inertial_Sensor_0000_6286.226900-if00",
        description="Main serial port for the CV7 device.",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Absolute path to the MicroStrain parameter YAML.",
    )

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="imu",
        description="Namespace for standalone IMU launch",
    )

    include_microstrain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(microstrain_launch_file),
        launch_arguments={
            "port": LaunchConfiguration("port"),
            "params_file": LaunchConfiguration("params_file"),
            "namespace": LaunchConfiguration("namespace"),
        }.items(),
    )

    imu_group = GroupAction([
        SetRemap(src="/ekf/status", dst="ekf/status"),
        SetRemap(src="imu/data", dst="data"),
        include_microstrain,
    ])

    return LaunchDescription([
        declare_port,
        declare_params_file,
        declare_namespace,
        imu_group,
    ])
