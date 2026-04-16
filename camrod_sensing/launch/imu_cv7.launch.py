#!/usr/bin/env python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    sensing_share = get_package_share_directory("camrod_sensing")
    microstrain_share = get_package_share_directory("microstrain_inertial_driver")
    microstrain_default_params_file = os.path.join(
        microstrain_share, "microstrain_inertial_driver_common", "config", "params.yml"
    )

    default_params_file = os.path.join(
        sensing_share, "config", "imu", "microstrain_cv7.yaml"
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

    # HH_260415: Launch driver node directly with respawn.
    # This recovers from intermittent startup-time serial lock contention
    # ("Device or resource busy") without requiring full bringup restart.
    microstrain_node = Node(
        package="microstrain_inertial_driver",
        executable="microstrain_inertial_driver_node",
        name="microstrain_inertial_driver",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            yaml.safe_load(open(microstrain_default_params_file, "r", encoding="utf-8")),
            LaunchConfiguration("params_file"),
            {"port": LaunchConfiguration("port")},
        ],
        respawn=True,
        respawn_delay=2.0,
    )

    imu_group = GroupAction([
        SetRemap(src="/ekf/status", dst="ekf/status"),
        SetRemap(src="imu/data", dst="data"),
        microstrain_node,
    ])

    return LaunchDescription([
        declare_port,
        declare_params_file,
        declare_namespace,
        imu_group,
    ])
