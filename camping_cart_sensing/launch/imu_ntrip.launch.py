#!/usr/bin/env python3
"""
imu_ntrip.launch.py

Purpose:
  - Launch MicroStrain GQ7 (GNSS/INS) driver.
  - Optionally launch an NTRIP client that publishes RTCM corrections on /rtcm.
  - The MicroStrain driver (with ntrip_interface_enable=true) subscribes to /rtcm
    and forwards RTCM correction data to the GQ7 via aux_port.

Usage:
  # GQ7 only (no NTRIP)
  ros2 launch camping_cart_sensing imu_ntrip.launch.py use_ntrip:=false

  # GQ7 + NTRIP (recommended for RTK test)
  ros2 launch camping_cart_sensing imu_ntrip.launch.py use_ntrip:=true

Examples:
  ros2 launch camping_cart_sensing imu_ntrip.launch.py microstrain_port:=/dev/ttyACM1
  ros2 launch camping_cart_sensing imu_ntrip.launch.py microstrain_params:=/abs/path/microstrain_gq7.yaml
  ros2 launch camping_cart_sensing imu_ntrip.launch.py ntrip_params:=/abs/path/ntrip_client.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("camping_cart_sensing")
    microstrain_pkg_share = get_package_share_directory("microstrain_inertial_driver")

    # Launch configs
    microstrain_params = LaunchConfiguration("microstrain_params")
    microstrain_port = LaunchConfiguration("microstrain_port")
    use_ntrip = LaunchConfiguration("use_ntrip")
    ntrip_params = LaunchConfiguration("ntrip_params")

    # Arguments
    declare_args = [
        DeclareLaunchArgument(
            "microstrain_params",
            default_value=os.path.join(pkg_share, "config", "microstrain_gq7.yaml"),
            description="ROS 2 parameter YAML for microstrain_inertial_driver (GQ7).",
        ),
        DeclareLaunchArgument(
            "microstrain_port",
            default_value="/dev/ttyACM1",
            description="Main port for the GQ7 (override YAML 'port' if needed).",
        ),
        DeclareLaunchArgument(
            "use_ntrip",
            default_value="true",
            description="If true, start the NTRIP client and feed RTCM corrections.",
        ),
        DeclareLaunchArgument(
            "ntrip_params",
            default_value=os.path.join(pkg_share, "config", "ntrip_client.yaml"),
            description="ROS 2 parameter YAML for ntrip_client.",
        ),
    ]

    # Upstream MicroStrain launch
    microstrain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(microstrain_pkg_share, "launch", "microstrain_launch.py")
        ),
        launch_arguments={
            "port": microstrain_port,
            "params_file": microstrain_params,
        }.items(),
    )

    # NTRIP client
    # - Publishes RTCM on /rtcm
    # - 'fix' input is remapped to the GQ7 LLH topic
    # - Force rtcm_msgs to avoid mavros_msgs dependency issues
    ntrip_node = Node(
        package="ntrip_client",
        executable="ntrip_ros.py",
        name="ntrip_client",
        output="screen",
        parameters=[
            ntrip_params,
            {"rtcm_message_package": "rtcm_msgs"},  # safety override
            {"rtcm_topic": "/rtcm"},                # safety override
        ],
        remappings=[
            ("fix", "/gnss_1/llh_position"),
            # If your NTRIP client version uses NMEA GGA input, you may also add:
            # ("nmea", "/nmea"),
        ],
        condition=IfCondition(use_ntrip),
    )

    return LaunchDescription(declare_args + [microstrain_launch, ntrip_node])
