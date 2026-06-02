#!/usr/bin/env python3
# Launch ublox_dgnss (dual antenna) GNSS stack with NTRIP corrections.
# NTRIP config is read from ublox_dgnss package's own ntrip_params.yaml.
# UbloxNavSatHpFixNode remaps 'fix' -> 'ublox_gps_node/fix' for downstream compatibility.

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_dgnss = get_package_share_directory("ublox_dgnss")

    # Read NTRIP config from ublox_dgnss package (same as original gnss_rover.launch.py).
    config_path = os.path.join(pkg_dgnss, "config", "ntrip_params.yaml")
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    ntrip_params = config["/ntrip_client"]["ros__parameters"]

    if "NTRIP_USERNAME" in os.environ:
        ntrip_params["username"] = os.environ["NTRIP_USERNAME"]
    if "NTRIP_PASSWORD" in os.environ:
        ntrip_params["password"] = os.environ["NTRIP_PASSWORD"]

    args = [
        DeclareLaunchArgument(
            "gnss_namespace",
            default_value="gnss",
            description="GNSS stack namespace",
        ),
        DeclareLaunchArgument(
            "rtcm_topic",
            default_value="rtcm",
            description="RTCM correction topic (relative to gnss namespace)",
        ),
        DeclareLaunchArgument(
            "enable_ntrip",
            default_value="true",
            description="Enable NTRIP client for RTCM corrections",
        ),
        DeclareLaunchArgument(
            "device_family",
            default_value=TextSubstitution(text="F9P"),
            description="u-blox device family (F9P, F9R, X20P)",
        ),
        DeclareLaunchArgument(
            "device_serial_string",
            default_value="",
            description="Serial string to select a specific device",
        ),
    ]

    gnss_namespace = LaunchConfiguration("gnss_namespace")
    rtcm_topic = LaunchConfiguration("rtcm_topic")
    enable_ntrip = LaunchConfiguration("enable_ntrip")
    device_family = LaunchConfiguration("device_family")
    device_serial_string = LaunchConfiguration("device_serial_string")

    rover_params = [
        {"DEVICE_FAMILY": device_family},
        {"DEVICE_SERIAL_STRING": device_serial_string},
        {"FRAME_ID": "gnss_link"},
        {"CFG_RATE_MEAS": 0xC8},        # 200 ms = 5 Hz
        {"CFG_RATE_NAV": 0x1},
        {"CFG_UART1INPROT_NMEA": False},
        {"CFG_UART1INPROT_RTCM3X": False},
        {"CFG_UART1INPROT_UBX": False},
        {"CFG_UART1OUTPROT_NMEA": False},
        {"CFG_UART1OUTPROT_RTCM3X": False},
        {"CFG_UART1OUTPROT_UBX": False},
        {"CFG-UART2-BAUDRATE": 0x70800},  # 460800
        {"CFG_UART2INPROT_NMEA": False},
        {"CFG_UART2INPROT_RTCM3X": True},
        {"CFG_UART2INPROT_UBX": False},
        {"CFG_UART2OUTPROT_NMEA": False},
        {"CFG_UART2OUTPROT_RTCM3X": False},
        {"CFG_UART2OUTPROT_UBX": False},
        {"CFG_USBINPROT_NMEA": False},
        {"CFG_USBINPROT_RTCM3X": False},
        {"CFG_USBINPROT_UBX": True},
        {"CFG_USBOUTPROT_NMEA": False},
        {"CFG_USBOUTPROT_RTCM3X": False},
        {"CFG_USBOUTPROT_UBX": True},
        {"CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB": 0x1},
        {"CFG_MSGOUT_UBX_NAV_COV_USB": 0x1},
        {"CFG_MSGOUT_UBX_NAV_STATUS_USB": 0x1},
        {"CFG_MSGOUT_UBX_NAV_PVT_USB": 0x1},
        {"CFG_MSGOUT_UBX_NAV_RELPOSNED_USB": 0x1},
    ]

    # UbloxDGNSSNode + UbloxNavSatHpFixNode share the same namespace so the
    # relative topic 'ubx_nav_hp_pos_llh' resolves identically for publisher
    # and subscriber.
    # UbloxNavSatHpFixNode remaps 'fix' -> 'ublox_gps_node/fix' so downstream
    # consumers see the same topic path as the ublox_gps driver.
    container_rover = ComposableNodeContainer(
        name="ublox_dgnss_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", "warn"],
        composable_node_descriptions=[
            ComposableNode(
                package="ublox_dgnss_node",
                plugin="ublox_dgnss::UbloxDGNSSNode",
                name="ublox_dgnss",
                namespace=gnss_namespace,
                parameters=rover_params,
                remappings=[
                    ("/ntrip_client/rtcm", ["/sensing/", gnss_namespace, "/ntrip_client/rtcm"]),
                    ("fix", "ublox_gps_node/fix"),
                    ("fix_velocity", "ublox_gps_node/fix_velocity"),
                ],
            ),
        ],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    container_ntrip = ComposableNodeContainer(
        name="ublox_dgnss_ntrip_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", "warn"],
        composable_node_descriptions=[
            ComposableNode(
                package="ntrip_client_node",
                plugin="ublox_dgnss::NTRIPClientNode",
                name="ntrip_client",
                namespace=gnss_namespace,
                parameters=[ntrip_params],
                remappings=[
                    ("/ntrip_client/rtcm", ["/sensing/", gnss_namespace, "/ntrip_client/rtcm"]),
                ],
            ),
        ],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(enable_ntrip),
    )

    return LaunchDescription(args + [container_rover, container_ntrip])