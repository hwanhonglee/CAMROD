#!/usr/bin/env python3
# HH_260317-00:00 Launch u-blox F9P (source build from camrod_sensing/external/ublox)
# with normalized sensing namespace/topic conventions.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    pkg_share = get_package_share_directory("camrod_sensing")

    default_ublox_yaml = os.path.join(pkg_share, "config", "gnss", "zed_f9p_rover.yaml")
    default_ntrip_yaml = os.path.join(pkg_share, "config", "gnss", "ntrip_client.yaml")

    args = [
        DeclareLaunchArgument(
            "ublox_param_file",
            default_value=default_ublox_yaml,
            description="ublox_gps parameter file",
        ),
        DeclareLaunchArgument(
            "ntrip_param_file",
            default_value=default_ntrip_yaml,
            description="ntrip_client parameter file",
        ),
        DeclareLaunchArgument(
            "enable_ntrip",
            default_value="false",
            description="Enable NTRIP client for RTCM corrections",
        ),
        DeclareLaunchArgument(
            "gnss_namespace",
            # HH_260317-00:00 Standalone GNSS launch defaults to /gnss/*.
            # sensing.launch.py overrides this to /sensing/gnss/*.
            default_value="gnss",
            description="GNSS stack namespace",
        ),
        DeclareLaunchArgument(
            "navsatfix_topic",
            # HH_260317-00:00 Relative default under gnss namespace:
            #   namespace=gnss + navsatfix -> /gnss/navsatfix
            default_value="navsatfix",
            description="Canonical NavSatFix topic used by localization (relative to gnss namespace)",
        ),
        DeclareLaunchArgument(
            "rtcm_topic",
            # HH_260317-00:00 Relative default under gnss namespace:
            #   namespace=gnss + rtcm -> /gnss/rtcm
            default_value="rtcm",
            description="RTCM correction topic (relative to gnss namespace)",
        ),
    ]

    ublox_param_file = LaunchConfiguration("ublox_param_file")
    ntrip_param_file = LaunchConfiguration("ntrip_param_file")
    enable_ntrip = LaunchConfiguration("enable_ntrip")
    gnss_namespace = LaunchConfiguration("gnss_namespace")
    navsatfix_topic = LaunchConfiguration("navsatfix_topic")
    rtcm_topic = LaunchConfiguration("rtcm_topic")

    ublox_node = Node(
        package="ublox_gps",
        executable="ublox_gps_node",
        name="ublox_gps_node",
        namespace=gnss_namespace,
        output="screen",
        parameters=[ublox_param_file],
        remappings=[
            # HH_260317-00:00 Canonical localization input topic.
            ("fix", navsatfix_topic),
            ("/fix", navsatfix_topic),
            # HH_260317-00:00 Keep correction stream namespaced by sensor model.
            ("rtcm", rtcm_topic),
            ("/rtcm", rtcm_topic),
        ],
    )

    ntrip_node = Node(
        package="ntrip_client",
        executable="ntrip_ros.py",
        name="ntrip_client",
        namespace=gnss_namespace,
        output="screen",
        parameters=[
            ntrip_param_file,
            {
                "rtcm_topic": rtcm_topic,
            },
        ],
        remappings=[
            ("rtcm", rtcm_topic),
            ("/rtcm", rtcm_topic),
            # HH_260317-00:00 NTRIP GGA source aligns with canonical GNSS fix stream.
            ("fix", navsatfix_topic),
            ("/fix", navsatfix_topic),
        ],
        condition=IfCondition(enable_ntrip),
    )

    return LaunchDescription(args + [ublox_node, ntrip_node])
