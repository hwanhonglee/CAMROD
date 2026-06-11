#!/usr/bin/env python3
# HH_260611: GNSS launch now uses ublox_gps_node for both SparkFun single-antenna
# and simpleRTK2B Heading dual-antenna modes. The legacy dGNSS fallback code was removed.
# Python ntrip_ros.py remains the only NTRIP client because it provides GGA feedback
# required for VRS/MAC networks like gnssdata.or.kr.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    gnss_namespace = context.perform_substitution(LaunchConfiguration("gnss_namespace"))
    rtcm_topic     = context.perform_substitution(LaunchConfiguration("rtcm_topic"))
    enable_ntrip   = context.perform_substitution(LaunchConfiguration("enable_ntrip")).strip().lower() in {
        "1", "true", "yes", "on"
    }
    ntrip_param_file = context.perform_substitution(LaunchConfiguration("ntrip_param_file"))

    ublox_param_file = context.perform_substitution(LaunchConfiguration("ublox_param_file"))
    ublox_dual_antenna = context.perform_substitution(
        LaunchConfiguration("ublox_dual_antenna")
    ).strip().lower() in {"1", "true", "yes", "on"}
    ublox_dual_forward_ntrip_to_rover = context.perform_substitution(
        LaunchConfiguration("ublox_dual_forward_ntrip_to_rover")
    ).strip().lower() in {"1", "true", "yes", "on"}
    # HH_260611: Split NTRIP RTCM from the rover input topic in dual mode so
    # moving-baseline RTCM from UART2 remains the only correction stream used for heading.
    ntrip_rtcm_topic = "ntrip_client/rtcm" if ublox_dual_antenna else rtcm_topic
    ublox_rtcm_topic = (
        ntrip_rtcm_topic
        if (not ublox_dual_antenna or ublox_dual_forward_ntrip_to_rover)
        else rtcm_topic
    )

    ublox_inline_params = {"rtcm_topic": ublox_rtcm_topic}
    if ublox_dual_antenna:
        ublox_inline_params.update({
            "dual_antenna": True,
            # HH_260611: Apply the dGNSS-verified rover I/O profile: UART2 RTCM input,
            # USB UBX output/control, and no second CORS RTCM input on rover USB.
            "dual_antenna.configure_usb": True,
            "dual_antenna.configure_navigation": False,
            # HH_260611: Keep the dGNSS-verified no-reset startup path by default.
            # GNSS warm reset is available as a manual fallback, but automatic reset can
            # drop an already-fixed carrier solution back to float during launch repeats.
            "dual_antenna.warm_start_on_startup": False,
            "dual_antenna.warm_start_wait_ms": 12000,
            "rate": 5.0,
            "nav_rate": 1,
            "publish.nav.cov": True,
            "publish.nav.status": True,
            "publish.nav.relposned": True,
            "publish.nav.heading": True,
            "publish.rxm.rtcm": True,
            "publish.nmea": False,
            "dual_antenna.block_rtcm_ids": [4072],
        })

    nodes = [
        Node(
            package="ublox_gps",
            executable="ublox_gps_node",
            name="ublox_gps_node",
            namespace=gnss_namespace,
            output="screen",
            # HH_260408: Disable auto-respawn for clean Ctrl+C and no duplicate nodes.
            parameters=[ublox_param_file, ublox_inline_params],
            remappings=[
                ("rtcm",  ublox_rtcm_topic),
                ("/rtcm", ublox_rtcm_topic),
            ],
        ),
    ]
    if enable_ntrip:
        nodes.append(_ntrip_node(gnss_namespace, ntrip_param_file, ntrip_rtcm_topic))
    return nodes


def _ntrip_node(namespace: str, param_file: str, rtcm_topic: str) -> Node:
    # HH_260408: Disable auto-respawn for clean Ctrl+C and no duplicate NTRIP nodes.
    return Node(
        package="ntrip_client",
        executable="ntrip_ros.py",
        name="ntrip_client",
        namespace=namespace,
        output="screen",
        parameters=[
            param_file,
            {"rtcm_topic": rtcm_topic},
        ],
        remappings=[
            ("rtcm",  rtcm_topic),
            ("/rtcm", rtcm_topic),
            ("fix",   "ublox_gps_node/fix"),
            ("/fix",  "ublox_gps_node/fix"),
        ],
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("camrod_sensing")
    cfg = os.path.join(pkg_share, "config", "gnss")

    return LaunchDescription([
        DeclareLaunchArgument("ublox_param_file",    default_value=os.path.join(cfg, "zed_f9p_rover.yaml"),
                              description="ublox_gps parameter file"),
        DeclareLaunchArgument("ublox_dual_antenna",  default_value="true",
                              description="Use ublox_gps for simpleRTK2B Heading moving-baseline rover"),
        DeclareLaunchArgument("ublox_dual_forward_ntrip_to_rover", default_value="false",
                              description="Keep false for simpleRTK2B Heading: CORS/NTRIP RTCM must not be a second rover input"),
        DeclareLaunchArgument("ntrip_param_file",    default_value=os.path.join(cfg, "ntrip_client.yaml"),
                              description="Python ntrip_ros.py parameter file"),
        DeclareLaunchArgument("enable_ntrip",        default_value="true",
                              description="Enable NTRIP client for RTCM corrections"),
        DeclareLaunchArgument("gnss_namespace",
                              # HH_260317-00:00 Standalone default /gnss/*; sensing.launch.py overrides to /sensing/gnss/*.
                              default_value="gnss",
                              description="GNSS stack namespace"),
        DeclareLaunchArgument("rtcm_topic",
                              # HH_260317-00:00 Relative default: namespace + rtcm -> /gnss/rtcm.
                              default_value="rtcm",
                              description="RTCM correction topic (relative to gnss namespace)"),
        OpaqueFunction(function=_launch_setup),
    ])
