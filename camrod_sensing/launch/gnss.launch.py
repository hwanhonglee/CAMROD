#!/usr/bin/env python3
# HH_260611: GNSS launch now uses ublox_gps_node for both SparkFun single-antenna
# and simpleRTK2B Heading dual-antenna modes. The old dGNSS fallback code was removed.
# Python ntrip_ros.py remains the only NTRIP client because it provides GGA feedback
# required for VRS/MAC networks like gnssdata.or.kr.
#
# HH_260722 - Use the hardware-verified correction ownership shown below as the
# production dual-antenna default:
#
#   NTRIP CORS
#       -> /gnss/ntrip_client/rtcm
#       -> moving_base_rtcm_writer
#       -> FTDI DN03DF8V by-id (POWER+XBEE / Lite moving-base UART1)
#       -> Lite solves absolute RTK and emits 4072.0 + MSM4 + 1230
#       -> board UART/XBee link
#       -> /dev/ttyACM0 receiver UART2 (Budget heading rover)
#       -> rover USB publishes NAV-PVT and NAV-RELPOSNED to ROS
#
# Do not connect the CORS topic directly to the heading rover in the field
# topology.  Both the CORS caster and moving base currently use RTCM station ID
# zero; mixing their independent reference/MSM streams on the rover invalidated
# RELPOSNED in the A/B test.  The `forward_ntrip_to_rover` switch is retained for
# diagnosis only.  It is not the production correction route.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_rtcm_topics(rtcm_topic, dual_antenna, forward_ntrip_to_rover):
    """Return NTRIP publisher and ublox subscriber topics for the selected mode."""
    # HH_260722 - Dual heading uses a dedicated relative NTRIP topic. Enabling
    # forwarding connects ublox to that same topic; disabling it keeps CORS on
    # the moving-base cascade without changing single-antenna behavior.
    ntrip_rtcm_topic = "ntrip_client/rtcm" if dual_antenna else rtcm_topic
    ublox_rtcm_topic = (
        ntrip_rtcm_topic
        if (not dual_antenna or forward_ntrip_to_rover)
        else rtcm_topic
    )
    return ntrip_rtcm_topic, ublox_rtcm_topic


def _use_base_rtcm_writer(dual_antenna, forward_ntrip_to_rover, device):
    """Select exactly one destination for an external correction stream."""
    # HH_260722 - A direct-rover diagnostic must not silently keep correcting
    # the base as well. That would recreate the two-reference collision this
    # launch is designed to avoid. ``__config__`` means the selected GNSS YAML
    # supplies the writer port; an explicitly empty override disables it.
    return dual_antenna and not forward_ntrip_to_rover and bool(device)


def _dual_antenna_runtime_params(usb_rtcm_in, warm_start_on_startup=False):
    """Build the runtime-only dual-antenna rover parameter overlay."""
    # HH_260722 - UART2 carries the corrected moving-base stream. Direct rover
    # USB RTCM is retained only as a diagnostic fallback.
    return {
        "dual_antenna": True,
        "dual_antenna.configure_usb": True,
        "dual_antenna.usb_rtcm_in": usb_rtcm_in,
        "dual_antenna.configure_navigation": False,
        # HH_260722 - Keep the no-reset startup path by default because an
        # automatic reset can discard an already-fixed carrier solution. Expose
        # one explicit recovery switch for the rare case where a direct-CORS
        # diagnostic has left the rover tracking the wrong reference.
        "dual_antenna.warm_start_on_startup": warm_start_on_startup,
        "dual_antenna.warm_start_wait_ms": 12000,
        # HH_260722 - The field moving-base solution is accepted at 1 Hz.
        # HH_260806 - This repository's dual-rover setup writes CFG-RATE to RAM
        # even when config_on_startup=false, so a receiver-saved 5 Hz profile is
        # intentionally overridden here. Keep base and rover epochs at the same
        # measured rate before changing this field contract.
        "rate": 1.0,
        "nav_rate": 1,
        "publish.nav.cov": True,
        "publish.nav.status": True,
        "publish.nav.relposned": True,
        "publish.nav.heading": True,
        "publish.rxm.rtcm": True,
        "publish.nmea": False,
        # HH_260722 - Production sends no ROS RTCM to the rover. Retain this
        # 4072 filter only for the isolated direct-rover diagnostic path.
        "dual_antenna.block_rtcm_ids": [4072],
    }


def _launch_setup(context, *args, **kwargs):
    gnss_namespace = context.perform_substitution(LaunchConfiguration("gnss_namespace"))
    rtcm_topic     = context.perform_substitution(LaunchConfiguration("rtcm_topic"))
    gnss_log_level = context.perform_substitution(LaunchConfiguration("gnss_log_level"))
    enable_ntrip   = context.perform_substitution(LaunchConfiguration("enable_ntrip")).strip().lower() in {
        "1", "true", "yes", "on"
    }
    ntrip_param_file = context.perform_substitution(LaunchConfiguration("ntrip_param_file"))
    base_rtcm_device = context.perform_substitution(
        LaunchConfiguration("ublox_dual_base_rtcm_device")
    ).strip()
    base_rtcm_baud = context.perform_substitution(
        LaunchConfiguration("ublox_dual_base_rtcm_baud")
    ).strip()

    ublox_param_file = context.perform_substitution(LaunchConfiguration("ublox_param_file"))
    ublox_dual_antenna = context.perform_substitution(
        LaunchConfiguration("ublox_dual_antenna")
    ).strip().lower() in {"1", "true", "yes", "on"}
    ublox_dual_forward_ntrip_to_rover = context.perform_substitution(
        LaunchConfiguration("ublox_dual_forward_ntrip_to_rover")
    ).strip().lower() in {"1", "true", "yes", "on"}
    ublox_dual_warm_start_on_startup = context.perform_substitution(
        LaunchConfiguration("ublox_dual_warm_start_on_startup")
    ).strip().lower() in {"1", "true", "yes", "on"}
    ntrip_rtcm_topic, ublox_rtcm_topic = _resolve_rtcm_topics(
        rtcm_topic,
        ublox_dual_antenna,
        ublox_dual_forward_ntrip_to_rover,
    )

    ublox_inline_params = {"rtcm_topic": ublox_rtcm_topic}
    if ublox_dual_antenna:
        ublox_inline_params.update(
            _dual_antenna_runtime_params(
                ublox_dual_forward_ntrip_to_rover,
                ublox_dual_warm_start_on_startup,
            )
        )

    nodes = [
        Node(
            package="ublox_gps",
            executable="ublox_gps_node",
            name="ublox_gps_node",
            namespace=gnss_namespace,
            output="log",
            # HH_260702 - Keep GNSS driver chatter out of the field console;
            # /system/status carries the operator-facing GNSS health.
            arguments=["--ros-args", "--log-level", gnss_log_level],
            # HH_260408: Disable auto-respawn for clean Ctrl+C and no duplicate nodes.
            parameters=[ublox_param_file, ublox_inline_params],
            remappings=[
                ("rtcm",  ublox_rtcm_topic),
                ("/rtcm", ublox_rtcm_topic),
            ],
        ),
    ]
    if enable_ntrip:
        # HH_260722 - One NTRIP publisher feeds exactly one correction owner in
        # production: the moving-base serial writer. The rover stays subscribed
        # to the isolated `rtcm` topic with USB RTCM input disabled, so UART2
        # remains the rover's only RTCM reference stream.
        nodes.append(_ntrip_node(gnss_namespace, ntrip_param_file, ntrip_rtcm_topic, gnss_log_level))
        if _use_base_rtcm_writer(
            ublox_dual_antenna,
            ublox_dual_forward_ntrip_to_rover,
            base_rtcm_device,
        ):
            nodes.append(
                _base_rtcm_writer_node(
                    gnss_namespace,
                    ublox_param_file,
                    base_rtcm_device,
                    base_rtcm_baud,
                    ntrip_rtcm_topic,
                    gnss_log_level,
                )
            )
    return nodes


def _base_rtcm_writer_node(
    namespace: str,
    param_file: str,
    device_override: str,
    baud_override: str,
    rtcm_topic: str,
    log_level: str,
) -> Node:
    # HH_260722 - Write CORS RTCM to the moving base instead of the heading
    # rover. Correcting the base first is what improves absolute NAV-PVT without
    # sacrificing the moving-baseline RELPOSNED solution.
    runtime_params = {"rtcm_topic": rtcm_topic}
    # HH_260727 - The normal path takes device/baud from the writer-specific
    # section of zed_f9p_rover.yaml. Keep explicit launch arguments only as
    # diagnostics/field overrides, and apply them after the YAML when present.
    if device_override != "__config__":
        runtime_params["device"] = device_override
    if baud_override != "__config__":
        runtime_params["baud"] = int(baud_override)

    return Node(
        package="camrod_sensing",
        executable="rtcm_serial_writer.py",
        name="moving_base_rtcm_writer",
        namespace=namespace,
        output="log",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            param_file,
            runtime_params,
        ],
    )


def _ntrip_node(namespace: str, param_file: str, rtcm_topic: str, log_level: str) -> Node:
    # HH_260408: Disable auto-respawn for clean Ctrl+C and no duplicate NTRIP nodes.
    return Node(
        package="ntrip_client",
        executable="ntrip_ros.py",
        name="ntrip_client",
        namespace=namespace,
        output="log",
        # HH_260702 - Keep GNSS/NTRIP details in log files; the console should
        # show the system-level health summary instead of per-driver chatter.
        arguments=["--ros-args", "--log-level", log_level],
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
        # HH_260722 - Field mode corrects the moving base and leaves rover USB
        # RTCM off so absolute position and moving-baseline heading stay fixed.
        DeclareLaunchArgument(
            "ublox_dual_forward_ntrip_to_rover",
            # HH_260722 - External reference corrections go to the moving base.
            # That receiver can stay RTK fixed while generating the time-matched
            # 4072.0/MSM stream used by the heading rover. Direct rover injection
            # is retained only as an explicit diagnostic fallback.
            default_value="false",
            description=(
                "Diagnostic fallback that routes NTRIP RTCM to rover USB instead "
                "of the moving base; keep false for field operation"
            ),
        ),
        DeclareLaunchArgument(
            "ublox_dual_warm_start_on_startup",
            # HH_260722 - A normal launch must preserve a healthy fixed solution;
            # enable this only once when recovering from a wrong-reference test.
            default_value="false",
            description=(
                "Warm-reset the heading rover once after startup; use only to "
                "recover from a stale/wrong RTCM reference, then return to false"
            ),
        ),
        DeclareLaunchArgument("ntrip_param_file",    default_value=os.path.join(cfg, "ntrip_client.yaml"),
                              description="Python ntrip_ros.py parameter file"),
        DeclareLaunchArgument(
            "ublox_dual_base_rtcm_device",
            # HH_260727 - The production value belongs to the selected GNSS
            # config. An explicit path remains available as a CLI override.
            default_value="__config__",
            description=(
                "Moving-base serial override; __config__ reads the writer section "
                "of ublox_param_file, empty disables the writer"
            ),
        ),
        DeclareLaunchArgument(
            "ublox_dual_base_rtcm_baud",
            default_value="__config__",
            description=(
                "Moving-base baud override; __config__ reads the writer section "
                "of ublox_param_file"
            ),
        ),
        DeclareLaunchArgument("enable_ntrip",        default_value="true",
                              description="Enable NTRIP client for RTCM corrections"),
        DeclareLaunchArgument("gnss_log_level",      default_value="error",
                              description="ROS log level for GNSS/NTRIP nodes"),
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
