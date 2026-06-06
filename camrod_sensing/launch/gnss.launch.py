#!/usr/bin/env python3
# HH_260604: Unified GNSS launch — selects ublox (single antenna) or ublox_dgnss (dual antenna)
# based on gnss_driver argument. All config loaded from camrod_sensing/config/gnss/.
# HH_260606 // Keep ublox/ublox_dgnss branching inside this single launch file.
# Both modes share Python ntrip_ros.py for NTRIP: supports GGA feedback (required for VRS/MAC
# networks like gnssdata.or.kr) and configurable reconnect logic.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("camrod_sensing")

    gnss_driver    = context.perform_substitution(LaunchConfiguration("gnss_driver"))
    gnss_namespace = context.perform_substitution(LaunchConfiguration("gnss_namespace"))
    rtcm_topic     = context.perform_substitution(LaunchConfiguration("rtcm_topic"))
    enable_ntrip   = context.perform_substitution(LaunchConfiguration("enable_ntrip")).strip().lower() in {
        "1", "true", "yes", "on"
    }
    ntrip_param_file = context.perform_substitution(LaunchConfiguration("ntrip_param_file"))

    # ── Single antenna: ublox_gps + ntrip_client (Python) ────────────────────
    if gnss_driver == "ublox":
        ublox_param_file = context.perform_substitution(LaunchConfiguration("ublox_param_file"))

        nodes = [
            Node(
                package="ublox_gps",
                executable="ublox_gps_node",
                name="ublox_gps_node",
                namespace=gnss_namespace,
                output="screen",
                # HH_260408: Disable auto-respawn for clean Ctrl+C and no duplicate nodes.
                parameters=[ublox_param_file],
                remappings=[
                    ("rtcm",  rtcm_topic),
                    ("/rtcm", rtcm_topic),
                ],
            ),
        ]
        if enable_ntrip:
            nodes.append(_ntrip_node(gnss_namespace, ntrip_param_file, rtcm_topic))
        return nodes

    # ── Dual antenna: UbloxDGNSSNode (composable) + ntrip_client (Python) ────
    elif gnss_driver == "ublox_dgnss":
        device_family         = context.perform_substitution(LaunchConfiguration("device_family"))
        device_serial_string  = context.perform_substitution(LaunchConfiguration("device_serial_string"))
        dgnss_rover_param_file = context.perform_substitution(LaunchConfiguration("dgnss_rover_param_file"))

        # HH_260604: Resolve TOML — explicit override or auto-select by device family.
        # HH_260606 // simpleRTK2B Heading defaults to a rover-safe F9P profile.
        # UART2 is included for readback/validation only; launch-time YAML values do
        # not overwrite the moving-base UART2 link configured in u-center.
        dgnss_ubx_config_file = context.perform_substitution(LaunchConfiguration("dgnss_ubx_config_file"))
        if not dgnss_ubx_config_file or dgnss_ubx_config_file == "__auto__":
            toml_name = "simplertk2b_heading_rover_ubx_config.toml" if device_family.upper() == "F9P" else f"{device_family.lower()}_ubx_config.toml"
            dgnss_ubx_config_file = os.path.join(
                pkg_share, "config", "gnss",
                toml_name,
            )

        # Hardware params from YAML; dynamic fields (device, TOML path) appended inline.
        rover_params = [
            dgnss_rover_param_file,
            {"DEVICE_FAMILY":        device_family},
            {"DEVICE_SERIAL_STRING": device_serial_string},
            {"UBX_CONFIG_FILE":      dgnss_ubx_config_file},
        ]

        # HH_260604: Use relative topic 'ntrip_client/rtcm' so the path resolves correctly
        # under any namespace prefix (gnss.launch standalone or sensing.launch PushRosNamespace).
        # UbloxDGNSSNode subscribes to absolute /ntrip_client/rtcm internally; remapped to
        # relative ntrip_client/rtcm which resolves to /{namespace}/gnss/ntrip_client/rtcm.
        nodes = [
            ComposableNodeContainer(
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
                            ("/ntrip_client/rtcm", "ntrip_client/rtcm"),
                            ("fix",          "ublox_gps_node/fix"),
                            ("fix_velocity", "ublox_gps_node/fix_velocity"),
                        ],
                    ),
                ],
                output="screen",
                respawn=True,
                respawn_delay=2.0,
            ),
        ]

        # HH_260604: Python ntrip_ros.py — provides GGA feedback for VRS/MAC NTRIP networks.
        # Relative rtcm_topic resolves under the node namespace automatically.
        if enable_ntrip:
            nodes.append(
                Node(
                    package="ntrip_client",
                    executable="ntrip_ros.py",
                    name="ntrip_client",
                    namespace=gnss_namespace,
                    output="screen",
                    parameters=[
                        ntrip_param_file,
                        {"rtcm_topic": "ntrip_client/rtcm"},
                    ],
                    remappings=[
                        ("rtcm",  "ntrip_client/rtcm"),
                        ("/rtcm", "ntrip_client/rtcm"),
                        ("fix",   "ublox_gps_node/fix"),
                        ("/fix",  "ublox_gps_node/fix"),
                    ],
                )
            )

        return nodes

    else:
        return [LogInfo(msg=f"[gnss.launch] Unknown gnss_driver: '{gnss_driver}'. Expected 'ublox' or 'ublox_dgnss'.")]


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
        DeclareLaunchArgument("gnss_driver",         default_value="ublox_dgnss",
                              description="GNSS driver: ublox (single) | ublox_dgnss (dual)"),
        DeclareLaunchArgument("ublox_param_file",    default_value=os.path.join(cfg, "zed_f9p_rover.yaml"),
                              description="ublox_gps parameter file (single antenna)"),
        DeclareLaunchArgument("ntrip_param_file",    default_value=os.path.join(cfg, "ntrip_client.yaml"),
                              description="Unified NTRIP parameter file (used by both modes)"),
        DeclareLaunchArgument("dgnss_rover_param_file", default_value=os.path.join(cfg, "ublox_dgnss_rover.yaml"),
                              description="UbloxDGNSSNode hardware config (dual antenna)"),
        DeclareLaunchArgument("dgnss_ubx_config_file",  default_value="__auto__",
                              description="UBX TOML config; '__auto__' selects {device_family}_ubx_config.toml"),
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
        DeclareLaunchArgument("device_family",       default_value="F9P",
                              description="u-blox device family for ublox_dgnss (F9P, F9R, X20P)"),
        DeclareLaunchArgument("device_serial_string", default_value="",
                              description="Serial string to select a specific ublox_dgnss device"),
        OpaqueFunction(function=_launch_setup),
    ])
