#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _inc(path, *through, condition=None, **overrides):
    args = {k: LaunchConfiguration(k) for k in through}
    args.update(overrides)
    kw = {"launch_arguments": args.items()}
    if condition is not None:
        kw["condition"] = condition
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), **kw)


def _truthy(raw: str) -> bool:
    return str(raw).strip().lower() in {"1", "true", "yes", "on"}


def _camera_executable_exists(executable_name: str) -> bool:
    prefix = get_package_prefix("camrod_sensing")
    return os.path.exists(os.path.join(prefix, "lib", "camrod_sensing", executable_name))


def _resolve_camera_enable(context, *args, **kwargs):
    requested_raw = context.perform_substitution(LaunchConfiguration("enable_camera"))
    front_raw = context.perform_substitution(LaunchConfiguration("enable_front_camera"))
    rear_raw = context.perform_substitution(LaunchConfiguration("enable_rear_camera"))
    device_path = context.perform_substitution(LaunchConfiguration("camera_device_path")).strip()

    requested = _truthy(requested_raw)
    front_requested = _truthy(front_raw)
    rear_requested = _truthy(rear_raw)
    has_device = bool(device_path) and os.path.exists(device_path)
    front_executable = _camera_executable_exists("camera_front_publisher_node")
    rear_executable = _camera_executable_exists("camera_rear_publisher_node")
    has_requested_executable = (
        (front_requested and front_executable) or
        (rear_requested and rear_executable)
    )
    # HH_260617: Camera publisher nodes are Jetson-only in the current CMake.
    # Do not include camera.launch.py on x86_64 just because /dev/video0 exists.
    effective = requested and has_device and has_requested_executable

    actions = [
        SetLaunchConfiguration("enable_camera_effective", "true" if effective else "false"),
    ]
    if requested and not has_device:
        actions.append(
            LogInfo(
                msg=(
                    f"[sensing.launch] camera disabled: device not found at '{device_path}'. "
                    "Run without camera pipeline."
                )
            )
        )
    if requested and has_device and not has_requested_executable:
        actions.append(
            LogInfo(
                msg=(
                    "[sensing.launch] camera disabled: requested camera publisher "
                    "executable is not installed in camrod_sensing. Run without camera pipeline."
                )
            )
        )
    return actions


def generate_launch_description():
    sensing_share = get_package_share_directory("camrod_sensing")

    camera_launch      = os.path.join(sensing_share, "launch", "camera.launch.py")
    gnss_launch        = os.path.join(sensing_share, "launch", "gnss.launch.py")
    imu_launch         = os.path.join(sensing_share, "launch", "imu.launch.py")
    lidar_launch  = os.path.join(sensing_share, "launch", "lidar.launch.py")
    radar_launch  = os.path.join(sensing_share, "launch", "radar.launch.py")

    default_paths = {
        "camera_params_file":             os.path.join(sensing_share, "config", "camera", "camera_params.yaml"),
        "gnss_param_file":          os.path.join(sensing_share, "config", "gnss", "zed_f9p_rover.yaml"),
        "ntrip_param_file":         os.path.join(sensing_share, "config", "gnss", "ntrip_client.yaml"),
        "imu_param_file":                 "__model_default__",
        "imu_converter_param_file":       os.path.join(sensing_share, "config", "imu", "platform_velocity_converter.yaml"),
        "ground_seg_param_file":          os.path.join(sensing_share, "config", "lidar", "ground_seg_params.yaml"),
        "lidar_cost_grid_param_file":     os.path.join(sensing_share, "config", "lidar", "cost_grid.yaml"),
        "radar_sensor_param_file":        os.path.join(sensing_share, "config", "radar", "sen0592_radar.yaml"),
        "radar_cost_grid_param_file":     os.path.join(sensing_share, "config", "radar", "cost_grid.yaml"),
        "inflation_cost_grid_param_file": os.path.join(sensing_share, "config", "inflation_cost_grid.yaml"),
        "vanjee_config_path":             os.path.join(sensing_share, "config", "lidar", "vanjee", "config.yaml"),
    }

    sensing_namespace = LaunchConfiguration("sensing_namespace")

    return LaunchDescription([
        DeclareLaunchArgument("sensing_namespace", default_value="sensing"),

        DeclareLaunchArgument("enable_camera",               default_value="true"),
        # HH_260528: Per-camera enable flags for dual econ camera setup.
        DeclareLaunchArgument("enable_front_camera",         default_value="true"),
        DeclareLaunchArgument("enable_rear_camera",          default_value="true"),
        DeclareLaunchArgument("enable_gnss",                 default_value="true"),
        DeclareLaunchArgument("enable_imu",                  default_value="true"),
        DeclareLaunchArgument("enable_radar",                default_value="true"),
        DeclareLaunchArgument("enable_radar_cost_grid",      default_value="true"),
        DeclareLaunchArgument("enable_lidar_driver",         default_value="true"),
        DeclareLaunchArgument("enable_lidar_cost_grid",      default_value="true"),
        DeclareLaunchArgument("enable_inflation_cost_grid",  default_value="true"),
        DeclareLaunchArgument("enable_vanjee_static_tf",     default_value="false"),
        ## HJ_260528
        DeclareLaunchArgument("enable_ntrip", default_value="true"),
        # HH_260611: gnss.launch.py now uses ublox_gps_node only; single vs dual
        # simpleRTK2B Heading behavior is controlled by ublox_dual_antenna.
        DeclareLaunchArgument(
            "ublox_dual_antenna",
            default_value="true",
            description="Use ublox_gps dual-antenna simpleRTK2B Heading mode",
        ),
        DeclareLaunchArgument(
            "ublox_dual_forward_ntrip_to_rover",
            default_value="false",
            description="Forward NTRIP RTCM into ublox_gps dual rover USB input; keep false for moving-baseline heading",
        ),

        # HH_260528: imu_mode → imu_model; cv7_param_file/gq7_param_file → imu_param_file.
        DeclareLaunchArgument("imu_model",                   default_value="cv7",
                              description="IMU model: cv7 | gq7"),

        *[DeclareLaunchArgument(k, default_value=v) for k, v in default_paths.items()],

        DeclareLaunchArgument("camera_device_path", default_value="/dev/video0"),

        DeclareLaunchArgument("gnss_namespace",   default_value="gnss"),
        DeclareLaunchArgument("gnss_rtcm_topic",  default_value="rtcm"),

        DeclareLaunchArgument("imu_velocity_topic",    default_value="/platform/status/velocity"),
        DeclareLaunchArgument("imu_input_topic",       default_value="data"),
        # Keep canonical absolute topic so diagnostics observe one stable velocity-converter stream.
        DeclareLaunchArgument(
            "imu_output_topic",
            default_value="/sensing/platform_velocity_converter/twist_with_covariance",
        ),
        DeclareLaunchArgument("imu_status_topic", default_value="status"),
        SetLaunchConfiguration("enable_camera_effective", "false"),
        OpaqueFunction(function=_resolve_camera_enable),

        GroupAction([
            PushRosNamespace(sensing_namespace),

            _inc(camera_launch,
                 condition=IfCondition(LaunchConfiguration("enable_camera_effective")),
                 camera_params_file=LaunchConfiguration("camera_params_file"),
                 enable_front_camera=LaunchConfiguration("enable_front_camera"),
                 enable_rear_camera=LaunchConfiguration("enable_rear_camera"),
                 # HH_260629: PushRosNamespace("sensing") above already adds /sensing,
                 # so pass the relative 'camera' (camera.launch standalone defaults to
                 # 'sensing/camera'). Avoids /sensing/sensing/camera double prefix.
                 camera_namespace="camera",
            ),

            # HH_260606 // Route all GNSS startup through gnss.launch.py only.
            # HH_260611: dGNSS fallback path was removed; this include always starts ublox_gps_node.
            _inc(gnss_launch,
                 "enable_ntrip",
                 condition=IfCondition(LaunchConfiguration("enable_gnss")),
                 # HH_260611: Preserve the same ublox_gps dual-antenna controls through sensing.launch.py.
                 ublox_dual_antenna=LaunchConfiguration("ublox_dual_antenna"),
                 ublox_dual_forward_ntrip_to_rover=LaunchConfiguration("ublox_dual_forward_ntrip_to_rover"),
                 ublox_param_file=LaunchConfiguration("gnss_param_file"),
                 ntrip_param_file=LaunchConfiguration("ntrip_param_file"),
                 gnss_namespace=LaunchConfiguration("gnss_namespace"),
                 rtcm_topic=LaunchConfiguration("gnss_rtcm_topic"),
            ),

            _inc(imu_launch,
                 "enable_imu", "imu_model",
                 imu_param_file=LaunchConfiguration("imu_param_file"),
                 use_ntrip=LaunchConfiguration("enable_ntrip"),
                 ntrip_param_file=LaunchConfiguration("ntrip_param_file"),
                 velocity_converter_param_file=LaunchConfiguration("imu_converter_param_file"),
                 velocity_topic=LaunchConfiguration("imu_velocity_topic"),
                 imu_topic=LaunchConfiguration("imu_input_topic"),
                 output_topic=LaunchConfiguration("imu_output_topic"),
                 imu_status_topic=LaunchConfiguration("imu_status_topic"),
                 module_namespace="imu",
            ),

            _inc(lidar_launch,
                 "ground_seg_param_file",
                 "lidar_cost_grid_param_file", "vanjee_config_path",
                 "enable_lidar_driver", "enable_lidar_cost_grid", "enable_vanjee_static_tf",
                 module_namespace="lidar",
                 vanjee_driver_namespace="vanjee",
                 preprocessor_input_topic="vanjee/points_raw",
                 preprocessor_output_topic="filtered_cloud",
                 lidar_filtered_topic="points_filtered",
            ),

            _inc(radar_launch,
                 "enable_radar", "enable_radar_cost_grid",
                 "radar_sensor_param_file", "radar_cost_grid_param_file",
                 module_namespace="radar",
            ),

            # HH_260424: inflation_cost_grid fuses lanelet + LiDAR + Radar + global_path into
            #   /planning/cost_grid/inflation. Used by Nav2 local costmap and cmd_vel_gate.
            Node(
                package="camrod_sensing",
                executable="inflation_cost_grid_node",
                name="inflation_cost_grid",
                # HH_260618: Inherit PushRosNamespace(sensing_namespace) once.
                # Setting namespace here again produced /sensing/sensing/inflation_cost_grid
                # and prevented /sensing/inflation_cost_grid YAML params from applying.
                output="screen",
                parameters=[LaunchConfiguration("inflation_cost_grid_param_file")],
                condition=IfCondition(LaunchConfiguration("enable_inflation_cost_grid")),
            ),
        ]),
    ])
