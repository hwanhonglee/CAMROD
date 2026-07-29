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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


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
    # HH_260617: Camera publisher nodes are Jetson-only in the current CMake.
    # Do not include camera.launch.py on x86_64 just because /dev/video0 exists.
    # HH_260729: Resolve each camera separately.  If only one executable/device
    # path is usable, the other channel must receive an explicit dummy instead
    # of being silently treated as enabled by the aggregate OR condition.
    front_effective = requested and front_requested and has_device and front_executable
    rear_effective = requested and rear_requested and has_device and rear_executable
    effective = front_effective or rear_effective
    missing_requested_executable = (
        (front_requested and not front_executable) or
        (rear_requested and not rear_executable)
    )

    actions = [
        SetLaunchConfiguration("enable_camera_effective", "true" if effective else "false"),
        SetLaunchConfiguration(
            "enable_front_camera_effective", "true" if front_effective else "false"
        ),
        SetLaunchConfiguration(
            "enable_rear_camera_effective", "true" if rear_effective else "false"
        ),
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
    if requested and has_device and missing_requested_executable:
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
        # HH_260729 - One non-enable policy switch covers every physical input.
        # Bringup forces it false in sim, where fake_sensors already owns data.
        DeclareLaunchArgument(
            "publish_sensor_dummies_when_disabled",
            default_value="true",
        ),
        # The composable camera+YOLO container may own the real front camera
        # outside this scoped sensing include; suppress a duplicate front dummy.
        DeclareLaunchArgument("front_camera_source_external", default_value="false"),
        DeclareLaunchArgument("enable_radar_cost_grid",      default_value="true"),
        # HH_260702 - Keep per-radar range details on topics/checkers by default;
        # enable this only for bench debugging so the field console stays concise.
        DeclareLaunchArgument("radar_log_status",            default_value="false"),
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
            # HH_260722 - External corrections belong on the moving base so
            # absolute RTK and the moving-baseline heading improve together.
            default_value="false",
            description=(
                "Diagnostic fallback that routes NTRIP RTCM to rover USB instead "
                "of the moving base; keep false for field operation"
            ),
        ),
        DeclareLaunchArgument(
            "ublox_dual_warm_start_on_startup",
            # HH_260722 - Preserve healthy carrier state on ordinary launches.
            default_value="false",
            description=(
                "One-shot heading-rover warm-reset recovery; keep false during "
                "normal repeated launches"
            ),
        ),
        DeclareLaunchArgument(
            "ublox_dual_base_rtcm_device",
            # HH_260727 - Resolve the normal value from gnss_param_file so this
            # aggregate launch and bringup cannot drift from standalone GNSS.
            default_value="__config__",
            description=(
                "Optional moving-base serial override; __config__ uses "
                "gnss_param_file"
            ),
        ),
        DeclareLaunchArgument(
            "ublox_dual_base_rtcm_baud",
            default_value="__config__",
            description="Optional moving-base baud override; __config__ uses gnss_param_file",
        ),

        # HH_260528: imu_mode → imu_model; cv7_param_file/gq7_param_file → imu_param_file.
        DeclareLaunchArgument("imu_model",                   default_value="cv7",
                              description="IMU model: cv7 | gq7"),

        *[DeclareLaunchArgument(k, default_value=v) for k, v in default_paths.items()],

        DeclareLaunchArgument("camera_device_path", default_value="/dev/video0"),

        DeclareLaunchArgument("gnss_namespace",   default_value="gnss"),
        DeclareLaunchArgument("gnss_rtcm_topic",  default_value="rtcm"),
        DeclareLaunchArgument("gnss_log_level",   default_value="error"),

        DeclareLaunchArgument("imu_velocity_topic",    default_value="/platform/status/velocity"),
        DeclareLaunchArgument("imu_input_topic",       default_value="data_ros"),
        DeclareLaunchArgument("imu_data_output_topic", default_value="data"),
        # Keep canonical absolute topic so diagnostics observe one stable velocity-converter stream.
        DeclareLaunchArgument(
            "imu_output_topic",
            default_value="/sensing/platform_velocity_converter/twist_with_covariance",
        ),
        DeclareLaunchArgument("imu_status_topic", default_value="status"),
        SetLaunchConfiguration("enable_camera_effective", "false"),
        SetLaunchConfiguration("enable_front_camera_effective", "false"),
        SetLaunchConfiguration("enable_rear_camera_effective", "false"),
        OpaqueFunction(function=_resolve_camera_enable),

        GroupAction([
            PushRosNamespace(sensing_namespace),

            # HH_260729: Publish only the disabled physical channels from one
            # low-overhead process.  Each stream carries a fresh dummy_active
            # marker so diagnostics show DUMMY/WARN, never healthy hardware.
            Node(
                package="camrod_sensing",
                executable="sensing_dummy_publisher.py",
                name="sensing_dummy_publisher",
                output="log",
                parameters=[{
                    "publish_rate_hz": 2.0,
                    "publish_gnss": ParameterValue(
                        PythonExpression([
                            "str('", LaunchConfiguration(
                                "publish_sensor_dummies_when_disabled"
                            ), "').lower() in ('1','true','yes','on') and not ",
                            "str('", LaunchConfiguration("enable_gnss"),
                            "').lower() in ('1','true','yes','on')",
                        ]),
                        value_type=bool,
                    ),
                    "publish_imu": ParameterValue(
                        PythonExpression([
                            "str('", LaunchConfiguration(
                                "publish_sensor_dummies_when_disabled"
                            ), "').lower() in ('1','true','yes','on') and not ",
                            "str('", LaunchConfiguration("enable_imu"),
                            "').lower() in ('1','true','yes','on')",
                        ]),
                        value_type=bool,
                    ),
                    "publish_lidar": ParameterValue(
                        PythonExpression([
                            "str('", LaunchConfiguration(
                                "publish_sensor_dummies_when_disabled"
                            ), "').lower() in ('1','true','yes','on') and not ",
                            "str('", LaunchConfiguration("enable_lidar_driver"),
                            "').lower() in ('1','true','yes','on')",
                        ]),
                        value_type=bool,
                    ),
                    "publish_front_camera": ParameterValue(
                        PythonExpression([
                            "str('", LaunchConfiguration(
                                "publish_sensor_dummies_when_disabled"
                            ), "').lower() in ('1','true','yes','on') and not ",
                            "str('", LaunchConfiguration("front_camera_source_external"),
                            "').lower() in ('1','true','yes','on') and not ",
                            "str('", LaunchConfiguration("enable_front_camera_effective"),
                            "').lower() in ('1','true','yes','on')",
                        ]),
                        value_type=bool,
                    ),
                    "publish_rear_camera": ParameterValue(
                        PythonExpression([
                            "str('", LaunchConfiguration(
                                "publish_sensor_dummies_when_disabled"
                            ), "').lower() in ('1','true','yes','on') and not ",
                            "str('", LaunchConfiguration("enable_rear_camera_effective"),
                            "').lower() in ('1','true','yes','on')",
                        ]),
                        value_type=bool,
                    ),
                }],
                condition=IfCondition(PythonExpression([
                    "str('", LaunchConfiguration(
                        "publish_sensor_dummies_when_disabled"
                    ), "').lower() in ('1','true','yes','on') and (",
                    "not str('", LaunchConfiguration("enable_gnss"),
                    "').lower() in ('1','true','yes','on') or ",
                    "not str('", LaunchConfiguration("enable_imu"),
                    "').lower() in ('1','true','yes','on') or ",
                    "not str('", LaunchConfiguration("enable_lidar_driver"),
                    "').lower() in ('1','true','yes','on') or (",
                    "not str('", LaunchConfiguration("front_camera_source_external"),
                    "').lower() in ('1','true','yes','on') and not str('",
                    LaunchConfiguration("enable_front_camera_effective"),
                    "').lower() in ('1','true','yes','on')) or ",
                    "not str('", LaunchConfiguration("enable_rear_camera_effective"),
                    "').lower() in ('1','true','yes','on'))",
                ])),
            ),

            _inc(camera_launch,
                 condition=IfCondition(LaunchConfiguration("enable_camera_effective")),
                 camera_params_file=LaunchConfiguration("camera_params_file"),
                 enable_front_camera=LaunchConfiguration("enable_front_camera_effective"),
                 enable_rear_camera=LaunchConfiguration("enable_rear_camera_effective"),
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
                 # HH_260722 - Preserve both halves of the verified cascade
                 # through the aggregate launch: rover output plus base correction.
                 ublox_dual_antenna=LaunchConfiguration("ublox_dual_antenna"),
                 ublox_dual_forward_ntrip_to_rover=LaunchConfiguration("ublox_dual_forward_ntrip_to_rover"),
                 ublox_dual_warm_start_on_startup=LaunchConfiguration("ublox_dual_warm_start_on_startup"),
                 ublox_dual_base_rtcm_device=LaunchConfiguration("ublox_dual_base_rtcm_device"),
                 ublox_dual_base_rtcm_baud=LaunchConfiguration("ublox_dual_base_rtcm_baud"),
                 ublox_param_file=LaunchConfiguration("gnss_param_file"),
                 ntrip_param_file=LaunchConfiguration("ntrip_param_file"),
                 gnss_namespace=LaunchConfiguration("gnss_namespace"),
                 rtcm_topic=LaunchConfiguration("gnss_rtcm_topic"),
                 gnss_log_level=LaunchConfiguration("gnss_log_level"),
            ),

            _inc(imu_launch,
                 "enable_imu", "imu_model",
                 imu_param_file=LaunchConfiguration("imu_param_file"),
                 use_ntrip=LaunchConfiguration("enable_ntrip"),
                 ntrip_param_file=LaunchConfiguration("ntrip_param_file"),
                 velocity_converter_param_file=LaunchConfiguration("imu_converter_param_file"),
                 velocity_topic=LaunchConfiguration("imu_velocity_topic"),
                 imu_topic=LaunchConfiguration("imu_input_topic"),
                 imu_data_output_topic=LaunchConfiguration("imu_data_output_topic"),
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
                 "enable_radar",
                 "enable_radar_cost_grid",
                 "radar_sensor_param_file", "radar_cost_grid_param_file",
                 "radar_log_status",
                 enable_radar_dummy_when_disabled=LaunchConfiguration(
                     "publish_sensor_dummies_when_disabled"
                 ),
                 module_namespace="radar",
            ),

            # HH_260720 - Feed the merged grid to Nav2 and cmd_vel_safety_gate.
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
