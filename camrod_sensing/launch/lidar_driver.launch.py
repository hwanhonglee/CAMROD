#!/usr/bin/env python3

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _resolve_vanjee_config_and_presence() -> tuple[str, bool]:
    sensing_share = get_package_share_directory("camrod_sensing")
    default_config = os.path.join(
        sensing_share, "config", "lidar", "vanjee", "config.yaml"
    )

    try:
        get_package_share_directory("vanjee_lidar_sdk")
        return default_config, True
    except Exception:
        return default_config, False


def _inject_calibration_paths(context, *args, **kwargs):
    """Resolve 750C calibration CSV paths from vanjee_lidar_sdk share and patch config.yaml."""
    try:
        sdk_share = get_package_share_directory("vanjee_lidar_sdk")
        va_csv = os.path.join(sdk_share, "param", "Vanjee_750C_VA.csv")
        ha_csv = os.path.join(sdk_share, "param", "Vanjee_750C_HA.csv")
    except Exception:
        va_csv = ""
        ha_csv = ""

    config_path = context.launch_configurations.get("vanjee_config_path", "")
    if not config_path or not os.path.isfile(config_path):
        return []

    try:
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        driver = cfg.get("lidar", [{}])[0].get("driver", {})
        driver["angle_path_ver"] = va_csv if os.path.isfile(va_csv) else ""
        driver["angle_path_hor"] = ha_csv if os.path.isfile(ha_csv) else ""

        tmp = tempfile.NamedTemporaryFile(
            mode="w", suffix="_vanjee_750.yaml", delete=False, encoding="utf-8"
        )
        yaml.dump(cfg, tmp, allow_unicode=True, default_flow_style=False)
        tmp.close()
        context.launch_configurations["vanjee_config_path"] = tmp.name
    except Exception as exc:
        print(f"[lidar_driver] WARNING: could not patch calibration paths: {exc}")

    return []


def generate_launch_description():
    sensing_share = get_package_share_directory("camrod_sensing")

    default_sensing_param = os.path.join(
        sensing_share, "config", "sensing_params.yaml"
    )
    default_lidar_preprocess_param = os.path.join(
        sensing_share, "config", "lidar", "preprocessor.yaml"
    )
    default_vanjee_config, has_vanjee_driver_pkg = _resolve_vanjee_config_and_presence()
    enable_driver_default = "true" if has_vanjee_driver_pkg else "false"

    declare_args = [
        DeclareLaunchArgument("sensing_param_file", default_value=default_sensing_param),
        DeclareLaunchArgument("lidar_preprocess_param_file", default_value=default_lidar_preprocess_param),
        DeclareLaunchArgument("vanjee_config_path", default_value=default_vanjee_config),

        DeclareLaunchArgument("enable_lidar_driver", default_value=enable_driver_default),
        DeclareLaunchArgument("enable_vanjee_static_tf", default_value="false"),

        DeclareLaunchArgument("module_namespace", default_value="lidar"),
        DeclareLaunchArgument("vanjee_driver_namespace", default_value="vanjee"),

        DeclareLaunchArgument("preprocessor_input_topic", default_value="vanjee/points_raw"),
        DeclareLaunchArgument("lidar_filtered_topic", default_value="points_filtered"),
        DeclareLaunchArgument("lidar_status_topic", default_value="status"),

        DeclareLaunchArgument("vanjee_tf_x", default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_y", default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_z", default_value="0.9"),
        DeclareLaunchArgument("vanjee_tf_roll", default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_pitch", default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_yaw", default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_parent", default_value="robot_base_link"),
        DeclareLaunchArgument("vanjee_tf_child", default_value="vanjee_lidar"),
    ]

    sensing_param_file = LaunchConfiguration("sensing_param_file")
    lidar_preprocess_param_file = LaunchConfiguration("lidar_preprocess_param_file")
    enable_lidar_driver = LaunchConfiguration("enable_lidar_driver")
    vanjee_config_path = LaunchConfiguration("vanjee_config_path")

    module_namespace = LaunchConfiguration("module_namespace")
    vanjee_driver_namespace = LaunchConfiguration("vanjee_driver_namespace")

    preprocessor_input_topic = LaunchConfiguration("preprocessor_input_topic")
    lidar_filtered_topic = LaunchConfiguration("lidar_filtered_topic")
    lidar_status_topic = LaunchConfiguration("lidar_status_topic")
    enable_vanjee_static_tf = LaunchConfiguration("enable_vanjee_static_tf")

    optional_driver_actions = []

    if has_vanjee_driver_pkg:
        vanjee_driver_node = Node(
            package="vanjee_lidar_sdk",
            executable="vanjee_lidar_sdk_node",
            name="vanjee_driver",
            namespace=[module_namespace, "/", vanjee_driver_namespace],
            output="screen",
            parameters=[
                {"config_path": vanjee_config_path}
            ],
            remappings=[
                # WLR-750 SDK publishes on /vanjee_points750
                ("/vanjee_points750", "points_raw"),
                ("vanjee_points750", "points_raw"),

                ("/vanjee_lidar_imu_packets", "imu_packets"),
                ("vanjee_lidar_imu_packets", "imu_packets"),
            ],
            condition=IfCondition(enable_lidar_driver),
        )
        optional_driver_actions.append(vanjee_driver_node)
    else:
        optional_driver_actions.append(
            LogInfo(
                msg="[lidar.launch] package 'vanjee_lidar_sdk' not found; skipping raw driver node."
            )
        )

    lidar_preprocessor_node = Node(
        package="camrod_sensing",
        executable="lidar_preprocessor_node",
        name="lidar_preprocessor",
        namespace=module_namespace,
        output="screen",
        parameters=[
            sensing_param_file,
            lidar_preprocess_param_file,
            {
                "input_topic": preprocessor_input_topic,
                "filtered_topic": lidar_filtered_topic,
                "lidar_status_topic": lidar_status_topic,
            },
        ],
    )

    if has_vanjee_driver_pkg:
        vanjee_static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_to_vanjee",
            arguments=[
                "--x", LaunchConfiguration("vanjee_tf_x"),
                "--y", LaunchConfiguration("vanjee_tf_y"),
                "--z", LaunchConfiguration("vanjee_tf_z"),
                "--roll", LaunchConfiguration("vanjee_tf_roll"),
                "--pitch", LaunchConfiguration("vanjee_tf_pitch"),
                "--yaw", LaunchConfiguration("vanjee_tf_yaw"),
                "--frame-id", LaunchConfiguration("vanjee_tf_parent"),
                "--child-frame-id", LaunchConfiguration("vanjee_tf_child"),
            ],
            output="screen",
            condition=IfCondition(
                PythonExpression([
                    "'",
                    enable_lidar_driver,
                    "' == 'true' and '",
                    enable_vanjee_static_tf,
                    "' == 'true'"
                ])
            ),
        )
        optional_driver_actions.append(vanjee_static_tf)

    return LaunchDescription(
        declare_args + [
            # Patch calibration paths into config before driver node reads it
            OpaqueFunction(function=_inject_calibration_paths),
            lidar_preprocessor_node,
        ] + optional_driver_actions
    )
