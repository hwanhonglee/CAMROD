#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
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


def generate_launch_description():
    sensing_share = get_package_share_directory("camrod_sensing")

    default_ground_seg_param = os.path.join(
        sensing_share, "config", "lidar", "ground_seg_params.yaml"
    )
    default_vanjee_config, has_vanjee_driver_pkg = _resolve_vanjee_config_and_presence()
    enable_driver_default = "true" if has_vanjee_driver_pkg else "false"

    declare_args = [
        DeclareLaunchArgument("ground_seg_param_file",  default_value=default_ground_seg_param),
        DeclareLaunchArgument("vanjee_config_path",     default_value=default_vanjee_config),

        DeclareLaunchArgument("enable_lidar_driver",       default_value=enable_driver_default),
        DeclareLaunchArgument("enable_vanjee_static_tf",   default_value="false"),

        DeclareLaunchArgument("module_namespace",          default_value="lidar"),
        DeclareLaunchArgument("vanjee_driver_namespace",   default_value="vanjee"),

        DeclareLaunchArgument("preprocessor_input_topic",  default_value="vanjee/points_raw"),
        DeclareLaunchArgument("preprocessor_output_topic", default_value="filtered_cloud"),
        DeclareLaunchArgument("lidar_filtered_topic",      default_value="points_filtered"),

        # HJ_260623: LiDAR preprocessor parameters (points_raw -> filtered_cloud)
        DeclareLaunchArgument("enable_lidar_preprocessor",          default_value="true"),
        DeclareLaunchArgument("preprocessor_marker_frame_id",       default_value="lidar_link"),
        DeclareLaunchArgument("preprocessor_angle_filter_deg",      default_value="64.0"),
        DeclareLaunchArgument("preprocessor_roi_x_min",             default_value="0.0"),
        DeclareLaunchArgument("preprocessor_roi_x_max",             default_value="3.0"),
        DeclareLaunchArgument("preprocessor_roi_y_min",             default_value="-1.5"),
        DeclareLaunchArgument("preprocessor_roi_y_max",             default_value="1.5"),
        DeclareLaunchArgument("preprocessor_roi_z_min",             default_value="-1.0"),
        DeclareLaunchArgument("preprocessor_roi_z_max",             default_value="1.0"),
        DeclareLaunchArgument("preprocessor_voxel_leaf_size",       default_value="0.03"),
        DeclareLaunchArgument("preprocessor_qos_depth",             default_value="2"),
        DeclareLaunchArgument("preprocessor_max_process_hz",        default_value="0.0"),

        DeclareLaunchArgument("vanjee_tf_x",       default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_y",       default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_z",       default_value="0.9"),
        DeclareLaunchArgument("vanjee_tf_roll",    default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_pitch",   default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_yaw",     default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_parent",  default_value="robot_center_link"),
        DeclareLaunchArgument("vanjee_tf_child",   default_value="vanjee_lidar"),
    ]

    ground_seg_param_file = LaunchConfiguration("ground_seg_param_file")
    enable_lidar_driver   = LaunchConfiguration("enable_lidar_driver")
    vanjee_config_path    = LaunchConfiguration("vanjee_config_path")

    module_namespace         = LaunchConfiguration("module_namespace")
    vanjee_driver_namespace  = LaunchConfiguration("vanjee_driver_namespace")

    preprocessor_input_topic = LaunchConfiguration("preprocessor_input_topic")
    preprocessor_output_topic = LaunchConfiguration("preprocessor_output_topic")
    lidar_filtered_topic     = LaunchConfiguration("lidar_filtered_topic")
    enable_vanjee_static_tf  = LaunchConfiguration("enable_vanjee_static_tf")

    # HJ_260623: LiDAR preprocessor LaunchConfigurations
    enable_lidar_preprocessor        = LaunchConfiguration("enable_lidar_preprocessor")
    preprocessor_marker_frame_id     = LaunchConfiguration("preprocessor_marker_frame_id")
    preprocessor_angle_filter_deg    = LaunchConfiguration("preprocessor_angle_filter_deg")
    preprocessor_roi_x_min           = LaunchConfiguration("preprocessor_roi_x_min")
    preprocessor_roi_x_max           = LaunchConfiguration("preprocessor_roi_x_max")
    preprocessor_roi_y_min           = LaunchConfiguration("preprocessor_roi_y_min")
    preprocessor_roi_y_max           = LaunchConfiguration("preprocessor_roi_y_max")
    preprocessor_roi_z_min           = LaunchConfiguration("preprocessor_roi_z_min")
    preprocessor_roi_z_max           = LaunchConfiguration("preprocessor_roi_z_max")
    preprocessor_voxel_leaf_size     = LaunchConfiguration("preprocessor_voxel_leaf_size")
    preprocessor_qos_depth           = LaunchConfiguration("preprocessor_qos_depth")
    preprocessor_max_process_hz      = LaunchConfiguration("preprocessor_max_process_hz")

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
                ("/vanjee_points750", "points_raw"),
                ("vanjee_points750", "points_raw"),

                ("/vanjee_lidar_imu_packets", "imu_packets"),
                ("vanjee_lidar_imu_packets", "imu_packets"),

                ("/lidar/vanjee/points_raw", "points_raw"),
                ("/lidar/vanjee/imu_packets", "imu_packets"),
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

    # ── LiDAR preprocessor (points_raw -> filtered_cloud)  HJ_260623 ─────
    # ground_segmentation 앞단 전처리: 각도필터 + ROI + Voxel 다운샘플로
    # 원본 포인트를 크게 줄여 ground_seg/perception 부하를 낮춘다.
    lidar_preprocessor_node = Node(
        package="camrod_sensing",
        executable="lidar_preprocessor_node",
        name="lidar_preprocessor",
        namespace=module_namespace,
        output="screen",
        parameters=[{
            "input_topic":      preprocessor_input_topic,
            "output_topic":     preprocessor_output_topic,
            "marker_frame_id":  preprocessor_marker_frame_id,
            "angle_filter_deg": preprocessor_angle_filter_deg,
            "roi_x_min":        preprocessor_roi_x_min,
            "roi_x_max":        preprocessor_roi_x_max,
            "roi_y_min":        preprocessor_roi_y_min,
            "roi_y_max":        preprocessor_roi_y_max,
            "roi_z_min":        preprocessor_roi_z_min,
            "roi_z_max":        preprocessor_roi_z_max,
            "voxel_leaf_size":  preprocessor_voxel_leaf_size,
            "qos_depth":        preprocessor_qos_depth,
            "max_process_hz":   preprocessor_max_process_hz,
        }],
        condition=IfCondition(
            PythonExpression([
                "'", enable_lidar_driver, "' == 'true' and '",
                enable_lidar_preprocessor, "' == 'true'"
            ])
        ),
    )

    # ── Ground Segmentation (DFKI) ──────────────────────────────────
    # HJ_260623: 입력을 points_raw -> filtered_cloud(lidar_preprocessor 출력)로 변경.
    ground_segmentation_node = Node(
        package="ground_segmentation_ros2",
        executable="ground_segmentation_ros2_node",
        name="ground_segmentation",
        namespace=module_namespace,
        output="screen",
        remappings=[
            ("/ground_segmentation/input_pointcloud", preprocessor_output_topic),
            ("/ground_segmentation/obstacle_points",  lidar_filtered_topic),
        ],
        parameters=[ground_seg_param_file],
        # HH_260604: Do not require the optional ground-segmentation package when LiDAR is disabled.
        condition=IfCondition(enable_lidar_driver),
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
        declare_args
        + [lidar_preprocessor_node, ground_segmentation_node]  # HJ_260623: preprocessor를 ground_seg 앞에 추가
        + optional_driver_actions
    )
