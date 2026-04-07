#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    sensing_share = get_package_share_directory("camrod_sensing")

    camera_launch = os.path.join(sensing_share, "launch", "camera.launch.py")
    gnss_launch = os.path.join(sensing_share, "launch", "gnss.launch.py")
    imu_launch = os.path.join(sensing_share, "launch", "imu.launch.py")
    lidar_launch = os.path.join(sensing_share, "launch", "lidar.launch.py")
    radar_launch = os.path.join(sensing_share, "launch", "radar.launch.py")

    default_paths = {
        "sensing_param_file": os.path.join(sensing_share, "config", "sensing_params.yaml"),
        "camera_preprocess_param_file": os.path.join(sensing_share, "config", "camera", "preprocessor.yaml"),
        "gnss_param_file": os.path.join(sensing_share, "config", "gnss", "zed_f9p_rover.yaml"),
        "ntrip_param_file": os.path.join(sensing_share, "config", "gnss", "ntrip_client.yaml"),
        "cv7_param_file": os.path.join(sensing_share, "config", "imu", "microstrain_cv7.yaml"),
        "gq7_param_file": os.path.join(sensing_share, "config", "imu", "microstrain_gq7.yaml"),
        "imu_converter_param_file": os.path.join(sensing_share, "config", "imu", "platform_velocity_converter.yaml"),
        "lidar_preprocess_param_file": os.path.join(sensing_share, "config", "lidar", "preprocessor.yaml"),
        "lidar_cost_grid_param_file": os.path.join(sensing_share, "config", "lidar", "cost_grid.yaml"),
        "radar_sensor_param_file": os.path.join(sensing_share, "config", "radar", "sen0592_radar.yaml"),
        "radar_cost_grid_param_file": os.path.join(sensing_share, "config", "radar", "cost_grid.yaml"),
        "vanjee_config_path": os.path.join(sensing_share, "config", "lidar", "vanjee", "config.yaml"),
    }

    sensing_namespace = LaunchConfiguration("sensing_namespace")

    return LaunchDescription([
        DeclareLaunchArgument("sensing_namespace", default_value="sensing"),

        DeclareLaunchArgument("enable_camera", default_value="true"),
        DeclareLaunchArgument("enable_gnss", default_value="true"),
        DeclareLaunchArgument("enable_imu", default_value="true"),
        DeclareLaunchArgument("enable_radar", default_value="true"),
        DeclareLaunchArgument("enable_radar_cost_grid", default_value="true"),
        DeclareLaunchArgument("enable_lidar_driver", default_value="true"),
        DeclareLaunchArgument("enable_lidar_cost_grid", default_value="true"),
        DeclareLaunchArgument("enable_vanjee_static_tf", default_value="false"),
        DeclareLaunchArgument("enable_ntrip", default_value="true"),

        DeclareLaunchArgument("imu_mode", default_value="cv7"),

        DeclareLaunchArgument("sensing_param_file", default_value=default_paths["sensing_param_file"]),
        DeclareLaunchArgument("camera_preprocess_param_file", default_value=default_paths["camera_preprocess_param_file"]),
        DeclareLaunchArgument("gnss_param_file", default_value=default_paths["gnss_param_file"]),
        DeclareLaunchArgument("ntrip_param_file", default_value=default_paths["ntrip_param_file"]),
        DeclareLaunchArgument("cv7_param_file", default_value=default_paths["cv7_param_file"]),
        DeclareLaunchArgument("gq7_param_file", default_value=default_paths["gq7_param_file"]),
        DeclareLaunchArgument("imu_converter_param_file", default_value=default_paths["imu_converter_param_file"]),
        DeclareLaunchArgument("lidar_preprocess_param_file", default_value=default_paths["lidar_preprocess_param_file"]),
        DeclareLaunchArgument("lidar_cost_grid_param_file", default_value=default_paths["lidar_cost_grid_param_file"]),
        DeclareLaunchArgument("radar_sensor_param_file", default_value=default_paths["radar_sensor_param_file"]),
        DeclareLaunchArgument("radar_cost_grid_param_file", default_value=default_paths["radar_cost_grid_param_file"]),
        DeclareLaunchArgument("vanjee_config_path", default_value=default_paths["vanjee_config_path"]),

        DeclareLaunchArgument("camera_input_image_topic", default_value="image_raw"),
        DeclareLaunchArgument("camera_input_camera_info_topic", default_value="camera_info"),
        DeclareLaunchArgument("camera_output_image_topic", default_value="processed/image"),
        DeclareLaunchArgument("camera_output_camera_info_topic", default_value="processed/camera_info"),
        DeclareLaunchArgument("camera_status_topic", default_value="status"),

        DeclareLaunchArgument("gnss_namespace", default_value="gnss"),
        DeclareLaunchArgument("gnss_rtcm_topic", default_value="rtcm"),

        DeclareLaunchArgument(
            "cv7_port",
            default_value="/dev/serial/by-id/usb-Lord_Microstrain_Lord_Inertial_Sensor_0000_6286.226900-if00",
        ),
        DeclareLaunchArgument("gq7_port", default_value="/dev/ttyACM1"),
        DeclareLaunchArgument("imu_velocity_topic", default_value="/platform/status/velocity"),
        DeclareLaunchArgument("imu_input_topic", default_value="data"),
        # Keep a canonical absolute output topic so system diagnostics and downstream modules
        # observe one stable velocity-converter stream regardless nested namespaces.
        DeclareLaunchArgument(
            "imu_output_topic",
            default_value="/sensing/platform_velocity_converter/twist_with_covariance",
        ),
        DeclareLaunchArgument("imu_status_topic", default_value="status"),

        GroupAction([
            PushRosNamespace(sensing_namespace),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
                launch_arguments={
                    "sensing_param_file": LaunchConfiguration("sensing_param_file"),
                    "camera_preprocess_param_file": LaunchConfiguration("camera_preprocess_param_file"),
                    "module_namespace": "camera",
                    "input_image_topic": LaunchConfiguration("camera_input_image_topic"),
                    "input_camera_info_topic": LaunchConfiguration("camera_input_camera_info_topic"),
                    "output_image_topic": LaunchConfiguration("camera_output_image_topic"),
                    "output_camera_info_topic": LaunchConfiguration("camera_output_camera_info_topic"),
                    "camera_status_topic": LaunchConfiguration("camera_status_topic"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("enable_camera")),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gnss_launch),
                launch_arguments={
                    "ublox_param_file": LaunchConfiguration("gnss_param_file"),
                    "ntrip_param_file": LaunchConfiguration("ntrip_param_file"),
                    "enable_ntrip": LaunchConfiguration("enable_ntrip"),
                    "gnss_namespace": LaunchConfiguration("gnss_namespace"),
                    "rtcm_topic": LaunchConfiguration("gnss_rtcm_topic"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("enable_gnss")),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(imu_launch),
                launch_arguments={
                    "enable_imu": LaunchConfiguration("enable_imu"),
                    "imu_mode": LaunchConfiguration("imu_mode"),
                    "module_namespace": "imu",
                    "cv7_params_file": LaunchConfiguration("cv7_param_file"),
                    "cv7_port": LaunchConfiguration("cv7_port"),
                    "gq7_params_file": LaunchConfiguration("gq7_param_file"),
                    "gq7_port": LaunchConfiguration("gq7_port"),
                    "use_ntrip": LaunchConfiguration("enable_ntrip"),
                    "ntrip_params_file": LaunchConfiguration("ntrip_param_file"),
                    "velocity_converter_param_file": LaunchConfiguration("imu_converter_param_file"),
                    "velocity_topic": LaunchConfiguration("imu_velocity_topic"),
                    "imu_topic": LaunchConfiguration("imu_input_topic"),
                    "output_topic": LaunchConfiguration("imu_output_topic"),
                    "imu_status_topic": LaunchConfiguration("imu_status_topic"),
                }.items(),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_launch),
                launch_arguments={
                    "sensing_param_file": LaunchConfiguration("sensing_param_file"),
                    "lidar_preprocess_param_file": LaunchConfiguration("lidar_preprocess_param_file"),
                    "lidar_cost_grid_param_file": LaunchConfiguration("lidar_cost_grid_param_file"),
                    "vanjee_config_path": LaunchConfiguration("vanjee_config_path"),
                    "enable_lidar_driver": LaunchConfiguration("enable_lidar_driver"),
                    "enable_lidar_cost_grid": LaunchConfiguration("enable_lidar_cost_grid"),
                    "enable_vanjee_static_tf": LaunchConfiguration("enable_vanjee_static_tf"),
                    "module_namespace": "lidar",
                    "vanjee_driver_namespace": "vanjee",
                    "preprocessor_input_topic": "vanjee/points_raw",
                    "lidar_filtered_topic": "points_filtered",
                    "lidar_status_topic": "status",
                }.items(),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(radar_launch),
                launch_arguments={
                    "enable_radar": LaunchConfiguration("enable_radar"),
                    "enable_radar_cost_grid": LaunchConfiguration("enable_radar_cost_grid"),
                    "module_namespace": "radar",
                    "radar_sensor_param_file": LaunchConfiguration("radar_sensor_param_file"),
                    "radar_cost_grid_param_file": LaunchConfiguration("radar_cost_grid_param_file"),
                }.items(),
            ),
        ]),
    ])
