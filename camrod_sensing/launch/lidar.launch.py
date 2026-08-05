#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    sensing_share = get_package_share_directory("camrod_sensing")
    lidar_driver_launch = os.path.join(sensing_share, "launch", "lidar_driver.launch.py")
    default_cost_grid_param = os.path.join(
        sensing_share, "config", "lidar", "cost_grid.yaml"
    )

    ground_seg_param_file    = LaunchConfiguration("ground_seg_param_file")
    enable_lidar_driver      = LaunchConfiguration("enable_lidar_driver")
    enable_lidar_cost_grid   = LaunchConfiguration("enable_lidar_cost_grid")
    use_lidar_processing_container = LaunchConfiguration("use_lidar_processing_container")
    vanjee_config_path       = LaunchConfiguration("vanjee_config_path")
    enable_vanjee_static_tf  = LaunchConfiguration("enable_vanjee_static_tf")
    module_namespace         = LaunchConfiguration("module_namespace")
    vanjee_driver_namespace  = LaunchConfiguration("vanjee_driver_namespace")
    preprocessor_input_topic = LaunchConfiguration("preprocessor_input_topic")
    preprocessor_output_topic = LaunchConfiguration("preprocessor_output_topic")
    lidar_filtered_topic     = LaunchConfiguration("lidar_filtered_topic")
    lidar_cost_grid_param_file = LaunchConfiguration("lidar_cost_grid_param_file")
    enable_dds_shared_memory = LaunchConfiguration("enable_dds_shared_memory")
    dds_shared_memory_cyclonedds_config = LaunchConfiguration(
        "dds_shared_memory_cyclonedds_config"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "ground_seg_param_file",
            default_value=os.path.join(sensing_share, "config", "lidar", "ground_seg_params.yaml"),
        ),
        DeclareLaunchArgument(
            "lidar_cost_grid_param_file",
            default_value=default_cost_grid_param,
        ),
        DeclareLaunchArgument(
            "vanjee_config_path",
            default_value=os.path.join(sensing_share, "config", "lidar", "vanjee", "config.yaml"),
        ),
        DeclareLaunchArgument("enable_lidar_driver",      default_value="true"),
        DeclareLaunchArgument("enable_lidar_cost_grid",   default_value="false"),
        # HH_260805 - Keep cost-grid enable independent inside the LiDAR container.
        DeclareLaunchArgument("use_lidar_processing_container", default_value="true"),
        DeclareLaunchArgument("enable_dds_shared_memory", default_value="false"),
        DeclareLaunchArgument(
            "dds_shared_memory_cyclonedds_config",
            default_value=os.path.join(
                sensing_share, "config", "middleware", "cyclonedds_lidar_shm.xml"
            ),
        ),
        DeclareLaunchArgument("enable_vanjee_static_tf",  default_value="false"),
        DeclareLaunchArgument("module_namespace",         default_value="lidar"),
        DeclareLaunchArgument("vanjee_driver_namespace",  default_value="vanjee"),
        DeclareLaunchArgument("preprocessor_input_topic", default_value="vanjee/points_raw"),
        DeclareLaunchArgument("preprocessor_output_topic", default_value="filtered_cloud"),
        DeclareLaunchArgument("lidar_filtered_topic",     default_value="points_filtered"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_driver_launch),
            launch_arguments={
                "ground_seg_param_file":    ground_seg_param_file,
                "enable_lidar_driver":      enable_lidar_driver,
                "enable_lidar_cost_grid":   enable_lidar_cost_grid,
                "use_lidar_processing_container": use_lidar_processing_container,
                "enable_dds_shared_memory": enable_dds_shared_memory,
                "dds_shared_memory_cyclonedds_config": (
                    dds_shared_memory_cyclonedds_config
                ),
                "lidar_cost_grid_param_file": lidar_cost_grid_param_file,
                "vanjee_config_path":       vanjee_config_path,
                "enable_vanjee_static_tf":  enable_vanjee_static_tf,
                "module_namespace":         module_namespace,
                "vanjee_driver_namespace":  vanjee_driver_namespace,
                "preprocessor_input_topic": preprocessor_input_topic,
                "preprocessor_output_topic": preprocessor_output_topic,
                "lidar_filtered_topic":     lidar_filtered_topic,
            }.items(),
        ),
    ])
