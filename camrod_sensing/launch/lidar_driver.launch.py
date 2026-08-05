#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


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


def _truthy(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _standalone_condition(enabled, use_container):
    return IfCondition(PythonExpression([
        "str('", enabled, "').lower() in ('1','true','yes','on') and not ",
        "str('", use_container, "').lower() in ('1','true','yes','on')",
    ]))


def _lidar_parameters() -> dict:
    return {
        "input_topic": LaunchConfiguration("preprocessor_input_topic"),
        "output_topic": LaunchConfiguration("preprocessor_output_topic"),
        "marker_frame_id": LaunchConfiguration("preprocessor_marker_frame_id"),
        "angle_filter_deg": LaunchConfiguration("preprocessor_angle_filter_deg"),
        "roi_x_min": LaunchConfiguration("preprocessor_roi_x_min"),
        "roi_x_max": LaunchConfiguration("preprocessor_roi_x_max"),
        "roi_y_min": LaunchConfiguration("preprocessor_roi_y_min"),
        "roi_y_max": LaunchConfiguration("preprocessor_roi_y_max"),
        "roi_z_min": LaunchConfiguration("preprocessor_roi_z_min"),
        "roi_z_max": LaunchConfiguration("preprocessor_roi_z_max"),
        "voxel_leaf_size": LaunchConfiguration("preprocessor_voxel_leaf_size"),
        "qos_depth": LaunchConfiguration("preprocessor_qos_depth"),
        "max_process_hz": LaunchConfiguration("preprocessor_max_process_hz"),
    }


def _build_lidar_processing_container(context, *args, **kwargs):
    del args, kwargs
    if not _truthy(context.perform_substitution(
        LaunchConfiguration("use_lidar_processing_container")
    )):
        return []

    driver_enabled = _truthy(context.perform_substitution(
        LaunchConfiguration("enable_lidar_driver")
    ))
    preprocessor_enabled = _truthy(context.perform_substitution(
        LaunchConfiguration("enable_lidar_preprocessor")
    ))
    cost_grid_enabled = _truthy(context.perform_substitution(
        LaunchConfiguration("enable_lidar_cost_grid")
    ))
    components = []

    if driver_enabled and preprocessor_enabled:
        components.append(
            ComposableNode(
                package="camrod_sensing",
                plugin="camrod::sensing::LidarPreprocessorNode",
                name="lidar_preprocessor",
                namespace=LaunchConfiguration("module_namespace"),
                parameters=[_lidar_parameters()],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )
    if driver_enabled:
        components.append(
            ComposableNode(
                package="ground_segmentation_ros2",
                plugin="GroundSegmentationNode",
                name="ground_segmentation",
                namespace=LaunchConfiguration("module_namespace"),
                parameters=[LaunchConfiguration("ground_seg_param_file")],
                remappings=[
                    ("/ground_segmentation/input_pointcloud",
                     LaunchConfiguration("preprocessor_output_topic")),
                    ("/ground_segmentation/obstacle_points",
                     LaunchConfiguration("lidar_filtered_topic")),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )
    if cost_grid_enabled:
        # HH_260805 - Cost-grid ownership is independent from the raw driver.
        # Operators can disable this component while retaining preprocessing,
        # or run it against externally supplied clouds with the driver off.
        components.append(
            ComposableNode(
                package="camrod_sensing",
                plugin="camrod::sensing::LidarCostGridNode",
                name="lidar_cost_grid",
                namespace=LaunchConfiguration("module_namespace"),
                parameters=[LaunchConfiguration("lidar_cost_grid_param_file")],
                # HH_260805 - Humble rejects intra-process publishers that use
                # transient-local durability. Keep the grid latched for late
                # subscribers and compose it with DDS transport instead.
                extra_arguments=[{"use_intra_process_comms": False}],
            )
        )

    if not components:
        return []
    return [
        ComposableNodeContainer(
            # HH_260805 - Finalize the scoped DDS context before unloading the
            # processing components, including the optional iceoryx transport.
            package="camrod_runtime",
            executable="scoped_component_container_mt",
            name="lidar_processing_container",
            namespace=LaunchConfiguration("module_namespace"),
            output="screen",
            composable_node_descriptions=components,
        )
    ]


def generate_launch_description():
    sensing_share = get_package_share_directory("camrod_sensing")
    default_ground_seg_param = os.path.join(
        sensing_share, "config", "lidar", "ground_seg_params.yaml"
    )
    default_cost_grid_param = os.path.join(
        sensing_share, "config", "lidar", "cost_grid.yaml"
    )
    default_vanjee_config, has_vanjee_driver_pkg = _resolve_vanjee_config_and_presence()
    default_cyclonedds_config = os.path.join(
        sensing_share, "config", "middleware", "cyclonedds_lidar_shm.xml"
    )
    enable_driver_default = "true" if has_vanjee_driver_pkg else "false"

    declare_args = [
        DeclareLaunchArgument("ground_seg_param_file", default_value=default_ground_seg_param),
        DeclareLaunchArgument("lidar_cost_grid_param_file", default_value=default_cost_grid_param),
        DeclareLaunchArgument("vanjee_config_path", default_value=default_vanjee_config),
        DeclareLaunchArgument("enable_lidar_driver", default_value=enable_driver_default),
        DeclareLaunchArgument("enable_lidar_cost_grid", default_value="false"),
        DeclareLaunchArgument("use_lidar_processing_container", default_value="true"),
        DeclareLaunchArgument("enable_dds_shared_memory", default_value="false"),
        DeclareLaunchArgument(
            "dds_shared_memory_cyclonedds_config",
            default_value=default_cyclonedds_config,
        ),
        DeclareLaunchArgument("enable_vanjee_static_tf", default_value="false"),
        DeclareLaunchArgument("module_namespace", default_value="lidar"),
        DeclareLaunchArgument("vanjee_driver_namespace", default_value="vanjee"),
        DeclareLaunchArgument("preprocessor_input_topic", default_value="vanjee/points_raw"),
        DeclareLaunchArgument("preprocessor_output_topic", default_value="filtered_cloud"),
        DeclareLaunchArgument("lidar_filtered_topic", default_value="points_filtered"),
        DeclareLaunchArgument("enable_lidar_preprocessor", default_value="true"),
        DeclareLaunchArgument("preprocessor_marker_frame_id", default_value="lidar_link"),
        DeclareLaunchArgument("preprocessor_angle_filter_deg", default_value="64.0"),
        DeclareLaunchArgument("preprocessor_roi_x_min", default_value="0.0"),
        DeclareLaunchArgument("preprocessor_roi_x_max", default_value="3.0"),
        DeclareLaunchArgument("preprocessor_roi_y_min", default_value="-1.5"),
        DeclareLaunchArgument("preprocessor_roi_y_max", default_value="1.5"),
        DeclareLaunchArgument("preprocessor_roi_z_min", default_value="-1.0"),
        DeclareLaunchArgument("preprocessor_roi_z_max", default_value="1.0"),
        DeclareLaunchArgument("preprocessor_voxel_leaf_size", default_value="0.03"),
        DeclareLaunchArgument("preprocessor_qos_depth", default_value="2"),
        DeclareLaunchArgument("preprocessor_max_process_hz", default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_x", default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_y", default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_z", default_value="0.9"),
        DeclareLaunchArgument("vanjee_tf_roll", default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_pitch", default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_yaw", default_value="0.0"),
        DeclareLaunchArgument("vanjee_tf_parent", default_value="robot_center_link"),
        DeclareLaunchArgument("vanjee_tf_child", default_value="vanjee_lidar"),
    ]

    enable_lidar_driver = LaunchConfiguration("enable_lidar_driver")
    enable_lidar_preprocessor = LaunchConfiguration("enable_lidar_preprocessor")
    enable_lidar_cost_grid = LaunchConfiguration("enable_lidar_cost_grid")
    use_container = LaunchConfiguration("use_lidar_processing_container")
    module_namespace = LaunchConfiguration("module_namespace")
    optional_driver_actions = []

    if has_vanjee_driver_pkg:
        optional_driver_actions.append(
            Node(
                package="vanjee_lidar_sdk",
                executable="vanjee_lidar_sdk_node",
                name="vanjee_driver",
                namespace=[module_namespace, "/", LaunchConfiguration("vanjee_driver_namespace")],
                output="screen",
                parameters=[{"config_path": LaunchConfiguration("vanjee_config_path")}],
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
        )
        optional_driver_actions.append(
            Node(
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
                condition=IfCondition(PythonExpression([
                    "'", enable_lidar_driver, "' == 'true' and '",
                    LaunchConfiguration("enable_vanjee_static_tf"), "' == 'true'",
                ])),
            )
        )
    else:
        optional_driver_actions.append(
            LogInfo(msg="[lidar.launch] package 'vanjee_lidar_sdk' not found; skipping raw driver node.")
        )

    standalone_nodes = [
        Node(
            package="camrod_sensing",
            executable="lidar_preprocessor_node",
            name="lidar_preprocessor",
            namespace=module_namespace,
            output="screen",
            parameters=[_lidar_parameters()],
            condition=IfCondition(PythonExpression([
                "'", enable_lidar_driver, "' == 'true' and '",
                enable_lidar_preprocessor, "' == 'true' and '", use_container,
                "' != 'true'",
            ])),
        ),
        Node(
            package="ground_segmentation_ros2",
            executable="ground_segmentation_ros2_node",
            name="ground_segmentation",
            namespace=module_namespace,
            output="screen",
            remappings=[
                ("/ground_segmentation/input_pointcloud",
                 LaunchConfiguration("preprocessor_output_topic")),
                ("/ground_segmentation/obstacle_points",
                 LaunchConfiguration("lidar_filtered_topic")),
            ],
            parameters=[LaunchConfiguration("ground_seg_param_file")],
            condition=_standalone_condition(enable_lidar_driver, use_container),
        ),
        Node(
            package="camrod_sensing",
            executable="lidar_cost_grid_node",
            name="lidar_cost_grid",
            namespace=module_namespace,
            output="screen",
            parameters=[LaunchConfiguration("lidar_cost_grid_param_file")],
            condition=_standalone_condition(enable_lidar_cost_grid, use_container),
        ),
    ]

    # HH_260805 - GroupAction restores the parent environment after this launch
    # scope. Only the Vanjee driver and LiDAR processing process opt into
    # CycloneDDS/iceoryx; the rest of CAMROD keeps its configured host RMW.
    lidar_runtime = GroupAction([
        SetEnvironmentVariable(
            "RMW_IMPLEMENTATION",
            "rmw_cyclonedds_cpp",
            condition=IfCondition(LaunchConfiguration("enable_dds_shared_memory")),
        ),
        SetEnvironmentVariable(
            "CYCLONEDDS_URI",
            LaunchConfiguration("dds_shared_memory_cyclonedds_config"),
            condition=IfCondition(LaunchConfiguration("enable_dds_shared_memory")),
        ),
        OpaqueFunction(function=_build_lidar_processing_container),
        *standalone_nodes,
        *optional_driver_actions,
    ])

    return LaunchDescription(declare_args + [lidar_runtime])
