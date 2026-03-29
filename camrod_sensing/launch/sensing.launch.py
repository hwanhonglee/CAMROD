import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def launch_setup(context, *args, **kwargs):
    sensing_share = get_package_share_directory("camrod_sensing")

    cfg = {
        "sensing_param_file": LaunchConfiguration("sensing_param_file"),
        "lidar_preprocess_param_file": LaunchConfiguration("lidar_preprocess_param_file"),
        "camera_preprocess_param_file": LaunchConfiguration("camera_preprocess_param_file"),
        "imu_converter_param_file": LaunchConfiguration("imu_converter_param_file"),
        "radar_param_file": LaunchConfiguration("radar_param_file"),
        "radar_sensor_param_file": LaunchConfiguration("radar_sensor_param_file"),
        "radar_cost_grid_param_file": LaunchConfiguration("radar_cost_grid_param_file"),
        "lidar_cost_grid_param_file": LaunchConfiguration("lidar_cost_grid_param_file"),
        "enable_radar": LaunchConfiguration("enable_radar"),
        "enable_radar_cost_grid": LaunchConfiguration("enable_radar_cost_grid"),
        "enable_lidar_cost_grid": LaunchConfiguration("enable_lidar_cost_grid"),
        "enable_lidar_driver": LaunchConfiguration("enable_lidar_driver"),
        "enable_imu": LaunchConfiguration("enable_imu"),
        "vanjee_config_path": LaunchConfiguration("vanjee_config_path"),
        "enable_vanjee_static_tf": LaunchConfiguration("enable_vanjee_static_tf"),
        "enable_module_validator": LaunchConfiguration("enable_module_validator"),
        "sensing_namespace": LaunchConfiguration("sensing_namespace"),
        "system_namespace": LaunchConfiguration("system_namespace"),
        "enable_gnss": LaunchConfiguration("enable_gnss"),
        "enable_ntrip": LaunchConfiguration("enable_ntrip"),
        "gnss_param_file": LaunchConfiguration("gnss_param_file"),
        "ntrip_param_file": LaunchConfiguration("ntrip_param_file"),
        "gnss_namespace": LaunchConfiguration("gnss_namespace"),
        "gnss_navsatfix_topic": LaunchConfiguration("gnss_navsatfix_topic"),
        "gnss_rtcm_topic": LaunchConfiguration("gnss_rtcm_topic"),
        "camera_input_image_topic": LaunchConfiguration("camera_input_image_topic"),
        "camera_input_camera_info_topic": LaunchConfiguration("camera_input_camera_info_topic"),
        "camera_output_image_topic": LaunchConfiguration("camera_output_image_topic"),
        "camera_output_camera_info_topic": LaunchConfiguration("camera_output_camera_info_topic"),
        "camera_status_topic": LaunchConfiguration("camera_status_topic"),
        "imu_input_topic": LaunchConfiguration("imu_input_topic"),
        "imu_output_topic": LaunchConfiguration("imu_output_topic"),
        "imu_status_topic": LaunchConfiguration("imu_status_topic"),
    }

    # ---------------------------------------------------------------------
    # LiDAR
    # sensing.launch.py result:
    #   /sensing/lidar/vanjee/points_raw
    #   /sensing/lidar/vanjee/imu_packets
    #   /sensing/lidar/points_filtered
    # ---------------------------------------------------------------------
    lidar_stack = GroupAction(
        actions=[
            PushRosNamespace(cfg["sensing_namespace"]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sensing_share, "launch", "lidar.launch.py")
                ),
                launch_arguments={
                    "sensing_param_file": cfg["sensing_param_file"],
                    "lidar_preprocess_param_file": cfg["lidar_preprocess_param_file"],
                    "enable_lidar_driver": cfg["enable_lidar_driver"],
                    "vanjee_config_path": cfg["vanjee_config_path"],

                    # relative module structure under /sensing
                    "module_namespace": "lidar",
                    "vanjee_driver_namespace": "vanjee",

                    # preprocessor relative topic
                    "preprocessor_input_topic": "vanjee/points_raw",
                    "lidar_filtered_topic": "points_filtered",
                    "lidar_status_topic": "status",

                    "enable_vanjee_static_tf": cfg["enable_vanjee_static_tf"],
                }.items(),
            ),
        ]
    )

    # ---------------------------------------------------------------------
    # GNSS
    # /sensing/gnss/...
    # ---------------------------------------------------------------------
    gnss_stack = GroupAction(
        actions=[
            PushRosNamespace(cfg["sensing_namespace"]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sensing_share, "launch", "gnss.launch.py")
                ),
                launch_arguments={
                    "ublox_param_file": cfg["gnss_param_file"],
                    "ntrip_param_file": cfg["ntrip_param_file"],
                    "enable_ntrip": cfg["enable_ntrip"],
                    "gnss_namespace": cfg["gnss_namespace"],
                    "navsatfix_topic": cfg["gnss_navsatfix_topic"],
                    "rtcm_topic": cfg["gnss_rtcm_topic"],
                }.items(),
                condition=IfCondition(cfg["enable_gnss"]),
            ),
        ]
    )

    # ---------------------------------------------------------------------
    # Camera
    # /sensing/camera/...
    # ---------------------------------------------------------------------
    camera_preprocessor = GroupAction(
        actions=[
            PushRosNamespace(cfg["sensing_namespace"]),
            Node(
                package="camrod_sensing",
                executable="camera_preprocessor_node",
                name="camera_preprocessor",
                namespace="camera",
                output="screen",
                parameters=[
                    cfg["sensing_param_file"],
                    cfg["camera_preprocess_param_file"],
                    {
                        "input_image_topic": cfg["camera_input_image_topic"],
                        "input_camera_info_topic": cfg["camera_input_camera_info_topic"],
                        "output_image_topic": cfg["camera_output_image_topic"],
                        "output_camera_info_topic": cfg["camera_output_camera_info_topic"],
                        "camera_status_topic": cfg["camera_status_topic"],
                    },
                ],
            ),
        ]
    )

    # ---------------------------------------------------------------------
    # IMU / Platform velocity
    # /sensing/imu/...
    # ---------------------------------------------------------------------
    imu_stack = GroupAction(
        actions=[
            PushRosNamespace(cfg["sensing_namespace"]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sensing_share, "launch", "imu.launch.py")
                ),
                launch_arguments={
                    "params_file": os.path.join(
                        sensing_share, "config", "imu", "microstrain_cv7.yaml"
                    ),
                    "namespace": "imu",
                }.items(),
            ),
        ],
        condition=IfCondition(cfg["enable_imu"]),
    )    
    
    velocity_converter = GroupAction(
        actions=[
            PushRosNamespace(cfg["sensing_namespace"]),
            Node(
                package="camrod_sensing",
                executable="platform_velocity_converter_node",
                name="platform_velocity_converter",
                namespace="imu",
                output="screen",
                parameters=[
                    cfg["sensing_param_file"],
                    cfg["imu_converter_param_file"],
                    {
                        "imu_topic": cfg["imu_input_topic"],
                        "output_topic": cfg["imu_output_topic"],
                        "imu_status_topic": cfg["imu_status_topic"],
                    },
                ],
            ),
        ],
        condition=IfCondition(cfg["enable_imu"]),
    )

    # ---------------------------------------------------------------------
    # Radar
    # /sensing/radar/...
    # ---------------------------------------------------------------------
    radar_stack = GroupAction(
        actions=[
            PushRosNamespace(cfg["sensing_namespace"]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sensing_share, "launch", "radar.launch.py")
                ),
                launch_arguments={
                    "radar_params": cfg["radar_sensor_param_file"],
                    "module_namespace": "radar",
                }.items(),
                condition=IfCondition(cfg["enable_radar"]),
            ),
        ]
    )

    radar_cost_grid = GroupAction(
        actions=[
            PushRosNamespace(cfg["sensing_namespace"]),
            Node(
                package="camrod_sensing",
                executable="radar_cost_grid_node",
                name="radar_cost_grid",
                namespace="radar",
                output="screen",
                parameters=[cfg["radar_cost_grid_param_file"]],
                condition=IfCondition(cfg["enable_radar_cost_grid"]),
            ),
        ]
    )

    lidar_cost_grid = GroupAction(
        actions=[
            PushRosNamespace(cfg["sensing_namespace"]),
            Node(
                package="camrod_sensing",
                executable="lidar_cost_grid_node",
                name="lidar_cost_grid",
                namespace="lidar",
                output="screen",
                parameters=[cfg["lidar_cost_grid_param_file"]],
                condition=IfCondition(cfg["enable_lidar_cost_grid"]),
            ),
        ]
    )

    # HH_260326: Removed sensing status/validator runtime nodes as requested.

    return [
        lidar_stack,
        gnss_stack,
        camera_preprocessor,
        imu_stack,
        velocity_converter,
        radar_stack,
        radar_cost_grid,
        lidar_cost_grid,
    ]

def generate_launch_description():
    sensing_share = get_package_share_directory("camrod_sensing")

    default_paths = {
        "sensing_param_file": os.path.join(
            sensing_share, "config", "sensing_params.yaml"
        ),
        "lidar_preprocess_param_file": os.path.join(
            sensing_share, "config", "lidar", "preprocessor.yaml"
        ),
        "camera_preprocess_param_file": os.path.join(
            sensing_share, "config", "camera", "preprocessor.yaml"
        ),
        "imu_converter_param_file": os.path.join(
            sensing_share, "config", "imu", "platform_velocity_converter.yaml"
        ),
        "radar_param_file": os.path.join(
            sensing_share, "config", "radar", "sen0592_radar.yaml"
        ),
        "radar_sensor_param_file": os.path.join(
            sensing_share, "config", "radar", "sen0592_radar.yaml"
        ),
        "radar_cost_grid_param_file": os.path.join(
            sensing_share, "config", "radar", "cost_grid.yaml"
        ),
        "lidar_cost_grid_param_file": os.path.join(
            sensing_share, "config", "lidar", "cost_grid.yaml"
        ),
        "gnss_param_file": os.path.join(
            sensing_share, "config", "gnss", "zed_f9p_rover.yaml"
        ),
        "ntrip_param_file": os.path.join(
            sensing_share, "config", "gnss", "ntrip_client.yaml"
        ),
        "vanjee_config_path": os.path.join(
            sensing_share, "config", "lidar", "vanjee", "config.yaml"
        ),
    }

    launch_arguments = []

    def add_launch_arg(name, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(
                name,
                default_value=default_value,
                description=description,
            )
        )

    # ---------------------------------------------------------------------
    # Common parameter files
    # ---------------------------------------------------------------------
    add_launch_arg(
        "sensing_param_file",
        default_paths["sensing_param_file"],
        "Legacy monolithic sensing parameter file",
    )

    # ---------------------------------------------------------------------
    # LiDAR parameters
    # ---------------------------------------------------------------------
    add_launch_arg(
        "lidar_preprocess_param_file",
        default_paths["lidar_preprocess_param_file"],
        "LiDAR preprocessor parameter file",
    )
    add_launch_arg(
        "lidar_cost_grid_param_file",
        default_paths["lidar_cost_grid_param_file"],
        "Near-range LiDAR cost grid parameter file",
    )
    add_launch_arg(
        "vanjee_config_path",
        default_paths["vanjee_config_path"],
        "Optional override path for Vanjee SDK config.yaml",
    )

    # ---------------------------------------------------------------------
    # Camera parameters
    # ---------------------------------------------------------------------
    add_launch_arg(
        "camera_preprocess_param_file",
        default_paths["camera_preprocess_param_file"],
        "Camera preprocessor parameter file",
    )

    # ---------------------------------------------------------------------
    # IMU parameters
    # ---------------------------------------------------------------------
    add_launch_arg(
        "imu_converter_param_file",
        default_paths["imu_converter_param_file"],
        "Platform velocity converter parameter file",
    )

    # ---------------------------------------------------------------------
    # Radar parameters
    # ---------------------------------------------------------------------
    add_launch_arg(
        "radar_param_file",
        default_paths["radar_param_file"],
        "Legacy radar serial sensor parameter file",
    )
    add_launch_arg(
        "radar_sensor_param_file",
        default_paths["radar_sensor_param_file"],
        "Radar serial sensor parameter file",
    )
    add_launch_arg(
        "radar_cost_grid_param_file",
        default_paths["radar_cost_grid_param_file"],
        "Near-range radar cost grid parameter file",
    )

    # ---------------------------------------------------------------------
    # GNSS parameters
    # ---------------------------------------------------------------------
    add_launch_arg(
        "gnss_param_file",
        default_paths["gnss_param_file"],
        "u-blox GNSS parameter file",
    )
    add_launch_arg(
        "ntrip_param_file",
        default_paths["ntrip_param_file"],
        "NTRIP client parameter file",
    )

    # ---------------------------------------------------------------------
    # Feature toggles
    # ---------------------------------------------------------------------
    add_launch_arg("enable_radar", "true", "Enable SEN0592 radar node")
    add_launch_arg("enable_radar_cost_grid", "true", "Enable radar cost grid node")
    add_launch_arg("enable_lidar_cost_grid", "true", "Enable LiDAR cost grid node")
    add_launch_arg("enable_lidar_driver", "true", "Enable Vanjee LiDAR driver node")
    add_launch_arg("enable_imu", "true", "Enable IMU driver and velocity converter")
    add_launch_arg("enable_vanjee_static_tf", "false", "Enable Vanjee static TF")
    add_launch_arg("enable_module_validator", "true", "Enable sensing module validator")
    add_launch_arg("enable_gnss", "true", "Enable GNSS stack")
    add_launch_arg("enable_ntrip", "true", "Enable NTRIP client")

    # ---------------------------------------------------------------------
    # Namespaces
    # ---------------------------------------------------------------------
    add_launch_arg("sensing_namespace", "sensing", "Top-level sensing namespace")
    add_launch_arg("system_namespace", "system", "Namespace for system validator nodes")
    add_launch_arg("gnss_namespace", "gnss", "Namespace for GNSS nodes under /sensing")

    # ---------------------------------------------------------------------
    # GNSS topics
    # ---------------------------------------------------------------------
    add_launch_arg("gnss_navsatfix_topic", "navsatfix", "GNSS NavSatFix topic name")
    add_launch_arg("gnss_rtcm_topic", "rtcm", "GNSS RTCM topic name")

    # ---------------------------------------------------------------------
    # Camera topics
    # ---------------------------------------------------------------------
    add_launch_arg("camera_input_image_topic", "image_raw", "Camera input image topic")
    add_launch_arg(
        "camera_input_camera_info_topic",
        "camera_info",
        "Camera input camera_info topic",
    )
    add_launch_arg(
        "camera_output_image_topic",
        "processed/image",
        "Camera processed image topic",
    )
    add_launch_arg(
        "camera_output_camera_info_topic",
        "processed/camera_info",
        "Camera processed camera_info topic",
    )
    add_launch_arg("camera_status_topic", "status", "Camera status topic")

    # ---------------------------------------------------------------------
    # IMU topics
    # ---------------------------------------------------------------------
    add_launch_arg("imu_input_topic", "data", "IMU input topic")
    add_launch_arg(
        "imu_output_topic",
        "platform_velocity_converter/twist_with_covariance",
        "Velocity converter output topic",
    )
    add_launch_arg("imu_status_topic", "status", "IMU status topic")

    return LaunchDescription(
        launch_arguments + [OpaqueFunction(function=launch_setup)]
    )
