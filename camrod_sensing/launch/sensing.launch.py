# HH_260109 Launch LiDAR/Camera preprocessing and velocity conversion pipeline.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    # HH_260315-00:00 Use sensing package as canonical source for sensing algorithms.
    bringup_share = get_package_share_directory('camrod_bringup')
    sensing_share = get_package_share_directory('camrod_sensing')
    default_param = os.path.join(sensing_share, 'config', 'sensing_params.yaml')
    default_lidar_preprocess_param = os.path.join(
        sensing_share, 'config', 'lidar', 'preprocessor.yaml')
    default_camera_preprocess_param = os.path.join(
        sensing_share, 'config', 'camera', 'preprocessor.yaml')
    default_imu_converter_param = os.path.join(
        sensing_share, 'config', 'imu', 'platform_velocity_converter.yaml')
    default_radar_param = os.path.join(
        sensing_share, 'config', 'radar', 'sen0592_radar.yaml')
    default_radar_sensor_param = os.path.join(
        sensing_share, 'config', 'radar', 'sen0592_radar.yaml')
    default_radar_grid_param = os.path.join(
        sensing_share, 'config', 'radar', 'cost_grid.yaml')
    default_lidar_grid_param = os.path.join(
        sensing_share, 'config', 'lidar', 'cost_grid.yaml')
    default_gnss_param = os.path.join(
        sensing_share, 'config', 'gnss', 'zed_f9p_rover.yaml')
    default_ntrip_param = os.path.join(
        sensing_share, 'config', 'gnss', 'ntrip_client.yaml')
    default_vanjee_config = os.path.join(
        get_package_share_directory('camrod_sensing'),
        'config', 'lidar', 'vanjee', 'config.yaml')

    # HH_260114 Unique param arg to avoid global name collisions.
    param_file_arg = DeclareLaunchArgument(
        'sensing_param_file',
        default_value=default_param,
        description='Legacy monolithic sensing parameter file (kept for compatibility)',
    )
    lidar_preprocess_param_file_arg = DeclareLaunchArgument(
        'lidar_preprocess_param_file',
        default_value=default_lidar_preprocess_param,
        description='LiDAR preprocessor parameter file',
    )
    camera_preprocess_param_file_arg = DeclareLaunchArgument(
        'camera_preprocess_param_file',
        default_value=default_camera_preprocess_param,
        description='Camera preprocessor parameter file',
    )
    imu_converter_param_file_arg = DeclareLaunchArgument(
        'imu_converter_param_file',
        default_value=default_imu_converter_param,
        description='Platform velocity converter parameter file',
    )
    radar_param_arg = DeclareLaunchArgument(
        'radar_param_file',
        default_value=default_radar_param,
        description='Legacy radar serial sensor parameter file (SEN0592)',
    )
    radar_sensor_param_arg = DeclareLaunchArgument(
        'radar_sensor_param_file',
        default_value=default_radar_sensor_param,
        description='Radar serial sensor parameter file (SEN0592)',
    )
    radar_grid_param_arg = DeclareLaunchArgument(
        'radar_cost_grid_param_file',
        default_value=default_radar_grid_param,
        description='Near-range radar cost grid parameter file',
    )
    lidar_grid_param_arg = DeclareLaunchArgument(
        'lidar_cost_grid_param_file',
        default_value=default_lidar_grid_param,
        description='Near-range lidar cost grid parameter file',
    )
    enable_radar_arg = DeclareLaunchArgument(
        'enable_radar',
        default_value='false',
        description='Enable SEN0592 serial radar node',
    )
    enable_radar_cost_grid_arg = DeclareLaunchArgument(
        'enable_radar_cost_grid',
        default_value='true',
        description='Enable near-range radar occupancy grid node',
    )
    enable_lidar_cost_grid_arg = DeclareLaunchArgument(
        'enable_lidar_cost_grid',
        default_value='true',
        description='Enable near-range lidar occupancy grid node',
    )
    enable_lidar_driver_arg = DeclareLaunchArgument(
        'enable_lidar_driver',
        # HH_260315-00:00 Keep driver off by default for sensorless bringup/tests.
        # Enable explicitly on real hardware.
        default_value='false',
        description='Enable Vanjee LiDAR SDK raw driver node',
    )
    vanjee_config_path_arg = DeclareLaunchArgument(
        'vanjee_config_path',
        default_value=default_vanjee_config,
        description='Optional override path for Vanjee SDK config.yaml',
    )
    vanjee_driver_namespace_arg = DeclareLaunchArgument(
        'vanjee_driver_namespace',
        default_value='sensing/lidar/vanjee',
        description='Namespace of Vanjee SDK LiDAR driver node',
    )
    lidar_raw_topic_arg = DeclareLaunchArgument(
        'lidar_raw_topic',
        default_value='lidar/vanjee/points_raw',
        description='Raw LiDAR topic name from Vanjee driver (relative to module namespace)',
    )
    lidar_filtered_topic_arg = DeclareLaunchArgument(
        'lidar_filtered_topic',
        default_value='lidar/points_filtered',
        description='Filtered LiDAR topic (relative to module namespace)',
    )
    lidar_imu_packets_topic_arg = DeclareLaunchArgument(
        'lidar_imu_packets_topic',
        default_value='lidar/vanjee/imu_packets',
        description='LiDAR IMU packets topic (relative to module namespace)',
    )
    lidar_diagnostic_topic_arg = DeclareLaunchArgument(
        'lidar_diagnostic_topic',
        default_value='lidar/diagnostic',
        description='LiDAR diagnostic topic (relative to module namespace)',
    )
    enable_vanjee_static_tf_arg = DeclareLaunchArgument(
        'enable_vanjee_static_tf',
        default_value='false',
        description='Publish static TF robot_base_link -> vanjee_lidar from lidar.launch',
    )
    enable_module_checker_arg = DeclareLaunchArgument(
        'enable_module_checker',
        default_value='true',
        description='Enable sensing module checker publisher',
    )
    module_namespace_arg = DeclareLaunchArgument(
        'module_namespace',
        default_value='sensing',
        description='Namespace for sensing module nodes',
    )
    system_namespace_arg = DeclareLaunchArgument(
        'system_namespace',
        default_value='system',
        description='Namespace for system-level checker nodes',
    )
    enable_gnss_arg = DeclareLaunchArgument(
        'enable_gnss',
        default_value='true',
        description='Enable ublox GNSS stack',
    )
    enable_ntrip_arg = DeclareLaunchArgument(
        'enable_ntrip',
        default_value='false',
        description='Enable NTRIP client for ublox GNSS',
    )
    gnss_param_arg = DeclareLaunchArgument(
        'gnss_param_file',
        default_value=default_gnss_param,
        description='ublox GNSS parameter file',
    )
    ntrip_param_arg = DeclareLaunchArgument(
        'ntrip_param_file',
        default_value=default_ntrip_param,
        description='NTRIP client parameter file',
    )
    gnss_namespace_arg = DeclareLaunchArgument(
        'gnss_namespace',
        default_value='sensing/gnss',
        description='ublox GNSS namespace',
    )
    gnss_navsatfix_topic_arg = DeclareLaunchArgument(
        'gnss_navsatfix_topic',
        default_value='navsatfix',
        description='Canonical GNSS NavSatFix topic (relative to gnss namespace)',
    )
    gnss_rtcm_topic_arg = DeclareLaunchArgument(
        'gnss_rtcm_topic',
        default_value='rtcm',
        description='GNSS RTCM topic (relative to gnss namespace)',
    )
    camera_input_image_topic_arg = DeclareLaunchArgument(
        'camera_input_image_topic',
        default_value='camera/image_raw',
        description='Camera input image topic (relative to module namespace)',
    )
    camera_input_camera_info_topic_arg = DeclareLaunchArgument(
        'camera_input_camera_info_topic',
        default_value='camera/camera_info',
        description='Camera input camera_info topic (relative to module namespace)',
    )
    camera_output_image_topic_arg = DeclareLaunchArgument(
        'camera_output_image_topic',
        default_value='camera/processed/image',
        description='Camera processed image topic (relative to module namespace)',
    )
    camera_output_camera_info_topic_arg = DeclareLaunchArgument(
        'camera_output_camera_info_topic',
        default_value='camera/processed/camera_info',
        description='Camera processed camera_info topic (relative to module namespace)',
    )
    camera_diagnostic_topic_arg = DeclareLaunchArgument(
        'camera_diagnostic_topic',
        default_value='camera/diagnostic',
        description='Camera diagnostic topic (relative to module namespace)',
    )
    imu_input_topic_arg = DeclareLaunchArgument(
        'imu_input_topic',
        default_value='imu/data',
        description='IMU input topic for velocity converter (relative to module namespace)',
    )
    imu_output_topic_arg = DeclareLaunchArgument(
        'imu_output_topic',
        default_value='platform_velocity_converter/twist_with_covariance',
        description='TwistWithCovariance output topic (relative to module namespace)',
    )
    imu_diagnostic_topic_arg = DeclareLaunchArgument(
        'imu_diagnostic_topic',
        default_value='imu/diagnostic',
        description='IMU diagnostic topic (relative to module namespace)',
    )
    param_file = LaunchConfiguration('sensing_param_file')
    lidar_preprocess_param_file = LaunchConfiguration('lidar_preprocess_param_file')
    camera_preprocess_param_file = LaunchConfiguration('camera_preprocess_param_file')
    imu_converter_param_file = LaunchConfiguration('imu_converter_param_file')
    radar_param_file = LaunchConfiguration('radar_param_file')
    radar_sensor_param_file = LaunchConfiguration('radar_sensor_param_file')
    radar_grid_param_file = LaunchConfiguration('radar_cost_grid_param_file')
    lidar_grid_param_file = LaunchConfiguration('lidar_cost_grid_param_file')
    enable_radar = LaunchConfiguration('enable_radar')
    enable_radar_cost_grid = LaunchConfiguration('enable_radar_cost_grid')
    enable_lidar_cost_grid = LaunchConfiguration('enable_lidar_cost_grid')
    enable_lidar_driver = LaunchConfiguration('enable_lidar_driver')
    vanjee_config_path = LaunchConfiguration('vanjee_config_path')
    vanjee_driver_namespace = LaunchConfiguration('vanjee_driver_namespace')
    lidar_raw_topic = LaunchConfiguration('lidar_raw_topic')
    lidar_filtered_topic = LaunchConfiguration('lidar_filtered_topic')
    lidar_imu_packets_topic = LaunchConfiguration('lidar_imu_packets_topic')
    lidar_diagnostic_topic = LaunchConfiguration('lidar_diagnostic_topic')
    enable_vanjee_static_tf = LaunchConfiguration('enable_vanjee_static_tf')
    enable_module_checker = LaunchConfiguration('enable_module_checker')
    module_namespace = LaunchConfiguration('module_namespace')
    system_namespace = LaunchConfiguration('system_namespace')
    enable_gnss = LaunchConfiguration('enable_gnss')
    enable_ntrip = LaunchConfiguration('enable_ntrip')
    gnss_param_file = LaunchConfiguration('gnss_param_file')
    ntrip_param_file = LaunchConfiguration('ntrip_param_file')
    gnss_namespace = LaunchConfiguration('gnss_namespace')
    gnss_navsatfix_topic = LaunchConfiguration('gnss_navsatfix_topic')
    gnss_rtcm_topic = LaunchConfiguration('gnss_rtcm_topic')
    camera_input_image_topic = LaunchConfiguration('camera_input_image_topic')
    camera_input_camera_info_topic = LaunchConfiguration('camera_input_camera_info_topic')
    camera_output_image_topic = LaunchConfiguration('camera_output_image_topic')
    camera_output_camera_info_topic = LaunchConfiguration('camera_output_camera_info_topic')
    camera_diagnostic_topic = LaunchConfiguration('camera_diagnostic_topic')
    imu_input_topic = LaunchConfiguration('imu_input_topic')
    imu_output_topic = LaunchConfiguration('imu_output_topic')
    imu_diagnostic_topic = LaunchConfiguration('imu_diagnostic_topic')

    # HH_260311-00:00 Delegate LiDAR raw driver + preprocessor composition to lidar.launch.py.
    lidar_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('camrod_sensing'), 'launch', 'lidar.launch.py')
        ),
        launch_arguments={
            'sensing_param_file': param_file,
            'lidar_preprocess_param_file': lidar_preprocess_param_file,
            'enable_lidar_driver': enable_lidar_driver,
            'vanjee_config_path': vanjee_config_path,
            'vanjee_driver_namespace': vanjee_driver_namespace,
            'module_namespace': module_namespace,
            'lidar_raw_topic': lidar_raw_topic,
            'lidar_filtered_topic': lidar_filtered_topic,
            'lidar_imu_packets_topic': lidar_imu_packets_topic,
            'lidar_diagnostic_topic': lidar_diagnostic_topic,
            'enable_vanjee_static_tf': enable_vanjee_static_tf,
        }.items(),
    )
    gnss_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('camrod_sensing'), 'launch', 'gnss.launch.py')
        ),
        launch_arguments={
            'ublox_param_file': gnss_param_file,
            'ntrip_param_file': ntrip_param_file,
            'enable_ntrip': enable_ntrip,
            'gnss_namespace': gnss_namespace,
            'navsatfix_topic': gnss_navsatfix_topic,
            'rtcm_topic': gnss_rtcm_topic,
        }.items(),
        condition=IfCondition(enable_gnss),
    )

    # HH_260109 Camera preprocessing (raw -> processed).
    camera_preprocessor = Node(
        package='camrod_sensing',
        executable='camera_preprocessor_node',
        name='camera_preprocessor',
        namespace=module_namespace,
        output='screen',
        # HH_260314-00:00 Layered params: legacy monolithic file + camera-specific override.
        parameters=[param_file, camera_preprocess_param_file, {
            'input_image_topic': camera_input_image_topic,
            'input_camera_info_topic': camera_input_camera_info_topic,
            'output_image_topic': camera_output_image_topic,
            'output_camera_info_topic': camera_output_camera_info_topic,
            'camera_diagnostic_topic': camera_diagnostic_topic,
        }],
    )

    # HH_260109 Convert platform velocity + IMU to twist_with_covariance.
    velocity_converter = Node(
        package='camrod_sensing',
        executable='platform_velocity_converter_node',
        name='platform_velocity_converter',
        namespace=module_namespace,
        output='screen',
        # HH_260314-00:00 Layered params: legacy monolithic file + imu-specific override.
        parameters=[param_file, imu_converter_param_file, {
            'imu_topic': imu_input_topic,
            'output_topic': imu_output_topic,
            'imu_diagnostic_topic': imu_diagnostic_topic,
        }],
    )

    # 2026-02-24: Optional ultrasonic radar input.
    radar_sensor = Node(
        package='camrod_sensing',
        executable='sen0592_radar_node',
        # HH_260315-00:00 Match node name with radar YAML keys for deterministic param binding.
        name='sen0592_radar_node',
        namespace=module_namespace,
        output='screen',
        # HH_260314-00:00 Layered params: legacy radar file + radar-specific override.
        parameters=[radar_param_file, radar_sensor_param_file],
        condition=IfCondition(enable_radar),
    )

    # 2026-02-24: Radar near-range cost grid map for short-distance collision layer/visualization.
    radar_cost_grid = Node(
        package='camrod_sensing',
        executable='radar_cost_grid_node',
        name='radar_cost_grid',
        namespace=module_namespace,
        output='screen',
        parameters=[radar_grid_param_file],
        condition=IfCondition(enable_radar_cost_grid),
    )

    lidar_cost_grid = Node(
        package='camrod_sensing',
        executable='lidar_cost_grid_node',
        name='lidar_cost_grid',
        namespace=module_namespace,
        output='screen',
        parameters=[lidar_grid_param_file],
        condition=IfCondition(enable_lidar_cost_grid),
    )

    sensing_checker = Node(
        package='camrod_system',
        executable='module_checker_node.py',
        name='sensing_checker',
        namespace=system_namespace,
        output='screen',
        condition=IfCondition(enable_module_checker),
        parameters=[{
            'module_name': 'sensing',
            'required_nodes': [
                '/sensing/lidar_preprocessor',
                '/sensing/camera_preprocessor',
                '/sensing/platform_velocity_converter',
            ],
            'required_topics': [
                '/sensing/lidar/vanjee/points_raw',
                '/sensing/lidar/near_cost_grid',
                '/sensing/radar/near_cost_grid',
            ],
            'diagnostic_topic': '/diagnostics',
            'status_name': 'sensing/checker',
            'check_period_s': 0.5,
            'warn_throttle_sec': 2.0,
            'publish_ok': True,
        }],
    )

    sensing_diagnostic = Node(
        package='camrod_sensing',
        executable='sensing_diagnostic_node.py',
        name='sensing_diagnostic',
        namespace=module_namespace,
        output='screen',
        parameters=[{
            # HH_260318-00:00 Module-local diagnostic topic (namespaced).
            'diagnostic_topic': 'diagnostic',
            'publish_period_s': 0.2,
            'stale_timeout_s': 1.5,
            # HH_260317-00:00 Relative topic defaults to follow module namespace rule.
            'topic_lidar_filtered': 'lidar/points_filtered',
            'topic_camera_image': 'camera/processed/image',
            'topic_camera_info': 'camera/processed/camera_info',
            'topic_twist_cov': 'platform_velocity_converter/twist_with_covariance',
            'topic_imu': 'imu/data',
            'topic_gnss_navsat': 'gnss/navsatfix',
            'topic_gnss_pose_cov': 'gnss/pose_with_covariance',
            'topic_lidar_cost': 'lidar/near_cost_grid',
            'topic_radar_cost': 'radar/near_cost_grid',
            'topic_radar_front': 'radar/front/range',
            'topic_radar_right1': 'radar/right1/range',
            'topic_radar_right2': 'radar/right2/range',
            'topic_radar_left1': 'radar/left1/range',
            'topic_radar_left2': 'radar/left2/range',
            'topic_radar_rear': 'radar/rear/range',
        }],
    )

    return LaunchDescription([
        param_file_arg,
        lidar_preprocess_param_file_arg,
        camera_preprocess_param_file_arg,
        imu_converter_param_file_arg,
        radar_param_arg,
        radar_sensor_param_arg,
        radar_grid_param_arg,
        lidar_grid_param_arg,
        enable_radar_arg,
        enable_radar_cost_grid_arg,
        enable_lidar_cost_grid_arg,
        enable_lidar_driver_arg,
        vanjee_config_path_arg,
        vanjee_driver_namespace_arg,
        lidar_raw_topic_arg,
        lidar_filtered_topic_arg,
        lidar_imu_packets_topic_arg,
        lidar_diagnostic_topic_arg,
        enable_vanjee_static_tf_arg,
        enable_module_checker_arg,
        module_namespace_arg,
        system_namespace_arg,
        enable_gnss_arg,
        enable_ntrip_arg,
        gnss_param_arg,
        ntrip_param_arg,
        gnss_namespace_arg,
        gnss_navsatfix_topic_arg,
        gnss_rtcm_topic_arg,
        camera_input_image_topic_arg,
        camera_input_camera_info_topic_arg,
        camera_output_image_topic_arg,
        camera_output_camera_info_topic_arg,
        camera_diagnostic_topic_arg,
        imu_input_topic_arg,
        imu_output_topic_arg,
        imu_diagnostic_topic_arg,
        lidar_stack,
        gnss_stack,
        camera_preprocessor,
        velocity_converter,
        radar_sensor,
        radar_cost_grid,
        lidar_cost_grid,
        sensing_diagnostic,
        sensing_checker,
    ])
