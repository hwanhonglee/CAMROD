r"""
NDT localization for camrod (C++), all nodes in ONE process (component container, intra-process).

  ros2 launch ndt_localization_cpp ndt_localization_camrod.launch.py \\
      map_pcd:=<final_results>/pointcloud_map.pcd \\
      camrod_map_info:=<camrod_ws>/src/camrod_map/config/map_info.yaml

Components in /localization/ndt_container
  /map/pointcloud_map_loader              autoware::map_loader::PointCloudMapLoaderNode
  /localization/ndt/points_preprocessor   ndt_localization_cpp::PointsPreprocessor   vanjee points -> ~/points
  /localization/ndt/ndt_scan_matcher      autoware::ndt_scan_matcher::NDTScanMatcher  -> ~/pose_with_covariance
  /localization/ndt/ndt_initial_pose_bridge ndt_localization_cpp::InitialPoseBridge  -> ~/initial_pose
  /localization/ndt/ndt_ekf_gate          ndt_localization_cpp::NdtEkfGate        -> /localization/ndt/pose_for_ekf
Frames: map = camrod map frame (LocalCartesian ENU of map_info.yaml), base = robot_center_link,
robot_center_link -> lidar_link from camrod's robot_state_publisher. Map PCD in the camrod frame.
pose_source: gnss_ndt (GNSS where good, NDT dead reckoning elsewhere, yaw from NDT) | odom (camrod EKF) | gnss
Output: /localization/ndt/pose_with_covariance (PoseWithCovarianceStamped, map, robot_center_link)
        /localization/ndt/pose_for_ekf (same, only while NVTL >= ekf_min_nvtl, EKF covariance) -> camrod EKF pose2
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

NS = '/localization/ndt'


def setup(context):
    a = lambda n: LaunchConfiguration(n).perform(context)
    share = get_package_share_directory('ndt_localization_cpp')
    if not a('map_pcd'):
        raise RuntimeError('ndt_localization: map_pcd:=<pointcloud_map.pcd> is required (camrod map frame PCD)')
    cmi = a('camrod_map_info') or os.path.join(get_package_share_directory('camrod_map'), 'config', 'map_info.yaml')
    # '' = inherit (e.g. a SetParameter(use_sim_time) of an including launch); true/false = force
    sim = {} if not a('use_sim_time') else {'use_sim_time': a('use_sim_time').lower() in ('true', '1')}
    ipc = [{'use_intra_process_comms': True}]
    nodes = [
        ComposableNode(
            package='autoware_map_loader', plugin='autoware::map_loader::PointCloudMapLoaderNode',
            name='pointcloud_map_loader', namespace='/map',
            parameters=[{'pcd_paths_or_directory': [a('map_pcd')], 'pcd_metadata_path': '',
                         'enable_whole_load': False, 'enable_downsampled_whole_load': False,
                         'enable_partial_load': False, 'enable_selected_load': False, 'leaf_size': 3.0,
                         'enable_differential_load': True, **sim}],
            remappings=[('service/get_differential_pcd_map', '/map/get_differential_pointcloud_map')]),
        ComposableNode(
            package='ndt_localization_cpp', plugin='ndt_localization_cpp::PointsPreprocessor',
            name='points_preprocessor', namespace=NS,
            # WLR-750: 0.8..40 m, voxel 0.5 m; vanjee stamps the first point -> restamp to the last ('timestamp')
            # NDT-only LiDAR frame (robot TF untouched): the cloud is relabelled ndt_lidar_link and the node publishes
            # base_frame -> ndt_lidar_link = ndt_lidar_xyz. x 0.99 = robot_params lidar 0.76336 + 0.23: NDT vs RTK on
            # 3 bags put the LiDAR 0.34 m ahead of the GNSS antenna (robot_params gnss x 0.65); with it the NDT robot
            # center matches the GNSS/EKF one (RMS 8-16 cm instead of 20-26 cm). lidar_link rotation is 0.
            parameters=[{'min_range': 0.8, 'max_range': 40.0, 'voxel_size': float(a('voxel_size')),
                         'time_field': 'timestamp', 'output_frame_id': 'ndt_lidar_link',
                         'output_frame_parent': a('base_frame'),
                         'output_frame_xyz': [float(v) for v in a('ndt_lidar_xyz').split()],
                         'output_frame_rpy': [0.0, 0.0, 0.0], **sim}],
            remappings=[('~/input', a('lidar_topic')), ('~/output', NS + '/points')],
            extra_arguments=ipc),
        ComposableNode(
            package='autoware_ndt_scan_matcher', plugin='autoware::ndt_scan_matcher::NDTScanMatcher',
            name='ndt_scan_matcher', namespace=NS,
            parameters=[f'{share}/config/ndt_scan_matcher_vanjee.param.yaml',
                        {'frame.base_frame': a('base_frame'), 'frame.map_frame': 'map', **sim}],
            remappings=[('points_raw', NS + '/points'),
                        ('ekf_pose_with_covariance', NS + '/initial_pose'),
                        ('regularization_pose_with_covariance', NS + '/regularization_pose'),
                        ('trigger_node_srv', NS + '/trigger_node'),
                        ('pcd_loader_service', '/map/get_differential_pointcloud_map'),
                        ('ndt_pose', NS + '/pose'),
                        ('ndt_pose_with_covariance', NS + '/pose_with_covariance')]),
        # no use_intra_process_comms here: the NDT has transient_local publishers, which rclcpp refuses with
        # intra-process ("intraprocess communication allowed only with volatile durability")
        ComposableNode(
            package='ndt_localization_cpp', plugin='ndt_localization_cpp::InitialPoseBridge',
            name='ndt_initial_pose_bridge', namespace=NS,
            parameters=[{'source': a('pose_source'), 'camrod_map_info': cmi, 'map_frame': 'map',
                         # GNSS antenna in robot_center_link: x/y = robot_params.yaml gnss (same frame as the EKF GNSS
                         # input), z 1.42 m measured against the map ground
                         'antenna_xyz': [float(v) for v in a('antenna_xyz').split()],
                         'ndt_zones_file': a('ndt_zones_file'), **sim}],
            remappings=[('~/output', NS + '/initial_pose'), ('~/ndt_pose', NS + '/pose_with_covariance'),
                        ('~/navpvt', a('navpvt_topic')), ('~/odom', '/localization/odometry_ros'),
                        ('~/imu', '/sensing/imu/data_ros'), ('~/trigger_node', NS + '/trigger_node'),
                        ('~/mode', NS + '/initial_pose_mode')]),
        ComposableNode(
            package='ndt_localization_cpp', plugin='ndt_localization_cpp::NdtEkfGate',
            name='ndt_ekf_gate', namespace=NS,
            # NDT -> EKF (pose2 in camrod ekf.yaml) only while NVTL >= min_nvtl (Vanjee normal level ~1.4)
            parameters=[{'min_nvtl': float(a('ekf_min_nvtl')), 'good_nvtl': 1.35, 'xy_std': 0.10, 'yaw_std_deg': 2.0,
                         'max_scale': 4.0, **sim}],
            remappings=[('~/ndt_pose', NS + '/pose_with_covariance'),
                        ('~/nvtl', NS + '/nearest_voxel_transformation_likelihood'),
                        ('~/output', NS + '/pose_for_ekf')],
            extra_arguments=ipc),
    ]
    return [ComposableNodeContainer(name='ndt_container', namespace='/localization', package='rclcpp_components',
                                    executable='component_container_mt', composable_node_descriptions=nodes,
                                    parameters=[sim] if sim else [], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_pcd', default_value='', description='NDT map PCD in the camrod map frame (required)'),
        DeclareLaunchArgument('camrod_map_info', default_value='',
                              description="map_info.yaml of the map origin ('' = camrod_map/config/map_info.yaml)"),
        DeclareLaunchArgument('pose_source', default_value='gnss_ndt', description='gnss_ndt | odom | gnss'),
        DeclareLaunchArgument('base_frame', default_value='robot_center_link'),
        DeclareLaunchArgument('lidar_topic', default_value='/sensing/lidar/vanjee/points_raw'),
        DeclareLaunchArgument('navpvt_topic', default_value='/sensing/gnss/ublox_gps_node/navpvt'),
        DeclareLaunchArgument('antenna_xyz', default_value='0.65 0.45 1.42'),
        DeclareLaunchArgument('ndt_lidar_xyz', default_value='0.99 0.0 0.59538',
                              description='LiDAR mount used by NDT only (robot_center_link), robot TF unchanged'),
        DeclareLaunchArgument('voxel_size', default_value='0.5'),
        DeclareLaunchArgument('ndt_zones_file', default_value=''),
        DeclareLaunchArgument('ekf_min_nvtl', default_value='1.2', description='NDT -> EKF only at NVTL >= this'),
        DeclareLaunchArgument('use_sim_time', default_value='', description="'' = inherit, true / false"),
        OpaqueFunction(function=setup),
    ])
