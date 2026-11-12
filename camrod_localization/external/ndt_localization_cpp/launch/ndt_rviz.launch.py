r"""
NDT matching view in its own RViz, separate from the operator RViz.

Shows the PCD map, the NDT-aligned LiDAR scan and the lanelet2 map (/map/markers from camrod_map, when it runs).

  ros2 launch ndt_localization_cpp ndt_rviz.launch.py map_pcd:=<pointcloud_map.pcd>

The scan (/localization/ndt/points_aligned: the ~2k preprocessed points placed with the NDT result, map frame) lies on
the map when the match is right. The map comes from a viz-only pointcloud_map_loader that voxel-downsamples the PCD
once (viz_leaf_size, default 0.5 m) and publishes it latched on /localization/ndt/viz/map; the localization map loader
is not touched. Both run only while this launch runs, so the robot pays nothing when the view is closed.
Cost while open: the map loader reads the whole PCD once at start (~1 GB peak RAM, a few seconds), then idles.
Heights: the map frame keeps the real height (ground ~290 m), the lanelet2 markers and the 2D EKF are at z = 0;
ndt_viz_flatten.py shifts map/scan/NDT pose by the current NDT z (TF map -> map_viz) so they overlap in RViz.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    a = lambda n: LaunchConfiguration(n).perform(context)
    if not a('map_pcd'):
        raise RuntimeError('ndt_rviz: map_pcd:=<pointcloud_map.pcd> is required')
    # '' = inherit (e.g. a SetParameter(use_sim_time) of an including launch); true/false = force
    sim = {} if not a('use_sim_time') else {'use_sim_time': a('use_sim_time').lower() in ('true', '1')}
    rviz_cfg = a('rviz_config') or (get_package_share_directory('ndt_localization_cpp') + '/rviz/ndt_viz.rviz')
    return [
        Node(package='autoware_map_loader', executable='autoware_pointcloud_map_loader',
             name='ndt_viz_map_loader', namespace='/localization/ndt/viz', output='log',
             parameters=[{'pcd_paths_or_directory': [a('map_pcd')], 'pcd_metadata_path': '',
                          'enable_whole_load': False, 'enable_downsampled_whole_load': True,
                          'enable_partial_load': False, 'enable_selected_load': False,
                          'enable_differential_load': False, 'leaf_size': float(a('viz_leaf_size')), **sim}],
             remappings=[('output/debug/downsampled_pointcloud_map', '/localization/ndt/viz/map')]),
        # map/scan/NDT pose are at the map's real height (~290 m); move them to the 2D plane of the lanelet2/EKF
        Node(package='ndt_localization_cpp', executable='ndt_viz_flatten.py', name='ndt_viz_flatten', output='log',
             parameters=[sim] if sim else []),
        Node(package='rviz2', executable='rviz2', name='ndt_rviz', output='log',
             arguments=['-d', rviz_cfg], parameters=[sim] if sim else []),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_pcd', default_value='', description='NDT map PCD (same file as the NDT uses)'),
        DeclareLaunchArgument('viz_leaf_size', default_value='0.5', description='map voxel size for display [m]'),
        DeclareLaunchArgument('rviz_config', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='', description="'' = inherit, true / false"),
        OpaqueFunction(function=setup),
    ])
