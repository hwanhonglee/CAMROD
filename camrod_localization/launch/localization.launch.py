import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def get_pkg_share(pkg: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(pkg)


def extract_map_ros_params(map_info_cfg: dict) -> dict:
    # HH_260330: Keep map_info parsing tolerant to key layout changes.
    if not isinstance(map_info_cfg, dict):
        return {}
    for key in (
        '/map/lanelet2_map',
        'map/lanelet2_map',
        'lanelet2_map',
        '/lanelet2_map',
    ):
        val = map_info_cfg.get(key)
        if isinstance(val, dict):
            params = val.get('ros__parameters')
            if isinstance(params, dict):
                return params
    for val in map_info_cfg.values():
        if isinstance(val, dict):
            params = val.get('ros__parameters')
            if isinstance(params, dict):
                return params
    return {}


def generate_launch_description():
    loc_cfg_dir = os.path.join(get_pkg_share('camrod_localization'), 'config')
    # HH_260330: Standalone localization launch uses package-local map config by default.
    map_info_path = os.path.join(get_pkg_share('camrod_map'), 'config', 'map_info.yaml')
    map_info_cfg = {}
    try:
        with open(map_info_path, 'r', encoding='utf-8') as f:
            map_info_cfg = yaml.safe_load(f) or {}
    except Exception:
        map_info_cfg = {}
    map_params = extract_map_ros_params(map_info_cfg)
    # HH_260327: Read unified reference from ros__parameters so map_info.yaml
    # stays compatible with ROS2 params parser used by lanelet2_map_node.
    map_ref_llh = {
        'lat': map_params.get('reference_lat', map_params.get('offset_lat', 0.0)),
        'lon': map_params.get('reference_lon', map_params.get('offset_lon', 0.0)),
        'alt': map_params.get('reference_alt', map_params.get('offset_alt', 0.0)),
    }
    map_ref_utm = {
        'easting': map_params.get('reference_utm_easting', 0.0),
        'northing': map_params.get('reference_utm_northing', 0.0),
        'alt': map_params.get('reference_utm_alt', map_ref_llh['alt']),
    }

    filter_type_arg = DeclareLaunchArgument(
        'filter_type',
        default_value='ekf',
        description='Localization filter type: ekf or eskf',
    )

    navsat_topic_arg = DeclareLaunchArgument(
        'navsat_topic',
        default_value='/sensing/gnss/ublox_gps_node/fix',
        description='NavSatFix input topic',
    )

    gnss_pose_arg = DeclareLaunchArgument(
        'gnss_pose_topic',
        default_value='/sensing/gnss/pose',
        description='Output local GNSS pose topic',
    )

    gnss_pose_cov_arg = DeclareLaunchArgument(
        'gnss_pose_cov_topic',
        default_value='/sensing/gnss/pose_with_covariance',
        description='Output local GNSS pose with covariance topic',
    )

    # HH_260327: Expose origin args so bringup can drive localization origin from map_info.yaml.
    origin_lat_arg = DeclareLaunchArgument(
        'origin_lat',
        default_value=str(map_ref_llh.get('lat', map_params.get('offset_lat', 0.0))),
        description='Localization origin latitude',
    )
    origin_lon_arg = DeclareLaunchArgument(
        'origin_lon',
        default_value=str(map_ref_llh.get('lon', map_params.get('offset_lon', 0.0))),
        description='Localization origin longitude',
    )
    origin_alt_arg = DeclareLaunchArgument(
        'origin_alt',
        default_value=str(map_ref_llh.get('alt', map_params.get('offset_alt', 0.0))),
        description='Localization origin altitude',
    )
    yaw_offset_deg_arg = DeclareLaunchArgument(
        'yaw_offset_deg',
        default_value=str(map_params.get('yaw_offset_deg', 0.0)),
        description='Localization yaw offset [deg]',
    )
    utm_origin_easting_arg = DeclareLaunchArgument(
        'utm_origin_easting',
        default_value=str(map_ref_utm.get('easting', 0.0)),
        description='Localization UTM origin easting [m]',
    )
    utm_origin_northing_arg = DeclareLaunchArgument(
        'utm_origin_northing',
        default_value=str(map_ref_utm.get('northing', 0.0)),
        description='Localization UTM origin northing [m]',
    )
    utm_origin_alt_arg = DeclareLaunchArgument(
        'utm_origin_alt',
        default_value=str(map_ref_utm.get('alt', map_ref_llh.get('alt', 0.0))),
        description='Localization UTM origin altitude [m]',
    )
    rotate_latlon_xy_by_yaw_offset_arg = DeclareLaunchArgument(
        'rotate_latlon_xy_by_yaw_offset',
        default_value='true' if bool(map_params.get('rotate_latlon_xy_by_yaw_offset', True)) else 'false',
        description='Rotate LLH ENU XY by yaw_offset_deg',
    )

    ekf_param_arg = DeclareLaunchArgument(
        'ekf_param_file',
        # HH_260330: Standalone localization launch uses package-local config by default.
        default_value=os.path.join(loc_cfg_dir, 'filter', 'ekf.yaml'),
        description='EKF parameter file',
    )

    eskf_param_arg = DeclareLaunchArgument(
        'eskf_param_file',
        # HH_260330: Standalone localization launch uses package-local config by default.
        default_value=os.path.join(loc_cfg_dir, 'filter', 'eskf.yaml'),
        description='ESKF parameter file',
    )


    wheel_bridge_arg = DeclareLaunchArgument(
        'wheel_bridge_enable',
        default_value='true',
        description='Enable wheel odometry bridge',
    )

    # HH_260326: Add launch-time wheel input/output selection for bridge generalization.
    wheel_input_topic_arg = DeclareLaunchArgument(
        'wheel_input_topic',
        default_value='/platform/status/wheel',
        description='Wheel bridge input topic',
    )

    wheel_input_type_arg = DeclareLaunchArgument(
        'wheel_input_type',
        default_value='twist',
        description='Wheel bridge input type: twist, avg_odom, or nav_odom',
    )

    wheel_output_topic_arg = DeclareLaunchArgument(
        'wheel_output_topic',
        default_value='/platform/wheel/odometry',
        description='Unified wheel odometry output topic',
    )

    # HH_260326: Add nav_msgs wheel odometry output for EKF type compatibility.
    wheel_nav_output_topic_arg = DeclareLaunchArgument(
        'wheel_nav_output_topic',
        default_value='/platform/wheel/nav_odometry',
        description='Wheel bridge nav_msgs/Odometry output topic for EKF',
    )

    # HH_260326: Temporary ESKF wheel source override to /rmp401/odom.
    eskf_force_rmp401_odom_arg = DeclareLaunchArgument(
        'eskf_force_rmp401_odom',
        default_value='true',
        description='When true, force ESKF wheel bridge input to /rmp401/odom (nav_odom)',
    )

    eskf_rmp401_odom_topic_arg = DeclareLaunchArgument(
        'eskf_rmp401_odom_topic',
        default_value='/rmp401/odom',
        description='Temporary nav odometry topic used when eskf_force_rmp401_odom is true',
    )

    drop_zone_yaml_arg = DeclareLaunchArgument(
        'drop_zones_yaml',
        # HH_260330: Standalone localization launch uses package-local config by default.
        default_value=os.path.join(loc_cfg_dir, 'drop_zones.yaml'),
        description='Drop zone definition YAML for drop_zone_matcher',
    )

    kimera_bridge_enable_arg = DeclareLaunchArgument(
        'kimera_bridge_enable',
        default_value='false',
        description='Enable Kimera CSV bridge',
    )

    kimera_bridge_param_arg = DeclareLaunchArgument(
        'kimera_bridge_param_file',
        # HH_260330: Standalone localization launch uses package-local config by default.
        default_value=os.path.join(loc_cfg_dir, 'kimera_bridge.yaml'),
        description='Kimera bridge parameter file',
    )

    pose_selector_enable_arg = DeclareLaunchArgument(
        'pose_selector_enable',
        default_value='false',
        description='Enable pose selector',
    )

    pose_selector_param_arg = DeclareLaunchArgument(
        'pose_selector_param_file',
        # HH_260330: Standalone localization launch uses package-local config by default.
        default_value=os.path.join(loc_cfg_dir, 'filter', 'pose_selector.yaml'),
        description='Pose selector parameter file',
    )

    module_validator_enable_arg = DeclareLaunchArgument(
        'enable_module_validator',
        default_value='true',
        description='Enable localization module validator publisher',
    )

    module_namespace_arg = DeclareLaunchArgument(
        'module_namespace',
        default_value='localization',
        description='Namespace for localization module nodes',
    )

    platform_namespace_arg = DeclareLaunchArgument(
        'platform_namespace',
        default_value='platform',
        description='Namespace for platform interface nodes',
    )

    system_namespace_arg = DeclareLaunchArgument(
        'system_namespace',
        default_value='system',
        description='Namespace for system validator nodes',
    )

    filter_type = LaunchConfiguration('filter_type')
    navsat_topic = LaunchConfiguration('navsat_topic')
    gnss_pose = LaunchConfiguration('gnss_pose_topic')
    gnss_pose_cov = LaunchConfiguration('gnss_pose_cov_topic')
    origin_lat = LaunchConfiguration('origin_lat')
    origin_lon = LaunchConfiguration('origin_lon')
    origin_alt = LaunchConfiguration('origin_alt')
    yaw_offset_deg = LaunchConfiguration('yaw_offset_deg')
    utm_origin_easting = LaunchConfiguration('utm_origin_easting')
    utm_origin_northing = LaunchConfiguration('utm_origin_northing')
    utm_origin_alt = LaunchConfiguration('utm_origin_alt')
    rotate_latlon_xy_by_yaw_offset = LaunchConfiguration('rotate_latlon_xy_by_yaw_offset')
    ekf_param = LaunchConfiguration('ekf_param_file')
    eskf_param = LaunchConfiguration('eskf_param_file')
    wheel_bridge_enable = LaunchConfiguration('wheel_bridge_enable')
    wheel_input_topic = LaunchConfiguration('wheel_input_topic')
    wheel_input_type = LaunchConfiguration('wheel_input_type')
    wheel_output_topic = LaunchConfiguration('wheel_output_topic')
    wheel_nav_output_topic = LaunchConfiguration('wheel_nav_output_topic')
    eskf_force_rmp401_odom = LaunchConfiguration('eskf_force_rmp401_odom')
    eskf_rmp401_odom_topic = LaunchConfiguration('eskf_rmp401_odom_topic')
    drop_zones_yaml = LaunchConfiguration('drop_zones_yaml')
    kimera_bridge_enable = LaunchConfiguration('kimera_bridge_enable')
    kimera_bridge_param = LaunchConfiguration('kimera_bridge_param_file')
    pose_selector_enable = LaunchConfiguration('pose_selector_enable')
    pose_selector_param = LaunchConfiguration('pose_selector_param_file')
    enable_module_validator = LaunchConfiguration('enable_module_validator')
    module_namespace = LaunchConfiguration('module_namespace')
    platform_namespace = LaunchConfiguration('platform_namespace')
    system_namespace = LaunchConfiguration('system_namespace')

    is_ekf = PythonExpression(["'", filter_type, "' == 'ekf'"])
    selector_on = PythonExpression(["'", pose_selector_enable, "' == 'true'"])
    selector_off = PythonExpression(["'", pose_selector_enable, "' == 'false'"])

    ekf_direct_cond = IfCondition(PythonExpression([
        "'", filter_type, "' == 'ekf' and '", pose_selector_enable, "' == 'false'"
    ]))
    ekf_selector_cond = IfCondition(PythonExpression([
        "'", filter_type, "' == 'ekf' and '", pose_selector_enable, "' == 'true'"
    ]))
    eskf_direct_cond = IfCondition(PythonExpression([
        "'", filter_type, "' == 'eskf' and '", pose_selector_enable, "' == 'false'"
    ]))
    eskf_selector_cond = IfCondition(PythonExpression([
        "'", filter_type, "' == 'eskf' and '", pose_selector_enable, "' == 'true'"
    ]))

    # HH_260327: Single map->odom TF authority for both EKF/ESKF.
    # This removes per-filter map->odom duplication risk when switching modes.
    ekf_map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ekf_map_to_odom_tf',
        namespace=module_namespace,
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(PythonExpression([
            "'", filter_type, "' == 'ekf' or '", filter_type, "' == 'eskf'"
        ])),
    )

    # HH_260326: Remove EKF odom->robot_base_link static fallback to avoid duplicate TF authority.
    # EKF must be the single publisher for odom->robot_base_link in EKF mode.

    navsat_to_pose = Node(
        package='camrod_localization',
        executable='navsat_to_pose_node',
        name='navsat_to_pose',
        namespace=module_namespace,
        output='screen',
        parameters=[{
            'navsat_topic': navsat_topic,
            'pose_topic': gnss_pose,
            'pose_cov_topic': gnss_pose_cov,
            # HH_260327: Single-source map/localization reference from map_info.yaml.
            'origin_lat': origin_lat,
            'origin_lon': origin_lon,
            'origin_alt': origin_alt,
            'yaw_offset_deg': yaw_offset_deg,
            'utm_origin_easting': utm_origin_easting,
            'utm_origin_northing': utm_origin_northing,
            'utm_origin_alt': utm_origin_alt,
            'rotate_latlon_xy_by_yaw_offset': rotate_latlon_xy_by_yaw_offset,
            'map_frame_id': 'map',
            'publish_covariance': True,
        }],
    )

    # EKF direct -> final topics
    ekf_direct = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_main',
        namespace=module_namespace,
        output='screen',
        parameters=[ekf_param, {
            # HH_260326: Force EKF wheel source to nav_msgs/Odometry topic.
            'odom0': wheel_nav_output_topic,
        }],
        condition=ekf_direct_cond,
    )

    # EKF selector mode -> primary topics, TF off
    ekf_for_selector = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_main',
        namespace=module_namespace,
        output='screen',
        parameters=[ekf_param, {
            'publish_tf': False,
            # HH_260326: Force EKF wheel source to nav_msgs/Odometry topic.
            'odom0': wheel_nav_output_topic,
        }],
        remappings=[
            ('odometry/filtered', 'primary/odometry'),
        ],
        condition=ekf_selector_cond,
    )

    # ESKF direct -> final topics
    eskf_direct = Node(
        package='camrod_localization',
        executable='localization_eskf_node',
        name='eskf_filter',
        namespace=module_namespace,
        output='screen',
        parameters=[eskf_param, {
            # HH_260327: map->odom is published once by static_transform_publisher.
            'publish_map_to_odom_tf': False,
        }],
        condition=eskf_direct_cond,
    )

    # ESKF selector mode -> primary topics, TF off
    eskf_for_selector = Node(
        package='camrod_localization',
        executable='localization_eskf_node',
        name='eskf_filter',
        namespace=module_namespace,
        output='screen',
        parameters=[eskf_param, {
            'pose_topic': '/localization/primary/pose',
            'pose_cov_topic': '/localization/primary/pose_with_covariance',
            'odom_topic': '/localization/primary/odometry',
            'twist_topic': '/localization/primary/twist',
            'publish_tf': False,
            'publish_map_to_odom_tf': False,
        }],
        condition=eskf_selector_cond,
    )

    # EKF direct mode only
    odom_to_pose_direct = Node(
        package='camrod_localization',
        executable='odometry_to_pose_node',
        name='odom_to_pose_bridge',
        namespace=module_namespace,
        output='screen',
        parameters=[{
            'input_topic': '/localization/odometry/filtered',
            'pose_topic': '/localization/pose',
            'pose_cov_topic': '/localization/pose_with_covariance',
            # HH_260327: Export EKF pose topics in map frame for consistency with ESKF.
            'output_frame_id': 'map',
        }],
        condition=ekf_direct_cond,
    )

    # EKF selector mode only
    odom_to_pose_primary = Node(
        package='camrod_localization',
        executable='odometry_to_pose_node',
        name='primary_odom_to_pose_bridge',
        namespace=module_namespace,
        output='screen',
        parameters=[{
            'input_topic': '/localization/primary/odometry',
            'pose_topic': '/localization/primary/pose',
            'pose_cov_topic': '/localization/primary/pose_with_covariance',
            'output_frame_id': 'map',
        }],
        condition=ekf_selector_cond,
    )

    wheel_bridge = Node(
        package='camrod_localization',
        executable='wheel_odometry_bridge_node',
        name='wheel_odometry_bridge',
        namespace=platform_namespace,
        output='screen',
        parameters=[{
            # HH_260326: Pass launch-configurable wheel bridge input/output parameters.
            'input_topic': PythonExpression([
                "'", eskf_rmp401_odom_topic, "' if ('", filter_type, "' == 'eskf' and '",
                eskf_force_rmp401_odom, "' == 'true') else '", wheel_input_topic, "'"
            ]),
            'output_topic': wheel_output_topic,
            'nav_output_topic': wheel_nav_output_topic,
            'odom_frame_id': 'odom',
            'base_frame_id': 'robot_base_link',
            'input_type': PythonExpression([
                "'nav_odom' if ('", filter_type, "' == 'eskf' and '",
                eskf_force_rmp401_odom, "' == 'true') else '", wheel_input_type, "'"
            ]),
        }],
        condition=IfCondition(wheel_bridge_enable),
    )

    drop_zone_matcher = Node(
        package='camrod_localization',
        executable='drop_zone_matcher_node',
        name='drop_zone_matcher',
        namespace=module_namespace,
        output='screen',
        parameters=[{
            # HH_260326: Pass drop zone data YAML directly to matcher.
            'drop_zones_yaml': drop_zones_yaml,
        }],
    )

    kimera_csv_bridge = Node(
        package='camrod_localization',
        executable='kimera_csv_bridge_node',
        name='kimera_csv_bridge',
        namespace=module_namespace,
        output='screen',
        parameters=[kimera_bridge_param],
        condition=IfCondition(kimera_bridge_enable),
    )

    pose_selector = Node(
        package='camrod_localization',
        executable='localization_pose_selector_node',
        name='pose_selector',
        namespace=module_namespace,
        output='screen',
        parameters=[pose_selector_param, {
            'primary_source_label': filter_type,
            'publish_selected_tf': True,
            'base_frame_id': 'robot_base_link',
        }],
        condition=IfCondition(selector_on),
    )


    return LaunchDescription([
        filter_type_arg,
        navsat_topic_arg,
        gnss_pose_arg,
        gnss_pose_cov_arg,
        origin_lat_arg,
        origin_lon_arg,
        origin_alt_arg,
        yaw_offset_deg_arg,
        utm_origin_easting_arg,
        utm_origin_northing_arg,
        utm_origin_alt_arg,
        rotate_latlon_xy_by_yaw_offset_arg,
        ekf_param_arg,
        eskf_param_arg,
        wheel_bridge_arg,
        wheel_input_topic_arg,
        wheel_input_type_arg,
        wheel_output_topic_arg,
        wheel_nav_output_topic_arg,
        eskf_force_rmp401_odom_arg,
        eskf_rmp401_odom_topic_arg,
        drop_zone_yaml_arg,
        kimera_bridge_enable_arg,
        kimera_bridge_param_arg,
        pose_selector_enable_arg,
        pose_selector_param_arg,
        module_validator_enable_arg,
        module_namespace_arg,
        platform_namespace_arg,
        system_namespace_arg,

        navsat_to_pose,
        ekf_map_to_odom_tf,
        ekf_direct,
        ekf_for_selector,
        eskf_direct,
        eskf_for_selector,
        odom_to_pose_direct,
        odom_to_pose_primary,
        wheel_bridge,
        drop_zone_matcher,
        kimera_csv_bridge,
        pose_selector,
    ])
