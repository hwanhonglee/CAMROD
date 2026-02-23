import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    navsat_topic_arg = DeclareLaunchArgument(
        'navsat_topic',
        default_value='/sensing/gnss/navsatfix',
        description='GNSS NavSatFix topic for navsat_to_pose',
    )
    gnss_pose_arg = DeclareLaunchArgument(
        'gnss_pose_topic',
        default_value='/sensing/gnss/pose',
        description='Intermediate GNSS pose topic',
    )
    # HH_260123 Deprecated: navsat_to_pose publishes covariance directly.
    gnss_pose_cov_arg = DeclareLaunchArgument(
        'gnss_pose_cov_topic',
        default_value='/sensing/gnss/pose_with_covariance',
        description='(Deprecated) GNSS pose with covariance topic',
    )
    origin_lat_arg = DeclareLaunchArgument('origin_lat', default_value='0.0', description='Map origin latitude')
    origin_lon_arg = DeclareLaunchArgument('origin_lon', default_value='0.0', description='Map origin longitude')
    origin_alt_arg = DeclareLaunchArgument('origin_alt', default_value='0.0', description='Map origin altitude')
    use_eskf_arg = DeclareLaunchArgument(
        'use_eskf',
        default_value='true',
        description='Use ESKF (true) or legacy robot_localization EKF (false)',
    )
    eskf_param_arg = DeclareLaunchArgument(
        'eskf_param_file',
        default_value=os.path.join(
            get_package_share_directory('camping_cart_bringup'),
            'config', 'localization', 'eskf.yaml'),
        description='ESKF parameter file',
    )
    supervisor_param_arg = DeclareLaunchArgument(
        'supervisor_param_file',
        default_value=os.path.join(
            get_package_share_directory('camping_cart_bringup'),
            'config', 'localization', 'supervisor.yaml'),
        description='Localization supervisor parameter file',
    )
    wheel_bridge_arg = DeclareLaunchArgument(
        'wheel_bridge_enable',
        default_value='true',
        description='Enable wheel odometry bridge (/platform/status/wheel -> /platform/wheel/odometry)',
    )
    drop_zone_param_arg = DeclareLaunchArgument(
        'drop_zone_param_file',
        default_value=os.path.join(
            get_package_share_directory('camping_cart_bringup'),
            'config', 'localization', 'drop_zone_matcher.yaml'),
        description='Drop zone matcher parameter file',
    )
    kimera_bridge_enable_arg = DeclareLaunchArgument(
        'kimera_bridge_enable',
        default_value='false',
        description='Enable Kimera CSV pose bridge (/localization/kimera_vio/* outputs)',
    )
    kimera_bridge_param_arg = DeclareLaunchArgument(
        'kimera_bridge_param_file',
        default_value=os.path.join(
            get_package_share_directory('camping_cart_localization'),
            'config', 'kimera_bridge.yaml'),
        description='Kimera CSV bridge parameter file',
    )
    pose_selector_enable_arg = DeclareLaunchArgument(
        'pose_selector_enable',
        default_value='true',
        description='Enable ESKF/Kimera pose selector for automatic fallback',
    )
    pose_selector_param_arg = DeclareLaunchArgument(
        'pose_selector_param_file',
        default_value=os.path.join(
            get_package_share_directory('camping_cart_localization'),
            'config', 'pose_selector.yaml'),
        description='Pose selector parameter file',
    )

    navsat_topic = LaunchConfiguration('navsat_topic')
    gnss_pose = LaunchConfiguration('gnss_pose_topic')
    gnss_pose_cov = LaunchConfiguration('gnss_pose_cov_topic')
    origin_lat = LaunchConfiguration('origin_lat')
    origin_lon = LaunchConfiguration('origin_lon')
    origin_alt = LaunchConfiguration('origin_alt')
    use_eskf = LaunchConfiguration('use_eskf')
    eskf_param = LaunchConfiguration('eskf_param_file')
    supervisor_param = LaunchConfiguration('supervisor_param_file')
    wheel_bridge_enable = LaunchConfiguration('wheel_bridge_enable')
    drop_zone_param = LaunchConfiguration('drop_zone_param_file')
    kimera_bridge_enable = LaunchConfiguration('kimera_bridge_enable')
    kimera_bridge_param = LaunchConfiguration('kimera_bridge_param_file')
    pose_selector_enable = LaunchConfiguration('pose_selector_enable')
    pose_selector_param = LaunchConfiguration('pose_selector_param_file')

    # HH_260121 GNSS -> UTM pose + world->map static TF. Publishes Pose and PoseWithCovariance.
    navsat_to_pose = Node(
        package='camping_cart_localization',
        executable='navsat_to_pose_node',
        name='navsat_to_pose',
        namespace='localization',
        output='screen',
        parameters=[{
            'navsat_topic': navsat_topic,
            'pose_topic': gnss_pose,
            'pose_cov_topic': gnss_pose_cov,
            'map_frame_id': 'map',
            'origin_lat': origin_lat,
            'origin_lon': origin_lon,
            'origin_alt': origin_alt,
            'publish_covariance': True,
        }],
    )

    # HH_260121 EKF fusion of GNSS/IMU/odom (legacy fallback).
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_main',
        namespace='localization',
        output='screen',
        parameters=[os.path.join(get_pkg_share('camping_cart_bringup'), 'config', 'localization', 'ekf.yaml')],
        condition=UnlessCondition(use_eskf),
    )

    # HH_260123 ESKF fusion (IMU + GNSS + wheel) with diagnostics.
    # 2026-02-09: Direct mode (selector disabled) publishes canonical /localization/* topics.
    eskf_direct = Node(
        package='camping_cart_localization',
        executable='localization_eskf_node',
        name='eskf_filter',
        namespace='localization',
        output='screen',
        parameters=[eskf_param],
        condition=IfCondition(PythonExpression([
            "'", use_eskf, "' == 'true' and '", pose_selector_enable, "' == 'false'"
        ])),
    )

    # 2026-02-09: Selector mode publishes ESKF to /localization/eskf/*, then selector outputs final /localization/*.
    eskf_for_selector = Node(
        package='camping_cart_localization',
        executable='localization_eskf_node',
        name='eskf_filter',
        namespace='localization',
        output='screen',
        parameters=[eskf_param, {
            'pose_topic': '/localization/eskf/pose',
            'pose_cov_topic': '/localization/eskf/pose_with_covariance',
            'odom_topic': '/localization/eskf/odometry',
            'twist_topic': '/localization/eskf/twist',
        }],
        condition=IfCondition(PythonExpression([
            "'", use_eskf, "' == 'true' and '", pose_selector_enable, "' == 'true'"
        ])),
    )

    # HH_260121 Convert filtered odom to pose topics for downstream consumers.
    odom_to_pose = Node(
        package='camping_cart_localization',
        executable='odometry_to_pose_node',
        name='odom_to_pose_bridge',
        namespace='localization',
        output='screen',
        parameters=[{
            'input_topic': '/localization/odometry/filtered',
            'pose_topic': '/localization/pose',
            'pose_cov_topic': '/localization/pose_with_covariance',
        }],
        # HH_260123 EKF(legacy)만 pose 브리지가 필요함. ESKF는 자체 pose/pose_cov를 퍼블리시하므로 중복 방지.
        condition=UnlessCondition(use_eskf),
    )

    # HH_260123 Supervisor for localization mode/confidence.
    supervisor = Node(
        package='camping_cart_localization',
        executable='localization_supervisor_node',
        name='supervisor',
        namespace='localization',
        output='screen',
        parameters=[supervisor_param],
    )

    # HH_260123 Optional wheel odometry bridge (status/wheel -> wheel/odometry).
    wheel_bridge = Node(
        package='camping_cart_localization',
        executable='wheel_odometry_bridge_node',
        name='wheel_odometry_bridge',
        namespace='platform',
        output='screen',
        parameters=[{
            'input_topic': '/platform/status/wheel',
            'output_topic': '/platform/wheel/odometry',
            'odom_frame_id': 'odom',
            'base_frame_id': 'robot_base_link',
            'input_type': 'twist',
        }],
        condition=IfCondition(wheel_bridge_enable),
    )

    # 2026-02-02 Drop zone matcher (initial localization OK gate).
    drop_zone_matcher = Node(
        package='camping_cart_localization',
        executable='drop_zone_matcher_node',
        name='drop_zone_matcher',
        namespace='localization',
        output='screen',
        parameters=[drop_zone_param],
    )

    # 2026-02-09: Optional bridge from zedLiveVIO CSV outputs into ROS fallback pose topics.
    kimera_csv_bridge = Node(
        package='camping_cart_localization',
        executable='kimera_csv_bridge_node',
        name='kimera_csv_bridge',
        namespace='localization',
        output='screen',
        parameters=[kimera_bridge_param],
        condition=IfCondition(kimera_bridge_enable),
    )

    # 2026-02-09: Automatic primary/fallback selector for /localization/pose* and /localization/odometry/filtered.
    pose_selector = Node(
        package='camping_cart_localization',
        executable='localization_pose_selector_node',
        name='pose_selector',
        namespace='localization',
        output='screen',
        parameters=[pose_selector_param],
        condition=IfCondition(PythonExpression([
            "'", use_eskf, "' == 'true' and '", pose_selector_enable, "' == 'true'"
        ])),
    )

    # HH_260121 Health monitor for localization pipeline.
    health_monitor = Node(
        package='camping_cart_localization',
        executable='localization_health_monitor_node',
        name='health_monitor',
        namespace='localization',
        output='screen',
        parameters=[os.path.join(get_pkg_share('camping_cart_bringup'), 'config', 'localization', 'health_monitor.yaml')],
    )

    return LaunchDescription([
        navsat_topic_arg,
        gnss_pose_arg,
        gnss_pose_cov_arg,
        origin_lat_arg,
        origin_lon_arg,
        origin_alt_arg,
        use_eskf_arg,
        eskf_param_arg,
        supervisor_param_arg,
        wheel_bridge_arg,
        drop_zone_param_arg,
        kimera_bridge_enable_arg,
        kimera_bridge_param_arg,
        pose_selector_enable_arg,
        pose_selector_param_arg,
        navsat_to_pose,
        ekf,
        eskf_direct,
        eskf_for_selector,
        odom_to_pose,
        supervisor,
        wheel_bridge,
        health_monitor,
        drop_zone_matcher,
        kimera_csv_bridge,
        pose_selector,
    ])


def get_pkg_share(pkg):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(pkg)
