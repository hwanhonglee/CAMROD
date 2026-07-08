import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_default_frontend_dir() -> str:
    """Resolve frontend build directory with package-share-first fallback."""
    share_dir = get_package_share_directory('camrod_ui')
    # File is at camrod_ui_robot/launch/ui.launch.py → go up 2 levels to package root.
    source_pkg_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))

    env_frontend_dir = os.environ.get('CAMROD_UI_FRONTEND_DIR', '').strip()
    if env_frontend_dir and os.path.isdir(env_frontend_dir):
        return env_frontend_dir

    source_frontend_dir = os.path.join(source_pkg_root, 'camrod_ui_robot', 'assets', 'frontend', 'build')
    if os.path.isdir(source_frontend_dir):
        return source_frontend_dir

    packaged_frontend_dir = os.path.join(share_dir, 'camrod_ui_robot', 'assets', 'frontend', 'build')
    if os.path.isdir(packaged_frontend_dir):
        return packaged_frontend_dir

    return os.path.join(share_dir, 'camrod_ui_robot', 'assets', 'web')


def _resolve_default_camping_sites_yaml() -> str:
    """Resolve planning camping-sites YAML for site->goal_pose dispatch."""
    try:
        planning_share = get_package_share_directory('camrod_planning')
        candidate = os.path.join(planning_share, 'config', 'camping_sites.yaml')
        if os.path.isfile(candidate):
            return candidate
    except PackageNotFoundError:
        pass

    # Source-workspace fallback.
    # Go up 3 levels: launch/ → camrod_ui_robot/ → camrod_ui/ → src/
    source_ws_candidate = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', '..', '..', 'camrod_planning', 'config', 'camping_sites.yaml')
    )
    if os.path.isfile(source_ws_candidate):
        return source_ws_candidate
    return ''


def generate_launch_description():
    default_frontend_dir = _resolve_default_frontend_dir()
    default_camping_sites_yaml = _resolve_default_camping_sites_yaml()

    enable_ui_backend_arg = DeclareLaunchArgument(
        'enable_ui_backend',
        default_value='true',
        description='Enable UI backend HTTP server and ROS topic bridge',
    )
    ui_host_arg = DeclareLaunchArgument(
        'ui_host',
        default_value='127.0.0.1',
        description='UI backend bind host',
    )
    ui_port_arg = DeclareLaunchArgument(
        'ui_port',
        default_value='8010',
        description='UI backend bind port',
    )
    frontend_dir_arg = DeclareLaunchArgument(
        'frontend_dir',
        default_value=default_frontend_dir,
        description='Static frontend directory for UI backend',
    )
    camping_sites_yaml_arg = DeclareLaunchArgument(
        'camping_sites_yaml',
        default_value=default_camping_sites_yaml,
        description='Camping site coordinates YAML used for destination->goal_pose dispatch',
    )
    planning_engage_topic_arg = DeclareLaunchArgument(
        'planning_engage_topic',
        default_value='/planning/engage',
        description='Manual planning engage topic used by UI engage/auto/stop buttons',
    )
    planning_mission_engage_topic_arg = DeclareLaunchArgument(
        'planning_mission_engage_topic',
        default_value='/planning/mission_engage',
        description='Mission engage topic used by UI destination/camping-site buttons',
    )
    platform_drive_enable_topic_arg = DeclareLaunchArgument(
        'platform_drive_enable_topic',
        default_value='/platform/drive_enable',
        description='Platform drive-enable topic armed together with UI engage commands',
    )
    site_maneuver_return_topic_arg = DeclareLaunchArgument(
        'site_maneuver_return_topic',
        default_value='/parking/site_maneuver/return',
        description='Rule-based campsite return trigger used by the UI return button',
    )
    site_maneuver_adopt_topic_arg = DeclareLaunchArgument(
        'site_maneuver_adopt_topic',
        default_value='/parking/site_maneuver/adopt',
        description='Rule-based campsite parked-state adoption trigger used when UI selects the current site',
    )
    arrival_pose_topic_arg = DeclareLaunchArgument(
        'arrival_pose_topic',
        default_value='/localization/pose',
        description='Pose topic used to detect already-arrived campsite selections',
    )
    manual_dock_enabled_arg = DeclareLaunchArgument(
        'manual_dock_enabled',
        default_value='true',
        description='Initial state of Manual Docking toggle in UI settings tab',
    )
    auto_dock_enabled_arg = DeclareLaunchArgument(
        'auto_dock_enabled',
        default_value='false',
        description='Initial state of Auto Docking toggle in UI settings tab',
    )

    ui_backend = Node(
        package='camrod_ui',
        executable='ui_backend_node',
        name='ui_backend',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_ui_backend')),
        parameters=[{
            'host': LaunchConfiguration('ui_host'),
            'port': LaunchConfiguration('ui_port'),
            'frontend_dir': LaunchConfiguration('frontend_dir'),
            # HH_260617: UI follows the system namespace for aggregated diagnostics.
            'diagnostics_agg_topic': '/system/diagnostics_agg',
            'site_names': [f'B{i}' for i in range(1, 14)],
            'ui_destination_topic': '/ui/selected_destination',
            'planning_engage_topic': LaunchConfiguration('planning_engage_topic'),
            'planning_mission_engage_topic': LaunchConfiguration('planning_mission_engage_topic'),
            'platform_drive_enable_topic': LaunchConfiguration('platform_drive_enable_topic'),
            'site_maneuver_return_topic': LaunchConfiguration('site_maneuver_return_topic'),
            'site_maneuver_adopt_topic': LaunchConfiguration('site_maneuver_adopt_topic'),
            'arrival_pose_topic': LaunchConfiguration('arrival_pose_topic'),
            # HH_260701 - If the robot is already inside the selected campsite,
            # show the arrival/return UI instead of sending a fresh Nav2 goal.
            'immediate_site_arrival_enabled': True,
            'site_arrival_center_radius_m': 2.5,
            'site_arrival_pose_timeout_s': 2.0,
            # HH_260617: Replace ambiguous goal-key naming with semantic mission-key dispatch.
            'planning_mission_key_topic': '/planning/mission_key',
            'planning_goal_pose_topic': '/goal_pose',
            'publish_mission_key': True,
            'publish_goal_pose': True,
            # HH_260630 - Destination/camping-site buttons use the mission latch,
            # while the manual UI engage button keeps controlling /planning/engage.
            'publish_engage_from_destination': False,
            'publish_mission_engage_from_destination': True,
            'publish_platform_drive_enable_with_engage': True,
            'default_goal_frame_id': 'map',
            # HH_260617: Fallback destination uses the same mission-key contract.
            'fallback_mission_key': 'camping_site_1',
            'fallback_to_first_known_goal': True,
            'camping_sites_yaml': LaunchConfiguration('camping_sites_yaml'),
            'manual_dock_enabled': LaunchConfiguration('manual_dock_enabled'),
            'auto_dock_enabled': LaunchConfiguration('auto_dock_enabled'),
        }],
    )

    return LaunchDescription([
        enable_ui_backend_arg,
        ui_host_arg,
        ui_port_arg,
        frontend_dir_arg,
        camping_sites_yaml_arg,
        planning_engage_topic_arg,
        planning_mission_engage_topic_arg,
        platform_drive_enable_topic_arg,
        site_maneuver_return_topic_arg,
        site_maneuver_adopt_topic_arg,
        arrival_pose_topic_arg,
        manual_dock_enabled_arg,
        auto_dock_enabled_arg,
        ui_backend,
    ])
