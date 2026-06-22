import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


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


def _extract_ros_params(data: object) -> dict:
    if not isinstance(data, dict):
        return {}
    wildcard = data.get('/**')
    if isinstance(wildcard, dict) and isinstance(wildcard.get('ros__parameters'), dict):
        return wildcard.get('ros__parameters', {})
    params = data.get('ros__parameters')
    if isinstance(params, dict):
        return params
    return data


def _normalize_profile_name(profile: str) -> str:
    value = str(profile or '').strip()
    if not value:
        return ''
    if value.startswith('copy_'):
        return value
    if value.startswith('copy'):
        suffix = value[4:].lstrip('_-')
        return f'copy_{suffix}' if suffix else 'copy'
    return value


def _resolve_profile_file(base_path: str, profile: str) -> str:
    # HHL_260622 - Standalone UI follows the active map profile just like bringup/planning.
    normalized = _normalize_profile_name(profile)
    if normalized:
        root, ext = os.path.splitext(base_path)
        candidate = f'{root} ({normalized}){ext}'
        if os.path.isfile(candidate):
            return candidate
    return base_path


def _resolve_map_profile_default() -> str:
    try:
        map_share = get_package_share_directory('camrod_map')
        map_info_path = os.path.join(map_share, 'config', 'map_info.yaml')
        with open(map_info_path, 'r', encoding='utf-8') as f:
            params = _extract_ros_params(yaml.safe_load(f) or {})
        return str(params.get('map_profile', params.get('profile', '')))
    except Exception:
        return ''


def _resolve_default_camping_sites_yaml() -> str:
    """Resolve planning camping-sites YAML for site->goal_pose dispatch."""
    map_profile = _resolve_map_profile_default()
    try:
        planning_share = get_package_share_directory('camrod_planning')
        candidate = os.path.join(planning_share, 'config', 'camping_sites.yaml')
        if os.path.isfile(candidate):
            return _resolve_profile_file(candidate, map_profile)
    except PackageNotFoundError:
        pass

    # Source-workspace fallback.
    # Go up 3 levels: launch/ → camrod_ui_robot/ → camrod_ui/ → src/
    source_ws_candidate = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', '..', '..', 'camrod_planning', 'config', 'camping_sites.yaml')
    )
    if os.path.isfile(source_ws_candidate):
        return _resolve_profile_file(source_ws_candidate, map_profile)
    return ''


def _resolve_default_site_access_yaml() -> str:
    """Resolve UI reservation/occupancy gate YAML."""
    try:
        ui_share = get_package_share_directory('camrod_ui')
        candidate = os.path.join(ui_share, 'config', 'site_access.yaml')
        if os.path.isfile(candidate):
            return candidate
    except PackageNotFoundError:
        pass

    # HHL_260621 - Source fallback keeps reservation/occupancy gate active before install.
    source_candidate = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'site_access.yaml')
    )
    if os.path.isfile(source_candidate):
        return source_candidate
    return ''


def generate_launch_description():
    default_frontend_dir = _resolve_default_frontend_dir()
    default_camping_sites_yaml = _resolve_default_camping_sites_yaml()
    default_site_access_yaml = _resolve_default_site_access_yaml()

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
    site_access_yaml_arg = DeclareLaunchArgument(
        'site_access_yaml',
        default_value=default_site_access_yaml,
        description='Reservation/occupancy YAML used to gate unsafe campsite dispatch',
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
            'planning_engage_topic': '/planning/engage',
            # HH_260617: Replace ambiguous goal-key naming with semantic mission-key dispatch.
            'planning_mission_key_topic': '/planning/mission_key',
            'planning_goal_pose_topic': '/goal_pose',
            'planning_return_to_drop_zone_topic': '/planning/state_machine/return_to_drop_zone',
            # HHL_260622 - Customer "usage complete" starts campsite crab-out
            # before planning is allowed to route back to the drop zone.
            'parking_site_return_topic': '/parking/site_maneuver/return',
            'publish_mission_key': True,
            'publish_goal_pose': True,
            'publish_engage_from_destination': True,
            'default_goal_frame_id': 'map',
            # HH_260617: Fallback destination uses the same mission-key contract.
            'fallback_mission_key': 'camping_site_1',
            'fallback_to_first_known_goal': True,
            'camping_sites_yaml': LaunchConfiguration('camping_sites_yaml'),
            'site_access_yaml': LaunchConfiguration('site_access_yaml'),
            'enable_site_access_gate': True,
            'require_reservation_code_for_delivery': False,
            # HHL_260621 - Do not let unmapped B-sites fall back to camping_site_1.
            'require_known_mission_key_for_delivery': True,
            # HHL_260622 - A new delivery may start only from drop-zone idle unless site_access.yaml overrides it.
            'enforce_delivery_start_state': True,
            'delivery_allowed_amr_states': [0],
        }],
    )

    return LaunchDescription([
        enable_ui_backend_arg,
        ui_host_arg,
        ui_port_arg,
        frontend_dir_arg,
        camping_sites_yaml_arg,
        site_access_yaml_arg,
        ui_backend,
    ])
