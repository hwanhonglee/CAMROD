import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        # HH_260807 - Match full bringup so the standalone package serves the
        # robot UI on the trusted robot LAN as well as on the local display.
        default_value='0.0.0.0',
        description='UI backend bind host',
    )
    ui_port_arg = DeclareLaunchArgument(
        'ui_port',
        default_value='8010',
        description='UI backend bind port',
    )
    # HH_260810 - Keep the RViz-replacement telemetry bridge available but
    # lease its high-bandwidth subscriptions only while the admin view is open.
    enable_operator_telemetry_arg = DeclareLaunchArgument(
        'enable_operator_telemetry',
        default_value='true',
        description='Enable the on-demand operator sensor and trajectory workspace',
    )
    operator_telemetry_stream_rate_hz_arg = DeclareLaunchArgument(
        'operator_telemetry_stream_rate_hz',
        default_value='10.0',
        description='Maximum selected-view WebSocket refresh rate in Hz',
    )
    enable_ui_guest_arg = DeclareLaunchArgument(
        'enable_ui_guest',
        default_value='true',
        description='Enable the WiFi guest UI alongside the Robot UI backend',
    )
    guest_host_arg = DeclareLaunchArgument(
        'guest_host',
        default_value='0.0.0.0',
        description='Guest UI bind host',
    )
    guest_port_arg = DeclareLaunchArgument(
        'guest_port',
        default_value='8012',
        description='Guest UI bind port',
    )
    # HH_260807 - Snap Chromium cannot enter its sandbox on this robot because
    # snap-confine lacks cap_dac_override.  Keep standalone and full bringup on
    # the field-verified WebKit path; Chromium remains an explicit override.
    enable_operator_ui_window_arg = DeclareLaunchArgument(
        'enable_operator_ui_window',
        default_value='true',
        description='Open the local operator UI in a managed kiosk window',
    )
    operator_ui_window_engine_arg = DeclareLaunchArgument(
        'operator_ui_window_engine',
        default_value='webkit',
        description='Operator UI renderer: webkit (default), chromium, or auto',
    )
    operator_ui_window_url_arg = DeclareLaunchArgument(
        'operator_ui_window_url',
        default_value='http://127.0.0.1:8010',
        description='URL loaded by the managed operator UI window',
    )
    operator_ui_window_width_arg = DeclareLaunchArgument(
        'operator_ui_window_width',
        default_value='1280',
        description='Initial operator UI window width in pixels',
    )
    operator_ui_window_height_arg = DeclareLaunchArgument(
        'operator_ui_window_height',
        default_value='800',
        description='Initial operator UI window height in pixels',
    )
    operator_ui_window_fullscreen_arg = DeclareLaunchArgument(
        'operator_ui_window_fullscreen',
        default_value='true',
        description='Open the managed operator UI window fullscreen',
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
    enable_campsite_occupancy_guard_arg = DeclareLaunchArgument(
        'enable_campsite_occupancy_guard',
        default_value='false',
        description='Block campsite dispatch using confirmed semantic tent occupancy',
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
    camping_site_maneuver_controller_operation_topic_arg = DeclareLaunchArgument(
        'camping_site_maneuver_controller_operation_topic',
        # HH_260720 - Campsite maneuver commands are owned by camrod_control.
        default_value='/control/camping_site_maneuver_controller/operation',
        description='Typed campsite operation topic used by the UI return button',
    )
    planning_return_to_drop_zone_topic_arg = DeclareLaunchArgument(
        'planning_return_to_drop_zone_topic',
        # HH_260818 - Expose the deferred post-CRAB_OUT planning handoff.
        default_value='/planning/state_machine/return_to_drop_zone',
        description='Typed drop-zone route request published after site exit',
    )
    manual_return_preempt_hold_s_arg = DeclareLaunchArgument(
        'manual_return_preempt_hold_s',
        # HH_260819 - This exceeds the 0.35 s command timeout without adding a
        # permanent timer or polling loop on the constrained ARM64 target.
        default_value='0.5',
        description='Stopped hold between cancelling an active Nav2 goal and dispatching Return',
    )
    platform_status_topic_arg = DeclareLaunchArgument(
        'platform_status_topic',
        default_value='/platform/status',
        description='Generated platform CAN and BMS status topic',
    )
    camping_site_maneuver_controller_adopt_topic_arg = DeclareLaunchArgument(
        'camping_site_maneuver_controller_adopt_topic',
        default_value='/control/camping_site_maneuver_controller/adopt',
        description='Campsite parked-state adoption trigger used when UI selects the current site',
    )
    drop_zone_maneuver_controller_operation_topic_arg = DeclareLaunchArgument(
        'drop_zone_maneuver_controller_operation_topic',
        # HH_260721 - UI requests bounded station departure before releasing a campsite goal.
        default_value='/control/drop_zone_maneuver_controller/operation',
        description='Typed drop-zone exit operation topic',
    )
    parking_operation_topic_arg = DeclareLaunchArgument(
        'parking_operation_topic',
        # HH_260721 - Cancel final parking before the UI starts charger departure.
        default_value='/parking/operation',
        description='Typed final-parking operation topic used during ownership handoff',
    )
    drop_zone_exit_complete_topic_arg = DeclareLaunchArgument(
        'drop_zone_exit_complete_topic',
        default_value='/control/drop_zone/exit_complete',
        description='Drop-zone straight-exit and yaw-alignment completion topic',
    )
    arrival_pose_topic_arg = DeclareLaunchArgument(
        'arrival_pose_topic',
        default_value='/localization/pose',
        description='Pose topic used to detect already-arrived campsite selections',
    )
    # HH_260724 - Expose the UI backend battery policy so bringup can mirror it.
    require_battery_for_mission_dispatch_arg = DeclareLaunchArgument(
        'require_battery_for_mission_dispatch',
        default_value='true',
        description='Require battery feedback before accepting a new campsite dispatch',
    )
    minimum_mission_dispatch_battery_percent_arg = DeclareLaunchArgument(
        'minimum_mission_dispatch_battery_percent',
        default_value='35.0',
        description='Minimum SOC percent for new campsite dispatch',
    )
    low_battery_return_after_current_mission_arg = DeclareLaunchArgument(
        'low_battery_return_after_current_mission',
        default_value='true',
        description='Latch low battery during a campsite mission and wait for user return',
    )
    low_battery_return_threshold_percent_arg = DeclareLaunchArgument(
        'low_battery_return_threshold_percent',
        default_value='35.0',
        description='SOC percent that starts the finish-current-mission return latch',
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
            'enable_operator_telemetry': ParameterValue(
                LaunchConfiguration('enable_operator_telemetry'),
                value_type=bool,
            ),
            # HH_260810 - Keep one bounded latest-value stream instead of
            # high-frequency HTTP polling on the 8-core ARM64 target.
            'telemetry_stream_rate_hz': ParameterValue(
                LaunchConfiguration('operator_telemetry_stream_rate_hz'),
                value_type=float,
            ),
            # HH_260617: UI follows the system namespace for aggregated diagnostics.
            'diagnostics_agg_topic': '/system/diagnostics_agg',
            # HH_260721 - Consume the platform-neutral operational service lifecycle.
            'service_state_topic': '/service/state',
            'site_names': [f'B{i}' for i in range(1, 14)],
            'ui_destination_topic': '/ui/selected_destination',
            'ui_camping_site_operation_request_topic': '/ui/camping_site_operation_request',
            'planning_engage_topic': LaunchConfiguration('planning_engage_topic'),
            'planning_mission_engage_topic': LaunchConfiguration('planning_mission_engage_topic'),
            'platform_drive_enable_topic': LaunchConfiguration('platform_drive_enable_topic'),
            'camping_site_maneuver_controller_operation_topic': LaunchConfiguration('camping_site_maneuver_controller_operation_topic'),
            'planning_return_to_drop_zone_topic': LaunchConfiguration('planning_return_to_drop_zone_topic'),
            'manual_return_preempt_hold_s': ParameterValue(
                LaunchConfiguration('manual_return_preempt_hold_s'),
                value_type=float,
            ),
            'camping_site_maneuver_controller_adopt_topic': LaunchConfiguration('camping_site_maneuver_controller_adopt_topic'),
            # HH_260721 - Defer site goals until the control-owned station exit completes.
            'drop_zone_maneuver_controller_operation_topic': LaunchConfiguration('drop_zone_maneuver_controller_operation_topic'),
            'parking_operation_topic': LaunchConfiguration('parking_operation_topic'),
            'drop_zone_exit_complete_topic': LaunchConfiguration('drop_zone_exit_complete_topic'),
            'arrival_pose_topic': LaunchConfiguration('arrival_pose_topic'),
            'planning_lanelet_pose_topic': '/planning/lanelet_pose',
            'platform_status_topic': LaunchConfiguration('platform_status_topic'),
            # HH_260727 - Standard ROS parameter service target for live steering tuning.
            'ranger_base_node_name': '/ranger_base_node',
            'steering_transition_parameter': 'steering_transition_rate_radps',
            # HH_260701 - If the robot is already inside the selected campsite,
            # show the arrival/return UI instead of sending a fresh Nav2 goal.
            'immediate_site_arrival_enabled': True,
            'site_arrival_center_radius_m': 2.5,
            'site_arrival_pose_timeout_s': 2.0,
            'site_arrival_roadside_offset_m': 0.30,
            'site_arrival_roadside_lateral_tolerance_m': 0.15,
            'site_arrival_roadside_forward_tolerance_m': 0.60,
            # HH_260617: Replace ambiguous goal-key naming with semantic mission-key dispatch.
            'planning_mission_key_topic': '/planning/mission_key',
            # HH_260810 - Site missions retain the regulated input while the
            # operator-map tool replaces RViz on the independent manual input.
            'planning_goal_pose_topic': '/planning/site_goal_pose_ros',
            'manual_goal_pose_topic': '/goal_pose',
            'publish_mission_key': True,
            'publish_goal_pose': True,
            # HH_260630 - Destination/camping-site buttons use the mission latch,
            # while the manual UI engage button keeps controlling /planning/engage.
            'publish_engage_from_destination': False,
            'publish_mission_engage_from_destination': True,
            # HH_260724 - Block new campsite dispatch unless the charger has restored SOC margin.
            'require_battery_for_mission_dispatch': ParameterValue(
                LaunchConfiguration('require_battery_for_mission_dispatch'),
                value_type=bool,
            ),
            'minimum_mission_dispatch_battery_percent': ParameterValue(
                LaunchConfiguration('minimum_mission_dispatch_battery_percent'),
                value_type=float,
            ),
            'low_battery_return_after_current_mission': ParameterValue(
                LaunchConfiguration('low_battery_return_after_current_mission'),
                value_type=bool,
            ),
            'low_battery_return_threshold_percent': ParameterValue(
                LaunchConfiguration('low_battery_return_threshold_percent'),
                value_type=float,
            ),
            'publish_platform_drive_enable_with_engage': True,
            'default_goal_frame_id': 'map',
            # HH_260617: Fallback destination uses the same mission-key contract.
            'fallback_mission_key': 'camping_site_1',
            'fallback_to_first_known_goal': True,
            'camping_sites_yaml': LaunchConfiguration('camping_sites_yaml'),
            # HH_260818 - One bringup flag controls both UI admission and the
            # control-side occupied-site start gate.
            'enable_campsite_occupancy_guard': ParameterValue(
                LaunchConfiguration('enable_campsite_occupancy_guard'),
                value_type=bool,
            ),
        }],
    )

    # HH_260803 - Start Guest UI from the same launch so destination and return
    # requests always have the Robot backend available as their scenario owner.
    guest_ui = Node(
        package='camrod_ui',
        executable='ui_guest_node',
        name='ui_guest',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_ui_guest')),
        parameters=[{
            'host': LaunchConfiguration('guest_host'),
            'port': LaunchConfiguration('guest_port'),
            'service_state_topic': '/service/state',
            'battery_topic': LaunchConfiguration('platform_status_topic'),
            # HH_260803 - Guest and Robot surfaces show the same control hold.
            'control_gate_status_topic': '/control/cmd_vel_safety_gate/status',
            'ui_destination_topic': '/ui/selected_destination',
            'ui_camping_site_operation_request_topic': '/ui/camping_site_operation_request',
            'minimum_mission_dispatch_battery_percent': ParameterValue(
                LaunchConfiguration('minimum_mission_dispatch_battery_percent'),
                value_type=float,
            ),
        }],
    )

    # HH_260727 - This is a non-ROS browser process, so launch it directly
    # instead of using launch_ros Node (which would append unsupported --ros-args).
    operator_ui_window = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    os.path.join(
                        get_package_prefix('camrod_ui'),
                        'lib',
                        'camrod_ui',
                        'camrod_ui_window',
                    ),
                    '--url',
                    LaunchConfiguration('operator_ui_window_url'),
                    '--width',
                    LaunchConfiguration('operator_ui_window_width'),
                    '--height',
                    LaunchConfiguration('operator_ui_window_height'),
                    '--engine',
                    LaunchConfiguration('operator_ui_window_engine'),
                    PythonExpression([
                        "'--fullscreen' if '",
                        LaunchConfiguration('operator_ui_window_fullscreen'),
                        "' == 'true' else '--no-fullscreen'",
                    ]),
                ],
                name='camrod_ui_window',
                output='screen',
            )
        ],
        condition=IfCondition(LaunchConfiguration('enable_operator_ui_window')),
    )

    return LaunchDescription([
        enable_ui_backend_arg,
        ui_host_arg,
        ui_port_arg,
        enable_operator_telemetry_arg,
        operator_telemetry_stream_rate_hz_arg,
        enable_ui_guest_arg,
        guest_host_arg,
        guest_port_arg,
        enable_operator_ui_window_arg,
        operator_ui_window_engine_arg,
        operator_ui_window_url_arg,
        operator_ui_window_width_arg,
        operator_ui_window_height_arg,
        operator_ui_window_fullscreen_arg,
        frontend_dir_arg,
        camping_sites_yaml_arg,
        enable_campsite_occupancy_guard_arg,
        planning_engage_topic_arg,
        planning_mission_engage_topic_arg,
        platform_drive_enable_topic_arg,
        camping_site_maneuver_controller_operation_topic_arg,
        planning_return_to_drop_zone_topic_arg,
        manual_return_preempt_hold_s_arg,
        platform_status_topic_arg,
        camping_site_maneuver_controller_adopt_topic_arg,
        drop_zone_maneuver_controller_operation_topic_arg,
        parking_operation_topic_arg,
        drop_zone_exit_complete_topic_arg,
        arrival_pose_topic_arg,
        require_battery_for_mission_dispatch_arg,
        minimum_mission_dispatch_battery_percent_arg,
        low_battery_return_after_current_mission_arg,
        low_battery_return_threshold_percent_arg,
        ui_backend,
        guest_ui,
        operator_ui_window,
    ])
