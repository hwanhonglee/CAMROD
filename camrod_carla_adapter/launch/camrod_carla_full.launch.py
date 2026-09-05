"""Run the complete CAMROD algorithms and production UI on a CARLA plant.

CARLA server, carla_ros_bridge, and actor spawning remain explicit lifecycle
owners. This launch owns only CAMROD, the physical Ranger controller, and the
conversion boundaries, so stopping it cannot destroy a shared CARLA world.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def _environment_path(name, ranger_relative_path=None, default=""):
    """Resolve a portable launch default without embedding a host home path."""
    configured = os.environ.get(name, "").strip()
    if configured:
        return os.path.abspath(os.path.expanduser(configured))

    ranger_root = os.environ.get("RANGER_CARLA_ROOT", "").strip()
    if ranger_root and ranger_relative_path:
        return os.path.abspath(
            os.path.join(
                os.path.expanduser(ranger_root), ranger_relative_path
            )
        )
    return default


def _include(path, arguments, condition=None):
    kwargs = {"launch_arguments": arguments.items()}
    if condition is not None:
        kwargs["condition"] = condition
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), **kwargs
    )


def generate_launch_description():
    adapter_share = get_package_share_directory("camrod_carla_adapter")
    controller_share = get_package_share_directory(
        "carla_extended_ackermann_control"
    )
    bringup_share = get_package_share_directory("camrod_bringup")
    control_share = get_package_share_directory("camrod_control")
    localization_share = get_package_share_directory("camrod_localization")
    perception_share = get_package_share_directory("camrod_perception")
    planning_share = get_package_share_directory("camrod_planning")
    sensing_share = get_package_share_directory("camrod_sensing")
    system_share = get_package_share_directory("camrod_system")
    nav2_bt_share = get_package_share_directory("nav2_bt_navigator")

    adapter_launch = os.path.join(
        adapter_share, "launch", "adapter.launch.py"
    )
    sensor_relay_launch = os.path.join(
        adapter_share, "launch", "sensor_relay.launch.py"
    )
    lidar_processing_launch = os.path.join(
        adapter_share, "launch", "carla_lidar_processing.launch.py"
    )
    radar_relay_launch = os.path.join(
        adapter_share, "launch", "radar_relay.launch.py"
    )
    platform_heartbeat_launch = os.path.join(
        adapter_share, "launch", "platform_heartbeat.launch.py"
    )
    charging_contact_launch = os.path.join(
        adapter_share, "launch", "charging_contact_emulator.launch.py"
    )
    # carla_extended_ackermann_control follows the upstream package layout
    # and installs launch files directly under share/<package> (without a
    # launch/ subdirectory).
    controller_launch = os.path.join(
        controller_share, "full_stack.launch.py"
    )
    bringup_launch = os.path.join(
        bringup_share, "launch", "bringup.launch.py"
    )
    launch_defaults = os.path.join(
        bringup_share, "config", "bringup", "launch_defaults.yaml"
    )
    alignment_config = os.path.join(
        adapter_share, "config", "woraksan_lane_anchor_alignment.yaml"
    )
    input_adapter_config = os.path.join(
        localization_share, "config", "source", "input_adapter.yaml"
    )
    lidar_cost_grid_config = os.path.join(
        sensing_share, "config", "lidar", "cost_grid.yaml"
    )
    nav2_runtime_disabled_config = os.path.join(
        planning_share, "config", "nav2_combo_profiles", "disabled.yaml"
    )
    parking_runtime_disabled_config = os.path.join(
        control_share, "config", "parking_runtime_profiles", "disabled.yaml"
    )
    perception_carla_config = os.path.join(
        adapter_share, "config", "perception_carla.yaml"
    )
    apriltag_detector_config = os.path.join(
        perception_share, "config", "apriltag_parking_detector.yaml"
    )
    system_checker_config = os.path.join(
        system_share, "config", "system_checker.yaml"
    )
    camrod_src_root = os.environ.get("CAMROD_SRC_ROOT", "").strip()
    if camrod_src_root:
        develop_lanelet_map = os.path.join(
            os.path.abspath(os.path.expanduser(camrod_src_root)),
            "lanelet2_maps.osm",
        )
    else:
        develop_lanelet_map = os.path.join(
            os.path.expanduser("~"), "camrod_ws", "src", "lanelet2_maps.osm"
        )
    nav_through_poses_bt = os.path.join(
        nav2_bt_share,
        "behavior_trees",
        "navigate_through_poses_w_replanning_and_recovery.xml",
    )
    nav_to_pose_bt = os.path.join(
        planning_share,
        "config",
        "bt",
        "navigate_to_pose_w_planner_selector.xml",
    )

    baseline_manifest = _environment_path(
        "RANGER_BASELINE_MANIFEST",
        os.path.join(".work", "evidence", "ranger_ros_backend_gate.json"),
    )
    physical_manifest = _environment_path(
        "RANGER_PHYSICAL_MANIFEST",
        os.path.join(
            ".work", "evidence", "ranger_physical_4ws_acceptance_gate.json"
        ),
    )
    accepted_carla_python_egg = _environment_path(
        "CARLA_PYTHON_EGG"
    ) or _environment_path("RANGER_CARLA_PYTHON_EGG")
    python_egg_cache = _environment_path(
        "CARLA_PYTHON_EGG_CACHE"
    ) or _environment_path("RANGER_PYTHON_EGG_CACHE")
    camrod_map_path = _environment_path(
        "CAMROD_LANELET_MAP", default=develop_lanelet_map
    )
    alignment_config = _environment_path(
        "CAMROD_MAP_ALIGNMENT_FILE", default=alignment_config
    )
    launch_defaults = _environment_path(
        "CAMROD_LAUNCH_DEFAULTS_FILE", default=launch_defaults
    )
    lidar_cost_grid_config = _environment_path(
        "CAMROD_CARLA_LIDAR_COST_GRID_FILE",
        default=lidar_cost_grid_config,
    )
    perception_carla_config = _environment_path(
        "CAMROD_CARLA_PERCEPTION_FILE",
        default=perception_carla_config,
    )

    declarations = [
        DeclareLaunchArgument(
            "role_name",
            default_value=os.environ.get("CARLA_ROLE_NAME", "ego_vehicle"),
        ),
        DeclareLaunchArgument(
            "host", default_value=os.environ.get("CARLA_HOST", "127.0.0.1")
        ),
        DeclareLaunchArgument(
            "port", default_value=os.environ.get("CARLA_PORT", "2000")
        ),
        DeclareLaunchArgument("launch_vehicle_control", default_value="true"),
        DeclareLaunchArgument(
            "recovery_breakaway_enable",
            default_value="false",
            description=(
                "Opt-in CARLA plant torque authorization used by historical "
                "Woraksan recovery runs; parity mode adds no such authority"
            ),
        ),
        DeclareLaunchArgument(
            "rotation_recovery_breakaway_enable",
            default_value="false",
            description=(
                "Opt-in CARLA campsite CRAB_OUT rotational-stall lease; "
                "ordinary and develop-parity rotation retain 2 N*m"
            ),
        ),
        DeclareLaunchArgument(
            "rotation_recovery_breakaway_status_timeout_sec",
            default_value="0.30",
            description=(
                "Freshness lease for the CARLA campsite rotation recovery "
                "status; develop parity retains 0.30 s"
            ),
        ),
        DeclareLaunchArgument("launch_sensor_relay", default_value="true"),
        DeclareLaunchArgument(
            "launch_lidar_processing", default_value="true"
        ),
        DeclareLaunchArgument(
            "compressed_image_max_rate_hz",
            default_value=os.environ.get(
                "CAMROD_CARLA_COMPRESSED_IMAGE_MAX_RATE_HZ", "5.0"
            ),
            description=(
                "Maximum wall-clock JPEG rate per CARLA camera; encoding is "
                "also disabled when the compressed topic has no subscribers"
            ),
        ),
        DeclareLaunchArgument(
            "operator_telemetry_tf_transform_enabled",
            default_value="true",
            description=(
                "Transform CARLA sensor overlays into the UI fixed frame. "
                "This is a display-side CARLA boundary adaptation; the "
                "ordinary CAMROD UI default remains disabled"
            ),
        ),
        DeclareLaunchArgument(
            "operator_telemetry_raw_lidar_bbox_overlay_enabled",
            default_value="false",
            description=(
                "Hide classless Euclidean LiDAR boxes in CARLA so the operator "
                "view shows semantic fusion obstacles and fusion-derived cost "
                "only; ordinary CAMROD retains its develop overlay"
            ),
        ),
        DeclareLaunchArgument(
            "operator_telemetry_tf_latest_fallback_tolerance_s",
            default_value=os.environ.get(
                "CAMROD_CARLA_UI_TF_LATEST_FALLBACK_TOLERANCE_S", "0.0"
            ),
            description=(
                "CARLA-only bound for the reception-stamp race between sensor "
                "callbacks and the 20 Hz odometry TF; production remains zero"
            ),
        ),
        DeclareLaunchArgument(
            "raw_image_max_rate_hz",
            default_value=os.environ.get(
                "CAMROD_CARLA_RAW_IMAGE_MAX_RATE_HZ", "10.0"
            ),
            description=(
                "Maximum wall-clock raw camera relay rate when a raw "
                "consumer exists; no-subscriber payload publication is skipped"
            ),
        ),
        DeclareLaunchArgument("launch_radar_relay", default_value="true"),
        DeclareLaunchArgument(
            "launch_platform_heartbeat", default_value="true"
        ),
        DeclareLaunchArgument(
            "launch_charging_contact_emulator", default_value="false"
        ),
        DeclareLaunchArgument(
            "carla_charging_contact_parking_status_topic",
            default_value="/parking/reverse_parking_controller/status",
            description=(
                "Parking-state source used only when the opt-in CARLA "
                "charging-contact emulator is launched"
            ),
        ),
        DeclareLaunchArgument(
            "platform_heartbeat_publish_rate_hz", default_value="5.0"
        ),
        DeclareLaunchArgument(
            "manual_cmd_vel_ros_topic",
            default_value="/control/manual_cmd_vel_ros",
        ),
        # CARLA may use the complete audited Ranger adapter envelope.  These
        # remain launch arguments so the operator can select a lower ceiling;
        # ordinary CAMROD keeps the UI package's conservative 0.20 defaults.
        DeclareLaunchArgument(
            "manual_drive_linear_limit_mps",
            default_value=os.environ.get(
                "CAMROD_MANUAL_LINEAR_LIMIT_MPS", "0.20"
            ),
        ),
        DeclareLaunchArgument(
            "manual_drive_lateral_limit_mps",
            default_value=os.environ.get(
                "CAMROD_MANUAL_LATERAL_LIMIT_MPS", "0.20"
            ),
        ),
        DeclareLaunchArgument(
            "manual_drive_angular_limit_radps",
            default_value=os.environ.get(
                "CAMROD_MANUAL_ANGULAR_LIMIT_RADPS", "0.20"
            ),
        ),
        # The UI publishes at 10 Hz and expires its lease at 0.25 s.  The
        # downstream 0.35 s command watchdog remains the final physical stop
        # boundary if the browser or WebSocket disappears.
        DeclareLaunchArgument(
            "manual_drive_deadman_timeout_s",
            default_value=os.environ.get(
                "CAMROD_MANUAL_DEADMAN_TIMEOUT_S", "0.25"
            ),
            description=(
                "CARLA UI heartbeat lease; downstream 0.35 s command "
                "watchdogs remain the physical stop boundary"
            ),
        ),
        DeclareLaunchArgument(
            "carla_route_heading_error_enter_deg",
            default_value=os.environ.get(
                "CAMROD_CARLA_ROUTE_HEADING_ERROR_ENTER_DEG", "75.0"
            ),
            description=(
                "Route-heading zero-turn threshold; defaults to the CAMROD "
                "production profile and remains overridable for experiments"
            ),
        ),
        DeclareLaunchArgument(
            "carla_route_heading_lookahead_m",
            default_value=os.environ.get(
                "CAMROD_CARLA_ROUTE_HEADING_LOOKAHEAD_M", "2.0"
            ),
            description=(
                "Route-heading preview distance; defaults to the CAMROD "
                "production profile and remains overridable for experiments"
            ),
        ),
        DeclareLaunchArgument(
            "carla_cmd_vel_gate_speed_scale",
            default_value=os.environ.get(
                "CAMROD_CARLA_CMD_VEL_GATE_SPEED_SCALE", "0.5"
            ),
            description=(
                "CARLA-only final cmd_vel scale. Unity preserves the bounded "
                "0.20 m/s reverse-return RPPReverse profile without slowing "
                "ordinary forward RPP; ordinary CAMROD retains its "
                "production 0.5 scale"
            ),
        ),
        DeclareLaunchArgument(
            "carla_navigation_minimum_ackermann_turn_radius_m",
            default_value=os.environ.get(
                "CAMROD_CARLA_NAVIGATION_MINIMUM_ACKERMANN_TURN_RADIUS_M",
                "0.0",
            ),
            description=(
                "CARLA-only final Nav2 radius used to stabilize the Ranger "
                "ACKERMANN/ZERO_TURN boundary; production remains disabled"
            ),
        ),
        DeclareLaunchArgument(
            "carla_cost_stop_threshold", default_value="85"
        ),
        DeclareLaunchArgument(
            "carla_cost_stop_latch_use_trigger_source_for_merged_clear",
            default_value="false",
            description=(
                "CARLA-only opt-in: release a merged-grid dynamic latch from "
                "fresh clear evidence on its saved trigger source"
            ),
        ),
        DeclareLaunchArgument(
            "carla_cost_stop_merged_dynamic_source_labels",
            default_value="",
            description=(
                "Optional CARLA-only allowlist for dynamic sources that may "
                "attribute a merged-grid stop"
            ),
        ),
        DeclareLaunchArgument(
            "carla_lanelet_safety_threshold", default_value="85"
        ),
        DeclareLaunchArgument(
            "carla_lanelet_safety_current_threshold", default_value="85"
        ),
        DeclareLaunchArgument(
            "carla_lanelet_safety_footprint_enable",
            default_value="true",
            description=(
                "CARLA-only planning-envelope lanelet check switch; true "
                "preserves the develop/full-launch safety default"
            ),
        ),
        DeclareLaunchArgument(
            "carla_lanelet_safety_check_reverse", default_value="false"
        ),
        DeclareLaunchArgument(
            "carla_nav2_reverse_controller", default_value=""
        ),
        DeclareLaunchArgument(
            "carla_reverse_goal_topic", default_value=""
        ),
        DeclareLaunchArgument(
            "carla_goal_snapper_pose_jump_check_topic", default_value=""
        ),
        DeclareLaunchArgument(
            "carla_route_safety_path_relative_recovery_enable",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "carla_route_safety_path_center_reentry_m",
            default_value="0.08",
            description=(
                "CARLA-only path-relative recovery CTE re-entry hysteresis; "
                "the full/develop-parity default remains 0.08 m"
            ),
        ),
        DeclareLaunchArgument(
            "carla_route_safety_zero_hold_pauses_limits",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "carla_route_safety_allow_corrective_yaw_beyond_limit",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "carla_goal_reissue_while_nav_active", default_value="false"
        ),
        DeclareLaunchArgument(
            "operator_telemetry_camera_raw_fallback_enabled",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "operator_telemetry_docking_rear_camera_fallback_enabled",
            default_value="false",
            description=(
                "Use the CARLA rear camera directly only when an explicit "
                "profile opts out of the production docking-camera path"
            ),
        ),
        DeclareLaunchArgument(
            "return_site_exit_rearm_enabled",
            default_value="false",
            description=(
                "Optional historical CARLA campsite Return re-arm behavior; "
                "develop-parity mode keeps the production state transition"
            ),
        ),
        DeclareLaunchArgument(
            "use_sim_planning_profile",
            default_value="false",
            description=(
                "Select deterministic simulation mission-state parameters; "
                "parity mode keeps the production planning profile"
            ),
        ),
        DeclareLaunchArgument(
            "use_sim_localization_profile",
            default_value="false",
            description=(
                "Select simulation localization parameters; parity mode "
                "keeps the production localization profile"
            ),
        ),
        DeclareLaunchArgument(
            "use_sim_parking_method",
            default_value="false",
            description=(
                "Select the simulation parking-method parameters; parity "
                "mode keeps the production AprilTag method"
            ),
        ),
        DeclareLaunchArgument(
            "carla_roadside_reverse_return_enable",
            default_value=os.environ.get(
                "CAMROD_CARLA_ROADSIDE_REVERSE_RETURN_ENABLE", "false"
            ),
            description=(
                "Exit a CARLA roadside campsite, preserve its outbound yaw, "
                "and retrace the validated reverse-shortest route instead of "
                "using the invalid Woraksan forward loop"
            ),
        ),
        DeclareLaunchArgument(
            "carla_crab_approach_slowdown_distance_m",
            default_value=os.environ.get(
                "CAMROD_CARLA_CRAB_APPROACH_SLOWDOWN_DISTANCE_M", "0.0"
            ),
            description=(
                "CARLA-only distance over which crab entry tapers toward its "
                "minimum speed; ordinary CAMROD keeps this disabled"
            ),
        ),
        DeclareLaunchArgument(
            "carla_crab_approach_min_speed_mps",
            default_value=os.environ.get(
                "CAMROD_CARLA_CRAB_APPROACH_MIN_SPEED_MPS", "0.0"
            ),
            description=(
                "CARLA-only minimum crab-entry speed during the final "
                "approach; ordinary CAMROD keeps this disabled"
            ),
        ),
        DeclareLaunchArgument(
            "carla_rotate_180_timeout_s",
            default_value=os.environ.get(
                "CAMROD_CARLA_ROTATE_180_TIMEOUT_S", "0.0"
            ),
            description=(
                "CARLA-only wall-clock bound for campsite ROTATE_180; "
                "ordinary CAMROD keeps an unbounded zero default"
            ),
        ),
        DeclareLaunchArgument(
            "carla_camping_site_max_angular_speed_radps",
            default_value=os.environ.get(
                "CAMROD_CARLA_CAMPING_SITE_MAX_ANGULAR_SPEED_RADPS", ""
            ),
            description=(
                "Optional CARLA-only campsite yaw-rate limit; empty preserves "
                "the develop control.yaml value"
            ),
        ),
        DeclareLaunchArgument(
            "carla_entry_position_tolerance_m",
            default_value=os.environ.get(
                "CAMROD_CARLA_ENTRY_POSITION_TOLERANCE_M", "0.15"
            ),
            description=(
                "CARLA-only crab-entry completion tolerance selected for the "
                "narrow v3 campsite geometry; ordinary CAMROD keeps 0.15 m"
            ),
        ),
        DeclareLaunchArgument(
            "carla_return_lateral_hysteresis_m",
            default_value=os.environ.get(
                "CAMROD_CARLA_RETURN_LATERAL_HYSTERESIS_M", ""
            ),
            description=(
                "Optional CARLA-only crab-return latched-band hysteresis; "
                "empty preserves the develop control.yaml value"
            ),
        ),
        DeclareLaunchArgument(
            "carla_rotate_entry_max_position_error_m",
            default_value=os.environ.get(
                "CAMROD_CARLA_ROTATE_ENTRY_MAX_POSITION_ERROR_M", "0.0"
            ),
            description=(
                "CARLA-only certified center tolerance for starting "
                "ROTATE_180 after bounded axis-by-axis centering; ordinary "
                "CAMROD keeps this disabled"
            ),
        ),
        DeclareLaunchArgument(
            "carla_rotate_entry_centering_max_initial_error_m",
            default_value=os.environ.get(
                "CAMROD_CARLA_ROTATE_ENTRY_CENTERING_MAX_INITIAL_ERROR_M",
                "0.30",
            ),
            description=(
                "CARLA-only maximum initial error recoverable by bounded "
                "axis-by-axis centering before ROTATE_180; ordinary CAMROD "
                "keeps its 0.30 m controller default"
            ),
        ),
        DeclareLaunchArgument(
            "carla_entry_anchor_centering_max_initial_error_m",
            default_value=os.environ.get(
                "CAMROD_CARLA_ENTRY_ANCHOR_CENTERING_MAX_INITIAL_ERROR_M",
                "0.0",
            ),
            description=(
                "CARLA-only maximum initial route-anchor error recoverable "
                "before crab entry; ordinary CAMROD keeps this disabled"
            ),
        ),
        DeclareLaunchArgument(
            "carla_entry_anchor_centering_max_speed_mps",
            default_value=os.environ.get(
                "CAMROD_CARLA_ENTRY_ANCHOR_CENTERING_MAX_SPEED_MPS", "0.12"
            ),
            description=(
                "CARLA-only maximum speed for axis-by-axis route-anchor "
                "centering before crab entry"
            ),
        ),
        DeclareLaunchArgument(
            "carla_entry_anchor_centering_timeout_s",
            default_value=os.environ.get(
                "CAMROD_CARLA_ENTRY_ANCHOR_CENTERING_TIMEOUT_S", "15"
            ),
            description=(
                "CARLA-only wall-clock timeout for route-anchor centering "
                "before crab entry"
            ),
        ),
        DeclareLaunchArgument(
            "carla_entry_anchor_centering_tolerance_m",
            default_value=os.environ.get(
                "CAMROD_CARLA_ENTRY_ANCHOR_CENTERING_TOLERANCE_M", "0.05"
            ),
            description=(
                "CARLA-only radial route-anchor tolerance required before "
                "crab entry"
            ),
        ),
        DeclareLaunchArgument(
            "carla_crab_entry_max_heading_drift_deg",
            default_value=os.environ.get(
                "CAMROD_CARLA_CRAB_ENTRY_MAX_HEADING_DRIFT_DEG", "0.0"
            ),
            description=(
                "CARLA-only fail-closed heading-drift limit during crab entry; "
                "ordinary CAMROD keeps this guard disabled"
            ),
        ),
        DeclareLaunchArgument(
            "carla_crab_entry_max_cross_track_error_m",
            default_value=os.environ.get(
                "CAMROD_CARLA_CRAB_ENTRY_MAX_CROSS_TRACK_ERROR_M", "0.0"
            ),
            description=(
                "CARLA-only fail-closed cross-track limit during crab entry; "
                "ordinary CAMROD keeps this guard disabled"
            ),
        ),
        DeclareLaunchArgument(
            "carla_crab_entry_body_yaw_compensation_deg",
            default_value=os.environ.get(
                "CAMROD_CARLA_CRAB_ENTRY_BODY_YAW_COMPENSATION_DEG", "0.0"
            ),
            description=(
                "CARLA-only direction-signed body-yaw correction for the "
                "88-degree physical crab-steering limit; ordinary CAMROD "
                "keeps the zero identity"
            ),
        ),
        DeclareLaunchArgument(
            "carla_crab_entry_body_yaw_alignment_tolerance_deg",
            default_value=os.environ.get(
                "CAMROD_CARLA_CRAB_ENTRY_BODY_YAW_ALIGNMENT_TOLERANCE_DEG",
                "0.5",
            ),
            description=(
                "CARLA-only precision tolerance for stationary crab-entry "
                "body-yaw precompensation and nominal restoration"
            ),
        ),
        DeclareLaunchArgument(
            "carla_crab_entry_body_yaw_alignment_timeout_s",
            default_value=os.environ.get(
                "CAMROD_CARLA_CRAB_ENTRY_BODY_YAW_ALIGNMENT_TIMEOUT_S", "15"
            ),
            description=(
                "CARLA-only wall-clock bound for stationary crab-entry "
                "body-yaw alignment"
            ),
        ),
        DeclareLaunchArgument(
            "carla_crab_out_yaw_recovery_enable",
            default_value=os.environ.get(
                "CAMROD_CARLA_CRAB_OUT_YAW_RECOVERY_ENABLE", "false"
            ),
            description=(
                "CARLA-only bounded stationary yaw recovery during the "
                "CRAB_OUT lateral stage; ordinary CAMROD keeps it disabled"
            ),
        ),
        DeclareLaunchArgument(
            "carla_crab_out_yaw_recovery_trigger_deg",
            default_value=os.environ.get(
                "CAMROD_CARLA_CRAB_OUT_YAW_RECOVERY_TRIGGER_DEG", "8.0"
            ),
            description="CARLA-only CRAB_OUT yaw-recovery trigger",
        ),
        DeclareLaunchArgument(
            "carla_crab_out_yaw_recovery_max_attempts",
            default_value=os.environ.get(
                "CAMROD_CARLA_CRAB_OUT_YAW_RECOVERY_MAX_ATTEMPTS", "3"
            ),
            description="CARLA-only CRAB_OUT yaw-recovery attempt bound",
        ),
        DeclareLaunchArgument(
            "carla_crab_out_yaw_recovery_global_timeout_s",
            default_value=os.environ.get(
                "CAMROD_CARLA_CRAB_OUT_YAW_RECOVERY_GLOBAL_TIMEOUT_S", "60.0"
            ),
            description=(
                "CARLA-only steady-clock bound from the first CRAB_OUT "
                "yaw-recovery trigger"
            ),
        ),
        DeclareLaunchArgument(
            "carla_roadside_reverse_handoff_distance_m",
            default_value=os.environ.get(
                "CAMROD_CARLA_ROADSIDE_REVERSE_HANDOFF_DISTANCE_M", "0.03"
            ),
            description=(
                "CARLA-only B11-B13 reverse centerline handoff. The live "
                "v27 brake/raycast audit retains about 0.25 m LEFT2 Terrain "
                "clearance at 0.10 m; hardware keeps its 0.03 m default"
            ),
        ),
        DeclareLaunchArgument(
            "carla_nav2_reverse_return_param_file",
            default_value=nav2_runtime_disabled_config,
            description=(
                "CARLA-only sparse Nav2 overlay enabling RPP reverse tracking"
            ),
        ),
        DeclareLaunchArgument(
            "carla_parking_runtime_override_param_file",
            default_value=parking_runtime_disabled_config,
            description=(
                "CARLA-only sparse reverse-parking geometry overlay"
            ),
        ),
        DeclareLaunchArgument(
            "carla_perception_runtime_override_param_file",
            default_value=perception_carla_config,
            description=(
                "CARLA-only sparse camera-to-LiDAR geometry overlay"
            ),
        ),
        DeclareLaunchArgument(
            "carla_apriltag_param_file",
            default_value=apriltag_detector_config,
            description=(
                "AprilTag detector profile forwarded into CAMROD bringup. "
                "The full/develop-parity default is the production profile; "
                "a CARLA wrapper may explicitly select a sensor-resolution "
                "adaptation without changing shared perception defaults"
            ),
        ),
        DeclareLaunchArgument(
            "carla_charging_contact_position_tolerance_m",
            default_value="0.35",
            description=(
                "Maximum measured vehicle-center distance from the canonical "
                "Drop Zone point before CARLA may assert charger contact"
            ),
        ),
        DeclareLaunchArgument(
            "carla_charging_contact_speed_tolerance_mps",
            default_value="0.05",
        ),
        DeclareLaunchArgument(
            "carla_charging_contact_dwell_s", default_value="1.0"
        ),
        DeclareLaunchArgument(
            "carla_charging_contact_state_timeout_s",
            default_value="2.0",
            description=(
                "Freshness bound for the CARLA-only IDLE + WAIT_DZ charging "
                "recovery after a CAMROD-only restart"
            ),
        ),
        DeclareLaunchArgument(
            "map_alignment_file", default_value=alignment_config
        ),
        DeclareLaunchArgument(
            "camrod_input_adapter_config",
            default_value=input_adapter_config,
        ),
        DeclareLaunchArgument(
            "carla_lidar_cost_grid_param_file",
            default_value=lidar_cost_grid_config,
            description=(
                "CARLA real-cloud cost raster profile; filters measured low "
                "road/parking-stop returns without changing UI sensor data"
            ),
        ),
        DeclareLaunchArgument(
            "nav2_bt_xml_nav_to_pose",
            default_value=nav_to_pose_bt,
            description=(
                "NavigateToPose tree. Parity mode uses CAMROD's production "
                "tree; a map-tuned wrapper may select a CARLA-specific tree"
            ),
        ),
        DeclareLaunchArgument(
            "nav2_bt_xml_nav_through_poses",
            default_value=nav_through_poses_bt,
        ),
        DeclareLaunchArgument(
            "camrod_launch_defaults_file", default_value=launch_defaults
        ),
        DeclareLaunchArgument(
            "camrod_system_checker_param_file",
            default_value=system_checker_config,
            description=(
                "CARLA graph-readiness manifest. The full CARLA composition "
                "uses production-shaped sensor relays, so it must not inherit "
                "the fake-sensor manifest selected by generic sim:=true"
            ),
        ),
        DeclareLaunchArgument(
            "camrod_map_path", default_value=camrod_map_path
        ),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("enable_plugin_api", default_value="true"),
        DeclareLaunchArgument("enable_api_ui", default_value="true"),
        DeclareLaunchArgument(
            "enable_operator_ui_window", default_value="true"
        ),
        DeclareLaunchArgument("api_ui_port", default_value="8010"),
        DeclareLaunchArgument(
            "operator_ui_window_url", default_value="http://127.0.0.1:8010"
        ),
        DeclareLaunchArgument("enable_voice", default_value="false"),
        DeclareLaunchArgument("initial_soc", default_value="0.8"),
        DeclareLaunchArgument("initial_charging", default_value="false"),
        DeclareLaunchArgument("initial_estop", default_value="false"),
        DeclareLaunchArgument("initial_error_code", default_value="0"),
        DeclareLaunchArgument("module_launch_gap_s", default_value="0.25"),
        DeclareLaunchArgument(
            "verified_baseline_manifest", default_value=baseline_manifest
        ),
        DeclareLaunchArgument(
            "verified_physical_four_wheel_manifest",
            default_value=physical_manifest,
        ),
        DeclareLaunchArgument(
            "extended_mode_backend",
            default_value="PHYSX_FOUR_WHEEL_STEERING",
        ),
        DeclareLaunchArgument(
            "accepted_carla_python_egg",
            default_value=accepted_carla_python_egg,
        ),
        DeclareLaunchArgument(
            "python_egg_cache", default_value=python_egg_cache
        ),
    ]

    actions = [
        _include(
            controller_launch,
            {
                "role_name": LaunchConfiguration("role_name"),
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "verified_baseline_manifest": LaunchConfiguration(
                    "verified_baseline_manifest"
                ),
                "verified_physical_four_wheel_manifest": LaunchConfiguration(
                    "verified_physical_four_wheel_manifest"
                ),
                "extended_mode_backend": LaunchConfiguration(
                    "extended_mode_backend"
                ),
                "accepted_carla_python_egg": LaunchConfiguration(
                    "accepted_carla_python_egg"
                ),
                "python_egg_cache": LaunchConfiguration("python_egg_cache"),
                "rotation_recovery_breakaway_enable": LaunchConfiguration(
                    "rotation_recovery_breakaway_enable"
                ),
            },
            condition=IfCondition(
                LaunchConfiguration("launch_vehicle_control")
            ),
        ),
        _include(
            adapter_launch,
            {
                "role_name": LaunchConfiguration("role_name"),
                "map_alignment_file": LaunchConfiguration(
                    "map_alignment_file"
                ),
                "recovery_breakaway_enable": LaunchConfiguration(
                    "recovery_breakaway_enable"
                ),
                "rotation_recovery_breakaway_enable": LaunchConfiguration(
                    "rotation_recovery_breakaway_enable"
                ),
                "rotation_recovery_breakaway_status_timeout_sec": (
                    LaunchConfiguration(
                        "rotation_recovery_breakaway_status_timeout_sec"
                    )
                ),
                # The standard /odom boundary is consumed by the unmodified
                # ranger_platform_bridge, which remains the sole owner of the
                # generated AvgOdometry /platform/status/odometry topic.
                "platform_odometry_topic": "/odom",
                "publish_platform_odometry": "true",
                "publish_metric_pose": "true",
                "publish_ground_truth_localization": "false",
                "publish_ground_truth_tf": "false",
                # CARLA's actual sensor actors own the canonical GNSS and IMU
                # boundaries. The fake-sensor include is disabled for both
                # below, so these relays are the sole publishers.
                "relay_imu": "true",
                "relay_gnss": "true",
            },
        ),
        _include(
            sensor_relay_launch,
            {
                "role_name": LaunchConfiguration("role_name"),
                "compressed_image_max_rate_hz": LaunchConfiguration(
                    "compressed_image_max_rate_hz"
                ),
                "raw_image_max_rate_hz": LaunchConfiguration(
                    "raw_image_max_rate_hz"
                ),
            },
            condition=IfCondition(LaunchConfiguration("launch_sensor_relay")),
        ),
        _include(
            lidar_processing_launch,
            {"enable": "true"},
            condition=IfCondition(
                LaunchConfiguration("launch_lidar_processing")
            ),
        ),
        _include(
            radar_relay_launch,
            {"role_name": LaunchConfiguration("role_name")},
            condition=IfCondition(LaunchConfiguration("launch_radar_relay")),
        ),
        _include(
            platform_heartbeat_launch,
            {
                "platform_heartbeat_publish_rate_hz": LaunchConfiguration(
                    "platform_heartbeat_publish_rate_hz"
                ),
                "initial_soc": LaunchConfiguration("initial_soc"),
                "initial_charging": LaunchConfiguration("initial_charging"),
                "initial_estop": LaunchConfiguration("initial_estop"),
                "initial_error_code": LaunchConfiguration(
                    "initial_error_code"
                ),
            },
            condition=IfCondition(
                LaunchConfiguration("launch_platform_heartbeat")
            ),
        ),
        _include(
            charging_contact_launch,
            {
                "enable": "true",
                "position_tolerance_m": LaunchConfiguration(
                    "carla_charging_contact_position_tolerance_m"
                ),
                "speed_tolerance_mps": LaunchConfiguration(
                    "carla_charging_contact_speed_tolerance_mps"
                ),
                "dwell_s": LaunchConfiguration(
                    "carla_charging_contact_dwell_s"
                ),
                "state_timeout_s": LaunchConfiguration(
                    "carla_charging_contact_state_timeout_s"
                ),
                "parking_status_topic": LaunchConfiguration(
                    "carla_charging_contact_parking_status_topic"
                ),
            },
            condition=IfCondition(
                LaunchConfiguration("launch_charging_contact_emulator")
            ),
        ),
        _include(
            bringup_launch,
            {
                "launch_defaults_file": LaunchConfiguration(
                    "camrod_launch_defaults_file"
                ),
                "map_path": LaunchConfiguration("camrod_map_path"),
                "clean_before_launch": "false",
                "clean_on_shutdown": "false",
                "sim": "true",
                "external_simulator": "true",
                "external_simulator_odometry_topic": "/odom",
                "external_simulator_odometry_timeout_s": "0.5",
                # CARLA owns odometry and all canonical sensor boundaries.
                # Keep the fake fixture out of the parity graph entirely.
                "enable_fake_sensors": "false",
                # Run the production mission-state contract even though the
                # hardware acquisition processes remain disabled by sim=true.
                "use_sim_planning_profile": LaunchConfiguration(
                    "use_sim_planning_profile"
                ),
                # CARLA relays already provide production-shaped odometry and
                # rear-camera inputs; retain production localization and
                # AprilTag parking parameter selection.
                "use_sim_localization_profile": LaunchConfiguration(
                    "use_sim_localization_profile"
                ),
                "use_sim_parking_method": LaunchConfiguration(
                    "use_sim_parking_method"
                ),
                # CARLA sensor_relay owns the canonical econ_front image and
                # CameraInfo topics consumed by production YOLO + fusion.
                "external_front_camera_source": LaunchConfiguration(
                    "launch_sensor_relay"
                ),
                # The same relay owns econ_rear image_raw/image_rect and
                # CameraInfo for the production AprilTag parking detector.
                "external_rear_camera_source": LaunchConfiguration(
                    "launch_sensor_relay"
                ),
                # The remaining external-motion fixture keeps its deterministic
                # 10 Hz cadence. All UI-visible fake sensor outputs are disabled
                # below and replaced by CARLA relays.
                "sim_fake_sensor_publish_rate_hz": "10.0",
                "sim_platform_status_enable": "true",
                "sim_publish_platform_status": "false",
                # CARLA radar relay is the sole owner of canonical seven-range
                # topics in this external-simulator composition.
                "sim_publish_fake_radar_ranges": "false",
                # Every UI-visible sensor topic must originate at a CARLA
                # actor. fake_sensor_publisher is not launched in this graph.
                "sim_publish_fake_gnss": "false",
                "sim_publish_fake_imu": "false",
                "sim_publish_fake_lidar_obstacle_cloud": "false",
                "sim_publish_velocity_converter_output": "false",
                "sim_publish_dummy_lidar_cost_grid": "false",
                # carla_lidar_processing.launch.py owns the optional LiDAR
                # visualization clusters. Production camera-LiDAR fusion is
                # the sole semantic /perception/obstacles publisher. Suppress
                # the ordinary cluster instance to avoid duplicate consumers.
                "perception_enable_lidar_obstacle": "false",
                # Rendered CARLA owns real camera actors and runs the production
                # YOLO/fusion chain. NullRHI selects the control-only spawn and
                # must not instantiate a detector against a missing camera or
                # an absent host-local TensorRT engine.
                "perception_enable_yolo": LaunchConfiguration(
                    "launch_sensor_relay"
                ),
                "perception_mode": PythonExpression([
                    "'camera_lidar' if '",
                    LaunchConfiguration("launch_sensor_relay"),
                    "'.lower() in ('1', 'true', 'yes', 'on') ",
                    "else 'lidar_only'",
                ]),
                "perception_runtime_override_param_file": (
                    LaunchConfiguration(
                        "carla_perception_runtime_override_param_file"
                    )
                ),
                # Full/develop parity resolves to the production detector file.
                # Only an explicit CARLA wrapper may replace this input.
                "apriltag_param_file": LaunchConfiguration(
                    "carla_apriltag_param_file"
                ),
                # CARLA wall-clock sensor cadence depends on rendered server
                # load. Apply only CARLA sensor thresholds, then inherit every
                # other simulation diagnostic before hardware defaults.
                "diagnostics_profile": "carla",
                "diagnostics_profile_fallback": "sim,default",
                # sim=true normally selects system_checker_sim.yaml, whose
                # graph contract requires /bringup/fake_sensor_publisher.
                # CARLA instead owns production-shaped sensor boundaries;
                # select that graph manifest explicitly without changing the
                # generic bringup mode-selection contract.
                "system_checker_param_file": LaunchConfiguration(
                    "camrod_system_checker_param_file"
                ),
                "platform_ranger_driver_enable": "false",
                "platform_ranger_bridge_enable": "true",
                # Parity mode selects CAMROD's production sensor-input adapter.
                # The tuned wrapper may explicitly select a metric CARLA pose
                # adapter; it is never injected into ordinary CAMROD defaults.
                "localization_adapter_param_file": LaunchConfiguration(
                    "camrod_input_adapter_config"
                ),
                "lidar_cost_grid_param_file": LaunchConfiguration(
                    "carla_lidar_cost_grid_param_file"
                ),
                "control_manual_cmd_vel_ros_topic": LaunchConfiguration(
                    "manual_cmd_vel_ros_topic"
                ),
                "control_manual_drive_linear_limit_mps": LaunchConfiguration(
                    "manual_drive_linear_limit_mps"
                ),
                "control_manual_drive_lateral_limit_mps": LaunchConfiguration(
                    "manual_drive_lateral_limit_mps"
                ),
                "control_manual_drive_angular_limit_radps": LaunchConfiguration(
                    "manual_drive_angular_limit_radps"
                ),
                "control_manual_drive_deadman_timeout_s": LaunchConfiguration(
                    "manual_drive_deadman_timeout_s"
                ),
                "control_camping_site_crab_approach_slowdown_distance_m": (
                    LaunchConfiguration(
                        "carla_crab_approach_slowdown_distance_m"
                    )
                ),
                "control_camping_site_crab_approach_min_speed_mps": (
                    LaunchConfiguration("carla_crab_approach_min_speed_mps")
                ),
                "control_camping_site_rotate_180_timeout_s": (
                    LaunchConfiguration("carla_rotate_180_timeout_s")
                ),
                "control_camping_site_max_angular_speed_radps": (
                    LaunchConfiguration(
                        "carla_camping_site_max_angular_speed_radps"
                    )
                ),
                "control_camping_site_entry_position_tolerance_m": (
                    LaunchConfiguration("carla_entry_position_tolerance_m")
                ),
                "control_camping_site_return_lateral_hysteresis_m": (
                    LaunchConfiguration(
                        "carla_return_lateral_hysteresis_m"
                    )
                ),
                "control_camping_site_rotate_entry_max_position_error_m": (
                    LaunchConfiguration(
                        "carla_rotate_entry_max_position_error_m"
                    )
                ),
                "control_camping_site_rotate_entry_centering_max_initial_error_m": (
                    LaunchConfiguration(
                        "carla_rotate_entry_centering_max_initial_error_m"
                    )
                ),
                "control_camping_site_entry_anchor_centering_max_initial_error_m": (
                    LaunchConfiguration(
                        "carla_entry_anchor_centering_max_initial_error_m"
                    )
                ),
                "control_camping_site_entry_anchor_centering_max_speed_mps": (
                    LaunchConfiguration(
                        "carla_entry_anchor_centering_max_speed_mps"
                    )
                ),
                "control_camping_site_entry_anchor_centering_timeout_s": (
                    LaunchConfiguration(
                        "carla_entry_anchor_centering_timeout_s"
                    )
                ),
                "control_camping_site_entry_anchor_centering_tolerance_m": (
                    LaunchConfiguration(
                        "carla_entry_anchor_centering_tolerance_m"
                    )
                ),
                "control_camping_site_crab_entry_max_heading_drift_deg": (
                    LaunchConfiguration(
                        "carla_crab_entry_max_heading_drift_deg"
                    )
                ),
                "control_camping_site_crab_entry_max_cross_track_error_m": (
                    LaunchConfiguration(
                        "carla_crab_entry_max_cross_track_error_m"
                    )
                ),
                "control_camping_site_crab_entry_body_yaw_compensation_deg": (
                    LaunchConfiguration(
                        "carla_crab_entry_body_yaw_compensation_deg"
                    )
                ),
                "control_camping_site_crab_entry_body_yaw_alignment_tolerance_deg": (
                    LaunchConfiguration(
                        "carla_crab_entry_body_yaw_alignment_tolerance_deg"
                    )
                ),
                "control_camping_site_crab_entry_body_yaw_alignment_timeout_s": (
                    LaunchConfiguration(
                        "carla_crab_entry_body_yaw_alignment_timeout_s"
                    )
                ),
                "control_camping_site_crab_out_yaw_recovery_enable": (
                    LaunchConfiguration(
                        "carla_crab_out_yaw_recovery_enable"
                    )
                ),
                "control_camping_site_crab_out_yaw_recovery_trigger_deg": (
                    LaunchConfiguration(
                        "carla_crab_out_yaw_recovery_trigger_deg"
                    )
                ),
                "control_camping_site_crab_out_yaw_recovery_max_attempts": (
                    LaunchConfiguration(
                        "carla_crab_out_yaw_recovery_max_attempts"
                    )
                ),
                "control_camping_site_crab_out_yaw_recovery_global_timeout_s": (
                    LaunchConfiguration(
                        "carla_crab_out_yaw_recovery_global_timeout_s"
                    )
                ),
                "control_camping_site_roadside_reverse_return_enable": (
                    LaunchConfiguration(
                        "carla_roadside_reverse_return_enable"
                    )
                ),
                "control_camping_site_roadside_reverse_handoff_distance_m": (
                    LaunchConfiguration(
                        "carla_roadside_reverse_handoff_distance_m"
                    )
                ),
                "planning_nav2_runtime_override_param_file": (
                    LaunchConfiguration(
                        "carla_nav2_reverse_return_param_file"
                    )
                ),
                # Parity mode keeps the develop controller and goal source.
                # The Woraksan tuned wrapper explicitly enables RPPReverse.
                "planning_nav2_reverse_controller": LaunchConfiguration(
                    "carla_nav2_reverse_controller"
                ),
                "planning_goal_snapper_reverse_auxiliary_input_goal_topic": (
                    LaunchConfiguration("carla_reverse_goal_topic")
                ),
                # The centerline snapper may switch lanelet branches by metres
                # even though the CARLA actor moves continuously. Detect true
                # teleports from raw fused localization instead.
                "planning_goal_snapper_pose_jump_check_topic": (
                    LaunchConfiguration(
                        "carla_goal_snapper_pose_jump_check_topic"
                    )
                ),
                "planning_goal_snapper_reissue_active_goal_after_route_recovery_when_nav_active": (
                    LaunchConfiguration("carla_goal_reissue_while_nav_active")
                ),
                "planning_state_machine_reverse_auto_goal_snapper_input_topic": (
                    LaunchConfiguration("carla_reverse_goal_topic")
                ),
                "parking_runtime_override_param_file": LaunchConfiguration(
                    "carla_parking_runtime_override_param_file"
                ),
                # Reverse navigation is ordinary DONE-state motion, not a
                # maneuver bypass. Enforce the rear lanelet corridor as well
                # as the always-on physical body and planning footprint checks.
                "control_cmd_vel_gate_lanelet_safety_check_reverse": (
                    LaunchConfiguration("carla_lanelet_safety_check_reverse")
                ),
                "control_cmd_vel_gate_route_safety_path_relative_recovery_enable": (
                    LaunchConfiguration(
                        "carla_route_safety_path_relative_recovery_enable"
                    )
                ),
                "control_cmd_vel_gate_route_safety_path_center_reentry_m": (
                    LaunchConfiguration(
                        "carla_route_safety_path_center_reentry_m"
                    )
                ),
                "control_route_safety_recovery_zero_hold_pauses_limits": (
                    LaunchConfiguration(
                        "carla_route_safety_zero_hold_pauses_limits"
                    )
                ),
                "control_route_safety_recovery_allow_corrective_yaw_beyond_limit": (
                    LaunchConfiguration(
                        "carla_route_safety_allow_corrective_yaw_beyond_limit"
                    )
                ),
                # HH_260829 - The visible v16 A/B run proved that the prior
                # CARLA-only 2.75 m / 45 degree override freshly armed an
                # unnecessary ZERO_TURN at the B10 hairpin.  The production
                # 2.0 m / 75 degree profile crossed that point with no
                # ZERO_TURN or collision, so CARLA now defaults to the same
                # controller geometry. Environment overrides remain available
                # for explicit experiments without changing production files.
                "control_cmd_vel_gate_route_heading_lookahead_m": (
                    LaunchConfiguration(
                        "carla_route_heading_lookahead_m"
                    )
                ),
                "control_cmd_vel_gate_route_heading_error_enter_deg": (
                    LaunchConfiguration(
                        "carla_route_heading_error_enter_deg"
                    )
                ),
                # Parity mode resolves to the production 0.5 scale. The tuned
                # wrapper alone selects unity for its historical reverse-return
                # profile, without mutating the shared control configuration.
                "control_cmd_vel_gate_speed_scale": LaunchConfiguration(
                    "carla_cmd_vel_gate_speed_scale"
                ),
                # Parity mode keeps the production 0.0 no-op. The historical
                # tuned wrapper alone applies the 0.82 m Ranger plant envelope.
                "control_cmd_vel_gate_navigation_minimum_ackermann_turn_radius_m": (
                    LaunchConfiguration(
                        "carla_navigation_minimum_ackermann_turn_radius_m"
                    )
                ),
                # Parity mode keeps all production cost thresholds at 85. The
                # historical tuned profile explicitly raises them to 100 for
                # the measured Woraksan map-inflation cohort.
                "control_cmd_vel_gate_cost_threshold": LaunchConfiguration(
                    "carla_cost_stop_threshold"
                ),
                "control_cmd_vel_gate_cost_stop_latch_use_trigger_source_for_merged_clear": (
                    LaunchConfiguration(
                        "carla_cost_stop_latch_use_trigger_source_for_merged_clear"
                    )
                ),
                "control_cmd_vel_gate_cost_stop_merged_dynamic_source_labels": (
                    LaunchConfiguration(
                        "carla_cost_stop_merged_dynamic_source_labels"
                    )
                ),
                "control_cmd_vel_gate_lanelet_safety_threshold": (
                    LaunchConfiguration("carla_lanelet_safety_threshold")
                ),
                "control_cmd_vel_gate_lanelet_safety_current_threshold": (
                    LaunchConfiguration("carla_lanelet_safety_current_threshold")
                ),
                "control_cmd_vel_gate_lanelet_safety_footprint_enable": (
                    LaunchConfiguration(
                        "carla_lanelet_safety_footprint_enable"
                    )
                ),
                "planning_nav2_bt_xml_nav_to_pose": LaunchConfiguration(
                    "nav2_bt_xml_nav_to_pose"
                ),
                "planning_nav2_bt_xml_nav_through_poses": LaunchConfiguration(
                    "nav2_bt_xml_nav_through_poses"
                ),
                "rviz": LaunchConfiguration("rviz"),
                "enable_plugin_api": LaunchConfiguration("enable_plugin_api"),
                "enable_api_ui": LaunchConfiguration("enable_api_ui"),
                # Keep develop's raw-camera fallback contract in parity mode.
                # CARLA's relay is subscriber-aware and rate-bounds raw frames;
                # the tuned wrapper may explicitly disable this fallback.
                "operator_telemetry_camera_raw_fallback_enabled": (
                    LaunchConfiguration(
                        "operator_telemetry_camera_raw_fallback_enabled"
                    )
                ),
                "operator_telemetry_tf_transform_enabled": (
                    LaunchConfiguration(
                        "operator_telemetry_tf_transform_enabled"
                    )
                ),
                "operator_telemetry_raw_lidar_bbox_overlay_enabled": (
                    LaunchConfiguration(
                        "operator_telemetry_raw_lidar_bbox_overlay_enabled"
                    )
                ),
                "operator_telemetry_tf_latest_fallback_tolerance_s": (
                    LaunchConfiguration(
                        "operator_telemetry_tf_latest_fallback_tolerance_s"
                    )
                ),
                "operator_telemetry_docking_rear_camera_fallback_enabled": (
                    LaunchConfiguration(
                        "operator_telemetry_docking_rear_camera_fallback_enabled"
                    )
                ),
                "return_site_exit_rearm_enabled": LaunchConfiguration(
                    "return_site_exit_rearm_enabled"
                ),
                "enable_operator_ui_window": LaunchConfiguration(
                    "enable_operator_ui_window"
                ),
                "api_ui_port": LaunchConfiguration("api_ui_port"),
                "operator_ui_window_url": LaunchConfiguration(
                    "operator_ui_window_url"
                ),
                "enable_voice": LaunchConfiguration("enable_voice"),
                "module_launch_gap_s": LaunchConfiguration(
                    "module_launch_gap_s"
                ),
            },
        ),
    ]

    return LaunchDescription([*declarations, *actions])


if __name__ == "__main__":
    generate_launch_description()
