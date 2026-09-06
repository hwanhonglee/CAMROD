"""Contracts for the full CAMROD-on-CARLA composition."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.utilities import perform_substitutions
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parent
FULL_LAUNCH = PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
DEVELOP_SITE_GEOMETRY_LAUNCH = (
    PACKAGE_ROOT
    / "launch"
    / "camrod_carla_develop_site_geometry.launch.py"
)
TUNED_LAUNCH = (
    PACKAGE_ROOT / "launch" / "camrod_carla_woraksan_tuned.launch.py"
)
CARLA_APRILTAG_CONFIG = (
    PACKAGE_ROOT / "config" / "apriltag_parking_detector_carla.yaml"
)
CARLA_APRILTAG_CONTROLLER_CONFIG = (
    PACKAGE_ROOT / "config" / "apriltag_parking_controller_carla.yaml"
)
CARLA_PERCEPTION_CONFIG = (
    PACKAGE_ROOT / "config" / "perception_carla.yaml"
)
CARLA_SITE_GEOMETRY_PERCEPTION_CONFIG = (
    PACKAGE_ROOT / "config" / "perception_carla_site_geometry.yaml"
)
PRODUCTION_APRILTAG_CONFIG = (
    REPO_ROOT
    / "camrod_perception"
    / "config"
    / "apriltag_parking_detector.yaml"
)
BRINGUP_APRILTAG_CONFIG = (
    REPO_ROOT
    / "camrod_bringup"
    / "config"
    / "perception"
    / "apriltag_parking_detector.yaml"
)
STATE_MACHINE_LAUNCH = (
    REPO_ROOT / "camrod_planning" / "launch" / "state_machine.launch.py"
)


def _deep_merge(base, overlay):
    """Mirror launch-style YAML overlay semantics for focused config contracts."""
    if isinstance(base, dict) and isinstance(overlay, dict):
        merged = dict(base)
        for key, value in overlay.items():
            merged[key] = _deep_merge(merged.get(key), value)
        return merged
    return overlay


def _load_module(path):
    spec = importlib.util.spec_from_file_location(
        path.name.replace(".", "_"), str(path)
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _resolve_rate(module, primary_name, primary_value, legacy_value):
    context = LaunchContext()
    context.launch_configurations[primary_name] = primary_value
    context.launch_configurations["publish_rate_hz"] = legacy_value
    substitution = module._prefer_explicit_legacy_rate(primary_name)
    return perform_substitutions(context, [substitution])


def test_develop_site_geometry_wrapper_is_the_exact_proven_carla_subset():
    module = _load_module(DEVELOP_SITE_GEOMETRY_LAUNCH)
    source = DEVELOP_SITE_GEOMETRY_LAUNCH.read_text(encoding="utf-8")

    assert module.DEVELOP_SITE_GEOMETRY_ARGUMENTS == {
        "operator_telemetry_docking_rear_camera_fallback_enabled": "true",
        "carla_cmd_vel_gate_speed_scale": "1.0",
        "carla_allow_manual_departure_while_charging": "true",
        "return_site_exit_rearm_enabled": "true",
        "launch_charging_contact_emulator": "true",
        "carla_charging_contact_parking_status_topic": (
            "/parking/apriltag_parking_controller/status"
        ),
        "recovery_breakaway_enable": "true",
        "rotation_recovery_breakaway_enable": "true",
        "rotation_recovery_breakaway_status_timeout_sec": "1.25",
        "carla_goal_snapper_pose_jump_check_topic": "/localization/pose",
        "carla_route_safety_path_relative_recovery_enable": "true",
        "carla_route_safety_path_center_reentry_m": "0.15",
        "carla_roadside_reverse_return_enable": "true",
        "carla_roadside_reverse_handoff_distance_m": "0.03",
        "carla_nav2_reverse_controller": "RPPReverse",
        "carla_reverse_goal_topic": "/planning/auto_reverse_goal_raw",
        "carla_lanelet_safety_check_reverse": "true",
        "carla_camping_site_maneuver_controller_static_bypass_phases": (
            "ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,"
            "ALIGN_RETRACE_YAW,ALIGN_OUTBOUND_LANE_YAW,REVERSE_OUT,CRAB_OUT"
        ),
        "carla_camping_site_maneuver_controller_lanelet_bypass_phases": (
            "ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,"
            "ALIGN_RETRACE_YAW,ALIGN_OUTBOUND_LANE_YAW,REVERSE_OUT,CRAB_OUT"
        ),
        "carla_return_goal_reached_distance_m": "0.35",
        "carla_lanelet_safety_footprint_enable": "false",
        "carla_cost_stop_latch_use_trigger_source_for_merged_clear": "true",
        "carla_cost_stop_merged_dynamic_source_labels": "radar",
        "carla_route_safety_zero_hold_pauses_limits": "true",
        "carla_route_safety_allow_corrective_yaw_beyond_limit": "true",
        "carla_goal_reissue_while_nav_active": "true",
        "carla_crab_approach_slowdown_distance_m": "1.0",
        "carla_crab_approach_min_speed_mps": "0.12",
        "carla_rotate_180_timeout_s": "90.0",
        "carla_camping_site_max_angular_speed_radps": "0.45",
        "carla_entry_position_tolerance_m": "0.05",
        "carla_rotate_entry_max_position_error_m": "0.05",
        "carla_rotate_entry_centering_max_initial_error_m": "0.65",
        "carla_entry_anchor_centering_max_initial_error_m": "0.0",
        "carla_entry_anchor_centering_max_speed_mps": "0.12",
        "carla_entry_anchor_centering_timeout_s": "15",
        "carla_entry_anchor_centering_tolerance_m": "0.05",
        "carla_crab_entry_max_heading_drift_deg": "0.0",
        "carla_crab_entry_body_yaw_compensation_deg": "2.0",
        "carla_crab_entry_body_yaw_alignment_tolerance_deg": "1.5",
        "carla_crab_entry_body_yaw_alignment_timeout_s": "15",
        "carla_crab_out_yaw_recovery_enable": "true",
        "carla_crab_out_yaw_recovery_trigger_deg": "8.0",
        "carla_crab_out_yaw_recovery_max_attempts": "8",
        "carla_crab_out_yaw_recovery_global_timeout_s": "90.0",
    }
    assert "camrod_carla_full.launch.py" in source
    assert "camrod_carla_woraksan_tuned.launch.py" not in source
    resolved = module.develop_site_geometry_arguments("/adapter-share")
    assert set(resolved) == {
        *module.DEVELOP_SITE_GEOMETRY_ARGUMENTS,
        "carla_perception_runtime_override_param_file",
        "carla_apriltag_param_file",
        "carla_parking_runtime_override_param_file",
        "carla_nav2_reverse_return_param_file",
    }
    assert resolved["carla_perception_runtime_override_param_file"] == (
        "/adapter-share/config/perception_carla_site_geometry.yaml"
    )
    assert resolved["carla_apriltag_param_file"] == (
        "/adapter-share/config/apriltag_parking_detector_carla.yaml"
    )
    assert resolved["carla_parking_runtime_override_param_file"] == (
        "/adapter-share/config/apriltag_parking_controller_carla.yaml"
    )
    assert resolved["carla_nav2_reverse_return_param_file"] == (
        "/adapter-share/config/nav2_carla_reverse_return.yaml"
    )
    assert resolved["carla_cmd_vel_gate_speed_scale"] == "1.0"
    assert (
        resolved[
            "operator_telemetry_docking_rear_camera_fallback_enabled"
        ]
        == "true"
    )
    assert resolved["carla_lanelet_safety_footprint_enable"] == "false"
    for forbidden_override in (
        "use_sim_planning_profile",
        "use_sim_localization_profile",
        "use_sim_parking_method",
        "manual_drive_linear_limit_mps",
        "carla_navigation_minimum_ackermann_turn_radius_m",
        "carla_cost_stop_threshold",
        "carla_lanelet_safety_threshold",
        "camrod_input_adapter_config",
        "carla_lidar_cost_grid_param_file",
    ):
        assert forbidden_override not in module.DEVELOP_SITE_GEOMETRY_ARGUMENTS

    # The shared CARLA full launch keeps this false.  Only the site-geometry
    # evidence overlay may substitute the real rear CARLA frame when the
    # event-driven AprilTag debug image is stale.
    full_source = FULL_LAUNCH.read_text(encoding="utf-8")
    fallback_declaration = full_source.index(
        '"operator_telemetry_docking_rear_camera_fallback_enabled"'
    )
    assert 'default_value="false"' in full_source[
        fallback_declaration:fallback_declaration + 360
    ]

    # The B1 plant residual is scoped to this wrapper: full/develop parity
    # and the historical tuned wrapper remain at their established 0.5 deg.
    compact_full = "".join(FULL_LAUNCH.read_text(encoding="utf-8").split())
    compact_tuned = "".join(
        TUNED_LAUNCH.read_text(encoding="utf-8").split()
    )
    full_module = _load_module(FULL_LAUNCH)
    assert (
        module.DEVELOP_SITE_GEOMETRY_ARGUMENTS[
            "carla_crab_entry_body_yaw_alignment_tolerance_deg"
        ]
        == "1.5"
    )
    assert (
        '"CAMROD_CARLA_CRAB_ENTRY_BODY_YAW_ALIGNMENT_TOLERANCE_DEG","0.5"'
        in compact_full
    )
    assert (
        '"carla_lanelet_safety_footprint_enable",default_value="true"'
        in compact_full
    )
    assert (
        '"carla_return_goal_reached_distance_m",default_value=""'
        in compact_full
    )
    assert (
        '"control_cmd_vel_gate_lanelet_safety_footprint_enable":('
        'LaunchConfiguration("carla_lanelet_safety_footprint_enable")'
        in compact_full
    )
    base_bypass = (
        "ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,"
        "ALIGN_RETRACE_YAW,REVERSE_OUT,CRAB_OUT"
    )
    assert full_module.DEVELOP_CAMPSITE_BYPASS_PHASES == base_bypass
    assert compact_full.count(
        "default_value=DEVELOP_CAMPSITE_BYPASS_PHASES"
    ) == 2
    assert (
        module.DEVELOP_SITE_GEOMETRY_ARGUMENTS[
            "carla_camping_site_maneuver_controller_static_bypass_phases"
        ].split(",").count("ALIGN_OUTBOUND_LANE_YAW")
        == 1
    )
    assert (
        module.DEVELOP_SITE_GEOMETRY_ARGUMENTS[
            "carla_camping_site_maneuver_controller_lanelet_bypass_phases"
        ].split(",").count("ALIGN_OUTBOUND_LANE_YAW")
        == 1
    )
    assert (
        '"control_cmd_vel_gate_camping_site_maneuver_controller_static_bypass_phases":('
        'LaunchConfiguration('
        '"carla_camping_site_maneuver_controller_static_bypass_phases"))'
        in compact_full
    )
    assert (
        '"control_cmd_vel_gate_camping_site_maneuver_controller_lanelet_bypass_phases":('
        'LaunchConfiguration('
        '"carla_camping_site_maneuver_controller_lanelet_bypass_phases"))'
        in compact_full
    )
    assert (
        '"carla_crab_entry_body_yaw_alignment_tolerance_deg":"0.5"'
        in compact_tuned
    )

    # v20 restores exact develop return-latch tolerances.  The narrow B2
    # simulator contact is handled by the isolated access map, not by masking
    # controller drift with a larger hysteresis.
    production = yaml.safe_load(
        (REPO_ROOT / "camrod_control" / "config" / "control.yaml").read_text(
            encoding="utf-8"
        )
    )["/control/camping_site_maneuver_controller"]["ros__parameters"]
    bringup = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "control"
            / "control.yaml"
        ).read_text(encoding="utf-8")
    )["/control/camping_site_maneuver_controller"]["ros__parameters"]
    for parameters in (production, bringup):
        assert parameters["return_lateral_transition_tolerance_m"] == 0.02
        assert parameters["return_lateral_hysteresis_m"] == 0.10
    assert "carla_return_lateral_hysteresis_m" not in (
        module.DEVELOP_SITE_GEOMETRY_ARGUMENTS
    )
    assert '"CAMROD_CARLA_RETURN_LATERAL_HYSTERESIS_M",""' in compact_full
    assert '"carla_return_lateral_hysteresis_m"' not in compact_tuned

    # The yaw hook is fully wired but remains inactive in every shared/default
    # composition; only this wrapper owns the measured CARLA retry budget.
    for parameters in (production, bringup):
        assert parameters["crab_out_yaw_recovery_enable"] is False
        assert parameters["crab_out_yaw_recovery_max_attempts"] == 3
        assert parameters["crab_out_yaw_recovery_global_timeout_s"] == 60.0
    assert (
        module.DEVELOP_SITE_GEOMETRY_ARGUMENTS[
            "carla_crab_out_yaw_recovery_enable"
        ]
        == "true"
    )
    assert (
        module.DEVELOP_SITE_GEOMETRY_ARGUMENTS[
            "carla_crab_out_yaw_recovery_max_attempts"
        ]
        == "8"
    )

    # v17 asks only the CARLA site wrapper for enough yaw-rate error to reach
    # the already accepted 2 N*m/wheel rotation cap. Production/develop and
    # the generic CARLA launch retain the 0.35 rad/s control.yaml value.
    for parameters in (production, bringup):
        assert parameters["max_angular_speed_radps"] == 0.35
    assert (
        module.DEVELOP_SITE_GEOMETRY_ARGUMENTS[
            "carla_camping_site_max_angular_speed_radps"
        ]
        == "0.45"
    )
    assert (
        '"CAMROD_CARLA_CAMPING_SITE_MAX_ANGULAR_SPEED_RADPS",""'
        in compact_full
    )
    assert '"carla_camping_site_max_angular_speed_radps"' not in compact_tuned


def test_site_apriltag_controller_overlay_changes_response_not_safety_limits():
    parameters = yaml.safe_load(
        CARLA_APRILTAG_CONTROLLER_CONFIG.read_text(encoding="utf-8")
    )["/parking/apriltag_parking_controller"]["ros__parameters"]

    assert parameters == {
        "heading_gain": 1.5,
        "lateral_to_heading_gain": 2.7,
        "reverse_approach_speed_mps": 0.2,
        "final_insertion_speed_mps": 0.05,
        "enable_bounded_lateral_retry": True,
        "retry_forward_distance_m": 0.8,
        "retry_forward_speed_mps": 0.20,
        "retry_forward_timeout_s": 30.0,
        "retry_yaw_alignment_timeout_s": 8.0,
        "retry_maximum_lateral_error_m": 0.15,
        "retry_maximum_heading_error_rad": 0.35,
        "retry_maximum_forward_exit_lateral_drift_m": 0.15,
        "retry_maximum_odometry_step_m": 0.10,
        "retry_minimum_tag_distance_m": 0.35,
        "retry_maximum_tag_distance_m": 0.45,
        "maximum_retries": 2,
    }
    source = CARLA_APRILTAG_CONTROLLER_CONFIG.read_text(encoding="utf-8")
    assert "translation_stop_tag_distance_m" not in parameters
    assert "final_lateral_tolerance_m" not in parameters
    assert "minimum_approach_turn_radius_m" not in parameters
    gated_timeout_budget_m = (
        parameters["retry_forward_speed_mps"] * 0.5
        * parameters["retry_forward_timeout_s"]
    )
    assert gated_timeout_budget_m == 3.0
    assert parameters["retry_forward_distance_m"] <= gated_timeout_budget_m
    assert (
        parameters["retry_forward_distance_m"]
        + parameters["retry_maximum_odometry_step_m"]
    ) == 0.9
    assert "Do not relax" in source
    assert "scope collision" in source


def test_carla_apriltag_profile_changes_only_detector_decimation():
    node = "/perception/apriltag_parking_detector"
    production_document = yaml.safe_load(
        PRODUCTION_APRILTAG_CONFIG.read_text(encoding="utf-8")
    )
    bringup_document = yaml.safe_load(
        BRINGUP_APRILTAG_CONFIG.read_text(encoding="utf-8")
    )
    carla_document = yaml.safe_load(
        CARLA_APRILTAG_CONFIG.read_text(encoding="utf-8")
    )

    assert bringup_document == production_document
    production = dict(production_document[node]["ros__parameters"])
    carla = dict(carla_document[node]["ros__parameters"])
    assert production.pop("quad_decimate") == 2.0
    assert carla.pop("quad_decimate") == 1.0
    assert carla == production
    assert carla["tag_family"] == "tag36h11"
    assert carla["target_tag_id"] == 3
    assert carla["tag_size"] == 0.16
    assert carla["n_threads"] == 2


def test_carla_perception_overlays_keep_develop_parity_and_scope_site_tuning():
    shared = yaml.safe_load(
        (
            REPO_ROOT / "camrod_perception" / "config" / "perception_params.yaml"
        ).read_text(encoding="utf-8")
    )
    carla = yaml.safe_load(CARLA_PERCEPTION_CONFIG.read_text(encoding="utf-8"))
    site = yaml.safe_load(
        CARLA_SITE_GEOMETRY_PERCEPTION_CONFIG.read_text(encoding="utf-8")
    )

    assert shared["/perception/yolov9mit"]["ros__parameters"][
        "min_confidence"
    ] == 0.5
    assert "/perception/yolov9mit" not in carla
    assert site["/perception/yolov9mit"]["ros__parameters"] == {
        "min_confidence": 0.95
    }
    expected_extrinsic = {
        "extrinsic_x": 0.0,
        "extrinsic_y": 0.00001,
        "extrinsic_z": -0.09970,
    }
    assert carla["/perception/obstacle_fusion"]["ros__parameters"] == (
        expected_extrinsic
    )
    assert site["/perception/obstacle_fusion"]["ros__parameters"] == (
        expected_extrinsic
    )


def test_full_and_site_launches_select_distinct_perception_overlays():
    full = FULL_LAUNCH.read_text(encoding="utf-8")
    site = _load_module(DEVELOP_SITE_GEOMETRY_LAUNCH)

    assert 'adapter_share, "config", "perception_carla.yaml"' in full
    assert site.develop_site_geometry_arguments("/adapter-share")[
        "carla_perception_runtime_override_param_file"
    ] == "/adapter-share/config/perception_carla_site_geometry.yaml"


def test_full_launch_defaults_to_production_apriltag_profile():
    source = FULL_LAUNCH.read_text(encoding="utf-8")
    tuned = TUNED_LAUNCH.read_text(encoding="utf-8")

    assert 'get_package_share_directory("camrod_perception")' in source
    assert (
        'perception_share, "config", "apriltag_parking_detector.yaml"'
        in source
    )
    declaration = source.split('"carla_apriltag_param_file",', 1)[1]
    declaration = declaration.split("DeclareLaunchArgument", 1)[0]
    assert "default_value=apriltag_detector_config" in declaration
    assert (
        '"apriltag_param_file": LaunchConfiguration(\n'
        '                    "carla_apriltag_param_file"'
        in source
    )
    assert "carla_apriltag_param_file" not in tuned


def test_full_launch_keeps_carla_lifecycle_external_and_enables_full_bringup():
    source = FULL_LAUNCH.read_text(encoding="utf-8")
    tuned = TUNED_LAUNCH.read_text(encoding="utf-8")
    assert 'get_package_share_directory("carla_ros_bridge")' not in source
    assert 'get_package_share_directory("carla_spawn_objects")' not in source
    assert '"sim": "true"' in source
    assert '"external_simulator": "true"' in source
    assert '"enable_fake_sensors": "false"' in source
    assert (
        '"use_sim_planning_profile": LaunchConfiguration('
        in source
    )
    assert (
        '"use_sim_localization_profile": LaunchConfiguration('
        in source
    )
    assert (
        '"use_sim_parking_method": LaunchConfiguration('
        in source
    )
    for setting in (
        "external_front_camera_source",
        "external_rear_camera_source",
        "perception_enable_yolo",
    ):
        assert (
            f'"{setting}": LaunchConfiguration(\n'
            '                    "launch_sensor_relay"'
            in source
        )
    assert "'camera_lidar' if '" in source
    assert "else 'lidar_only'" in source
    assert '"sim_platform_status_enable": "true"' in source
    assert (
        '"operator_telemetry_raw_lidar_bbox_overlay_enabled",'
        in source
    )
    raw_bbox_declaration = source.split(
        '"operator_telemetry_raw_lidar_bbox_overlay_enabled",', 1
    )[1].split("DeclareLaunchArgument", 1)[0]
    assert 'default_value="false"' in raw_bbox_declaration
    assert (
        '"operator_telemetry_raw_lidar_bbox_overlay_enabled": ('
        in source
    )
    assert '"clean_before_launch": "false"' in source
    assert '"platform_odometry_topic": "/odom"' in source
    assert '"platform_ranger_bridge_enable": "true"' in source
    assert '"diagnostics_profile": "carla"' in source
    assert '"diagnostics_profile_fallback": "sim,default"' in source
    assert 'get_package_share_directory("camrod_system")' in source
    assert (
        'system_share, "config", "system_checker.yaml"'
        in source
    )
    assert '"camrod_system_checker_param_file",' in source
    assert "default_value=system_checker_config" in source
    assert "default_value=system_checker_sim_config" not in source
    assert (
        '"system_checker_param_file": LaunchConfiguration('
        in source
    )
    assert (
        '"camrod_system_checker_param_file"\n'
        '                )'
        in source
    )
    assert '"planning_nav2_bt_xml_nav_to_pose"' in source
    assert '"navigate_to_pose_w_planner_selector.xml"' in source
    assert '"planning_nav2_bt_xml_nav_through_poses"' in source
    assert '"navigate_through_poses_w_replanning_and_recovery.xml"' in source
    assert '"enable_api_ui": LaunchConfiguration("enable_api_ui")' in source
    assert (
        '"operator_telemetry_camera_raw_fallback_enabled": ('
        in source
    )
    assert (
        '"CAMROD_CARLA_UI_TF_LATEST_FALLBACK_TOLERANCE_S", "0.0"'
        in source
    )
    assert (
        '"operator_telemetry_tf_latest_fallback_tolerance_s": ('
        in source
    )
    assert '"operator_telemetry_tf_transform_enabled": (' in source
    assert (
        '"operator_telemetry_docking_rear_camera_fallback_enabled": ('
        in source
    )
    assert '"return_site_exit_rearm_enabled": LaunchConfiguration(' in source
    assert 'controller_share, "full_stack.launch.py"' in source
    assert 'controller_share, "launch", "full_stack.launch.py"' not in source
    assert '"nav2_bt_xml_nav_to_pose": tuned_nav_to_pose' in tuned
    assert '"nav2_bt_xml_nav_through_poses": tuned_nav_through_poses' in tuned
    assert '"operator_telemetry_camera_raw_fallback_enabled": "false"' in tuned
    assert '"operator_telemetry_tf_latest_fallback_tolerance_s": "0.075"' in tuned
    assert '"operator_telemetry_tf_transform_enabled": "true"' in tuned
    assert (
        '"operator_telemetry_docking_rear_camera_fallback_enabled": "true"'
        in tuned
    )
    assert '"return_site_exit_rearm_enabled": "true"' in tuned
    assert '"use_sim_planning_profile": "true"' in tuned
    assert '"use_sim_localization_profile": "true"' in tuned
    assert '"use_sim_parking_method": "true"' in tuned
    assert '"recovery_breakaway_enable": "true"' in tuned


def test_recovery_breakaway_authority_is_explicit_opt_in_only():
    source = FULL_LAUNCH.read_text(encoding="utf-8")
    tuned = TUNED_LAUNCH.read_text(encoding="utf-8")
    site_geometry = _load_module(DEVELOP_SITE_GEOMETRY_LAUNCH)
    adapter_launch = (
        PACKAGE_ROOT / "launch" / "adapter.launch.py"
    ).read_text(encoding="utf-8")
    command_config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "command_adapter.yaml").read_text(
            encoding="utf-8"
        )
    )["/**"]["ros__parameters"]

    assert command_config["recovery_breakaway_enable"] is False
    assert '"recovery_breakaway_enable",\n            default_value="false"' in source
    assert '"recovery_breakaway_enable",\n            default_value="false"' in adapter_launch
    assert '"recovery_breakaway_enable": LaunchConfiguration(' in source
    assert '"recovery_breakaway_enable": "true"' in tuned
    assert (
        site_geometry.DEVELOP_SITE_GEOMETRY_ARGUMENTS[
            "recovery_breakaway_enable"
        ]
        == "true"
    )


def test_site_recovery_opt_ins_do_not_change_full_launch_defaults():
    source = FULL_LAUNCH.read_text(encoding="utf-8")
    site_geometry = _load_module(DEVELOP_SITE_GEOMETRY_LAUNCH)
    expected = {
        "carla_route_safety_path_relative_recovery_enable",
        "carla_route_safety_zero_hold_pauses_limits",
        "carla_route_safety_allow_corrective_yaw_beyond_limit",
        "carla_goal_reissue_while_nav_active",
    }

    assert {
        key
        for key, value in site_geometry.DEVELOP_SITE_GEOMETRY_ARGUMENTS.items()
        if key in expected and value == "true"
    } == expected
    for key in expected:
        declaration = source.index(f'"{key}"')
        assert 'default_value="false"' in source[declaration:declaration + 180]

    reentry_key = "carla_route_safety_path_center_reentry_m"
    assert site_geometry.DEVELOP_SITE_GEOMETRY_ARGUMENTS[reentry_key] == "0.15"
    declaration = source.index(f'"{reentry_key}"')
    assert 'default_value="0.08"' in source[declaration:declaration + 260]
    assert (
        '"control_cmd_vel_gate_route_safety_path_center_reentry_m"'
        in source
    )
    assert 'LaunchConfiguration(\n                        "' + reentry_key in source


def test_full_launch_defaults_carla_route_heading_to_production_profile():
    full_launch = (
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
    ).read_text(encoding="utf-8")
    production_defaults = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]

    assert '"CAMROD_CARLA_ROUTE_HEADING_ERROR_ENTER_DEG", "75.0"' in full_launch
    assert '"CAMROD_CARLA_ROUTE_HEADING_LOOKAHEAD_M", "2.0"' in full_launch
    assert (
        '"control_cmd_vel_gate_route_heading_lookahead_m": ('
        in full_launch
    )
    assert (
        '"control_cmd_vel_gate_route_heading_error_enter_deg": ('
        in full_launch
    )
    assert (
        production_defaults["control"][
            "cmd_vel_gate_route_heading_error_enter_deg"
        ]
        == 75.0
    )
    assert (
        production_defaults["control"][
            "cmd_vel_gate_route_heading_error_exit_deg"
        ]
        == 5.0
    )
    assert (
        production_defaults["control"][
            "cmd_vel_gate_route_heading_lookahead_m"
        ]
        == 2.0
    )


def test_reverse_roadside_return_is_profile_scoped_and_not_a_full_default():
    """Develop-parity stays forward-only; CARLA site wrappers opt in."""
    full_launch = FULL_LAUNCH.read_text(encoding="utf-8")
    tuned_launch = TUNED_LAUNCH.read_text(encoding="utf-8")
    site = _load_module(DEVELOP_SITE_GEOMETRY_LAUNCH)
    production_defaults = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]

    assert (
        '"CAMROD_CARLA_ROADSIDE_REVERSE_RETURN_ENABLE", "false"'
        in full_launch
    )
    assert (
        '"control_camping_site_roadside_reverse_return_enable": ('
        in full_launch
    )
    assert (
        'LaunchConfiguration(\n'
        '                        "carla_roadside_reverse_return_enable"\n'
        "                    )"
        in full_launch
    )
    assert (
        '"CAMROD_CARLA_ROADSIDE_REVERSE_HANDOFF_DISTANCE_M", "0.03"'
        in full_launch
    )
    assert (
        '"control_camping_site_roadside_reverse_handoff_distance_m": ('
        in full_launch
    )
    assert (
        'LaunchConfiguration(\n'
        '                        "carla_roadside_reverse_handoff_distance_m"\n'
        "                    )"
        in full_launch
    )
    assert "Exit a CARLA roadside campsite, preserve its outbound yaw" in full_launch
    assert '"planning_nav2_runtime_override_param_file": (' in full_launch
    assert (
        '"control_cmd_vel_gate_lanelet_safety_check_reverse": ('
        in full_launch
    )
    assert '"carla_roadside_reverse_return_enable": "true"' in tuned_launch
    assert '"carla_roadside_reverse_handoff_distance_m": "0.10"' in tuned_launch
    assert '"carla_lanelet_safety_check_reverse": "true"' in tuned_launch
    assert site.DEVELOP_SITE_GEOMETRY_ARGUMENTS[
        "carla_roadside_reverse_return_enable"
    ] == "true"
    assert site.DEVELOP_SITE_GEOMETRY_ARGUMENTS[
        "carla_roadside_reverse_handoff_distance_m"
    ] == "0.03"
    assert site.DEVELOP_SITE_GEOMETRY_ARGUMENTS[
        "carla_nav2_reverse_controller"
    ] == "RPPReverse"
    assert site.DEVELOP_SITE_GEOMETRY_ARGUMENTS[
        "carla_reverse_goal_topic"
    ] == "/planning/auto_reverse_goal_raw"
    assert site.DEVELOP_SITE_GEOMETRY_ARGUMENTS[
        "carla_lanelet_safety_check_reverse"
    ] == "true"
    assert (
        production_defaults["control"][
            "camping_site_roadside_reverse_return_enable"
        ]
        is False
    )
    assert (
        production_defaults["control"][
            "camping_site_roadside_reverse_handoff_distance_m"
        ]
        == 0.03
    )


def test_campsite_tuning_is_typed_forwarded_and_isolated_from_full_defaults():
    """The full profile keeps develop values and the tuned wrapper opts in."""
    full_launch = FULL_LAUNCH.read_text(encoding="utf-8")
    tuned_launch = TUNED_LAUNCH.read_text(encoding="utf-8")
    bringup_launch = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    maneuvers_launch = (
        REPO_ROOT / "camrod_control" / "launch" / "maneuvers.launch.py"
    ).read_text(encoding="utf-8")
    compact_full_launch = "".join(full_launch.split())
    compact_maneuvers_launch = "".join(maneuvers_launch.split())

    shared_parameters = (
        "crab_approach_slowdown_distance_m",
        "crab_approach_min_speed_mps",
        "rotate_180_timeout_s",
    )
    for parameter in shared_parameters:
        assert (
            f'DeclareLaunchArgument("{parameter}",default_value="0")'
            in compact_maneuvers_launch
        )
        assert f'"{parameter}": ParameterValue(' in maneuvers_launch
        assert (
            f'LaunchConfiguration("{parameter}")'
            in compact_maneuvers_launch
        )

    assert maneuvers_launch.count("value_type=float") >= 12

    bringup_names = (
        "control_camping_site_crab_approach_slowdown_distance_m",
        "control_camping_site_crab_approach_min_speed_mps",
        "control_camping_site_rotate_180_timeout_s",
    )
    for parameter, bringup_name in zip(shared_parameters, bringup_names):
        assert f"'{bringup_name}'," in bringup_launch
        assert f"'control/camping_site_{parameter}'," in bringup_launch
        assert (
            f"'{parameter}': lc[\n"
            f"            '{bringup_name}'\n"
            "        ]"
        ) in bringup_launch
        assert f'"{bringup_name}": (' in full_launch

    assert '"CAMROD_CARLA_CRAB_APPROACH_SLOWDOWN_DISTANCE_M", "0.0"' in full_launch
    assert '"CAMROD_CARLA_CRAB_APPROACH_MIN_SPEED_MPS", "0.0"' in full_launch
    assert '"CAMROD_CARLA_ROTATE_180_TIMEOUT_S", "0.0"' in full_launch
    assert '"carla_crab_approach_slowdown_distance_m"' in full_launch
    assert '"carla_crab_approach_min_speed_mps"' in full_launch
    assert '"carla_rotate_180_timeout_s"' in full_launch
    assert '"carla_crab_approach_slowdown_distance_m": "1.0"' in tuned_launch
    assert '"carla_crab_approach_min_speed_mps": "0.12"' in tuned_launch
    assert '"carla_rotate_180_timeout_s": "60.0"' in tuned_launch

    geometry_overrides = (
        (
            "camping_site_entry_position_tolerance_m",
            "entry_position_tolerance_m",
            "",
            "CAMROD_CARLA_ENTRY_POSITION_TOLERANCE_M",
            "0.15",
            "0.05",
            "carla_entry_position_tolerance_m",
        ),
        (
            "camping_site_rotate_entry_max_position_error_m",
            "rotate_entry_max_position_error_m",
            "0",
            "CAMROD_CARLA_ROTATE_ENTRY_MAX_POSITION_ERROR_M",
            "0.0",
            "0.05",
            "carla_rotate_entry_max_position_error_m",
        ),
        (
            "camping_site_rotate_entry_centering_max_initial_error_m",
            "rotate_entry_centering_max_initial_error_m",
            "0.30",
            "CAMROD_CARLA_ROTATE_ENTRY_CENTERING_MAX_INITIAL_ERROR_M",
            "0.30",
            "0.50",
            "carla_rotate_entry_centering_max_initial_error_m",
        ),
    )
    for (
        launch_name,
        node_parameter,
        shared_default,
        environment_name,
        parity_default,
        carla_default,
        carla_launch_name,
    ) in geometry_overrides:
        bringup_name = f"control_{launch_name}"
        assert (
            f'DeclareLaunchArgument("{launch_name}",'
            f'default_value="{shared_default}"'
        ) in compact_maneuvers_launch
        if node_parameter == "entry_position_tolerance_m":
            assert (
                'overrides["entry_position_tolerance_m"] = ParameterValue('
                in maneuvers_launch
            )
        else:
            assert f'"{node_parameter}": ParameterValue(' in maneuvers_launch
        assert (
            f'LaunchConfiguration("{launch_name}")'
            in compact_maneuvers_launch
        )
        assert f"'{bringup_name}'," in bringup_launch
        assert f"'control/{launch_name}'," in bringup_launch
        assert (
            f"'{launch_name}': lc[\n"
            f"            '{bringup_name}'\n"
            "        ]"
        ) in bringup_launch
        assert f'"{environment_name}","{parity_default}"' in compact_full_launch
        assert f'"{bringup_name}": (' in full_launch
        assert f'"{carla_launch_name}"' in full_launch
        assert f'"{carla_launch_name}":"{carla_default}"' in "".join(
            tuned_launch.split()
        )

    maneuvers_module = _load_module(
        REPO_ROOT / "camrod_control" / "launch" / "maneuvers.launch.py"
    )
    context = LaunchContext()
    context.launch_configurations[
        "camping_site_entry_position_tolerance_m"
    ] = ""
    context.launch_configurations["return_lateral_hysteresis_m"] = ""
    context.launch_configurations[
        "camping_site_max_angular_speed_radps"
    ] = ""
    overrides = maneuvers_module._camping_site_parameter_overrides(context)
    assert "entry_position_tolerance_m" not in overrides
    assert "return_lateral_hysteresis_m" not in overrides
    assert "max_angular_speed_radps" not in overrides

    context.launch_configurations[
        "camping_site_entry_position_tolerance_m"
    ] = "0.05"
    overrides = maneuvers_module._camping_site_parameter_overrides(context)
    assert "entry_position_tolerance_m" in overrides

    context.launch_configurations["return_lateral_hysteresis_m"] = "0.13"
    overrides = maneuvers_module._camping_site_parameter_overrides(context)
    assert "return_lateral_hysteresis_m" in overrides

    context.launch_configurations[
        "camping_site_max_angular_speed_radps"
    ] = "0.45"
    overrides = maneuvers_module._camping_site_parameter_overrides(context)
    assert "max_angular_speed_radps" in overrides

    assert (
        'DeclareLaunchArgument("return_lateral_hysteresis_m",default_value="")'
        in compact_maneuvers_launch
    )
    assert "'control_camping_site_return_lateral_hysteresis_m'," in bringup_launch
    assert (
        "'return_lateral_hysteresis_m': lc[\n"
        "            'control_camping_site_return_lateral_hysteresis_m'\n"
        "        ]"
    ) in bringup_launch
    assert '"control_camping_site_return_lateral_hysteresis_m": (' in full_launch
    assert (
        'DeclareLaunchArgument('
        '"camping_site_max_angular_speed_radps",default_value="")'
        in compact_maneuvers_launch
    )
    assert "'control_camping_site_max_angular_speed_radps'," in bringup_launch
    assert (
        "'camping_site_max_angular_speed_radps': lc[\n"
        "            'control_camping_site_max_angular_speed_radps'\n"
        "        ]"
    ) in bringup_launch
    assert '"control_camping_site_max_angular_speed_radps": (' in full_launch

    bringup_entry_default = bringup_launch.split(
        "'control_camping_site_entry_position_tolerance_m',", 1
    )[1][:300]
    assert "'control/camping_site_entry_position_tolerance_m'," in (
        bringup_entry_default
    )
    assert "                ''," in bringup_entry_default

    anchor_centering_overrides = (
        (
            "entry_anchor_centering_max_initial_error_m",
            "0",
            "CAMROD_CARLA_ENTRY_ANCHOR_CENTERING_MAX_INITIAL_ERROR_M",
            "0.0",
            "0.0",
        ),
        (
            "entry_anchor_centering_max_speed_mps",
            "0.12",
            "CAMROD_CARLA_ENTRY_ANCHOR_CENTERING_MAX_SPEED_MPS",
            "0.12",
            "0.12",
        ),
        (
            "entry_anchor_centering_timeout_s",
            "15",
            "CAMROD_CARLA_ENTRY_ANCHOR_CENTERING_TIMEOUT_S",
            "15",
            "15",
        ),
        (
            "entry_anchor_centering_tolerance_m",
            "0.05",
            "CAMROD_CARLA_ENTRY_ANCHOR_CENTERING_TOLERANCE_M",
            "0.05",
            "0.05",
        ),
        (
            "crab_entry_max_heading_drift_deg",
            "0",
            "CAMROD_CARLA_CRAB_ENTRY_MAX_HEADING_DRIFT_DEG",
            "0.0",
            "5.0",
        ),
        (
            "crab_entry_max_cross_track_error_m",
            "0",
            "CAMROD_CARLA_CRAB_ENTRY_MAX_CROSS_TRACK_ERROR_M",
            "0.0",
            "0.10",
        ),
        (
            "crab_entry_body_yaw_compensation_deg",
            "0",
            "CAMROD_CARLA_CRAB_ENTRY_BODY_YAW_COMPENSATION_DEG",
            "0.0",
            "2.0",
        ),
        (
            "crab_entry_body_yaw_alignment_tolerance_deg",
            "0.5",
            "CAMROD_CARLA_CRAB_ENTRY_BODY_YAW_ALIGNMENT_TOLERANCE_DEG",
            "0.5",
            "0.5",
        ),
        (
            "crab_entry_body_yaw_alignment_timeout_s",
            "15",
            "CAMROD_CARLA_CRAB_ENTRY_BODY_YAW_ALIGNMENT_TIMEOUT_S",
            "15",
            "15",
        ),
    )
    compact_tuned_launch = "".join(tuned_launch.split())
    for parameter, shared_default, environment_name, parity_default, tuned_default in (
        anchor_centering_overrides
    ):
        bringup_name = f"control_camping_site_{parameter}"
        carla_launch_name = f"carla_{parameter}"
        assert (
            f'DeclareLaunchArgument("{parameter}",'
            f'default_value="{shared_default}"'
        ) in compact_maneuvers_launch
        assert f'"{parameter}": ParameterValue(' in maneuvers_launch
        assert (
            f'LaunchConfiguration("{parameter}")'
            in compact_maneuvers_launch
        )
        assert f"'{bringup_name}'," in bringup_launch
        assert f"'control/camping_site_{parameter}'," in bringup_launch
        assert (
            f"'{parameter}': lc[\n"
            f"            '{bringup_name}'\n"
            "        ]"
        ) in bringup_launch
        assert f'"{environment_name}","{parity_default}"' in compact_full_launch
        assert f'"{bringup_name}": (' in full_launch
        assert f'"{carla_launch_name}"' in full_launch
        assert f'"{carla_launch_name}":"{tuned_default}"' in compact_tuned_launch


def test_carla_reverse_return_overlay_is_slow_and_reverse_capable():
    """The reverse overlay is reachable through the scoped CARLA wrappers."""
    full_launch = FULL_LAUNCH.read_text(encoding="utf-8")
    tuned_launch = TUNED_LAUNCH.read_text(encoding="utf-8")
    site = _load_module(DEVELOP_SITE_GEOMETRY_LAUNCH)
    overlay_path = PACKAGE_ROOT / "config" / "nav2_carla_reverse_return.yaml"
    overlay = yaml.safe_load(overlay_path.read_text(encoding="utf-8"))
    controller = overlay["controller_server"]["ros__parameters"]
    reverse_rpp = controller["RPPReverse"]

    assert controller["controller_plugins"] == ["RPP", "RPPReverse", "RotationShim"]
    assert controller["RPP"] == {
        "desired_linear_vel": 0.555556,
        "min_approach_linear_velocity": 0.138889,
        "regulated_linear_scaling_min_speed": 0.166667,
    }
    assert reverse_rpp == {
        "plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
        "allow_reversing": True,
        "use_rotate_to_heading": False,
        "desired_linear_vel": 0.20,
        "max_angular_accel": 0.8,
        "use_velocity_scaled_lookahead_dist": False,
        "lookahead_dist": 0.80,
        "min_lookahead_dist": 0.80,
        "max_lookahead_dist": 0.80,
        "lookahead_time": 1.8,
        "rotate_to_heading_angular_vel": 0.35,
        "transform_tolerance": 0.5,
        "use_interpolation": True,
        "min_approach_linear_velocity": 0.10,
        "approach_velocity_scaling_dist": 0.6,
        "use_collision_detection": True,
        "max_allowed_time_to_collision_up_to_carrot": 1.0,
        "use_regulated_linear_velocity_scaling": True,
        "regulated_linear_scaling_min_radius": 3.0,
        "regulated_linear_scaling_min_speed": 0.20,
        "use_cost_regulated_linear_velocity_scaling": True,
        "cost_scaling_dist": 0.6,
        "cost_scaling_gain": 1.0,
    }

    production_vehicle = yaml.safe_load(
        (REPO_ROOT / "camrod_planning" / "config" / "nav2_vehicle.yaml").read_text(
            encoding="utf-8"
        )
    )
    production_rpp = production_vehicle["controller_server"]["ros__parameters"]["RPP"]
    assert production_rpp["allow_reversing"] is False
    assert production_rpp["use_rotate_to_heading"] is False
    assert production_rpp["desired_linear_vel"] == 1.111111
    assert production_rpp["use_velocity_scaled_lookahead_dist"] is False
    assert production_rpp["lookahead_dist"] == 1.2
    assert production_rpp["min_lookahead_dist"] == 1.1
    assert production_rpp["max_lookahead_dist"] == 2.0
    assert production_rpp["regulated_linear_scaling_min_speed"] == 0.333333

    production_gate_scale = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]["control"]["cmd_vel_gate_speed_scale"]
    assert production_gate_scale == 0.5
    assert (
        '"CAMROD_CARLA_CMD_VEL_GATE_SPEED_SCALE", "0.5"'
        in full_launch
    )
    assert (
        '"control_cmd_vel_gate_speed_scale": LaunchConfiguration('
        in full_launch
    )
    assert (
        reverse_rpp["regulated_linear_scaling_min_speed"] * 1.0 == 0.20
    )

    assert '"carla_nav2_reverse_controller", default_value=""' in full_launch
    assert '"carla_reverse_goal_topic", default_value=""' in full_launch
    assert 'default_value=nav2_runtime_disabled_config' in full_launch
    assert '"carla_cmd_vel_gate_speed_scale": "1.0"' in tuned_launch
    assert '"carla_nav2_reverse_controller": "RPPReverse"' in tuned_launch
    assert (
        '"carla_reverse_goal_topic": "/planning/auto_reverse_goal_raw"'
        in tuned_launch
    )
    assert '"carla_nav2_reverse_return_param_file": tuned_nav2' in tuned_launch
    assert site.develop_site_geometry_arguments("/adapter-share")[
        "carla_nav2_reverse_return_param_file"
    ] == "/adapter-share/config/nav2_carla_reverse_return.yaml"


def test_carla_runtime_overlay_prescales_forward_rpp_and_isolates_reverse():
    """Unity gate keeps torque while forward RPP matches production speed."""
    production_vehicle = yaml.safe_load(
        (REPO_ROOT / "camrod_planning" / "config" / "nav2_vehicle.yaml").read_text(
            encoding="utf-8"
        )
    )
    production_controller_profile = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_planning"
            / "config"
            / "nav2_controller_profiles"
            / "production.yaml"
        ).read_text(encoding="utf-8")
    )
    reverse_overlay = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "nav2_carla_reverse_return.yaml").read_text(
            encoding="utf-8"
        )
    )

    merged = _deep_merge(
        _deep_merge(production_vehicle, production_controller_profile),
        reverse_overlay,
    )
    controller = merged["controller_server"]["ros__parameters"]

    assert controller["controller_plugins"] == ["RPP", "RPPReverse", "RotationShim"]
    assert controller["RPP"]["desired_linear_vel"] == 0.555556
    assert controller["RPP"]["min_approach_linear_velocity"] == 0.138889
    assert controller["RPP"]["regulated_linear_scaling_min_speed"] == 0.166667
    assert controller["RPP"]["allow_reversing"] is False
    assert controller["RotationShim"]["desired_linear_vel"] == 0.555556
    assert controller["RotationShim"]["min_approach_linear_velocity"] == 0.138889
    assert (
        controller["RotationShim"]["regulated_linear_scaling_min_speed"]
        == 0.166667
    )
    assert controller["RPPReverse"]["desired_linear_vel"] == 0.20
    assert controller["RPPReverse"]["allow_reversing"] is True


def test_carla_navigation_tree_excludes_generic_spin_recovery():
    """A full-body in-lane spin is impossible in narrow Woraksan lanes."""
    tree = (
        PACKAGE_ROOT / "config" / "navigate_to_pose_carla.xml"
    ).read_text(encoding="utf-8")
    production_tree = (
        REPO_ROOT
        / "camrod_planning"
        / "config"
        / "bt"
        / "navigate_to_pose_w_planner_selector.xml"
    ).read_text(encoding="utf-8")

    assert "<Spin " not in tree
    assert "<BackUp " in tree
    assert "<FollowPath " in tree
    assert "<Spin " in production_tree


def test_reverse_goal_source_is_routed_before_the_return_goal():
    """Return goals alone enter the regulated_reverse selector path."""
    state_machine = (
        REPO_ROOT / "camrod_planning" / "scripts" / "planning_state_machine_node.py"
    ).read_text(encoding="utf-8")
    snapper = (
        REPO_ROOT / "camrod_planning" / "src" / "goal_snapper_node.cpp"
    ).read_text(encoding="utf-8")
    selector = (
        REPO_ROOT / "camrod_planning" / "scripts" / "nav2_selector_latch_node.py"
    ).read_text(encoding="utf-8")

    assert 'key_name == self.return_mission_key and reverse_publisher is not None' in state_machine
    assert 'snapAvgGoal(*msg, "reverse_aux_avg", "regulated_reverse")' in snapper
    assert 'source.data = "regulated_reverse"' in snapper
    assert 'normalized.startswith("regulated_reverse")' in selector
    assert '"reverse_controller_id"' in selector


def test_nav2_radius_stabilizer_is_tuned_only_and_above_adapter_boundary():
    """Develop parity leaves the stabilizer off; tuned mode opts into 0.82 m."""
    full_launch = FULL_LAUNCH.read_text(encoding="utf-8")
    tuned_launch = TUNED_LAUNCH.read_text(encoding="utf-8")
    adapter = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "command_adapter.yaml").read_text(
            encoding="utf-8"
        )
    )["/**"]["ros__parameters"]
    production_defaults = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]["control"]
    package_gate = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_control"
            / "config"
            / "cmd_vel_safety_gate.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    bringup_gate = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "control"
            / "cmd_vel_safety_gate.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]

    carla_radius = 0.82
    assert carla_radius > adapter["minimum_turn_radius_m"]
    assert (
        '"CAMROD_CARLA_NAVIGATION_MINIMUM_ACKERMANN_TURN_RADIUS_M",\n'
        '                "0.0",'
    ) in full_launch
    assert (
        '"carla_navigation_minimum_ackermann_turn_radius_m": "0.82"'
        in tuned_launch
    )
    assert (
        '"control_cmd_vel_gate_navigation_minimum_ackermann_turn_radius_m": ('
        in full_launch
    )
    assert (
        production_defaults[
            "cmd_vel_gate_navigation_minimum_ackermann_turn_radius_m"
        ]
        == 0.0
    )
    assert package_gate["navigation_minimum_ackermann_turn_radius_m"] == 0.0
    assert bringup_gate["navigation_minimum_ackermann_turn_radius_m"] == 0.0


def test_parking_and_charger_emulation_are_opt_in_carla_profiles_only():
    """Full/develop parity stays off; tuned and site wrappers opt in."""
    full_launch = FULL_LAUNCH.read_text(encoding="utf-8")
    tuned_launch = TUNED_LAUNCH.read_text(encoding="utf-8")
    site_module = _load_module(DEVELOP_SITE_GEOMETRY_LAUNCH)
    overlay = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "parking_carla.yaml").read_text(
            encoding="utf-8"
        )
    )["/parking/reverse_parking_controller"]["ros__parameters"]
    production = yaml.safe_load(
        (REPO_ROOT / "camrod_control" / "config" / "parking.yaml").read_text(
            encoding="utf-8"
        )
    )["/parking/reverse_parking_controller"]["ros__parameters"]

    assert overlay == {"maximum_reverse_distance_m": 4.2}
    assert production["maximum_reverse_distance_m"] == 1.5
    assert production["complete_without_charging"] is False
    assert '"launch_charging_contact_emulator", default_value="false"' in full_launch
    assert 'default_value=parking_runtime_disabled_config' in full_launch
    assert '"launch_charging_contact_emulator": "true"' in tuned_launch
    assert (
        site_module.DEVELOP_SITE_GEOMETRY_ARGUMENTS[
            "launch_charging_contact_emulator"
        ]
        == "true"
    )
    assert site_module.DEVELOP_SITE_GEOMETRY_ARGUMENTS[
        "carla_charging_contact_parking_status_topic"
    ] == "/parking/apriltag_parking_controller/status"
    assert '"carla_parking_runtime_override_param_file": tuned_parking' in tuned_launch
    assert '"parking_runtime_override_param_file": LaunchConfiguration(' in full_launch
    assert '"carla_charging_contact_position_tolerance_m"' in full_launch
    assert '"carla_charging_contact_speed_tolerance_mps"' in full_launch
    assert '"carla_charging_contact_state_timeout_s"' in full_launch
    assert '"carla_charging_contact_parking_status_topic"' in full_launch
    assert 'default_value="/parking/reverse_parking_controller/status"' in full_launch
    assert '"state_timeout_s": LaunchConfiguration(' in full_launch
    assert '"parking_status_topic": LaunchConfiguration(' in full_launch


def test_charging_contact_rate_isolated_from_heartbeat_legacy_alias():
    """An empty sibling ``publish_rate_hz`` must not disable contact launch."""
    charging_launch = (
        PACKAGE_ROOT / "launch" / "charging_contact_emulator.launch.py"
    ).read_text(encoding="utf-8")

    assert '"charging_contact_publish_rate_hz"' in charging_launch
    assert 'default_value="/planning/state_machine/state"' in charging_launch
    assert 'DeclareLaunchArgument("state_timeout_s", default_value="2.0")' in charging_launch
    assert 'DeclareLaunchArgument("publish_rate_hz"' not in charging_launch
    assert (
        'LaunchConfiguration(\n'
        '                        "charging_contact_publish_rate_hz"\n'
        '                    )'
        in charging_launch
    )


def test_reverse_runtime_overlay_is_forwarded_last_without_production_mutation():
    """Bringup forwards the sparse overlay through planning into Nav2's final chain."""
    bringup_source = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    planning_source = (
        REPO_ROOT / "camrod_planning" / "launch" / "planning.launch.py"
    ).read_text(encoding="utf-8")
    state_machine_source = (
        REPO_ROOT / "camrod_planning" / "launch" / "state_machine.launch.py"
    ).read_text(encoding="utf-8")
    launch_defaults = yaml.safe_load((
        REPO_ROOT
        / "camrod_bringup"
        / "config"
        / "bringup"
        / "launch_defaults.yaml"
    ).read_text(encoding="utf-8"))["bringup"]
    nav2_source = (
        REPO_ROOT / "camrod_planning" / "launch" / "nav2_lanelet.launch.py"
    ).read_text(encoding="utf-8")

    assert "'planning_nav2_runtime_override_param_file'," in bringup_source
    assert "'nav2_runtime_override_param_file': lc[" in bringup_source
    assert "'planning_nav2_reverse_controller'," in bringup_source
    assert "'nav2_reverse_controller': lc['planning_nav2_reverse_controller']" in bringup_source
    assert "'goal_snapper_reverse_auxiliary_input_goal_topic': lc[" in bringup_source
    assert "'planning_state_machine_reverse_auto_goal_snapper_input_topic': lc[" in bringup_source
    assert "'planning_state_machine_return_goal_reached_distance_m': lc[" in bringup_source
    assert "'nav2_runtime_override_param_file'," in planning_source
    assert "'nav2_reverse_controller'," in planning_source
    assert "'goal_snapper_reverse_auxiliary_input_goal_topic'," in planning_source
    assert "'planning_state_machine_reverse_auto_goal_snapper_input_topic'," in planning_source
    assert "'planning_state_machine_return_goal_reached_distance_m'," in planning_source
    assert (
        launch_defaults["planning"][
            "state_machine_return_goal_reached_distance_m"
        ]
        == ""
    )
    assert "default_value=''" in state_machine_source
    assert "if return_goal_reached_distance:" in state_machine_source
    assert (
        "runtime_parameters['return_goal_reached_distance_m']"
        in state_machine_source
    )
    assert (
        "'return_goal_reached_distance_m': LaunchConfiguration("
        not in state_machine_source
    )
    assert "nav2_runtime_override_params = RewrittenYaml(" in nav2_source
    chain_start = nav2_source.index("nav2_param_chain = [")
    runtime_index = nav2_source.index("nav2_runtime_override_params,", chain_start)
    immutable_index = nav2_source.index("force_base_link_overrides,", chain_start)
    assert runtime_index < immutable_index


def test_return_handoff_override_is_optional_and_explicit_030_is_preserved():
    module = _load_module(STATE_MACHINE_LAUNCH)
    module.Node = lambda **kwargs: kwargs

    def runtime_parameters(value: str):
        context = LaunchContext()
        context.launch_configurations[
            "planning_state_machine_return_goal_reached_distance_m"
        ] = value
        return module._state_machine_node(context)[0]["parameters"][1]

    assert "return_goal_reached_distance_m" not in runtime_parameters("")
    assert "return_goal_reached_distance_m" in runtime_parameters("0.30")
    assert "return_goal_reached_distance_m" in runtime_parameters("0.35")


def test_pose_jump_source_is_empty_by_default_and_tuned_to_raw_localization():
    """The full profile preserves develop; tuned mode opts into the raw pose."""
    full_launch = FULL_LAUNCH.read_text(encoding="utf-8")
    tuned_launch = TUNED_LAUNCH.read_text(encoding="utf-8")
    bringup_source = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    planning_source = (
        REPO_ROOT / "camrod_planning" / "launch" / "planning.launch.py"
    ).read_text(encoding="utf-8")
    lanelet_tools_source = (
        REPO_ROOT / "camrod_planning" / "launch" / "lanelet_tools.launch.py"
    ).read_text(encoding="utf-8")
    package_defaults = yaml.safe_load(
        (
            REPO_ROOT / "camrod_planning" / "config" / "goal_snapper.yaml"
        ).read_text(encoding="utf-8")
    )["/planning/goal_snapper"]["ros__parameters"]
    bringup_defaults = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "planning"
            / "goal_snapper.yaml"
        ).read_text(encoding="utf-8")
    )["/planning/goal_snapper"]["ros__parameters"]

    assert (
        '"carla_goal_snapper_pose_jump_check_topic", default_value=""'
        in full_launch
    )
    assert (
        '"planning_goal_snapper_pose_jump_check_topic": (' in full_launch
    )
    assert (
        '"carla_goal_snapper_pose_jump_check_topic": "/localization/pose"'
        in tuned_launch
    )
    site_launch = DEVELOP_SITE_GEOMETRY_LAUNCH.read_text(encoding="utf-8")
    assert (
        '"carla_goal_snapper_pose_jump_check_topic": "/localization/pose"'
        in site_launch
    )
    assert "'planning_goal_snapper_pose_jump_check_topic'," in bringup_source
    assert "'goal_snapper_pose_jump_check_topic': lc[" in bringup_source
    assert "'goal_snapper_pose_jump_check_topic'," in planning_source
    assert "'goal_snapper_pose_jump_check_topic'," in lanelet_tools_source
    assert (
        "'pose_jump_check_topic': LaunchConfiguration("
        in lanelet_tools_source
    )
    assert package_defaults["current_pose_topic"] == "/planning/lanelet_pose"
    assert package_defaults["pose_jump_check_topic"] == ""
    assert bringup_defaults["current_pose_topic"] == "/planning/lanelet_pose"
    assert bringup_defaults["pose_jump_check_topic"] == ""


def test_fake_sensor_rate_isolated_from_platform_heartbeat_launch_scope():
    """A sibling 5 Hz include must not down-rate 10 Hz readiness topics."""
    full_launch = (
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
    ).read_text(encoding="utf-8")
    heartbeat_launch = (
        PACKAGE_ROOT / "launch" / "platform_heartbeat.launch.py"
    ).read_text(encoding="utf-8")
    fake_launch = (
        REPO_ROOT / "camrod_bringup" / "launch" / "fake_sensors.launch.py"
    ).read_text(encoding="utf-8")
    bringup_impl = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    defaults = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]

    assert '"platform_heartbeat_publish_rate_hz"' in heartbeat_launch
    assert (
        '"platform_heartbeat_publish_rate_hz", default_value="5.0"'
        in full_launch
    )
    assert (
        '"platform_heartbeat_publish_rate_hz": LaunchConfiguration('
        in full_launch
    )
    assert (
        'DeclareLaunchArgument(\n            "publish_rate_hz",'
        in heartbeat_launch
    )
    assert 'default_value="",' in heartbeat_launch
    assert 'LaunchConfiguration("publish_rate_hz")' in heartbeat_launch
    assert (
        '"platform_heartbeat_publish_rate_hz"'
        in heartbeat_launch
    )
    assert '"sim_fake_sensor_publish_rate_hz": "10.0"' in full_launch
    assert "'fake_sensor_publish_rate_hz'" in fake_launch
    assert "'fake_sensor_publish_rate_hz'\n    )" in fake_launch
    assert "LaunchConfiguration('publish_rate_hz')" in fake_launch
    assert "'publish_rate_hz',\n        default_value=''," in fake_launch
    assert "PythonExpression([" in fake_launch
    assert "'sim_fake_sensor_publish_rate_hz'" in bringup_impl
    assert "'fake_sensor_publish_rate_hz': lc[" in bringup_impl
    assert defaults["sim"]["fake_sensor_publish_rate_hz"] == 10.0


def test_rate_alias_defaults_do_not_cross_includes_and_legacy_is_supported():
    heartbeat_module = _load_module(
        PACKAGE_ROOT / "launch" / "platform_heartbeat.launch.py"
    )
    fake_module = _load_module(
        REPO_ROOT / "camrod_bringup" / "launch" / "fake_sensors.launch.py"
    )

    assert _resolve_rate(
        heartbeat_module,
        "platform_heartbeat_publish_rate_hz",
        "5.0",
        "",
    ) == "5.0"
    assert _resolve_rate(
        fake_module,
        "fake_sensor_publish_rate_hz",
        "10.0",
        "",
    ) == "10.0"
    assert _resolve_rate(
        heartbeat_module,
        "platform_heartbeat_publish_rate_hz",
        "5.0",
        "7.5",
    ) == "7.5"
    assert _resolve_rate(
        fake_module,
        "fake_sensor_publish_rate_hz",
        "10.0",
        "12.0",
    ) == "12.0"


def test_subset_launch_forwards_multi_pose_selector_tree():
    source = (
        PACKAGE_ROOT / "launch" / "camrod_carla.launch.py"
    ).read_text(encoding="utf-8")
    assert '"nav2_bt_xml_nav_through_poses"' in source
    assert '"navigate_through_poses_w_planner_selector.xml"' in source
    assert (
        '"nav2_bt_xml_nav_through_poses": LaunchConfiguration('
        in source
    )


def test_full_launch_uses_production_localization_and_tuned_uses_metric_pose():
    """The CARLA metric-pose adapter is an explicit historical opt-in."""
    full = FULL_LAUNCH.read_text(encoding="utf-8")
    tuned = TUNED_LAUNCH.read_text(encoding="utf-8")
    subset = (
        PACKAGE_ROOT / "launch" / "camrod_carla.launch.py"
    ).read_text(encoding="utf-8")
    bringup = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    ordinary_defaults = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]
    carla_params = yaml.safe_load(
        (
            PACKAGE_ROOT
            / "config"
            / "camrod_input_adapter_carla.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]

    assert '"camrod_input_adapter_config"' in subset
    assert '"camrod_input_adapter_carla.yaml"' in subset
    assert '"camrod_input_adapter_config"' in full
    assert 'localization_share, "config", "source", "input_adapter.yaml"' in full
    assert '"camrod_input_adapter_config": tuned_input_adapter' in tuned

    assert (
        '"localization_adapter_param_file": LaunchConfiguration('
        in full
    )
    assert "'localization_adapter_param_file'," in bringup
    assert (
        "localization_args['adapter_param_file'] = lc["
        in bringup
    )
    assert ordinary_defaults["localization"]["adapter_param_file"] == (
        "localization/source/input_adapter.yaml"
    )
    assert carla_params["utm_pose_topic"] == "/camrod_carla/metric_pose"
    assert carla_params["navsat_topic"] == "/camrod_carla/unused_navsat"
    assert carla_params["enable_gnss_heading"] is False


def test_ui_return_terminal_uses_the_same_authored_drop_zone_as_bringup():
    bringup = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    ui_launch = (
        REPO_ROOT / "camrod_ui" / "camrod_ui_robot" / "launch" / "ui.launch.py"
    ).read_text(encoding="utf-8")

    assert "'drop_zones_yaml': lc['planning_state_machine_keypoints_yaml']" in bringup
    assert "drop_zones_yaml_arg = DeclareLaunchArgument(" in ui_launch
    assert "'drop_zones_yaml': LaunchConfiguration('drop_zones_yaml')" in ui_launch
    assert "drop_zones_yaml_arg," in ui_launch


def test_dedicated_manual_twist_is_enabled_only_by_carla_compositions():
    control_launch = (
        REPO_ROOT / "camrod_control" / "launch" / "cmd_vel_safety_gate.launch.py"
    ).read_text(encoding="utf-8")
    bringup = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    ui_launch = (
        REPO_ROOT / "camrod_ui" / "camrod_ui_robot" / "launch" / "ui.launch.py"
    ).read_text(encoding="utf-8")

    assert (
        '"manual_input_topic": LaunchConfiguration("manual_cmd_vel_ros_topic")'
        in control_launch
    )
    assert (
        'DeclareLaunchArgument("manual_cmd_vel_ros_topic", default_value="")'
        in control_launch
    )
    assert (
        "cfg_get(launch_cfg, 'control/manual_cmd_vel_ros_topic', '')"
        in bringup
    )
    assert (
        "'manual_cmd_vel_ros_topic': lc['control_manual_cmd_vel_ros_topic']"
        in bringup
    )
    assert bringup.count(
        "'manual_cmd_vel_ros_topic': lc['control_manual_cmd_vel_ros_topic']"
    ) >= 2
    assert (
        "'manual_cmd_vel_ros_topic': LaunchConfiguration("
        "'manual_cmd_vel_ros_topic')"
    ) in ui_launch

    for filename in (
        "camrod_carla.launch.py",
        "camrod_carla_full.launch.py",
    ):
        source = (PACKAGE_ROOT / "launch" / filename).read_text(
            encoding="utf-8"
        )
        assert '"manual_cmd_vel_ros_topic"' in source
        assert 'default_value="/control/manual_cmd_vel_ros"' in source

    subset = (
        PACKAGE_ROOT / "launch" / "camrod_carla.launch.py"
    ).read_text(encoding="utf-8")
    full = FULL_LAUNCH.read_text(encoding="utf-8")
    tuned = TUNED_LAUNCH.read_text(encoding="utf-8")
    gate_source = (
        REPO_ROOT / "camrod_control" / "src" / "cmd_vel_safety_gate_node.cpp"
    ).read_text(encoding="utf-8")
    manual_departure_policy = (
        REPO_ROOT
        / "camrod_control"
        / "include"
        / "camrod_control"
        / "manual_charging_departure_authorization.hpp"
    ).read_text(encoding="utf-8")
    gate_defaults = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_control"
            / "config"
            / "cmd_vel_safety_gate.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    assert '"manual_cmd_vel_ros_topic": LaunchConfiguration(' in subset
    assert '"control_manual_cmd_vel_ros_topic": LaunchConfiguration(' in full
    assert gate_defaults["allow_manual_departure_while_charging"] is False
    assert gate_defaults["manual_charging_departure_command_timeout_s"] == 0.35
    assert '"carla_allow_manual_departure_while_charging"' in full
    declaration = full.index('"carla_allow_manual_departure_while_charging"')
    assert 'default_value="false"' in full[declaration:declaration + 420]
    assert (
        '"control_cmd_vel_gate_allow_manual_departure_while_charging": ('
        in full
    )
    site = _load_module(DEVELOP_SITE_GEOMETRY_LAUNCH)
    assert site.DEVELOP_SITE_GEOMETRY_ARGUMENTS[
        "carla_allow_manual_departure_while_charging"
    ] == "true"
    for token in (
        "manual_charging_departure_command_timeout_s",
        "command_source_arbiter_.manualSourceActive()",
        "command_source_arbiter_.maneuverActive()",
        "gate_policy_.missionEnabled()",
        'charging_departure_auth=" +',
        '"manual_drive"',
    ):
        assert token in gate_source
    for token in (
        "context.manual_source_active && !context.maneuver_active",
        "context.manual_engaged && !context.mission_engaged",
        "context.platform_drive_enabled && context.charging",
        "context.battery_ready_for_departure",
    ):
        assert token in manual_departure_policy
    for name, parity_default, tuned_default in (
        ("manual_drive_linear_limit_mps", "0.20", "1.40"),
        ("manual_drive_lateral_limit_mps", "0.20", "1.00"),
        ("manual_drive_angular_limit_radps", "0.20", "0.7853"),
        ("manual_drive_deadman_timeout_s", "0.25", "0.75"),
    ):
        assert f'"{name}"' in full
        assert f'"{parity_default}"' in full
        assert f'"control_{name}": LaunchConfiguration(' in full
        assert f'"{name}": "{tuned_default}"' in tuned


def test_full_uses_develop_cost_thresholds_and_tuned_uses_hard_costs():
    full = FULL_LAUNCH.read_text(encoding="utf-8")
    tuned = TUNED_LAUNCH.read_text(encoding="utf-8")
    compact_full = "".join(full.split())

    for declaration in (
        "carla_cost_stop_threshold",
        "carla_lanelet_safety_threshold",
        "carla_lanelet_safety_current_threshold",
    ):
        assert f'"{declaration}", default_value="85"' in full

    for argument, declaration in (
        ("control_cmd_vel_gate_cost_threshold", "carla_cost_stop_threshold"),
        (
            "control_cmd_vel_gate_lanelet_safety_threshold",
            "carla_lanelet_safety_threshold",
        ),
        (
            "control_cmd_vel_gate_lanelet_safety_current_threshold",
            "carla_lanelet_safety_current_threshold",
        ),
    ):
        assert f'"{argument}":' in full
        assert f'LaunchConfiguration("{declaration}")' in compact_full

    assert '"carla_cost_stop_threshold": "100"' in tuned
    assert '"carla_lanelet_safety_threshold": "100"' in tuned
    assert '"carla_lanelet_safety_current_threshold": "100"' in tuned


def test_composition_launches_have_no_host_absolute_paths_and_use_env_gates():
    for filename in (
        "camrod_carla.launch.py",
        "camrod_carla_full.launch.py",
    ):
        source = (PACKAGE_ROOT / "launch" / filename).read_text(
            encoding="utf-8"
        )
        assert "/home/hong" not in source
        assert "/home/nvidia" not in source
        assert "RANGER_CARLA_ROOT" in source
        assert "RANGER_BASELINE_MANIFEST" in source
        assert "RANGER_PHYSICAL_MANIFEST" in source
        assert "RANGER_CARLA_PYTHON_EGG" in source
        assert "RANGER_PYTHON_EGG_CACHE" in source
        assert "ranger_ros_backend_gate_20260814" not in source
        assert "ranger_physical_4ws_acceptance_gate_20260823" not in source


def test_full_and_tuned_launches_use_distinct_lanelet_map_cohorts():
    full = FULL_LAUNCH.read_text(encoding="utf-8")
    tuned = TUNED_LAUNCH.read_text(encoding="utf-8")
    subset = (
        PACKAGE_ROOT / "launch" / "camrod_carla.launch.py"
    ).read_text(encoding="utf-8")

    for source in (subset, full):
        assert '"camrod_map_path", default_value=camrod_map_path' in source
        assert '"map_path": LaunchConfiguration("camrod_map_path")' in source

    assert '"lanelet2_maps.osm"' in full
    assert '"CAMROD_LANELET_MAP", default=develop_lanelet_map' in full
    assert '"woraksan_carla_lanelet2.osm"' not in full
    assert '"camrod_map_path": tuned_lanelet_map' in tuned
    assert '"config", "woraksan_carla_lanelet2.osm"' in tuned


def test_sensor_relay_uses_camrod_canonical_topics():
    source = (
        PACKAGE_ROOT
        / "src"
        / "camrod_carla_adapter"
        / "sensor_relay_node.py"
    ).read_text(encoding="utf-8")
    for topic in (
        "/sensing/camera/econ_front/image_raw",
        "/sensing/camera/econ_front/image_rect",
        "/sensing/camera/econ_front/camera_info",
        "/sensing/camera/econ_rear/image_raw",
        "/sensing/camera/econ_rear/image_rect",
        "/sensing/camera/econ_rear/camera_info",
        "/sensing/lidar/vanjee/points_raw",
    ):
        assert topic in source
    assert "/sensing/lidar/points_filtered" not in source
    assert "/perception/obstacles" not in source


def test_full_carla_composition_uses_only_actual_carla_ui_sensor_sources():
    full = (
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
    ).read_text(encoding="utf-8")
    fake_launch = (
        REPO_ROOT / "camrod_bringup" / "launch" / "fake_sensors.launch.py"
    ).read_text(encoding="utf-8")
    relay_launch = (
        PACKAGE_ROOT / "launch" / "sensor_relay.launch.py"
    ).read_text(encoding="utf-8")
    lidar_processing_launch = (
        PACKAGE_ROOT / "launch" / "carla_lidar_processing.launch.py"
    ).read_text(encoding="utf-8")

    assert '"relay_imu": "true"' in full
    assert '"relay_gnss": "true"' in full
    assert '"compressed_image_max_rate_hz"' in full
    assert '"CAMROD_CARLA_COMPRESSED_IMAGE_MAX_RATE_HZ", "5.0"' in full
    assert '"CAMROD_CARLA_RAW_IMAGE_MAX_RATE_HZ", "10.0"' in full
    assert (
        '"compressed_image_max_rate_hz": LaunchConfiguration('
        in full
    )
    assert '"raw_image_max_rate_hz": LaunchConfiguration(' in full
    for argument in (
        "sim_publish_fake_gnss",
        "sim_publish_fake_imu",
        "sim_publish_fake_lidar_obstacle_cloud",
        "sim_publish_fake_radar_ranges",
        "sim_publish_velocity_converter_output",
        "sim_publish_dummy_lidar_cost_grid",
    ):
        assert f'"{argument}": "false"' in full

    for argument in (
        "publish_fake_gnss",
        "publish_fake_imu",
        "publish_fake_lidar_obstacle_cloud",
        "publish_fake_radar_ranges",
        "publish_velocity_converter_output",
        "publish_dummy_lidar_cost_grid",
    ):
        assert f"'{argument}'" in fake_launch

    assert '"lidar_output"' in relay_launch
    assert '"lidar_filtered_output"' not in relay_launch
    assert '"obstacle_cloud_output"' not in relay_launch
    assert 'executable="carla_lidar_filter"' in lidar_processing_launch
    assert 'executable="obstacle_lidar_node"' in lidar_processing_launch
    assert '"publish_cluster_cloud": True' in lidar_processing_launch
    assert '"perception_enable_lidar_obstacle": "false"' in full

    feedback_config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "feedback_bridge.yaml").read_text(
            encoding="utf-8"
        )
    )["/**"]["ros__parameters"]
    assert feedback_config["publish_gnss_heading_from_imu"] is True
    assert feedback_config["output_gnss_heading_topic"] == (
        "/sensing/gnss/navheading"
    )
    assert feedback_config["gnss_heading_yaw_bias_rad"] == (
        1.5707963267948966
    )
    assert feedback_config["imu_frame_id"] == "imu_link"
    assert feedback_config["gnss_frame_id"] == "gnss_link"
    assert feedback_config["input_gnss_right_topic"] == (
        "/carla/ego_vehicle/gnss_right"
    )
    assert feedback_config["publish_gnss_ui_compat"] is True
    assert feedback_config["output_gnss_navpvt_topic"] == (
        "/sensing/gnss/ublox_gps_node/navpvt"
    )
    assert feedback_config["output_gnss_navcov_topic"] == (
        "/sensing/gnss/navcov"
    )
    assert feedback_config["output_gnss_relpos_topic"] == (
        "/sensing/gnss/navrelposned"
    )


def test_carla_diagnostics_match_rendered_sensor_contract():
    root = (
        REPO_ROOT
        / "camrod_bringup"
        / "config"
        / "system"
        / "diagnostics"
        / "carla"
    )
    camera = yaml.safe_load(
        (root / "sensing" / "camera_checker.yaml").read_text(
            encoding="utf-8"
        )
    )["/system/camera_checker"]["ros__parameters"]
    assert camera["econ_front"]["expected_fps"] == 2.0
    assert camera["econ_rear"]["expected_fps"] == 2.0
    assert camera["econ_front"]["expected_width"] == 800
    assert camera["econ_front"]["expected_height"] == 600
    assert camera["econ_rear"]["expected_width"] == 960
    assert camera["econ_rear"]["expected_height"] == 720
    assert camera["econ_rear"]["image_type"] == "compressed"
    assert camera["econ_rear"]["image_topic"] == (
        "/sensing/camera/econ_rear/image_raw/compressed"
    )
    assert camera["econ_rear"]["expected_encoding"] == ""

    aggregator = yaml.safe_load(
        (root / "aggregator" / "diagnostics_config.yaml").read_text(
            encoding="utf-8"
        )
    )
    ignored = set(aggregator["global"]["ignored_names"])
    for active_name in (
        "camera_checker: /sensor/camera/econ_front",
        "camera_checker: /sensor/camera/econ_rear",
        "lidar_checker: /sensor/lidar/main",
        "radar_checker: /sensor/radar/FRONT1",
        "radar_checker: /sensor/radar/REAR",
        "perception_obstacle_checker: /perception/obstacles/camera_detections",
        "perception_obstacle_checker: /perception/obstacles/fused_obstacles",
    ):
        assert active_name not in ignored

    perception = yaml.safe_load(
        (root / "perception" / "perception_obstacle_checker.yaml").read_text(
            encoding="utf-8"
        )
    )["/system/perception_obstacle_checker"]["ros__parameters"]
    assert perception["obstacle_names"] == [
        "camera_detections",
        "fused_obstacles",
    ]
    assert perception["camera_detections"]["topic"] == (
        "/perception/camera/detections_2d"
    )
    assert perception["camera_detections"]["expected_hz"] == 2.0

    for filename, node_name, rate_key, stream_name in (
        ("gnss_checker.yaml", "gnss_checker", "expected_hz", "main"),
        ("imu_checker.yaml", "imu_checker", "expected_hz", "main"),
        ("lidar_checker.yaml", "lidar_checker", "expected_hz", "main"),
        (
            "wheel_odometry_checker.yaml",
            "wheel_odometry_checker",
            "expected_hz",
            "main",
        ),
    ):
        params = yaml.safe_load(
            (root / "sensing" / filename).read_text(encoding="utf-8")
        )[f"/system/{node_name}"]["ros__parameters"]
        assert params[stream_name][rate_key] == 2.0
