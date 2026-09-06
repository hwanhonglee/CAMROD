"""Pure contracts for the UI-driven camping-site matrix runner."""
# flake8: noqa

import importlib.util
import json
from pathlib import Path
import sys

import pytest


SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "virtual_carla" / "camping_site_matrix.py"
SPEC = importlib.util.spec_from_file_location("camping_site_matrix", SCRIPT)
assert SPEC and SPEC.loader
matrix = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = matrix
SPEC.loader.exec_module(matrix)


CAMROD = Path(__file__).resolve().parents[1].parent
SITES = CAMROD / "camrod_planning" / "config" / "camping_sites.yaml"
DROP_ZONES = CAMROD / "camrod_bringup" / "config" / "map" / "drop_zones.yaml"


def test_active_configuration_contains_all_thirteen_sites_and_drop_zone():
    sites = matrix.load_sites(SITES)
    drop_zone = matrix.load_drop_zone(DROP_ZONES)

    assert tuple(sites) == matrix.DEFAULT_SITES
    assert sites["B11"].service_mode == "roadside_stop"
    assert sites["B12"].service_mode == "roadside_stop"
    assert drop_zone.source_id == "dz_area_2320"
    assert (drop_zone.x_m, drop_zone.y_m) == pytest.approx((-14.2347, 39.7863))


def test_site_selection_defaults_to_all_and_rejects_duplicates_or_unknown():
    assert matrix.parse_site_selection(None) == matrix.DEFAULT_SITES
    assert matrix.parse_site_selection("b1,B12") == ("B1", "B12")
    with pytest.raises(matrix.MatrixError, match="duplicates"):
        matrix.parse_site_selection("B1,b1")
    with pytest.raises(matrix.MatrixError, match="B1..B13"):
        matrix.parse_site_selection("B0")


def test_plan_is_explicitly_read_only_and_records_config_provenance(tmp_path):
    sites = matrix.load_sites(SITES)
    drop_zone = matrix.load_drop_zone(DROP_ZONES)
    output = tmp_path / "matrix.json"
    report = matrix.build_plan(
        sites,
        ("B1", "B12"),
        drop_zone,
        output=output,
        config_paths={"camping_sites": SITES, "drop_zones": DROP_ZONES},
    )
    matrix.write_json_atomic(output, report)
    decoded = json.loads(output.read_text(encoding="utf-8"))

    assert decoded["status"] == "PLAN_ONLY"
    assert decoded["scope"]["motion_commands_sent"] is False
    assert decoded["scope"]["pose_teleport_used"] is False
    assert decoded["scope"]["fake_sensor_data_used"] is False
    assert decoded["scope"]["selected_sites"] == ["B1", "B12"]
    assert decoded["scope"]["mission_intent"] == "delivery"
    assert decoded["scope"]["unattempted_sites"] == [f"B{i}" for i in range(2, 12)] + ["B13"]
    assert decoded["sites"][0]["elapsed_s"] == 0.0
    assert decoded["sites"][0]["dispatch_started_at_utc"] == ""
    assert decoded["sites"][0]["outbound_duration_s"] == 0.0
    assert decoded["sites"][0]["return_duration_s"] == 0.0
    assert decoded["sites"][0]["outbound_distance_m"] == 0.0
    assert decoded["sites"][0]["return_distance_m"] == 0.0
    assert decoded["sites"][0]["total_odom_distance_m"] == 0.0
    assert decoded["sites"][0]["mission_intent"] == "delivery"
    assert decoded["sites"][0]["configured_service_mode"] == "turnaround"
    assert len(decoded["active_configuration"]["sha256"]["camping_sites"]) == 64
    assert len(decoded["active_configuration"]["sha256"]["drop_zones"]) == 64
    assert not list(tmp_path.glob("*.tmp"))


def test_motion_command_scope_latches_observed_runtime_motion():
    report = {"scope": {"motion_commands_sent": False}}

    matrix.update_motion_command_scope(
        report,
        {"motion_metrics": {"motion_command_observed": True}},
    )
    assert report["scope"]["motion_commands_sent"] is True

    # A later stationary snapshot must not erase evidence from the mission.
    matrix.update_motion_command_scope(
        report,
        {"motion_metrics": {"motion_command_observed": False}},
    )
    assert report["scope"]["motion_commands_sent"] is True


def test_default_plan_does_not_create_its_suggested_report(tmp_path, monkeypatch):
    monkeypatch.setenv("RANGER_WORK_ROOT", str(tmp_path))
    assert matrix.main([]) == 0
    assert not (tmp_path / "evidence").exists()


def test_physical_readiness_is_strict_and_actor_identity_is_stable():
    ready = {
        "ready": True,
        "physical_gate_accepted": True,
        "physx_substep_control_verified": True,
        "independent_wheel_drive_available": True,
        "motion_backend": "PHYSX_FOUR_WHEEL_STEERING",
        "actor_id": 51,
        "physical_manifest_sha256": "1" * 64,
        "production_authorization_sha256": "2" * 64,
        "ros_integration_sha256": "3" * 64,
        "imported_libcarla_sha256": "4" * 64,
        "wheel_torque_safety_cap_nm": 20.0,
    }
    assert matrix.validate_physical_status(ready) == 51
    assert matrix.validate_physical_status(ready, 51) == 51
    with pytest.raises(matrix.MatrixError, match="actor_id changed"):
        matrix.validate_physical_status({**ready, "actor_id": 52}, 51)
    with pytest.raises(matrix.MatrixError, match="readiness lost"):
        matrix.validate_physical_status({**ready, "ready": False})


def test_pose_error_is_planar_and_missing_pose_is_not_accepted():
    drop_zone = matrix.DropZone("dz", 10.0, 20.0, 0.0, 0.0)
    assert matrix.pose_error_m({"x_m": 13.0, "y_m": 24.0}, drop_zone) == pytest.approx(5.0)
    assert matrix.pose_error_m({}, drop_zone) is None


def test_start_and_final_drop_zone_tolerances_are_separate():
    args = matrix._parser().parse_args([])
    assert args.start_drop_zone_tolerance_m == pytest.approx(5.0)
    assert args.drop_zone_tolerance_m == pytest.approx(3.0)
    assert args.start_drop_zone_tolerance_m > args.drop_zone_tolerance_m

    args.start_drop_zone_tolerance_m = 0.0
    with pytest.raises(matrix.MatrixError, match="start-drop-zone"):
        matrix._validate_args(args)


def test_policy_phases_are_required_in_order():
    phases = ["IDLE", *matrix.SITE_PHASES["turnaround"], "IDLE"]
    assert matrix.contains_ordered_subsequence(
        phases, matrix.SITE_PHASES["turnaround"]
    )
    assert not matrix.contains_ordered_subsequence(
        [phase for phase in phases if phase != "ROTATE_180"],
        matrix.SITE_PHASES["turnaround"],
    )


def test_return_waits_for_service_state_and_ordered_controller_phase():
    expected = matrix.SITE_PHASES["turnaround"]
    service_state_arrived_first = {
        "service_state": {"state": matrix.WAITING_FOR_RETURN_REQUEST},
        "sequences": {
            "site_phases": ["CRAB_IN", "ROTATE_180", "UNLOAD_WAIT"]
        },
    }

    # Reproduces the live observation race: lifecycle state 11 must not let
    # the harness request Return while controller evidence ends at UNLOAD_WAIT.
    assert not matrix.arrival_ready_for_return(
        service_state_arrived_first, expected
    )

    controller_caught_up = {
        **service_state_arrived_first,
        "sequences": {
            "site_phases": [
                "CRAB_IN", "ROTATE_180", "UNLOAD_WAIT", "WAIT_RETURN"
            ]
        },
    }
    assert matrix.arrival_ready_for_return(controller_caught_up, expected)

    controller_arrived_first = {
        **controller_caught_up,
        "service_state": {"state": matrix.DROP_ZONE_PARKING},
    }
    assert not matrix.arrival_ready_for_return(controller_arrived_first, expected)

    out_of_order = {
        **controller_caught_up,
        "sequences": {
            "site_phases": [
                "CRAB_IN", "ROTATE_180", "WAIT_RETURN", "UNLOAD_WAIT"
            ]
        },
    }
    assert not matrix.arrival_ready_for_return(out_of_order, expected)


def test_recall_waits_for_guest_loading_and_always_uses_roadside_policy():
    turnaround_site = matrix.Site(
        "B1", "camping_site_1", 1.0, 2.0, 0.0, 0.0, "turnaround"
    )
    expected = matrix.SITE_PHASES["roadside_stop"]
    snapshot = {
        "service_state": {"state": matrix.GUEST_LOADING_WAIT},
        "sequences": {
            "site_phases": ["CRAB_IN", "UNLOAD_WAIT", "WAIT_RETURN"]
        },
    }

    assert matrix.effective_service_mode(turnaround_site, "recall") == "roadside_stop"
    assert matrix.arrival_ready_for_return(
        snapshot,
        expected,
        expected_service_state=matrix.GUEST_LOADING_WAIT,
    )
    assert not matrix.arrival_ready_for_return(snapshot, expected)
    assert matrix.required_service_state_ids("recall") == (
        matrix.RECALL_TO_SITE_ROAD,
        matrix.GUEST_LOADING_WAIT,
        matrix.RETURN_WITH_CARGO,
        matrix.DROP_ZONE_PARKING,
        matrix.WAITING_FOR_CHARGING,
        matrix.CHARGING,
    )


def test_sensor_audit_requires_exact_36_streams_and_13_actors(tmp_path):
    audit = tmp_path / "sensor-audit.json"
    document = {
        "status": "PASS",
        "passed": True,
        "summary": {
            "streams_checked": 36,
            "stream_failures": 0,
            "actors_checked": 13,
            "actor_failures": 0,
        },
    }
    audit.write_text(json.dumps(document), encoding="utf-8")
    accepted = matrix.load_sensor_audit(audit)
    assert accepted["status"] == "PASS"
    assert len(accepted["sha256"]) == 64

    document["summary"]["stream_failures"] = 1
    audit.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(matrix.MatrixError, match="sensor-source audit rejected"):
        matrix.load_sensor_audit(audit)


def test_runtime_profile_audit_binds_live_map_lanelet_and_parameters(tmp_path):
    lanelet = tmp_path / "lanelet2_maps.osm"
    lanelet.write_text("lanelet-map", encoding="utf-8")
    umap = tmp_path / "Woraksan_ramp.umap"
    umap.write_bytes(b"ue-map")
    profile = tmp_path / "runtime-profile.json"
    live_map = "map_package/Maps/Woraksan_ramp/Woraksan_ramp"
    actor_id = 42
    document = {
        "schema": "camrod.virtual_carla.runtime_profile_audit.v1",
        "status": "PASS",
        "accepted": True,
        "errors": [],
        "profile": "develop-parity",
        "carla": {
            "normalized_map_name": live_map,
            "ego_actor": {
                "actor_id": actor_id,
                "type_id": "vehicle.ranger.default",
                "role_name": "ego_vehicle",
            },
            "world_settings": {
                "synchronous_mode": True,
                "fixed_delta_seconds": 0.05,
                "no_rendering_mode": False,
            },
            "ue_map_asset": {
                "path": str(umap),
                "sha256": matrix.sha256_file(umap),
            },
        },
        "physical_four_wheel_bridge": {
            "status": "READY",
            "actor_id": actor_id,
            "motion_backend": "PHYSX_FOUR_WHEEL_STEERING",
        },
        "lanelet_map": {
            "path": str(lanelet),
            "sha256": matrix.sha256_file(lanelet),
        },
        "selected_live_parameters": json.loads(
            json.dumps(matrix.DEVELOP_PARITY_RUNTIME_SIGNATURE)
        ),
    }
    profile.write_text(json.dumps(document), encoding="utf-8")
    accepted = matrix.load_runtime_profile_audit(
        profile,
        expected_carla_map=f"/Game/{live_map}",
        expected_carla_town=live_map,
        expected_lanelet_map=lanelet,
        expected_actor_id=actor_id,
        expected_role_name="ego_vehicle",
    )
    assert accepted["status"] == "PASS"
    assert accepted["profile"] == "develop-parity"
    assert accepted["actor_id"] == actor_id
    assert accepted["carla_map"] == live_map
    assert accepted["lanelet_map"]["sha256"] == matrix.sha256_file(lanelet)

    for node, parameter, stale_value in (
        (
            "/perception/apriltag_parking_detector",
            "quad_decimate",
            2.0,
        ),
    ):
        expected_value = matrix.DEVELOP_PARITY_RUNTIME_SIGNATURE[node][parameter]
        document["selected_live_parameters"][node][parameter] = stale_value
        profile.write_text(json.dumps(document), encoding="utf-8")
        with pytest.raises(matrix.MatrixError, match=parameter):
            matrix.load_runtime_profile_audit(
                profile,
                expected_carla_map=f"/Game/{live_map}",
                expected_carla_town=live_map,
                expected_lanelet_map=lanelet,
                expected_actor_id=actor_id,
                expected_role_name="ego_vehicle",
            )
        document["selected_live_parameters"][node][parameter] = expected_value

    document["profile"] = "develop-plus-carla-site-geometry-v27"
    document["selected_live_parameters"] = json.loads(
        json.dumps(matrix.DEVELOP_SITE_GEOMETRY_RUNTIME_SIGNATURE)
    )
    profile.write_text(json.dumps(document), encoding="utf-8")
    accepted = matrix.load_runtime_profile_audit(
        profile,
        expected_carla_map=f"/Game/{live_map}",
        expected_carla_town=live_map,
        expected_lanelet_map=lanelet,
        expected_actor_id=actor_id,
        expected_role_name="ego_vehicle",
    )
    assert accepted["profile"] == "develop-plus-carla-site-geometry-v27"

    document["profile"] = "develop-plus-carla-site-geometry-v17"
    profile.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(matrix.MatrixError, match="runtime-profile audit rejected"):
        matrix.load_runtime_profile_audit(
            profile,
            expected_carla_map=f"/Game/{live_map}",
            expected_carla_town=live_map,
            expected_lanelet_map=lanelet,
            expected_actor_id=actor_id,
            expected_role_name="ego_vehicle",
        )

    document["profile"] = "develop-plus-carla-site-geometry-v27"

    for node, parameter, stale_value in (
        (
            "/control/camping_site_maneuver_controller",
            "entry_anchor_centering_max_initial_error_m",
            0.65,
        ),
        (
            "/planning/planning_state_machine",
            "return_goal_reached_distance_m",
            0.30,
        ),
        ("/perception/yolov9mit", "min_confidence", 0.50),
        ("/ui_backend", "return_site_exit_rearm_enabled", False),
        (
            "/control/cmd_vel_safety_gate",
            "cost_stop_latch_use_trigger_source_for_merged_clear",
            False,
        ),
        (
            "/control/cmd_vel_safety_gate",
            "camping_site_maneuver_controller_static_bypass_phases",
            "",
        ),
    ):
        expected_value = matrix.DEVELOP_SITE_GEOMETRY_RUNTIME_SIGNATURE[
            node
        ][parameter]
        document["selected_live_parameters"][node][parameter] = stale_value
        profile.write_text(json.dumps(document), encoding="utf-8")
        with pytest.raises(matrix.MatrixError, match=parameter):
            matrix.load_runtime_profile_audit(
                profile,
                expected_carla_map=f"/Game/{live_map}",
                expected_carla_town=live_map,
                expected_lanelet_map=lanelet,
                expected_actor_id=actor_id,
                expected_role_name="ego_vehicle",
            )
        document["selected_live_parameters"][node][parameter] = expected_value

    document["profile"] = "woraksan-tuned"
    profile.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(matrix.MatrixError, match="runtime-profile audit rejected"):
        matrix.load_runtime_profile_audit(
            profile,
            expected_carla_map=f"/Game/{live_map}",
            expected_carla_town=live_map,
            expected_lanelet_map=lanelet,
            expected_actor_id=actor_id,
            expected_role_name="ego_vehicle",
        )


def test_runtime_profile_signatures_differ_only_in_proven_carla_adaptations():
    parity = matrix.DEVELOP_PARITY_RUNTIME_SIGNATURE
    site = matrix.DEVELOP_SITE_GEOMETRY_RUNTIME_SIGNATURE
    controller = "/control/camping_site_maneuver_controller"
    command_adapter = "/camrod_twist_to_4ws"

    assert tuple(matrix.RUNTIME_PROFILE_SIGNATURES) == (
        "develop-parity",
        "develop-plus-carla-site-geometry-v27",
    )
    charger = "/carla_charging_contact_emulator"
    detector = "/perception/apriltag_parking_detector"
    assert set(site) == set(parity) | {charger}
    recovery_nodes = {
        "/control/cmd_vel_safety_gate",
        "/control/route_safety_recovery_controller",
        "/planning/goal_snapper",
        "/planning/controller_server",
        "/planning/nav2_selector_latch",
        "/planning/planning_state_machine",
        "/parking/apriltag_parking_controller",
        "/perception/yolov9mit",
        "/ui_backend",
    }
    for node in parity:
        if node not in ({controller, command_adapter} | recovery_nodes):
            assert site[node] == parity[node]
    assert charger not in parity
    assert site[charger] == {
        "drop_zone_id": "drop_zone",
        "pose_topic": "/localization/pose",
        "odometry_topic": "/odom",
        "parking_status_topic": (
            "/parking/apriltag_parking_controller/status"
        ),
        "planning_state_topic": "/planning/state_machine/state",
        "charging_topic": "/camrod_carla/platform_heartbeat/charging",
        "position_tolerance_m": 0.35,
        "speed_tolerance_mps": 0.05,
        "pose_timeout_s": 0.5,
        "odometry_timeout_s": 0.5,
        "state_timeout_s": 2.0,
        "dwell_s": 1.0,
        "publish_rate_hz": 10.0,
    }
    assert site[detector] == parity[detector] == {
        "image_topic": "/sensing/camera/econ_rear/image_rect",
        "camera_info_topic": "/sensing/camera/econ_rear/camera_info",
        "camera_frame_id": "camera_rear",
        "tag_family": "tag36h11",
        "target_tag_id": 3,
        "tag_size": 0.16,
        "quad_decimate": 1.0,
        "n_threads": 2,
        "roi_scale": 3.0,
        "roi_full_search_interval": 30,
        "max_reproj_error_px": 2.0,
        "publish_tf": True,
        "publish_debug_image": True,
        "debug_jpeg_quality": 80,
    }
    assert parity["/ui_backend"][
        "telemetry_docking_rear_camera_fallback_enabled"
    ] is False
    assert site["/ui_backend"][
        "telemetry_docking_rear_camera_fallback_enabled"
    ] is True
    assert parity["/ui_backend"]["return_site_exit_rearm_enabled"] is False
    assert site["/ui_backend"]["return_site_exit_rearm_enabled"] is True
    assert parity["/planning/planning_state_machine"][
        "return_goal_reached_distance_m"
    ] == 0.30
    assert site["/planning/planning_state_machine"][
        "return_goal_reached_distance_m"
    ] == 0.35
    assert parity["/perception/yolov9mit"]["min_confidence"] == 0.5
    assert site["/perception/yolov9mit"]["min_confidence"] == 0.95
    assert parity[command_adapter]["recovery_breakaway_enable"] is False
    assert site[command_adapter]["recovery_breakaway_enable"] is True
    assert parity[command_adapter][
        "rotation_recovery_breakaway_enable"
    ] is False
    assert site[command_adapter][
        "rotation_recovery_breakaway_enable"
    ] is True
    assert parity[command_adapter][
        "rotation_recovery_breakaway_status_timeout_sec"
    ] == 0.30
    assert site[command_adapter][
        "rotation_recovery_breakaway_status_timeout_sec"
    ] == 1.25
    assert site["/control/cmd_vel_safety_gate"][
        "route_safety_path_relative_recovery_enable"
    ] is True
    assert parity["/control/cmd_vel_safety_gate"][
        "route_safety_path_relative_recovery_enable"
    ] is False
    assert parity["/control/cmd_vel_safety_gate"][
        "route_safety_path_center_reentry_m"
    ] == 0.08
    assert site["/control/cmd_vel_safety_gate"][
        "route_safety_path_center_reentry_m"
    ] == 0.15
    assert parity["/control/cmd_vel_safety_gate"][
        "lanelet_safety_footprint_enable"
    ] is True
    assert site["/control/cmd_vel_safety_gate"][
        "lanelet_safety_footprint_enable"
    ] is False
    assert parity["/control/cmd_vel_safety_gate"][
        "lanelet_safety_check_reverse"
    ] is False
    assert site["/control/cmd_vel_safety_gate"][
        "lanelet_safety_check_reverse"
    ] is True
    for profile in (parity, site):
        gate = profile["/control/cmd_vel_safety_gate"]
        assert gate["lanelet_safety_enable"] is True
        assert gate["lanelet_safety_body_hard_stop_enable"] is True
        assert gate["lanelet_safety_body_hard_stop_threshold"] == 100
    assert site["/control/route_safety_recovery_controller"] == {
        "zero_hold_pauses_limits": True,
        "allow_corrective_yaw_beyond_limit": True,
    }
    assert site["/planning/goal_snapper"][
        "reissue_active_goal_after_route_recovery_when_nav_active"
    ] is True
    assert parity["/planning/goal_snapper"]["pose_jump_check_topic"] == ""
    assert (
        site["/planning/goal_snapper"]["pose_jump_check_topic"]
        == "/localization/pose"
    )
    assert parity["/planning/goal_snapper"][
        "reverse_auxiliary_input_goal_topic"
    ] == ""
    assert site["/planning/goal_snapper"][
        "reverse_auxiliary_input_goal_topic"
    ] == "/planning/auto_reverse_goal_raw"
    assert parity["/planning/nav2_selector_latch"][
        "reverse_controller_id"
    ] == "RPP"
    assert site["/planning/nav2_selector_latch"][
        "reverse_controller_id"
    ] == "RPPReverse"
    assert site["/planning/controller_server"]["controller_plugins"] == [
        "RPP", "RPPReverse", "RotationShim"
    ]
    assert site["/planning/controller_server"][
        "RPPReverse.allow_reversing"
    ] is True
    parking = "/parking/apriltag_parking_controller"
    assert parity[parking]["heading_gain"] == 1.5
    assert parity[parking]["lateral_to_heading_gain"] == 2.5
    assert parity[parking]["reverse_approach_speed_mps"] == 0.2
    assert parity[parking]["final_insertion_speed_mps"] == 0.05
    assert site[parking]["heading_gain"] == 1.5
    assert site[parking]["lateral_to_heading_gain"] == 2.7
    assert site[parking]["reverse_approach_speed_mps"] == 0.2
    assert site[parking]["final_insertion_speed_mps"] == 0.05
    assert site[parking]["translation_stop_tag_distance_m"] == 0.40
    assert site[parking]["final_lateral_tolerance_m"] == 0.03
    assert site[parking]["minimum_approach_turn_radius_m"] == 0.85
    assert parity[parking]["enable_bounded_lateral_retry"] is False
    assert parity[parking]["retry_forward_speed_mps"] == 0.10
    assert parity[parking]["retry_forward_timeout_s"] == 25.0
    assert site[parking]["enable_bounded_lateral_retry"] is True
    assert site[parking]["retry_forward_distance_m"] == 0.8
    assert site[parking]["retry_forward_speed_mps"] == 0.20
    assert site[parking]["retry_forward_timeout_s"] == 30.0
    assert site[parking]["retry_yaw_alignment_timeout_s"] == 8.0
    assert site[parking]["retry_maximum_lateral_error_m"] == 0.15
    assert site[parking]["retry_maximum_heading_error_rad"] == 0.35
    assert site[parking]["retry_maximum_forward_exit_lateral_drift_m"] == 0.15
    assert site[parking]["retry_maximum_odometry_step_m"] == 0.10
    assert site[parking]["retry_minimum_tag_distance_m"] == 0.35
    assert site[parking]["retry_maximum_tag_distance_m"] == 0.45
    assert site[parking]["maximum_retries"] == 2
    gated_timeout_budget_m = (
        site[parking]["retry_forward_speed_mps"]
        * site["/control/cmd_vel_safety_gate"]["speed_scale"]
        * site[parking]["retry_forward_timeout_s"]
    )
    assert gated_timeout_budget_m == 6.0
    assert site[parking]["retry_forward_distance_m"] <= gated_timeout_budget_m
    assert (
        site[parking]["retry_forward_distance_m"]
        + site[parking]["retry_maximum_odometry_step_m"]
    ) == 0.9
    assert site[controller]["rotate_180_timeout_s"] == 90.0
    assert parity[controller]["max_angular_speed_radps"] == 0.35
    assert site[controller]["max_angular_speed_radps"] == 0.45
    assert parity[controller]["roadside_reverse_return_enable"] is False
    assert site[controller]["roadside_reverse_return_enable"] is True
    assert site[controller]["roadside_reverse_handoff_distance_m"] == 0.03
    assert site[controller]["entry_position_tolerance_m"] == 0.05
    assert site[controller]["rotate_entry_centering_max_initial_error_m"] == 0.65
    assert site[controller]["entry_anchor_centering_max_initial_error_m"] == 0.0
    assert site[controller]["enable_live_lanelet_return_handoff"] is True
    assert site[controller]["return_lanelet_handoff_distance_m"] == 0.15
    assert site[controller]["return_lanelet_handoff_hold_s"] == 1.20
    assert site[parking]["tag_wait_timeout_s"] == 60.0
    assert site["/ui_backend"]["charging_departure_delay_s"] == 7.0
    assert site[controller]["crab_entry_body_yaw_compensation_deg"] == 2.0
    assert site[controller][
        "crab_entry_body_yaw_alignment_tolerance_deg"
    ] == 1.5
    assert parity[controller]["return_lateral_transition_tolerance_m"] == 0.02
    assert site[controller]["return_lateral_transition_tolerance_m"] == 0.02
    assert parity[controller]["return_lateral_hysteresis_m"] == 0.10
    assert site[controller]["return_lateral_hysteresis_m"] == 0.10
    assert parity[controller]["crab_out_yaw_recovery_enable"] is False
    assert site[controller]["crab_out_yaw_recovery_enable"] is True
    assert site[controller]["crab_out_yaw_recovery_trigger_deg"] == 8.0
    assert site[controller]["crab_out_yaw_recovery_max_attempts"] == 8
    assert site[controller]["crab_out_yaw_recovery_global_timeout_s"] == 90.0
    assert parity[controller]["rotate_180_timeout_s"] == 0.0
    assert parity[controller]["entry_position_tolerance_m"] == 0.15
    assert parity[controller][
        "crab_entry_body_yaw_alignment_tolerance_deg"
    ] == 0.5


def test_report_create_only_and_ui_endpoint_are_fail_closed(tmp_path):
    report = tmp_path / "matrix.json"
    matrix.write_json_atomic(report, {"status": "PLAN_ONLY"}, create_only=True)
    with pytest.raises(matrix.MatrixError, match="refusing to overwrite"):
        matrix.write_json_atomic(report, {"status": "PASS"}, create_only=True)
    assert json.loads(report.read_text(encoding="utf-8"))["status"] == "PLAN_ONLY"

    assert matrix.UIClient("http://127.0.0.1:8010").base_url.endswith(":8010")
    with pytest.raises(matrix.MatrixError, match="local http"):
        matrix.UIClient("https://example.com:8010")


def test_operator_dispatch_selects_delivery_or_typed_recall_endpoint():
    client = matrix.UIClient("http://127.0.0.1:8010")
    calls = []
    client.post = lambda path, query=None: calls.append((path, query)) or {
        "success": True
    }

    client.dispatch("B1", "delivery")
    client.dispatch("B2", "recall")

    assert calls == [
        ("/ui/destination", {"site": "B1", "run": "true"}),
        (
            "/ui/camping_site_recall",
            {"site": "B2", "intent": "recall"},
        ),
    ]


def test_guest_return_authority_requires_authenticated_usage_complete_source():
    snapshot = {
        "sequences": {
            "ui_operation_requests": [
                {"operation": matrix.RETURN_OPERATION, "source": "guest:usage_complete"}
            ]
        }
    }
    assert matrix.return_source_observed(snapshot, "guest:usage_complete")
    assert not matrix.return_source_observed(snapshot, "http:manual_return")
    assert not matrix.return_source_observed(
        {"sequences": {"ui_operation_requests": [
            {"operation": 2, "source": "guest:usage_complete"}
        ]}},
        "guest:usage_complete",
    )
    assert not matrix.return_source_observed(
        {"sequences": {"ui_operation_requests": [
            {"operation": "not-an-integer", "source": "guest:usage_complete"}
        ]}},
        "guest:usage_complete",
    )
    assert not matrix.return_source_observed(
        {"sequences": "malformed"}, "guest:usage_complete"
    )

    controller_snapshot = {
        "sequences": {
            "controller_operation_requests": [{
                "operation": matrix.RETURN_OPERATION,
                "source": "ws:usage_complete:site_exit_first",
            }]
        }
    }
    assert matrix.return_source_observed(
        controller_snapshot, "ws:usage_complete:site_exit_first"
    )

    tokenized_controller_snapshot = {
        "sequences": {
            "controller_operation_requests": [{
                "operation": matrix.RETURN_OPERATION,
                "source": (
                    "ws:usage_complete:ui_return_token="
                    "g1788692609018001-s1-326ebcaa9335f:site_exit_first"
                ),
            }]
        }
    }
    assert matrix.return_source_observed(
        tokenized_controller_snapshot,
        "ws:usage_complete:site_exit_first",
    )

    for rejected_source in (
        "ws:usage_complete:ui_return_token=:site_exit_first",
        "ws:usage_complete:ui_return_token=g0-s1-deadbeef:site_exit_first",
        "ws:usage_complete:ui_return_token=g1-s0-deadbeef:site_exit_first",
        "ws:usage_complete:ui_return_token=g1-s1-NOTHEX:site_exit_first",
        "ws:usage_complete:ui_return_token=g1-s1-deadbeef:spoof:site_exit_first",
        "ws-spoof:usage_complete:ui_return_token=g1-s1-deadbeef:site_exit_first",
    ):
        rejected = {
            "sequences": {
                "controller_operation_requests": [{
                    "operation": matrix.RETURN_OPERATION,
                    "source": rejected_source,
                }]
            }
        }
        assert not matrix.return_source_observed(
            rejected,
            "ws:usage_complete:site_exit_first",
        )


def test_operator_browser_dispatch_source_is_bound_to_ros_message_type():
    snapshot = {
        "sequences": {
            "destination_requests": [
                {"site": "B4", "run": True, "source": "ws"}
            ],
            "recall_requests": [
                {"site_name": "B7", "source": "robot_ui:recall"}
            ],
        }
    }
    assert matrix.dispatch_source_observed(
        snapshot,
        site="B4",
        mission_intent="delivery",
        expected_source="ws",
    )
    assert matrix.dispatch_source_observed(
        snapshot,
        site="B7",
        mission_intent="recall",
        expected_source="robot_ui:recall",
    )
    assert not matrix.dispatch_source_observed(
        snapshot,
        site="B4",
        mission_intent="delivery",
        expected_source="robot_ui:recall",
    )


def test_operator_browser_dispatch_accepts_only_canonical_backend_echo_marker():
    canonical = {
        "sequences": {
            "destination_requests": [{
                "site": "B1",
                "run": True,
                "source": "ws|ui_backend_echo=1938466-885848621332495-1",
            }]
        }
    }
    assert matrix.dispatch_source_observed(
        canonical,
        site="B1",
        mission_intent="delivery",
        expected_source="ws",
    )

    for rejected_source in (
        "ws-spoof|ui_backend_echo=1938466-1-1",
        "ws|ui_backend_echo=",
        "ws|ui_backend_echo=token|spoof",
        "ws:usage_complete",
    ):
        rejected = {
            "sequences": {
                "destination_requests": [{
                    "site": "B1",
                    "run": True,
                    "source": rejected_source,
                }]
            }
        }
        assert not matrix.dispatch_source_observed(
            rejected,
            site="B1",
            mission_intent="delivery",
            expected_source="ws",
        )


def test_operator_recall_dispatch_accepts_deferred_charging_destination_echo():
    canonical = {
        "sequences": {
            "destination_requests": [{
                "site": "B1",
                "run": True,
                "source": (
                    "robot_ui:recall|ui_backend_echo="
                    "2013497-890842327946299-14"
                ),
            }],
            "recall_requests": [],
        }
    }
    assert matrix.dispatch_source_observed(
        canonical,
        site="B1",
        mission_intent="recall",
        expected_source="robot_ui:recall",
    )

    for rejected in (
        {
            "site": "B2",
            "run": True,
            "source": canonical["sequences"]["destination_requests"][0]["source"],
        },
        {
            "site": "B1",
            "run": False,
            "source": canonical["sequences"]["destination_requests"][0]["source"],
        },
        {
            "site": "B1",
            "run": True,
            "source": "ws|ui_backend_echo=2013497-1-1",
        },
    ):
        snapshot = {
            "sequences": {
                "destination_requests": [rejected],
                "recall_requests": [],
            }
        }
        assert not matrix.dispatch_source_observed(
            snapshot,
            site="B1",
            mission_intent="recall",
            expected_source="robot_ui:recall",
        )


def test_guest_browser_arguments_are_local_and_opt_in():
    default = matrix._parser().parse_args([])
    assert default.return_authority == "operator_rest"
    assert default.mission_intent == "delivery"
    guest = matrix._parser().parse_args([
        "--return-authority", "guest_browser",
        "--mission-intent", "recall",
        "--guest-cdp-url", "http://127.0.0.1:9223",
        "--guest-ui-url", "http://localhost:8012",
    ])
    assert guest.return_authority == "guest_browser"
    matrix._validate_args(guest)
    invalid_guest = matrix._parser().parse_args([
        "--return-authority", "guest_browser",
    ])
    with pytest.raises(matrix.MatrixError, match="requires --mission-intent recall"):
        matrix._validate_args(invalid_guest)
    assert matrix.GuestBrowserClient._local_http_url(
        guest.guest_cdp_url, "--guest-cdp-url"
    ) == "http://127.0.0.1:9223"
    with pytest.raises(matrix.MatrixError, match="local http"):
        matrix.GuestBrowserClient._local_http_url(
            "http://example.com:9223", "--guest-cdp-url"
        )

    operator_browser = matrix._parser().parse_args([
        "--return-authority", "operator_browser",
        "--operator-cdp-url", "http://127.0.0.1:9224",
    ])
    matrix._validate_args(operator_browser)
    assert operator_browser.operator_cdp_url == "http://127.0.0.1:9224"
    operator_contract = matrix.ui_authority_contract(operator_browser)
    assert operator_contract["return_authority"] == "operator_browser"
    assert operator_contract["expected_dispatch_source"] == "ws"
    assert operator_contract["expected_return_source"] == (
        "ws:usage_complete:site_exit_first"
    )
    assert "pointer/text events" in operator_contract["motion_path"]


def test_authority_contract_preserves_operator_and_identifies_guest_path():
    operator = matrix._parser().parse_args([])
    operator_contract = matrix.ui_authority_contract(operator)
    assert operator_contract["motion_path"] == (
        "UI REST -> CAMROD planning -> CAMROD control -> "
        "CARLA physical 4WS bridge"
    )
    assert operator_contract["ui_endpoints"]["return"] == "POST /ui/manual_return"
    assert operator_contract["expected_return_source"] == ""

    guest = matrix._parser().parse_args([
        "--return-authority", "guest_browser",
        "--mission-intent", "recall",
    ])
    guest_contract = matrix.ui_authority_contract(guest)
    assert "visible Guest browser page" in guest_contract["motion_path"]
    assert "page-owned Guest WebSocket" in guest_contract["motion_path"]
    assert guest_contract["expected_return_operation"] == matrix.RETURN_OPERATION
    assert guest_contract["expected_return_source"] == "guest:usage_complete"
    assert guest_contract["expected_arrival_state"] == matrix.GUEST_LOADING_WAIT
    assert guest_contract["mission_intent"] == "recall"
    assert "sendUsageComplete" in guest_contract["ui_endpoints"]["return"]

    operator_recall = matrix._parser().parse_args([
        "--mission-intent", "recall",
    ])
    recall_contract = matrix.ui_authority_contract(operator_recall)
    assert recall_contract["expected_arrival_state"] == matrix.GUEST_LOADING_WAIT
    assert "camping_site_recall" in recall_contract["ui_endpoints"]["dispatch"]
    assert "typed PlanningRecallRequest" in recall_contract["motion_path"]


def test_site_metrics_expose_outbound_return_and_total_distance_and_time():
    metrics = matrix.mission_segment_metrics(
        dispatch_distance_m=0.25,
        arrival_distance_m=41.75,
        return_request_distance_m=41.80,
        final_distance_m=86.30,
        dispatch_monotonic=100.0,
        arrival_monotonic=220.125,
        return_request_monotonic=230.0,
        final_monotonic=350.25,
    )
    assert metrics == {
        "outbound_duration_s": 130.0,
        "return_duration_s": 120.25,
        "outbound_distance_m": 41.5,
        "return_distance_m": 44.5,
        "total_odom_distance_m": 86.05,
    }
    assert metrics["outbound_duration_s"] + metrics["return_duration_s"] == pytest.approx(
        250.25
    )

    with pytest.raises(matrix.MatrixError, match="odometry regressed"):
        matrix.mission_segment_metrics(
            dispatch_distance_m=0.0,
            arrival_distance_m=10.0,
            return_request_distance_m=9.0,
            final_distance_m=20.0,
            dispatch_monotonic=1.0,
            arrival_monotonic=2.0,
            return_request_monotonic=3.0,
            final_monotonic=4.0,
        )


def test_motion_measurement_resets_at_dispatch_without_losing_pose_baseline():
    observer = matrix.RosObservation.__new__(matrix.RosObservation)
    observer.latest = {
        "carla_odometry": {"x_m": 12.5, "y_m": -3.25},
    }
    observer._cmd_vel_samples = 99
    observer._cmd_vel_max_abs = 3.0
    observer._motion_command_observed = True
    observer._carla_odom_samples = 50
    observer._carla_odom_distance_m = 7.5
    observer._last_carla_odom_xy = None

    observer.begin_motion_measurement()

    assert observer._cmd_vel_samples == 0
    assert observer._cmd_vel_max_abs == 0.0
    assert observer._motion_command_observed is False
    assert observer._carla_odom_samples == 0
    assert observer._carla_odom_distance_m == 0.0
    assert observer._last_carla_odom_xy == (12.5, -3.25)


def test_collision_event_is_passive_optional_structured_evidence():
    class Value:
        pass

    message = Value()
    message.other_actor_id = 73
    message.normal_impulse = Value()
    message.normal_impulse.x = 1.25
    message.normal_impulse.y = -2.5
    message.normal_impulse.z = 3.75
    message.header = Value()
    message.header.stamp = Value()
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    message.header.frame_id = "ego_vehicle"

    assert matrix.collision_event_record(
        message, received_at_utc="2026-09-03T00:00:00Z"
    ) == {
        "received_at_utc": "2026-09-03T00:00:00Z",
        "other_actor_id": 73,
        "normal_impulse": {"x": 1.25, "y": -2.5, "z": 3.75},
        "timestamp": {"sec": 123, "nanosec": 456},
        "frame": "ego_vehicle",
    }

    contract = matrix.observation_contract()
    assert "/carla/ego_vehicle/collision" not in contract["required_topics"]
    assert contract["optional_evidence_topics"] == [
        "/carla/ego_vehicle/collision"
    ]
    assert matrix.observation_contract("test_robot")[
        "optional_evidence_topics"
    ] == ["/carla/test_robot/collision"]


def test_collision_accumulator_resets_per_site_and_snapshot_does_not_fail_without_events():
    observer = matrix.RosObservation.__new__(matrix.RosObservation)
    observer.latest = {
        "service_state": None, "pose": None, "cmd_vel": None,
        "carla_odometry": None, "physical": None, "site": None,
        "parking": {}, "drop_zone": None, "gate": None,
        "ui_operation_request": None,
    }
    observer._physical_identity = None
    observer._collision_topic = "/carla/ego_vehicle/collision"
    observer._collision_subscriber_created = False
    observer._collision_events = [{"other_actor_id": 1}]
    observer._collision_total_event_count = 1
    observer._collision_stale_queued_event_count = 0
    observer._collision_max_normal_impulse_norm = 0.0
    observer._collision_cutoff_stamp_ns = None
    observer._latest_odom_stamp_ns = 123
    observer._latest_carla_stamp_ns = 456
    for name in (
        "_service_state_ids", "_service_state_names", "_site_phases",
        "_parking_phases", "_drop_zone_phases", "_gate_states",
        "_ui_operation_requests", "_controller_operation_requests",
        "_destination_requests", "_recall_requests",
    ):
        setattr(observer, name, [])
    observer._cmd_vel_samples = 0
    observer._cmd_vel_max_abs = 0.0
    observer._motion_command_observed = False
    observer._carla_odom_samples = 0
    observer._carla_odom_distance_m = 0.0
    observer._last_carla_odom_xy = None
    observer._physical_samples = 0

    observer.begin_site()
    evidence = observer.snapshot()["collision_evidence"]
    assert evidence == {
        "topic": "/carla/ego_vehicle/collision",
        "subscriber_created": False,
        "publisher_count": None,
        "publisher_discovered": None,
        "subscriber_and_publisher_discovered": None,
        "event_count": 0,
        "sampled_event_count": 0,
        "omitted_event_count": 0,
        "stale_queued_event_count": 0,
        "maximum_normal_impulse_norm": 0.0,
        "sample_policy": {
            "maximum_samples": 256,
            "preserved_first_samples": 64,
            "preserved_recent_samples": 192,
        },
        "events": [],
    }
    assert observer._collision_cutoff_stamp_ns == 456


def test_collision_queue_cutoff_and_bounded_first_recent_samples():
    class Value:
        pass

    def event(index):
        message = Value()
        message.other_actor_id = index
        message.normal_impulse = Value()
        message.normal_impulse.x = float(index)
        message.normal_impulse.y = 0.0
        message.normal_impulse.z = 0.0
        message.header = Value()
        message.header.stamp = Value()
        message.header.stamp.sec = index
        message.header.stamp.nanosec = 0
        message.header.frame_id = "ego_vehicle"
        return message

    observer = matrix.RosObservation.__new__(matrix.RosObservation)
    observer._collision_events = []
    observer._collision_total_event_count = 0
    observer._collision_stale_queued_event_count = 0
    observer._collision_max_normal_impulse_norm = 0.0
    observer._collision_cutoff_stamp_ns = 10_000_000_000
    observer._latest_carla_stamp_ns = 10_000_000_000

    observer._collision(event(10))
    assert observer._collision_stale_queued_event_count == 1
    assert observer._collision_total_event_count == 0

    for index in range(11, 311):
        observer._collision(event(index))

    assert observer._collision_total_event_count == 300
    assert len(observer._collision_events) == matrix.COLLISION_SAMPLE_LIMIT
    indices = [sample["event_index"] for sample in observer._collision_events]
    assert indices[:matrix.COLLISION_FIRST_SAMPLE_LIMIT] == list(range(1, 65))
    assert indices[matrix.COLLISION_FIRST_SAMPLE_LIMIT:] == list(range(109, 301))
    assert observer._collision_max_normal_impulse_norm == 310.0
    assert observer._latest_carla_stamp_ns == 310_000_000_000


def test_collision_publisher_discovery_is_not_reported_as_a_dds_match():
    class Node:
        @staticmethod
        def count_publishers(topic):
            assert topic == "/carla/test_robot/collision"
            return 1

    observer = matrix.RosObservation.__new__(matrix.RosObservation)
    observer.node = Node()
    observer.latest = {}
    observer._physical_identity = None
    observer._collision_topic = "/carla/test_robot/collision"
    observer._collision_subscriber_created = False
    observer._collision_events = []
    observer._collision_total_event_count = 0
    observer._collision_stale_queued_event_count = 0
    observer._collision_max_normal_impulse_norm = 0.0
    for name in (
        "_service_state_ids", "_service_state_names", "_site_phases",
        "_parking_phases", "_drop_zone_phases", "_gate_states",
        "_ui_operation_requests", "_controller_operation_requests",
        "_destination_requests", "_recall_requests",
    ):
        setattr(observer, name, [])
    observer._cmd_vel_samples = 0
    observer._cmd_vel_max_abs = 0.0
    observer._motion_command_observed = False
    observer._carla_odom_samples = 0
    observer._carla_odom_distance_m = 0.0
    observer._physical_samples = 0

    evidence = observer.snapshot()["collision_evidence"]
    assert evidence["publisher_count"] == 1
    assert evidence["publisher_discovered"] is True
    assert evidence["subscriber_and_publisher_discovered"] is False


def test_final_callback_drain_processes_terminal_queue_for_bounded_time(monkeypatch):
    observer = matrix.RosObservation.__new__(matrix.RosObservation)
    calls = []
    clock = iter((10.0, 10.0, 10.10, 10.20, 10.26))

    monkeypatch.setattr(matrix.time, "monotonic", lambda: next(clock))
    observer.spin_once = lambda timeout_s: calls.append(timeout_s)

    observer.drain_final_callbacks()

    assert calls == [0.01, 0.01, 0.01]


class _FakeCDPConnection:
    def __init__(self, frames):
        self.frames = list(frames)
        self.sent = []
        self.timeouts = []
        self.closed = False

    def send(self, payload):
        self.sent.append(json.loads(payload))

    def settimeout(self, timeout):
        self.timeouts.append(timeout)

    def recv(self):
        if not self.frames:
            raise RuntimeError("no fake CDP frame")
        return json.dumps(self.frames.pop(0))

    def close(self):
        self.closed = True


def _guest_client_with_connection(connection):
    client = matrix.GuestBrowserClient.__new__(matrix.GuestBrowserClient)
    client.timeout_s = 1.0
    client._command_id = 0
    client._connection = connection
    client._target = {}
    return client


def test_cdp_evaluate_ignores_events_and_requires_by_value_response():
    connection = _FakeCDPConnection([
        {"method": "Runtime.consoleAPICalled", "params": {}},
        {
            "id": 1,
            "result": {
                "result": {
                    "type": "object",
                    "value": {"accepted": True},
                }
            },
        },
    ])
    client = _guest_client_with_connection(connection)
    assert client._evaluate("6 * 7") == {"accepted": True}
    assert connection.sent == [{
        "id": 1,
        "method": "Runtime.evaluate",
        "params": {
            "expression": "6 * 7",
            "returnByValue": True,
            "awaitPromise": True,
        },
    }]
    assert connection.timeouts

    malformed_connection = _FakeCDPConnection([
        {"id": 1, "result": {"result": {"type": "undefined"}}},
    ])
    malformed = _guest_client_with_connection(malformed_connection)
    with pytest.raises(matrix.MatrixError, match="by-value"):
        malformed._evaluate("undefined")
    assert malformed_connection.closed is True

    javascript_error_connection = _FakeCDPConnection([
        {
            "id": 1,
            "result": {
                "exceptionDetails": {"text": "ReferenceError"},
                "result": {"type": "object", "value": None},
            },
        },
    ])
    javascript_error = _guest_client_with_connection(
        javascript_error_connection
    )
    with pytest.raises(matrix.MatrixError, match="JavaScript failed"):
        javascript_error._evaluate("missingName")
    assert javascript_error_connection.closed is True


def test_guest_actions_call_page_lexicals_and_verify_exact_ws_frames():
    client = matrix.GuestBrowserClient.__new__(matrix.GuestBrowserClient)
    expressions = []

    def evaluate(expression):
        expressions.append(expression)
        action = "usage_complete" if "sendUsageComplete" in expression else "navigate"
        return {"accepted": True, "action": action}

    client._evaluate = evaluate
    assert client.dispatch("B1")["accepted"] is True
    assert client.request_return()["accepted"] is True

    dispatch_expression, return_expression = expressions
    assert "window.ws" not in dispatch_expression
    assert "selectSite(requestedSite)" in dispatch_expression
    assert "openConfirm(); confirmNavigate();" in dispatch_expression
    assert "frame.action !== 'navigate'" in dispatch_expression
    assert "frame.site !== requestedSite" in dispatch_expression
    assert "currentPhase !== 'arrived'" in return_expression
    assert "currentState !== 8" in return_expression
    assert "sendUsageComplete()" in return_expression
    assert "frame.action !== 'usage_complete'" in return_expression

    rejected_connection = _FakeCDPConnection([])
    rejected = _guest_client_with_connection(rejected_connection)
    rejected._evaluate = lambda expression: {
        "accepted": False,
        "reason": "guest_ui_not_ready",
    }
    with pytest.raises(matrix.MatrixError, match="rejected navigate"):
        rejected.dispatch("B2")
    assert rejected_connection.closed is True


def _operator_client_with_fake_visible_dom(probe):
    client = matrix.OperatorBrowserClient.__new__(matrix.OperatorBrowserClient)
    client.timeout_s = 1.0
    client._interactions = []
    client._typed_value = ""
    client.cdp_calls = []
    client._install_transport_probe = lambda: None

    def element(selector):
        return {
            "count": 1,
            "visibleCount": 1,
            "disabled": False,
            "pressed": "true",
            "x": 100.0,
            "y": 200.0,
            "value": (
                client._typed_value
                if selector == '[data-ui="operator-site-code-input"]'
                else None
            ),
        }

    def call(method, params):
        client.cdp_calls.append((method, dict(params)))
        if method == "Input.insertText":
            client._typed_value = params["text"]
        return {"result": {}}

    client._element = element
    client._call = call
    client._wait_probe = lambda predicate, description: predicate(probe)
    return client


def test_operator_browser_uses_real_pointer_and_text_events_for_full_flow():
    probe = {
        "websocket": [
            {"frame": {"site": "B7", "state": True}},
            {"frame": {"usage_complete": True}},
        ],
        "http": [],
    }
    client = _operator_client_with_fake_visible_dom(probe)

    dispatch = client.dispatch("B7", "delivery")
    returned = client.request_return()

    assert dispatch["source"] == "ws"
    assert dispatch["transport"] == (
        "visible_operator_page_websocket_via_cdp_input"
    )
    assert returned["source"] == "ws:usage_complete"
    assert returned["expected_ros_source"] == (
        "ws:usage_complete:site_exit_first"
    )
    methods = [method for method, _ in client.cdp_calls]
    assert "Input.dispatchMouseEvent" in methods
    assert methods.count("Input.insertText") == 1
    selectors = [item["selector"] for item in dispatch["interactions"]]
    assert '[data-ui="operator-intent-delivery"]' in selectors
    assert '[data-ui="operator-site-page-1"]' in selectors
    assert '[data-ui="operator-site-B7"]' in selectors
    assert '[data-ui="operator-site-code-confirm"]' in selectors
    assert returned["interactions"][0]["selector"] == (
        '[data-ui="operator-arrival-return-confirm"]'
    )


def test_operator_browser_recall_requires_real_successful_frontend_http_record():
    probe = {
        "websocket": [],
        "http": [{
            "url": (
                "http://127.0.0.1:8010/ui/camping_site_recall?"
                "site=B10&intent=recall"
            ),
            "method": "POST",
            "status": 200,
            "ok": True,
            "body": {"success": True, "intent": "recall", "site": "B10"},
        }],
    }
    client = _operator_client_with_fake_visible_dom(probe)
    response = client.dispatch("B10", "recall")

    assert response["intent"] == "recall"
    assert response["source"] == "robot_ui:recall"
    assert response["transport"] == "visible_operator_page_http_via_cdp_input"
    selectors = [item["selector"] for item in response["interactions"]]
    assert '[data-ui="operator-intent-recall"]' in selectors
    assert '[data-ui="operator-site-page-1"]' in selectors


def test_guest_constructor_closes_cdp_socket_when_readiness_fails(monkeypatch):
    connections = []

    def fail_after_socket(self):
        connection = _FakeCDPConnection([])
        connections.append(connection)
        self._connection = connection
        raise matrix.MatrixError("visible page not ready")

    monkeypatch.setattr(matrix.GuestBrowserClient, "_connect", fail_after_socket)
    with pytest.raises(matrix.MatrixError, match="visible page not ready"):
        matrix.GuestBrowserClient(
            "http://127.0.0.1:9223",
            "http://127.0.0.1:8012",
            "http://127.0.0.1:8010",
        )
    assert len(connections) == 1
    assert connections[0].closed is True


def test_run_matrix_closes_guest_client_when_ros_observer_init_fails(
    tmp_path, monkeypatch
):
    class FakeGuestClient:
        expected_return_source = "guest:usage_complete"

        def __init__(self, *args, **kwargs):
            self.closed = False

        def close(self):
            self.closed = True

    guest = FakeGuestClient()
    monkeypatch.setattr(matrix, "GuestBrowserClient", lambda *args: guest)

    def fail_observer(*args, **kwargs):
        raise matrix.MatrixError("observer init failed")

    monkeypatch.setattr(matrix, "RosObservation", fail_observer)
    args = matrix._parser().parse_args([
        "--return-authority", "guest_browser",
        "--mission-intent", "recall",
        "--output", str(tmp_path / "report.json"),
    ])
    report = {"scope": {}}
    with pytest.raises(matrix.MatrixError, match="observer init failed"):
        matrix.run_matrix(args, {}, matrix.DropZone("dz", 0, 0, 0, 0), (), report)
    assert guest.closed is True
