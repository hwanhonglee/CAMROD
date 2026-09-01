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
    assert decoded["scope"]["unattempted_sites"] == [f"B{i}" for i in range(2, 12)] + ["B13"]
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


def test_report_create_only_and_ui_endpoint_are_fail_closed(tmp_path):
    report = tmp_path / "matrix.json"
    matrix.write_json_atomic(report, {"status": "PLAN_ONLY"}, create_only=True)
    with pytest.raises(matrix.MatrixError, match="refusing to overwrite"):
        matrix.write_json_atomic(report, {"status": "PASS"}, create_only=True)
    assert json.loads(report.read_text(encoding="utf-8"))["status"] == "PLAN_ONLY"

    assert matrix.UIClient("http://127.0.0.1:8010").base_url.endswith(":8010")
    with pytest.raises(matrix.MatrixError, match="local http"):
        matrix.UIClient("https://example.com:8010")
