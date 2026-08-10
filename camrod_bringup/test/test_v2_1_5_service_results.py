"""Validate committed v2.1.5 service evidence and regenerated visuals."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from PIL import Image


SRC_ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = (
    SRC_ROOT
    / "docs"
    / "assets"
    / "module-guides"
    / "bringup"
    / "test-results"
    / "v2-1-5-service-validation-20260807"
)
ENDURANCE_EVIDENCE = (
    SRC_ROOT
    / "docs"
    / "assets"
    / "module-guides"
    / "bringup"
    / "test-results"
    / "b1-b10-service-endurance-20260807"
)
RPP_EVIDENCE = (
    SRC_ROOT
    / "docs"
    / "assets"
    / "module-guides"
    / "planning"
    / "test-results"
    / "rpp-lookahead-service-ab-20260807"
)
RENDERER = (
    SRC_ROOT
    / "camrod_bringup"
    / "scripts"
    / "visualization"
    / "render_v2_1_5_service_results.py"
)
RPP_RENDERER = (
    SRC_ROOT
    / "camrod_bringup"
    / "scripts"
    / "visualization"
    / "render_rpp_service_ab.py"
)
MAP_SHA256 = "8cd05c66f846cae8718b5af148d123718f403a086f2e7d16165da89fb625e021"


def _assert_manifest(root: Path) -> None:
    """Verify every committed evidence file against its SHA-256 inventory."""
    manifest = json.loads((root / "evidence-manifest.json").read_text())
    assert manifest["map_sha256"] == MAP_SHA256
    for name, expected in manifest["files"].items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == expected


def test_v2_1_5_service_reports_retain_release_acceptance() -> None:
    """Lock repeated service, safe obstacle hold, and B2 recovery results."""
    service = json.loads((EVIDENCE / "repeated-service-soak.json").read_text())
    soak = next(item for item in service["checks"] if item["name"] == "repeated_service_soak")
    assert service["overall_pass"] is True
    assert soak["success"] is True
    assert soak["metrics"]["cycles_completed"] == 3
    assert soak["metrics"]["bringup_restart_count"] == 0
    assert all(cycle["boundary_recovery_ok"] for cycle in soak["metrics"]["cycles"])
    assert all(not cycle["boundary_retry_latched"] for cycle in soak["metrics"]["cycles"])
    assert soak["metrics"]["cycles"][1]["obstacle_resume_seen"] is True

    obstacle = json.loads((EVIDENCE / "obstacle-safe-hold.json").read_text())
    check = next(item for item in obstacle["checks"] if item["name"] == "obstacle_replan")
    assert obstacle["overall_pass"] is True
    assert check["metrics"]["fallback_failed_hold_seen"] is True
    assert check["metrics"]["fallback_selector_seen"] is False
    assert check["metrics"]["resumed_after_obstacle_clear"] is True

    for trial_path in sorted(EVIDENCE.glob("b2-boundary-recovery-trial-*.json")):
        trial = json.loads(trial_path.read_text())
        assert trial["map"] == {
            "source_file": "lanelet2_maps.osm",
            "map_version": 17,
            "sha256": MAP_SHA256,
        }
        assert trial["mission_completed"] is True
        assert trial["automatic_recovery_motion"] == "REVERSE_YAW_RIGHT"
        assert trial["second_hold"] is None
        assert trial["rapid_recontact_latched"] is False

    # HH_260810 - This acceptance run remains historical map-v17 evidence.
    # The current user-authored map has its own source-derived coordinate test.
    assert hashlib.sha256((SRC_ROOT / "lanelet2_maps.osm").read_bytes()).hexdigest() != MAP_SHA256


def test_v2_1_5_service_visuals_are_reproducible(tmp_path: Path) -> None:
    """Regenerate a temporary copy and validate every output format."""
    for name in (
        "repeated-service-soak.json",
        "obstacle-safe-hold.json",
        "b2-boundary-recovery-trial-1.json",
        "b2-boundary-recovery-trial-2.json",
        "b2-boundary-recovery-trial-3.json",
    ):
        (tmp_path / name).write_bytes((EVIDENCE / name).read_bytes())

    subprocess.run(
        [sys.executable, str(RENDERER), "--evidence-dir", str(tmp_path)],
        cwd=SRC_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    for name in (
        "repeated-service-summary.png",
        "obstacle-safe-hold.png",
        "b2-boundary-recovery.png",
    ):
        with Image.open(tmp_path / name) as visual:
            assert visual.width == 1600
            assert visual.height >= 880
            assert visual.format == "PNG"

    with Image.open(tmp_path / "repeated-service-timeline.gif") as visual:
        assert visual.size == (1400, 520)
        assert visual.n_frames == 24
        assert visual.format == "GIF"


def test_ten_cycle_endurance_visuals_are_reproducible(tmp_path: Path) -> None:
    """Render ten service cycles without the legacy three-card limit."""
    # HH_260807 - The final endurance evidence uses the same report schema. A
    # compact synthetic fixture keeps this renderer contract fast and isolated.
    cycles = []
    for cycle in range(1, 11):
        cycles.append(
            {
                "cycle": cycle,
                "mission_key": f"camping_site_{cycle}",
                "success": True,
                "elapsed_s": 120.0 + cycle,
                "service_states": ["WAITING_FOR_RETURN_REQUEST", "CHARGING"],
                "obstacle_required": cycle == 5,
                "boundary_hold_seen": cycle % 2 == 0,
                "boundary_recovery_ok": True,
            }
        )
    report = {
        "overall_pass": True,
        "checks": [
            {
                "name": "repeated_service_soak",
                "success": True,
                "detail": "ok",
                "metrics": {
                    "cycles_requested": 10,
                    "cycles_completed": 10,
                    "first_cycle_seeded_near_route": True,
                    "full_outbound_cycles": 9,
                    "elapsed_s": 1255.0,
                    "bringup_restart_count": 0,
                    "cycles": cycles,
                },
            }
        ],
    }
    report_path = tmp_path / "b1-b10-service-endurance.json"
    report_path.write_text(json.dumps(report), encoding="utf-8")

    subprocess.run(
        [sys.executable, str(RENDERER), "--endurance-report", str(report_path)],
        cwd=SRC_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    with Image.open(tmp_path / "b1-b10-service-endurance.png") as visual:
        assert visual.size == (1600, 1160)
        assert visual.format == "PNG"
    with Image.open(tmp_path / "b1-b10-service-endurance.gif") as visual:
        assert visual.size == (1400, 620)
        assert visual.n_frames == 10
        assert visual.format == "GIF"


def test_committed_ten_cycle_endurance_covers_the_full_service_contract() -> None:
    """Lock the measured B1-B10 lifecycle, obstacle, and recovery result."""
    report = json.loads(
        (ENDURANCE_EVIDENCE / "b1-b10-service-endurance.json").read_text()
    )
    check = next(item for item in report["checks"] if item["name"] == "repeated_service_soak")
    metrics = check["metrics"]
    cycles = metrics["cycles"]

    assert report["overall_pass"] is True
    assert check["success"] is True
    assert metrics["cycles_requested"] == 10
    assert metrics["cycles_completed"] == 10
    assert metrics["elapsed_s"] == 2210.611
    assert metrics["bringup_restart_count"] == 0
    assert [cycle["cycle"] for cycle in cycles] == list(range(1, 11))
    assert [cycle["mission_key"] for cycle in cycles] == [
        f"camping_site_{index}" for index in range(1, 11)
    ]

    required_site_phases = {
        "CRAB_IN",
        "ROTATE_180",
        "UNLOAD_WAIT",
        "WAIT_RETURN",
        "ALIGN_RETRACE_YAW",
        "CRAB_OUT",
        "DONE",
    }
    required_states = {
        "MOVING_TO_SITE",
        "SITE_ENTRY",
        "UNLOAD_WAIT",
        "WAITING_FOR_RETURN_REQUEST",
        "RETURN_WITH_CARGO",
        "DROP_ZONE_PARKING",
        "WAITING_FOR_CHARGING",
        "CHARGING",
    }
    for cycle in cycles:
        phases = {event.split("|", 1)[0] for event in cycle["site_phases"]}
        assert cycle["success"] is True
        assert cycle["return_sent_after_wait"] is True
        assert cycle["route_motion_seen"] is True
        assert cycle["charger_disconnected"] is True
        assert required_site_phases <= phases
        assert required_states <= set(cycle["service_states"])
        assert any("ALIGN_PARKING_YAW" in event for event in cycle["drop_phases"])
        assert any("PARKED" in event for event in cycle["parking_phases"])
        assert cycle["boundary_retry_latched"] is False
        assert cycle["boundary_recovery_ok"] is True

    # HH_260807 - Cycle 1 is deliberately seeded near B1; cycles 2-10 must
    # prove the charger departure and complete outbound route without restart.
    assert "DEPARTING_CHARGER" not in cycles[0]["service_states"]
    assert all("DEPARTING_CHARGER" in cycle["service_states"] for cycle in cycles[1:])
    assert sum(cycle["boundary_hold_seen"] for cycle in cycles) == 9
    assert all(
        cycle["boundary_recovery_motion_seen"] and cycle["boundary_recovery_released"]
        for cycle in cycles
        if cycle["boundary_hold_seen"]
    )

    obstacle = cycles[4]
    assert obstacle["obstacle_required"] is True
    assert obstacle["obstacle_stop_seen"] is True
    assert obstacle["obstacle_cleared"] is True
    assert obstacle["obstacle_resume_seen"] is True
    assert all(not cycle["obstacle_required"] for cycle in cycles[:4] + cycles[5:])

    scope = json.loads((ENDURANCE_EVIDENCE / "run-scope.json").read_text())
    assert scope["run"]["full_outbound_cycles"] == 9
    assert scope["observed_acceptance"]["post_start_system_or_path_faults"] == 0
    assert scope["observed_acceptance"]["run_owned_residual_processes"] == 0
    assert scope["observed_acceptance"]["return_anchor_error_max_m"] == 0.04

    geometry = json.loads(
        (ENDURANCE_EVIDENCE / "b4-return-anchor-geometry.json").read_text()
    )
    assert geometry["route_snap"]["lanelet_id"] == 2193
    assert geometry["cross_section"]["centered_clearance_per_side_m"] < 0.14
    assert geometry["observed_before_return"]["nav2_arrival_to_route_snap_m"] == 0.27
    assert geometry["selected_return_contract"]["return_position_tolerance_m"] == 0.04
    assert geometry["selected_return_contract"]["post_done_margin_or_body_hold"] is False

    path_log = (ENDURANCE_EVIDENCE / "path-grace-smoke.log").read_text()
    assert "reason=empty_route" in path_log
    assert "[SYSTEM] OK" in path_log
    assert "[SYSTEM] WARN" not in path_log
    assert "[SYSTEM] ERROR" not in path_log
    ui_log = (ENDURANCE_EVIDENCE / "ui-clean-shutdown.log").read_text()
    assert "ui_backend ready" in ui_log
    assert "ui_guest ready" in ui_log
    assert ui_log.count("process has finished cleanly") == 2
    assert "Traceback" not in ui_log
    assert "RuntimeError" not in ui_log

    _assert_manifest(ENDURANCE_EVIDENCE)


def test_committed_endurance_visuals_regenerate_from_the_raw_report(tmp_path: Path) -> None:
    """Regenerate the final ten-cycle visual and validate the committed media."""
    report = ENDURANCE_EVIDENCE / "b1-b10-service-endurance.json"
    temporary_report = tmp_path / report.name
    temporary_report.write_bytes(report.read_bytes())
    subprocess.run(
        [sys.executable, str(RENDERER), "--endurance-report", str(temporary_report)],
        cwd=SRC_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    for root in (ENDURANCE_EVIDENCE, tmp_path):
        with Image.open(root / "b1-b10-service-endurance.png") as visual:
            assert visual.size == (1600, 1160)
            assert visual.format == "PNG"
        with Image.open(root / "b1-b10-service-endurance.gif") as visual:
            assert visual.size == (1400, 620)
            assert visual.n_frames == 10
            assert visual.format == "GIF"


def test_rpp_service_ab_evidence_and_renderer_are_reproducible(tmp_path: Path) -> None:
    """Lock the rejected scaled preview and selected fixed 1.1 m profile."""
    comparison = json.loads((RPP_EVIDENCE / "comparison.json").read_text())
    assert comparison["map"]["sha256"] == MAP_SHA256
    assert comparison["baseline"]["rapid_recontact_latched"] is True
    assert comparison["baseline"]["recontact_after_release_s"] == 0.8499291
    assert comparison["baseline"]["service_completed"] is False
    assert comparison["selected"]["lookahead_m"] == 1.1
    assert comparison["selected"]["cycles_completed"] == 2
    assert comparison["selected"]["bringup_restart_count"] == 0
    assert comparison["selected"]["service_completed"] is True

    output = tmp_path / "rpp-lookahead-service-ab.png"
    subprocess.run(
        [
            sys.executable,
            str(RPP_RENDERER),
            "--input",
            str(RPP_EVIDENCE / "comparison.json"),
            "--output",
            str(output),
        ],
        cwd=SRC_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    with Image.open(output) as visual:
        assert visual.size == (1600, 920)
        assert visual.format == "PNG"
    _assert_manifest(RPP_EVIDENCE)
