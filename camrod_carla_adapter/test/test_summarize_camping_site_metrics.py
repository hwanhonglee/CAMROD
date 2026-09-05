"""Contracts for deterministic camping-site time/distance evidence summaries."""
# flake8: noqa

import csv
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys

import pytest


SCRIPT = (
    Path(__file__).resolve().parents[2]
    / "scripts"
    / "virtual_carla"
    / "summarize_camping_site_metrics.py"
)
SPEC = importlib.util.spec_from_file_location("summarize_camping_site_metrics", SCRIPT)
assert SPEC and SPEC.loader
metrics = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = metrics
SPEC.loader.exec_module(metrics)


CONFIG_HASHES = {
    "camping_sites": "a" * 64,
    "drop_zones": "b" * 64,
}
PHYSICAL_HASH = "c" * 64
GATE_HASH = "d" * 64


def _site(site="B1", status="PASS", waiting=True, **updates):
    row = {
        "site": site,
        "status": status,
        "service_mode": "roadside_stop" if site in {"B11", "B12"} else "turnaround",
        "elapsed_s": 30.0,
        "motion_metrics": {"carla_odometry_distance_m": 12.0},
        "actor_id": int(site[1:]) + 100,
        "drop_zone_error_m": 0.25 if status == "PASS" else None,
        "final_service_state": {
            "state": 13 if status == "PASS" else 1,
            "state_name": "CHARGING" if status == "PASS" else "MOVING_TO_SITE",
        },
        "parking_confirmed": status == "PASS",
        "charging_confirmed": status == "PASS",
        "final_physical_four_wheel_status": {
            "actor_id": int(site[1:]) + 100,
            "physical_manifest_sha256": PHYSICAL_HASH,
        },
        "milestones": [],
    }
    if waiting:
        row["milestones"].append(
            {
                "event": "WAITING_FOR_RETURN_REQUEST with ordered WAIT_RETURN phase",
                "elapsed_s": 11.5,
                "observation": {
                    "service_state": {
                        "state": 11,
                        "state_name": "WAITING_FOR_RETURN_REQUEST",
                    },
                    "motion_metrics": {"carla_odometry_distance_m": 4.25},
                },
            }
        )
    row.update(updates)
    return row


def _report(path, sites, status=None, config_hashes=None):
    finalized = [site for site in sites if site["status"] in {"PASS", "FAIL"}]
    report_status = status or ("FAIL" if any(s["status"] == "FAIL" for s in finalized) else "PASS")
    document = {
        "schema": metrics.MATRIX_SCHEMA,
        "status": report_status,
        "scope": {"selected_sites": [site["site"] for site in sites]},
        "active_configuration": {
            "sha256": config_hashes or CONFIG_HASHES,
            "baseline_gate_manifest_sha256": GATE_HASH,
        },
        "sites": sites,
    }
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


def _verify_sha256s(directory):
    lines = (directory / "SHA256SUMS").read_text(encoding="utf-8").splitlines()
    assert len(lines) == 3
    for line in lines:
        digest, name = line.split("  ", 1)
        assert hashlib.sha256((directory / name).read_bytes()).hexdigest() == digest


def test_unsorted_individual_reports_emit_numeric_site_order_and_measured_splits(tmp_path):
    b12 = _report(tmp_path / "b12.json", [_site("B12")])
    b2 = _report(tmp_path / "b2.json", [_site("B2")])
    summary = metrics.build_summary([b12, b2])

    assert [row["site"] for row in summary["sites"]] == ["B2", "B12"]
    row = summary["sites"][0]
    assert row["outbound_duration_s"] == 11.5
    assert row["return_duration_s"] == 18.5
    assert row["outbound_distance_m"] == 4.25
    assert row["return_distance_m"] == 7.75
    assert row["split_duration_source"] == "WAITING_FOR_RETURN_REQUEST milestone"
    assert summary["aggregate"]["pass_count"] == 2
    assert summary["aggregate"]["total_elapsed_s"] == 60.0
    assert summary["aggregate"]["total_odom_distance_m"] == 24.0

    output = tmp_path / "summary"
    metrics.write_summary(output, summary)
    decoded = json.loads((output / "camping_site_metrics.json").read_text())
    assert [row["site"] for row in decoded["sites"]] == ["B2", "B12"]
    csv_rows = list(
        csv.DictReader(io.StringIO((output / "camping_site_metrics.csv").read_text()))
    )
    assert [row["site"] for row in csv_rows] == ["B2", "B12"]
    assert csv_rows[0]["source_report_sha256"] == hashlib.sha256(b2.read_bytes()).hexdigest()
    assert "| B2 | PASS |" in (output / "camping_site_metrics.md").read_text()
    _verify_sha256s(output)


def test_outputs_are_byte_deterministic_and_output_must_be_new(tmp_path):
    report = _report(tmp_path / "b1.json", [_site("B1")])
    summary = metrics.build_summary([report])
    first = tmp_path / "first"
    second = tmp_path / "second"
    metrics.write_summary(first, summary)
    metrics.write_summary(second, summary)

    for name in (
        "camping_site_metrics.json",
        "camping_site_metrics.csv",
        "camping_site_metrics.md",
        "SHA256SUMS",
    ):
        assert (first / name).read_bytes() == (second / name).read_bytes()
    with pytest.raises(metrics.SummaryError, match="already exists"):
        metrics.write_summary(first, summary)


def test_missing_wait_boundary_is_explicitly_unavailable_not_fabricated(tmp_path):
    report = _report(tmp_path / "failure.json", [_site("B3", status="FAIL", waiting=False)])
    summary = metrics.build_summary([report])
    row = summary["sites"][0]

    assert row["total_elapsed_s"] == 30.0
    assert row["total_odom_distance_m"] == 12.0
    assert row["outbound_duration_s"] is None
    assert row["return_duration_s"] is None
    assert row["outbound_distance_m"] is None
    assert row["return_distance_m"] is None
    assert row["split_duration_source"].startswith("UNAVAILABLE:")
    assert summary["aggregate"]["split_duration_complete"] is False
    assert summary["aggregate"]["outbound_duration_s"] is None

    output = tmp_path / "summary"
    metrics.write_summary(output, summary)
    assert "UNAVAILABLE" in (output / "camping_site_metrics.csv").read_text()
    assert "no value was estimated" in (output / "camping_site_metrics.md").read_text()

    missing_measurements = _site("B4", status="FAIL")
    missing_measurements["milestones"][0].pop("elapsed_s")
    missing_measurements["milestones"][0]["observation"].pop("motion_metrics")
    missing_report = _report(tmp_path / "missing-measurements.json", [missing_measurements])
    missing_row = metrics.build_summary([missing_report])["sites"][0]
    assert missing_row["outbound_duration_s"] is None
    assert missing_row["outbound_distance_m"] is None
    assert missing_row["split_duration_source"].startswith("UNAVAILABLE:")


def test_explicit_splits_are_accepted_but_partial_or_inconsistent_splits_reject(tmp_path):
    explicit = _site(
        "B4",
        waiting=False,
        outbound_duration_s=10.0,
        return_duration_s=20.0,
        outbound_distance_m=5.0,
        return_distance_m=7.0,
    )
    report = _report(tmp_path / "explicit.json", [explicit])
    row = metrics.build_summary([report])["sites"][0]
    assert row["split_duration_source"] == "explicit"

    partial = _site("B5", outbound_duration_s=10.0)
    partial_report = _report(tmp_path / "partial.json", [partial])
    with pytest.raises(metrics.SummaryError, match="partial explicit split"):
        metrics.build_summary([partial_report])

    bad = _site(
        "B6",
        outbound_duration_s=10.0,
        return_duration_s=10.0,
        outbound_distance_m=5.0,
        return_distance_m=7.0,
    )
    bad_report = _report(tmp_path / "bad.json", [bad])
    with pytest.raises(metrics.SummaryError, match="inconsistent .* duration split"):
        metrics.build_summary([bad_report])


def test_duplicate_sites_and_inconsistent_configuration_fail_closed(tmp_path):
    first = _report(tmp_path / "one.json", [_site("B7")])
    second = _report(tmp_path / "two.json", [_site("B7")])
    with pytest.raises(metrics.SummaryError, match="duplicate finalized site B7"):
        metrics.build_summary([first, second])

    different = _report(
        tmp_path / "different.json",
        [_site("B8")],
        config_hashes={"camping_sites": "e" * 64, "drop_zones": "b" * 64},
    )
    with pytest.raises(metrics.SummaryError, match="inconsistent active configuration"):
        metrics.build_summary([first, different])


def test_missing_required_data_interrupted_input_and_full_coverage_gate_reject(tmp_path):
    missing_actor = _site("B9")
    missing_actor.pop("actor_id")
    report = _report(tmp_path / "missing.json", [missing_actor])
    with pytest.raises(metrics.SummaryError, match="actor_id must be a positive integer"):
        metrics.build_summary([report])

    interrupted = _report(
        tmp_path / "interrupted.json",
        [{**_site("B10"), "status": "INTERRUPTED"}],
        status="FAIL",
    )
    with pytest.raises(metrics.SummaryError, match="PASS/FAIL/NOT_ATTEMPTED"):
        metrics.build_summary([interrupted])

    partial = _report(tmp_path / "partial.json", [_site("B13")])
    with pytest.raises(metrics.SummaryError, match="coverage is incomplete"):
        metrics.build_summary([partial], require_all_sites=True)


def test_combined_b1_b13_report_passes_full_coverage_and_preserves_hashes(tmp_path):
    sites = [_site(f"B{number}") for number in range(1, 14)]
    report = _report(tmp_path / "all.json", sites)
    summary = metrics.build_summary([report], require_all_sites=True)

    assert summary["coverage"] == "B1_B13_COMPLETE"
    assert summary["aggregate"]["site_count"] == 13
    assert summary["aggregate"]["pass_count"] == 13
    assert summary["aggregate"]["missing_sites"] == []
    assert summary["shared_gate_manifest_sha256"] == GATE_HASH
    assert summary["shared_physical_manifest_sha256"] == PHYSICAL_HASH
    assert {row["gate_manifest_sha256"] for row in summary["sites"]} == {GATE_HASH}
    assert {row["physical_manifest_sha256"] for row in summary["sites"]} == {
        PHYSICAL_HASH
    }
