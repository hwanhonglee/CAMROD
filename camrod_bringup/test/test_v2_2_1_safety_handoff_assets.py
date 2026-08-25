"""Regression checks for the v2.2.1 safety-handoff release record."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from PIL import Image
import yaml


# HH_260825 - Bind the operator-facing v2.2.1 claims to measured AMD64 data,
# synchronized deployment YAML, and explicit field-acceptance limitations.
REPO_ROOT = Path(__file__).resolve().parents[2]
RESULT_DIR = (
    REPO_ROOT
    / "docs/assets/module-guides/bringup/test-results"
    / "v2-2-1-safety-handoff-20260825"
)


def test_measured_safety_handoff_result_passes() -> None:
    result = json.loads((RESULT_DIR / "result.json").read_text(encoding="utf-8"))

    assert result["schema"] == "camrod.v2_2_1.safety_handoff_sim.v1"
    assert result["environment"]["architecture"] == "amd64"
    assert result["environment"]["field_or_hardware_acceptance"] is False

    handoff = result["campsite_live_lanelet_handoff"]
    assert handoff["status"] == "PASS"
    assert handoff["site"] == "B8"
    assert handoff["projection_distance_m"] <= handoff[
        "configured_maximum_distance_m"
    ]
    assert handoff["historical_entry_anchor_error_at_candidate_m"] > handoff[
        "configured_maximum_distance_m"
    ]
    assert handoff["stationary_hold_s"] == 1.2
    assert handoff["return_request_source"] == "done_live_lanelet"

    departure = result["charging_departure_dwell"]
    assert departure["status"] == "PASS"
    assert departure["configured_delay_s"] == 7.0
    assert 6.8 <= departure["observed_delay_s"] <= 7.2
    assert not any(departure["authorization_during_dwell"].values())
    assert departure["release"]["parking_cancel_count"] == 1
    assert departure["release"]["drop_zone_exit_count"] == 1
    assert departure["release"]["site_goal_before_exit_complete"] is False

    radar = result["front_radar_near_field"]
    assert radar["status"] == "PASS"
    assert radar["injected_channel"] == "FRONT1"
    assert radar["injected_range_m"] == 0.3
    assert radar["published_cost"] == 95
    assert radar["other_channel_usable_window_m"] == 0.1


def test_release_media_and_claim_boundary_are_present() -> None:
    for filename in (
        "README.md",
        "result.json",
        "v2-2-1-safety-handoff-summary.svg",
    ):
        assert (RESULT_DIR / filename).is_file()

    with Image.open(RESULT_DIR / "v2-2-1-safety-handoff-summary.png") as image:
        assert image.size == (1400, 820)
        image.verify()

    readme = (RESULT_DIR / "README.md").read_text(encoding="utf-8")
    assert "Physical radar multipath" in readme
    assert "ARM64 resource use remain field work" in readme

    # HH_260825 - Detect stale hand-edited claims or regenerated media before
    # the operator-facing evidence is published under a release tag.
    sums = (RESULT_DIR / "SHA256SUMS").read_text(encoding="ascii").splitlines()
    assert len(sums) == 4
    for entry in sums:
        digest, filename = entry.split("  ", 1)
        actual = hashlib.sha256((RESULT_DIR / filename).read_bytes()).hexdigest()
        assert actual == digest


def test_v2_2_1_config_mirrors_and_release_docs_stay_synchronized() -> None:
    mirror_pairs = (
        (
            "camrod_control/config/control.yaml",
            "camrod_bringup/config/control/control.yaml",
        ),
        (
            "camrod_control/config/cmd_vel_safety_gate.yaml",
            "camrod_bringup/config/control/cmd_vel_safety_gate.yaml",
        ),
        (
            "camrod_sensing/config/radar/cost_grid.yaml",
            "camrod_bringup/config/sensing/radar/cost_grid.yaml",
        ),
        (
            "camrod_sensing/config/radar/sen0592_radar.yaml",
            "camrod_bringup/config/sensing/radar/sen0592_radar.yaml",
        ),
    )
    for package_path, bringup_path in mirror_pairs:
        assert (REPO_ROOT / package_path).read_bytes() == (
            REPO_ROOT / bringup_path
        ).read_bytes()

    defaults = yaml.safe_load(
        (REPO_ROOT / "camrod_bringup/config/bringup/launch_defaults.yaml").read_text(
            encoding="utf-8"
        )
    )["bringup"]
    assert defaults["system"]["api_ui_charging_departure_delay_s"] == 7.0

    gate = yaml.safe_load(
        (REPO_ROOT / "camrod_control/config/cmd_vel_safety_gate.yaml").read_text(
            encoding="utf-8"
        )
    )["/**"]["ros__parameters"]
    assert gate["front_dynamic_stop_use_local_path"] is True
    assert gate["front_dynamic_path_width_m"] == 1.27
    assert "radar" in gate["cost_stop_dynamic_source_labels"].split(",")

    release_notes = (REPO_ROOT / "docs/V2_2_1_RELEASE_NOTES.md").read_text(
        encoding="utf-8"
    )
    for token in (
        "return_lanelet_handoff_distance_m",
        "api_ui_charging_departure_delay_s",
        "stop_candidate_max_ranges_m",
        "ARM64 8-core/16-GB",
    ):
        assert token in release_notes
