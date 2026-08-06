"""Lock the active drive-in and constrained-roadside campsite policy."""

import json
from pathlib import Path

import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_CONFIG = SRC_ROOT / "camrod_planning" / "config" / "camping_sites.yaml"
BRINGUP_CONFIG = (
    SRC_ROOT / "camrod_bringup" / "config" / "planning" / "camping_sites.yaml"
)
PACKAGE_COPY_CONFIG = (
    SRC_ROOT / "camrod_planning" / "config" / "camping_sites (copy_park_moved).yaml"
)
BRINGUP_COPY_CONFIG = (
    SRC_ROOT
    / "camrod_bringup"
    / "config"
    / "planning"
    / "camping_sites (copy_park_moved).yaml"
)
EVIDENCE_SUMMARY = (
    SRC_ROOT
    / "docs"
    / "assets"
    / "test_result"
    / "camping-site-sequencing-20260806"
    / "campsite-policy-summary.json"
)


def _sites(path: Path) -> list[dict]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))["camping_sites"]


def test_active_campsite_config_is_byte_synchronized() -> None:
    """Standalone planning and full bringup must select the same maneuver policy."""
    expected = PACKAGE_CONFIG.read_bytes()
    assert BRINGUP_CONFIG.read_bytes() == expected
    # HH_260806 - A manually selected named copy must not restore the unsafe
    # former B12/B13 turnaround/service-pose policy.
    assert PACKAGE_COPY_CONFIG.read_bytes() == expected
    assert BRINGUP_COPY_CONFIG.read_bytes() == expected


def test_drive_in_sites_turn_and_constrained_sites_skip_zero_turn() -> None:
    """B1-B10 turn inside; B11-B13 must never zero-turn at the roadside stop."""
    modes = {
        site["type"]: str(site.get("service_mode", "turnaround")).strip().lower()
        for site in _sites(PACKAGE_CONFIG)
    }

    assert modes
    assert len(modes) == 13
    for index in range(1, 11):
        assert modes[f"camping_site_{index}"] == "turnaround"
    for index in range(11, 14):
        assert modes[f"camping_site_{index}"] == "roadside_stop"


def test_roadside_sites_use_map_centers_without_legacy_service_pose() -> None:
    """A copied B12 service pose must not redirect B11-B13 into unsafe terrain."""
    sites = {site["type"]: site for site in _sites(PACKAGE_CONFIG)}
    for index in range(11, 14):
        site = sites[f"camping_site_{index}"]
        assert not any(key.startswith("service_") and key != "service_mode" for key in site)


def test_checked_in_runtime_evidence_covers_all_thirteen_sites() -> None:
    """Every active site must have direct ordered runtime evidence."""
    summary = json.loads(EVIDENCE_SUMMARY.read_text(encoding="utf-8"))
    runtime = summary["runtime"]
    assert summary["config_contract_pass"] is True
    assert set(runtime) == {f"B{index}" for index in range(1, 14)}

    turnaround_sequence = [
        "CRAB_IN",
        "ROTATE_180",
        "UNLOAD_WAIT",
        "WAIT_RETURN",
        "ALIGN_RETRACE_YAW",
        "CRAB_OUT",
        "DONE",
    ]
    for index in range(1, 11):
        result = runtime[f"B{index}"]
        assert result["pass"] is True
        assert result["service_mode"] == "turnaround"
        assert result["arrival_only"] is False
        assert result["phase_sequence"] == turnaround_sequence
        assert result["rotate_180"] is True
        assert float(result["site_distance_m"]) > 0.0

    roadside_sequence = ["CRAB_IN", "UNLOAD_WAIT", "WAIT_RETURN"]
    for index in range(11, 14):
        result = runtime[f"B{index}"]
        assert result["pass"] is True
        assert result["service_mode"] == "roadside_stop"
        assert result["arrival_only"] is True
        assert result["phase_sequence"] == roadside_sequence
        assert result["rotate_180"] is False
