# Copyright 2026 hwanhonglee
"""Lock the package and deployed Ranger charging safety parameters."""

from pathlib import Path

import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]


def _parameters(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))["/**"][
        "ros__parameters"
    ]


def test_charging_fast_path_is_narrow_and_mirrored() -> None:
    files = (
        SRC_ROOT / "camrod_platform/config/ranger_driver.yaml",
        SRC_ROOT / "camrod_bringup/config/platform/ranger_driver.yaml",
    )
    profiles = [_parameters(path) for path in files]
    expected = {
        "apriltag_parking_status_topic": (
            "/parking/apriltag_parking_controller/status"
        ),
        "charging_current_threshold_a": 0.3,
        "charging_current_positive_is_charging": True,
        "charging_min_consecutive_samples": 2,
        "charging_confirm_s": 10.0,
        "charging_release_s": 3.0,
        "charging_fast_confirm_s": 1.5,
        "charging_fast_status_ttl_s": 1.5,
        "charging_sample_max_gap_s": 1.0,
        "charging_assertion_false_grace_s": 0.75,
    }
    for profile in profiles:
        assert {key: profile[key] for key in expected} == expected


def test_fast_confirmation_does_not_replace_global_regen_filter() -> None:
    """The fast duration must remain a distinct, more tightly scoped knob."""
    profile = _parameters(
        SRC_ROOT / "camrod_platform/config/ranger_driver.yaml"
    )
    assert profile["charging_confirm_s"] >= 10.0
    assert profile["charging_fast_confirm_s"] == 1.5
    assert profile["charging_release_s"] == 3.0
    assert profile["charging_sample_max_gap_s"] == 1.0
    assert profile["charging_assertion_false_grace_s"] == 0.75
