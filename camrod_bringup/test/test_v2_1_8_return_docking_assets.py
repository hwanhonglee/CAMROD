"""Regression checks for v2.1.8 campsite-return and docking evidence."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

from PIL import Image
import yaml


# HH_260818 - Keep release media tied to a passing machine-readable simulation
# result, synchronized YAML, and an installed reproducible renderer.
REPO_ROOT = Path(__file__).resolve().parents[2]
ASSETS = REPO_ROOT / "docs/assets/module-guides"
BRINGUP = ASSETS / "bringup/test-results/b8-return-docking-20260819"
CONTROL = ASSETS / "control/test-results/campsite-return-docking-20260819"
PLATFORM = ASSETS / "platform/test-results/normal-crab-selection-20260819"
UI = ASSETS / "ui/test-results/docking-workspace-20260819"
UI_PREEMPT = (
    ASSETS
    / "ui/test-results/manual-return-preemption-amd64-20260819"
)


def verify_manifest(directory: Path) -> None:
    entries = (directory / "SHA256SUMS").read_text(encoding="ascii").splitlines()
    assert entries
    for entry in entries:
        digest, filename = entry.split("  ", 1)
        target = directory / filename
        assert target.is_file(), target
        assert hashlib.sha256(target.read_bytes()).hexdigest() == digest


def test_b8_report_and_release_summary_pass() -> None:
    report = json.loads((BRINGUP / "b8-entry-return-report.json").read_text())
    normal = next(
        item for item in report["checks"] if item["name"] == "manual_goal_nav"
    )
    check = next(item for item in report["checks"] if item["name"] == "camping_site_smoke")
    assert report["overall_pass"] is True
    assert normal["success"] is True
    assert normal["metrics"]["normal_motion_mode_ok"] is True
    assert normal["metrics"]["cmd_lateral_max_mps"] <= 0.02
    assert normal["metrics"]["moved_m"] >= 3.5
    assert check["success"] is True
    assert check["metrics"]["site_phase_sequence"] == (
        "CRAB_IN -> ROTATE_180 -> UNLOAD_WAIT -> WAIT_RETURN -> "
        "ALIGN_RETRACE_YAW -> CRAB_OUT -> DONE"
    )
    assert check["metrics"]["rotate_180"] is True
    assert check["metrics"]["crab_out"] is True
    assert check["metrics"]["done"] is True
    assert check["metrics"]["site_path_updates"] >= 2
    assert check["metrics"]["site_path_ok"] is True
    assert check["metrics"]["crab_command_mode_ok"] is True
    for phase in ("CRAB_IN", "CRAB_OUT"):
        command = check["metrics"]["crab_phase_raw_components"][phase]
        assert command["samples"] > 0
        assert command["lateral_samples"] > 0
        assert command["mixed_axis_samples"] == 0
        assert command["angular_z_abs_max_radps"] <= 1.0e-6
        assert command["linear_y_abs_max_mps"] > 0.02
    assert (
        check["metrics"]["crab_phase_raw_components"]["CRAB_IN"]
        ["longitudinal_samples"]
        == 0
    )
    assert check["metrics"]["camping_return_via_ui"] is True
    assert check["metrics"]["manual_return_action"] == "site_exit_then_return"
    assert check["metrics"]["drop_zone_return_before_site_done"] is False
    assert check["metrics"]["manual_return_order_ok"] is True

    summary = json.loads((BRINGUP / "result-summary.json").read_text())
    assert summary["classification"] == "amd64_ros2_simulation"
    assert summary["normal_route_pass"] is True
    assert summary["normal_motion_mode_ok"] is True
    assert summary["normal_route_lateral_max_mps"] <= 0.02
    assert summary["normal_parallel_deadband_mps"] == 0.02
    assert summary["crab_return_timeout_s"] == 90.0
    assert summary["boundary_recovery_maximum_attempts"] == 50
    assert summary["reverse_slowdown_remaining_m"] == 0.30
    assert summary["docking_slowdown_remaining_m"] == 0.60
    assert summary["charging_immediate_stop"] is True
    assert summary["camping_return_via_ui"] is True
    assert summary["manual_return_action"] == "site_exit_then_return"
    assert summary["drop_zone_return_before_site_done"] is False
    assert summary["manual_return_order_ok"] is True
    assert summary["site_path_ok"] is True
    assert summary["crab_command_mode_ok"] is True


def test_release_images_and_animations_are_renderable() -> None:
    images = {
        PLATFORM / "normal-vs-crab-mode-selection.png": (1600, 900),
        CONTROL / "b8-same-anchor-return.png": (1600, 1040),
        CONTROL / "parking-slowdown-profile.png": (1600, 900),
        BRINGUP / "b8-entry-return-summary.png": (1600, 1040),
        UI / "operator-docking-workspace.png": (1600, 1000),
        UI_PREEMPT / "manual-return-preemption.png": (1600, 920),
    }
    for path, expected_size in images.items():
        with Image.open(path) as image:
            assert image.size == expected_size
            image.verify()

    animations = {
        PLATFORM / "normal-vs-crab-mode-selection.gif": 17,
        CONTROL / "b8-entry-return-sequence.gif": 7,
        BRINGUP / "b8-entry-return-sequence.gif": 7,
    }
    for path, minimum_frames in animations.items():
        with Image.open(path) as image:
            assert getattr(image, "n_frames", 1) >= minimum_frames


def test_asset_manifests_and_readmes_are_complete() -> None:
    for directory in (BRINGUP, CONTROL, PLATFORM, UI, UI_PREEMPT):
        assert (directory / "README.md").is_file()
        verify_manifest(directory)


def test_live_manual_return_preemption_result_passes() -> None:
    """The live full graph must stop before publishing one fresh recall."""
    result = json.loads(
        (UI_PREEMPT / "manual-return-preemption.json").read_text()
    )
    assert result["classification"] == "amd64_ros2_isolated_full_graph"
    assert result["pass"] is True
    assert result["outbound_displacement_before_return_m"] >= 2.0
    assert result["first_return_action"] == "return_preempting"
    assert result["second_return_action"] == "return_preempting"
    assert result["third_return_action"] == "return_in_progress"
    assert result["first_zero_delay_s"] <= 0.20
    assert result["hold_window_max_command"] == 0.0
    assert result["recall_count"] == 1
    assert 0.45 <= result["recall_delay_s"] <= 0.75
    assert result["service_state_sequence"] == ["RETURNING_TO_DROP_ZONE"]
    assert result["return_paths"][0]["poses"] > 1
    assert result["return_paths"][0]["length_m"] > 0.0


def test_changed_config_mirrors_are_byte_identical() -> None:
    pairs = (
        (
            REPO_ROOT / "camrod_control/config/control.yaml",
            REPO_ROOT / "camrod_bringup/config/control/control.yaml",
        ),
        (
            REPO_ROOT / "camrod_control/config/parking.yaml",
            REPO_ROOT / "camrod_bringup/config/control/parking.yaml",
        ),
    )
    for source, mirror in pairs:
        assert source.read_bytes() == mirror.read_bytes(), f"{source} != {mirror}"

    # HH_260818 - Ranger keeps an intentional standalone/deployment steering
    # A/B profile. Synchronize only the newly owned normal/crab selector here.
    ranger_files = (
        REPO_ROOT / "camrod_platform/config/ranger_driver.yaml",
        REPO_ROOT / "camrod_bringup/config/platform/ranger_driver.yaml",
    )
    ranger_values = [
        yaml.safe_load(path.read_text())["/**"]["ros__parameters"]
        ["parallel_command_lateral_deadband_mps"]
        for path in ranger_files
    ]
    assert ranger_values == [0.02, 0.02]


def test_historical_renderer_and_live_probe_are_installed() -> None:
    renderer = (
        REPO_ROOT
        / "camrod_bringup/scripts/visualization/render_v2_1_8_return_docking_results.py"
    )
    assert renderer.is_file()
    assert os.access(renderer, os.X_OK)
    cmake = (REPO_ROOT / "camrod_bringup/CMakeLists.txt").read_text()
    assert f"scripts/visualization/{renderer.name}" in cmake
    parameter_reference = (REPO_ROOT / "docs/RUNTIME_PARAMETER_REFERENCE.md").read_text()
    assert "parallel_command_lateral_deadband_mps" in parameter_reference
    assert "`0.30 m` remaining" in parameter_reference
    assert "UI camera range `0.80 -> 0.40 m`" in parameter_reference
    assert "Charging CAN feedback immediately publishes zero" in parameter_reference

    # HH_260819 - Keep the live production-endpoint probe available on AMD64
    # and ARM64 installs; only its measured acceptance result remains host-specific.
    probe = (
        REPO_ROOT
        / "camrod_bringup/scripts/manual_return_preemption_probe.py"
    )
    assert probe.is_file()
    assert os.access(probe, os.X_OK)
    assert f"scripts/{probe.name}" in cmake


def test_sim_platform_gate_keeps_bridge_owned_status_heartbeat() -> None:
    launch_source = (
        REPO_ROOT / "camrod_bringup/launch/_bringup_impl.py"
    ).read_text(encoding="utf-8")
    fake_sensor_block = launch_source.split("fake_sensors_args = {", 1)[1].split(
        "apply_cfg_overrides(fake_sensors_args", 1
    )[0]
    # HH_260818 - The fake node emits hardware-boundary BatteryState while the
    # platform bridge alone owns the normalized status consumed by the gate.
    assert "'publish_simulated_platform_status': 'true'" in fake_sensor_block
