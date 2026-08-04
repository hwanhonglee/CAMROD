"""Keep source-derived package visuals reproducible and reviewable."""

import json
from pathlib import Path
import re
import subprocess
import sys

from PIL import Image


SRC_ROOT = Path(__file__).resolve().parents[2]
RENDERER = (
    SRC_ROOT / "camrod_bringup" / "scripts" / "render_module_readme_assets.py"
)
LOCALIZATION_EVIDENCE = (
    SRC_ROOT
    / "docs"
    / "evidence"
    / "module-guides"
    / "localization"
    / "pose-chain-sim-20260804.json"
)
BRINGUP_EVIDENCE = (
    SRC_ROOT
    / "docs"
    / "evidence"
    / "module-guides"
    / "bringup"
    / "campsite-smoke-20260804.json"
)
FIELD_REPORT = (
    SRC_ROOT
    / "docs"
    / "evidence"
    / "module-guides"
    / "bringup"
    / "field-stationary-20260731.json"
)

CAMROD_READMES = (
    SRC_ROOT / "README.md",
    SRC_ROOT / "camrod_bringup" / "README.md",
    SRC_ROOT / "camrod_common" / "README.md",
    SRC_ROOT / "camrod_common" / "avg_msgs" / "README.md",
    SRC_ROOT / "camrod_control" / "README.md",
    SRC_ROOT / "camrod_localization" / "README.md",
    SRC_ROOT / "camrod_map" / "README.md",
    SRC_ROOT / "camrod_perception" / "README.md",
    SRC_ROOT / "camrod_planning" / "README.md",
    SRC_ROOT / "camrod_platform" / "README.md",
    SRC_ROOT / "camrod_sensing" / "README.md",
    SRC_ROOT / "camrod_sensor_kit" / "README.md",
    SRC_ROOT / "camrod_system" / "README.md",
    SRC_ROOT / "camrod_ui" / "README.md",
    SRC_ROOT / "camrod_voice" / "README.md",
)

VISUAL_DOCS = CAMROD_READMES + (
    SRC_ROOT / "docs" / "MODULE_VISUAL_GUIDE.md",
    SRC_ROOT / "docs" / "V2_1_3_BOUNDARY_RECOVERY_VALIDATION.md",
    SRC_ROOT / "docs" / "V2_1_3_ROBOT_CENTER_MIGRATION.md",
)

MODULE_OWNED_RELEASE_ASSETS = (
    "control/automatic-owner-policy.png",
    "control/automatic-owner-route-retry-contact-sheet.png",
    "control/automatic-owner-route-retry.gif",
    "control/first-route-boundary-stop-location.png",
    "control/pre-owner-manual-no-yaw.gif",
    "control/pre-owner-manual-yaw-aware.gif",
    "control/pre-owner-robot-center-contact-sheet.png",
    "control/pre-owner-robot-center-recovery.gif",
    "planning/robot-center-narrow-route-risk-map.png",
    "sensor-kit/rear-axle-vs-robot-center-drive.gif",
    "ui/guest-mission-dispatch-ready.png",
    "ui/guest-route-safety-hold.png",
)


def test_renderer_recreates_every_documented_asset(tmp_path: Path) -> None:
    """Every README image must be generated from current config and evidence."""
    subprocess.run(
        [
            sys.executable,
            str(RENDERER),
            "--repo-root",
            str(SRC_ROOT),
            "--output-root",
            str(tmp_path),
            "--localization-report",
            str(LOCALIZATION_EVIDENCE),
            "--bringup-report",
            str(BRINGUP_EVIDENCE),
        ],
        cwd=SRC_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    expected_pngs = (
        "bringup/full-stack-mission-contract.png",
        "bringup/field-stationary-report-20260731.png",
        "bringup/simulation-evidence-20260804.png",
        "common/interface-contract-and-dependencies.png",
        "control/command-safety-and-recovery.png",
        "localization/pose-generation-and-timing.png",
        "map/lanelet-map-and-cost-grids.png",
        "planning/nav2-servers-and-mission-states.png",
        "perception/yolo-lidar-and-parking-pipelines.png",
        "platform/ranger-command-and-status.png",
        "sensing/sensor-processing-and-cost-fusion.png",
        "sensing/ground-segmentation-schematic.png",
        "sensor-kit/reference-frame-before-after.png",
        "sensor-kit/sensor-mount-side-view.png",
        "sensor-kit/sensor-x-before-after.png",
        "system/diagnostic-severity-and-surfaces.png",
        "ui/robot-and-guest-mission-state.png",
        "voice/voice-events-and-priority.png",
    )
    for relative_path in expected_pngs:
        with Image.open(tmp_path / relative_path) as image:
            assert image.width >= 2000
            assert image.height >= 1000
            assert image.format == "PNG"

    with Image.open(
        tmp_path / "bringup" / "mission-lifecycle-contract.gif"
    ) as animation:
        assert animation.size == (1200, 480)
        assert animation.n_frames == 10
        assert animation.format == "GIF"


def test_renderer_can_target_one_package(tmp_path: Path) -> None:
    """Package maintainers can regenerate only the assets they own."""
    subprocess.run(
        [
            sys.executable,
            str(RENDERER),
            "--repo-root",
            str(SRC_ROOT),
            "--output-root",
            str(tmp_path),
            "--module",
            "sensing",
        ],
        cwd=SRC_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    generated = sorted(
        path.relative_to(tmp_path).as_posix()
        for path in tmp_path.rglob("*")
        if path.is_file()
    )
    assert generated == [
        "sensing/ground-segmentation-schematic.png",
        "sensing/sensor-processing-and-cost-fusion.png",
    ]


def test_renderer_can_target_sensor_kit_without_runtime_evidence(
    tmp_path: Path,
) -> None:
    """Sensor-kit diagrams need only its package-owned geometry source."""
    subprocess.run(
        [
            sys.executable,
            str(RENDERER),
            "--repo-root",
            str(SRC_ROOT),
            "--output-root",
            str(tmp_path),
            "--module",
            "sensor-kit",
        ],
        cwd=SRC_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    generated = sorted(
        path.relative_to(tmp_path).as_posix()
        for path in tmp_path.rglob("*")
        if path.is_file()
    )
    assert generated == [
        "sensor-kit/reference-frame-before-after.png",
        "sensor-kit/sensor-mount-side-view.png",
        "sensor-kit/sensor-x-before-after.png",
    ]


def test_every_visual_doc_links_package_owned_assets() -> None:
    """Every package/release guide must link existing module-owned visuals."""
    image_pattern = re.compile(r"!\[[^]]*\]\(([^)]+)\)")
    for document in VISUAL_DOCS:
        targets = image_pattern.findall(document.read_text(encoding="utf-8"))
        assert targets, f"{document.relative_to(SRC_ROOT)} has no visual"

        resolved_targets = [
            (document.parent / target).resolve() for target in targets
        ]
        assert all(target.is_file() for target in resolved_targets)
        assert any(
            "docs/assets/module-guides" in target.as_posix()
            for target in resolved_targets
        ), f"{document.relative_to(SRC_ROOT)} has no module-owned visual"


def test_release_visuals_are_decodable_and_owned_by_modules() -> None:
    """Release PNG/GIF evidence must not drift back into a version folder."""
    asset_root = SRC_ROOT / "docs" / "assets" / "module-guides"
    for relative_path in MODULE_OWNED_RELEASE_ASSETS:
        path = asset_root / relative_path
        with Image.open(path) as visual:
            assert visual.width >= 500
            assert visual.height >= 300
            assert visual.format == ("GIF" if path.suffix == ".gif" else "PNG")
            if path.suffix == ".gif":
                assert visual.n_frames > 1

    assert not (SRC_ROOT / "docs" / "assets" / "v2.1.3").exists()


def test_bringup_evidence_references_committed_raw_log_lines() -> None:
    """Every summarized runtime event must resolve to preserved raw evidence."""
    report = json.loads(BRINGUP_EVIDENCE.read_text(encoding="utf-8"))
    references = [
        (reference, "[SYSTEM] OK")
        for reference in report["stack_startup"]["system_ok_evidence"]
    ]
    event_markers = {
        "CRAB_IN started": " CRAB_IN:",
        "route safety hold": "route safety hold activated",
        "crab entry timeout": "crab entry timeout",
    }
    for case in report["cases"]:
        references.extend(
            (event["source"], event_markers[event["event"]])
            for event in case["events"]
        )

    for reference, marker in references:
        relative_path, line_number_text = reference.rsplit(":", 1)
        lines = (SRC_ROOT / relative_path).read_text(encoding="utf-8").splitlines()
        line_number = int(line_number_text)
        assert 1 <= line_number <= len(lines)
        assert marker in lines[line_number - 1]


def test_field_summary_matches_report_and_marks_raw_logs_external() -> None:
    """Reported physical values must stay traceable without claiming raw data."""
    summary = json.loads(FIELD_REPORT.read_text(encoding="utf-8"))
    report = (
        SRC_ROOT / summary["source_report"]
    ).read_text(encoding="utf-8")

    assert summary["raw_files_committed"] is False
    assert f"{summary['radar_disabled']['duration_s']:.3f}" in report
    assert f"{summary['front_camera_yolo']['rate_hz']:.3f} Hz" in report
    assert f"{summary['rear_camera']['raw_rate_hz']:.3f} Hz" in report
    assert f"{summary['localization']['selected_pose_age_p95_ms']:.1f}" in report
    assert f"{summary['resource_profile']['cpu_average_percent']:.2f}%" in report
