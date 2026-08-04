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
RUNTIME_CAPTURE_REPORT = (
    SRC_ROOT
    / "docs"
    / "evidence"
    / "module-guides"
    / "bringup"
    / "runtime-visual-capture-20260804.json"
)
MAP_V14_RECOVERY_ROOT = (
    SRC_ROOT / "docs" / "evidence" / "v2.1.3" / "map-v14-boundary-recovery"
)
MAP_V14_RECOVERY_SHA256 = (
    "2f69deed24ae47e6762a7653e29e5574438a1ec4b9144b8a3b0a01165f404dbe"
)
RUNTIME_CAPTURE_ASSETS = (
    "bringup/runtime-full-stack-b6-20260804.png",
    "common/runtime-interface-terminal-20260804.png",
    "control/runtime-boundary-retry-latch-20260804.png",
    "control/runtime-retry-latch-terminal-20260804.png",
    "localization/runtime-pose-tf-20260804.png",
    "map/runtime-lanelet-map-20260804.png",
    "perception/runtime-obstacle-bboxes-20260804.png",
    "planning/runtime-b6-global-local-path-20260804.png",
    "platform/runtime-robot-geometry-20260804.png",
    "platform/runtime-status-terminal-20260804.png",
    "sensing/runtime-lidar-radar-costs-20260804.png",
    "sensor-kit/runtime-sensor-tf-20260804.png",
    "system/runtime-health-terminal-20260804.png",
    "voice/runtime-event-terminal-20260804.png",
)
ROBOT_UI_KEYPAD_CAPTURE = "ui/robot-ui-site-verification-keypad.png"

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
    SRC_ROOT / "docs" / "FOR_MERGE_INTEGRATION_20260804.md",
    SRC_ROOT / "docs" / "MODULE_VISUAL_GUIDE.md",
    SRC_ROOT / "docs" / "V2_1_3_BOUNDARY_RECOVERY_VALIDATION.md",
    SRC_ROOT / "docs" / "V2_1_3_ROBOT_CENTER_MIGRATION.md",
)

MODULE_OWNED_RELEASE_ASSETS = (
    "control/automatic-owner-policy.png",
    "control/automatic-owner-route-retry-contact-sheet.png",
    "control/automatic-owner-route-retry.gif",
    "control/first-route-boundary-stop-location.png",
    "control/map-v14-boundary-recovery-contact-sheet.png",
    "control/map-v14-boundary-recovery-policy.png",
    "control/map-v14-boundary-recovery.gif",
    "control/pre-owner-manual-no-yaw.gif",
    "control/pre-owner-manual-yaw-aware.gif",
    "control/pre-owner-robot-center-contact-sheet.png",
    "control/pre-owner-robot-center-recovery.gif",
    "planning/robot-center-narrow-route-risk-map.png",
    "sensor-kit/rear-axle-vs-robot-center-drive.gif",
    "ui/guest-mission-dispatch-ready.png",
    "ui/guest-route-safety-hold.png",
)

README_RUNTIME_ASSETS = {
    SRC_ROOT / "README.md": (RUNTIME_CAPTURE_ASSETS[0],),
    SRC_ROOT / "camrod_bringup" / "README.md": (RUNTIME_CAPTURE_ASSETS[0],),
    SRC_ROOT / "camrod_common" / "README.md": (RUNTIME_CAPTURE_ASSETS[1],),
    SRC_ROOT / "camrod_common" / "avg_msgs" / "README.md": (
        RUNTIME_CAPTURE_ASSETS[1],
    ),
    SRC_ROOT / "camrod_control" / "README.md": RUNTIME_CAPTURE_ASSETS[2:4],
    SRC_ROOT / "camrod_localization" / "README.md": (
        RUNTIME_CAPTURE_ASSETS[4],
    ),
    SRC_ROOT / "camrod_map" / "README.md": (RUNTIME_CAPTURE_ASSETS[5],),
    SRC_ROOT / "camrod_perception" / "README.md": (RUNTIME_CAPTURE_ASSETS[6],),
    SRC_ROOT / "camrod_planning" / "README.md": (RUNTIME_CAPTURE_ASSETS[7],),
    SRC_ROOT / "camrod_platform" / "README.md": RUNTIME_CAPTURE_ASSETS[8:10],
    SRC_ROOT / "camrod_sensing" / "README.md": (RUNTIME_CAPTURE_ASSETS[10],),
    SRC_ROOT / "camrod_sensor_kit" / "README.md": (
        RUNTIME_CAPTURE_ASSETS[11],
    ),
    SRC_ROOT / "camrod_system" / "README.md": (RUNTIME_CAPTURE_ASSETS[12],),
    SRC_ROOT / "camrod_ui" / "README.md": (
        ROBOT_UI_KEYPAD_CAPTURE,
        "ui/guest-mission-dispatch-ready.png",
        "ui/guest-route-safety-hold.png",
    ),
    SRC_ROOT / "camrod_voice" / "README.md": (RUNTIME_CAPTURE_ASSETS[13],),
}


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

    # HH_260805 - Package architecture visuals share layout semantics but use
    # distinct paired accents so the module identity is visible at a glance.
    themed_architectures = (
        "bringup/full-stack-mission-contract.png",
        "common/interface-contract-and-dependencies.png",
        "control/command-safety-and-recovery.png",
        "localization/pose-generation-and-timing.png",
        "map/lanelet-map-and-cost-grids.png",
        "perception/yolo-lidar-and-parking-pipelines.png",
        "planning/nav2-servers-and-mission-states.png",
        "platform/ranger-command-and-status.png",
        "sensing/sensor-processing-and-cost-fusion.png",
        "sensor-kit/reference-frame-before-after.png",
        "system/diagnostic-severity-and-surfaces.png",
        "ui/robot-and-guest-mission-state.png",
        "voice/voice-events-and-priority.png",
    )
    header_pairs = []
    for relative_path in themed_architectures:
        with Image.open(tmp_path / relative_path).convert("RGB") as image:
            pair = (
                image.getpixel((image.width // 4, 5)),
                image.getpixel((image.width * 7 // 8, 5)),
            )
            assert pair[0] != pair[1]
            header_pairs.append(pair)
    assert len({pair[0] for pair in header_pairs}) == len(header_pairs)

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


def test_runtime_captures_are_decodable_and_linked_by_each_package() -> None:
    """Every CAMROD README must expose its actual runtime surface."""
    asset_root = SRC_ROOT / "docs" / "assets" / "module-guides"
    image_pattern = re.compile(r"!\[[^]]*\]\(([^)]+)\)")

    for relative_path in RUNTIME_CAPTURE_ASSETS + (ROBOT_UI_KEYPAD_CAPTURE,):
        with Image.open(asset_root / relative_path) as visual:
            assert visual.width >= 1200
            assert visual.height >= 700
            assert visual.format == "PNG"

    for readme, expected_assets in README_RUNTIME_ASSETS.items():
        linked_assets = {
            (readme.parent / target).resolve()
            for target in image_pattern.findall(readme.read_text(encoding="utf-8"))
        }
        for relative_path in expected_assets:
            assert (asset_root / relative_path).resolve() in linked_assets

    all_visuals = tuple(asset_root.rglob("*"))
    assert sum(path.suffix.lower() == ".png" for path in all_visuals) == 42
    assert sum(path.suffix.lower() == ".gif" for path in all_visuals) == 7


def test_map_v14_recovery_evidence_is_historical_and_fails_closed() -> None:
    """Recovery visuals must stay bound to their historical v14 map input."""
    runs = {
        name: json.loads((MAP_V14_RECOVERY_ROOT / filename).read_text())
        for name, filename in {
            "route": "route-retry.json",
            "reverse": "static-reverse-retry.json",
            "crab": "one-sided-crab.json",
        }.items()
    }

    for run in runs.values():
        assert run["map"]["map_version"] == 14
        assert run["map"]["sha256"] == MAP_V14_RECOVERY_SHA256
        assert run["mission_completed"] is False
        assert run["final_output"] == {
            "linear_x": 0.0,
            "linear_y": 0.0,
            "angular_z": 0.0,
        }

    assert runs["route"]["route_lanelet_ids"] == [754, 2751, 2720]
    assert runs["route"]["automatic_recovery_motion"] == "REVERSE"
    assert runs["route"]["rapid_recontact_latched"] is True
    assert 0.0 < runs["route"]["rapid_recontact_after_release_s"] <= 5.0

    assert runs["reverse"]["automatic_recovery_motion"] == "REVERSE"
    assert runs["reverse"]["rapid_recontact_latched"] is True
    assert runs["reverse"]["recovery_displacement_m"] <= 0.40

    assert runs["crab"]["route_lanelet_ids"] == [4677]
    assert runs["crab"]["automatic_recovery_motion"] == "CRAB_LEFT"
    assert runs["crab"]["rapid_recontact_latched"] is False
    assert runs["crab"]["maximum_recovery_abs_linear_y_mps"] <= 0.05


def test_runtime_capture_metadata_is_complete_and_not_field_evidence() -> None:
    """Live screens must retain traceability and an explicit evidence limit."""
    report = json.loads(RUNTIME_CAPTURE_REPORT.read_text(encoding="utf-8"))
    files = tuple(
        item["file"].split("docs/assets/module-guides/", 1)[1]
        for item in report["screenshots"]
    )

    assert report["environment"]["field_claim"] is False
    assert report["environment"]["map_version"] == 14
    assert report["environment"]["launch_command"].endswith(
        "bringup.launch.py sim:=true rviz:=false"
    )
    assert report["environment"]["rviz_capture_mode"].startswith(
        "separately attached"
    )
    assert sorted(files) == sorted(RUNTIME_CAPTURE_ASSETS)
    assert report["route_retry_containment"]["max_automatic_releases"] == 1
    assert report["route_retry_containment"]["rapid_recontact_window_s"] == 5.0
    assert (
        report["route_retry_containment"]["map_v14_recontact_after_release_s"]
        == 0.275737362
    )
    assert report["operator_stop"]["http_status"] == 200

    raw_excerpt = SRC_ROOT / report["raw_excerpt"]
    raw_text = raw_excerpt.read_text(encoding="utf-8")
    assert "rapid route recontact latched after 1 automatic release" in raw_text
    assert "user-provided lanelet2_maps.osm revision 14" in raw_text
    assert "linear: {x: 0.0, y: 0.0, z: 0.0}" in raw_text
    assert "HTTP 200" in raw_text
    assert "OPERATOR_STOPPED(16)" in raw_text


def test_bringup_docs_reference_only_existing_launch_entrypoint() -> None:
    """Documentation must not reintroduce launch files absent from the package."""
    text = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (SRC_ROOT / "README.md", SRC_ROOT / "camrod_bringup" / "README.md")
    )
    for nonexistent in (
        "bringup_sim.launch.py",
        "bringup_minimal.launch.py",
        "rviz.launch.py",
    ):
        assert nonexistent not in text

    assert "bringup.launch.py sim:=true rviz:=true" in text
    assert (SRC_ROOT / "camrod_bringup" / "launch" / "bringup.launch.py").is_file()


def test_bringup_evidence_is_self_contained_after_raw_log_pruning() -> None:
    """Normalized events must retain provenance after duplicate logs are pruned."""
    report = json.loads(BRINGUP_EVIDENCE.read_text(encoding="utf-8"))

    retention = report["raw_source_retention"]
    assert retention["committed"] is False
    assert retention["removed_at"] == "2026-08-05"
    assert "normalized" in retention["reason"]

    system_records = report["stack_startup"]["system_ok_evidence"]
    assert len(system_records) == 2
    assert all("[SYSTEM] OK" in record for record in system_records)
    assert all("original line" in record for record in system_records)

    expected_events = {
        "CRAB_IN started",
        "route safety hold",
        "crab entry timeout",
    }
    for case in report["cases"]:
        assert {event["event"] for event in case["events"]} == expected_events
        for event in case["events"]:
            assert isinstance(event["stamp"], float)
            assert "original line" in event["source"]
            assert ".log" not in event["source"]
        hold = next(
            event for event in case["events"] if event["event"] == "route safety hold"
        )
        assert hold["reason"] == "lanelet_footprint_cost"


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
