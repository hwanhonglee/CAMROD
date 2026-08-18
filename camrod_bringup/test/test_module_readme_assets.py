"""Keep source-derived package visuals reproducible and reviewable."""

import hashlib
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys

from PIL import Image


SRC_ROOT = Path(__file__).resolve().parents[2]
RENDERER = (
    SRC_ROOT
    / "camrod_bringup"
    / "scripts"
    / "visualization"
    / "render_module_readme_assets.py"
)
AUTOMATIC_RECOVERY_RENDERER = (
    SRC_ROOT
    / "camrod_bringup"
    / "scripts"
    / "visualization"
    / "render_automatic_recovery_results.py"
)
LOCALIZATION_EVIDENCE = (
    SRC_ROOT
    / "docs"
    / "assets"
    / "module-guides"
    / "localization"
    / "evidence"
    / "pose-chain-20260804"
    / "pose-chain-sim-20260804.json"
)
BRINGUP_EVIDENCE = (
    SRC_ROOT
    / "docs"
    / "assets"
    / "module-guides"
    / "bringup"
    / "evidence"
    / "campsite-smoke-20260804"
    / "campsite-smoke-20260804.json"
)
FIELD_REPORT = (
    SRC_ROOT
    / "docs"
    / "assets"
    / "module-guides"
    / "bringup"
    / "evidence"
    / "field-stationary-20260731"
    / "field-stationary-20260731.json"
)
RUNTIME_CAPTURE_REPORT = (
    SRC_ROOT
    / "docs"
    / "assets"
    / "module-guides"
    / "bringup"
    / "evidence"
    / "runtime-capture-20260804"
    / "runtime-visual-capture-20260804.json"
)
MAP_V14_RECOVERY_ROOT = (
    SRC_ROOT
    / "docs/assets/module-guides/control/evidence/map-v14-boundary-recovery"
)
MAP_V14_RECOVERY_SHA256 = (
    "2f69deed24ae47e6762a7653e29e5574438a1ec4b9144b8a3b0a01165f404dbe"
)
MAP_V15_RECOVERY_ROOT = (
    SRC_ROOT
    / "docs/assets/module-guides/control/evidence/map-v15-boundary-recovery"
)
MAP_V15_RECOVERY_SHA256 = (
    "e0b50f09c61fbd5429e528c2b3d8d2799a0dab9f83bb79b06dd0da0403efe36d"
)
RUNTIME_CAPTURE_ASSETS = (
    "bringup/evidence/runtime-capture-20260804/runtime-full-stack-b6-20260804.png",
    "common/evidence/runtime-capture-20260804/runtime-interface-terminal-20260804.png",
    "control/evidence/runtime-capture-20260804/runtime-boundary-retry-latch-20260804.png",
    "control/evidence/runtime-capture-20260804/runtime-retry-latch-terminal-20260804.png",
    "localization/evidence/runtime-capture-20260804/runtime-pose-tf-20260804.png",
    "map/evidence/runtime-capture-20260804/runtime-lanelet-map-20260804.png",
    "perception/evidence/runtime-capture-20260804/runtime-obstacle-bboxes-20260804.png",
    "planning/evidence/runtime-capture-20260804/runtime-b6-global-local-path-20260804.png",
    "platform/evidence/runtime-capture-20260804/runtime-robot-geometry-20260804.png",
    "platform/evidence/runtime-capture-20260804/runtime-status-terminal-20260804.png",
    "sensing/evidence/runtime-capture-20260804/runtime-lidar-radar-costs-20260804.png",
    "sensor-kit/evidence/runtime-capture-20260804/runtime-sensor-tf-20260804.png",
    "system/evidence/runtime-capture-20260804/runtime-health-terminal-20260804.png",
    "voice/evidence/runtime-capture-20260804/runtime-event-terminal-20260804.png",
)
ROBOT_UI_KEYPAD_CAPTURE = (
    "ui/evidence/ui-captures/robot-ui-site-verification-keypad.png"
)
OPERATOR_MANUAL_GOAL_CAPTURE = (
    "ui/evidence/ui-captures/operator-manual-goal-20260810.png"
)
AMD64_TOPOLOGY_CAPTURE = (
    "runtime/test-results/amd64-runtime-topology-20260805/"
    "runtime-topology-amd64-ab-20260805.png"
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
    SRC_ROOT / "camrod_runtime" / "README.md",
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
    SRC_ROOT / "docs" / "V2_1_5_RELEASE_NOTES.md",
    # HH_260810 - The post-tag transport/handoff figures are owned by module
    # result folders and must remain resolvable from the current release notes.
    SRC_ROOT / "docs" / "V2_1_7_RELEASE_NOTES.md",
)

MODULE_OWNED_RELEASE_ASSETS = (
    "control/test-results/automatic-recovery-v2.1.3/automatic-owner-policy.png",
    "control/test-results/automatic-recovery-v2.1.3/automatic-owner-route-retry-contact-sheet.png",
    "control/test-results/automatic-recovery-v2.1.3/automatic-owner-route-retry.gif",
    "control/test-results/map-v14-boundary-recovery/map-v14-boundary-recovery-contact-sheet.png",
    "control/test-results/map-v14-boundary-recovery/map-v14-boundary-recovery-policy.png",
    "control/test-results/map-v14-boundary-recovery/map-v14-boundary-recovery.gif",
    "control/test-results/map-v15-boundary-recovery/map-v15-boundary-recovery-contact-sheet.png",
    "control/test-results/map-v15-boundary-recovery/map-v15-boundary-recovery-policy.png",
    "control/test-results/map-v15-boundary-recovery/map-v15-boundary-recovery.gif",
    "control/test-results/pre-owner-boundary-recovery-20260803/pre-owner-robot-center-contact-sheet.png",
    "control/test-results/pre-owner-boundary-recovery-20260803/pre-owner-robot-center-recovery.gif",
    "planning/test-results/pre-owner-boundary-feasibility-20260803/robot-center-narrow-route-risk-map.png",
    "sensor-kit/test-results/reference-frame-20260803/rear-axle-vs-robot-center-drive.gif",
    OPERATOR_MANUAL_GOAL_CAPTURE,
    "ui/evidence/ui-captures/guest-mission-dispatch-ready.png",
    "ui/evidence/ui-captures/guest-route-safety-hold.png",
)

README_RUNTIME_ASSETS = {
    SRC_ROOT / "README.md": (
        RUNTIME_CAPTURE_ASSETS[0],
        OPERATOR_MANUAL_GOAL_CAPTURE,
    ),
    SRC_ROOT / "camrod_bringup" / "README.md": (
        RUNTIME_CAPTURE_ASSETS[0],
        OPERATOR_MANUAL_GOAL_CAPTURE,
    ),
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
    SRC_ROOT / "camrod_planning" / "README.md": (
        RUNTIME_CAPTURE_ASSETS[7],
        OPERATOR_MANUAL_GOAL_CAPTURE,
    ),
    SRC_ROOT / "camrod_platform" / "README.md": RUNTIME_CAPTURE_ASSETS[8:10],
    SRC_ROOT / "camrod_runtime" / "README.md": (
        "runtime/guide/scoped-component-lifecycle.png",
        "runtime/guide/scoped-component-lifecycle.gif",
    ),
    SRC_ROOT / "camrod_sensing" / "README.md": (RUNTIME_CAPTURE_ASSETS[10],),
    SRC_ROOT / "camrod_sensor_kit" / "README.md": (
        RUNTIME_CAPTURE_ASSETS[11],
    ),
    SRC_ROOT / "camrod_system" / "README.md": (
        RUNTIME_CAPTURE_ASSETS[12],
        AMD64_TOPOLOGY_CAPTURE,
    ),
    SRC_ROOT / "camrod_ui" / "README.md": (
        ROBOT_UI_KEYPAD_CAPTURE,
        OPERATOR_MANUAL_GOAL_CAPTURE,
        "ui/evidence/ui-captures/guest-mission-dispatch-ready.png",
        "ui/evidence/ui-captures/guest-route-safety-hold.png",
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
        "bringup/guide/full-stack-mission-contract.png",
        "bringup/test-results/field-stationary-20260731/field-stationary-report-20260731.png",
        "bringup/guide/package-technology-evidence.png",
        "bringup/test-results/campsite-smoke-20260804/simulation-evidence-20260804.png",
        "common/guide/interface-contract-and-dependencies.png",
        "control/guide/command-safety-and-recovery.png",
        "localization/guide/pose-generation-and-timing.png",
        "map/guide/lanelet-map-and-cost-grids.png",
        "planning/guide/nav2-servers-and-mission-states.png",
        "perception/guide/yolo-lidar-and-parking-pipelines.png",
        "platform/guide/ranger-command-and-status.png",
        "runtime/guide/scoped-component-lifecycle.png",
        "sensing/guide/sensor-processing-and-cost-fusion.png",
        "sensing/guide/ground-segmentation-schematic.png",
        "sensor-kit/guide/reference-frame-before-after.png",
        "sensor-kit/guide/sensor-mount-side-view.png",
        "sensor-kit/guide/sensor-x-before-after.png",
        "system/guide/diagnostic-severity-and-surfaces.png",
        "ui/guide/robot-and-guest-mission-state.png",
        "ui/test-results/operator-telemetry-amd64-20260810/operator-telemetry-resource-profile.png",
        "voice/guide/voice-events-and-priority.png",
    )
    for relative_path in expected_pngs:
        with Image.open(tmp_path / relative_path) as image:
            assert image.width >= 2000
            assert image.height >= 1000
            assert image.format == "PNG"

    # HH_260805 - Package architecture visuals share layout semantics but use
    # distinct paired accents so the module identity is visible at a glance.
    themed_architectures = (
        "bringup/guide/full-stack-mission-contract.png",
        "common/guide/interface-contract-and-dependencies.png",
        "control/guide/command-safety-and-recovery.png",
        "localization/guide/pose-generation-and-timing.png",
        "map/guide/lanelet-map-and-cost-grids.png",
        "perception/guide/yolo-lidar-and-parking-pipelines.png",
        "planning/guide/nav2-servers-and-mission-states.png",
        "platform/guide/ranger-command-and-status.png",
        "runtime/guide/scoped-component-lifecycle.png",
        "sensing/guide/sensor-processing-and-cost-fusion.png",
        "sensor-kit/guide/reference-frame-before-after.png",
        "system/guide/diagnostic-severity-and-surfaces.png",
        "ui/guide/robot-and-guest-mission-state.png",
        "voice/guide/voice-events-and-priority.png",
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
        tmp_path / "bringup" / "guide" / "mission-lifecycle-contract.gif"
    ) as animation:
        assert animation.size == (1200, 480)
        assert animation.n_frames == 10

    with Image.open(
        tmp_path
        / "ui/test-results/operator-telemetry-amd64-20260810/operator-telemetry-workspace.gif"
    ) as animation:
        assert animation.size == (1280, 800)
        assert animation.n_frames == 6
        assert animation.format == "GIF"
    with Image.open(
        tmp_path / "bringup" / "guide" / "package-technology-evidence.gif"
    ) as animation:
        assert animation.size == (1200, 720)
        assert animation.n_frames == 14
        assert animation.format == "GIF"
    with Image.open(
        tmp_path / "runtime" / "guide" / "scoped-component-lifecycle.gif"
    ) as animation:
        assert animation.size == (1200, 480)
        assert animation.n_frames == 7
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
        "sensing/guide/ground-segmentation-schematic.png",
        "sensing/guide/sensor-processing-and-cost-fusion.png",
    ]


def test_visualization_tool_ownership_is_explicit() -> None:
    """Offline renderers are central while operational tools remain local."""
    scripts = SRC_ROOT / "camrod_bringup" / "scripts"
    visualization = scripts / "visualization"
    assert list(scripts.glob("render_*.py")) == []

    expected = {
        "render_automatic_recovery_results.py",
        "render_boundary_recovery_results.py",
        "render_camping_site_sequence_results.py",
        "render_historical_boundary_summaries.py",
        "render_module_readme_assets.py",
        "render_operator_transport_handoff_results.py",
        "render_park_operating_points.py",
        # HH_260819 - Keep the measured Return/lease A/B reproducible beside
        # the current campsite renderer instead of hand-editing its chart.
        "render_return_resource_profile.py",
        "render_robot_boundary_validation.py",
        "render_rpp_service_ab.py",
        "render_tapered_rounded_boundary.py",
        "render_tapered_rounded_road_sim.py",
        "render_v2_1_5_service_results.py",
        "render_v2_1_8_return_docking_results.py",
    }
    renderers = tuple(visualization.glob("render_*.py"))
    assert {path.name for path in renderers} == expected
    # HH_260810 - Symlink installs inherit source mode; a non-executable source
    # silently produces a listed but unusable ros2-run entrypoint.
    assert all(path.stat().st_mode & 0o111 for path in renderers)

    cmake = (SRC_ROOT / "camrod_bringup" / "CMakeLists.txt").read_text()
    for filename in expected:
        assert f"scripts/visualization/{filename}" in cmake

    assert (SRC_ROOT / "camrod_planning/scripts/path_visualizer_node.py").is_file()
    assert (SRC_ROOT / "camrod_sensing/scripts/radar_status_gui.py").is_file()
    assert (SRC_ROOT / "camrod_platform/scripts/velocity_kph_gui.py").is_file()

    ownership = (visualization / "README.md").read_text(encoding="utf-8")
    assert "Files under repository `util/`" in ownership
    assert "ros2 run camrod_bringup" in ownership


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
        "sensor-kit/guide/gnss-left-antenna-lever-arm.png",
        "sensor-kit/guide/reference-frame-before-after.png",
        "sensor-kit/guide/sensor-mount-side-view.png",
        "sensor-kit/guide/sensor-x-before-after.png",
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


def test_map_v15_release_recovery_visuals_are_hash_guarded(tmp_path: Path) -> None:
    """Released recovery evidence must reject the later deployed map geometry."""
    # HH_260818 - Historical map-v15 evidence must reject both a newer map
    # version and a same-version file with a different source hash.
    active_map = SRC_ROOT / "lanelet2_maps.osm"
    active_sha256 = hashlib.sha256(active_map.read_bytes()).hexdigest()
    assert active_sha256 != MAP_V15_RECOVERY_SHA256

    result = subprocess.run(
        [
            sys.executable,
            str(AUTOMATIC_RECOVERY_RENDERER),
            "--map",
            str(active_map),
            "--route",
            str(MAP_V15_RECOVERY_ROOT / "route-retry.json"),
            "--reverse",
            str(MAP_V15_RECOVERY_ROOT / "static-reverse-retry.json"),
            "--crab",
            str(MAP_V15_RECOVERY_ROOT / "one-sided-crab.json"),
            "--artifact-prefix",
            "map-v15-boundary-recovery",
            "--output-dir",
            str(tmp_path),
        ],
        cwd=SRC_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert (
        "evidence map v15 does not match OSM map v22" in result.stderr
        or "evidence OSM hash does not match the selected map" in result.stderr
    )


def test_recovery_renderer_uses_captured_geometry_without_relabeling_history() -> None:
    """New 10 cm runs and metadata-free release runs keep distinct envelopes."""
    spec = importlib.util.spec_from_file_location(
        "camrod_automatic_recovery_renderer", AUTOMATIC_RECOVERY_RENDERER
    )
    assert spec is not None and spec.loader is not None
    renderer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(renderer)

    metadata_free_runs = ({}, {}, {})
    assert renderer.evidence_planning_extents(metadata_free_runs, 15) == (
        0.80837,
        0.78323,
        0.58505,
        0.58495,
    )

    current_geometry = {
        "geometry": {
            "planning_boundary_extents_m": {
                "front": 0.80837,
                "rear": 0.78323,
                "left": 0.63505,
                "right": 0.63495,
            }
        }
    }
    assert renderer.evidence_planning_extents(
        (current_geometry, current_geometry, current_geometry), 17
    ) == (0.80837, 0.78323, 0.63505, 0.63495)


def test_runtime_captures_are_decodable_and_linked_by_each_package() -> None:
    """Every CAMROD README must expose its actual runtime surface."""
    asset_root = SRC_ROOT / "docs" / "assets" / "module-guides"
    image_pattern = re.compile(r"!\[[^]]*\]\(([^)]+)\)")

    for relative_path in RUNTIME_CAPTURE_ASSETS + (
        ROBOT_UI_KEYPAD_CAPTURE,
        AMD64_TOPOLOGY_CAPTURE,
    ):
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
    # HH_260810 - Package-owned test results may grow without invalidating the
    # runtime-capture contract. Keep the established inventory as a floor.
    png_count = sum(path.suffix.lower() == ".png" for path in all_visuals)
    gif_count = sum(path.suffix.lower() == ".gif" for path in all_visuals)
    assert png_count >= 48
    assert gif_count >= 10

    # HH_260810 - Keep the human-readable inventory exact as package-owned
    # test-result media grows; stale totals make an otherwise valid index hard
    # to audit from another workstation.
    visual_guide = (SRC_ROOT / "docs" / "MODULE_VISUAL_GUIDE.md").read_text(
        encoding="utf-8"
    )
    assert f"**{png_count} PNGs and {gif_count} GIFs**" in visual_guide


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


def test_map_v15_release_evidence_exercises_staged_yaw_and_crab() -> None:
    """Released map-v15 visuals must retain their adaptive runtime evidence."""
    runs = {
        name: json.loads((MAP_V15_RECOVERY_ROOT / filename).read_text())
        for name, filename in {
            "route": "route-retry.json",
            "reverse": "static-reverse-retry.json",
            "crab": "one-sided-crab.json",
        }.items()
    }

    for run in runs.values():
        assert run["map"]["map_version"] == 15
        assert run["map"]["sha256"] == MAP_V15_RECOVERY_SHA256
        assert run["mission_completed"] is False
        assert run["final_output"] == {
            "linear_x": 0.0,
            "linear_y": 0.0,
            "angular_z": 0.0,
        }
        assert run["maximum_recovery_output_mps"] <= 0.05
        assert run["maximum_recovery_abs_angular_z_radps"] <= 0.05

    assert runs["route"]["automatic_recovery_motion"] == "REVERSE_YAW_RIGHT"
    assert runs["route"]["rapid_recontact_latched"] is True
    assert 0.0 < runs["route"]["rapid_recontact_after_release_s"] <= 5.0
    assert runs["route"]["recovery_displacement_m"] <= 0.40

    assert runs["reverse"]["automatic_recovery_motion"] == "REVERSE_YAW_RIGHT"
    assert runs["reverse"]["rapid_recontact_latched"] is True
    assert runs["reverse"]["recovery_displacement_m"] <= 0.40

    assert runs["crab"]["automatic_recovery_motion"] == "CRAB_LEFT"
    assert runs["crab"]["maximum_recovery_abs_linear_y_mps"] <= 0.05
    assert runs["crab"]["rapid_recontact_latched"] is False


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
