"""Keep source-derived package visuals reproducible and reviewable."""

import json
from pathlib import Path
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
        "bringup/simulation-evidence-20260804.png",
        "localization/pose-generation-and-timing.png",
        "planning/nav2-servers-and-mission-states.png",
        "perception/yolo-lidar-and-parking-pipelines.png",
        "sensing/sensor-processing-and-cost-fusion.png",
        "sensing/ground-segmentation-schematic.png",
        "sensor-kit/tf-and-sensor-mount-layout.png",
        "sensor-kit/rear-axle-to-center-ledger.png",
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
        "sensor-kit/rear-axle-to-center-ledger.png",
        "sensor-kit/tf-and-sensor-mount-layout.png",
    ]


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
