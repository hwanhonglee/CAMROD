"""Keep the active tapered/rounded boundary visuals source synchronized."""

# HH_260810 - Re-render the current contour and reject stale documentation,
# malformed animation output, or Nav2 local/global footprint drift.

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from PIL import Image
import pytest


SRC_ROOT = Path(__file__).resolve().parents[2]
RENDERER = (
    SRC_ROOT
    / "camrod_bringup"
    / "scripts"
    / "visualization"
    / "render_tapered_rounded_boundary.py"
)
ASSET_ROOT = (
    SRC_ROOT
    / "docs"
    / "assets"
    / "test_result"
    / "tapered-rounded-boundary-20260810"
)
PNG_NAME = "tapered-rounded-boundary-geometry.png"
GIF_NAME = "tapered-rounded-boundary-motion.gif"
RESULT_NAME = "result.json"
MANIFEST_NAME = "SHA256SUMS"


def _render(output_dir: Path) -> None:
    """Run the production documentation renderer into a temporary directory."""
    subprocess.run(
        [
            sys.executable,
            str(RENDERER),
            "--repo-root",
            str(SRC_ROOT),
            "--output-dir",
            str(output_dir),
        ],
        cwd=SRC_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )


def _sha256(path: Path) -> str:
    """Return one file SHA-256 digest."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _manifest(path: Path) -> dict[str, str]:
    """Parse a sha256sum-compatible manifest."""
    entries = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        digest, filename = line.split("  ", 1)
        entries[filename] = digest
    return entries


def test_renderer_recreates_current_geometry_and_motion(tmp_path: Path) -> None:
    """Generated visuals must retain exact dimensions and four motion phases."""
    _render(tmp_path)

    assert {path.name for path in tmp_path.iterdir()} == {
        PNG_NAME,
        GIF_NAME,
        RESULT_NAME,
        MANIFEST_NAME,
    }
    with Image.open(tmp_path / PNG_NAME) as image:
        assert image.format == "PNG"
        assert image.size == (2400, 1350)
    with Image.open(tmp_path / GIF_NAME) as animation:
        assert animation.format == "GIF"
        assert animation.size == (1200, 720)
        assert animation.n_frames == 48
        assert animation.info["duration"] == 120

    result = json.loads((tmp_path / RESULT_NAME).read_text(encoding="utf-8"))
    assert result["classification"] == "SOURCE-DERIVED"
    assert result["runtime_claim"] is False
    assert result["field_claim"] is False
    assert result["frame"] == "robot_center_link"
    assert result["physical_boundary"]["bounding_length_m"] == pytest.approx(
        1.39160
    )
    assert result["physical_boundary"]["bounding_width_m"] == pytest.approx(
        1.07000
    )
    assert result["physical_boundary"]["front_taper_m"] == pytest.approx(0.12)
    assert result["physical_boundary"]["front_shoulder_depth_m"] == pytest.approx(
        0.12
    )
    assert result["physical_boundary"]["corner_radius_m"] == pytest.approx(0.05)
    assert result["physical_boundary"]["point_count"] == 30
    assert result["planning_boundary"]["bounding_length_m"] == pytest.approx(
        1.59160
    )
    assert result["planning_boundary"]["bounding_width_m"] == pytest.approx(
        1.27000
    )
    assert result["planning_boundary"]["corner_radius_m"] == pytest.approx(0.15)
    assert result["planning_boundary"]["point_count"] == 30
    assert result["planning_boundary"]["nav2_local_match"] is True
    assert result["planning_boundary"]["nav2_global_match"] is True
    assert [
        phase["name"] for phase in result["motion_visualization"]["phases"]
    ] == ["FORWARD", "CURVED DRIVE", "CRAB RIGHT", "ZERO TURN"]


def test_committed_assets_match_current_sources_and_manifest(tmp_path: Path) -> None:
    """A geometry/config edit must require regenerating the committed record."""
    _render(tmp_path)

    regenerated = json.loads((tmp_path / RESULT_NAME).read_text(encoding="utf-8"))
    committed = json.loads((ASSET_ROOT / RESULT_NAME).read_text(encoding="utf-8"))
    assert committed == regenerated

    manifest = _manifest(ASSET_ROOT / MANIFEST_NAME)
    assert set(manifest) == {PNG_NAME, GIF_NAME, RESULT_NAME}
    for filename, expected_digest in manifest.items():
        assert _sha256(ASSET_ROOT / filename) == expected_digest


def test_package_guides_link_the_new_geometry_and_motion_assets() -> None:
    """Every package that owns or consumes the contour must expose the visuals."""
    documents = (
        SRC_ROOT / "README.md",
        SRC_ROOT / "camrod_bringup" / "README.md",
        SRC_ROOT / "camrod_control" / "README.md",
        SRC_ROOT / "camrod_planning" / "README.md",
        SRC_ROOT / "camrod_platform" / "README.md",
        SRC_ROOT / "camrod_sensor_kit" / "README.md",
        SRC_ROOT / "docs" / "MODULE_VISUAL_GUIDE.md",
    )
    for document in documents:
        text = document.read_text(encoding="utf-8")
        assert PNG_NAME in text, document.relative_to(SRC_ROOT)
        assert GIF_NAME in text, document.relative_to(SRC_ROOT)

    record = (ASSET_ROOT / "README.md").read_text(encoding="utf-8")
    assert "SOURCE-DERIVED" in record
    assert "not a ROS simulation" in record
    assert PNG_NAME in record
    assert GIF_NAME in record
