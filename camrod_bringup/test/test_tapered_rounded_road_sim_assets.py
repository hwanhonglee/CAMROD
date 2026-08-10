"""Keep the measured road simulation visuals tied to current ROS evidence."""

# HH_260810 - Reject stale map/geometry overlays and untraceable road-run
# documentation while keeping physical-road claims explicitly false.

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
    / "render_tapered_rounded_road_sim.py"
)
ASSET_ROOT = (
    SRC_ROOT
    / "docs"
    / "assets"
    / "module-guides"
    / "control"
    / "test-results"
    / "tapered-rounded-boundary-road-sim-20260810"
)
MAP_PATH = SRC_ROOT / "lanelet2_maps.osm"
RUN_NAME = "runtime-b2-recovery.json"
PNG_NAME = "tapered-rounded-boundary-road-sim.png"
GIF_NAME = "tapered-rounded-boundary-road-sim.gif"
SUMMARY_NAME = "road-sim-summary.json"
MANIFEST_NAME = "SHA256SUMS"
MAP_SHA256 = "8cd05c66f846cae8718b5af148d123718f403a086f2e7d16165da89fb625e021"


def _sha256(path: Path) -> str:
    """Return one file SHA-256 digest."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _manifest(path: Path) -> dict[str, str]:
    """Parse one sha256sum-compatible manifest."""
    entries = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        digest, filename = line.split("  ", 1)
        entries[filename] = digest
    return entries


@pytest.fixture(scope="module")
def rendered() -> Path:
    """Return immutable map-v17 artifacts after the active map changed."""
    return ASSET_ROOT


def test_renderer_rejects_current_map_for_the_historical_run(tmp_path: Path) -> None:
    """A current-map edit must not relabel the recorded map-v17 trajectory."""
    result = subprocess.run(
        [
            sys.executable,
            str(RENDERER),
            "--repo-root",
            str(SRC_ROOT),
            "--map",
            str(MAP_PATH),
            "--run",
            str(ASSET_ROOT / RUN_NAME),
            "--output-dir",
            str(tmp_path),
        ],
        cwd=SRC_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "runtime evidence map version does not match selected OSM" in result.stderr


def test_committed_historical_measured_run_is_complete(rendered: Path) -> None:
    """The map-v17 record must retain its documented PNG, GIF, and facts."""
    # HH_260810 - README and immutable raw-run provenance now live beside the
    # derived files. Require the release artifacts without rejecting provenance.
    assert {
        PNG_NAME,
        GIF_NAME,
        SUMMARY_NAME,
        MANIFEST_NAME,
    }.issubset({path.name for path in rendered.iterdir()})
    with Image.open(rendered / PNG_NAME) as image:
        assert image.format == "PNG"
        assert image.size == (2400, 1350)
    with Image.open(rendered / GIF_NAME) as animation:
        assert animation.format == "GIF"
        assert animation.size == (1200, 720)
        assert animation.n_frames == 80
        assert animation.info["duration"] == 120

    summary = json.loads((rendered / SUMMARY_NAME).read_text(encoding="utf-8"))
    assert summary["classification"] == "MEASURED ROS SIM"
    assert summary["field_claim"] is False
    assert summary["map"] == {
        "map_version": 17,
        "sha256": MAP_SHA256,
        "source_file": "lanelet2_maps.osm",
    }
    assert summary["route_lanelet_ids"] == [2751, 2720, 2744, 2690]
    assert summary["pose_samples"] == 511
    assert summary["measured_duration_s"] == pytest.approx(29.700)
    assert summary["automatic_recovery_motion"] == "REVERSE_YAW_RIGHT"
    assert summary["recovery_displacement_m"] == pytest.approx(0.0972)
    assert summary["recovery_yaw_delta_deg"] == pytest.approx(-4.545)
    assert summary["maximum_recovery_output_mps"] == pytest.approx(0.05)
    assert summary["maximum_recovery_abs_angular_z_radps"] == pytest.approx(0.05)
    assert summary["second_hold"] is None
    assert summary["rapid_recontact_latched"] is False
    assert summary["mission_completed"] is True
    assert summary["final_output"] == {
        "angular_z": 0.0,
        "linear_x": 0.0,
        "linear_y": 0.0,
    }


def test_contact_classification_and_shape_are_current(rendered: Path) -> None:
    """The replay must distinguish recoverable margin from physical contact."""
    summary = json.loads((rendered / SUMMARY_NAME).read_text(encoding="utf-8"))
    classification = summary["first_hold_boundary_classification"]
    assert classification["physical_body"]["cost_100_contact"] is False
    assert classification["physical_body"]["maximum_cost"] == 0
    assert classification["physical_body"]["sampled_cells"] == 648
    assert classification["planning_boundary"]["cost_100_contact"] is True
    assert classification["planning_boundary"]["maximum_cost"] == 100
    assert classification["planning_boundary"]["sampled_cells"] == 873

    geometry = summary["geometry"]
    assert geometry["frame"] == "robot_center_link"
    assert geometry["planning_margin_m"] == {
        "front": 0.1,
        "left": 0.1,
        "rear": 0.1,
        "right": 0.1,
    }
    assert geometry["boundary_shape"] == {
        "corner_samples": 4,
        "front_shoulder_depth_m": 0.12,
        "front_taper_m": 0.12,
        "physical_corner_radius_m": 0.05,
        "physical_point_count": 30,
        "planning_corner_radius_m": 0.15,
        "planning_point_count": 30,
    }


def test_committed_summary_manifest_and_document_links(rendered: Path) -> None:
    """Committed artifacts and owning package guides must remain traceable."""
    regenerated = json.loads((rendered / SUMMARY_NAME).read_text(encoding="utf-8"))
    committed = json.loads((ASSET_ROOT / SUMMARY_NAME).read_text(encoding="utf-8"))
    assert committed == regenerated

    manifest = _manifest(ASSET_ROOT / MANIFEST_NAME)
    assert set(manifest) == {RUN_NAME, PNG_NAME, GIF_NAME, SUMMARY_NAME}
    for filename, digest in manifest.items():
        assert _sha256(ASSET_ROOT / filename) == digest

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
    assert "MEASURED ROS SIM" in record
    assert "not physical road evidence" in " ".join(record.split())
