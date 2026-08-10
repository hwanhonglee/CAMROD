"""Validate current Park semantic coordinates and their rendered contract."""

# HH_260810 - Keep area-exported mission coordinates synchronized with the
# exact active OSM while preserving runtime service modes outside the map.

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from PIL import Image
import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
ACTIVE_MAP = SRC_ROOT / "lanelet2_maps.osm"
MAP_SHA256 = "689c49854f3e5d93b59ccde13799f9748a669956cf9bbfa7c121f369ecdb1b39"
RENDERER = (
    SRC_ROOT
    / "camrod_bringup"
    / "scripts"
    / "visualization"
    / "render_park_operating_points.py"
)
ASSET_ROOT = (
    SRC_ROOT
    / "docs"
    / "assets"
    / "module-guides"
    / "map"
    / "test-results"
    / "park-operating-points-20260810"
)
DROP_FILES = (
    SRC_ROOT / "camrod_map" / "config" / "drop_zones.yaml",
    SRC_ROOT / "camrod_localization" / "config" / "drop_zones.yaml",
    SRC_ROOT / "camrod_bringup" / "config" / "map" / "drop_zones.yaml",
    SRC_ROOT
    / "camrod_bringup"
    / "config"
    / "localization"
    / "drop_zones.yaml",
)
SITE_FILES = (
    SRC_ROOT / "camrod_planning" / "config" / "camping_sites.yaml",
    SRC_ROOT
    / "camrod_bringup"
    / "config"
    / "planning"
    / "camping_sites.yaml",
)


def _sha256(path: Path) -> str:
    """Return one file SHA-256 digest."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _manifest(path: Path) -> dict[str, str]:
    """Read a sha256sum-compatible manifest."""
    entries = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        digest, filename = line.split("  ", 1)
        entries[filename] = digest
    return entries


def test_current_area_export_configs_are_synchronized() -> None:
    """Every runtime consumer must receive the same Park coordinates."""
    assert len({path.read_bytes() for path in DROP_FILES}) == 1
    assert len({path.read_bytes() for path in SITE_FILES}) == 1

    drops = yaml.safe_load(DROP_FILES[0].read_text())["drop_zones"]
    sites = yaml.safe_load(SITE_FILES[0].read_text())["camping_sites"]
    assert len(drops) == 1
    assert drops[0]["x"] == -14.2347
    assert drops[0]["y"] == 39.7863
    assert drops[0]["yaw_deg"] == -82.2127
    assert [site["type"] for site in sites] == [
        f"camping_site_{index}" for index in range(1, 14)
    ]
    assert [site["service_mode"] for site in sites[:10]] == ["turnaround"] * 10
    assert [site["service_mode"] for site in sites[10:]] == ["roadside_stop"] * 3
    assert sites[0]["x"] == 25.8687
    assert sites[12]["x"] == 0.610449


def test_current_operating_point_report_matches_map_identity() -> None:
    """The committed source report must be bound to the user map revision."""
    report = json.loads((ASSET_ROOT / "park-operating-points.json").read_text())
    assert _sha256(ACTIVE_MAP) == MAP_SHA256
    assert report["map"] == {
        "source_file": "lanelet2_maps.osm",
        "map_version": 15,
        "sha256": MAP_SHA256,
        "node_count": 1592,
        "lanelet_count": 55,
        "semantic_area_count": 14,
    }
    assert report["validation"] == {
        "camping_site_count": 13,
        "drop_zone_count": 1,
        "parking_lot_count": 3,
        "service_modes_preserved": True,
        "runtime_test_required": True,
    }
    assert [item["id"] for item in report["parking_lots"]] == [1146, 1378, 1615]


def test_operating_point_visual_is_reproducible(tmp_path: Path) -> None:
    """Regenerate the current map report and validate committed artifacts."""
    subprocess.run(
        [
            sys.executable,
            str(RENDERER),
            "--output-dir",
            str(tmp_path),
        ],
        cwd=SRC_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    regenerated = json.loads(
        (tmp_path / "park-operating-points.json").read_text(encoding="utf-8")
    )
    committed = json.loads(
        (ASSET_ROOT / "park-operating-points.json").read_text(encoding="utf-8")
    )
    assert regenerated == committed
    with Image.open(tmp_path / "park-operating-points.png") as visual:
        assert visual.format == "PNG"
        assert visual.size == (2400, 1500)

    manifest = _manifest(ASSET_ROOT / "SHA256SUMS")
    assert set(manifest) == {
        "park-operating-points.png",
        "park-operating-points.json",
    }
    for filename, digest in manifest.items():
        assert _sha256(ASSET_ROOT / filename) == digest


def test_map_and_planning_guides_link_the_current_coordinate_report() -> None:
    """Owning package guides must expose the current map-derived result."""
    for document in (
        SRC_ROOT / "camrod_map" / "README.md",
        SRC_ROOT / "camrod_planning" / "README.md",
        SRC_ROOT / "camrod_bringup" / "README.md",
        SRC_ROOT / "docs" / "MODULE_VISUAL_GUIDE.md",
    ):
        text = document.read_text(encoding="utf-8")
        assert "park-operating-points.png" in text, document.relative_to(SRC_ROOT)
