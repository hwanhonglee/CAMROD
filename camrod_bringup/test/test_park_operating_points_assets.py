"""Validate active Park coordinates separately from archived map-v22 imagery."""

# HH_260810 - Keep area-exported mission coordinates synchronized with the
# exact active OSM while preserving runtime service modes outside the map.
# HH_260818 - Rebind the reproducible asset to map v22; the exported service
# coordinates themselves did not change.

import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys

from PIL import Image
import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
ACTIVE_MAP = SRC_ROOT / "lanelet2_maps.osm"
MAP_SHA256 = "2c96514fa788e46ab5061a0ebc130a732557045d0baa3b67bb9f9dbcb132fef7"
HISTORICAL_MAP_SHA256 = "8fa13157b8e956559ad29b1bf49b4357ec6d252b0259debfb40a946b29f24e59"
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
    assert [(drop["id"], drop["parking_method"]) for drop in drops] == [
        ("dz_area_7019", "auto"),
    ]
    assert drops[0]["x"] == -11.3585
    assert drops[0]["y"] == 40.0901
    # HH_260909 - Station yaw retrimmed to -88.2127 deg.
    # HH_260911 - Map revision 27 returns the station yaw to -82.2127 deg.
    assert drops[0]["yaw_deg"] == -82.2127
    assert [site["type"] for site in sites] == [
        f"camping_site_{index}" for index in range(1, 14)
    ]
    assert [site["service_mode"] for site in sites[:10]] == ["turnaround"] * 10
    assert [site["service_mode"] for site in sites[10:]] == ["roadside_stop"] * 3
    assert sites[0]["x"] == 25.0481
    assert sites[12]["x"] == 0.610449


def test_historical_operating_point_report_retains_map_v22_identity() -> None:
    """Archived source evidence must not claim validation of the active map."""
    report = json.loads((ASSET_ROOT / "park-operating-points.json").read_text())
    assert _sha256(ACTIVE_MAP) == MAP_SHA256
    assert report["map"] == {
        "source_file": "lanelet2_maps.osm",
        "map_version": 22,
        "sha256": HISTORICAL_MAP_SHA256,
        "node_count": 1652,
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


def test_historical_operating_point_visual_is_reproducible(tmp_path: Path) -> None:
    """Render archived inputs only; never overwrite the historical artifacts."""
    # HH_260907 - The active map has a relocated zone and narrower campsite
    # polygons. Use the hash-bound historical snapshot, not current geometry,
    # when checking reproducibility of the existing map-v22 image/report.
    historical_map = SRC_ROOT / "lanelet2_maps_(copy_park_v1.0.11).osm"
    assert _sha256(historical_map) == HISTORICAL_MAP_SHA256
    inputs = tmp_path / "historical_inputs"
    inputs.mkdir()
    shutil.copyfile(historical_map, inputs / "lanelet2_maps.osm")
    historical_report = json.loads((ASSET_ROOT / "park-operating-points.json").read_text())
    for key in ("drop_zones", "camping_sites"):
        # Archived service policy comes from the immutable report; current
        # user-owned YAML variants may represent different operating policies.
        (inputs / f"{key}.yaml").write_text(yaml.safe_dump({key: historical_report[key]}))
    subprocess.run(
        [
            sys.executable,
            str(RENDERER),
            "--map",
            str(inputs / "lanelet2_maps.osm"),
            "--drop-zones",
            str(inputs / "drop_zones.yaml"),
            "--camping-sites",
            str(inputs / "camping_sites.yaml"),
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


def test_active_semantic_geometry_uses_the_shared_local_cartesian_projector() -> None:
    """Bound active centroids/corners to the same loader math as area_exporter."""
    import lanelet2

    params = yaml.safe_load(
        (SRC_ROOT / "camrod_map/config/map_info.yaml").read_text()
    )["/**"]["ros__parameters"]
    projector = lanelet2.projection.LocalCartesianProjector(
        lanelet2.io.Origin(
            params["offset_lat"], params["offset_lon"], params["offset_alt"]
        )
    )
    lanelet_map = lanelet2.io.load(str(ACTIVE_MAP), projector)
    exported = {
        item["id"]: item
        for path, key in ((DROP_FILES[0], "drop_zones"), (SITE_FILES[0], "camping_sites"))
        for item in yaml.safe_load(path.read_text())[key]
    }
    for area in lanelet_map.areaLayer:
        record = exported[f"dz_area_{area.id}"]
        corners = []
        for line in area.outerBound:
            for point in line:
                xy = (point.x, point.y)
                if not corners or math.dist(corners[-1], xy) > 0.10:
                    corners.append(xy)
        if len(corners) > 1 and math.dist(corners[0], corners[-1]) <= 0.10:
            corners.pop()
        pairs = list(zip(corners, corners[1:] + corners[:1]))
        cross = [a[0] * b[1] - b[0] * a[1] for a, b in pairs]
        center = [
            sum((a[index] + b[index]) * c for (a, b), c in zip(pairs, cross))
            / (3.0 * sum(cross))
            for index in (0, 1)
        ]
        assert record["x"] == float(format(center[0], ".6g"))
        assert record["y"] == float(format(center[1], ".6g"))
        assert record["z"] == 0
        assert record["corners"] == [
            {"x": float(format(x, ".6g")), "y": float(format(y, ".6g")), "z": 0}
            for x, y in corners
        ]
        if record["type"] == "drop_zone":
            assert record["parking_method"] == str(area.attributes["parking_method"])
            assert record["yaw_deg"] == float(area.attributes["yaw_deg"])
        else:
            # Narrower polygons can change the inferred longest-edge yaw by
            # 90 degrees. Explicit map metadata must retain operating heading.
            assert record["yaw_deg"] == float(area.attributes["yaw_deg"])
            assert record["service_mode"] == str(area.attributes["service_mode"])
    assert len(exported) == len(lanelet_map.areaLayer) == 14


def test_active_map_changes_only_approved_metadata_and_retired_zone_relation() -> None:
    """Retire relation 2320, retaining all user-authored way/node geometry."""
    import xml.etree.ElementTree as ET

    active = ET.parse(ACTIVE_MAP).getroot()
    snapshot = ET.parse(SRC_ROOT / "lanelet2_maps_(copy_park_v1.0.13).osm").getroot()
    retired = snapshot.find("relation[@id='2320']")
    assert retired is not None
    assert active.find("relation[@id='2320']") is None
    snapshot.remove(retired)
    for relation in active.findall("relation"):
        for tag in list(relation.findall("tag")):
            if tag.attrib["k"] in ("parking_method", "service_mode"):
                relation.remove(tag)
            elif tag.attrib["k"] == "yaw_deg" and any(
                t.attrib == {"k": "subtype", "v": f"camping_site_{index}"}
                for t in relation.findall("tag") for index in range(1, 14)
            ):
                relation.remove(tag)
    # ElementTree retains tag-tail whitespace, so compare element structure
    # and attributes, not incidental serialization indentation.
    def structure(element):
        return element.tag, element.attrib, [structure(child) for child in element]

    assert structure(active) == structure(snapshot)


def test_keypoints_share_the_single_current_parking_and_docking_area() -> None:
    """Both return aliases use area 7019, never the retired vehicle entrance."""
    paths = [
        SRC_ROOT / "camrod_planning/config/planning_state_machine_keypoints.yaml",
        SRC_ROOT / "camrod_bringup/config/planning/planning_state_machine_keypoints.yaml",
    ]
    assert paths[0].read_bytes() == paths[1].read_bytes()
    keypoints = yaml.safe_load(paths[0].read_text())["keypoints"]
    drops = yaml.safe_load(DROP_FILES[0].read_text())["drop_zones"]
    assert len(drops) == 1 and drops[0]["id"] == "dz_area_7019"
    assert set(keypoints) == {"drop_zone", "garage"}
    for key in ("drop_zone", "garage"):
        assert keypoints[key]["frame_id"] == "map"
        assert all(keypoints[key][axis] == drops[0][axis] for axis in ("x", "y", "z", "yaw_deg"))


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
