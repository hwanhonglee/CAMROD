"""Validate the active map separately from historical runtime evidence."""

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


SRC_ROOT = Path(__file__).resolve().parents[2]
ACTIVE_MAP = SRC_ROOT / "lanelet2_maps.osm"
RUNTIME_REPORT = (
    SRC_ROOT
    / "docs"
    / "assets"
    / "module-guides"
    / "bringup"
    / "evidence"
    / "runtime-capture-20260804"
    / "runtime-visual-capture-20260804.json"
)
ACTIVE_MAP_SHA256 = (
    "8fa13157b8e956559ad29b1bf49b4357ec6d252b0259debfb40a946b29f24e59"
)


def _tags(element: ET.Element) -> dict[str, str]:
    return {
        tag.attrib["k"]: tag.attrib["v"]
        for tag in element.findall("tag")
    }


def test_active_park_map_matches_the_current_user_revision() -> None:
    """Deployment must bind derived configs to the current active map."""
    # HH_260818 - lanelet2_maps.osm is the active source. Named copy files are
    # user-owned snapshots and are intentionally not overwritten or required
    # to match this revision.
    assert hashlib.sha256(ACTIVE_MAP.read_bytes()).hexdigest() == ACTIVE_MAP_SHA256

    root = ET.parse(ACTIVE_MAP).getroot()
    metadata = root.find("MetaInfo")
    assert metadata is not None
    assert metadata.attrib["map_version"] == "22"

    relations = [_tags(relation) for relation in root.findall("relation")]
    assert sum(tags.get("type") == "lanelet" for tags in relations) == 55
    assert sum(tags.get("type") == "multipolygon" for tags in relations) == 14
    assert len(root.findall("node")) == 1652
    assert len(root.findall("way")) == 236


def test_historical_runtime_capture_identifies_map_revision_14() -> None:
    """Historical screenshots and timing data must retain their map input."""
    report = json.loads(RUNTIME_REPORT.read_text(encoding="utf-8"))
    environment = report["environment"]
    containment = report["route_retry_containment"]

    assert environment["map_file"] == ACTIVE_MAP.name
    assert environment["map_version"] == 14
    assert environment["map_commit"] == "95304cfb0"
    assert environment["map_stats"]["lanelets"] == 55
    assert environment["map_stats"]["areas"] == 14
    assert containment["map_v14_recontact_after_release_s"] == 0.275737362
