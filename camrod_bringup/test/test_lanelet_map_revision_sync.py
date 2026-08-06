"""Validate the active map separately from historical runtime evidence."""

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


SRC_ROOT = Path(__file__).resolve().parents[2]
ACTIVE_MAP = SRC_ROOT / "lanelet2_maps.osm"
PARK_MAP_COPY = SRC_ROOT / "lanelet2_maps_(copy_park_v1.0.5).osm"
RUNTIME_REPORT = (
    SRC_ROOT
    / "docs"
    / "evidence"
    / "module-guides"
    / "bringup"
    / "runtime-visual-capture-20260804.json"
)
ACTIVE_MAP_SHA256 = (
    "fd9c1855573784e4e4e952f931c87e3b2c2858fa20f9c8ae5c2ad9adfc32d0cf"
)


def _tags(element: ET.Element) -> dict[str, str]:
    return {
        tag.attrib["k"]: tag.attrib["v"]
        for tag in element.findall("tag")
    }


def test_active_and_named_park_maps_are_revision_16_and_synchronized() -> None:
    """Deployment must not silently use a stale copy of the user map edit."""
    # HH_260806 - Keep this user-authored pair synchronized for the current site.
    # A different field must receive a new revision instead of silently
    # replacing this deployed geometry under the same test contract.
    assert ACTIVE_MAP.read_bytes() == PARK_MAP_COPY.read_bytes()
    assert hashlib.sha256(ACTIVE_MAP.read_bytes()).hexdigest() == ACTIVE_MAP_SHA256

    root = ET.parse(ACTIVE_MAP).getroot()
    metadata = root.find("MetaInfo")
    assert metadata is not None
    assert metadata.attrib["map_version"] == "16"

    relations = [_tags(relation) for relation in root.findall("relation")]
    assert sum(tags.get("type") == "lanelet" for tags in relations) == 55
    assert sum(tags.get("type") == "multipolygon" for tags in relations) == 14
    assert len(root.findall("node")) == 1658


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
