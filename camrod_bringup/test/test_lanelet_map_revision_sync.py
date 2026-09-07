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
    "2c96514fa788e46ab5061a0ebc130a732557045d0baa3b67bb9f9dbcb132fef7"
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
    assert metadata.attrib["map_version"] == "23"

    relations = [_tags(relation) for relation in root.findall("relation")]
    assert sum(tags.get("type") == "lanelet" for tags in relations) == 55
    assert sum(tags.get("type") == "multipolygon" for tags in relations) == 14
    # Preserve every node in the operator's 1.0.13 snapshot, including nodes
    # that are not members of a current semantic area.
    assert len(root.findall("node")) == 1662
    assert len(root.findall("way")) == 237

    # HH_260907 - The new area is shared by parking and docking. The former
    # drop-zone relation is removed because that space is a vehicle entrance.
    drops = {
        relation.attrib["id"]: _tags(relation)
        for relation in root.findall("relation")
        if _tags(relation).get("subtype") == "drop_zone"
    }
    assert set(drops) == {"7019"}
    assert drops["7019"]["parking_method"] == "auto"
    assert root.find("relation[@id='2320']") is None
    assert root.find("way[@id='2316']") is not None
    assert {tags["yaw_deg"] for tags in drops.values()} == {"-82.2127"}


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
