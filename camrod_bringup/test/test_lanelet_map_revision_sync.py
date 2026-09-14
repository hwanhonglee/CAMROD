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
# HH_260909 - Active map advanced to revision 24: one node retired and the
# shared parking/docking station yaw retrimmed to -88.2127 deg.
# HH_260911 - Active map advanced to revision 27: the lanelet 2744 right-turn
# geometry was retightened (centerline way 6998 drops node 6996 and its
# remaining nodes and boundary nodes move in), and the station yaw returns to
# -82.2127 deg.
ACTIVE_MAP_SHA256 = (
    "57cd044cb714f3f4c899868b5287c6c435e14395422e4f75eb66fd8eaa091fbb"
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
    assert metadata.attrib["map_version"] == "27"

    relations = [_tags(relation) for relation in root.findall("relation")]
    assert sum(tags.get("type") == "lanelet" for tags in relations) == 55
    assert sum(tags.get("type") == "multipolygon" for tags in relations) == 14
    # Preserve every node in the operator's 1.0.13 snapshot, including nodes
    # that are not members of a current semantic area.
    # HH_260909 - Revision 24 retires one node from the 1.0.13 snapshot.
    # HH_260911 - Revision 27 retires node 6996 from the lanelet 2744
    # centerline as well.
    assert len(root.findall("node")) == 1660
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
    # HH_260909 - Station yaw retrimmed to -88.2127 deg in the active map.
    # HH_260911 - Revision 27 returns the station yaw to -82.2127 deg.
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
