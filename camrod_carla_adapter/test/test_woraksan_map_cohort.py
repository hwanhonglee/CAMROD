"""Contracts for the virtual-only Woraksan CARLA Lanelet2 cohort."""

import importlib.util
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

from lanelet2.core import BasicPoint2d
from lanelet2.geometry import inside
from lanelet2.io import Origin, load
from lanelet2.projection import LocalCartesianProjector


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent
PRODUCTION_MAP = SOURCE_ROOT / "lanelet2_maps.osm"
VIRTUAL_MAP = PACKAGE_ROOT / "config" / "woraksan_carla_lanelet2.osm"
GENERATOR = (
    PACKAGE_ROOT / "tools" / "generate_woraksan_carla_lanelet_map.py"
)
TARGET_WAYS = {
    6304,
    6147,
    6214,
    6975,
}
MOVED_ENDPOINTS = {6201, 6141, 6146}
PRESERVED_BOUNDARY_WAYS = {
    2755,
    2756,
    2256,
    2226,
    2449,
    2450,
    2742,
    2743,
    2719,
    2716,
}
ORIGIN = Origin(36.8435737, 128.0925646, 0.0)


def _load_generator_module():
    spec = importlib.util.spec_from_file_location(
        "woraksan_carla_map_generator_test", GENERATOR
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _by_id(root, element_type):
    return {
        int(element.attrib["id"]): element
        for element in root.findall(element_type)
    }


def _canonical(element):
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        (element.text or "").strip(),
        tuple(_canonical(child) for child in element),
    )


def _tag_semantics(way):
    return tuple(
        sorted(
            tuple(sorted(tag.attrib.items()))
            for tag in way.findall("tag")
        )
    )


def _heading_jumps(points):
    headings = [
        math.atan2(right.y - left.y, right.x - left.x)
        for left, right in zip(points, points[1:])
    ]
    return [
        abs(
            math.degrees(
                math.atan2(
                    math.sin(right - left), math.cos(right - left)
                )
            )
        )
        for left, right in zip(headings, headings[1:])
    ]


def _points(line_string):
    return [line_string[index] for index in range(len(line_string))]


def _point_to_segment_distance(point, start, end):
    segment_x = end.x - start.x
    segment_y = end.y - start.y
    length_squared = segment_x**2 + segment_y**2
    if length_squared <= 0.0:
        return math.hypot(point.x - start.x, point.y - start.y)
    ratio = (
        (point.x - start.x) * segment_x
        + (point.y - start.y) * segment_y
    ) / length_squared
    ratio = min(1.0, max(0.0, ratio))
    nearest_x = start.x + ratio * segment_x
    nearest_y = start.y + ratio * segment_y
    return math.hypot(point.x - nearest_x, point.y - nearest_y)


def _distance_to_linestring(point, line_string):
    points = _points(line_string)
    return min(
        _point_to_segment_distance(point, start, end)
        for start, end in zip(points, points[1:])
    )


def test_virtual_map_is_a_reproducible_generated_artifact(tmp_path):
    regenerated = tmp_path / VIRTUAL_MAP.name
    subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            "--source",
            str(PRODUCTION_MAP),
            "--output",
            str(regenerated),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    assert regenerated.read_bytes() == VIRTUAL_MAP.read_bytes()


def test_virtual_map_changes_only_the_authorized_centerline_surface():
    production_root = ET.parse(PRODUCTION_MAP).getroot()
    virtual_root = ET.parse(VIRTUAL_MAP).getroot()
    production_nodes = _by_id(production_root, "node")
    virtual_nodes = _by_id(virtual_root, "node")
    production_ways = _by_id(production_root, "way")
    virtual_ways = _by_id(virtual_root, "way")
    production_relations = _by_id(production_root, "relation")
    virtual_relations = _by_id(virtual_root, "relation")

    assert production_nodes.keys() <= virtual_nodes.keys()
    generated_node_ids = set(virtual_nodes) - set(production_nodes)
    assert generated_node_ids == set(range(7000, 7071))
    for node_id, production_node in production_nodes.items():
        if node_id not in MOVED_ENDPOINTS:
            assert _canonical(virtual_nodes[node_id]) == _canonical(
                production_node
            )

    assert virtual_ways.keys() == production_ways.keys()
    for way_id, production_way in production_ways.items():
        if way_id in TARGET_WAYS:
            assert _tag_semantics(virtual_ways[way_id]) == _tag_semantics(
                production_way
            )
        else:
            assert _canonical(virtual_ways[way_id]) == _canonical(
                production_way
            )

    assert virtual_relations.keys() == production_relations.keys()
    for relation_id, production_relation in production_relations.items():
        assert _canonical(virtual_relations[relation_id]) == _canonical(
            production_relation
        )

    for way_id in PRESERVED_BOUNDARY_WAYS:
        assert _canonical(virtual_ways[way_id]) == _canonical(
            production_ways[way_id]
        )


def test_corrected_connector_is_continuous_inside_lanelets_and_smooth():
    virtual = load(str(VIRTUAL_MAP), LocalCartesianProjector(ORIGIN))
    production = load(str(PRODUCTION_MAP), LocalCartesianProjector(ORIGIN))

    corrected = []
    for lanelet_id in (2757, 2285):
        lanelet = virtual.laneletLayer[lanelet_id]
        centerline = _points(lanelet.centerline)
        assert all(
            inside(lanelet, BasicPoint2d(point.x, point.y))
            for point in centerline[1:-1]
        )
        if corrected:
            assert corrected[-1].id == centerline[0].id
            corrected.extend(centerline[1:])
        else:
            corrected.extend(centerline)

    # The first ten 2451 points are the frozen Road54 narrow-lane prefix.
    lanelet_2451 = virtual.laneletLayer[2451]
    prefix = _points(lanelet_2451.centerline)[:10]
    assert all(
        inside(lanelet_2451, BasicPoint2d(point.x, point.y))
        for point in prefix[1:]
    )
    assert corrected[-1].id == prefix[0].id
    corrected.extend(prefix[1:])
    assert max(_heading_jumps(corrected)) < 3.1

    assert virtual.laneletLayer[2757].leftBound[-1].id == (
        virtual.laneletLayer[2285].leftBound[0].id
    )
    assert virtual.laneletLayer[2757].rightBound[-1].id == (
        virtual.laneletLayer[2285].rightBound[0].id
    )
    assert virtual.laneletLayer[2285].leftBound[-1].id == (
        virtual.laneletLayer[2451].leftBound[0].id
    )
    assert virtual.laneletLayer[2285].rightBound[-1].id == (
        virtual.laneletLayer[2451].rightBound[0].id
    )

    production_2757 = _points(production.laneletLayer[2757].centerline)
    production_2285 = _points(production.laneletLayer[2285].centerline)
    original = production_2757 + production_2285[1:]
    assert max(_heading_jumps(original)) > 10.0


def test_corrected_centerlines_retain_the_frozen_xodr_samples():
    generator = _load_generator_module()
    virtual = load(str(VIRTUAL_MAP), LocalCartesianProjector(ORIGIN))
    expected_by_lanelet = {
        2757: generator.WAY_6304_XY,
        2285: generator.WAY_6147_XY,
        2451: generator.WAY_6214_XODR_PREFIX_XY,
    }
    for lanelet_id, expected in expected_by_lanelet.items():
        centerline = _points(virtual.laneletLayer[lanelet_id].centerline)
        for expected_x, expected_y in expected:
            nearest = min(
                math.hypot(point.x - expected_x, point.y - expected_y)
                for point in centerline
            )
            assert nearest < 2.0e-6


def test_b12_return_centerline_moves_inward_without_changing_the_shared_seam():
    generator = _load_generator_module()
    virtual_root = ET.parse(VIRTUAL_MAP).getroot()
    virtual_nodes = _by_id(virtual_root, "node")
    virtual_ways = _by_id(virtual_root, "way")
    refs = [int(nd.attrib["ref"]) for nd in virtual_ways[6975].findall("nd")]

    assert refs == [
        6963,
        7067,
        7068,
        7069,
        7070,
        6970,
        6971,
        6972,
        6973,
        6974,
        5395,
    ]
    projector = LocalCartesianProjector(ORIGIN)
    for generated_id, (_, expected) in zip(
        range(7067, 7071), generator.WAY_6975_INWARD_REPLACEMENTS
    ):
        node = virtual_nodes[generated_id]
        point = projector.forward(
            generator.GPSPoint(
                float(node.attrib["lat"]),
                float(node.attrib["lon"]),
                float(next(
                    tag.attrib["v"]
                    for tag in node.findall("tag")
                    if tag.attrib.get("k") == "ele"
                )),
            )
        )
        assert math.hypot(point.x - expected[0], point.y - expected[1]) < 2.0e-6

    virtual = load(str(VIRTUAL_MAP), projector)
    lanelet = virtual.laneletLayer[2744]
    centerline = _points(lanelet.centerline)
    assert centerline[0].id == 6963
    assert all(
        inside(lanelet, BasicPoint2d(point.x, point.y))
        for point in centerline[1:-1]
    )
    assert max(_heading_jumps(centerline)) < 39.0


def test_virtual_map_is_packaged_with_the_adapter():
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    assert 'glob("config/*.osm")' in setup_source


def test_generator_never_expands_drivable_boundaries():
    source = GENERATOR.read_text(encoding="utf-8")
    assert "NARROW_CORRIDOR_HALF_WIDTH_M" not in source
    assert "_offset_corridor" not in source
    assert "install_boundary" not in source
