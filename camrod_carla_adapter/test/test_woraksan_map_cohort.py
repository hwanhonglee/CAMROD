"""Contracts for the virtual-only Woraksan CARLA Lanelet2 cohort."""

import importlib.util
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

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
    first_generated_id = max(int(element.attrib["id"]) for element in production_root
                             if element.tag in {"node", "way", "relation"}) + 1
    # Three connector ways add 67 samples. The old B12 four-point correction
    # is retired because develop authored a different centerline and nodes.
    assert generated_node_ids == set(range(first_generated_id, first_generated_id + 67))
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


def test_b12_authored_replacement_is_preserved_without_reviving_deleted_geometry():
    generator = _load_generator_module()
    production_root = ET.parse(PRODUCTION_MAP).getroot()
    virtual_root = ET.parse(VIRTUAL_MAP).getroot()
    assert not generator._legacy_b12_adjustment_enabled(production_root)
    assert virtual_root.find("way[@id='6975']") is None
    authored_way = production_root.find("way[@id='6998']")
    assert authored_way is not None
    assert _canonical(virtual_root.find("way[@id='6998']")) == _canonical(authored_way)
    assert _canonical(virtual_root.find("relation[@id='2744']")) == _canonical(
        production_root.find("relation[@id='2744']"))
    refs = [int(nd.attrib["ref"]) for nd in authored_way.findall("nd")]
    assert 7020 in refs
    for node_id in refs:
        assert _canonical(virtual_root.find(f"node[@id='{node_id}']")) == _canonical(
            production_root.find(f"node[@id='{node_id}']"))
    projector = LocalCartesianProjector(ORIGIN)
    virtual = load(str(VIRTUAL_MAP), projector)
    lanelet = virtual.laneletLayer[2744]
    centerline = _points(lanelet.centerline)
    assert centerline[0].id == 6963
    assert all(
        inside(lanelet, BasicPoint2d(point.x, point.y))
        for point in centerline[1:-1]
    )
    assert max(_heading_jumps(centerline)) < 39.0


def test_generator_recognizes_legacy_b12_only_with_its_original_samples():
    generator = _load_generator_module()
    root = ET.fromstring('''<osm><relation id="2744">
      <member type="way" role="centerline" ref="6975"/></relation>
      <way id="6975"><nd ref="6978"/><nd ref="6977"/>
      <nd ref="6976"/><nd ref="6969"/></way></osm>''')
    assert generator._legacy_b12_adjustment_enabled(root)
    root.find("way").remove(root.find("way/nd"))
    with pytest.raises(RuntimeError, match="review required"):
        generator._legacy_b12_adjustment_enabled(root)


def test_generator_rejects_unknown_replacement_without_inventing_geometry(tmp_path):
    generator = _load_generator_module()
    tree = ET.parse(PRODUCTION_MAP)
    relation = tree.getroot().find("relation[@id='2744']")
    for member in relation.findall("member"):
        if member.get("role") == "centerline":
            member.set("ref", "999999")
    source = tmp_path / "unknown_source.osm"
    output = tmp_path / "must_not_exist.osm"
    tree.write(source)
    with pytest.raises(RuntimeError, match="review required"):
        generator.generate(source, output)
    assert not output.exists()


def test_generated_ids_do_not_collide_with_later_authored_primitive_ids(tmp_path):
    generator = _load_generator_module()
    tree = ET.parse(PRODUCTION_MAP)
    ET.SubElement(tree.getroot(), "relation", {"id": "1000000"})
    source = tmp_path / "higher_relation_id.osm"
    output = tmp_path / "generated.osm"
    tree.write(source)
    generator.generate(source, output)
    generated = ET.parse(output).getroot()
    new_ids = set(_by_id(generated, "node")) - set(_by_id(tree.getroot(), "node"))
    assert new_ids == set(range(1000001, 1000068))
    assert _canonical(generated.find("relation[@id='1000000']")) == _canonical(
        tree.getroot().find("relation[@id='1000000']"))


def test_virtual_map_is_packaged_with_the_adapter():
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    assert 'glob("config/*.osm")' in setup_source


def test_generator_never_expands_drivable_boundaries():
    source = GENERATOR.read_text(encoding="utf-8")
    assert "NARROW_CORRIDOR_HALF_WIDTH_M" not in source
    assert "_offset_corridor" not in source
    assert "install_boundary" not in source
