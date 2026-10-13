"""HH_261007 - Offline-only contracts for the audited CARLA detour clone."""

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from lanelet2.io import Origin, load
from lanelet2.projection import LocalCartesianProjector
from lanelet2.routing import RoutingGraph
from lanelet2.traffic_rules import Locations, Participants, create
import pytest
from shapely.geometry import LineString, Point, Polygon, shape
from shapely.ops import unary_union


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parent
SOURCE = REPO_ROOT / "lanelet2_maps.osm"
AUDIT = (REPO_ROOT / "docs/evidence/virtual_carla"
         / "b1_b11_access_live_20261007/shoulder_candidate_audit.json")
GENERATOR = PACKAGE_ROOT / "tools/generate_object_preserving_detour.py"
TARGET_WAYS = (2755, 2756, 6304, 2256, 2226, 6147, 2449, 2450, 6214)
TARGET_LANELETS = (2757, 2285, 2451)
EXTERNAL_NODES = (2213, 2207, 6201, 2268, 2244, 6213)
NEIGHBOR_LANELETS = (2219, 2292, 2943)
SEAMS = (
    (2219, 2757, (2213, 2207, 6201)),
    (2757, 2285, (2255, 2225, 6141)),
    (2285, 2451, (2260, 2231, 6146)),
    (2451, 2292, (2268, 2244, 6213)),
    (2943, 2292, (2268, 2244, 6213)),
)


def _by_id(root, kind):
    return {int(element.attrib["id"]): element for element in root.findall(kind)}


def _canonical(element):
    return (element.tag, tuple(sorted(element.attrib.items())),
            (element.text or "").strip(),
            tuple(_canonical(child) for child in element))


def _references(way):
    return [int(element.attrib["ref"]) for element in way.findall("nd")]


def _tags(element):
    return {tag.attrib["k"]: tag.attrib["v"] for tag in element.findall("tag")}


def _load_map(path):
    # HH_261007 - Use the runtime projector, not rounded local_x/local_y tags.
    return load(str(path), LocalCartesianProjector(
        Origin(36.8435737, 128.0925646, 0.0)))


def _polygon(lanelet):
    return Polygon([(point.x, point.y) for point in lanelet.leftBound]
                   + [(point.x, point.y) for point in reversed(lanelet.rightBound)])


def _routing_successors(lane_map):
    graph = RoutingGraph(lane_map, create(Locations.Germany, Participants.Vehicle))
    successors = {}
    for lanelet in lane_map.laneletLayer:
        for directed in (lanelet, lanelet.invert()):
            successors[(directed.id, directed.inverted())] = {
                (following.id, following.inverted())
                for following in graph.following(directed, False)
            }
    return successors


@pytest.fixture(scope="module")
def generator():
    spec = importlib.util.spec_from_file_location("object_preserving_detour_test", GENERATOR)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def audited():
    return json.loads(AUDIT.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def generated(generator, tmp_path_factory):
    before_source = SOURCE.read_bytes()
    before_audit = AUDIT.read_bytes()
    output = tmp_path_factory.mktemp("object_preserving_detour") / "carla_only.osm"
    report = generator.generate(SOURCE, AUDIT, output, grid_resolution_m=0.10)
    # HH_261007 - These byte checks cover the entire generation transaction.
    assert SOURCE.read_bytes() == before_source
    assert AUDIT.read_bytes() == before_audit
    assert output.is_file() and output.resolve() != SOURCE.resolve()
    assert isinstance(report, dict)
    return {
        "output": output,
        "report": report,
        "source_root": ET.fromstring(before_source),
        "clone_root": ET.parse(output).getroot(),
        "source_map": _load_map(SOURCE),
        "clone_map": _load_map(output),
    }


def test_generator_scope_constants_are_explicit(generator):
    assert tuple(generator.TARGET_WAYS) == TARGET_WAYS
    assert tuple(generator.TARGET_LANELETS) == TARGET_LANELETS


def test_generation_is_source_isolated_and_returns_a_report(generated, audited):
    assert isinstance(generated["report"], dict)
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == audited["source_map_sha256"]
    assert generated["output"].read_bytes() != SOURCE.read_bytes()


def test_existing_output_is_rejected_without_overwriting(generator, tmp_path):
    output = tmp_path / "existing.osm"
    sentinel = b"HH_261007 - Existing user data must remain intact.\n"
    output.write_bytes(sentinel)
    with pytest.raises(FileExistsError):
        generator.generate(SOURCE, AUDIT, output)
    assert output.read_bytes() == sentinel


def test_source_cannot_be_the_output(generator, tmp_path):
    # HH_261007 - Exercise the overwrite guard on a disposable source copy.
    source = tmp_path / "source.osm"
    original = SOURCE.read_bytes()
    source.write_bytes(original)
    with pytest.raises((FileExistsError, ValueError)):
        generator.generate(source, AUDIT, source)
    assert source.read_bytes() == original


def test_output_symlink_cannot_overwrite_source(generator, tmp_path):
    source = tmp_path / "source.osm"
    original = SOURCE.read_bytes()
    source.write_bytes(original)
    output = tmp_path / "output.osm"
    output.symlink_to(source)
    with pytest.raises((FileExistsError, ValueError)):
        generator.generate(source, AUDIT, output)
    assert output.is_symlink()
    assert source.read_bytes() == original


def test_audit_hash_mismatch_fails_without_an_output(generator, audited, tmp_path):
    invalid = copy.deepcopy(audited)
    invalid["source_map_sha256"] = "0" * 64
    audit = tmp_path / "invalid_hash.json"
    audit.write_text(json.dumps(invalid), encoding="utf-8")
    output = tmp_path / "must_not_exist.osm"
    original = SOURCE.read_bytes()
    with pytest.raises(ValueError):
        generator.generate(SOURCE, audit, output)
    assert not output.exists()
    assert SOURCE.read_bytes() == original


def test_changed_source_fails_the_frozen_audit(generator, tmp_path):
    source = tmp_path / "changed_source.osm"
    modified = SOURCE.read_bytes() + b"\n"
    source.write_bytes(modified)
    output = tmp_path / "must_not_exist.osm"
    with pytest.raises(ValueError):
        generator.generate(source, AUDIT, output)
    assert not output.exists()
    assert source.read_bytes() == modified


def test_unrelated_elements_and_original_ids_are_preserved(generated):
    original = generated["source_root"]
    clone = generated["clone_root"]
    original_nodes, clone_nodes = _by_id(original, "node"), _by_id(clone, "node")
    original_ways, clone_ways = _by_id(original, "way"), _by_id(clone, "way")
    original_relations = _by_id(original, "relation")
    clone_relations = _by_id(clone, "relation")
    assert original.attrib == clone.attrib
    assert original_nodes.keys() <= clone_nodes.keys()
    assert original_ways.keys() == clone_ways.keys()
    assert original_relations.keys() == clone_relations.keys()

    allowed_nodes = set().union(*(_references(original_ways[wid]) for wid in TARGET_WAYS))
    allowed_nodes -= set(EXTERNAL_NODES)
    for node_id, node in original_nodes.items():
        if node_id not in allowed_nodes:
            assert _canonical(clone_nodes[node_id]) == _canonical(node), node_id
        else:
            old_tags = {key: value for key, value in _tags(node).items()
                        if key not in {"local_x", "local_y", "ele"}}
            new_tags = {key: value for key, value in _tags(clone_nodes[node_id]).items()
                        if key not in {"local_x", "local_y", "ele"}}
            assert old_tags == new_tags, node_id

    changed_ways = set()
    for way_id, way in original_ways.items():
        if _canonical(clone_ways[way_id]) != _canonical(way):
            changed_ways.add(way_id)
        if way_id not in TARGET_WAYS:
            assert _canonical(clone_ways[way_id]) == _canonical(way), way_id
        else:
            assert clone_ways[way_id].attrib == way.attrib, way_id
            assert _tags(clone_ways[way_id]) == _tags(way), way_id
    assert changed_ways <= set(TARGET_WAYS)
    assert {6304, 6147, 6214} <= changed_ways
    for relation_id, relation in original_relations.items():
        assert _canonical(clone_relations[relation_id]) == _canonical(relation), relation_id
    for kind in {child.tag for child in original} - {"node", "way", "relation"}:
        assert [_canonical(e) for e in clone.findall(kind)] == [
            _canonical(e) for e in original.findall(kind)]


def test_new_nodes_have_unique_ids_and_are_only_used_by_target_ways(generated):
    original, clone = generated["source_root"], generated["clone_root"]
    nodes, ways = _by_id(clone, "node"), _by_id(clone, "way")
    new_nodes = nodes.keys() - _by_id(original, "node").keys()
    original_ids = {int(element.attrib["id"]) for element in original
                    if element.tag in {"node", "way", "relation"}}
    clone_ids = [int(element.attrib["id"]) for element in clone
                 if element.tag in {"node", "way", "relation"}]
    assert new_nodes
    assert not new_nodes & original_ids
    assert len(clone_ids) == len(set(clone_ids))
    referenced = set()
    for way_id, way in ways.items():
        refs = set(_references(way))
        assert refs <= nodes.keys(), way_id
        if way_id not in TARGET_WAYS:
            assert not refs & new_nodes, way_id
        else:
            referenced |= refs
    assert new_nodes <= referenced


@pytest.mark.parametrize("previous_id,next_id,node_ids", SEAMS)
def test_original_shared_seam_ids_are_retained(generated, previous_id, next_id, node_ids):
    ways = _by_id(generated["clone_root"], "way")
    relations = _by_id(generated["clone_root"], "relation")
    for role, node_id in zip(("left", "right", "centerline"), node_ids):
        relation_ways = []
        for lanelet_id in (previous_id, next_id):
            members = [member for member in relations[lanelet_id].findall("member")
                       if member.attrib["role"] == role]
            assert len(members) == 1
            relation_ways.append(ways[int(members[0].attrib["ref"])])
        assert _references(relation_ways[0])[-1] == node_id
        assert _references(relation_ways[1])[0] == node_id


def test_full_routing_successor_graph_is_unchanged(generated):
    original = _routing_successors(generated["source_map"])
    clone = _routing_successors(generated["clone_map"])
    assert clone == original
    for previous_id, next_id, _ in SEAMS:
        assert (next_id, False) in clone[(previous_id, False)]


def test_loaded_target_centerlines_follow_the_audited_candidate(generated, audited):
    lane_map = generated["clone_map"]
    centerlines = []
    for lanelet_id, centerline_id in zip(TARGET_LANELETS, (6304, 6147, 6214)):
        lanelet = lane_map.laneletLayer[lanelet_id]
        assert lanelet.centerline.id == centerline_id
        polygon = _polygon(lanelet)
        assert polygon.is_valid and polygon.area > 0.0
        centerline = LineString([(point.x, point.y) for point in lanelet.centerline])
        assert centerline.is_simple
        coverage = polygon.buffer(2e-5)
        if lanelet_id == 2451:
            # HH_261007 - Original shared node 6213 is 0.397 mm beyond its
            # original end cap. Preserve that external node exactly; permit
            # only its inherited tiny endpoint discrepancy, not path drift.
            original = generated["source_map"].laneletLayer[lanelet_id]
            endpoint = Point(original.centerline[-1].x, original.centerline[-1].y)
            inherited = endpoint.distance(_polygon(original))
            assert 0.0 < inherited < 0.0005
            coverage = coverage.union(endpoint.buffer(inherited + 2e-5))
        assert coverage.covers(centerline)
        centerlines.append(centerline)
    candidate = unary_union(centerlines)
    assert len(audited["candidate_samples"]) == 721
    distances = [candidate.distance(Point(row["map_xy_m"]))
                 for row in audited["candidate_samples"]]
    assert max(distances) < 0.005


def test_full_audited_planning_footprint_is_contained(generated, audited):
    # HH_261007 - Outer-cap footprints legitimately extend into original roads;
    # do not shrink the audited contour or demand each footprint fit one lanelet.
    clone, original = generated["clone_map"], generated["source_map"]
    polygons = [_polygon(clone.laneletLayer[lanelet_id]) for lanelet_id in TARGET_LANELETS]
    polygons += [_polygon(original.laneletLayer[lanelet_id])
                 for lanelet_id in NEIGHBOR_LANELETS]
    assert all(polygon.is_valid for polygon in polygons)
    corridor = unary_union(polygons)
    swept = shape(audited["swept_gate_polygon"])
    assert swept.is_valid and swept.area > 0.0
    # HH_261007 - Only accommodate the serialized geodetic round-trip error.
    uncovered = swept.difference(corridor.buffer(2e-5))
    assert uncovered.is_empty, f"Uncovered full planning footprint: {uncovered.area:.9f} m^2"


def test_no_new_road_permission_outside_the_finite_audited_corridor(generated, audited):
    # HH_261007 - Existing authored road is retained permission, not a reason
    # to authorize any new shoulder beyond the checked planning sweep margin.
    original = generated["source_map"]
    allowed = unary_union([shape(audited["corridor_polygon"]), *[
        _polygon(original.laneletLayer[i]) for i in TARGET_LANELETS]])
    clone = unary_union([_polygon(generated["clone_map"].laneletLayer[i])
                         for i in TARGET_LANELETS])
    assert allowed.buffer(2e-5).covers(clone)
