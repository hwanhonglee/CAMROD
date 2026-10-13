"""HH_261007 - Reproduce narrowly scoped CARLA access geometry without motion."""

import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from lanelet2.io import Origin, load
from lanelet2.projection import LocalCartesianProjector
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parent
ACCESS_PATH = PACKAGE_ROOT / "config" / "manual_service_access.yaml"
SITE_IDS = ("camping_site_1", "camping_site_11")


def _xy(point):
    return point["x"], point["y"]


def _tags(element):
    return {tag.attrib["k"]: tag.attrib["v"] for tag in element.findall("tag")}


def _edges(polygon):
    return zip(polygon, polygon[1:] + polygon[:1])


def _cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _project(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    norm2 = dx * dx + dy * dy
    assert norm2 > 0.0
    ratio = max(0.0, min(1.0, ((point[0] - start[0]) * dx
                             + (point[1] - start[1]) * dy) / norm2))
    return start[0] + ratio * dx, start[1] + ratio * dy


def _inside(point, polygon):
    # HH_261007 - Include exact boundaries; road polygons need not be convex.
    inside = False
    for start, end in _edges(polygon):
        if math.dist(point, _project(point, start, end)) < 1e-9:
            return True
        if (start[1] > point[1]) != (end[1] > point[1]):
            crossing_x = start[0] + ((point[1] - start[1])
                                     * (end[0] - start[0]) / (end[1] - start[1]))
            if crossing_x > point[0]:
                inside = not inside
    return inside


def _strictly_convex(polygon):
    turns = [_cross(polygon[index - 1], polygon[index],
                    polygon[(index + 1) % len(polygon)])
             for index in range(len(polygon))]
    return all(turn > 1e-9 for turn in turns) or all(turn < -1e-9 for turn in turns)


def _positive_area_overlap(left, right):
    # HH_261007 - Separating-axis proof for the authored convex quads. Shared
    # edges are allowed; a positive-area overlap with another site is not.
    assert _strictly_convex(left) and _strictly_convex(right)
    for polygon in (left, right):
        for start, end in _edges(polygon):
            axis = (end[1] - start[1], start[0] - end[0])
            a = [point[0] * axis[0] + point[1] * axis[1] for point in left]
            b = [point[0] * axis[0] + point[1] * axis[1] for point in right]
            if max(a) <= min(b) + 1e-8 or max(b) <= min(a) + 1e-8:
                return False
    return True


def _segment_inside_union(start, end, polygons):
    # HH_261007 - Check all boundary-crossing intervals, not only endpoints or
    # a coarse sample that could miss a narrow road/site seam.
    delta = (end[0] - start[0], end[1] - start[1])
    parameters = {0.0, 1.0}
    for polygon in polygons:
        for edge_start, edge_end in _edges(polygon):
            edge_delta = (edge_end[0] - edge_start[0], edge_end[1] - edge_start[1])
            offset = (edge_start[0] - start[0], edge_start[1] - start[1])
            denominator = delta[0] * edge_delta[1] - delta[1] * edge_delta[0]
            if abs(denominator) < 1e-12:
                continue
            along = (offset[0] * edge_delta[1] - offset[1] * edge_delta[0]) / denominator
            across = (offset[0] * delta[1] - offset[1] * delta[0]) / denominator
            if 0.0 <= along <= 1.0 and 0.0 <= across <= 1.0:
                parameters.add(along)
    ordered = sorted(parameters)
    checks = ordered + [(left + right) * 0.5 for left, right in zip(ordered, ordered[1:])]
    return all(any(_inside((start[0] + t * delta[0], start[1] + t * delta[1]), polygon)
                   for polygon in polygons) for t in checks)


@pytest.fixture(scope="module")
def geometry():
    config = yaml.safe_load(ACCESS_PATH.read_text(encoding="utf-8"))
    map_path = REPO_ROOT / config["source_map"]
    sites = yaml.safe_load((REPO_ROOT / config["source_sites"]).read_text(encoding="utf-8"))
    root = ET.parse(map_path).getroot()
    nodes = {node.attrib["id"]: (float(_tags(node)["local_x"]),
                                float(_tags(node)["local_y"])) for node in root.findall("node")}
    ways = {way.attrib["id"]: [nodes[nd.attrib["ref"]] for nd in way.findall("nd")]
            for way in root.findall("way")}
    relations = {relation.attrib["id"]: relation for relation in root.findall("relation")}
    origin = yaml.safe_load((REPO_ROOT / "camrod_map/config/map_info.yaml").read_text())["/**"]["ros__parameters"]
    runtime = load(str(map_path), LocalCartesianProjector(
        Origin(origin["offset_lat"], origin["offset_lon"], origin["offset_alt"])))
    return config, {site["type"]: site for site in sites["camping_sites"]}, ways, relations, runtime


def test_access_geometry_is_bound_to_the_active_unchanged_sources(geometry):
    config, _, _, _, _ = geometry
    for key in ("source_map", "source_sites"):
        assert hashlib.sha256((REPO_ROOT / config[key]).read_bytes()).hexdigest() == config[key + "_sha256"]


def test_only_b1_b11_are_opted_in_without_mission_policy_overrides(geometry):
    config, sites, _, _, _ = geometry
    assert config["frame_id"] == "map"
    assert [entry["site"] for entry in config["service_access"]] == list(SITE_IDS)
    assert sites["camping_site_1"]["service_mode"] == "turnaround"
    assert sites["camping_site_11"]["service_mode"] == "roadside_stop"
    for entry in config["service_access"]:
        assert set(entry) == {"site", "lanelet_ids", "site_edge_corner_indices",
                              "road_overlap_m", "provenance", "corners"}


@pytest.mark.parametrize("site_id", SITE_IDS)
def test_connector_is_finite_strictly_convex_and_only_touches_site_frontage(geometry, site_id):
    config, sites, _, _, _ = geometry
    entry = next(entry for entry in config["service_access"] if entry["site"] == site_id)
    polygon = [_xy(point) for point in entry["corners"]]
    assert len(polygon) == 4 and all(math.isfinite(value) for point in polygon for value in point)
    assert _strictly_convex(polygon)
    area = abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in _edges(polygon))) * 0.5
    assert 0.0 < area < 2.5
    assert entry["site_edge_corner_indices"] == [1, 2]
    assert polygon[:2] == [_xy(sites[site_id]["corners"][index]) for index in (1, 2)]
    for site in sites.values():
        assert not _positive_area_overlap(polygon, [_xy(point) for point in site["corners"]])
    other = next(item for item in config["service_access"] if item["site"] != site_id)
    assert not _positive_area_overlap(polygon, [_xy(point) for point in other["corners"]])


@pytest.mark.parametrize("site_id", SITE_IDS)
def test_connector_reproduces_exact_endpoint_projections_and_bounded_overlap(geometry, site_id):
    config, sites, ways, relations, _ = geometry
    entry = next(entry for entry in config["service_access"] if entry["site"] == site_id)
    provenance = entry["provenance"]
    assert provenance["derivation"] == "nearest_local_xy_boundary_projection_then_roadward_extension"
    assert provenance["road_boundary_role"] == "left"
    assert provenance["site_area_relation_id"] == sites[site_id]["id"].removeprefix("dz_area_")
    assert _tags(relations[provenance["site_area_relation_id"]])["subtype"] == site_id
    assert len(entry["lanelet_ids"]) == len(set(entry["lanelet_ids"]))
    boundaries = []
    actual_way_ids = []
    for lanelet_id in entry["lanelet_ids"]:
        relation = relations[lanelet_id]
        assert _tags(relation)["type"] == "lanelet" and _tags(relation)["subtype"] == "road"
        way_id = next(member.attrib["ref"] for member in relation.findall("member")
                      if member.attrib["role"] == "left")
        actual_way_ids.append(way_id)
        boundaries.extend(zip(ways[way_id], ways[way_id][1:]))
    assert actual_way_ids == provenance["road_boundary_way_ids"]
    assert entry["road_overlap_m"] == 0.10 <= 0.15
    polygon = [_xy(point) for point in entry["corners"]]
    for index, source in enumerate(polygon[:2]):
        projected = min((_project(source, start, end) for start, end in boundaries),
                        key=lambda point: math.dist(source, point))
        assert projected == pytest.approx(_xy(provenance[f"projected_site_corner_{index + 1}"]), abs=1e-10)
        gap = math.dist(source, projected)
        assert 0.30 < gap < 0.50
        expected = tuple(projected[axis] + 0.10 * (projected[axis] - source[axis]) / gap
                         for axis in (0, 1))
        assert polygon[3 - index] == pytest.approx(expected, abs=1e-10)


@pytest.mark.parametrize("site_id", SITE_IDS)
def test_road_facing_edge_stays_inside_local_and_runtime_road_union(geometry, site_id):
    config, _, ways, relations, runtime = geometry
    entry = next(entry for entry in config["service_access"] if entry["site"] == site_id)
    polygon = [_xy(point) for point in entry["corners"]]
    local_roads, runtime_roads = [], []
    for lanelet_id in entry["lanelet_ids"]:
        members = {member.attrib["role"]: member.attrib["ref"]
                   for member in relations[lanelet_id].findall("member")}
        local_roads.append(ways[members["left"]] + list(reversed(ways[members["right"]])))
        lanelet = runtime.laneletLayer[int(lanelet_id)]
        runtime_roads.append([(point.x, point.y) for point in lanelet.leftBound]
                             + [(point.x, point.y) for point in reversed(list(lanelet.rightBound))])
    assert _segment_inside_union(polygon[2], polygon[3], local_roads)
    assert _segment_inside_union(polygon[2], polygon[3], runtime_roads)
    # HH_261007 - The source/export projection mismatch is measured, not ignored.
    for local, projected in zip(local_roads, runtime_roads):
        assert len(local) == len(projected)
        assert max(math.dist(a, b) for a, b in zip(local, projected)) < 0.015


@pytest.mark.parametrize("site_id", SITE_IDS)
def test_exact_site_edge_matches_runtime_area_export(geometry, site_id):
    config, sites, _, _, runtime = geometry
    entry = next(entry for entry in config["service_access"] if entry["site"] == site_id)
    area = runtime.areaLayer[int(entry["provenance"]["site_area_relation_id"])]
    points = [(point.x, point.y) for line in area.outerBound for point in line]
    for corner in sites[site_id]["corners"]:
        assert min(math.dist(_xy(corner), point) for point in points) < 0.0001
