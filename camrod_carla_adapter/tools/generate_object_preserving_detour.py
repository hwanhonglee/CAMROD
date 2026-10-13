#!/usr/bin/env python3
"""HH_261007 - Build an offline CARLA-only copy from the finite shoulder audit.

Only lanelets 2757/2285/2451 and their geometry are changed. This does not
activate a map, authorize motion, or change any CARLA object or safety setting.
The original map, external seam nodes, relation tags and routing remain exact.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from lanelet2.core import BasicPoint3d
from lanelet2.io import Origin, load
from lanelet2.projection import LocalCartesianProjector
from scipy.interpolate import CubicSpline
from shapely.geometry import LineString, Point, Polygon, shape
from shapely.ops import unary_union


TARGET_LANELETS = (2757, 2285, 2451)
TARGET_WAYS = (2755, 2756, 6304, 2256, 2226, 6147, 2449, 2450, 6214)
EXTERNAL_NODES = (2213, 2207, 6201, 2268, 2244, 6213)
CORRIDOR_MARGIN_M = 0.075
BOUNDARY_INSET_M = 0.002
CHORD_SIMPLIFICATION_M = 0.00025
ORIGIN = Origin(36.8435737, 128.0925646, 0.0)
REPOSITORY = Path(__file__).resolve().parents[2]


def _polygon(lanelet):
    return Polygon([(p.x, p.y) for p in lanelet.leftBound]
                   + [(p.x, p.y) for p in reversed(list(lanelet.rightBound))])


def _parts(geometry):
    if geometry.geom_type == "LineString":
        return [geometry]
    return [part for part in getattr(geometry, "geoms", ())
            if part.geom_type == "LineString"]


def _signed_inside(point, left, right, inside):
    edge = np.asarray(right) - np.asarray(left)
    sign = np.cross(edge, np.asarray(inside) - np.asarray(left))
    return float(np.cross(edge, np.asarray(point) - np.asarray(left)) * sign)


def _read_audit(source, audit, grid_resolution_m):
    if (type(grid_resolution_m) not in (int, float)
            or not math.isfinite(grid_resolution_m) or grid_resolution_m <= 0
            or grid_resolution_m / math.sqrt(2) >= CORRIDOR_MARGIN_M - BOUNDARY_INSET_M):
        raise ValueError("grid half-diagonal must fit the audited margin minus the 0.002 m chord inset")
    document = json.loads(audit.read_text(encoding="utf-8"))
    if hashlib.sha256(source.read_bytes()).hexdigest() != document.get("source_map_sha256"):
        raise ValueError("source map does not match the finite geometry audit")
    if (document.get("objects_and_map_unchanged") is not True
            or document.get("driving_proof") is not False
            or document.get("source_export_meshes_verified") != 15):
        raise ValueError("missing unchanged-object finite-audit provenance")
    checks = document.get("mesh_projection_checks") or []
    if not checks or any(
            row.get("gate_intersecting_stations_m") != []
            or not math.isfinite(row.get("min_gate_footprint_distance_m", math.nan))
            or row["min_gate_footprint_distance_m"] <= CORRIDOR_MARGIN_M
            for row in checks):
        raise ValueError("audited planning footprint/corridor is not statically clear")
    support = document.get("support_checks") or []
    rays = (document.get("live_support") or {}).get("rays") or []
    if (not support or not rays or any(not row.get("hits") for row in rays)
            or any(row.get("error") or row.get("height_clipped_prop_intersections") != []
                   for row in support)):
        raise ValueError("finite ground-support/body-height evidence is incomplete")
    swept = shape(document["swept_gate_polygon"])
    corridor = shape(document["corridor_polygon"])
    if (not swept.is_valid or not corridor.is_valid or corridor.geom_type != "Polygon"
            or corridor.hausdorff_distance(swept.buffer(CORRIDOR_MARGIN_M)) > 1e-8):
        raise ValueError("corridor differs from the audited planning sweep plus 0.075 m")
    return document, swept, corridor


def _geometry(source, document, swept, corridor):
    lane_map = load(str(source), LocalCartesianProjector(ORIGIN))
    lines = [LineString([(p.x, p.y) for p in lane_map.laneletLayer[i].centerline])
             for i in TARGET_LANELETS]
    cuts = (-lines[0].length, 0.0, lines[1].length, lines[1].length + lines[2].length)
    knots = np.asarray(document["knots_parameter_m"], dtype=float)
    coordinates = np.asarray(document["knots_map_xy_m"], dtype=float)
    if (not np.isfinite(knots).all() or not np.isfinite(coordinates).all()
            or abs(knots[0] - cuts[0]) > 1e-8 or abs(knots[-1] - cuts[-1]) > 1e-8):
        raise ValueError("audit does not cover the exact three-lanelet span")
    curve = CubicSpline(knots, coordinates, bc_type="natural")
    if (np.linalg.norm(curve(cuts[0]) - np.asarray(lines[0].coords[0])) > 1e-6
            or np.linalg.norm(curve(cuts[-1]) - np.asarray(lines[-1].coords[-1])) > 1e-6):
        raise ValueError("audit moves an external centerline seam")
    samples = document["candidate_samples"]
    # HH_261007 - Densify only the same audited spline/envelope; fine boundary
    # chords remain inside its curved sides without increasing road permission.
    stations = sorted({float(row["station_m"]) for row in samples} | set(cuts)
                      | set(np.linspace(knots[0], knots[-1], 4 * (len(samples) - 1) + 1)))
    if any(np.linalg.norm(curve(row["station_m"]) - row["map_xy_m"]) > 1e-7
           for row in samples):
        raise ValueError("audit samples differ from its explicit spline")

    # HH_261007 - Cross-sections follow the audited swept polygon, not a fixed
    # normal width: a long body's corner sweep grows on the tight curved portion.
    bounds = {"left": [], "right": [], "centerline": []}
    # HH_261007 - Leave 2 mm inside the finite audit envelope so straight OSM
    # chords cannot authorize tiny unverified slivers at concave boundary arcs.
    sampled_corridor = corridor.buffer(-BOUNDARY_INSET_M)
    for station in stations:
        point, tangent = curve(station), curve(station, 1)
        norm = np.linalg.norm(tangent)
        if not math.isfinite(norm) or norm <= 1e-9:
            raise ValueError("audit has a degenerate tangent")
        normal = np.array((-tangent[1], tangent[0])) / norm
        section = LineString([point - 4 * normal, point + 4 * normal]).intersection(sampled_corridor)
        parts = _parts(section)
        if not parts:
            raise ValueError("audited corridor has no transverse section")
        segment = min(parts, key=lambda item: item.distance(Point(point)))
        if segment.distance(Point(point)) > 1e-8:
            raise ValueError("centerline leaves the audited corridor")
        ends = sorted(segment.coords, key=lambda xy: np.dot(np.asarray(xy) - point, normal))
        bounds["left"].append((station, tuple(ends[-1])))
        bounds["right"].append((station, tuple(ends[0])))
        bounds["centerline"].append((station, tuple(point)))

    first, last = (lane_map.laneletLayer[i] for i in (2757, 2451))
    start = ((first.leftBound[0].x, first.leftBound[0].y),
             (first.rightBound[0].x, first.rightBound[0].y))
    end = ((last.leftBound[-1].x, last.leftBound[-1].y),
           (last.rightBound[-1].x, last.rightBound[-1].y))
    for side_index, role in enumerate(("left", "right")):
        # HH_261007 - Clip only the local caps. An infinite end half-plane
        # would cut the earlier arm of this curved road. Original cap vertices
        # remain exact, and no new authorized area may leave the audited sweep.
        retained = []
        for station, point in bounds[role][1:-1]:
            if (station - cuts[0] < 2.0
                    and _signed_inside(point, *start, curve(cuts[0] + 0.1)) < 0):
                continue
            if (cuts[-1] - station < 2.0
                    and _signed_inside(point, *end, curve(cuts[-1] - 0.1)) < 0):
                continue
            retained.append((station, point))
        bounds[role] = [(cuts[0], start[side_index]), *retained,
                        (cuts[-1], end[side_index])]

    geometries, polygons = {}, []
    for index, lanelet_id in enumerate(TARGET_LANELETS):
        a, b = cuts[index:index + 2]
        for offset, role in enumerate(("left", "right", "centerline")):
            points = [(station, point) for station, point in bounds[role] if a <= station <= b]
            if not points or points[0][0] != a or points[-1][0] != b:
                raise ValueError("clipping removed an internal shared seam")
            # HH_261007 - Retain sub-millimetre geometry without burdening the
            # live raster with thousands of redundant straight-edge vertices.
            retained = set(LineString([point for _, point in points]).simplify(
                CHORD_SIMPLIFICATION_M, preserve_topology=True).coords)
            points = [(station, point) for station, point in points if point in retained]
            geometries[TARGET_WAYS[index * 3 + offset]] = points
        polygon = Polygon([point for _, point in geometries[TARGET_WAYS[index * 3]]]
                          + [point for _, point in reversed(geometries[TARGET_WAYS[index * 3 + 1]])])
        if not polygon.is_valid:
            raise ValueError(f"paired boundaries self-intersect in lanelet {lanelet_id}")
        polygons.append(polygon)

    joined = unary_union(polygons)
    neighbors = [_polygon(lane_map.laneletLayer[i]) for i in (2219, 2292, 2943)]
    if swept.difference(unary_union([joined, *neighbors])).area > 1e-8:
        raise ValueError("paired boundaries fail full audited planning-footprint containment")
    allowed = unary_union([corridor, *[_polygon(lane_map.laneletLayer[i]) for i in TARGET_LANELETS]])
    excess = joined.difference(allowed)
    if excess.area > 1e-8:
        raise ValueError(f"new road area exceeds the audited corridor: area={excess.area}, bounds={excess.bounds}")
    return geometries, lane_map, joined


def generate(source: Path, audit: Path, output: Path, *, grid_resolution_m=0.10) -> dict:
    source, audit, output = Path(source), Path(audit), Path(output)
    # HH_261007 - Never overwrite the original, a symlink target, or prior evidence.
    if output.resolve() in {source.resolve(), audit.resolve()}:
        raise ValueError("output must be a separate new CARLA-only artifact")
    if output.exists() or output.is_symlink():
        raise FileExistsError(output)
    source_bytes = source.read_bytes()
    audit_bytes = audit.read_bytes()
    document, swept, corridor = _read_audit(source, audit, grid_resolution_m)
    geometries, lane_map, joined = _geometry(source, document, swept, corridor)
    root = ET.fromstring(source_bytes)
    nodes = {int(element.attrib["id"]): element for element in root.findall("node")}
    ways = {int(element.attrib["id"]): element for element in root.findall("way")}
    refs = {way_id: [int(nd.attrib["ref"]) for nd in ways[way_id].findall("nd")]
            for way_id in TARGET_WAYS}
    target_nodes = {node for ids in refs.values() for node in ids}
    for way_id, way in ways.items():
        if way_id not in TARGET_WAYS:
            shared = {int(nd.attrib["ref"]) for nd in way.findall("nd")} & target_nodes
            if shared - set(EXTERNAL_NODES):
                raise ValueError("target geometry has an unaudited external node dependency")
    projector = LocalCartesianProjector(ORIGIN)
    original_xyz = {node_id: lane_map.pointLayer[node_id] for node_id in target_nodes}
    next_id = max(int(e.attrib["id"]) for e in root if "id" in e.attrib) + 1
    insertion = next(index for index, element in enumerate(root) if element.tag == "way")
    modified_nodes = set()
    for way_id in TARGET_WAYS:
        data = geometries[way_id]
        old_refs = refs[way_id]
        first_z, last_z = original_xyz[old_refs[0]].z, original_xyz[old_refs[-1]].z
        new_refs = []
        for index, (station, point) in enumerate(data):
            if index in (0, len(data) - 1):
                node_id = old_refs[0 if index == 0 else -1]
                node = nodes[node_id]
            else:
                node_id = next_id
                next_id += 1
                node = ET.Element("node", {"id": str(node_id)})
                root.insert(insertion, node)
                insertion += 1
                nodes[node_id] = node
            new_refs.append(node_id)
            if node_id in EXTERNAL_NODES:
                continue
            ratio = (station - data[0][0]) / (data[-1][0] - data[0][0])
            elevation = first_z + ratio * (last_z - first_z)
            gps = projector.reverse(BasicPoint3d(*point, elevation))
            node.set("lat", f"{gps.lat:.12f}")
            node.set("lon", f"{gps.lon:.12f}")
            for key, value in (("local_x", point[0]), ("local_y", point[1]), ("ele", gps.ele)):
                tag = node.find(f"tag[@k='{key}']")
                if tag is None:
                    tag = ET.SubElement(node, "tag", {"k": key})
                tag.set("v", f"{value:.9f}")
            modified_nodes.add(node_id)
        way = ways[way_id]
        for nd in list(way.findall("nd")):
            way.remove(nd)
        for index, node_id in enumerate(new_refs):
            way.insert(index, ET.Element("nd", {"ref": str(node_id)}))
    ET.indent(root, space="  ")
    encoded = ET.tostring(root, encoding="utf-8", xml_declaration=True) + b"\n"
    if source.read_bytes() != source_bytes or audit.read_bytes() != audit_bytes:
        raise ValueError("source map or audit changed during generation")
    with output.open("xb") as stream:
        stream.write(encoded)
    return {
        "schema": "camrod.offline_object_preserving_detour.v1",
        "source_sha256": hashlib.sha256(source_bytes).hexdigest(),
        "audit_sha256": hashlib.sha256(audit_bytes).hexdigest(),
        "output_sha256": hashlib.sha256(encoded).hexdigest(),
        "target_lanelets": list(TARGET_LANELETS), "target_ways": list(TARGET_WAYS),
        "modified_or_new_nodes": sorted(modified_nodes),
        "external_nodes_preserved": list(EXTERNAL_NODES),
        "grid_resolution_m": grid_resolution_m, "corridor_margin_m": CORRIDOR_MARGIN_M,
        "boundary_chord_inset_m": BOUNDARY_INSET_M,
        "chord_simplification_tolerance_m": CHORD_SIMPLIFICATION_M,
        "road_area_m2": joined.area, "driving_proof": False, "activated": False,
        "qualification": document["qualification"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=REPOSITORY / "lanelet2_maps.osm")
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, help="new exclusive JSON provenance file")
    parser.add_argument("--grid-resolution-m", type=float, default=0.10)
    args = parser.parse_args()
    if args.report is not None:
        if args.report.resolve() in {args.source.resolve(), args.audit.resolve(), args.output.resolve()}:
            parser.error("--report must differ from source, audit and output")
        if args.report.exists() or args.report.is_symlink():
            raise FileExistsError(args.report)
    report = generate(args.source, args.audit, args.output, grid_resolution_m=args.grid_resolution_m)
    if args.report is not None:
        with args.report.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write("\n")
    summary = {key: value for key, value in report.items() if key != "modified_or_new_nodes"}
    summary["modified_or_new_node_count"] = len(report["modified_or_new_nodes"])
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
