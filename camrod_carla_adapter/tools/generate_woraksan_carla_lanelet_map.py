#!/usr/bin/env python3
"""Generate the virtual-only Woraksan Lanelet2 map used with CARLA.

The production CAMROD map is georeferenced correctly, but its explicit
centerline around lanelets 2757 -> 2285 is not the center of CARLA XODR
Road26.  On the 1.25 m-wide connector that local shape error is large enough
to put the Ranger front-left corner into Terrain.  Replacing only the
centerline intentionally does not authorize more drivable surface: the
physical connector is narrower than the Ranger at several terrain overlaps,
so the unchanged production boundaries remain a fail-closed safety limit.
The virtual B12 return also needs four interior samples of lanelet 2744 moved
inward from its outer S-bend boundary.  This generator keeps every production
boundary/relation/tag and changes only those four explicit centerline ways.

The coordinates below are a frozen export from the accepted Woraksan CARLA
map (Road26 lane 2 -> Road20 lane 2 -> Road54 lane -1), transformed through
``woraksan_lane_anchor_alignment.yaml``.  A frozen export makes generation
deterministic and ensures that building CAMROD does not require a running
CARLA server.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from lanelet2.core import BasicPoint3d, GPSPoint
from lanelet2.io import Origin
from lanelet2.projection import LocalCartesianProjector


ORIGIN = Origin(36.8435737, 128.0925646, 0.0)
REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = REPOSITORY_ROOT / "lanelet2_maps.osm"
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT
    / "camrod_carla_adapter"
    / "config"
    / "woraksan_carla_lanelet2.osm"
)

# HH_260830 - Road26/lane 2, from the 2219->2757 boundary to the
# 2757->2285 boundary.  The final point is the exact intersection with the
# preserved lanelet boundary rather than a rounded XODR station.
WAY_6304_XY = (
    (27.1360933137, -14.3261004636),
    (27.262580741, -14.445737538),
    (27.469781898, -14.633368976),
    (27.657125635, -14.804316852),
    (27.826334288, -14.960475162),
    (27.991236155, -15.123786032),
    (28.152124967, -15.296584058),
    (28.312245128, -15.481217182),
    (28.475533399, -15.670584344),
    (28.649571492, -15.866293431),
    (28.837893718, -16.061661090),
    (29.005343809, -16.234083363),
    (29.177058740, -16.415720033),
    (29.348874760, -16.597343352),
    (29.521934371, -16.777763133),
    (29.694581992, -16.956521614),
    (29.859466701, -17.130389659),
    (30.019794756, -17.306212740),
    (30.176399668, -17.485352704),
    (30.3076653409, -17.6447146599),
)

# HH_260830 - Continue through the remaining Road26 connector, the short
# Road20 link and Road54 until the preserved 2285->2451 boundary.  Duplicate
# XODR road-junction samples were intentionally collapsed.
WAY_6147_XY = (
    (30.3076653409, -17.6447146599),
    (30.328131309, -17.669417445),
    (30.486580637, -17.858908588),
    (30.645728056, -18.051710888),
    (30.801320176, -18.242483771),
    (30.954890504, -18.430134327),
    (31.116231375, -18.623447794),
    (31.282693477, -18.816633465),
    (31.449239499, -19.006403074),
    (31.618801044, -19.201070752),
    (31.813457586, -19.409992026),
    (31.925253108, -19.527991012),
    (32.095436438, -19.710435446),
    (32.267546190, -19.891768847),
    (32.441942850, -20.070969832),
    (32.618069475, -20.248505699),
    (32.765778489, -20.397016512),
    (32.928519344, -20.578440520),
    (33.084409007, -20.765475000),
    (33.233693527, -20.958000742),
    (33.3100432001, -21.0622328367),
)

# HH_260830 - Stay on Road54 through its narrow prefix.  At s~=3 m the lane
# has widened from about 1.20 m to 1.88 m, so a curvature-continuous blend can
# safely return to the original lanelet 2451 centerline.
WAY_6214_XODR_PREFIX_XY = (
    (33.3100432001, -21.0622328367),
    (33.376412957, -21.155678238),
    (33.512624517, -21.358167980),
    (33.642393057, -21.565136182),
    (33.765800593, -21.776260502),
    (33.882925325, -21.991218597),
    (33.993872159, -22.209705293),
    (34.098747904, -22.431409690),
    (34.197670816, -22.656036151),
    (34.290770595, -22.883296665),
)

# HH_260830 - B12 reverse-return evidence showed the planning footprint
# touching the outer edge of the 2744 -> 2720 S bend.  These are projected
# map-frame coordinates: node 6978 moves inward by 0.09 m and the following
# plateau by 0.15 m.  The common 2744/2720 seam (6963), every boundary and the
# relation topology stay untouched.  New nodes make the virtual-only delta
# explicit instead of silently mutating production node identities.
WAY_6975_INWARD_REPLACEMENTS = (
    (6978, (9.085585903, 45.336725709)),
    (6977, (9.614894557, 45.484708450)),
    (6976, (10.143302623, 45.451895124)),
    (6969, (10.950121780, 45.064342302)),
)

TARGET_WAYS = (6304, 6147, 6214, 6975)
MOVED_ENDPOINTS = (6201, 6141, 6146)
FIRST_GENERATED_NODE_ID = 7000


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser


def _elements_by_id(root: ET.Element, tag: str) -> dict[int, ET.Element]:
    return {int(element.attrib["id"]): element for element in root.findall(tag)}


def _tag_value(node: ET.Element, key: str) -> str:
    for tag in node.findall("tag"):
        if tag.attrib.get("k") == key:
            return tag.attrib["v"]
    raise RuntimeError(f"node {node.attrib['id']} lacks tag {key}")


def _set_tag(node: ET.Element, key: str, value: str) -> None:
    for tag in node.findall("tag"):
        if tag.attrib.get("k") == key:
            tag.attrib["v"] = value
            return
    ET.SubElement(node, "tag", {"k": key, "v": value})


def _project_node(projector: LocalCartesianProjector, node: ET.Element) -> np.ndarray:
    point = projector.forward(
        GPSPoint(
            float(node.attrib["lat"]),
            float(node.attrib["lon"]),
            float(_tag_value(node, "ele")),
        )
    )
    return np.array((point.x, point.y), dtype=float)


def _cumulative_distances(points: tuple[tuple[float, float], ...]) -> np.ndarray:
    array = np.asarray(points, dtype=float)
    return np.concatenate(
        (np.array((0.0,)), np.cumsum(np.linalg.norm(np.diff(array, axis=0), axis=1)))
    )


def _interpolated_elevations(
    points: tuple[tuple[float, float], ...], start_ele: float, end_ele: float
) -> tuple[float, ...]:
    distances = _cumulative_distances(points)
    if distances[-1] <= 0.0:
        raise RuntimeError("centerline must have non-zero length")
    ratio = distances / distances[-1]
    return tuple(start_ele + (end_ele - start_ele) * value for value in ratio)


def _set_node_position(
    projector: LocalCartesianProjector,
    node: ET.Element,
    x_m: float,
    y_m: float,
    elevation_m: float,
) -> None:
    gps = projector.reverse(BasicPoint3d(x_m, y_m, elevation_m))
    node.attrib["lat"] = f"{gps.lat:.11f}"
    node.attrib["lon"] = f"{gps.lon:.11f}"
    _set_tag(node, "local_x", f"{x_m:.9f}")
    _set_tag(node, "local_y", f"{y_m:.9f}")
    _set_tag(node, "ele", f"{elevation_m:.6f}")


def _append_generated_node(
    root: ET.Element,
    first_way_index: int,
    projector: LocalCartesianProjector,
    node_id: int,
    point: tuple[float, float],
    elevation_m: float,
) -> ET.Element:
    node = ET.Element(
        "node",
        {
            "id": str(node_id),
            "lat": "0",
            "lon": "0",
        },
    )
    _set_node_position(projector, node, point[0], point[1], elevation_m)
    root.insert(first_way_index, node)
    return node


def _replace_way_refs(way: ET.Element, node_ids: list[int]) -> None:
    children = list(way)
    for child in children:
        if child.tag == "nd":
            way.remove(child)
    for index, node_id in enumerate(node_ids):
        # OSM way node references precede its tags.  Inserting at the old tag
        # index after removing the references would incorrectly put them after
        # the tags and some Lanelet2 readers reject that ordering.
        way.insert(index, ET.Element("nd", {"ref": str(node_id)}))


def _hermite_blend(
    start: np.ndarray,
    start_direction: np.ndarray,
    end: np.ndarray,
    end_direction: np.ndarray,
    start_scale_m: float = 3.5,
    end_scale_m: float = 6.0,
    spacing_m: float = 0.20,
) -> tuple[tuple[float, float], ...]:
    """Return interior samples of a C1 Hermite blend at near-uniform spacing."""
    start_direction = start_direction / np.linalg.norm(start_direction)
    end_direction = end_direction / np.linalg.norm(end_direction)
    parameter = np.linspace(0.0, 1.0, 5001)[:, np.newaxis]
    h00 = 2.0 * parameter**3 - 3.0 * parameter**2 + 1.0
    h10 = parameter**3 - 2.0 * parameter**2 + parameter
    h01 = -2.0 * parameter**3 + 3.0 * parameter**2
    h11 = parameter**3 - parameter**2
    dense = (
        h00 * start
        + h10 * start_scale_m * start_direction
        + h01 * end
        + h11 * end_scale_m * end_direction
    )
    distance = np.concatenate(
        (
            np.array((0.0,)),
            np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1)),
        )
    )
    sample_distance = np.arange(spacing_m, distance[-1], spacing_m)
    x = np.interp(sample_distance, distance, dense[:, 0])
    y = np.interp(sample_distance, distance, dense[:, 1])
    return tuple((float(px), float(py)) for px, py in zip(x, y))


def generate(source: Path, output: Path) -> None:
    tree = ET.parse(source)
    root = tree.getroot()
    nodes = _elements_by_id(root, "node")
    ways = _elements_by_id(root, "way")
    for node_id in MOVED_ENDPOINTS:
        if node_id not in nodes:
            raise RuntimeError(f"source map lacks expected node {node_id}")
    for way_id in TARGET_WAYS:
        if way_id not in ways:
            raise RuntimeError(f"source map lacks expected way {way_id}")
    if any(node_id >= FIRST_GENERATED_NODE_ID for node_id in nodes):
        raise RuntimeError(
            f"generated node range >= {FIRST_GENERATED_NODE_ID} is already occupied"
        )

    projector = LocalCartesianProjector(ORIGIN)
    first_way_index = next(
        index for index, element in enumerate(list(root)) if element.tag == "way"
    )
    next_node_id = FIRST_GENERATED_NODE_ID

    def install_centerline(
        way_id: int,
        points: tuple[tuple[float, float], ...],
        start_node_id: int,
        end_node_id: int,
    ) -> None:
        nonlocal next_node_id, first_way_index
        start_ele = float(_tag_value(nodes[start_node_id], "ele"))
        end_ele = float(_tag_value(nodes[end_node_id], "ele"))
        elevations = _interpolated_elevations(points, start_ele, end_ele)
        _set_node_position(
            projector, nodes[start_node_id], *points[0], elevations[0]
        )
        _set_node_position(
            projector, nodes[end_node_id], *points[-1], elevations[-1]
        )
        refs = [start_node_id]
        for point, elevation in zip(points[1:-1], elevations[1:-1]):
            _append_generated_node(
                root,
                first_way_index,
                projector,
                next_node_id,
                point,
                elevation,
            )
            refs.append(next_node_id)
            next_node_id += 1
            first_way_index += 1
        refs.append(end_node_id)
        _replace_way_refs(ways[way_id], refs)

    install_centerline(6304, WAY_6304_XY, 6201, 6141)
    install_centerline(6147, WAY_6147_XY, 6141, 6146)

    # Join Road54 to the original wide-road centerline at node 6210.  The
    # outgoing tangent is exactly the existing 6210->6211 segment, so the
    # replacement does not introduce another discontinuity after the blend.
    join_node_id = 6210
    next_node = nodes[6211]
    join = _project_node(projector, nodes[join_node_id])
    outgoing = _project_node(projector, next_node) - join
    prefix = np.asarray(WAY_6214_XODR_PREFIX_XY, dtype=float)
    blend = _hermite_blend(prefix[-1], prefix[-1] - prefix[-2], join, outgoing)
    way_6214_points = tuple(WAY_6214_XODR_PREFIX_XY) + blend + (
        (float(join[0]), float(join[1])),
    )
    start_ele = float(_tag_value(nodes[6146], "ele"))
    end_ele = float(_tag_value(nodes[join_node_id], "ele"))
    elevations = _interpolated_elevations(way_6214_points, start_ele, end_ele)
    _set_node_position(
        projector, nodes[6146], *way_6214_points[0], elevations[0]
    )
    refs = [6146]
    for point, elevation in zip(way_6214_points[1:-1], elevations[1:-1]):
        _append_generated_node(
            root,
            first_way_index,
            projector,
            next_node_id,
            point,
            elevation,
        )
        refs.append(next_node_id)
        next_node_id += 1
        first_way_index += 1
    refs.extend((join_node_id, 6211, 6212, 6213))
    _replace_way_refs(ways[6214], refs)

    # Preserve way 6975's endpoints and unaffected samples.  Only the four
    # audited outer-bend samples receive virtual node IDs 7067..7070.
    replacement_refs: dict[int, int] = {}
    for source_node_id, point in WAY_6975_INWARD_REPLACEMENTS:
        elevation = float(_tag_value(nodes[source_node_id], "ele"))
        _append_generated_node(
            root,
            first_way_index,
            projector,
            next_node_id,
            point,
            elevation,
        )
        replacement_refs[source_node_id] = next_node_id
        next_node_id += 1
        first_way_index += 1
    original_refs = [int(nd.attrib["ref"]) for nd in ways[6975].findall("nd")]
    _replace_way_refs(
        ways[6975],
        [replacement_refs.get(node_id, node_id) for node_id in original_refs],
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="UTF-8", xml_declaration=True, short_empty_elements=True)


def main() -> int:
    arguments = _parser().parse_args()
    generate(arguments.source.resolve(), arguments.output.resolve())
    print(arguments.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
