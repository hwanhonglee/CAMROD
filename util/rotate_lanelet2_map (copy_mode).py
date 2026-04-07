#!/usr/bin/env python3
"""
Rotate Lanelet2 OSM geometry in XY plane.

This utility updates:
- node lat/lon (via projection inverse transform)
- node local_x/local_y tags (if present, or created with --write-local-tags)
"""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from typing import List, Optional, Tuple

from pyproj import Transformer


def _find_tag(node_elem: ET.Element, key: str) -> Optional[ET.Element]:
    for tag in node_elem.findall("tag"):
        if tag.get("k") == key:
            return tag
    return None


def _as_float(value: Optional[str], default: float = 0.0) -> float:
    if value is None:
        return default
    try:
        return float(value)
    except Exception:
        return default


def _rotate_xy(x: float, y: float, cx: float, cy: float, yaw_rad: float) -> Tuple[float, float]:
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    dx = x - cx
    dy = y - cy
    rx = c * dx - s * dy + cx
    ry = s * dx + c * dy + cy
    return rx, ry


def main() -> None:
    parser = argparse.ArgumentParser(description="Rotate Lanelet2 OSM in local/map XY.")
    parser.add_argument("--input", required=True, help="Input lanelet2 .osm path")
    parser.add_argument("--output", required=True, help="Output lanelet2 .osm path")
    parser.add_argument("--origin-lat", type=float, required=True, help="Map projector origin latitude")
    parser.add_argument("--origin-lon", type=float, required=True, help="Map projector origin longitude")
    parser.add_argument("--map-epsg", type=int, default=32652, help="Projected EPSG (default: 32652)")
    parser.add_argument("--yaw-deg", type=float, required=True, help="Rotation yaw in degrees (+CCW)")
    parser.add_argument(
        "--center-mode",
        choices=["origin", "centroid", "custom"],
        default="origin",
        help="Rotation center in local coordinates",
    )
    parser.add_argument("--center-x", type=float, default=0.0, help="Custom center local_x (m)")
    parser.add_argument("--center-y", type=float, default=0.0, help="Custom center local_y (m)")
    parser.add_argument(
        "--write-local-tags",
        action="store_true",
        help="Create local_x/local_y tags for nodes that do not have them",
    )
    args = parser.parse_args()

    tree = ET.parse(args.input)
    root = tree.getroot()

    llh_to_map = Transformer.from_crs("EPSG:4326", f"EPSG:{args.map_epsg}", always_xy=True)
    map_to_llh = Transformer.from_crs(f"EPSG:{args.map_epsg}", "EPSG:4326", always_xy=True)

    origin_x, origin_y = llh_to_map.transform(args.origin_lon, args.origin_lat)
    yaw_rad = math.radians(args.yaw_deg)

    nodes: List[Tuple[ET.Element, float, float]] = []
    for n in root.findall("node"):
        lat = _as_float(n.get("lat"))
        lon = _as_float(n.get("lon"))
        mx, my = llh_to_map.transform(lon, lat)
        lx = mx - origin_x
        ly = my - origin_y
        nodes.append((n, lx, ly))

    if not nodes:
        raise RuntimeError("No <node> elements found in OSM.")

    if args.center_mode == "origin":
        cx, cy = 0.0, 0.0
    elif args.center_mode == "custom":
        cx, cy = args.center_x, args.center_y
    else:
        cx = sum(lx for _, lx, _ in nodes) / float(len(nodes))
        cy = sum(ly for _, _, ly in nodes) / float(len(nodes))

    for node_elem, lx, ly in nodes:
        rlx, rly = _rotate_xy(lx, ly, cx, cy, yaw_rad)
        rmx = origin_x + rlx
        rmy = origin_y + rly
        rlon, rlat = map_to_llh.transform(rmx, rmy)

        node_elem.set("lat", f"{rlat:.11f}")
        node_elem.set("lon", f"{rlon:.11f}")

        lx_tag = _find_tag(node_elem, "local_x")
        ly_tag = _find_tag(node_elem, "local_y")
        if lx_tag is not None or args.write_local_tags:
            if lx_tag is None:
                lx_tag = ET.SubElement(node_elem, "tag", {"k": "local_x", "v": "0.0"})
            lx_tag.set("v", f"{rlx:.4f}")
        if ly_tag is not None or args.write_local_tags:
            if ly_tag is None:
                ly_tag = ET.SubElement(node_elem, "tag", {"k": "local_y", "v": "0.0"})
            ly_tag.set("v", f"{rly:.4f}")

    tree.write(args.output, encoding="UTF-8", xml_declaration=True)
    print(
        "Rotated map written:",
        args.output,
        f"(yaw_deg={args.yaw_deg:.6f}, center_mode={args.center_mode}, center=({cx:.3f},{cy:.3f}))",
    )


if __name__ == "__main__":
    main()
