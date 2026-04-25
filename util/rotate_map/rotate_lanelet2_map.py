#!/usr/bin/env python3
"""
Transform Lanelet2 OSM geometry in local/map XY, PCD-style.

Behavior:
- Convert node lat/lon -> projected XY
- Convert projected XY -> local XY using given origin
- Apply rigid transform directly in local XY:
      p' = R * p + t
- Convert local XY -> projected XY -> lat/lon
- Optionally update/create local_x/local_y tags

This is intentionally similar to the pointcloud transform style:
no center-mode, no centroid rotation, just direct transform.
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


def make_2d_transform(yaw_deg: float, tx: float, ty: float) -> Tuple[float, float, float, float]:
    yaw_rad = math.radians(yaw_deg)
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    return c, s, tx, ty


def apply_2d_transform(x: float, y: float, c: float, s: float, tx: float, ty: float) -> Tuple[float, float]:
    rx = c * x - s * y + tx
    ry = s * x + c * y + ty
    return rx, ry


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Transform Lanelet2 OSM in local/map XY (PCD-style rigid transform)."
    )
    parser.add_argument("--input", required=True, help="Input lanelet2 .osm path")
    parser.add_argument("--output", required=True, help="Output lanelet2 .osm path")
    parser.add_argument("--origin-lat", type=float, required=True, help="Map projector origin latitude")
    parser.add_argument("--origin-lon", type=float, required=True, help="Map projector origin longitude")
    parser.add_argument("--map-epsg", type=int, default=32652, help="Projected EPSG (default: 32652)")
    parser.add_argument("--yaw-deg", type=float, default=0.0, help="Yaw in degrees (+CCW)")
    parser.add_argument("--tx", type=float, default=0.0, help="Translation X in local XY")
    parser.add_argument("--ty", type=float, default=0.0, help="Translation Y in local XY")
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
    c, s, tx, ty = make_2d_transform(args.yaw_deg, args.tx, args.ty)

    node_count = 0

    for node_elem in root.findall("node"):
        lat = _as_float(node_elem.get("lat"))
        lon = _as_float(node_elem.get("lon"))

        # lat/lon -> projected map XY
        mx, my = llh_to_map.transform(lon, lat)

        # projected XY -> local XY
        lx = mx - origin_x
        ly = my - origin_y

        # apply direct rigid transform in local XY
        tlx, tly = apply_2d_transform(lx, ly, c, s, tx, ty)

        # local XY -> projected XY
        tmx = origin_x + tlx
        tmy = origin_y + tly

        # projected XY -> lat/lon
        tlon, tlat = map_to_llh.transform(tmx, tmy)

        node_elem.set("lat", f"{tlat:.11f}")
        node_elem.set("lon", f"{tlon:.11f}")

        lx_tag = _find_tag(node_elem, "local_x")
        ly_tag = _find_tag(node_elem, "local_y")

        if lx_tag is not None or args.write_local_tags:
            if lx_tag is None:
                lx_tag = ET.SubElement(node_elem, "tag", {"k": "local_x", "v": "0.0"})
            lx_tag.set("v", f"{tlx:.4f}")

        if ly_tag is not None or args.write_local_tags:
            if ly_tag is None:
                ly_tag = ET.SubElement(node_elem, "tag", {"k": "local_y", "v": "0.0"})
            ly_tag.set("v", f"{tly:.4f}")

        node_count += 1

    if node_count == 0:
        raise RuntimeError("No <node> elements found in OSM.")

    tree.write(args.output, encoding="UTF-8", xml_declaration=True)

    print(f"[OK] Saved: {args.output}")
    print(f"     yaw_deg={args.yaw_deg}")
    print(f"     tx={args.tx}, ty={args.ty}")
    print(f"     origin=({args.origin_lat}, {args.origin_lon}), epsg={args.map_epsg}")
    print(f"     nodes={node_count}")


if __name__ == "__main__":
    main()
