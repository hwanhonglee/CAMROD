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
import shutil
import subprocess
import xml.etree.ElementTree as ET
from typing import List, Optional, Tuple

try:
    from pyproj import CRS, Transformer
except ImportError:
    CRS = None
    Transformer = None


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


def _utm_proj_args(map_epsg: int) -> List[str]:
    if 32601 <= map_epsg <= 32660:
        return [
            "+proj=utm",
            f"+zone={map_epsg - 32600}",
            "+datum=WGS84",
            "+units=m",
            "+no_defs",
        ]
    if 32701 <= map_epsg <= 32760:
        return [
            "+proj=utm",
            f"+zone={map_epsg - 32700}",
            "+south",
            "+datum=WGS84",
            "+units=m",
            "+no_defs",
        ]
    raise RuntimeError(
        f"cs2cs fallback only supports WGS84 UTM EPSG 32601..32660/32701..32760; got {map_epsg}. "
        "Install python3-pyproj for other projected CRS values."
    )


def _local_tmerc_proj_args(origin_lat: float, origin_lon: float, scale_factor: float) -> List[str]:
    return [
        "+proj=tmerc",
        f"+lat_0={origin_lat:.12f}",
        f"+lon_0={origin_lon:.12f}",
        f"+k={scale_factor:.12f}",
        "+x_0=0",
        "+y_0=0",
        "+ellps=WGS84",
        "+units=m",
        "+no_defs",
    ]


def _run_cs2cs(
    points: List[Tuple[float, float]],
    source_args: List[str],
    target_args: List[str],
) -> List[Tuple[float, float]]:
    if shutil.which("cs2cs") is None:
        raise RuntimeError("Neither python3-pyproj nor the cs2cs command is available.")

    payload = "".join(f"{x:.12f} {y:.12f}\n" for x, y in points)
    proc = subprocess.run(
        ["cs2cs", "-f", "%.12f", *source_args, "+to", *target_args],
        input=payload,
        text=True,
        capture_output=True,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"cs2cs failed: {proc.stderr.strip()}")

    transformed: List[Tuple[float, float]] = []
    for line in proc.stdout.splitlines():
        fields = line.split()
        if len(fields) < 2:
            continue
        transformed.append((float(fields[0]), float(fields[1])))
    if len(transformed) != len(points):
        raise RuntimeError(
            f"cs2cs returned {len(transformed)} points for {len(points)} inputs."
        )
    return transformed


def _make_projectors(
    projection: str,
    origin_lat: float,
    origin_lon: float,
    map_epsg: int,
    scale_factor: float,
):
    if projection == "local-tmerc":
        proj4 = " ".join(_local_tmerc_proj_args(origin_lat, origin_lon, scale_factor))
        if Transformer is not None:
            projected_crs = CRS.from_proj4(proj4)
        else:
            projected_crs = None
        projected_args = _local_tmerc_proj_args(origin_lat, origin_lon, scale_factor)
    else:
        projected_crs = f"EPSG:{map_epsg}" if Transformer is not None else None
        projected_args = _utm_proj_args(map_epsg)

    geographic_args = ["+proj=longlat", "+datum=WGS84", "+no_defs"]

    if Transformer is not None:
        forward_transformer = Transformer.from_crs("EPSG:4326", projected_crs, always_xy=True)
        inverse_transformer = Transformer.from_crs(projected_crs, "EPSG:4326", always_xy=True)

        def forward(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
            return [forward_transformer.transform(lon, lat) for lon, lat in points]

        def inverse(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
            return [inverse_transformer.transform(x, y) for x, y in points]

        return forward, inverse, "pyproj"

    def forward(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        return _run_cs2cs(points, geographic_args, projected_args)

    def inverse(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        return _run_cs2cs(points, projected_args, geographic_args)

    return forward, inverse, "cs2cs"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Transform Lanelet2 OSM in local/map XY (PCD-style rigid transform)."
    )
    parser.add_argument("--input", required=True, help="Input lanelet2 .osm path")
    parser.add_argument("--output", required=True, help="Output lanelet2 .osm path")
    parser.add_argument("--origin-lat", type=float, required=True, help="Map projector origin latitude")
    parser.add_argument("--origin-lon", type=float, required=True, help="Map projector origin longitude")
    parser.add_argument("--map-epsg", type=int, default=32652, help="Projected EPSG (default: 32652)")
    parser.add_argument(
        "--projection",
        choices=("utm", "local-tmerc"),
        default="utm",
        help="XY projection used for the rigid transform (default: utm)",
    )
    parser.add_argument(
        "--scale-factor",
        type=float,
        default=0.9996,
        help="Scale factor for --projection local-tmerc (default: 0.9996)",
    )
    parser.add_argument("--yaw-deg", type=float, default=0.0, help="Yaw in degrees (+CCW)")
    parser.add_argument("--tx", type=float, default=0.0, help="Translation X in local XY")
    parser.add_argument("--ty", type=float, default=0.0, help="Translation Y in local XY")
    parser.add_argument(
        "--map-version",
        type=int,
        help="Optional Lanelet2 MetaInfo map_version for the output",
    )
    parser.add_argument(
        "--write-local-tags",
        action="store_true",
        help="Create local_x/local_y tags for nodes that do not have them",
    )
    args = parser.parse_args()

    parser_with_comments = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(args.input, parser=parser_with_comments)
    root = tree.getroot()

    forward, inverse, backend = _make_projectors(
        args.projection,
        args.origin_lat,
        args.origin_lon,
        args.map_epsg,
        args.scale_factor,
    )
    origin_x, origin_y = forward([(args.origin_lon, args.origin_lat)])[0]
    c, s, tx, ty = make_2d_transform(args.yaw_deg, args.tx, args.ty)

    nodes = root.findall("node")
    if not nodes:
        raise RuntimeError("No <node> elements found in OSM.")

    lon_lat = [
        (_as_float(node_elem.get("lon")), _as_float(node_elem.get("lat")))
        for node_elem in nodes
    ]
    projected_xy = forward(lon_lat)
    transformed_local_xy: List[Tuple[float, float]] = []

    for mx, my in projected_xy:
        lx = mx - origin_x
        ly = my - origin_y
        transformed_local_xy.append(apply_2d_transform(lx, ly, c, s, tx, ty))

    transformed_projected_xy = [
        (origin_x + tlx, origin_y + tly)
        for tlx, tly in transformed_local_xy
    ]
    transformed_lon_lat = inverse(transformed_projected_xy)

    for node_elem, (tlx, tly), (tlon, tlat) in zip(
        nodes, transformed_local_xy, transformed_lon_lat
    ):

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

    if args.map_version is not None:
        meta_info = root.find("MetaInfo")
        if meta_info is None:
            meta_info = ET.Element("MetaInfo", {"format_version": "1", "validation_version": "1"})
            root.insert(0, meta_info)
        meta_info.set("map_version", str(args.map_version))

    tree.write(args.output, encoding="UTF-8", xml_declaration=True)

    print(f"[OK] Saved: {args.output}")
    print(f"     yaw_deg={args.yaw_deg}")
    print(f"     tx={args.tx}, ty={args.ty}")
    print(
        f"     origin=({args.origin_lat}, {args.origin_lon}), projection={args.projection}, "
        f"epsg={args.map_epsg}, scale_factor={args.scale_factor}"
    )
    print(f"     backend={backend}, nodes={len(nodes)}")


if __name__ == "__main__":
    main()
