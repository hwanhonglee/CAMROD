#!/usr/bin/env python3
"""Render and record operating points extracted from the current Park map."""

# HH_260810 - Keep semantic mission coordinates traceable to the exact OSM,
# shared LocalCartesian origin, and synchronized runtime YAML files.

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import lanelet2
from lanelet2.io import Origin
from lanelet2.projection import LocalCartesianProjector
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Polygon
import yaml


plt.switch_backend("Agg")

PNG_NAME = "park-operating-points.png"
JSON_NAME = "park-operating-points.json"
MANIFEST_NAME = "SHA256SUMS"
TURNAROUND_COLOR = "#2b7a9b"
ROADSIDE_COLOR = "#d17a22"
DROP_COLOR = "#7b4f9d"
PARKING_COLOR = "#4f8a64"


def sha256(path: Path) -> str:
    """Return one file SHA-256 digest."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_parameters(path: Path) -> dict:
    """Load the wildcard ROS parameter mapping from one YAML file."""
    return yaml.safe_load(path.read_text(encoding="utf-8"))["/**"][
        "ros__parameters"
    ]


def load_semantic_list(path: Path, key: str) -> list[dict]:
    """Load a semantic-area list from generated YAML."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    values = data[key]
    if not isinstance(values, list):
        raise ValueError(f"{path}: {key} is not a list")
    return values


def numeric_site_index(site: dict) -> int:
    """Return the numeric suffix of a camping-site type."""
    return int(str(site["type"]).rsplit("_", 1)[1])


def polygon_xy(area: dict) -> list[tuple[float, float]]:
    """Return map-plane polygon corners from one semantic YAML record."""
    return [(float(point["x"]), float(point["y"])) for point in area["corners"]]


def lanelet_polygon(lanelet) -> list[tuple[float, float]]:
    """Return one Lanelet2 road polygon."""
    left = [(point.x, point.y) for point in lanelet.leftBound]
    right = [(point.x, point.y) for point in lanelet.rightBound]
    return left + list(reversed(right))


def attribute_value(primitive, key: str, default: str = "") -> str:
    """Read one Python Lanelet2 attribute without assuming dict.get()."""
    return str(primitive.attributes[key]) if key in primitive.attributes else default


def polygon_metrics(points) -> dict:
    """Return centroid, area, and bounds for one map polygon."""
    xy = [(float(point.x), float(point.y)) for point in points]
    if len(xy) < 3:
        raise ValueError("polygon requires at least three points")
    cross_sum = 0.0
    center_x_sum = 0.0
    center_y_sum = 0.0
    for index, point in enumerate(xy):
        following = xy[(index + 1) % len(xy)]
        cross = point[0] * following[1] - following[0] * point[1]
        cross_sum += cross
        center_x_sum += (point[0] + following[0]) * cross
        center_y_sum += (point[1] + following[1]) * cross
    signed_area = 0.5 * cross_sum
    if abs(signed_area) < 1.0e-9:
        center_x = sum(point[0] for point in xy) / len(xy)
        center_y = sum(point[1] for point in xy) / len(xy)
    else:
        center_x = center_x_sum / (6.0 * signed_area)
        center_y = center_y_sum / (6.0 * signed_area)
    return {
        "center": {"x": round(center_x, 4), "y": round(center_y, 4)},
        "area_m2": round(abs(signed_area), 2),
        "bounds": {
            "min_x": round(min(point[0] for point in xy), 3),
            "min_y": round(min(point[1] for point in xy), 3),
            "max_x": round(max(point[0] for point in xy), 3),
            "max_y": round(max(point[1] for point in xy), 3),
        },
    }


def map_identity(path: Path) -> dict:
    """Read stable identity and element counts from Lanelet2 OSM."""
    root = ET.parse(path).getroot()
    metadata = root.find("MetaInfo")
    relation_tags = [
        {tag.attrib["k"]: tag.attrib["v"] for tag in relation.findall("tag")}
        for relation in root.findall("relation")
    ]
    return {
        "source_file": path.name,
        "map_version": (
            int(metadata.attrib["map_version"]) if metadata is not None else None
        ),
        "sha256": sha256(path),
        "node_count": len(root.findall("node")),
        "lanelet_count": sum(
            tags.get("type") == "lanelet" for tags in relation_tags
        ),
        "semantic_area_count": sum(
            tags.get("type") == "multipolygon" for tags in relation_tags
        ),
    }


def yaw_arrow(axis, x: float, y: float, yaw_deg: float, color: str) -> None:
    """Draw a short semantic-yaw arrow."""
    yaw = math.radians(yaw_deg)
    length = 1.7
    axis.add_patch(
        FancyArrowPatch(
            (x, y),
            (x + math.cos(yaw) * length, y + math.sin(yaw) * length),
            arrowstyle="-|>",
            mutation_scale=9,
            linewidth=1.2,
            color=color,
            zorder=8,
        )
    )


def draw_overview(axis, lanelet_map, sites: list[dict], drops: list[dict]) -> None:
    """Draw roads, parking lots, and service polygons in map coordinates."""
    extent_x = []
    extent_y = []
    for lanelet in lanelet_map.laneletLayer:
        points = lanelet_polygon(lanelet)
        extent_x.extend(point[0] for point in points)
        extent_y.extend(point[1] for point in points)
        axis.add_patch(
            Polygon(
                points,
                closed=True,
                facecolor="#f3f5f6",
                edgecolor="#c8d0d4",
                linewidth=0.65,
                zorder=0,
            )
        )

    for parking in lanelet_map.polygonLayer:
        if attribute_value(parking, "type") != "parking_lot":
            continue
        points = [(point.x, point.y) for point in parking]
        extent_x.extend(point[0] for point in points)
        extent_y.extend(point[1] for point in points)
        axis.add_patch(
            Polygon(
                points,
                closed=True,
                facecolor=PARKING_COLOR,
                edgecolor="#28553a",
                alpha=0.12,
                linewidth=1.1,
                hatch="..",
                zorder=1,
            )
        )

    for site in sites:
        mode = str(site["service_mode"])
        color = ROADSIDE_COLOR if mode == "roadside_stop" else TURNAROUND_COLOR
        points = polygon_xy(site)
        extent_x.extend(point[0] for point in points)
        extent_y.extend(point[1] for point in points)
        axis.add_patch(
            Polygon(
                points,
                closed=True,
                facecolor=color,
                edgecolor=color,
                alpha=0.20,
                linewidth=1.3,
                zorder=4,
            )
        )
        x = float(site["x"])
        y = float(site["y"])
        site_index = numeric_site_index(site)
        axis.text(
            x,
            y,
            f"B{site_index}",
            ha="center",
            va="center",
            fontsize=8,
            fontweight="bold",
            color="#162a33",
            zorder=9,
        )
        yaw_arrow(axis, x, y, float(site["yaw_deg"]), color)

    for drop in drops:
        points = polygon_xy(drop)
        extent_x.extend(point[0] for point in points)
        extent_y.extend(point[1] for point in points)
        axis.add_patch(
            Polygon(
                points,
                closed=True,
                facecolor=DROP_COLOR,
                edgecolor=DROP_COLOR,
                alpha=0.24,
                linewidth=1.8,
                zorder=5,
            )
        )
        x = float(drop["x"])
        y = float(drop["y"])
        axis.text(
            x,
            y,
            "DZ",
            ha="center",
            va="center",
            fontsize=8,
            fontweight="bold",
            color="#2b1636",
            zorder=9,
        )
        yaw_arrow(axis, x, y, float(drop["yaw_deg"]), DROP_COLOR)

    axis.set_title("Current Park map | semantic operating points", loc="left")
    padding = 3.0
    axis.set_xlim(min(extent_x) - padding, max(extent_x) + padding)
    axis.set_ylim(min(extent_y) - padding, max(extent_y) + padding)
    axis.set_xlabel("map X (m)")
    axis.set_ylabel("map Y (m)")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(color="#e3e8ea", linewidth=0.6)
    axis.set_facecolor("#ffffff")
    for spine in axis.spines.values():
        spine.set_color("#c7d0d4")


def draw_table(axis, report: dict) -> None:
    """Draw an operator-readable coordinate and provenance table."""
    axis.axis("off")
    map_data = report["map"]
    origin = report["origin"]
    axis.text(
        0.0,
        0.99,
        "Park Operating Coordinates",
        transform=axis.transAxes,
        fontsize=17,
        fontweight="bold",
        va="top",
        color="#18323d",
    )
    axis.text(
        0.0,
        0.94,
        (
            f"map v{map_data['map_version']}  SHA {map_data['sha256'][:12]}...\n"
            f"origin {origin['latitude']:.7f}, {origin['longitude']:.7f}"
        ),
        transform=axis.transAxes,
        fontsize=9,
        va="top",
        color="#53656d",
    )

    headers = ("ID", "X (m)", "Y (m)", "yaw", "mode")
    rows = []
    for site in report["camping_sites"]:
        rows.append(
            (
                f"B{numeric_site_index(site)}",
                f"{float(site['x']):.3f}",
                f"{float(site['y']):.3f}",
                f"{float(site['yaw_deg']):.1f}",
                "roadside" if site["service_mode"] == "roadside_stop" else "turn",
            )
        )
    drop = report["drop_zones"][0]
    rows.append(
        (
            "DZ",
            f"{float(drop['x']):.3f}",
            f"{float(drop['y']):.3f}",
            f"{float(drop['yaw_deg']):.1f}",
            "reverse",
        )
    )
    table = axis.table(
        cellText=rows,
        colLabels=headers,
        cellLoc="right",
        colLoc="right",
        colWidths=[0.11, 0.20, 0.20, 0.16, 0.23],
        bbox=[0.0, 0.30, 1.0, 0.58],
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    for (row, _column), cell in table.get_celld().items():
        cell.set_edgecolor("#d2dadd")
        if row == 0:
            cell.set_facecolor("#e8eef0")
            cell.set_text_props(weight="bold", color="#18323d")
        elif row == len(rows):
            cell.set_facecolor("#f0eaf4")

    axis.text(
        0.0,
        0.25,
        "Parking-lot polygons",
        transform=axis.transAxes,
        fontsize=10,
        fontweight="bold",
        color="#28553a",
    )
    parking_lines = [
        (
            f"way {item['id']}: center "
            f"({item['center']['x']:.3f}, {item['center']['y']:.3f}), "
            f"{item['area_m2']:.1f} m2"
        )
        for item in report["parking_lots"]
    ]
    axis.text(
        0.0,
        0.22,
        "\n".join(parking_lines),
        transform=axis.transAxes,
        fontsize=8.5,
        va="top",
        linespacing=1.45,
        color="#43565e",
    )
    axis.text(
        0.0,
        0.03,
        (
            "Geometry: camrod_map area_exporter + shared Park origin\n"
            "Policy: B1-B10 turnaround | B11-B13 roadside_stop\n"
            "Arrows show map-authored semantic yaw; coordinates are map-frame meters."
        ),
        transform=axis.transAxes,
        fontsize=8,
        va="bottom",
        color="#62737a",
    )


def build_report(
    map_path: Path,
    map_info_path: Path,
    camping_sites_path: Path,
    drop_zones_path: Path,
) -> tuple[dict, object]:
    """Load and validate map/config inputs, then return one stable report."""
    parameters = load_parameters(map_info_path)
    origin = {
        "latitude": float(parameters["offset_lat"]),
        "longitude": float(parameters["offset_lon"]),
        "altitude": float(parameters["offset_alt"]),
        "yaw_offset_deg": float(parameters["yaw_offset_deg"]),
    }
    projector = LocalCartesianProjector(
        Origin(origin["latitude"], origin["longitude"], origin["altitude"])
    )
    lanelet_map = lanelet2.io.load(str(map_path), projector)
    sites = sorted(
        load_semantic_list(camping_sites_path, "camping_sites"),
        key=numeric_site_index,
    )
    drops = load_semantic_list(drop_zones_path, "drop_zones")
    if [numeric_site_index(site) for site in sites] != list(range(1, 14)):
        raise ValueError("camping-site YAML must contain B1 through B13 exactly once")
    if len(drops) != 1 or drops[0]["type"] != "drop_zone":
        raise ValueError("drop-zone YAML must contain exactly one drop_zone")
    for site in sites:
        expected_mode = (
            "roadside_stop" if numeric_site_index(site) >= 11 else "turnaround"
        )
        if site.get("service_mode") != expected_mode:
            raise ValueError(f"{site['type']} service_mode is not {expected_mode}")

    parking_lots = []
    for parking in lanelet_map.polygonLayer:
        if attribute_value(parking, "type") != "parking_lot":
            continue
        parking_lots.append(
            {"id": int(parking.id), "type": "parking_lot", **polygon_metrics(parking)}
        )
    parking_lots.sort(key=lambda item: item["id"])
    if len(parking_lots) != 3:
        raise ValueError("current Park map must expose three parking_lot polygons")

    report = {
        "generated_for": "2026-08-10 current Park coordinate synchronization",
        "field_claim": False,
        "map": map_identity(map_path),
        "origin": origin,
        "inputs": {
            "map_info": str(map_info_path.name),
            "camping_sites": str(camping_sites_path.name),
            "drop_zones": str(drop_zones_path.name),
        },
        "camping_sites": sites,
        "drop_zones": drops,
        "parking_lots": parking_lots,
        "validation": {
            "camping_site_count": len(sites),
            "drop_zone_count": len(drops),
            "parking_lot_count": len(parking_lots),
            "service_modes_preserved": True,
            "runtime_test_required": True,
        },
    }
    return report, lanelet_map


def render(report: dict, lanelet_map, output_dir: Path) -> None:
    """Write the map overview, source report, and checksums."""
    output_dir.mkdir(parents=True, exist_ok=True)
    figure = plt.figure(figsize=(16, 10))
    grid = figure.add_gridspec(
        1,
        2,
        width_ratios=(2.05, 1.0),
        left=0.06,
        right=0.98,
        bottom=0.08,
        top=0.88,
        wspace=0.14,
    )
    draw_overview(
        figure.add_subplot(grid[0, 0]),
        lanelet_map,
        report["camping_sites"],
        report["drop_zones"],
    )
    draw_table(figure.add_subplot(grid[0, 1]), report)
    figure.suptitle(
        "CAMROD Park Semantic Coordinate Contract",
        x=0.02,
        ha="left",
        fontsize=20,
        fontweight="bold",
        color="#18323d",
    )
    figure.savefig(output_dir / PNG_NAME, dpi=150, facecolor="white")
    plt.close(figure)

    (output_dir / JSON_NAME).write_text(
        json.dumps(report, indent=2, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    manifest_lines = [
        f"{sha256(output_dir / name)}  {name}" for name in (PNG_NAME, JSON_NAME)
    ]
    (output_dir / MANIFEST_NAME).write_text(
        "\n".join(manifest_lines) + "\n",
        encoding="utf-8",
    )


def parse_args() -> argparse.Namespace:
    """Parse source and output paths."""
    repo_root = Path(__file__).resolve().parents[3]
    default_output = (
        repo_root
        / "docs"
        / "assets"
        / "module-guides"
        / "map"
        / "test-results"
        / "park-operating-points-20260810"
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=repo_root / "lanelet2_maps.osm")
    parser.add_argument(
        "--map-info",
        type=Path,
        default=repo_root / "camrod_map" / "config" / "map_info.yaml",
    )
    parser.add_argument(
        "--camping-sites",
        type=Path,
        default=repo_root / "camrod_planning" / "config" / "camping_sites.yaml",
    )
    parser.add_argument(
        "--drop-zones",
        type=Path,
        default=repo_root / "camrod_map" / "config" / "drop_zones.yaml",
    )
    parser.add_argument("--output-dir", type=Path, default=default_output)
    return parser.parse_args()


def main() -> int:
    """Render current Park operating coordinates."""
    args = parse_args()
    report, lanelet_map = build_report(
        args.map.resolve(),
        args.map_info.resolve(),
        args.camping_sites.resolve(),
        args.drop_zones.resolve(),
    )
    render(report, lanelet_map, args.output_dir.resolve())
    print(f"wrote {args.output_dir.resolve() / PNG_NAME}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
