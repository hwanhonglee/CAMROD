#!/usr/bin/env python3
"""Render the active tapered, rounded robot boundary and rigid-body motion."""

# HH_260810 - Keep the boundary documentation derived from the canonical
# sensor-kit geometry and reject drift from the Nav2 local/global footprints.

import argparse
import ast
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Polygon
import numpy as np
from PIL import Image
import yaml


plt.switch_backend("Agg")


PNG_NAME = "tapered-rounded-boundary-geometry.png"
GIF_NAME = "tapered-rounded-boundary-motion.gif"
RESULT_NAME = "result.json"
MANIFEST_NAME = "SHA256SUMS"
EPSILON = 1.0e-9


@dataclass(frozen=True)
class BoundaryContract:
    """Canonical body values plus the generated and deployed contours."""

    extents: dict
    margins: dict
    front_taper: float
    front_shoulder_depth: float
    corner_radius: float
    corner_samples: int
    physical: tuple
    planning: tuple
    nav2_local: tuple
    nav2_global: tuple
    source_hashes: dict


def add(lhs, rhs):
    """Add two planar points."""
    return lhs[0] + rhs[0], lhs[1] + rhs[1]


def subtract(lhs, rhs):
    """Subtract two planar points."""
    return lhs[0] - rhs[0], lhs[1] - rhs[1]


def multiply(point, scale):
    """Scale one planar point."""
    return point[0] * scale, point[1] * scale


def dot(lhs, rhs):
    """Return the planar dot product."""
    return lhs[0] * rhs[0] + lhs[1] * rhs[1]


def cross(lhs, rhs):
    """Return the scalar planar cross product."""
    return lhs[0] * rhs[1] - lhs[1] * rhs[0]


def norm(point):
    """Return the Euclidean point norm."""
    return math.hypot(point[0], point[1])


def normalized(point):
    """Return a unit point, or zero for a degenerate input."""
    length = norm(point)
    if length <= EPSILON:
        return 0.0, 0.0
    return point[0] / length, point[1] / length


def signed_area(polygon):
    """Return the signed area of one polygon."""
    twice_area = 0.0
    for index, current in enumerate(polygon):
        twice_area += cross(current, polygon[(index + 1) % len(polygon)])
    return 0.5 * twice_area


def make_sharp_boundary(extents, front_taper, shoulder_depth):
    """Build the same clockwise six-corner body used by robot_boundary.hpp."""
    front = max(0.0, extents["front"])
    rear = max(0.0, extents["rear"])
    left = max(0.0, extents["left"])
    right = max(0.0, extents["right"])
    total_width = left + right
    total_length = front + rear
    taper = min(max(front_taper, 0.0), 0.45 * total_width)
    depth = min(max(shoulder_depth, 0.0), 0.45 * total_length)
    if taper <= EPSILON or depth <= EPSILON:
        return (
            (front, left),
            (front, -right),
            (-rear, -right),
            (-rear, left),
        )
    return (
        (front, left - taper),
        (front, -right + taper),
        (front - depth, -right),
        (-rear, -right),
        (-rear, left),
        (front - depth, left),
    )


def round_convex_polygon(sharp_polygon, requested_radius, requested_samples):
    """Mirror the C++ tangent-arc fillet construction exactly."""
    if len(sharp_polygon) < 3 or requested_radius <= EPSILON:
        return tuple(sharp_polygon)
    clockwise = signed_area(sharp_polygon) < 0.0
    samples = max(1, requested_samples)
    rounded = []
    for index, vertex in enumerate(sharp_polygon):
        previous = sharp_polygon[(index - 1) % len(sharp_polygon)]
        next_point = sharp_polygon[(index + 1) % len(sharp_polygon)]
        toward_previous = subtract(previous, vertex)
        toward_next = subtract(next_point, vertex)
        previous_length = norm(toward_previous)
        next_length = norm(toward_next)
        if previous_length <= EPSILON or next_length <= EPSILON:
            rounded.append(vertex)
            continue

        previous_unit = multiply(toward_previous, 1.0 / previous_length)
        next_unit = multiply(toward_next, 1.0 / next_length)
        interior_angle = math.acos(
            min(1.0, max(-1.0, dot(previous_unit, next_unit)))
        )
        half_angle = 0.5 * interior_angle
        tangent_denominator = math.tan(half_angle)
        if half_angle <= EPSILON or abs(tangent_denominator) <= EPSILON:
            rounded.append(vertex)
            continue

        tangent_distance = requested_radius / tangent_denominator
        tangent_distance = min(
            tangent_distance, 0.49 * min(previous_length, next_length)
        )
        effective_radius = tangent_distance * tangent_denominator
        tangent_start = add(vertex, multiply(previous_unit, tangent_distance))
        tangent_end = add(vertex, multiply(next_unit, tangent_distance))
        bisector = normalized(add(previous_unit, next_unit))
        center_distance = effective_radius / math.sin(half_angle)
        center = add(vertex, multiply(bisector, center_distance))

        start_vector = subtract(tangent_start, center)
        end_vector = subtract(tangent_end, center)
        start_angle = math.atan2(start_vector[1], start_vector[0])
        arc_angle = math.atan2(
            cross(start_vector, end_vector), dot(start_vector, end_vector)
        )
        if clockwise and arc_angle > 0.0:
            arc_angle -= 2.0 * math.pi
        elif not clockwise and arc_angle < 0.0:
            arc_angle += 2.0 * math.pi

        for sample in range(samples + 1):
            ratio = sample / samples
            angle = start_angle + ratio * arc_angle
            rounded.append(
                (
                    center[0] + effective_radius * math.cos(angle),
                    center[1] + effective_radius * math.sin(angle),
                )
            )
    return tuple(rounded)


def directional_margin(outward_normal, margins):
    """Return the directional anisotropic margin used by the C++ generator."""
    longitudinal = margins["front"] if outward_normal[0] >= 0.0 else margins["rear"]
    lateral = margins["left"] if outward_normal[1] >= 0.0 else margins["right"]
    return math.hypot(
        outward_normal[0] * max(0.0, longitudinal),
        outward_normal[1] * max(0.0, lateral),
    )


def intersect_lines(first, second, fallback):
    """Intersect two infinite lines represented by point and direction."""
    first_point, first_direction = first
    second_point, second_direction = second
    denominator = cross(first_direction, second_direction)
    if abs(denominator) <= EPSILON:
        return fallback
    distance = cross(
        subtract(second_point, first_point), second_direction
    ) / denominator
    return add(first_point, multiply(first_direction, distance))


def offset_convex_polygon(polygon, margins):
    """Offset a convex polygon with the same directional-margin rule as C++."""
    clockwise = signed_area(polygon) < 0.0
    shifted_edges = []
    for index, point in enumerate(polygon):
        direction = subtract(polygon[(index + 1) % len(polygon)], point)
        edge_length = norm(direction)
        if edge_length <= EPSILON:
            return tuple(polygon)
        unit = multiply(direction, 1.0 / edge_length)
        outward = (-unit[1], unit[0]) if clockwise else (unit[1], -unit[0])
        margin = directional_margin(outward, margins)
        shifted_edges.append((add(point, multiply(outward, margin)), direction))

    expanded = []
    for index, point in enumerate(polygon):
        expanded.append(
            intersect_lines(shifted_edges[index - 1], shifted_edges[index], point)
        )
    return tuple(expanded)


def make_boundaries(extents, margins, front_taper, shoulder_depth, radius, samples):
    """Create physical and exact offset planning contours."""
    sharp = make_sharp_boundary(extents, front_taper, shoulder_depth)
    physical = round_convex_polygon(sharp, radius, samples)
    expanded_sharp = offset_convex_polygon(sharp, margins)
    minimum_margin = max(0.0, min(margins.values()))
    planning = round_convex_polygon(
        expanded_sharp, max(0.0, radius) + minimum_margin, samples
    )
    return sharp, physical, expanded_sharp, planning


def sha256(path):
    """Return one file SHA-256."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_footprint(value):
    """Normalize a ROS footprint string/list to immutable float points."""
    parsed = ast.literal_eval(value) if isinstance(value, str) else value
    return tuple((float(point[0]), float(point[1])) for point in parsed)


def load_contract(repo_root):
    """Load canonical YAML and reject any generated/deployed contour mismatch."""
    sensor_path = repo_root / "camrod_sensor_kit" / "config" / "robot_params.yaml"
    nav2_path = repo_root / "camrod_planning" / "config" / "nav2_vehicle.yaml"
    sensor = yaml.safe_load(sensor_path.read_text(encoding="utf-8"))
    robot = sensor["/**"]["ros__parameters"]["robot"]
    body = robot["body_extents"]
    extents = {
        key: float(body[key]) for key in ("front", "rear", "left", "right")
    }
    margins = {
        "front": float(body["planning_margin"]),
        "rear": float(body["planning_margin"]),
        "left": float(body["planning_lateral_margin"]),
        "right": float(body["planning_lateral_margin"]),
    }
    front_taper = float(body["front_taper"])
    shoulder_depth = float(body["front_shoulder_depth"])
    radius = float(body["corner_radius"])
    samples = int(body["corner_samples"])
    _, physical, _, planning = make_boundaries(
        extents,
        margins,
        front_taper,
        shoulder_depth,
        radius,
        samples,
    )

    nav2 = yaml.safe_load(nav2_path.read_text(encoding="utf-8"))
    nav2_local = parse_footprint(
        nav2["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
    )
    nav2_global = parse_footprint(
        nav2["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"]
    )
    generated_six_decimal = tuple(
        (round(point[0], 6), round(point[1], 6)) for point in planning
    )
    if generated_six_decimal != nav2_local or nav2_local != nav2_global:
        raise ValueError(
            "generated planning boundary does not match Nav2 local/global footprints"
        )

    return BoundaryContract(
        extents=extents,
        margins=margins,
        front_taper=front_taper,
        front_shoulder_depth=shoulder_depth,
        corner_radius=radius,
        corner_samples=samples,
        physical=physical,
        planning=planning,
        nav2_local=nav2_local,
        nav2_global=nav2_global,
        source_hashes={
            sensor_path.relative_to(repo_root).as_posix(): sha256(sensor_path),
            nav2_path.relative_to(repo_root).as_posix(): sha256(nav2_path),
        },
    )


def display_points(points):
    """Map robot X-forward/Y-left coordinates to front-up display coordinates."""
    return np.asarray([(-point[1], point[0]) for point in points])


def add_boundary_patch(axis, points, face, edge, alpha, linewidth, label=None):
    """Add one closed boundary polygon to a front-up top view."""
    patch = Polygon(
        display_points(points),
        closed=True,
        facecolor=face,
        edgecolor=edge,
        alpha=alpha,
        linewidth=linewidth,
        joinstyle="round",
        label=label,
    )
    axis.add_patch(patch)
    return patch


def style_axis(axis):
    """Apply the shared quiet documentation style."""
    axis.set_facecolor("#ffffff")
    for spine in axis.spines.values():
        spine.set_color("#d8e0e3")
    axis.grid(color="#e7ecee", linewidth=0.8, zorder=0)


def render_geometry(contract, output):
    """Render exact geometry, dimensions, construction, and safety meaning."""
    sharp = make_sharp_boundary(
        contract.extents,
        contract.front_taper,
        contract.front_shoulder_depth,
    )
    figure = plt.figure(figsize=(16, 9), facecolor="#f3f6f7")
    grid = figure.add_gridspec(
        2,
        2,
        width_ratios=(1.45, 1.0),
        height_ratios=(1.08, 0.92),
        left=0.055,
        right=0.96,
        top=0.82,
        bottom=0.12,
        hspace=0.27,
        wspace=0.22,
    )
    main_axis = figure.add_subplot(grid[:, 0])
    detail_axis = figure.add_subplot(grid[0, 1])
    policy_axis = figure.add_subplot(grid[1, 1])

    figure.text(
        0.055,
        0.925,
        "Tapered-front, rounded robot boundary",
        fontsize=26,
        fontweight="bold",
        color="#17272d",
    )
    figure.text(
        0.055,
        0.875,
        "One source contour shared by visualization, Nav2, and final lanelet safety",
        fontsize=13.5,
        color="#50636b",
    )
    figure.text(
        0.945,
        0.925,
        "SOURCE-DERIVED",
        ha="right",
        fontsize=12,
        fontweight="bold",
        color="#7a4d00",
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "#fff2cf", "edgecolor": "#dfb75b"},
    )

    style_axis(main_axis)
    main_axis.set_title("Current top-view envelopes", loc="left", fontsize=15, fontweight="bold")
    add_boundary_patch(
        main_axis,
        contract.planning,
        "#f5c85b",
        "#b47600",
        0.30,
        3.0,
        "Planning boundary (+0.10 m)",
    )
    add_boundary_patch(
        main_axis,
        contract.physical,
        "#53c7d7",
        "#00788a",
        0.58,
        3.0,
        "Physical body (hard-stop envelope)",
    )
    sharp_display = display_points(sharp)
    sharp_closed = np.vstack((sharp_display, sharp_display[0]))
    main_axis.plot(
        sharp_closed[:, 0],
        sharp_closed[:, 1],
        color="#245a62",
        linestyle=(0, (4, 4)),
        linewidth=1.2,
        alpha=0.65,
        label="Sharp construction polygon",
    )
    main_axis.scatter([0.0], [0.0], s=85, color="#17272d", zorder=8)
    main_axis.text(0.04, -0.02, "robot_center_link", fontsize=11, color="#17272d")
    main_axis.annotate(
        "+X FRONT",
        xy=(0.0, 0.96),
        xytext=(0.0, 0.18),
        ha="center",
        va="center",
        fontsize=11,
        fontweight="bold",
        color="#9a3b24",
        arrowprops={"arrowstyle": "-|>", "color": "#c1492f", "lw": 2.5},
    )
    main_axis.annotate(
        "+Y LEFT",
        xy=(-0.94, 0.0),
        xytext=(-0.22, 0.0),
        ha="center",
        va="bottom",
        fontsize=10,
        fontweight="bold",
        color="#40565e",
        arrowprops={"arrowstyle": "-|>", "color": "#40565e", "lw": 1.7},
    )

    planning_width = contract.extents["left"] + contract.extents["right"] + 0.20
    planning_length = contract.extents["front"] + contract.extents["rear"] + 0.20
    width_y = -0.91
    main_axis.annotate(
        "",
        xy=(-contract.extents["left"] - 0.10, width_y),
        xytext=(contract.extents["right"] + 0.10, width_y),
        arrowprops={"arrowstyle": "<->", "color": "#8b620d", "lw": 1.8},
    )
    main_axis.text(
        0.0,
        width_y - 0.055,
        f"planning width  {planning_width:.5f} m",
        ha="center",
        va="top",
        fontsize=10.5,
        color="#75500a",
        fontweight="bold",
    )
    length_x = 0.88
    main_axis.annotate(
        "",
        xy=(length_x, contract.extents["front"] + 0.10),
        xytext=(length_x, -contract.extents["rear"] - 0.10),
        arrowprops={"arrowstyle": "<->", "color": "#8b620d", "lw": 1.8},
    )
    main_axis.text(
        length_x + 0.04,
        0.0,
        f"planning length  {planning_length:.5f} m",
        rotation=90,
        ha="left",
        va="center",
        fontsize=10.5,
        color="#75500a",
        fontweight="bold",
    )
    main_axis.set_xlim(-1.05, 1.08)
    main_axis.set_ylim(-1.02, 1.03)
    main_axis.set_aspect("equal")
    main_axis.set_xticks(np.arange(-1.0, 1.01, 0.25))
    main_axis.set_yticks(np.arange(-1.0, 1.01, 0.25))
    main_axis.legend(loc="upper left", frameon=True, facecolor="#ffffff", fontsize=10)

    style_axis(detail_axis)
    detail_axis.set_title("Front construction detail", loc="left", fontsize=14, fontweight="bold")
    add_boundary_patch(
        detail_axis,
        contract.planning,
        "#f5c85b",
        "#b47600",
        0.22,
        2.3,
    )
    add_boundary_patch(
        detail_axis,
        contract.physical,
        "#53c7d7",
        "#00788a",
        0.50,
        2.3,
    )
    detail_axis.plot(
        sharp_closed[:, 0],
        sharp_closed[:, 1],
        color="#245a62",
        linestyle=(0, (4, 4)),
        linewidth=1.2,
    )
    detail_axis.annotate(
        "short front face",
        xy=(0.0, contract.extents["front"]),
        xytext=(0.0, contract.extents["front"] + 0.14),
        ha="center",
        fontsize=10.5,
        fontweight="bold",
        arrowprops={"arrowstyle": "->", "color": "#40565e"},
    )
    detail_axis.annotate(
        "0.12 m taper\n0.12 m depth",
        xy=(-contract.extents["left"] + 0.06, contract.extents["front"] - 0.06),
        xytext=(-0.76, 0.37),
        ha="center",
        fontsize=10,
        color="#40565e",
        arrowprops={"arrowstyle": "->", "color": "#40565e"},
    )
    detail_axis.text(
        0.76,
        0.37,
        "body R0.05 m\nplanning R0.15 m",
        ha="center",
        fontsize=10,
        color="#40565e",
    )
    detail_axis.set_xlim(-0.88, 0.88)
    detail_axis.set_ylim(0.23, 0.94)
    detail_axis.set_aspect("equal")
    detail_axis.set_xticks([])
    detail_axis.set_yticks([])

    policy_axis.set_facecolor("#ffffff")
    policy_axis.set_xlim(0.0, 1.0)
    policy_axis.set_ylim(0.0, 1.0)
    policy_axis.axis("off")
    policy_axis.set_title("Dimensions and runtime meaning", loc="left", fontsize=14, fontweight="bold")
    rows = (
        ("Physical", "1.39160 x 1.07000 m", "cyan", "ordinary overlap => stop"),
        ("Planning", "1.59160 x 1.27000 m", "yellow", "projected recovery limit"),
        ("Origin", "robot_center_link", "black", "axle midpoint"),
        ("Polygon", "30 points each", "black", "4 arc segments / corner"),
    )
    y = 0.79
    colors = {"cyan": "#00788a", "yellow": "#b47600", "black": "#263a42"}
    for name, value, color_key, meaning in rows:
        policy_axis.add_patch(
            plt.Rectangle((0.02, y - 0.055), 0.012, 0.115, color=colors[color_key])
        )
        policy_axis.text(0.06, y + 0.027, name, fontsize=11.5, fontweight="bold", va="center")
        policy_axis.text(0.34, y + 0.027, value, fontsize=11.3, va="center", color="#263a42")
        policy_axis.text(0.34, y - 0.030, meaning, fontsize=9.8, va="center", color="#64757c")
        y -= 0.205

    figure.text(
        0.055,
        0.035,
        "Repository geometry only | Nav2 local/global footprints matched at 6 decimals | not runtime or field evidence",
        fontsize=10.5,
        color="#684c21",
        fontweight="bold",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150, facecolor=figure.get_facecolor())
    plt.close(figure)


def transform_points(points, x, y, yaw):
    """Transform robot-local points into one fixed map frame."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return np.asarray(
        [
            (
                x + cosine * point[0] - sine * point[1],
                y + sine * point[0] + cosine * point[1],
            )
            for point in points
        ]
    )


def motion_poses(frames_per_phase=12):
    """Create continuous forward, curve, crab, and zero-turn poses."""
    poses = []
    start_x, start_y = -2.7, -1.05
    start_yaw = math.radians(15.0)
    for frame in range(frames_per_phase):
        ratio = frame / (frames_per_phase - 1)
        distance = 1.8 * ratio
        poses.append(
            {
                "phase": "FORWARD",
                "detail": "center translates | yaw fixed",
                "x": start_x + distance * math.cos(start_yaw),
                "y": start_y + distance * math.sin(start_yaw),
                "yaw": start_yaw,
                "progress": ratio,
            }
        )
    forward_end = poses[-1]
    curve_delta = math.radians(50.0)
    radius = 2.0
    for frame in range(frames_per_phase):
        ratio = frame / (frames_per_phase - 1)
        yaw = start_yaw + curve_delta * ratio
        poses.append(
            {
                "phase": "CURVED DRIVE",
                "detail": "center and yaw change together",
                "x": forward_end["x"] + radius * (math.sin(yaw) - math.sin(start_yaw)),
                "y": forward_end["y"] - radius * (math.cos(yaw) - math.cos(start_yaw)),
                "yaw": yaw,
                "progress": ratio,
            }
        )
    curve_end = poses[-1]
    for frame in range(frames_per_phase):
        ratio = frame / (frames_per_phase - 1)
        distance = 0.90 * ratio
        poses.append(
            {
                "phase": "CRAB RIGHT",
                "detail": "lateral translation | yaw fixed",
                "x": curve_end["x"] + distance * math.sin(curve_end["yaw"]),
                "y": curve_end["y"] - distance * math.cos(curve_end["yaw"]),
                "yaw": curve_end["yaw"],
                "progress": ratio,
            }
        )
    crab_end = poses[-1]
    for frame in range(frames_per_phase):
        ratio = frame / (frames_per_phase - 1)
        poses.append(
            {
                "phase": "ZERO TURN",
                "detail": "center fixed | yaw changes",
                "x": crab_end["x"],
                "y": crab_end["y"],
                "yaw": crab_end["yaw"] + math.radians(120.0) * ratio,
                "progress": ratio,
            }
        )
    return tuple(poses)


def render_motion_frame(contract, poses, index):
    """Render one animation frame as an RGB Pillow image."""
    pose = poses[index]
    figure, axis = plt.subplots(figsize=(12, 7.2), facecolor="#f3f6f7")
    figure.subplots_adjust(left=0.07, right=0.97, top=0.82, bottom=0.12)
    style_axis(axis)
    axis.set_aspect("equal")
    axis.set_xlim(-3.55, 3.15)
    axis.set_ylim(-2.15, 2.15)
    axis.set_xlabel("map X (m)")
    axis.set_ylabel("map Y (m)")
    axis.set_xticks(np.arange(-3.0, 3.1, 0.5))
    axis.set_yticks(np.arange(-2.0, 2.1, 0.5))

    centers = np.asarray([(item["x"], item["y"]) for item in poses[: index + 1]])
    axis.plot(
        centers[:, 0],
        centers[:, 1],
        color="#596c73",
        linewidth=2.0,
        linestyle=(0, (3, 3)),
        alpha=0.80,
        zorder=2,
    )

    phase_starts = (0, 12, 24, 36)
    for phase_index in phase_starts:
        if phase_index >= index:
            continue
        ghost = poses[phase_index]
        ghost_points = transform_points(
            contract.planning, ghost["x"], ghost["y"], ghost["yaw"]
        )
        axis.add_patch(
            Polygon(
                ghost_points,
                closed=True,
                facecolor="none",
                edgecolor="#b7c0c3",
                linewidth=1.1,
                linestyle=(0, (3, 3)),
                alpha=0.65,
                zorder=1,
            )
        )

    planning = transform_points(
        contract.planning, pose["x"], pose["y"], pose["yaw"]
    )
    physical = transform_points(
        contract.physical, pose["x"], pose["y"], pose["yaw"]
    )
    axis.add_patch(
        Polygon(
            planning,
            closed=True,
            facecolor="#f5c85b",
            edgecolor="#b47600",
            alpha=0.30,
            linewidth=3.0,
            joinstyle="round",
            zorder=4,
        )
    )
    axis.add_patch(
        Polygon(
            physical,
            closed=True,
            facecolor="#53c7d7",
            edgecolor="#00788a",
            alpha=0.67,
            linewidth=3.0,
            joinstyle="round",
            zorder=5,
        )
    )
    axis.scatter([pose["x"]], [pose["y"]], color="#17272d", s=85, zorder=7)
    heading_length = 0.88
    axis.annotate(
        "",
        xy=(
            pose["x"] + heading_length * math.cos(pose["yaw"]),
            pose["y"] + heading_length * math.sin(pose["yaw"]),
        ),
        xytext=(pose["x"], pose["y"]),
        arrowprops={"arrowstyle": "-|>", "color": "#c1492f", "lw": 2.8},
        zorder=8,
    )
    axis.text(
        pose["x"] + 0.05,
        pose["y"] - 0.13,
        "robot_center_link",
        fontsize=9.5,
        color="#17272d",
        zorder=8,
    )

    if pose["phase"] == "CRAB RIGHT":
        right_x = math.sin(pose["yaw"])
        right_y = -math.cos(pose["yaw"])
        axis.annotate(
            "",
            xy=(pose["x"] + 1.0 * right_x, pose["y"] + 1.0 * right_y),
            xytext=(pose["x"] + 0.3 * right_x, pose["y"] + 0.3 * right_y),
            arrowprops={"arrowstyle": "-|>", "color": "#b47600", "lw": 2.4},
        )
        axis.text(
            pose["x"] + 1.02 * right_x,
            pose["y"] + 1.02 * right_y,
            "CRAB RIGHT",
            ha="left",
            va="top",
            color="#684c21",
            fontsize=9.5,
            fontweight="bold",
        )
    elif pose["phase"] == "ZERO TURN":
        axis.add_patch(
            Arc(
                (pose["x"], pose["y"]),
                1.95,
                1.95,
                theta1=15,
                theta2=300,
                color="#7d3c98",
                linewidth=2.5,
                zorder=3,
            )
        )
        axis.text(
            pose["x"] - 1.18,
            pose["y"] + 0.96,
            "yaw about fixed center",
            fontsize=9.5,
            fontweight="bold",
            color="#6d3486",
        )

    figure.text(
        0.07,
        0.93,
        "Boundary motion about robot_center_link",
        fontsize=22,
        fontweight="bold",
        color="#17272d",
    )
    figure.text(
        0.07,
        0.875,
        f"{pose['phase']}  |  {pose['detail']}",
        fontsize=13,
        fontweight="bold",
        color="#40565e",
    )
    figure.text(
        0.95,
        0.91,
        f"x {pose['x']:+.2f} m   y {pose['y']:+.2f} m   yaw {math.degrees(pose['yaw']):.1f} deg",
        ha="right",
        fontsize=11,
        color="#40565e",
    )
    figure.text(
        0.07,
        0.045,
        "cyan: physical hard-stop envelope   |   yellow: exact +0.10 m planning contour   |   SOURCE-DERIVED, not runtime evidence",
        fontsize=10.5,
        color="#684c21",
        fontweight="bold",
    )

    figure.canvas.draw()
    width, height = figure.canvas.get_width_height()
    rgba = np.asarray(figure.canvas.buffer_rgba())
    image = Image.fromarray(rgba[:, :, :3].copy(), mode="RGB")
    if image.size != (width, height):
        raise RuntimeError("rendered frame size does not match the Matplotlib canvas")
    plt.close(figure)
    return image


def render_motion(contract, output):
    """Render the four-phase rigid-body boundary animation."""
    poses = motion_poses()
    frames = [render_motion_frame(contract, poses, index) for index in range(len(poses))]
    output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=120,
        loop=0,
        disposal=2,
        optimize=False,
    )


def point_list(points):
    """Round points to the precision deployed in Nav2 YAML."""
    return [[round(point[0], 6), round(point[1], 6)] for point in points]


def result_payload(contract):
    """Return the stable machine-readable provenance for both visuals."""
    return {
        "schema_version": 1,
        "date": "2026-08-10",
        "classification": "SOURCE-DERIVED",
        "runtime_claim": False,
        "field_claim": False,
        "frame": "robot_center_link",
        "coordinate_convention": {"x": "forward", "y": "left"},
        "source_sha256": contract.source_hashes,
        "physical_boundary": {
            "bounding_length_m": round(contract.extents["front"] + contract.extents["rear"], 5),
            "bounding_width_m": round(contract.extents["left"] + contract.extents["right"], 5),
            "extents_m": contract.extents,
            "front_taper_m": contract.front_taper,
            "front_shoulder_depth_m": contract.front_shoulder_depth,
            "corner_radius_m": contract.corner_radius,
            "corner_samples": contract.corner_samples,
            "point_count": len(contract.physical),
            "points_xy_m": point_list(contract.physical),
        },
        "planning_boundary": {
            "bounding_length_m": round(
                contract.extents["front"]
                + contract.extents["rear"]
                + contract.margins["front"]
                + contract.margins["rear"],
                5,
            ),
            "bounding_width_m": round(
                contract.extents["left"]
                + contract.extents["right"]
                + contract.margins["left"]
                + contract.margins["right"],
                5,
            ),
            "margins_m": contract.margins,
            "corner_radius_m": round(
                contract.corner_radius + min(contract.margins.values()), 6
            ),
            "point_count": len(contract.planning),
            "points_xy_m": point_list(contract.planning),
            "nav2_local_match": point_list(contract.planning) == point_list(contract.nav2_local),
            "nav2_global_match": point_list(contract.planning) == point_list(contract.nav2_global),
        },
        "motion_visualization": {
            "frame_count": len(motion_poses()),
            "frame_duration_ms": 120,
            "phases": [
                {"name": "FORWARD", "center": "translates", "yaw": "fixed"},
                {"name": "CURVED DRIVE", "center": "translates", "yaw": "changes"},
                {"name": "CRAB RIGHT", "center": "translates laterally", "yaw": "fixed"},
                {"name": "ZERO TURN", "center": "fixed", "yaw": "changes"},
            ],
            "interpretation": "Both contours are rigid transforms of one robot-centered geometry.",
        },
    }


def write_json(contract, output):
    """Write deterministic geometry and provenance metadata."""
    output.write_text(
        json.dumps(result_payload(contract), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def write_manifest(output_dir):
    """Hash every generated artifact without self-referencing the manifest."""
    lines = []
    for name in (PNG_NAME, GIF_NAME, RESULT_NAME):
        lines.append(f"{sha256(output_dir / name)}  {name}")
    (output_dir / MANIFEST_NAME).write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    """Render the current source-derived boundary documentation set."""
    default_repo_root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=default_repo_root)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()

    repo_root = args.repo_root.resolve()
    output_dir = args.output_dir
    if output_dir is None:
        output_dir = (
            repo_root
            / "docs"
            / "assets"
            / "test_result"
            / "tapered-rounded-boundary-20260810"
        )
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    contract = load_contract(repo_root)
    render_geometry(contract, output_dir / PNG_NAME)
    render_motion(contract, output_dir / GIF_NAME)
    write_json(contract, output_dir / RESULT_NAME)
    write_manifest(output_dir)


if __name__ == "__main__":
    main()
