#!/usr/bin/env python3
"""Render a recorded center-frame boundary recovery simulation.

The renderer deliberately separates observed runtime behavior from geometric
what-if analysis. Recovery commands in the input run are probe commands; the
safety gate admits or rejects them but does not generate those commands.
"""

import argparse
import json
import math
from pathlib import Path

import lanelet2
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from lanelet2.io import Origin
from lanelet2.projection import LocalCartesianProjector
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch, Polygon as PolygonPatch
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union


ORIGIN = Origin(36.8435737, 128.0925646, 0.0)
ROUTE_IDS = (754, 2751, 2720)
FRONT, REAR = 0.85837, 0.83323
LEFT, RIGHT = 0.63505, 0.63495
LEGACY_REAR_AXLE_X = -0.443
FPS = 8


def lane_polygon(lanelet):
    left = [(point.x, point.y) for point in lanelet.leftBound]
    right = [(point.x, point.y) for point in lanelet.rightBound]
    return left + list(reversed(right))


def footprint_points(x, y, yaw):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [
        (x + cosine * px - sine * py, y + sine * px + cosine * py)
        for px, py in (
            (FRONT, LEFT), (FRONT, -RIGHT),
            (-REAR, -RIGHT), (-REAR, LEFT),
        )
    ]


def body_point(x, y, yaw, px, py=0.0):
    return (
        x + math.cos(yaw) * px - math.sin(yaw) * py,
        y + math.sin(yaw) * px + math.cos(yaw) * py,
    )


def resample(points, step=0.10):
    output = []
    for start, end in zip(points, points[1:]):
        count = max(1, math.ceil(math.dist(start, end) / step))
        for index in range(count):
            ratio = index / count
            output.append((
                start[0] + ratio * (end[0] - start[0]),
                start[1] + ratio * (end[1] - start[1]),
            ))
    output.append(points[-1])
    return output


def route_points(lanelet_map):
    points = []
    for lanelet_id in ROUTE_IDS:
        current = [
            (point.x, point.y)
            for point in lanelet_map.laneletLayer[lanelet_id].centerline
        ]
        if points and math.dist(points[-1], current[0]) < 0.5:
            current = current[1:]
        points.extend(current)
    return resample(points)[5:]


def map_geometry(lanelet_map):
    polygons = []
    for lanelet in lanelet_map.laneletLayer:
        polygon = Polygon(lane_polygon(lanelet))
        if polygon.is_valid and polygon.area > 0.0:
            polygons.append(polygon)
    return unary_union(polygons)


def transformed_footprint(x, y, path_yaw, lateral=0.0, yaw_offset_deg=0.0):
    x -= math.sin(path_yaw) * lateral
    y += math.cos(path_yaw) * lateral
    yaw = path_yaw + math.radians(yaw_offset_deg)
    return Polygon(footprint_points(x, y, yaw))


def analyze_route(drivable, points):
    samples = []
    for index in range(0, len(points), 2):
        x, y = points[index]
        if not (-1.0 <= x <= 8.1 and 44.0 <= y <= 46.2):
            continue
        previous = points[max(0, index - 2)]
        following = points[min(len(points) - 1, index + 2)]
        yaw = math.atan2(following[1] - previous[1], following[0] - previous[0])
        centered = transformed_footprint(x, y, yaw)
        center_fits = drivable.covers(centered)
        adjustment = None
        if not center_fits:
            # Bounded geometry search, not a dynamically feasible trajectory.
            for lateral in np.arange(-0.40, 0.4001, 0.05):
                for yaw_offset in range(-20, 21, 2):
                    if drivable.covers(
                        transformed_footprint(x, y, yaw, lateral, yaw_offset)
                    ):
                        adjustment = [round(float(lateral), 2), yaw_offset]
                        break
                if adjustment is not None:
                    break
        clearance = (
            centered.boundary.distance(drivable.boundary)
            if center_fits
            else -math.sqrt(centered.difference(drivable).area)
        )
        normal = (-math.sin(yaw), math.cos(yaw))
        cross_section = LineString([
            (x - 5.0 * normal[0], y - 5.0 * normal[1]),
            (x + 5.0 * normal[0], y + 5.0 * normal[1]),
        ])
        samples.append({
            "xy": [round(x, 4), round(y, 4)],
            "yaw_deg": round(math.degrees(yaw), 3),
            "lane_width_m": round(drivable.intersection(cross_section).length, 3),
            "clearance_m": round(clearance, 4),
            "classification": (
                "safe" if center_fits else "adjustable" if adjustment else "no_fit_found"
            ),
            "first_adjustment": adjustment,
        })
    return samples


def draw_map(ax, lanelet_map, route_samples=None):
    for lanelet_id in ROUTE_IDS:
        lanelet = lanelet_map.laneletLayer[lanelet_id]
        color = "#eef2f3" if lanelet_id == 2720 else "#f5e8e8"
        edge = "#77858c" if lanelet_id == 2720 else "#a04a4a"
        ax.add_patch(PolygonPatch(
            lane_polygon(lanelet), closed=True, facecolor=color,
            edgecolor=edge, linewidth=1.5, alpha=0.95, zorder=1,
        ))
    if route_samples:
        colors = {
            "safe": "#16864b",
            "adjustable": "#e28b12",
            "no_fit_found": "#c62828",
        }
        for first, second in zip(route_samples, route_samples[1:]):
            ax.plot(
                [first["xy"][0], second["xy"][0]],
                [first["xy"][1], second["xy"][1]],
                color=colors[first["classification"]], linewidth=4.0,
                solid_capstyle="round", zorder=3,
            )
    else:
        for lanelet_id in ROUTE_IDS:
            center = [
                (point.x, point.y)
                for point in lanelet_map.laneletLayer[lanelet_id].centerline
            ]
            ax.plot(
                [point[0] for point in center], [point[1] for point in center],
                color="#276789", linewidth=2.0, zorder=2,
            )
    ax.set_xlim(-2.8, 9.6)
    ax.set_ylim(43.7, 46.65)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#d7dde0", linewidth=0.55)
    ax.set_xlabel("map x [m]")
    ax.set_ylabel("map y [m]")


def pose_arrays(run):
    poses = [event for event in run["timeline"] if event["event"] == "pose"]
    return (
        np.asarray([event["t"] for event in poses]),
        np.asarray([event["x"] for event in poses]),
        np.asarray([event["y"] for event in poses]),
        np.unwrap(np.asarray([event["yaw_rad"] for event in poses])),
    )


def event(run, event_type, name=None, occurrence=0):
    matches = [
        item for item in run["timeline"]
        if item["event"] == event_type and (name is None or item.get("name") == name)
    ]
    return matches[occurrence]


def candidate(run, name):
    return next(item for item in run["candidates"] if item["name"] == name)


def draw_robot(ax, x, y, yaw, color, alpha=0.78, label_center=True):
    patch = PolygonPatch(
        footprint_points(x, y, yaw), closed=True, facecolor=color,
        edgecolor="white", linewidth=1.8, alpha=alpha, zorder=7,
    )
    ax.add_patch(patch)
    center = ax.plot(
        [x], [y], marker="o", markersize=5, color="white",
        markeredgecolor="#111827", zorder=9,
    )[0]
    rear = body_point(x, y, yaw, LEGACY_REAR_AXLE_X)
    ax.plot(
        [rear[0]], [rear[1]], marker="D", markersize=4,
        color="#111827", zorder=9,
    )
    ax.plot(
        [x, x + 0.78 * math.cos(yaw)],
        [y, y + 0.78 * math.sin(yaw)],
        color="#111827", linewidth=2.2, zorder=9,
    )
    if label_center:
        ax.text(x, y + 0.72, "center", fontsize=7, ha="center", zorder=10)
    return patch, center


def render_contact_sheet(output, run, lanelet_map, route_samples):
    times, xs, ys, yaws = pose_arrays(run)
    first_hold = event(run, "hold", "forward_run_1")
    second_hold = event(run, "hold", "same_path_retry")
    reverse = candidate(run, "reverse_escape")

    fig = plt.figure(figsize=(16, 9), facecolor="#f4f6f7")
    grid = fig.add_gridspec(2, 3, width_ratios=(1.25, 1.25, 0.9), hspace=0.25, wspace=0.20)
    axes = [fig.add_subplot(grid[row, col]) for row in range(2) for col in range(2)]
    panel = fig.add_subplot(grid[:, 2])
    panel.axis("off")
    for ax in axes:
        draw_map(ax, lanelet_map)

    # 1: observed forward RPP path and changing yaw.
    hold_index = int(np.searchsorted(times, first_hold["t"]))
    axes[0].plot(xs[:hold_index], ys[:hold_index], color="#155fa0", linewidth=2.0, zorder=4)
    for index in np.linspace(10, max(10, hold_index - 1), 5, dtype=int):
        draw_robot(axes[0], xs[index], ys[index], yaws[index], "#1976d2", 0.28, False)
    axes[0].set_title("1. RPP drive: yaw follows curved path", loc="left", fontweight="bold")
    axes[0].set_xlim(-2.4, 5.6)
    axes[0].set_ylim(43.9, 46.35)

    # 2: first raster boundary contact and blocked crab probes.
    x, y, yaw = first_hold["x"], first_hold["y"], first_hold["yaw_rad"]
    draw_robot(axes[1], x, y, yaw, "#c62828", 0.75)
    for direction, label, offset in [((0.0, 1.0), "LEFT BLOCKED", 0.0), ((0.0, -1.0), "RIGHT BLOCKED", -0.3)]:
        dx = math.cos(yaw) * direction[0] - math.sin(yaw) * direction[1]
        dy = math.sin(yaw) * direction[0] + math.cos(yaw) * direction[1]
        axes[1].annotate(
            "", xy=(x + 0.75 * dx, y + 0.75 * dy), xytext=(x, y),
            arrowprops={"arrowstyle": "-|>", "color": "#c62828", "lw": 2.5}, zorder=12,
        )
        axes[1].text(x + 0.84 * dx, y + 0.84 * dy + offset, label,
                     color="#a31b1b", fontsize=8, fontweight="bold", ha="center")
    axes[1].set_title(
        f"2. First hold: ({x:.2f}, {y:.2f}), yaw {first_hold['yaw_deg']:+.2f} deg",
        loc="left", fontweight="bold",
    )
    axes[1].set_xlim(2.1, 5.55)
    axes[1].set_ylim(43.9, 46.35)

    # 3: observed reverse escape, with yaw held by the manual probe.
    start = reverse["start_xy"]
    end = reverse["end_xy"]
    reverse_yaw = math.radians(reverse["start_yaw_deg"])
    draw_robot(axes[2], start[0], start[1], reverse_yaw, "#d98210", 0.32, False)
    draw_robot(axes[2], end[0], end[1], reverse_yaw, "#16864b", 0.78, False)
    axes[2].annotate(
        "", xy=end, xytext=start,
        arrowprops={"arrowstyle": "-|>", "color": "#16864b", "lw": 3.0}, zorder=12,
    )
    axes[2].set_title(
        f"3. Reverse allowed: {reverse['displacement_m']:.2f} m; yaw held",
        loc="left", fontweight="bold",
    )
    axes[2].set_xlim(2.45, 5.0)
    axes[2].set_ylim(43.95, 46.25)

    # 4: same path retries, steering resumes, then the gate holds again.
    retry_start = event(run, "stage", "same_path_retry")["t"]
    mask = (times >= retry_start) & (times <= second_hold["t"])
    axes[3].plot(xs[mask], ys[mask], color="#6b3fb5", linewidth=2.5, zorder=4)
    draw_robot(
        axes[3], second_hold["x"], second_hold["y"], second_hold["yaw_rad"],
        "#c62828", 0.78,
    )
    axes[3].set_title(
        f"4. Retry blocked again: yaw {second_hold['yaw_deg']:+.2f} deg",
        loc="left", fontweight="bold",
    )
    axes[3].set_xlim(2.85, 5.85)
    axes[3].set_ylim(43.8, 46.25)

    panel.text(0.0, 0.98, "Observed full-bringup result", fontsize=15, fontweight="bold", va="top")
    result_lines = [
        ("Center frame", "robot_center_link"),
        ("Planning footprint", "1.6916 x 1.2700 m"),
        ("First boundary hold", f"{first_hold['yaw_deg']:+.2f} deg"),
        ("Left crab probe", "BLOCKED"),
        ("Right crab probe", "BLOCKED"),
        ("Reverse probe", f"ALLOWED, {reverse['max_output_mps']:.2f} m/s"),
        ("Hold after reverse", "RELEASED"),
        ("Same path retry", "BLOCKED AGAIN"),
        ("Mission completed", "NO"),
    ]
    y = 0.90
    for label, value in result_lines:
        panel.text(0.0, y, label, fontsize=9.5, color="#53616a")
        panel.text(0.52, y, value, fontsize=10, fontweight="bold",
                   color="#c62828" if "BLOCK" in value or value == "NO" else "#1c5f3b")
        y -= 0.050
    panel.text(0.0, y - 0.01, "Active parameters", fontsize=13, fontweight="bold")
    y -= 0.065
    for line in (
        "RPP: 15 Hz, desired 0.4 m/s",
        "Gate scale: 0.5 -> max 0.20 m/s",
        "Lookahead min/default/max: 1.1/1.1/2.0 m",
        "Heading rotate: disabled",
        "Footprint stop cost: 100",
        "Recovery: 0.25 m probe, 1.0 s clear",
    ):
        panel.text(0.0, y, line, fontsize=9.1, color="#39474f")
        y -= 0.040
    panel.text(
        0.0, 0.02,
        "IMPORTANT\nCrab/reverse commands were manually injected.\nThe gate only admitted or rejected them.\nNo automatic recovery owner is implemented.",
        fontsize=9.5, color="#37474f", va="bottom",
        bbox={"boxstyle": "round,pad=0.55", "facecolor": "#fff3df", "edgecolor": "#e2a64b"},
    )
    fig.suptitle(
        "CAMROD robot-center boundary behavior - recorded 2026-08-03",
        fontsize=18, fontweight="bold", y=0.985,
    )
    fig.savefig(output, dpi=120, bbox_inches="tight")
    plt.close(fig)


def render_risk_map(output, run, lanelet_map, route_samples):
    first_hold = event(run, "hold", "forward_run_1")
    second_hold = event(run, "hold", "same_path_retry")
    fig = plt.figure(figsize=(14, 8), facecolor="#f4f6f7")
    ax = fig.add_axes([0.06, 0.12, 0.67, 0.78])
    panel = fig.add_axes([0.76, 0.12, 0.22, 0.78])
    panel.axis("off")
    draw_map(ax, lanelet_map, route_samples)
    for hold, marker, label, offset in (
        (first_hold, "X", "runtime hold #1", (8, 10)),
        (second_hold, "P", "runtime hold #2", (10, -18)),
    ):
        ax.scatter([hold["x"]], [hold["y"]], marker=marker, s=110,
                   color="#111827", edgecolor="white", linewidth=1.2, zorder=8)
        ax.annotate(label, (hold["x"], hold["y"]), xytext=offset,
                    textcoords="offset points", fontsize=9, fontweight="bold")
    legend = [
        Line2D([0], [0], color="#16864b", lw=4, label="center footprint fits"),
        Line2D([0], [0], color="#e28b12", lw=4, label="bounded adjustment found"),
        Line2D([0], [0], color="#c62828", lw=4, label="no fit found"),
    ]
    ax.legend(handles=legend, loc="lower left", framealpha=0.95)
    ax.set_title("Planning-footprint feasibility along the tested route", fontsize=14, fontweight="bold")

    first_adjustable = next(item for item in route_samples if item["classification"] == "adjustable")
    first_no_fit = next(item for item in route_samples if item["classification"] == "no_fit_found")
    narrowest = min(route_samples, key=lambda item: item["lane_width_m"])
    panel.text(0.0, 0.98, "Risk summary", fontsize=16, fontweight="bold", va="top")
    lines = [
        ("Planning size", "1.6916 x 1.2700 m"),
        ("Physical size", "1.4916 x 1.0700 m"),
        ("Margin", "0.10 m each side"),
        ("First runtime hold", f"({first_hold['x']:.2f}, {first_hold['y']:.2f})"),
        ("First center failure", f"({first_adjustable['xy'][0]:.2f}, {first_adjustable['xy'][1]:.2f})"),
        ("First no-fit sample", f"({first_no_fit['xy'][0]:.2f}, {first_no_fit['xy'][1]:.2f})"),
        ("Narrowest sampled width", f"{narrowest['lane_width_m']:.2f} m"),
    ]
    y = 0.88
    for label, value in lines:
        panel.text(0.0, y, label, fontsize=9.5, color="#58656c")
        panel.text(0.0, y - 0.038, value, fontsize=11, fontweight="bold", color="#263238")
        y -= 0.095
    panel.text(
        0.0, 0.13,
        "Method\nCenter pose tested first. Failed samples search\nlateral +/-0.40 m and yaw +/-20 deg.\nThis is geometry only, not a drivable trajectory.",
        fontsize=9.3, color="#455a64", va="bottom",
        bbox={"boxstyle": "round,pad=0.5", "facecolor": "#edf1f3", "edgecolor": "#b5c0c5"},
    )
    panel.text(
        0.0, 0.02,
        "Runtime stops use raster cost 100.\nThey can occur before vector-map overlap.",
        fontsize=9.2, color="#a31b1b", va="bottom", fontweight="bold",
    )
    fig.suptitle("CAMROD boundary risk map - robot_center_link", fontsize=18, fontweight="bold")
    fig.savefig(output, dpi=120, bbox_inches="tight")
    plt.close(fig)


def render_gif(output, run, lanelet_map, route_samples):
    times, xs, ys, yaws = pose_arrays(run)
    first_hold = event(run, "hold", "forward_run_1")
    second_hold = event(run, "hold", "same_path_retry")
    left_end = event(run, "candidate", "left_crab_probe")["t"]
    right_end = event(run, "candidate", "right_crab_probe")["t"]
    reverse_end = event(run, "candidate", "reverse_escape")["t"]
    retry_start = event(run, "stage", "same_path_retry")["t"]
    final_time = min(times[-1], second_hold["t"] + 0.35)
    segments = [
        (event(run, "stage", "forward_run_1")["t"], first_hold["t"], 5.0,
         "RPP FORWARD + STEERING", "yaw changes with curved path", (1.0, 0.0), "#1976d2"),
        (first_hold["t"], event(run, "stage", "left_crab_probe")["t"], 1.4,
         "BOUNDARY COST 100", "full planning footprint stopped", (0.0, 0.0), "#c62828"),
        (event(run, "stage", "left_crab_probe")["t"], left_end, 1.4,
         "LEFT CRAB PROBE", "BLOCKED by projected footprint", (0.0, 1.0), "#c62828"),
        (left_end, right_end, 1.4,
         "RIGHT CRAB PROBE", "BLOCKED by projected footprint", (0.0, -1.0), "#c62828"),
        (right_end, reverse_end, 3.6,
         "REVERSE PROBE", "ALLOWED; yaw held by probe", (-1.0, 0.0), "#16864b"),
        (reverse_end, retry_start, 1.0,
         "CLEAR EVIDENCE", "hold released after 1.0 s", (0.0, 0.0), "#16864b"),
        (retry_start, second_hold["t"], 2.8,
         "SAME PATH RETRY", "RPP steering and yaw resume", (1.0, 0.0), "#6b3fb5"),
        (second_hold["t"], final_time, 1.6,
         "BOUNDARY COST 100 AGAIN", "mission does not complete", (0.0, 0.0), "#c62828"),
    ]
    display_edges = [0.0]
    for segment in segments:
        display_edges.append(display_edges[-1] + segment[2])
    duration = display_edges[-1]

    def display_to_actual(display_time):
        for index, segment in enumerate(segments):
            if display_time <= display_edges[index + 1] or index == len(segments) - 1:
                ratio = np.clip(
                    (display_time - display_edges[index]) / segment[2], 0.0, 1.0
                )
                return segment[0] + ratio * (segment[1] - segment[0]), index
        return segments[-1][1], len(segments) - 1

    fig = plt.figure(figsize=(12.8, 7.2), facecolor="#f4f6f7")
    ax = fig.add_axes([0.055, 0.12, 0.68, 0.78])
    panel = fig.add_axes([0.765, 0.12, 0.215, 0.78])
    panel.axis("off")
    draw_map(ax, lanelet_map, route_samples)
    trail, = ax.plot([], [], color="#252b2e", linewidth=1.8, alpha=0.55, zorder=4)
    robot = PolygonPatch(
        footprint_points(xs[0], ys[0], yaws[0]), closed=True,
        facecolor="#1976d2", edgecolor="white", linewidth=2.0,
        alpha=0.78, zorder=7,
    )
    ax.add_patch(robot)
    center_marker, = ax.plot([], [], marker="o", markersize=6, color="white",
                             markeredgecolor="#111827", zorder=9)
    rear_marker, = ax.plot([], [], marker="D", markersize=5, color="#111827", zorder=9)
    heading_line, = ax.plot([], [], color="#111827", linewidth=3.0, zorder=10)
    command_arrow = None
    status = ax.text(
        0.02, 0.96, "", transform=ax.transAxes, va="top", fontsize=11,
        fontweight="bold", color="white",
        bbox={"boxstyle": "round,pad=0.45", "facecolor": "#1976d2", "edgecolor": "none"},
        zorder=12,
    )
    yaw_label = ax.text(0.985, 0.03, "", transform=ax.transAxes, ha="right",
                        fontsize=10.5, color="#111827", fontweight="bold")
    panel.text(0.0, 0.98, "Center-frame simulation", fontsize=14, fontweight="bold", va="top")
    panel.text(0.0, 0.91, "o  robot_center_link", fontsize=9.5, color="#263238")
    panel.text(0.0, 0.865, "D  legacy rear-axle link (-0.443 m)", fontsize=9.5, color="#263238")
    panel.text(0.0, 0.79, "Observed", fontsize=12, fontweight="bold")
    checks = [
        ("Normal RPP yaw", "MOVES", "#1976d2"),
        ("Left crab", "BLOCKED", "#c62828"),
        ("Right crab", "BLOCKED", "#c62828"),
        ("Reverse", "ALLOWED", "#16864b"),
        ("Recovery yaw", "HELD", "#d98210"),
        ("Same route", "BLOCKED AGAIN", "#c62828"),
    ]
    for index, (label, value, color) in enumerate(checks):
        y = 0.735 - index * 0.085
        panel.text(0.0, y, label, fontsize=9.3, color="#59666c")
        panel.text(0.52, y, value, fontsize=9.7, fontweight="bold", color=color)
    panel.text(0.0, 0.19, "Active RPP / gate", fontsize=12, fontweight="bold")
    panel.text(
        0.0, 0.145,
        "15 Hz | lookahead 1.1 m\n0.4 m/s x gate 0.5\nfootprint 1.6916 x 1.2700 m",
        fontsize=9.3, color="#37474f", va="top",
    )
    panel.text(
        0.0, 0.015,
        "Probe commands are manual.\nAutomatic recovery owner: NOT IMPLEMENTED",
        fontsize=9.2, color="#a31b1b", va="bottom", fontweight="bold",
    )

    def update(frame):
        nonlocal command_arrow
        display_time = min(frame / FPS, duration)
        actual_time, index = display_to_actual(display_time)
        segment = segments[index]
        x = float(np.interp(actual_time, times, xs))
        y = float(np.interp(actual_time, times, ys))
        yaw = float(np.interp(actual_time, times, yaws))
        mask = times <= actual_time
        trail.set_data(xs[mask], ys[mask])
        robot.set_xy(footprint_points(x, y, yaw))
        robot.set_facecolor(segment[6])
        center_marker.set_data([x], [y])
        rear = body_point(x, y, yaw, LEGACY_REAR_AXLE_X)
        rear_marker.set_data([rear[0]], [rear[1]])
        heading_line.set_data(
            [x, x + 0.78 * math.cos(yaw)],
            [y, y + 0.78 * math.sin(yaw)],
        )
        if command_arrow is not None:
            command_arrow.remove()
            command_arrow = None
        direction = segment[5]
        if direction != (0.0, 0.0):
            dx = math.cos(yaw) * direction[0] - math.sin(yaw) * direction[1]
            dy = math.sin(yaw) * direction[0] + math.cos(yaw) * direction[1]
            command_arrow = FancyArrowPatch(
                (x, y), (x + 0.72 * dx, y + 0.72 * dy),
                arrowstyle="-|>", mutation_scale=18, linewidth=3.0,
                color=segment[6], zorder=11,
            )
            ax.add_patch(command_arrow)
        status.set_text(f"{segment[3]}\n{segment[4]}")
        status.get_bbox_patch().set_facecolor(segment[6])
        yaw_deg = math.degrees(math.atan2(math.sin(yaw), math.cos(yaw)))
        yaw_label.set_text(f"yaw {yaw_deg:+.2f} deg | recorded t {actual_time:.2f} s")
        return trail, robot, center_marker, rear_marker, heading_line, status, yaw_label

    frame_count = math.ceil(duration * FPS)
    gif = animation.FuncAnimation(
        fig, update, frames=frame_count, interval=1000 / FPS, blit=False, repeat=True
    )
    gif.save(output, writer=animation.PillowWriter(fps=FPS), dpi=92)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with args.run.open("r", encoding="utf-8") as stream:
        run = json.load(stream)
    lanelet_map = lanelet2.io.load(str(args.map), LocalCartesianProjector(ORIGIN))
    points = route_points(lanelet_map)
    route_samples = analyze_route(map_geometry(lanelet_map), points)
    # HH_260804 / v2.1.3 - Keep pre-owner evidence visibly distinct from the
    # automatic recovery controller introduced after this manual probe run.
    analysis_path = args.output_dir / "robot-center-route-samples.json"
    contact_path = args.output_dir / "pre-owner-robot-center-contact-sheet.png"
    risk_path = args.output_dir / "robot-center-narrow-route-risk-map.png"
    gif_path = args.output_dir / "pre-owner-robot-center-recovery.gif"
    with analysis_path.open("w", encoding="utf-8") as stream:
        json.dump({"route_ids": ROUTE_IDS, "samples": route_samples}, stream, indent=2)
    render_contact_sheet(contact_path, run, lanelet_map, route_samples)
    render_risk_map(risk_path, run, lanelet_map, route_samples)
    render_gif(gif_path, run, lanelet_map, route_samples)
    for path in (contact_path, risk_path, gif_path, analysis_path):
        print(path)


if __name__ == "__main__":
    main()
