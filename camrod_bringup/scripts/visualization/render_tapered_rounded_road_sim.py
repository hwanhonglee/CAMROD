#!/usr/bin/env python3
"""Render a measured ROS road run with the active tapered/rounded boundaries."""

# HH_260810 - Draw the current contour over recorded map-v17 ROS poses so the
# shape animation remains distinct from both a source-only transform and RViz art.

import argparse
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import lanelet2
from lanelet2.io import Origin
from lanelet2.projection import LocalCartesianProjector
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgba
from matplotlib.patches import FancyArrowPatch, Polygon
import numpy as np
from PIL import Image

from render_tapered_rounded_boundary import load_contract, transform_points


plt.switch_backend("Agg")

ORIGIN = Origin(36.8435737, 128.0925646, 0.0)
PNG_NAME = "tapered-rounded-boundary-road-sim.png"
GIF_NAME = "tapered-rounded-boundary-road-sim.gif"
SUMMARY_NAME = "road-sim-summary.json"
MANIFEST_NAME = "SHA256SUMS"
RUN_NAME = "runtime-b2-recovery.json"
ROUTE_HIGHLIGHT = "#e78118"
PHYSICAL_FILL = "#53c7d7"
PHYSICAL_EDGE = "#00788a"
PLANNING_FILL = "#f5c85b"
PLANNING_EDGE = "#b47600"
HOLD_RED = "#c23b32"
RECOVERY_GREEN = "#198754"


def sha256(path):
    """Return one file SHA-256 digest."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_json(path):
    """Load one UTF-8 JSON record."""
    return json.loads(path.read_text(encoding="utf-8"))


def map_identity(path):
    """Return the selected OSM map version and SHA."""
    root = ET.parse(path).getroot()
    meta = root.find("MetaInfo")
    return {
        "map_version": int(meta.attrib["map_version"]) if meta is not None else None,
        "sha256": sha256(path),
    }


def validate_run(run, map_path, contract):
    """Reject stale map, geometry, or incomplete runtime records."""
    selected_map = map_identity(map_path)
    if run.get("map", {}).get("map_version") != selected_map["map_version"]:
        raise ValueError("runtime evidence map version does not match selected OSM")
    if run.get("map", {}).get("sha256") != selected_map["sha256"]:
        raise ValueError("runtime evidence map SHA does not match selected OSM")
    if run.get("scenario") != "b2_recontact":
        raise ValueError("road renderer requires the b2_recontact runtime scenario")
    if not run.get("mission_completed"):
        raise ValueError("runtime road scenario did not complete")

    geometry = run.get("geometry", {})
    for side, expected in contract.extents.items():
        actual = geometry.get("physical_body_extents_m", {}).get(side)
        if actual is None or not math.isclose(actual, expected, abs_tol=1.0e-9):
            raise ValueError(f"runtime physical {side} extent is stale")
        planning_expected = expected + contract.margins[side]
        planning_actual = geometry.get("planning_boundary_extents_m", {}).get(side)
        if planning_actual is None or not math.isclose(
            planning_actual, planning_expected, abs_tol=1.0e-9
        ):
            raise ValueError(f"runtime planning {side} extent is stale")

    shape = geometry.get("boundary_shape", {})
    expected_shape = {
        "front_taper_m": contract.front_taper,
        "front_shoulder_depth_m": contract.front_shoulder_depth,
        "physical_corner_radius_m": contract.corner_radius,
        "planning_corner_radius_m": contract.corner_radius
        + min(contract.margins.values()),
        "corner_samples": contract.corner_samples,
        "physical_point_count": len(contract.physical),
        "planning_point_count": len(contract.planning),
    }
    for key, expected in expected_shape.items():
        actual = shape.get(key)
        if isinstance(expected, float):
            if actual is None or not math.isclose(actual, expected, abs_tol=1.0e-9):
                raise ValueError(f"runtime boundary shape {key} is stale")
        elif actual != expected:
            raise ValueError(f"runtime boundary shape {key} is stale")
    if geometry.get("frame") != "robot_center_link":
        raise ValueError("runtime boundary frame is not robot_center_link")


def lane_polygon(lanelet):
    """Return one Lanelet2 polygon in map coordinates."""
    left = [(point.x, point.y) for point in lanelet.leftBound]
    right = [(point.x, point.y) for point in lanelet.rightBound]
    return left + list(reversed(right))


def draw_route(axis, lanelet_map, route_ids, context=False):
    """Draw actual Lanelet2 road geometry and highlight the measured route."""
    if context:
        for lanelet in lanelet_map.laneletLayer:
            axis.add_patch(
                Polygon(
                    lane_polygon(lanelet),
                    closed=True,
                    facecolor="#f6f7f7",
                    edgecolor="#d5dcdf",
                    linewidth=0.55,
                    zorder=0,
                )
            )
    for lanelet_id in route_ids:
        lanelet = lanelet_map.laneletLayer[lanelet_id]
        axis.add_patch(
            Polygon(
                lane_polygon(lanelet),
                closed=True,
                facecolor="#e5eaec",
                edgecolor="#687981",
                linewidth=1.6,
                zorder=1,
            )
        )
        centerline = np.asarray([(point.x, point.y) for point in lanelet.centerline])
        axis.plot(
            centerline[:, 0],
            centerline[:, 1],
            color=ROUTE_HIGHLIGHT,
            linewidth=2.0,
            linestyle=(0, (5, 4)),
            alpha=0.85,
            zorder=2,
        )
    axis.set_facecolor("#ffffff")
    axis.grid(color="#e2e7e9", linewidth=0.65, zorder=-1)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("map X (m)")
    axis.set_ylabel("map Y (m)")
    for spine in axis.spines.values():
        spine.set_color("#cbd4d8")


def pose_events(run):
    """Return timestamped pose events as NumPy arrays."""
    events = [event for event in run["timeline"] if event["event"] == "pose"]
    return {
        "events": events,
        "t": np.asarray([event["t"] for event in events]),
        "x": np.asarray([event["x"] for event in events]),
        "y": np.asarray([event["y"] for event in events]),
        "yaw": np.unwrap(np.asarray([event["yaw_rad"] for event in events])),
    }


def pose_at(poses, timestamp):
    """Interpolate one pose from the measured timeline."""
    return (
        float(np.interp(timestamp, poses["t"], poses["x"])),
        float(np.interp(timestamp, poses["t"], poses["y"])),
        float(np.interp(timestamp, poses["t"], poses["yaw"])),
    )


def milestone_times(run):
    """Index probe milestones by name."""
    return {
        event["name"]: event["t"]
        for event in run["timeline"]
        if event["event"] == "milestone"
    }


def indexed_events(run, event_name):
    """Return event timestamps and records for last-value lookup."""
    events = [event for event in run["timeline"] if event["event"] == event_name]
    return [event["t"] for event in events], events


def event_at(indexed, timestamp):
    """Return the latest event at one measured timestamp."""
    times, events = indexed
    index = bisect_right(times, timestamp) - 1
    return events[index] if index >= 0 else None


def route_hit(run):
    """Extract the measured world hit point from the gate timeline."""
    pattern = re.compile(r"route_hit_world=([-+0-9.]+),([-+0-9.]+)")
    for event in run["timeline"]:
        if event.get("event") != "gate":
            continue
        match = pattern.search(event.get("message", ""))
        if match:
            return float(match.group(1)), float(match.group(2))
    raise ValueError("runtime gate timeline has no route hit point")


def draw_boundaries(
    axis,
    contract,
    pose,
    alpha=1.0,
    planning_edge=PLANNING_EDGE,
    body_edge=PHYSICAL_EDGE,
    linewidth=2.2,
    labels=False,
):
    """Draw exact physical and planning contours at one measured map pose."""
    x, y, yaw = pose
    planning = transform_points(contract.planning, x, y, yaw)
    physical = transform_points(contract.physical, x, y, yaw)
    axis.add_patch(
        Polygon(
            planning,
            closed=True,
            facecolor=to_rgba(PLANNING_FILL, 0.28 * alpha),
            edgecolor=to_rgba(planning_edge, alpha),
            linewidth=linewidth,
            joinstyle="round",
            zorder=6,
            label="planning +0.10 m" if labels else None,
        )
    )
    axis.add_patch(
        Polygon(
            physical,
            closed=True,
            facecolor=to_rgba(PHYSICAL_FILL, 0.62 * alpha),
            edgecolor=to_rgba(body_edge, alpha),
            linewidth=linewidth,
            joinstyle="round",
            zorder=7,
            label="physical body" if labels else None,
        )
    )
    axis.scatter([x], [y], s=38, color="#17272d", edgecolor="white", zorder=9)
    axis.plot(
        [x, x + 0.75 * math.cos(yaw)],
        [y, y + 0.75 * math.sin(yaw)],
        color="#c1492f",
        linewidth=2.3,
        zorder=9,
    )


def phase_at(timestamp, milestones):
    """Classify one frame against measured hold/recovery milestones."""
    first_hold = milestones["first_hold"]
    recovery_start = milestones["automatic_recovery_started"]
    release = milestones["hold_released"]
    complete = milestones["route_completed_after_recovery"]
    if timestamp < first_hold:
        return "RPP ROAD DRIVE", "#1769aa"
    if timestamp < recovery_start:
        return "PLANNING-MARGIN HOLD", HOLD_RED
    if timestamp < release:
        return "REVERSE_YAW_RIGHT", RECOVERY_GREEN
    if timestamp < complete:
        return "ROUTE RESUMED", "#6b4ca5"
    return "GOAL COMPLETE", "#187044"


def overview_limits(poses):
    """Return fixed route limits with room for complete planning contours."""
    return (
        (float(poses["x"].min() - 1.15), float(poses["x"].max() + 1.15)),
        (float(poses["y"].min() - 1.15), float(poses["y"].max() + 1.15)),
    )


def draw_measured_trajectory(axis, poses, milestones):
    """Draw drive, recovery, and resumed route segments with distinct colors."""
    segments = (
        (poses["t"] <= milestones["first_hold"], "#1769aa", "RPP drive"),
        (
            (poses["t"] >= milestones["first_hold"])
            & (poses["t"] <= milestones["hold_released"]),
            RECOVERY_GREEN,
            "bounded recovery",
        ),
        (
            poses["t"] >= milestones["hold_released"],
            "#6b4ca5",
            "route resumed",
        ),
    )
    for mask, color, label in segments:
        axis.plot(
            poses["x"][mask],
            poses["y"][mask],
            color=color,
            linewidth=2.8,
            label=label,
            zorder=4,
        )


def render_png(run, map_path, contract, output):
    """Render route overview, hold/release details, and measured values."""
    lanelet_map = lanelet2.io.load(str(map_path), LocalCartesianProjector(ORIGIN))
    route_ids = tuple(run["route_lanelet_ids"])
    poses = pose_events(run)
    milestones = milestone_times(run)
    hit_x, hit_y = route_hit(run)
    hold_pose = (
        run["first_hold"]["x"],
        run["first_hold"]["y"],
        run["first_hold"]["yaw_rad"],
    )
    release_pose = (
        run["hold_release"]["x"],
        run["hold_release"]["y"],
        run["hold_release"]["yaw_rad"],
    )
    start_pose = pose_at(poses, poses["t"][0])
    end_pose = pose_at(poses, poses["t"][-1])

    figure = plt.figure(figsize=(16, 9), facecolor="#f3f6f7")
    grid = figure.add_gridspec(
        2,
        3,
        width_ratios=(1.22, 1.10, 0.88),
        left=0.05,
        right=0.97,
        top=0.82,
        bottom=0.09,
        hspace=0.26,
        wspace=0.22,
    )
    overview = figure.add_subplot(grid[:, 0])
    hold_axis = figure.add_subplot(grid[0, 1])
    recovery_axis = figure.add_subplot(grid[1, 1])
    metrics = figure.add_subplot(grid[:, 2])

    figure.text(
        0.05,
        0.93,
        "Tapered/rounded boundary on the measured map-v17 road run",
        fontsize=25,
        fontweight="bold",
        color="#17272d",
    )
    figure.text(
        0.05,
        0.875,
        "B2 RPP drive -> planning-margin contact -> REVERSE_YAW_RIGHT -> same-route completion",
        fontsize=13.5,
        color="#50636b",
    )
    figure.text(
        0.955,
        0.93,
        "MEASURED ROS SIM",
        ha="right",
        fontsize=12,
        fontweight="bold",
        color="#17623f",
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "#e5f4eb", "edgecolor": "#77b58e"},
    )

    draw_route(overview, lanelet_map, route_ids, context=True)
    draw_measured_trajectory(overview, poses, milestones)
    x_limits, y_limits = overview_limits(poses)
    overview.set_xlim(*x_limits)
    overview.set_ylim(*y_limits)
    overview.set_title("1. Complete measured road trajectory", loc="left", fontsize=14, fontweight="bold")
    draw_boundaries(overview, contract, start_pose, alpha=0.34, linewidth=1.3)
    draw_boundaries(overview, contract, hold_pose, alpha=0.65, planning_edge=HOLD_RED, linewidth=1.8)
    draw_boundaries(overview, contract, release_pose, alpha=0.65, body_edge=RECOVERY_GREEN, linewidth=1.8)
    draw_boundaries(overview, contract, end_pose, alpha=0.55, linewidth=1.5)
    overview.annotate("start", start_pose[:2], xytext=(-24, 14), textcoords="offset points", fontsize=9)
    overview.annotate("margin hold", hold_pose[:2], xytext=(-66, -24), textcoords="offset points", fontsize=9, color=HOLD_RED)
    overview.annotate("goal", end_pose[:2], xytext=(12, -18), textcoords="offset points", fontsize=9, color="#187044")
    overview.legend(loc="lower left", fontsize=9, facecolor="white")

    draw_route(hold_axis, lanelet_map, route_ids)
    hold_axis.set_title("2. Planning-only contact", loc="left", fontsize=14, fontweight="bold")
    draw_boundaries(
        hold_axis,
        contract,
        hold_pose,
        planning_edge=HOLD_RED,
        linewidth=3.0,
        labels=True,
    )
    hold_axis.scatter([hit_x], [hit_y], marker="x", s=120, linewidth=3.0, color=HOLD_RED, zorder=12)
    hold_axis.annotate(
        "cost 100 sample\nplanning contour only",
        (hit_x, hit_y),
        xytext=(-118, 68),
        textcoords="offset points",
        fontsize=9.5,
        fontweight="bold",
        color=HOLD_RED,
        arrowprops={"arrowstyle": "->", "color": HOLD_RED, "lw": 1.5},
        bbox={
            "boxstyle": "round,pad=0.3",
            "facecolor": "white",
            "edgecolor": "#d78a84",
            "alpha": 0.95,
        },
    )
    hold_axis.set_xlim(hold_pose[0] - 1.65, hold_pose[0] + 1.65)
    hold_axis.set_ylim(hold_pose[1] - 1.65, hold_pose[1] + 1.65)
    hold_axis.legend(loc="lower left", fontsize=8.5, facecolor="white")

    draw_route(recovery_axis, lanelet_map, route_ids)
    recovery_axis.set_title("3. Bounded reverse-yaw and release", loc="left", fontsize=14, fontweight="bold")
    recovery_mask = (
        (poses["t"] >= milestones["first_hold"])
        & (poses["t"] <= milestones["hold_released"])
    )
    recovery_axis.plot(
        poses["x"][recovery_mask],
        poses["y"][recovery_mask],
        color=RECOVERY_GREEN,
        linewidth=3.2,
        zorder=5,
    )
    draw_boundaries(recovery_axis, contract, hold_pose, alpha=0.35, planning_edge=HOLD_RED, linewidth=1.6)
    draw_boundaries(recovery_axis, contract, release_pose, alpha=1.0, body_edge=RECOVERY_GREEN, linewidth=2.6)
    recovery_axis.add_patch(
        FancyArrowPatch(
            hold_pose[:2],
            release_pose[:2],
            arrowstyle="-|>",
            mutation_scale=18,
            linewidth=2.5,
            color=RECOVERY_GREEN,
            zorder=12,
        )
    )
    recovery_axis.set_xlim(hold_pose[0] - 1.35, hold_pose[0] + 1.35)
    recovery_axis.set_ylim(hold_pose[1] - 1.35, hold_pose[1] + 1.35)
    yaw_delta = math.degrees(
        math.atan2(
            math.sin(release_pose[2] - hold_pose[2]),
            math.cos(release_pose[2] - hold_pose[2]),
        )
    )
    recovery_axis.text(
        0.03,
        0.04,
        f"travel {run['recovery_displacement_m']:.4f} m\nyaw delta {yaw_delta:+.2f} deg",
        transform=recovery_axis.transAxes,
        fontsize=10,
        fontweight="bold",
        color="#17623f",
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": "#8dbba0"},
        zorder=14,
    )

    metrics.set_xlim(0.0, 1.0)
    metrics.set_ylim(0.0, 1.0)
    metrics.axis("off")
    metrics.set_title("Measured result", loc="left", fontsize=14, fontweight="bold")
    classification = run["first_hold_boundary_classification"]
    rows = (
        ("Map / route", "v17 | 2751, 2720, 2744, 2690", "#40565e"),
        ("Physical body", "cost 100 contact: NO", PHYSICAL_EDGE),
        ("Planning contour", "cost 100 contact: YES", HOLD_RED),
        ("Recovery", run["automatic_recovery_motion"], RECOVERY_GREEN),
        ("Recovery motion", f"{run['recovery_displacement_m']:.4f} m", RECOVERY_GREEN),
        ("Peak recovery", f"{run['maximum_recovery_output_mps']:.2f} m/s | {run['maximum_recovery_abs_angular_z_radps']:.2f} rad/s", "#40565e"),
        ("Route result", "COMPLETE | no second hold", "#187044"),
        ("Final Twist", "0.0 / 0.0 / 0.0", "#40565e"),
    )
    y = 0.91
    for label, value, color in rows:
        metrics.add_patch(plt.Rectangle((0.01, y - 0.045), 0.014, 0.105, color=color))
        metrics.text(0.055, y + 0.022, label, fontsize=10.5, fontweight="bold", va="center")
        metrics.text(0.055, y - 0.025, value, fontsize=10.1, color="#40565e", va="center")
        y -= 0.102
    metrics.text(
        0.02,
        0.025,
        (
            f"Pose samples: {len(poses['events'])}\n"
            f"Runtime: {poses['t'][-1] - poses['t'][0]:.3f} s\n"
            f"Planning samples at hold: {classification['planning_boundary']['sampled_cells']}\n"
            f"Physical samples at hold: {classification['physical_body']['sampled_cells']}"
        ),
        fontsize=9.5,
        color="#596b73",
        va="bottom",
    )

    figure.text(
        0.05,
        0.032,
        "Full ROS sim pose/gate/controller timeline | exact current 30-point boundaries | map SHA 8cd05c...5e021 | physical road remains pending",
        fontsize=10.3,
        color="#684c21",
        fontweight="bold",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150, facecolor=figure.get_facecolor())
    plt.close(figure)


def animation_times(poses, milestones):
    """Sample the long drive and short recovery densely enough to see both."""
    start = float(poses["t"][0])
    first_hold = milestones["first_hold"]
    release = milestones["hold_released"]
    end = float(poses["t"][-1])
    return np.concatenate(
        (
            np.linspace(start, first_hold, 32, endpoint=False),
            np.linspace(first_hold, release, 20, endpoint=False),
            np.linspace(release, end, 28),
        )
    )


def render_gif_frame(
    run,
    lanelet_map,
    contract,
    poses,
    milestones,
    timestamp,
    output_index,
    output_events,
    gate_events,
    hit,
):
    """Render one measured route frame with overview and local boundary detail."""
    pose = pose_at(poses, timestamp)
    phase, phase_color = phase_at(timestamp, milestones)
    output = event_at(output_events, timestamp) or {
        "linear_x": 0.0,
        "linear_y": 0.0,
        "angular_z": 0.0,
    }
    gate = event_at(gate_events, timestamp) or {"state": "STARTUP"}
    route_ids = tuple(run["route_lanelet_ids"])
    figure = plt.figure(figsize=(12, 7.2), facecolor="#f3f6f7")
    grid = figure.add_gridspec(
        1,
        2,
        width_ratios=(0.92, 1.18),
        left=0.06,
        right=0.97,
        top=0.81,
        bottom=0.13,
        wspace=0.19,
    )
    overview = figure.add_subplot(grid[0, 0])
    local = figure.add_subplot(grid[0, 1])

    draw_route(overview, lanelet_map, route_ids, context=True)
    x_limits, y_limits = overview_limits(poses)
    overview.set_xlim(*x_limits)
    overview.set_ylim(*y_limits)
    overview.set_title("Measured map-v17 route", loc="left", fontsize=11.5, fontweight="bold")
    trail_mask = poses["t"] <= timestamp
    overview.plot(
        poses["x"][trail_mask],
        poses["y"][trail_mask],
        color=phase_color,
        linewidth=2.7,
        zorder=4,
    )
    overview.scatter([pose[0]], [pose[1]], s=70, color=phase_color, edgecolor="white", zorder=8)
    overview.scatter(
        [poses["x"][0], poses["x"][-1]],
        [poses["y"][0], poses["y"][-1]],
        marker="o",
        s=30,
        color=["#1769aa", "#187044"],
        zorder=7,
    )

    draw_route(local, lanelet_map, route_ids)
    local.set_title("Current 30-point boundaries", loc="left", fontsize=11.5, fontweight="bold")
    local_trail = (
        (poses["t"] <= timestamp)
        & (poses["t"] >= max(float(poses["t"][0]), timestamp - 4.0))
    )
    local.plot(
        poses["x"][local_trail],
        poses["y"][local_trail],
        color=phase_color,
        linewidth=2.3,
        zorder=4,
    )
    planning_edge = HOLD_RED if phase == "PLANNING-MARGIN HOLD" else PLANNING_EDGE
    body_edge = RECOVERY_GREEN if phase == "REVERSE_YAW_RIGHT" else PHYSICAL_EDGE
    draw_boundaries(
        local,
        contract,
        pose,
        planning_edge=planning_edge,
        body_edge=body_edge,
        linewidth=3.0,
        labels=True,
    )
    local.set_xlim(pose[0] - 2.05, pose[0] + 2.05)
    local.set_ylim(pose[1] - 2.05, pose[1] + 2.05)
    if timestamp >= milestones["first_hold"] - 0.25:
        local.scatter([hit[0]], [hit[1]], marker="x", s=95, linewidth=2.5, color=HOLD_RED, zorder=12)
    if milestones["first_hold"] <= timestamp <= milestones["hold_released"]:
        hold = run["first_hold"]
        local.add_patch(
            FancyArrowPatch(
                (hold["x"], hold["y"]),
                pose[:2],
                arrowstyle="-|>",
                mutation_scale=16,
                linewidth=2.0,
                color=RECOVERY_GREEN,
                zorder=12,
            )
        )
    local.legend(loc="lower left", fontsize=8.5, facecolor="white")

    speed = math.hypot(output.get("linear_x", 0.0), output.get("linear_y", 0.0))
    figure.text(
        0.06,
        0.93,
        "Measured road simulation: tapered/rounded boundary",
        fontsize=21,
        fontweight="bold",
        color="#17272d",
    )
    figure.text(
        0.06,
        0.865,
        phase,
        fontsize=13,
        fontweight="bold",
        color=phase_color,
    )
    figure.text(
        0.96,
        0.91,
        (
            f"t {timestamp:05.2f} s   x {pose[0]:+.2f}   y {pose[1]:+.2f}   "
            f"yaw {math.degrees(pose[2]):+.1f} deg\n"
            f"output {speed:.3f} m/s   wz {output.get('angular_z', 0.0):+.3f} rad/s   gate {gate.get('state', 'UNKNOWN')}"
        ),
        ha="right",
        va="top",
        fontsize=10,
        color="#40565e",
    )
    figure.text(
        0.06,
        0.045,
        "MEASURED ROS SIM | body clear | planning margin contact -> bounded recovery",
        fontsize=10.2,
        color="#684c21",
        fontweight="bold",
    )
    figure.text(
        0.96,
        0.045,
        f"physical-road: NO | frame {output_index + 1:02d}/80",
        ha="right",
        fontsize=9.5,
        color="#596b73",
    )

    figure.canvas.draw()
    width, height = figure.canvas.get_width_height()
    rgba = np.asarray(figure.canvas.buffer_rgba())
    image = Image.fromarray(rgba[:, :, :3].copy(), mode="RGB")
    if image.size != (width, height):
        raise RuntimeError("rendered frame size does not match the Matplotlib canvas")
    plt.close(figure)
    return image


def render_gif(run, map_path, contract, output):
    """Render the measured drive, dense recovery, and route-resume animation."""
    lanelet_map = lanelet2.io.load(str(map_path), LocalCartesianProjector(ORIGIN))
    poses = pose_events(run)
    milestones = milestone_times(run)
    times = animation_times(poses, milestones)
    output_events = indexed_events(run, "output")
    gate_events = indexed_events(run, "gate")
    hit = route_hit(run)
    frames = [
        render_gif_frame(
            run,
            lanelet_map,
            contract,
            poses,
            milestones,
            float(timestamp),
            index,
            output_events,
            gate_events,
            hit,
        )
        for index, timestamp in enumerate(times)
    ]
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


def summary_payload(run, run_path, map_path, contract):
    """Return stable measured-run facts used by documentation and tests."""
    poses = pose_events(run)
    milestones = milestone_times(run)
    hold = run["first_hold"]
    release = run["hold_release"]
    yaw_delta = math.degrees(
        math.atan2(
            math.sin(release["yaw_rad"] - hold["yaw_rad"]),
            math.cos(release["yaw_rad"] - hold["yaw_rad"]),
        )
    )
    return {
        "schema_version": 1,
        "classification": "MEASURED ROS SIM",
        "field_claim": False,
        "captured_at_utc": run["captured_at_utc"],
        "scenario": run["scenario"],
        "launch_command": (
            "ROS_DOMAIN_ID=47 ros2 launch camrod_bringup bringup.launch.py "
            "sim:=true rviz:=false clean_before_launch:=false"
        ),
        "probe_command": (
            "automatic_route_recovery_probe.py --scenario b2_recontact "
            "--map lanelet2_maps.osm"
        ),
        "map": run["map"],
        "map_source_sha256": sha256(map_path),
        "runtime_record_sha256": sha256(run_path),
        "geometry": run["geometry"],
        "geometry_source_sha256": contract.source_hashes,
        "route_lanelet_ids": run["route_lanelet_ids"],
        "pose_samples": len(poses["events"]),
        "measured_duration_s": round(float(poses["t"][-1] - poses["t"][0]), 3),
        "milestones_s": {key: round(value, 3) for key, value in milestones.items()},
        "first_hold_boundary_classification": run[
            "first_hold_boundary_classification"
        ],
        "automatic_recovery_motion": run["automatic_recovery_motion"],
        "recovery_displacement_m": run["recovery_displacement_m"],
        "recovery_yaw_delta_deg": round(yaw_delta, 3),
        "maximum_recovery_output_mps": run["maximum_recovery_output_mps"],
        "maximum_recovery_abs_angular_z_radps": run[
            "maximum_recovery_abs_angular_z_radps"
        ],
        "second_hold": run["second_hold"],
        "rapid_recontact_latched": run["rapid_recontact_latched"],
        "mission_completed": run["mission_completed"],
        "final_output": run["final_output"],
        "animation": {
            "frame_count": 80,
            "frame_duration_ms": 120,
            "sampling": "32 drive + 20 dense recovery + 28 resumed route",
        },
    }


def write_summary(run, run_path, map_path, contract, output):
    """Write the stable aggregate runtime result."""
    output.write_text(
        json.dumps(
            summary_payload(run, run_path, map_path, contract),
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )


def write_manifest(output_dir, run_path):
    """Hash raw runtime input and every derived road-simulation artifact."""
    files = (
        (RUN_NAME, run_path),
        (PNG_NAME, output_dir / PNG_NAME),
        (GIF_NAME, output_dir / GIF_NAME),
        (SUMMARY_NAME, output_dir / SUMMARY_NAME),
    )
    lines = [f"{sha256(path)}  {name}" for name, path in files]
    (output_dir / MANIFEST_NAME).write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    """Validate and render one current measured road-simulation record."""
    repo_root_default = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=repo_root_default)
    parser.add_argument("--map", type=Path)
    parser.add_argument("--run", type=Path)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()

    repo_root = args.repo_root.resolve()
    default_output = (
        repo_root
        / "docs"
        / "assets"
        / "module-guides"
        / "control"
        / "test-results"
        / "tapered-rounded-boundary-road-sim-20260810"
    )
    output_dir = (args.output_dir or default_output).resolve()
    map_path = (args.map or (repo_root / "lanelet2_maps.osm")).resolve()
    run_path = (args.run or (default_output / RUN_NAME)).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    run = load_json(run_path)
    contract = load_contract(repo_root)
    validate_run(run, map_path, contract)
    render_png(run, map_path, contract, output_dir / PNG_NAME)
    render_gif(run, map_path, contract, output_dir / GIF_NAME)
    write_summary(run, run_path, map_path, contract, output_dir / SUMMARY_NAME)
    write_manifest(output_dir, run_path)


if __name__ == "__main__":
    main()
