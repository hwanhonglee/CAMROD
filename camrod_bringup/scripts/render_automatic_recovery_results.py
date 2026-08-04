#!/usr/bin/env python3
"""Render automatic reverse/crab recovery runs recorded by the probe."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import lanelet2
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from lanelet2.io import Origin
from lanelet2.projection import LocalCartesianProjector
from matplotlib.patches import FancyArrowPatch, Polygon as PolygonPatch


ORIGIN = Origin(36.8435737, 128.0925646, 0.0)
ROUTE_IDS = (754, 2751, 2720)
CRAB_LANELET_ID = 4677
# HH_260805 - Preserve the map-v14 evidence geometry: 0.10 m longitudinal and
# 0.05 m lateral margins. Current runtime geometry is sourced by the main renderer.
FRONT, REAR = 0.85837, 0.83323
LEFT, RIGHT = 0.58505, 0.58495
FPS = 8


def load(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def map_version(map_path):
    meta = ET.parse(map_path).getroot().find("MetaInfo")
    return int(meta.attrib["map_version"]) if meta is not None else None


def evidence_map_version(runs, fallback, fallback_sha256=None):
    versions = {
        run.get("map", {}).get("map_version")
        for run in runs
        if run.get("map", {}).get("map_version") is not None
    }
    if len(versions) > 1:
        raise ValueError(f"input evidence uses multiple map versions: {sorted(versions)}")
    evidence_version = next(iter(versions), fallback)
    if evidence_version != fallback:
        raise ValueError(
            f"evidence map v{evidence_version} does not match OSM map v{fallback}"
        )
    hashes = {
        run.get("map", {}).get("sha256")
        for run in runs
        if run.get("map", {}).get("sha256")
    }
    if len(hashes) > 1:
        raise ValueError("input evidence uses multiple OSM map hashes")
    if hashes and fallback_sha256 and next(iter(hashes)) != fallback_sha256:
        raise ValueError("evidence OSM hash does not match the selected map")
    return evidence_version


def retry_latched(run):
    if "rapid_recontact_latched" in run:
        return bool(run["rapid_recontact_latched"])
    return any(
        event.get("event") == "gate"
        and any(
            marker in event.get("message", "")
            for marker in (
                "route_safety_retry_latched",
                "rapid_recontact_latched",
                "rapid route recontact latched",
            )
        )
        for event in run.get("timeline", [])
    )


def lane_polygon(lanelet):
    left = [(point.x, point.y) for point in lanelet.leftBound]
    right = [(point.x, point.y) for point in lanelet.rightBound]
    return left + list(reversed(right))


def footprint(x, y, yaw):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [
        (x + cosine * px - sine * py, y + sine * px + cosine * py)
        for px, py in (
            (FRONT, LEFT),
            (FRONT, -RIGHT),
            (-REAR, -RIGHT),
            (-REAR, LEFT),
        )
    ]


def milestone(run, name):
    return next(
        event
        for event in run["timeline"]
        if event["event"] == "milestone" and event["name"] == name
    )


def poses(run):
    values = [event for event in run["timeline"] if event["event"] == "pose"]
    return (
        np.asarray([event["t"] for event in values]),
        np.asarray([event["x"] for event in values]),
        np.asarray([event["y"] for event in values]),
        np.unwrap(np.asarray([event["yaw_rad"] for event in values])),
    )


def pose_at(run, timestamp):
    times, xs, ys, yaws = poses(run)
    return (
        float(np.interp(timestamp, times, xs)),
        float(np.interp(timestamp, times, ys)),
        float(np.interp(timestamp, times, yaws)),
    )


def draw_lanelets(ax, lanelet_map, lanelet_ids):
    for lanelet_id in lanelet_ids:
        lanelet = lanelet_map.laneletLayer[lanelet_id]
        ax.add_patch(
            PolygonPatch(
                lane_polygon(lanelet),
                closed=True,
                facecolor="#eef2f3",
                edgecolor="#52646d",
                linewidth=1.4,
                zorder=1,
            )
        )
        centerline = [(point.x, point.y) for point in lanelet.centerline]
        ax.plot(
            [point[0] for point in centerline],
            [point[1] for point in centerline],
            color="#66828f",
            linewidth=1.2,
            linestyle="--",
            zorder=2,
        )
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#d6dde0", linewidth=0.5)
    ax.set_xlabel("map x [m]")
    ax.set_ylabel("map y [m]")


def draw_robot(ax, x, y, yaw, color, alpha=0.75):
    patch = PolygonPatch(
        footprint(x, y, yaw),
        closed=True,
        facecolor=color,
        edgecolor="white",
        linewidth=1.7,
        alpha=alpha,
        zorder=7,
    )
    ax.add_patch(patch)
    ax.plot([x], [y], "o", color="white", markeredgecolor="#111827", zorder=9)
    ax.plot(
        [x, x + 0.72 * math.cos(yaw)],
        [y, y + 0.72 * math.sin(yaw)],
        color="#111827",
        linewidth=2.2,
        zorder=9,
    )
    return patch


def arrow_body(ax, start, yaw, body_x, body_y, color, label):
    world_x = math.cos(yaw) * body_x - math.sin(yaw) * body_y
    world_y = math.sin(yaw) * body_x + math.cos(yaw) * body_y
    end = (start[0] + world_x, start[1] + world_y)
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle="-|>",
        mutation_scale=18,
        linewidth=2.8,
        color=color,
        zorder=11,
    )
    ax.add_patch(arrow)
    ax.text(
        end[0],
        end[1],
        label,
        color=color,
        fontsize=8.5,
        fontweight="bold",
        ha="center",
        va="bottom",
        zorder=12,
    )


def trajectory(run, start_name, end_name):
    start = milestone(run, start_name)["t"]
    end = milestone(run, end_name)["t"]
    times, xs, ys, yaws = poses(run)
    mask = (times >= start) & (times <= end)
    return times[mask], xs[mask], ys[mask], yaws[mask]


def render_contact_sheet(output, lanelet_map, route, reverse, crab, version):
    fig = plt.figure(figsize=(16, 9), facecolor="#f4f6f7")
    grid = fig.add_gridspec(2, 3, width_ratios=(1.2, 1.2, 0.85), hspace=0.27, wspace=0.22)
    axes = [fig.add_subplot(grid[row, col]) for row in range(2) for col in range(2)]
    panel = fig.add_subplot(grid[:, 2])
    panel.axis("off")

    for index in (0, 1, 3):
        draw_lanelets(axes[index], lanelet_map, ROUTE_IDS)
        axes[index].set_xlim(-2.5, 6.8)
        axes[index].set_ylim(43.8, 46.4)
    for index in (1, 3):
        axes[index].set_xlim(2.6, 6.2)
        axes[index].set_ylim(43.8, 46.1)
    draw_lanelets(axes[2], lanelet_map, (CRAB_LANELET_ID,))
    axes[2].set_xlim(94.7, 98.2)
    axes[2].set_ylim(-18.8, -15.8)

    times, xs, ys, yaws = poses(route)
    first = milestone(route, "first_hold")
    before = times <= first["t"]
    axes[0].plot(xs[before], ys[before], color="#1769aa", linewidth=2.3, zorder=4)
    indices = np.flatnonzero(before)
    for index in indices[np.linspace(0, len(indices) - 1, 5, dtype=int)]:
        draw_robot(axes[0], xs[index], ys[index], yaws[index], "#1976d2", 0.25)
    axes[0].set_title(
        "1. Normal RPP drive: curved-path yaw",
        loc="left",
        fontsize=11,
        fontweight="bold",
    )

    reverse_hold = milestone(reverse, "first_hold")
    reverse_release = milestone(reverse, "hold_released")
    _, rxs, rys, _ = trajectory(reverse, "first_hold", "hold_released")
    axes[1].plot(rxs, rys, color="#16864b", linewidth=3.0, zorder=4)
    draw_robot(
        axes[1],
        reverse_hold["x"],
        reverse_hold["y"],
        reverse_hold["yaw_rad"],
        "#c62828",
        0.38,
    )
    draw_robot(
        axes[1],
        reverse_release["x"],
        reverse_release["y"],
        reverse_release["yaw_rad"],
        "#16864b",
        0.78,
    )
    arrow_body(
        axes[1],
        (reverse_hold["x"], reverse_hold["y"]),
        reverse_hold["yaw_rad"],
        -0.72,
        0.0,
        "#16864b",
        "AUTO REVERSE",
    )
    axes[1].set_title(
        "2. Both crab sides blocked: reverse -0.05 m/s",
        loc="left",
        fontsize=11,
        fontweight="bold",
    )

    crab_hold = milestone(crab, "first_hold")
    crab_release = milestone(crab, "hold_released")
    _, cxs, cys, _ = trajectory(crab, "first_hold", "hold_released")
    axes[2].plot(cxs, cys, color="#16864b", linewidth=3.0, zorder=4)
    draw_robot(
        axes[2], crab_hold["x"], crab_hold["y"], crab_hold["yaw_rad"],
        "#c62828", 0.40,
    )
    draw_robot(
        axes[2], crab_release["x"], crab_release["y"], crab_release["yaw_rad"],
        "#16864b", 0.78,
    )
    arrow_body(
        axes[2],
        (crab_hold["x"], crab_hold["y"]),
        crab_hold["yaw_rad"],
        0.0,
        0.72,
        "#16864b",
        "AUTO CRAB LEFT",
    )
    axes[2].set_title(
        "3. One-sided contact: crab 0.05 m/s, yaw held",
        loc="left",
        fontsize=11,
        fontweight="bold",
    )

    route_release = milestone(route, "hold_released")
    route_second = milestone(route, "second_hold")
    _, txs, tys, _ = trajectory(route, "hold_released", "second_hold")
    axes[3].plot(txs, tys, color="#6b3fb5", linewidth=3.0, zorder=4)
    draw_robot(
        axes[3],
        route_release["x"],
        route_release["y"],
        route_release["yaw_rad"],
        "#6b3fb5",
        0.32,
    )
    draw_robot(
        axes[3],
        route_second["x"],
        route_second["y"],
        route_second["yaw_rad"],
        "#c62828",
        0.80,
    )
    retry_distance = route["same_goal_retry_displacement_m"]
    retry_yaw = route["same_goal_retry_yaw_delta_deg"]
    recontact_s = route.get("rapid_recontact_after_release_s")
    axes[3].set_title(
        f"4. Retry: {retry_distance:.3f} m, yaw {retry_yaw:+.2f} deg, "
        f"{'latched' if retry_latched(route) else 'held'}",
        loc="left",
        fontsize=11,
        fontweight="bold",
    )

    panel.text(
        0.0,
        0.98,
        f"Map v{version} recovery result",
        fontsize=15,
        fontweight="bold",
        va="top",
    )
    summary = [
        ("Map source", f"v{version}"),
        ("One-side contact", "CRAB_LEFT"),
        ("Crab displacement", f"{crab['recovery_displacement_m']:.3f} m"),
        ("Both sides blocked", "REVERSE"),
        ("Reverse displacement", f"{reverse['recovery_displacement_m']:.3f} m"),
        ("Retry displacement", f"{retry_distance:.3f} m"),
        ("Retry yaw change", f"{retry_yaw:+.2f} deg"),
        (
            "Rapid recontact",
            f"{recontact_s:.3f} s / LATCHED"
            if recontact_s is not None and retry_latched(route)
            else "not latched",
        ),
        ("Route finish", "NO: fail-closed"),
    ]
    y = 0.90
    for label, value in summary:
        panel.text(0.0, y, label, fontsize=9.2, color="#58656c")
        panel.text(
            0.58, y, value, fontsize=9.6, fontweight="bold",
            color=(
                "#b3261e"
                if value.startswith("NO") or value == "not latched"
                else "#1f5135"
            ),
        )
        y -= 0.045
    panel.text(0.0, y - 0.01, "Verified parameters", fontsize=13, fontweight="bold")
    y -= 0.075
    for line in (
        "RPP 15 Hz | lookahead 1.1 m",
        "0.4 m/s x gate 0.5 = 0.2 m/s",
        "recovery raw 0.10 x gate 0.5",
        "maximum distance 0.40 m",
        "clear evidence 1.0 s",
        "angular.z = 0 during boundary contact",
    ):
        panel.text(0.0, y, line, fontsize=9.2, color="#37474f")
        y -= 0.041
    panel.text(
        0.0,
        0.02,
        "Safety rule\nOne automatic route release is allowed.\n"
        "Rapid same-route recontact latches the hold.\n"
        "Output remains zero until stop/replan/re-engage.",
        fontsize=9.4,
        color="#263238",
        va="bottom",
        bbox={"boxstyle": "round,pad=0.55", "facecolor": "#edf4ef", "edgecolor": "#8aab94"},
    )
    fig.suptitle(
        f"CAMROD map v{version} automatic boundary recovery - full bringup simulation",
        fontsize=18,
        fontweight="bold",
        y=0.985,
    )
    fig.savefig(output, dpi=125, bbox_inches="tight")
    plt.close(fig)


def render_policy(output):
    fig, ax = plt.subplots(figsize=(13, 7.2), facecolor="#f4f6f7")
    ax.axis("off")
    columns = ["Projected left", "Projected right", "Projected reverse", "Action"]
    rows = [
        ["CLEAR", "BLOCKED", "any", "CRAB_LEFT"],
        ["BLOCKED", "CLEAR", "any", "CRAB_RIGHT"],
        ["BLOCKED", "BLOCKED", "CLEAR", "REVERSE"],
        ["CLEAR", "CLEAR", "any", "STOP: ambiguous"],
        ["BLOCKED", "BLOCKED", "BLOCKED", "STOP: no safe candidate"],
    ]
    table = ax.table(
        cellText=rows,
        colLabels=columns,
        cellLoc="center",
        colLoc="center",
        bbox=[0.03, 0.31, 0.94, 0.55],
    )
    table.auto_set_font_size(False)
    table.set_fontsize(11)
    for (row, column), cell in table.get_celld().items():
        cell.set_edgecolor("#9aa8ae")
        if row == 0:
            cell.set_facecolor("#314a55")
            cell.get_text().set_color("white")
            cell.get_text().set_fontweight("bold")
        elif "STOP" in cell.get_text().get_text():
            cell.set_facecolor("#fde9e7")
        elif column == 3:
            cell.set_facecolor("#e2f1e7")
            cell.get_text().set_fontweight("bold")
        else:
            cell.set_facecolor("#f9fbfb")
    ax.text(0.03, 0.95, "Automatic recovery decision policy", fontsize=20, fontweight="bold")
    ax.text(
        0.03,
        0.90,
        "Every candidate uses the projected complete 1.6916 x 1.2700 m footprint and live obstacle interlocks.",
        fontsize=11,
        color="#455a64",
    )
    ax.text(
        0.03,
        0.22,
        "Direction lock",
        fontsize=13,
        fontweight="bold",
    )
    ax.text(
        0.03,
        0.16,
        "The first unique candidate remains fixed until hold release. A later block stops motion; it never switches direction.",
        fontsize=10.5,
        color="#37474f",
    )
    ax.text(
        0.03,
        0.08,
        "Rotation during contact: disabled. RPP steering resumes after 1.0 s of fresh full-footprint clearance.",
        fontsize=10.5,
        color="#37474f",
    )
    fig.savefig(output, dpi=130, bbox_inches="tight")
    plt.close(fig)


def render_gif(output, lanelet_map, route, reverse, crab, version):
    reverse_start = milestone(reverse, "first_hold")["t"]
    reverse_end = milestone(reverse, "second_hold")["t"]
    crab_start = milestone(crab, "first_hold")["t"]
    crab_end = milestone(crab, "hold_released")["t"]
    reverse_times, reverse_xs, reverse_ys, reverse_yaws = poses(reverse)
    crab_times, crab_xs, crab_ys, crab_yaws = poses(crab)

    duration = 12.0
    fig, axes = plt.subplots(1, 2, figsize=(12.8, 7.2), facecolor="#f4f6f7")
    draw_lanelets(axes[0], lanelet_map, ROUTE_IDS)
    axes[0].set_xlim(2.3, 6.5)
    axes[0].set_ylim(43.7, 46.35)
    axes[0].set_title("Narrow route: reverse then retry yaw", fontweight="bold")
    draw_lanelets(axes[1], lanelet_map, (CRAB_LANELET_ID,))
    axes[1].set_xlim(94.0, 99.0)
    axes[1].set_ylim(-19.5, -15.0)
    axes[1].set_title("One-sided contact: automatic crab left", fontweight="bold")

    trails = [axes[0].plot([], [], color="#4e2a84", linewidth=2.2)[0],
              axes[1].plot([], [], color="#16864b", linewidth=2.2)[0]]
    robots = []
    statuses = []
    for ax, initial in zip(
        axes,
        (pose_at(reverse, reverse_start), pose_at(crab, crab_start)),
    ):
        robot = PolygonPatch(
            footprint(*initial),
            closed=True,
            facecolor="#c62828",
            edgecolor="white",
            linewidth=2.0,
            alpha=0.78,
            zorder=7,
        )
        ax.add_patch(robot)
        robots.append(robot)
        statuses.append(
            ax.text(
                0.03, 0.95, "", transform=ax.transAxes, va="top",
                color="white", fontsize=10, fontweight="bold",
                bbox={"boxstyle": "round,pad=0.4", "facecolor": "#c62828", "edgecolor": "none"},
                zorder=12,
            )
        )

    reverse_release = milestone(reverse, "hold_released")["t"]
    reverse_second_hold = milestone(reverse, "second_hold")["t"]
    crab_release = milestone(crab, "hold_released")["t"]

    def update(frame):
        display_time = frame / FPS
        # HH_260804 - Complete motion in 75% of the animation and retain a
        # final dwell so reviewers can read the fail-closed retry-latch state.
        progress = min(1.0, display_time / (duration * 0.75))
        reverse_actual = reverse_start + progress * (
            reverse_end - reverse_start
        )
        crab_actual = crab_start + progress * (
            crab_end - crab_start
        )
        for index, values in enumerate(
            (
                (reverse_actual, reverse_times, reverse_xs, reverse_ys, reverse_yaws),
                (crab_actual, crab_times, crab_xs, crab_ys, crab_yaws),
            )
        ):
            actual, times, xs, ys, yaws = values
            x = float(np.interp(actual, times, xs))
            y = float(np.interp(actual, times, ys))
            yaw = float(np.interp(actual, times, yaws))
            mask = (times >= (reverse_start if index == 0 else crab_start)) & (times <= actual)
            trails[index].set_data(xs[mask], ys[mask])
            robots[index].set_xy(footprint(x, y, yaw))
            release = reverse_release if index == 0 else crab_release
            if actual < release:
                label = "AUTO REVERSE  -0.05 m/s" if index == 0 else "AUTO CRAB LEFT  0.05 m/s"
                color = "#16864b"
            elif index == 0 and actual < reverse_second_hold:
                label = "RPP RESUMED  yaw changes"
                color = "#6b3fb5"
            elif index == 0 and retry_latched(reverse):
                label = "RAPID RETRY LATCHED  output zero"
                color = "#b3261e"
            elif index == 0:
                label = "SECOND ROUTE HOLD"
                color = "#b3261e"
            else:
                label = "HOLD RELEASED  yaw held"
                color = "#1769aa"
            statuses[index].set_text(label + f"\nyaw {math.degrees(yaw):+.2f} deg")
            statuses[index].get_bbox_patch().set_facecolor(color)
            robots[index].set_facecolor(color)
        return trails + robots + statuses

    fig.suptitle(
        f"CAMROD map v{version} production-owned boundary recovery",
        fontsize=18,
        fontweight="bold",
    )
    gif = animation.FuncAnimation(
        fig,
        update,
        frames=math.ceil(duration * FPS),
        interval=1000 / FPS,
        blit=False,
        repeat=True,
    )
    gif.save(output, writer=animation.PillowWriter(fps=FPS), dpi=92)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--route", required=True, type=Path)
    parser.add_argument("--reverse", required=True, type=Path)
    parser.add_argument("--crab", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument(
        "--artifact-prefix",
        default="",
        help="optional output filename prefix; omitted preserves legacy names",
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    lanelet_map = lanelet2.io.load(str(args.map), LocalCartesianProjector(ORIGIN))
    route = load(args.route)
    reverse = load(args.reverse)
    crab = load(args.crab)
    version = evidence_map_version(
        (route, reverse, crab),
        map_version(args.map),
        hashlib.sha256(args.map.read_bytes()).hexdigest(),
    )
    if args.artifact_prefix:
        contact = args.output_dir / f"{args.artifact_prefix}-contact-sheet.png"
        policy = args.output_dir / f"{args.artifact_prefix}-policy.png"
        gif = args.output_dir / f"{args.artifact_prefix}.gif"
    else:
        # Keep the established documentation paths stable for existing runs.
        contact = args.output_dir / "automatic-owner-route-retry-contact-sheet.png"
        policy = args.output_dir / "automatic-owner-policy.png"
        gif = args.output_dir / "automatic-owner-route-retry.gif"
    render_contact_sheet(contact, lanelet_map, route, reverse, crab, version)
    render_policy(policy)
    render_gif(gif, lanelet_map, route, reverse, crab, version)
    for path in (contact, policy, gif):
        print(path)


if __name__ == "__main__":
    main()
