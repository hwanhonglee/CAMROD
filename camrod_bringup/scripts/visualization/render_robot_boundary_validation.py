#!/usr/bin/env python3
"""Render the fixed 2026-08-06 robot-boundary evidence as one concise figure."""

# HH_260807 - This renderer intentionally preserves the historical 5 cm
# geometry captured by the four raw scenario records. It is not the active
# 10 cm production geometry renderer.

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle

try:
    from render_tapered_rounded_boundary import load_contract
except ModuleNotFoundError:
    from camrod_bringup.scripts.visualization.render_tapered_rounded_boundary import (
        load_contract,
    )


RESULT_FILES = {
    "route": "runtime-route-clear.json",
    "margin_stop": "runtime-margin-contact-stop.json",
    "margin_recovery": "runtime-margin-recovery.json",
    "body_stop": "runtime-physical-body-hard-stop.json",
}


def load_results(input_dir):
    results = {}
    for key, filename in RESULT_FILES.items():
        path = input_dir / filename
        with path.open("r", encoding="utf-8") as stream:
            results[key] = json.load(stream)
    return results


def add_result_row(axis, y, color, title, detail):
    axis.add_patch(Rectangle((0.02, y - 0.105), 0.012, 0.16, color=color))
    axis.text(0.05, y, title, fontsize=13, fontweight="bold", va="center")
    axis.text(0.31, y, detail, fontsize=11.5, color="#34454d", va="center")
    axis.text(
        0.95,
        y,
        "PASS",
        fontsize=11.5,
        fontweight="bold",
        color="#176b45",
        ha="right",
        va="center",
    )


def render(results, contract, output):
    route = results["route"]
    margin_stop = results["margin_stop"]
    margin_recovery = results["margin_recovery"]
    body_stop = results["body_stop"]

    intervals = margin_recovery["live_lateral_sweep"]["margin_only_intervals"]
    margin_offset = margin_recovery["live_lateral_sweep"][
        "selected_lateral_offset_m"
    ]
    body_offset = body_stop["live_lateral_sweep"]["selected_lateral_offset_m"]

    figure = plt.figure(figsize=(16, 9), facecolor="#f4f7f8")
    grid = figure.add_gridspec(
        2,
        2,
        height_ratios=(1.15, 1.0),
        left=0.055,
        right=0.965,
        top=0.84,
        bottom=0.10,
        hspace=0.29,
        wspace=0.22,
    )
    sweep_axis = figure.add_subplot(grid[0, 0])
    geometry_axis = figure.add_subplot(grid[0, 1])
    result_axis = figure.add_subplot(grid[1, :])

    figure.text(
        0.055,
        0.93,
        "Boundary policy record with current rounded contours",
        fontsize=25,
        fontweight="bold",
        color="#15262d",
    )
    figure.text(
        0.055,
        0.885,
        "Current contours are source-derived; policy rows retain the 2026-08-06 measured simulation values.",
        fontsize=13,
        color="#51646d",
    )

    sweep_axis.set_title("Live lateral sweep on map v15", loc="left", fontweight="bold")
    sweep_axis.axhspan(0.38, 0.62, color="#dbe8df", alpha=0.85)
    sweep_axis.text(-0.48, 0.50, "route reference", va="center", color="#315f46")
    for interval in intervals:
        sweep_axis.axvspan(
            interval["start_offset_m"],
            interval["end_offset_m"],
            color="#efb647",
            alpha=0.75,
        )
    sweep_axis.axvline(margin_offset, color="#c98014", linewidth=2.5)
    sweep_axis.scatter([margin_offset], [0.70], color="#c98014", s=85, zorder=5)
    sweep_axis.text(
        margin_offset,
        0.79,
        f"margin test  {margin_offset:+.2f} m",
        ha="center",
        color="#945b08",
        fontweight="bold",
    )
    sweep_axis.axvline(body_offset, color="#ba3434", linewidth=2.5)
    sweep_axis.scatter([body_offset], [0.25], color="#ba3434", s=85, zorder=5)
    sweep_axis.text(
        body_offset,
        0.13,
        f"body test  {body_offset:+.2f} m",
        ha="center",
        color="#8a2323",
        fontweight="bold",
    )
    sweep_axis.set_xlim(-0.50, 0.50)
    sweep_axis.set_ylim(0.0, 1.0)
    sweep_axis.set_xlabel("lateral offset from the controlled base pose (m)")
    sweep_axis.set_yticks([])
    sweep_axis.grid(axis="x", color="#d7dfe2", linewidth=0.8)
    for spine in sweep_axis.spines.values():
        spine.set_color("#b8c5ca")

    geometry_axis.set_title("Two independent safety envelopes", loc="left", fontweight="bold")
    geometry_axis.set_aspect("equal")
    planning = Polygon(
        contract.planning,
        closed=True,
        facecolor="#f4c864",
        edgecolor="#aa7000",
        linewidth=2.5,
        alpha=0.50,
        joinstyle="round",
        label="planning boundary",
    )
    body = Polygon(
        contract.physical,
        closed=True,
        facecolor="#55bfd0",
        edgecolor="#137587",
        linewidth=2.5,
        alpha=0.72,
        joinstyle="round",
        label="physical body",
    )
    geometry_axis.add_patch(planning)
    geometry_axis.add_patch(body)
    geometry_axis.scatter([0], [0], color="#1f2f36", s=45, zorder=5)
    geometry_axis.annotate(
        "robot_center_link",
        (0, 0),
        xytext=(0.04, 0.09),
        fontsize=10.5,
        color="#1f2f36",
    )
    geometry_axis.text(
        -0.72,
        0.67,
        (
            "Planning: 1.59160 x 1.27000 m (30 points)\n"
            "10 cm margin, recoverable only after projection"
        ),
        color="#805400",
        fontsize=11,
        fontweight="bold",
    )
    geometry_axis.text(
        -0.66,
        -0.76,
        "Physical: 1.39160 x 1.07000 m (30 points)\ncontact => hard stop, no recovery command",
        color="#0d6575",
        fontsize=11,
        fontweight="bold",
    )
    geometry_axis.set_xlim(-0.85, 0.87)
    geometry_axis.set_ylim(-0.84, 0.84)
    geometry_axis.set_xticks([])
    geometry_axis.set_yticks([])
    for spine in geometry_axis.spines.values():
        spine.set_visible(False)

    result_axis.set_title("Measured final-policy outcomes", loc="left", fontweight="bold")
    result_axis.set_xlim(0, 1)
    result_axis.set_ylim(0, 1)
    result_axis.axis("off")
    add_result_row(
        result_axis,
        0.82,
        "#277a52",
        "Normal route",
        (
            f"{route['displacement_m']:.4f} m, goal error "
            f"{route['goal_error_m']:.4f} m, no route hold"
        ),
    )
    add_result_row(
        result_axis,
        0.60,
        "#d69720",
        "Margin stop",
        (
            "body max cost 70 / planning max cost 100; "
            f"final output {margin_stop['maximum_final_output_mps_during_challenge']:.1f} m/s"
        ),
    )
    add_result_row(
        result_axis,
        0.38,
        "#2c72b8",
        "Margin recovery",
        (
            f"{margin_recovery['automatic_recovery_motion']}, "
            f"{margin_recovery['recovery_displacement_m']:.3f} m at <= "
            f"{margin_recovery['maximum_recovery_output_mps']:.2f} m/s"
        ),
    )
    add_result_row(
        result_axis,
        0.16,
        "#ba3434",
        "Physical hard stop",
        (
            "owner motion false, recovery output "
            f"{body_stop['maximum_recovery_output_mps']:.1f} m/s, "
            f"pose variation {body_stop['observed_displacement_m']:.4f} m"
        ),
    )

    map_info = route["map"]
    figure.text(
        0.055,
        0.035,
        (
            f"AMD64 SIM RECORD | map v{map_info['map_version']} "
            f"SHA {map_info['sha256'][:12]}...{map_info['sha256'][-4:]} | "
            "CURRENT CONTOUR REPLAY; measured historical geometry remains in JSON | FIELD PASS: FALSE"
        ),
        fontsize=10.5,
        color="#6a3d3d",
        fontweight="bold",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150, facecolor=figure.get_facecolor())
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[3],
    )
    args = parser.parse_args()
    render(load_results(args.input_dir), load_contract(args.repo_root.resolve()), args.output)


if __name__ == "__main__":
    main()
