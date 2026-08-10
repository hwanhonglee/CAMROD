#!/usr/bin/env python3
"""Render historical measurements beside the active rounded boundary contract."""

# HH_260810 - Historical bags/grids are not committed, so these figures retain
# recorded JSON metrics without pretending to recompute them. Every boundary
# drawing is the current source-derived tapered/rounded contour.

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402

try:
    from render_tapered_rounded_boundary import load_contract
except ModuleNotFoundError:
    from camrod_bringup.scripts.visualization.render_tapered_rounded_boundary import (
        load_contract,
    )


BG = "#f3f6f7"
INK = "#17272d"
MUTED = "#586970"
CYAN = "#007f8f"
CYAN_FILL = "#8bd6df"
AMBER = "#a86d00"
AMBER_FILL = "#f5cd70"
RED = "#b42318"
GREEN = "#18794e"
BLUE = "#1769aa"


def load_json(path: Path) -> dict:
    """Load one committed structured result."""
    return json.loads(path.read_text(encoding="utf-8"))


def style_axis(axis) -> None:
    """Apply the shared documentation chart style."""
    axis.set_facecolor("white")
    axis.grid(color="#e1e7e9", linewidth=0.8, zorder=0)
    for spine in axis.spines.values():
        spine.set_color("#bec9cd")


def draw_current_contours(axis, contract) -> None:
    """Draw only the active 30-point physical/planning contours."""
    axis.add_patch(
        Polygon(
            contract.planning,
            closed=True,
            facecolor=AMBER_FILL,
            edgecolor=AMBER,
            alpha=0.42,
            linewidth=2.8,
            joinstyle="round",
            label="current planning contour",
        )
    )
    axis.add_patch(
        Polygon(
            contract.physical,
            closed=True,
            facecolor=CYAN_FILL,
            edgecolor=CYAN,
            alpha=0.72,
            linewidth=2.8,
            joinstyle="round",
            label="current physical contour",
        )
    )
    axis.scatter([0.0], [0.0], color=INK, s=55, zorder=6)
    axis.annotate(
        "+X front",
        xy=(0.78, 0.0),
        xytext=(0.18, 0.0),
        arrowprops={"arrowstyle": "-|>", "color": RED, "lw": 2.2},
        color=RED,
        va="center",
        fontweight="bold",
    )
    axis.set_xlim(-0.95, 1.0)
    axis.set_ylim(-0.80, 0.80)
    axis.set_aspect("equal")
    axis.set_xticks([])
    axis.set_yticks([])
    axis.legend(loc="lower left", fontsize=9, framealpha=0.95)
    style_axis(axis)


def header(figure, title: str, subtitle: str) -> None:
    """Add a stable title and evidence classification."""
    figure.text(0.055, 0.94, title, fontsize=24, fontweight="bold", color=INK)
    figure.text(0.055, 0.895, subtitle, fontsize=12.5, color=MUTED)
    figure.text(
        0.95,
        0.93,
        "HISTORICAL METRICS + CURRENT CONTOUR",
        ha="right",
        fontsize=10.5,
        fontweight="bold",
        color="#7a4d00",
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "#fff2cf", "edgecolor": "#d6ae52"},
    )


def save(figure, output: Path) -> None:
    """Write one high-resolution PNG."""
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150, facecolor=figure.get_facecolor())
    plt.close(figure)


def render_cmd_vel(result: dict, contract, output: Path) -> None:
    """Render the stop-go correction without reconstructing the absent bag."""
    before = result["reported_run"]
    after = result["corrected_full_route_run"]
    contact = result["first_real_margin_contact"]
    false_positive = result["raster_false_positive_regression"]

    figure = plt.figure(figsize=(16, 9), facecolor=BG)
    header(
        figure,
        "B7 cmd_vel stop-go diagnosis",
        "Recorded AMD64 metrics | current tapered/rounded geometry shown only as source context",
    )
    grid = figure.add_gridspec(
        1, 3, left=0.055, right=0.96, top=0.82, bottom=0.12, wspace=0.24
    )
    comparison = figure.add_subplot(grid[0, 0])
    geometry = figure.add_subplot(grid[0, 1])
    summary = figure.add_subplot(grid[0, 2])

    categories = ("stationary\nhandoffs", "input stale\nevents")
    before_values = (before["stationary_command_source_handoffs"], before["cmd_vel_input_stale_events"])
    after_values = (after["stationary_command_source_handoffs"], after["cmd_vel_input_stale_events"])
    x_values = (0, 1)
    comparison.bar([x - 0.18 for x in x_values], before_values, 0.36, color=RED, label="before")
    comparison.bar([x + 0.18 for x in x_values], after_values, 0.36, color=GREEN, label="corrected")
    comparison.set_xticks(x_values, categories)
    comparison.set_ylabel("event count")
    comparison.set_title("Artificial stop-go removed", loc="left", fontweight="bold")
    comparison.legend()
    style_axis(comparison)

    draw_current_contours(geometry, contract)
    geometry.set_title("Active source geometry", loc="left", fontweight="bold")
    geometry.text(
        0.02,
        0.97,
        "30 points each\nfront taper 0.12 m\nbody R0.05 m / planning R0.15 m",
        transform=geometry.transAxes,
        va="top",
        fontsize=10,
        color=INK,
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "white", "edgecolor": "#b7c4c8"},
    )

    summary.axis("off")
    summary.set_title("Recorded outcomes", loc="left", fontweight="bold")
    rows = (
        ("Clear-drive window", f"{after['clear_window_before_first_real_margin_contact_s']:.2f} s", GREEN),
        ("Nav2 command rate", f"{after['nav2_command_rate_hz']:.3f} Hz", BLUE),
        ("Gate output rate", f"{after['gate_output_rate_hz']:.3f} Hz", BLUE),
        ("Coarse-grid false stop", "passed after 0.05 m grid", GREEN),
        ("Vector body clearance", f"{false_positive['physical_body_vector_clearance_m'] * 100:.2f} cm", GREEN),
        ("First real stop", contact["classification"], AMBER),
        ("Planning overrun", f"{contact['planning_boundary_maximum_overrun_m'] * 1000:.2f} mm", AMBER),
        ("Route completed", "NO", RED),
    )
    y_value = 0.91
    for label, value, color in rows:
        summary.text(0.0, y_value, label, fontsize=9.5, color=MUTED)
        summary.text(0.0, y_value - 0.04, value, fontsize=11, color=color, fontweight="bold", wrap=True)
        y_value -= 0.105
    figure.text(
        0.055,
        0.045,
        "The historical bag/grid is not committed. Metrics are copied from result.json; no contact pose was recomputed with the current contour.",
        fontsize=10,
        color="#6f4c18",
        fontweight="bold",
    )
    save(figure, output)


def render_route_margin(result: dict, contract, output: Path) -> None:
    """Summarize the first historical margin contact and current geometry."""
    contact = result["initial_margin_contact"]
    search = result["offline_body_safe_search"]
    figure = plt.figure(figsize=(16, 9), facecolor=BG)
    header(
        figure,
        "Route boundary recovery: first margin contact",
        "Recorded 2026-08-06 classification | current rounded contour shown without reclassifying the old grid",
    )
    grid = figure.add_gridspec(
        1, 3, left=0.055, right=0.96, top=0.82, bottom=0.12, wspace=0.24
    )
    geometry = figure.add_subplot(grid[0, 0])
    costs = figure.add_subplot(grid[0, 1])
    summary = figure.add_subplot(grid[0, 2])
    draw_current_contours(geometry, contract)
    geometry.set_title("Current source geometry", loc="left", fontweight="bold")

    costs.bar(
        ("physical body", "planning margin"),
        (contact["physical_perimeter_max_cost"], 100),
        color=(CYAN, AMBER),
    )
    costs.axhline(100, color=RED, linestyle="--", linewidth=1.5, label="stop cost")
    costs.set_ylim(0, 112)
    costs.set_ylabel("recorded maximum raster cost")
    costs.set_title("Historical first-contact sample", loc="left", fontweight="bold")
    costs.legend(loc="lower right")
    style_axis(costs)

    summary.axis("off")
    summary.set_title("Recorded result", loc="left", fontweight="bold")
    target = search["target_local_path_index_13"]
    lines = (
        ("Pose", f"({contact['pose']['x_m']:.3f}, {contact['pose']['y_m']:.3f})"),
        ("Yaw", f"{contact['pose']['yaw_deg']:+.2f} deg"),
        ("Physical cost-100", "NO"),
        ("Planning cost-100 points", str(contact["planning_perimeter_cost_100_points"])),
        ("Short body-safe search", "FOUND" if target["path_found"] else "NOT FOUND"),
        ("Search steps", str(target["step_count"])),
        ("Current-contour recompute", "NOT CLAIMED"),
    )
    y_value = 0.91
    for label, value in lines:
        summary.text(0.0, y_value, label, fontsize=9.5, color=MUTED)
        summary.text(0.0, y_value - 0.042, value, fontsize=11, color=INK, fontweight="bold")
        y_value -= 0.115
    figure.text(
        0.055,
        0.045,
        "Historical rectangular dimensions remain in result.json. This replacement contains no rectangular top-view boundary claim.",
        fontsize=10,
        color="#6f4c18",
        fontweight="bold",
    )
    save(figure, output)


def render_route_body(result: dict, contract, output: Path) -> None:
    """Summarize the historical body-contact interval without a fake map replay."""
    body = result["body_only_diagnostic"]
    figure = plt.figure(figsize=(16, 9), facecolor=BG)
    header(
        figure,
        "Route boundary recovery: downstream hard-stop interval",
        "Recorded body-only diagnostic | current rounded contour context | no absent-map reconstruction",
    )
    grid = figure.add_gridspec(
        1, 2, left=0.055, right=0.96, top=0.82, bottom=0.12, wspace=0.24
    )
    interval = figure.add_subplot(grid[0, 0])
    geometry = figure.add_subplot(grid[0, 1])
    start_y = body["start_pose"]["y_m"]
    end_y = body["end_pose"]["y_m"]
    first_y = body["first_physical_body_contact"]["y_m"]
    last_y = body["last_physical_body_contact"]["y_m"]
    interval.plot((0, 0), (start_y, end_y), color=BLUE, linewidth=8, alpha=0.30)
    interval.plot((0, 0), (first_y, last_y), color=RED, linewidth=14, solid_capstyle="round")
    interval.scatter((0, 0, 0, 0), (start_y, first_y, last_y, end_y), color=(GREEN, RED, RED, INK), s=80, zorder=5)
    for y_value, label in (
        (start_y, "diagnostic start"),
        (first_y, "first physical contact"),
        (last_y, "last physical contact"),
        (end_y, "diagnostic end"),
    ):
        interval.text(0.04, y_value, f"{label}  Y={y_value:.3f} m", va="center", fontsize=10, color=INK)
    interval.set_xlim(-0.15, 0.85)
    interval.set_ylabel("recorded map Y [m]")
    interval.set_xticks([])
    interval.set_title("Recorded contact span", loc="left", fontweight="bold")
    style_axis(interval)
    interval.text(
        0.04,
        0.08,
        f"{body['physical_body_cost_100_samples']} / {body['pose_samples']} samples\n"
        f"drive displacement {body['enabled_drive_displacement_m']:.3f} m",
        transform=interval.transAxes,
        fontsize=11,
        color=INK,
        bbox={"boxstyle": "round,pad=0.45", "facecolor": "white", "edgecolor": "#b7c4c8"},
    )
    draw_current_contours(geometry, contract)
    geometry.set_title("Current source geometry", loc="left", fontweight="bold")
    geometry.text(
        0.02,
        0.97,
        "Physical contact remains fail-closed.\nThe current contour was not sampled\nagainst the missing historical grid.",
        transform=geometry.transAxes,
        va="top",
        fontsize=10.5,
        color=RED,
        fontweight="bold",
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "#fdeceb", "edgecolor": "#d79a94"},
    )
    figure.text(
        0.055,
        0.045,
        "Recorded metric verdict is preserved: forcing the historical corridor was unsafe. Current map/contour acceptance requires a new run.",
        fontsize=10,
        color="#6f4c18",
        fontweight="bold",
    )
    save(figure, output)


def main() -> None:
    """Render every historical boundary summary from committed JSON."""
    repo_root_default = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root_default)
    parser.add_argument("--cmd-vel-dir", type=Path)
    parser.add_argument("--route-dir", type=Path)
    args = parser.parse_args()

    repo_root = args.repo_root.resolve()
    cmd_dir = (
        args.cmd_vel_dir
        or repo_root
        / "docs/assets/module-guides/control/test-results/cmd-vel-stop-go-20260806"
    ).resolve()
    route_dir = (
        args.route_dir
        or repo_root
        / "docs/assets/module-guides/control/test-results/route-boundary-recovery-20260806"
    ).resolve()
    contract = load_contract(repo_root)
    render_cmd_vel(load_json(cmd_dir / "result.json"), contract, cmd_dir / "b7-stop-go-diagnosis.png")
    route = load_json(route_dir / "result.json")
    render_route_margin(route, contract, route_dir / "01-margin-contact-analysis.png")
    render_route_body(route, contract, route_dir / "03-body-only-drive-trajectory.png")
    for path in (
        cmd_dir / "b7-stop-go-diagnosis.png",
        route_dir / "01-margin-contact-analysis.png",
        route_dir / "03-body-only-drive-trajectory.png",
    ):
        print(path)


if __name__ == "__main__":
    main()
