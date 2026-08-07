#!/usr/bin/env python3
"""Render source-derived package diagrams and committed runtime evidence."""

import argparse
from io import BytesIO
import json
from pathlib import Path
import re
import warnings

import matplotlib

# This renderer is strictly 2D; ignore a host-only optional mplot3d warning.
warnings.filterwarnings("ignore", message="Unable to import Axes3D.*")
matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
import yaml  # noqa: E402


BG = "#f4f7f8"
INK = "#17242b"
MUTED = "#5d6b73"
GREEN = "#18794e"
GREEN_BG = "#e8f5ed"
BLUE = "#1769aa"
BLUE_BG = "#e9f2fb"
AMBER = "#a85d00"
AMBER_BG = "#fff3dd"
RED = "#b42318"
RED_BG = "#fdeceb"
GRAY_BG = "#e8edef"
WHITE = "#ffffff"

# HH_260805 - Give each package a distinct, paired color identity while
# retaining shared safety red and evidence semantics across the document set.
DEFAULT_THEME = {
    "bg": BG,
    "ink": INK,
    "muted": MUTED,
    "surface": WHITE,
    "edge": "#9aaab2",
    "primary": BLUE,
    "primary_bg": BLUE_BG,
    "secondary": GREEN,
    "secondary_bg": GREEN_BG,
    "accent": AMBER,
    "accent_bg": AMBER_BG,
}
MODULE_THEMES = {
    "bringup": (
        "#1f6f54", "#e4f3eb", "#d97706", "#fff1d6", "#2b6cb0", "#e7f0fb", "#f5f8f4"
    ),
    "common": (
        "#0b7285", "#e3f6f8", "#4954a1", "#eceefa", "#b55d12", "#fff0df", "#f4f8f9"
    ),
    "control": (
        "#a93226", "#f9e9e7", "#00796b", "#e2f3ef", "#b7791f", "#fff2d8", "#f8f5f4"
    ),
    "localization": (
        "#0b6e99", "#e1f1f7", "#3f7d44", "#e8f3e7", "#c46d17", "#fff0df", "#f4f8f9"
    ),
    "map": (
        "#4f772d", "#eaf3df", "#16817a", "#e1f3f1", "#b36b00", "#fff1d9", "#f5f8f3"
    ),
    "perception": (
        "#a23b72", "#f8e8f1", "#087f8c", "#e1f3f5", "#d97706", "#fff1d6", "#f8f5f7"
    ),
    "planning": (
        "#275dad", "#e7eef9", "#00897b", "#e1f4f1", "#d56a00", "#fff0dc", "#f4f7fa"
    ),
    "platform": (
        "#37474f", "#e8edef", "#0e7490", "#e1f1f5", "#c88a00", "#fff3d7", "#f5f7f7"
    ),
    "sensing": (
        "#007c91", "#e0f2f5", "#2f855a", "#e6f3eb", "#c05621", "#fcecdf", "#f4f8f8"
    ),
    "sensor-kit": (
        "#2b6e4f", "#e4f2e9", "#2468a2", "#e5eff8", "#a56a00", "#fff1d8", "#f5f8f6"
    ),
    "system": (
        "#9a5b00", "#fff0d8", "#2f6f8f", "#e4f0f5", "#8c3d62", "#f6e7ee", "#f8f6f3"
    ),
    "ui": (
        "#206a44", "#e4f1e8", "#2878b5", "#e5f0f8", "#c56c15", "#fff0df", "#f5f8f6"
    ),
    "voice": (
        "#126e75", "#e1f1f2", "#a44569", "#f6e7ed", "#b8790c", "#fff2d9", "#f6f8f7"
    ),
}


def module_theme(module: str | None) -> dict:
    """Return one complete semantic palette for a package visual."""
    theme = dict(DEFAULT_THEME)
    if module in MODULE_THEMES:
        (
            theme["primary"],
            theme["primary_bg"],
            theme["secondary"],
            theme["secondary_bg"],
            theme["accent"],
            theme["accent_bg"],
            theme["bg"],
        ) = MODULE_THEMES[module]
    return theme


def themed_color(axis, color: str) -> str:
    """Resolve shared semantic colors through the current package palette."""
    theme = getattr(axis, "_camrod_theme", DEFAULT_THEME)
    semantic = {
        BG: "bg",
        INK: "ink",
        MUTED: "muted",
        WHITE: "surface",
        "#9aaab2": "edge",
        "#71828b": "edge",
        BLUE: "primary",
        BLUE_BG: "primary_bg",
        GREEN: "secondary",
        GREEN_BG: "secondary_bg",
        AMBER: "accent",
        AMBER_BG: "accent_bg",
    }
    return theme.get(semantic.get(color, ""), color)


def load_yaml(path: Path) -> dict:
    """Load one YAML source file."""
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def load_json(path: Path) -> dict:
    """Load one JSON evidence file."""
    return json.loads(path.read_text(encoding="utf-8"))


def ros_params(path: Path, node_name: str) -> dict:
    """Return a ROS parameter mapping from a package config."""
    return load_yaml(path)[node_name]["ros__parameters"]


def setup_figure(title: str, subtitle: str, size=(16, 9), module: str | None = None):
    """Create a package-themed documentation canvas."""
    theme = module_theme(module)
    figure = plt.figure(figsize=size, facecolor=theme["bg"])
    figure._camrod_theme = theme
    axis = figure.add_axes((0, 0, 1, 1))
    axis._camrod_theme = theme
    axis.set_xlim(0, 1)
    axis.set_ylim(0, 1)
    axis.axis("off")
    # Two-color header rails make package families recognizable at a glance.
    axis.add_patch(Rectangle((0.0, 0.986), 0.76, 0.014, color=theme["primary"], zorder=10))
    axis.add_patch(Rectangle((0.76, 0.986), 0.24, 0.014, color=theme["secondary"], zorder=10))
    figure.text(0.045, 0.95, title, color=theme["ink"], fontsize=22, fontweight="bold", va="top")
    figure.text(0.045, 0.905, subtitle, color=theme["muted"], fontsize=10.5, va="top")
    return figure, axis


def draw_box(
    axis,
    x,
    y,
    width,
    height,
    title,
    lines=(),
    face=WHITE,
    edge="#9aaab2",
    title_color=INK,
    linewidth=1.25,
    title_size=10.5,
    body_size=8.4,
):
    """Draw one fixed-size architecture or state box."""
    face = themed_color(axis, face)
    edge = themed_color(axis, edge)
    title_color = themed_color(axis, title_color)
    body_color = themed_color(axis, MUTED)
    patch = FancyBboxPatch(
        (x, y),
        width,
        height,
        boxstyle="round,pad=0.008,rounding_size=0.008",
        linewidth=linewidth,
        edgecolor=edge,
        facecolor=face,
        zorder=2,
    )
    axis.add_patch(patch)
    axis.text(
        x + 0.012,
        y + height - 0.018,
        title,
        color=title_color,
        fontsize=title_size,
        fontweight="bold",
        va="top",
        zorder=3,
    )
    if lines:
        axis.text(
            x + 0.012,
            y + height - 0.052,
            "\n".join(lines),
            color=body_color,
            fontsize=body_size,
            linespacing=1.28,
            va="top",
            zorder=3,
        )
    return patch


def draw_arrow(axis, start, end, color="#71828b", width=1.5, style="-|>"):
    """Draw a stable connector between boxes."""
    color = themed_color(axis, color)
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle=style,
        mutation_scale=12,
        linewidth=width,
        color=color,
        shrinkA=2,
        shrinkB=2,
        zorder=1,
    )
    axis.add_patch(arrow)
    return arrow


def section_label(axis, x, y, text, color=BLUE):
    """Add a compact section label."""
    color = themed_color(axis, color)
    axis.text(x, y, text.upper(), color=color, fontsize=8.5, fontweight="bold", va="bottom")


def footer(figure, text: str):
    """Add source/evidence classification to the canvas."""
    theme = getattr(figure, "_camrod_theme", DEFAULT_THEME)
    figure.text(0.045, 0.028, text, color=theme["muted"], fontsize=8.3, va="bottom")


def save_figure(figure, path: Path):
    """Write a high-resolution PNG and close its canvas."""
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=150, facecolor=figure.get_facecolor())
    plt.close(figure)


def parse_message_constants(path: Path) -> list[str]:
    """Read ordered integer constants from a ROS message definition."""
    pattern = re.compile(r"^int32\s+([A-Z][A-Z0-9_]+)=\d+$")
    values = []
    for line in path.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line.strip())
        if match:
            values.append(match.group(1))
    return values


def render_bringup_contract(repo_root: Path, output_root: Path):
    """Render full-stack startup and expected mission lifecycle."""
    states = parse_message_constants(
        repo_root / "camrod_common" / "avg_msgs" / "msg" / "AvgServiceState.msg"
    )
    figure, axis = setup_figure(
        "CAMROD full bringup and mission contract",
        "Launch ownership, service lifecycle, battery admission, parking, and charge feedback",
        module="bringup",
    )

    section_label(axis, 0.045, 0.85, "Dependency-ordered startup")
    packages = [
        "platform",
        "sensor_kit",
        "map",
        "sensing",
        "perception",
        "localization",
        "planning",
        "control",
        "system + UI",
    ]
    start_x = 0.045
    gap = 0.008
    width = (0.91 - gap * (len(packages) - 1)) / len(packages)
    for index, package in enumerate(packages):
        x = start_x + index * (width + gap)
        draw_box(
            axis,
            x,
            0.775,
            width,
            0.058,
            package,
            face=BLUE_BG if index < 7 else GREEN_BG,
            edge=BLUE if index < 7 else GREEN,
            title_size=8.2,
        )
        if index:
            draw_arrow(axis, (x - gap + 0.001, 0.804), (x - 0.002, 0.804), width=1.0)

    section_label(axis, 0.045, 0.71, "Expected service lifecycle")
    lifecycle = [
        "DROP_ZONE_WAIT",
        "DEPARTING_DROP_ZONE",
        "MOVING_TO_SITE",
        "SITE_ENTRY",
        "WAITING_FOR_RETURN_REQUEST",
        "RETURN_WITH_CARGO",
        "RETURNING_TO_DROP_ZONE",
        "DROP_ZONE_PARKING",
        "WAITING_FOR_CHARGING",
        "CHARGING",
    ]
    descriptions = {
        "DROP_ZONE_WAIT": "parked; no CAN charge",
        "DEPARTING_DROP_ZONE": "bounded exit phases",
        "MOVING_TO_SITE": "Nav2 route active",
        "SITE_ENTRY": "site maneuver owns cmd",
        "WAITING_FOR_RETURN_REQUEST": "operator unload/return",
        "RETURN_WITH_CARGO": "site exit in progress",
        "RETURNING_TO_DROP_ZONE": "Nav2 return route",
        "DROP_ZONE_PARKING": "reverse or AprilTag",
        "WAITING_FOR_CHARGING": "parked, contact pending",
        "CHARGING": "CAN charging feedback",
    }
    columns = 5
    box_width = 0.166
    box_height = 0.105
    for index, state in enumerate(lifecycle):
        row = index // columns
        col = index % columns
        if row == 1:
            col = columns - 1 - col
        x = 0.045 + col * 0.184
        y = 0.56 if row == 0 else 0.385
        color = GREEN if state in {"DROP_ZONE_WAIT", "CHARGING"} else BLUE
        face = GREEN_BG if color == GREEN else BLUE_BG
        draw_box(
            axis,
            x,
            y,
            box_width,
            box_height,
            state,
            (descriptions[state],),
            face=face,
            edge=color,
            title_color=color,
            title_size=8.5,
            body_size=7.2,
        )
        if index and index != columns:
            previous_row = (index - 1) // columns
            if previous_row == row:
                if row == 0:
                    draw_arrow(axis, (x - 0.018, y + 0.052), (x - 0.003, y + 0.052))
                else:
                    draw_arrow(axis, (x + box_width + 0.003, y + 0.052), (x + box_width + 0.018, y + 0.052))
        if index == columns:
            draw_arrow(axis, (0.045 + 4 * 0.184 + 0.083, 0.555), (0.045 + 4 * 0.184 + 0.083, 0.5))

    section_label(axis, 0.045, 0.31, "Admission and safety policy")
    draw_box(
        axis,
        0.045,
        0.13,
        0.275,
        0.155,
        "Battery mission gate",
        (
            ">= 35%: new mission admitted",
            "< 35%: finish current mission; no new departure",
            "<= 20%: hard safety stop",
        ),
        face=AMBER_BG,
        edge=AMBER,
        title_color=AMBER,
    )
    draw_box(
        axis,
        0.345,
        0.13,
        0.275,
        0.155,
        "Parking and public state",
        (
            "controller phase: PARKED",
            "no charge CAN: DROP_ZONE_WAIT",
            "contact pending: WAITING_FOR_CHARGING",
            "charge CAN true: CHARGING",
        ),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
    )
    draw_box(
        axis,
        0.645,
        0.13,
        0.31,
        0.155,
        "Observed-state surfaces",
        (
            "/service/state -> robot UI + guest UI",
            "manual engage -> planning RUNNING display",
            "/system/status is health, not mission state",
            f"message contract: {len(states)} public states",
        ),
        face=GRAY_BG,
        edge="#72848d",
    )
    footer(
        figure,
        "SOURCE-DERIVED CONTRACT. The adjacent simulation-evidence image is the runtime verdict; this diagram is not a PASS claim.",
    )
    save_figure(figure, output_root / "bringup" / "full-stack-mission-contract.png")
    render_lifecycle_gif(lifecycle, descriptions, output_root / "bringup" / "mission-lifecycle-contract.gif")


def render_lifecycle_gif(states: list[str], descriptions: dict, output: Path):
    """Animate expected state order without presenting it as runtime footage."""
    frames = []
    for active_index in range(len(states)):
        figure, axis = setup_figure(
            "Expected service-state progression",
            "CONTRACT ANIMATION - not runtime recording",
            size=(12, 4.8),
            module="bringup",
        )
        for index, state in enumerate(states):
            row = index // 5
            col = index % 5 if row == 0 else 4 - (index % 5)
            x = 0.045 + col * 0.19
            y = 0.57 if row == 0 else 0.24
            active = index == active_index
            draw_box(
                axis,
                x,
                y,
                0.165,
                0.16,
                state,
                (descriptions[state],),
                face=GREEN_BG if active else WHITE,
                edge=GREEN if active else "#a9b6bc",
                title_color=GREEN if active else INK,
                linewidth=2.5 if active else 1.0,
                title_size=8.5,
                body_size=7.0,
            )
        footer(figure, f"Contract step {active_index + 1}/{len(states)}. Runtime PASS must come from sim_validation_runner evidence.")
        buffer = BytesIO()
        figure.savefig(buffer, format="png", dpi=100, facecolor=figure.get_facecolor())
        plt.close(figure)
        buffer.seek(0)
        frames.append(Image.open(buffer).convert("P", palette=Image.Palette.ADAPTIVE))
    output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=900,
        loop=0,
        optimize=True,
    )


def render_bringup_evidence(repo_root: Path, report_path: Path, output_root: Path):
    """Render the actual campsite smoke result, including failures."""
    report = load_json(report_path)
    figure, axis = setup_figure(
        "Full bringup simulation evidence - 2026-08-04",
        f"Historical baseline {report['baseline_commit']} | reverse parking | "
        "current policy documented separately",
        module="bringup",
    )

    section_label(axis, 0.045, 0.84, "Validated prerequisites")
    draw_box(
        axis,
        0.045,
        0.68,
        0.27,
        0.135,
        "STACK STARTUP: PASS",
        (
            f"{report['stack_startup']['runtime_node_count']} ROS nodes observed",
            "[SYSTEM] OK / all modules healthy",
            "sim platform status enabled",
        ),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
    )
    draw_box(
        axis,
        0.34,
        0.68,
        0.27,
        0.135,
        "POSE CHAIN: PASS",
        ("30 s probe status OK", "GNSS/IMU/wheel inputs present", "selected pose measured at 20 Hz"),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
    )
    draw_box(
        axis,
        0.635,
        0.68,
        0.32,
        0.135,
        "ROUND TRIP: NOT DEMONSTRATED",
        ("site entry stops before unload", "return/parking/charging not reached", "do not use the contract GIF as PASS evidence"),
        face=RED_BG,
        edge=RED,
        title_color=RED,
    )

    section_label(axis, 0.045, 0.62, "Observed campsite cases")
    for row, case in enumerate(report["cases"]):
        y = 0.43 - row * 0.18
        draw_box(
            axis,
            0.045,
            y,
            0.15,
            0.12,
            f"{case['mission_key']} / {case['service_mode']}",
            (f"cmd y={case['command']['linear_y_mps']:+.2f} m/s",),
            face=BLUE_BG,
            edge=BLUE,
            title_color=BLUE,
            title_size=8.8,
            body_size=7.4,
        )
        event_x = [0.23, 0.46, 0.69]
        colors = [BLUE, AMBER, RED]
        faces = [BLUE_BG, AMBER_BG, RED_BG]
        for index, event in enumerate(case["events"]):
            draw_box(
                axis,
                event_x[index],
                y,
                0.195,
                0.12,
                event["event"].upper(),
                ((event.get("reason") or f"t={event['stamp']:.3f}"),),
                face=faces[index],
                edge=colors[index],
                title_color=colors[index],
                title_size=8.2,
                body_size=7.2,
            )
            if index == 0:
                draw_arrow(axis, (0.198, y + 0.06), (0.225, y + 0.06))
            else:
                draw_arrow(axis, (event_x[index - 1] + 0.198, y + 0.06), (event_x[index] - 0.005, y + 0.06))
        axis.text(0.915, y + 0.06, case["verdict"], color=RED, fontsize=10, fontweight="bold", va="center")

    experiment = report["rejected_experiment"]
    draw_box(
        axis,
        0.045,
        0.075,
        0.43,
        0.13,
        "REJECTED EXPERIMENT",
        (
            "Existing campsite Areas were rasterized temporarily",
            f"B6 costs C/FL/FR/RR/RL: {experiment['sampled_costs']['center']}/100/100/0/100",
            "Still blocked; source change was reverted",
        ),
        face=AMBER_BG,
        edge=AMBER,
        title_color=AMBER,
    )
    draw_box(
        axis,
        0.5,
        0.075,
        0.455,
        0.13,
        "REQUIRED MAP EVIDENCE",
        (
            "Survey explicit service_access polygons",
            "Join each road lanelet to its maneuver Area",
            "At capture: rerun B6 + B12 with then-active 0.05 m margin",
        ),
        face=GRAY_BG,
        edge="#72848d",
    )
    try:
        evidence_label = report_path.relative_to(repo_root).as_posix()
    except ValueError:
        evidence_label = report_path.name
    footer(
        figure,
        f"RUNTIME EVIDENCE: {evidence_label} | Failure is retained as a release-blocking result, not hidden.",
    )
    save_figure(figure, output_root / "bringup" / "simulation-evidence-20260804.png")


def render_field_stationary_report(report_path: Path, output_root: Path):
    """Render the preserved field report without implying committed raw logs."""
    report = load_json(report_path)
    radar = report["radar_disabled"]
    front = report["front_camera_yolo"]
    rear = report["rear_camera"]
    gnss = report["gnss_imu"]
    localization = report["localization"]
    resource = report["resource_profile"]

    figure, axis = setup_figure(
        "Physical stationary field report - 2026-07-31",
        "Robot hardware connected; physical E-stop held; no motion commanded; raw files remain on the Jetson and are not committed",
        module="bringup",
    )
    section_label(axis, 0.045, 0.84, "Completed stationary checks")
    draw_box(
        axis,
        0.045,
        0.60,
        0.28,
        0.20,
        "RADAR DISABLED: PASS",
        (
            f"{radar['duration_s']:.3f} s / {radar['messages_per_channel']} messages per channel",
            f"{radar['grid_messages']} grids; active evidence {radar['active_evidence']}",
            f"cost>=85 cells {radar['high_cost_cells']}; stop events {radar['cost_stop_events']}",
        ),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
    )
    draw_box(
        axis,
        0.36,
        0.60,
        0.28,
        0.20,
        "FRONT CAMERA + YOLO: PASS",
        (
            f"{front['duration_s']:.0f} s / {front['frames']} frames / {front['rate_hz']:.3f} Hz",
            f"JPEG decode {front['decode_success']}/{front['frames']}; failures {front['decode_failure']}",
            f"detections {front['detection_rate_hz']:.3f} Hz; crash/restart {front['crash_or_restart_count']}",
        ),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
    )
    draw_box(
        axis,
        0.675,
        0.60,
        0.28,
        0.20,
        "REAR CAMERA RATE: FAIL",
        (
            f"{rear['duration_s']:.3f} s / raw {rear['raw_rate_hz']:.3f} Hz",
            f"target {rear['target_raw_rate_hz']:.0f} Hz / max gap {rear['raw_max_gap_s']:.3f} s",
            f"monitoring JPEG {rear['compressed_rate_hz']:.3f} Hz",
        ),
        face=RED_BG,
        edge=RED,
        title_color=RED,
    )

    section_label(axis, 0.045, 0.53, "Partial measurements and bottleneck")
    draw_box(
        axis,
        0.045,
        0.28,
        0.28,
        0.20,
        "GNSS + IMU: PARTIAL",
        (
            f"{gnss['duration_s']:.3f} s / NAV-PVT {gnss['nav_pvt_rate_hz']:.3f} Hz",
            f"RTK Fixed samples {gnss['rtk_fixed_samples']}",
            f"IMU {gnss['imu_rate_hz']:.1f} Hz",
        ),
        face=AMBER_BG,
        edge=AMBER,
        title_color=AMBER,
    )
    draw_box(
        axis,
        0.36,
        0.28,
        0.28,
        0.20,
        "LOCALIZATION: RATE PASS",
        (
            f"{localization['duration_s']:.0f} s / final {localization['final_pose_rate_hz']:.2f} Hz",
            f"selected-pose age p95 {localization['selected_pose_age_p95_ms']:.1f} ms",
            f"GNSS-final p95 {localization['gnss_to_final_xy_p95_m']:.3f} m / {localization['gnss_to_final_yaw_p95_deg']:.2f} deg",
        ),
        face=AMBER_BG,
        edge=AMBER,
        title_color=AMBER,
    )
    draw_box(
        axis,
        0.675,
        0.28,
        0.28,
        0.20,
        "JETSON CPU: SATURATED",
        (
            f"{resource['duration_s']:.0f} s / CPU {resource['cpu_average_percent']:.2f}%",
            f"GPU {resource['gpu_average_percent']:.2f}% / RAM {resource['ram_used_gb']:.2f}/{resource['ram_total_gb']:.2f} GB",
            f"CPU temperature {resource['cpu_temperature_c']:.1f} C",
        ),
        face=RED_BG,
        edge=RED,
        title_color=RED,
    )
    draw_box(
        axis,
        0.045,
        0.075,
        0.91,
        0.12,
        "Evidence boundary",
        (
            f"Normalized from {report['source_report']} at baseline {report['baseline_commit']}; raw_files_committed={str(report['raw_files_committed']).lower()}.",
            "Pass labels apply only to stationary transport/lifetime checks. No driving, boundary recovery, steering, or detection-accuracy PASS was performed.",
        ),
        face=GRAY_BG,
        edge="#72848d",
        body_size=7.8,
    )
    footer(figure, "FIELD REPORT / RAW LOG EXTERNAL. Re-run and commit raw evidence before release-grade physical performance claims.")
    save_figure(
        figure,
        output_root / "bringup" / "field-stationary-report-20260731.png",
    )


def render_localization(repo_root: Path, report_path: Path, output_root: Path):
    """Render pose generation inputs and measured sim output timing."""
    report = load_json(report_path)
    field = ros_params(repo_root / "camrod_localization" / "config" / "filter" / "ekf.yaml", "/**")
    simulation = ros_params(repo_root / "camrod_localization" / "config" / "filter" / "ekf_sim.yaml", "/**")
    topics = {entry["name"]: entry for entry in report["topics"]}

    figure, axis = setup_figure(
        "Localization pose generation and measured timing",
        "GNSS absolute XY + dual-GNSS heading + IMU/wheel yaw rate -> EKF prediction -> selected robot pose",
        module="localization",
    )
    section_label(axis, 0.045, 0.84, "Pose generation chain")
    inputs = [
        ("GNSS position", "/sensing/gnss/pose...", "absolute XY; planar Z is clamped"),
        ("Dual-GNSS yaw", "same pose topic", "used only with valid heading covariance"),
        ("IMU", "/sensing/imu/data_ros", "yaw rate; planar mode clamps roll/pitch rates"),
        ("Wheel odometry", "/localization/input/wheel...", "vx, vy, yaw rate"),
    ]
    for index, (title, topic, detail) in enumerate(inputs):
        y = 0.70 - index * 0.105
        draw_box(axis, 0.045, y, 0.23, 0.08, title, (topic, detail), face=BLUE_BG, edge=BLUE, title_color=BLUE, body_size=7.3)
        draw_arrow(axis, (0.278, y + 0.04), (0.34, 0.615), color=BLUE)
    draw_box(
        axis,
        0.35,
        0.53,
        0.21,
        0.18,
        "robot_localization EKF",
        (
            f"field profile: {field['frequency']:.0f} Hz",
            f"sim profile: {simulation['frequency']:.0f} Hz",
            "2D mode + predict_to_current_time",
            "smooth lagged measurements",
            "base: robot_center_link",
        ),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
    )
    draw_arrow(axis, (0.563, 0.62), (0.61, 0.62), color=GREEN)
    draw_box(
        axis,
        0.62,
        0.56,
        0.15,
        0.12,
        "Odometry mirror",
        ("EKF Odometry", "-> map-frame pose"),
        face=GRAY_BG,
        edge="#72848d",
        title_size=9.2,
    )
    draw_arrow(axis, (0.773, 0.62), (0.81, 0.62), color=GREEN)
    draw_box(
        axis,
        0.82,
        0.56,
        0.135,
        0.12,
        "Pose selector",
        ("primary-only default", "/localization/pose_ros"),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
        title_size=9.0,
        body_size=7.2,
    )

    section_label(axis, 0.045, 0.355, "30-second stationary simulation measurement")
    names = ["gnss_pose", "imu", "wheel_odom", "ekf_odom", "selected_pose"]
    labels = ["GNSS pose", "IMU", "Wheel odom", "EKF odom", "Selected pose"]
    rates = [topics[name]["rate_hz"] for name in names]
    ages = [topics[name]["header_age_ms"]["p95"] for name in names]
    rate_axis = figure.add_axes((0.065, 0.09, 0.41, 0.23), facecolor=WHITE)
    age_axis = figure.add_axes((0.535, 0.09, 0.41, 0.23), facecolor=WHITE)
    y_positions = np.arange(len(labels))
    rate_axis.barh(y_positions, rates, color=[BLUE, BLUE, BLUE, GREEN, GREEN])
    rate_axis.set_yticks(y_positions, labels, fontsize=8)
    rate_axis.invert_yaxis()
    rate_axis.set_xlabel("measured rate [Hz]", fontsize=8)
    rate_axis.set_xlim(0, max(rates) * 1.12)
    rate_axis.set_title("Input cadence vs generated pose", loc="left", fontsize=10, fontweight="bold", color=INK)
    rate_axis.grid(axis="x", alpha=0.2)
    for index, value in enumerate(rates):
        rate_axis.text(value + 0.3, index, f"{value:.1f}", va="center", fontsize=8, color=INK)
    age_axis.barh(y_positions, ages, color=[BLUE, BLUE, BLUE, GREEN, GREEN])
    age_axis.set_yticks(y_positions, labels, fontsize=8)
    age_axis.invert_yaxis()
    age_axis.set_xlabel("header age p95 [ms]", fontsize=8)
    age_axis.set_xlim(0, max(ages) * 1.12)
    age_axis.set_title("Message freshness", loc="left", fontsize=10, fontweight="bold", color=INK)
    age_axis.grid(axis="x", alpha=0.2)
    for index, value in enumerate(ages):
        age_axis.text(value + 0.04, index, f"{value:.2f}", va="center", fontsize=8, color=INK)
    footer(
        figure,
        "MEASURED SIM EVIDENCE: output cadence doubled from 10 Hz inputs to 20 Hz selected pose. Stationary noiseless sim does not prove field accuracy or jitter reduction.",
    )
    save_figure(figure, output_root / "localization" / "pose-generation-and-timing.png")


def render_planning(repo_root: Path, output_root: Path):
    """Render Nav2 servers, selected plugins, cost sources, and state ownership."""
    nav2 = load_yaml(repo_root / "camrod_planning" / "config" / "nav2_base.yaml")
    planner = nav2["planner_server"]["ros__parameters"]
    controller = nav2["controller_server"]["ros__parameters"]
    production_planners = load_yaml(
        repo_root
        / "camrod_planning"
        / "config"
        / "nav2_planner_profiles"
        / "production.yaml"
    )["planner_server"]["ros__parameters"]["planner_plugins"]
    # HH_260805 - Render the instantiated production load set, not every
    # controller definition retained in the base development configuration.
    production_controllers = load_yaml(
        repo_root
        / "camrod_planning"
        / "config"
        / "nav2_controller_profiles"
        / "production.yaml"
    )["controller_server"]["ros__parameters"]["controller_plugins"]
    obstacle = ros_params(
        repo_root / "camrod_planning" / "config" / "obstacle_replan_monitor.yaml",
        "/planning/obstacle_replan_monitor",
    )
    defaults = load_yaml(
        repo_root / "camrod_bringup" / "config" / "bringup" / "launch_defaults.yaml"
    )
    planning_defaults = defaults["bringup"]["planning"]

    figure, axis = setup_figure(
        "Planning runtime: Nav2 servers, selectors, and mission states",
        "Default full bringup selects LaneletRoute + RPP; grid and sampling plugins remain fallback/selectable",
        module="planning",
    )
    section_label(axis, 0.045, 0.84, "Route-to-command execution")
    chain = [
        (0.045, "UI / RViz goal", ("mission key + site goal", "manual goals remain visible"), BLUE),
        (0.225, "Goal snapper", ("lanelet component", "source-aware release"), BLUE),
        (
            0.405,
            "Planner server",
            tuple(f"loaded: {name}" for name in production_planners)
            + (
                "opt-in profile: "
                f"{len(planner['planner_plugins']) - len(production_planners)} more",
            ),
            GREEN,
        ),
        (0.625, "BT + smoother", ("PlannerSelector", "ControllerSelector", "recovery behaviors"), BLUE),
        (
            0.805,
            "Controller server",
            tuple(f"loaded: {name}" for name in production_controllers)
            + (
                "opt-in profile: "
                f"{len(controller['controller_plugins']) - len(production_controllers)} more",
            ),
            GREEN,
        ),
    ]
    widths = [0.15, 0.15, 0.19, 0.15, 0.15]
    for index, ((x, title, lines, color), width) in enumerate(zip(chain, widths)):
        draw_box(
            axis,
            x,
            0.65,
            width,
            0.16,
            title,
            lines,
            face=GREEN_BG if color == GREEN else BLUE_BG,
            edge=color,
            title_color=color,
            title_size=9.3,
            body_size=7.0,
        )
        if index:
            previous_x = chain[index - 1][0]
            previous_width = widths[index - 1]
            draw_arrow(axis, (previous_x + previous_width + 0.004, 0.73), (x - 0.005, 0.73))

    draw_box(
        axis,
        0.045,
        0.49,
        0.435,
        0.11,
        "Selector policy",
        (
            f"full bringup: {planning_defaults['nav2_selected_planner']} + {planning_defaults['nav2_selected_controller']}",
            "Nav2 core: planner + controller scoped container",
            f"obstacle >= {obstacle['block_hold_s']:.0f} s: SmacLattice; restore LaneletRoute",
        ),
        face=GRAY_BG,
        edge="#72848d",
        body_size=7.7,
    )
    draw_box(
        axis,
        0.505,
        0.49,
        0.45,
        0.11,
        "Cost and authorization inputs",
        (
            "global costmap: lanelet + lidar + radar + inflation",
            "controller 15 Hz; RPP 0.4 m/s; lookahead 1.1..2.0 m",
            "cmd_vel safety gate is final motion authority",
        ),
        face=AMBER_BG,
        edge=AMBER,
        title_color=AMBER,
        body_size=7.7,
    )

    section_label(axis, 0.045, 0.43, "Mission and control ownership")
    stages = [
        ("WAIT_DZ", "drop-zone wait"),
        ("RUNNING", "Nav2 owns route"),
        ("GOAL_REACHED", "arrival handoff"),
        ("SITE MANEUVER", "control owns raw cmd"),
        ("RETURNING", "Nav2 return route"),
        ("PARKING", "selected parking node"),
        ("CHARGING", "CAN feedback state"),
    ]
    box_width = 0.117
    for index, (title, detail) in enumerate(stages):
        x = 0.045 + index * 0.132
        color = GREEN if title in {"WAIT_DZ", "CHARGING"} else BLUE
        draw_box(
            axis,
            x,
            0.25,
            box_width,
            0.12,
            title,
            (detail,),
            face=GREEN_BG if color == GREEN else BLUE_BG,
            edge=color,
            title_color=color,
            title_size=8.5,
            body_size=7.0,
        )
        if index:
            draw_arrow(axis, (x - 0.014, 0.31), (x - 0.003, 0.31))
    draw_box(
        axis,
        0.045,
        0.075,
        0.91,
        0.11,
        "Health and mission state are separate",
        (
            "/planning/state describes planner lifecycle; /service/state describes user-visible operation; /system/status reports health severity.",
            "An expected Nav2 cancel during SITE_ENTRY must not remain a stale planning warning; ERROR_STOP and OPERATOR_STOPPED are explicit terminal states.",
        ),
        face=WHITE,
        edge="#9aaab2",
        body_size=8.0,
    )
    footer(
        figure,
        "SOURCE-DERIVED: Nav2 base + production planner/controller profiles, "
        "bringup defaults, and planning/UI state contracts.",
    )
    save_figure(figure, output_root / "planning" / "nav2-servers-and-mission-states.png")


def render_perception(repo_root: Path, output_root: Path):
    """Render LiDAR, YOLO fusion, occupancy, and AprilTag paths."""
    params = load_yaml(repo_root / "camrod_perception" / "config" / "perception_params.yaml")
    fusion = params["/perception/obstacle_fusion"]["ros__parameters"]
    lidar = params["/perception/obstacle_lidar"]["ros__parameters"]
    yolo = params["/perception/yolov9mit"]["ros__parameters"]
    occupancy = params["/perception/campsite_occupancy"]["ros__parameters"]
    tag = ros_params(
        repo_root / "camrod_perception" / "config" / "apriltag_parking_detector.yaml",
        "/perception/apriltag_parking_detector",
    )

    figure, axis = setup_figure(
        "Perception pipelines and observable outputs",
        "Simulation uses LiDAR-only mode; camera, TensorRT YOLO, fusion, and AprilTag evidence must be captured on Jetson",
        module="perception",
    )
    section_label(axis, 0.045, 0.84, "Camera + LiDAR field mode")
    field_boxes = [
        (0.045, "Front camera", ("rectified image", yolo["input_image_topic"]), BLUE),
        (0.23, "YOLOv9-MIT", (f"{yolo['model_type']} @ {yolo['throttle_fps']:.0f} fps", f"conf/IoU {yolo['min_confidence']:.2f}/{yolo['min_iou']:.2f}"), AMBER),
        (0.415, "Detection2D", (yolo["output_boundingbox_topic"], "COCO labels"), BLUE),
        (0.60, "Camera-LiDAR fusion", (fusion["fusion_filter_mode"], "+ filtered point cloud"), GREEN),
        (0.805, "Fused outputs", (fusion["output_topic"], fusion["out_det3d_topic"]), GREEN),
    ]
    widths = [0.155, 0.155, 0.155, 0.175, 0.15]
    for index, ((x, title, lines, color), width) in enumerate(zip(field_boxes, widths)):
        draw_box(axis, x, 0.68, width, 0.13, title, lines, face=AMBER_BG if color == AMBER else GREEN_BG if color == GREEN else BLUE_BG, edge=color, title_color=color, title_size=8.8, body_size=6.9)
        if index:
            previous_x = field_boxes[index - 1][0]
            draw_arrow(axis, (previous_x + widths[index - 1] + 0.003, 0.745), (x - 0.004, 0.745))

    section_label(axis, 0.045, 0.62, "LiDAR-only path - active in simulation")
    draw_box(axis, 0.045, 0.48, 0.205, 0.11, "Filtered LiDAR", ("/sensing/lidar/points_filtered",), face=BLUE_BG, edge=BLUE, title_color=BLUE)
    draw_arrow(axis, (0.253, 0.535), (0.29, 0.535))
    draw_box(
        axis,
        0.30,
        0.48,
        0.27,
        0.11,
        "Euclidean clustering + boxes",
        (f"tolerance {lidar['cluster_tolerance']:.2f} m", f"points {lidar['min_cluster_size']}..{lidar['max_cluster_size']}"),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
    )
    draw_arrow(axis, (0.573, 0.535), (0.61, 0.535))
    draw_box(axis, 0.62, 0.48, 0.335, 0.11, "Obstacle outputs", ("/perception/obstacles", "/perception/lidar/bboxes"), face=GREEN_BG, edge=GREEN, title_color=GREEN)

    section_label(axis, 0.045, 0.42, "Semantic occupancy and parking")
    draw_box(
        axis,
        0.045,
        0.24,
        0.42,
        0.14,
        "Campsite occupancy",
        (
            f"Detection3D input; confidence >= {occupancy['minimum_confidence']:.2f}",
            f"confirm {occupancy['confirm_hits']} hits within {occupancy['confirm_window_s']:.0f} s",
            f"occupied hold {occupancy['occupied_hold_s']:.0f} s -> mission-key availability",
        ),
        face=AMBER_BG,
        edge=AMBER,
        title_color=AMBER,
    )
    draw_box(
        axis,
        0.495,
        0.24,
        0.46,
        0.14,
        "Rear-camera AprilTag parking",
        (
            f"{tag['tag_family']} ID {tag['target_tag_id']} / {tag['tag_size']:.2f} m",
            "capture + image_proc + detector: one container",
            "launched only when parking_method:=apriltag",
        ),
        face=BLUE_BG,
        edge=BLUE,
        title_color=BLUE,
    )
    draw_box(
        axis,
        0.045,
        0.075,
        0.91,
        0.105,
        "Evidence boundary",
        (
            "Architecture and thresholds above are source-configured. Current sim publishes no physical camera scene, so no YOLO accuracy, fusion alignment, or AprilTag detection PASS is claimed.",
            "Use camera_payload_probe.py plus topic/RViz captures on the Jetson to replace the field-pending label with measured media.",
        ),
        face=RED_BG,
        edge=RED,
        title_color=RED,
        body_size=7.8,
    )
    footer(figure, "SOURCE-DERIVED + FIELD PENDING. No synthetic detection boxes are presented as real YOLO output.")
    save_figure(figure, output_root / "perception" / "yolo-lidar-and-parking-pipelines.png")


# HH_260804 - Compare the former rear-axle origin with the current axle-midpoint
# origin used consistently by Dual-Ackermann, crab, and zero-turn consumers.
# The diagrams read canonical YAML/evidence and never imply a physical remount.
def sensor_kit_mounts(params: dict) -> list[dict]:
    """Return every configured sensor mount in README display order."""
    mounts = [
        {"name": "IMU", **params["imu"]},
        {"name": "GNSS*", **params["gnss"]},
        {"name": "LiDAR", **params["lidar"]},
        {"name": "Camera front", **params["camera"]["front"]},
        {"name": "Camera rear", **params["camera"]["rear"]},
    ]
    radar_names = {
        "front1": "Radar front 1",
        "front2": "Radar front 2",
        "left1": "Radar left 1",
        "left2": "Radar left 2",
        "right1": "Radar right 1",
        "right2": "Radar right 2",
        "rear": "Radar rear",
    }
    mounts.extend(
        {"name": radar_names[key], **pose}
        for key, pose in params["radar"].items()
    )
    return mounts


def draw_reference_vehicle(axis, robot: dict, rear_axle_origin: bool) -> None:
    """Draw the same 4WS chassis in one of the two coordinate systems."""
    extents = robot["body_extents"]
    offset = robot["center_offset_from_rear_axle"]
    wheelbase = robot["wheelbase"]
    coordinate_shift = offset if rear_axle_origin else 0.0

    body_rear = -extents["rear"] + coordinate_shift
    body_front = extents["front"] + coordinate_shift
    rear_axle = 0.0 if rear_axle_origin else -offset
    midpoint = offset if rear_axle_origin else 0.0
    front_axle = wheelbase if rear_axle_origin else offset
    reference_x = rear_axle if rear_axle_origin else midpoint
    longitudinal_margin = extents["planning_margin"]
    lateral_margin = extents.get("planning_lateral_margin", longitudinal_margin)

    axis.add_patch(
        Rectangle(
            (body_rear - longitudinal_margin, -extents["right"] - lateral_margin),
            body_front - body_rear + 2 * longitudinal_margin,
            extents["left"] + extents["right"] + 2 * lateral_margin,
            fill=False,
            edgecolor=RED,
            linewidth=1.3,
            linestyle="--",
        )
    )
    axis.add_patch(
        Rectangle(
            (body_rear, -extents["right"]),
            body_front - body_rear,
            extents["left"] + extents["right"],
            facecolor=GRAY_BG,
            edgecolor=INK,
            linewidth=1.5,
        )
    )

    # Both panels show the same symmetric 4WS pose; only the origin changes.
    wheel_y = max(extents["left"], extents["right"]) + 0.015
    for axle_x, yaw_deg in ((rear_axle, -16.0), (front_axle, 16.0)):
        yaw = np.deg2rad(yaw_deg)
        dx = 0.10 * np.cos(yaw)
        dy = 0.10 * np.sin(yaw)
        for side in (-1.0, 1.0):
            axis.plot(
                [axle_x - dx, axle_x + dx],
                [side * wheel_y - dy, side * wheel_y + dy],
                color=BLUE,
                linewidth=5.0,
                solid_capstyle="round",
                zorder=5,
            )

    axis.axvline(rear_axle, color="#83939b", linestyle="-.", linewidth=1.0)
    axis.axvline(front_axle, color="#83939b", linestyle="-.", linewidth=1.0)
    axis.scatter(
        [reference_x],
        [0.0],
        marker="x",
        s=110,
        linewidth=3.0,
        color=AMBER if rear_axle_origin else GREEN,
        zorder=7,
    )
    if rear_axle_origin:
        axis.scatter(
            [midpoint],
            [0.0],
            marker="o",
            s=50,
            facecolors=WHITE,
            edgecolors=GREEN,
            linewidth=1.5,
            zorder=6,
        )
    else:
        axis.scatter(
            [rear_axle],
            [0.0],
            marker="o",
            s=50,
            facecolors=WHITE,
            edgecolors=AMBER,
            linewidth=1.5,
            zorder=6,
        )

    labels = (
        ("Rear", rear_axle),
        ("Center", midpoint),
        ("Front", front_axle),
    )
    for name, x_value in labels:
        axis.text(
            x_value,
            -0.70,
            f"{name}\nx={x_value:+.3f}",
            ha="center",
            va="top",
            fontsize=7.0,
            color=MUTED,
            fontweight="bold" if x_value == reference_x else "normal",
        )

    reference_name = "robot_base_link" if rear_axle_origin else "robot_center_link"
    axis.text(
        reference_x,
        0.66,
        f"{reference_name}\nX origin",
        ha="center",
        va="bottom",
        fontsize=8.0,
        color=AMBER if rear_axle_origin else GREEN,
        fontweight="bold",
    )
    axis.annotate(
        "+X forward",
        xy=(body_front + longitudinal_margin + 0.12, 0.0),
        xytext=(body_front + longitudinal_margin - 0.15, 0.0),
        arrowprops={"arrowstyle": "-|>", "color": INK},
        fontsize=7.0,
        color=INK,
        va="center",
    )
    axis.set_xlim(
        body_rear - longitudinal_margin - 0.20,
        body_front + longitudinal_margin + 0.22,
    )
    axis.set_ylim(-0.82, 0.82)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xticks([])
    axis.set_yticks([])
    for spine in axis.spines.values():
        spine.set_color("#aebbc1")


def render_sensor_kit(repo_root: Path, output_root: Path):
    """Render a concise before/after frame comparison and sensor-X ledger."""
    params = ros_params(
        repo_root / "camrod_sensor_kit" / "config" / "robot_params.yaml", "/**"
    )
    evidence = load_json(
        repo_root
        / "docs"
        / "evidence"
        / "v2.1.3"
        / "reference-frame"
        / "rear-axle-vs-robot-center-summary.json"
    )
    robot = params["robot"]
    extents = robot["body_extents"]
    offset = robot["center_offset_from_rear_axle"]
    common = evidence["common_route_segment"]
    rear_result = evidence["rear_axle_reference"]
    center_result = evidence["axle_midpoint_reference"]

    figure, canvas = setup_figure(
        "Reference frame: rear axle to axle midpoint",
        "Same 4WS robot | Dual-Ackermann, crab, and zero-turn use one central reference",
        size=(16, 9),
        module="sensor-kit",
    )
    canvas.text(
        0.5,
        0.855,
        f"X(center) = X(rear axle) - {offset:.3f} m",
        ha="center",
        va="center",
        fontsize=12,
        color=BLUE,
        fontweight="bold",
    )

    previous = figure.add_axes((0.055, 0.40, 0.41, 0.40), facecolor=WHITE)
    current = figure.add_axes((0.535, 0.40, 0.41, 0.40), facecolor=WHITE)
    draw_reference_vehicle(previous, robot, rear_axle_origin=True)
    draw_reference_vehicle(current, robot, rear_axle_origin=False)
    previous.set_title(
        "BEFORE  |  rear-axle origin",
        loc="left",
        fontsize=12,
        color=AMBER,
        fontweight="bold",
    )
    current.set_title(
        "CURRENT  |  axle-midpoint origin",
        loc="left",
        fontsize=12,
        color=GREEN,
        fontweight="bold",
    )

    table_axis = figure.add_axes((0.055, 0.105, 0.89, 0.20))
    table_axis.axis("off")
    rows = (
        ("Runtime origin", "robot_base_link @ rear axle", "robot_center_link @ axle midpoint"),
        (
            "Axle X [m]",
            f"rear 0 | center +{offset:.3f} | front +{robot['wheelbase']:.3f}",
            f"rear -{offset:.3f} | center 0 | front +{offset:.3f}",
        ),
        (
            "Body front / rear [m]",
            f"{extents['front'] + offset:.5f} / {extents['rear'] - offset:.5f}",
            f"{extents['front']:.5f} / {extents['rear']:.5f}",
        ),
        (
            "Planning front / rear [m]",
            f"{extents['front'] + offset + extents['planning_margin']:.5f} / "
            f"{extents['rear'] - offset + extents['planning_margin']:.5f}",
            f"{extents['front'] + extents['planning_margin']:.5f} / "
            f"{extents['rear'] + extents['planning_margin']:.5f}",
        ),
        (
            "Sensor XYZ reference",
            "rear-axle origin",
            "sensor_kit_base_link @ center",
        ),
    )
    table = table_axis.table(
        cellText=rows,
        colLabels=("Value", "Before", "Current"),
        cellLoc="center",
        colLoc="center",
        loc="center",
        colWidths=(0.24, 0.38, 0.38),
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8.2)
    table.scale(1.0, 1.32)
    for (row, column), cell in table.get_celld().items():
        cell.set_edgecolor("#b4c0c6")
        cell.set_linewidth(0.7)
        if row == 0:
            cell.set_facecolor(INK)
            cell.set_text_props(color=WHITE, fontweight="bold")
        elif column == 0:
            cell.set_facecolor(GRAY_BG)
            cell.set_text_props(color=INK, fontweight="bold", ha="left")
        else:
            cell.set_facecolor(WHITE if row % 2 else "#f1f5f6")

    figure.text(
        0.5,
        0.345,
        "SIM A/B  |  "
        f"cross-track RMS {common['rear_axle_reference']['cross_track_rms_m']:.4f} -> "
        f"{common['axle_midpoint_reference']['cross_track_rms_m']:.4f} m  |  "
        f"yaw RMS {common['rear_axle_reference']['yaw_error_rms_deg']:.3f} -> "
        f"{common['axle_midpoint_reference']['yaw_error_rms_deg']:.3f} deg  |  "
        f"progress to hold {rear_result['route_progress_to_hold_m']:.3f} -> "
        f"{center_result['route_progress_to_hold_m']:.3f} m",
        ha="center",
        va="center",
        fontsize=8.5,
        color=BLUE,
        fontweight="bold",
    )
    footer(
        figure,
        "SOURCE CONFIG: current "
        f"{extents['planning_margin']:.2f} m longitudinal / "
        f"{extents['planning_lateral_margin']:.2f} m lateral margin. "
        "SIM A/B: historical v2.1.3 0.10 m all-side margin; both held at the narrow boundary.",
    )
    save_figure(
        figure,
        output_root / "sensor-kit" / "reference-frame-before-after.png",
    )

    mounts = sensor_kit_mounts(params)
    render_sensor_mount_side_view(
        params,
        mounts,
        output_root / "sensor-kit" / "sensor-mount-side-view.png",
    )
    render_sensor_x_before_after(
        offset,
        mounts,
        output_root / "sensor-kit" / "sensor-x-before-after.png",
    )
    render_gnss_lever_arm(
        params,
        output_root / "sensor-kit" / "gnss-left-antenna-lever-arm.png",
    )


def render_sensor_x_before_after(
    offset: float, mounts: list[dict], output: Path
) -> None:
    """Render frame-migrated X values and the measured GNSS lateral offset."""
    figure, axis = setup_figure(
        "Sensor X coordinates: before and current",
        "Frame migration plus measured GNSS Y=+0.45 m | metres",
        size=(14, 8),
        module="sensor-kit",
    )
    axis.text(
        0.5,
        0.835,
        f"mount X = previous X - {offset:.3f} m  |  GNSS mount X=0.000, Y=+0.450 m",
        ha="center",
        va="center",
        fontsize=12,
        color=BLUE,
        fontweight="bold",
    )
    rows = []
    for mount in mounts:
        # HH_260806 - GNSS X remains zero while its measured lateral offset is
        # documented by the dedicated top-view lever-arm figure.
        previous_x = (
            0.0
            if mount["name"].startswith("GNSS")
            else mount["x"] + offset
        )
        rows.append(
            (
                mount["name"],
                f"{previous_x:+.5f}",
                f"{mount['x']:+.5f}",
            )
        )
    table_axis = figure.add_axes((0.10, 0.10, 0.80, 0.66))
    table_axis.axis("off")
    table = table_axis.table(
        cellText=rows,
        colLabels=(
            "Sensor",
            "Before: rear-axle X [m]",
            "Current: center X [m]",
        ),
        cellLoc="center",
        colLoc="center",
        loc="center",
        colWidths=(0.36, 0.32, 0.32),
    )
    table.auto_set_font_size(False)
    table.set_fontsize(9.0)
    table.scale(1.0, 1.42)
    for (row, column), cell in table.get_celld().items():
        cell.set_edgecolor("#aebbc1")
        cell.set_linewidth(0.7)
        if row == 0:
            cell.set_facecolor(INK)
            cell.set_text_props(color=WHITE, fontweight="bold")
        elif column == 0:
            cell.set_text_props(ha="left", color=INK, fontweight="bold")
            cell.set_facecolor(GRAY_BG if row % 2 else WHITE)
        else:
            cell.set_facecolor(WHITE if row % 2 else "#edf2f4")
    footer(
        figure,
        "GNSS* uses X=0.000 m and measured Y=+0.450 m; localization subtracts "
        "the heading-rotated lateral lever arm before publishing robot center.",
    )
    save_figure(figure, output)


def render_sensor_mount_side_view(
    params: dict, mounts: list[dict], output: Path
) -> None:
    """Render physical sensor heights with current and former X scales."""
    robot = params["robot"]
    extents = robot["body_extents"]
    offset = robot["center_offset_from_rear_axle"]
    figure, _ = setup_figure(
        "Physical sensor mount side view",
        "Current configured layout | bottom: center X | top: historical rear-axle scale",
        size=(14, 8),
        module="sensor-kit",
    )
    plot = figure.add_axes((0.08, 0.15, 0.84, 0.64), facecolor=WHITE)
    plot.add_patch(
        Rectangle(
            (-extents["rear"], extents["bottom_z"]),
            extents["front"] + extents["rear"],
            extents["top_z"] - extents["bottom_z"],
            facecolor=GRAY_BG,
            edgecolor=INK,
            linewidth=1.5,
        )
    )
    for axle_x in (-offset, offset):
        plot.axvline(axle_x, color="#7f9098", linewidth=1.0, linestyle="-.")

    styles = {
        "IMU": (GREEN, "D"),
        "GNSS": (AMBER, "P"),
        "LiDAR": (BLUE, "o"),
        "Camera": ("#7251b5", "s"),
        "Radar": (RED, "^"),
    }

    def category(name: str) -> str:
        if name.startswith("Radar"):
            return "Radar"
        if name.startswith("Camera"):
            return "Camera"
        if name.startswith("GNSS"):
            return "GNSS"
        return name

    legend_seen = set()
    for mount in mounts:
        group = category(mount["name"])
        color, marker = styles[group]
        label = group if group not in legend_seen else None
        legend_seen.add(group)
        plot.scatter(
            [mount["x"]],
            [mount["z"]],
            s=52,
            marker=marker,
            color=color,
            edgecolors=WHITE,
            linewidth=0.7,
            label=label,
            zorder=6,
        )

    plot.scatter(
        [0.0], [0.0], marker="x", s=90, linewidth=2.5, color=GREEN, zorder=7
    )
    plot.scatter(
        [-offset],
        [0.0],
        marker="x",
        s=90,
        linewidth=2.5,
        color=AMBER,
        zorder=7,
    )
    plot.text(
        0.02,
        -0.09,
        "robot center",
        fontsize=7.5,
        color=GREEN,
        fontweight="bold",
    )
    plot.text(
        -offset - 0.02,
        -0.09,
        "rear axle",
        fontsize=7.5,
        color=AMBER,
        fontweight="bold",
        ha="right",
    )

    labels = (
        (params["imu"]["x"], params["imu"]["z"], "IMU", GREEN, -0.10, 0.07),
        (
            params["gnss"]["x"],
            params["gnss"]["z"],
            "GNSS*",
            AMBER,
            -0.05,
            0.07,
        ),
        (
            params["lidar"]["x"],
            params["lidar"]["z"],
            "LiDAR",
            BLUE,
            0.03,
            0.08,
        ),
        (
            params["camera"]["front"]["x"],
            params["camera"]["front"]["z"],
            "front camera",
            "#7251b5",
            0.03,
            0.03,
        ),
        (
            params["camera"]["rear"]["x"],
            params["camera"]["rear"]["z"],
            "rear camera",
            "#7251b5",
            -0.03,
            -0.09,
        ),
        (
            params["radar"]["front1"]["x"],
            params["radar"]["front1"]["z"],
            "front radar x2",
            RED,
            -0.05,
            -0.05,
        ),
        (
            params["radar"]["left1"]["x"],
            params["radar"]["left1"]["z"],
            "side radar pair 1",
            RED,
            0.01,
            -0.10,
        ),
        (
            params["radar"]["left2"]["x"],
            params["radar"]["left2"]["z"],
            "side radar pair 2",
            RED,
            -0.01,
            0.07,
        ),
        (
            params["radar"]["rear"]["x"],
            params["radar"]["rear"]["z"],
            "rear radar",
            RED,
            -0.03,
            0.07,
        ),
    )
    for x, z, label, color, dx, dz in labels:
        plot.text(
            x + dx,
            z + dz,
            label,
            fontsize=7.0,
            color=color,
            fontweight="bold",
            ha="right" if dx < 0 else "left",
        )

    plot.annotate(
        "+X FRONT",
        xy=(0.94, 1.01),
        xytext=(0.70, 1.01),
        arrowprops={"arrowstyle": "-|>", "color": INK},
        color=INK,
        fontsize=8.0,
        fontweight="bold",
        va="center",
    )
    plot.set_xlim(-0.95, 1.00)
    plot.set_ylim(-0.23, 1.10)
    plot.set_xlabel("Current robot-center X [m]", fontsize=9)
    plot.set_ylabel("Z up [m]", fontsize=9)
    plot.grid(alpha=0.18)
    plot.tick_params(labelsize=8, colors=MUTED)
    for spine in plot.spines.values():
        spine.set_color("#aebbc1")
    previous_axis = plot.secondary_xaxis(
        "top",
        functions=(lambda x: x + offset, lambda x: x - offset),
    )
    previous_axis.set_xlabel("Previous rear-axle X [m]", fontsize=9)
    previous_axis.tick_params(labelsize=8, colors=MUTED)
    plot.legend(loc="upper left", ncol=5, fontsize=7.0, framealpha=0.92)
    footer(
        figure,
        "SOURCE CONFIG. GNSS* is the left antenna at Y=+0.450 m; X/Z and the "
        "remaining dual-GNSS survey stay pending. Other mounts retain the -0.443 m X conversion.",
    )
    save_figure(figure, output)


def render_gnss_lever_arm(params: dict, output: Path) -> None:
    """Render the measured left antenna and yaw-aware center conversion."""
    robot = params["robot"]
    extents = robot["body_extents"]
    gnss = params["gnss"]
    figure, _ = setup_figure(
        "GNSS left-antenna lever arm",
        "Physical TF and localization position reference | top view | metres",
        size=(14, 8),
        module="sensor-kit",
    )

    plot = figure.add_axes((0.055, 0.12, 0.54, 0.70), facecolor=WHITE)
    planning = Rectangle(
        (
            -extents["rear"] - extents["planning_margin"],
            -extents["right"] - extents["planning_lateral_margin"],
        ),
        extents["front"] + extents["rear"] + 2.0 * extents["planning_margin"],
        extents["left"] + extents["right"] + 2.0 * extents["planning_lateral_margin"],
        facecolor=AMBER_BG,
        edgecolor=AMBER,
        linewidth=2.0,
        label="planning boundary",
    )
    body = Rectangle(
        (-extents["rear"], -extents["right"]),
        extents["front"] + extents["rear"],
        extents["left"] + extents["right"],
        facecolor=BLUE_BG,
        edgecolor=BLUE,
        linewidth=2.2,
        label="active body",
    )
    plot.add_patch(planning)
    plot.add_patch(body)
    plot.scatter([0.0], [0.0], s=95, color=GREEN, zorder=5)
    plot.scatter([gnss["x"]], [gnss["y"]], s=115, color=RED, zorder=6)
    plot.annotate(
        "",
        xy=(gnss["x"], gnss["y"]),
        xytext=(0.0, 0.0),
        arrowprops={"arrowstyle": "-|>", "color": RED, "linewidth": 2.4},
    )
    plot.text(0.025, -0.06, "robot_center_link", color=GREEN, fontsize=9, fontweight="bold")
    plot.text(
        gnss["x"] + 0.025,
        gnss["y"] + 0.075,
        "left antenna\n(0.000, +0.450, 0.000)",
        color=RED,
        fontsize=9,
        fontweight="bold",
    )
    plot.text(
        -0.08,
        gnss["y"] * 0.50,
        "+Y left  0.45 m",
        color=RED,
        fontsize=9,
        rotation=90,
        ha="right",
        va="center",
    )
    plot.annotate(
        "+X forward",
        xy=(0.62, -0.32),
        xytext=(0.20, -0.32),
        arrowprops={"arrowstyle": "-|>", "color": INK},
        color=INK,
        fontsize=8,
        va="center",
    )
    plot.set_xlim(-0.82, 0.86)
    plot.set_ylim(-0.62, 0.68)
    plot.set_aspect("equal", adjustable="box")
    plot.set_xlabel("Robot body X", fontsize=9)
    plot.set_ylabel("Robot body Y (+left)", fontsize=9)
    plot.grid(alpha=0.16)
    plot.legend(loc="lower left", fontsize=8)

    table_axis = figure.add_axes((0.63, 0.20, 0.33, 0.52))
    table_axis.axis("off")
    rows = (
        ("Raw NavSatFix", "left antenna point"),
        ("Required yaw", "fresh dual-GNSS"),
        ("Map correction", "subtract R(yaw)[0, 0.45]"),
        ("Published pose", "robot_center_link"),
        ("Stale heading", "withhold corrected fix"),
        ("Verification", "X/Z + moving survey pending"),
    )
    table = table_axis.table(
        cellText=rows,
        colLabels=("Contract", "Current behavior"),
        cellLoc="left",
        colLoc="left",
        loc="center",
        colWidths=(0.40, 0.60),
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8.2)
    table.scale(1.0, 1.55)
    for (row, column), cell in table.get_celld().items():
        cell.set_edgecolor("#aebbc1")
        if row == 0:
            cell.set_facecolor(INK)
            cell.set_text_props(color=WHITE, fontweight="bold")
        elif column == 0:
            cell.set_facecolor(GRAY_BG)
            cell.set_text_props(color=INK, fontweight="bold")
        else:
            cell.set_facecolor(WHITE if row % 2 else "#edf2f4")
    footer(
        figure,
        "SOURCE CONFIG + UNIT-TESTED TRANSFORM. Lateral Y is measured; full dual-GNSS geometry and moving field residuals remain pending.",
    )
    save_figure(figure, output)


def render_sensing(repo_root: Path, output_root: Path):
    """Render sensor-specific processing and final cost-grid fusion."""
    sensing_defaults = load_yaml(
        repo_root / "camrod_bringup" / "config" / "bringup" / "launch_defaults.yaml"
    )["bringup"]["sensing"]
    lidar = ros_params(
        repo_root / "camrod_sensing" / "config" / "lidar" / "ground_seg_params.yaml",
        "/sensing/lidar/ground_segmentation",
    )
    lidar_cost = ros_params(
        repo_root / "camrod_sensing" / "config" / "lidar" / "cost_grid.yaml",
        "/sensing/lidar/lidar_cost_grid",
    )
    radar = ros_params(
        repo_root / "camrod_sensing" / "config" / "radar" / "cost_grid.yaml",
        "/sensing/radar/radar_cost_grid",
    )
    inflation = ros_params(
        repo_root / "camrod_sensing" / "config" / "inflation_cost_grid.yaml",
        "/sensing/inflation_cost_grid",
    )

    figure, axis = setup_figure(
        "Sensing pipelines: raw devices to planning cost",
        "Each row shows physical input, processing ownership, canonical output, and downstream consumer",
        module="sensing",
    )
    rows = [
        (
            "LiDAR",
            "Vanjee 750C points_raw",
            f"intra-process container\nROI + {lidar['downsample_resolution']:.2f} m voxel + ground filter",
            "/sensing/lidar/points_filtered",
            f"perception + optional grid (default {'ON' if sensing_defaults['enable_lidar_cost_grid'] else 'OFF'})\n{lidar_cost['width']}x{lidar_cost['height']} @ {lidar_cost['resolution']:.2f} m",
            GREEN,
        ),
        (
            "Radar x7",
            "FRONT1/2, LEFT1/2, RIGHT1/2, REAR",
            "range validation\nfixed-return exclusions",
            "/sensing/radar/*/range",
            f"radar cost grid\n{radar['width']}x{radar['height']} @ {radar['resolution']:.2f} m",
            AMBER,
        ),
        (
            "Cameras",
            "front + rear econ",
            "front camera/YOLO container\nrear capture/rectify/tag container",
            "rectified raw/compressed + CameraInfo",
            "YOLO/fusion + AprilTag parking",
            BLUE,
        ),
        (
            "Dual GNSS",
            "F9P rover + moving-base RTCM",
            "NTRIP correction\nNAV-PVT + RELPOSNED",
            "fix + absolute pose + heading",
            "localization EKF",
            GREEN,
        ),
        (
            "IMU + wheels",
            "CV7/GQ7 + Ranger status",
            "frame/covariance conversion\nvelocity adapter",
            "IMU + wheel odometry + twist",
            "localization EKF + diagnostics",
            BLUE,
        ),
    ]
    headers = ["Sensor", "Raw input", "Processing", "Canonical output", "Consumer"]
    xs = [0.045, 0.16, 0.36, 0.57, 0.775]
    widths = [0.09, 0.175, 0.185, 0.18, 0.18]
    for x, width, heading in zip(xs, widths, headers):
        axis.text(x + width / 2, 0.835, heading, ha="center", va="bottom", fontsize=8, fontweight="bold", color=MUTED)
    for row_index, row in enumerate(rows):
        y = 0.69 - row_index * 0.125
        color = row[-1]
        for column, value in enumerate(row[:-1]):
            face = GREEN_BG if color == GREEN else AMBER_BG if color == AMBER else BLUE_BG
            draw_box(
                axis,
                xs[column],
                y,
                widths[column],
                0.095,
                value.split("\n", 1)[0],
                tuple(value.split("\n")[1:]),
                face=face if column == 0 else WHITE,
                edge=color if column == 0 else "#a9b6bc",
                title_color=color if column == 0 else INK,
                title_size=7.8,
                body_size=6.7,
            )
            if column:
                draw_arrow(axis, (xs[column - 1] + widths[column - 1] + 0.003, y + 0.047), (xs[column] - 0.004, y + 0.047), width=1.0)

    section_label(axis, 0.045, 0.17, "Robot-centered cost fusion")
    sources = " + ".join(topic.rsplit("/", 1)[-1] for topic in inflation["input_topics"])
    draw_box(
        axis,
        0.045,
        0.055,
        0.91,
        0.09,
        "/planning/cost_grid/inflation",
        (f"{sources}", f"{inflation['width']}x{inflation['height']} @ {inflation['resolution']:.2f} m; stale timeouts remain source-specific"),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
        body_size=7.5,
    )
    footer(figure, "SOURCE-DERIVED. Front/YOLO, rear/AprilTag, and LiDAR processing have bounded containers; the LiDAR rasterizer is default OFF. Hardware quality still requires field logs.")
    save_figure(figure, output_root / "sensing" / "sensor-processing-and-cost-fusion.png")
    render_ground_segmentation_schematic(lidar, output_root / "sensing" / "ground-segmentation-schematic.png")


def render_ground_segmentation_schematic(params: dict, output: Path):
    """Render a clearly labeled algorithm schematic from active thresholds."""
    theme = module_theme("sensing")
    rng = np.random.default_rng(260804)
    x_ground = np.linspace(0.0, 5.0, 260)
    z_ground = 0.025 * x_ground + rng.normal(0.0, 0.018, x_ground.size)
    x_obstacle = rng.uniform(2.0, 2.65, 90)
    z_obstacle = rng.uniform(0.18, 1.15, 90)
    x_low = rng.uniform(3.4, 4.1, 35)
    z_low = 0.025 * x_low + rng.uniform(0.06, 0.13, 35)

    figure = plt.figure(figsize=(14, 7), facecolor=theme["bg"])
    figure._camrod_theme = theme
    figure.add_artist(
        Rectangle(
            (0.0, 0.986),
            0.76,
            0.014,
            color=theme["primary"],
            transform=figure.transFigure,
        )
    )
    figure.add_artist(
        Rectangle(
            (0.76, 0.986),
            0.24,
            0.014,
            color=theme["secondary"],
            transform=figure.transFigure,
        )
    )
    figure.text(
        0.055,
        0.94,
        "LiDAR ground segmentation",
        fontsize=21,
        fontweight="bold",
        color=theme["ink"],
        va="top",
    )
    figure.text(0.055, 0.885, "ALGORITHM SCHEMATIC - deterministic synthetic points, not a field point-cloud capture", fontsize=10, color=RED, va="top")
    axes = [
        figure.add_axes((0.07, 0.18, 0.4, 0.6), facecolor=theme["surface"]),
        figure.add_axes((0.54, 0.18, 0.4, 0.6), facecolor=theme["surface"]),
    ]
    for axis in axes:
        axis.set_xlim(0, 5)
        axis.set_ylim(-0.1, 1.3)
        axis.set_xlabel("forward x [m]")
        axis.set_ylabel("height z [m]")
        axis.grid(alpha=0.2)
    axes[0].scatter(x_ground, z_ground, s=9, color="#7f8c93", alpha=0.65, label="ground candidates")
    axes[0].scatter(x_obstacle, z_obstacle, s=13, color=RED, alpha=0.8, label="obstacle points")
    axes[0].scatter(
        x_low,
        z_low,
        s=12,
        color=theme["accent"],
        alpha=0.75,
        label="near-ground returns",
    )
    axes[0].set_title("Input after ROI + voxel downsample", loc="left", fontsize=11, fontweight="bold")
    axes[0].legend(loc="upper left", fontsize=8)
    retained = z_low - 0.025 * x_low > params["groundInlierThreshold"]
    axes[1].scatter(
        x_obstacle,
        z_obstacle,
        s=13,
        color=theme["secondary"],
        alpha=0.85,
        label="retained nonground",
    )
    axes[1].scatter(x_low[retained], z_low[retained], s=12, color=theme["secondary"], alpha=0.75)
    axes[1].scatter(x_ground[::5], z_ground[::5], s=8, color="#b9c3c8", alpha=0.35, label="removed ground (reference)")
    axes[1].set_title("Filtered nonground output", loc="left", fontsize=11, fontweight="bold")
    axes[1].legend(loc="upper left", fontsize=8)
    figure.text(
        0.5,
        0.075,
        "Active config: "
        f"voxel {params['downsample_resolution']:.2f} m | "
        f"cell {params['cellSizeX']:.2f} x {params['cellSizeY']:.2f} m | "
        f"slope <= {params['slopeThresholdDegrees']:.0f} deg | "
        f"inlier {params['groundInlierThreshold']:.2f} m",
        ha="center",
        color=theme["muted"],
        fontsize=9,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150, facecolor=figure.get_facecolor())
    plt.close(figure)


# HH_260804 - Give every CAMROD-owned package a compact, reproducible README
# visual. Values are read from package sources or committed evidence so a
# configured threshold is never presented as measured runtime performance.
def render_common(repo_root: Path, output_root: Path):
    """Render the shared-interface inventory and dependency boundary."""
    interface_root = repo_root / "camrod_common" / "avg_msgs"
    messages = sorted((interface_root / "msg").glob("*.msg"))
    services = sorted((interface_root / "srv").glob("*.srv"))
    dependents = []
    for package_xml in sorted(repo_root.glob("camrod_*/package.xml")):
        if "avg_msgs" in package_xml.read_text(encoding="utf-8"):
            dependents.append(package_xml.parent.name)

    figure, axis = setup_figure(
        "Shared interface contract",
        "avg_msgs is the generated ROS 2 boundary shared by CAMROD runtime packages",
        module="common",
    )
    section_label(axis, 0.045, 0.84, "Interface inventory")
    draw_box(
        axis,
        0.045,
        0.64,
        0.25,
        0.16,
        f"{len(messages)} messages",
        ("geometry + sensor payloads", "mission, service, and health state", "module bundle messages"),
        face=BLUE_BG,
        edge=BLUE,
        title_color=BLUE,
    )
    draw_box(
        axis,
        0.325,
        0.64,
        0.25,
        0.16,
        f"{len(services)} services",
        tuple(path.stem for path in services),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
    )
    draw_box(
        axis,
        0.605,
        0.64,
        0.35,
        0.16,
        f"{len(dependents)} dependent packages",
        tuple(dependents[:6]) + (("+ remaining CAMROD packages",) if len(dependents) > 6 else ()),
        face=GRAY_BG,
        edge="#72848d",
        body_size=7.2,
    )

    section_label(axis, 0.045, 0.57, "Ownership boundary")
    stages = [
        (0.045, "Package source", ("msg/*.msg", "srv/*.srv"), BLUE),
        (0.29, "rosidl generator", ("C / C++ / Python", "typesupport"), AMBER),
        (0.535, "CAMROD nodes", ("publish + subscribe", "typed state and payloads"), GREEN),
        (0.78, "UI / logs / tests", ("same numeric enums", "contract inspection"), BLUE),
    ]
    for index, (x, title, lines, color) in enumerate(stages):
        draw_box(
            axis,
            x,
            0.37,
            0.175,
            0.145,
            title,
            lines,
            face=BLUE_BG if color == BLUE else AMBER_BG if color == AMBER else GREEN_BG,
            edge=color,
            title_color=color,
            body_size=7.4,
        )
        if index:
            draw_arrow(axis, (x - 0.065, 0.442), (x - 0.005, 0.442))

    draw_box(
        axis,
        0.045,
        0.12,
        0.44,
        0.16,
        "What this package guarantees",
        (
            "one checked-in definition per generated interface",
            "build-order contract before dependent packages",
            "enum constants shared by backend, UI, and diagnostics",
        ),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
    )
    draw_box(
        axis,
        0.515,
        0.12,
        0.44,
        0.16,
        "Performance statement",
        (
            "No runtime throughput or latency is measured here.",
            "Message/service counts are source inventory, not performance.",
            "Transport performance must be measured per publisher path.",
        ),
        face=AMBER_BG,
        edge=AMBER,
        title_color=AMBER,
    )
    footer(
        figure,
        "SOURCE INVENTORY: camrod_common/avg_msgs/msg, srv, package.xml, and dependent package manifests.",
    )
    save_figure(figure, output_root / "common" / "interface-contract-and-dependencies.png")


def render_control(repo_root: Path, output_root: Path):
    """Render command authorization, footprint safety, and measured recovery."""
    gate = ros_params(
        repo_root / "camrod_control" / "config" / "cmd_vel_safety_gate.yaml",
        "/**",
    )
    recovery = ros_params(
        repo_root / "camrod_control" / "config" / "control.yaml",
        "/control/route_safety_recovery_controller",
    )
    crab = load_json(
        repo_root
        / "docs/evidence/v2.1.3/boundary-recovery/automatic-owner-one-sided-crab.json"
    )
    retry = load_json(
        repo_root
        / "docs/evidence/v2.1.3/boundary-recovery/automatic-owner-route-retry.json"
    )

    figure, axis = setup_figure(
        "Control command safety and boundary recovery",
        "Every navigation or maneuver command passes one final authorization and complete-footprint gate",
        module="control",
    )
    section_label(axis, 0.045, 0.84, "Command path")
    chain = [
        (0.045, "Command owners", ("Nav2 + site/drop-zone", "parking + recovery"), BLUE),
        (0.285, "Safety gate", ("engage / CAN / SOC", "localization / cost"), AMBER),
        (0.525, "Generated command", ("/control/cmd_vel", "/control/cmd_vel_ros"), GREEN),
        (0.765, "Ranger platform", ("Dual-Ackermann", "crab / zero-turn"), BLUE),
    ]
    for index, (x, title, lines, color) in enumerate(chain):
        draw_box(
            axis,
            x,
            0.67,
            0.19,
            0.13,
            title,
            lines,
            face=BLUE_BG if color == BLUE else AMBER_BG if color == AMBER else GREEN_BG,
            edge=color,
            title_color=color,
            body_size=7.2,
        )
        if index:
            draw_arrow(axis, (x - 0.05, 0.735), (x - 0.005, 0.735))

    section_label(axis, 0.045, 0.61, "Active safety values")
    draw_box(
        axis,
        0.045,
        0.40,
        0.28,
        0.17,
        "Mission and hard stop",
        (
            f"new mission >= {gate['minimum_mission_departure_battery_percentage'] * 100:.0f}% SOC",
            f"hard stop <= {gate['critical_battery_percentage'] * 100:.0f}% SOC",
            f"input timeout {gate['input_timeout_s']:.2f} s",
        ),
        face=AMBER_BG,
        edge=AMBER,
        title_color=AMBER,
    )
    draw_box(
        axis,
        0.36,
        0.40,
        0.28,
        0.17,
        "Complete footprint",
        (
            f"front/rear {gate['lanelet_safety_footprint_front_m']:.3f}/{gate['lanelet_safety_footprint_rear_m']:.3f} m",
            f"left/right {gate['lanelet_safety_footprint_left_m']:.3f}/{gate['lanelet_safety_footprint_right_m']:.3f} m",
            f"cost {gate['lanelet_safety_footprint_threshold']} or unknown -> hold",
        ),
        face=RED_BG,
        edge=RED,
        title_color=RED,
        body_size=7.5,
    )
    draw_box(
        axis,
        0.675,
        0.40,
        0.28,
        0.17,
        "Bounded recovery owner",
        (
            f"raw <= {recovery['maximum_speed_mps']:.2f} m/s; yaw <= {recovery['maximum_angular_speed_radps']:.2f} rad/s",
            f"travel <= {recovery['maximum_distance_m']:.2f} m; yaw <= {recovery['maximum_yaw_change_deg']:.0f} deg",
            f"reverse -> safe yaw/crab; clear proof {gate['route_safety_recovery_clear_required_s']:.1f} s",
        ),
        face=BLUE_BG,
        edge=BLUE,
        title_color=BLUE,
        body_size=7.3,
    )

    section_label(axis, 0.045, 0.34, "Historical v2.1.3 simulation evidence")
    draw_box(
        axis,
        0.045,
        0.13,
        0.43,
        0.17,
        "One side clear -> crab",
        (
            f"recovery displacement {crab['recovery_displacement_m']:.4f} m",
            f"platform lateral output <= {crab['maximum_recovery_abs_linear_y_mps']:.2f} m/s",
            "route hold released; mission completion not demonstrated",
        ),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
    )
    draw_box(
        axis,
        0.525,
        0.13,
        0.43,
        0.17,
        "Both sides blocked -> retry path",
        (
            f"release displacement {retry['recovery_displacement_m']:.4f} m",
            f"RPP retry {retry['same_goal_retry_displacement_m']:.4f} m / {retry['same_goal_retry_yaw_delta_deg']:+.4f} deg",
            "second mapped boundary hold occurred; mission not complete",
        ),
        face=AMBER_BG,
        edge=AMBER,
        title_color=AMBER,
    )
    footer(
        figure,
        "CURRENT CONFIG + MEASURED SIM. Map-v15 staged yaw/crab runtime evidence is published separately; real-robot acceptance remains pending.",
    )
    save_figure(figure, output_root / "control" / "command-safety-and-recovery.png")


def render_map(repo_root: Path, output_root: Path):
    """Render the Lanelet map, semantic areas, and dual-grid outputs."""
    map_info = ros_params(repo_root / "camrod_map/config/map_info.yaml", "/**")
    grid = ros_params(
        repo_root / "camrod_map/config/lanelet_cost_grid.yaml",
        "/map/lanelet_boundary_cost_grid",
    )

    figure, axis = setup_figure(
        "Lanelet map and cost-grid products",
        "One map/origin source feeds visualization, route masks, planning cost, and semantic service areas",
        module="map",
    )
    section_label(axis, 0.045, 0.84, "Map processing")
    chain = [
        (0.045, "Lanelet2 OSM", (Path(map_info["map_path"]).name, "LocalCartesian origin"), BLUE),
        (0.275, "Map provider", ("lanelets + centerlines", "world -> map"), GREEN),
        (0.505, "Cost-grid builder", ("route mask", "all-lane planning base"), AMBER),
        (0.735, "Consumers", ("Nav2 + goal snapper", "sensing filter + RViz"), BLUE),
    ]
    for index, (x, title, lines, color) in enumerate(chain):
        draw_box(
            axis,
            x,
            0.66,
            0.20,
            0.14,
            title,
            lines,
            face=BLUE_BG if color == BLUE else GREEN_BG if color == GREEN else AMBER_BG,
            edge=color,
            title_color=color,
            body_size=7.1,
        )
        if index:
            draw_arrow(axis, (x - 0.03, 0.73), (x - 0.005, 0.73))

    section_label(axis, 0.045, 0.60, "Active products and values")
    draw_box(
        axis,
        0.045,
        0.38,
        0.28,
        0.18,
        "Active-route mask",
        (
            f"{grid['width']} x {grid['height']} @ {grid['resolution']:.2f} m",
            f"mode {grid['cost_mode']} / outside {grid['outside_value']}",
            "rebuild on route path, not every pose",
        ),
        face=BLUE_BG,
        edge=BLUE,
        title_color=BLUE,
    )
    draw_box(
        axis,
        0.36,
        0.38,
        0.28,
        0.18,
        "Nav2 planning base",
        (
            f"{grid['secondary.width']} x {grid['secondary.height']} @ {grid['secondary.resolution']:.2f} m",
            f"centerline half-width {grid['secondary.centerline_half_width']:.2f} m",
            f"edge/off-lane {grid['secondary.lanelet_boundary_value']}/{grid['secondary.outside_value']}",
        ),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
    )
    draw_box(
        axis,
        0.675,
        0.38,
        0.28,
        0.18,
        "Map reference",
        (
            f"origin {map_info['offset_lat']:.7f}, {map_info['offset_lon']:.7f}",
            f"yaw offset {map_info['yaw_offset_deg']:.1f} deg",
            f"visual radius {map_info['progressive_visualization_radius_m']:.0f} m",
        ),
        face=GRAY_BG,
        edge="#72848d",
    )

    draw_box(
        axis,
        0.045,
        0.13,
        0.43,
        0.17,
        "Boundary semantics",
        (
            "planning edge cost 98 remains traversable",
            "outside cost 100 is a complete-footprint stop",
            "lane_change=yes can open configured crossings",
        ),
        face=RED_BG,
        edge=RED,
        title_color=RED,
    )
    draw_box(
        axis,
        0.525,
        0.13,
        0.43,
        0.17,
        "Current evidence limit",
        (
            "Grid topology and values are source-configured.",
            "B6/B12 service-access geometry is not surveyed.",
            "Current full-mission smoke therefore fails closed.",
        ),
        face=AMBER_BG,
        edge=AMBER,
        title_color=AMBER,
    )
    footer(figure, "SOURCE-DERIVED + KNOWN SIM LIMIT. Grid dimensions are configuration, not build-time performance.")
    save_figure(figure, output_root / "map" / "lanelet-map-and-cost-grids.png")


def render_platform(repo_root: Path, output_root: Path):
    """Render Ranger command adaptation and normalized status feedback."""
    params = ros_params(repo_root / "camrod_platform/config/ranger_driver.yaml", "/**")
    visual = ros_params(
        repo_root / "camrod_platform/config/robot_visualization.yaml",
        "/platform/robot_visualization",
    )
    figure, axis = setup_figure(
        "Ranger command and normalized platform status",
        "The platform package owns the hardware boundary; planning and UI consume stable CAMROD contracts",
        module="platform",
    )
    section_label(axis, 0.045, 0.84, "Command and feedback")
    draw_box(axis, 0.045, 0.65, 0.2, 0.15, "Final Twist", ("/control/cmd_vel_ros", "robot_center_link"), face=BLUE_BG, edge=BLUE, title_color=BLUE)
    draw_arrow(axis, (0.248, 0.725), (0.295, 0.725))
    draw_box(axis, 0.305, 0.65, 0.25, 0.15, "Ranger driver", ("CAN can0 / Dual-Ackermann", f"update loop {params['update_rate']} Hz"), face=GREEN_BG, edge=GREEN, title_color=GREEN)
    draw_arrow(axis, (0.558, 0.725), (0.605, 0.725))
    draw_box(axis, 0.615, 0.65, 0.34, 0.15, "Normalized feedback", ("odom + wheel/actuator + BMS", "/platform/status @ <= 10 Hz"), face=BLUE_BG, edge=BLUE, title_color=BLUE)

    section_label(axis, 0.045, 0.59, "Active transition and status values")
    draw_box(
        axis,
        0.045,
        0.36,
        0.28,
        0.19,
        "Steering transition",
        (
            f"rate {params['steering_transition_rate_radps']:.2f} rad/s",
            f"full speed <= {params['steering_transition_full_speed_error_rad']:.2f} rad error",
            f"translation stopped >= {params['steering_transition_stop_error_rad']:.2f} rad",
        ),
        face=AMBER_BG,
        edge=AMBER,
        title_color=AMBER,
    )
    draw_box(
        axis,
        0.36,
        0.36,
        0.28,
        0.19,
        "Battery / charging",
        (
            f"charge current > {params['charging_current_threshold_a']:.1f} A",
            f"debounce {params['charging_min_consecutive_samples']} samples",
            f"status heartbeat {params['platform_status_publish_rate_hz']:.0f} Hz",
        ),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
    )
    draw_box(
        axis,
        0.675,
        0.36,
        0.28,
        0.19,
        "Pose and visualization",
        (
            f"base/status frame {params['base_frame']}",
            f"marker + boundary {visual['publish_rate_hz']:.0f} Hz",
            f"localization fallback {visual['localization_pose_timeout_s']:.1f} s",
        ),
        face=BLUE_BG,
        edge=BLUE,
        title_color=BLUE,
        body_size=7.2,
    )
    draw_box(
        axis,
        0.045,
        0.12,
        0.91,
        0.15,
        "Evidence boundary",
        (
            "Rates and thresholds above are active configuration. The driver is intended for Jetson + Ranger CAN; this workstation must not retune that hardware profile.",
            "No CAN bus latency, steering settling time, battery-current sign, or physical wheel-angle accuracy is claimed without a robot log.",
        ),
        face=GRAY_BG,
        edge="#72848d",
        body_size=7.8,
    )
    footer(figure, "SOURCE-DERIVED; PHYSICAL RANGER PERFORMANCE PENDING. Simulation verifies interfaces, not CAN or actuator timing.")
    save_figure(figure, output_root / "platform" / "ranger-command-and-status.png")


def render_system(repo_root: Path, output_root: Path):
    """Render graph checks, diagnostic aggregation, and severity semantics."""
    checker = ros_params(repo_root / "camrod_system/config/system_checker.yaml", "/system/system_checker")
    aggregator = load_yaml(
        repo_root / "camrod_system/config/diagnostics/default/aggregator/diagnostics_config.yaml"
    )
    hardware = load_yaml(
        repo_root / "camrod_system/config/diagnostics/default/hw/hw_gpu_checker.yaml"
    )["hw_checker"]["ros__parameters"]
    figure, axis = setup_figure(
        "Diagnostic aggregation and operator severity",
        "Health, mission lifecycle, and command authorization remain three separate state surfaces",
        module="system",
    )
    section_label(axis, 0.045, 0.84, "Health pipeline")
    chain = [
        (0.045, "Graph manifest", (f"{len(checker['required_modules'])} required modules", "node/topic/type/publisher"), BLUE),
        (0.285, "Checker containers", ("24 checkers / 4 groups", "independent fault domains"), BLUE),
        (0.525, "Core container", ("4 aggregate/status nodes", f"{aggregator['global']['publish_rate_hz']:.0f} Hz grouped stream"), GREEN),
        (0.765, "Operator surfaces", ("metadata retained", "status + terminal + UI"), GREEN),
    ]
    for index, (x, title, lines, color) in enumerate(chain):
        draw_box(axis, x, 0.66, 0.19, 0.14, title, lines, face=BLUE_BG if color == BLUE else GREEN_BG, edge=color, title_color=color, body_size=7.1)
        if index:
            draw_arrow(axis, (x - 0.05, 0.73), (x - 0.005, 0.73))

    section_label(axis, 0.045, 0.60, "Severity and timing")
    draw_box(axis, 0.045, 0.39, 0.28, 0.17, "OK", ("required data fresh", "normal mission states stay OK", "waiting/charging are not faults"), face=GREEN_BG, edge=GREEN, title_color=GREEN)
    draw_box(axis, 0.36, 0.39, 0.28, 0.17, "WARN", ("recoverable degradation", "dummy hardware is visible WARN", "single Nav abort may be WARN"), face=AMBER_BG, edge=AMBER, title_color=AMBER)
    draw_box(axis, 0.675, 0.39, 0.28, 0.17, "ERROR", ("fault or required update missing", "ROS STALE normalizes to ERROR", "severity clears on fresh recovery"), face=RED_BG, edge=RED, title_color=RED)

    draw_box(
        axis,
        0.045,
        0.13,
        0.43,
        0.17,
        "Active timing",
        (
            f"graph check {checker['check_period_s']:.0f} Hz / grace {checker['startup_grace_s']:.0f} s",
            f"aggregator default timeout {aggregator['global']['timeout_s']:.0f} s",
            "system summary startup grace 10 s",
        ),
        face=BLUE_BG,
        edge=BLUE,
        title_color=BLUE,
    )
    draw_box(
        axis,
        0.525,
        0.13,
        0.43,
        0.17,
        "Selected hardware thresholds",
        (
            f"CPU WARN/ERROR {hardware['cpu']['warn_threshold']:.0f}/{hardware['cpu']['error_threshold']:.0f}%",
            f"memory {hardware['memory']['warn_threshold']:.0f}/{hardware['memory']['error_threshold']:.0f}%",
            f"disk {hardware['disk']['warn_threshold']:.0f}/{hardware['disk']['error_threshold']:.0f}%",
        ),
        face=GRAY_BG,
        edge="#72848d",
    )
    footer(figure, "SOURCE-DERIVED. The four-node core and four checker containers are default ON; standalone fallbacks remain available. Thresholds are policy, not measured Jetson utilization.")
    save_figure(figure, output_root / "system" / "diagnostic-severity-and-surfaces.png")


def render_ui(repo_root: Path, output_root: Path):
    """Render Robot/Guest UI parity and the committed lifecycle probe."""
    evidence = load_json(repo_root / "docs/evidence/v2.1.3/ui/guest-mission-lifecycle.json")
    service_states = parse_message_constants(
        repo_root / "camrod_common/avg_msgs/msg/AvgServiceState.msg"
    )
    figure, axis = setup_figure(
        "Robot UI and Guest UI mission/state contract",
        "Both surfaces share one backend mission contract while exposing role-appropriate controls",
        module="ui",
    )
    section_label(axis, 0.045, 0.84, "Shared command path")
    draw_box(axis, 0.045, 0.65, 0.20, 0.15, "Robot UI :8010", ("manual engage / stop", "destination + diagnostics"), face=BLUE_BG, edge=BLUE, title_color=BLUE)
    draw_box(axis, 0.29, 0.65, 0.20, 0.15, "Guest UI :8012", ("destination / return", "shared safety overlay"), face=GREEN_BG, edge=GREEN, title_color=GREEN)
    draw_arrow(axis, (0.248, 0.725), (0.555, 0.725))
    draw_arrow(axis, (0.493, 0.725), (0.555, 0.725))
    draw_box(axis, 0.565, 0.65, 0.19, 0.15, "ROS backend", ("mission key + goal", "engage + operation"), face=AMBER_BG, edge=AMBER, title_color=AMBER)
    draw_arrow(axis, (0.758, 0.725), (0.805, 0.725))
    draw_box(axis, 0.815, 0.65, 0.14, 0.15, "Robot stack", ("planning/control", "/service/state"), face=GRAY_BG, edge="#72848d", title_size=9.2)

    section_label(axis, 0.045, 0.60, "Operator-visible rules")
    draw_box(axis, 0.045, 0.38, 0.28, 0.18, "Mission admission", ("new site >= 35% SOC", "unknown battery blocks dispatch", "below threshold: finish current site"), face=AMBER_BG, edge=AMBER, title_color=AMBER)
    draw_box(axis, 0.36, 0.38, 0.28, 0.18, "Three status layers", (f"service lifecycle: {len(service_states)} states", "gate: enabled / safety hold", "health: OK / WARN / ERROR"), face=BLUE_BG, edge=BLUE, title_color=BLUE)
    draw_box(axis, 0.675, 0.38, 0.28, 0.18, "Manual control", ("manual engage shows driving", "operator stop -> OPERATOR_STOPPED", "cancelled warning can recover"), face=GREEN_BG, edge=GREEN, title_color=GREEN)

    section_label(axis, 0.045, 0.32, "Committed browser + ROS integration evidence")
    draw_box(
        axis,
        0.045,
        0.12,
        0.43,
        0.16,
        "Mission lifecycle",
        (
            f"battery {evidence['initial_battery']}% / {evidence['site_count']} sites",
            "mission key, departure, moving, and return observed",
            "return MotionOperation observed",
        ),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
    )
    draw_box(
        axis,
        0.525,
        0.12,
        0.43,
        0.16,
        "Safety and stop overlay",
        (
            f"gate {evidence['safety_overlay']['control_gate_state']} observed",
            f"operator stop -> {evidence['operator_stopped']['service_state_name']}",
            "browser contract passed; physical drive not claimed",
        ),
        face=AMBER_BG,
        edge=AMBER,
        title_color=AMBER,
    )
    footer(figure, "MEASURED UI/ROS INTEGRATION EVIDENCE + SOURCE POLICY. This is not a physical mission PASS.")
    save_figure(figure, output_root / "ui" / "robot-and-guest-mission-state.png")


def render_voice(repo_root: Path, output_root: Path):
    """Render voice-event sources, priority policy, and readiness contract."""
    params = ros_params(
        repo_root / "camrod_voice/config/voice_event_adapter.yaml",
        "/voice/voice_event_adapter",
    )
    figure, axis = setup_figure(
        "Voice event mapping and priority queue",
        "Runtime state changes become typed AudioRequest keys and pre-recorded WAV playback",
        module="voice",
    )
    section_label(axis, 0.045, 0.84, "Event-to-audio path")
    chain = [
        (0.045, "Runtime inputs", ("platform + system", "localization + gate + Nav2"), BLUE),
        (0.285, "Event policy", ("edge-triggered mapping", "readiness + mission context"), GREEN),
        (0.525, "AudioRequest", ("category.file_name", "priority + interrupt"), AMBER),
        (0.765, "Announcer", ("priority queue", "WAV -> ALSA/PulseAudio"), BLUE),
    ]
    for index, (x, title, lines, color) in enumerate(chain):
        draw_box(axis, x, 0.65, 0.19, 0.15, title, lines, face=BLUE_BG if color == BLUE else GREEN_BG if color == GREEN else AMBER_BG, edge=color, title_color=color, body_size=7.1)
        if index:
            draw_arrow(axis, (x - 0.05, 0.725), (x - 0.005, 0.725))

    section_label(axis, 0.045, 0.59, "Active policy")
    priorities = [
        (0.045, "0 INFO", "ordinary cue", GRAY_BG, "#72848d"),
        (0.275, "1 NOTICE", "mission / charging", BLUE_BG, BLUE),
        (0.505, "2 WARNING", "obstacle / battery critical", AMBER_BG, AMBER),
        (0.735, "3 CRITICAL", "ESTOP; may interrupt", RED_BG, RED),
    ]
    for x, title, line, face, color in priorities:
        draw_box(axis, x, 0.43, 0.20, 0.11, title, (line,), face=face, edge=color, title_color=color, title_size=9.0, body_size=7.2)

    draw_box(
        axis,
        0.045,
        0.17,
        0.43,
        0.17,
        "Readiness and battery",
        (
            f"startup delay {params['startup_delay_s']:.1f} s",
            f"ready check every {params['readiness_check_period_s']:.1f} s / {len(params['readiness_required_modules'])} modules",
            f"battery low <= {params['battery_low_threshold'] * 100:.0f}%; critical <= {params['battery_critical_threshold'] * 100:.0f}%",
        ),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
    )
    draw_box(
        axis,
        0.525,
        0.17,
        0.43,
        0.17,
        "Performance statement",
        (
            "Queue policy is source-tested; audio assets are deterministic.",
            "Speaker loudness, Bluetooth delay, and audibility are field pending.",
            "No acoustic latency number is claimed from this workstation.",
        ),
        face=AMBER_BG,
        edge=AMBER,
        title_color=AMBER,
    )
    footer(figure, "SOURCE-DERIVED POLICY; PHYSICAL AUDIO PERFORMANCE PENDING.")
    save_figure(figure, output_root / "voice" / "voice-events-and-priority.png")


def main():
    """Render every module guide from configs and committed evidence."""
    default_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=default_root)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--localization-report", type=Path)
    parser.add_argument("--bringup-report", type=Path)
    parser.add_argument("--field-report", type=Path)
    parser.add_argument(
        "--module",
        action="append",
        choices=(
            "bringup",
            "common",
            "control",
            "localization",
            "map",
            "planning",
            "perception",
            "platform",
            "sensing",
            "sensor-kit",
            "system",
            "ui",
            "voice",
        ),
        help="render only this package (repeatable; default: render every package)",
    )
    args = parser.parse_args()

    repo_root = args.repo_root.resolve()
    output_root = (args.output_root or repo_root / "docs" / "assets" / "module-guides").resolve()
    localization_report = (
        args.localization_report
        or repo_root / "docs" / "evidence" / "module-guides" / "localization" / "pose-chain-sim-20260804.json"
    ).resolve()
    bringup_report = (
        args.bringup_report
        or repo_root / "docs" / "evidence" / "module-guides" / "bringup" / "campsite-smoke-20260804.json"
    ).resolve()
    field_report = (
        args.field_report
        or repo_root
        / "docs/evidence/module-guides/bringup/field-stationary-20260731.json"
    ).resolve()

    selected_modules = set(
        args.module
        or (
            "bringup",
            "common",
            "control",
            "localization",
            "map",
            "planning",
            "perception",
            "platform",
            "sensing",
            "sensor-kit",
            "system",
            "ui",
            "voice",
        )
    )
    if "bringup" in selected_modules:
        render_bringup_contract(repo_root, output_root)
        render_bringup_evidence(repo_root, bringup_report, output_root)
        render_field_stationary_report(field_report, output_root)
    if "common" in selected_modules:
        render_common(repo_root, output_root)
    if "control" in selected_modules:
        render_control(repo_root, output_root)
    if "localization" in selected_modules:
        render_localization(repo_root, localization_report, output_root)
    if "map" in selected_modules:
        render_map(repo_root, output_root)
    if "planning" in selected_modules:
        render_planning(repo_root, output_root)
    if "perception" in selected_modules:
        render_perception(repo_root, output_root)
    if "platform" in selected_modules:
        render_platform(repo_root, output_root)
    if "sensing" in selected_modules:
        render_sensing(repo_root, output_root)
    if "sensor-kit" in selected_modules:
        render_sensor_kit(repo_root, output_root)
    if "system" in selected_modules:
        render_system(repo_root, output_root)
    if "ui" in selected_modules:
        render_ui(repo_root, output_root)
    if "voice" in selected_modules:
        render_voice(repo_root, output_root)
    print(f"Rendered module README assets under {output_root}")


if __name__ == "__main__":
    main()
