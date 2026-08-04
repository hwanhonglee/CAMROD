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


def load_yaml(path: Path) -> dict:
    """Load one YAML source file."""
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def load_json(path: Path) -> dict:
    """Load one JSON evidence file."""
    return json.loads(path.read_text(encoding="utf-8"))


def ros_params(path: Path, node_name: str) -> dict:
    """Return a ROS parameter mapping from a package config."""
    return load_yaml(path)[node_name]["ros__parameters"]


def setup_figure(title: str, subtitle: str, size=(16, 9)):
    """Create a consistent documentation canvas."""
    figure = plt.figure(figsize=size, facecolor=BG)
    axis = figure.add_axes((0, 0, 1, 1))
    axis.set_xlim(0, 1)
    axis.set_ylim(0, 1)
    axis.axis("off")
    figure.text(0.045, 0.95, title, color=INK, fontsize=22, fontweight="bold", va="top")
    figure.text(0.045, 0.905, subtitle, color=MUTED, fontsize=10.5, va="top")
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
            color=MUTED,
            fontsize=body_size,
            linespacing=1.28,
            va="top",
            zorder=3,
        )
    return patch


def draw_arrow(axis, start, end, color="#71828b", width=1.5, style="-|>"):
    """Draw a stable connector between boxes."""
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
    axis.text(x, y, text.upper(), color=color, fontsize=8.5, fontweight="bold", va="bottom")


def footer(figure, text: str):
    """Add source/evidence classification to the canvas."""
    figure.text(0.045, 0.028, text, color=MUTED, fontsize=8.3, va="bottom")


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
        f"Baseline {report['baseline_commit']} | reverse parking | safety policy unchanged",
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
            "Rerun B6 + B12 with 0.10 m margin and full-footprint gate",
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


def render_localization(repo_root: Path, report_path: Path, output_root: Path):
    """Render pose generation inputs and measured sim output timing."""
    report = load_json(report_path)
    field = ros_params(repo_root / "camrod_localization" / "config" / "filter" / "ekf.yaml", "/**")
    simulation = ros_params(repo_root / "camrod_localization" / "config" / "filter" / "ekf_sim.yaml", "/**")
    topics = {entry["name"]: entry for entry in report["topics"]}

    figure, axis = setup_figure(
        "Localization pose generation and measured timing",
        "GNSS absolute position + dual-GNSS heading + IMU + wheel velocity -> EKF prediction -> selected robot pose",
    )
    section_label(axis, 0.045, 0.84, "Pose generation chain")
    inputs = [
        ("GNSS position", "/sensing/gnss/pose...", "absolute XYZ + covariance"),
        ("Dual-GNSS yaw", "same pose topic", "used only with valid heading covariance"),
        ("IMU", "/sensing/imu/data_ros", "field: roll/pitch/rates; sim: fake yaw too"),
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
    defaults = load_yaml(
        repo_root / "camrod_bringup" / "config" / "bringup" / "launch_defaults.yaml"
    )
    planning_defaults = defaults["bringup"]["planning"]

    figure, axis = setup_figure(
        "Planning runtime: Nav2 servers, selectors, and mission states",
        "Default full bringup selects LaneletRoute + RPP; grid and sampling plugins remain fallback/selectable",
    )
    section_label(axis, 0.045, 0.84, "Route-to-command execution")
    chain = [
        (0.045, "UI / RViz goal", ("mission key + site goal", "manual goals remain visible"), BLUE),
        (0.225, "Goal snapper", ("lanelet component", "source-aware release"), BLUE),
        (0.405, "Planner server", tuple(planner["planner_plugins"]), GREEN),
        (0.625, "BT + smoother", ("PlannerSelector", "ControllerSelector", "recovery behaviors"), BLUE),
        (0.805, "Controller server", tuple(controller["controller_plugins"]), GREEN),
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
            "manual RViz goal: LaneletRoute + RotationShim(RPP)",
            "obstacle fallback: SmacLattice; restore LaneletRoute",
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
    footer(figure, "SOURCE-DERIVED: camrod_planning/config/nav2_base.yaml, bringup launch defaults, planning/UI state contracts.")
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
            "rear image -> image_proc -> tag pose + debug image",
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


# HH_260804 - Keep the center-frame migration inspectable without suggesting
# that any physical sensor was moved. Both images are generated from the one
# package-owned geometry source used by xacro and RobotParams consumers.
def sensor_kit_mounts(params: dict) -> list[dict]:
    """Return every configured mount as a flat plotting/ledger sequence."""
    mounts = [
        {"name": "IMU", "short": "IMU", "category": "IMU", **params["imu"]},
        {"name": "GNSS*", "short": "GNSS", "category": "GNSS", **params["gnss"]},
        {"name": "LiDAR", "short": "LiDAR", "category": "LiDAR", **params["lidar"]},
        {
            "name": "Camera front",
            "short": "Cam F",
            "category": "Camera",
            **params["camera"]["front"],
        },
        {
            "name": "Camera rear",
            "short": "Cam R",
            "category": "Camera",
            **params["camera"]["rear"],
        },
    ]
    radar_labels = {
        "front1": "RF1",
        "front2": "RF2",
        "left1": "RL1",
        "left2": "RL2",
        "right1": "RR1",
        "right2": "RR2",
        "rear": "RRear",
    }
    mounts.extend(
        {
            "name": f"Radar {key}",
            "short": radar_labels[key],
            "category": "Radar",
            **pose,
        }
        for key, pose in params["radar"].items()
    )
    return mounts


def render_sensor_kit(repo_root: Path, output_root: Path):
    """Render the static TF contract and measured mount-coordinate conversion."""
    params = ros_params(
        repo_root / "camrod_sensor_kit" / "config" / "robot_params.yaml", "/**"
    )
    robot = params["robot"]
    extents = robot["body_extents"]
    offset = robot["center_offset_from_rear_axle"]
    mounts = sensor_kit_mounts(params)
    colors = {
        "IMU": GREEN,
        "GNSS": AMBER,
        "LiDAR": BLUE,
        "Camera": "#7251b5",
        "Radar": RED,
    }
    markers = {"IMU": "D", "GNSS": "P", "LiDAR": "o", "Camera": "s", "Radar": "^"}

    figure, canvas = setup_figure(
        "Sensor kit frames and physical mount layout",
        "Current robot_center_link geometry, legacy rear-axle compatibility, and fixed sensor transforms",
        size=(16, 10),
    )
    section_label(canvas, 0.045, 0.845, "Static TF ownership")
    draw_box(
        canvas,
        0.045,
        0.73,
        0.19,
        0.09,
        "robot_center_link",
        ("Nav2/control + body origin", "front/rear axle midpoint"),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
        body_size=7.2,
    )
    draw_box(
        canvas,
        0.335,
        0.73,
        0.20,
        0.09,
        "sensor_kit_base_link",
        ("fixed, coincident child", "xyz = (0, 0, 0)"),
        face=BLUE_BG,
        edge=BLUE,
        title_color=BLUE,
        body_size=7.2,
    )
    draw_box(
        canvas,
        0.64,
        0.73,
        0.315,
        0.09,
        "Fixed sensor children",
        ("IMU + GNSS + LiDAR + cameras + 7 radars", "camera links also own optical-frame children"),
        face=WHITE,
        edge="#899aa3",
        body_size=7.2,
    )
    draw_box(
        canvas,
        0.335,
        0.625,
        0.20,
        0.07,
        "robot_base_link",
        (f"legacy rear axle | x = -{offset:.3f} m",),
        face=AMBER_BG,
        edge=AMBER,
        title_color=AMBER,
        body_size=7.2,
    )
    draw_arrow(canvas, (0.235, 0.775), (0.335, 0.775), color=GREEN)
    draw_arrow(canvas, (0.535, 0.775), (0.64, 0.775), color=BLUE)
    draw_arrow(canvas, (0.235, 0.755), (0.335, 0.66), color=AMBER)

    top = figure.add_axes((0.055, 0.10, 0.43, 0.46), facecolor=WHITE)
    side = figure.add_axes((0.535, 0.10, 0.41, 0.46), facecolor=WHITE)
    for plot in (top, side):
        plot.grid(alpha=0.18)
        plot.tick_params(labelsize=7, colors=MUTED)
        for spine in plot.spines.values():
            spine.set_color("#b8c4ca")

    margin = extents["planning_margin"]
    top.add_patch(
        Rectangle(
            (-extents["rear"] - margin, -extents["right"] - margin),
            extents["front"] + extents["rear"] + 2 * margin,
            extents["left"] + extents["right"] + 2 * margin,
            fill=False,
            edgecolor=RED,
            linewidth=1.3,
            linestyle="--",
            label=f"planning boundary (+{margin:.2f} m/side)",
        )
    )
    top.add_patch(
        Rectangle(
            (-extents["rear"], -extents["right"]),
            extents["front"] + extents["rear"],
            extents["left"] + extents["right"],
            facecolor=GRAY_BG,
            edgecolor=INK,
            linewidth=1.4,
            alpha=0.75,
            label="physical body",
        )
    )
    for axle_x, axle_name in ((-offset, "rear axle"), (offset, "front axle")):
        top.plot(
            [axle_x, axle_x],
            [-extents["right"], extents["left"]],
            color="#75858d",
            linewidth=1.1,
            linestyle=":" if axle_x > 0 else "-.",
        )
        top.text(axle_x, -0.73, axle_name, ha="center", fontsize=6.8, color=MUTED)
    top.scatter([0.0], [0.0], color=GREEN, marker="x", s=65, linewidth=2.0, zorder=8)
    top.scatter([-offset], [0.0], color=AMBER, marker="x", s=65, linewidth=2.0, zorder=8)
    top.text(0.02, -0.055, "robot center", fontsize=7, color=GREEN, fontweight="bold")
    # Centerline sensors share nearly identical top-view coordinates. Use one
    # callout per physical stack so labels do not imply artificial separation.
    label_offsets = {
        "RF1": (0.02, -0.10),
        "RF2": (0.02, 0.07),
        "RL1": (0.02, 0.055),
        "RL2": (-0.03, 0.055),
        "RR1": (0.02, -0.10),
        "RR2": (-0.03, -0.10),
        "RRear": (-0.02, 0.07),
    }
    legend_seen = set()
    for mount in mounts:
        category = mount["category"]
        label = category if category not in legend_seen else None
        legend_seen.add(category)
        top.scatter(
            [mount["x"]],
            [mount["y"]],
            s=37,
            marker=markers[category],
            color=colors[category],
            edgecolors=WHITE,
            linewidth=0.6,
            zorder=6,
            label=label,
        )
        if category == "Radar":
            dx, dy = label_offsets[mount["short"]]
            top.text(
                mount["x"] + dx,
                mount["y"] + dy,
                mount["short"],
                fontsize=6.4,
                color=colors[category],
                ha="right" if dx < 0 else "left",
                fontweight="bold",
            )
        if category in ("Camera", "Radar"):
            yaw = np.deg2rad(mount["yaw"])
            top.arrow(
                mount["x"],
                mount["y"],
                0.11 * np.cos(yaw),
                0.11 * np.sin(yaw),
                width=0.004,
                head_width=0.035,
                color=colors[category],
                length_includes_head=True,
                zorder=5,
            )
    callouts = (
        (
            (params["gnss"]["x"], params["gnss"]["y"]),
            (-0.93, 0.18),
            "GNSS* placeholder\n+ rear-axle origin",
            AMBER,
        ),
        (
            (params["camera"]["rear"]["x"], params["camera"]["rear"]["y"]),
            (-0.94, -0.24),
            "rear camera\n+ rear radar",
            "#7251b5",
        ),
        (
            (params["lidar"]["x"], params["lidar"]["y"]),
            (0.43, 0.27),
            "IMU + LiDAR + front camera\n(centerline mounts)",
            BLUE,
        ),
    )
    for point, label_position, label, color in callouts:
        top.annotate(
            label,
            xy=point,
            xytext=label_position,
            arrowprops={"arrowstyle": "-", "color": color, "linewidth": 0.8},
            fontsize=6.4,
            color=color,
            fontweight="bold",
            ha="left",
        )
    top.annotate(
        "FRONT +X",
        xy=(0.96, 0.0),
        xytext=(0.73, 0.0),
        arrowprops={"arrowstyle": "-|>", "color": INK},
        color=INK,
        fontsize=7.5,
        fontweight="bold",
        va="center",
    )
    top.set_xlim(-1.0, 1.05)
    top.set_ylim(-0.82, 0.82)
    top.set_aspect("equal", adjustable="box")
    top.set_xlabel("x forward [m]", fontsize=8)
    top.set_ylabel("y left [m]", fontsize=8)
    top.set_title("Top view: body, safety boundary, and mount yaw", loc="left", fontsize=10, fontweight="bold", color=INK)
    top.legend(loc="upper left", ncol=2, fontsize=6.5, framealpha=0.92)

    side.add_patch(
        Rectangle(
            (-extents["rear"], extents["bottom_z"]),
            extents["front"] + extents["rear"],
            extents["top_z"] - extents["bottom_z"],
            facecolor=GRAY_BG,
            edgecolor=INK,
            linewidth=1.4,
            alpha=0.75,
        )
    )
    for axle_x in (-offset, offset):
        side.axvline(axle_x, color="#75858d", linewidth=1.0, linestyle="-.")
    side.scatter([0.0], [0.0], color=GREEN, marker="x", s=65, linewidth=2.0, zorder=8)
    side.scatter([-offset], [0.0], color=AMBER, marker="x", s=65, linewidth=2.0, zorder=8)
    side.text(0.015, -0.08, "robot center", fontsize=7, color=GREEN, fontweight="bold")
    side.text(-offset - 0.015, -0.08, "rear axle", fontsize=7, color=AMBER, ha="right", fontweight="bold")
    for mount in mounts:
        side.scatter(
            [mount["x"]],
            [mount["z"]],
            s=38,
            marker=markers[mount["category"]],
            color=colors[mount["category"]],
            edgecolors=WHITE,
            linewidth=0.6,
            zorder=6,
        )
    side_labels = (
        (params["imu"]["x"], params["imu"]["z"], "IMU", GREEN, -0.10, 0.07),
        (params["gnss"]["x"], params["gnss"]["z"], "GNSS*", AMBER, -0.05, 0.07),
        (params["lidar"]["x"], params["lidar"]["z"], "LiDAR", BLUE, 0.03, 0.08),
        (params["camera"]["front"]["x"], params["camera"]["front"]["z"], "front camera", "#7251b5", 0.03, 0.03),
        (params["camera"]["rear"]["x"], params["camera"]["rear"]["z"], "rear camera", "#7251b5", -0.03, -0.09),
        (params["radar"]["front1"]["x"], params["radar"]["front1"]["z"], "front radars x2", RED, -0.06, -0.04),
        (params["radar"]["left1"]["x"], params["radar"]["left1"]["z"], "side radar pair 1", RED, 0.01, -0.10),
        (params["radar"]["left2"]["x"], params["radar"]["left2"]["z"], "side radar pair 2", RED, -0.01, 0.07),
        (params["radar"]["rear"]["x"], params["radar"]["rear"]["z"], "rear radar", RED, -0.03, 0.07),
    )
    for x, z, label, color, dx, dz in side_labels:
        side.text(
            x + dx,
            z + dz,
            label,
            fontsize=6.5,
            color=color,
            ha="right" if dx < 0 else "left",
            fontweight="bold",
        )
    side.annotate(
        "FRONT +X",
        xy=(0.96, 1.01),
        xytext=(0.70, 1.01),
        arrowprops={"arrowstyle": "-|>", "color": INK},
        color=INK,
        fontsize=7.5,
        fontweight="bold",
        va="center",
    )
    side.set_xlim(-1.0, 1.05)
    side.set_ylim(-0.24, 1.10)
    side.set_xlabel("x forward [m]", fontsize=8)
    side.set_ylabel("z up [m]", fontsize=8)
    side.set_title("Side view: unchanged physical mount positions", loc="left", fontsize=10, fontweight="bold", color=INK)
    footer(
        figure,
        "SOURCE-DERIVED from camrod_sensor_kit/config/robot_params.yaml. GNSS* is the retained rear-axle placeholder; the physical antenna lever arm remains FIELD PENDING.",
    )
    save_figure(
        figure, output_root / "sensor-kit" / "tf-and-sensor-mount-layout.png"
    )

    render_sensor_kit_conversion_ledger(
        robot, mounts, output_root / "sensor-kit" / "rear-axle-to-center-ledger.png"
    )


def render_sensor_kit_conversion_ledger(robot: dict, mounts: list[dict], output: Path):
    """Render exact old/new X coordinates while preserving physical invariants."""
    offset = robot["center_offset_from_rear_axle"]
    extents = robot["body_extents"]
    figure, axis = setup_figure(
        "Rear-axle to robot-center coordinate ledger",
        "Same physical robot and sensor mounts; only the coordinate origin changed",
        size=(16, 10),
    )
    draw_box(
        axis,
        0.045,
        0.72,
        0.25,
        0.13,
        "Previous reference",
        (
            "robot_base_link at rear axle",
            f"body x: {-extents['rear'] + offset:+.5f} to {extents['front'] + offset:+.5f} m",
        ),
        face=AMBER_BG,
        edge=AMBER,
        title_color=AMBER,
        body_size=8.5,
    )
    draw_box(
        axis,
        0.375,
        0.72,
        0.25,
        0.13,
        "Coordinate conversion",
        (
            f"current_x = previous_x - {offset:.3f} m",
            "y, z, roll, pitch, yaw unchanged",
        ),
        face=BLUE_BG,
        edge=BLUE,
        title_color=BLUE,
        body_size=8.5,
    )
    draw_box(
        axis,
        0.705,
        0.72,
        0.25,
        0.13,
        "Current reference",
        (
            "robot_center_link at axle midpoint",
            f"body x: {-extents['rear']:+.5f} to {extents['front']:+.5f} m",
        ),
        face=GREEN_BG,
        edge=GREEN,
        title_color=GREEN,
        body_size=8.5,
    )
    draw_arrow(axis, (0.295, 0.785), (0.375, 0.785), color=BLUE)
    draw_arrow(axis, (0.625, 0.785), (0.705, 0.785), color=BLUE)
    axis.text(
        0.5,
        0.675,
        f"Invariant body: {robot['length']:.5f} x {robot['width']:.5f} m  |  "
        f"wheelbase {robot['wheelbase']:.3f} m  |  planning margin {extents['planning_margin']:.2f} m per side",
        ha="center",
        fontsize=9.5,
        color=INK,
        fontweight="bold",
    )

    columns = ("Mount", "Previous rear-axle x [m]", "Current center x [m]", "Delta x [m]", "Current y [m]", "Current z [m]")
    rows = []
    for mount in mounts:
        rows.append(
            (
                mount["name"],
                f"{mount['x'] + offset:+.5f}",
                f"{mount['x']:+.5f}",
                f"{-offset:+.3f}",
                f"{mount['y']:+.5f}",
                f"{mount['z']:+.5f}",
            )
        )
    table_axis = figure.add_axes((0.045, 0.085, 0.91, 0.53))
    table_axis.axis("off")
    table = table_axis.table(
        cellText=rows,
        colLabels=columns,
        cellLoc="center",
        colLoc="center",
        loc="center",
        colWidths=(0.18, 0.20, 0.19, 0.13, 0.15, 0.15),
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8.0)
    table.scale(1.0, 1.42)
    for (row, column), cell in table.get_celld().items():
        cell.set_edgecolor("#aebbc1")
        cell.set_linewidth(0.7)
        if row == 0:
            cell.set_facecolor(INK)
            cell.set_text_props(color=WHITE, fontweight="bold")
        elif row % 2:
            cell.set_facecolor(WHITE)
        else:
            cell.set_facecolor("#edf2f4")
        if column == 0 and row > 0:
            cell.set_text_props(ha="left", fontweight="bold", color=INK)
    footer(
        figure,
        "SOURCE-DERIVED. Delta is exactly -0.443 m for every X coordinate; physical mounts did not move. GNSS* remains a placeholder until the antenna lever arm is measured on the robot.",
    )
    save_figure(figure, output)


def render_sensing(repo_root: Path, output_root: Path):
    """Render sensor-specific processing and final cost-grid fusion."""
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
    )
    rows = [
        (
            "LiDAR",
            "Vanjee 750C points_raw",
            f"ROI + {lidar['downsample_resolution']:.2f} m voxel\nDFKI ground segmentation",
            "/sensing/lidar/points_filtered",
            f"perception -> lidar grid\n{lidar_cost['width']}x{lidar_cost['height']} @ {lidar_cost['resolution']:.2f} m",
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
            "front VPI/NvJPEG\nrear GStreamer/image_proc",
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
    footer(figure, "SOURCE-DERIVED. Hardware signal quality and detection correctness require field logs; topic existence alone is insufficient.")
    save_figure(figure, output_root / "sensing" / "sensor-processing-and-cost-fusion.png")
    render_ground_segmentation_schematic(lidar, output_root / "sensing" / "ground-segmentation-schematic.png")


def render_ground_segmentation_schematic(params: dict, output: Path):
    """Render a clearly labeled algorithm schematic from active thresholds."""
    rng = np.random.default_rng(260804)
    x_ground = np.linspace(0.0, 5.0, 260)
    z_ground = 0.025 * x_ground + rng.normal(0.0, 0.018, x_ground.size)
    x_obstacle = rng.uniform(2.0, 2.65, 90)
    z_obstacle = rng.uniform(0.18, 1.15, 90)
    x_low = rng.uniform(3.4, 4.1, 35)
    z_low = 0.025 * x_low + rng.uniform(0.06, 0.13, 35)

    figure = plt.figure(figsize=(14, 7), facecolor=BG)
    figure.text(0.055, 0.94, "LiDAR ground segmentation", fontsize=21, fontweight="bold", color=INK, va="top")
    figure.text(0.055, 0.885, "ALGORITHM SCHEMATIC - deterministic synthetic points, not a field point-cloud capture", fontsize=10, color=RED, va="top")
    axes = [figure.add_axes((0.07, 0.18, 0.4, 0.6)), figure.add_axes((0.54, 0.18, 0.4, 0.6))]
    for axis in axes:
        axis.set_xlim(0, 5)
        axis.set_ylim(-0.1, 1.3)
        axis.set_xlabel("forward x [m]")
        axis.set_ylabel("height z [m]")
        axis.grid(alpha=0.2)
    axes[0].scatter(x_ground, z_ground, s=9, color="#7f8c93", alpha=0.65, label="ground candidates")
    axes[0].scatter(x_obstacle, z_obstacle, s=13, color=RED, alpha=0.8, label="obstacle points")
    axes[0].scatter(x_low, z_low, s=12, color=AMBER, alpha=0.75, label="near-ground returns")
    axes[0].set_title("Input after ROI + voxel downsample", loc="left", fontsize=11, fontweight="bold")
    axes[0].legend(loc="upper left", fontsize=8)
    retained = z_low - 0.025 * x_low > params["groundInlierThreshold"]
    axes[1].scatter(x_obstacle, z_obstacle, s=13, color=GREEN, alpha=0.85, label="retained nonground")
    axes[1].scatter(x_low[retained], z_low[retained], s=12, color=GREEN, alpha=0.75)
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
        color=MUTED,
        fontsize=9,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150, facecolor=figure.get_facecolor())
    plt.close(figure)


def main():
    """Render every module guide from configs and committed evidence."""
    default_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=default_root)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--localization-report", type=Path)
    parser.add_argument("--bringup-report", type=Path)
    parser.add_argument(
        "--module",
        action="append",
        choices=(
            "bringup",
            "localization",
            "planning",
            "perception",
            "sensing",
            "sensor-kit",
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

    selected_modules = set(
        args.module
        or (
            "bringup",
            "localization",
            "planning",
            "perception",
            "sensing",
            "sensor-kit",
        )
    )
    if "bringup" in selected_modules:
        render_bringup_contract(repo_root, output_root)
        render_bringup_evidence(repo_root, bringup_report, output_root)
    if "localization" in selected_modules:
        render_localization(repo_root, localization_report, output_root)
    if "planning" in selected_modules:
        render_planning(repo_root, output_root)
    if "perception" in selected_modules:
        render_perception(repo_root, output_root)
    if "sensing" in selected_modules:
        render_sensing(repo_root, output_root)
    if "sensor-kit" in selected_modules:
        render_sensor_kit(repo_root, output_root)
    print(f"Rendered module README assets under {output_root}")


if __name__ == "__main__":
    main()
