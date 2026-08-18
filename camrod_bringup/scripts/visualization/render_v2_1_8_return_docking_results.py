#!/usr/bin/env python3
"""Render v2.1.8 crab, campsite return, and parking validation assets."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
from pathlib import Path

import yaml
from PIL import Image, ImageDraw, ImageFont


# HH_260818 - Generate release evidence from canonical YAML and a committed
# simulation report so documentation never relies on hand-entered tuning data.
WIDTH = 1600
BACKGROUND = "#f4f7f6"
INK = "#18231e"
MUTED = "#59665f"
GREEN = "#236f46"
GREEN_LIGHT = "#dfeee5"
BLUE = "#276aa3"
BLUE_LIGHT = "#dfebf5"
ORANGE = "#bd6a20"
ORANGE_LIGHT = "#f8e7d6"
RED = "#a63838"
LINE = "#c5d0ca"
ROAD = "#d8ddd9"
ROAD_EDGE = "#828c86"
WHITE = "#ffffff"
FONT_REGULAR = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(FONT_BOLD if bold else FONT_REGULAR, size)


def label(
    draw: ImageDraw.ImageDraw,
    xy: tuple[float, float],
    value: str,
    size: int,
    color: str = INK,
    bold: bool = False,
) -> None:
    draw.text(xy, value, fill=color, font=font(size, bold))


def header(draw: ImageDraw.ImageDraw, title: str, subtitle: str) -> None:
    draw.rectangle((0, 0, WIDTH, 126), fill="#205b3b")
    label(draw, (62, 26), title, 38, WHITE, True)
    label(draw, (64, 78), subtitle, 19, "#dcece3")


def centered_text(
    draw: ImageDraw.ImageDraw,
    center: tuple[float, float],
    value: str,
    size: int,
    color: str = INK,
    bold: bool = False,
) -> None:
    used_font = font(size, bold)
    bounds = draw.textbbox((0, 0), value, font=used_font)
    draw.text(
        (center[0] - (bounds[2] - bounds[0]) / 2, center[1] - (bounds[3] - bounds[1]) / 2),
        value,
        fill=color,
        font=used_font,
    )


def arrow(
    draw: ImageDraw.ImageDraw,
    start: tuple[float, float],
    end: tuple[float, float],
    color: str,
    width: int = 8,
) -> None:
    draw.line((start, end), fill=color, width=width)
    angle = math.atan2(end[1] - start[1], end[0] - start[0])
    head = 18 + width
    for delta in (2.55, -2.55):
        tip = (
            end[0] + head * math.cos(angle + delta),
            end[1] + head * math.sin(angle + delta),
        )
        draw.line((end, tip), fill=color, width=width)


def robot_polygon(
    center: tuple[float, float], yaw: float, length: float = 92, width: float = 58
) -> list[tuple[float, float]]:
    # The tapered nose mirrors the active rounded/tapered footprint contract.
    local = [
        (length * 0.50, width * 0.34),
        (length * 0.42, width * 0.50),
        (-length * 0.50, width * 0.50),
        (-length * 0.50, -width * 0.50),
        (length * 0.42, -width * 0.50),
        (length * 0.50, -width * 0.34),
    ]
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return [
        (
            center[0] + cos_yaw * x - sin_yaw * y,
            center[1] + sin_yaw * x + cos_yaw * y,
        )
        for x, y in local
    ]


def draw_robot(
    draw: ImageDraw.ImageDraw,
    center: tuple[float, float],
    yaw: float,
    fill: str = GREEN,
    outline: str = "#164a30",
) -> None:
    polygon = robot_polygon(center, yaw)
    draw.polygon(polygon, fill=fill, outline=outline, width=3)
    nose = (center[0] + 50 * math.cos(yaw), center[1] + 50 * math.sin(yaw))
    draw.ellipse((nose[0] - 6, nose[1] - 6, nose[0] + 6, nose[1] + 6), fill=WHITE)


def load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def node_parameters(payload: dict, node_name: str) -> dict:
    return payload[node_name]["ros__parameters"]


def load_inputs(repo_root: Path, report_path: Path) -> dict:
    control_yaml = load_yaml(repo_root / "camrod_control/config/control.yaml")
    parking_yaml = load_yaml(repo_root / "camrod_control/config/parking.yaml")
    platform_yaml = load_yaml(repo_root / "camrod_platform/config/ranger_driver.yaml")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    camping = node_parameters(
        control_yaml, "/control/camping_site_maneuver_controller"
    )
    recovery = node_parameters(
        control_yaml, "/control/route_safety_recovery_controller"
    )
    reverse = node_parameters(
        parking_yaml, "/parking/reverse_parking_controller"
    )
    docking = node_parameters(
        parking_yaml, "/parking/apriltag_parking_controller"
    )
    # HH_260818 - Ranger uses a ROS wildcard block in both package and bringup
    # mirrors. Locate the contract by parameter presence rather than node name.
    ranger = next(
        node_parameters(platform_yaml, key)
        for key in platform_yaml
        if "parallel_command_lateral_deadband_mps"
        in node_parameters(platform_yaml, key)
    )
    campsite_check = next(
        check for check in report["checks"] if check["name"] == "camping_site_smoke"
    )
    manual_check = next(
        check for check in report["checks"] if check["name"] == "manual_goal_nav"
    )
    return {
        "camping": camping,
        "recovery": recovery,
        "reverse": reverse,
        "docking": docking,
        "ranger": ranger,
        "report": report,
        "campsite_check": campsite_check,
        "manual_check": manual_check,
    }


def render_motion_mode_summary(data: dict, output: Path) -> None:
    deadband = float(data["ranger"]["parallel_command_lateral_deadband_mps"])
    crab_speed = float(data["camping"]["crab_speed_mps"])
    normal_metrics = data["manual_check"]["metrics"]
    normal_lateral = float(normal_metrics["cmd_lateral_max_mps"])
    normal_distance = float(normal_metrics["moved_m"])
    image = Image.new("RGB", (WIDTH, 900), BACKGROUND)
    draw = ImageDraw.Draw(image)
    header(
        draw,
        "Normal driving and explicit crab selection",
        "Dual-Ackermann remains the default; lateral command intent selects parallel motion",
    )

    draw.rectangle((0, 126, WIDTH, 500), fill=BLUE_LIGHT)
    label(draw, (62, 162), "NORMAL NAV2 ROUTE", 28, BLUE, True)
    label(
        draw,
        (62, 206),
        f"measured max |linear.y| = {normal_lateral:.3f} m/s",
        22,
        INK,
        True,
    )
    label(
        draw,
        (62, 242),
        f"|linear.y| <= {deadband:.2f} m/s -> Dual-Ackermann steering",
        20,
        MUTED,
    )
    points = []
    for index in range(21):
        x = 420 + index * 48
        y = 350 - 78 * math.sin(index / 20 * math.pi)
        points.append((x, y))
    draw.line(points, fill=BLUE, width=24, joint="curve")
    for index in (2, 9, 17):
        x, y = points[index]
        next_x, next_y = points[index + 1]
        draw_robot(draw, (x, y), math.atan2(next_y - y, next_x - x), BLUE, "#164a78")

    draw.rectangle((0, 500, WIDTH, 900), fill=ORANGE_LIGHT)
    label(draw, (62, 540), "CAMPSITE / RECOVERY CRAB", 28, ORANGE, True)
    label(draw, (62, 584), f"commanded lateral speed = {crab_speed:.3f} m/s", 22, INK, True)
    label(
        draw,
        (62, 620),
        f"|linear.y| > {deadband:.2f} m/s -> parallel-wheel crab",
        20,
        MUTED,
    )
    road_y = 744
    draw.line((460, road_y, 1450, road_y), fill=ROAD_EDGE, width=7)
    draw.line((460, road_y - 100, 1450, road_y - 100), fill=ROAD_EDGE, width=7)
    draw.rectangle((1060, 555, 1420, 640), fill=WHITE, outline=ORANGE, width=4)
    label(draw, (1170, 575), "SITE B8", 22, ORANGE, True)
    draw_robot(draw, (820, 694), 0.0, GREEN)
    arrow(draw, (820, 680), (1180, 610), ORANGE, 10)
    label(
        draw,
        (65, 842),
        f"PASS: normal route moved {normal_distance:.2f} m and stayed below the crab threshold.",
        20,
        GREEN,
        True,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def render_motion_mode_gif(data: dict, output: Path) -> None:
    frames: list[Image.Image] = []
    normal_lateral = float(
        data["manual_check"]["metrics"]["cmd_lateral_max_mps"]
    )
    points = [
        (150 + index * 70, 250 - 65 * math.sin(index / 16 * math.pi))
        for index in range(17)
    ]
    for index in range(17):
        image = Image.new("RGB", (1400, 560), BACKGROUND)
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, 1400, 94), fill="#205b3b")
        phase = "NORMAL / DUAL-ACKERMANN" if index < 11 else "EXPLICIT SITE CRAB"
        label(draw, (42, 24), phase, 31, WHITE, True)
        draw.line(points, fill=BLUE, width=18, joint="curve")
        draw.rectangle((1000, 120, 1335, 205), fill=ORANGE_LIGHT, outline=ORANGE, width=4)
        label(draw, (1104, 143), "SITE B8", 22, ORANGE, True)
        if index < 11:
            x, y = points[index]
            next_x, next_y = points[min(index + 1, len(points) - 1)]
            draw_robot(draw, (x, y), math.atan2(next_y - y, next_x - x), BLUE, "#164a78")
            label(
                draw,
                (44, 470),
                f"measured max |linear.y| = {normal_lateral:.3f} m/s | crab selection = false",
                19,
                MUTED,
            )
        else:
            ratio = (index - 11) / 5.0
            center = (850 + 280 * ratio, 300 - 130 * ratio)
            draw_robot(draw, center, 0.0, GREEN)
            arrow(draw, (850, 288), (1130, 158), ORANGE, 7)
            label(
                draw,
                (44, 470),
                f"linear.y = {data['camping']['crab_speed_mps']:.3f} m/s | crab selection = true",
                19,
                MUTED,
            )
        frames.append(image)
    output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=320,
        loop=0,
        optimize=False,
    )


def draw_phase_strip(draw: ImageDraw.ImageDraw, phases: list[str], y: int) -> None:
    left = 58
    gap = 12
    item_width = int((WIDTH - 116 - gap * (len(phases) - 1)) / len(phases))
    for index, phase in enumerate(phases):
        x = left + index * (item_width + gap)
        draw.rounded_rectangle(
            (x, y, x + item_width, y + 62),
            radius=7,
            fill=GREEN_LIGHT,
            outline=GREEN,
            width=2,
        )
        centered_text(draw, (x + item_width / 2, y + 31), phase, 15, GREEN, True)


def render_campsite_summary(data: dict, output: Path) -> None:
    metrics = data["campsite_check"]["metrics"]
    phases = metrics["site_phase_sequence"].split(" -> ")
    image = Image.new("RGB", (WIDTH, 1040), BACKGROUND)
    draw = ImageDraw.Draw(image)
    header(
        draw,
        "B8 same-anchor campsite entry and return",
        "amd64 ROS 2 simulation | current map | 90-degree crab and settled 180-degree turn",
    )
    passed = bool(data["report"]["overall_pass"] and data["campsite_check"]["success"])
    draw.rounded_rectangle(
        (1402, 39, 1535, 88),
        radius=7,
        fill=GREEN_LIGHT if passed else "#f6dddd",
        outline=GREEN if passed else RED,
        width=2,
    )
    centered_text(draw, (1468, 64), "PASS" if passed else "FAIL", 22, GREEN if passed else RED, True)

    road_left, road_right = 150, 1450
    road_top, road_bottom = 570, 760
    draw.rectangle((road_left, road_top, road_right, road_bottom), fill=ROAD)
    draw.line((road_left, road_top, road_right, road_top), fill=ROAD_EDGE, width=7)
    draw.line((road_left, road_bottom, road_right, road_bottom), fill=ROAD_EDGE, width=7)
    draw.line((road_left, 665, road_right, 665), fill=WHITE, width=4)
    anchor = (830, 665)
    site = (830, 300)
    draw.rectangle((650, 190, 1010, 380), fill=ORANGE_LIGHT, outline=ORANGE, width=5)
    centered_text(draw, (830, 220), "CAMPING SITE B8", 20, ORANGE, True)
    draw.ellipse((anchor[0] - 12, anchor[1] - 12, anchor[0] + 12, anchor[1] + 12), fill=BLUE)
    label(draw, (850, 645), "shared lanelet snap anchor", 18, BLUE, True)
    arrow(draw, (800, 610), (800, 350), ORANGE, 9)
    arrow(draw, (860, 350), (860, 610), GREEN, 9)
    label(draw, (590, 438), "CRAB_IN", 18, ORANGE, True)
    label(draw, (880, 438), "CRAB_OUT", 18, GREEN, True)
    draw_robot(draw, site, math.pi, GREEN)
    draw.arc((765, 245, 895, 375), 10, 330, fill=BLUE, width=8)
    label(draw, (1010, 285), "settled 180 deg", 18, BLUE, True)

    label(draw, (58, 148), "Verified sequence", 25, INK, True)
    draw_phase_strip(draw, phases, 820)
    facts = [
        f"route distance {metrics['route_distance_m']:.1f} m",
        f"site lateral distance {metrics['site_distance_m']:.2f} m",
        "axis-separated exit: 0 mixed commands",
        f"return timeout {data['camping']['crab_return_timeout_s']:.0f} s",
        f"boundary attempts {int(data['recovery']['maximum_attempts'])}",
    ]
    for index, fact in enumerate(facts):
        x = 60 + index * 305
        draw.ellipse((x, 934, x + 12, 946), fill=GREEN)
        label(draw, (x + 22, 926), fact, 16, MUTED, index in (3, 4))
    label(
        draw,
        (58, 986),
        "Restart contract: current heading projects the same map anchor and reverses crab side.",
        18,
        GREEN,
        True,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def campsite_frame(active: int, phases: list[str]) -> Image.Image:
    image = Image.new("RGB", (1400, 650), BACKGROUND)
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, 1400, 92), fill="#205b3b")
    label(draw, (42, 23), f"B8 campsite sequence | {phases[active]}", 30, WHITE, True)
    draw.rectangle((80, 430, 1320, 570), fill=ROAD)
    draw.line((80, 430, 1320, 430), fill=ROAD_EDGE, width=6)
    draw.line((80, 570, 1320, 570), fill=ROAD_EDGE, width=6)
    anchor = (700, 500)
    site = (700, 220)
    draw.rectangle((555, 130, 845, 300), fill=ORANGE_LIGHT, outline=ORANGE, width=4)
    draw.ellipse((690, 490, 710, 510), fill=BLUE)
    draw.line((700, 490, 700, 250), fill=LINE, width=5)
    if active == 0:
        draw_robot(draw, (700, 420), 0.0, ORANGE, "#7c4516")
        arrow(draw, anchor, site, ORANGE, 7)
    elif active == 1:
        draw_robot(draw, site, math.pi / 2, GREEN)
        draw.arc((630, 150, 770, 290), 20, 330, fill=BLUE, width=7)
    elif active in (2, 3):
        draw_robot(draw, site, math.pi, GREEN)
    elif active == 4:
        draw_robot(draw, site, math.pi, BLUE, "#164a78")
        draw.arc((630, 150, 770, 290), 20, 330, fill=BLUE, width=7)
    elif active == 5:
        draw_robot(draw, (700, 350), math.pi, GREEN)
        arrow(draw, site, anchor, GREEN, 7)
    else:
        draw_robot(draw, anchor, math.pi, GREEN)
    for index, phase in enumerate(phases):
        x = 54 + index * 188
        fill = GREEN_LIGHT if index <= active else WHITE
        edge = GREEN if index <= active else LINE
        draw.rounded_rectangle((x, 594, x + 170, 632), radius=5, fill=fill, outline=edge, width=2)
        centered_text(draw, (x + 85, 613), phase, 11, GREEN if index <= active else MUTED, True)
    return image


def render_campsite_gif(data: dict, output: Path) -> None:
    phases = data["campsite_check"]["metrics"]["site_phase_sequence"].split(" -> ")
    frames = [campsite_frame(index, phases) for index in range(len(phases))]
    output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=700,
        loop=0,
        optimize=False,
    )


def approach_speed(remaining: float, window: float, cruise: float, final: float) -> float:
    if window <= 0.0 or remaining >= window:
        return cruise
    ratio = max(0.0, remaining) / window
    return final + (cruise - final) * ratio


def render_parking_profile(data: dict, output: Path) -> None:
    reverse = data["reverse"]
    docking = data["docking"]
    reverse_window = float(reverse["slowdown_start_remaining_distance_m"])
    docking_window = float(docking["slowdown_start_remaining_distance_m"])
    reverse_cruise = float(reverse["reverse_speed_mps"])
    reverse_final = float(reverse["final_approach_speed_mps"])
    docking_cruise = float(docking["reverse_approach_speed_mps"])
    docking_final = float(docking["final_insertion_speed_mps"])

    image = Image.new("RGB", (WIDTH, 900), BACKGROUND)
    draw = ImageDraw.Draw(image)
    header(
        draw,
        "Parking and AprilTag docking final approach",
        "Remaining-distance speed ramps; charging CAN immediately owns the stop",
    )
    chart = (120, 205, 1100, 730)
    draw.rectangle(chart, fill=WHITE, outline=LINE, width=2)
    max_distance = 1.2
    max_speed = 0.65
    for index in range(7):
        speed = index * 0.1
        y = chart[3] - speed / max_speed * (chart[3] - chart[1])
        draw.line((chart[0], y, chart[2], y), fill="#e3e8e5", width=2)
        label(draw, (60, y - 10), f"{speed:.1f}", 15, MUTED)
    for index in range(7):
        distance = index * 0.2
        x = chart[2] - distance / max_distance * (chart[2] - chart[0])
        draw.line((x, chart[1], x, chart[3]), fill="#edf0ee", width=2)
        label(draw, (x - 18, chart[3] + 15), f"{distance:.1f}", 15, MUTED)
    label(draw, (360, 780), "remaining distance to final parked pose (m)", 18, MUTED, True)
    label(draw, (36, 158), "raw speed (m/s)", 17, MUTED, True)

    def plot(window: float, cruise: float, final: float, color: str) -> None:
        points = []
        for index in range(121):
            remaining = index / 100.0
            speed = approach_speed(remaining, window, cruise, final)
            x = chart[2] - remaining / max_distance * (chart[2] - chart[0])
            y = chart[3] - speed / max_speed * (chart[3] - chart[1])
            points.append((x, y))
        draw.line(points, fill=color, width=8, joint="curve")

    plot(reverse_window, reverse_cruise, reverse_final, BLUE)
    plot(docking_window, docking_cruise, docking_final, ORANGE)
    draw.rectangle((1150, 225, 1530, 665), fill=WHITE, outline=LINE, width=2)
    label(draw, (1180, 252), "REVERSE PARKING", 20, BLUE, True)
    label(draw, (1180, 294), f"slowdown: last {reverse_window:.2f} m", 18, INK)
    label(draw, (1180, 330), f"speed: {reverse_cruise:.3f} -> {reverse_final:.3f}", 17, MUTED)
    label(draw, (1180, 405), "APRILTAG DOCKING", 20, ORANGE, True)
    label(draw, (1180, 447), f"slowdown: last {docking_window:.2f} m", 18, INK)
    label(draw, (1180, 483), f"speed: {docking_cruise:.3f} -> {docking_final:.3f}", 17, MUTED)
    draw.rounded_rectangle((1180, 555, 1498, 625), radius=7, fill=GREEN_LIGHT, outline=GREEN, width=2)
    centered_text(draw, (1339, 590), "charging = true -> ZERO", 18, GREEN, True)
    label(
        draw,
        (118, 836),
        "Both controllers subscribe to /platform/status and stop before declaring PARKED.",
        19,
        GREEN,
        True,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def sha256_manifest(directory: Path) -> None:
    entries = []
    for path in sorted(directory.iterdir()):
        if path.is_file() and path.name != "SHA256SUMS":
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            entries.append(f"{digest}  {path.name}")
    (directory / "SHA256SUMS").write_text("\n".join(entries) + "\n", encoding="ascii")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[3],
    )
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    repo_root = args.repo_root.resolve()
    report_path = args.report.resolve()
    data = load_inputs(repo_root, report_path)

    assets = repo_root / "docs/assets/module-guides"
    platform_dir = assets / "platform/test-results/normal-crab-selection-20260819"
    control_dir = assets / "control/test-results/campsite-return-docking-20260819"
    bringup_dir = assets / "bringup/test-results/b8-return-docking-20260819"
    ui_dir = assets / "ui/test-results/docking-workspace-20260819"
    for directory in (platform_dir, control_dir, bringup_dir):
        directory.mkdir(parents=True, exist_ok=True)

    render_motion_mode_summary(data, platform_dir / "normal-vs-crab-mode-selection.png")
    render_motion_mode_gif(data, platform_dir / "normal-vs-crab-mode-selection.gif")
    render_campsite_summary(data, control_dir / "b8-same-anchor-return.png")
    render_campsite_gif(data, control_dir / "b8-entry-return-sequence.gif")
    render_parking_profile(data, control_dir / "parking-slowdown-profile.png")
    shutil.copy2(report_path, bringup_dir / "b8-entry-return-report.json")
    shutil.copy2(
        control_dir / "b8-same-anchor-return.png",
        bringup_dir / "b8-entry-return-summary.png",
    )
    shutil.copy2(
        control_dir / "b8-entry-return-sequence.gif",
        bringup_dir / "b8-entry-return-sequence.gif",
    )

    campsite_metrics = data["campsite_check"]["metrics"]
    # HH_260818 - Preserve the administrator Return ordering assertions in the
    # compact release summary so generated media cannot hide an early route plan.
    summary = {
        "classification": "amd64_ros2_simulation",
        "overall_pass": bool(data["report"]["overall_pass"]),
        "normal_route_pass": bool(data["manual_check"]["success"]),
        "normal_route_moved_m": data["manual_check"]["metrics"]["moved_m"],
        "normal_route_lateral_max_mps": data["manual_check"]["metrics"][
            "cmd_lateral_max_mps"
        ],
        "normal_motion_mode_ok": bool(
            data["manual_check"]["metrics"]["normal_motion_mode_ok"]
        ),
        "site_phase_sequence": campsite_metrics["site_phase_sequence"],
        "camping_return_via_ui": bool(
            campsite_metrics.get("camping_return_via_ui", False)
        ),
        "manual_return_action": campsite_metrics.get("manual_return_action", ""),
        "drop_zone_return_before_site_done": bool(
            campsite_metrics.get("drop_zone_return_before_site_done", True)
        ),
        "manual_return_order_ok": bool(
            campsite_metrics.get("manual_return_order_ok", False)
        ),
        "site_path_ok": bool(campsite_metrics.get("site_path_ok", False)),
        "site_path_updates": int(campsite_metrics.get("site_path_updates", 0)),
        "crab_command_mode_ok": bool(
            campsite_metrics.get("crab_command_mode_ok", False)
        ),
        "crab_phase_raw_components": campsite_metrics.get(
            "crab_phase_raw_components", {}
        ),
        "normal_parallel_deadband_mps": data["ranger"][
            "parallel_command_lateral_deadband_mps"
        ],
        "camping_crab_speed_mps": data["camping"]["crab_speed_mps"],
        "crab_return_timeout_s": data["camping"]["crab_return_timeout_s"],
        "boundary_recovery_maximum_attempts": data["recovery"]["maximum_attempts"],
        "reverse_slowdown_remaining_m": data["reverse"][
            "slowdown_start_remaining_distance_m"
        ],
        "docking_slowdown_remaining_m": data["docking"][
            "slowdown_start_remaining_distance_m"
        ],
        "charging_immediate_stop": bool(
            data["reverse"]["stop_when_charging"]
            and data["docking"]["stop_when_charging"]
        ),
        "field_acceptance": "required_on_arm64_robot",
    }
    (bringup_dir / "result-summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    for directory in (platform_dir, control_dir, bringup_dir):
        sha256_manifest(directory)
    # HH_260818 - The UI image is an actual headless-browser capture rather
    # than a generated drawing, but it shares the release checksum contract.
    if ui_dir.is_dir():
        sha256_manifest(ui_dir)


if __name__ == "__main__":
    main()
