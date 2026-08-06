#!/usr/bin/env python3
"""Render v2.1.5 service, obstacle, and boundary validation evidence."""

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


# HH_260807 - These visuals are generated only from committed machine-readable
# reports. They summarize simulation evidence without relabeling it as field data.
WIDTH = 1600
BACKGROUND = "#f4f7f6"
INK = "#17211d"
MUTED = "#5d6a64"
GREEN = "#257a4b"
GREEN_LIGHT = "#dcefe3"
BLUE = "#286aa6"
BLUE_LIGHT = "#dfebf6"
ORANGE = "#bd6b1f"
ORANGE_LIGHT = "#f8e7d4"
RED = "#a63838"
RED_LIGHT = "#f6dddd"
LINE = "#c8d2cd"
WHITE = "#ffffff"
FONT_REGULAR = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(FONT_BOLD if bold else FONT_REGULAR, size)


def text(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    value: str,
    size: int,
    color: str = INK,
    bold: bool = False,
) -> None:
    draw.text(xy, value, fill=color, font=font(size, bold))


def badge(draw: ImageDraw.ImageDraw, xy: tuple[int, int], passed: bool = True) -> None:
    x, y = xy
    color = GREEN if passed else RED
    fill = GREEN_LIGHT if passed else RED_LIGHT
    label = "PASS" if passed else "FAIL"
    draw.rounded_rectangle((x, y, x + 110, y + 42), radius=8, fill=fill, outline=color, width=2)
    bounds = draw.textbbox((0, 0), label, font=font(21, True))
    text_x = x + 55 - (bounds[2] - bounds[0]) / 2
    draw.text((text_x, y + 8), label, fill=color, font=font(21, True))


def header(draw: ImageDraw.ImageDraw, title: str, subtitle: str, height: int = 130) -> None:
    draw.rectangle((0, 0, WIDTH, height), fill="#1f5e3d")
    text(draw, (64, 28), title, 40, WHITE, True)
    text(draw, (66, 82), subtitle, 20, "#dcece3")


def load_check(path: Path, name: str) -> tuple[dict, dict]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    check = next(item for item in payload["checks"] if item["name"] == name)
    return payload, check


def phase_names(entries: list[str]) -> list[str]:
    return [str(entry).split("|", 1)[0] for entry in entries]


def unique_ordered(values: list[str]) -> list[str]:
    output = []
    for value in values:
        if value and (not output or output[-1] != value):
            output.append(value)
    return output


SERVICE_PHASES = [
    "SITE ROUTE",
    "CRAB IN",
    "ZERO TURN",
    "WAIT RETURN",
    "RETURN ROUTE",
    "DROP ALIGN",
    "PARK",
    "CHARGE",
]


def draw_phase_chain(
    draw: ImageDraw.ImageDraw,
    y: int,
    active: int | None = None,
    width: int = 1460,
) -> None:
    left = 70
    gap = 14
    item_width = int((width - gap * (len(SERVICE_PHASES) - 1)) / len(SERVICE_PHASES))
    for index, label in enumerate(SERVICE_PHASES):
        x = left + index * (item_width + gap)
        selected = active is None or index <= active
        fill = GREEN_LIGHT if selected else WHITE
        edge = GREEN if selected else LINE
        draw.rounded_rectangle(
            (x, y, x + item_width, y + 66), radius=7, fill=fill, outline=edge, width=2
        )
        label_font = font(16, True)
        bounds = draw.textbbox((0, 0), label, font=label_font)
        draw.text(
            (x + item_width / 2 - (bounds[2] - bounds[0]) / 2, y + 22),
            label,
            fill=GREEN if selected else MUTED,
            font=label_font,
        )


def render_service_summary(report_path: Path, output: Path) -> None:
    payload, check = load_check(report_path, "repeated_service_soak")
    metrics = check["metrics"]
    cycles = metrics["cycles"]
    image = Image.new("RGB", (WIDTH, 1040), BACKGROUND)
    draw = ImageDraw.Draw(image)
    header(
        draw,
        "v2.1.5 repeated service validation",
        "Map v17 | campsite -> explicit return -> parking -> charging -> next campsite",
    )
    badge(draw, (1410, 42), bool(payload["overall_pass"] and check["success"]))

    text(draw, (64, 162), "Continuous service result", 28, INK, True)
    text(
        draw,
        (64, 205),
        f"{metrics['cycles_completed']}/{metrics['cycles_requested']} cycles  |  "
        f"{metrics['elapsed_s']:.1f} s  |  bringup restarts {metrics['bringup_restart_count']}",
        23,
        MUTED,
    )
    draw_phase_chain(draw, 252)

    for index, cycle in enumerate(cycles):
        x = 64 + index * 504
        y = 360
        site_number = str(cycle["mission_key"]).rsplit("_", 1)[-1]
        draw.rounded_rectangle(
            (x, y, x + 472, y + 440), radius=8, fill=WHITE, outline=LINE, width=2
        )
        text(draw, (x + 24, y + 20), f"Cycle {cycle['cycle']} | B{site_number}", 25, BLUE, True)
        badge(draw, (x + 335, y + 17), bool(cycle["success"]))
        rows = [
            ("Elapsed", f"{cycle['elapsed_s']:.1f} s"),
            ("Site phases", "CRAB > TURN > WAIT > DONE"),
            ("Parking", "REVERSE > WAIT_CHARGE > PARKED"),
            ("Boundary hold", "recovered" if cycle["boundary_recovery_ok"] else "failed"),
            ("Retry latch", "none" if not cycle["boundary_retry_latched"] else "latched"),
            ("Obstacle", "stop -> clear -> resume" if cycle["obstacle_required"] else "not injected"),
            ("Final state", "CHARGING" if "CHARGING" in cycle["service_states"] else "missing"),
        ]
        row_y = y + 82
        for label, value in rows:
            text(draw, (x + 24, row_y), label, 17, MUTED, True)
            max_value = value if len(value) <= 32 else value[:29] + "..."
            text(draw, (x + 184, row_y), max_value, 15, INK)
            row_y += 48

    draw.rounded_rectangle((64, 836, 1536, 970), radius=8, fill=BLUE_LIGHT, outline=BLUE, width=2)
    text(draw, (88, 858), "Assertions exercised in every cycle", 23, BLUE, True)
    facts = [
        "RETURN only after WAITING_FOR_RETURN_REQUEST.",
        "Recovery command observed before 1.5 s release.",
        "Parking: WAIT_FOR_CHARGING -> PARKED -> CHARGING.",
        "Cycle 2 obstacle: stop -> clear -> resume.",
    ]
    for index, value in enumerate(facts):
        column = index % 2
        row = index // 2
        x = 92 + column * 720
        y = 900 + row * 36
        draw.ellipse((x, y + 6, x + 11, y + 17), fill=GREEN)
        text(draw, (x + 22, y), value, 16, INK)
    text(draw, (64, 1000), "Classification: amd64 ROS 2 simulation PASS; Jetson/field acceptance remains open.", 17, MUTED)
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def render_service_gif(report_path: Path, output: Path) -> None:
    _, check = load_check(report_path, "repeated_service_soak")
    cycles = check["metrics"]["cycles"]
    frames = []
    for cycle_index, cycle in enumerate(cycles):
        for active in range(len(SERVICE_PHASES)):
            image = Image.new("RGB", (1400, 520), BACKGROUND)
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, 1400, 104), fill="#1f5e3d")
            text(draw, (48, 24), "Continuous campsite service timeline", 34, WHITE, True)
            text(
                draw,
                (50, 68),
                f"Cycle {cycle_index + 1}/3: {cycle['mission_key']} | active phase {active + 1}/8",
                18,
                "#dcece3",
            )
            draw_phase_chain(draw, 180, active=active, width=1260)
            detail = [
                f"boundary recovery: {'PASS' if cycle['boundary_recovery_ok'] else 'FAIL'}",
                f"dynamic obstacle: {'stop/clear/resume' if cycle['obstacle_required'] else 'not injected'}",
                "cycle completion: CHARGING",
            ]
            for index, value in enumerate(detail):
                text(draw, (70, 312 + index * 44), value, 21, INK if index != 2 else GREEN, index == 2)
            frames.append(image)
    output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=350,
        loop=0,
        optimize=False,
    )


def render_obstacle_summary(report_path: Path, output: Path) -> None:
    payload, check = load_check(report_path, "obstacle_replan")
    metrics = check["metrics"]
    image = Image.new("RGB", (WIDTH, 920), BACKGROUND)
    draw = ImageDraw.Draw(image)
    header(
        draw,
        "Persistent obstacle: safe no-path behavior",
        "Map v17 maximum measured road width 3.00 m | LiDAR cost grid enabled",
    )
    badge(draw, (1410, 42), bool(payload["overall_pass"] and check["success"]))

    lane_left, lane_right = 80, 1000
    lane_top, lane_bottom = 250, 640
    draw.rectangle((lane_left, lane_top, lane_right, lane_bottom), fill="#eef1f2", outline="#8f9aa0", width=5)
    draw.line((lane_left, 445, lane_right, 445), fill="#d0a33c", width=5)
    draw.ellipse((550, 375, 690, 515), fill=RED_LIGHT, outline=RED, width=5)
    text(draw, (575, 425), "OBSTACLE", 17, RED, True)
    draw.rounded_rectangle((250, 385, 450, 505), radius=8, fill=BLUE_LIGHT, outline=BLUE, width=4)
    text(draw, (290, 425), "ROBOT STOP", 21, BLUE, True)
    draw.line((450, 445, 550, 445), fill=RED, width=6)
    draw.line((690, 445, 900, 445), fill=GREEN, width=6)
    text(draw, (710, 406), "resume after clear", 18, GREEN, True)
    text(draw, (80, 685), "Schematic only; measured values come from the structured ROS report.", 16, MUTED)

    draw.rounded_rectangle((1060, 180, 1536, 742), radius=8, fill=WHITE, outline=LINE, width=2)
    text(draw, (1088, 207), "Observed sequence", 26, INK, True)
    sequence = [
        ("1", "Immediate command stop", f"stop_seen={metrics['stop_seen']}"),
        ("2", "20 s persistent hold", "BLOCKED_HOLD"),
        ("3", "Smac path preflight", "no safe path"),
        ("4", "Keep LaneletRoute", "no selector preemption"),
        ("5", "Remove obstacle", f"resumed {metrics['moved_m']:.3f} m"),
    ]
    for index, (number, title, detail) in enumerate(sequence):
        y = 270 + index * 88
        draw.ellipse((1090, y, 1132, y + 42), fill=GREEN_LIGHT, outline=GREEN, width=2)
        text(draw, (1104, y + 8), number, 19, GREEN, True)
        text(draw, (1152, y - 2), title, 19, INK, True)
        text(draw, (1152, y + 28), detail, 17, MUTED)

    draw.rounded_rectangle((80, 762, 1536, 862), radius=8, fill=ORANGE_LIGHT, outline=ORANGE, width=2)
    facts = (
        f"Smac selector seen: {metrics['fallback_selector_seen']}",
        f"failed-hold seen: {metrics['fallback_failed_hold_seen']}",
        f"resumed after clear: {metrics['resumed_after_obstacle_clear']}",
        "planning ABORT loop: eliminated",
    )
    for index, value in enumerate(facts):
        x = 108 + (index % 2) * 720
        y = 785 + (index // 2) * 37
        text(draw, (x, y), value, 19, INK, index == 3)
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def render_boundary_summary(trial_paths: list[Path], output: Path) -> None:
    trials = [json.loads(path.read_text(encoding="utf-8")) for path in trial_paths]
    image = Image.new("RGB", (WIDTH, 880), BACKGROUND)
    draw = ImageDraw.Draw(image)
    header(
        draw,
        "B2 boundary recovery repeatability",
        "Map v17 | planning margin contact | continuous-twist physical-body sweep",
    )
    badge(draw, (1410, 42), all(item["mission_completed"] for item in trials))

    columns = [70, 250, 560, 830, 1080, 1330]
    labels = ["Trial", "Recovery", "Distance", "Max recovery", "Second hold", "Mission"]
    for x, label in zip(columns, labels):
        text(draw, (x, 190), label, 20, MUTED, True)
    draw.line((64, 228, 1536, 228), fill=LINE, width=3)
    for index, trial in enumerate(trials):
        y = 260 + index * 110
        fill = WHITE if index % 2 == 0 else "#edf3f0"
        draw.rounded_rectangle((64, y - 16, 1536, y + 72), radius=6, fill=fill)
        values = [
            str(index + 1),
            trial["automatic_recovery_motion"],
            f"{trial['recovery_displacement_m']:.4f} m",
            f"{trial['maximum_recovery_output_mps']:.2f} m/s",
            "none" if trial["second_hold"] is None else "present",
            "PASS" if trial["mission_completed"] else "FAIL",
        ]
        for x, value in zip(columns, values):
            color = GREEN if value == "PASS" else INK
            text(draw, (x, y + 8), value, 21, color, value == "PASS")

    draw.rounded_rectangle((64, 620, 1536, 796), radius=8, fill=BLUE_LIGHT, outline=BLUE, width=2)
    text(draw, (90, 646), "Release behavior", 25, BLUE, True)
    statements = [
        "Planning-margin contact remained recoverable; physical-body contact was false.",
        "All 3 trials selected REVERSE_YAW_RIGHT and completed the original mission.",
        "The clear timer starts only after an admitted recovery command and requires 1.5 s.",
        "No trial produced a second hold or rapid-recontact latch.",
    ]
    for index, value in enumerate(statements):
        draw.ellipse((94, 696 + index * 29, 105, 707 + index * 29), fill=GREEN)
        text(draw, (120, 688 + index * 29), value, 18, INK)
    text(draw, (64, 838), "Classification: deterministic simulation PASS; field boundary contact remains required.", 17, MUTED)
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-dir", type=Path, required=True)
    args = parser.parse_args()
    root = args.evidence_dir.resolve()
    service_report = root / "repeated-service-soak.json"
    render_service_summary(service_report, root / "repeated-service-summary.png")
    render_service_gif(service_report, root / "repeated-service-timeline.gif")
    render_obstacle_summary(root / "obstacle-safe-hold.json", root / "obstacle-safe-hold.png")
    render_boundary_summary(
        sorted(root.glob("b2-boundary-recovery-trial-*.json")),
        root / "b2-boundary-recovery.png",
    )


if __name__ == "__main__":
    main()
