#!/usr/bin/env python3
"""Render campsite maneuver policy evidence from structured sim reports."""

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
import yaml


# HH_260806 - Keep release visuals reproducible from checked-in reports and
# active campsite policy instead of hand-editing screenshots.
WIDTH = 1600
HEIGHT = 1120
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
FONT_REGULAR = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(FONT_BOLD if bold else FONT_REGULAR, size)


def load_metrics(path: Path) -> tuple[bool, dict]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    check = next(item for item in payload["checks"] if item["name"] == "camping_site_smoke")
    return bool(payload["overall_pass"] and check["success"]), check["metrics"]


def phase_sequence(metrics: dict) -> list[str]:
    raw = str(metrics.get("site_phase_sequence", "")).strip()
    if raw:
        return [item.strip() for item in raw.split("->") if item.strip()]
    if metrics.get("service_mode") == "roadside_stop":
        return ["CRAB_IN", "UNLOAD_WAIT", "WAIT_RETURN"]
    sequence = ["CRAB_IN"]
    if metrics.get("rotate_180"):
        sequence.append("ROTATE_180")
    sequence.extend(["UNLOAD_WAIT", "WAIT_RETURN"])
    if metrics.get("align_retrace_yaw"):
        sequence.append("ALIGN_RETRACE_YAW")
    if metrics.get("crab_out"):
        sequence.append("CRAB_OUT")
    if metrics.get("done"):
        sequence.append("DONE")
    return sequence


def load_policy(path: Path) -> dict[str, str]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    return {
        str(site["type"]): str(site.get("service_mode", "turnaround"))
        for site in payload["camping_sites"]
    }


def draw_text(draw: ImageDraw.ImageDraw, xy: tuple[int, int], text: str, size: int,
              color: str = INK, bold: bool = False) -> None:
    draw.text(xy, text, fill=color, font=font(size, bold))


def fitted_font(draw: ImageDraw.ImageDraw, text: str, maximum_size: int,
                maximum_width: int, bold: bool = False,
                minimum_size: int = 10) -> ImageFont.FreeTypeFont:
    size = maximum_size
    while size > minimum_size:
        candidate = font(size, bold)
        bounds = draw.textbbox((0, 0), text, font=candidate)
        if bounds[2] - bounds[0] <= maximum_width:
            return candidate
        size -= 1
    return font(minimum_size, bold)


def draw_badge(draw: ImageDraw.ImageDraw, xy: tuple[int, int], text: str,
               passed: bool) -> None:
    x, y = xy
    fill = GREEN_LIGHT if passed else RED_LIGHT
    color = GREEN if passed else RED
    draw.rounded_rectangle((x, y, x + 112, y + 42), radius=8, fill=fill, outline=color, width=2)
    label = "PASS" if passed else "FAIL"
    box = draw.textbbox((0, 0), label, font=font(22, True))
    draw.text((x + 56 - (box[2] - box[0]) / 2, y + 8), label, fill=color, font=font(22, True))


def draw_phase_row(draw: ImageDraw.ImageDraw, y: int, label: str, phases: list[str],
                   color: str, light: str) -> None:
    draw_text(draw, (72, y + 13), label, 24, INK, True)
    start_x = 330
    available = WIDTH - start_x - 220
    gap = 12
    box_width = int((available - gap * (len(phases) - 1)) / max(1, len(phases)))
    for index, phase in enumerate(phases):
        x = start_x + index * (box_width + gap)
        draw.rounded_rectangle(
            (x, y, x + box_width, y + 58), radius=8, fill=light, outline=color, width=2
        )
        display = phase.replace("_", " ")
        phase_font = fitted_font(draw, display, 17, box_width - 14, True)
        bounds = draw.textbbox((0, 0), display, font=phase_font)
        draw.text(
            (x + box_width / 2 - (bounds[2] - bounds[0]) / 2, y + 18),
            display,
            fill=color,
            font=phase_font,
        )


def draw_turnaround_site_grid(
    draw: ImageDraw.ImageDraw,
    reports: dict[str, tuple[bool, dict]],
    top: int,
) -> None:
    """Show every B1-B10 runtime result instead of only representative sites."""
    # HH_260806 - A release PASS now requires one direct report per drive-in site.
    gap = 16
    left = 64
    tile_width = int((WIDTH - 2 * left - 4 * gap) / 5)
    tile_height = 72
    for index in range(1, 11):
        row = (index - 1) // 5
        column = (index - 1) % 5
        x = left + column * (tile_width + gap)
        y = top + row * (tile_height + 14)
        passed, metrics = reports[f"B{index}"]
        color = GREEN if passed else RED
        light = GREEN_LIGHT if passed else RED_LIGHT
        draw.rounded_rectangle(
            (x, y, x + tile_width, y + tile_height),
            radius=8,
            fill="#ffffff",
            outline=color,
            width=2,
        )
        draw_text(draw, (x + 16, y + 18), f"B{index}", 24, BLUE, True)
        distance = float(metrics.get("site_distance_m", 0.0))
        draw_text(draw, (x + 78, y + 22), f"{distance:.2f} m", 18, MUTED)
        status = "PASS" if passed else "FAIL"
        status_font = fitted_font(draw, status, 18, 68, True)
        status_bounds = draw.textbbox((0, 0), status, font=status_font)
        badge_left = x + tile_width - 84
        draw.rounded_rectangle(
            (badge_left, y + 17, x + tile_width - 12, y + 54),
            radius=7,
            fill=light,
            outline=color,
            width=1,
        )
        draw.text(
            (
                badge_left + 36 - (status_bounds[2] - status_bounds[0]) / 2,
                y + 25,
            ),
            status,
            fill=color,
            font=status_font,
        )


def render_summary(output: Path, policy: dict[str, str], reports: dict[str, tuple[bool, dict]]) -> None:
    image = Image.new("RGB", (WIDTH, HEIGHT), BACKGROUND)
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, WIDTH, 132), fill="#1f5e3d")
    draw_text(draw, (64, 32), "Campsite maneuver validation", 42, "#ffffff", True)
    draw_text(
        draw,
        (66, 87),
        "Map v16 | command-source arbitration | optional LiDAR cost-grid OFF contract",
        22,
        "#dcece3",
    )

    draw_text(draw, (64, 166), "Active policy", 28, INK, True)
    turnaround_ok = all(policy.get(f"camping_site_{index}") == "turnaround" for index in range(1, 11))
    roadside_ok = all(policy.get(f"camping_site_{index}") == "roadside_stop" for index in range(11, 14))
    draw.rounded_rectangle((64, 210, 770, 310), radius=8, fill="#ffffff", outline=LINE, width=2)
    draw_text(draw, (88, 232), "B1-B10", 27, BLUE, True)
    draw_text(draw, (230, 235), "Crab-in + 180 deg zero-turn", 22)
    draw_badge(draw, (630, 235), "", turnaround_ok)
    draw.rounded_rectangle((830, 210, 1536, 310), radius=8, fill="#ffffff", outline=LINE, width=2)
    draw_text(draw, (854, 232), "B11-B13", 27, ORANGE, True)
    draw_text(draw, (1015, 237), "0.60 m roadside; no site turn", 19)
    draw_badge(draw, (1396, 235), "", roadside_ok)

    draw_text(draw, (64, 348), "B1-B10 full round-trip simulation", 28, INK, True)
    draw_text(draw, (720, 354), "tile value = map-derived lateral entry distance", 18, MUTED)
    draw_turnaround_site_grid(draw, reports, 394)

    turnaround_runtime_ok = all(reports[f"B{index}"][0] for index in range(1, 11))
    roadside_pass = all(reports[key][0] for key in ("B11", "B12", "B13"))
    draw_phase_row(
        draw,
        590,
        "B1-B10 turnaround",
        phase_sequence(reports["B1"][1]),
        BLUE,
        BLUE_LIGHT,
    )
    draw_badge(draw, (1418, 598), "", turnaround_runtime_ok)
    draw_phase_row(
        draw,
        678,
        "B11-B13 arrival",
        phase_sequence(reports["B11"][1]),
        ORANGE,
        ORANGE_LIGHT,
    )
    draw_badge(draw, (1418, 686), "", roadside_pass)

    draw.rounded_rectangle((64, 780, 1010, 1034), radius=8, fill="#ffffff", outline=LINE, width=2)
    draw_text(draw, (90, 806), "Command sequencing guarantees", 26, INK, True)
    guarantees = [
        "Maneuver phases exclusively own final cmd_vel; Nav2 input is ignored.",
        "Crab completion precedes ROTATE_180; rotation completion precedes unload wait.",
        "Maneuver release and Nav2 rotation-to-translation each hold zero for 0.5 s.",
        "Campsite lanelet bypass is phase-scoped; live LiDAR/radar stops remain active.",
    ]
    for index, item in enumerate(guarantees):
        y = 854 + index * 40
        draw.ellipse((92, y + 8, 104, y + 20), fill=GREEN)
        draw_text(draw, (120, y), item, 19, MUTED)

    draw.rounded_rectangle((1040, 780, 1536, 1034), radius=8, fill=ORANGE_LIGHT, outline=ORANGE, width=2)
    draw_text(draw, (1066, 806), "Field-pending return", 26, ORANGE, True)
    pending = [
        "B11-B13 tests stop at WAIT_RETURN.",
        "No RETURN or zero-turn is issued on site.",
        "The earlier on-lane return turn was blocked",
        "by physical-body lanelet cost in simulation.",
        "Choose return geometry after site survey.",
    ]
    for index, item in enumerate(pending):
        draw_text(draw, (1068, 854 + index * 31), item, 18, INK)

    draw_text(draw, (64, 1072), "Source: structured ROS 2 sim reports in this directory", 18, MUTED)
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def draw_timeline_frame(sequences: dict[str, list[str]], active_index: int) -> Image.Image:
    image = Image.new("RGB", (1400, 760), BACKGROUND)
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, 1400, 104), fill="#1f5e3d")
    draw_text(draw, (48, 24), "Campsite phase order: turnaround vs roadside arrival", 34, "#ffffff", True)
    draw_text(draw, (50, 69), "Runtime-backed phase contract; diagram geometry is schematic", 18, "#dcece3")

    rows = [
        ("B1-B10 turnaround", sequences["B10"], BLUE, BLUE_LIGHT, 180),
        ("B11 roadside arrival", sequences["B11"], ORANGE, ORANGE_LIGHT, 470),
    ]
    for label, phases, color, light, y in rows:
        draw_text(draw, (52, y - 48), label, 26, INK, True)
        draw.line((80, y + 32, 1320, y + 32), fill=LINE, width=8)
        spacing = 1200 / max(1, len(phases) - 1)
        current = min(active_index, len(phases) - 1)
        for index, phase in enumerate(phases):
            x = int(80 + spacing * index)
            active = index == current
            reached = index <= current
            radius = 25 if active else 19
            fill = color if reached else "#ffffff"
            draw.ellipse((x - radius, y + 32 - radius, x + radius, y + 32 + radius), fill=fill, outline=color, width=3)
            if reached:
                draw_text(draw, (x - 8, y + 17), str(index + 1), 19, "#ffffff", True)
            display = phase.replace("_", " ")
            label_font = font(15 if len(display) < 18 else 12, active)
            bounds = draw.textbbox((0, 0), display, font=label_font)
            draw.text((x - (bounds[2] - bounds[0]) / 2, y + 72), display, fill=color if active else MUTED, font=label_font)
        active_phase = phases[current]
        callout_fill = light
        draw.rounded_rectangle((490, y + 118, 910, y + 176), radius=8, fill=callout_fill, outline=color, width=2)
        draw_text(draw, (516, y + 133), f"Active: {active_phase}", 21, color, True)
    draw_text(draw, (52, 710), "B11-B13 return maneuver intentionally excluded pending field geometry decision.", 19, RED, True)
    return image


def render_timeline_gif(output: Path, reports: dict[str, tuple[bool, dict]]) -> None:
    sequences = {
        "B10": phase_sequence(reports["B10"][1]),
        "B11": phase_sequence(reports["B11"][1]),
    }
    frame_count = max(len(value) for value in sequences.values())
    frames = [draw_timeline_frame(sequences, index) for index in range(frame_count)]
    frames.extend([frames[-1].copy(), frames[-1].copy()])
    output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=850,
        loop=0,
        optimize=False,
    )


def write_summary(path: Path, policy: dict[str, str], reports: dict[str, tuple[bool, dict]]) -> None:
    payload = {
        "map_revision": 16,
        "policy": {
            "turnaround_sites": [f"B{index}" for index in range(1, 11)],
            "roadside_arrival_sites": ["B11", "B12", "B13"],
            "roadside_max_lateral_offset_m": 0.60,
            "roadside_return_policy": "field_pending",
        },
        "runtime": {
            key: {
                "pass": passed,
                "service_mode": metrics.get("service_mode"),
                "arrival_only": bool(metrics.get("arrival_only", False)),
                "phase_sequence": phase_sequence(metrics),
                "rotate_180": bool(metrics.get("rotate_180", False)),
                "max_final_cmd": metrics.get("cmd_max"),
                "site_distance_m": metrics.get("site_distance_m"),
            }
            for key, (passed, metrics) in reports.items()
        },
        "config_contract_pass": all(
            policy.get(f"camping_site_{index}")
            == ("turnaround" if index <= 10 else "roadside_stop")
            for index in range(1, 14)
        ),
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--result-dir", required=True, type=Path)
    parser.add_argument("--camping-sites", required=True, type=Path)
    args = parser.parse_args()
    reports = {
        **{
            f"B{index}": load_metrics(
                args.result_dir / f"b{index}-turnaround.json"
            )
            for index in range(1, 11)
        },
        "B11": load_metrics(args.result_dir / "b11-roadside-arrival.json"),
        "B12": load_metrics(args.result_dir / "b12-roadside-arrival.json"),
        "B13": load_metrics(args.result_dir / "b13-roadside-arrival.json"),
    }
    policy = load_policy(args.camping_sites)
    render_summary(args.result_dir / "campsite-policy-validation.png", policy, reports)
    render_timeline_gif(args.result_dir / "campsite-phase-sequence.gif", reports)
    write_summary(args.result_dir / "campsite-policy-summary.json", policy, reports)


if __name__ == "__main__":
    main()
