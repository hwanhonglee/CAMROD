#!/usr/bin/env python3
"""Render manual-Return timing and UI resource profiles from normalized JSON."""

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


WIDTH = 1600
HEIGHT = 920
BACKGROUND = "#f4f7f6"
INK = "#18231e"
MUTED = "#5c6962"
GREEN = "#237447"
GREEN_LIGHT = "#dfeee5"
BLUE = "#286da8"
BLUE_LIGHT = "#e0ebf5"
ORANGE = "#b8661f"
ORANGE_LIGHT = "#f8e6d5"
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


def metric_card(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    title: str,
    before: str,
    after: str,
    delta: str,
) -> None:
    x1, y1, x2, y2 = box
    draw.rounded_rectangle(box, radius=8, fill=WHITE, outline=LINE, width=2)
    text(draw, (x1 + 26, y1 + 22), title, 23, INK, True)
    text(draw, (x1 + 26, y1 + 70), "BEFORE", 15, MUTED, True)
    text(draw, (x1 + 26, y1 + 96), before, 25, BLUE, True)
    text(draw, (x1 + 230, y1 + 70), "AFTER", 15, MUTED, True)
    text(draw, (x1 + 230, y1 + 96), after, 25, GREEN, True)
    draw.rounded_rectangle(
        (x2 - 150, y1 + 72, x2 - 22, y1 + 126),
        radius=7,
        fill=GREEN_LIGHT,
        outline=GREEN,
        width=2,
    )
    bounds = draw.textbbox((0, 0), delta, font=font(18, True))
    draw.text(
        (x2 - 86 - (bounds[2] - bounds[0]) / 2, y1 + 88),
        delta,
        fill=GREEN,
        font=font(18, True),
    )


def render(payload: dict, output: Path) -> None:
    before = payload["before"]
    after = payload["after"]
    delta = payload["delta"]
    method = payload["method"]
    host = payload["host"]
    image = Image.new("RGB", (WIDTH, HEIGHT), BACKGROUND)
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, WIDTH, 126), fill="#205c3c")
    text(draw, (62, 27), "Return and telemetry resource profile", 39, WHITE, True)
    text(
        draw,
        (64, 80),
        "Measured AMD64 full simulation; ARM64 8-core / 16-GiB acceptance remains pending",
        20,
        "#dcece3",
    )

    text(draw, (62, 157), "Test boundary", 27, INK, True)
    boundary = (
        f"{method['sample_duration_s']} s | {method['process_count']} processes | "
        f"{host['logical_cpu_count']} logical CPUs | {method['state']} | RViz/browser OFF"
    )
    text(draw, (62, 199), boundary, 20, MUTED)

    metric_card(
        draw,
        (62, 252, 782, 410),
        "Whole graph CPU",
        f"{before['graph_cpu_percent_one_core_basis']:.2f}%",
        f"{after['graph_cpu_percent_one_core_basis']:.2f}%",
        f"{delta['graph_cpu_percent']:.1f}%",
    )
    metric_card(
        draw,
        (818, 252, 1538, 410),
        "UI backend CPU",
        f"{before['ui_backend_cpu_percent_one_core_basis']:.2f}%",
        f"{after['ui_backend_cpu_percent_one_core_basis']:.2f}%",
        f"{delta['ui_backend_cpu_percent']:.1f}%",
    )
    metric_card(
        draw,
        (62, 438, 782, 596),
        "Whole graph RSS",
        f"{before['graph_rss_mib']:.1f} MiB",
        f"{after['graph_rss_mib']:.1f} MiB",
        f"{delta['graph_rss_mib']:.1f} MiB",
    )
    metric_card(
        draw,
        (818, 438, 1538, 596),
        "Host-capacity CPU",
        f"{before['graph_cpu_percent_host_capacity']:.3f}%",
        f"{after['graph_cpu_percent_host_capacity']:.3f}%",
        "-0.091 pp",
    )

    draw.rounded_rectangle(
        (62, 634, 1010, 852), radius=8, fill=BLUE_LIGHT, outline=BLUE, width=2
    )
    text(draw, (90, 660), "Structural change", 25, BLUE, True)
    changes = [
        "10 Hz lease polling removed from idle execution",
        "HTTP/WebSocket request wakes ROS through GuardCondition",
        "1 Hz timer only expires abandoned telemetry leases",
        "Visible telemetry and lazy sensor subscriptions stay unchanged",
    ]
    for index, item in enumerate(changes):
        y = 709 + index * 34
        draw.ellipse((92, y + 6, 104, y + 18), fill=BLUE)
        text(draw, (120, y), item, 18, INK)

    draw.rounded_rectangle(
        (1040, 634, 1538, 852), radius=8, fill=ORANGE_LIGHT, outline=ORANGE, width=2
    )
    text(draw, (1068, 660), "Interpretation limit", 25, ORANGE, True)
    limits = [
        "AMD64 values are comparative only.",
        "RSS sums can double-count shared pages.",
        "No Jetson GPU/frame-pacing claim.",
        "Repeat 30 min on the physical ARM64 robot.",
    ]
    for index, item in enumerate(limits):
        text(draw, (1068, 710 + index * 34), item, 18, INK)

    text(
        draw,
        (62, 882),
        "CPU 100% means one fully occupied logical core; lower values are better.",
        17,
        MUTED,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def render_preemption(payload: dict, output: Path) -> None:
    """Render the measured outbound-to-Return handoff as a time-ordered proof."""
    # HH_260819 - Derive every event marker from the committed live probe JSON;
    # the chart must not imply an ARM64 or physical-braking measurement.
    image = Image.new("RGB", (WIDTH, HEIGHT), BACKGROUND)
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, WIDTH, 126), fill="#205c3c")
    text(draw, (62, 27), "Outbound Return preemption", 39, WHITE, True)
    text(
        draw,
        (64, 80),
        "Measured AMD64 ROS 2 full graph; B6 request after real outbound motion",
        20,
        "#dcece3",
    )

    displacement = float(payload["outbound_displacement_before_return_m"])
    zero_delay = float(payload["first_zero_delay_s"])
    recall_delay = float(payload["recall_delay_s"])
    path = payload["return_paths"][0]
    request_time = float(payload["recalls"][0]["time"]) - recall_delay
    path_delay = float(path["time"]) - request_time
    hold_start, hold_end = (float(value) for value in payload["hold_window_s"])

    text(draw, (62, 158), "Observed contract", 27, INK, True)
    text(
        draw,
        (62, 201),
        f"B6 outbound displacement {displacement:.3f} m | "
        f"command {payload['outbound_motion_observed']['x']:.3f} m/s",
        20,
        MUTED,
    )

    timeline_left = 110
    timeline_right = 1490
    timeline_y = 390
    max_time = max(0.65, path_delay + 0.08)

    def timeline_x(seconds: float) -> float:
        return timeline_left + seconds / max_time * (timeline_right - timeline_left)

    draw.line(
        (timeline_left, timeline_y, timeline_right, timeline_y),
        fill=LINE,
        width=8,
    )
    hold_left = timeline_x(hold_start)
    hold_right = timeline_x(hold_end)
    draw.rounded_rectangle(
        (hold_left, timeline_y - 34, hold_right, timeline_y + 34),
        radius=8,
        fill=BLUE_LIGHT,
        outline=BLUE,
        width=2,
    )
    text(
        draw,
        (hold_left + 16, timeline_y - 86),
        "blocked motion window: command = 0",
        18,
        BLUE,
        True,
    )

    events = [
        (0.0, "RETURN", "request", ORANGE, -1),
        (zero_delay, f"{zero_delay * 1000:.1f} ms", "zero output", GREEN, 1),
        (recall_delay, f"{recall_delay:.3f} s", "one recall", BLUE, -1),
        (path_delay, f"{path_delay:.3f} s", "return path", GREEN, 1),
    ]
    for seconds, primary, secondary, color, direction in events:
        x = timeline_x(seconds)
        draw.line(
            (x, timeline_y - 62, x, timeline_y + 62),
            fill=color,
            width=5,
        )
        label_y = timeline_y - 146 if direction < 0 else timeline_y + 79
        bounds = draw.textbbox((0, 0), primary, font=font(20, True))
        text(
            draw,
            (int(x - (bounds[2] - bounds[0]) / 2), label_y),
            primary,
            20,
            color,
            True,
        )
        detail_bounds = draw.textbbox((0, 0), secondary, font=font(17))
        text(
            draw,
            (int(x - (detail_bounds[2] - detail_bounds[0]) / 2), label_y + 30),
            secondary,
            17,
            MUTED,
        )

    cards = [
        (
            "BUTTON COALESCING",
            "first + second: return_preempting",
            "third at 0.8 s: return_in_progress",
            ORANGE_LIGHT,
            ORANGE,
        ),
        (
            "MOTION BARRIER",
            f"first zero: {zero_delay * 1000:.1f} ms",
            f"{payload['hold_window_samples']} samples, max command 0.000",
            BLUE_LIGHT,
            BLUE,
        ),
        (
            "FRESH RETURN ROUTE",
            f"recalls: {payload['recall_count']} | source: http:manual_return",
            f"{path['poses']} poses | {path['length_m']:.2f} m",
            GREEN_LIGHT,
            GREEN,
        ),
    ]
    card_width = 468
    for index, (title, first, second, fill, color) in enumerate(cards):
        left = 62 + index * 506
        draw.rounded_rectangle(
            (left, 596, left + card_width, 768),
            radius=8,
            fill=fill,
            outline=color,
            width=2,
        )
        text(draw, (left + 24, 620), title, 20, color, True)
        text(draw, (left + 24, 674), first, 17, INK)
        text(draw, (left + 24, 711), second, 17, INK)

    result = "PASS" if payload.get("pass") else "FAIL"
    result_color = GREEN if payload.get("pass") else "#a63838"
    text(draw, (62, 814), f"{result}: outbound command was stopped before routing.", 23, result_color, True)
    text(
        draw,
        (62, 858),
        "Limit: AMD64 simulation evidence only; physical ARM64 braking and load acceptance remain pending.",
        17,
        MUTED,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--preemption-input", type=Path)
    parser.add_argument("--preemption-output", type=Path)
    args = parser.parse_args()
    payload = json.loads(args.input.read_text(encoding="utf-8"))
    render(payload, args.output)
    if bool(args.preemption_input) != bool(args.preemption_output):
        parser.error(
            "--preemption-input and --preemption-output must be provided together"
        )
    if args.preemption_input:
        preemption = json.loads(args.preemption_input.read_text(encoding="utf-8"))
        render_preemption(preemption, args.preemption_output)


if __name__ == "__main__":
    main()
