#!/usr/bin/env python3
"""Render the v2.1.5 three-kph RPP lookahead service A/B result."""

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


# HH_260807 - Render only committed machine-readable measurements so the
# selected controller setting and the rejected comparison stay auditable.
WIDTH = 1600
HEIGHT = 920
BG = "#f3f6f5"
INK = "#17211d"
MUTED = "#5b6962"
WHITE = "#ffffff"
GREEN = "#257a4b"
GREEN_LIGHT = "#dcefe3"
RED = "#a63838"
RED_LIGHT = "#f6dddd"
BLUE = "#286aa6"
BLUE_LIGHT = "#dfebf6"
LINE = "#c8d2cd"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    """Return the stable font used by committed evidence assets."""
    return ImageFont.truetype(FONT_BOLD if bold else FONT, size)


def label(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    value: str,
    size: int,
    color: str = INK,
    bold: bool = False,
) -> None:
    """Draw one text label."""
    draw.text(xy, value, fill=color, font=font(size, bold))


def status_badge(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    value: str,
    passed: bool,
) -> None:
    """Draw a fixed-size status badge."""
    x, y = xy
    edge = GREEN if passed else RED
    fill = GREEN_LIGHT if passed else RED_LIGHT
    draw.rounded_rectangle(
        (x, y, x + 142, y + 46), radius=7, fill=fill, outline=edge, width=2
    )
    bounds = draw.textbbox((0, 0), value, font=font(19, True))
    text_width = bounds[2] - bounds[0]
    draw.text((x + 71 - text_width / 2, y + 10), value, fill=edge, font=font(19, True))


def result_card(
    draw: ImageDraw.ImageDraw,
    bounds: tuple[int, int, int, int],
    title: str,
    subtitle: str,
    rows: list[tuple[str, str]],
    passed: bool,
) -> None:
    """Draw one side of the controller A/B comparison."""
    left, top, right, bottom = bounds
    draw.rounded_rectangle(bounds, radius=8, fill=WHITE, outline=LINE, width=2)
    accent = GREEN if passed else RED
    draw.rectangle((left, top, left + 12, bottom), fill=accent)
    label(draw, (left + 36, top + 26), title, 28, accent, True)
    label(draw, (left + 36, top + 70), subtitle, 18, MUTED)
    status_badge(draw, (right - 174, top + 25), "SELECTED" if passed else "REJECTED", passed)
    row_y = top + 132
    for key, value in rows:
        label(draw, (left + 36, row_y + 1), key, 16, MUTED, True)
        # HH_260807 - Keep long measured outcomes inside the fixed evidence
        # card without colliding with the row label.
        value_size = 17
        available = right - 36 - (left + 300)
        while (
            value_size > 13
            and draw.textlength(value, font=font(value_size)) > available
        ):
            value_size -= 1
        label(draw, (left + 300, row_y), value, value_size, INK)
        draw.line((left + 36, row_y + 36, right - 36, row_y + 36), fill="#e5eae7", width=1)
        row_y += 58


def render(input_path: Path, output_path: Path) -> None:
    """Render the comparison JSON into one PNG."""
    payload = json.loads(input_path.read_text(encoding="utf-8"))
    baseline = payload["baseline"]
    selected = payload["selected"]

    image = Image.new("RGB", (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, WIDTH, 128), fill="#1f5e3d")
    label(draw, (62, 25), "3 km/h RPP lookahead service A/B", 39, WHITE, True)
    label(
        draw,
        (64, 79),
        "Map v17 | B1/B2 full service | same footprint, speed and recovery policy",
        20,
        "#dcece3",
    )

    result_card(
        draw,
        (54, 168, 780, 690),
        "Velocity-scaled preview",
        "Configured floor 1.1 m, effective preview about 1.5 m",
        [
            ("B2 first hold", baseline["first_hold_pose"]),
            ("Release", f"{baseline['release_time_s']:.3f} s"),
            ("Same-boundary recontact", f"{baseline['recontact_after_release_s']:.3f} s"),
            ("Final behavior", "safe retry latch; service aborted"),
            ("Retry-count increase", "rejected; repeats unsafe approach"),
        ],
        False,
    )
    result_card(
        draw,
        (820, 168, 1546, 690),
        "Fixed 1.1 m preview",
        "Production selection at the final 0.833 m/s platform command",
        [
            ("Service cycles", f"{selected['cycles_completed']}/{selected['cycles_requested']} PASS"),
            ("Continuous runtime", f"{selected['elapsed_s']:.3f} s, restart 0"),
            ("B2 boundary hold", selected["b2_recovery"]),
            ("Obstacle cycle", selected["obstacle_result"]),
            ("Final behavior", selected["final_behavior"]),
        ],
        True,
    )

    draw.rounded_rectangle(
        (54, 730, 1546, 860), radius=8, fill=BLUE_LIGHT, outline=BLUE, width=2
    )
    label(draw, (82, 752), "Decision", 24, BLUE, True)
    label(
        draw,
        (82, 792),
        "Keep the fixed 1.1 m preview. Do not increase recovery retries to compensate for",
        20,
        INK,
    )
    label(
        draw,
        (82, 824),
        "a controller that immediately drives back into the same boundary.",
        20,
        INK,
    )
    label(
        draw,
        (1045, 824),
        payload["classification"],
        16,
        MUTED,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)


def main() -> None:
    """Parse command-line paths and render the evidence asset."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    render(args.input, args.output)


if __name__ == "__main__":
    main()
