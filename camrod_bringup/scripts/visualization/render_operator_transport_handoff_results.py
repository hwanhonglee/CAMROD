#!/usr/bin/env python3
"""Render the measured operator transport and return-handoff policy results."""

# HH_260810 - Keep post-v2.1.7 UI timing and system handoff evidence
# reproducible without presenting summary animation as a raw runtime capture.

import argparse
import json
from pathlib import Path
import warnings

warnings.filterwarnings("ignore", message="Unable to import Axes3D.*")
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Rectangle  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

UI_RESULT = "ui/test-results/operator-telemetry-websocket-amd64-20260810"
SYSTEM_RESULT = "system/test-results/return-handoff-nav-status-20260810"
UI_PNG = "operator-telemetry-websocket-amd64.png"
UI_GIF = "operator-telemetry-websocket-amd64.gif"
SYSTEM_PNG = "return-handoff-nav-status.png"
SYSTEM_GIF = "return-handoff-nav-status.gif"

INK = "#17313a"
MUTED = "#5d7078"
LINE = "#cad7db"
WHITE = "#ffffff"
UI_BG = "#f3f8f5"
UI_GREEN = "#206a44"
UI_BLUE = "#2878b5"
UI_AMBER = "#c56c15"
SYSTEM_BG = "#faf7f1"
SYSTEM_AMBER = "#9a5b00"
SYSTEM_BLUE = "#2f6f8f"
SYSTEM_RED = "#b42318"
OK = "#18794e"


def load_json(path: Path) -> dict:
    """Load one UTF-8 JSON record."""
    return json.loads(path.read_text(encoding="utf-8"))


def validate_measurement(record: dict) -> None:
    """Reject a stale or over-claimed operator transport record."""
    if record.get("classification") != "MEASURED_AMD64_STANDALONE":
        raise ValueError("operator transport classification is not the AMD64 baseline")
    if record.get("target_runtime", {}).get("accepted_by_this_measurement") is not False:
        raise ValueError("operator transport record must not claim ARM64 acceptance")
    scope = record.get("scope", {})
    if scope.get("sensor_publishers") is not False:
        raise ValueError("this renderer requires the sensorless standalone measurement")
    if float(scope.get("stream_rate_parameter_hz", 0.0)) != 10.0:
        raise ValueError("operator transport record is not the 10 Hz profile")
    websocket = record.get("websocket", {})
    if int(websocket.get("frames", 0)) < 2:
        raise ValueError("operator transport record has no frame sample")
    if not 9.0 <= float(websocket.get("effective_rate_hz", 0.0)) <= 10.5:
        raise ValueError("operator transport effective rate is outside the expected band")


def validate_policy(record: dict) -> None:
    """Reject an incomplete return-handoff policy regression record."""
    if record.get("classification") != "POLICY_REGRESSION":
        raise ValueError("return handoff result is not a policy regression")
    if record.get("field_claim") is not False:
        raise ValueError("return handoff policy must not claim field evidence")
    if float(record.get("transition_grace_s", -1.0)) != 3.0:
        raise ValueError("return handoff transition grace is stale")
    cases = {case["id"]: case for case in record.get("cases", [])}
    required = {
        "outgoing_goal_inside_grace": "SUPPRESSED_HANDOFF",
        "new_return_goal_inside_grace": "VISIBLE_ABORT",
        "return_goal_after_grace": "VISIBLE_ABORT",
    }
    if {key: cases.get(key, {}).get("expected") for key in required} != required:
        raise ValueError("return handoff policy cases are incomplete or stale")


def canvas(title: str, subtitle: str, background: str, primary: str):
    """Create one fixed documentation canvas."""
    figure = plt.figure(figsize=(16, 9), facecolor=background)
    axis = figure.add_axes((0, 0, 1, 1))
    axis.set_xlim(0, 1)
    axis.set_ylim(0, 1)
    axis.axis("off")
    axis.add_patch(Rectangle((0, 0.985), 0.76, 0.015, color=primary))
    axis.add_patch(Rectangle((0.76, 0.985), 0.24, 0.015, color=UI_BLUE))
    figure.text(0.045, 0.945, title, color=INK, fontsize=24, fontweight="bold", va="top")
    figure.text(0.045, 0.897, subtitle, color=MUTED, fontsize=11, va="top")
    return figure, axis


def panel(axis, x, y, width, height, edge=LINE, face=WHITE, linewidth=1.2):
    """Draw one bounded result panel."""
    patch = FancyBboxPatch(
        (x, y),
        width,
        height,
        boxstyle="round,pad=0.009,rounding_size=0.012",
        edgecolor=edge,
        facecolor=face,
        linewidth=linewidth,
    )
    axis.add_patch(patch)
    return patch


def badge(axis, x, y, text, color, face):
    """Draw one evidence-class badge."""
    width = max(0.115, 0.0085 * len(text))
    panel(axis, x, y, width, 0.040, edge=color, face=face, linewidth=1.0)
    axis.text(
        x + width / 2,
        y + 0.020,
        text,
        ha="center",
        va="center",
        fontsize=8.4,
        color=color,
        fontweight="bold",
    )


def metric(axis, x, y, width, label, value, color):
    """Draw a compact measured-value card."""
    panel(axis, x, y, width, 0.105, edge=LINE, face=WHITE)
    axis.text(x + 0.015, y + 0.078, label, fontsize=8.5, color=MUTED, va="top")
    axis.text(
        x + 0.015,
        y + 0.050,
        value,
        fontsize=16,
        color=color,
        fontweight="bold",
        va="top",
    )


def save_png(figure, path: Path) -> None:
    """Write one deterministic 1600x900 PNG."""
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=100, facecolor=figure.get_facecolor())
    plt.close(figure)


def figure_image(figure) -> Image.Image:
    """Convert a Matplotlib canvas to one RGB Pillow image."""
    figure.canvas.draw()
    rgba = np.asarray(figure.canvas.buffer_rgba()).copy()
    return Image.fromarray(rgba[:, :, :3], mode="RGB")


def save_animation(frames: list[Image.Image], path: Path) -> None:
    """Write a 24-frame 1000x600 GIF without dropping duplicate states."""
    path.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        path,
        save_all=True,
        append_images=frames[1:],
        duration=160,
        loop=0,
        optimize=False,
        disposal=2,
    )


def render_ui_png(record: dict, path: Path) -> None:
    """Render the exact standalone AMD64 timing/resource summary."""
    websocket = record["websocket"]
    intervals = websocket["interval_ms"]
    payload = websocket["payload_bytes"]
    resources = record["process_resources"]
    cleanup = record["lease_cleanup"]
    figure, axis = canvas(
        "Operator Telemetry WebSocket",
        "Measured standalone backend timing, payload, process cost, and lease cleanup",
        UI_BG,
        UI_GREEN,
    )
    badge(axis, 0.045, 0.825, "MEASURED AMD64 STANDALONE", UI_GREEN, "#e4f1e8")
    badge(axis, 0.265, 0.825, "NO SENSOR PUBLISHERS", UI_AMBER, "#fff0df")
    badge(axis, 0.435, 0.825, "NOT ARM64 ACCEPTANCE", SYSTEM_RED, "#fdeceb")

    panel(axis, 0.045, 0.405, 0.570, 0.375)
    axis.text(0.068, 0.745, "Frame interval percentile summary", color=INK, fontsize=13, fontweight="bold")
    axis.text(
        0.068,
        0.713,
        "Aggregate percentiles from 201 frames; this is not a reconstructed raw trace.",
        color=MUTED,
        fontsize=8.7,
    )
    minimum, maximum = 99.5, 101.5
    x0, x1, line_y = 0.090, 0.570, 0.565
    axis.plot([x0, x1], [line_y, line_y], color=LINE, linewidth=5, solid_capstyle="round")
    for tick in (99.5, 100.0, 100.5, 101.0, 101.5):
        x = x0 + (tick - minimum) / (maximum - minimum) * (x1 - x0)
        axis.plot([x, x], [line_y - 0.016, line_y + 0.016], color="#81959d", linewidth=1)
        axis.text(x, line_y - 0.040, f"{tick:.1f}", ha="center", fontsize=7.7, color=MUTED)
    # HH_260810 - Stagger close percentile labels and add leaders so the mean
    # and p95 remain readable at README scale without moving their data marks.
    markers = (
        ("mean", intervals["mean"], UI_BLUE, 0.652, -0.032, "right"),
        ("p50", intervals["p50"], UI_GREEN, 0.615, -0.032, "right"),
        ("p95", intervals["p95"], UI_AMBER, 0.652, 0.026, "left"),
        ("p99", intervals["p99"], "#8c3d62", 0.615, -0.010, "right"),
        ("max", intervals["max"], SYSTEM_RED, 0.652, 0.012, "left"),
    )
    for label, value, color, label_y, label_offset, alignment in markers:
        x = x0 + (float(value) - minimum) / (maximum - minimum) * (x1 - x0)
        axis.scatter([x], [line_y], s=90, color=color, edgecolor=WHITE, linewidth=1.3, zorder=4)
        label_x = x + label_offset
        axis.plot(
            [x, label_x],
            [line_y + 0.012, label_y - 0.008],
            color=color,
            linewidth=0.8,
            alpha=0.75,
        )
        axis.text(
            label_x,
            label_y,
            f"{label} {float(value):.3f} ms",
            ha=alignment,
            fontsize=7.5,
            color=color,
            fontweight="bold",
        )
    axis.text(0.090, 0.470, "Configured cadence", fontsize=8.5, color=MUTED)
    axis.text(0.218, 0.470, "100.000 ms", fontsize=10, color=INK, fontweight="bold")
    axis.text(0.355, 0.470, "Observed p95", fontsize=8.5, color=MUTED)
    axis.text(0.470, 0.470, f"{intervals['p95']:.3f} ms", fontsize=10, color=INK, fontweight="bold")

    metric(axis, 0.645, 0.675, 0.145, "Effective rate", f"{websocket['effective_rate_hz']:.3f} Hz", UI_GREEN)
    metric(axis, 0.810, 0.675, 0.145, "Frames / duration", f"{websocket['frames']} / {websocket['duration_s']:.3f}s", UI_BLUE)
    metric(axis, 0.645, 0.545, 0.145, "Mean payload", f"{payload['mean']:.1f} B", UI_AMBER)
    metric(axis, 0.810, 0.545, 0.145, "Maximum payload", f"{payload['max']} B", "#8c3d62")
    metric(axis, 0.645, 0.415, 0.145, "Stream parameter", f"{record['scope']['stream_rate_parameter_hz']:.1f} Hz", UI_BLUE)
    metric(axis, 0.810, 0.415, 0.145, "REST fallback", "1.0 Hz", MUTED)

    panel(axis, 0.045, 0.145, 0.440, 0.205)
    axis.text(0.068, 0.315, "Backend process cost", color=INK, fontsize=12, fontweight="bold")
    cpu = resources["cpu_one_logical_core_percent"]
    rss = resources["rss_kib"]
    axis.text(0.068, 0.266, "CPU, idle -> active", color=MUTED, fontsize=8.8)
    axis.text(0.250, 0.266, f"{cpu['idle_mean']:.2f}% -> {cpu['active_mean']:.2f}%", color=UI_GREEN, fontsize=12, fontweight="bold")
    axis.text(0.068, 0.218, "RSS, idle -> active", color=MUTED, fontsize=8.8)
    axis.text(0.250, 0.218, f"{rss['idle']:,} -> {rss['active']:,} KiB", color=UI_BLUE, fontsize=12, fontweight="bold")
    axis.text(0.068, 0.170, "Measured RSS delta", color=MUTED, fontsize=8.8)
    axis.text(0.250, 0.170, f"+{rss['delta']:,} KiB", color=UI_AMBER, fontsize=12, fontweight="bold")

    panel(axis, 0.515, 0.145, 0.440, 0.205)
    axis.text(0.538, 0.315, "Lease cleanup", color=INK, fontsize=12, fontweight="bold")
    axis.plot([0.555, 0.900], [0.238, 0.238], color=LINE, linewidth=4, solid_capstyle="round")
    normal_x = 0.555 + min(cleanup["normal_close_detected_ms"] / 13000.0, 1.0) * 0.345
    silent_x = 0.555 + min(cleanup["silent_client_expired_s"] / 13.0, 1.0) * 0.345
    axis.scatter([normal_x], [0.238], s=90, color=OK, edgecolor=WHITE, zorder=4)
    axis.scatter([silent_x], [0.238], s=90, color=UI_AMBER, edgecolor=WHITE, zorder=4)
    axis.text(0.555, 0.280, f"normal close {cleanup['normal_close_detected_ms']:.1f} ms", fontsize=9, color=OK, fontweight="bold")
    axis.text(0.722, 0.190, f"silent lease {cleanup['silent_client_expired_s']:.3f} s", fontsize=9, color=UI_AMBER, fontweight="bold")
    axis.text(0.555, 0.165, "0 s", fontsize=7.5, color=MUTED)
    axis.text(0.888, 0.165, "13 s", fontsize=7.5, color=MUTED)

    axis.text(
        0.500,
        0.075,
        "ARM64 8-core / 16-GB acceptance still requires the live six-view, sensor, DDS, GPU, and 30-minute test.",
        ha="center",
        color=SYSTEM_RED,
        fontsize=9.3,
        fontweight="bold",
    )
    save_png(figure, path)


def render_ui_gif(record: dict, path: Path) -> None:
    """Animate the measured summary contract without inventing raw samples."""
    websocket = record["websocket"]
    cleanup = record["lease_cleanup"]
    frames = []
    for frame_index in range(24):
        figure = plt.figure(figsize=(10, 6), facecolor=UI_BG)
        axis = figure.add_axes((0, 0, 1, 1))
        axis.set_xlim(0, 1)
        axis.set_ylim(0, 1)
        axis.axis("off")
        axis.add_patch(Rectangle((0, 0.98), 0.72, 0.02, color=UI_GREEN))
        axis.add_patch(Rectangle((0.72, 0.98), 0.28, 0.02, color=UI_BLUE))
        axis.text(0.050, 0.925, "10 Hz selected-view transport", fontsize=18, color=INK, fontweight="bold")
        axis.text(0.050, 0.880, "MEASURED SUMMARY ANIMATION - not a raw frame trace", fontsize=9, color=SYSTEM_RED, fontweight="bold")
        panel(axis, 0.050, 0.515, 0.900, 0.285)
        stages = ((0.13, "ROS latest"), (0.38, "bounded JSON"), (0.63, "WebSocket"), (0.87, "selected view"))
        for x, label in stages:
            axis.scatter([x], [0.660], s=520, color="#e4f1e8", edgecolor=UI_GREEN, linewidth=1.4)
            axis.text(x, 0.660, label, ha="center", va="center", fontsize=8.5, color=INK, fontweight="bold")
        axis.plot([0.16, 0.84], [0.660, 0.660], color=LINE, linewidth=5, zorder=0)
        travel = (frame_index % 12) / 11.0
        axis.scatter([0.13 + 0.74 * travel], [0.660], s=95, color=UI_BLUE, edgecolor=WHITE, linewidth=1.2, zorder=5)
        axis.text(0.500, 0.555, f"{websocket['effective_rate_hz']:.3f} Hz | p95 {websocket['interval_ms']['p95']:.3f} ms | {websocket['payload_bytes']['mean']:.1f} B mean", ha="center", fontsize=10, color=UI_BLUE, fontweight="bold")
        panel(axis, 0.050, 0.155, 0.430, 0.285)
        panel(axis, 0.520, 0.155, 0.430, 0.285)
        axis.text(0.075, 0.395, "Normal close", fontsize=12, color=INK, fontweight="bold")
        axis.text(0.075, 0.350, "server receive detects disconnect", fontsize=8.6, color=MUTED)
        normal_active = frame_index % 12 >= 8
        axis.text(0.075, 0.260, "RELEASED" if normal_active else "ACTIVE", fontsize=17, color=OK if normal_active else UI_BLUE, fontweight="bold")
        axis.text(0.075, 0.205, f"observed {cleanup['normal_close_detected_ms']:.1f} ms", fontsize=9.5, color=OK)
        axis.text(0.545, 0.395, "Silent client", fontsize=12, color=INK, fontweight="bold")
        axis.text(0.545, 0.350, "4 s heartbeat stops; 12 s lease expires", fontsize=8.6, color=MUTED)
        silent_active = frame_index >= 18
        axis.text(0.545, 0.260, "EXPIRED" if silent_active else "LEASED", fontsize=17, color=UI_AMBER if silent_active else UI_BLUE, fontweight="bold")
        axis.text(0.545, 0.205, f"observed {cleanup['silent_client_expired_s']:.3f} s", fontsize=9.5, color=UI_AMBER)
        frames.append(figure_image(figure))
        plt.close(figure)
    save_animation(frames, path)


def case_title(case: dict) -> str:
    """Return a short human-readable policy-case title."""
    return {
        "outgoing_goal_inside_grace": "Outgoing site goal",
        "new_return_goal_inside_grace": "New return goal",
        "return_goal_after_grace": "Abort after grace",
    }[case["id"]]


def render_policy_cards(axis, record: dict, highlight: int | None = None) -> None:
    """Draw the three return-handoff regression cases."""
    for index, case in enumerate(record["cases"]):
        x = 0.045 + index * 0.315
        visible = case["expected"] == "VISIBLE_ABORT"
        color = SYSTEM_RED if visible else OK
        face = "#fff4f2" if visible else "#edf7f1"
        linewidth = 3.0 if highlight == index else 1.4
        panel(axis, x, 0.315, 0.280, 0.435, edge=color, face=face, linewidth=linewidth)
        axis.text(x + 0.020, 0.700, f"CASE {index + 1}", fontsize=8.2, color=MUTED, fontweight="bold")
        axis.text(x + 0.020, 0.660, case_title(case), fontsize=13, color=INK, fontweight="bold")
        axis.text(x + 0.020, 0.605, f"ABORTED at {case['transition_age_s']:.2f} s", fontsize=10, color=SYSTEM_BLUE, fontweight="bold")
        pre = "yes" if case["goal_observed_before_transition"] else "no"
        axis.text(x + 0.020, 0.555, f"UUID seen before transition: {pre}", fontsize=8.7, color=MUTED)
        axis.plot([x + 0.025, x + 0.255], [0.490, 0.490], color=LINE, linewidth=4, solid_capstyle="round")
        transition_x = x + 0.025
        grace_x = x + 0.025 + min(record["transition_grace_s"] / 4.0, 1.0) * 0.230
        abort_x = x + 0.025 + min(case["transition_age_s"] / 4.0, 1.0) * 0.230
        axis.scatter([transition_x], [0.490], s=75, color=SYSTEM_BLUE, edgecolor=WHITE, zorder=4)
        axis.scatter([grace_x], [0.490], s=75, color=SYSTEM_AMBER, edgecolor=WHITE, zorder=4)
        axis.scatter([abort_x], [0.490], s=105, color=color, edgecolor=WHITE, zorder=5)
        axis.text(transition_x, 0.455, "0", fontsize=7.5, ha="center", color=MUTED)
        axis.text(grace_x, 0.455, "3.0 s", fontsize=7.5, ha="center", color=SYSTEM_AMBER)
        axis.text(x + 0.140, 0.390, case["expected"].replace("_", " "), fontsize=11.5, color=color, fontweight="bold", ha="center")
        axis.text(x + 0.140, 0.345, case["operator_surface"], fontsize=8.4, color=MUTED, ha="center")


def render_system_png(record: dict, path: Path) -> None:
    """Render the bounded goal-identity return-handoff policy."""
    figure, axis = canvas(
        "Return Handoff Nav Status",
        "Goal UUID and transition age decide whether ABORTED is an expected handoff or a visible route fault",
        SYSTEM_BG,
        SYSTEM_AMBER,
    )
    badge(axis, 0.045, 0.825, "POLICY REGRESSION", SYSTEM_AMBER, "#fff0d8")
    badge(axis, 0.205, 0.825, "3.0 S BOUNDED GRACE", SYSTEM_BLUE, "#e4f0f5")
    badge(axis, 0.390, 0.825, "NOT A FIELD CAPTURE", SYSTEM_RED, "#fdeceb")
    render_policy_cards(axis, record)
    panel(axis, 0.045, 0.115, 0.910, 0.130, edge=SYSTEM_BLUE, face="#edf4f7")
    axis.text(0.070, 0.205, "Decision rule", fontsize=11, color=SYSTEM_BLUE, fontweight="bold")
    axis.text(
        0.070,
        0.160,
        "Suppress only when RETURNING_TO_DROP_ZONE age <= 3.0 s AND the goal UUID existed before transition.",
        fontsize=10.5,
        color=INK,
        fontweight="bold",
    )
    axis.text(
        0.500,
        0.060,
        "Source and unit-test evidence only: real return-route failure remains WARN/ERROR; physical mission acceptance is separate.",
        ha="center",
        fontsize=8.8,
        color=SYSTEM_RED,
        fontweight="bold",
    )
    save_png(figure, path)


def render_system_gif(record: dict, path: Path) -> None:
    """Animate the three unit-tested return-handoff decisions."""
    frames = []
    for frame_index in range(24):
        active_case = min(frame_index // 8, 2)
        case_progress = (frame_index % 8) / 7.0
        figure = plt.figure(figsize=(10, 6), facecolor=SYSTEM_BG)
        axis = figure.add_axes((0, 0, 1, 1))
        axis.set_xlim(0, 1)
        axis.set_ylim(0, 1)
        axis.axis("off")
        axis.add_patch(Rectangle((0, 0.98), 0.72, 0.02, color=SYSTEM_AMBER))
        axis.add_patch(Rectangle((0.72, 0.98), 0.28, 0.02, color=SYSTEM_BLUE))
        axis.text(0.050, 0.925, "Return goal handoff policy", fontsize=18, color=INK, fontweight="bold")
        axis.text(0.050, 0.880, "POLICY REGRESSION ANIMATION - not a runtime capture", fontsize=9, color=SYSTEM_RED, fontweight="bold")
        # Keep all 24 frames distinct while showing the currently evaluated
        # case; the moving marker is presentation, not an elapsed-time sample.
        axis.plot([0.050, 0.950], [0.825, 0.825], color=LINE, linewidth=2)
        axis.scatter(
            [0.050 + 0.900 * case_progress],
            [0.825],
            s=55,
            color=SYSTEM_BLUE,
            edgecolor=WHITE,
            linewidth=1.0,
            zorder=5,
        )
        render_policy_cards(axis, record, highlight=active_case)
        active = record["cases"][active_case]
        result_color = OK if active["expected"] == "SUPPRESSED_HANDOFF" else SYSTEM_RED
        axis.text(0.500, 0.235, active["reason"], ha="center", fontsize=10, color=INK, fontweight="bold")
        axis.text(0.500, 0.165, active["operator_surface"], ha="center", fontsize=14, color=result_color, fontweight="bold")
        axis.text(0.500, 0.095, "New return-goal failures are never hidden by the transition grace.", ha="center", fontsize=9, color=SYSTEM_BLUE)
        frames.append(figure_image(figure))
        plt.close(figure)
    save_animation(frames, path)


def parse_args():
    """Parse repository and output locations."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[3],
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        help="Asset root; defaults to <repo>/docs/assets/module-guides",
    )
    parser.add_argument("--ui-measurement", type=Path)
    parser.add_argument("--handoff-policy", type=Path)
    return parser.parse_args()


def main() -> None:
    """Validate structured inputs and render four package-owned assets."""
    args = parse_args()
    repo_root = args.repo_root.resolve()
    source_root = repo_root / "docs" / "assets" / "module-guides"
    output_root = (args.output_root or source_root).resolve()
    measurement_path = args.ui_measurement or source_root / UI_RESULT / "measurement.json"
    policy_path = args.handoff_policy or source_root / SYSTEM_RESULT / "policy-cases.json"
    measurement = load_json(measurement_path)
    policy = load_json(policy_path)
    validate_measurement(measurement)
    validate_policy(policy)

    ui_output = output_root / UI_RESULT
    system_output = output_root / SYSTEM_RESULT
    render_ui_png(measurement, ui_output / UI_PNG)
    render_ui_gif(measurement, ui_output / UI_GIF)
    render_system_png(policy, system_output / SYSTEM_PNG)
    render_system_gif(policy, system_output / SYSTEM_GIF)
    print(f"Rendered operator/system result assets under {output_root}")


if __name__ == "__main__":
    main()
