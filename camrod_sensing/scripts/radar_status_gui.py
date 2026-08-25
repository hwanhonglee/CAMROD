#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Live top-down dashboard for the seven physical SEN0592 range sensors.

HH_260807 - Visualize only the real ``sen0592_radar_node`` ``Range`` outputs.
This tool never starts or publishes dummy radar data.  Each SEN0592 provides one
scalar distance somewhere inside its field of view, not a measured bearing, so
the canvas draws a range arc rather than claiming an exact obstacle position.

Run after the physical radar driver and workspace setup are active::

  ros2 run camrod_sensing radar_status_gui.py

The dashboard subscribes to the seven
``/sensing/radar/<channel>/range_ros`` topics with SensorDataQoS.
"""

from collections import deque
from dataclasses import dataclass
import math
import os
import sys
import threading
import tkinter as tk
from tkinter import font as tkfont

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
import yaml


COLOR_BG = "#171a1f"
COLOR_PANEL = "#232831"
COLOR_GRID = "#343b46"
COLOR_TEXT = "#e8edf2"
COLOR_LABEL = "#9ca8b7"
COLOR_ROBOT = "#4f6378"

STATE_COLORS = {
    "hit": "#e44f4f",
    "filtered": "#3f8fc4",
    "outside_stop": "#71879b",
    "no_target": "#35b66a",
    "stale": "#d28b35",
    "invalid": "#b26ee5",
    "no_data": "#68717d",
}

STATE_LABELS = {
    "hit": "RAW HIT",
    "filtered": "FILTERED BODY ECHO",
    "outside_stop": "OUTSIDE STOP WINDOW",
    "no_target": "NO TARGET",
    "stale": "STALE",
    "invalid": "INVALID",
    "no_data": "NO DATA",
}

SENSOR_ORDER = (
    "FRONT1",
    "FRONT2",
    "LEFT1",
    "LEFT2",
    "RIGHT1",
    "RIGHT2",
    "REAR",
)

# HH_260807 - Runtime robot_params.yaml mount poses, expressed in
# robot_center_link: x forward, y left, yaw counter-clockwise. Side radar XY
# values use the latest center-referenced field measurements.
MOUNT_POSES = {
    "FRONT1": (0.62787, -0.11005, 0.0),
    "FRONT2": (0.62787, 0.11005, 0.0),
    "LEFT1": (0.38, 0.53, math.pi / 2.0),
    "LEFT2": (-0.38, 0.53, math.pi / 2.0),
    "RIGHT1": (0.38, -0.53, -math.pi / 2.0),
    "RIGHT2": (-0.38, -0.53, -math.pi / 2.0),
    "REAR": (-0.61733, 0.0, math.pi),
}

# Current measured body envelope relative to robot_center_link.
BODY_FRONT_M = 0.70837
BODY_REAR_M = -0.68323
BODY_LEFT_M = 0.53505
BODY_RIGHT_M = -0.53495


@dataclass(frozen=True)
class ChannelSpec:
    """Static topic, mount, and configured range data for one physical sensor."""

    name: str
    frame_id: str
    port: str
    topic: str
    x_m: float
    y_m: float
    yaw_rad: float
    configured_max_m: float
    configured_enabled: bool
    field_of_view_rad: float


@dataclass(frozen=True)
class ChannelSnapshot:
    """GUI-safe primitive snapshot copied from one ROS subscription."""

    spec: ChannelSpec
    state: str
    range_m: float | None
    min_range_m: float | None
    max_range_m: float
    stop_candidate_max_range_m: float
    field_of_view_rad: float
    age_s: float | None
    rate_hz: float | None
    frame_id: str


def resolve_topic(prefix: str, topic: str) -> str:
    """Resolve one YAML-relative topic under the standalone radar namespace."""
    if topic.startswith("/"):
        return topic
    return f"{prefix.rstrip('/')}/{topic.lstrip('/')}"


def classify_sample(
    range_m: float | None,
    min_range_m: float | None,
    max_range_m: float,
    age_s: float | None,
    stale_timeout_s: float,
    fixed_return_bands: tuple[tuple[float, float], ...] = (),
    stop_candidate_max_range_m: float | None = None,
) -> str:
    """Classify a real Range sample with the deployed self-echo exclusions."""
    if range_m is None or age_s is None:
        return "no_data"
    if age_s > stale_timeout_s:
        return "stale"
    if math.isinf(range_m):
        return "no_target" if range_m > 0.0 else "invalid"
    if not math.isfinite(range_m):
        return "invalid"
    if range_m > max_range_m:
        # The physical driver intentionally emits max_range + 0.001 m when the
        # sensor replied but reported no accepted target.
        return "no_target"
    if min_range_m is None or range_m < min_range_m:
        return "invalid"
    if (
        stop_candidate_max_range_m is not None
        and range_m > stop_candidate_max_range_m
    ):
        return "outside_stop"
    if any(minimum <= range_m <= maximum for minimum, maximum in fixed_return_bands):
        return "filtered"
    return "hit"


def rolling_hz(stamps: tuple[float, ...] | list[float]) -> float | None:
    """Return receive rate from an ordered timestamp window."""
    if len(stamps) < 2:
        return None
    duration = stamps[-1] - stamps[0]
    if duration <= 0.0:
        return None
    return (len(stamps) - 1) / duration


def world_to_canvas(
    x_m: float,
    y_m: float,
    center_x_px: float,
    center_y_px: float,
    pixels_per_m: float,
) -> tuple[float, float]:
    """Map robot +X to screen up and robot +Y to screen left."""
    return (
        center_x_px - y_m * pixels_per_m,
        center_y_px - x_m * pixels_per_m,
    )


def beam_arc_points(
    spec: ChannelSpec,
    distance_m: float,
    field_of_view_rad: float,
    steps: int = 24,
) -> tuple[tuple[float, float], ...]:
    """Return world points on the possible-bearing arc for a scalar range."""
    step_count = max(2, steps)
    half_fov = max(0.0, field_of_view_rad) / 2.0
    points = []
    for index in range(step_count + 1):
        fraction = index / step_count
        angle = spec.yaw_rad - half_fov + 2.0 * half_fov * fraction
        points.append(
            (
                spec.x_m + distance_m * math.cos(angle),
                spec.y_m + distance_m * math.sin(angle),
            )
        )
    return tuple(points)


def load_channel_specs(config_file: str, topic_prefix: str) -> tuple[ChannelSpec, ...]:
    """Load the live driver topic/range contract from the installed YAML."""
    with open(config_file, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)

    params = document["/**"]["ros__parameters"]
    names = list(params["sensor_names"])
    frames = list(params["frame_ids"])
    ports = list(params["ports"])
    topics = list(params["standard_ros_topics"])
    max_ranges = list(params["software_max_ranges_m"])
    enabled = list(params.get("sensor_enabled", [True] * len(names)))
    fov = float(params["range_message_field_of_view_rad"])

    expected_length = len(SENSOR_ORDER)
    arrays = (names, frames, ports, topics, max_ranges, enabled)
    if any(len(values) != expected_length for values in arrays):
        raise ValueError("radar GUI requires exactly seven aligned channel entries")
    if tuple(names) != SENSOR_ORDER:
        raise ValueError(f"unexpected radar channel order: {names}")

    specs = []
    for index, name in enumerate(names):
        x_m, y_m, yaw_rad = MOUNT_POSES[name]
        specs.append(
            ChannelSpec(
                name=name,
                frame_id=str(frames[index]),
                port=str(ports[index]),
                topic=resolve_topic(topic_prefix, str(topics[index])),
                x_m=x_m,
                y_m=y_m,
                yaw_rad=yaw_rad,
                configured_max_m=float(max_ranges[index]),
                configured_enabled=bool(enabled[index]),
                field_of_view_rad=fov,
            )
        )
    return tuple(specs)


def load_fixed_return_bands(
    config_file: str,
) -> dict[str, tuple[tuple[float, float], ...]]:
    """Load the exact named fixed-return profile used by radar_cost_grid."""
    with open(config_file, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)

    params = document["/sensing/radar/radar_cost_grid"]["ros__parameters"]
    output = {name: [] for name in SENSOR_ORDER}
    if not bool(params.get("fixed_return_filter_enable", False)):
        return {name: tuple() for name in SENSOR_ORDER}

    for raw_spec in params.get("fixed_return_bands", []):
        fields = str(raw_spec).split(":")
        if len(fields) != 3:
            raise ValueError(f"invalid fixed-return band: {raw_spec!r}")
        sensor, minimum_text, maximum_text = fields
        sensor = sensor.strip().upper()
        if sensor not in output:
            raise ValueError(f"unknown fixed-return sensor: {sensor}")
        minimum = float(minimum_text)
        maximum = float(maximum_text)
        if not (math.isfinite(minimum) and math.isfinite(maximum)):
            raise ValueError(f"non-finite fixed-return band: {raw_spec!r}")
        if minimum < 0.0 or maximum <= minimum:
            raise ValueError(f"invalid fixed-return interval: {raw_spec!r}")
        output[sensor].append((minimum, maximum))
    return {name: tuple(output[name]) for name in SENSOR_ORDER}


def load_stop_candidate_max_range_m(config_file: str) -> float:
    """Load the backward-compatible scalar stop cutoff."""
    with open(config_file, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    params = document["/sensing/radar/radar_cost_grid"]["ros__parameters"]
    value = float(params["stop_candidate_max_range_m"])
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("stop_candidate_max_range_m must be finite and positive")
    return value


def load_stop_candidate_max_ranges_m(config_file: str) -> dict[str, float]:
    """Load the cost-grid's ordered per-channel stop cutoffs by sensor name."""
    with open(config_file, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    params = document["/sensing/radar/radar_cost_grid"]["ros__parameters"]

    fallback = float(params["stop_candidate_max_range_m"])
    if not math.isfinite(fallback) or fallback <= 0.0:
        raise ValueError("stop_candidate_max_range_m must be finite and positive")

    raw_values = params.get("stop_candidate_max_ranges_m", [])
    if not raw_values:
        return {name: fallback for name in SENSOR_ORDER}

    topics = tuple(str(topic) for topic in params["input_topics"])
    labels = tuple(topic.rstrip("/").split("/")[-2].upper() for topic in topics)
    if labels != SENSOR_ORDER:
        raise ValueError(
            "cost-grid input_topics order must match physical SENSOR_ORDER"
        )
    if len(raw_values) != len(labels):
        raise ValueError(
            "stop_candidate_max_ranges_m must match input_topics size"
        )

    values = tuple(float(value) for value in raw_values)
    if any(not math.isfinite(value) or value <= 0.0 for value in values):
        raise ValueError(
            "stop_candidate_max_ranges_m values must be finite and positive"
        )
    return dict(zip(labels, values))


class RadarStatusNode(Node):
    """Subscribe to the seven real standard Range topics and track freshness."""

    def __init__(self):
        super().__init__("radar_status_gui")

        package_share = get_package_share_directory("camrod_sensing")
        default_config = os.path.join(
            package_share, "config", "radar", "sen0592_radar.yaml"
        )
        default_cost_config = os.path.join(
            package_share, "config", "radar", "cost_grid.yaml"
        )
        self.declare_parameter("sensor_config_file", default_config)
        self.declare_parameter("cost_grid_config_file", default_cost_config)
        self.declare_parameter("topic_prefix", "/sensing/radar")
        self.declare_parameter("stale_timeout_s", 1.0)
        self.declare_parameter("rate_window_s", 2.0)

        config_file = str(self.get_parameter("sensor_config_file").value)
        cost_config_file = str(self.get_parameter("cost_grid_config_file").value)
        topic_prefix = str(self.get_parameter("topic_prefix").value)
        self.stale_timeout_s = float(
            self.get_parameter("stale_timeout_s").value
        )
        self.rate_window_s = float(self.get_parameter("rate_window_s").value)
        if self.stale_timeout_s <= 0.0 or self.rate_window_s <= 0.0:
            raise ValueError("stale_timeout_s and rate_window_s must be positive")

        self.specs = load_channel_specs(config_file, topic_prefix)
        self.fixed_return_bands = load_fixed_return_bands(cost_config_file)
        self.stop_candidate_max_ranges_m = load_stop_candidate_max_ranges_m(
            cost_config_file
        )
        disabled = [spec.name for spec in self.specs if not spec.configured_enabled]
        if disabled:
            self.get_logger().warning(
                "physical radar GUI config has disabled channels: "
                + ", ".join(disabled)
            )

        self.lock = threading.Lock()
        self.samples = {spec.name: None for spec in self.specs}
        self.receive_stamps = {spec.name: deque() for spec in self.specs}
        # Keep explicit references without shadowing rclpy.Node.subscriptions.
        self._range_subscriptions = []

        for spec in self.specs:
            subscription = self.create_subscription(
                Range,
                spec.topic,
                lambda msg, name=spec.name: self._range_cb(name, msg),
                qos_profile_sensor_data,
            )
            self._range_subscriptions.append(subscription)

        self.get_logger().info(
            f"physical radar GUI subscribed to {len(self.specs)} Range topics "
            f"from {config_file}; self-echo profile {cost_config_file}"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _range_cb(self, name: str, msg: Range):
        now = self._now()
        with self.lock:
            self.samples[name] = (
                float(msg.range),
                float(msg.min_range),
                float(msg.max_range),
                float(msg.field_of_view),
                str(msg.header.frame_id),
                now,
            )
            stamps = self.receive_stamps[name]
            stamps.append(now)
            cutoff = now - self.rate_window_s
            while stamps and stamps[0] < cutoff:
                stamps.popleft()

    def snapshot(self) -> tuple[ChannelSnapshot, ...]:
        now = self._now()
        output = []
        with self.lock:
            for spec in self.specs:
                sample = self.samples[spec.name]
                rate = rolling_hz(tuple(self.receive_stamps[spec.name]))
                if sample is None:
                    range_m = None
                    min_range_m = None
                    max_range_m = spec.configured_max_m
                    fov = spec.field_of_view_rad
                    frame_id = spec.frame_id
                    age_s = None
                else:
                    (
                        range_m,
                        min_range_m,
                        max_range_m,
                        fov,
                        frame_id,
                        received_at,
                    ) = sample
                    age_s = max(0.0, now - received_at)

                state = classify_sample(
                    range_m,
                    min_range_m,
                    max_range_m,
                    age_s,
                    self.stale_timeout_s,
                    self.fixed_return_bands[spec.name],
                    self.stop_candidate_max_ranges_m[spec.name],
                )
                output.append(
                    ChannelSnapshot(
                        spec=spec,
                        state=state,
                        range_m=range_m,
                        min_range_m=min_range_m,
                        max_range_m=max_range_m,
                        stop_candidate_max_range_m=(
                            self.stop_candidate_max_ranges_m[spec.name]
                        ),
                        field_of_view_rad=fov,
                        age_s=age_s,
                        rate_hz=rate,
                        frame_id=frame_id,
                    )
                )
        return tuple(output)


class SummaryTile(tk.Frame):
    """Compact count tile for one radar state."""

    def __init__(self, parent, title: str, color: str, title_font, value_font):
        super().__init__(parent, bg=color, bd=0)
        tk.Label(
            self, text=title, fg="#ffffff", bg=color, font=title_font
        ).pack(pady=(7, 0))
        self.value_label = tk.Label(
            self, text="0", fg="#ffffff", bg=color, font=value_font
        )
        self.value_label.pack(pady=(0, 7))

    def set(self, value: int):
        self.value_label.config(text=str(value))


class ChannelRow(tk.Frame):
    """One physical sensor row in the numeric dashboard."""

    def __init__(self, parent, spec: ChannelSpec, header_font, mono_font):
        super().__init__(parent, bg=COLOR_PANEL, highlightthickness=1)
        self.spec = spec
        self.grid_columnconfigure(1, weight=1)

        self.name_label = tk.Label(
            self,
            text=spec.name,
            fg="#ffffff",
            bg=STATE_COLORS["no_data"],
            font=header_font,
            width=9,
        )
        self.name_label.grid(row=0, column=0, rowspan=2, sticky="nsew")

        self.state_label = tk.Label(
            self,
            text="NO DATA",
            fg=COLOR_TEXT,
            bg=COLOR_PANEL,
            font=header_font,
            anchor="w",
        )
        self.state_label.grid(row=0, column=1, sticky="ew", padx=8, pady=(4, 0))

        self.range_label = tk.Label(
            self,
            text="--",
            fg=COLOR_TEXT,
            bg=COLOR_PANEL,
            font=mono_font,
            anchor="e",
            width=14,
        )
        self.range_label.grid(row=0, column=2, sticky="e", padx=8, pady=(4, 0))

        self.detail_label = tk.Label(
            self,
            text=f"{os.path.basename(spec.port)}  {spec.frame_id}",
            fg=COLOR_LABEL,
            bg=COLOR_PANEL,
            font=mono_font,
            anchor="w",
        )
        self.detail_label.grid(
            row=1, column=1, columnspan=2, sticky="ew", padx=8, pady=(0, 4)
        )

    def update_value(self, snap: ChannelSnapshot):
        color = STATE_COLORS[snap.state]
        self.config(highlightbackground=color)
        self.name_label.config(bg=color)
        state_text = STATE_LABELS[snap.state]
        if snap.state == "outside_stop":
            state_text = (
                f"OUTSIDE {snap.stop_candidate_max_range_m * 100.0:.1f} CM STOP"
            )
        self.state_label.config(text=state_text, fg=color)

        if snap.range_m is None:
            range_text = "--"
        elif snap.state == "no_target":
            range_text = f"> {snap.max_range_m:.3f} m"
        elif math.isfinite(snap.range_m):
            range_text = f"{snap.range_m:.3f} m"
        else:
            range_text = str(snap.range_m)
        self.range_label.config(text=range_text)

        rate_text = "-- Hz" if snap.rate_hz is None else f"{snap.rate_hz:4.1f} Hz"
        age_text = "-- age" if snap.age_s is None else f"{snap.age_s:4.2f}s age"
        frame_text = snap.frame_id or snap.spec.frame_id
        self.detail_label.config(
            text=(
                f"{os.path.basename(snap.spec.port)}  {rate_text}  "
                f"{age_text}  stop<={snap.stop_candidate_max_range_m:.3f}m  "
                f"{frame_text}"
            )
        )


class RadarStatusGui:
    """Tk dashboard with a truthful scalar-range top-down visualization."""

    CANVAS_WIDTH = 760
    CANVAS_HEIGHT = 680
    CENTER_X = 380.0
    CENTER_Y = 410.0
    PIXELS_PER_M = 180.0

    def __init__(self, root: tk.Tk, node: RadarStatusNode):
        self.root = root
        self.node = node
        root.title("Physical Radar Status Monitor")
        root.configure(bg=COLOR_BG)
        root.geometry("1280x820")

        title_font = tkfont.Font(family="DejaVu Sans", size=14, weight="bold")
        header_font = tkfont.Font(family="DejaVu Sans", size=11, weight="bold")
        mono_font = tkfont.Font(family="DejaVu Sans Mono", size=10)
        tile_value_font = tkfont.Font(
            family="DejaVu Sans Mono", size=18, weight="bold"
        )

        heading = tk.Frame(root, bg=COLOR_BG)
        heading.pack(fill="x", padx=12, pady=(10, 5))
        tk.Label(
            heading,
            text="SEN0592 PHYSICAL RADAR — LIVE RANGE + STOP FILTER",
            fg=COLOR_TEXT,
            bg=COLOR_BG,
            font=title_font,
        ).pack(side="left")
        tk.Label(
            heading,
            text="7 × physical /range_ros  •  self-echo aware  •  no dummy",
            fg=COLOR_LABEL,
            bg=COLOR_BG,
            font=mono_font,
        ).pack(side="right")

        tiles = tk.Frame(root, bg=COLOR_BG)
        tiles.pack(fill="x", padx=12, pady=5)
        for column in range(7):
            tiles.grid_columnconfigure(column, weight=1, uniform="summary")

        self.tile_hit = SummaryTile(
            tiles, "RAW HIT", STATE_COLORS["hit"], header_font, tile_value_font
        )
        self.tile_filtered = SummaryTile(
            tiles,
            "FILTERED ECHO",
            STATE_COLORS["filtered"],
            header_font,
            tile_value_font,
        )
        self.tile_outside_stop = SummaryTile(
            tiles,
            "OUTSIDE STOP",
            STATE_COLORS["outside_stop"],
            header_font,
            tile_value_font,
        )
        self.tile_clear = SummaryTile(
            tiles,
            "NO TARGET",
            STATE_COLORS["no_target"],
            header_font,
            tile_value_font,
        )
        self.tile_stale = SummaryTile(
            tiles, "STALE", STATE_COLORS["stale"], header_font, tile_value_font
        )
        self.tile_invalid = SummaryTile(
            tiles,
            "INVALID",
            STATE_COLORS["invalid"],
            header_font,
            tile_value_font,
        )
        self.tile_fresh = SummaryTile(
            tiles, "FRESH", "#3478bd", header_font, tile_value_font
        )
        for column, tile in enumerate(
            (
                self.tile_hit,
                self.tile_filtered,
                self.tile_outside_stop,
                self.tile_clear,
                self.tile_stale,
                self.tile_invalid,
                self.tile_fresh,
            )
        ):
            tile.grid(row=0, column=column, sticky="nsew", padx=3)

        content = tk.Frame(root, bg=COLOR_BG)
        content.pack(fill="both", expand=True, padx=12, pady=(5, 10))

        self.canvas = tk.Canvas(
            content,
            width=self.CANVAS_WIDTH,
            height=self.CANVAS_HEIGHT,
            bg="#11151a",
            highlightthickness=1,
            highlightbackground=COLOR_GRID,
        )
        self.canvas.pack(side="left", fill="both", expand=False)

        side = tk.Frame(content, bg=COLOR_BG)
        side.pack(side="left", fill="both", expand=True, padx=(10, 0))
        tk.Label(
            side,
            text="PHYSICAL CHANNELS",
            fg=COLOR_TEXT,
            bg=COLOR_BG,
            font=header_font,
        ).pack(anchor="w", pady=(0, 5))

        self.rows = {}
        for spec in node.specs:
            row = ChannelRow(side, spec, header_font, mono_font)
            row.pack(fill="x", pady=3)
            self.rows[spec.name] = row

        note = (
            "One scalar distance is reported somewhere on the shown arc.\n"
            "Blue = measured body echo excluded from stopping. Active raw stop\n"
            "windows: F1 (22,52] cm, F2 (11.7,41.7] cm, REAR (10.6,20.6] cm;\n"
            "LEFT/RIGHT cutoff is 10 cm. Gray is outside that channel's window."
        )
        tk.Label(
            side,
            text=note,
            fg=COLOR_LABEL,
            bg=COLOR_BG,
            font=mono_font,
            justify="left",
            anchor="w",
        ).pack(fill="x", pady=(10, 0))

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self.update_loop)

    def _to_canvas(self, x_m: float, y_m: float) -> tuple[float, float]:
        return world_to_canvas(
            x_m,
            y_m,
            self.CENTER_X,
            self.CENTER_Y,
            self.PIXELS_PER_M,
        )

    def _draw_grid_and_robot(self):
        for value in (index * 0.5 for index in range(-3, 6)):
            x1, y1 = self._to_canvas(value, -1.5)
            x2, y2 = self._to_canvas(value, 1.5)
            self.canvas.create_line(x1, y1, x2, y2, fill=COLOR_GRID, dash=(2, 5))
        for value in (index * 0.5 for index in range(-3, 4)):
            x1, y1 = self._to_canvas(-1.4, value)
            x2, y2 = self._to_canvas(2.3, value)
            self.canvas.create_line(x1, y1, x2, y2, fill=COLOR_GRID, dash=(2, 5))

        corners_world = (
            (BODY_FRONT_M, BODY_LEFT_M),
            (BODY_FRONT_M, BODY_RIGHT_M),
            (BODY_REAR_M, BODY_RIGHT_M),
            (BODY_REAR_M, BODY_LEFT_M),
        )
        corners = [coordinate for point in corners_world for coordinate in self._to_canvas(*point)]
        self.canvas.create_polygon(
            *corners,
            fill=COLOR_ROBOT,
            outline="#a5b4c3",
            width=2,
        )
        center_x, center_y = self._to_canvas(0.0, 0.0)
        front_x, front_y = self._to_canvas(0.48, 0.0)
        self.canvas.create_line(
            center_x,
            center_y,
            front_x,
            front_y,
            fill="#ffffff",
            width=3,
            arrow=tk.LAST,
        )
        self.canvas.create_text(
            center_x,
            center_y + 18,
            text="robot_center_link",
            fill="#dce6ef",
            font=("DejaVu Sans Mono", 9),
        )

    def _draw_channel(self, snap: ChannelSnapshot):
        spec = snap.spec
        color = STATE_COLORS[snap.state]
        origin = self._to_canvas(spec.x_m, spec.y_m)
        fov = snap.field_of_view_rad or spec.field_of_view_rad

        max_arc_world = beam_arc_points(spec, snap.max_range_m, fov)
        max_arc = [coordinate for point in max_arc_world for coordinate in self._to_canvas(*point)]
        left_end = self._to_canvas(*max_arc_world[0])
        right_end = self._to_canvas(*max_arc_world[-1])
        self.canvas.create_line(*origin, *left_end, fill="#4b5664", dash=(3, 4))
        self.canvas.create_line(*origin, *right_end, fill="#4b5664", dash=(3, 4))
        self.canvas.create_line(*max_arc, fill="#4b5664", dash=(3, 4))

        if snap.state == "hit" and snap.range_m is not None:
            distance = max(0.0, min(snap.range_m, snap.max_range_m))
            arc_world = beam_arc_points(spec, distance, fov)
            arc = [coordinate for point in arc_world for coordinate in self._to_canvas(*point)]
            self.canvas.create_line(*arc, fill=color, width=5)
            center_world = (
                spec.x_m + distance * math.cos(spec.yaw_rad),
                spec.y_m + distance * math.sin(spec.yaw_rad),
            )
            center = self._to_canvas(*center_world)
            self.canvas.create_line(*origin, *center, fill=color, width=2)
            radius = 5
            self.canvas.create_oval(
                center[0] - radius,
                center[1] - radius,
                center[0] + radius,
                center[1] + radius,
                fill=color,
                outline="#ffffff",
            )
        elif snap.state in ("filtered", "outside_stop") and snap.range_m is not None:
            distance = max(0.0, min(snap.range_m, snap.max_range_m))
            arc_world = beam_arc_points(spec, distance, fov)
            arc = [
                coordinate
                for point in arc_world
                for coordinate in self._to_canvas(*point)
            ]
            self.canvas.create_line(*arc, fill=color, width=3, dash=(4, 3))
        elif snap.state == "no_target":
            self.canvas.create_line(*max_arc, fill=color, width=2, dash=(6, 4))

        radius = 6
        self.canvas.create_oval(
            origin[0] - radius,
            origin[1] - radius,
            origin[0] + radius,
            origin[1] + radius,
            fill=color,
            outline="#ffffff",
        )
        label_distance = 0.16
        label_world = (
            spec.x_m - label_distance * math.cos(spec.yaw_rad),
            spec.y_m - label_distance * math.sin(spec.yaw_rad),
        )
        label = self._to_canvas(*label_world)
        self.canvas.create_text(
            *label,
            text=spec.name,
            fill=COLOR_TEXT,
            font=("DejaVu Sans", 9, "bold"),
        )

    def update_loop(self):
        snapshots = self.node.snapshot()
        counts = {state: 0 for state in STATE_COLORS}
        for snap in snapshots:
            counts[snap.state] += 1
            self.rows[snap.spec.name].update_value(snap)

        self.tile_hit.set(counts["hit"])
        self.tile_filtered.set(counts["filtered"])
        self.tile_outside_stop.set(counts["outside_stop"])
        self.tile_clear.set(counts["no_target"])
        self.tile_stale.set(counts["stale"] + counts["no_data"])
        self.tile_invalid.set(counts["invalid"])
        self.tile_fresh.set(
            counts["hit"]
            + counts["filtered"]
            + counts["outside_stop"]
            + counts["no_target"]
            + counts["invalid"]
        )

        self.canvas.delete("all")
        self._draw_grid_and_robot()
        for snap in snapshots:
            self._draw_channel(snap)

        self.root.after(100, self.update_loop)

    def on_close(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()


def ros_spin_thread(node: RadarStatusNode):
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


def main():
    rclpy.init(args=sys.argv)
    node = RadarStatusNode()
    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    RadarStatusGui(root, node)
    root.mainloop()

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
