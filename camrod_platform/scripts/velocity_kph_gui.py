#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Live km/h speedometer for the real platform ground velocity.

Subscribes to ``/platform/status/velocity`` (``avg_msgs/AvgTwistStamped``),
published by ``ranger_platform_bridge_node`` from the real Ranger CAN driver.
Ground speed is the vector magnitude of body-frame ``vx``/``vy`` converted
from m/s to km/h, matching what ``camrod_localization``'s EKF fuses as the
``odom0`` wheel input.

Run after the physical platform driver and workspace setup are active::

  ros2 run camrod_platform velocity_kph_gui.py

Optional parameters (``--ros-args -p name:=value``)::

  velocity_topic     default "/platform/status/velocity"
  cruise_speed_kph   default 2.0  (green/amber gauge boundary)
  max_speed_kph      default 5.0  (amber/red gauge boundary)
  gauge_max_kph      default 8.0  (dial upper bound)
  stale_timeout_s    default 1.0
  rate_window_s      default 2.0
  history_window_s   default 20.0 (trend strip duration)
"""

from collections import deque
from dataclasses import dataclass
import math
import sys
import threading
import tkinter as tk
from tkinter import font as tkfont

from avg_msgs.msg import AvgTwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


COLOR_BG = "#171a1f"
COLOR_PANEL = "#232831"
COLOR_GRID = "#343b46"
COLOR_TEXT = "#e8edf2"
COLOR_LABEL = "#9ca8b7"
COLOR_NEEDLE = "#e8edf2"

ZONE_COLORS = {
    "cruise": "#35b66a",
    "brisk": "#d28b35",
    "over_max": "#e44f4f",
}

STATE_COLORS = {
    "fresh": "#35b66a",
    "stale": "#d28b35",
    "no_data": "#68717d",
}


def mps_to_kph(mps: float) -> float:
    """Convert linear speed from m/s to km/h."""
    return mps * 3.6


def ground_speed_mps(vx: float, vy: float) -> float:
    """Return body-frame ground speed magnitude, matching EKF odom0 vx/vy fusion."""
    return math.hypot(vx, vy)


def rolling_hz(stamps) -> float | None:
    """Return receive rate from an ordered timestamp window."""
    if len(stamps) < 2:
        return None
    duration = stamps[-1] - stamps[0]
    if duration <= 0.0:
        return None
    return (len(stamps) - 1) / duration


def classify_freshness(age_s: float | None, stale_timeout_s: float) -> str:
    """Classify freshness the same way the radar/pose checkers do."""
    if age_s is None:
        return "no_data"
    if age_s > stale_timeout_s:
        return "stale"
    return "fresh"


def zone_for_speed(speed_kph: float, cruise_speed_kph: float, max_speed_kph: float) -> str:
    """Return which configured speed band a reading falls into."""
    if speed_kph > max_speed_kph:
        return "over_max"
    if speed_kph > cruise_speed_kph:
        return "brisk"
    return "cruise"


def value_to_gauge_angle(
    value: float,
    min_value: float,
    max_value: float,
    start_deg: float = 210.0,
    end_deg: float = -30.0,
) -> float:
    """Map a clamped value to a screen angle (0 deg = east, ccw positive)."""
    if max_value <= min_value:
        return start_deg
    fraction = (value - min_value) / (max_value - min_value)
    fraction = max(0.0, min(1.0, fraction))
    return start_deg + fraction * (end_deg - start_deg)


def gauge_point(
    center_x: float, center_y: float, radius: float, angle_deg: float
) -> tuple:
    """Return the canvas point at ``radius`` along ``angle_deg`` from center."""
    angle_rad = math.radians(angle_deg)
    return (
        center_x + radius * math.cos(angle_rad),
        center_y - radius * math.sin(angle_rad),
    )


@dataclass(frozen=True)
class VelocitySnapshot:
    """GUI-safe primitive snapshot copied from the subscribed twist."""

    vx_mps: float | None
    vy_mps: float | None
    speed_mps: float | None
    speed_kph: float | None
    age_s: float | None
    rate_hz: float | None
    state: str
    frame_id: str
    history: tuple


class VelocityStatusNode(Node):
    """Subscribe to the real platform velocity twist and track freshness."""

    def __init__(self):
        super().__init__("velocity_kph_gui")

        self.velocity_topic = str(
            self.declare_parameter("velocity_topic", "/platform/status/velocity").value
        )
        # HH_260807 - Match the final platform cruise contract. This is a gauge
        # boundary only; it never writes a speed command.
        self.cruise_speed_kph = float(self.declare_parameter("cruise_speed_kph", 2.0).value)
        self.max_speed_kph = float(self.declare_parameter("max_speed_kph", 5.0).value)
        self.gauge_max_kph = float(self.declare_parameter("gauge_max_kph", 8.0).value)
        self.stale_timeout_s = float(self.declare_parameter("stale_timeout_s", 1.0).value)
        self.rate_window_s = float(self.declare_parameter("rate_window_s", 2.0).value)
        self.history_window_s = float(
            self.declare_parameter("history_window_s", 20.0).value
        )

        if self.stale_timeout_s <= 0.0 or self.rate_window_s <= 0.0:
            raise ValueError("stale_timeout_s and rate_window_s must be positive")
        if self.history_window_s <= 0.0:
            raise ValueError("history_window_s must be positive")
        if not (0.0 < self.cruise_speed_kph < self.max_speed_kph < self.gauge_max_kph):
            raise ValueError(
                "require 0 < cruise_speed_kph < max_speed_kph < gauge_max_kph"
            )

        self.lock = threading.Lock()
        self.sample = None
        self.receive_stamps: deque = deque()
        self.history: deque = deque()

        self.velocity_sub = self.create_subscription(
            AvgTwistStamped,
            self.velocity_topic,
            self._velocity_cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"velocity kph GUI subscribed to {self.velocity_topic}; "
            f"cruise={self.cruise_speed_kph:.1f} km/h max={self.max_speed_kph:.1f} km/h"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _velocity_cb(self, msg: AvgTwistStamped):
        now = self._now()
        vx = float(msg.twist.linear.x)
        vy = float(msg.twist.linear.y)
        speed_mps = ground_speed_mps(vx, vy)
        speed_kph = mps_to_kph(speed_mps)
        with self.lock:
            self.sample = (vx, vy, speed_mps, speed_kph, str(msg.header.frame_id), now)

            self.receive_stamps.append(now)
            rate_cutoff = now - self.rate_window_s
            while self.receive_stamps and self.receive_stamps[0] < rate_cutoff:
                self.receive_stamps.popleft()

            self.history.append((now, speed_kph))
            history_cutoff = now - self.history_window_s
            while self.history and self.history[0][0] < history_cutoff:
                self.history.popleft()

    def snapshot(self) -> VelocitySnapshot:
        now = self._now()
        with self.lock:
            rate = rolling_hz(tuple(self.receive_stamps))
            history = tuple(self.history)
            if self.sample is None:
                return VelocitySnapshot(
                    vx_mps=None,
                    vy_mps=None,
                    speed_mps=None,
                    speed_kph=None,
                    age_s=None,
                    rate_hz=rate,
                    state="no_data",
                    frame_id="",
                    history=history,
                )
            vx, vy, speed_mps, speed_kph, frame_id, received_at = self.sample
            age_s = max(0.0, now - received_at)
            return VelocitySnapshot(
                vx_mps=vx,
                vy_mps=vy,
                speed_mps=speed_mps,
                speed_kph=speed_kph,
                age_s=age_s,
                rate_hz=rate,
                state=classify_freshness(age_s, self.stale_timeout_s),
                frame_id=frame_id,
                history=history,
            )


class VelocityKphGui:
    """Tk speedometer dashboard for the real platform ground velocity."""

    GAUGE_WIDTH = 480
    GAUGE_HEIGHT = 300
    CENTER_X = 240.0
    CENTER_Y = 250.0
    RADIUS = 190.0
    START_DEG = 210.0
    END_DEG = -30.0

    TRAIL_WIDTH = 480
    TRAIL_HEIGHT = 110

    def __init__(self, root: tk.Tk, node: VelocityStatusNode):
        self.root = root
        self.node = node
        root.title("Platform Velocity Monitor (km/h)")
        root.configure(bg=COLOR_BG)
        root.geometry("560x680")

        title_font = tkfont.Font(family="DejaVu Sans", size=14, weight="bold")
        header_font = tkfont.Font(family="DejaVu Sans", size=11, weight="bold")
        mono_font = tkfont.Font(family="DejaVu Sans Mono", size=10)
        digital_font = tkfont.Font(family="DejaVu Sans Mono", size=40, weight="bold")

        heading = tk.Frame(root, bg=COLOR_BG)
        heading.pack(fill="x", padx=12, pady=(10, 5))
        tk.Label(
            heading,
            text="PLATFORM VELOCITY — LIVE km/h",
            fg=COLOR_TEXT,
            bg=COLOR_BG,
            font=title_font,
        ).pack(side="left")
        tk.Label(
            heading,
            text=node.velocity_topic,
            fg=COLOR_LABEL,
            bg=COLOR_BG,
            font=mono_font,
        ).pack(side="right")

        self.canvas = tk.Canvas(
            root,
            width=self.GAUGE_WIDTH,
            height=self.GAUGE_HEIGHT,
            bg="#11151a",
            highlightthickness=1,
            highlightbackground=COLOR_GRID,
        )
        self.canvas.pack(padx=12, pady=(0, 5))

        self.digital_label = tk.Label(
            root, text="-- km/h", fg=COLOR_TEXT, bg=COLOR_BG, font=digital_font
        )
        self.digital_label.pack(pady=(0, 0))

        self.detail_label = tk.Label(
            root, text="", fg=COLOR_LABEL, bg=COLOR_BG, font=mono_font
        )
        self.detail_label.pack(pady=(2, 5))

        tk.Label(
            root,
            text=f"TREND (last {node.history_window_s:.0f} s)",
            fg=COLOR_LABEL,
            bg=COLOR_BG,
            font=header_font,
        ).pack(anchor="w", padx=12)

        self.trail_canvas = tk.Canvas(
            root,
            width=self.TRAIL_WIDTH,
            height=self.TRAIL_HEIGHT,
            bg="#11151a",
            highlightthickness=1,
            highlightbackground=COLOR_GRID,
        )
        self.trail_canvas.pack(padx=12, pady=(2, 5))

        self.status_label = tk.Label(
            root,
            text="NO DATA",
            fg=COLOR_TEXT,
            bg=COLOR_PANEL,
            font=header_font,
            anchor="w",
            padx=8,
            pady=6,
        )
        self.status_label.pack(fill="x", padx=12, pady=(0, 10))

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self.update_loop)

    def _draw_gauge_face(self):
        node = self.node
        cx, cy, r = self.CENTER_X, self.CENTER_Y, self.RADIUS

        zone_bounds = (
            (0.0, node.cruise_speed_kph, ZONE_COLORS["cruise"]),
            (node.cruise_speed_kph, node.max_speed_kph, ZONE_COLORS["brisk"]),
            (node.max_speed_kph, node.gauge_max_kph, ZONE_COLORS["over_max"]),
        )
        for v0, v1, color in zone_bounds:
            angle0 = value_to_gauge_angle(
                v0, 0.0, node.gauge_max_kph, self.START_DEG, self.END_DEG
            )
            angle1 = value_to_gauge_angle(
                v1, 0.0, node.gauge_max_kph, self.START_DEG, self.END_DEG
            )
            self.canvas.create_arc(
                cx - r,
                cy - r,
                cx + r,
                cy + r,
                start=angle1,
                extent=(angle0 - angle1),
                style=tk.ARC,
                outline=color,
                width=18,
            )

        step = max(1.0, round(node.gauge_max_kph / 8.0))
        tick = 0.0
        while tick <= node.gauge_max_kph + 1e-6:
            angle = value_to_gauge_angle(
                tick, 0.0, node.gauge_max_kph, self.START_DEG, self.END_DEG
            )
            outer = gauge_point(cx, cy, r + 12, angle)
            inner = gauge_point(cx, cy, r - 6, angle)
            self.canvas.create_line(*inner, *outer, fill=COLOR_GRID, width=2)
            label_point = gauge_point(cx, cy, r + 28, angle)
            self.canvas.create_text(
                *label_point,
                text=f"{tick:g}",
                fill=COLOR_LABEL,
                font=("DejaVu Sans Mono", 9),
            )
            tick += step

        for marker_value in (node.cruise_speed_kph, node.max_speed_kph):
            angle = value_to_gauge_angle(
                marker_value, 0.0, node.gauge_max_kph, self.START_DEG, self.END_DEG
            )
            outer = gauge_point(cx, cy, r + 12, angle)
            inner = gauge_point(cx, cy, r - 18, angle)
            self.canvas.create_line(*inner, *outer, fill=COLOR_TEXT, width=3)

    def _draw_needle(self, speed_kph: float):
        node = self.node
        cx, cy = self.CENTER_X, self.CENTER_Y
        clamped = max(0.0, min(speed_kph, node.gauge_max_kph))
        angle = value_to_gauge_angle(clamped, 0.0, node.gauge_max_kph, self.START_DEG, self.END_DEG)
        tip = gauge_point(cx, cy, self.RADIUS - 30, angle)
        self.canvas.create_line(cx, cy, *tip, fill=COLOR_NEEDLE, width=4)
        self.canvas.create_oval(cx - 8, cy - 8, cx + 8, cy + 8, fill=COLOR_NEEDLE, outline="")

    def _draw_trail(self, history):
        canvas = self.trail_canvas
        width, height = self.TRAIL_WIDTH, self.TRAIL_HEIGHT
        node = self.node
        for fraction in (0.25, 0.5, 0.75):
            y = height * (1.0 - fraction)
            canvas.create_line(0, y, width, y, fill=COLOR_GRID, dash=(2, 4))

        if len(history) < 2:
            return
        t0 = history[0][0]
        t1 = history[-1][0]
        duration = max(1e-3, t1 - t0)
        points = []
        for t, kph in history:
            x = (t - t0) / duration * width
            y = height - (max(0.0, min(kph, node.gauge_max_kph)) / node.gauge_max_kph) * height
            points.extend((x, y))
        canvas.create_line(*points, fill=ZONE_COLORS["cruise"], width=2)

    def update_loop(self):
        snap = self.node.snapshot()

        self.canvas.delete("all")
        self._draw_gauge_face()
        if snap.speed_kph is not None:
            self._draw_needle(snap.speed_kph)

        if snap.speed_kph is None:
            self.digital_label.config(text="-- km/h", fg=COLOR_LABEL)
            self.detail_label.config(text="waiting for velocity ...")
        else:
            zone = zone_for_speed(
                snap.speed_kph, self.node.cruise_speed_kph, self.node.max_speed_kph
            )
            color = ZONE_COLORS[zone]
            self.digital_label.config(text=f"{snap.speed_kph:5.2f} km/h", fg=color)
            direction = "FWD" if snap.vx_mps >= 0.0 else "REV"
            self.detail_label.config(
                text=(
                    f"{snap.speed_mps:.3f} m/s  "
                    f"vx={snap.vx_mps:+.3f} vy={snap.vy_mps:+.3f}  {direction}"
                )
            )

        self.trail_canvas.delete("all")
        self._draw_trail(snap.history)

        state_color = STATE_COLORS[snap.state]
        rate_text = "-- Hz" if snap.rate_hz is None else f"{snap.rate_hz:4.1f} Hz"
        age_text = "-- age" if snap.age_s is None else f"{snap.age_s:4.2f}s age"
        frame_text = snap.frame_id or "--"
        self.status_label.config(
            text=f"{snap.state.upper():8s}  {rate_text}  {age_text}  frame={frame_text}",
            bg=state_color,
        )

        self.root.after(100, self.update_loop)

    def on_close(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()


def ros_spin_thread(node: VelocityStatusNode):
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


def main():
    rclpy.init(args=sys.argv)
    node = VelocityStatusNode()
    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    VelocityKphGui(root, node)
    root.mainloop()

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
