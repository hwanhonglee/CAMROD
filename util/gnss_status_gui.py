#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HH_260723 - Provide a field GNSS dashboard without changing the sensing graph.

Tkinter GUI dashboard for u-blox GNSS status:
  - Fix type            (ublox_msgs/NavPVT.fix_type)
  - Flags                (ublox_msgs/NavPVT.flags, ublox_msgs/NavRELPOSNED9.flags)
  - Carrier solution      (NavPVT.flags bits 6-7, NavRELPOSNED9.flags bits 3-4)
  - Heading               (motion / vehicle heading from NavPVT,
                            RTK baseline heading from NavRELPOSNED9)
  - h_acc / v_acc / rel_pos_length
  - Covariance            (ublox_msgs/NavCOV position & velocity covariance)

ROS spinning runs on a background thread; Tkinter owns the main thread.

Run (after sourcing the workspace):
  python3 src/util/gnss_status_gui.py
  python3 src/util/gnss_status_gui.py --ros-args -p navpvt_topic:=/gnss/ublox_gps_node/navpvt
"""

import sys
import threading
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from ublox_msgs.msg import NavPVT, NavCOV, NavRELPOSNED9

FIX_TYPE_NAMES = {
    0: "NO FIX",
    1: "DEAD RECKONING",
    2: "2D FIX",
    3: "3D FIX",
    4: "GNSS+DR COMBINED",
    5: "TIME ONLY",
}

CARRIER_SOLN_NAMES = {0: "NONE", 1: "FLOAT", 2: "FIXED"}

NAVSATFIX_STATUS_NAMES = {
    -1: "NO FIX",
    0: "FIX",
    1: "SBAS FIX",
    2: "GBAS FIX",
}

NAVSATFIX_COV_TYPE_NAMES = {
    0: "UNKNOWN",
    1: "APPROXIMATED",
    2: "DIAGONAL_KNOWN",
    3: "KNOWN",
}

COLOR_BG = "#1e1e1e"
COLOR_PANEL = "#262626"
COLOR_TEXT = "#e0e0e0"
COLOR_LABEL = "#9a9a9a"
COLOR_STALE = "#e05a5a"

STATUS_COLORS = {
    "good": "#2fae4e",   # 3D fix / carrier fixed
    "warn": "#e0a52f",   # 2D fix / carrier float
    "bad": "#d1453d",    # no fix / carrier none
    "unknown": "#555555",
}

STALE_TIMEOUT_SEC = 2.0


def fix_type_status(fix_type: int) -> str:
    if fix_type in (3, 4):
        return "good"
    if fix_type in (1, 2, 5):
        return "warn"
    return "bad"


def carrier_status(carr_soln: int) -> str:
    if carr_soln == 2:
        return "good"
    if carr_soln == 1:
        return "warn"
    return "bad"


class GnssStatusNode(Node):
    def __init__(self):
        super().__init__("gnss_status_gui")

        self.declare_parameter("navpvt_topic", "/gnss/ublox_gps_node/navpvt")
        self.declare_parameter("navcov_topic", "/gnss/navcov")
        self.declare_parameter("navrelposned_topic", "/gnss/navrelposned")
        self.declare_parameter("navsatfix_topic", "/gnss/ublox_gps_node/fix")

        navpvt_topic = self.get_parameter("navpvt_topic").value
        navcov_topic = self.get_parameter("navcov_topic").value
        navrelposned_topic = self.get_parameter("navrelposned_topic").value
        navsatfix_topic = self.get_parameter("navsatfix_topic").value

        self.lock = threading.Lock()
        self.navpvt_msg = None
        self.navpvt_stamp = None
        self.navcov_msg = None
        self.navcov_stamp = None
        self.navrelposned_msg = None
        self.navrelposned_stamp = None
        self.navsatfix_msg = None
        self.navsatfix_stamp = None

        self.create_subscription(NavPVT, navpvt_topic, self._navpvt_cb, 10)
        self.create_subscription(NavCOV, navcov_topic, self._navcov_cb, 10)
        self.create_subscription(
            NavRELPOSNED9, navrelposned_topic, self._navrelposned_cb, 10
        )
        self.create_subscription(NavSatFix, navsatfix_topic, self._navsatfix_cb, 10)

        self.topics = {
            "NavPVT": navpvt_topic,
            "NavCOV": navcov_topic,
            "NavRELPOSNED9": navrelposned_topic,
            "NavSatFix": navsatfix_topic,
        }

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _navpvt_cb(self, msg: NavPVT):
        with self.lock:
            self.navpvt_msg = msg
            self.navpvt_stamp = self._now()

    def _navcov_cb(self, msg: NavCOV):
        with self.lock:
            self.navcov_msg = msg
            self.navcov_stamp = self._now()

    def _navrelposned_cb(self, msg: NavRELPOSNED9):
        with self.lock:
            self.navrelposned_msg = msg
            self.navrelposned_stamp = self._now()

    def _navsatfix_cb(self, msg: NavSatFix):
        with self.lock:
            self.navsatfix_msg = msg
            self.navsatfix_stamp = self._now()

    def snapshot(self):
        with self.lock:
            return {
                "navpvt": (self.navpvt_msg, self.navpvt_stamp),
                "navcov": (self.navcov_msg, self.navcov_stamp),
                "navrelposned": (self.navrelposned_msg, self.navrelposned_stamp),
                "navsatfix": (self.navsatfix_msg, self.navsatfix_stamp),
                "now": self._now(),
            }


class ValueRow:
    """A label:value row rendered in a grid, with helpers to update text/color."""

    def __init__(self, parent, row, label_text, value_font, col=0):
        tk.Label(
            parent, text=label_text, fg=COLOR_LABEL, bg=COLOR_PANEL,
            font=value_font, anchor="w",
        ).grid(row=row, column=col, sticky="w", padx=(10, 6), pady=1)
        self.value_label = tk.Label(
            parent, text="--", fg=COLOR_TEXT, bg=COLOR_PANEL,
            font=value_font, anchor="w", justify="left",
        )
        self.value_label.grid(row=row, column=col + 1, sticky="w", padx=(0, 10), pady=1)

    def set(self, text, color=COLOR_TEXT):
        self.value_label.config(text=text, fg=color)


class StatusTile(tk.Frame):
    def __init__(self, parent, title, value_font, title_font):
        super().__init__(parent, bg=STATUS_COLORS["unknown"], bd=0)
        self.title_label = tk.Label(
            self, text=title, fg="#ffffff", bg=self["bg"], font=title_font,
        )
        self.title_label.pack(pady=(10, 2))
        self.value_label = tk.Label(
            self, text="--", fg="#ffffff", bg=self["bg"], font=value_font,
        )
        self.value_label.pack(pady=(0, 10))

    def set(self, text, status_key):
        color = STATUS_COLORS.get(status_key, STATUS_COLORS["unknown"])
        self.config(bg=color)
        self.title_label.config(bg=color)
        self.value_label.config(bg=color, text=text)


class GnssStatusGui:
    def __init__(self, root: tk.Tk, node: GnssStatusNode):
        self.root = root
        self.node = node

        root.title("GNSS Status Monitor")
        root.configure(bg=COLOR_BG)
        root.geometry("940x1040")

        title_font = tkfont.Font(family="DejaVu Sans", size=13, weight="bold")
        tile_title_font = tkfont.Font(family="DejaVu Sans", size=11, weight="bold")
        tile_value_font = tkfont.Font(family="DejaVu Sans Mono", size=20, weight="bold")
        mono_font = tkfont.Font(family="DejaVu Sans Mono", size=11)
        header_font = tkfont.Font(family="DejaVu Sans", size=12, weight="bold")

        # --- status tiles row ---
        tiles_frame = tk.Frame(root, bg=COLOR_BG)
        tiles_frame.pack(fill="x", padx=12, pady=(12, 6))
        tiles_frame.grid_columnconfigure((0, 1, 2), weight=1, uniform="tile")

        self.tile_fix = StatusTile(tiles_frame, "FIX TYPE", tile_value_font, tile_title_font)
        self.tile_fix.grid(row=0, column=0, sticky="nsew", padx=4)

        self.tile_carr_pvt = StatusTile(
            tiles_frame, "CARRIER SOLN (PVT)", tile_value_font, tile_title_font
        )
        self.tile_carr_pvt.grid(row=0, column=1, sticky="nsew", padx=4)

        self.tile_carr_rel = StatusTile(
            tiles_frame, "CARRIER SOLN (RELPOS)", tile_value_font, tile_title_font
        )
        self.tile_carr_rel.grid(row=0, column=2, sticky="nsew", padx=4)

        # --- details: NavPVT | NavRELPOSNED9 ---
        details_frame = tk.Frame(root, bg=COLOR_BG)
        details_frame.pack(fill="x", padx=12, pady=6)
        details_frame.grid_columnconfigure((0, 1), weight=1, uniform="details")

        pvt_panel = tk.LabelFrame(
            details_frame, text=" NavPVT ", fg=COLOR_TEXT, bg=COLOR_PANEL,
            font=header_font, labelanchor="nw", bd=1,
        )
        pvt_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 6))

        rel_panel = tk.LabelFrame(
            details_frame, text=" NavRELPOSNED9 ", fg=COLOR_TEXT, bg=COLOR_PANEL,
            font=header_font, labelanchor="nw", bd=1,
        )
        rel_panel.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        self.pvt_age = ValueRow(pvt_panel, 0, "Age", mono_font)
        self.pvt_fixtype_raw = ValueRow(pvt_panel, 1, "fix_type", mono_font)
        self.pvt_numsv = ValueRow(pvt_panel, 2, "Num SV", mono_font)
        self.pvt_flags = ValueRow(pvt_panel, 3, "flags", mono_font)
        self.pvt_flags2 = ValueRow(pvt_panel, 4, "flags2", mono_font)
        self.pvt_hacc = ValueRow(pvt_panel, 5, "h_acc", mono_font)
        self.pvt_vacc = ValueRow(pvt_panel, 6, "v_acc", mono_font)
        self.pvt_head_motion = ValueRow(pvt_panel, 7, "Heading (motion)", mono_font)
        self.pvt_head_veh = ValueRow(pvt_panel, 8, "Heading (vehicle)", mono_font)

        self.rel_age = ValueRow(rel_panel, 0, "Age", mono_font)
        self.rel_flags = ValueRow(rel_panel, 1, "Flags", mono_font)
        self.rel_relposvalid = ValueRow(rel_panel, 2, "relPosValid", mono_font)
        self.rel_heading = ValueRow(rel_panel, 3, "Heading (RTK bl)", mono_font)
        self.rel_head_acc = ValueRow(rel_panel, 4, "Heading acc", mono_font)
        self.rel_length = ValueRow(rel_panel, 5, "rel_pos_length", mono_font)

        # --- NavSatFix ---
        fix_frame = tk.LabelFrame(
            root, text=" NavSatFix (/fix) ", fg=COLOR_TEXT, bg=COLOR_PANEL,
            font=header_font, labelanchor="nw", bd=1,
        )
        fix_frame.pack(fill="x", padx=12, pady=6)
        fix_frame.grid_columnconfigure(4, weight=1)

        self.fix_age = ValueRow(fix_frame, 0, "Age", mono_font)
        self.fix_status = ValueRow(fix_frame, 1, "status.status", mono_font)
        self.fix_service = ValueRow(fix_frame, 2, "status.service", mono_font)
        self.fix_lat = ValueRow(fix_frame, 0, "latitude", mono_font, col=2)
        self.fix_lon = ValueRow(fix_frame, 1, "longitude", mono_font, col=2)
        self.fix_alt = ValueRow(fix_frame, 2, "altitude", mono_font, col=2)
        self.fix_cov_type = ValueRow(fix_frame, 3, "position_covariance_type", mono_font)

        tk.Label(
            fix_frame, text="position_covariance [m^2] (ENU, row-major)",
            fg=COLOR_LABEL, bg=COLOR_PANEL, font=mono_font,
        ).grid(row=4, column=0, columnspan=4, sticky="w", padx=10, pady=(8, 2))
        self.fix_cov_grid = self._build_full_matrix(
            fix_frame, row=5, mono_font=mono_font, headers=("E", "N", "U")
        )

        # --- covariance ---
        cov_frame = tk.LabelFrame(
            root, text=" NavCOV ", fg=COLOR_TEXT, bg=COLOR_PANEL,
            font=header_font, labelanchor="nw", bd=1,
        )
        cov_frame.pack(fill="both", expand=True, padx=12, pady=(6, 12))

        self.cov_age = ValueRow(cov_frame, 0, "Age", mono_font)
        self.cov_valid = ValueRow(cov_frame, 1, "Valid (pos/vel)", mono_font)

        tk.Label(
            cov_frame, text="Position covariance [m^2]", fg=COLOR_LABEL,
            bg=COLOR_PANEL, font=mono_font,
        ).grid(row=2, column=0, columnspan=2, sticky="w", padx=10, pady=(8, 2))
        self.pos_cov_grid = self._build_matrix(cov_frame, row=3, mono_font=mono_font)

        tk.Label(
            cov_frame, text="Velocity covariance [m^2/s^2]", fg=COLOR_LABEL,
            bg=COLOR_PANEL, font=mono_font,
        ).grid(row=7, column=0, columnspan=2, sticky="w", padx=10, pady=(10, 2))
        self.vel_cov_grid = self._build_matrix(cov_frame, row=8, mono_font=mono_font)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(200, self.update_loop)

    def _build_matrix(self, parent, row, mono_font):
        """3x3 symmetric matrix display, returns dict of Label refs keyed 'nn','ne',..."""
        headers = ["", "N", "E", "D"]
        for c, h in enumerate(headers):
            tk.Label(
                parent, text=h, fg=COLOR_LABEL, bg=COLOR_PANEL, font=mono_font,
                width=12, anchor="center",
            ).grid(row=row, column=c, padx=2, pady=1)

        rows = ["N", "E", "D"]
        keys = [["nn", "ne", "nd"], ["ne", "ee", "ed"], ["nd", "ed", "dd"]]
        labels = {}
        for r, rname in enumerate(rows):
            tk.Label(
                parent, text=rname, fg=COLOR_LABEL, bg=COLOR_PANEL, font=mono_font,
                width=12, anchor="center",
            ).grid(row=row + 1 + r, column=0, padx=2, pady=1)
            for c in range(3):
                key = keys[r][c]
                lbl = tk.Label(
                    parent, text="--", fg=COLOR_TEXT, bg=COLOR_PANEL, font=mono_font,
                    width=12, anchor="center",
                )
                lbl.grid(row=row + 1 + r, column=c + 1, padx=2, pady=1)
                labels.setdefault(key, lbl)
        return labels

    def _build_full_matrix(self, parent, row, mono_font, headers=("0", "1", "2")):
        """Full (non-symmetric) 3x3 matrix display, returns list of 9 Labels in row-major order."""
        for c, h in enumerate([""] + list(headers)):
            tk.Label(
                parent, text=h, fg=COLOR_LABEL, bg=COLOR_PANEL, font=mono_font,
                width=12, anchor="center",
            ).grid(row=row, column=c, padx=2, pady=1)

        labels = []
        for r in range(3):
            tk.Label(
                parent, text=headers[r], fg=COLOR_LABEL, bg=COLOR_PANEL, font=mono_font,
                width=12, anchor="center",
            ).grid(row=row + 1 + r, column=0, padx=2, pady=1)
            for c in range(3):
                lbl = tk.Label(
                    parent, text="--", fg=COLOR_TEXT, bg=COLOR_PANEL, font=mono_font,
                    width=12, anchor="center",
                )
                lbl.grid(row=row + 1 + r, column=c + 1, padx=2, pady=1)
                labels.append(lbl)
        return labels

    def _age_text_color(self, now, stamp):
        if stamp is None:
            return "no data", COLOR_STALE
        age = now - stamp
        color = COLOR_STALE if age > STALE_TIMEOUT_SEC else COLOR_TEXT
        return f"{age:.1f} s ago", color

    def update_loop(self):
        snap = self.node.snapshot()
        now = snap["now"]

        navpvt_msg, navpvt_stamp = snap["navpvt"]
        navrel_msg, navrel_stamp = snap["navrelposned"]
        navcov_msg, navcov_stamp = snap["navcov"]
        navsatfix_msg, navsatfix_stamp = snap["navsatfix"]

        # --- NavPVT ---
        age_text, age_color = self._age_text_color(now, navpvt_stamp)
        self.pvt_age.set(age_text, age_color)

        if navpvt_msg is not None:
            m = navpvt_msg
            fix_name = FIX_TYPE_NAMES.get(m.fix_type, f"UNKNOWN({m.fix_type})")
            self.tile_fix.set(fix_name, fix_type_status(m.fix_type))

            carr_soln = (m.flags >> 6) & 0b11
            carr_name = CARRIER_SOLN_NAMES.get(carr_soln, str(carr_soln))
            self.tile_carr_pvt.set(carr_name, carrier_status(carr_soln))

            head_veh_valid = bool(m.flags & 0x20)

            self.pvt_fixtype_raw.set(str(m.fix_type))
            self.pvt_numsv.set(str(m.num_sv))
            self.pvt_flags.set(str(m.flags))
            self.pvt_flags2.set(str(m.flags2))
            self.pvt_hacc.set(f"{m.h_acc} mm ({m.h_acc * 1e-3:.3f} m)")
            self.pvt_vacc.set(f"{m.v_acc} mm ({m.v_acc * 1e-3:.3f} m)")
            self.pvt_head_motion.set(
                f"{m.heading * 1e-5:8.3f} deg  (acc {m.head_acc * 1e-5:.2f} deg, "
                f"gSpeed {m.g_speed * 1e-3:.2f} m/s)"
            )
            self.pvt_head_veh.set(
                f"{m.head_veh * 1e-5:8.3f} deg  (valid={head_veh_valid})"
            )
        else:
            self.tile_fix.set("--", "unknown")
            self.tile_carr_pvt.set("--", "unknown")

        # --- NavRELPOSNED9 ---
        age_text, age_color = self._age_text_color(now, navrel_stamp)
        self.rel_age.set(age_text, age_color)

        if navrel_msg is not None:
            m = navrel_msg
            carr_soln = (m.flags >> 3) & 0b11
            carr_name = CARRIER_SOLN_NAMES.get(carr_soln, str(carr_soln))
            self.tile_carr_rel.set(carr_name, carrier_status(carr_soln))

            rel_pos_valid = bool(m.flags & 0x04)
            head_valid = bool(m.flags & 0x100)

            self.rel_flags.set(f"0b{m.flags:016b} (0x{m.flags:04X})")
            self.rel_relposvalid.set(str(rel_pos_valid))
            heading_str = (
                f"{m.rel_pos_heading * 1e-5:8.3f} deg" if head_valid else "n/a (invalid)"
            )
            self.rel_heading.set(heading_str)
            self.rel_head_acc.set(f"{m.acc_heading * 1e-5:.2f} deg")
            full_len_m = (m.rel_pos_length + m.rel_pos_hp_length * 1e-2) * 1e-2
            self.rel_length.set(f"{m.rel_pos_length} cm (raw) / {full_len_m:.3f} m (hp)")
        else:
            self.tile_carr_rel.set("--", "unknown")

        # --- NavSatFix ---
        age_text, age_color = self._age_text_color(now, navsatfix_stamp)
        self.fix_age.set(age_text, age_color)

        if navsatfix_msg is not None:
            m = navsatfix_msg
            status_name = NAVSATFIX_STATUS_NAMES.get(
                m.status.status, f"UNKNOWN({m.status.status})"
            )
            self.fix_status.set(f"{m.status.status} ({status_name})")
            self.fix_service.set(str(m.status.service))
            self.fix_lat.set(f"{m.latitude:.8f}")
            self.fix_lon.set(f"{m.longitude:.8f}")
            self.fix_alt.set(f"{m.altitude:.3f} m")
            cov_type_name = NAVSATFIX_COV_TYPE_NAMES.get(
                m.position_covariance_type, f"UNKNOWN({m.position_covariance_type})"
            )
            self.fix_cov_type.set(f"{m.position_covariance_type} ({cov_type_name})")
            for i, lbl in enumerate(self.fix_cov_grid):
                lbl.config(text=f"{m.position_covariance[i]: .6f}")

        # --- NavCOV ---
        age_text, age_color = self._age_text_color(now, navcov_stamp)
        self.cov_age.set(age_text, age_color)

        if navcov_msg is not None:
            m = navcov_msg
            self.cov_valid.set(f"{bool(m.pos_cov_valid)} / {bool(m.vel_cov_valid)}")

            pos_vals = {
                "nn": m.pos_cov_nn, "ne": m.pos_cov_ne, "nd": m.pos_cov_nd,
                "ee": m.pos_cov_ee, "ed": m.pos_cov_ed, "dd": m.pos_cov_dd,
            }
            for key, lbl in self.pos_cov_grid.items():
                lbl.config(text=f"{pos_vals[key]: .6f}")

            vel_vals = {
                "nn": m.vel_cov_nn, "ne": m.vel_cov_ne, "nd": m.vel_cov_nd,
                "ee": m.vel_cov_ee, "ed": m.vel_cov_ed, "dd": m.vel_cov_dd,
            }
            for key, lbl in self.vel_cov_grid.items():
                lbl.config(text=f"{vel_vals[key]: .6f}")

        self.root.after(200, self.update_loop)

    def on_close(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()


def ros_spin_thread(node: GnssStatusNode):
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


def main():
    rclpy.init(args=sys.argv)
    node = GnssStatusNode()

    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    GnssStatusGui(root, node)
    root.mainloop()

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
