#!/usr/bin/env python3
"""HH_261006 - One bounded local-CARLA manual diagnostic, never scenario proof.

Requires the CARLA/ROS environment and an existing visible localhost Robot UI.
Only real browser key events command motion. No ROS publisher, CARLA control,
parameter change, mission dispatch, or automatic retry is permitted here.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time


UI_URL = "http://127.0.0.1:8010"
CDP_URL = "http://127.0.0.1:9224"
MAX_AGE_S = 0.5
AXES = ("x", "y", "z", "roll", "pitch", "yaw")


def duration(value):
    number = float(value)
    if not math.isfinite(number) or not 0.0 < number <= 3.0:
        raise argparse.ArgumentTypeError("duration must be finite and in (0, 3] seconds")
    return number


def require_idle_mission(state, *, before_arm=False):
    # HH_261006 - Unknown authority is not equivalent to an inactive mission.
    if (state.get("mission_dispatch_active") is not False
            or type(state.get("mission_dispatch_generation")) is not int
            or state.get("mission_dispatch_generation") != 0
            or state.get("mission_dispatch_owner") != ""
            or state.get("mission_dispatch_site") != ""
            or state.get("backend_startup_recovery")
            or any(state.get(name) for name in (
                "mission_retryable", "mission_retry_site", "mission_retry_owner",
                "departure_delay_active", "battery_return_pending", "battery_return_started",
                "redock_pending", "redock_waiting_for_can"))
            or (before_arm and state.get("engaged") is not False)):
        raise RuntimeError("active, retryable, unknown, or changed mission authority")
    resume = state.get("manual_resume")
    if (not isinstance(resume, dict) or resume.get("pending") is not False
            or any(resume.get(name) for name in (
                "available", "eligible", "mission_id", "can_resume", "token"))):
        raise RuntimeError("unknown or existing manual-resume mission context")


def require_zero(command):
    if not command or any(not math.isfinite(command.get(axis, math.nan))
                          or abs(command[axis]) > 0.001 for axis in AXES):
        raise RuntimeError("nonzero or unknown input/output command")


def motion_metrics(sample, origin):
    actor = sample["actor"]
    position = actor["transform"]["location_m"]
    initial = origin["actor"]["transform"]["location_m"]
    speed = math.sqrt(sum(actor["linear_velocity_mps"][axis] ** 2 for axis in "xyz"))
    displacement = math.hypot(position["x"] - initial["x"], position["y"] - initial["y"])
    yaw = (actor["yaw_degrees"] - origin["actor"]["yaw_degrees"] + 180.0) % 360.0 - 180.0
    if not all(math.isfinite(value) for value in (speed, displacement, yaw)):
        raise RuntimeError("nonfinite physical motion feedback")
    return {"speed_mps": speed, "displacement_m": displacement, "yaw_drift_deg": yaw}


def require_bounded_motion(sample, origin, *, stationary=False):
    if sample["actor"]["id"] != origin["actor"]["id"]:
        raise RuntimeError("physical actor identity changed")
    if len(sample["wheels"]) != 4 or any(wheel["is_in_air"] for wheel in sample["wheels"]):
        raise RuntimeError("four grounded physical wheels are required")
    values = motion_metrics(sample, origin)
    if (values["speed_mps"] > (0.02 if stationary else 0.4)
            or values["displacement_m"] > (0.02 if stationary else 0.75)
            or abs(values["yaw_drift_deg"]) > (1.0 if stationary else 12.0)):
        raise RuntimeError("physical motion limit exceeded: " + json.dumps(values))
    if stationary and abs(sample["actor"]["angular_velocity_degrees_per_second"]["z"]) > 1.0:
        raise RuntimeError("actor is already rotating")
    return values


class Observations:
    """HH_261006 - Passive workers never share the pulse watchdog's I/O path."""

    def __init__(self):
        self.start = time.monotonic()
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.latest = {}
        self.rows = []
        self.errors = []
        self.threads = []

    def record(self, source, data, source_stamp=None):
        now = time.monotonic()
        row = {"source": source, "received_monotonic_s": now,
               "elapsed_s": now - self.start, "wall_time_ns": time.time_ns(), "data": data}
        with self.lock:
            previous = self.latest.get(source)
            row["source_stamp"] = source_stamp
            row["source_advanced_monotonic_s"] = (
                previous["source_advanced_monotonic_s"] if previous
                and source_stamp is not None and previous["source_stamp"] == source_stamp else now)
            self.rows.append(row)
            self.latest[source] = row

    def get(self, source, *, advancing=False):
        with self.lock:
            if self.errors:
                raise RuntimeError("observer failed: " + self.errors[0])
            row = self.latest.get(source)
        now = time.monotonic()
        # HH_261006 - Gate status is published every 500 ms; allow scheduling jitter.
        maximum_age = 0.8 if source == "gate" else MAX_AGE_S
        if row is None or not 0 <= now - row["received_monotonic_s"] <= maximum_age:
            raise RuntimeError("missing or stale " + source)
        if advancing and now - row["source_advanced_monotonic_s"] > MAX_AGE_S:
            raise RuntimeError("non-advancing " + source)
        return row["data"]

    def worker(self, function):
        def run():
            try:
                function()
            except Exception as error:
                with self.lock:
                    self.errors.append(str(error))
        thread = threading.Thread(target=run, daemon=True)
        self.threads.append(thread)
        thread.start()


def run(args, result):
    # HH_261006 - Lazy dependencies let syntax/pure-guard tests run without ROS/CARLA.
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts/virtual_carla"))
    import carla
    import rclpy
    from avg_msgs.msg import ModuleState
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from manual_4ws_evidence import ManualDriveBrowser
    from camping_site_matrix import UIClient
    from record_physical_wheel_telemetry import capture_sample, select_unique_ranger

    observed = Observations()
    browser = release_browser = node = None
    owned_login = False
    phase = {"value": "preflight"}
    ui = UIClient(UI_URL, timeout_s=0.2)
    rclpy.init()
    try:
        node = rclpy.create_node("bounded_manual_crab_diagnostic")

        def receive(source):
            def callback(message):
                stamp = None
                if isinstance(message, Twist):
                    data = dict(zip(AXES, (message.linear.x, message.linear.y, message.linear.z,
                                           message.angular.x, message.angular.y, message.angular.z)))
                elif isinstance(message, Odometry):
                    p, v = message.pose.pose.position, message.twist.twist.linear
                    q = message.pose.pose.orientation
                    data = {"position": dict(x=p.x, y=p.y, z=p.z),
                            "orientation": dict(x=q.x, y=q.y, z=q.z, w=q.w),
                            "linear_velocity": dict(x=v.x, y=v.y, z=v.z),
                            "yaw_rate_radps": message.twist.twist.angular.z}
                    stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
                else:
                    data = {"state": message.operating_state, "message": message.message,
                            "level": message.level}
                observed.record(source, data, stamp)
            return callback

        subscriptions = [node.create_subscription(kind, topic, receive(name), qos_profile_sensor_data)
                         for kind, topic, name in (
                             (Twist, "/control/manual_cmd_vel_ros", "manual"),
                             (Twist, "/control/cmd_vel_ros", "output"),
                             (Odometry, "/carla/ego_vehicle/odometry", "odometry"),
                             (ModuleState, "/control/cmd_vel_safety_gate/status", "gate"))]

        def ros_loop():
            while not observed.stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.02)
        observed.worker(ros_loop)

        # HH_261006 - Read-only first tick initializes a new client's actor cache.
        client = carla.Client("127.0.0.1", 2000)
        client.set_timeout(4.0)
        world = client.get_world()
        world.wait_for_tick(4.0)
        actor = select_unique_ranger(world)
        client.set_timeout(0.2)
        result["actor_id"] = actor.id

        def physical_loop():
            sequence = 0
            due = time.monotonic()
            while not observed.stop.is_set():
                sample = capture_sample(carla, actor, sequence, time.monotonic() - observed.start,
                                        time.time_ns())
                sample["diagnostic_phase"] = phase["value"]
                observed.record("physical", sample, sample["world_frame"])
                sequence += 1
                due += 0.1
                observed.stop.wait(max(0.0, due - time.monotonic()))
        observed.worker(physical_loop)

        state = ui.state()
        require_idle_mission(state, before_arm=True)
        result["before_state"] = state
        browser = ManualDriveBrowser(CDP_URL, UI_URL, timeout_s=2.0)
        before = browser.snapshot()
        result["before_ui"] = before
        if before.get("armed") is not False:
            raise RuntimeError("manual control must be unarmed before owned login")
        # A second CDP transport controls the same page, not a second manual lease.
        # HH_261006 - Slow observer RPCs must never occupy the release connection.
        release_browser = ManualDriveBrowser(CDP_URL, UI_URL, timeout_s=0.2)
        if release_browser._target.get("id") != browser._target.get("id"):
            raise RuntimeError("release connection does not target the same visible page")
        time.sleep(0.6)
        initial = observed.get("physical", advancing=True)
        require_bounded_motion(initial, initial, stationary=True)
        require_zero(observed.get("output"))
        # Unarmed UI need not publish manual zeros periodically. Any observed
        # input must nevertheless be zero; missing input is recorded explicitly.
        with observed.lock:
            manual_before = observed.latest.get("manual")
        if manual_before:
            require_zero(manual_before["data"])
        result["prelogin_manual_observation"] = manual_before
        observed.get("odometry", advancing=True)
        require_idle_mission(ui.state(), before_arm=True)
        if browser.snapshot().get("armed") is not False:
            raise RuntimeError("manual owner changed before login")
        owned_login = True
        result["login"] = browser.ensure_fresh_administrator_session()
        require_idle_mission(ui.state(), before_arm=True)
        require_bounded_motion(observed.get("physical", advancing=True), initial, stationary=True)
        require_zero(observed.get("output"))
        result["arm"] = browser.arm()
        limits = {key: result["arm"]["snapshot"].get(key) for key in (
            "linearLimitMps", "lateralLimitMps", "angularLimitRadps", "scale")}
        result["unchanged_manual_limits"] = limits
        browser.timeout_s = 0.2

        def ui_loop():
            while not observed.stop.is_set():
                observed.record("ui_state", ui.state())
                observed.record("ui_panel", browser.snapshot())
                observed.stop.wait(0.05)
        observed.worker(ui_loop)

        def check(armed, origin, stationary=False):
            require_idle_mission(observed.get("ui_state"))
            panel = observed.get("ui_panel")
            if (panel.get("armed") is not armed or panel.get("connected") is not True
                    # HH_261007 - Current UI appends gate permission text to
                    # ARMED; the live ENABLED gate check below stays mandatory.
                    or panel.get("panelVisible") is not True
                    or not str(panel.get("stateText", "")).startswith("ARMED")
                    or any(panel.get(key) != value for key, value in limits.items())):
                raise RuntimeError("manual owner, connectivity, or input limits changed")
            if observed.get("gate")["state"] != "ENABLED":
                raise RuntimeError("gate is not ENABLED")
            observed.get("odometry", advancing=True)
            return require_bounded_motion(observed.get("physical", advancing=True), origin,
                                          stationary=stationary)

        # HH_261006 - Arming may precede ENABLED; wait without issuing any drive key.
        deadline = time.monotonic() + 4.0
        while True:
            try:
                check(True, initial, stationary=True)
                require_zero(observed.get("manual"))
                require_zero(observed.get("output"))
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.02)
        origin = observed.get("physical", advancing=True)
        result["pulse_origin"] = origin["actor"]
        phase["value"] = "pulse"
        pulse_start = time.monotonic()
        result["key_down_requested_monotonic_s"] = pulse_start
        # HH_261006 - Only this one real key-down can start the diagnostic.
        release_browser.dispatch_key(args.key, True)
        try:
            while time.monotonic() - pulse_start < args.duration_s:
                result["last_motion_metrics"] = check(True, origin)
                for source in ("manual", "output"):
                    command = observed.get(source)
                    if (any(not math.isfinite(command[axis]) for axis in AXES)
                            or any(abs(command[axis]) > 0.001 for axis in AXES if axis != "y")
                            or abs(command["y"]) > 0.4
                            or command["y"] * (1 if args.key == "KeyZ" else -1) < -0.001):
                        raise RuntimeError("unexpected input/owner on " + source)
                observed.stop.wait(min(0.01, max(0.0, args.duration_s - (time.monotonic() - pulse_start))))
            result["status"] = "diagnostic_duration_reached"
        finally:
            result["key_up_requested_monotonic_s"] = time.monotonic()
            release_browser.dispatch_key(args.key, False)
    finally:
        # HH_261006 - Never disarm a pre-existing owner on a rejected preflight.
        # Once login is ours, each cleanup action is attempted independently.
        phase["value"] = "cleanup"
        cleanup = result.setdefault("cleanup", {})
        if owned_login and release_browser:
            for label, action in (
                    ("key_up", lambda: release_browser.dispatch_key(args.key, False)),
                    ("zero", release_browser.zero), ("disarm", release_browser.disarm)):
                try:
                    action()
                    cleanup[label] = "sent"
                except Exception as error:
                    cleanup[label] = "failed: " + str(error)
            time.sleep(0.6)
            try:
                final = release_browser.snapshot()
                cleanup["ui"] = final
                require_zero(observed.get("output"))
                sample = observed.get("physical", advancing=True)
                require_bounded_motion(sample, sample, stationary=True)
                if any(abs(wheel["external_drive_torque_nm"]) > 1e-6 for wheel in sample["wheels"]):
                    raise RuntimeError("physical external torque remains after cleanup")
                require_idle_mission(ui.state(), before_arm=True)
                cleanup["verified_unarmed_and_stationary"] = final.get("armed") is False
            except Exception as error:
                cleanup["verification_error"] = str(error)
        observed.stop.set()
        for thread in observed.threads:
            thread.join(timeout=0.5)
        with observed.lock:
            result["observations"] = list(observed.rows)
            result["observer_errors"] = list(observed.errors)
        if release_browser:
            result["real_key_interactions"] = list(release_browser._interactions)
        for connection in (browser, release_browser):
            if connection:
                connection.close()
        if node:
            node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="NEW .json evidence file")
    parser.add_argument("--key", choices=("KeyZ", "KeyC"), required=True)
    parser.add_argument("--duration-s", type=duration, default=2.5)
    args = parser.parse_args(argv)
    if args.output.suffix.lower() != ".json" or not args.output.parent.is_dir():
        parser.error("output must be a NEW .json file in an existing directory")
    result = {"schema": "camrod.manual_crab_diagnostic.v1", "diagnostic_only": True,
              "scenario_evidence": False, "scenario_pass": False, "motion_mode": "manual",
              "key": args.key, "requested_duration_s": args.duration_s,
              "limits": {"duration_s": 3.0, "yaw_drift_deg": 12.0,
                         "displacement_m": 0.75, "speed_mps": 0.4},
              "wheel_sample_target_hz": 10, "manual_defaults_changed": False,
              "gates_changed": False, "ui_url": UI_URL, "cdp_url": CDP_URL,
              "carla_host": "127.0.0.1", "status": "preflight_rejected"}
    # HH_261006 - Exclusive reservation prevents overwriting any original evidence.
    try:
        output = args.output.open("x", encoding="utf-8")
    except OSError as error:
        parser.error(str(error))
    previous_handler = signal.getsignal(signal.SIGTERM)

    def interrupted(signum, frame):
        raise KeyboardInterrupt("termination requested")
    signal.signal(signal.SIGTERM, interrupted)
    exit_code = 0
    try:
        run(args, result)
        if result.get("cleanup", {}).get("verified_unarmed_and_stationary") is not True:
            result["status"] = "cleanup_unverified"
            exit_code = 1
    except (Exception, KeyboardInterrupt) as error:
        result.update(status="diagnostic_aborted", error=str(error))
        exit_code = 1
    finally:
        signal.signal(signal.SIGTERM, previous_handler)
        with output:
            json.dump(result, output, indent=2, allow_nan=False)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
    print(json.dumps({key: result[key] for key in ("status", "diagnostic_only", "scenario_pass")}))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
