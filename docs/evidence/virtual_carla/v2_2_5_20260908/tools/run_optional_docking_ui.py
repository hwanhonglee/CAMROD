#!/usr/bin/env python3
"""Optional reverse-PARKED -> visible Robot UI Dock -> charging evidence.

Default prints the planned action only. --run clicks the production Dock
button using CDP pointer events and accepts its actual native confirmation.
It publishes no ROS command and never changes battery, pose, charging input,
camera data or controller parameters. After a Dock click attempt, failure or
interruption requests the existing production UI STOP endpoint and observes
stopping; it never converts a failed Dock into a successful result.
"""
import argparse
import base64
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time


DOCK_CONFIRMATION_TEXT = "배터리 잔량과 관계없이 충전 도킹을 요청합니다. 진행하시겠습니까?"


def validate_initial_dock_preflight(snapshot, extras, image_present, ui_state, *, now):
    """Cross-check independent sources; never inject HTTP state into ROS."""
    required_ui = {"service_state": 0, "service_state_name": "DROP_ZONE_WAIT",
                   "mission_phase": "READY", "ready": True,
                   "engaged": False, "mission_dispatch_active": False}
    if not isinstance(ui_state, dict) or any(
            ui_state.get(key) != value or type(ui_state.get(key)) is not type(value)
            for key, value in required_ui.items()):
        raise RuntimeError("Initial fresh UI state is not inactive, disengaged, READY DROP_ZONE_WAIT")
    service = snapshot.get("service_state")
    if service is not None and (not isinstance(service, dict) or service.get("state") != 0
                               or service.get("state_name") != "DROP_ZONE_WAIT"):
        raise RuntimeError("Initial ROS service state contradicts the fresh UI DROP_ZONE_WAIT state")
    parking = snapshot.get("parking") or {}
    public, reverse = parking.get("dispatcher") or {}, parking.get("reverse") or {}
    if not (public.get("operating_state") == "PARKED"
            and "parking_method=reverse" in public.get("message", "")
            and reverse.get("operating_state") == "PARKED"):
        raise RuntimeError("Initial actual ROS dispatcher/reverse PARKED evidence is missing or inconsistent")
    speed = (snapshot.get("carla_odometry") or {}).get("speed_mps")
    if type(speed) not in (int, float) or not math.isfinite(speed) or not 0 <= speed < .05:
        raise RuntimeError("Initial actual ROS odometry is missing, non-finite or not stopped")
    platform = extras.get("platform") or {}
    received = platform.get("received_monotonic")
    if (platform.get("is_charging") is not False or type(received) not in (int, float)
            or not math.isfinite(received) or not 0 <= now-received < 1.0):
        raise RuntimeError("Initial actual ROS non-charging platform evidence is missing or stale")
    camera_received = extras.get("rear_camera_received_monotonic")
    # Preserve the original contract: only an observed true dummy flag rejects.
    # Missing flags remain explicitly unknown, never manufactured as false.
    if (image_present is not True or extras.get("dummy_camera_active") is True
            or type(camera_received) not in (int, float) or not math.isfinite(camera_received)
            or not 0 <= now-camera_received < 1.0):
        raise RuntimeError("Initial actual rear camera is missing/stale or observed dummy flag is active")


def observe_initial_ui_state(matrix, ui_url):
    record = {"source": "production HTTP GET /ui/state, not a ROS service message",
              "requested_utc": datetime.datetime.now(datetime.timezone.utc).isoformat()}
    state = matrix.UIClient(ui_url, timeout_s=3.0).state()
    keys = ("service_state", "service_state_name", "mission_phase", "ready", "engaged",
            "mission_dispatch_active", "mission_dispatch_generation", "mission_dispatch_owner")
    record["state"] = {key: state.get(key) for key in keys}
    record["received_utc"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    return record


def make_dock_browser_class(matrix):
    class DockBrowser(matrix.OperatorBrowserClient):
        """Reuse matrix CDP observation pump, deadlines and native dialog guard."""
        def __init__(self, *positional, **named):
            self._accept_expected_confirmation = False
            self._expected_confirmation_text = DOCK_CONFIRMATION_TEXT
            self._observed_confirmation = None
            super().__init__(*positional, **named)

        @property
        def accept_docking_dialog(self):
            return self._accept_expected_confirmation

        @accept_docking_dialog.setter
        def accept_docking_dialog(self, value):
            self._accept_expected_confirmation = bool(value)
            self._expected_confirmation_text = DOCK_CONFIRMATION_TEXT

        @property
        def dialogs(self):
            return ([{**self._observed_confirmation, "accepted": True}]
                    if self._observed_confirmation is not None else [])

    return DockBrowser


def safety_stop_after_failure(matrix, observer, browser, ui_url, evidence, *, timeout_s=3.0):
    """Best-effort existing STOP + fresh passive stop evidence, never acceptance."""
    record = {"requested": False, "stop_observed": False, "observations": [],
              "observation_errors": [], "started_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
              "scope": "failure cleanup only; original Dock remains FAIL/INTERRUPTED"}
    evidence["safety_stop"] = record
    before_stamp = getattr(observer, "_latest_odom_stamp_ns", None)
    before_samples = getattr(observer, "_carla_odom_samples", 0)
    try:
        # Shared browser.stop() clears its observation hook before the REST
        # request. A fatal observation must not suppress the safety request.
        if browser is not None:
            browser._observation_hook = None
        stopper = browser if browser is not None else matrix.UIClient(ui_url, timeout_s=3.0)
        record["requested"] = True
        record["request_response"] = stopper.stop()
    except Exception as error:
        record["request_error"] = str(error)[:600]
    started = time.monotonic()
    deadline = started + timeout_s
    settled_since = None
    while time.monotonic() < deadline:
        try:
            observer.spin_once(.05)
        except Exception as error:
            # Keep collecting the existing callbacks after a latched fatal
            # condition, solely to document braking. This never resumes motion.
            record["observation_errors"] = (record["observation_errors"] + [str(error)[:400]])[-3:]
        try:
            snapshot = observer.snapshot()
            stamp = getattr(observer, "_latest_odom_stamp_ns", None)
            fresh = (before_stamp is not None and stamp is not None and stamp > before_stamp
                     and getattr(observer, "_carla_odom_samples", 0) > before_samples)
            speed = (snapshot.get("carla_odometry") or {}).get("speed_mps")
            service = snapshot.get("service_state") or {}
            now = time.monotonic()
            stopped = (fresh and service.get("state") == 16 and isinstance(speed, (int, float))
                       and 0 <= speed <= .05)
            row = {"elapsed_s": now-started, "fresh_odometry": fresh, "odom_stamp_ns": stamp,
                   "service_state": service, "speed_mps": speed}
            if not record["observations"] or now-started-record["observations"][-1]["elapsed_s"] >= .2:
                record["observations"].append(row)
            settled_since = (now if settled_since is None else settled_since) if stopped else None
            if settled_since is not None and now-settled_since >= .3:
                record["stop_observed"] = True
                record["final_observation"] = row
                break
        except Exception as error:
            record["observation_errors"] = (record["observation_errors"] + [str(error)[:400]])[-3:]
    record["finished_utc"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--camrod-root", type=Path, default=Path(os.environ.get("CAMROD_SRC_ROOT", "/home/hong/camrod_ws/src")))
    parser.add_argument("--cdp-url", default="http://127.0.0.1:9224")
    parser.add_argument("--ui-url", default="http://127.0.0.1:8010")
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "optional_docking")
    parser.add_argument("--timeout", type=float, default=300)
    args = parser.parse_args()
    if not args.run:
        print("PLAN: from reverse PARKED + DROP_ZONE_WAIT, click visible Robot UI 도킹, confirm native dialog, observe real tag/camera and simulated contact feedback. No action executed.")
        return
    sys.path.insert(0, str(args.camrod_root / "scripts/virtual_carla"))
    import camping_site_matrix as matrix
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from avg_msgs.msg import AvgPlatformStatus, AvgAprilTagPose, MotionOperation, AvgBool
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    DockBrowser = make_dock_browser_class(matrix)

    args.output.mkdir(parents=True, exist_ok=False)
    observer = matrix.RosObservation("ego_vehicle")
    browser = None
    evidence = {"status": "RUNNING", "scenario": "reverse_park_then_explicit_ui_dock",
                "started_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                "audio_directory": "../mission_audio", "injected_sensor_or_state": False,
                "charging_feedback_source": "CARLA pose/speed/dwell contact emulator, not physical charger hardware"}
    evidence["test_source_provenance"] = {
        "scope": "test helpers only; not a claim about the running robot binary",
        "dock_helper": {"path": str(Path(__file__).resolve()),
                        "sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},
        "imported_matrix": {"path": str(Path(matrix.__file__).resolve()),
                            "sha256": hashlib.sha256(Path(matrix.__file__).read_bytes()).hexdigest()},
    }
    extras = {"platform": None, "tag": None, "tag_count_after_click": 0,
              "dummy_camera_active": None, "rear_camera_received_monotonic": None,
              "operation_requests": []}
    latest_image = [None]
    clicked = [False]
    bridge = CvBridge()

    def platform(msg):
        extras["platform"] = {"is_charging": bool(msg.is_charging), "control_mode": int(msg.control_mode),
                              "estop": bool(msg.estop), "error_code": int(msg.error_code),
                              "battery_percentage": float(msg.battery_percentage), "received_monotonic": time.monotonic()}

    def tag(msg):
        extras["tag"] = message_to_ordereddict(msg)
        if clicked[0] and msg.id == 3 and msg.family == "tag36h11":
            extras["tag_count_after_click"] += 1

    def operation(msg):
        extras["operation_requests"].append(message_to_ordereddict(msg))

    def rear_camera(msg):
        latest_image[0] = msg
        extras["rear_camera_received_monotonic"] = time.monotonic()

    subs = [
        observer.node.create_subscription(AvgPlatformStatus, "/platform/status", platform, qos_profile_sensor_data),
        observer.node.create_subscription(AvgAprilTagPose, "/perception/apriltag_parking_detector/tag_pose", tag, qos_profile_sensor_data),
        observer.node.create_subscription(MotionOperation, "/parking/operation", operation, 20),
        observer.node.create_subscription(Image, "/sensing/camera/econ_rear/image_rect", rear_camera, qos_profile_sensor_data),
        observer.node.create_subscription(AvgBool, "/sensing/camera/econ_rear/dummy_active", lambda msg: extras.__setitem__("dummy_camera_active", bool(msg.data)), qos_profile_sensor_data),
    ]

    def screenshot(name):
        value = browser._call("Page.captureScreenshot", {"format": "png", "captureBeyondViewport": False})
        (args.output / name).write_bytes(base64.b64decode(value["result"]["data"]))

    def rear_png(name):
        if latest_image[0] is None:
            return False
        cv2.imwrite(str(args.output / name), bridge.imgmsg_to_cv2(latest_image[0], desired_encoding="bgr8"))
        return True

    try:
        end = time.monotonic() + 4
        while time.monotonic() < end:
            observer.spin_once(.1)
        initial = observer.snapshot()
        evidence["initial"] = initial
        evidence["initial_extra"] = json.loads(json.dumps(extras))
        evidence["rear_camera_dummy_flag_scope"] = (
            "not observed; unknown is preserved, not evidence of false"
            if extras["dummy_camera_active"] is None else "actual ROS dummy-status received")
        evidence["initial_service_observation_source"] = (
            "actual ROS /service/state" if initial.get("service_state") is not None
            else "ROS event absent after 4 s late join; independent HTTP state cross-check required")
        evidence["initial_ui_state"] = observe_initial_ui_state(matrix, args.ui_url)
        observer.pump_ui_callbacks()
        validate_initial_dock_preflight(initial, extras, latest_image[0] is not None,
            evidence["initial_ui_state"]["state"], now=time.monotonic())
        browser = DockBrowser(args.cdp_url, args.ui_url, timeout_s=15,
                              observation_hook=observer.pump_ui_callbacks)
        browser._call("Page.enable", {})
        screenshot("01_reverse_parked_before_dock.png")
        rear_png("02_rear_camera_before_dock.png")
        evidence["pre_click_ui_state"] = observe_initial_ui_state(matrix, args.ui_url)
        observer.pump_ui_callbacks()
        evidence["pre_click_ros_observation"] = observer.snapshot()
        validate_initial_dock_preflight(evidence["pre_click_ros_observation"], extras,
            latest_image[0] is not None, evidence["pre_click_ui_state"]["state"], now=time.monotonic())
        browser.accept_docking_dialog = True
        clicked[0] = True
        evidence["pointer_click"] = browser._click(
            'button[title="자동 주차 정책과 별도로 충전 도킹 요청"]', "optional production Dock button")
        def dock_reply(probe):
            for item in reversed(probe.get("http", [])):
                body = item.get("body") or {}
                if item.get("url", "").endswith("/ui/dock") and item.get("method") == "POST":
                    if item.get("status") == 200 and body.get("success") and body.get("action") == "docking_requested":
                        return item
            return None
        evidence["production_ui_request"] = browser._wait_probe(dock_reply, "Dock HTTP reply from visible page")
        evidence["native_confirm"] = browser.dialogs
        start = time.monotonic()
        saved_tag = False
        last_record = 0
        with (args.output / "observations.jsonl").open("x") as stream:
            while time.monotonic() - start < args.timeout:
                observer.spin_once(.05)
                current = observer.snapshot()
                if time.monotonic() - last_record >= .5:
                    stream.write(json.dumps({"elapsed_s": time.monotonic()-start, "snapshot": current, "extra": extras}, ensure_ascii=False) + "\n")
                    stream.flush()
                    last_record = time.monotonic()
                if extras["tag_count_after_click"] and not saved_tag:
                    saved_tag = rear_png("03_actual_rear_tag_detected.png")
                    screenshot("04_ui_docking_in_progress.png")
                if matrix.parking_completion(current, "charging") == "charging":
                    forced_start = any(item.get("source") == "http:manual_dock:force_docking" and item.get("operation") == int(MotionOperation.START) for item in extras["operation_requests"])
                    platform_now = extras["platform"] or {}
                    if not (forced_start and extras["tag_count_after_click"] > 0 and saved_tag
                            and platform_now.get("is_charging") is True
                            and time.monotonic()-platform_now.get("received_monotonic", 0) < 1.0
                            and current.get("carla_odometry", {}).get("speed_mps", 1) < .05):
                        raise RuntimeError("Charging state reached but command/tag/camera/platform evidence missing")
                    screenshot("05_ui_charging_complete.png")
                    rear_png("06_rear_camera_charging_complete.png")
                    evidence.update(status="PASS", duration_s=time.monotonic()-start, final=current,
                                    final_extra=extras, charging_confirmed=True)
                    break
            else:
                raise RuntimeError("Optional docking timed out; inspect observations for actual stopping reason")
    except BaseException as error:
        evidence.update(status="INTERRUPTED" if isinstance(error, KeyboardInterrupt) else "FAIL",
                        error=str(error), final_extra=extras)
        if clicked[0]:
            try:
                safety_stop_after_failure(matrix, observer, browser, args.ui_url, evidence)
            except BaseException as stop_error:
                evidence.setdefault("safety_stop", {})["cleanup_error"] = str(stop_error)[:600]
        try:
            evidence["final"] = observer.snapshot()
        except Exception as snapshot_error:
            evidence["final_observation_error"] = str(snapshot_error)[:600]
        raise
    finally:
        evidence["completed_utc"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        (args.output / "result.json").write_text(json.dumps(evidence, ensure_ascii=False, indent=2))
        if browser is not None:
            browser.close()
        observer.close()
    print(json.dumps({"status": evidence["status"], "output": str(args.output)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
