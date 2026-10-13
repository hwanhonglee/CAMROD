#!/usr/bin/env python3
"""HH_261008 - One visible-UI B1-B13 delivery and return, diagnostic only.

Requires the existing ROS/CARLA Python environment and visible operator UI.
Never launches a process, changes a profile, ticks CARLA, or moves an actor
directly. A road start requires explicit --allow-road-start; it is not initial
parking proof. Both start modes require genuinely observed final parking.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sys
import time

SOURCE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOURCE / "scripts/virtual_carla"))
from camping_site_matrix import (  # noqa: E402
    MatrixError, RosObservation, UIClient, arrival_ready_for_return,
    contains_ordered_subsequence, expected_site_phases, load_drop_zone,
    load_sites, mission_identity, parking_completion, return_acknowledgement,
    return_context,
)
from record_physical_wheel_telemetry import capture_sample, select_unique_ranger  # noqa: E402
from repeat_simulator_missions import (  # noqa: E402
    Intervention, Runner as RepeatRunner, SupervisedBrowserClient, check_owner,
    stationary_parked, stop_owned, verify_forward_road_return_runtime,
)

MAX_SPEED_MPS = 2.0
UPRIGHT_DEGREES = 18.0
FRESH_SECONDS = 1.5
INHERITED_STOP_MAX_SECONDS = 16.0
DISPATCH_HANDOFF_MAX_SECONDS = 0.5


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def exact_clear(state):
    return (state.get("mission_dispatch_active") is False
            and type(state.get("mission_dispatch_generation")) is int
            and state["mission_dispatch_generation"] == 0
            and all(state.get("mission_dispatch_" + key) == ""
                    for key in ("site", "owner", "intent")))


def require_owned_active(state, identity):
    """HH_261007 - Inactive identity is never authority to click Stop."""
    check_owner(state, identity, stopping=True)
    if identity is None or state.get("mission_dispatch_active") is not True:
        raise Intervention("no still-active owned test mission to stop")
    if mission_identity(state) != identity:
        raise Intervention("test mission ownership changed")


def physical_bounds(sample, actor_id):
    actor = sample.get("actor") or {}
    if (actor.get("id") != actor_id or actor.get("type_id") != "vehicle.ranger.default"
            or actor.get("role_name") != "ego_vehicle" or sample.get("api_version") != 1):
        raise MatrixError("physical telemetry actor/API identity changed")
    wheels = sample.get("wheels") or []
    if len(wheels) != 4 or {w.get("wheel_index") for w in wheels} != {0, 1, 2, 3}:
        raise MatrixError("four distinct live physical wheels are required")
    telemetry = sample.get("telemetry") or {}
    max_speed = 0.0
    for transform, velocity in (
            (actor.get("transform") or {}, actor.get("linear_velocity_mps") or {}),
            (telemetry.get("body_transform") or {},
             telemetry.get("body_linear_velocity_mps") or {})):
        rotation = transform.get("rotation_degrees") or {}
        position = transform.get("location_m") or {}
        values = [velocity.get(k) for k in ("x", "y", "z")]
        if not all(finite(v) for v in values + [position.get(k) for k in ("x", "y", "z")]
                   + [rotation.get(k) for k in ("pitch", "yaw", "roll")]):
            raise MatrixError("missing/non-finite physical motion telemetry")
        speed = math.sqrt(sum(v * v for v in values))
        max_speed = max(max_speed, speed)
        if speed > MAX_SPEED_MPS:
            raise MatrixError("physical speed exceeds 2 m/s diagnostic bound")
        if max(abs(rotation["pitch"]), abs(rotation["roll"])) > UPRIGHT_DEGREES:
            raise MatrixError("physical roll/pitch exceeds 18 degree upright bound")
    return max_speed


class PhysicalDistance:
    """HH_261007 - Secondary sampled XY distance, never the primary mileage goal."""
    def __init__(self):
        self.previous = None
        self.last_advance = None
        self.xy_m = 0.0
        self.sim_seconds = 0.0

    def observe(self, sample, now):
        telemetry = sample["telemetry"]
        frame = telemetry.get("world_frame")
        delta = telemetry.get("world_delta_seconds")
        location = telemetry["body_transform"]["location_m"]
        if type(frame) is not int or frame <= 0 or not finite(delta) or not 0 < delta <= 1:
            raise MatrixError("invalid physical frame/time evidence")
        current = (frame, location["x"], location["y"])
        if self.previous is not None:
            frames = frame - self.previous[0]
            if frames < 0:
                raise MatrixError("physical world frame regressed")
            if frames == 0:
                if now - self.last_advance > FRESH_SECONDS:
                    raise MatrixError("physical world frame stopped advancing")
                return
            step = math.hypot(current[1] - self.previous[1], current[2] - self.previous[2])
            if step > MAX_SPEED_MPS * frames * delta + 0.1:
                raise MatrixError("physical displacement discontinuity; distance not bridged")
            self.xy_m += step
            self.sim_seconds += frames * delta
        self.previous, self.last_advance = current, now


class FreshObservation(RosObservation):
    """HH_261007 - Timestamp real callbacks; never invent cold-start history."""
    def __init__(self, *args, **kwargs):
        self.received = {}
        self.service_stop_sequence = 0
        super().__init__(*args, **kwargs)

    def _service(self, msg):
        # HH_261007 - Count actual stop callbacks, not repeated cached snapshots.
        if int(getattr(msg, "state", -1)) == 16:
            self.service_stop_sequence += 1
        super()._service(msg)

    def _pose(self, msg):
        super()._pose(msg)
        self.received["pose"] = time.monotonic()

    def _odom(self, msg):
        super()._odom(msg)
        self.received["carla_odometry"] = time.monotonic()

    def _gate(self, msg):
        super()._gate(msg)
        self.received["gate"] = time.monotonic()

    def fresh(self, now):
        return all(0 <= now - self.received.get(key, -math.inf) <= FRESH_SECONDS
                   for key in ("pose", "carla_odometry", "gate"))


class GuardedBrowser(SupervisedBrowserClient):
    # HH_261007 - Keep this guard even when failed-sensor cleanup removes the
    # observation hook. Recheck at actual input, after DOM readiness waits.
    input_guard = None
    dispatch_confirmation_started = None
    dispatch_confirmation_finished = None
    _cleanup_stop_confirmation_selector = None

    def _click(self, selector, description):
        # HH_261007 - Cleanup can start on either production service surface.
        # Select only a visible, unobscured button; the normal pointer path
        # still rechecks readiness, stability and authority before real input.
        stop_confirmation = None
        if selector == ".dd-stop-action":
            self._cleanup_stop_confirmation_selector = None
            routes = ((".dd-stop-action", ".dd-stop-confirm"),
                      (".preview-stop-btn", '[data-ui="operator-stop-confirm-yes"]'))
            eligible = []
            for action, confirmation in routes:
                target = self._element(action)
                if (target.get("visibleCount") == 1 and target.get("disabled") is False
                        and target.get("hit") is True and target.get("settled") is True
                        and target.get("pageReady") is True):
                    eligible.append((action, confirmation))
            if len(eligible) != 1:
                raise MatrixError("cleanup needs exactly one visible production stop surface")
            selector, stop_confirmation = eligible[0]
        elif selector == ".dd-stop-confirm":
            selector = self._cleanup_stop_confirmation_selector
            self._cleanup_stop_confirmation_selector = None
            if selector is None:
                raise MatrixError("cleanup stop confirmation has no selected production surface")
        # HH_261007 - Service/menu clicks are still preflight. The final typed
        # code click establishes dispatch ambiguity; only its completed real
        # pointer action closes the inherited idle-stop exception. Marking it
        # before pointer readiness would reject the old stop before any send.
        final_dispatch = selector == '[data-ui="operator-site-code-confirm"]'
        if final_dispatch and self.dispatch_confirmation_started is not None:
            self.dispatch_confirmation_started()
        result = super()._click(selector, description)
        if stop_confirmation is not None:
            self._cleanup_stop_confirmation_selector = stop_confirmation
        if final_dispatch and self.dispatch_confirmation_finished is not None:
            self.dispatch_confirmation_finished()
        return result

    def _call(self, method, params):
        if method in {"Input.dispatchMouseEvent", "Input.insertText",
                      "Input.dispatchKeyEvent", "Page.handleJavaScriptDialog"}:
            if self.input_guard is not None:
                self.input_guard()
        return super()._call(method, params)


class GuardedUI(UIClient):
    stop_guard = None

    def stop(self):
        if self.stop_guard is not None:
            self.stop_guard()
        return super().stop()


class Evidence:
    """HH_261007 - Stream <=5 Hz wheel records; preserve prior files exclusively.

    The JSON closes on success, handled errors and SIGINT/SIGTERM. SIGKILL or
    power loss can leave a partial JSON prefix; flushed samples remain on disk.
    """
    def __init__(self, stream):
        self.stream, self.count = stream, 0
        stream.write('{"schema":"camrod.carla.site_roundtrip.diagnostic.v1","samples":[')
        stream.flush()

    def sample(self, record):
        encoded = json.dumps(record, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        self.stream.write((",\n" if self.count else "\n") + encoded)
        self.stream.flush()
        self.count += 1

    def finish(self, result):
        self.stream.write('],"result":' + json.dumps(result, ensure_ascii=False, allow_nan=False) + '}\n')
        self.stream.flush()


def start_ready(state, snapshot, *, road_start, drop_zone, fresh, physical_speed):
    check_owner(state, None)
    if not exact_clear(state) or state.get("engaged") is not False:
        return False
    if not fresh or physical_speed > 0.03 or (snapshot.get("physical") or {}).get("ready") is not True:
        return False
    pose = snapshot.get("pose") or {}
    speed = (snapshot.get("carla_odometry") or {}).get("speed_mps")
    if not all(finite(pose.get(k)) for k in ("x_m", "y_m", "z_m")):
        return False
    if not finite(speed) or not 0 <= speed <= 0.03:
        return False
    gate = snapshot.get("gate") or {}
    if gate.get("level") != 0 or "MANUAL" in str(gate.get("operating_state", "")):
        return False
    # HH_261007 - Explicit diagnostic road starts do not fabricate initial
    # drop-zone parking and do not override the native B11 roadside policy.
    return road_start or stationary_parked(
        state, snapshot, drop_zone, allow_late_join_ui_state=True)


class Roundtrip:
    observe_parking_clear = RepeatRunner.observe_parking_clear

    def __init__(self, args, observer, ui, carla, actor, evidence):
        self.args, self.observer, self.ui = args, observer, ui
        self.carla, self.actor, self.evidence = carla, actor, evidence
        self.browser, self.identity = None, None
        self.started = time.monotonic()
        self.last_sample = self.last_visibility = -math.inf
        self.deadline = math.inf
        self.stage = "preflight"
        self.dispatch_pending = self.stopping = False
        self._inherited_dispatch_stop = None
        self._parking_active_identity = self._parking_clear_since = None
        self._parking_completion_observed = False
        self.distance = PhysicalDistance()
        self.physical_speed = math.inf
        self.drop_zone = load_drop_zone(SOURCE / "camrod_bringup/config/map/drop_zones.yaml")
        self.site = load_sites(SOURCE / "camrod_planning/config/camping_sites.yaml")[args.site]
        # HH_261008 - All three native roadside sites retain their catalog policy.
        if args.site in ("B11", "B12", "B13") and self.site.service_mode != "roadside_stop":
            raise MatrixError(f"{args.site} source policy is not roadside_stop; no override permitted")
        self.report = {"status": "RUNNING", "site": args.site, "intent": "delivery",
                       "started_utc": utc_now(), "events": [], "roundtrip_verified": False,
                       "formal_acceptance": False, "service_mode": self.site.service_mode,
                       "start_basis": "explicit_stationary_road_start" if args.allow_road_start else "stationary_parked",
                       "bounds": {"per_leg_wall_seconds": args.leg_timeout_s,
                                  "sample_hz_max": args.sample_hz, "speed_mps_max": MAX_SPEED_MPS,
                                  "upright_degrees_max": UPRIGHT_DEGREES},
                       "distance_basis": "sampled_atomic_physical_body_xy_secondary_not_primary_goal"}

    def event(self, name, **details):
        record = {"utc": utc_now(), "elapsed_wall_s": time.monotonic() - self.started,
                  "event": name, **details}
        self.report["events"].append(record)
        print(json.dumps(record, ensure_ascii=False), flush=True)

    def guard(self):
        state = self.ui.state()
        if self.stopping:
            require_owned_active(state, self.identity)
        else:
            check_owner(state, self.identity)
            if self.identity is None and (not exact_clear(state) or state.get("engaged") is not False):
                raise Intervention("dispatch denied: operator or ambiguous inactive ownership")
        if self.browser is not None:
            self.check_page()

    def sample(self):
        self.observer.pump_ui_callbacks()
        now = time.monotonic()
        if now >= self.deadline:
            raise MatrixError(f"bounded {self.stage} timeout")
        state, snap = self.ui.state(), self.observer.snapshot()
        # HH_261007 - A successful final dispatch click can publish before its
        # response returns. Until identity is confirmed, observe but never stop
        # an ambiguous mission, even one with the requested destination.
        candidate = self.identity
        if self.dispatch_pending and state.get("mission_dispatch_active") is True:
            candidate = mission_identity(state)
        check_owner(state, candidate, allow_inactive=self.stage == "return")
        # HH_261007 - Manual disarm legitimately leaves OPERATOR_STOPPED (16)
        # at an inactive road start. Never extend this exception past dispatch,
        # an owned mission, execution errors, or an ambiguous cleared identity.
        idle_stopped_preflight = (self.stage == "preflight" and self.identity is None
                                  and exact_clear(state) and state.get("engaged") is False)
        inherited_dispatch_stop = self._allow_inherited_dispatch_stop(state, snap, now)
        if (not (idle_stopped_preflight or inherited_dispatch_stop) and (state.get("service_state") == 16
                or (snap.get("service_state") or {}).get("state") == 16)):
            # HH_261007 - Keep both cache values at rejection; otherwise a
            # pre-sample handoff failure leaves only the preceding healthy row.
            report = getattr(self, "report", None)
            if report is not None:
                report["service_stop_fault"] = {
                    "ui_service_state": state.get("service_state"),
                    "ros_service_state": (snap.get("service_state") or {}).get("state"),
                    "engaged": state.get("engaged"),
                    "dispatch_stop_observation": getattr(self, "_dispatch_stop_observation", None)}
            raise MatrixError("service failure observed")
        if "MANUAL" in str((snap.get("gate") or {}).get("operating_state", "")):
            raise Intervention("manual control gate observed")
        if (snap.get("collision_evidence") or {}).get("event_count", 0):
            raise MatrixError("collision evidence observed")
        if self.identity is not None:
            self.observe_parking_clear(state, snap, self.stage == "return", now)
            if not self.observer.fresh(now):
                raise MatrixError("pose/odometry/gate observation is stale")
        if now - self.last_sample >= 1.0 / self.args.sample_hz:
            self.last_sample = now
            physical = capture_sample(self.carla, self.actor, sequence=self.evidence.count,
                                      elapsed_s=now - self.started, wall_time_ns=time.time_ns())
            record = {"stage": self.stage, "elapsed_wall_s": now - self.started,
                      "physical": physical, "ui": {key: state.get(key) for key in (
                          "mission_dispatch_active", "mission_dispatch_generation", "mission_dispatch_site",
                          "mission_dispatch_owner", "mission_dispatch_intent", "service_state",
                          "service_state_name", "control_mode", "operation_mode", "engaged",
                          "manual_resume", "mission_execution_error", "service_state_description")},
                      "inherited_dispatch_stop_allowed": inherited_dispatch_stop,
                      "dispatch_stop_observation": getattr(self, "_dispatch_stop_observation", None),
                      "observation": {key: snap.get(key) for key in (
                          "site", "drop_zone", "gate", "parking", "cmd_vel", "pose", "carla_odometry", "physical")}}
            # Preserve the actual violating wheel sample before failing closed.
            self.evidence.sample(record)
            self.physical_speed = physical_bounds(physical, self.actor.id)
            self.distance.observe(physical, now)
        return state, snap

    def check_page(self):
        page = self.browser._evaluate(
            "({visibility: document.visibilityState, href: location.href, "
            "manualArmed: Boolean(document.querySelector('[data-ui=\"manual-drive-panel\"][data-armed=\"true\"]'))})")
        if (not isinstance(page, dict) or page.get("visibility") != "visible"
                or page.get("manualArmed") is not False
                or str(page.get("href", "")).rstrip("/") != self.args.ui_url.rstrip("/")):
            raise Intervention("operator page hidden, navigated away, or manually armed; no further UI control")

    def visible(self):
        now = time.monotonic()
        if now - self.last_visibility >= 1.0:
            self.last_visibility = now
            self.check_page()

    def wait(self, label, predicate, *, timeout=None):
        limit = min(self.deadline, time.monotonic() + timeout) if timeout is not None else self.deadline
        while time.monotonic() < limit:
            state, snap = self.sample()
            self.visible()
            if predicate(state, snap):
                self.event(label)
                return state, snap
            time.sleep(min(0.05, 1.0 / self.args.sample_hz))
        raise MatrixError(f"bounded timeout: {label}")

    def acquire_dispatch_identity(self):
        # HH_261007 - The visible browser proves an outgoing WebSocket frame,
        # not synchronous backend acceptance. Allow only a bounded read-only
        # acknowledgement wait; never retry dispatch or assume stop authority.
        limit = min(self.deadline, time.monotonic() + 5.0)
        while time.monotonic() < limit:
            state = self.ui.state()
            if state.get("mission_dispatch_active") is True:
                candidate = mission_identity(state)
                check_owner(state, candidate)
                if (candidate["site"] != self.args.site or candidate["intent"] != "delivery"
                        or candidate["owner"] != "operator"):
                    raise Intervention("visible dispatch did not acquire requested identity")
                return candidate
            check_owner(state, None)
            if not exact_clear(state):
                raise Intervention("ambiguous identity during dispatch acknowledgement")
            self.sample()
            time.sleep(0.05)
        raise MatrixError("dispatch acknowledgement timed out; authority unconfirmed, no automatic retry/stop")

    def run(self):
        # HH_261007 - No UI action until the current operator releases control.
        release_deadline = time.monotonic() + 30.0
        while time.monotonic() < release_deadline:
            try:
                state, snap = self.sample()
                ready = start_ready(state, snap, road_start=self.args.allow_road_start,
                                    drop_zone=self.drop_zone, fresh=self.observer.fresh(time.monotonic()),
                                    physical_speed=self.physical_speed)
            except Intervention:
                ready = False
            if ready:
                break
            time.sleep(0.05)
        else:
            raise Intervention("no stationary released ready start within 30s; no dispatch")
        self.event("released_stationary_start", observation=snap)
        self.browser = GuardedBrowser(self.args.cdp_url, self.args.ui_url, timeout_s=5,
                                      observation_hook=self.sample)
        self.browser.authority_guard = self.browser.input_guard = self.guard
        self.browser.dispatch_confirmation_started = self._begin_dispatch_confirmation
        self.browser.dispatch_confirmation_finished = self._finish_dispatch_confirmation
        self.ui.stop_guard = self.guard
        self.event("forward_runtime_before_dispatch", evidence=verify_forward_road_return_runtime(self.observer))
        self.guard()
        self.observer.begin_site()
        self.distance = PhysicalDistance()
        self.deadline = time.monotonic() + self.args.leg_timeout_s
        outbound_started = time.monotonic()
        try:
            response = self.browser.dispatch(self.args.site, "delivery")
            self._finish_dispatch_confirmation()
            candidate = self.acquire_dispatch_identity()
            self.identity = candidate
            self.report["identity"] = candidate
        except BaseException:
            self.report["dispatch_authority_unconfirmed"] = True
            raise
        finally:
            self.dispatch_pending = False
        self.event("visible_dispatch", identity=self.identity, response=response)
        phases = expected_site_phases(self.site, "delivery")
        state, snap = self.wait("wait_return_observed", lambda s, p:
                               s.get("service_state") == 11
                               and (p.get("site") or {}).get("operating_state") == "WAIT_RETURN"
                               and arrival_ready_for_return(p, phases))
        self.report["outbound"] = {"elapsed_wall_s": time.monotonic() - outbound_started,
                                   "sampled_xy_m": self.distance.xy_m}
        return_distance, return_started = self.distance.xy_m, time.monotonic()
        self.stage, self.deadline = "return", return_started + self.args.leg_timeout_s
        self.event("forward_runtime_before_return", evidence=verify_forward_road_return_runtime(self.observer))
        state, snap = self.sample()
        response = self.browser.request_return(return_context(state, snap, self.identity))
        self.event("visible_return", response=response)
        _, ack_snap = self.wait("fresh_return_acknowledgement", lambda _s, p:
                                return_acknowledgement(p, response) is not None, timeout=10)
        self.report["return_acknowledgement"] = return_acknowledgement(ack_snap, response)
        self.wait("current_mission_parked", lambda s, p: bool(parking_completion(p))
                  and stationary_parked(s, p, self.drop_zone))
        self.observer.drain_final_callbacks()
        state, snap = self.sample()
        if (not parking_completion(snap) or not stationary_parked(state, snap, self.drop_zone)
                or not contains_ordered_subsequence(snap.get("sequences", {}).get("site_phases", []), phases)
                or "ALIGN_PARKING_YAW" not in snap.get("sequences", {}).get("drop_zone_phases", [])):
            raise MatrixError("final current-mission parking or ordered phase proof missing")
        self.report.update(status="COMPLETED_OBSERVED", roundtrip_verified=True,
                           final_observation=snap,
                           return_leg={"elapsed_wall_s": time.monotonic() - return_started,
                                       "sampled_xy_m": self.distance.xy_m - return_distance})
        self.event("diagnostic_roundtrip_completed")

    def _begin_dispatch_confirmation(self):
        # HH_261007 - A final click may dispatch before its CDP response arrives.
        self.dispatch_pending = True
        state = self.ui.state()
        if exact_clear(state) and state.get("engaged") is False and state.get("service_state") == 16:
            self._inherited_dispatch_stop = {
                "deadline": time.monotonic() + INHERITED_STOP_MAX_SECONDS,
                "description": state.get("service_state_description"),
                "stop_sequence": self.observer.service_stop_sequence,
                "identity": None,
            }

    def _allow_inherited_dispatch_stop(self, state, snap, now):
        # HH_261007 - Voice gating commits identity before replacing cached 16.
        # This is a bounded inherited-state exception, not proof of voice state.
        pending = getattr(self, "_inherited_dispatch_stop", None)
        if pending is None:
            self._dispatch_stop_observation = None
            return False
        service = (snap.get("service_state") or {}).get("state")
        gate = snap.get("gate") or {}

        def decision(classification, allowed=False, close=True):
            # HH_261007 - Preserve the observation that distinguished cached
            # cross-channel skew from a new stop, even before physical sampling.
            record = {"classification": classification, "allowed": allowed,
                      "ui_service_state": state.get("service_state"),
                      "ros_service_state": service, "engaged": state.get("engaged"),
                      "gate": dict(gate), "stop_sequence": self.observer.service_stop_sequence,
                      "observed_monotonic_s": now,
                      "handoff_deadline": pending.get("handoff_deadline")}
            self._dispatch_stop_observation = record
            report = getattr(self, "report", None)
            if report is not None:
                records = report.setdefault("dispatch_stop_observations", [])
                if not records or records[-1]["classification"] != classification:
                    records.append(record)
            if close:
                self._inherited_dispatch_stop = None
            return allowed

        if self.observer.service_stop_sequence != pending["stop_sequence"]:
            return decision("new_ros_stop_callback")
        if now >= pending["deadline"]:
            return decision("inherited_stop_deadline_expired")
        if state.get("mission_dispatch_active") is not True:
            if pending["identity"] is not None:
                return decision("owned_dispatch_became_inactive")
            return False
        candidate = mission_identity(state)
        if (candidate["site"] != self.args.site or candidate["owner"] != "operator"
                or candidate["intent"] != "delivery"
                or (pending["identity"] is not None and candidate != pending["identity"])):
            raise Intervention("inherited stop grace cannot adopt a changed dispatch identity")
        check_owner(state, getattr(self, "identity", None) or candidate)
        owned_before = pending["identity"] == candidate or getattr(self, "identity", None) == candidate
        pending["identity"] = candidate
        ui_service = state.get("service_state")
        # HH_261007 - UI and ROS do not update atomically at voice release.
        # Only explicit departure (15 / MOVING1=1) opposite inherited cached16
        # gets <=0.5 s, never a refreshed deadline or an engaged-only exception.
        skew = ((ui_service == 16 and service in (15, 1))
                or (service == 16 and ui_service in (15, 1)))
        handoff_deadline = pending.get("handoff_deadline")
        if handoff_deadline is not None and now >= handoff_deadline:
            return decision("dispatch_handoff_expired")
        if skew:
            if (not owned_before or self.observer.fresh(now) is not True
                    or gate.get("level") != 0
                    or gate.get("operating_state") not in ("STANDBY", "ENABLED", "DEPARTING_CHARGER")
                    or (ui_service == 16
                        and state.get("service_state_description") != pending["description"])):
                return decision("dispatch_handoff_safety_rejected")
            if handoff_deadline is None:
                pending["handoff_deadline"] = min(pending["deadline"], now + DISPATCH_HANDOFF_MAX_SECONDS)
            return decision("bounded_cross_channel_departure_handoff", True, False)
        if ui_service != 16 and service != 16:
            return decision("dispatch_handoff_completed")
        if handoff_deadline is not None:
            return decision("dispatch_handoff_regressed_to_stop")
        if (state.get("engaged") is not False or ui_service != 16
                or service not in (None, 16)
                or state.get("service_state_description") != pending["description"]
                or gate.get("level") != 0 or gate.get("operating_state") != "STANDBY"):
            return decision("stop_without_safe_departure_handoff")
        return decision("inherited_voice_wait_stop_cache", True, False)

    def _finish_dispatch_confirmation(self):
        # HH_261007 - Close idle preflight once the actual dispatch click ran;
        # only the independently bounded owned inherited-stop check remains.
        self.stage = "outbound"
        self.dispatch_pending = True

    def cleanup_fault(self, error):
        if isinstance(error, Intervention) or self.identity is None or self.browser is None:
            return {"requested": False, "reason": "no confirmed retained test authority"}
        # HH_261007 - A manual takeover/replacement suppresses cleanup, even if
        # the initiating error was a physical fault or timeout.
        require_owned_active(self.ui.state(), self.identity)
        if "MANUAL" in str((self.observer.snapshot().get("gate") or {}).get("operating_state", "")):
            raise Intervention("manual gate suppresses failed-test stop")
        self.stopping = True
        return stop_owned(self.browser, self.ui, self.identity)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    # HH_261008 - Every catalog site uses its native phases and identical safety checks.
    parser.add_argument("--site", choices=tuple(f"B{index}" for index in range(1, 14)), required=True)
    parser.add_argument("--output", type=Path, required=True, help="new exclusive JSON file")
    parser.add_argument("--allow-road-start", action="store_true",
                        help="diagnostic: allow released stationary current road pose, not initial PARKED proof")
    parser.add_argument("--leg-timeout-s", type=float, default=900)
    parser.add_argument("--sample-hz", type=float, default=5)
    parser.add_argument("--ui-url", default="http://127.0.0.1:8010")
    parser.add_argument("--cdp-url", default="http://127.0.0.1:9224")
    parser.add_argument("--expected-actor-id", type=int)
    args = parser.parse_args(argv)
    if not finite(args.leg_timeout_s) or not 0 < args.leg_timeout_s <= 900:
        parser.error("--leg-timeout-s must be in (0,900]")
    if not finite(args.sample_hz) or not 0 < args.sample_hz <= 5:
        parser.error("--sample-hz must be in (0,5]")
    if args.expected_actor_id is not None and args.expected_actor_id <= 0:
        parser.error("--expected-actor-id must be positive")
    return args


def main(argv=None):
    args = parse_args(argv)
    # HH_261007 - Reserve before imports/connections/actions; never overwrite.
    with args.output.open("x", encoding="utf-8") as stream:
        evidence = Evidence(stream)
        observer = runner = None
        report = {"status": "NOT_STARTED", "roundtrip_verified": False,
                  "formal_acceptance": False, "site": args.site}
        old_handler = signal.getsignal(signal.SIGTERM)

        def interrupted(*_):
            raise KeyboardInterrupt("SIGTERM")

        signal.signal(signal.SIGTERM, interrupted)
        try:
            import carla
            ui = GuardedUI(args.ui_url, timeout_s=1)
            client = carla.Client("127.0.0.1", 2000)
            client.set_timeout(1)
            world = client.get_world()
            world.wait_for_tick(3)
            actor = select_unique_ranger(world)
            if args.expected_actor_id is not None and actor.id != args.expected_actor_id:
                raise MatrixError("existing Ranger does not match expected actor ID")
            observer = FreshObservation("ego_vehicle", expected_actor_id=actor.id)
            runner = Roundtrip(args, observer, ui, carla, actor, evidence)
            report = runner.report
            report["actor_id"] = actor.id
            report["world_map"] = world.get_map().name
            runner.run()
            return 0
        except (Exception, KeyboardInterrupt) as error:
            report.update(status="INTERVENTION" if isinstance(error, Intervention) else "FAILED",
                          error=str(error), exception=type(error).__name__)
            if runner is not None:
                try:
                    report["stop_cleanup"] = runner.cleanup_fault(error)
                except Exception as cleanup_error:
                    report["stop_cleanup"] = {"requested_or_confirmed": False, "error": str(cleanup_error)}
                try:
                    report["final_observation"] = observer.snapshot()
                except Exception:
                    pass
            return 1
        finally:
            signal.signal(signal.SIGTERM, old_handler)
            report["finished_utc"], report["sample_count"] = utc_now(), evidence.count
            if runner is not None:
                report["elapsed_wall_s"] = time.monotonic() - runner.started
                report["sampled_xy_m"] = runner.distance.xy_m
                report["sampled_sim_seconds"] = runner.distance.sim_seconds
            evidence.finish(report)
            for resource in (runner.browser if runner else None, observer):
                if resource is not None:
                    try:
                        resource.close()
                    except Exception as error:
                        print(f"observer/browser close failed: {error}", file=sys.stderr)
            print(json.dumps({k: report.get(k) for k in
                              ("status", "site", "roundtrip_verified", "sample_count", "error")}), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
