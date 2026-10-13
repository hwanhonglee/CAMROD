#!/usr/bin/env python3
"""Test-only visible-UI simulator repetitions, not a strict v27 qualification.

Requires an independently started simulation mileage recorder and sourced ROS.
Never changes controller parameters, clears safety stops, or writes a database.
Initial dispatch requires current observed stationary parking. Every subsequent
dispatch additionally requires this process's complete return/parking history.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shutil
import signal
import sys
import time
from urllib.request import urlopen

SOURCE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOURCE / "scripts/virtual_carla"))
from camping_site_matrix import (
    MatrixError, OperatorBrowserClient, UIClient, RosObservation,
    arrival_ready_for_return, capture_ui_handoff_view, contains_ordered_subsequence,
    expected_mission_owner, expected_site_phases, load_drop_zone, load_sites,
    mission_identity, parking_completion, parse_site_selection, pose_error_m,
    recall_requires_final_confirmation, return_acknowledgement, return_context,
    write_json_atomic,
)


class Intervention(MatrixError):
    """A user/replacement mission owns control; never stop or resume it."""


# HH_261007 - This test-only CARLA runner requires forward road returns; these
# read-only checks do not alter local reverse parking, recovery or manual/CRAB.
FORWARD_ROAD_RETURN_PARAMETERS = {
    "/control/camping_site_maneuver_controller": {
        "roadside_reverse_return_enable": False,
    },
    "/planning/planning_state_machine": {
        "reverse_auto_goal_snapper_input_topic": "",
    },
    "/planning/goal_snapper": {
        "reverse_auxiliary_input_goal_topic": "",
    },
    "/planning/nav2_selector_latch": {
        "controller_id": "RPP", "reverse_controller_id": "RPP",
    },
    "/planning/controller_server": {
        "RPP.allow_reversing": False,
    },
}


def verify_forward_road_return_runtime(observer, *, timeout_s=5.0,
                                       get_parameters_type=None):
    """Fresh, bounded GetParameters readback; never set a runtime parameter."""
    if not math.isfinite(timeout_s) or not 0 < timeout_s <= 5.0:
        raise MatrixError("forward road-return preflight timeout must be in (0,5]")
    if get_parameters_type is None:
        from rcl_interfaces.srv import GetParameters
        get_parameters_type = GetParameters
    started = time.monotonic()
    deadline = started + timeout_s
    clients, futures = {}, {}

    def wait_for(predicate):
        while not predicate():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise MatrixError("forward road-return runtime preflight timed out")
            observer.spin_once(min(0.05, remaining))
        if time.monotonic() > deadline:
            raise MatrixError("forward road-return runtime preflight timed out")

    try:
        for node in FORWARD_ROAD_RETURN_PARAMETERS:
            clients[node] = observer.node.create_client(
                get_parameters_type, node + "/get_parameters")
        wait_for(lambda: all(client.service_is_ready() for client in clients.values()))
        # HH_261007 - Duplicate node names can make parameter RPCs ambiguous.
        graph = [namespace.rstrip("/") + "/" + name
                 for name, namespace in observer.node.get_node_names_and_namespaces()]
        for node, expected in FORWARD_ROAD_RETURN_PARAMETERS.items():
            if graph.count(node) != 1:
                raise MatrixError(f"forward road-return node is missing or ambiguous: {node}")
            request = get_parameters_type.Request()
            request.names = list(expected)
            futures[node] = clients[node].call_async(request)
        wait_for(lambda: all(future.done() for future in futures.values()))
        selected = {}
        for node, expected in FORWARD_ROAD_RETURN_PARAMETERS.items():
            response = futures[node].result()
            values = getattr(response, "values", None)
            if values is None or len(values) != len(expected):
                raise MatrixError(f"forward road-return parameter response missing: {node}")
            selected[node] = {}
            for (name, wanted), value in zip(expected.items(), values):
                # rcl_interfaces/ParameterType: BOOL=1 and STRING=4. Check the
                # wire type as well as Python type; undeclared is not false/empty.
                wire_type = 1 if type(wanted) is bool else 4
                actual = getattr(value, "bool_value" if wire_type == 1 else "string_value", None)
                if (getattr(value, "type", None) != wire_type
                        or type(actual) is not type(wanted) or actual != wanted):
                    raise MatrixError(
                        f"forward road-return runtime mismatch: {node}.{name}="
                        f"{actual!r} expected {wanted!r}")
                selected[node][name] = actual
        return {"source": "fresh_ros_get_parameters",
                "selected_live_parameters": selected,
                "elapsed_wall_s": time.monotonic() - started}
    finally:
        for future in futures.values():
            if not future.done():
                future.cancel()
        for client in clients.values():
            observer.node.destroy_client(client)


def check_owner(state, identity, allow_inactive=False, *, stopping=False):
    if state.get("backend_startup_recovery"):
        raise Intervention("backend startup recovery intervened")
    resume = state.get("manual_resume") or {}
    if resume.get("pending") or resume.get("available") or resume.get("eligible") or resume.get("mission_id"):
        raise Intervention("manual resume context appeared")
    if state.get("mission_execution_error") and not stopping:
        raise MatrixError("mission execution error reported")
    if state.get("control_mode") == 0 or state.get("operation_mode") == "MANUAL":
        raise Intervention("manual control observed")
    if state.get("mission_dispatch_active") is True:
        if identity is None or mission_identity(state) != identity:
            raise Intervention("mission owner, site, intent or generation changed")
    elif identity is not None and not allow_inactive:
        raise Intervention("mission became inactive before verified parking")


def stationary_parked(state, snapshot, drop_zone, *, allow_late_join_ui_state=False):
    """Current physical parking proof; no fabricated sequence for cold start."""
    if state.get("mission_dispatch_active") is not False:
        return False
    service = snapshot.get("service_state") or {}
    service_state = service.get("state")
    # HH_261006 - The volatile service topic can be absent on a cold join.
    # Only explicit initial-boundary callers may use the current HTTP terminal
    # state; a present ROS value (even malformed/conflicting) is never replaced.
    if (allow_late_join_ui_state and snapshot.get("service_state") is None
            and type(state.get("service_state")) is int
            and state["service_state"] in (0, 13)):
        service_state = state["service_state"]
    gate = snapshot.get("gate") or {}
    parking = snapshot.get("parking") or {}
    dispatcher = parking.get("dispatcher") or {}
    speed = (snapshot.get("carla_odometry") or {}).get("speed_mps", math.inf)
    error = pose_error_m(snapshot.get("pose") or {}, drop_zone)
    valid_terminal = ((service_state == 0 and gate.get("operating_state") == "STANDBY")
                      or (service_state == 13 and gate.get("operating_state") == "CHARGING"))
    controller_parked = any((parking.get(name) or {}).get("operating_state") == "PARKED"
                            and (parking.get(name) or {}).get("level") == 0
                            for name in ("reverse", "apriltag"))
    return bool(valid_terminal and state.get("service_state") == service_state
                and gate.get("level") == 0 and dispatcher.get("level") == 0
                and dispatcher.get("operating_state") == "PARKED" and controller_parked
                and math.isfinite(speed) and 0 <= speed <= 0.05
                and error is not None and error <= 3.0
                and (snapshot.get("physical") or {}).get("ready") is True)


def read_mileage(path, previous=None, now=None):
    now = time.time() if now is None else now
    if not path.is_file() or not 0 <= now - path.stat().st_mtime <= 5.0:
        raise MatrixError("independent mileage summary missing or stale")
    summary = json.loads(path.read_text())
    if (summary.get("environment") != "simulation" or summary.get("status") != "recording"
            or summary.get("measurement_source") != "/carla/ego_vehicle/odometry"
            or summary.get("distance_basis") != "raw_sensor_planar_position_displacements"):
        raise MatrixError("mileage source is not the live independent simulation odometry recorder")
    distance = summary.get("xy_distance_m")
    if isinstance(distance, bool) or not isinstance(distance, (int, float)) or not math.isfinite(distance) or distance < 0:
        raise MatrixError("invalid measured mileage")
    if previous and (summary.get("started_utc") != previous.get("started_utc")
                     or distance < previous["xy_distance_m"]):
        raise MatrixError("mileage recorder restarted or distance decreased")
    return summary


def stop_owned(client, ui, identity):
    """Same generation check before both production stop confirmations."""
    current = ui.state()
    check_owner(current, identity, stopping=True)
    client._observation_hook = None  # Failed sensor observation must not block UI stop.
    client.authority_guard = None
    transport = "visible_UI_confirmation"
    # HH_261007 - The service error screen can omit navigation's stop control.
    # Only an explicitly failed, still-owned mission may use the existing STOP
    # endpoint for cleanup. This is not evidence of a successful UI scenario.
    missing_stop = bool(current.get("mission_execution_error")) and client._evaluate(
        "document.querySelectorAll('.dd-stop-action').length === 0") is True
    if missing_stop:
        current = ui.state()
        check_owner(current, identity, stopping=True)
        if not current.get("mission_execution_error"):
            raise MatrixError("failed mission cleared before stop cleanup")
        ui.stop()
        transport = "production_stop_endpoint_error_cleanup"
    else:
        client._click(".dd-stop-action", "open failed-test stop confirmation")
        check_owner(ui.state(), identity, stopping=True)
        client._click(".dd-stop-confirm", "confirm failed-test mission stop")
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        current = ui.state()
        if current.get("mission_dispatch_active") is False:
            return {"requested": True, "mission_inactive_observed": True,
                    "identity": identity, "scenario_pass": False,
                    "transport": transport}
        check_owner(current, identity, stopping=True)
        time.sleep(0.1)
    raise MatrixError("stop requested but owned mission remains active")


class SupervisedBrowserClient(OperatorBrowserClient):
    """Revalidate ownership before every real pointer action, not just each phase."""
    authority_guard = None

    def _click(self, *args, **kwargs):
        if self.authority_guard is not None:
            self.authority_guard()
        return super()._click(*args, **kwargs)


class Runner:
    def __init__(self, args, observer, client, ui):
        self.args, self.observer, self.client, self.ui = args, observer, client, ui
        self.identity = None
        self.mileage = None
        self.last_frame = -math.inf
        self.last_signature = None
        self.last_records = -math.inf
        self.frame_count = 0
        self.previous_generation = None
        self.quota_warning = False
        self._parking_active_identity = None
        self._parking_clear_since = None
        self._parking_completion_observed = False
        self.report = {"status": "RUNNING", "qualification": "supervised_test_only_not_v27",
                       "cycles": [], "events": [], "target_m": args.target_m,
                       "max_cycles": args.max_cycles}
        self.drop_zone = load_drop_zone(SOURCE / "camrod_bringup/config/map/drop_zones.yaml")
        self.sites = load_sites(SOURCE / "camrod_planning/config/camping_sites.yaml")
        client.authority_guard = lambda: check_owner(ui.state(), self.identity)

    def event(self, name, **details):
        record = {"utc": datetime.now(timezone.utc).isoformat(), "event": name, **details}
        self.report["events"].append(record)
        self.report["mileage"] = self.mileage
        write_json_atomic(self.args.output / "summary.json", self.report)
        with (self.args.output / "events.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        print(json.dumps(record, ensure_ascii=False), flush=True)

    def observe_parking_clear(self, state, snapshot, allow_inactive, now):
        # HH_261006 - HTTP clears its identity before the independently queued
        # ROS PARKED callback can arrive. Wait only for observation, never for
        # new motion, and never manufacture parking or ordered phase history.
        if self.identity is None:
            self._parking_active_identity = None
            self._parking_clear_since = None
            self._parking_completion_observed = False
            return False
        if state.get("mission_dispatch_active") is True:
            if self._parking_clear_since is not None:
                raise Intervention("mission reactivated during parking observation")
            self._parking_active_identity = dict(self.identity)
            return False
        if (state.get("mission_dispatch_active") is not False
                or type(state.get("mission_dispatch_generation")) is not int
                or state["mission_dispatch_generation"] != 0
                or state.get("mission_dispatch_site") != ""
                or state.get("mission_dispatch_owner") != ""
                or state.get("mission_dispatch_intent") != ""):
            raise Intervention("inactive mission identity changed or is ambiguous")
        complete = bool(parking_completion(snapshot))
        if (self._parking_clear_since is not None
                and not self._parking_completion_observed
                and now - self._parking_clear_since > 2.0):
            raise Intervention("parking observation grace expired without timely ROS completion")
        if complete:
            self._parking_completion_observed = True
            return False
        if (not allow_inactive or self._parking_active_identity != self.identity
                or self._parking_completion_observed
                or type(state.get("service_state")) is not int
                or state["service_state"] not in (0, 13)):
            raise Intervention("mission cleared without current observed parking completion")
        if self._parking_clear_since is None:
            self._parking_clear_since = now
            self.event("awaiting_ros_parking_after_owned_http_clear",
                       identity=self.identity, observation_grace_s=2.0,
                       parking_completion_observed=False)
        return True

    def sample(self, allow_inactive=False):
        self.observer.pump_ui_callbacks()
        self.observer.spin_once(0.05)
        state, snapshot = self.ui.state(), self.observer.snapshot()
        check_owner(state, self.identity, allow_inactive)
        if (state.get("service_state") == 16
                or (snapshot.get("service_state") or {}).get("state") == 16):
            raise MatrixError("service failure state observed")
        if "MANUAL" in str((snapshot.get("gate") or {}).get("operating_state", "")):
            raise Intervention("manual control gate observed")
        if (snapshot.get("collision_evidence") or {}).get("event_count", 0):
            raise MatrixError("collision evidence observed")
        now = time.monotonic()
        parking_pending = self.observe_parking_clear(state, snapshot, allow_inactive, now)
        if shutil.disk_usage(self.args.output).free < 15 * 1024**3:
            raise MatrixError("disk floor breached: less than 15 GiB free")
        self.mileage = read_mileage(self.args.mileage_summary, self.mileage)
        if now - self.last_records >= 10:
            with urlopen("http://127.0.0.1:8010/api/mission-records?limit=1", timeout=3) as response:
                records = json.load(response)
            recorder = records.get("recorder") or {}
            quota, used = recorder.get("quota_bytes"), recorder.get("telemetry_bytes")
            if (isinstance(quota, (int, float)) and quota > 0
                    and isinstance(used, (int, float)) and used >= quota):
                raise MatrixError("mission recorder quota reached; no configuration changed")
            if "quota" in str(recorder.get("error", "")).lower():
                raise MatrixError("mission recorder reports quota/degraded capture; no configuration changed")
            if (not self.quota_warning and isinstance(quota, (int, float)) and quota > 0
                    and isinstance(used, (int, float)) and used >= 0.9 * quota):
                self.quota_warning = True
                self.event("recorder_quota_warning_no_config_changed", telemetry_bytes=used, quota_bytes=quota)
            self.last_records = now
        signature = (state.get("mission_dispatch_generation"), state.get("service_state"),
                     (snapshot.get("site") or {}).get("operating_state"),
                     (snapshot.get("drop_zone") or {}).get("operating_state"),
                     (snapshot.get("parking", {}).get("dispatcher") or {}).get("operating_state"))
        # HH_261006 - Keep pumping fresh ROS while the short grace is pending;
        # optional browser screenshots must not consume the observation budget.
        if not parking_pending and signature != self.last_signature and now - self.last_frame >= 10:
            if self.frame_count < 1000:
                capture_ui_handoff_view(self.client, self.args.output / "frames", f"{self.frame_count:04d}")
                self.frame_count += 1
            self.last_frame, self.last_signature = now, signature
            self.event("state_observed", state_signature=signature, identity=self.identity)
        return state, snapshot

    def wait(self, label, predicate, *, timeout=None, allow_inactive=False):
        deadline = time.monotonic() + (self.args.phase_timeout_s if timeout is None else timeout)
        while time.monotonic() < deadline:
            state, snapshot = self.sample(allow_inactive)
            if predicate(state, snapshot):
                self.event(label)
                return state, snapshot
            time.sleep(0.1)
        raise MatrixError(f"phase timeout / stalled: {label}")

    def confirm(self, final=False):
        # HH_261007 - Recheck the live routing contract before every Return click.
        self.preflight_forward_road_return()
        state, snapshot = self.sample()
        context = return_context(state, snapshot, self.identity, final_return=final)
        response = self.client.request_return(context)
        self.event("visible_return_clicked", final_return=final, response=response)
        self.wait("fresh_return_acknowledgement", lambda _s, snap: return_acknowledgement(snap, response) is not None, timeout=10)

    def preflight_forward_road_return(self):
        evidence = verify_forward_road_return_runtime(self.observer)
        self.event("forward_road_return_runtime_verified", **evidence)

    def wait_stationary_boundary(self):
        # HH_261006 - This exemption is only for the first unowned cold join.
        # Completed/adopted missions still require actual ROS terminal evidence.
        cold_join = (self.identity is None and self.previous_generation is None
                     and not self.report["cycles"]
                     and self.args.adopt_generation is None)
        state, snapshot = self.wait(
            "stationary_inactive_drop_zone",
            lambda state, snap: stationary_parked(
                state, snap, self.drop_zone, allow_late_join_ui_state=cold_join),
            timeout=30)
        if cold_join and snapshot.get("service_state") is None:
            self.event(
                "initial_parking_service_state_from_current_http_ui",
                service_state_source="current_http_ui_state",
                service_state=state["service_state"], initial_boundary_only=True,
                ros_service_state_observed=False, parking_history_inferred=False)
        return state, snapshot

    def run(self):
        selected = parse_site_selection(self.args.sites)
        index = 0
        if self.args.adopt_generation is not None:
            state = self.ui.state()
            candidate = mission_identity(state)
            if (candidate["generation"] != self.args.adopt_generation or candidate["site"] != selected[0]
                    or candidate["intent"] != "delivery" or candidate["owner"] != "operator"):
                raise Intervention("explicit adoption does not match the existing first delivery")
            self.preflight_forward_road_return()
            self.identity = candidate
            self.event("adopted_existing_delivery", identity=candidate, history_before_adoption_unobserved=True)
        while True:
            site_key, intent = selected[(index // 2) % len(selected)], ("delivery" if index % 2 == 0 else "recall")
            site = self.sites[site_key]
            if self.identity is None:
                self.wait_stationary_boundary()
                if self.mileage["xy_distance_m"] >= self.args.target_m:
                    self.report["status"] = "TARGET_REACHED_PARKED"
                    self.event("target_reached_at_verified_parked_boundary")
                    return
                self.preflight_forward_road_return()
                self.observer.begin_site()
                # Recheck UI identity immediately before dispatch; never replace an active mission.
                check_owner(self.ui.state(), None)
                try:
                    response = self.client.dispatch(site_key, intent)
                except Exception:
                    # No generation is assumed owned when a transport failure
                    # makes the dispatch result ambiguous. Do not cancel a human
                    # mission merely because its destination happens to match.
                    self.report["dispatch_authority_unconfirmed"] = {
                        "site": site_key, "intent": intent,
                        "operator_check_required": True,
                        "note": "Final UI click may have started motion; automatic redispatch/stop prohibited without proven identity."}
                    raise
                candidate = mission_identity(self.ui.state())
                if (candidate["site"] != site_key or candidate["intent"] != intent
                        or candidate["owner"] != expected_mission_owner("operator_browser", intent)
                        or candidate["generation"] == self.previous_generation):
                    raise Intervention("visible dispatch did not acquire matching identity")
                self.identity = candidate
                self.event("visible_dispatch", identity=candidate, response=response)
            phases = expected_site_phases(site, intent)
            self.wait("authoritative_arrival", lambda _state, snap: arrival_ready_for_return(snap, phases, expected_service_state=8 if intent == "recall" else 11))
            self.confirm()
            if recall_requires_final_confirmation(site, intent):
                self.wait("explicit_final_loading_ready", lambda state, snap: state.get("recall_final_return_ready") is True
                          and (snap.get("site") or {}).get("operating_state") == "RECALL_RETURN_WAIT"
                          and contains_ordered_subsequence(snap.get("sequences", {}).get("site_phases", []), phases[:8]))
                self.confirm(final=True)
            state, snapshot = self.wait("full_return_and_parking", lambda state, snap: bool(parking_completion(snap))
                                       and stationary_parked(state, snap, self.drop_zone), allow_inactive=True)
            self.observer.drain_final_callbacks()
            state, snapshot = self.sample(allow_inactive=True)
            if not parking_completion(snapshot) or not stationary_parked(state, snapshot, self.drop_zone):
                raise MatrixError("parking proof changed during final callback drain")
            if not contains_ordered_subsequence(snapshot.get("sequences", {}).get("site_phases", []), phases):
                raise MatrixError("complete site phase history not observed; adoption cannot invent earlier phases")
            if "ALIGN_PARKING_YAW" not in snapshot.get("sequences", {}).get("drop_zone_phases", []):
                raise MatrixError("return parking alignment was not observed")
            self.report["cycles"].append({"identity": self.identity, "status": "COMPLETED_OBSERVED", "observation": snapshot})
            self.event("cycle_completed", identity=self.identity)
            self.previous_generation = self.identity["generation"]
            self.identity = None
            index += 1
            # HH_261007 - A qualification limit only ends after the unchanged
            # fresh physical parking and complete mission-history checks above.
            if (self.args.max_cycles is not None
                    and len(self.report["cycles"]) >= self.args.max_cycles):
                self.report["status"] = "CYCLE_LIMIT_REACHED_PARKED"
                self.event("cycle_limit_reached_at_verified_parked_boundary",
                           completed_cycles=len(self.report["cycles"]))
                return


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--mileage-summary", required=True, type=Path)
    parser.add_argument("--target-m", default=10000.0, type=float)
    parser.add_argument("--max-cycles", type=int,
                        help="optional positive limit; stop only after observed return and parking")
    parser.add_argument("--sites", default="B13")
    parser.add_argument("--adopt-generation", type=int)
    parser.add_argument("--phase-timeout-s", default=900.0, type=float)
    args = parser.parse_args(argv)
    if not math.isfinite(args.target_m) or args.target_m <= 0 or not 0 < args.phase_timeout_s <= 3600:
        parser.error("positive finite target and phase timeout in (0,3600] required")
    if args.max_cycles is not None and args.max_cycles <= 0:
        parser.error("--max-cycles must be a positive integer")
    parse_site_selection(args.sites)
    args.output.mkdir(parents=True, exist_ok=False)
    observer = client = runner = None
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    try:
        observer = RosObservation("ego_vehicle")
        client = SupervisedBrowserClient("http://127.0.0.1:9224", "http://127.0.0.1:8010", observation_hook=observer.pump_ui_callbacks)
        runner = Runner(args, observer, client, UIClient("http://127.0.0.1:8010"))
        runner.run()
        return 0
    except (Exception, KeyboardInterrupt) as error:
        if runner is not None:
            runner.report["status"] = "INTERRUPTED" if isinstance(error, KeyboardInterrupt) else "FAILED"
            try:
                runner.event("halted_no_further_dispatch", reason=str(error), exception=type(error).__name__)
            except Exception as evidence_error:
                print(f"Failure evidence write failed: {evidence_error}; original: {error}", file=sys.stderr, flush=True)
            if runner.identity is not None and not isinstance(error, Intervention):
                try:
                    result = stop_owned(client, runner.ui, runner.identity)
                    runner.event("identity_bound_stop", result=result)
                except Exception as stop_error:
                    print(f"Owned-stop or its evidence not confirmed: {stop_error}", file=sys.stderr, flush=True)
                    try:
                        runner.event("stop_not_confirmed", reason=str(stop_error))
                    except Exception:
                        pass
        else:
            write_json_atomic(args.output / "summary.json", {"status": "FAILED", "reason": str(error)})
        return 1
    finally:
        if client is not None:
            client.close()
        if observer is not None:
            observer.close()


if __name__ == "__main__":
    raise SystemExit(main())
