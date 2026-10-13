"""HH_261007 - Offline-only safety and full-roundtrip contract tests."""
from copy import deepcopy
from dataclasses import replace
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import probe_carla_site_roundtrip as probe
from camping_site_matrix import DropZone, expected_site_phases, load_sites


IDENTITY = {"site": "B1", "owner": "operator", "intent": "delivery", "generation": 123}


def clear():
    return {"mission_dispatch_active": False, "mission_dispatch_generation": 0,
            "mission_dispatch_site": "", "mission_dispatch_owner": "",
            "mission_dispatch_intent": "", "service_state": 0, "engaged": False}


def active():
    return {**clear(), "mission_dispatch_active": True,
            **{"mission_dispatch_" + key: value for key, value in IDENTITY.items()}}


def physical(frame=100, x=0.0, y=0.0):
    transform = {"location_m": {"x": x, "y": y, "z": 1.5},
                 "rotation_degrees": {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}}
    velocity = {"x": 0.0, "y": 0.0, "z": 0.0}
    return {"api_version": 1, "actor": {"id": 7, "type_id": "vehicle.ranger.default",
            "role_name": "ego_vehicle", "transform": deepcopy(transform),
            "linear_velocity_mps": deepcopy(velocity)},
            "telemetry": {"body_transform": transform,
                          "body_linear_velocity_mps": velocity,
                          "world_frame": frame, "world_delta_seconds": 0.05},
            "wheels": [{"wheel_index": i} for i in range(4)]}


def parked():
    return {"service_state": {"state": 0, "state_name": "DROP_ZONE_WAIT"},
            "gate": {"level": 0, "operating_state": "STANDBY"},
            "parking": {"dispatcher": {"level": 0, "operating_state": "PARKED",
                        "message": "parking_method=reverse charging_required=false"},
                        "reverse": {"level": 0, "operating_state": "PARKED"}},
            "carla_odometry": {"speed_mps": 0.0},
            "pose": {"x_m": 0.0, "y_m": 0.0, "z_m": 0.0},
            "physical": {"ready": True},
            "sequences": {"parking_phases": ["reverse:REVERSE_APPROACH", "reverse:PARKED"],
                          "service_state_ids": [10, 0], "drop_zone_phases": ["ALIGN_PARKING_YAW"]}}


class RoundtripTests(unittest.TestCase):
    def setUp(self):
        self.zone = DropZone("test", 0, 0, 0, 0)

    def test_cli_bounds_and_explicit_road_start(self):
        args = probe.parse_args(["--site", "B11", "--output", "/unused/new.json"])
        self.assertEqual((args.leg_timeout_s, args.sample_hz), (900, 5))
        self.assertFalse(args.allow_road_start)
        for extra in (["--leg-timeout-s", "901"], ["--leg-timeout-s", "nan"],
                      ["--sample-hz", "5.01"], ["--sample-hz", "0"]):
            with self.subTest(extra=extra), patch("sys.stderr", io.StringIO()), self.assertRaises(SystemExit):
                probe.parse_args(["--site", "B1", "--output", "/unused/new.json", *extra])

    def test_cli_accepts_all_thirteen_sites_with_unchanged_safety_defaults(self):
        # HH_261008 - Widen only the destination set, never bounds or start authority.
        for site in (f"B{index}" for index in range(1, 14)):
            with self.subTest(site=site):
                args = probe.parse_args(["--site", site, "--output", "/unused/new.json"])
                self.assertEqual(args.site, site)
                self.assertEqual((args.leg_timeout_s, args.sample_hz), (900, 5))
                self.assertFalse(args.allow_road_start)
        for site in ("B0", "B14", "b1", "B01"):
            with self.subTest(site=site), patch("sys.stderr", io.StringIO()), self.assertRaises(SystemExit):
                probe.parse_args(["--site", site, "--output", "/unused/new.json"])

    def test_existing_output_is_never_overwritten_before_live_import(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "prior.json"
            path.write_text("prior evidence", encoding="utf-8")
            with self.assertRaises(FileExistsError):
                probe.main(["--site", "B1", "--output", str(path)])
            self.assertEqual(path.read_text(), "prior evidence")

    def test_streamed_evidence_is_json_and_records_fault_sample(self):
        stream = io.StringIO()
        evidence = probe.Evidence(stream)
        evidence.sample({"physical": physical()})
        evidence.sample({"stage": "return"})
        evidence.finish({"status": "FAILED", "roundtrip_verified": False})
        saved = json.loads(stream.getvalue())
        self.assertEqual(len(saved["samples"]), 2)
        self.assertFalse(saved["result"]["roundtrip_verified"])

    def test_bounds_check_atomic_body_as_well_as_actor_cache(self):
        for field, value in (("roll", 18.01), ("pitch", -18.01), ("yaw", float("nan"))):
            sample = physical()
            sample["telemetry"]["body_transform"]["rotation_degrees"][field] = value
            with self.subTest(field=field), self.assertRaises(probe.MatrixError):
                probe.physical_bounds(sample, 7)
        sample = physical()
        sample["telemetry"]["body_linear_velocity_mps"]["y"] = 2.01
        with self.assertRaises(probe.MatrixError):
            probe.physical_bounds(sample, 7)
        self.assertEqual(probe.physical_bounds(physical(), 7), 0)

    def test_actor_or_wheel_identity_change_is_rejected(self):
        with self.assertRaises(probe.MatrixError):
            probe.physical_bounds(physical(), 8)
        sample = physical()
        sample["wheels"][3]["wheel_index"] = 2
        with self.assertRaises(probe.MatrixError):
            probe.physical_bounds(sample, 7)

    def test_real_distance_has_no_ten_metre_cutoff(self):
        distance = probe.PhysicalDistance()
        for index in range(101):
            distance.observe(physical(100 + index * 4, x=index * 0.2), index * 0.2)
        self.assertAlmostEqual(distance.xy_m, 20)
        self.assertAlmostEqual(distance.sim_seconds, 20)

    def test_distance_duplicate_regression_stale_and_jump(self):
        distance = probe.PhysicalDistance()
        distance.observe(physical(), 0)
        distance.observe(physical(), 1)
        self.assertEqual(distance.xy_m, 0)
        for sample, now in ((physical(), 1.6), (physical(99), 1.1),
                            (physical(104, x=100), 1.1)):
            with self.subTest(frame=sample["telemetry"]["world_frame"]), self.assertRaises(probe.MatrixError):
                distance.observe(sample, now)
        self.assertEqual(distance.xy_m, 0)

    def test_explicit_road_start_does_not_require_initial_parking(self):
        snap = parked()
        snap["pose"]["x_m"] = 50
        snap["parking"] = {}
        kwargs = dict(drop_zone=self.zone, fresh=True, physical_speed=0)
        self.assertTrue(probe.start_ready(clear(), snap, road_start=True, **kwargs))
        self.assertFalse(probe.start_ready(clear(), snap, road_start=False, **kwargs))
        self.assertFalse(probe.start_ready(clear(), snap, road_start=True,
                                          **{**kwargs, "fresh": False}))

    def test_armed_or_ambiguous_start_is_rejected(self):
        kwargs = dict(road_start=True, drop_zone=self.zone, fresh=True, physical_speed=0)
        self.assertFalse(probe.start_ready({**clear(), "engaged": True}, parked(), **kwargs))
        self.assertFalse(probe.start_ready({**clear(), "mission_dispatch_generation": 1}, parked(), **kwargs))
        with self.assertRaises(probe.Intervention):
            probe.start_ready({**clear(), "operation_mode": "MANUAL"}, parked(), **kwargs)

    def test_stop_requires_still_active_exact_identity_and_no_manual_owner(self):
        probe.require_owned_active(active(), IDENTITY)
        for state in (clear(), {**active(), "mission_dispatch_generation": 124},
                      {**active(), "control_mode": 0},
                      {**active(), "manual_resume": {"eligible": True}},
                      {**active(), "backend_startup_recovery": True}):
            with self.subTest(state=state), self.assertRaises(probe.Intervention):
                probe.require_owned_active(state, IDENTITY)

    def test_cleanup_never_stops_on_intervention_or_replacement(self):
        runner = object.__new__(probe.Roundtrip)
        runner.identity, runner.browser, runner.ui = IDENTITY, Mock(), Mock()
        runner.ui.state.return_value = {**active(), "mission_dispatch_generation": 456}
        with patch.object(probe, "stop_owned") as stop:
            self.assertFalse(runner.cleanup_fault(probe.Intervention("manual"))["requested"])
            with self.assertRaises(probe.Intervention):
                runner.cleanup_fault(probe.MatrixError("timeout"))
            stop.assert_not_called()

    def test_only_owned_fault_uses_existing_stop_contract(self):
        runner = object.__new__(probe.Roundtrip)
        runner.identity, runner.browser, runner.ui = IDENTITY, Mock(), Mock()
        runner.observer = Mock()
        runner.observer.snapshot.return_value = {}
        runner.ui.state.return_value = active()
        with patch.object(probe, "stop_owned", return_value={"requested": True}) as stop:
            self.assertTrue(runner.cleanup_fault(probe.MatrixError("fault"))["requested"])
            stop.assert_called_once_with(runner.browser, runner.ui, IDENTITY)
        self.assertTrue(runner.stopping)

    def test_cleanup_pairs_each_visible_production_stop_with_its_confirmation(self):
        # HH_261007 - The service screen is valid when navigation is closed.
        ready = {"visibleCount": 1, "disabled": False, "hit": True,
                 "settled": True, "pageReady": True}
        for action, confirmation in (
                (".dd-stop-action", ".dd-stop-confirm"),
                (".preview-stop-btn", '[data-ui="operator-stop-confirm-yes"]')):
            browser = object.__new__(probe.GuardedBrowser)
            browser._element = Mock(side_effect=lambda selector:
                                    ready if selector == action else {"visibleCount": 0})
            with self.subTest(action=action), patch.object(probe.SupervisedBrowserClient, "_click") as click:
                browser._click(".dd-stop-action", "open cleanup stop")
                browser._click(".dd-stop-confirm", "confirm cleanup stop")
                self.assertEqual([call.args[0] for call in click.call_args_list], [action, confirmation])
            self.assertIsNone(browser._cleanup_stop_confirmation_selector)

    def test_cleanup_refuses_missing_ambiguous_disabled_or_obscured_stop_surface(self):
        ready = {"visibleCount": 1, "disabled": False, "hit": True,
                 "settled": True, "pageReady": True}
        for target in ({"visibleCount": 0}, ready, {**ready, "disabled": True},
                       {**ready, "hit": False}, {**ready, "settled": False}):
            browser = object.__new__(probe.GuardedBrowser)
            browser._element = Mock(return_value=target)
            with self.subTest(target=target), patch.object(probe.SupervisedBrowserClient, "_click") as click:
                with self.assertRaises(probe.MatrixError):
                    browser._click(".dd-stop-action", "open cleanup stop")
                click.assert_not_called()

    def test_cleanup_never_confirms_without_successful_owned_open(self):
        browser = object.__new__(probe.GuardedBrowser)
        with patch.object(probe.SupervisedBrowserClient, "_click") as click:
            with self.assertRaises(probe.MatrixError):
                browser._click(".dd-stop-confirm", "confirm cleanup stop")
            click.assert_not_called()

    def test_visible_manual_arm_or_hidden_page_is_intervention(self):
        runner = object.__new__(probe.Roundtrip)
        runner.browser = Mock()
        runner.args = SimpleNamespace(ui_url="http://127.0.0.1:8010")
        normal = {"visibility": "visible", "href": "http://127.0.0.1:8010/", "manualArmed": False}
        runner.browser._evaluate.return_value = normal
        runner.check_page()
        for page in ({**normal, "manualArmed": True}, {**normal, "visibility": "hidden"}):
            runner.browser._evaluate.return_value = page
            with self.assertRaises(probe.Intervention):
                runner.check_page()

    def test_pointer_guard_still_runs_after_observation_hook_removed(self):
        browser = object.__new__(probe.GuardedBrowser)
        browser._observation_hook = browser.authority_guard = None
        browser.input_guard = Mock(side_effect=probe.Intervention("new owner"))
        with patch.object(probe.SupervisedBrowserClient, "_call") as underlying:
            with self.assertRaises(probe.Intervention):
                browser._call("Input.dispatchMouseEvent", {"type": "mouseReleased"})
            underlying.assert_not_called()

    def test_b11_b12_b13_preserve_native_roadside_stop_policy(self):
        # HH_261008 - The catalog drives phases; roadside sites never turn around.
        sites = load_sites(probe.SOURCE / "camrod_planning/config/camping_sites.yaml")
        for key in ("B11", "B12", "B13"):
            with self.subTest(site=key):
                self.assertEqual(sites[key].service_mode, "roadside_stop")
                self.assertEqual(expected_site_phases(sites[key], "delivery"), (
                    "CRAB_IN", "UNLOAD_WAIT", "WAIT_RETURN", "CRAB_OUT", "DONE",
                ))
        self.assertIn("ROTATE_180", expected_site_phases(sites["B1"], "delivery"))

    def test_each_roadside_site_rejects_changed_catalog_policy(self):
        # HH_261008 - CLI widening cannot silently override roadside source policy.
        sites = load_sites(probe.SOURCE / "camrod_planning/config/camping_sites.yaml")
        for key in ("B11", "B12", "B13"):
            changed = {**sites, key: replace(sites[key], service_mode="turnaround")}
            args = SimpleNamespace(site=key)
            with self.subTest(site=key), patch.object(probe, "load_sites", return_value=changed):
                with self.assertRaisesRegex(probe.MatrixError, key + " source policy"):
                    probe.Roundtrip(args, Mock(), Mock(), None, SimpleNamespace(id=7),
                                    probe.Evidence(io.StringIO()))

    def test_b2_loads_native_turnaround_and_all_ordered_delivery_phases(self):
        # HH_261007 - No roadside shortcut or phase override is allowed for B2.
        site = load_sites(probe.SOURCE / "camrod_planning/config/camping_sites.yaml")["B2"]
        self.assertEqual(site.key, "B2")
        self.assertEqual(site.service_mode, "turnaround")
        self.assertEqual(expected_site_phases(site, "delivery"), (
            "CRAB_IN", "ROTATE_180", "UNLOAD_WAIT", "WAIT_RETURN",
            "ALIGN_RETRACE_YAW", "CRAB_OUT", "DONE",
        ))

    def test_dispatch_identity_waits_for_backend_after_outgoing_frame(self):
        runner = object.__new__(probe.Roundtrip)
        runner.args = SimpleNamespace(site="B1")
        runner.deadline = float("inf")
        runner.identity = None
        runner.ui, runner.sample = Mock(), Mock()
        runner.ui.state.side_effect = [clear(), active()]
        with patch.object(probe.time, "sleep"):
            self.assertEqual(runner.acquire_dispatch_identity(), IDENTITY)
        runner.sample.assert_called_once()
        self.assertIsNone(runner.identity)

    def test_dispatch_acknowledgement_never_adopts_wrong_owner(self):
        runner = object.__new__(probe.Roundtrip)
        runner.args = SimpleNamespace(site="B1")
        runner.deadline = float("inf")
        runner.ui, runner.sample = Mock(), Mock()
        runner.ui.state.return_value = {**active(), "mission_dispatch_owner": "guest"}
        with self.assertRaises(probe.Intervention):
            runner.acquire_dispatch_identity()
        runner.sample.assert_not_called()

    def test_final_parked_cannot_be_satisfied_by_stale_phase(self):
        snapshot = parked()
        self.assertTrue(probe.parking_completion(snapshot))
        self.assertTrue(probe.stationary_parked(clear(), snapshot, self.zone))
        snapshot["sequences"]["parking_phases"] = ["reverse:PARKED"]
        self.assertFalse(probe.parking_completion(snapshot))

    def test_inactive_operator_stopped_only_allowed_before_dispatch(self):
        runner = object.__new__(probe.Roundtrip)
        runner.observer, runner.ui = Mock(), Mock()
        runner.identity = None
        runner.dispatch_pending = False
        runner.stage = "preflight"
        runner.deadline = float("inf")
        runner.last_sample = 100
        runner.args = SimpleNamespace(sample_hz=5)
        runner.ui.state.return_value = {**clear(), "service_state": 16}
        runner.observer.snapshot.return_value = {"service_state": {"state": 16}}
        with patch.object(probe.time, "monotonic", return_value=100):
            runner.sample()
            # HH_261007 - Preparing the final pointer action still precedes
            # dispatch; the old stop is allowed only with exact idle ownership.
            runner.dispatch_pending = True
            runner.sample()
            runner.ui.state.return_value["engaged"] = True
            with self.assertRaisesRegex(probe.MatrixError, "service failure"):
                runner.sample()
            runner.ui.state.return_value["engaged"] = False
            runner.stage = "outbound"
            with self.assertRaisesRegex(probe.MatrixError, "service failure"):
                runner.sample()
            runner.stage = "preflight"
            runner.ui.state.return_value["mission_execution_error"] = "real error"
            with self.assertRaisesRegex(probe.MatrixError, "execution error"):
                runner.sample()

    def test_final_dispatch_click_closes_preflight_only_after_real_pointer_action(self):
        # HH_261007 - Menu readiness must not look like an issued mission.
        browser = object.__new__(probe.GuardedBrowser)
        events = []
        browser.dispatch_confirmation_started = lambda: events.append("pending")
        browser.dispatch_confirmation_finished = lambda: events.append("outbound")
        with patch.object(probe.SupervisedBrowserClient, "_click",
                          side_effect=lambda *args: events.append("pointer")):
            browser._click('[data-ui="operator-site-B1"]', "select site")
            self.assertEqual(events, ["pointer"])
            events.clear()
            browser._click('[data-ui="operator-site-code-confirm"]', "dispatch")
        self.assertEqual(events, ["pending", "pointer", "outbound"])

    def test_final_dispatch_pointer_error_does_not_claim_command_completed(self):
        browser = object.__new__(probe.GuardedBrowser)
        browser.dispatch_confirmation_started = Mock()
        browser.dispatch_confirmation_finished = Mock()
        with patch.object(probe.SupervisedBrowserClient, "_click",
                          side_effect=probe.Intervention("operator takeover")):
            with self.assertRaises(probe.Intervention):
                browser._click('[data-ui="operator-site-code-confirm"]', "dispatch")
        browser.dispatch_confirmation_started.assert_called_once()
        browser.dispatch_confirmation_finished.assert_not_called()

    def inherited_stop_runner(self):
        # HH_261007 - Reproduce accepted identity before the voice-gated release.
        runner = object.__new__(probe.Roundtrip)
        runner.args = SimpleNamespace(site="B1", sample_hz=5)
        runner.observer, runner.ui = Mock(), Mock()
        runner.observer.service_stop_sequence = 3
        runner.observer.fresh.return_value = True
        runner.report = {}
        runner.ui.state.return_value = {**clear(), "service_state": 16,
                                       "service_state_description": "old stop"}
        with patch.object(probe.time, "monotonic", return_value=100):
            runner._begin_dispatch_confirmation()
        runner._finish_dispatch_confirmation()
        runner.identity = None
        runner.deadline, runner.last_sample = float("inf"), 110
        runner.ui.state.return_value = {**active(), "service_state": 16,
                                       "service_state_description": "old stop"}
        runner.observer.snapshot.return_value = {
            "service_state": None, "gate": {"level": 0, "operating_state": "STANDBY"}}
        return runner

    def test_voice_wait_inherited_stop_requires_matching_identity_and_expires(self):
        runner = self.inherited_stop_runner()
        state, snap = runner.ui.state(), runner.observer.snapshot()
        self.assertTrue(runner._allow_inherited_dispatch_stop(state, snap, 115.99))
        self.assertEqual(runner._inherited_dispatch_stop["identity"], IDENTITY)
        self.assertFalse(runner._allow_inherited_dispatch_stop(state, snap, 116))
        self.assertFalse(runner._allow_inherited_dispatch_stop(state, snap, 110))
        for key, value in (("owner", "guest"), ("site", "B2"), ("generation", 124)):
            runner = self.inherited_stop_runner()
            state = runner.ui.state()
            runner._allow_inherited_dispatch_stop(state, snap, 110)
            state["mission_dispatch_" + key] = value
            with self.subTest(key=key), self.assertRaises(probe.Intervention):
                runner._allow_inherited_dispatch_stop(state, snap, 111)

    def test_voice_wait_grace_closes_permanently_on_new_state_engage_or_fault(self):
        for change in ("engage", "ui_nonstop", "ros_other_state", "fresh_ros_stop", "new_description", "gate_fault"):
            runner = self.inherited_stop_runner()
            state, snap = runner.ui.state(), runner.observer.snapshot()
            baseline_state, baseline_snap = deepcopy(state), deepcopy(snap)
            if change == "engage":
                state["engaged"] = True
            elif change == "ui_nonstop":
                state["service_state"] = 1
            elif change == "ros_other_state":
                snap["service_state"] = {"state": 11}
            elif change == "fresh_ros_stop":
                runner.observer.service_stop_sequence += 1
            elif change == "new_description":
                state["service_state_description"] = "new stop"
            else:
                snap["gate"]["level"] = 2
            with self.subTest(change=change):
                self.assertFalse(runner._allow_inherited_dispatch_stop(state, snap, 110))
                self.assertFalse(runner._allow_inherited_dispatch_stop(baseline_state, baseline_snap, 111))

    def departure_handoff_runner(self, ui_service=16, ros_service=15):
        runner = self.inherited_stop_runner()
        state, snap = runner.ui.state(), runner.observer.snapshot()
        self.assertTrue(runner._allow_inherited_dispatch_stop(state, snap, 109))
        runner.identity = IDENTITY
        runner.observe_parking_clear = Mock()
        state.update(service_state=ui_service, engaged=True)
        if ui_service != 16:
            state["service_state_description"] = "departing"
        snap["service_state"] = {"state": ros_service}
        snap["gate"]["operating_state"] = "ENABLED"
        return runner, state, snap

    def test_departure_handoff_tolerates_both_channel_orders_and_moving_states(self):
        # HH_261007 - B1_04 failed on asynchronous release, not a fresh stop.
        for ui_state, ros_state in ((16, 15), (15, 16), (16, 1), (1, 16)):
            runner, state, snap = self.departure_handoff_runner(ui_state, ros_state)
            with self.subTest(ui=ui_state, ros=ros_state):
                with patch.object(probe.time, "monotonic", return_value=110):
                    runner.sample()
                self.assertEqual(runner._dispatch_stop_observation["classification"],
                                 "bounded_cross_channel_departure_handoff")
                self.assertTrue(runner._allow_inherited_dispatch_stop(state, snap, 110.49))
                self.assertEqual(runner._inherited_dispatch_stop["handoff_deadline"], 110.5)
                self.assertFalse(runner._allow_inherited_dispatch_stop(state, snap, 110.5))
                self.assertEqual(runner.report["dispatch_stop_observations"][-1]["classification"],
                                 "dispatch_handoff_expired")
                self.assertFalse(runner._allow_inherited_dispatch_stop(state, snap, 110.51))

    def test_departure_handoff_completion_and_stop_regression_close_permanently(self):
        for replacement, expected in ((15, "dispatch_handoff_completed"),
                                      (16, "dispatch_handoff_regressed_to_stop")):
            runner, state, snap = self.departure_handoff_runner()
            self.assertTrue(runner._allow_inherited_dispatch_stop(state, snap, 110))
            state["service_state"] = replacement
            snap["service_state"]["state"] = replacement
            self.assertFalse(runner._allow_inherited_dispatch_stop(state, snap, 110.1))
            self.assertEqual(runner._dispatch_stop_observation["classification"], expected)
            state["service_state"] = 16
            self.assertFalse(runner._allow_inherited_dispatch_stop(state, snap, 110.2))

    def test_departure_handoff_cannot_extend_original_deadline_or_adopt_new_identity(self):
        runner, state, snap = self.departure_handoff_runner()
        self.assertTrue(runner._allow_inherited_dispatch_stop(state, snap, 115.8))
        self.assertEqual(runner._inherited_dispatch_stop["handoff_deadline"], 116)
        self.assertFalse(runner._allow_inherited_dispatch_stop(state, snap, 116))
        runner, state, snap = self.departure_handoff_runner()
        state["mission_dispatch_generation"] += 1
        with self.assertRaises(probe.Intervention):
            runner._allow_inherited_dispatch_stop(state, snap, 110)
        runner, state, snap = self.departure_handoff_runner()
        runner.identity = None
        runner._inherited_dispatch_stop["identity"] = None
        self.assertFalse(runner._allow_inherited_dispatch_stop(state, snap, 110))

    def test_departure_handoff_never_masks_fresh_stop_stale_gate_or_faults(self):
        for failure in ("fresh_stop", "stale", "gate_fault", "manual_gate", "new_description",
                        "execution", "collision", "manual", "manual_resume", "inactive"):
            runner, state, snap = self.departure_handoff_runner()
            self.assertTrue(runner._allow_inherited_dispatch_stop(state, snap, 109.9))
            if failure == "fresh_stop":
                runner.observer.service_stop_sequence += 1
            elif failure == "stale":
                runner.observer.fresh.return_value = False
            elif failure == "gate_fault":
                snap["gate"]["level"] = 2
            elif failure == "manual_gate":
                snap["gate"]["operating_state"] = "MANUAL"
            elif failure == "new_description":
                state["service_state_description"] = "new stop"
            elif failure == "execution":
                state["mission_execution_error"] = "real error"
            elif failure == "collision":
                snap["collision_evidence"] = {"event_count": 1}
            elif failure == "manual":
                state["operation_mode"] = "MANUAL"
            elif failure == "manual_resume":
                state["manual_resume"] = {"eligible": True}
            else:
                state["mission_dispatch_active"] = False
            with self.subTest(failure=failure), patch.object(probe.time, "monotonic", return_value=110):
                with self.assertRaises((probe.MatrixError, probe.Intervention)):
                    runner.sample()
            if failure == "fresh_stop":
                self.assertEqual(runner.report["dispatch_stop_observations"][-1]["classification"],
                                 "new_ros_stop_callback")
                self.assertEqual(runner.report["service_stop_fault"]["ui_service_state"], 16)
                self.assertEqual(runner.report["service_stop_fault"]["ros_service_state"], 15)

    def test_voice_wait_keeps_execution_collision_manual_and_fresh_stop_guards(self):
        for failure in ("execution", "collision", "manual", "fresh_stop"):
            runner = self.inherited_stop_runner()
            state, snap = runner.ui.state(), runner.observer.snapshot()
            if failure == "execution":
                state["mission_execution_error"] = "new failure"
            elif failure == "collision":
                snap["collision_evidence"] = {"event_count": 1}
            elif failure == "manual":
                state["operation_mode"] = "MANUAL"
            else:
                runner.observer.service_stop_sequence += 1
                snap["service_state"] = {"state": 16}
            with self.subTest(failure=failure), patch.object(probe.time, "monotonic", return_value=110):
                with self.assertRaises((probe.MatrixError, probe.Intervention)):
                    runner.sample()

    def test_fresh_observation_counts_each_real_stop_callback(self):
        observer = object.__new__(probe.FreshObservation)
        observer.service_stop_sequence = 0
        with patch.object(probe.RosObservation, "_service"):
            for state in (16, 16, 1):
                observer._service(SimpleNamespace(state=state))
        self.assertEqual(observer.service_stop_sequence, 2)

    def test_full_workflow_dispatch_wait_return_click_then_current_parked(self):
        self.assert_full_workflow("B1")

    def test_b2_full_workflow_requires_arrival_return_ack_and_current_parking(self):
        # HH_261007 - Exercise B2 through the same final proof as the B1 workflow.
        self.assert_full_workflow("B2")

    def test_remaining_sites_require_arrival_return_ack_and_current_parking(self):
        # HH_261008 - Both catalog policies retain the complete owned-roundtrip proof.
        for site in (f"B{index}" for index in range(3, 14)):
            with self.subTest(site=site):
                self.assert_full_workflow(site)

    def assert_full_workflow(self, site):
        args = SimpleNamespace(site=site, allow_road_start=True, leg_timeout_s=900,
                               sample_hz=5, ui_url="http://127.0.0.1:8010", cdp_url="http://127.0.0.1:9224")
        observer, ui, browser = Mock(), Mock(), Mock()
        observer.fresh.return_value = True
        active_state = {**active(), "mission_dispatch_site": site}
        ui.state.return_value = active_state
        runner = probe.Roundtrip(args, observer, ui, None, SimpleNamespace(id=7), probe.Evidence(io.StringIO()))
        runner.drop_zone = self.zone
        runner.physical_speed = 0
        final = parked()
        final["sequences"]["site_phases"] = list(expected_site_phases(runner.site, "delivery"))
        arrival = {"service_state": {"state": 11}, "site": {"operating_state": "WAIT_RETURN"},
                   "sequences": {"site_phases": final["sequences"]["site_phases"]}}
        runner.sample = Mock(side_effect=[(clear(), parked()), ({**active_state, "service_state": 11}, arrival),
                                          (clear(), final)])
        runner.guard = Mock()
        runner.event = Mock()
        calls = []

        def wait(label, predicate, **kwargs):
            calls.append(label)
            pair = ({**active_state, "service_state": 11}, arrival) if label == "wait_return_observed" else (clear(), final)
            self.assertTrue(predicate(*pair))
            return pair

        runner.wait = wait
        with patch.object(probe, "GuardedBrowser", return_value=browser), \
                patch.object(probe, "verify_forward_road_return_runtime", return_value={}), \
                patch.object(probe, "return_acknowledgement", return_value={"token": "fresh"}):
            runner.run()
        browser.dispatch.assert_called_once_with(site, "delivery")
        browser.request_return.assert_called_once()
        observer.begin_site.assert_called_once()
        observer.drain_final_callbacks.assert_called_once()
        self.assertEqual(calls, ["wait_return_observed", "fresh_return_acknowledgement", "current_mission_parked"])
        self.assertTrue(runner.report["roundtrip_verified"])
        self.assertFalse(runner.report["formal_acceptance"])
        self.assertEqual(runner.report["identity"]["site"], site)


if __name__ == "__main__":
    unittest.main()
