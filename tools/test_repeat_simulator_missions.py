"""Pure safety tests; no ROS/UI process, request, or command is started."""
import json
from copy import deepcopy
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from repeat_simulator_missions import (
    FORWARD_ROAD_RETURN_PARAMETERS, Intervention, MatrixError, Runner,
    check_owner, main, read_mileage, stationary_parked, stop_owned,
    verify_forward_road_return_runtime,
)
from camping_site_matrix import DropZone, parking_completion


IDENTITY = {"site": "B13", "intent": "delivery", "owner": "operator", "generation": 123}


def active_state():
    return {"mission_dispatch_active": True,
            **{f"mission_dispatch_{key}": value for key, value in IDENTITY.items()}}


def cleared_state():
    return {"mission_dispatch_active": False, "mission_dispatch_generation": 0,
            "mission_dispatch_site": "", "mission_dispatch_owner": "",
            "mission_dispatch_intent": "", "service_state": 0}


def observation_runner():
    runner = object.__new__(Runner)
    runner.identity = dict(IDENTITY)
    runner._parking_active_identity = None
    runner._parking_clear_since = None
    runner._parking_completion_observed = False
    runner.observer, runner.ui, runner.client, runner.event = Mock(), Mock(), Mock(), Mock()
    runner.args = SimpleNamespace(output=Path("/unused"), mileage_summary=Path("/unused/summary"))
    runner.mileage = None
    runner.last_records = runner.last_frame = 100.0
    runner.last_signature = None
    runner.frame_count = 0
    return runner


def sample_at(runner, state, snapshot, now, allow_inactive=True):
    runner.ui.state.return_value = state
    runner.observer.snapshot.return_value = snapshot
    with patch("repeat_simulator_missions.time.monotonic", return_value=now), \
            patch("repeat_simulator_missions.shutil.disk_usage",
                  return_value=SimpleNamespace(free=20 * 1024**3)), \
            patch("repeat_simulator_missions.read_mileage", return_value={"xy_distance_m": 1.0}):
        return runner.sample(allow_inactive=allow_inactive)


def parked_snapshot():
    return {"service_state": {"state": 0, "state_name": "DROP_ZONE_WAIT"},
            "gate": {"level": 0, "operating_state": "STANDBY"},
            "parking": {
                "dispatcher": {"level": 0, "operating_state": "PARKED",
                               "message": "parking_method=reverse charging_required=false"},
                "reverse": {"level": 0, "operating_state": "PARKED"}},
            "carla_odometry": {"speed_mps": 0.0}, "pose": {"x_m": 0, "y_m": 0},
            "physical": {"ready": True},
            "sequences": {"parking_phases": ["reverse:REVERSE_APPROACH", "reverse:PARKED"],
                          "service_state_ids": [10, 0]}}


def parameter_observer():
    # HH_261007 - Fake parameter services only; tests never import or start ROS.
    observer = Mock()
    service_type = SimpleNamespace(Request=SimpleNamespace)
    clients = {}
    for node, expected in FORWARD_ROAD_RETURN_PARAMETERS.items():
        values = [SimpleNamespace(type=1 if type(value) is bool else 4,
                                  bool_value=value, string_value=value)
                  for value in expected.values()]
        client = Mock()
        client.service_is_ready.return_value = True
        client.call_async.return_value.done.return_value = True
        client.call_async.return_value.result.return_value = SimpleNamespace(values=values)
        clients[node] = client
    observer.node.create_client.side_effect = lambda _kind, path: clients[path.removesuffix("/get_parameters")]
    observer.node.get_node_names_and_namespaces.return_value = [
        (node.rsplit("/", 1)[1], node.rsplit("/", 1)[0]) for node in clients]
    return observer, service_type, clients


class ForwardRoadReturnPreflightTests(unittest.TestCase):
    def test_live_typed_contract_is_read_only_and_every_check_is_fresh(self):
        observer, service, clients = parameter_observer()
        for _ in range(2):
            evidence = verify_forward_road_return_runtime(observer, get_parameters_type=service)
            self.assertEqual(evidence["source"], "fresh_ros_get_parameters")
            self.assertEqual(evidence["selected_live_parameters"], FORWARD_ROAD_RETURN_PARAMETERS)
        for node, client in clients.items():
            self.assertEqual(client.call_async.call_count, 2)
            self.assertEqual(client.call_async.call_args.args[0].names,
                             list(FORWARD_ROAD_RETURN_PARAMETERS[node]))
        self.assertEqual(observer.node.destroy_client.call_count, 2 * len(clients))
        observer.node.create_publisher.assert_not_called()
        self.assertTrue(all(call.args[1].endswith("/get_parameters")
                            for call in observer.node.create_client.call_args_list))

    def test_every_reverse_routing_or_wrong_type_value_fails_closed(self):
        for node, expected in FORWARD_ROAD_RETURN_PARAMETERS.items():
            for index, (name, wanted) in enumerate(expected.items()):
                for fault in ("value", "wire_type", "python_type"):
                    with self.subTest(node=node, name=name, fault=fault):
                        observer, service, clients = parameter_observer()
                        value = clients[node].call_async.return_value.result.return_value.values[index]
                        field = "bool_value" if type(wanted) is bool else "string_value"
                        if fault == "wire_type":
                            value.type = 0  # Undeclared defaults must never look valid.
                        elif fault == "python_type":
                            setattr(value, field, 0 if type(wanted) is bool else None)
                        else:
                            setattr(value, field, True if type(wanted) is bool else "RPPReverse")
                        with self.assertRaisesRegex(MatrixError, "runtime mismatch"):
                            verify_forward_road_return_runtime(observer, get_parameters_type=service)
                        self.assertEqual(observer.node.destroy_client.call_count, len(clients))

    def test_missing_or_duplicate_runtime_node_and_missing_response_fail_closed(self):
        for fault in ("missing_node", "duplicate_node", "missing_response", "short_response"):
            with self.subTest(fault=fault):
                observer, service, clients = parameter_observer()
                graph = observer.node.get_node_names_and_namespaces.return_value
                first = next(iter(clients.values())).call_async.return_value
                if fault == "missing_node":
                    graph.pop()
                elif fault == "duplicate_node":
                    graph.append(graph[0])
                elif fault == "missing_response":
                    first.result.return_value = None
                else:
                    first.result.return_value.values = []
                with self.assertRaises(MatrixError):
                    verify_forward_road_return_runtime(observer, get_parameters_type=service)
                self.assertEqual(observer.node.destroy_client.call_count, len(clients))

    def test_service_or_response_timeout_pumps_callbacks_and_destroys_clients(self):
        for fault in ("service", "response"):
            with self.subTest(fault=fault):
                observer, service, clients = parameter_observer()
                first = next(iter(clients.values()))
                if fault == "service":
                    first.service_is_ready.return_value = False
                else:
                    first.call_async.return_value.done.return_value = False
                clock = [0.0]
                observer.spin_once.side_effect = lambda _timeout: clock.__setitem__(0, clock[0] + 0.1)
                with patch("repeat_simulator_missions.time.monotonic", side_effect=lambda: clock[0]):
                    with self.assertRaisesRegex(MatrixError, "timed out"):
                        verify_forward_road_return_runtime(observer, timeout_s=0.2,
                                                           get_parameters_type=service)
                observer.spin_once.assert_called()
                self.assertEqual(observer.node.destroy_client.call_count, len(clients))
                if fault == "response":
                    first.call_async.return_value.cancel.assert_called_once()

    def test_runtime_rpc_error_and_observer_failure_propagate_without_motion(self):
        for observer_failure in (False, True):
            observer, service, clients = parameter_observer()
            first = next(iter(clients.values()))
            if observer_failure:
                first.service_is_ready.return_value = False
                observer.spin_once.side_effect = MatrixError("physical readiness lost")
            else:
                first.call_async.return_value.result.side_effect = MatrixError("RPC failed")
            with self.assertRaises(MatrixError):
                verify_forward_road_return_runtime(observer, get_parameters_type=service)
            self.assertEqual(observer.node.destroy_client.call_count, len(clients))

    def test_invalid_timeout_is_rejected_before_ros_access(self):
        for timeout in (0, -1, 5.1, float("nan"), float("inf")):
            observer = Mock()
            with self.assertRaises(MatrixError):
                verify_forward_road_return_runtime(observer, timeout_s=timeout)
            observer.node.create_client.assert_not_called()

    def test_mismatch_prevents_dispatch_and_does_not_acquire_adopted_authority(self):
        for adopted in (None, IDENTITY["generation"]):
            runner = observation_runner()
            runner.identity = None
            runner.args = SimpleNamespace(sites="B13", adopt_generation=adopted, target_m=100)
            runner.sites = {"B13": object()}
            runner.mileage = {"xy_distance_m": 1.0}
            runner.wait_stationary_boundary = Mock()
            runner.ui.state.return_value = active_state()
            with patch("repeat_simulator_missions.verify_forward_road_return_runtime",
                       side_effect=MatrixError("reverse road route forbidden")):
                with self.assertRaises(MatrixError):
                    runner.run()
            self.assertIsNone(runner.identity)
            runner.client.dispatch.assert_not_called()
            runner.observer.begin_site.assert_not_called()

    def test_every_return_confirmation_rechecks_before_fresh_sample_or_click(self):
        runner = observation_runner()
        runner.sample = Mock(return_value=(active_state(), parked_snapshot()))
        runner.wait = Mock()
        with patch("repeat_simulator_missions.verify_forward_road_return_runtime",
                   return_value={"source": "fresh_ros_get_parameters"}) as verify, \
                patch("repeat_simulator_missions.return_context", return_value={}):
            runner.confirm()
            runner.confirm(final=True)
        self.assertEqual(verify.call_count, 2)
        self.assertEqual(runner.client.request_return.call_count, 2)
        runner.sample.reset_mock()
        runner.client.request_return.reset_mock()
        with patch("repeat_simulator_missions.verify_forward_road_return_runtime",
                   side_effect=MatrixError("runtime drift")):
            with self.assertRaises(MatrixError):
                runner.confirm()
        runner.sample.assert_not_called()
        runner.client.request_return.assert_not_called()


class SafetyTests(unittest.TestCase):
    def test_cycle_limit_requires_a_complete_freshly_parked_cycle(self):
        # HH_261007 - Mileage target remains genuine; a limit is not a pass shortcut.
        for fault in (None, "parking", "history"):
            with self.subTest(fault=fault):
                runner = observation_runner()
                runner.identity = runner.previous_generation = None
                runner.args = SimpleNamespace(sites="B13", adopt_generation=None,
                                              target_m=10000.0, max_cycles=1)
                runner.sites = {"B13": object()}
                runner.drop_zone = DropZone("drop", 0, 0, 0, 0)
                runner.report = {"status": "RUNNING", "cycles": [], "target_m": 10000.0}
                runner.mileage = {"xy_distance_m": 123.0}
                runner.wait_stationary_boundary = Mock()
                runner.preflight_forward_road_return = Mock()
                runner.confirm = Mock()
                runner.ui.state.side_effect = [cleared_state(), active_state()]
                snapshot = parked_snapshot()
                snapshot["sequences"]["site_phases"] = ["ARRIVED", "RETURN_DONE"]
                snapshot["sequences"]["drop_zone_phases"] = ["ALIGN_PARKING_YAW"]
                if fault == "parking":
                    snapshot["gate"]["level"] = 1
                elif fault == "history":
                    snapshot["sequences"]["drop_zone_phases"] = []
                def wait(label, predicate, **_kwargs):
                    if label == "full_return_and_parking" and not predicate(cleared_state(), snapshot):
                        raise MatrixError("parking not proven")
                    return cleared_state(), snapshot
                runner.wait = wait
                runner.sample = Mock(return_value=(cleared_state(), snapshot))
                with patch("repeat_simulator_missions.expected_site_phases",
                           return_value=["ARRIVED", "RETURN_DONE"]):
                    if fault is None:
                        runner.run()
                        self.assertEqual(runner.report["status"], "CYCLE_LIMIT_REACHED_PARKED")
                        self.assertEqual(len(runner.report["cycles"]), 1)
                        self.assertIsNone(runner.identity)
                        runner.sample.assert_called_once_with(allow_inactive=True)
                        runner.event.assert_any_call(
                            "cycle_limit_reached_at_verified_parked_boundary", completed_cycles=1)
                    else:
                        with self.assertRaises(MatrixError):
                            runner.run()
                        self.assertEqual(runner.report["status"], "RUNNING")
                        self.assertFalse(runner.report["cycles"])
                runner.client.dispatch.assert_called_once_with("B13", "delivery")
                self.assertEqual(runner.report["target_m"], 10000.0)

    def test_invalid_cycle_limit_rejected_before_any_runtime_access(self):
        for limit in ("0", "-1", "1.5", "invalid"):
            with self.subTest(limit=limit), \
                    patch("repeat_simulator_missions.RosObservation") as ros, \
                    patch("sys.stderr"):
                with self.assertRaises(SystemExit):
                    main(["--output", "/unused", "--mileage-summary", "/unused/summary",
                          "--max-cycles", limit])
                ros.assert_not_called()

    def test_owner_change_stops_supervision(self):
        state = active_state()
        state["mission_dispatch_generation"] += 1
        with self.assertRaises(Intervention):
            check_owner(state, IDENTITY)

    def test_pending_manual_resume_is_not_permission(self):
        state = {**active_state(), "manual_resume": {"pending": True, "can_resume": True}}
        with self.assertRaises(Intervention):
            check_owner(state, IDENTITY)

    def test_inactive_before_return_is_intervention(self):
        with self.assertRaises(Intervention):
            check_owner({"mission_dispatch_active": False}, IDENTITY)

    def test_existing_active_cannot_be_replaced(self):
        with self.assertRaises(Intervention):
            check_owner(active_state(), None)

    def test_healthy_parking_and_physical_stationarity(self):
        state = {"mission_dispatch_active": False, "service_state": 0}
        snapshot = parked_snapshot()
        drop = DropZone("drop", 0, 0, 0, 0)
        self.assertTrue(stationary_parked(state, snapshot, drop))
        self.assertEqual(parking_completion(snapshot), "reverse")
        snapshot["carla_odometry"]["speed_mps"] = 0.06
        self.assertFalse(stationary_parked(state, snapshot, drop))

    def test_stale_parked_without_history_cannot_prove_repeat(self):
        snapshot = parked_snapshot()
        snapshot["sequences"] = {}
        self.assertFalse(parking_completion(snapshot))

    def test_delayed_ros_parking_within_owned_observation_grace(self):
        # HH_261006 - The transient snapshot cannot pass parking completion.
        runner = observation_runner()
        pending = parked_snapshot()
        pending["parking"]["dispatcher"]["operating_state"] = "PARKING"
        pending["parking"]["reverse"]["operating_state"] = "REVERSE_APPROACH"
        pending["sequences"]["parking_phases"] = ["reverse:REVERSE_APPROACH"]
        sample_at(runner, active_state(), pending, 100.0)
        _, observed = sample_at(runner, cleared_state(), pending, 100.1)
        self.assertFalse(parking_completion(observed))
        self.assertFalse(stationary_parked(cleared_state(), observed, DropZone("drop", 0, 0, 0, 0)))
        _, observed = sample_at(runner, cleared_state(), parked_snapshot(), 102.0)
        self.assertEqual(parking_completion(observed), "reverse")
        self.assertEqual(runner.observer.pump_ui_callbacks.call_count, 3)
        self.assertEqual(runner.observer.spin_once.call_count, 3)
        runner.event.assert_called_once_with(
            "awaiting_ros_parking_after_owned_http_clear", identity=IDENTITY,
            observation_grace_s=2.0, parking_completion_observed=False)

    def test_parking_grace_expires_and_late_completion_does_not_extend_it(self):
        for late_complete in (False, True):
            with self.subTest(late_complete=late_complete):
                runner = observation_runner()
                pending = parked_snapshot()
                pending["sequences"] = {}
                sample_at(runner, active_state(), pending, 100.0)
                sample_at(runner, cleared_state(), pending, 100.1)
                sample_at(runner, cleared_state(), pending, 101.9)
                late = parked_snapshot() if late_complete else pending
                with self.assertRaisesRegex(Intervention, "grace expired"):
                    sample_at(runner, cleared_state(), late, 102.100001)

    def test_grace_requires_observed_owner_and_explicit_return_wait(self):
        pending = parked_snapshot()
        pending["sequences"] = {}
        runner = observation_runner()
        with self.assertRaises(Intervention):
            sample_at(runner, cleared_state(), pending, 100.0)
        sample_at(runner, active_state(), pending, 100.0)
        with self.assertRaises(Intervention):
            sample_at(runner, cleared_state(), pending, 100.1, allow_inactive=False)
        with self.assertRaises(Intervention):
            sample_at(runner, {**cleared_state(), "service_state": 10}, pending, 100.1)

    def test_replacement_manual_failure_and_collision_abort_during_grace(self):
        cases = (
            ({**active_state(), "mission_dispatch_generation": 124}, {}, Intervention),
            ({**cleared_state(), "mission_dispatch_generation": 124}, {}, Intervention),
            ({**cleared_state(), "control_mode": 0}, {}, Intervention),
            ({**cleared_state(), "operation_mode": "MANUAL"}, {}, Intervention),
            ({**cleared_state(), "manual_resume": {"pending": True}}, {}, Intervention),
            ({**cleared_state(), "mission_execution_error": "failed"}, {}, MatrixError),
            (cleared_state(), {"service_state": {"state": 16}}, MatrixError),
            (cleared_state(), {"gate": {"operating_state": "MANUAL"}}, Intervention),
            (cleared_state(), {"collision_evidence": {"event_count": 1}}, MatrixError),
        )
        for state, override, error in cases:
            with self.subTest(state=state, override=override):
                runner = observation_runner()
                pending = parked_snapshot()
                pending["sequences"] = {}
                sample_at(runner, active_state(), pending, 100.0)
                sample_at(runner, cleared_state(), pending, 100.1)
                with self.assertRaises(error):
                    sample_at(runner, state, {**pending, **override}, 100.2)

    def test_reactivation_or_lost_parking_proof_cannot_restart_grace(self):
        for reactivate in (False, True):
            runner = observation_runner()
            pending = parked_snapshot()
            pending["sequences"] = {}
            sample_at(runner, active_state(), pending, 100.0)
            sample_at(runner, cleared_state(), pending, 100.1)
            if reactivate:
                with self.assertRaises(Intervention):
                    sample_at(runner, active_state(), pending, 100.2)
            else:
                sample_at(runner, cleared_state(), parked_snapshot(), 100.2)
                with self.assertRaises(Intervention):
                    sample_at(runner, cleared_state(), pending, 100.3)

    def test_http_failure_wins_over_an_earlier_ros_parked_snapshot(self):
        runner = observation_runner()
        sample_at(runner, active_state(), parked_snapshot(), 100.0)
        with self.assertRaisesRegex(MatrixError, "service failure"):
            sample_at(runner, {**cleared_state(), "service_state": 16},
                      parked_snapshot(), 100.1)

    def test_pending_parking_grace_skips_only_optional_screenshot(self):
        runner = observation_runner()
        pending = parked_snapshot()
        pending["sequences"] = {}
        sample_at(runner, active_state(), pending, 100.0)
        runner.last_frame = 0.0
        with patch("repeat_simulator_missions.capture_ui_handoff_view") as capture:
            sample_at(runner, cleared_state(), pending, 100.1)
        capture.assert_not_called()

    def test_cold_join_accepts_only_explicit_current_ui_terminal_without_ros(self):
        # HH_261006 - Neither the default predicate nor history gains an exemption.
        drop = DropZone("drop", 0, 0, 0, 0)
        for terminal, gate in ((0, "STANDBY"), (13, "CHARGING")):
            for missing_key in (False, True):
                with self.subTest(terminal=terminal, missing_key=missing_key):
                    state = {"mission_dispatch_active": False, "service_state": terminal}
                    snapshot = parked_snapshot()
                    snapshot["service_state"] = None
                    snapshot["gate"]["operating_state"] = gate
                    snapshot["sequences"] = {}
                    if missing_key:
                        del snapshot["service_state"]
                    original = deepcopy(snapshot)
                    self.assertFalse(stationary_parked(state, snapshot, drop))
                    self.assertTrue(stationary_parked(
                        state, snapshot, drop, allow_late_join_ui_state=True))
                    self.assertEqual(snapshot, original)
                    self.assertFalse(parking_completion(snapshot))

    def test_cold_join_never_replaces_present_conflicting_or_malformed_ros(self):
        state = {"mission_dispatch_active": False, "service_state": 0}
        drop = DropZone("drop", 0, 0, 0, 0)
        for service in ({"state": 13}, {"state": 10}, {"state": 16},
                        {"state": None}, {}):
            with self.subTest(service=service):
                snapshot = parked_snapshot()
                snapshot["service_state"] = service
                self.assertFalse(stationary_parked(
                    state, snapshot, drop, allow_late_join_ui_state=True))

    def test_cold_join_rejects_nonterminal_or_invalid_ui_state(self):
        snapshot = parked_snapshot()
        snapshot["service_state"] = None
        drop = DropZone("drop", 0, 0, 0, 0)
        for service in (None, False, "0", 10, 11, 16):
            with self.subTest(service=service):
                self.assertFalse(stationary_parked(
                    {"mission_dispatch_active": False, "service_state": service},
                    snapshot, drop, allow_late_join_ui_state=True))

    def test_cold_join_preserves_every_physical_parking_guard(self):
        state = {"mission_dispatch_active": False, "service_state": 0}
        drop = DropZone("drop", 0, 0, 0, 0)
        mutations = (
            lambda snap: snap["gate"].update(level=1),
            lambda snap: snap["gate"].update(operating_state="CHARGING"),
            lambda snap: snap["parking"]["dispatcher"].update(level=1),
            lambda snap: snap["parking"]["dispatcher"].update(operating_state="IDLE"),
            lambda snap: snap["parking"]["reverse"].update(level=1),
            lambda snap: snap["parking"]["reverse"].update(operating_state="IDLE"),
            lambda snap: snap["physical"].update(ready=False),
            lambda snap: snap["carla_odometry"].update(speed_mps=0.06),
            lambda snap: snap["carla_odometry"].update(speed_mps=float("nan")),
            lambda snap: snap["pose"].update(x_m=4),
            lambda snap: snap.pop("pose"),
        )
        for index, mutate in enumerate(mutations):
            with self.subTest(guard=index):
                snapshot = parked_snapshot()
                snapshot["service_state"] = None
                mutate(snapshot)
                self.assertFalse(stationary_parked(
                    state, snapshot, drop, allow_late_join_ui_state=True))
        snapshot = parked_snapshot()
        snapshot["service_state"] = None
        self.assertFalse(stationary_parked(
            {**state, "mission_dispatch_active": True}, snapshot, drop,
            allow_late_join_ui_state=True))

    def test_runner_limits_fallback_to_first_cold_boundary_and_reports_source(self):
        state = {"mission_dispatch_active": False, "service_state": 0}
        snapshot = parked_snapshot()
        snapshot["service_state"] = None
        snapshot["sequences"] = {}
        runner = object.__new__(Runner)
        runner.args = SimpleNamespace(adopt_generation=None)
        runner.identity = runner.previous_generation = None
        runner.report = {"cycles": []}
        runner.drop_zone = DropZone("drop", 0, 0, 0, 0)
        runner.event = Mock()
        def wait(_label, predicate, **_kwargs):
            if not predicate(state, snapshot):
                raise MatrixError("parking evidence rejected")
            return state, snapshot
        runner.wait = wait
        self.assertEqual(runner.wait_stationary_boundary(), (state, snapshot))
        runner.event.assert_called_once_with(
            "initial_parking_service_state_from_current_http_ui",
            service_state_source="current_http_ui_state", service_state=0,
            initial_boundary_only=True, ros_service_state_observed=False,
            parking_history_inferred=False)
        for identity, previous, cycles, adopted in (
                (IDENTITY, None, [], None), (None, 123, [], None),
                (None, None, [{"status": "COMPLETED_OBSERVED"}], None),
                (None, None, [], 123)):
            with self.subTest(identity=identity, previous=previous,
                              cycles=cycles, adopted=adopted):
                runner.identity, runner.previous_generation = identity, previous
                runner.report["cycles"] = cycles
                runner.args.adopt_generation = adopted
                with self.assertRaises(MatrixError):
                    runner.wait_stationary_boundary()
        self.assertEqual(runner.event.call_count, 1)

    def test_stop_checks_identity_before_second_confirmation(self):
        client, ui = Mock(), Mock()
        replacement = {**active_state(), "mission_dispatch_generation": 124}
        ui.state.side_effect = [active_state(), replacement]
        with self.assertRaises(Intervention):
            stop_owned(client, ui, IDENTITY)
        self.assertEqual(client._click.call_count, 1)

    def test_failure_does_not_prevent_owned_stop(self):
        client, ui = Mock(), Mock()
        failed = {**active_state(), "mission_execution_error": "planner failed"}
        ui.state.side_effect = [failed, failed, {"mission_dispatch_active": False}]
        self.assertTrue(stop_owned(client, ui, IDENTITY)["requested"])
        self.assertEqual(client._click.call_count, 2)

    def test_missing_error_screen_stop_uses_owned_cleanup_not_ui_pass(self):
        # HH_261007 - Error cleanup is distinct from successful visible controls.
        client, ui = Mock(), Mock()
        failed = {**active_state(), "mission_execution_error": "crab timeout"}
        client._evaluate.return_value = True
        ui.state.side_effect = [failed, failed, cleared_state()]
        result = stop_owned(client, ui, IDENTITY)
        ui.stop.assert_called_once_with()
        client._click.assert_not_called()
        self.assertFalse(result["scenario_pass"])
        self.assertEqual(result["transport"], "production_stop_endpoint_error_cleanup")

    def test_error_cleanup_never_stops_replacement_or_recovered_mission(self):
        for newer, error in (
            ({**active_state(), "mission_dispatch_generation": 124}, Intervention),
            ({**active_state(), "operation_mode": "MANUAL"}, Intervention),
            (active_state(), MatrixError),
        ):
            client, ui = Mock(), Mock()
            client._evaluate.return_value = True
            ui.state.side_effect = [
                {**active_state(), "mission_execution_error": "crab timeout"}, newer]
            with self.assertRaises(error):
                stop_owned(client, ui, IDENTITY)
            ui.stop.assert_not_called()
            client._click.assert_not_called()

    def test_mileage_only_accepts_live_independent_source_and_no_restart(self):
        summary = {"environment": "simulation", "status": "recording",
                   "measurement_source": "/carla/ego_vehicle/odometry",
                   "distance_basis": "raw_sensor_planar_position_displacements",
                   "xy_distance_m": 42.0, "started_utc": "session-one"}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "summary.json"
            path.write_text(json.dumps(summary))
            self.assertEqual(read_mileage(path)["xy_distance_m"], 42.0)
            with self.assertRaises(MatrixError):
                read_mileage(path, {**summary, "xy_distance_m": 43.0})
            with self.assertRaises(MatrixError):
                read_mileage(path, {**summary, "started_utc": "different"})
            with self.assertRaises(MatrixError):
                read_mileage(path, now=path.stat().st_mtime + 6)

    def test_active_or_far_vehicle_is_not_stationary_drop_zone(self):
        snapshot = parked_snapshot()
        drop = DropZone("drop", 0, 0, 0, 0)
        self.assertFalse(stationary_parked(active_state(), snapshot, drop))
        snapshot["pose"]["x_m"] = 4
        self.assertFalse(stationary_parked({"mission_dispatch_active": False, "service_state": 0}, snapshot, drop))


if __name__ == "__main__":
    unittest.main()
