"""Reset must release the backend-owned mission before restoring the world."""

import threading
import time
import types
import unittest
from unittest import mock

from camrod_ui_tester.simulation_state import Pose2D, ScenarioEngine
from camrod_ui_tester.simulator_node import UiSimulatorNode


class _Logger:
    def __init__(self):
        self.warnings = []

    def warn(self, message):
        self.warnings.append(str(message))

    def info(self, message):
        pass


def _stub_node(*, backend_stop_url="http://127.0.0.1:8010/ui/stop"):
    node = types.SimpleNamespace()
    node.backend_stop_url = backend_stop_url
    node.backend_stop_timeout_s = 2.0
    node._backend_stop_suppress_until = 0.0
    node._started_monotonic = time.monotonic() - 60.0
    node._lock = threading.RLock()
    node._events = []
    node._event_sequence = 0
    node._engine = ScenarioEngine(
        station_pose=Pose2D(0.0, 0.0, 0.0),
        sites={"B1": Pose2D(12.0, 4.0, 90.0)},
    )
    node._scenario_profiles = {}
    logger = _Logger()
    node.get_logger = lambda: logger
    node.RESET_SCENARIOS = UiSimulatorNode.RESET_SCENARIOS
    node._record_event = lambda *args, **kwargs: node._events.append((args, kwargs))
    node._request_backend_stop = (
        lambda reason: UiSimulatorNode._request_backend_stop(node, reason)
    )
    node._apply_patch_locked = (
        lambda values: UiSimulatorNode._apply_patch_locked(node, values)
    )
    return node, logger


class _Response:
    def __init__(self, status=200):
        self.status = status

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


class SimulatorBackendStopTest(unittest.TestCase):
    def test_reset_stop_posts_to_backend_and_arms_suppression(self):
        node, _ = _stub_node()
        with mock.patch(
            "camrod_ui_tester.simulator_node.urllib.request.urlopen",
            return_value=_Response(200),
        ) as urlopen:
            accepted = UiSimulatorNode._request_backend_stop(node, "reset")
        self.assertTrue(accepted)
        request = urlopen.call_args.args[0]
        self.assertEqual(request.full_url, "http://127.0.0.1:8010/ui/stop")
        self.assertEqual(request.get_method(), "POST")
        self.assertGreater(node._backend_stop_suppress_until, time.monotonic())

    def test_empty_url_disables_the_hook_without_network_access(self):
        node, _ = _stub_node(backend_stop_url="")
        with mock.patch(
            "camrod_ui_tester.simulator_node.urllib.request.urlopen"
        ) as urlopen:
            accepted = UiSimulatorNode._request_backend_stop(node, "reset")
        self.assertFalse(accepted)
        urlopen.assert_not_called()

    def test_unreachable_backend_warns_but_does_not_break_the_reset(self):
        node, logger = _stub_node()
        with mock.patch(
            "camrod_ui_tester.simulator_node.urllib.request.urlopen",
            side_effect=OSError("connection refused"),
        ):
            accepted = UiSimulatorNode._request_backend_stop(node, "reset")
        self.assertFalse(accepted)
        self.assertTrue(
            any("backend stop request failed" in entry for entry in logger.warnings)
        )

    def test_reset_scenarios_release_the_backend_mission_first(self):
        node, _ = _stub_node()
        requested = []
        node._request_backend_stop = lambda reason: requested.append(reason)
        for scenario in ("ready", "boot"):
            UiSimulatorNode.apply_scenario(node, scenario)
        self.assertEqual(requested, ["scenario=ready", "scenario=boot"])
        UiSimulatorNode.apply_scenario(node, "low_battery")
        self.assertEqual(requested, ["scenario=ready", "scenario=boot"])

    def test_operator_stopped_echo_is_ignored_inside_suppression_window(self):
        node, _ = _stub_node()
        node._engine.apply_scenario("ready")
        node._backend_stop_suppress_until = time.monotonic() + 4.0
        stopped = types.SimpleNamespace(state=16, description="Stopped by operator")
        UiSimulatorNode._on_service_state(node, stopped)
        self.assertEqual(node._engine.state.service_state, "DROP_ZONE_WAIT")
        node._backend_stop_suppress_until = 0.0
        UiSimulatorNode._on_service_state(node, stopped)
        self.assertEqual(node._engine.state.service_state, "OPERATOR_STOPPED")


if __name__ == "__main__":
    unittest.main()
