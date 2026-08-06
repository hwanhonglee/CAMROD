"""Regression coverage for the Robot UI operator-stop command path."""

from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest


sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "runtime" / "python"),
)

from avg_msgs.msg import AvgPlatformStatus, AvgServiceState, MotionOperation  # noqa: E402
from camrod_ui.ui_backend_node import UiBackendNode  # noqa: E402


class _FakeLogger:

    def __init__(self) -> None:
        self.info_messages = []
        self.warning_messages = []

    def info(self, message: str) -> None:
        self.info_messages.append(message)

    def warn(self, message: str) -> None:
        self.warning_messages.append(message)


class _FakeCancelClient:

    def __init__(self, ready: bool) -> None:
        self.ready = ready
        self.requests = []

    def service_is_ready(self) -> bool:
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return object()


class _FakeLoop:

    def __init__(self) -> None:
        self.wake_calls = 0

    def is_running(self) -> bool:
        return True

    def call_soon_threadsafe(self, callback) -> None:
        self.wake_calls += 1
        callback()


class _FakeThread:

    def __init__(self) -> None:
        self.alive = True
        self.join_timeouts = []

    def is_alive(self) -> bool:
        return self.alive

    def join(self, timeout: float) -> None:
        self.join_timeouts.append(timeout)
        self.alive = False


class _FakeBackend:

    def __init__(self) -> None:
        self.nav2_cancel_action_topics = ["/ready/cancel", "/missing/cancel"]
        self.nav2_cancel_clients = [
            _FakeCancelClient(ready=True),
            _FakeCancelClient(ready=False),
        ]
        self.operations = []
        self.logger = _FakeLogger()

    def get_logger(self):
        return self.logger

    def _record_operation(self, owner: str, operation: int, source: str) -> None:
        self.operations.append((owner, operation, source))

    def _publish_camping_site_operation(self, operation: int, source: str) -> None:
        self._record_operation("camping_site", operation, source)

    def _publish_drop_zone_operation(self, operation: int, source: str) -> None:
        self._record_operation("drop_zone", operation, source)

    def _publish_parking_operation(self, operation: int, source: str) -> None:
        self._record_operation("parking", operation, source)


class UiBackendStopTest(unittest.TestCase):

    def test_charging_contact_does_not_cancel_active_station_departure(self) -> None:
        published_states = []
        logger = _FakeLogger()
        backend = SimpleNamespace(
            _drop_zone_exit_active=True,
            _latest_platform_is_charging=False,
            _latest_service_state=int(AvgServiceState.DEPARTING_CHARGER),
            _runtime_policy=SimpleNamespace(update_platform=lambda **_kwargs: None),
        )
        backend.get_logger = lambda: logger
        backend._update_runtime_state = lambda callback: callback()
        backend._publish_service_state = (
            lambda state, source: published_states.append((state, source))
        )
        backend._update_low_battery_return_policy = lambda *_args, **_kwargs: None

        message = AvgPlatformStatus()
        message.is_charging = True
        message.battery_state_available = False
        UiBackendNode._on_platform_status(backend, message)

        self.assertTrue(backend._latest_platform_is_charging)
        self.assertEqual(published_states, [])
        self.assertIn("preserving departure authorization", logger.info_messages[-1])

    def test_duplicate_destination_is_idempotent_during_station_departure(self) -> None:
        logger = _FakeLogger()
        backend = SimpleNamespace(
            _drop_zone_exit_active=True,
            _pending_site_after_drop_zone_exit=("B2", "camping_site_2", "first"),
        )
        backend.get_logger = lambda: logger

        result = UiBackendNode._apply_destination_command(
            backend,
            site="B2",
            run=True,
            source="retry",
        )

        self.assertEqual(result["mission_key"], "camping_site_2")
        self.assertFalse(result["goal_pose_published"])
        self.assertIn("already pending", result["message"])

    def test_both_ui_nodes_accept_external_shutdown_as_clean_exit(self) -> None:
        runtime_dir = Path(__file__).resolve().parents[1] / "runtime" / "python" / "camrod_ui"
        for filename in ("ui_backend_node.py", "ui_guest_node.py"):
            source = (runtime_dir / filename).read_text(encoding="utf-8")
            self.assertIn("from rclpy.executors import ExternalShutdownException", source)
            self.assertIn("except (KeyboardInterrupt, ExternalShutdownException):", source)

    def test_http_server_is_stopped_before_node_destruction(self) -> None:
        server = SimpleNamespace(should_exit=False)
        loop = _FakeLoop()
        thread = _FakeThread()
        backend = SimpleNamespace(
            _server_stop_requested=threading.Event(),
            _uvicorn_server=server,
            _main_loop=loop,
            _server_thread=thread,
        )
        backend.get_logger = lambda: _FakeLogger()

        UiBackendNode._stop_fastapi_server(backend, join_timeout_s=1.25)

        self.assertTrue(backend._server_stop_requested.is_set())
        self.assertTrue(server.should_exit)
        self.assertEqual(loop.wake_calls, 1)
        self.assertEqual(thread.join_timeouts, [1.25])

    def test_cancel_uses_rclpy_call_async_and_cancels_every_motion_owner(self) -> None:
        backend = _FakeBackend()

        UiBackendNode._cancel_active_motion(backend, source="http_stop")

        self.assertEqual(len(backend.nav2_cancel_clients[0].requests), 1)
        self.assertEqual(backend.nav2_cancel_clients[1].requests, [])
        self.assertEqual(
            backend.operations,
            [
                ("camping_site", MotionOperation.CANCEL, "http_stop:operator_stop"),
                ("drop_zone", MotionOperation.CANCEL, "http_stop:operator_stop"),
                ("parking", MotionOperation.CANCEL, "http_stop:operator_stop"),
            ],
        )
        self.assertIn("/ready/cancel", backend.logger.info_messages[0])

    def test_arrival_uses_active_mission_site_after_destination_is_cleared(self) -> None:
        backend = SimpleNamespace(
            _latest_service_state=None,
            _active_mission_site="B6",
            _lock=threading.Lock(),
            _state=SimpleNamespace(
                destination={"site": "", "run": False},
                service_state=-1,
                service_state_name="",
                service_state_description="",
                battery_percentage=80,
            ),
            publish_mission_engage_from_destination=False,
            arrival_notifications=[],
            broadcasts=[],
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._schedule_broadcast = backend.broadcasts.append
        backend._notify_site_arrival = lambda site, state, source: (
            backend.arrival_notifications.append((site, state, source))
        )
        backend._publish_engage = lambda *_args, **_kwargs: None
        backend._update_low_battery_return_policy = lambda *_args, **_kwargs: None

        message = AvgServiceState()
        message.state = AvgServiceState.WAITING_FOR_RETURN_REQUEST
        message.state_name = "WAITING_FOR_RETURN_REQUEST"
        message.description = "Waiting for return request"
        UiBackendNode._on_service_state(backend, message)

        self.assertEqual(
            backend.arrival_notifications,
            [
                (
                    "B6",
                    AvgServiceState.WAITING_FOR_RETURN_REQUEST,
                    f"service_state:{AvgServiceState.WAITING_FOR_RETURN_REQUEST}",
                )
            ],
        )

    def test_battery_rejected_destination_does_not_become_active_site(self) -> None:
        backend = SimpleNamespace(
            _active_mission_site="B4",
            _lock=threading.Lock(),
            broadcasts=[],
        )
        backend.get_logger = lambda: _FakeLogger()
        backend._resolve_mission_key_for_site = lambda site: f"camping_site_{site[1:]}"
        backend._is_site_occupied = lambda _site: False
        backend._site_arrival_match = lambda _site: (
            False,
            "camping_site_6",
            20.0,
            "outside_arrival_radius",
        )
        backend._mission_dispatch_battery_block = lambda _site: {
            "message": "battery below mission minimum",
            "battery_percentage": 30,
            "minimum_battery_percentage": 35,
        }
        backend._schedule_broadcast = backend.broadcasts.append

        result = UiBackendNode._apply_destination_command(
            backend,
            site="B6",
            run=True,
            source="test",
        )

        self.assertTrue(result["blocked"])
        self.assertEqual(backend._active_mission_site, "B4")


if __name__ == "__main__":
    unittest.main()
