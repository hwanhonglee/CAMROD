"""Regression coverage for the merged Guest UI transport and kiosk options."""

import asyncio
import ast
import os
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import patch

import yaml


sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "runtime" / "python"),
)

from camrod_ui.operator_ui_window import (  # noqa: E402
    _build_chromium_command,
    _build_parser,
    _find_chromium_browser,
    _wait_for_ui_ready,
)
from camrod_ui.ui_guest_node import UiGuestNode  # noqa: E402


class _ConcurrentSendProbe:

    def __init__(self) -> None:
        self.active = 0
        self.overlap_observed = False
        self.payloads = []

    async def send_json(self, payload: dict) -> None:
        self.active += 1
        self.overlap_observed = self.overlap_observed or self.active > 1
        await asyncio.sleep(0.01)
        self.payloads.append(payload)
        self.active -= 1


class _GuestTransport:

    def __init__(self) -> None:
        self._guest_ws_send_lock = asyncio.Lock()
        self._guest_ws_lock = threading.Lock()
        self._guest_ws = None
        self._last_client_ip = None
        self._grace_until = 0.0
        self.grace_period_s = 60


class _FakeServerLoop:
    def __init__(self) -> None:
        self.wake_calls = 0

    def is_running(self) -> bool:
        return True

    def call_soon_threadsafe(self, callback) -> None:
        self.wake_calls += 1
        callback()


class _FakeServerThread:
    def __init__(self) -> None:
        self.alive = True
        self.join_timeouts = []

    def is_alive(self) -> bool:
        return self.alive

    def join(self, timeout: float) -> None:
        self.join_timeouts.append(timeout)
        self.alive = False


class GuestTransportTest(unittest.IsolatedAsyncioTestCase):
    async def test_concurrent_json_writes_are_serialized(self) -> None:
        transport = _GuestTransport()
        websocket = _ConcurrentSendProbe()

        await asyncio.gather(
            UiGuestNode._send_guest_payload(transport, websocket, {"id": 1}),
            UiGuestNode._send_guest_payload(transport, websocket, {"id": 2}),
        )

        self.assertFalse(websocket.overlap_observed)
        self.assertEqual(websocket.payloads, [{"id": 1}, {"id": 2}])

    async def test_connection_claim_and_grace_do_not_require_an_await(self) -> None:
        transport = _GuestTransport()
        first = object()
        second = object()

        self.assertIsNone(
            UiGuestNode._claim_guest_connection(
                transport, first, "192.0.2.10", now_s=100.0
            )
        )
        self.assertEqual(
            UiGuestNode._claim_guest_connection(
                transport, second, "192.0.2.11", now_s=100.0
            ),
            0,
        )

        UiGuestNode._release_guest_connection(transport, first, now_s=100.0)
        self.assertEqual(
            UiGuestNode._claim_guest_connection(
                transport, second, "192.0.2.11", now_s=102.0
            ),
            58,
        )
        self.assertIsNone(
            UiGuestNode._claim_guest_connection(
                transport, second, "192.0.2.10", now_s=102.0
            )
        )

    async def test_guest_http_server_stops_before_ros_entities(self) -> None:
        server = type("Server", (), {"should_exit": False})()
        loop = _FakeServerLoop()
        thread = _FakeServerThread()
        transport = type("Transport", (), {})()
        transport._server_stop_requested = threading.Event()
        transport._uvicorn_server = server
        transport._main_loop = loop
        transport._server_thread = thread
        transport.get_logger = lambda: type(
            "Logger", (), {"warn": lambda _self, _message: None}
        )()

        UiGuestNode._stop_fastapi_server(transport, join_timeout_s=0.75)

        self.assertTrue(transport._server_stop_requested.is_set())
        self.assertTrue(server.should_exit)
        self.assertEqual(loop.wake_calls, 1)
        self.assertEqual(thread.join_timeouts, [0.75])


class OperatorWindowTest(unittest.TestCase):

    def test_fullscreen_defaults_on_and_can_be_disabled(self) -> None:
        self.assertTrue(_build_parser().parse_args([]).fullscreen)
        self.assertFalse(_build_parser().parse_args(["--no-fullscreen"]).fullscreen)

    def test_webkit_engine_is_default_with_explicit_chromium_override(self) -> None:
        parser = _build_parser()
        self.assertEqual(parser.parse_args([]).engine, "webkit")
        self.assertEqual(parser.parse_args(["--engine", "webkit"]).engine, "webkit")
        self.assertEqual(
            parser.parse_args(["--engine", "chromium"]).engine,
            "chromium",
        )

        with patch.dict(os.environ, {"CAMROD_UI_BROWSER": ""}), patch(
            "camrod_ui.operator_ui_window.shutil.which",
            side_effect=lambda name: "/usr/bin/chromium" if name == "chromium" else None,
        ):
            self.assertEqual(_find_chromium_browser(), "/usr/bin/chromium")

    def test_readiness_probe_retries_without_busy_spinning(self) -> None:
        stop_requested = threading.Event()
        with patch(
            "camrod_ui.operator_ui_window._ui_is_ready",
            side_effect=(False, False, True),
        ) as ready:
            self.assertTrue(
                _wait_for_ui_ready(
                    "http://127.0.0.1:8010",
                    stop_requested,
                    poll_interval_s=0.001,
                )
            )
        self.assertEqual(ready.call_count, 3)

    def test_chromium_command_uses_private_kiosk_profile(self) -> None:
        args = _build_parser().parse_args([])
        command = _build_chromium_command("/usr/bin/chromium", args, "/tmp/profile")

        self.assertEqual(command[0], "/usr/bin/chromium")
        self.assertIn("--user-data-dir=/tmp/profile", command)
        self.assertIn("--kiosk", command)
        self.assertEqual(command[-1], "http://127.0.0.1:8010")
        self.assertNotIn("--disable-gpu", command)
        self.assertIn("--enable-gpu-rasterization", command)
        self.assertIn("--enable-zero-copy", command)

        windowed = _build_parser().parse_args(["--no-fullscreen"])
        windowed_command = _build_chromium_command(
            "/usr/bin/chromium", windowed, "/tmp/profile"
        )
        self.assertIn("--app=http://127.0.0.1:8010", windowed_command)
        self.assertIn("--window-size=1280,800", windowed_command)
        self.assertNotIn("--kiosk", windowed_command)

    def test_kiosk_and_heartbeat_are_wired_through_source_assets(self) -> None:
        src_root = Path(__file__).resolve().parents[2]
        defaults = yaml.safe_load(
            (
                src_root
                / "camrod_bringup"
                / "config"
                / "bringup"
                / "launch_defaults.yaml"
            ).read_text(encoding="utf-8")
        )
        system = defaults["bringup"]["system"]
        self.assertTrue(system["operator_ui_window_fullscreen"])
        self.assertEqual(system["operator_ui_window_engine"], "webkit")

        launch_text = (
            src_root
            / "camrod_ui"
            / "camrod_ui_robot"
            / "launch"
            / "ui.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn("default_value='0.0.0.0'", launch_text)
        self.assertIn("operator_ui_window_fullscreen", launch_text)
        self.assertIn("operator_ui_window_engine", launch_text)
        self.assertIn("default_value='webkit'", launch_text)
        self.assertIn("'--fullscreen' if '", launch_text)
        self.assertIn("else '--no-fullscreen'", launch_text)

        bringup_text = (
            src_root / "camrod_bringup" / "launch" / "_bringup_impl.py"
        ).read_text(encoding="utf-8")
        self.assertGreaterEqual(
            bringup_text.count("'operator_ui_window_fullscreen'"),
            2,
        )
        self.assertGreaterEqual(bringup_text.count("'operator_ui_window_engine'"), 2)

        window_text = (
            src_root
            / "camrod_ui"
            / "runtime"
            / "python"
            / "camrod_ui"
            / "operator_ui_window.py"
        ).read_text(encoding="utf-8")
        self.assertIn("window.set_deletable(False)", window_text)
        self.assertIn("HardwareAccelerationPolicy.ALWAYS", window_text)
        self.assertIn('"set_enable_smooth_scrolling": True', window_text)
        self.assertIn('"set_enable_webgl": False', window_text)
        self.assertIn("window.fullscreen()", window_text)
        self.assertIn("CHROMIUM_CANDIDATES", window_text)
        self.assertIn("_run_chromium", window_text)
        self.assertIn("_run_webkit", window_text)

        guest_html = (
            src_root
            / "camrod_ui"
            / "camrod_ui_guest"
            / "assets"
            / "guest_frontend"
            / "index.html"
        ).read_text(encoding="utf-8")
        self.assertIn("{ action: 'heartbeat' }", guest_html)
        self.assertIn("}, 10000);", guest_html)

        guest_backend = (
            src_root
            / "camrod_ui"
            / "runtime"
            / "python"
            / "camrod_ui"
            / "ui_guest_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn('if action == "heartbeat":', guest_backend)
        self.assertIn("if idle_cycles >= 3:", guest_backend)
        self.assertIn("await asyncio.to_thread", guest_backend)


class BackendEndpointContractTest(unittest.TestCase):

    def test_ros_touching_rest_handlers_run_off_the_event_loop(self) -> None:
        source_path = (
            Path(__file__).resolve().parents[1]
            / "runtime"
            / "python"
            / "camrod_ui"
            / "ui_backend_node.py"
        )
        tree = ast.parse(source_path.read_text(encoding="utf-8"))
        sync_functions = {
            node.name for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
        }
        async_functions = {
            node.name
            for node in ast.walk(tree)
            if isinstance(node, ast.AsyncFunctionDef)
        }

        expected_sync = {
            "get_state",
            "get_destination",
            "get_diagnostics",
            "get_api_diagnostics",
            "post_engage",
            "post_headlight",
            "post_operation_mode",
            "post_auto",
            "post_stop",
            "post_destination",
        }
        self.assertTrue(expected_sync.issubset(sync_functions))
        self.assertTrue(expected_sync.isdisjoint(async_functions))


if __name__ == "__main__":
    unittest.main()
