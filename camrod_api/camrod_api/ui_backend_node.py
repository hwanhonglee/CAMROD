#!/usr/bin/env python3
# HH_260327: Lightweight HTTP backend for CAMROD plugin API + status panel.

from __future__ import annotations

import json
import mimetypes
import errno
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List
from urllib.parse import parse_qs, urlparse

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from camrod_api.api_common import to_diag_level_int


@dataclass
class ApiState:
    """In-memory snapshot exposed by the HTTP API."""

    engaged: bool = False
    ready: bool = False
    operation_mode: str = "STOP"
    ready_message: str = ""
    module_states: List[Dict[str, Any]] = field(default_factory=list)
    diagnostics_agg_count: int = 0
    diagnostics_agg_error_count: int = 0


class UiBackendNode(Node):
    """HTTP backend that bridges web UI actions to ROS plugin API services."""

    # Initializes ROS interfaces and optional embedded HTTP server.
    def __init__(self) -> None:
        super().__init__("ui_backend")

        # HH_260415: Localhost-first default to avoid unintended external exposure.
        self.host = str(self.declare_parameter("host", "127.0.0.1").value)
        self.port = int(self.declare_parameter("port", 8010).value)
        self.enable_http_server = bool(self.declare_parameter("enable_http_server", True).value)
        # HH_260415: Avoid fatal crash when bringup is relaunched while old UI backend still holds the port.
        self.port_auto_increment = bool(self.declare_parameter("port_auto_increment", True).value)
        self.port_search_max = int(self.declare_parameter("port_search_max", 20).value)
        self.fail_on_bind_error = bool(self.declare_parameter("fail_on_bind_error", False).value)
        self.frontend_dir = Path(
            str(self.declare_parameter("frontend_dir", "").value)
        )

        self.api_set_engage_service = str(
            self.declare_parameter("api_set_engage_service", "/api/plugin/set/engage").value
        )
        self.api_set_operation_mode_service = str(
            self.declare_parameter(
                "api_set_operation_mode_service", "/api/plugin/set/operation_mode"
            ).value
        )
        self.api_set_auto_service = str(
            self.declare_parameter(
                "api_set_auto_service", "/api/plugin/set/operation_mode/auto"
            ).value
        )
        self.api_set_stop_service = str(
            self.declare_parameter(
                "api_set_stop_service", "/api/plugin/set/operation_mode/stop"
            ).value
        )

        self.api_get_engage_topic = str(
            self.declare_parameter("api_get_engage_topic", "/api/plugin/get/engage").value
        )
        self.api_ready_topic = str(
            self.declare_parameter("api_ready_topic", "/api/plugin/get/ready").value
        )
        self.api_operation_mode_topic = str(
            self.declare_parameter(
                "api_operation_mode_topic", "/api/plugin/get/operation_mode"
            ).value
        )
        self.api_module_states_topic = str(
            self.declare_parameter(
                "api_module_states_topic", "/api/plugin/get/module_states"
            ).value
        )
        self.api_ready_message_topic = str(
            self.declare_parameter(
                "api_ready_message_topic", "/api/plugin/get/ready_message"
            ).value
        )
        self.diagnostics_agg_topic = str(
            self.declare_parameter("diagnostics_agg_topic", "/diagnostics_agg").value
        )

        self._lock = threading.Lock()
        self._state = ApiState()

        self.sub_engage = self.create_subscription(
            Bool,
            self.api_get_engage_topic,
            self._on_engage,
            10,
        )
        self.sub_ready = self.create_subscription(
            Bool,
            self.api_ready_topic,
            self._on_ready,
            10,
        )
        self.sub_mode = self.create_subscription(
            String,
            self.api_operation_mode_topic,
            self._on_mode,
            10,
        )
        self.sub_ready_message = self.create_subscription(
            String,
            self.api_ready_message_topic,
            self._on_ready_message,
            10,
        )
        self.sub_module_states = self.create_subscription(
            DiagnosticArray,
            self.api_module_states_topic,
            self._on_module_states,
            10,
        )
        self.sub_diagnostics_agg = self.create_subscription(
            DiagnosticArray,
            self.diagnostics_agg_topic,
            self._on_diagnostics_agg,
            10,
        )

        self.cli_set_engage = self.create_client(SetBool, self.api_set_engage_service)
        self.cli_set_operation_mode = self.create_client(
            SetBool, self.api_set_operation_mode_service
        )
        self.cli_set_auto = self.create_client(Trigger, self.api_set_auto_service)
        self.cli_set_stop = self.create_client(Trigger, self.api_set_stop_service)

        self._server: ThreadingHTTPServer | None = None
        self._server_thread: threading.Thread | None = None
        if self.enable_http_server:
            self._start_http_server()

        self.get_logger().info(
            "ui_backend ready: "
            f"host={self.host} port={self.port} "
            f"frontend_dir={str(self.frontend_dir) if self.frontend_dir else '(builtin)'}"
        )

    # Ensures HTTP server is terminated before ROS node shutdown.
    def destroy_node(self) -> bool:
        self._shutdown_http_server()
        return super().destroy_node()

    # Updates engage state cache from ROS topic.
    def _on_engage(self, msg: Bool) -> None:
        with self._lock:
            self._state.engaged = bool(msg.data)

    # Updates ready state cache from ROS topic.
    def _on_ready(self, msg: Bool) -> None:
        with self._lock:
            self._state.ready = bool(msg.data)

    # Updates operation mode cache from ROS topic.
    def _on_mode(self, msg: String) -> None:
        with self._lock:
            self._state.operation_mode = str(msg.data)

    # Updates latest readiness reason text.
    def _on_ready_message(self, msg: String) -> None:
        with self._lock:
            self._state.ready_message = str(msg.data)

    # Converts module diagnostic array into JSON-friendly state list.
    def _on_module_states(self, msg: DiagnosticArray) -> None:
        states: List[Dict[str, Any]] = []
        for status in msg.status:
            states.append(
                {
                    "name": status.name,
                    "level": to_diag_level_int(status.level),
                    "message": status.message,
                }
            )
        with self._lock:
            self._state.module_states = states

    # Tracks aggregated diagnostics counts for quick UI checks.
    def _on_diagnostics_agg(self, msg: DiagnosticArray) -> None:
        error_count = 0
        error_level = to_diag_level_int(DiagnosticStatus.ERROR)
        for status in msg.status:
            if to_diag_level_int(status.level) >= error_level:
                error_count += 1
        with self._lock:
            self._state.diagnostics_agg_count = len(msg.status)
            self._state.diagnostics_agg_error_count = error_count

    # Returns an atomic copy of current API/UI state.
    def _snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "engaged": self._state.engaged,
                "ready": self._state.ready,
                "operation_mode": self._state.operation_mode,
                "ready_message": self._state.ready_message,
                "module_states": list(self._state.module_states),
                "diagnostics_agg_count": self._state.diagnostics_agg_count,
                "diagnostics_agg_error_count": self._state.diagnostics_agg_error_count,
            }

    # Waits for async ROS service call future and normalizes response tuple.
    def _wait_future(self, future, timeout_sec: float = 2.0) -> tuple[bool, str]:
        end = time.time() + timeout_sec
        while time.time() < end:
            if future.done():
                try:
                    result = future.result()
                except Exception as exc:  # noqa: BLE001
                    return False, str(exc)
                if hasattr(result, "success"):
                    return bool(result.success), str(result.message)
                return True, "ok"
            time.sleep(0.02)
        return False, "timeout"

    # Calls engage service from HTTP action.
    def set_engage(self, value: bool) -> Dict[str, Any]:
        if not self.cli_set_engage.wait_for_service(timeout_sec=0.2):
            return {"success": False, "message": f"service unavailable: {self.api_set_engage_service}"}
        req = SetBool.Request()
        req.data = bool(value)
        success, message = self._wait_future(self.cli_set_engage.call_async(req))
        return {"success": success, "message": message}

    # Calls operation-mode service from HTTP action.
    def set_operation_mode(self, value: bool) -> Dict[str, Any]:
        if not self.cli_set_operation_mode.wait_for_service(timeout_sec=0.2):
            return {
                "success": False,
                "message": f"service unavailable: {self.api_set_operation_mode_service}",
            }
        req = SetBool.Request()
        req.data = bool(value)
        success, message = self._wait_future(self.cli_set_operation_mode.call_async(req))
        return {"success": success, "message": message}

    # Calls AUTO shortcut service from HTTP action.
    def set_auto(self) -> Dict[str, Any]:
        if not self.cli_set_auto.wait_for_service(timeout_sec=0.2):
            return {"success": False, "message": f"service unavailable: {self.api_set_auto_service}"}
        success, message = self._wait_future(self.cli_set_auto.call_async(Trigger.Request()))
        return {"success": success, "message": message}

    # Calls STOP shortcut service from HTTP action.
    def set_stop(self) -> Dict[str, Any]:
        if not self.cli_set_stop.wait_for_service(timeout_sec=0.2):
            return {"success": False, "message": f"service unavailable: {self.api_set_stop_service}"}
        success, message = self._wait_future(self.cli_set_stop.call_async(Trigger.Request()))
        return {"success": success, "message": message}

    # Starts embedded HTTP server and binds request handlers.
    def _start_http_server(self) -> None:
        node = self

        # HH_260415: Reusable TCP server reduces bind failures after rapid restart.
        class ReusableThreadingHTTPServer(ThreadingHTTPServer):
            allow_reuse_address = True

        class Handler(BaseHTTPRequestHandler):
            # Redirects default HTTP access log into ROS debug logger.
            def log_message(self, fmt: str, *args) -> None:  # noqa: A003
                node.get_logger().debug("http: " + (fmt % args))

            # Writes JSON response with UTF-8 payload.
            def _send_json(self, code: int, payload: Dict[str, Any]) -> None:
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            # Serves static file from frontend directory.
            def _send_file(self, file_path: Path) -> None:
                if not file_path.exists() or not file_path.is_file():
                    self.send_error(404, "Not Found")
                    return
                mime, _ = mimetypes.guess_type(str(file_path))
                mime = mime or "application/octet-stream"
                data = file_path.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", mime)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            # Handles API state/health and static frontend GET requests.
            def do_GET(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                if parsed.path == "/api/state":
                    self._send_json(200, node._snapshot())
                    return
                if parsed.path == "/api/health":
                    self._send_json(200, {"ok": True, "node": node.get_name()})
                    return

                base_dir = node.frontend_dir if node.frontend_dir and node.frontend_dir.exists() else (Path(__file__).resolve().parent.parent / "web")
                rel_path = parsed.path.lstrip("/") or "index.html"
                rel_parts = Path(rel_path).parts
                # HH_260415: Reject traversal while allowing symlink-install frontend files.
                if any(part == ".." for part in rel_parts):
                    self.send_error(403, "Forbidden")
                    return
                candidate = base_dir / rel_path
                if not candidate.exists() and rel_path != "index.html":
                    candidate = base_dir / "index.html"
                self._send_file(candidate)

            # Handles API command POST requests from UI.
            def do_POST(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                query = parse_qs(parsed.query)

                if parsed.path == "/api/engage":
                    value_raw = query.get("value", ["false"])[0]
                    value = str(value_raw).lower() in {"1", "true", "yes", "on"}
                    result = node.set_engage(value)
                    self._send_json(200 if result.get("success") else 503, result)
                    return

                if parsed.path == "/api/operation_mode":
                    value_raw = query.get("auto", ["false"])[0]
                    value = str(value_raw).lower() in {"1", "true", "yes", "on"}
                    result = node.set_operation_mode(value)
                    self._send_json(200 if result.get("success") else 503, result)
                    return

                if parsed.path == "/api/auto":
                    result = node.set_auto()
                    self._send_json(200 if result.get("success") else 503, result)
                    return

                if parsed.path == "/api/stop":
                    result = node.set_stop()
                    self._send_json(200 if result.get("success") else 503, result)
                    return

                self.send_error(404, "Not Found")

        bind_error: Exception | None = None
        base_port = int(self.port)
        max_tries = max(1, int(self.port_search_max) + 1) if self.port_auto_increment else 1
        for offset in range(max_tries):
            try_port = base_port + offset
            try:
                self._server = ReusableThreadingHTTPServer((self.host, try_port), Handler)
                self.port = try_port
                bind_error = None
                break
            except OSError as exc:
                bind_error = exc
                if exc.errno == errno.EADDRINUSE and self.port_auto_increment and offset + 1 < max_tries:
                    self.get_logger().warn(
                        f"ui backend port busy: {self.host}:{try_port}, retrying next port"
                    )
                    continue
                break

        if self._server is None:
            message = (
                f"ui backend http bind failed on {self.host}:{base_port}"
                + (f" ({bind_error})" if bind_error else "")
            )
            if self.fail_on_bind_error:
                raise RuntimeError(message) from bind_error
            self.get_logger().error(message)
            self.get_logger().warn("ui backend will keep ROS bridge alive without embedded HTTP server")
            return

        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name="camrod_api_http",
            daemon=True,
        )
        self._server_thread.start()
        self.get_logger().info(f"ui backend http listening: http://{self.host}:{self.port}")

    # Stops embedded HTTP server if running.
    def _shutdown_http_server(self) -> None:
        if self._server is None:
            return
        try:
            self._server.shutdown()
            self._server.server_close()
        except Exception:  # noqa: BLE001
            pass
        self._server = None
        if self._server_thread is not None:
            self._server_thread.join(timeout=1.0)
            self._server_thread = None


# Spins the UI backend node.
def main() -> None:
    rclpy.init()
    node = UiBackendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
