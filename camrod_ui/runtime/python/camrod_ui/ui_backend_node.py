#!/usr/bin/env python3
# HH_260421: UI backend simplified to direct destination-driven engage/goal dispatch.

from __future__ import annotations

import errno
import json
import math
import mimetypes
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List
from urllib.parse import parse_qs, urlparse

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String

from camrod_ui.api_common import to_diag_level_int


@dataclass
class ApiState:
    """In-memory snapshot exposed by the HTTP API."""

    engaged: bool = False
    ready: bool = False
    operation_mode: str = "STOP"
    ready_message: str = ""
    module_states: List[Dict[str, Any]] = field(default_factory=list)
    diagnostics: List[Dict[str, Any]] = field(default_factory=list)
    diagnostics_agg_count: int = 0
    diagnostics_agg_error_count: int = 0
    destination: Dict[str, Any] = field(
        default_factory=lambda: {"site": "", "run": False}
    )


@dataclass
class GoalKeypoint:
    """Resolved keypoint for destination dispatch."""

    key: str
    frame_id: str
    x: float
    y: float
    z: float
    yaw_deg: float


class UiBackendNode(Node):
    """HTTP backend that bridges UI destination commands to planning topics."""

    # Initializes ROS interfaces and optional embedded HTTP server.
    def __init__(self) -> None:
        super().__init__("ui_backend")

        # HTTP server parameters.
        self.host = str(self.declare_parameter("host", "127.0.0.1").value)
        self.port = int(self.declare_parameter("port", 8010).value)
        self.enable_http_server = bool(self.declare_parameter("enable_http_server", True).value)
        # Avoid fatal crash when UI backend is relaunched while previous process still holds the port.
        self.port_auto_increment = bool(self.declare_parameter("port_auto_increment", True).value)
        self.port_search_max = int(self.declare_parameter("port_search_max", 20).value)
        self.fail_on_bind_error = bool(self.declare_parameter("fail_on_bind_error", False).value)
        self.frontend_dir = Path(str(self.declare_parameter("frontend_dir", "").value))
        self.web_fallback_dir = self._resolve_web_fallback_dir()

        # Topic and destination dispatch parameters.
        self.ui_destination_topic = str(
            self.declare_parameter("ui_destination_topic", "/ui/selected_destination").value
        )
        self.planning_engage_topic = str(
            self.declare_parameter("planning_engage_topic", "/planning/engage").value
        )
        self.planning_engaged_state_topic = str(
            self.declare_parameter("planning_engaged_state_topic", "/planning/engaged").value
        )
        self.planning_goal_key_topic = str(
            self.declare_parameter("planning_goal_key_topic", "/planning/state_machine/goal_key").value
        )
        self.planning_goal_pose_topic = str(
            self.declare_parameter("planning_goal_pose_topic", "/goal_pose").value
        )
        self.publish_goal_key = bool(self.declare_parameter("publish_goal_key", True).value)
        self.publish_goal_pose = bool(self.declare_parameter("publish_goal_pose", True).value)
        self.publish_engage_from_destination = bool(
            self.declare_parameter("publish_engage_from_destination", True).value
        )
        self.default_goal_frame_id = str(self.declare_parameter("default_goal_frame_id", "map").value)
        self.fallback_goal_key = str(self.declare_parameter("fallback_goal_key", "camping_site_1").value)
        self.fallback_to_first_known_goal = bool(
            self.declare_parameter("fallback_to_first_known_goal", True).value
        )

        self.camping_sites_yaml = str(self.declare_parameter("camping_sites_yaml", "").value)
        self.site_names = [
            str(site)
            for site in self.declare_parameter("site_names", [f"B{i}" for i in range(1, 14)]).value
        ]
        if not self.site_names:
            self.site_names = [f"B{i}" for i in range(1, 14)]

        raw_site_to_goal = self.declare_parameter("site_to_goal_key_map", []).value
        self.site_to_goal_key_map = self._parse_site_goal_map(raw_site_to_goal)

        self.diagnostics_agg_topic = str(
            self.declare_parameter("diagnostics_agg_topic", "/diagnostics_agg").value
        )

        self._keypoints_by_goal_key = self._load_camping_site_keypoints(self.camping_sites_yaml)
        self._lock = threading.Lock()
        self._state = ApiState()

        # Subscriptions.
        self.sub_engaged = self.create_subscription(
            Bool,
            self.planning_engaged_state_topic,
            self._on_engaged_state,
            10,
        )
        self.sub_destination = self.create_subscription(
            String,
            self.ui_destination_topic,
            self._on_destination_command,
            10,
        )
        self.sub_diagnostics_agg = self.create_subscription(
            DiagnosticArray,
            self.diagnostics_agg_topic,
            self._on_diagnostics_agg,
            10,
        )

        # Publishers.
        self.pub_destination = self.create_publisher(String, self.ui_destination_topic, 10)
        self.pub_engage = self.create_publisher(Bool, self.planning_engage_topic, 10)
        self.pub_goal_key = self.create_publisher(String, self.planning_goal_key_topic, 10)
        self.pub_goal_pose = self.create_publisher(PoseStamped, self.planning_goal_pose_topic, 10)

        self._server: ThreadingHTTPServer | None = None
        self._server_thread: threading.Thread | None = None
        if self.enable_http_server:
            self._start_http_server()

        self.get_logger().info(
            "ui_backend ready: "
            f"host={self.host} port={self.port} "
            f"frontend_dir={str(self.frontend_dir) if self.frontend_dir else '(builtin)'} "
            f"destination_topic={self.ui_destination_topic} "
            f"engage_topic={self.planning_engage_topic} "
            f"goal_key_topic={self.planning_goal_key_topic} "
            f"goal_pose_topic={self.planning_goal_pose_topic} "
            f"camping_sites_yaml={self.camping_sites_yaml if self.camping_sites_yaml else '(none)'}"
        )

    # Resolves fallback static-web directory when frontend build path is absent.
    def _resolve_web_fallback_dir(self) -> Path:
        try:
            package_share = Path(get_package_share_directory("camrod_ui"))
            packaged_web = package_share / "assets" / "web"
            if packaged_web.exists():
                return packaged_web
        except PackageNotFoundError:
            pass
        source_web = Path(__file__).resolve().parents[2] / "assets" / "web"
        return source_web

    # Parses optional site-to-goal-key remap parameter.
    def _parse_site_goal_map(self, raw_value: object) -> Dict[str, str]:
        parsed: Dict[str, str] = {}
        if not isinstance(raw_value, list):
            return parsed
        for item in raw_value:
            entry = str(item).strip()
            if not entry or ":" not in entry:
                continue
            site, goal_key = entry.split(":", maxsplit=1)
            site = site.strip()
            goal_key = goal_key.strip()
            if site and goal_key:
                parsed[site] = goal_key
        return parsed

    # Loads camping-site keypoints used to build goal_pose from site selection.
    def _load_camping_site_keypoints(self, yaml_path: str) -> Dict[str, GoalKeypoint]:
        keypoints: Dict[str, GoalKeypoint] = {}
        path = Path(yaml_path).expanduser() if yaml_path else None
        if path is None or not str(path):
            self.get_logger().warn("camping_sites_yaml is empty: goal_pose dispatch will use goal-key only")
            return keypoints
        if not path.exists():
            self.get_logger().warn(
                f"camping_sites_yaml not found: {str(path)} (goal_pose dispatch may be unavailable)"
            )
            return keypoints

        try:
            with path.open("r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"failed to read camping_sites_yaml {str(path)}: {exc}")
            return keypoints

        raw_sites = data.get("camping_sites", [])
        if not isinstance(raw_sites, list):
            self.get_logger().warn("camping_sites_yaml has no list key 'camping_sites'")
            return keypoints

        for idx, site in enumerate(raw_sites, start=1):
            if not isinstance(site, dict):
                continue
            key = str(site.get("type", "")).strip() or f"camping_site_{idx}"
            if key in keypoints:
                continue
            keypoints[key] = GoalKeypoint(
                key=key,
                frame_id=str(site.get("frame_id", self.default_goal_frame_id)).strip()
                or self.default_goal_frame_id,
                x=float(site.get("x", 0.0)),
                y=float(site.get("y", 0.0)),
                z=float(site.get("z", 0.0)),
                yaw_deg=float(site.get("yaw_deg", 0.0)),
            )

        self.get_logger().info(
            f"loaded {len(keypoints)} camping-site keypoints from {str(path)}"
        )
        return keypoints

    # Ensures HTTP server is terminated before ROS node shutdown.
    def destroy_node(self) -> bool:
        self._shutdown_http_server()
        return super().destroy_node()

    # Derives operation-mode text from engaged/ready state.
    def _compute_operation_mode(self, engaged: bool, ready: bool) -> str:
        if engaged and ready:
            return "AUTO"
        if engaged and not ready:
            return "WAITING_FOR_READY"
        return "STOP"

    # Extracts logical module name from a diagnostic status entry.
    def _extract_module_name(self, status: DiagnosticStatus) -> str:
        for kv in status.values:
            if kv.key == "category" and kv.value:
                return str(kv.value)
        if status.hardware_id:
            return str(status.hardware_id)
        if status.name:
            parts = [p for p in status.name.strip("/").split("/") if p]
            if parts:
                return parts[0]
        return "unknown"

    # Converts yaw in degrees to quaternion components.
    def _yaw_deg_to_quaternion(self, yaw_deg: float) -> tuple[float, float, float, float]:
        yaw_rad = math.radians(float(yaw_deg))
        half = yaw_rad * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    # Updates engage state cache from planning engaged-state topic.
    def _on_engaged_state(self, msg: Bool) -> None:
        with self._lock:
            self._state.engaged = bool(msg.data)
            self._state.operation_mode = self._compute_operation_mode(
                self._state.engaged,
                self._state.ready,
            )

    # Updates diagnostics cache and derives ready/module states.
    def _on_diagnostics_agg(self, msg: DiagnosticArray) -> None:
        diagnostics: List[Dict[str, Any]] = []
        module_levels: Dict[str, int] = {}
        module_messages: Dict[str, str] = {}

        error_count = 0
        error_level = to_diag_level_int(DiagnosticStatus.ERROR)

        for status in msg.status:
            level = to_diag_level_int(status.level)
            module = self._extract_module_name(status)
            prev = module_levels.get(module, -1)
            if level >= prev:
                module_levels[module] = level
                module_messages[module] = status.message

            if level >= error_level:
                error_count += 1

            diagnostics.append(
                {
                    "name": status.name,
                    "module": module,
                    "level": level,
                    "message": status.message,
                    "values": [{"key": kv.key, "value": kv.value} for kv in status.values],
                }
            )

        module_states = [
            {
                "name": module,
                "level": module_levels[module],
                "message": module_messages.get(module, ""),
            }
            for module in sorted(module_levels.keys())
        ]

        ready = bool(msg.status) and (error_count == 0)
        if not msg.status:
            ready_message = "no diagnostics yet"
        elif error_count > 0:
            ready_message = f"diagnostics errors: {error_count}"
        else:
            ready_message = "ready"

        with self._lock:
            self._state.diagnostics = diagnostics
            self._state.module_states = module_states
            self._state.diagnostics_agg_count = len(msg.status)
            self._state.diagnostics_agg_error_count = error_count
            self._state.ready = ready
            self._state.ready_message = ready_message
            self._state.operation_mode = self._compute_operation_mode(
                self._state.engaged,
                self._state.ready,
            )

    # Resolves goal key for a selected UI site.
    def _resolve_goal_key_for_site(self, site: str) -> str | None:
        if site in self.site_to_goal_key_map:
            mapped = self.site_to_goal_key_map[site]
            if mapped in self._keypoints_by_goal_key:
                return mapped

        site_text = str(site).strip().upper()
        if site_text.startswith("B") and site_text[1:].isdigit():
            candidate = f"camping_site_{int(site_text[1:])}"
            if candidate in self._keypoints_by_goal_key:
                return candidate

        if self.fallback_goal_key in self._keypoints_by_goal_key:
            self.get_logger().warn(
                f"no exact goal key for site={site}; fallback to {self.fallback_goal_key}"
            )
            return self.fallback_goal_key

        if self.fallback_to_first_known_goal and self._keypoints_by_goal_key:
            fallback = sorted(self._keypoints_by_goal_key.keys())[0]
            self.get_logger().warn(
                f"no exact goal key for site={site}; fallback to first known key={fallback}"
            )
            return fallback

        # If no keypoint is available, still allow key-name-only dispatch for state-machine users.
        if site_text.startswith("B") and site_text[1:].isdigit():
            return f"camping_site_{int(site_text[1:])}"
        return None

    # Publishes direct engage command.
    def _publish_engage(self, enabled: bool, source: str) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.pub_engage.publish(msg)

        with self._lock:
            # Optimistic UI state update; topic feedback from /planning/engaged will refresh this.
            self._state.engaged = bool(enabled)
            self._state.operation_mode = self._compute_operation_mode(
                self._state.engaged,
                self._state.ready,
            )

        self.get_logger().info(
            f"engage command ({source}) -> {self.planning_engage_topic}: "
            f"{str(bool(enabled)).lower()}"
        )

    # Publishes goal key and optional goal pose derived from keypoint YAML.
    def _publish_goal_for_site(self, site: str, source: str) -> Dict[str, Any]:
        goal_key = self._resolve_goal_key_for_site(site)
        if not goal_key:
            return {
                "goal_key": "",
                "goal_pose_published": False,
                "message": f"no goal key resolved for site={site}",
            }

        if self.publish_goal_key:
            key_msg = String()
            key_msg.data = goal_key
            self.pub_goal_key.publish(key_msg)
            self.get_logger().info(
                f"goal key ({source}) -> {self.planning_goal_key_topic}: {goal_key}"
            )

        pose_published = False
        goal = self._keypoints_by_goal_key.get(goal_key)
        if self.publish_goal_pose and goal is not None:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = goal.frame_id
            pose.pose.position.x = goal.x
            pose.pose.position.y = goal.y
            pose.pose.position.z = goal.z
            qx, qy, qz, qw = self._yaw_deg_to_quaternion(goal.yaw_deg)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self.pub_goal_pose.publish(pose)
            pose_published = True
            self.get_logger().info(
                f"goal pose ({source}) -> {self.planning_goal_pose_topic}: "
                f"key={goal_key} site={site} xy=({goal.x:.2f},{goal.y:.2f})"
            )

        if self.publish_goal_pose and goal is None:
            self.get_logger().warn(
                f"goal pose dispatch skipped: keypoint for '{goal_key}' is missing in camping_sites_yaml"
            )

        return {
            "goal_key": goal_key,
            "goal_pose_published": pose_published,
            "message": "ok",
        }

    # Parses destination JSON payload from /ui/selected_destination.
    def _parse_destination_payload(self, raw: str) -> tuple[str, bool] | None:
        text = str(raw).strip()
        if not text:
            return None
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            self.get_logger().warn(f"invalid destination JSON: {text}")
            return None

        if not isinstance(payload, dict):
            self.get_logger().warn(f"destination payload must be JSON object: {text}")
            return None

        site = str(payload.get("site", "")).strip()
        run = bool(payload.get("run", False))
        if not site:
            with self._lock:
                site = str(self._state.destination.get("site", "")).strip()

        if not site:
            self.get_logger().warn("destination payload has empty site")
            return None

        return (site, run)

    # Applies destination command semantics: engage + goal dispatch.
    def _apply_destination_command(self, site: str, run: bool, source: str) -> Dict[str, Any]:
        if self.publish_engage_from_destination:
            self._publish_engage(run, source=f"{source}:destination")

        # Only dispatch target goals when run=true.
        if not run:
            return {
                "site": site,
                "run": False,
                "goal_key": "",
                "goal_pose_published": False,
                "message": "run=false -> engage off, goal dispatch skipped",
            }

        goal_result = self._publish_goal_for_site(site=site, source=source)
        return {
            "site": site,
            "run": True,
            "goal_key": goal_result.get("goal_key", ""),
            "goal_pose_published": bool(goal_result.get("goal_pose_published", False)),
            "message": str(goal_result.get("message", "ok")),
        }

    # Handles /ui/selected_destination messages and relays to planning topics.
    def _on_destination_command(self, msg: String) -> None:
        parsed = self._parse_destination_payload(msg.data)
        if parsed is None:
            return
        site, run = parsed
        result = self._apply_destination_command(site=site, run=run, source="destination_topic")
        self.get_logger().info(
            "destination dispatch summary: "
            f"site={site} run={str(run).lower()} "
            f"goal_key={result.get('goal_key', '')} "
            f"goal_pose={str(bool(result.get('goal_pose_published', False))).lower()}"
        )

    # Publishes the unified destination selection topic with site and run state only.
    def _publish_destination_command(self, site: str, run: bool, source: str) -> Dict[str, Any]:
        payload = {"site": site, "run": bool(run)}

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_destination.publish(msg)

        with self._lock:
            self._state.destination = dict(payload)

        self.get_logger().info(
            f"destination command ({source}) -> {self.ui_destination_topic}: "
            f"site={site}, run={str(bool(run)).lower()}"
        )
        return payload

    # Returns an atomic copy of current API/UI state.
    def _snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "engaged": self._state.engaged,
                "ready": self._state.ready,
                "operation_mode": self._state.operation_mode,
                "ready_message": self._state.ready_message,
                "module_states": list(self._state.module_states),
                "diagnostics": list(self._state.diagnostics),
                "diagnostics_agg_count": self._state.diagnostics_agg_count,
                "diagnostics_agg_error_count": self._state.diagnostics_agg_error_count,
                "destination": dict(self._state.destination),
            }

    # Publishes engage command from HTTP action.
    def set_engage(self, value: bool) -> Dict[str, Any]:
        self._publish_engage(bool(value), source="http")
        return {"success": True, "message": "engage command published", "value": bool(value)}

    # Publishes operation-mode command from HTTP action.
    def set_operation_mode(self, value: bool) -> Dict[str, Any]:
        # In this stack, operation-mode bool is equivalent to engage command.
        self._publish_engage(bool(value), source="http_operation_mode")
        return {
            "success": True,
            "message": "operation_mode command forwarded as engage",
            "auto": bool(value),
        }

    # Publishes AUTO shortcut from HTTP action.
    def set_auto(self) -> Dict[str, Any]:
        self._publish_engage(True, source="http_auto")
        return {"success": True, "message": "auto command published"}

    # Publishes STOP shortcut from HTTP action.
    def set_stop(self) -> Dict[str, Any]:
        self._publish_engage(False, source="http_stop")
        return {"success": True, "message": "stop command published"}

    # Publishes destination selection from UI as a single site/run message.
    def set_destination(self, site: str, run: bool) -> Dict[str, Any]:
        normalized_site = str(site).strip()
        if not normalized_site:
            return {"success": False, "message": "site is required"}
        if normalized_site not in self.site_names:
            return {
                "success": False,
                "message": f"unknown site: {normalized_site}",
                "valid_sites": list(self.site_names),
            }

        payload = self._publish_destination_command(
            site=normalized_site,
            run=bool(run),
            source="http_ui_destination",
        )
        return {"success": True, "destination": payload}

    # Starts embedded HTTP server and binds request handlers.
    def _start_http_server(self) -> None:
        node = self

        # Reusable TCP server reduces bind failures after rapid restart.
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

            # Handles UI API state/health endpoints and static frontend GET requests.
            def do_GET(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                if parsed.path == "/ui/state":
                    self._send_json(200, node._snapshot())
                    return
                if parsed.path == "/ui/health":
                    self._send_json(200, {"ok": True, "node": node.get_name()})
                    return
                if parsed.path == "/ui/destination":
                    snapshot = node._snapshot()
                    self._send_json(
                        200,
                        {
                            "destination": snapshot.get("destination", {"site": "", "run": False}),
                            "valid_sites": list(node.site_names),
                        },
                    )
                    return
                if parsed.path == "/ui/diagnostics":
                    snapshot = node._snapshot()
                    self._send_json(
                        200,
                        {"status": snapshot.get("diagnostics", [])},
                    )
                    return

                base_dir = (
                    node.frontend_dir
                    if node.frontend_dir and node.frontend_dir.exists()
                    else node.web_fallback_dir
                )
                rel_path = parsed.path.lstrip("/") or "index.html"
                rel_parts = Path(rel_path).parts
                # Reject traversal while allowing symlink-install frontend files.
                if any(part == ".." for part in rel_parts):
                    self.send_error(403, "Forbidden")
                    return
                candidate = base_dir / rel_path
                if not candidate.exists() and rel_path != "index.html":
                    candidate = base_dir / "index.html"
                self._send_file(candidate)

            # Handles UI command POST requests.
            def do_POST(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                query = parse_qs(parsed.query)

                if parsed.path == "/ui/engage":
                    value_raw = query.get("value", ["false"])[0]
                    value = str(value_raw).lower() in {"1", "true", "yes", "on"}
                    result = node.set_engage(value)
                    self._send_json(200 if result.get("success") else 503, result)
                    return

                if parsed.path == "/ui/operation_mode":
                    value_raw = query.get("auto", ["false"])[0]
                    value = str(value_raw).lower() in {"1", "true", "yes", "on"}
                    result = node.set_operation_mode(value)
                    self._send_json(200 if result.get("success") else 503, result)
                    return

                if parsed.path == "/ui/auto":
                    result = node.set_auto()
                    self._send_json(200 if result.get("success") else 503, result)
                    return

                if parsed.path == "/ui/stop":
                    result = node.set_stop()
                    self._send_json(200 if result.get("success") else 503, result)
                    return

                if parsed.path == "/ui/destination":
                    site = str(query.get("site", [""])[0]).strip()
                    run_raw = query.get("run", query.get("value", ["false"]))[0]
                    run = str(run_raw).lower() in {"1", "true", "yes", "on"}
                    result = node.set_destination(site=site, run=run)
                    self._send_json(200 if result.get("success") else 400, result)
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
            name="camrod_ui_http",
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
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
