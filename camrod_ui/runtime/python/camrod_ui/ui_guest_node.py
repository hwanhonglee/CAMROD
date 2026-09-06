#!/usr/bin/env python3
# HJ_260601: Guest UI backend for WiFi-accessible robot recall service.
#            Single-user WebSocket lock, GUEST_RECALL_SERVICE(4) publish on recall.
# Site-specific recall:
#            mobile B1~B13 selection -> typed /ui/selected_destination ->
#            ui_backend_node publishes PlanningRecallRequest for the roadside
#            wait pose. usage_complete uses the shared typed RETURN operation.
"""Guest UI backend: mobile/laptop access for robot recall service.

Binds to 0.0.0.0 so devices on the same WiFi can reach it.
Only one WebSocket client is allowed at a time (exclusive lock).
"""

from __future__ import annotations

import asyncio
import copy
import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
import uvicorn
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from avg_msgs.msg import (
    AvgPlatformStatus,
    AvgServiceState,
    ModuleState,
    MotionOperation,
    UiDestinationCommand,
)
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def normalize_platform_battery_percent(value: float) -> int:
    """Convert canonical AvgPlatformStatus SOC ratio to a display percent."""
    if not math.isfinite(float(value)):
        return -1
    return max(0, min(100, int(round(float(value) * 100.0))))


def destination_request_owner(source: str) -> str:
    """Classify which UI is allowed to operate an accepted destination."""
    normalized = str(source).strip().lower()
    if normalized.startswith("guest"):
        return "guest"
    if normalized.startswith("robot_ui:"):
        return "robot"
    return "operator" if normalized else ""


def guest_mission_dispatch_ready(
    state: int,
    battery: int,
    minimum: int,
    active_request_intent: str = "",
) -> bool:
    """Apply stationary/SOC admission and reject an already reserved mission."""
    return state in {
        int(AvgServiceState.DROP_ZONE_WAIT),
        # Parking has already completed before this healthy charger-contact
        # wait.  A missing/intermittent CAN charge signal must not prevent a
        # sufficiently charged robot from starting the station-exit sequence.
        int(AvgServiceState.WAITING_FOR_CHARGING),
        int(AvgServiceState.CHARGING),
    } and battery >= minimum and not str(active_request_intent).strip()


def guest_mission_cancel_available(
    state: int,
    request_intent: str,
    active_site: str,
    request_owner: str,
) -> bool:
    """Allow public cancellation only for the Guest UI's own active recall."""
    return (
        str(request_intent).strip() == "recall"
        and str(request_owner).strip() == "guest"
        and bool(str(active_site).strip())
        and int(state) in {
            int(AvgServiceState.MOVING_TO_SITE),
            int(AvgServiceState.RETURNING_TO_DROP_ZONE),
            int(AvgServiceState.GUEST_RECALL_SERVICE),
            int(AvgServiceState.SITE_ENTRY),
            int(AvgServiceState.RECALL_TO_SITE_ROAD),
            int(AvgServiceState.RETURN_WITH_CARGO),
            int(AvgServiceState.DROP_ZONE_PARKING),
            int(AvgServiceState.DEPARTING_CHARGER),
            int(AvgServiceState.DEPARTING_DROP_ZONE),
        }
    )


def guest_usage_complete_available(
    state: int,
    request_intent: str,
    active_site: str,
    request_owner: str,
) -> bool:
    """Allow return only after the active Guest recall reached its wait pose."""
    return (
        int(state) == int(AvgServiceState.GUEST_LOADING_WAIT)
        and str(request_intent).strip() == "recall"
        and str(request_owner).strip() == "guest"
        and bool(str(active_site).strip())
    )


def guest_gate_safety_hold(operating_state: str, message: str = "") -> bool:
    """Recognize the same command-gate hold states shown by the Robot UI."""
    state = str(operating_state).strip().upper()
    detail = str(message).strip().lower()
    return (
        state in {"SAFETY_HOLD", "ROUTE_SAFETY_HOLD"}
        or "cost_stop_latched" in detail
        or "cost_hold=" in detail
        or "route_safety_hold=" in detail
    )


class UiGuestNode(Node):
    """Lightweight FastAPI node serving the guest recall UI over WiFi."""

    def __init__(self) -> None:
        super().__init__("ui_guest")

        self.host = str(self.declare_parameter("host", "0.0.0.0").value)
        self.port = int(self.declare_parameter("port", 8012).value)
        self.service_state_topic = str(
            self.declare_parameter("service_state_topic", "/service/state").value
        )
        self.battery_topic = str(
            self.declare_parameter("battery_topic", "/platform/status").value
        )
        self.control_gate_status_topic = str(
            self.declare_parameter(
                "control_gate_status_topic",
                "/control/cmd_vel_safety_gate/status",
            ).value
        )
        self.grace_period_s = int(self.declare_parameter("grace_period_s", 60).value)
        self.dispatch_ack_timeout_s = max(
            1.0,
            float(
                self.declare_parameter("dispatch_ack_timeout_s", 5.0).value
            ),
        )
        self.minimum_mission_dispatch_battery_percent = max(
            0,
            min(
                100,
                int(
                    round(
                        float(
                            self.declare_parameter(
                                "minimum_mission_dispatch_battery_percent", 35.0
                            ).value
                        )
                    )
                ),
            ),
        )

        # HH_260721 - Use the same destination command contract as ui_backend_node.
        self.ui_destination_topic = str(
            self.declare_parameter("ui_destination_topic", "/ui/selected_destination").value
        )
        self.ui_destination_dispatch_status_topic = str(
            self.declare_parameter(
                "ui_destination_dispatch_status_topic",
                "/ui/destination_dispatch_status",
            ).value
        )
        self.ui_camping_site_operation_request_topic = str(
            self.declare_parameter(
                "ui_camping_site_operation_request_topic",
                "/ui/camping_site_operation_request",
            ).value
        )
        self.site_names = [
            str(s)
            for s in self.declare_parameter("site_names", [f"B{i}" for i in range(1, 14)]).value
        ]
        if not self.site_names:
            self.site_names = [f"B{i}" for i in range(1, 14)]

        self._lock = threading.Lock()
        self._service_state: int = AvgServiceState.DROP_ZONE_WAIT
        self._active_site: str = ""
        # Persist the accepted UiDestinationCommand source classification so
        # a refreshed/reconnected browser does not guess recall from a shared
        # service state such as RETURN_WITH_CARGO.
        self._active_request_intent: str = ""
        # Preserve who accepted the recall. The public Guest UI may observe a
        # Robot UI recall, but it must not cancel or return that operator-owned
        # mission.
        self._active_request_owner: str = ""
        # Backend-issued generation binds CANCEL/RETURN to the exact accepted
        # Guest mission. A delayed operation from a previous B-site (or a
        # previous visit to the same B-site) cannot affect its successor.
        self._active_mission_generation: int = 0
        self._dispatch_request_sequence: int = 0
        self._pending_dispatch_nonce: str = ""
        self._pending_dispatch_deadline_s: float = 0.0
        self._active_request_retryable: bool = False
        self._mission_terminal_clear_armed: bool = False
        # Guard repeated browser clicks while the service-state transition from
        # GUEST_LOADING_WAIT to RETURN_WITH_CARGO is still in flight.
        self._guest_return_request_pending: bool = False
        self._battery: int = -1
        self._safety_hold: bool = False
        self._control_gate_state: str = "UNKNOWN"
        # HH_260721 - Preserve the physical charging state when a destination is cleared.
        self._is_charging: bool = False

        # Single-client WebSocket exclusive lock.
        self._guest_ws: Optional[WebSocket] = None
        self._guest_ws_initializing: Optional[WebSocket] = None
        self._guest_ws_initializing_queue: list[dict] = []
        self._guest_ws_lock = threading.Lock()
        # HJ_260804 - ROS callbacks and request responses can schedule writes
        # concurrently. Serialize frames on the uvicorn event loop so one
        # connection never receives overlapping send_json() calls.
        self._guest_ws_send_lock = asyncio.Lock()
        # Every identity mutation carries a monotonic revision. The browser
        # can reject a timeout frame that was scheduled after, but represents
        # state older than, a concurrent backend admission ACK.
        self._dispatch_identity_revision: int = 0
        self._main_loop: Optional[asyncio.AbstractEventLoop] = None
        self._uvicorn_server: Optional[uvicorn.Server] = None
        self._server_thread: Optional[threading.Thread] = None
        self._server_stop_requested = threading.Event()

        # Grace period: hold lock briefly after disconnect so same user can reconnect.
        self._last_client_ip: Optional[str] = None
        self._grace_until: float = 0.0

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Do not subscribe to raw controller /service/state heartbeats here.
        # ui_backend_node first validates mission generation, return token, and
        # physical arrival, then mirrors the accepted lifecycle in the
        # transient-local destination-dispatch stream below. Consuming both
        # would let a stale raw arrival expose a false Guest Return button.
        self.sub_service_state = None
        self.sub_battery = self.create_subscription(
            AvgPlatformStatus,
            self.battery_topic,
            self._on_battery,
            10,
        )
        self.sub_destination = self.create_subscription(
            UiDestinationCommand,
            self.ui_destination_topic,
            self._on_destination_command,
            10,
        )
        self.sub_destination_dispatch_status = self.create_subscription(
            String,
            self.ui_destination_dispatch_status_topic,
            self._on_destination_dispatch_status,
            state_qos,
        )
        self.sub_control_gate_status = self.create_subscription(
            ModuleState,
            self.control_gate_status_topic,
            self._on_control_gate_status,
            state_qos,
        )

        self.pub_service_state = self.create_publisher(
            AvgServiceState, self.service_state_topic, 10
        )
        # The guest source marker makes this a roadside recall; the backend
        # must never translate it into an ordinary campsite-entry goal.
        self.pub_destination = self.create_publisher(
            UiDestinationCommand,
            self.ui_destination_topic,
            10,
        )
        self.pub_operation_request = self.create_publisher(
            MotionOperation,
            self.ui_camping_site_operation_request_topic,
            10,
        )

        # A request topic is volatile and the backend can be restarting.  This
        # bounded watchdog releases only a local generation-0 reservation when
        # no authoritative dispatch ACK arrives; accepted missions (generation
        # > 0) are never timed out here.
        self._dispatch_ack_watchdog = self.create_timer(
            min(0.5, self.dispatch_ack_timeout_s),
            self._expire_pending_dispatch_ack,
        )

        self._start_fastapi_server()

        self.get_logger().info(
            f"ui_guest ready: host={self.host} port={self.port} "
            f"destination_topic={self.ui_destination_topic}"
        )

    def destroy_node(self) -> bool:
        # HH_260805 - Drain the Guest HTTP/WebSocket server before ROS entities
        # disappear so shutdown cannot race an in-flight transport callback.
        self._stop_fastapi_server()
        return super().destroy_node()

    def _stop_fastapi_server(self, join_timeout_s: float = 3.0) -> None:
        self._server_stop_requested.set()
        server = self._uvicorn_server
        if server is not None:
            server.should_exit = True

        loop = self._main_loop
        if loop is not None and loop.is_running():
            try:
                loop.call_soon_threadsafe(lambda: None)
            except RuntimeError:
                pass

        thread = self._server_thread
        if (
            thread is not None
            and thread is not threading.current_thread()
            and thread.is_alive()
        ):
            thread.join(timeout=max(0.0, float(join_timeout_s)))
            if thread.is_alive():
                self.get_logger().warn(
                    "guest ui fastapi thread did not stop before timeout"
                )

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def _on_service_state(self, msg: AvgServiceState) -> None:
        state = int(msg.state)
        with self._lock:
            previous_state = self._service_state
            self._service_state = state
            # Service state is lifecycle telemetry, not mission-identity
            # authority. Clearing site/owner here can race a newly admitted
            # mission from the backend's separate DataWriter. The backend's
            # dispatch-status snapshot is the only authority for identity.
            if (
                previous_state == int(AvgServiceState.GUEST_LOADING_WAIT)
                and state != previous_state
            ):
                self._guest_return_request_pending = False
            active_site = self._active_site
            request_intent = self._active_request_intent
            request_owner = getattr(self, "_active_request_owner", "")
            identity_revision = int(
                getattr(self, "_dispatch_identity_revision", 0)
            )
        phase = str(msg.state_name).strip() or self._state_name_of(state)
        # HH_260721 - Forward both numeric and symbolic service state to guest clients.
        self._schedule_broadcast({
            "service_state": state,
            "service_state_name": phase,
            "phase": self._phase_of(state),
            "site": active_site,
            "request_intent": request_intent,
            "request_owner": request_owner,
            "identity_revision": identity_revision,
        })
        self.get_logger().info(f"[guest] Service state received: {phase}({state})")

    def _on_battery(self, msg: AvgPlatformStatus) -> None:
        # HH_260721 - Track charging even when battery percentage is unavailable.
        with self._lock:
            self._is_charging = bool(msg.is_charging)
        if not msg.battery_state_available:
            return
        # HH_260803 - AvgPlatformStatus carries SOC as a [0, 1] fraction. Match
        # the Robot UI conversion so 0.80 is shown and gated as 80%, not 1%.
        pct = normalize_platform_battery_percent(msg.battery_percentage)
        if pct < 0:
            return
        with self._lock:
            self._battery = pct
        self._schedule_broadcast({
            "battery": pct,
            "minimum_battery_percentage": self.minimum_mission_dispatch_battery_percent,
            "mission_battery_ready": pct >= self.minimum_mission_dispatch_battery_percent,
        })

    def _on_destination_command(self, msg: UiDestinationCommand) -> None:
        # The command topic is a request, not admission proof. The backend may
        # reject it because another UI already owns station departure; only
        # _on_destination_dispatch_status is authoritative for site/owner.
        self.get_logger().info(
            "[guest] destination request observed; awaiting backend admission: "
            f"site={str(msg.site).strip()} run={bool(msg.run)} "
            f"source={str(msg.source).strip()}"
        )

    def _on_destination_dispatch_status(self, msg: String) -> None:
        """Apply only the backend-authoritative destination owner and site."""
        try:
            payload = json.loads(str(msg.data))
        except (json.JSONDecodeError, TypeError, ValueError):
            self.get_logger().warn("[guest] invalid destination dispatch status")
            return
        if not isinstance(payload, dict):
            self.get_logger().warn("[guest] destination dispatch status is not an object")
            return

        accepted = bool(payload.get("accepted", False))
        active_site = str(payload.get("active_site", "")).strip()
        active_source = str(payload.get("active_source", "")).strip()
        active_intent = str(payload.get("active_intent", "")).strip()
        retryable = bool(payload.get("retryable", False))
        try:
            active_generation = int(payload.get("active_generation", 0))
        except (TypeError, ValueError):
            active_generation = 0
        has_service_state = "service_state" in payload
        try:
            authoritative_service_state = (
                int(payload["service_state"])
                if has_service_state
                else int(getattr(self, "_service_state", -1))
            )
        except (TypeError, ValueError):
            self.get_logger().warn(
                "[guest] dispatch status has invalid service state"
            )
            return
        authoritative_service_name = ""
        if has_service_state:
            authoritative_service_name = str(
                payload.get("service_state_name", "")
            ).strip() or UiGuestNode._state_name_of(
                self, authoritative_service_state
            )
        if active_site and active_site not in self.site_names:
            self.get_logger().warn(
                f"[guest] dispatch status has invalid active site: {active_site}"
            )
            return
        if active_intent not in {"", "delivery", "recall"}:
            self.get_logger().warn(
                f"[guest] dispatch status has invalid intent: {active_intent}"
            )
            return
        request_owner = destination_request_owner(active_source)
        request_source = str(payload.get("request_source", "")).strip()
        with self._lock:
            pending_nonce = str(
                getattr(self, "_pending_dispatch_nonce", "")
            ).strip()
            request_nonce = ""
            for component in request_source.split(":"):
                if component.startswith("r="):
                    request_nonce = component.split("=", 1)[1].strip()
            # A local generation-0 reservation is intentionally stronger than
            # an older Guest operation/lifecycle frame.  usage_complete and
            # cancel statuses predate the dispatch nonce, so merely checking a
            # *present* mismatching nonce allowed their late arrival to erase a
            # freshly selected B-site.  During this short ACK window accept
            # only the exact Guest dispatch nonce, or an explicit positive
            # generation claimed by another UI (which is newer authority).
            local_generation = int(
                getattr(self, "_active_mission_generation", 0)
            )
            status_owner = destination_request_owner(request_source)
            active_owner = destination_request_owner(active_source)
            correlated_guest_ack = bool(
                status_owner == "guest" and request_nonce == pending_nonce
            )
            newer_cross_owner_authority = bool(
                active_site
                and active_generation > 0
                and active_owner in {"operator", "robot"}
            )
            if (
                pending_nonce
                and local_generation == 0
                and not correlated_guest_ack
                and not newer_cross_owner_authority
            ):
                self.get_logger().warn(
                    "[guest] stale/unrelated dispatch status ignored during "
                    "pending reservation: "
                    f"owner={status_owner or 'none'} request_nonce="
                    f"{request_nonce or 'none'} pending={pending_nonce}"
                )
                return
            previous_identity = (
                self._active_site,
                self._active_request_intent,
                getattr(self, "_active_request_owner", ""),
                int(getattr(self, "_active_mission_generation", 0)),
            )
            previous_service_state = int(
                getattr(self, "_service_state", -1)
            )
            if has_service_state:
                self._service_state = authoritative_service_state
                if (
                    previous_service_state
                    == int(AvgServiceState.GUEST_LOADING_WAIT)
                    and authoritative_service_state
                    != previous_service_state
                ):
                    self._guest_return_request_pending = False
            self._active_site = active_site
            self._active_request_intent = active_intent if active_site else ""
            self._active_request_owner = request_owner if active_site else ""
            self._active_mission_generation = (
                active_generation if active_site else 0
            )
            self._active_request_retryable = bool(active_site and retryable)
            # Any current authoritative status resolves a generation-0 local
            # reservation. For Guest responses, a nonce mismatch was rejected
            # above; operator/Robot ownership is independently authoritative.
            self._pending_dispatch_nonce = ""
            self._pending_dispatch_deadline_s = 0.0
            next_identity = (
                self._active_site,
                self._active_request_intent,
                self._active_request_owner,
                self._active_mission_generation,
            )
            if next_identity != previous_identity:
                self._mission_terminal_clear_armed = False
                self._guest_return_request_pending = False
            elif (
                not accepted
                and destination_request_owner(request_source) == "guest"
            ):
                # A correlated Guest operation rejection must release its
                # one-shot UI latch. Unrelated operator/Robot ACKs preserve it.
                self._guest_return_request_pending = False
            request_intent = self._active_request_intent
            request_owner = self._active_request_owner
            mission_retryable = self._active_request_retryable
            self._dispatch_identity_revision = int(
                getattr(self, "_dispatch_identity_revision", 0)
            ) + 1
            identity_revision = self._dispatch_identity_revision

        broadcast = {
            "site": active_site,
            "request_intent": request_intent,
            "request_owner": request_owner,
            "mission_generation": active_generation if active_site else 0,
            "dispatch_accepted": accepted,
            "dispatch_error": str(payload.get("error", "")),
            "dispatch_request_site": str(payload.get("request_site", "")).strip(),
            "mission_retryable": mission_retryable,
            "identity_revision": identity_revision,
        }
        if has_service_state:
            broadcast.update({
                "service_state": authoritative_service_state,
                "service_state_name": authoritative_service_name,
                "phase": UiGuestNode._phase_of(
                    self, authoritative_service_state
                ),
            })
        self._schedule_broadcast(broadcast)

    def _expire_pending_dispatch_ack(self) -> None:
        """Release a Guest picker reservation if the backend never ACKed it."""
        expired_site = ""
        expired_nonce = ""
        with self._lock:
            deadline = float(
                getattr(self, "_pending_dispatch_deadline_s", 0.0)
            )
            generation = int(
                getattr(self, "_active_mission_generation", 0)
            )
            if (
                not getattr(self, "_pending_dispatch_nonce", "")
                or deadline <= 0.0
                or time.monotonic() < deadline
                or generation > 0
            ):
                return
            expired_site = str(self._active_site).strip()
            expired_nonce = str(self._pending_dispatch_nonce).strip()
            self._active_site = ""
            self._active_request_intent = ""
            self._active_request_owner = ""
            self._active_mission_generation = 0
            self._active_request_retryable = False
            self._mission_terminal_clear_armed = False
            self._guest_return_request_pending = False
            self._pending_dispatch_nonce = ""
            self._pending_dispatch_deadline_s = 0.0
            self._dispatch_identity_revision = int(
                getattr(self, "_dispatch_identity_revision", 0)
            ) + 1
            identity_revision = self._dispatch_identity_revision
        self.get_logger().warn(
            "[guest] destination ACK timeout; local reservation released: "
            f"site={expired_site} nonce={expired_nonce}"
        )
        self._schedule_broadcast({
            "error": "dispatch_ack_timeout",
            "site": "",
            "request_intent": "",
            "request_owner": "",
            "mission_generation": 0,
            "mission_retryable": False,
            "identity_revision": identity_revision,
            "message": "Robot controller did not acknowledge the request; retry",
        })

    def _on_control_gate_status(self, msg: ModuleState) -> None:
        # HH_260803 - Safety hold is an overlay on the service lifecycle. A
        # diagnostic WARN alone must not replace MOVING, ENTERING, or RETURNING.
        gate_state = str(msg.operating_state).strip() or "UNKNOWN"
        safety_hold = guest_gate_safety_hold(gate_state, msg.message)
        with self._lock:
            self._control_gate_state = gate_state
            self._safety_hold = safety_hold
        self._schedule_broadcast({
            "safety_hold": safety_hold,
            "control_gate_state": gate_state,
            "control_gate_message": str(msg.message),
        })

    # ── WebSocket broadcast ───────────────────────────────────────────────────

    def _schedule_broadcast(self, payload: dict) -> None:
        if self._main_loop is not None:
            asyncio.run_coroutine_threadsafe(
                self._broadcast(payload), self._main_loop
            )

    async def _broadcast(self, payload: dict) -> None:
        with self._guest_ws_lock:
            if self._guest_ws_initializing is not None:
                self._guest_ws_initializing_queue.append(
                    copy.deepcopy(payload)
                )
                return
            ws = self._guest_ws
        if ws is None:
            return
        try:
            await self._send_guest_payload(ws, payload)
        except Exception:
            with self._guest_ws_lock:
                if self._guest_ws is ws:
                    self._guest_ws = None

    async def _send_guest_payload(self, ws: WebSocket, payload: dict) -> None:
        """Write one complete JSON frame without overlapping another sender."""
        async with self._guest_ws_send_lock:
            await ws.send_json(payload)

    def _claim_guest_connection(
        self,
        ws: WebSocket,
        client_ip: str,
        now_s: Optional[float] = None,
    ) -> Optional[int]:
        """Claim the single guest slot without awaiting under a thread lock."""
        now = time.time() if now_s is None else float(now_s)
        with self._guest_ws_lock:
            if (
                self._guest_ws is not None
                or getattr(self, "_guest_ws_initializing", None) is not None
            ):
                return 0
            if now < self._grace_until and client_ip != self._last_client_ip:
                return max(1, math.ceil(self._grace_until - now))
            self._guest_ws_initializing = ws
            self._guest_ws_initializing_queue = []
            self._last_client_ip = client_ip
            self._grace_until = 0.0
        return None

    def _release_guest_connection(
        self,
        ws: WebSocket,
        now_s: Optional[float] = None,
    ) -> None:
        """Release only the connection that currently owns the guest slot."""
        now = time.time() if now_s is None else float(now_s)
        with self._guest_ws_lock:
            if self._guest_ws is ws:
                self._guest_ws = None
                self._grace_until = now + self.grace_period_s
            elif getattr(self, "_guest_ws_initializing", None) is ws:
                self._guest_ws_initializing = None
                self._guest_ws_initializing_queue = []
                self._grace_until = now + self.grace_period_s

    # ── ROS2 publish ─────────────────────────────────────────────────────────

    def _phase_of(self, state: int) -> str:
        """서비스 상태값 → 프론트가 읽기 쉬운 phase 문자열."""
        return {
            AvgServiceState.DROP_ZONE_WAIT: "idle",
            AvgServiceState.MOVING_TO_SITE: "moving",
            AvgServiceState.SITE_ARRIVED: "arrived",
            AvgServiceState.RETURNING_TO_DROP_ZONE: "returning",
            AvgServiceState.GUEST_RECALL_SERVICE: "recall",
            # HH_260721 - Preserve actionable guest phases for every service state.
            AvgServiceState.SITE_ENTRY: "moving",
            AvgServiceState.UNLOAD_WAIT: "arrived",
            AvgServiceState.WAITING_FOR_RETURN_REQUEST: "arrived",
            AvgServiceState.RECALL_TO_SITE_ROAD: "moving",
            AvgServiceState.GUEST_LOADING_WAIT: "arrived",
            AvgServiceState.RETURN_WITH_CARGO: "returning",
            AvgServiceState.DROP_ZONE_PARKING: "parking",
            AvgServiceState.WAITING_FOR_CHARGING: "waiting_for_charging",
            AvgServiceState.CHARGING: "charging",
            AvgServiceState.DEPARTING_CHARGER: "moving",
            AvgServiceState.DEPARTING_DROP_ZONE: "moving",
            AvgServiceState.OPERATOR_STOPPED: "stopped",
        }.get(state, "unknown")

    def _state_name_of(self, state: int) -> str:
        # HH_260721 - Preserve the complete symbolic contract for initial guest connections.
        return {
            AvgServiceState.DROP_ZONE_WAIT: "DROP_ZONE_WAIT",
            AvgServiceState.MOVING_TO_SITE: "MOVING_TO_SITE",
            AvgServiceState.SITE_ARRIVED: "SITE_ARRIVED",
            AvgServiceState.RETURNING_TO_DROP_ZONE: "RETURNING_TO_DROP_ZONE",
            AvgServiceState.GUEST_RECALL_SERVICE: "GUEST_RECALL_SERVICE",
            AvgServiceState.SITE_ENTRY: "SITE_ENTRY",
            AvgServiceState.UNLOAD_WAIT: "UNLOAD_WAIT",
            AvgServiceState.RECALL_TO_SITE_ROAD: "RECALL_TO_SITE_ROAD",
            AvgServiceState.GUEST_LOADING_WAIT: "GUEST_LOADING_WAIT",
            AvgServiceState.RETURN_WITH_CARGO: "RETURN_WITH_CARGO",
            AvgServiceState.DROP_ZONE_PARKING: "DROP_ZONE_PARKING",
            AvgServiceState.WAITING_FOR_RETURN_REQUEST: "WAITING_FOR_RETURN_REQUEST",
            AvgServiceState.WAITING_FOR_CHARGING: "WAITING_FOR_CHARGING",
            AvgServiceState.CHARGING: "CHARGING",
            AvgServiceState.DEPARTING_CHARGER: "DEPARTING_CHARGER",
            AvgServiceState.DEPARTING_DROP_ZONE: "DEPARTING_DROP_ZONE",
            AvgServiceState.OPERATOR_STOPPED: "OPERATOR_STOPPED",
        }.get(state, f"UNKNOWN_{state}")

    def _publish_guest_recall(self) -> None:
        msg = AvgServiceState()
        msg.state = AvgServiceState.GUEST_RECALL_SERVICE
        msg.state_name = "GUEST_RECALL_SERVICE"
        # HH_260721 - Keep service descriptions English at the ROS boundary.
        msg.description = "Guest recall requested"
        self.pub_service_state.publish(msg)
        self.get_logger().info(
            f"[guest] published GUEST_RECALL_SERVICE(4) -> {self.service_state_topic}"
        )

    def _publish_navigate(self, site: str) -> None:
        """Publish a site-specific guest request for typed roadside recall."""
        with self._lock:
            nonce = str(
                getattr(self, "_pending_dispatch_nonce", "")
            ).strip()
        msg = UiDestinationCommand()
        msg.site = site
        msg.run = True
        msg.source = f"guest:dispatch:r={nonce}" if nonce else "guest"
        self.pub_destination.publish(msg)
        self.get_logger().info(
            f"[guest] roadside recall -> {self.ui_destination_topic}: site={site}"
        )

    def _publish_usage_complete(self) -> None:
        """Request the same backend-owned return sequence as the Robot UI."""
        with self._lock:
            site = str(self._active_site).strip()
            generation = int(self._active_mission_generation)
        msg = MotionOperation()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.operation = MotionOperation.RETURN
        msg.source = f"guest:usage_complete:site={site}:g={generation}"
        self.pub_operation_request.publish(msg)
        self.get_logger().info(
            "[guest] usage_complete -> RETURN request -> "
            f"{self.ui_camping_site_operation_request_topic}"
        )

    def _publish_cancel(self) -> None:
        """Request a backend-owned stop of the active guest mission."""
        with self._lock:
            site = str(self._active_site).strip()
            generation = int(self._active_mission_generation)
        msg = MotionOperation()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.operation = MotionOperation.CANCEL
        msg.source = f"guest:cancel:site={site}:g={generation}"
        self.pub_operation_request.publish(msg)
        self.get_logger().info(
            "[guest] cancel -> CANCEL request -> "
            f"{self.ui_camping_site_operation_request_topic}"
        )

    def _reserve_guest_recall_request(
        self, site: str
    ) -> tuple[bool, str, int, int]:
        """Atomically admit one Guest recall before ROS callbacks can echo it.

        The charger/drop-zone departure controller deliberately keeps a
        terminal service state for several seconds.  Reserving the intent
        under the same lock as admission prevents a second browser click from
        replacing the accepted B-site during that interval.
        """
        with self._lock:
            current = int(self._service_state)
            battery = int(self._battery)
            active_intent = str(self._active_request_intent).strip()
            active_site = str(self._active_site).strip()
            active_owner = str(
                getattr(self, "_active_request_owner", "")
            ).strip()
            retry_same_guest_recall = bool(
                getattr(self, "_active_request_retryable", False)
                and active_intent == "recall"
                and active_owner == "guest"
                and active_site == str(site).strip()
            )
            if not guest_mission_dispatch_ready(
                current,
                battery,
                self.minimum_mission_dispatch_battery_percent,
                "" if retry_same_guest_recall else active_intent,
            ):
                if active_intent:
                    error = "mission_already_active"
                elif battery < self.minimum_mission_dispatch_battery_percent:
                    error = "battery_not_ready"
                else:
                    error = "robot_not_ready"
                return False, error, current, battery

            reserved_site = str(site).strip()
            self._active_site = reserved_site
            self._active_request_intent = "recall"
            self._active_request_owner = "guest"
            self._active_mission_generation = 0
            self._dispatch_request_sequence = int(
                getattr(self, "_dispatch_request_sequence", 0)
            ) + 1
            self._pending_dispatch_nonce = (
                f"{self._dispatch_request_sequence}-{time.monotonic_ns():x}"
            )
            self._pending_dispatch_deadline_s = (
                time.monotonic()
                + float(getattr(self, "dispatch_ack_timeout_s", 5.0))
            )
            self._active_request_retryable = False
            self._mission_terminal_clear_armed = False
            self._guest_return_request_pending = False
            self._dispatch_identity_revision = int(
                getattr(self, "_dispatch_identity_revision", 0)
            ) + 1
            identity_revision = self._dispatch_identity_revision

        self._schedule_broadcast({
            "site": reserved_site,
            "request_intent": "recall",
            "request_owner": "guest",
            "mission_retryable": False,
            "mission_generation": 0,
            "identity_revision": identity_revision,
        })
        return True, "", current, battery

    def _reserve_guest_return_request(self) -> tuple[bool, str, int]:
        """Authorize exactly one return click for this Guest recall arrival."""
        with self._lock:
            current = int(self._service_state)
            if int(getattr(self, "_active_mission_generation", 0)) <= 0:
                return False, "awaiting_admission_ack", current
            if self._guest_return_request_pending:
                return False, "request_in_progress", current
            if not guest_usage_complete_available(
                current,
                self._active_request_intent,
                self._active_site,
                getattr(self, "_active_request_owner", ""),
            ):
                return False, "not_guest_recall_arrival", current
            self._guest_return_request_pending = True
        return True, "", current

    def _guest_cancel_is_owned(self) -> tuple[bool, int]:
        """Check that a public cancel targets the Guest UI's own recall."""
        with self._lock:
            current = int(self._service_state)
            available = guest_mission_cancel_available(
                current,
                self._active_request_intent,
                self._active_site,
                getattr(self, "_active_request_owner", ""),
            ) and int(getattr(self, "_active_mission_generation", 0)) > 0
        return available, current

    # ── Path resolution ───────────────────────────────────────────────────────

    def _resolve_guest_html(self) -> Optional[Path]:
        # parents[3] = package root (runtime/python/camrod_ui → runtime/python → runtime → pkg)
        source = Path(__file__).resolve().parents[3] / "camrod_ui_guest" / "assets" / "guest_frontend" / "index.html"
        if source.exists():
            return source
        # Installed (colcon build) path.
        try:
            share = Path(get_package_share_directory("camrod_ui"))
            installed = share / "camrod_ui_guest" / "assets" / "guest_frontend" / "index.html"
            if installed.exists():
                return installed
        except PackageNotFoundError:
            pass
        return None

    # ── FastAPI server ────────────────────────────────────────────────────────

    def _start_fastapi_server(self) -> None:
        node = self
        app = FastAPI(title="camrod_ui_guest")
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_methods=["*"],
            allow_headers=["*"],
        )

        @app.websocket("/ws")
        async def guest_ws(ws: WebSocket) -> None:
            await ws.accept()
            client_ip = ws.client.host

            # HJ_260804 - The claim is fully synchronous. Never suspend the
            # asyncio server while holding a threading.Lock, or simultaneous
            # refresh attempts can deadlock the only uvicorn event-loop thread.
            reject_grace_remaining = node._claim_guest_connection(ws, client_ip)
            if reject_grace_remaining is not None:
                try:
                    await node._send_guest_payload(ws, {
                        "locked": True,
                        "grace_remaining": reject_grace_remaining,
                    })
                    await ws.close(1008)
                except Exception:
                    pass
                return

            try:
                # Registration and this initial send share one finally block;
                # an aborted first frame cannot leave the guest slot occupied.
                with node._lock:
                    state = node._service_state
                    active_site = node._active_site
                    request_intent = node._active_request_intent
                    request_owner = node._active_request_owner
                    mission_retryable = node._active_request_retryable
                    mission_generation = node._active_mission_generation
                    identity_revision = node._dispatch_identity_revision
                    battery = node._battery
                    safety_hold = node._safety_hold
                    control_gate_state = node._control_gate_state
                await node._send_guest_payload(ws, {
                    "service_state": state,
                    "service_state_name": node._state_name_of(state),
                    "phase": node._phase_of(state),
                    "site": active_site,
                    "request_intent": request_intent,
                    "request_owner": request_owner,
                    "mission_retryable": mission_retryable,
                    "mission_generation": mission_generation,
                    "identity_revision": identity_revision,
                    "battery": battery,
                    "minimum_battery_percentage": (
                        node.minimum_mission_dispatch_battery_percent
                    ),
                    "mission_battery_ready": (
                        battery >= node.minimum_mission_dispatch_battery_percent
                    ),
                    "safety_hold": safety_hold,
                    "control_gate_state": control_gate_state,
                    "sites": node.site_names,
                })

                # The socket remains in an initializing queue from claim until
                # this snapshot and every newer callback frame are delivered.
                # Promotion to the live slot is atomic with observing an empty
                # queue, so a lifecycle update cannot arrive before an older
                # initial snapshot and roll the browser backward.
                while True:
                    with node._guest_ws_lock:
                        if node._guest_ws_initializing is not ws:
                            raise RuntimeError(
                                "guest websocket initialization was cancelled"
                            )
                        if not node._guest_ws_initializing_queue:
                            node._guest_ws_initializing = None
                            node._guest_ws = ws
                            break
                        pending_payloads = list(
                            node._guest_ws_initializing_queue
                        )
                        node._guest_ws_initializing_queue.clear()
                    for queued_payload in pending_payloads:
                        await node._send_guest_payload(ws, queued_payload)

                idle_cycles = 0
                while True:
                    try:
                        raw = await asyncio.wait_for(ws.receive_text(), timeout=15.0)
                    except asyncio.TimeoutError:
                        # The frontend sends an application heartbeat every
                        # 10 s. Three missed intervals identify a stale slot.
                        idle_cycles += 1
                        if idle_cycles >= 3:
                            node.get_logger().warn(
                                "[guest] idle timeout (45s); closing stale connection"
                            )
                            try:
                                await ws.close(1000)
                            except Exception:
                                pass
                            break
                        continue
                    idle_cycles = 0
                    payload = json.loads(raw)
                    action = payload.get("action")

                    if action == "heartbeat":
                        continue

                    # Keep the legacy browser action name while publishing a
                    # typed roadside recall for the selected B1-B13 site.
                    if action == "navigate":
                        site = str(payload.get("site", "")).strip()
                        if site not in node.site_names:
                            await node._send_guest_payload(
                                ws, {"error": "invalid_site", "site": site}
                            )
                        else:
                            # Admission and reservation are one critical
                            # section. Publishing happens only after ownership
                            # is recorded, so the terminal departure dwell can
                            # never admit a competing B-site.
                            admitted, error, current, battery = (
                                await asyncio.to_thread(
                                    node._reserve_guest_recall_request, site
                                )
                            )
                            if admitted:
                                await asyncio.to_thread(node._publish_navigate, site)
                            else:
                                await node._send_guest_payload(ws, {
                                    "error": error,
                                    "service_state": current,
                                    "battery": battery,
                                    "minimum_battery_percentage": (
                                        node.minimum_mission_dispatch_battery_percent
                                    ),
                                })

                    # Legacy no-site recall action retained for compatibility.
                    elif action == "recall":
                        admitted, error, current, battery = (
                            await asyncio.to_thread(
                                node._reserve_guest_recall_request, "legacy"
                            )
                        )
                        if admitted:
                            await asyncio.to_thread(node._publish_guest_recall)
                        else:
                            await node._send_guest_payload(ws, {
                                "error": error,
                                "service_state": current,
                                "battery": battery,
                            })

                    # (확장) 이용 완료 → 드롭존 복귀
                    elif action == "usage_complete":
                        admitted, error, current = await asyncio.to_thread(
                            node._reserve_guest_return_request
                        )
                        if admitted:
                            try:
                                await asyncio.to_thread(node._publish_usage_complete)
                            except Exception:
                                with node._lock:
                                    node._guest_return_request_pending = False
                                raise
                        else:
                            await node._send_guest_payload(ws, {
                                "error": error,
                                "service_state": current,
                            })

                    # 이동/복귀/도킹 중 취소 → Robot UI와 동일한 backend stop 경로.
                    elif action == "cancel":
                        available, current = await asyncio.to_thread(
                            node._guest_cancel_is_owned
                        )
                        if available:
                            await asyncio.to_thread(node._publish_cancel)
                            await node._send_guest_payload(
                                ws, {"cancel_requested": True}
                            )
                        else:
                            await node._send_guest_payload(ws, {
                                "error": "robot_not_active",
                                "service_state": current,
                            })

            except WebSocketDisconnect:
                pass
            except Exception as exc:
                # Always reach slot cleanup, including an aborted initial send
                # or a frame write racing a browser disconnect.
                node.get_logger().warn(f"[guest] connection handler error: {exc}")
            finally:
                node._release_guest_connection(ws)

        html_path = node._resolve_guest_html()

        if html_path:
            # index.html 단일 파일만이 아니라, 같은 폴더의 정적 파일(사이트 이미지 등)도
            # 서빙되도록 변경. 없는 경로는 index.html로 폴백(SPA 동작).
            guest_dir = html_path.parent
            html_real = Path(os.path.realpath(str(html_path)))
            no_store_headers = {
                "Cache-Control": "no-store, no-cache, must-revalidate",
                "Pragma": "no-cache",
            }

            @app.get("/{full_path:path}")
            async def serve_html(full_path: str) -> FileResponse:
                candidate = guest_dir / full_path
                real = Path(os.path.realpath(str(candidate)))
                if real.is_file():
                    if real == html_real:
                        return FileResponse(
                            str(real), headers=no_store_headers
                        )
                    return FileResponse(str(real))
                return FileResponse(
                    str(html_real), headers=no_store_headers
                )
        else:
            @app.get("/")
            async def serve_fallback() -> JSONResponse:
                return JSONResponse({"message": "guest frontend not found"}, status_code=503)

        def _run() -> None:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            node._main_loop = loop
            config = uvicorn.Config(
                app,
                host=node.host,
                port=node.port,
                log_level="warning",
                loop="asyncio",
            )
            server = uvicorn.Server(config)
            node._uvicorn_server = server
            if node._server_stop_requested.is_set():
                server.should_exit = True
            node.get_logger().info(
                f"guest ui fastapi listening: http://{node.host}:{node.port}"
            )
            try:
                loop.run_until_complete(server.serve())
            finally:
                node._uvicorn_server = None
                node._main_loop = None
                loop.close()

        self._server_thread = threading.Thread(
            target=_run,
            name="camrod_ui_guest_fastapi",
            daemon=True,
        )
        self._server_thread.start()


def main() -> None:
    rclpy.init()
    node = UiGuestNode()
    try:
        rclpy.spin(node)
    # HH_260805 - Treat launch-driven context shutdown like terminal SIGINT so
    # Guest UI teardown does not intermittently report a process failure.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    # HH_260807 - Treat Humble's take_message conversion race as shutdown only
    # when the context is already invalid; never hide a live runtime failure.
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
