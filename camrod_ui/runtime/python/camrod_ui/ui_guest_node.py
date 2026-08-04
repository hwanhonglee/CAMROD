#!/usr/bin/env python3
# HJ_260601: Guest UI backend for WiFi-accessible robot recall service.
#            Single-user WebSocket lock, GUEST_RECALL_SERVICE(4) publish on recall.
# (확장) 사이트 선택 호출(경우 B) 지원:
#            mobile에서 B1~B13 선택 → /ui/selected_destination publish →
#            기존 ui_backend_node가 goal_pose/engage 반응. usage_complete 추가.
"""Guest UI backend: mobile/laptop access for robot recall service.

Binds to 0.0.0.0 so devices on the same WiFi can reach it.
Only one WebSocket client is allowed at a time (exclusive lock).
"""

from __future__ import annotations

import asyncio
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
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def normalize_platform_battery_percent(value: float) -> int:
    """Convert canonical AvgPlatformStatus SOC ratio to a display percent."""
    if not math.isfinite(float(value)):
        return -1
    return max(0, min(100, int(round(float(value) * 100.0))))


def guest_mission_dispatch_ready(state: int, battery: int, minimum: int) -> bool:
    """Apply the Robot UI's stationary-state and SOC admission boundary."""
    return state in {
        int(AvgServiceState.DROP_ZONE_WAIT),
        int(AvgServiceState.CHARGING),
    } and battery >= minimum


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
        self._battery: int = -1
        self._safety_hold: bool = False
        self._control_gate_state: str = "UNKNOWN"
        # HH_260721 - Preserve the physical charging state when a destination is cleared.
        self._is_charging: bool = False

        # Single-client WebSocket exclusive lock.
        self._guest_ws: Optional[WebSocket] = None
        self._guest_ws_lock = threading.Lock()
        # HJ_260804 - ROS callbacks and request responses can schedule writes
        # concurrently. Serialize frames on the uvicorn event loop so one
        # connection never receives overlapping send_json() calls.
        self._guest_ws_send_lock = asyncio.Lock()
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
        self.sub_service_state = self.create_subscription(
            AvgServiceState,
            self.service_state_topic,
            self._on_service_state,
            10,
        )
        self.sub_battery = self.create_subscription(
            AvgPlatformStatus,
            self.battery_topic,
            self._on_battery,
            10,
        )
        self.sub_destination = self.create_subscription(
            UiDestinationCommand,
            self.ui_destination_topic,
            self._on_destination_cleared,
            10,
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
        # (확장) 사이트 선택 → 목적지 토픽 publish (ui_backend_node가 goal_pose 처리)
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
            self._service_state = state
        phase = str(msg.state_name).strip() or self._state_name_of(state)
        # HH_260721 - Forward both numeric and symbolic service state to guest clients.
        self._schedule_broadcast({
            "service_state": state,
            "service_state_name": phase,
            "phase": self._phase_of(state),
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

    def _on_destination_cleared(self, msg: UiDestinationCommand) -> None:
        if msg.run:
            return
        with self._lock:
            # HH_260721 - Clearing a destination must not hide active charging.
            state = (
                AvgServiceState.CHARGING
                if self._is_charging
                else AvgServiceState.DROP_ZONE_WAIT
            )
            self._service_state = state
        self._schedule_broadcast({
            "service_state": state,
            "service_state_name": self._state_name_of(state),
            "phase": self._phase_of(state),
        })
        self.get_logger().info(
            f"[guest] Destination cleared -> {self._state_name_of(state)}"
        )

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
            if self._guest_ws is not None:
                return 0
            if now < self._grace_until and client_ip != self._last_client_ip:
                return max(1, math.ceil(self._grace_until - now))
            self._guest_ws = ws
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
        """(확장) 사이트 선택 호출 → ui_backend_node가 받아 goal_pose/engage를 반응."""
        msg = UiDestinationCommand()
        msg.site = site
        msg.run = True
        msg.source = "guest"
        self.pub_destination.publish(msg)
        self.get_logger().info(
            f"[guest] navigate -> {self.ui_destination_topic}: site={site}"
        )

    def _publish_usage_complete(self) -> None:
        """Request the same backend-owned return sequence as the Robot UI."""
        msg = MotionOperation()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.operation = MotionOperation.RETURN
        msg.source = "guest:usage_complete"
        self.pub_operation_request.publish(msg)
        self.get_logger().info(
            "[guest] usage_complete -> RETURN request -> "
            f"{self.ui_camping_site_operation_request_topic}"
        )

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
                    battery = node._battery
                    safety_hold = node._safety_hold
                    control_gate_state = node._control_gate_state
                await node._send_guest_payload(ws, {
                    "service_state": state,
                    "service_state_name": node._state_name_of(state),
                    "phase": node._phase_of(state),
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

                    # (확장) 사이트 선택 호출 (경우 B): B1~B13 중 선택 → 해당 사이트로 이동
                    if action == "navigate":
                        site = str(payload.get("site", "")).strip()
                        if site not in node.site_names:
                            await node._send_guest_payload(
                                ws, {"error": "invalid_site", "site": site}
                            )
                        else:
                            # Keep ROS lock contention and publish work off the
                            # single uvicorn event-loop thread.
                            def _dispatch_navigate() -> tuple[int, int]:
                                with node._lock:
                                    current = node._service_state
                                    battery = node._battery
                                if guest_mission_dispatch_ready(
                                    current,
                                    battery,
                                    node.minimum_mission_dispatch_battery_percent,
                                ):
                                    node._publish_navigate(site)
                                return current, battery

                            current, battery = await asyncio.to_thread(
                                _dispatch_navigate
                            )
                            if not guest_mission_dispatch_ready(
                                current,
                                battery,
                                node.minimum_mission_dispatch_battery_percent,
                            ):
                                error = (
                                    "battery_not_ready"
                                    if battery < node.minimum_mission_dispatch_battery_percent
                                    else "robot_not_ready"
                                )
                                await node._send_guest_payload(ws, {
                                    "error": error,
                                    "service_state": current,
                                    "battery": battery,
                                    "minimum_battery_percentage": (
                                        node.minimum_mission_dispatch_battery_percent
                                    ),
                                })

                    # 단순 호출 (드롭존으로) — 기존 동작 유지
                    elif action == "recall":
                        def _dispatch_recall() -> int:
                            with node._lock:
                                current = node._service_state
                            if current in {
                                AvgServiceState.DROP_ZONE_WAIT,
                                AvgServiceState.CHARGING,
                            }:
                                node._publish_guest_recall()
                            return current

                        current = await asyncio.to_thread(_dispatch_recall)
                        if current not in {
                            AvgServiceState.DROP_ZONE_WAIT,
                            AvgServiceState.CHARGING,
                        }:
                            await node._send_guest_payload(ws, {
                                "error": "robot_not_ready",
                                "service_state": current,
                            })

                    # (확장) 이용 완료 → 드롭존 복귀
                    elif action == "usage_complete":
                        await asyncio.to_thread(node._publish_usage_complete)

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

            @app.get("/{full_path:path}")
            async def serve_html(full_path: str) -> FileResponse:
                candidate = guest_dir / full_path
                real = Path(os.path.realpath(str(candidate)))
                if real.is_file():
                    return FileResponse(str(real))
                return FileResponse(str(os.path.realpath(str(html_path))))
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
