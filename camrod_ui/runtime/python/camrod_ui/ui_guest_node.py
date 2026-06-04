#!/usr/bin/env python3
# HJ_260601: Guest UI backend for WiFi-accessible robot recall service.
#            Single-user WebSocket lock, GUEST_RECALL_SERVICE(4) publish on recall.
"""Guest UI backend: mobile/laptop access for robot recall service.

Binds to 0.0.0.0 so devices on the same WiFi can reach it.
Only one WebSocket client is allowed at a time (exclusive lock).
"""

from __future__ import annotations

import asyncio
import json
import os
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
import uvicorn
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from avg_msgs.msg import AvgAmrServiceState
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from rclpy.node import Node
from std_msgs.msg import Int32


class UiGuestNode(Node):
    """Lightweight FastAPI node serving the guest recall UI over WiFi."""

    def __init__(self) -> None:
        super().__init__("ui_guest")

        self.host = str(self.declare_parameter("host", "0.0.0.0").value)
        self.port = int(self.declare_parameter("port", 8012).value)
        self.amr_service_state_topic = str(
            self.declare_parameter("amr_service_state_topic", "/AMR_service_state").value
        )
        self.battery_topic = str(
            self.declare_parameter("battery_topic", "/battery_percentage").value
        )
        self.grace_period_sec = int(self.declare_parameter("grace_period_sec", 60).value)

        self._lock = threading.Lock()
        self._amr_state: int = AvgAmrServiceState.DROP_ZONE_WAIT
        self._battery: int = -1

        # Single-client WebSocket exclusive lock.
        self._guest_ws: Optional[WebSocket] = None
        self._guest_ws_lock = threading.Lock()
        self._main_loop: Optional[asyncio.AbstractEventLoop] = None

        # Grace period: hold lock briefly after disconnect so same user can reconnect.
        self._last_client_ip: Optional[str] = None
        self._grace_until: float = 0.0

        self.sub_amr = self.create_subscription(
            AvgAmrServiceState,
            self.amr_service_state_topic,
            self._on_amr_state,
            10,
        )
        self.sub_battery = self.create_subscription(
            Int32,
            self.battery_topic,
            self._on_battery,
            10,
        )

        self.pub_amr = self.create_publisher(
            AvgAmrServiceState,
            self.amr_service_state_topic,
            10,
        )

        self._start_fastapi_server()

        self.get_logger().info(
            f"ui_guest ready: host={self.host} port={self.port}"
        )

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def _on_amr_state(self, msg: AvgAmrServiceState) -> None:
        state = int(msg.state)
        with self._lock:
            self._amr_state = state
        self._schedule_broadcast({"amr_state": state})
        self.get_logger().info(f"[guest] AMR state received: {state}")

    def _on_battery(self, msg: Int32) -> None:
        pct = max(0, min(100, int(msg.data)))
        with self._lock:
            self._battery = pct
        self._schedule_broadcast({"battery": pct})

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
            await ws.send_json(payload)
        except Exception:
            with self._guest_ws_lock:
                if self._guest_ws is ws:
                    self._guest_ws = None

    # ── ROS2 publish ─────────────────────────────────────────────────────────

    def _publish_guest_recall(self) -> None:
        msg = AvgAmrServiceState()
        msg.state = AvgAmrServiceState.GUEST_RECALL_SERVICE
        msg.description = "Guest 호출 요청"
        self.pub_amr.publish(msg)
        self.get_logger().info(
            f"[guest] published GUEST_RECALL_SERVICE(4) -> {self.amr_service_state_topic}"
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

            with node._guest_ws_lock:
                if node._guest_ws is not None:
                    # 다른 사람이 활성 세션 중
                    await ws.send_json({"locked": True, "grace_remaining": 0})
                    await ws.close(1008)
                    return

                now = time.time()
                if now < node._grace_until and client_ip != node._last_client_ip:
                    # 유예 기간 중 + 다른 IP → 차단, 남은 시간 전달
                    remaining = int(node._grace_until - now)
                    await ws.send_json({"locked": True, "grace_remaining": remaining})
                    await ws.close(1008)
                    return

                node._guest_ws = ws
                node._last_client_ip = client_ip
                node._grace_until = 0.0

            # Send initial state.
            with node._lock:
                state = node._amr_state
                battery = node._battery
            await ws.send_json({"amr_state": state, "battery": battery})

            try:
                while True:
                    raw = await ws.receive_text()
                    payload = json.loads(raw)

                    if payload.get("action") == "recall":
                        with node._lock:
                            current = node._amr_state
                        if current == AvgAmrServiceState.DROP_ZONE_WAIT:
                            node._publish_guest_recall()
                        else:
                            await ws.send_json({"error": "robot_not_ready", "amr_state": current})

            except WebSocketDisconnect:
                pass
            finally:
                with node._guest_ws_lock:
                    if node._guest_ws is ws:
                        node._guest_ws = None
                        node._grace_until = time.time() + node.grace_period_sec
                        # _last_client_ip 유지 → 같은 IP는 유예 기간 내 즉시 재접속 가능

        html_path = node._resolve_guest_html()

        if html_path:
            @app.get("/{full_path:path}")
            async def serve_html(full_path: str) -> FileResponse:
                real = Path(os.path.realpath(str(html_path)))
                return FileResponse(str(real))
        else:
            @app.get("/")
            async def serve_fallback() -> JSONResponse:
                return JSONResponse({"message": "guest frontend not found"}, status_code=503)

        def _run() -> None:
            loop = asyncio.new_event_loop()
            node._main_loop = loop
            config = uvicorn.Config(
                app,
                host=node.host,
                port=node.port,
                log_level="warning",
                loop="asyncio",
            )
            server = uvicorn.Server(config)
            node.get_logger().info(
                f"guest ui fastapi listening: http://{node.host}:{node.port}"
            )
            loop.run_until_complete(server.serve())

        thread = threading.Thread(target=_run, name="camrod_ui_guest_fastapi", daemon=True)
        thread.start()


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
