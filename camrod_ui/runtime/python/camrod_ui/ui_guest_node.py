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
import os
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
import uvicorn
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from avg_msgs.msg import AvgAmrServiceState, UiDestinationCommand
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
        self.grace_period_s = int(self.declare_parameter("grace_period_s", 60).value)

        # (확장) 사이트 선택 호출용 — ui_backend_node가 구독하는 토픽과 동일하게 맞춤
        self.ui_destination_topic = str(
            self.declare_parameter("ui_destination_topic", "/ui/selected_destination").value
        )
        self.site_names = [
            str(s)
            for s in self.declare_parameter("site_names", [f"B{i}" for i in range(1, 14)]).value
        ]
        if not self.site_names:
            self.site_names = [f"B{i}" for i in range(1, 14)]

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
        self.sub_destination = self.create_subscription(
            UiDestinationCommand,
            self.ui_destination_topic,
            self._on_destination_cleared,
            10,
        )

        self.pub_amr = self.create_publisher(
            AvgAmrServiceState,
            self.amr_service_state_topic,
            10,
        )
        # (확장) 사이트 선택 → 목적지 토픽 publish (ui_backend_node가 goal_pose 처리)
        self.pub_destination = self.create_publisher(
            UiDestinationCommand,
            self.ui_destination_topic,
            10,
        )

        self._start_fastapi_server()

        self.get_logger().info(
            f"ui_guest ready: host={self.host} port={self.port} "
            f"destination_topic={self.ui_destination_topic}"
        )

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def _on_amr_state(self, msg: AvgAmrServiceState) -> None:
        state = int(msg.state)
        with self._lock:
            self._amr_state = state
        # (확장) phase 문자열 동봉 → 프론트가 정수값 하드코딩에 의존하지 않게
        self._schedule_broadcast({"amr_state": state, "phase": self._phase_of(state)})
        self.get_logger().info(f"[guest] AMR state received: {state}")

    def _on_battery(self, msg: Int32) -> None:
        pct = max(0, min(100, int(msg.data)))
        with self._lock:
            self._battery = pct
        self._schedule_broadcast({"battery": pct})

    def _on_destination_cleared(self, msg: UiDestinationCommand) -> None:
        if msg.run:
            return
        with self._lock:
            self._amr_state = AvgAmrServiceState.DROP_ZONE_WAIT
        self._schedule_broadcast({
            "amr_state": AvgAmrServiceState.DROP_ZONE_WAIT,
            "phase": "idle",
        })
        self.get_logger().info("[guest] destination cleared -> broadcast idle to guest UI")

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

    def _phase_of(self, state: int) -> str:
        """AMR 상태값 → 프론트가 읽기 쉬운 phase 문자열."""
        return {
            AvgAmrServiceState.DROP_ZONE_WAIT: "idle",
            AvgAmrServiceState.MOVING_TO_SITE: "moving",
            AvgAmrServiceState.SITE_ARRIVED: "arrived",
            AvgAmrServiceState.RETURNING_TO_DROP_ZONE: "returning",
            AvgAmrServiceState.GUEST_RECALL_SERVICE: "recall",
        }.get(state, "unknown")

    def _publish_guest_recall(self) -> None:
        msg = AvgAmrServiceState()
        msg.state = AvgAmrServiceState.GUEST_RECALL_SERVICE
        msg.description = "Guest 호출 요청"
        self.pub_amr.publish(msg)
        self.get_logger().info(
            f"[guest] published GUEST_RECALL_SERVICE(4) -> {self.amr_service_state_topic}"
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
        """(확장) 이용 완료 → 드롭존 복귀 상태 발행 (키오스크 usage_complete와 동일 신호)."""
        msg = AvgAmrServiceState()
        msg.state = AvgAmrServiceState.RETURNING_TO_DROP_ZONE
        msg.description = "이용 완료 → Drop Zone 복귀"
        self.pub_amr.publish(msg)
        self.get_logger().info(
            f"[guest] usage_complete -> RETURNING_TO_DROP_ZONE -> {self.amr_service_state_topic}"
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
                    await ws.send_json({"locked": True, "grace_remaining": 0})
                    await ws.close(1008)
                    return

                now = time.time()
                if now < node._grace_until and client_ip != node._last_client_ip:
                    remaining = int(node._grace_until - now)
                    await ws.send_json({"locked": True, "grace_remaining": remaining})
                    await ws.close(1008)
                    return

                node._guest_ws = ws
                node._last_client_ip = client_ip
                node._grace_until = 0.0

            # Send initial state — phase + 사이트 목록 동봉
            with node._lock:
                state = node._amr_state
                battery = node._battery
            await ws.send_json({
                "amr_state": state,
                "phase": node._phase_of(state),
                "battery": battery,
                "sites": node.site_names,
            })

            try:
                while True:
                    raw = await ws.receive_text()
                    payload = json.loads(raw)
                    action = payload.get("action")

                    # (확장) 사이트 선택 호출 (경우 B): B1~B13 중 선택 → 해당 사이트로 이동
                    if action == "navigate":
                        site = str(payload.get("site", "")).strip()
                        if site not in node.site_names:
                            await ws.send_json({"error": "invalid_site", "site": site})
                        else:
                            with node._lock:
                                current = node._amr_state
                            if current == AvgAmrServiceState.DROP_ZONE_WAIT:
                                node._publish_navigate(site)
                            else:
                                await ws.send_json({"error": "robot_not_ready", "amr_state": current})

                    # 단순 호출 (드롭존으로) — 기존 동작 유지
                    elif action == "recall":
                        with node._lock:
                            current = node._amr_state
                        if current == AvgAmrServiceState.DROP_ZONE_WAIT:
                            node._publish_guest_recall()
                        else:
                            await ws.send_json({"error": "robot_not_ready", "amr_state": current})

                    # (확장) 이용 완료 → 드롭존 복귀
                    elif action == "usage_complete":
                        node._publish_usage_complete()

            except WebSocketDisconnect:
                pass
            finally:
                with node._guest_ws_lock:
                    if node._guest_ws is ws:
                        node._guest_ws = None
                        node._grace_until = time.time() + node.grace_period_s
                        # _last_client_ip 유지 → 같은 IP는 유예 기간 내 즉시 재접속 가능

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
