"""Manager UI for campsite PINs and read-only robot monitoring.

Served on its own port (default 8014) only to the robot's own Wi-Fi/LAN
(lan_guard). One shared manager account; the shipped default must be changed
on first login. PINs live in SiteAccessStore, which ui_backend_node (Robot UI)
and ui_guest_node (Guest UI) read when campers enter a PIN.

The HTTP layer is thin; ManagerService holds the rules so they can be tested
without a web client.
"""

from __future__ import annotations

import asyncio
import secrets
import threading
import time
from pathlib import Path
from typing import Dict, Optional, Tuple
from urllib.parse import urlsplit

import rclpy
import uvicorn
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, JSONResponse, Response
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from camrod_ui.lan_guard import install_lan_guard
from camrod_ui.manager_monitoring import MONITORING_VIEWS, ManagerMonitoring
from camrod_ui.site_access_store import AttemptLimiter, SiteAccessStore, valid_pin

SESSION_COOKIE = "camrod_manager_session"
LOGIN_MAX_FAILURES = 5
LOGIN_LOCKOUT_S = 300.0

MESSAGES = {
    "unauthenticated": "다시 로그인해 주세요.",
    "must_change": "처음 로그인했어요. 관리자 아이디와 비밀번호를 먼저 바꿔 주세요.",
    "login_failed": "아이디 또는 비밀번호가 맞지 않아요.",
    "login_locked": "로그인을 여러 번 틀렸어요. 잠시 후 다시 시도해 주세요.",
    "unknown_site": "없는 사이트예요.",
    "invalid_pin": "비밀번호는 숫자 4자리로 입력해 주세요.",
    "invalid_username": "아이디는 영문·숫자 3~32자로 입력해 주세요.",
    "weak_password": "새 비밀번호는 4자 이상으로 입력해 주세요.",
    "wrong_password": "지금 비밀번호가 맞지 않아요.",
    "password_mismatch": "새 비밀번호 확인이 일치하지 않아요.",
    "monitoring_unavailable": "로봇 상태를 가져오지 못했습니다.",
}


def _error(code: str, status: int, **extra) -> Tuple[int, dict]:
    return status, {"ok": False, "error": code, "message": MESSAGES.get(code, code), **extra}


class ManagerService:
    """Login sessions and site-PIN management rules (no HTTP)."""

    def __init__(self, store: SiteAccessStore, session_ttl_s: float = 1800.0,
                 clock=time.monotonic) -> None:
        self.store = store
        self.session_ttl_s = float(session_ttl_s)
        self._clock = clock
        self._sessions: Dict[str, Tuple[str, float]] = {}
        self._lock = threading.Lock()
        self.login_limiter = AttemptLimiter(LOGIN_MAX_FAILURES, LOGIN_LOCKOUT_S, clock=clock)

    # ── sessions ────────────────────────────────────────────────────────
    def login(self, username: str, password: str, client: str) -> Tuple[int, dict, Optional[str]]:
        locked = self.login_limiter.remaining_lock_s(client)
        if locked > 0:
            status, body = _error("login_locked", 429, locked_s=int(locked + 0.999))
            return status, body, None
        ok = self.store.verify_manager(username, password)
        self.login_limiter.record(client, ok)
        if not ok:
            status, body = _error("login_failed", 401)
            return status, body, None
        token = secrets.token_urlsafe(32)
        with self._lock:
            self._sessions[token] = (self.store.manager_username(), self._clock() + self.session_ttl_s)
        return 200, {"ok": True, **self.session_info(token)}, token

    def logout(self, token: Optional[str]) -> None:
        with self._lock:
            self._sessions.pop(token or "", None)

    def _session_user(self, token: Optional[str], *, refresh: bool = True) -> Optional[str]:
        with self._lock:
            entry = self._sessions.get(token or "")
            if not entry:
                return None
            username, expires = entry
            if expires <= self._clock():
                self._sessions.pop(token, None)
                return None
            # Sliding expiry while the manager keeps working.
            if refresh:
                self._sessions[token] = (username, self._clock() + self.session_ttl_s)
            return username

    def session_info(self, token: Optional[str]) -> dict:
        username = self._session_user(token)
        if username is None:
            return {"authenticated": False}
        return {"authenticated": True, "username": self.store.manager_username(),
                "must_change": self.store.manager_must_change()}

    def _authorize(self, token: Optional[str], allow_must_change: bool = False, *, refresh: bool = True):
        if self._session_user(token, refresh=refresh) is None:
            return _error("unauthenticated", 401)
        if not allow_must_change and self.store.manager_must_change():
            return _error("must_change", 403)
        return None

    # ── sites ───────────────────────────────────────────────────────────
    def list_sites(self, token: Optional[str]) -> Tuple[int, dict]:
        denied = self._authorize(token)
        if denied:
            return denied
        pins = self.store.pins()
        sites = [{"site": site, "updated_at": pins.get(site, {}).get("updated_at", ""),
                  "updated_by": pins.get(site, {}).get("updated_by", "")}
                 for site in self.store.sites]
        return 200, {"ok": True, "sites": sites}

    def reveal_pin(self, token: Optional[str], site: str) -> Tuple[int, dict]:
        denied = self._authorize(token)
        if denied:
            return denied
        site = str(site or "").strip().upper()
        entry = self.store.pins().get(site)
        if not entry:
            return _error("unknown_site", 404)
        return 200, {"ok": True, "site": site, "pin": entry["pin"]}

    def set_pin(self, token: Optional[str], site: str, pin: str) -> Tuple[int, dict]:
        denied = self._authorize(token)
        if denied:
            return denied
        if not valid_pin(pin):
            return _error("invalid_pin", 400)
        try:
            entry = self.store.set_pin(site, pin, by=self.store.manager_username())
        except ValueError as exc:
            return _error(str(exc), 404 if str(exc) == "unknown_site" else 400)
        return 200, {"ok": True, "site": str(site).strip().upper(),
                     "updated_at": entry["updated_at"], "updated_by": entry["updated_by"]}

    def history(self, token: Optional[str], limit: int = 100) -> Tuple[int, dict]:
        denied = self._authorize(token)
        if denied:
            return denied
        return 200, {"ok": True, "history": self.store.history(limit)}

    def change_account(self, token: Optional[str], current_password: str, new_username: str,
                       new_password: str, confirm_password: str) -> Tuple[int, dict]:
        denied = self._authorize(token, allow_must_change=True)
        if denied:
            return denied
        if str(new_password or "") != str(confirm_password or ""):
            return _error("password_mismatch", 400)
        try:
            self.store.change_manager(current_password, new_username, new_password)
        except PermissionError as exc:
            return _error(str(exc), 403)
        except ValueError as exc:
            return _error(str(exc), 400)
        # Every other session must log in again with the new account.
        with self._lock:
            keep = self._sessions.get(token or "")
            self._sessions = {token: (self.store.manager_username(), keep[1])} if keep else {}
        return 200, {"ok": True, **self.session_info(token)}


def _same_origin(request: Request) -> bool:
    origin = request.headers.get("origin")
    if not origin:
        return True
    try:
        parsed = urlsplit(origin)
    except ValueError:
        return False
    return (parsed.scheme, parsed.netloc) == (request.url.scheme, request.url.netloc)


class UiManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("ui_manager")
        self.host = str(self.declare_parameter("host", "0.0.0.0").value)
        self.port = int(self.declare_parameter("port", 8014).value)
        sites = [str(s).strip().upper() for s in self.declare_parameter(
            "site_names", [f"B{i}" for i in range(1, 14)]).value if str(s).strip()]
        ttl = float(self.declare_parameter("session_ttl_s", 1800.0).value)
        self.store = SiteAccessStore(sites=sites)
        self.store.ensure_initialized()
        self.service = ManagerService(self.store, session_ttl_s=ttl)
        self.monitoring = ManagerMonitoring(
            robot_port=int(self.declare_parameter("robot_ui_port", 8010).value),
            video_port=int(self.declare_parameter("video_server_port", 8080).value),
            front_topic=str(self.declare_parameter(
                "front_camera_topic", "/sensing/camera/econ_front/image_raw").value),
            rear_topic=str(self.declare_parameter(
                "rear_camera_topic", "/sensing/camera/econ_rear/image_raw").value),
        )
        self._platform_sample = (None, {})
        self._occupancy_sample = (None, None)
        self._subscribe_monitoring()
        self._server: Optional[uvicorn.Server] = None
        self._thread = threading.Thread(target=self._run_server, name="ui_manager_http", daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"manager_ui on http://{self.host}:{self.port} sites={len(sites)} store={self.store.path}")

    def _subscribe_monitoring(self) -> None:
        from avg_msgs.msg import AvgPlatformStatus, CampsiteOccupancy

        platform_topic = str(self.declare_parameter("platform_status_topic", "/platform/status").value)
        occupancy_topic = str(self.declare_parameter(
            "campsite_occupancy_topic", "/perception/camping_sites/occupancy").value)
        self.create_subscription(AvgPlatformStatus, platform_topic,
                                 self._on_platform_status, qos_profile_sensor_data)
        self.create_subscription(CampsiteOccupancy, occupancy_topic,
                                 self._on_site_occupancy, qos_profile_sensor_data)

    def _on_platform_status(self, message) -> None:
        self._platform_sample = (time.monotonic(), {
            "state": message.vehicle_state,
            "control_mode": message.control_mode,
            "motion_mode": message.motion_mode,
            "error_code": message.error_code,
            "estop": message.estop,
            "charging": message.is_charging,
        })

    def _on_site_occupancy(self, message) -> None:
        self._occupancy_sample = (time.monotonic(), list(message.occupied_mission_keys))

    def _frontend_dir(self) -> Optional[Path]:
        try:
            share = Path(get_package_share_directory("camrod_ui"))
        except PackageNotFoundError:
            return None
        path = share / "camrod_ui_manager" / "assets" / "manager_frontend"
        return path if (path / "index.html").is_file() else None

    def _build_app(self) -> FastAPI:
        app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)
        install_lan_guard(app, self.get_logger())
        service = self.service

        def reply(result, token_to_set: Optional[str] = None, clear: bool = False) -> JSONResponse:
            status, body = result[0], result[1]
            response = JSONResponse(body, status_code=status,
                                    headers={"Cache-Control": "no-store"})
            if token_to_set:
                response.set_cookie(SESSION_COOKIE, token_to_set, httponly=True, samesite="strict",
                                    max_age=int(service.session_ttl_s), path="/")
            if clear:
                response.delete_cookie(SESSION_COOKIE, path="/")
            return response

        async def payload(request: Request) -> dict:
            try:
                data = await request.json()
            except (ValueError, UnicodeError):
                return {}
            return data if isinstance(data, dict) else {}

        def token_of(request: Request) -> Optional[str]:
            return request.cookies.get(SESSION_COOKIE)

        @app.middleware("http")
        async def same_origin_writes(request: Request, call_next):
            if request.method not in {"GET", "HEAD"} and not _same_origin(request):
                return JSONResponse({"ok": False, "error": "same_origin_required"}, status_code=403)
            return await call_next(request)

        @app.get("/api/session")
        async def get_session(request: Request):
            return reply((200, {"ok": True, **service.session_info(token_of(request))}))

        @app.post("/api/login")
        async def post_login(request: Request):
            data = await payload(request)
            client = (request.client.host if request.client else "") or "unknown"
            status, body, token = service.login(data.get("username", ""), data.get("password", ""), client)
            return reply((status, body), token_to_set=token)

        @app.post("/api/logout")
        async def post_logout(request: Request):
            service.logout(token_of(request))
            return reply((200, {"ok": True}), clear=True)

        @app.get("/api/sites")
        async def get_sites(request: Request):
            return reply(service.list_sites(token_of(request)))

        @app.post("/api/sites/{site}/reveal")
        async def post_reveal(site: str, request: Request):
            return reply(service.reveal_pin(token_of(request), site))

        @app.post("/api/sites/{site}/pin")
        async def post_pin(site: str, request: Request):
            data = await payload(request)
            return reply(service.set_pin(token_of(request), site, data.get("pin", "")))

        @app.get("/api/history")
        async def get_history(request: Request):
            return reply(service.history(token_of(request)))

        @app.post("/api/account")
        async def post_account(request: Request):
            data = await payload(request)
            return reply(service.change_account(
                token_of(request), data.get("current_password", ""), data.get("new_username", ""),
                data.get("new_password", ""), data.get("confirm_password", "")))

        @app.get("/api/monitoring/camera/{camera}")
        def get_monitoring_camera(camera: str, request: Request):
            denied = service._authorize(token_of(request), refresh=False)
            if denied:
                return reply(denied)
            if camera not in self.monitoring.camera_topics:
                return reply(_error("unknown_camera", 404))
            frame = self.monitoring.camera_frame(camera)
            if frame is None:
                return Response(status_code=204, headers=no_store)
            return Response(frame, media_type="image/jpeg", headers=no_store)

        @app.get("/api/monitoring/{view}")
        def get_monitoring(view: str, request: Request):
            denied = service._authorize(token_of(request), refresh=False)
            if denied:
                return reply(denied)
            if view not in MONITORING_VIEWS:
                return reply(_error("unknown_view", 404))
            now = time.monotonic()
            platform_at, platform = self._platform_sample
            occupancy_at, occupancy = self._occupancy_sample
            data = self.monitoring.snapshot(
                view,
                {**platform, "age_s": round(now - platform_at, 1)}
                if platform_at is not None and now - platform_at <= 2.5 else {},
                occupancy if occupancy_at is not None and now - occupancy_at <= 5 else None,
                self.store.sites,
            )
            return reply((200, data) if data is not None else _error("monitoring_unavailable", 503))

        frontend = self._frontend_dir()
        no_store = {"Cache-Control": "no-store"}
        if frontend is not None:
            root = frontend

            @app.get("/{full_path:path}")
            async def serve(full_path: str):
                # Installed files may be symlinks (symlink-install), so check the
                # requested path itself rather than its resolved target.
                parts = Path(full_path).parts
                candidate = root.joinpath(*parts) if parts else root / "index.html"
                if ".." not in parts and not Path(full_path).is_absolute() and candidate.is_file():
                    if candidate.name == "index.html":
                        return FileResponse(str(candidate), headers=no_store)
                    return FileResponse(str(candidate))
                return FileResponse(str(root / "index.html"), headers=no_store)
        else:
            @app.get("/")
            async def missing():
                return JSONResponse({"message": "manager frontend not found"}, status_code=503)
        return app

    def _run_server(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        config = uvicorn.Config(self._build_app(), host=self.host, port=self.port,
                                log_level="warning", loop="asyncio")
        self._server = uvicorn.Server(config)
        try:
            loop.run_until_complete(self._server.serve())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"manager_ui server stopped: {exc}")
        finally:
            loop.close()

    def destroy_node(self) -> bool:
        if self._server is not None:
            self._server.should_exit = True
        if self._thread.is_alive():
            self._thread.join(timeout=3.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = UiManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
