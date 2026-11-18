"""HJ_261118 - Admin-only "bringup 종료" route.

The kiosk is started by double-clicking an icon (or the camrod-bringup user
unit), both of which end in ``ros2 launch camrod_bringup bringup.launch.py``.
This route finds that launch process among the backend's ancestors and sends
it SIGINT, the same signal as Ctrl-C, so launch shuts every node down in order
(the kiosk window included). It never publishes motion, goals or engage.
"""

import os
import signal
import threading
import time

from fastapi import Request
from fastapi.responses import JSONResponse

from camrod_ui.safety_reset import stationary_reset_blockers
from camrod_ui.safety_reset_api import admin_request_denied

# Give the HTTP response time to reach the kiosk before launch tears it down.
SHUTDOWN_SIGNAL_DELAY_S = 1.5

_BLOCKER_MESSAGES = {
    "mission_active": "미션이 진행 중이에요. 미션을 끝내거나 운행 정지 후 종료해 주세요.",
    "robot_not_stationary": "로봇이 움직이고 있어요. 완전히 멈춘 뒤 종료해 주세요.",
}


def _read_cmdline(proc_root, pid):
    try:
        with open(os.path.join(proc_root, str(pid), "cmdline"), "rb") as handle:
            return [part.decode("utf-8", "replace") for part in handle.read().split(b"\0") if part]
    except OSError:
        return []


def _read_ppid(proc_root, pid):
    try:
        with open(os.path.join(proc_root, str(pid), "stat"), "r", encoding="utf-8") as handle:
            stat = handle.read()
        # The command name may contain spaces/parentheses; fields follow the last ')'.
        return int(stat.rsplit(")", 1)[1].split()[1])
    except (OSError, ValueError, IndexError):
        return 0


def is_ros2_launch_cmdline(argv):
    """True for ``ros2 launch ...`` (python3 /opt/ros/.../bin/ros2 launch ...)."""
    for index, part in enumerate(argv[:-1]):
        if os.path.basename(part) == "ros2" and argv[index + 1] == "launch":
            return True
    return False


def find_launch_ancestor(start_pid=None, proc_root="/proc", max_depth=16):
    """Return the nearest ancestor PID running ``ros2 launch``, or 0."""
    pid = _read_ppid(proc_root, start_pid if start_pid is not None else os.getpid())
    for _ in range(max_depth):
        if pid <= 1:
            return 0
        if is_ros2_launch_cmdline(_read_cmdline(proc_root, pid)):
            return pid
        pid = _read_ppid(proc_root, pid)
    return 0


def bringup_shutdown_blockers(mission_active, motion_observation, now_ros_s, now_monotonic_s):
    """Only an active mission or measured motion blocks shutdown.

    Stale or missing motion feedback does not block: a broken platform link is
    exactly when an operator may need to stop the stack.
    """
    blockers = []
    if mission_active:
        blockers.append("mission_active")
    motion = stationary_reset_blockers(motion_observation, now_ros_s, now_monotonic_s)
    if "robot_not_stationary" in motion:
        blockers.append("robot_not_stationary")
    return blockers


def _status(node):
    mission_active = bool(node._mission_dispatch_snapshot().get("mission_dispatch_active"))
    blockers = bringup_shutdown_blockers(
        mission_active,
        getattr(node, "_safety_reset_motion_observation", None),
        node._now_s(), time.monotonic())
    launch_pid = find_launch_ancestor()
    return {
        "available": launch_pid > 0,
        "launch_pid": launch_pid,
        "blockers": blockers,
        "messages": [_BLOCKER_MESSAGES[b] for b in blockers],
    }


def register_bringup_shutdown_routes(app, node):
    lock = threading.Lock()
    state = {"requested": False}

    @app.get("/api/admin/bringup/shutdown")
    def get_bringup_shutdown(request: Request):
        denied = admin_request_denied(request)
        if denied is not None:
            return denied
        return JSONResponse({**_status(node), "requested": state["requested"]})

    @app.post("/api/admin/bringup/shutdown")
    async def post_bringup_shutdown(request: Request):
        denied = admin_request_denied(request)
        if denied is not None:
            return denied
        try:
            payload = await request.json()
        except (ValueError, UnicodeError):
            payload = None
        if not isinstance(payload, dict) or payload.get("confirm") is not True:
            return JSONResponse(
                {"success": False, "error": "explicit_confirmation_required",
                 "message": "종료 확인이 필요합니다."}, status_code=400)
        status = _status(node)
        if not status["available"]:
            return JSONResponse(
                {"success": False, "error": "launch_not_found", **status,
                 "message": "바탕화면 아이콘으로 실행된 상태가 아니라 종료할 수 없어요."},
                status_code=503)
        if status["blockers"]:
            return JSONResponse(
                {"success": False, "error": "shutdown_blocked", **status,
                 "message": " ".join(status["messages"])}, status_code=409)
        with lock:
            if not state["requested"]:
                state["requested"] = True
                pid = status["launch_pid"]
                node.get_logger().warning(
                    f"admin requested bringup shutdown; SIGINT to ros2 launch pid {pid}")

                def _send():
                    try:
                        os.kill(pid, signal.SIGINT)
                    except OSError as error:
                        state["requested"] = False
                        node.get_logger().error(f"bringup shutdown signal failed: {error}")

                timer = threading.Timer(SHUTDOWN_SIGNAL_DELAY_S, _send)
                timer.daemon = True
                timer.start()
        return JSONResponse({"success": True, **status, "requested": True,
                             "message": "소프트웨어를 종료합니다. 잠시 후 화면이 닫혀요."})
