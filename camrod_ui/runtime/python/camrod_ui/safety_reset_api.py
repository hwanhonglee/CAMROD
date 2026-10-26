"""HH_261024 - Authenticated, explicit safety reset routes; no startup action."""

import os
from urllib.parse import urlsplit

from fastapi import Request
from fastapi.responses import JSONResponse

from camrod_ui.safety_reset import admin_authorized


def register_safety_reset_routes(app, node):
    def authorize(request):
        # HH_261024 - This new privileged action checks credentials server-side.
        # Existing diagnostic login remains unchanged. Use TLS/a trusted robot
        # LAN; Basic authentication is not encryption or Internet hardening.
        if not admin_authorized(
                request.headers.get("authorization", ""),
                os.environ.get("CAMROD_ADMIN_USER", "admin"),
                os.environ.get("CAMROD_ADMIN_PASSWORD", "1234")):
            return JSONResponse(
                {"success": False, "error": "admin_authentication_required",
                 "message": "관리자 인증이 필요합니다."}, status_code=401,
                headers={"WWW-Authenticate": 'Basic realm="CAMROD admin"'})
        origin = request.headers.get("origin")
        if origin:
            try:
                parsed = urlsplit(origin)
            except ValueError:
                return JSONResponse(
                    {"success": False, "error": "same_origin_required",
                     "message": "잘못된 요청 출처입니다."}, status_code=403)
            if (parsed.scheme, parsed.netloc) != (request.url.scheme, request.url.netloc):
                return JSONResponse(
                    {"success": False, "error": "same_origin_required",
                     "message": "로봇 관리자 화면에서 직접 요청해 주세요."}, status_code=403)
        return None

    @app.get("/api/admin/safety/reset")
    def get_admin_safety_reset(request: Request):
        denied = authorize(request)
        if denied is not None:
            return denied
        return JSONResponse(node.get_admin_safety_reset_status())

    @app.post("/api/admin/safety/reset")
    async def post_admin_safety_reset(request: Request):
        denied = authorize(request)
        if denied is not None:
            return denied
        try:
            payload = await request.json()
        except (ValueError, UnicodeError):
            payload = None
        # HH_261024 - Strings, numeric truthiness and a passive page visit are
        # never authority to terminate a mission or request recovery reset.
        if not isinstance(payload, dict) or payload.get("confirm") is not True:
            return JSONResponse(
                {"success": False, "error": "explicit_confirmation_required",
                 "message": "현재 미션 종료와 정지 유지에 명시적으로 동의해야 합니다."},
                status_code=400)
        result = await node.request_admin_safety_reset(confirm=True)
        code = 200 if result.get("success") else (
            503 if result.get("available") is False else 409)
        return JSONResponse(result, status_code=code)
