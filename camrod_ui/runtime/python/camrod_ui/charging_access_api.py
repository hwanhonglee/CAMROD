"""HH_261102 - Explicit cargo-access actions, never motion on page access.

This is the existing public robot-LAN control boundary, not customer identity
authentication. A same-origin check only prevents cross-site browser commands.
"""

from urllib.parse import urlsplit

from fastapi import Request
from fastapi.responses import JSONResponse


def register_charging_access_routes(app, node):
    @app.get("/api/charging-access")
    def get_charging_access():
        return JSONResponse(node.charging_access_snapshot())

    @app.post("/ui/charging_access")
    def post_charging_access(request: Request, action: str = "", confirm: str = ""):
        # HH_261102 - Visiting/refreshing a charging page cannot authorize motion.
        origin = request.headers.get("origin")
        if origin:
            try:
                parsed = urlsplit(origin)
                matches = (parsed.scheme, parsed.netloc) == (
                    request.url.scheme, request.url.netloc)
            except ValueError:
                matches = False
            if not matches:
                return JSONResponse(
                    {"success": False, "error": "same_origin_required",
                     "message": "로봇 화면에서 직접 요청해 주세요."}, status_code=403)
        if action != "leave" or confirm != "true":
            return JSONResponse(
                {"success": False, "error": "explicit_confirmation_required",
                 "message": "이동 방향과 주변 안전을 확인한 후 실행해 주세요."},
                status_code=400)
        result = node.request_charging_access(action, confirm=True)
        return JSONResponse(result, status_code=200 if result.get("success") else 409)
