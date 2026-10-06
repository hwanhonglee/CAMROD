"""HH_261006 - Serve installed UI assets without disguising missing files as HTML."""

from pathlib import Path, PurePosixPath
from typing import Iterable, Optional

from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, JSONResponse, Response


NO_STORE_HEADERS = {
    "Cache-Control": "no-store, no-cache, must-revalidate",
    "Pragma": "no-cache",
}
ASSET_OR_API_ROOTS = frozenset({"assets", "models", "static", "api", "ui", "ws"})


def configured_frontend_path(value: str) -> Optional[Path]:
    # HH_261006 - An unset ROS parameter is not Path('.'); never expose the CWD.
    value = value.strip()
    return Path(value) if value else None


def find_frontend_directory(candidates: Iterable[Optional[Path]]) -> Optional[Path]:
    """Use the first complete build in the established configured/install/source order."""
    for candidate in candidates:
        if candidate is None:
            continue
        try:
            if candidate.is_dir() and (candidate / "index.html").is_file():
                return candidate.resolve()
        except (OSError, RuntimeError):
            continue
    return None


def register_frontend_routes(app: FastAPI, frontend_dir: Optional[Path]) -> None:
    """Register this catch-all after the backend's explicit API routes."""
    frontend_dir = find_frontend_directory([frontend_dir])

    @app.api_route("/{full_path:path}", methods=["GET", "HEAD"])
    async def serve_frontend(request: Request, full_path: str) -> Response:
        def error(message: str, status: int) -> Response:
            response = JSONResponse(
                {"detail": message}, status_code=status, headers=NO_STORE_HEADERS
            )
            if request.method == "HEAD":
                # HH_261006 - Asset health probes get GET's status/headers, not a body.
                response.body = b""
            return response

        if frontend_dir is None:
            return error("frontend not available; build and install camrod_ui", 503)

        path = PurePosixPath(full_path)
        if (path.is_absolute() or "\\" in full_path or "\x00" in full_path
                or any(part.startswith(".") for part in path.parts)):
            return error("Frontend path not found", 404)

        # HH_261006 - Reject traversal lexically, not by realpath containment:
        # colcon --symlink-install intentionally links published assets to source.
        try:
            candidate = (frontend_dir / full_path).resolve()
            if candidate.is_file():
                if candidate == (frontend_dir / "index.html").resolve():
                    headers = NO_STORE_HEADERS
                elif path.parts and path.parts[0] == "static":
                    headers = {"Cache-Control": "public, max-age=31536000, immutable"}
                else:
                    headers = {"Cache-Control": "no-cache"}
                return FileResponse(str(candidate), headers=headers, method=request.method)
        except (OSError, RuntimeError):
            return error("Frontend path not found", 404)

        # HH_261006 - GLB/PNG/JS failures must remain 404, never a successful HTML
        # response that hides incomplete deployment from browsers and health checks.
        if path.suffix or (path.parts and path.parts[0] in ASSET_OR_API_ROOTS):
            return error("Frontend asset or endpoint not found", 404)

        index = frontend_dir / "index.html"
        if not index.is_file():
            return error("frontend not available; build and install camrod_ui", 503)
        return FileResponse(str(index), headers=NO_STORE_HEADERS, method=request.method)
