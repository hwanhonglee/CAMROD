"""HJ_261120 - Robot UI site-PIN check before a delivery/recall is sent.

The kiosk asks the camper for the 4-digit PIN that staff set in manager_ui.
Attempts are limited per site so the 10,000 combinations cannot be walked.
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

from fastapi import Request
from fastapi.responses import JSONResponse

from camrod_ui.site_access_store import valid_pin


def evaluate_site_pin(store, limiter, payload: Any) -> Tuple[int, Dict[str, Any]]:
    """Pure decision: (HTTP status, body) for one verify request."""
    if not isinstance(payload, dict):
        return 400, {"ok": False, "error": "invalid_request"}
    site = str(payload.get("site") or "").strip().upper()
    pin = payload.get("pin")
    if not site or not valid_pin(pin):
        return 400, {"ok": False, "error": "invalid_request"}
    if store is None:
        return 503, {"ok": False, "error": "pin_store_unavailable"}
    locked = limiter.remaining_lock_s(site)
    if locked > 0:
        return 429, {"ok": False, "error": "locked", "locked_s": int(locked + 0.999)}
    ok = bool(store.verify_pin(site, pin))
    limiter.record(site, ok)
    if ok:
        return 200, {"ok": True}
    locked = limiter.remaining_lock_s(site)
    if locked > 0:
        return 429, {"ok": False, "error": "locked", "locked_s": int(locked + 0.999)}
    failures = getattr(limiter, "_failures", {}).get(site, 0)
    return 403, {"ok": False, "error": "wrong_pin",
                 "remaining_attempts": max(0, int(limiter.max_failures) - int(failures))}


def register_site_pin_routes(app, node) -> None:
    @app.post("/ui/site_pin/verify")
    async def post_site_pin_verify(request: Request):
        try:
            payload: Optional[Any] = await request.json()
        except (ValueError, UnicodeError):
            payload = None
        status, body = evaluate_site_pin(
            getattr(node, "_site_access", None), node._site_pin_limiter, payload)
        return JSONResponse(body, status_code=status)
