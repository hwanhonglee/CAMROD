"""HJ_261120 - Serve Robot, Guest and manager UIs only to the robot's own Wi-Fi/LAN.

A request (HTTP or WebSocket) is accepted when the client address is the robot
itself (loopback) or lies inside a subnet of one of the robot's own IPv4
interfaces, i.e. the device joined the same Wi-Fi network. The subnet is read
from the interfaces at runtime, so the campsite network range does not need to
be configured. Extra ranges can be added with CAMROD_UI_ALLOWED_CIDRS
("10.0.0.0/8,192.168.5.0/24"); CAMROD_UI_LAN_GUARD=0 disables the check.

This limits who can reach the pages; it is not authentication. Anyone who
knows the Wi-Fi password is on the same network, so manager_ui still requires
its own login.
"""

from __future__ import annotations

import ipaddress
import json
import os
import subprocess
import threading
import time
from typing import Callable, Iterable, List, Optional

_REFRESH_S = 30.0

DENIED_HTML = (
    "<!doctype html><html lang='ko'><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<title>접속 불가</title></head>"
    "<body style=\"font-family:'Pretendard','Noto Sans KR',sans-serif;background:#f2ede3;"
    "color:#2b2420;display:grid;place-items:center;min-height:100vh;margin:0\">"
    "<div style='background:#fdfbf6;border:1.5px solid #ddd2c0;border-radius:20px;"
    "padding:32px 28px;max-width:360px;text-align:center'>"
    "<h1 style='font-size:1.3rem;margin:0 0 10px'>캠핑장 와이파이에서만 이용할 수 있어요</h1>"
    "<p style='color:#6e645a;line-height:1.6;margin:0'>캠핑장 와이파이에 연결한 뒤 다시 열어 주세요.<br>"
    "Please connect to the campsite Wi-Fi and try again.</p></div></body></html>"
)


def _interface_networks() -> List[ipaddress.IPv4Network]:
    try:
        out = subprocess.run(["ip", "-j", "-4", "addr", "show"], capture_output=True,
                             text=True, timeout=2.0, check=False).stdout
        entries = json.loads(out or "[]")
    except (OSError, ValueError, subprocess.SubprocessError):
        return []
    networks = []
    for entry in entries:
        for addr in entry.get("addr_info", []) or []:
            try:
                networks.append(ipaddress.ip_network(
                    f"{addr['local']}/{int(addr['prefixlen'])}", strict=False))
            except (KeyError, ValueError, TypeError):
                continue
    return networks


def _parse_cidrs(text: str) -> List[ipaddress._BaseNetwork]:
    networks = []
    for part in str(text or "").split(","):
        part = part.strip()
        if part:
            try:
                networks.append(ipaddress.ip_network(part, strict=False))
            except ValueError:
                continue
    return networks


class LanPolicy:
    def __init__(self, extra_cidrs: Iterable[str] = (),
                 interface_networks: Optional[Callable[[], list]] = None,
                 clock=time.monotonic) -> None:
        self.enabled = os.environ.get("CAMROD_UI_LAN_GUARD", "1").strip().lower() not in {"0", "false", "off", "no"}
        self.extra = _parse_cidrs(os.environ.get("CAMROD_UI_ALLOWED_CIDRS", ""))
        self.extra += _parse_cidrs(",".join(extra_cidrs))
        self._interfaces = interface_networks or _interface_networks
        self._clock = clock
        self._lock = threading.Lock()
        self._cached: list = []
        self._cached_at = -1e9

    def _local_networks(self) -> list:
        with self._lock:
            if self._clock() - self._cached_at > _REFRESH_S:
                self._cached = list(self._interfaces())
                self._cached_at = self._clock()
            return self._cached

    def allows(self, host: Optional[str]) -> bool:
        if not self.enabled:
            return True
        try:
            address = ipaddress.ip_address(str(host or "").split("%")[0])
        except ValueError:
            return False
        if isinstance(address, ipaddress.IPv6Address) and address.ipv4_mapped:
            address = address.ipv4_mapped
        if address.is_loopback:
            return True
        return any(address in network for network in self._local_networks() + self.extra
                   if network.version == address.version)


class LanGuardMiddleware:
    """Pure ASGI middleware so WebSocket upgrades are checked as well."""

    def __init__(self, app, policy: Optional[LanPolicy] = None, logger=None) -> None:
        self.app = app
        self.policy = policy or LanPolicy()
        self.logger = logger

    async def __call__(self, scope, receive, send):
        if scope.get("type") not in {"http", "websocket"}:
            return await self.app(scope, receive, send)
        client = scope.get("client") or (None, None)
        if self.policy.allows(client[0]):
            return await self.app(scope, receive, send)
        if self.logger is not None:
            try:
                self.logger.warning(f"lan guard rejected client {client[0]} path={scope.get('path')}")
            except Exception:  # noqa: BLE001
                pass
        if scope["type"] == "websocket":
            await send({"type": "websocket.close", "code": 1008})
            return
        body = DENIED_HTML.encode("utf-8")
        await send({"type": "http.response.start", "status": 403,
                    "headers": [(b"content-type", b"text/html; charset=utf-8"),
                                (b"content-length", str(len(body)).encode())]})
        await send({"type": "http.response.body", "body": body})


def install_lan_guard(app, logger=None, extra_cidrs: Iterable[str] = ()) -> LanPolicy:
    policy = LanPolicy(extra_cidrs)
    app.add_middleware(LanGuardMiddleware, policy=policy, logger=logger)
    return policy
