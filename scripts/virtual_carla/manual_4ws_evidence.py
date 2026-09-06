#!/usr/bin/env python3
"""Collect and judge manual 4WS evidence through the visible Robot UI.

This runner never publishes ROS or calls a CARLA control/lifecycle API.  Its
only command authority is a real pointer/text/keyboard event delivered to the
single visible Robot UI page through local Chrome DevTools Protocol (CDP).
ROS and the patched CARLA wheel API are used by sibling observer processes
only.  The separation is intentional: a passing result proves the production
UI -> manual WebSocket -> CAMROD safety gate -> physical 4WS path.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import secrets
import subprocess
import sys
import tempfile
import time
from typing import Any, Callable, Iterable, Mapping, Sequence
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen


SCRIPT_DIR = Path(__file__).resolve().parent
SOURCE_ROOT = SCRIPT_DIR.parent.parent
BASE_PATH = SCRIPT_DIR / "camping_site_matrix.py"
BASE_SPEC = importlib.util.spec_from_file_location(
    "_camrod_manual_evidence_camping_site_matrix", BASE_PATH
)
if BASE_SPEC is None or BASE_SPEC.loader is None:
    raise RuntimeError(f"cannot load visible Robot UI CDP helper: {BASE_PATH}")
BASE = importlib.util.module_from_spec(BASE_SPEC)
sys.modules.setdefault(BASE_SPEC.name, BASE)
BASE_SPEC.loader.exec_module(BASE)

MatrixError = BASE.MatrixError

SCHEMA = "camrod.virtual_carla.manual_4ws_evidence.v1"
TRACE_SCHEMA = "camrod.virtual_carla.manual_4ws_ros_trace.v1"
SCENARIO_MANIFEST_SCHEMA = "camrod.virtual_carla.manual_4ws_scenario.v1"
COLLECTION_SCHEMA = "camrod.virtual_carla.manual_4ws_collection.v1"
EXPECTED_UI_TITLE = "Robot UI"
MANUAL_WS_PATH = "/ws/manual-drive"
EXPECTED_SCENARIOS = ("straight", "turn", "crab", "zero_turn")
WHEEL_NAMES = ("FL", "FR", "RL", "RR")
STEER_FIELDS = (
    "front_left_steer",
    "front_right_steer",
    "rear_left_steer",
    "rear_right_steer",
)
DRIVE_SIGN_FIELDS = (
    "front_left_drive_sign",
    "front_right_drive_sign",
    "rear_left_drive_sign",
    "rear_right_drive_sign",
)
TORQUE_FIELDS = (
    "front_left_wheel_torque_nm",
    "front_right_wheel_torque_nm",
    "rear_left_wheel_torque_nm",
    "rear_right_wheel_torque_nm",
)
CRITICAL_TOPICS = (
    "/control/manual_cmd_vel_ros",
    "/control/cmd_vel_ros",
    "/carla/ego_vehicle/extended_ackermann_cmd",
    "/carla/ego_vehicle/physical_four_wheel_cmd",
    "/carla/ego_vehicle/physical_four_wheel_status",
    "/carla/ego_vehicle/odometry",
    "/control/cmd_vel_safety_gate/status",
    "/planning/engage",
    "/platform/drive_enable",
    "/carla/ego_vehicle/collision",
)


@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    label: str
    mode: str
    mode_id: int
    positive_keys: tuple[str, ...]
    inverse_keys: tuple[str, ...]
    positive_axes: tuple[int, int, int]
    inverse_axes: tuple[int, int, int]


SCENARIOS: dict[str, ScenarioSpec] = {
    "straight": ScenarioSpec(
        "straight", "straight / reverse", "ackermann", 0,
        ("KeyW",), ("KeyS",), (1, 0, 0), (-1, 0, 0),
    ),
    "turn": ScenarioSpec(
        "turn", "dual Ackermann left / retrace", "ackermann", 0,
        ("KeyW", "KeyA"), ("KeyS", "KeyD"), (1, 1, 0), (-1, -1, 0),
    ),
    "crab": ScenarioSpec(
        "crab", "left / right crab", "crab", 1,
        ("KeyZ",), ("KeyC",), (0, 0, 1), (0, 0, -1),
    ),
    "zero_turn": ScenarioSpec(
        "zero_turn", "counter-clockwise / clockwise zero turn", "zero_turn", 2,
        ("KeyA",), ("KeyD",), (0, 1, 0), (0, -1, 0),
    ),
}

KEYS: dict[str, tuple[str, int]] = {
    "KeyW": ("w", 87),
    "KeyA": ("a", 65),
    "KeyS": ("s", 83),
    "KeyD": ("d", 68),
    "KeyZ": ("z", 90),
    "KeyC": ("c", 67),
    "Space": (" ", 32),
    "Escape": ("Escape", 27),
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise MatrixError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise MatrixError(f"{label} must be a finite number") from error
    if not math.isfinite(result):
        raise MatrixError(f"{label} must be a finite number")
    return result


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                return digest.hexdigest()
            digest.update(block)


def artifact(path: Path, root: Path | None = None) -> dict[str, Any]:
    if not path.is_file() or path.is_symlink() or path.stat().st_size <= 0:
        raise MatrixError(f"required evidence artifact is missing or empty: {path}")
    display = path.relative_to(root) if root is not None else path
    return {
        "path": str(display),
        "bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def write_json_exclusive(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise MatrixError(f"cannot read JSON evidence {path}: {error}") from None
    if not isinstance(value, dict):
        raise MatrixError(f"JSON evidence must be an object: {path}")
    return value


def require_output_directory(raw: str) -> Path:
    path = Path(raw).expanduser()
    if not path.is_absolute() or path == Path("/"):
        raise MatrixError(f"output directory must be absolute and not /: {raw}")
    if path.is_symlink() or not path.is_dir():
        raise MatrixError(f"output directory must be an existing regular directory: {path}")
    return path.resolve()


class ManualDriveBrowser(BASE.OperatorBrowserClient):
    """Visible-page-only administrator/manual-drive interaction boundary."""

    def bind_x11_window(
        self,
        window_id: str,
        display: str,
        xauthority: str = "",
    ) -> dict[str, Any]:
        """Prove that this exact CDP target owns the selected visible X11 window.

        A unique, short-lived title is written to the already connected page and
        must appear on the explicit X11 window id.  The original title is then
        restored and verified through both CDP and X11 before any drive action.
        This closes the otherwise ambiguous gap between a matching CDP URL and
        a separate same-title browser window selected by the pixel recorder.
        """
        if re.fullmatch(r"0x[0-9a-fA-F]+", str(window_id)) is None:
            raise MatrixError(f"X11 Robot UI window id must be hexadecimal: {window_id!r}")
        if not str(display).strip():
            raise MatrixError("X11 DISPLAY is required to bind the CDP target")
        if xauthority:
            authority = Path(xauthority).expanduser()
            if not authority.is_file() or not os.access(authority, os.R_OK):
                raise MatrixError(f"X11 authority is not a readable file: {authority}")

        environment = os.environ.copy()
        environment["DISPLAY"] = str(display)
        environment["LC_ALL"] = "C.UTF-8"
        if xauthority:
            environment["XAUTHORITY"] = str(Path(xauthority).expanduser())

        def inspect() -> str:
            try:
                completed = subprocess.run(
                    ["xwininfo", "-id", str(window_id)],
                    env=environment,
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=min(2.0, self.timeout_s),
                )
            except (OSError, subprocess.SubprocessError) as error:
                raise MatrixError(f"cannot inspect bound X11 Robot UI window: {error}") from None
            if completed.returncode != 0:
                raise MatrixError(
                    "cannot inspect bound X11 Robot UI window "
                    f"{window_id}: {completed.stderr.strip()}"
                )
            return completed.stdout

        original = self._evaluate("document.title")
        if str(original).strip() != EXPECTED_UI_TITLE:
            raise MatrixError(f"CDP Robot UI title changed before X11 binding: {original!r}")
        token = f"{EXPECTED_UI_TITLE} [camrod-bind-{secrets.token_hex(12)}]"
        observed_token = False
        try:
            changed = self._evaluate(
                "(() => {document.title=" + json.dumps(token) + "; return document.title;})()"
            )
            if changed != token:
                raise MatrixError("CDP target refused the temporary X11 binding title")
            deadline = time.monotonic() + min(3.0, self.timeout_s)
            while time.monotonic() < deadline:
                if token in inspect():
                    observed_token = True
                    break
                time.sleep(0.05)
            if not observed_token:
                raise MatrixError(
                    "selected X11 Robot UI window is not the connected CDP target: "
                    f"window_id={window_id} target_id={self._target.get('id')!r}"
                )
        finally:
            # Restoration is safety-relevant: a tokenized title must never be
            # mistaken for a normal evidence window after a failed binding.
            try:
                self._evaluate(
                    "(() => {document.title=" + json.dumps(str(original))
                    + "; return document.title;})()"
                )
            except Exception as error:
                raise MatrixError(
                    "failed to restore the Robot UI title after X11 binding"
                ) from error

        deadline = time.monotonic() + min(3.0, self.timeout_s)
        restored_x11 = False
        while time.monotonic() < deadline:
            info = inspect()
            if token not in info and EXPECTED_UI_TITLE in info and "Map State: IsViewable" in info:
                restored_x11 = True
                break
            time.sleep(0.05)
        restored_cdp = self._evaluate("document.title")
        if not restored_x11 or restored_cdp != EXPECTED_UI_TITLE:
            raise MatrixError(
                "Robot UI title/X11 visibility did not recover after target binding: "
                f"cdp={restored_cdp!r} x11_restored={restored_x11}"
            )
        self._bound_x11 = {
            "verified": True,
            "method": "temporary unique document.title observed on explicit X11 window",
            "target_id": str(self._target.get("id", "")),
            "window_id": str(window_id).lower(),
            "display": str(display),
            "title_restored": EXPECTED_UI_TITLE,
        }
        self._bound_x11_inspect = inspect
        return dict(self._bound_x11)

    def assert_motion_liveness(self, description: str) -> dict[str, Any]:
        value = self._evaluate(
            "(() => {"
            "const panel=document.querySelector('[data-ui=\"manual-drive-panel\"]');"
            "const state=document.querySelector('[data-ui=\"manual-drive-state\"]');"
            "return {title:document.title, href:document.location.href,"
            "ready:document.readyState, visibility:document.visibilityState,"
            "panelVisible:Boolean(panel && panel.getBoundingClientRect().width>0 && "
            "panel.getBoundingClientRect().height>0),"
            "connected:panel ? panel.dataset.connected === 'true' : false,"
            "armed:panel ? panel.dataset.armed === 'true' : false,"
            "stateText:state ? state.textContent.trim() : ''};})()"
        )
        accepted = (
            isinstance(value, Mapping)
            and value.get("ready") == "complete"
            and value.get("visibility") == "visible"
            and str(value.get("href", "")).rstrip("/") == self.operator_ui_url
            and value.get("title") == EXPECTED_UI_TITLE
            and value.get("panelVisible") is True
            and value.get("connected") is True
            and value.get("armed") is True
            and value.get("stateText") == "ARMED"
        )
        if not accepted:
            raise MatrixError(f"Robot UI liveness failed during {description}: {value!r}")
        binding = getattr(self, "_bound_x11", None)
        inspect = getattr(self, "_bound_x11_inspect", None)
        if not isinstance(binding, Mapping) or binding.get("verified") is not True or not callable(inspect):
            raise MatrixError("Robot UI motion refused without a verified CDP-to-X11 binding")
        info = inspect()
        if EXPECTED_UI_TITLE not in info or "Map State: IsViewable" not in info:
            raise MatrixError(
                f"bound Robot UI X11 window lost visibility during {description}"
            )
        return dict(value)

    def _connect_operator(self) -> None:
        request = Request(
            f"{self.debugging_url}/json",
            method="GET",
            headers={"Accept": "application/json"},
        )
        try:
            with urlopen(request, timeout=self.timeout_s) as response:  # nosec B310 - localhost validated by parent
                targets = json.loads(response.read().decode("utf-8"))
        except (HTTPError, URLError, TimeoutError, OSError, json.JSONDecodeError) as error:
            raise MatrixError(f"Robot UI browser CDP discovery failed: {error}") from None
        if not isinstance(targets, list):
            raise MatrixError("Robot UI CDP /json response must be a list")
        matches = [
            item for item in targets
            if isinstance(item, dict)
            and item.get("type") == "page"
            and str(item.get("url", "")).rstrip("/") == self.operator_ui_url
            and str(item.get("webSocketDebuggerUrl", "")).strip()
        ]
        if len(matches) != 1:
            raise MatrixError(
                "expected exactly one visible Robot UI CDP page at "
                f"{self.operator_ui_url}, found {len(matches)}"
            )
        self._target = dict(matches[0])
        websocket_url = str(self._target["webSocketDebuggerUrl"])
        parsed = urlparse(websocket_url)
        if (
            parsed.scheme != "ws"
            or parsed.hostname not in {"127.0.0.1", "localhost"}
            or parsed.username is not None
            or parsed.password is not None
            or parsed.query
            or parsed.fragment
        ):
            raise MatrixError(f"Robot UI returned non-local CDP URL: {websocket_url}")
        try:
            import websocket

            self._connection = websocket.create_connection(
                websocket_url,
                timeout=self.timeout_s,
                origin=self.operator_ui_url,
            )
        except Exception as error:
            raise MatrixError(f"Robot UI CDP WebSocket connection failed: {error}") from None
        ready = self._evaluate(
            "(() => ({title:document.title, href:document.location.href, "
            "ready:document.readyState, visibility:document.visibilityState, "
            "rootReady:Boolean(document.getElementById('root'))}))()"
        )
        if (
            not isinstance(ready, dict)
            or ready.get("ready") != "complete"
            or str(ready.get("href", "")).rstrip("/") != self.operator_ui_url
            or str(ready.get("title", "")).strip() != EXPECTED_UI_TITLE
            or ready.get("visibility") != "visible"
            or ready.get("rootReady") is not True
        ):
            raise MatrixError(f"visible Robot UI page is not ready: {ready!r}")

    def _install_transport_probe(self) -> None:
        value = self._evaluate(
            "(() => {"
            "if (!window.__camrodManualEvidenceInstalled) {"
            "const originalSend = WebSocket.prototype.send;"
            "WebSocket.prototype.send = function(payload) {"
            "const store = window.__camrodManualEvidence;"
            "let pathname = ''; try { pathname = new URL(this.url, location.href).pathname; } catch (_) {}"
            "if (store && Array.isArray(store.websocket) && pathname === '/ws/manual-drive') {"
            "let frame = null; try { frame = JSON.parse(String(payload)); } catch (_) {}"
            "store.websocket.push({sent_at_unix_ms:Date.now(), "
            "sent_at_performance_ms:performance.now(), url:String(this.url), "
            "payload:String(payload), frame:frame});"
            "}"
            "return originalSend.apply(this, arguments);"
            "};"
            "window.__camrodManualEvidenceInstalled = true;"
            "}"
            "window.__camrodManualEvidence = {websocket:[]};"
            "return {accepted:true, installed:true};"
            "})()"
        )
        self._accepted(value, "manual-drive transport probe installation")

    def clear_probe(self) -> None:
        value = self._evaluate(
            "(() => {window.__camrodManualEvidence = {websocket:[]}; "
            "return {accepted:true};})()"
        )
        self._accepted(value, "manual-drive transport probe reset")

    def probe(self) -> dict[str, Any]:
        value = self._evaluate(
            "(() => {const value=window.__camrodManualEvidence; "
            "return value ? JSON.parse(JSON.stringify(value)) : null;})()"
        )
        return dict(value) if isinstance(value, Mapping) else {}

    def snapshot(self) -> dict[str, Any]:
        value = self._evaluate(
            "(() => {"
            "const panel=document.querySelector('[data-ui=\"manual-drive-panel\"]');"
            "const state=document.querySelector('[data-ui=\"manual-drive-state\"]');"
            "const toggle=document.querySelector('[data-ui=\"manual-drive-toggle\"]');"
            "const scale=document.querySelector('[data-ui=\"manual-drive-scale\"]');"
            "return {"
            "panelVisible:Boolean(panel && panel.getBoundingClientRect().width>0 && panel.getBoundingClientRect().height>0),"
            "connected:panel ? panel.dataset.connected === 'true' : false,"
            "armed:panel ? panel.dataset.armed === 'true' : false,"
            "mode:panel ? panel.dataset.mode : '',"
            "linearLimitMps:panel ? Number(panel.dataset.linearLimitMps) : null,"
            "lateralLimitMps:panel ? Number(panel.dataset.lateralLimitMps) : null,"
            "angularLimitRadps:panel ? Number(panel.dataset.angularLimitRadps) : null,"
            "stateText:state ? state.textContent.trim() : '',"
            "expanded:toggle ? toggle.getAttribute('aria-expanded') === 'true' : false,"
            "scale:scale ? Number(scale.value) : null,"
            "loginVisible:Boolean(document.querySelector('[data-ui=\"operator-admin-login-modal\"]'))," 
            "adminVisible:Boolean(document.querySelector('[data-ui=\"operator-admin-exit\"]'))"
            "};})()"
        )
        return dict(value) if isinstance(value, Mapping) else {}

    def scroll_pointer_target_into_view(
        self, selector: str, description: str
    ) -> dict[str, Any]:
        """Expose one enabled element and prove its center is a real hit target.

        The expanded manual-drive dock can place the ARM button a few pixels
        below Chrome's content viewport even though the element itself has a
        non-zero layout rectangle.  CDP pointer coordinates outside that
        viewport are silently ignored.  Scrolling is only viewport navigation;
        the subsequent control action remains a real CDP mouse event.
        """
        literal = json.dumps(selector)
        value = self._evaluate(
            "(() => {"
            f"const selector = {literal};"
            "const matches = Array.from(document.querySelectorAll(selector));"
            "if (matches.length !== 1) return {accepted:false, count:matches.length};"
            "const element = matches[0];"
            "element.scrollIntoView({block:'center', inline:'nearest'});"
            "const rect = element.getBoundingClientRect();"
            "const x = rect.left + rect.width / 2;"
            "const y = rect.top + rect.height / 2;"
            "const inViewport = x >= 0 && y >= 0 && "
            "x < window.innerWidth && y < window.innerHeight;"
            "const hit = inViewport ? document.elementFromPoint(x, y) : null;"
            "const accepted = Boolean(!element.disabled && inViewport && hit && "
            "(hit === element || element.contains(hit)));"
            "return {accepted:accepted, count:1, disabled:Boolean(element.disabled),"
            "x:x, y:y, viewportWidth:window.innerWidth,"
            "viewportHeight:window.innerHeight,"
            "hitTag:hit ? hit.tagName : '',"
            "hitUi:hit ? hit.getAttribute('data-ui') : ''};"
            "})()"
        )
        self._accepted(value, f"visible pointer target for {description}")
        record = {
            "stage": f"scroll {description} into visible viewport",
            "selector": selector,
            "x": round(float(value["x"]), 3),
            "y": round(float(value["y"]), 3),
            "viewport_width": int(value["viewportWidth"]),
            "viewport_height": int(value["viewportHeight"]),
            "hit_tag": str(value.get("hitTag", "")),
            "hit_ui": str(value.get("hitUi", "")),
            "transport": "CDP.Runtime.evaluate scrollIntoView/read-only hit test",
            "at_unix_ns": time.time_ns(),
        }
        self._interactions.append(record)
        return record

    def wait_snapshot(
        self,
        predicate: Callable[[Mapping[str, Any]], bool],
        description: str,
        timeout_s: float | None = None,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + (self.timeout_s if timeout_s is None else timeout_s)
        last: dict[str, Any] = {}
        while time.monotonic() < deadline:
            last = self.snapshot()
            if predicate(last):
                return last
            time.sleep(0.05)
        raise MatrixError(f"Robot UI did not reach {description}: {last!r}")

    def long_press(self, selector: str, duration_s: float = 1.65) -> dict[str, Any]:
        element = self._wait_element(selector, "administrator entry")
        x, y = float(element["x"]), float(element["y"])
        self._call("Input.dispatchMouseEvent", {"type": "mouseMoved", "x": x, "y": y})
        self._call("Input.dispatchMouseEvent", {
            "type": "mousePressed", "x": x, "y": y,
            "button": "left", "buttons": 1, "clickCount": 1,
        })
        try:
            time.sleep(duration_s)
        finally:
            self._call("Input.dispatchMouseEvent", {
                "type": "mouseReleased", "x": x, "y": y,
                "button": "left", "buttons": 0, "clickCount": 1,
            })
        record = {
            "stage": "administrator long press",
            "selector": selector,
            "duration_s": duration_s,
            "x": round(x, 3),
            "y": round(y, 3),
            "transport": "CDP.Input.dispatchMouseEvent",
            "at_unix_ns": time.time_ns(),
        }
        self._interactions.append(record)
        return record

    def input_text(self, selector: str, text: str, description: str) -> dict[str, Any]:
        element = self._wait_element(selector, description)
        if element.get("value") not in (None, ""):
            raise MatrixError(f"{description} was not empty before input")
        self._click(selector, f"focus {description}")
        self._call("Input.insertText", {"text": text})
        deadline = time.monotonic() + self.timeout_s
        last: Any = None
        while time.monotonic() < deadline:
            last = self._element(selector).get("value")
            if last == text:
                break
            time.sleep(0.05)
        if last != text:
            raise MatrixError(f"{description} did not receive the expected text")
        record = {
            "stage": description,
            "selector": selector,
            "text_length": len(text),
            "secret_redacted": "password" in description,
            "transport": "CDP.Input.insertText",
            "at_unix_ns": time.time_ns(),
        }
        self._interactions.append(record)
        return record

    def dispatch_key(self, code: str, pressed: bool) -> dict[str, Any]:
        if code not in KEYS:
            raise MatrixError(f"unsupported manual evidence key: {code}")
        key, virtual_key = KEYS[code]
        params: dict[str, Any] = {
            "type": "rawKeyDown" if pressed else "keyUp",
            "key": key,
            "code": code,
            "windowsVirtualKeyCode": virtual_key,
            "nativeVirtualKeyCode": virtual_key,
            "modifiers": 0,
        }
        if pressed and code not in {"Escape"}:
            params.update({"text": key, "unmodifiedText": key, "autoRepeat": False})
        self._call("Input.dispatchKeyEvent", params)
        record = {
            "stage": "key_down" if pressed else "key_up",
            "code": code,
            "at_unix_ns": time.time_ns(),
            "transport": "CDP.Input.dispatchKeyEvent",
        }
        self._interactions.append(record)
        return record

    def key_press(self, code: str) -> tuple[dict[str, Any], dict[str, Any]]:
        down = self.dispatch_key(code, True)
        time.sleep(0.03)
        up = self.dispatch_key(code, False)
        return down, up

    def ensure_fresh_administrator_session(self) -> dict[str, Any]:
        """Exit stale diagnostics, then prove a new visible admin login."""
        self._interactions = []
        current = self.snapshot()
        if current.get("armed"):
            self.key_press("Space")
            self.key_press("Escape")
            self.wait_snapshot(lambda item: not item.get("armed"), "STANDBY before re-login")
        if current.get("adminVisible"):
            self._click('[data-ui="operator-admin-exit"]', "exit stale administrator session")
        elif current.get("loginVisible"):
            self._click('[data-ui="operator-admin-login-cancel"]', "cancel stale administrator login")
        self._wait_element('[data-ui="operator-admin-entry"]', "administrator entry")
        self.long_press('[data-ui="operator-admin-entry"]')
        self._wait_element('[data-ui="operator-admin-id"]', "administrator id")
        self.input_text('[data-ui="operator-admin-id"]', "admin", "administrator id")
        self.input_text(
            '[data-ui="operator-admin-password"]', "1234", "administrator password"
        )
        self._click('[data-ui="operator-admin-login"]', "submit administrator login")
        self._wait_element(
            '[data-ui="operator-diagnostic-tab-camera"]', "administrator camera tab"
        )
        self._click(
            '[data-ui="operator-diagnostic-tab-camera"]', "open administrator camera tab"
        )
        panel = self.wait_snapshot(
            lambda item: item.get("panelVisible") and item.get("connected"),
            "connected manual-drive panel",
            timeout_s=max(15.0, self.timeout_s),
        )
        if not panel.get("expanded"):
            self._click('[data-ui="manual-drive-toggle"]', "expand manual-drive controls")
            panel = self.wait_snapshot(
                lambda item: item.get("expanded") is True,
                "expanded manual-drive controls",
            )
        return {
            "authenticated_via_visible_form": True,
            "credentials": {"id": "admin", "password": "REDACTED"},
            "snapshot": panel,
            "interactions": list(self._interactions),
        }

    def arm(self) -> dict[str, Any]:
        snapshot = self.snapshot()
        if not snapshot.get("panelVisible") or not snapshot.get("connected"):
            raise MatrixError(f"manual-drive panel is not connected: {snapshot!r}")
        if snapshot.get("armed"):
            raise MatrixError("manual-drive panel was unexpectedly already ARMED")
        self.scroll_pointer_target_into_view(
            '[data-ui="manual-drive-arm"]', "ARM manual drive"
        )
        self.clear_probe()
        self._click('[data-ui="manual-drive-arm"]', "ARM manual drive")
        armed = self.wait_snapshot(
            lambda item: item.get("armed") and item.get("stateText") == "ARMED",
            "ARMED manual-drive state",
        )
        frames = self.wait_for_frames(
            lambda frame: frame.get("type") == "arm", "ARM WebSocket frame"
        )
        return {"snapshot": armed, "frames": frames, "interactions": list(self._interactions)}

    def wait_for_frames(
        self,
        predicate: Callable[[Mapping[str, Any]], bool],
        description: str,
    ) -> list[dict[str, Any]]:
        deadline = time.monotonic() + self.timeout_s
        last: list[dict[str, Any]] = []
        while time.monotonic() < deadline:
            raw = self.probe().get("websocket", [])
            last = [dict(item) for item in raw if isinstance(item, Mapping)]
            if any(
                isinstance(item.get("frame"), Mapping) and predicate(item["frame"])
                for item in last
            ):
                return last
            time.sleep(0.05)
        raise MatrixError(f"Robot UI did not emit {description}: {last!r}")

    def zero(self) -> None:
        self.key_press("Space")

    def disarm(self) -> dict[str, Any]:
        self.key_press("Space")
        self.key_press("Escape")
        snapshot = self.wait_snapshot(
            lambda item: item.get("panelVisible") and not item.get("armed"),
            "disarmed manual-drive state",
        )
        frames = self.wait_for_frames(
            lambda frame: frame.get("type") == "disarm", "DISARM WebSocket frame"
        )
        return {"snapshot": snapshot, "frames": frames, "interactions": list(self._interactions)}

    def fail_safe(self, held_keys: Iterable[str] = ()) -> bool:
        for code in reversed(tuple(held_keys)):
            try:
                self.dispatch_key(code, False)
            except Exception:
                pass
        for code in ("Space", "Escape"):
            try:
                self.key_press(code)
            except Exception:
                pass
        try:
            snapshot = self.snapshot()
        except Exception:
            return False
        return (
            snapshot.get("panelVisible") is True
            and snapshot.get("armed") is False
            and snapshot.get("stateText") in {"STANDBY", "OFFLINE"}
        )


def _message_stamp(message: Any) -> dict[str, int] | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if isinstance(sec, int) and isinstance(nanosec, int) and sec >= 0 and 0 <= nanosec < 1_000_000_000:
        return {"sec": sec, "nanosec": nanosec}
    return None


def _vector(value: Any) -> dict[str, float]:
    return {axis: finite(getattr(value, axis, math.nan), f"vector.{axis}") for axis in ("x", "y", "z")}


def decode_twist(message: Any) -> dict[str, Any]:
    return {"linear": _vector(message.linear), "angular": _vector(message.angular)}


def quaternion_yaw_degrees(orientation: Any) -> float:
    x = finite(getattr(orientation, "x", math.nan), "orientation.x")
    y = finite(getattr(orientation, "y", math.nan), "orientation.y")
    z = finite(getattr(orientation, "z", math.nan), "orientation.z")
    w = finite(getattr(orientation, "w", math.nan), "orientation.w")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-9:
        raise MatrixError("odometry orientation quaternion is zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(yaw)


def decode_odometry(message: Any) -> dict[str, Any]:
    pose = message.pose.pose
    twist = message.twist.twist
    return {
        "stamp": _message_stamp(message),
        "position_m": _vector(pose.position),
        "yaw_degrees": quaternion_yaw_degrees(pose.orientation),
        "linear_velocity_mps": _vector(twist.linear),
        "angular_velocity_radps": _vector(twist.angular),
    }


def decode_extended(message: Any) -> dict[str, Any]:
    return {
        "stamp": _message_stamp(message),
        "drive_mode": int(message.drive_mode.mode),
        "speed_mps": finite(message.speed, "extended.speed"),
        "steering_angle_rad": finite(message.steering_angle, "extended.steering_angle"),
        "steering_angle_velocity_radps": finite(
            message.steering_angle_velocity, "extended.steering_angle_velocity"
        ),
        "crab_angle_rad": finite(message.crab_angle, "extended.crab_angle"),
        "rear_steering_angle_rad": finite(
            message.rear_steering_angle, "extended.rear_steering_angle"
        ),
        "yaw_rate_cmd_radps": finite(message.yaw_rate_cmd, "extended.yaw_rate_cmd"),
        "recovery_breakaway_authorized": bool(message.recovery_breakaway_authorized),
    }


def decode_physical_control(message: Any) -> dict[str, Any]:
    result: dict[str, Any] = {
        "stamp": _message_stamp(message),
        "sequence": int(message.sequence),
        "drive_mode": int(message.drive_mode.mode),
        "api_version": str(message.api_version),
        "motion_backend": str(message.motion_backend),
        "physical_gate_accepted": bool(message.physical_gate_accepted),
        "physical_manifest_sha256": str(message.physical_manifest_sha256),
        "requires_independent_wheel_drive": bool(message.requires_independent_wheel_drive),
        "independent_wheel_torque_active": bool(message.independent_wheel_torque_active),
        "reset": bool(message.reset),
    }
    result["steer_radians"] = [finite(getattr(message, name), name) for name in STEER_FIELDS]
    result["drive_signs"] = [int(getattr(message, name)) for name in DRIVE_SIGN_FIELDS]
    result["speed_scales"] = [
        finite(getattr(message, name), name)
        for name in (
            "front_left_speed_scale", "front_right_speed_scale",
            "rear_left_speed_scale", "rear_right_speed_scale",
        )
    ]
    result["torques_nm"] = [finite(getattr(message, name), name) for name in TORQUE_FIELDS]
    return result


def decode_physical_status(message: Any) -> dict[str, Any]:
    fields = (
        "ready", "api_version", "motion_backend", "physical_gate_accepted",
        "physical_manifest_sha256", "physx_substep_control_verified",
        "physx_substep_threshold_mps", "physx_substep_low_count",
        "physx_substep_high_count", "production_authorization_sha256",
        "ros_integration_sha256", "actor_id", "last_applied_sequence",
        "independent_wheel_drive_available", "wheel_torque_safety_cap_nm",
        "carla_python_origin", "imported_libcarla_path",
        "imported_libcarla_sha256", "reason",
    )
    result = {name: getattr(message, name) for name in fields}
    result["stamp"] = _message_stamp(message)
    return result


def decode_module_state(message: Any) -> dict[str, Any]:
    return {
        "level": int(getattr(message, "level", -1)),
        "operating_state": str(getattr(message, "operating_state", "")),
        "message": str(getattr(message, "message", "")),
    }


class RosTraceObserver:
    """ROS 2 subscriber-only observer used while the page drives the robot."""

    def __init__(self, output: Path, role_name: str = "ego_vehicle") -> None:
        try:
            import rclpy
            from rclpy.qos import QoSProfile, QoSReliabilityPolicy
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
            from avg_msgs.msg import AvgBool, ModuleState
            from carla_extended_ackermann_msgs.msg import (
                ExtendedAckermannDrive,
                PhysicalFourWheelControl,
                PhysicalFourWheelStatus,
            )
        except (ImportError, ModuleNotFoundError) as error:
            raise MatrixError(f"live ROS interfaces unavailable: {error}") from None
        if output.exists() or output.is_symlink():
            raise MatrixError(f"refusing to overwrite ROS trace: {output}")
        self.rclpy = rclpy
        self.output = output
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.stream = output.open("x", encoding="utf-8", newline="\n")
        self.started_ns = time.time_ns()
        self.records: list[dict[str, Any]] = []
        self.counts = {topic: 0 for topic in CRITICAL_TOPICS}
        self.last_received_ns = {topic: 0 for topic in CRITICAL_TOPICS}
        self.publisher_counts: dict[str, int] = {}
        self.fatal_error = ""
        self.expected_actor_id: int | None = None
        self.closed = False
        self._write({
            "schema": TRACE_SCHEMA,
            "record_type": "header",
            "created_at_utc": utc_now(),
            "read_only": True,
            "publisher_created": False,
            "topics": list(CRITICAL_TOPICS),
        })
        rclpy.init(args=[])
        self.node = rclpy.create_node("camrod_manual_4ws_evidence_observer")
        reliable = QoSProfile(depth=100, reliability=QoSReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=100, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._subscriptions: list[Any] = []

        def subscribe(message_type: Any, topic: str, decoder: Callable[[Any], dict[str, Any]], qos: Any) -> None:
            self._subscriptions.append(self.node.create_subscription(
                message_type, topic,
                lambda message, selected_topic=topic, selected_decoder=decoder: self._callback(
                    selected_topic, selected_decoder, message
                ),
                qos,
            ))

        subscribe(Twist, "/control/manual_cmd_vel_ros", decode_twist, reliable)
        subscribe(Twist, "/control/cmd_vel_ros", decode_twist, reliable)
        subscribe(
            ExtendedAckermannDrive,
            f"/carla/{role_name}/extended_ackermann_cmd",
            decode_extended,
            reliable,
        )
        subscribe(
            PhysicalFourWheelControl,
            f"/carla/{role_name}/physical_four_wheel_cmd",
            decode_physical_control,
            reliable,
        )
        subscribe(
            PhysicalFourWheelStatus,
            f"/carla/{role_name}/physical_four_wheel_status",
            decode_physical_status,
            reliable,
        )
        subscribe(Odometry, f"/carla/{role_name}/odometry", decode_odometry, best_effort)
        subscribe(ModuleState, "/control/cmd_vel_safety_gate/status", decode_module_state, reliable)
        # CAMROD publishes both safety-arm boundaries as avg_msgs/AvgBool.
        # Subscribing with std_msgs/Bool leaves a same-named DDS endpoint in
        # the graph but can never receive a sample because the ROS types do
        # not match, which made a real visible UI ARM look like a failure.
        subscribe(
            AvgBool,
            "/planning/engage",
            lambda message: {"data": bool(message.data)},
            reliable,
        )
        subscribe(
            AvgBool,
            "/platform/drive_enable",
            lambda message: {"data": bool(message.data)},
            reliable,
        )
        try:
            from carla_msgs.msg import CarlaCollisionEvent
        except (ImportError, ModuleNotFoundError):
            CarlaCollisionEvent = None
        if CarlaCollisionEvent is not None:
            subscribe(
                CarlaCollisionEvent,
                f"/carla/{role_name}/collision",
                lambda message: BASE.collision_event_record(message),
                reliable,
            )

    def _write(self, value: Mapping[str, Any]) -> None:
        self.stream.write(json.dumps(
            value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False
        ) + "\n")
        self.stream.flush()

    def _callback(
        self, topic: str, decoder: Callable[[Any], dict[str, Any]], message: Any
    ) -> None:
        received_ns = time.time_ns()
        try:
            if topic.endswith("/physical_four_wheel_status"):
                actor_id = BASE.validate_physical_status(message, self.expected_actor_id)
                if self.expected_actor_id is None:
                    self.expected_actor_id = actor_id
            decoded = decoder(message)
            record = {
                "schema": TRACE_SCHEMA,
                "record_type": "sample",
                "topic": topic,
                "received_at_unix_ns": received_ns,
                "elapsed_wall_seconds": (received_ns - self.started_ns) / 1.0e9,
                "message": decoded,
            }
            self.records.append(record)
            self.counts[topic] = self.counts.get(topic, 0) + 1
            self.last_received_ns[topic] = received_ns
            self._write(record)
        except Exception as error:
            self.fatal_error = f"{topic}: {error}"

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_s)
        while time.monotonic() < deadline:
            self.rclpy.spin_once(
                self.node,
                timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())),
            )
            if self.fatal_error:
                raise MatrixError(f"ROS observation failed closed: {self.fatal_error}")

    def wait_ready(self, timeout_s: float, require_odometry: bool = True) -> None:
        deadline = time.monotonic() + timeout_s
        physical = "/carla/ego_vehicle/physical_four_wheel_status"
        odometry = "/carla/ego_vehicle/odometry"
        while time.monotonic() < deadline:
            self.spin_for(0.1)
            physical_ready = self.counts.get(physical, 0) >= 1
            odometry_ready = self.counts.get(odometry, 0) >= 2
            if physical_ready and (odometry_ready or not require_odometry):
                # Allow graph discovery for command/collision publishers that
                # may not have emitted a sample during the readiness window.
                self.spin_for(0.25)
                self.publisher_counts = {
                    topic: int(self.node.count_publishers(topic)) for topic in CRITICAL_TOPICS
                }
                return
        raise MatrixError(
            "timed out waiting for physical status/odometry: "
            f"physical={self.counts.get(physical, 0)} odometry={self.counts.get(odometry, 0)}"
        )

    def assert_motion_liveness(self, maximum_age_s: float = 2.0) -> dict[str, Any]:
        """Fail closed if the subscriber graph or live control feedback disappears."""
        if self.closed or not self.rclpy.ok():
            raise MatrixError("ROS observation context is not live")
        if self.fatal_error:
            raise MatrixError(f"ROS observation failed closed: {self.fatal_error}")
        required = (
            "/control/manual_cmd_vel_ros",
            "/control/cmd_vel_ros",
            "/carla/ego_vehicle/extended_ackermann_cmd",
            "/carla/ego_vehicle/physical_four_wheel_cmd",
            "/carla/ego_vehicle/physical_four_wheel_status",
            "/carla/ego_vehicle/odometry",
        )
        publisher_counts = {
            topic: int(self.node.count_publishers(topic)) for topic in required
        }
        missing = [topic for topic, count in publisher_counts.items() if count < 1]
        if missing:
            raise MatrixError(
                "motion refused because critical ROS publishers disappeared: "
                + ", ".join(missing)
            )
        now_ns = time.time_ns()
        stale = []
        ages: dict[str, float] = {}
        for topic in required:
            received_ns = int(self.last_received_ns.get(topic, 0))
            age_s = (now_ns - received_ns) / 1.0e9 if received_ns > 0 else math.inf
            ages[topic] = age_s
            if not math.isfinite(age_s) or age_s > maximum_age_s:
                stale.append(f"{topic}={age_s:.3f}s")
        if stale:
            raise MatrixError(
                "motion refused because critical ROS feedback is stale: "
                + ", ".join(stale)
            )
        return {"publisher_counts": publisher_counts, "age_seconds": ages}

    def close(self, status: str = "COMPLETED", error: str = "") -> None:
        if self.closed:
            return
        self.closed = True
        try:
            self._write({
                "schema": TRACE_SCHEMA,
                "record_type": "footer",
                "completed_at_utc": utc_now(),
                "status": status,
                "error": error or self.fatal_error or None,
                "sample_counts": self.counts,
                "publisher_counts": self.publisher_counts,
                "expected_actor_id": self.expected_actor_id,
            })
            os.fsync(self.stream.fileno())
        finally:
            self.stream.close()
            try:
                self.node.destroy_node()
            finally:
                if self.rclpy.ok():
                    self.rclpy.shutdown()


def trace_samples(records: Sequence[Mapping[str, Any]], topic: str) -> list[dict[str, Any]]:
    return [
        dict(record) for record in records
        if record.get("record_type") == "sample" and record.get("topic") == topic
    ]


def samples_in_window(
    records: Sequence[Mapping[str, Any]], topic: str, start_ns: int, end_ns: int
) -> list[dict[str, Any]]:
    return [
        record for record in trace_samples(records, topic)
        if start_ns <= int(record.get("received_at_unix_ns", -1)) <= end_ns
    ]


def twist_values(record: Mapping[str, Any]) -> tuple[float, float, float]:
    message = record["message"]
    return (
        float(message["linear"]["x"]),
        float(message["linear"]["y"]),
        float(message["angular"]["z"]),
    )


def is_zero_twist(record: Mapping[str, Any], tolerance: float = 1.0e-3) -> bool:
    return all(abs(value) <= tolerance for value in twist_values(record))


def semantic_twist_match(
    values: tuple[float, float, float], axes: tuple[int, int, int], minimum: float = 0.03
) -> bool:
    for value, direction in zip(values, axes):
        if direction == 0 and abs(value) > 0.02:
            return False
        if direction > 0 and value < minimum:
            return False
        if direction < 0 and value > -minimum:
            return False
    return True


def ui_axes_to_twist_axes(
    axes: tuple[int, int, int],
) -> tuple[int, int, int]:
    """Map UI (forward, turn, crab) into ROS (x, y, yaw) ordering."""
    forward, turn, crab = axes
    return forward, crab, turn


def validate_ui_frames(
    records: Sequence[Mapping[str, Any]],
    spec: ScenarioSpec,
    phases: Mapping[str, Mapping[str, int]],
    hold_s: float,
) -> dict[str, Any]:
    frames: list[tuple[int, dict[str, Any]]] = []
    for record in records:
        frame = record.get("frame")
        if not isinstance(frame, dict):
            raise MatrixError("manual WebSocket probe captured a non-object frame")
        timestamp_ns = int(finite(record.get("sent_at_unix_ms"), "frame timestamp") * 1.0e6)
        frame_type = frame.get("type")
        required = {"type", "seq"} if frame_type in {"arm", "disarm"} else {
            "type", "seq", "mode", "forward", "turn", "crab", "scale"
        }
        if set(frame) != required:
            raise MatrixError(
                f"manual WebSocket {frame_type!r} frame fields are not exact: {sorted(frame)}"
            )
        if isinstance(frame.get("seq"), bool) or not isinstance(frame.get("seq"), int):
            raise MatrixError("manual WebSocket seq must be an exact integer")
        if frame_type == "drive":
            if frame.get("mode") not in {"ackermann", "crab", "zero_turn"}:
                raise MatrixError(f"invalid manual-drive mode: {frame.get('mode')!r}")
            for axis in ("forward", "turn", "crab"):
                if isinstance(frame.get(axis), bool) or frame.get(axis) not in {-1, 0, 1}:
                    raise MatrixError(f"invalid manual-drive direction {axis}")
            scale = finite(frame.get("scale"), "manual scale")
            if not 0.1 <= scale <= 1.0:
                raise MatrixError("manual scale is outside [0.1, 1.0]")
        frames.append((timestamp_ns, frame))
    sequences = [frame["seq"] for _, frame in frames]
    if not sequences or any(right <= left for left, right in zip(sequences, sequences[1:])):
        raise MatrixError("manual WebSocket sequence is empty or not strictly increasing")

    def expected_count(name: str, axes: tuple[int, int, int]) -> tuple[int, float]:
        phase = phases[name]
        matched = [
            frame for stamp, frame in frames
            if int(phase["start_ns"]) <= stamp <= int(phase["end_ns"])
            and frame.get("type") == "drive"
            and frame.get("mode") == spec.mode
            and tuple(frame.get(axis) for axis in ("forward", "turn", "crab")) == axes
        ]
        rate = len(matched) / max(hold_s, 1.0e-9)
        if rate < 5.0:
            raise MatrixError(f"{name} UI drive frame rate is too low: {rate:.2f} Hz")
        return len(matched), rate

    positive_count, positive_rate = expected_count("positive", spec.positive_axes)
    inverse_count, inverse_rate = expected_count("inverse", spec.inverse_axes)
    zero_latencies: list[float] = []
    for name in ("positive", "inverse"):
        released_ns = int(phases[name]["release_ns"])
        zero = next((
            stamp for stamp, frame in frames
            if stamp >= released_ns
            and frame.get("type") == "drive"
            and all(frame.get(axis) == 0 for axis in ("forward", "turn", "crab"))
        ), None)
        if zero is None or zero - released_ns > 350_000_000:
            raise MatrixError(f"{name} UI exact zero was not observed within 0.35 s")
        zero_latencies.append((zero - released_ns) / 1.0e9)
    return {
        "strict_sequence": True,
        "positive_matching_frames": positive_count,
        "inverse_matching_frames": inverse_count,
        "positive_rate_hz": positive_rate,
        "inverse_rate_hz": inverse_rate,
        "zero_latency_seconds": zero_latencies,
    }


def _rate_and_semantics(
    records: Sequence[Mapping[str, Any]],
    topic: str,
    phase: Mapping[str, int],
    axes: tuple[int, int, int],
    hold_s: float,
) -> dict[str, Any]:
    selected = samples_in_window(
        records, topic, int(phase["start_ns"]), int(phase["end_ns"])
    )
    rate = len(selected) / max(hold_s, 1.0e-9)
    matches = [record for record in selected if semantic_twist_match(twist_values(record), axes)]
    fraction = len(matches) / len(selected) if selected else 0.0
    if rate < 5.0:
        raise MatrixError(f"{topic} rate is too low in motion phase: {rate:.2f} Hz")
    if fraction < 0.80:
        raise MatrixError(
            f"{topic} semantic match is too low: {len(matches)}/{len(selected)} ({fraction:.3f})"
        )
    return {"samples": len(selected), "rate_hz": rate, "semantic_match_fraction": fraction}


def _zero_latency(
    records: Sequence[Mapping[str, Any]], topic: str, released_ns: int
) -> float:
    candidates = [
        record for record in trace_samples(records, topic)
        if int(record["received_at_unix_ns"]) >= released_ns and is_zero_twist(record)
    ]
    if not candidates:
        raise MatrixError(f"{topic} did not emit a post-release zero")
    latency = (int(candidates[0]["received_at_unix_ns"]) - released_ns) / 1.0e9
    if latency > 0.35:
        raise MatrixError(f"{topic} zero latency exceeds 0.35 s: {latency:.3f} s")
    return latency


def _physical_zero_latency(
    records: Sequence[Mapping[str, Any]], released_ns: int
) -> float:
    topic = "/carla/ego_vehicle/physical_four_wheel_cmd"
    candidates = [
        record for record in trace_samples(records, topic)
        if int(record["received_at_unix_ns"]) >= released_ns
        and record["message"].get("independent_wheel_torque_active") is False
        and all(abs(float(value)) <= 1.0e-6 for value in record["message"].get("torques_nm", []))
    ]
    if not candidates:
        raise MatrixError("physical 4WS controller did not emit a post-release torque zero")
    latency = (int(candidates[0]["received_at_unix_ns"]) - released_ns) / 1.0e9
    if latency > 0.35:
        raise MatrixError(f"physical 4WS torque-zero latency exceeds 0.35 s: {latency:.3f} s")
    return latency


def _latest_bool(records: Sequence[Mapping[str, Any]], topic: str) -> bool | None:
    selected = trace_samples(records, topic)
    return bool(selected[-1]["message"]["data"]) if selected else None


def validate_setup_trace(records: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    engage = _latest_bool(records, "/planning/engage")
    drive_enable = _latest_bool(records, "/platform/drive_enable")
    if engage is not True or drive_enable is not True:
        raise MatrixError(
            f"visible UI ARM did not produce engage/drive-enable true: {engage}, {drive_enable}"
        )
    manual = trace_samples(records, "/control/manual_cmd_vel_ros")
    if not manual or not is_zero_twist(manual[-1]):
        raise MatrixError("manual ARM setup did not retain an exact zero command")
    return {
        "planning_engage": engage,
        "platform_drive_enable": drive_enable,
        "manual_zero_while_armed": True,
    }


def validate_teardown_trace(records: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for topic in ("/control/manual_cmd_vel_ros", "/control/cmd_vel_ros"):
        selected = trace_samples(records, topic)
        if not selected or not is_zero_twist(selected[-1]):
            raise MatrixError(f"teardown did not leave {topic} at exact zero")
        result[topic] = "zero"
    physical = trace_samples(records, "/carla/ego_vehicle/physical_four_wheel_cmd")
    if not physical:
        raise MatrixError("teardown did not observe a physical 4WS command")
    final = physical[-1]["message"]
    if (
        final.get("independent_wheel_torque_active") is not False
        or any(abs(float(value)) > 1.0e-6 for value in final.get("torques_nm", []))
    ):
        raise MatrixError("teardown did not leave all four physical wheel torques at zero")
    engage = _latest_bool(records, "/planning/engage")
    drive_enable = _latest_bool(records, "/platform/drive_enable")
    if engage is not False or drive_enable is not False:
        raise MatrixError(
            f"teardown did not clear engage/drive-enable: {engage}, {drive_enable}"
        )
    result.update({
        "physical_four_wheel_torque": "zero",
        "planning_engage": engage,
        "platform_drive_enable": drive_enable,
    })
    return result


def validate_already_disarmed_trace(
    records: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Prove durable zero state when cleanup follows an earlier fail-safe."""
    final_commands = trace_samples(records, "/control/cmd_vel_ros")
    if not final_commands or not is_zero_twist(final_commands[-1]):
        raise MatrixError("already-disarmed cleanup did not observe final cmd_vel zero")
    physical = trace_samples(
        records, "/carla/ego_vehicle/physical_four_wheel_cmd"
    )
    if not physical:
        raise MatrixError("already-disarmed cleanup did not observe physical 4WS zero")
    final_physical = physical[-1]["message"]
    if (
        final_physical.get("independent_wheel_torque_active") is not False
        or any(
            abs(float(value)) > 1.0e-6
            for value in final_physical.get("torques_nm", [])
        )
    ):
        raise MatrixError("already-disarmed cleanup retained physical wheel torque")
    gate = trace_samples(records, "/control/cmd_vel_safety_gate/status")
    if not gate:
        raise MatrixError("already-disarmed cleanup did not observe gate status")
    gate_message = str(gate[-1]["message"].get("message", ""))
    if (
        "engage=false(manual=false,mission=false)" not in gate_message
        or "platform_drive_enable=false" not in gate_message
    ):
        raise MatrixError(
            "already-disarmed cleanup did not prove both authorization gates closed"
        )
    return {
        "/control/cmd_vel_ros": "zero",
        "physical_four_wheel_torque": "zero",
        "authorization": "closed_from_durable_gate_status",
    }


def validate_ros_trace(
    records: Sequence[Mapping[str, Any]],
    run_document: Mapping[str, Any],
    spec: ScenarioSpec,
    hold_s: float,
) -> dict[str, Any]:
    phases = run_document["phases"]
    manual_topic = "/control/manual_cmd_vel_ros"
    final_topic = "/control/cmd_vel_ros"
    extended_topic = "/carla/ego_vehicle/extended_ackermann_cmd"
    physical_topic = "/carla/ego_vehicle/physical_four_wheel_cmd"
    status_topic = "/carla/ego_vehicle/physical_four_wheel_status"
    odom_topic = "/carla/ego_vehicle/odometry"
    collision_topic = "/carla/ego_vehicle/collision"

    publisher_counts = run_document.get("ros_observation", {}).get("publisher_counts", {})
    missing_publishers = [
        topic for topic in (
            manual_topic, final_topic, extended_topic, physical_topic,
            status_topic, odom_topic, collision_topic,
        )
        if int(publisher_counts.get(topic, 0)) < 1
    ]
    if missing_publishers:
        raise MatrixError("critical ROS publishers were absent: " + ", ".join(missing_publishers))

    motion: dict[str, Any] = {}
    for name, ui_axes in (
        ("positive", spec.positive_axes),
        ("inverse", spec.inverse_axes),
    ):
        phase = phases[name]
        twist_axes = ui_axes_to_twist_axes(ui_axes)
        motion[name] = {
            "manual": _rate_and_semantics(
                records, manual_topic, phase, twist_axes, hold_s
            ),
            "safety_output": _rate_and_semantics(
                records, final_topic, phase, twist_axes, hold_s
            ),
        }
    zero = {
        topic: [
            _zero_latency(records, topic, int(phases[name]["release_ns"]))
            for name in ("positive", "inverse")
        ]
        for topic in (manual_topic, final_topic)
    }
    zero[physical_topic] = [
        _physical_zero_latency(records, int(phases[name]["release_ns"]))
        for name in ("positive", "inverse")
    ]

    extended_modes: dict[str, Any] = {}
    for name in ("positive", "inverse"):
        phase = phases[name]
        selected = samples_in_window(
            records, extended_topic, int(phase["start_ns"]), int(phase["end_ns"])
        )
        matching = [record for record in selected if int(record["message"]["drive_mode"]) == spec.mode_id]
        fraction = len(matching) / len(selected) if selected else 0.0
        if len(selected) < 5 or fraction < 0.80:
            raise MatrixError(
                f"extended 4WS mode {spec.mode} missing in {name}: "
                f"{len(matching)}/{len(selected)}"
            )
        if any(bool(record["message"].get("recovery_breakaway_authorized")) for record in matching):
            raise MatrixError("manual 4WS command unexpectedly authorized recovery breakaway")
        extended_modes[name] = {
            "samples": len(selected), "matching": len(matching), "fraction": fraction
        }

    physical_all = trace_samples(records, physical_topic)
    sequences = [int(record["message"]["sequence"]) for record in physical_all]
    if not sequences or any(right <= left for left, right in zip(sequences, sequences[1:])):
        raise MatrixError("physical 4WS command sequence is empty or not strictly increasing")
    physical_modes: dict[str, Any] = {}
    for name in ("positive", "inverse"):
        phase = phases[name]
        selected = samples_in_window(
            records, physical_topic, int(phase["start_ns"]), int(phase["end_ns"])
        )
        active = [
            record for record in selected
            if int(record["message"]["drive_mode"]) == spec.mode_id
            and bool(record["message"]["independent_wheel_torque_active"])
            and not bool(record["message"]["reset"])
        ]
        if len(active) < 5:
            raise MatrixError(f"{name} has fewer than five active physical 4WD commands")
        for record in active:
            message = record["message"]
            if (
                message["motion_backend"] != "PHYSX_FOUR_WHEEL_STEERING"
                or message["physical_gate_accepted"] is not True
                or message["requires_independent_wheel_drive"] is not True
            ):
                raise MatrixError(f"{name} physical command lost its gate/backend contract")
            if any(abs(float(value)) <= 0.01 or abs(float(value)) > 20.000001 for value in message["torques_nm"]):
                raise MatrixError(f"{name} physical command has invalid four-wheel torque")
        physical_modes[name] = {"samples": len(selected), "active_samples": len(active)}

    statuses = trace_samples(records, status_topic)
    if not statuses:
        raise MatrixError("physical 4WS status was not observed")
    actor_ids = {int(record["message"]["actor_id"]) for record in statuses}
    if len(actor_ids) != 1 or next(iter(actor_ids)) <= 0:
        raise MatrixError(f"physical actor identity changed or is invalid: {sorted(actor_ids)}")
    if any(
        record["message"].get("ready") is not True
        or record["message"].get("physical_gate_accepted") is not True
        or record["message"].get("physx_substep_control_verified") is not True
        or record["message"].get("independent_wheel_drive_available") is not True
        or abs(float(record["message"].get("wheel_torque_safety_cap_nm", math.nan)) - 20.0) > 1.0e-6
        for record in statuses
    ):
        raise MatrixError("physical 4WS readiness was not continuously valid")
    if len(trace_samples(records, odom_topic)) < 4:
        raise MatrixError("CARLA odometry did not produce enough samples")
    collisions = trace_samples(records, collision_topic)
    if collisions:
        raise MatrixError(f"CARLA collision sensor reported {len(collisions)} event(s)")
    return {
        "motion_topics": motion,
        "zero_latency_seconds": zero,
        "extended_modes": extended_modes,
        "physical_commands": physical_modes,
        "physical_sequence_strict": True,
        "physical_status_samples": len(statuses),
        "actor_id": next(iter(actor_ids)),
        "collision_events": 0,
        "critical_publishers_present": True,
    }


def load_trace(path: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    records: list[dict[str, Any]] = []
    footer: dict[str, Any] | None = None
    try:
        with path.open("r", encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, 1):
                value = json.loads(line)
                if not isinstance(value, dict):
                    raise MatrixError(f"{path}:{line_number} is not a JSON object")
                if value.get("record_type") == "sample":
                    records.append(value)
                elif value.get("record_type") == "footer":
                    footer = value
    except (OSError, json.JSONDecodeError) as error:
        raise MatrixError(f"cannot read ROS trace {path}: {error}") from None
    if footer is None or footer.get("status") != "COMPLETED" or footer.get("error") not in (None, ""):
        raise MatrixError(f"ROS trace has no clean COMPLETED footer: {footer!r}")
    return records, footer


def load_wheel_samples(path: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    footer: dict[str, Any] | None = None
    header_seen = False
    try:
        with path.open("r", encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, 1):
                value = json.loads(line)
                if not isinstance(value, dict):
                    raise MatrixError(f"{path}:{line_number} is not a JSON object")
                record_type = value.get("record_type")
                if record_type == "header":
                    header_seen = True
                    if value.get("read_only") is not True:
                        raise MatrixError("physical wheel recorder was not declared read-only")
                elif record_type == "sample":
                    samples.append(value)
                elif record_type == "footer":
                    footer = value
    except (OSError, json.JSONDecodeError) as error:
        raise MatrixError(f"cannot read physical wheel telemetry {path}: {error}") from None
    if not header_seen or footer is None or footer.get("status") not in {"STOPPED", "COMPLETED"}:
        raise MatrixError("physical wheel telemetry is missing a valid header/footer")
    if not samples:
        raise MatrixError("physical wheel telemetry has no samples")
    identities = {
        (sample["actor"]["id"], sample["actor"]["type_id"], sample["actor"]["role_name"])
        for sample in samples
    }
    if len(identities) != 1:
        raise MatrixError(f"physical wheel actor identity changed: {identities!r}")
    identity = next(iter(identities))
    if identity[0] <= 0 or identity[1:] != ("vehicle.ranger.default", "ego_vehicle"):
        raise MatrixError(f"physical wheel actor identity is not the Ranger ego: {identity!r}")
    for sample in samples:
        names = tuple(wheel.get("canonical_location") for wheel in sample.get("wheels", []))
        if names != WHEEL_NAMES:
            raise MatrixError(f"physical wheel sample is not exact FL/FR/RL/RR: {names!r}")
    return samples, footer


def normalize_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def _nearest_sample(samples: Sequence[Mapping[str, Any]], timestamp_ns: int) -> Mapping[str, Any]:
    if not samples:
        raise MatrixError("cannot select a sample from an empty sequence")
    return min(samples, key=lambda item: abs(int(item["timestamp_unix_ns"]) - timestamp_ns))


def _wheel_window(
    samples: Sequence[Mapping[str, Any]], start_ns: int, end_ns: int
) -> list[dict[str, Any]]:
    return [
        dict(sample) for sample in samples
        if start_ns <= int(sample["timestamp_unix_ns"]) <= end_ns
    ]


def motion_metrics(
    samples: Sequence[Mapping[str, Any]], phase: Mapping[str, int]
) -> dict[str, Any]:
    selected = _wheel_window(samples, int(phase["start_ns"]), int(phase["end_ns"]))
    if len(selected) < 5:
        raise MatrixError(f"physical wheel phase has too few samples: {len(selected)}")
    first, last = selected[0], selected[-1]
    first_location = first["actor"]["transform"]["location_m"]
    last_location = last["actor"]["transform"]["location_m"]
    dx = float(last_location["x"]) - float(first_location["x"])
    dy = float(last_location["y"]) - float(first_location["y"])
    yaw = math.radians(float(first["actor"]["yaw_degrees"]))
    body_forward = math.cos(yaw) * dx + math.sin(yaw) * dy
    body_right = -math.sin(yaw) * dx + math.cos(yaw) * dy
    path = 0.0
    max_step = 0.0
    for left, right in zip(selected, selected[1:]):
        a = left["actor"]["transform"]["location_m"]
        b = right["actor"]["transform"]["location_m"]
        step = math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))
        max_step = max(max_step, step)
        path += step
    if max_step > 2.0:
        raise MatrixError(f"physical actor teleported/reset by {max_step:.3f} m")
    yaw_delta = normalize_degrees(
        float(last["actor"]["yaw_degrees"]) - float(first["actor"]["yaw_degrees"])
    )
    grounded = {
        name: sum(not bool(sample["wheels"][index]["is_in_air"]) for sample in selected) / len(selected)
        for index, name in enumerate(WHEEL_NAMES)
    }
    loaded = {
        name: sum(float(sample["wheels"][index]["current_tire_load_n"]) > 1.0 for sample in selected) / len(selected)
        for index, name in enumerate(WHEEL_NAMES)
    }
    if min(grounded.values()) < 0.80 or min(loaded.values()) < 0.80:
        raise MatrixError(f"one or more physical wheels lost grounded/load evidence: {grounded}, {loaded}")
    return {
        "samples": len(selected),
        "path_distance_m": path,
        "body_forward_m": body_forward,
        "body_right_m": body_right,
        "translation_m": math.hypot(dx, dy),
        "yaw_delta_degrees": yaw_delta,
        "maximum_step_m": max_step,
        "grounded_fraction": grounded,
        "loaded_fraction": loaded,
    }


def validate_motion_metrics(
    spec: ScenarioSpec, positive: Mapping[str, Any], inverse: Mapping[str, Any]
) -> dict[str, Any]:
    for name, metrics in (("positive", positive), ("inverse", inverse)):
        if spec.name != "zero_turn" and float(metrics["path_distance_m"]) < 0.30:
            raise MatrixError(f"{spec.name} {name} path is below 0.30 m")
    if spec.name == "straight":
        if positive["body_forward_m"] < 0.25 or inverse["body_forward_m"] > -0.25:
            raise MatrixError("straight/reverse body-frame displacement is insufficient or reversed")
        if any(abs(float(item["body_right_m"])) > 0.15 for item in (positive, inverse)):
            raise MatrixError("straight/reverse lateral leakage exceeds 0.15 m")
        if any(abs(float(item["yaw_delta_degrees"])) > 3.0 for item in (positive, inverse)):
            raise MatrixError("straight/reverse yaw drift exceeds 3 degrees")
    elif spec.name == "turn":
        if any(abs(float(item["yaw_delta_degrees"])) < 5.0 for item in (positive, inverse)):
            raise MatrixError("dual Ackermann yaw change is below 5 degrees")
        if float(positive["yaw_delta_degrees"]) * float(inverse["yaw_delta_degrees"]) >= 0.0:
            raise MatrixError("turn retrace yaw did not reverse direction")
    elif spec.name == "crab":
        # CARLA/Unreal is left-handed (+Y right); Robot UI Z is ROS +Y left.
        if positive["body_right_m"] > -0.25 or inverse["body_right_m"] < 0.25:
            raise MatrixError("crab lateral displacement is insufficient or reversed")
        if any(abs(float(item["body_forward_m"])) > 0.15 for item in (positive, inverse)):
            raise MatrixError("crab longitudinal leakage exceeds 0.15 m")
        if any(abs(float(item["yaw_delta_degrees"])) > 3.0 for item in (positive, inverse)):
            raise MatrixError("crab yaw drift exceeds 3 degrees")
    elif spec.name == "zero_turn":
        if any(abs(float(item["yaw_delta_degrees"])) < 8.0 for item in (positive, inverse)):
            raise MatrixError("zero-turn yaw change is below 8 degrees")
        if any(float(item["translation_m"]) > 0.20 for item in (positive, inverse)):
            raise MatrixError("zero-turn center translation exceeds 0.20 m")
        if float(positive["yaw_delta_degrees"]) * float(inverse["yaw_delta_degrees"]) >= 0.0:
            raise MatrixError("zero-turn inverse leg did not reverse yaw direction")
    return {"positive": dict(positive), "inverse": dict(inverse)}


def validate_wheel_command_pattern(spec: ScenarioSpec, command: Mapping[str, Any]) -> None:
    angles = [math.degrees(float(value)) for value in command["steer_radians"]]
    signs = [int(value) for value in command["drive_signs"]]
    if any(value == 0 for value in signs):
        raise MatrixError("active physical command contains a zero drive sign")
    if spec.name == "straight":
        if max(abs(value) for value in angles) > 2.0 or len(set(signs)) != 1:
            raise MatrixError("straight physical command is not zero-steer four-wheel drive")
    elif spec.name == "turn":
        if not (angles[0] * angles[2] < 0.0 and angles[1] * angles[3] < 0.0):
            raise MatrixError("dual Ackermann command does not oppose front/rear steering")
        if len(set(signs)) != 1:
            raise MatrixError("dual Ackermann command does not drive all wheels together")
    elif spec.name == "crab":
        if min(abs(value) for value in angles) < 80.0:
            raise MatrixError("crab physical steering is below 80 degrees")
        if max(angles) - min(angles) > 2.0 or len(set(signs)) != 1:
            raise MatrixError("crab command does not align/drive all four wheels together")
    elif spec.name == "zero_turn":
        if not (
            angles[0] * angles[1] < 0.0
            and angles[0] * angles[2] < 0.0
            and angles[0] * angles[3] > 0.0
        ):
            raise MatrixError("zero-turn command does not form the required X steering pattern")
        if not (signs[0] == signs[2] and signs[1] == signs[3] and signs[0] == -signs[1]):
            raise MatrixError("zero-turn command does not use opposite-side drive polarity")


def validate_physical_wheel_readback(
    spec: ScenarioSpec,
    wheel_samples: Sequence[Mapping[str, Any]],
    ros_records: Sequence[Mapping[str, Any]],
    phases: Mapping[str, Mapping[str, int]],
) -> dict[str, Any]:
    topic = "/carla/ego_vehicle/physical_four_wheel_cmd"
    command_records = trace_samples(ros_records, topic)
    phase_results: dict[str, Any] = {}
    for phase_name in ("positive", "inverse"):
        phase = phases[phase_name]
        # The controller enables bounded wheel torque while steering slews out
        # of the previous zero pose.  Judge the commanded geometry over the
        # same 0.5 s settled window used for physical wheel readback below;
        # otherwise a legitimate first zero-angle transition frame is falsely
        # treated as the requested steady 4WS pose.
        settled_start_ns = int(phase["start_ns"]) + 500_000_000
        commands = [
            record for record in samples_in_window(
                ros_records, topic, settled_start_ns, int(phase["end_ns"])
            )
            if int(record["message"]["drive_mode"]) == spec.mode_id
            and bool(record["message"]["independent_wheel_torque_active"])
        ]
        if len(commands) < 5:
            raise MatrixError(f"{phase_name} has insufficient active physical command samples")
        for command in commands:
            validate_wheel_command_pattern(spec, command["message"])
        selected_wheels = _wheel_window(
            wheel_samples,
            settled_start_ns,
            int(phase["end_ns"]),
        )
        comparisons = 0
        matching = 0
        telemetry_torque_active = {name: 0 for name in WHEEL_NAMES}
        for sample in selected_wheels:
            sample_ns = int(sample["timestamp_unix_ns"])
            nearest = min(
                commands,
                key=lambda record: abs(int(record["received_at_unix_ns"]) - sample_ns),
            )
            if abs(int(nearest["received_at_unix_ns"]) - sample_ns) > 500_000_000:
                continue
            target = [math.degrees(float(value)) for value in nearest["message"]["steer_radians"]]
            measured = [float(wheel["steer_angle_degrees"]) for wheel in sample["wheels"]]
            comparisons += 1
            if max(abs(a - b) for a, b in zip(target, measured)) <= 2.0:
                matching += 1
            for index, name in enumerate(WHEEL_NAMES):
                if abs(float(sample["wheels"][index]["external_drive_torque_nm"])) > 0.01:
                    telemetry_torque_active[name] += 1
        fraction = matching / comparisons if comparisons else 0.0
        if comparisons < 5 or fraction < 0.80:
            raise MatrixError(
                f"{phase_name} physical steer readback match is too low: "
                f"{matching}/{comparisons} ({fraction:.3f})"
            )
        torque_fractions = {
            name: count / len(selected_wheels) if selected_wheels else 0.0
            for name, count in telemetry_torque_active.items()
        }
        if min(torque_fractions.values()) < 0.50:
            raise MatrixError(
                f"{phase_name} patched PhysX telemetry lacks four-wheel torque: {torque_fractions}"
            )
        phase_results[phase_name] = {
            "command_samples": len(commands),
            "readback_comparisons": comparisons,
            "steering_match_fraction_2deg": fraction,
            "telemetry_torque_active_fraction": torque_fractions,
        }
    sequences = [int(record["message"]["sequence"]) for record in command_records]
    return {"phases": phase_results, "physical_command_sequences": len(sequences)}


def validate_capture(scenario_dir: Path) -> dict[str, Any]:
    visual = scenario_dir / "visual"
    manifest_path = visual / "capture_manifest.json"
    manifest = read_json(manifest_path)
    if manifest.get("status") != "PASS" or manifest.get("scope", {}).get("kind") != "actual X11 desktop-region recording":
        raise MatrixError("visual capture is not a PASS actual-X11 recording")
    x11 = manifest.get("x11", {})
    if (
        x11.get("geometry_validated_side_by_side") is not True
        or x11.get("same_titles_and_geometry_before_after_capture") is not True
    ):
        raise MatrixError("visual capture did not preserve side-by-side window geometry")
    windows = x11.get("windows", {})
    if "CarlaUE4" not in str(windows.get("carla", {}).get("title", "")):
        raise MatrixError("visual capture did not bind the CarlaUE4 window")
    if EXPECTED_UI_TITLE not in str(windows.get("camrod_operator_ui", {}).get("title", "")):
        raise MatrixError("visual capture did not bind the visible Robot UI window")
    if float(manifest.get("recording", {}).get("actual_duration_s", 0.0)) < 12.0:
        raise MatrixError("visual capture is shorter than 12 seconds")
    artifacts = {
        "manifest": artifact(manifest_path, scenario_dir),
        "png": artifact(visual / "representative_contact_sheet.png", scenario_dir),
        "gif": artifact(visual / "representative_motion.gif", scenario_dir),
    }
    for key, filename in (
        ("contact_sheet_png", "representative_contact_sheet.png"),
        ("representative_gif", "representative_motion.gif"),
    ):
        expected = manifest.get("artifacts", {}).get(key, {})
        path = visual / filename
        if expected.get("sha256") != sha256_file(path) or int(expected.get("bytes", -1)) != path.stat().st_size:
            raise MatrixError(f"visual {key} hash/size does not match its capture manifest")
    return {"actual_x11": True, "artifacts": artifacts}


def _control_frame(records: Sequence[Mapping[str, Any]], expected_type: str) -> dict[str, Any]:
    matches: list[dict[str, Any]] = []
    for record in records:
        frame = record.get("frame")
        if isinstance(frame, dict) and frame.get("type") == expected_type:
            if set(frame) != {"type", "seq"}:
                raise MatrixError(f"{expected_type} frame fields are not exact: {sorted(frame)}")
            if isinstance(frame.get("seq"), bool) or not isinstance(frame.get("seq"), int):
                raise MatrixError(f"{expected_type} seq is not an exact integer")
            matches.append(frame)
    if not matches:
        raise MatrixError(f"visible Robot UI did not emit an exact {expected_type} frame")
    return matches[-1]


def _browser_arguments(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "debugging_url": args.operator_cdp_url,
        "operator_ui_url": args.ui_url,
        "timeout_s": args.timeout_s,
    }


def _bind_browser_to_capture(
    browser: ManualDriveBrowser, args: argparse.Namespace
) -> dict[str, Any]:
    return browser.bind_x11_window(
        args.x11_window_id,
        args.display,
        args.xauthority,
    )


def _force_visible_fail_safe(
    browser: ManualDriveBrowser | None,
    args: argparse.Namespace,
    held_keys: Iterable[str] = (),
) -> bool:
    """Use the existing page connection, then one fresh CDP attachment if needed."""
    if browser is not None and browser.fail_safe(held_keys):
        return True
    if browser is not None:
        browser.close()
    fallback: ManualDriveBrowser | None = None
    try:
        fallback = ManualDriveBrowser(**_browser_arguments(args))
        try:
            _bind_browser_to_capture(fallback, args)
        except Exception:
            # The target list still proved exactly one local Robot UI URL.  A
            # safety ZERO/DISARM must not be withheld merely because X11 went
            # away; the run will remain FAIL and cannot become evidence.
            pass
        return fallback.fail_safe(held_keys)
    except Exception:
        return False
    finally:
        if fallback is not None:
            fallback.close()


def command_setup(args: argparse.Namespace) -> int:
    output = require_output_directory(args.output_dir)
    document_path = output / "session_setup.json"
    trace_path = output / "ros_trace.jsonl"
    browser: ManualDriveBrowser | None = None
    observer: RosTraceObserver | None = None
    armed = False
    arm_attempted = False
    try:
        browser = ManualDriveBrowser(**_browser_arguments(args))
        x11_binding = _bind_browser_to_capture(browser, args)
        observer = RosTraceObserver(trace_path, role_name=args.role_name)
        observer.wait_ready(args.ros_ready_timeout_s, require_odometry=False)
        authentication = browser.ensure_fresh_administrator_session()
        arm_attempted = True
        arm_result = browser.arm()
        armed = True
        observer.spin_for(1.0)
        browser.assert_motion_liveness("post-ARM setup verification")
        observer.assert_motion_liveness()
        frames = browser.probe().get("websocket", [])
        arm_frame = _control_frame(frames, "arm")
        snapshot = browser.snapshot()
        if not snapshot.get("armed") or snapshot.get("stateText") != "ARMED":
            raise MatrixError(f"manual-drive ARM did not remain latched: {snapshot!r}")
        status_topic = "/carla/ego_vehicle/physical_four_wheel_status"
        if observer.counts.get(status_topic, 0) < 1:
            raise MatrixError("physical 4WS readiness was not observed during setup")
        setup_checks = validate_setup_trace(observer.records)
        publisher_counts = dict(observer.publisher_counts)
        observer.close()
        observer = None
        document = {
            "schema": SCHEMA,
            "record_type": "session_setup",
            "status": "PASS",
            "created_at_utc": utc_now(),
            "authority": {
                "kind": "visible Robot UI administrator form via CDP",
                "server_authenticated": False,
                "frontend_administrator_gate_exercised": True,
                "direct_websocket_or_ros_command": False,
                "cdp_x11_binding": x11_binding,
            },
            "authentication": authentication,
            "arm": {
                "frame": arm_frame,
                "snapshot": snapshot,
                "interactions": arm_result["interactions"],
            },
            "ros_observation": {
                "subscriber_only": True,
                "publisher_counts": publisher_counts,
                "checks": setup_checks,
            },
            "artifacts": {"ros_trace": artifact(trace_path, output)},
        }
        # The close operation above intentionally leaves the browser page and
        # its page-owned manual WebSocket alive and ARMED for scenario runs.
        write_json_exclusive(document_path, document)
        print(f"[manual-4ws] setup PASS: {document_path}")
        return 0
    except BaseException as error:
        # arm() can fail after its visible click changed page/backend state.
        # Treat every attempted ARM as potentially live and always force the
        # same visible-page ZERO/DISARM path before finalizing observers.
        fail_safe_confirmed = False
        if browser is not None and (armed or arm_attempted):
            fail_safe_confirmed = _force_visible_fail_safe(browser, args)
        if observer is not None:
            try:
                observer.spin_for(0.5)
            except Exception:
                pass
            observer.close("ERROR", str(error))
        failure = {
            "schema": SCHEMA,
            "record_type": "session_setup",
            "status": "FAIL",
            "created_at_utc": utc_now(),
            "error": str(error),
            "fail_safe_zero_disarm_confirmed": fail_safe_confirmed,
        }
        if not document_path.exists():
            write_json_exclusive(document_path, failure)
        raise
    finally:
        if browser is not None:
            browser.close()


def _press_chord(browser: ManualDriveBrowser, keys: Sequence[str]) -> int:
    for index, code in enumerate(keys):
        browser.dispatch_key(code, True)
        if index + 1 < len(keys):
            time.sleep(0.05)
    return time.time_ns()


def _release_chord(browser: ManualDriveBrowser, keys: Sequence[str]) -> int:
    ordered = list(reversed(keys))
    for code in ordered[:-1]:
        browser.dispatch_key(code, False)
        time.sleep(0.03)
    release_ns = time.time_ns()
    browser.dispatch_key(ordered[-1], False)
    return release_ns


def _observe_with_motion_liveness(
    observer: RosTraceObserver,
    browser: ManualDriveBrowser,
    duration_s: float,
    description: str,
) -> None:
    """Spin subscriptions while continuously proving UI and ROS authority health."""
    deadline = time.monotonic() + max(0.0, duration_s)
    browser.assert_motion_liveness(description)
    observer.assert_motion_liveness()
    while time.monotonic() < deadline:
        observer.spin_for(min(0.20, max(0.0, deadline - time.monotonic())))
        browser.assert_motion_liveness(description)
        observer.assert_motion_liveness()


def command_scenario(args: argparse.Namespace) -> int:
    output = require_output_directory(args.output_dir)
    spec = SCENARIOS[args.scenario]
    run_path = output / "scenario_run.json"
    interactions_path = output / "ui_interactions.json"
    trace_path = output / "ros_trace.jsonl"
    browser: ManualDriveBrowser | None = None
    observer: RosTraceObserver | None = None
    held: list[str] = []
    phases: dict[str, dict[str, int]] = {}
    try:
        browser = ManualDriveBrowser(**_browser_arguments(args))
        x11_binding = _bind_browser_to_capture(browser, args)
        snapshot_before = browser.snapshot()
        if (
            not snapshot_before.get("panelVisible")
            or not snapshot_before.get("connected")
            or not snapshot_before.get("armed")
            or snapshot_before.get("stateText") != "ARMED"
        ):
            raise MatrixError(
                "scenario requires the setup-owned connected ARMED Robot UI: "
                f"{snapshot_before!r}"
            )
        browser._interactions = []
        browser.clear_probe()
        observer = RosTraceObserver(trace_path, role_name=args.role_name)
        observer.wait_ready(args.ros_ready_timeout_s, require_odometry=True)
        # Graph discovery can make status/odometry ready a fraction of a
        # second before all already-armed zero-command streams arrive.
        observer.spin_for(0.5)
        _observe_with_motion_liveness(
            observer, browser, args.initial_zero_s, "initial zero phase"
        )

        held = list(spec.positive_keys)
        browser.assert_motion_liveness("immediately before positive motion")
        observer.assert_motion_liveness()
        positive_start_ns = _press_chord(browser, held)
        _observe_with_motion_liveness(
            observer, browser, args.hold_s, "positive motion phase"
        )
        positive_end_ns = time.time_ns()
        positive_release_ns = _release_chord(browser, held)
        held = []
        browser.zero()
        phases["positive"] = {
            "start_ns": positive_start_ns,
            "end_ns": positive_end_ns,
            "release_ns": positive_release_ns,
        }
        _observe_with_motion_liveness(
            observer, browser, args.zero_hold_s, "inter-leg exact-zero phase"
        )

        held = list(spec.inverse_keys)
        browser.assert_motion_liveness("immediately before inverse motion")
        observer.assert_motion_liveness()
        inverse_start_ns = _press_chord(browser, held)
        _observe_with_motion_liveness(
            observer, browser, args.hold_s, "inverse motion phase"
        )
        inverse_end_ns = time.time_ns()
        inverse_release_ns = _release_chord(browser, held)
        held = []
        browser.zero()
        phases["inverse"] = {
            "start_ns": inverse_start_ns,
            "end_ns": inverse_end_ns,
            "release_ns": inverse_release_ns,
        }
        _observe_with_motion_liveness(
            observer, browser, args.settle_s, "final exact-zero settle phase"
        )

        websocket_records = [
            dict(item) for item in browser.probe().get("websocket", [])
            if isinstance(item, Mapping)
        ]
        ui_checks = validate_ui_frames(websocket_records, spec, phases, args.hold_s)
        snapshot_after = browser.snapshot()
        if not snapshot_after.get("armed") or snapshot_after.get("stateText") != "ARMED":
            raise MatrixError(f"manual-drive authority was lost during scenario: {snapshot_after!r}")
        provisional = {
            "phases": phases,
            "ros_observation": {"publisher_counts": observer.publisher_counts},
        }
        ros_checks = validate_ros_trace(
            observer.records, provisional, spec, args.hold_s
        )
        observer.close()
        observer = None
        interactions = {
            "schema": SCHEMA,
            "record_type": "visible_ui_interactions",
            "scenario": spec.name,
            "transport": "CDP.Input.dispatchKeyEvent",
            "direct_websocket_send": False,
            "direct_ros_publish": False,
            "events": list(browser._interactions),
            "websocket_records": websocket_records,
        }
        write_json_exclusive(interactions_path, interactions)
        document = {
            "schema": SCHEMA,
            "record_type": "scenario_run",
            "scenario": spec.name,
            "label": spec.label,
            "status": "PASS",
            "created_at_utc": utc_now(),
            "scenario_contract": asdict(spec),
            "timing": {
                "initial_zero_s": args.initial_zero_s,
                "hold_s": args.hold_s,
                "zero_hold_s": args.zero_hold_s,
                "settle_s": args.settle_s,
            },
            "phases": phases,
            "ui": {
                "snapshot_before": snapshot_before,
                "snapshot_after": snapshot_after,
                "checks": ui_checks,
                "cdp_x11_binding": x11_binding,
            },
            "websocket_records": websocket_records,
            "ros_observation": {
                "subscriber_only": True,
                "create_publisher_called": False,
                "publisher_counts": observer.publisher_counts if observer else provisional["ros_observation"]["publisher_counts"],
                "checks": ros_checks,
            },
            "artifacts": {
                "ui_interactions": artifact(interactions_path, output),
                "ros_trace": artifact(trace_path, output),
            },
        }
        write_json_exclusive(run_path, document)
        print(f"[manual-4ws] {spec.name} UI/ROS phase PASS: {run_path}")
        return 0
    except BaseException as error:
        fail_safe_confirmed = _force_visible_fail_safe(browser, args, held)
        if observer is not None:
            try:
                observer.spin_for(0.75)
            except Exception:
                pass
            observer.close("ERROR", str(error))
        failure_path = output / "scenario_failure.json"
        if not failure_path.exists():
            write_json_exclusive(failure_path, {
                "schema": SCHEMA,
                "record_type": "scenario_run",
                "scenario": spec.name,
                "status": "FAIL",
                "created_at_utc": utc_now(),
                "error": str(error),
                "phases": phases,
                "fail_safe_zero_disarm_attempted": browser is not None,
                "fail_safe_zero_disarm_confirmed": fail_safe_confirmed,
            })
        raise
    finally:
        if browser is not None:
            browser.close()


def command_teardown(args: argparse.Namespace) -> int:
    output = require_output_directory(args.output_dir)
    document_path = output / "session_teardown.json"
    trace_path = output / "ros_trace.jsonl"
    browser: ManualDriveBrowser | None = None
    observer: RosTraceObserver | None = None
    try:
        browser = ManualDriveBrowser(**_browser_arguments(args))
        x11_binding = _bind_browser_to_capture(browser, args)
        browser._interactions = []
        browser.clear_probe()
        snapshot_before = browser.snapshot()
        was_armed = bool(snapshot_before.get("armed"))
        observer = RosTraceObserver(trace_path, role_name=args.role_name)
        observer.wait_ready(args.ros_ready_timeout_s, require_odometry=False)
        # When still armed, wait for the page-owned 10 Hz exact-zero heartbeat
        # before sending the visible DISARM boundary. This closes the DDS
        # discovery race without publishing any command from the recorder.
        if was_armed:
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                manual = trace_samples(
                    observer.records, "/control/manual_cmd_vel_ros"
                )
                if manual and is_zero_twist(manual[-1]):
                    break
                observer.spin_for(0.10)
            else:
                raise MatrixError(
                    "teardown did not observe the armed page's exact-zero heartbeat"
                )
            result = browser.disarm()
            deadline = time.monotonic() + 2.0
            last_error: Exception | None = None
            while time.monotonic() < deadline:
                observer.spin_for(0.10)
                try:
                    teardown_checks = validate_teardown_trace(observer.records)
                    break
                except MatrixError as error:
                    last_error = error
            else:
                raise MatrixError(
                    f"teardown ROS boundaries did not settle: {last_error}"
                )
        else:
            # A scenario exception already performs visible ZERO/DISARM before
            # the shell invokes its second cleanup layer. Volatile UI boundary
            # messages are correctly absent here; prove the durable downstream
            # zero and closed gate state instead of manufacturing another frame.
            observer.spin_for(1.0)
            teardown_checks = validate_already_disarmed_trace(observer.records)
            result = {"interactions": [], "frames": []}
        frames = browser.probe().get("websocket", [])
        disarm_frame = _control_frame(frames, "disarm") if was_armed else None
        snapshot = browser.snapshot()
        if snapshot.get("armed") or snapshot.get("stateText") not in {"STANDBY", "OFFLINE"}:
            raise MatrixError(f"manual-drive teardown did not disarm: {snapshot!r}")
        observer.close()
        observer = None
        write_json_exclusive(document_path, {
            "schema": SCHEMA,
            "record_type": "session_teardown",
            "status": "PASS",
            "created_at_utc": utc_now(),
            "fail_safe": {
                "zero_before_disarm": was_armed,
                "durable_zero_confirmed": True,
                "frame": disarm_frame,
                "already_disarmed": not was_armed,
            },
            "snapshot_before": snapshot_before,
            "cdp_x11_binding": x11_binding,
            "snapshot": snapshot,
            "ros_observation": {
                "subscriber_only": True,
                "checks": teardown_checks,
            },
            "interactions": result["interactions"],
            "websocket_records": frames,
            "artifacts": {"ros_trace": artifact(trace_path, output)},
        })
        print(f"[manual-4ws] teardown PASS: {document_path}")
        return 0
    except BaseException as error:
        fail_safe_confirmed = _force_visible_fail_safe(browser, args)
        if observer is not None:
            try:
                observer.spin_for(0.5)
            except Exception:
                pass
            observer.close("ERROR", str(error))
        if not document_path.exists():
            write_json_exclusive(document_path, {
                "schema": SCHEMA,
                "record_type": "session_teardown",
                "status": "FAIL",
                "created_at_utc": utc_now(),
                "error": str(error),
                "fail_safe_zero_disarm_attempted": browser is not None,
                "fail_safe_zero_disarm_confirmed": fail_safe_confirmed,
            })
        raise
    finally:
        if browser is not None:
            browser.close()


def _return_error(
    wheel_samples: Sequence[Mapping[str, Any]], phases: Mapping[str, Mapping[str, int]]
) -> dict[str, float]:
    start = _nearest_sample(wheel_samples, int(phases["positive"]["start_ns"]))
    finish = _nearest_sample(wheel_samples, int(phases["inverse"]["end_ns"]))
    a = start["actor"]["transform"]["location_m"]
    b = finish["actor"]["transform"]["location_m"]
    return {
        "translation_m": math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"])),
        "yaw_degrees": abs(normalize_degrees(
            float(finish["actor"]["yaw_degrees"]) - float(start["actor"]["yaw_degrees"])
        )),
    }


def command_evaluate(args: argparse.Namespace) -> int:
    scenario_dir = require_output_directory(args.scenario_dir)
    spec = SCENARIOS[args.scenario]
    manifest_path = scenario_dir / "scenario_manifest.json"
    try:
        run_document = read_json(scenario_dir / "scenario_run.json")
        if run_document.get("scenario") != spec.name or run_document.get("status") != "PASS":
            raise MatrixError("scenario_run.json does not identify a successful requested scenario")
        hold_s = finite(run_document.get("timing", {}).get("hold_s"), "hold_s")
        ui_records = run_document.get("websocket_records", [])
        ui_checks = validate_ui_frames(ui_records, spec, run_document["phases"], hold_s)
        ros_records, ros_footer = load_trace(scenario_dir / "ros_trace.jsonl")
        ros_checks = validate_ros_trace(ros_records, run_document, spec, hold_s)
        wheel_path = scenario_dir / "physical_wheels.jsonl"
        wheel_samples, wheel_footer = load_wheel_samples(wheel_path)
        wheel_manifest_path = scenario_dir / "physical_wheels.manifest.json"
        wheel_manifest = read_json(wheel_manifest_path)
        if wheel_manifest.get("status") not in {"STOPPED", "COMPLETED"}:
            raise MatrixError("physical wheel manifest is not cleanly finalized")
        output_record = wheel_manifest.get("output", {})
        if (
            output_record.get("sha256") != sha256_file(wheel_path)
            or int(output_record.get("bytes", -1)) != wheel_path.stat().st_size
        ):
            raise MatrixError("physical wheel JSONL hash/size does not match its manifest")
        positive = motion_metrics(wheel_samples, run_document["phases"]["positive"])
        inverse = motion_metrics(wheel_samples, run_document["phases"]["inverse"])
        kinematics = validate_motion_metrics(spec, positive, inverse)
        readback = validate_physical_wheel_readback(
            spec, wheel_samples, ros_records, run_document["phases"]
        )
        return_error = _return_error(wheel_samples, run_document["phases"])
        if return_error["translation_m"] > 0.50 or return_error["yaw_degrees"] > 12.0:
            raise MatrixError(
                "inverse leg did not approximately retrace the test motion: "
                f"translation={return_error['translation_m']:.3f} m "
                f"yaw={return_error['yaw_degrees']:.3f} deg"
            )
        wheel_identity = wheel_samples[0]["actor"]
        if int(wheel_identity["id"]) != int(ros_checks["actor_id"]):
            raise MatrixError("ROS physical status actor id differs from wheel telemetry actor id")
        visual = validate_capture(scenario_dir)
        wheel_summary_dir = scenario_dir / "wheel_summary"
        summary_artifacts = {
            name: artifact(wheel_summary_dir / name, scenario_dir)
            for name in (
                "wheel_summary.json", "wheel_measurements.csv",
                "wheel_summary.png", "SHA256SUMS",
            )
        }
        document = {
            "schema": SCENARIO_MANIFEST_SCHEMA,
            "scenario": spec.name,
            "label": spec.label,
            "status": "PASS",
            "created_at_utc": utc_now(),
            "authority": {
                "command_source": "visible Robot UI CDP pointer/keyboard input",
                "page_owned_manual_websocket": True,
                "direct_websocket_send": False,
                "ros_subscriber_only": True,
                "carla_observer_only": True,
            },
            "acceptance": {
                "ui_protocol": ui_checks,
                "ros_pipeline": ros_checks,
                "physical_kinematics": kinematics,
                "physical_wheel_readback": readback,
                "inverse_retrace_error": return_error,
                "actual_x11_capture": visual,
            },
            "actor": {
                "id": int(wheel_identity["id"]),
                "type_id": wheel_identity["type_id"],
                "role_name": wheel_identity["role_name"],
            },
            "footer_status": {
                "ros": ros_footer.get("status"),
                "physical_wheels": wheel_footer.get("status"),
            },
            "artifacts": {
                "scenario_run": artifact(scenario_dir / "scenario_run.json", scenario_dir),
                "ui_interactions": artifact(scenario_dir / "ui_interactions.json", scenario_dir),
                "ros_trace": artifact(scenario_dir / "ros_trace.jsonl", scenario_dir),
                "physical_wheels": artifact(wheel_path, scenario_dir),
                "physical_wheels_manifest": artifact(wheel_manifest_path, scenario_dir),
                "wheel_summary": summary_artifacts,
                "visual": visual["artifacts"],
            },
        }
        write_json_exclusive(manifest_path, document)
        print(f"[manual-4ws] {spec.name} full evidence PASS: {manifest_path}")
        return 0
    except BaseException as error:
        if not manifest_path.exists():
            write_json_exclusive(manifest_path, {
                "schema": SCENARIO_MANIFEST_SCHEMA,
                "scenario": spec.name,
                "status": "FAIL",
                "created_at_utc": utc_now(),
                "error": str(error),
            })
        raise


def write_text_exclusive(path: Path, text: str) -> None:
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
        stream.write(text)
        stream.flush()
        os.fsync(stream.fileno())


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def publish_collection_atomically(
    root: Path,
    document: Mapping[str, Any],
    csv_text: str,
    report_text: str,
) -> None:
    """Publish a collection with the JSON status as the final commit marker.

    All content and hashes are completed in a same-filesystem private staging
    directory.  CSV, report, and hashes are linked into place first; the
    summary JSON is linked last.  Any exception removes only artifacts linked
    by this invocation, so the shell cleanup can still publish a FAIL summary.
    """
    filenames = (
        "manual_4ws_summary.json",
        "manual_4ws_summary.csv",
        "manual_4ws_report.md",
        "SHA256SUMS",
    )
    finals = {name: root / name for name in filenames}
    occupied = [str(path) for path in finals.values() if path.exists() or path.is_symlink()]
    if occupied:
        raise MatrixError(
            "refusing to overwrite collection artifacts: " + ", ".join(occupied)
        )
    staging = Path(tempfile.mkdtemp(prefix=".manual-4ws-summary-", dir=root))
    published: list[Path] = []
    try:
        write_json_exclusive(staging / "manual_4ws_summary.json", document)
        write_text_exclusive(staging / "manual_4ws_summary.csv", csv_text)
        write_text_exclusive(staging / "manual_4ws_report.md", report_text)

        hash_entries: dict[str, str] = {}
        for path in root.rglob("*"):
            if staging == path or staging in path.parents:
                continue
            if path in finals.values():
                continue
            if path.is_file() and not path.is_symlink():
                relative = str(path.relative_to(root))
                hash_entries[relative] = sha256_file(path)
        for name in filenames[:3]:
            hash_entries[name] = sha256_file(staging / name)
        hashes_text = "".join(
            f"{digest}  {relative}\n"
            for relative, digest in sorted(hash_entries.items())
        )
        write_text_exclusive(staging / "SHA256SUMS", hashes_text)
        _fsync_directory(staging)

        # The PASS/FAIL summary is intentionally last: observing it guarantees
        # that every sibling index/report/hash artifact was already durable.
        for name in (
            "manual_4ws_summary.csv",
            "manual_4ws_report.md",
            "SHA256SUMS",
            "manual_4ws_summary.json",
        ):
            os.link(staging / name, finals[name])
            published.append(finals[name])
        _fsync_directory(root)
    except BaseException:
        for path in reversed(published):
            try:
                path.unlink()
            except OSError:
                pass
        try:
            _fsync_directory(root)
        except OSError:
            pass
        raise
    finally:
        for name in filenames:
            try:
                (staging / name).unlink()
            except OSError:
                pass
        try:
            staging.rmdir()
        except OSError:
            pass


def command_summarize(args: argparse.Namespace) -> int:
    root = require_output_directory(args.output_root)
    summary_path = root / "manual_4ws_summary.json"
    manifests: list[dict[str, Any]] = []
    for name in EXPECTED_SCENARIOS:
        path = root / name / "scenario_manifest.json"
        if path.is_file():
            manifests.append(read_json(path))
    requested_status = args.status
    if requested_status == "auto":
        missing = sorted(set(EXPECTED_SCENARIOS) - {item.get("scenario") for item in manifests})
        failed = [item.get("scenario") for item in manifests if item.get("status") != "PASS"]
        if missing or failed:
            raise MatrixError(f"cannot finalize PASS collection; missing={missing} failed={failed}")
        status = "PASS"
    else:
        status = "FAIL"
    setup_path = root / "session" / "session_setup.json"
    teardown_path = root / "session_teardown" / "session_teardown.json"
    if status == "PASS":
        if read_json(setup_path).get("status") != "PASS":
            raise MatrixError("manual evidence setup did not pass")
        if read_json(teardown_path).get("status") != "PASS":
            raise MatrixError("manual evidence fail-safe teardown did not pass")
    document = {
        "schema": COLLECTION_SCHEMA,
        "status": status,
        "created_at_utc": utc_now(),
        "scenario_order": list(EXPECTED_SCENARIOS),
        "completed_scenarios": [item.get("scenario") for item in manifests],
        "passed_scenarios": [
            item.get("scenario") for item in manifests if item.get("status") == "PASS"
        ],
        "failure": {
            "scenario": args.failure_scenario,
            "reason": args.failure_reason,
        } if status == "FAIL" else None,
        "session": {
            "setup": artifact(setup_path, root) if setup_path.is_file() else None,
            "teardown": artifact(teardown_path, root) if teardown_path.is_file() else None,
        },
        "scenario_manifests": [
            artifact(root / str(item["scenario"]) / "scenario_manifest.json", root)
            for item in manifests
        ],
        "statement": (
            "Every vehicle command was generated by real CDP pointer/keyboard input "
            "to the visible Robot UI. ROS and patched CARLA wheel telemetry were "
            "subscriber/readback-only evidence sources."
        ),
    }
    rows = [
        ["scenario", "status", "path_positive_m", "path_inverse_m", "return_translation_m", "return_yaw_deg"],
    ]
    for item in manifests:
        acceptance = item.get("acceptance", {})
        kinematics = acceptance.get("physical_kinematics", {})
        retrace = acceptance.get("inverse_retrace_error", {})
        rows.append([
            str(item.get("scenario", "")), str(item.get("status", "")),
            str(kinematics.get("positive", {}).get("path_distance_m", "")),
            str(kinematics.get("inverse", {}).get("path_distance_m", "")),
            str(retrace.get("translation_m", "")), str(retrace.get("yaw_degrees", "")),
        ])
    import io
    buffer = io.StringIO(newline="")
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerows(rows)
    report = [
        "# Manual 4WS CARLA evidence",
        "",
        f"- Status: **{status}**",
        "- Authority: visible Robot UI administrator surface and real CDP input events",
        "- Observers: ROS subscriber-only trace and patched CARLA physical-wheel readback",
        "",
        "| Scenario | Status | PNG | GIF | Physical wheels |",
        "|---|---:|---:|---:|---:|",
    ]
    by_name = {str(item.get("scenario")): item for item in manifests}
    for name in EXPECTED_SCENARIOS:
        item = by_name.get(name, {})
        report.append(
            f"| {name} | {item.get('status', 'NOT RUN')} | "
            f"{name}/visual/representative_contact_sheet.png | "
            f"{name}/visual/representative_motion.gif | "
            f"{name}/wheel_summary/wheel_summary.png |"
        )
    if status == "FAIL":
        report.extend(["", f"Failure: `{args.failure_scenario}` — {args.failure_reason}"])
    publish_collection_atomically(
        root,
        document,
        buffer.getvalue(),
        "\n".join(report) + "\n",
    )
    print(f"[manual-4ws] collection {status}: {summary_path}")
    return 0 if status == "PASS" else 1


def print_plan(args: argparse.Namespace) -> int:
    print("[manual-4ws] PLAN ONLY -- no CDP, ROS, CARLA, X11, or filesystem action occurred.")
    print("Scenario order: straight, turn, crab, zero_turn")
    print("Command authority: visible Robot UI admin form -> real CDP click/key events only")
    print("Observers: ROS subscriptions + record_physical_wheel_telemetry.py")
    print("Visuals: capture_ui_evidence.sh actual side-by-side X11 PNG/GIF")
    print("Fail-safe: release held keys -> Space ZERO -> Escape DISARM")
    return 0


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    subparsers = result.add_subparsers(dest="action")
    subparsers.add_parser("plan", help="print the offline plan (default)")

    def live(name: str, help_text: str) -> argparse.ArgumentParser:
        command = subparsers.add_parser(name, help=help_text)
        command.add_argument("--output-dir", required=True)
        command.add_argument("--ui-url", default="http://127.0.0.1:8010")
        command.add_argument("--operator-cdp-url", default="http://127.0.0.1:9224")
        command.add_argument("--role-name", default="ego_vehicle")
        command.add_argument(
            "--x11-window-id", required=True,
            help="explicit X11 Robot UI window id proven to own the CDP target",
        )
        command.add_argument("--display", required=True)
        command.add_argument("--xauthority", default="")
        command.add_argument("--timeout-s", type=float, default=10.0)
        command.add_argument("--ros-ready-timeout-s", type=float, default=10.0)
        return command

    live("setup", "authenticate visibly and ARM through the Robot UI")
    scenario = live("scenario", "run one keyboard scenario and record ROS observations")
    scenario.add_argument("--scenario", choices=EXPECTED_SCENARIOS, required=True)
    scenario.add_argument("--initial-zero-s", type=float, default=1.0)
    scenario.add_argument("--hold-s", type=float, default=4.0)
    scenario.add_argument("--zero-hold-s", type=float, default=2.0)
    scenario.add_argument("--settle-s", type=float, default=2.0)
    live("teardown", "send visible-page ZERO and DISARM")
    evaluate = subparsers.add_parser("evaluate", help="offline fail-closed scenario acceptance")
    evaluate.add_argument("--scenario", choices=EXPECTED_SCENARIOS, required=True)
    evaluate.add_argument("--scenario-dir", required=True)
    summarize = subparsers.add_parser("summarize", help="write collection index/report/hashes")
    summarize.add_argument("--output-root", required=True)
    summarize.add_argument("--status", choices=("auto", "FAIL"), default="auto")
    summarize.add_argument("--failure-scenario", default="")
    summarize.add_argument("--failure-reason", default="")
    return result


def validate_cli(args: argparse.Namespace) -> None:
    for name in ("timeout_s", "ros_ready_timeout_s"):
        if hasattr(args, name) and not 0.1 <= finite(getattr(args, name), name) <= 120.0:
            raise MatrixError(f"{name} must be in [0.1, 120]")
    for name in ("initial_zero_s", "hold_s", "zero_hold_s", "settle_s"):
        if hasattr(args, name) and not 0.25 <= finite(getattr(args, name), name) <= 30.0:
            raise MatrixError(f"{name} must be in [0.25, 30]")
    if hasattr(args, "role_name") and args.role_name != "ego_vehicle":
        raise MatrixError("manual physical evidence is restricted to role_name=ego_vehicle")
    if hasattr(args, "x11_window_id") and re.fullmatch(
        r"0x[0-9a-fA-F]+", str(args.x11_window_id)
    ) is None:
        raise MatrixError("x11_window_id must be an explicit hexadecimal X11 id")


def main(argv: Sequence[str] | None = None) -> int:
    args = parser().parse_args(argv)
    if args.action in (None, "plan"):
        return print_plan(args)
    try:
        validate_cli(args)
        if args.action == "setup":
            return command_setup(args)
        if args.action == "scenario":
            return command_scenario(args)
        if args.action == "teardown":
            return command_teardown(args)
        if args.action == "evaluate":
            return command_evaluate(args)
        if args.action == "summarize":
            return command_summarize(args)
        raise MatrixError(f"unknown action: {args.action}")
    except (MatrixError, OSError, ValueError, KeyError, TypeError) as error:
        print(f"[manual-4ws] ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
