"""HJ_261120 - Site PINs and the shared manager account for manager_ui.

One JSON file under the per-user state root holds, per site, the 4-digit PIN
campers type before the robot is sent (Robot UI) or called (Guest UI), the
sites a manager closed for delivery ("disabled"), plus the single shared
manager account. manager_ui writes it; ui_backend_node and
ui_guest_node only read it (they reload when the file changes), so the three
processes never need a network hop to agree.

PINs are stored readable on purpose: staff must be able to show a camper the
current number. The file is created 0600. The manager password is stored only
as a salted PBKDF2 hash.
"""

from __future__ import annotations

import fcntl
import hashlib
import hmac
import json
import os
import secrets
import threading
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Optional

PIN_LENGTH = 4
DEFAULT_MANAGER_USERNAME = "admin"
DEFAULT_MANAGER_PASSWORD = "1234"
PBKDF2_ITERATIONS = 200_000
HISTORY_LIMIT = 500


def default_site_access_path() -> Path:
    state_root = os.environ.get("XDG_STATE_HOME", "").strip()
    root = Path(state_root).expanduser() if state_root else Path.home() / ".local" / "state"
    return root / "camrod" / "site_access.json"


def valid_pin(pin: object) -> bool:
    return isinstance(pin, str) and len(pin) == PIN_LENGTH and pin.isascii() and pin.isdigit()


def random_pin() -> str:
    return f"{secrets.randbelow(10 ** PIN_LENGTH):0{PIN_LENGTH}d}"


def _now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def _hash_password(password: str, salt: bytes, iterations: int = PBKDF2_ITERATIONS) -> str:
    return hashlib.pbkdf2_hmac("sha256", password.encode("utf-8"), salt, iterations).hex()


class SiteAccessStore:
    """File-backed PIN/account store shared by the three UI processes."""

    def __init__(self, path: Optional[Path] = None, sites: Iterable[str] = ()) -> None:
        self.path = Path(path) if path else default_site_access_path()
        self.sites = [str(site).strip().upper() for site in sites if str(site).strip()]
        self._lock = threading.RLock()
        self._cache: Optional[dict] = None
        self._cache_mtime_ns: Optional[int] = None

    # ── file I/O ────────────────────────────────────────────────────────
    @contextmanager
    def _file_lock(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        lock_path = self.path.with_suffix(self.path.suffix + ".lock")
        with open(lock_path, "a+") as handle:
            fcntl.flock(handle, fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(handle, fcntl.LOCK_UN)

    def _read_file(self) -> dict:
        try:
            stat = self.path.stat()
        except FileNotFoundError:
            return {}
        if self._cache is not None and self._cache_mtime_ns == stat.st_mtime_ns:
            return self._cache
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            data = {}
        if not isinstance(data, dict):
            data = {}
        self._cache, self._cache_mtime_ns = data, stat.st_mtime_ns
        return data

    def _write_file(self, data: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(self.path.suffix + f".{os.getpid()}.tmp")
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(data, handle, ensure_ascii=False, indent=2)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, self.path)
        os.chmod(self.path, 0o600)
        self._cache = data
        self._cache_mtime_ns = self.path.stat().st_mtime_ns

    def _normalized(self, data: dict) -> tuple[dict, bool]:
        """Fill missing sections; every configured site gets a random PIN once."""
        changed = False
        pins = data.get("pins")
        if not isinstance(pins, dict):
            pins, changed = {}, True
        for site in self.sites:
            entry = pins.get(site)
            if not isinstance(entry, dict) or not valid_pin(entry.get("pin")):
                pins[site] = {"pin": random_pin(), "updated_at": _now_iso(), "updated_by": "system"}
                changed = True
        manager = data.get("manager")
        if not isinstance(manager, dict) or not manager.get("hash"):
            salt = secrets.token_bytes(16)
            manager = {
                "username": os.environ.get("CAMROD_MANAGER_USER", DEFAULT_MANAGER_USERNAME),
                "salt": salt.hex(),
                "iterations": PBKDF2_ITERATIONS,
                "hash": _hash_password(
                    os.environ.get("CAMROD_MANAGER_PASSWORD", DEFAULT_MANAGER_PASSWORD), salt),
                # The shipped default must be replaced on first login.
                "must_change": True,
                "updated_at": _now_iso(),
            }
            changed = True
        history = data.get("history")
        if not isinstance(history, list):
            history, changed = [], True
        # HJ_261120 - Sites a manager closed for delivery ("사용 중" 지정). Optional
        # section, kept as-is; entries for sites no longer configured are ignored.
        disabled = data.get("disabled")
        if not isinstance(disabled, dict):
            disabled = {}
        disabled = {site: entry for site, entry in disabled.items()
                    if isinstance(site, str) and isinstance(entry, dict)}
        return {"version": 1, "pins": pins, "manager": manager, "history": history,
                "disabled": disabled}, changed

    def _load(self, persist: bool) -> dict:
        data, changed = self._normalized(dict(self._read_file()))
        if changed and persist:
            self._write_file(data)
        return data

    def _mutate(self, apply) -> object:
        with self._lock, self._file_lock():
            self._cache = None  # another process may have written since our last read
            data = self._load(persist=False)
            result = apply(data)
            self._write_file(data)
            return result

    def ensure_initialized(self) -> None:
        with self._lock, self._file_lock():
            self._cache = None
            self._load(persist=True)

    # ── PINs ────────────────────────────────────────────────────────────
    def pins(self) -> Dict[str, dict]:
        with self._lock:
            data = self._load(persist=False)
            return {site: dict(data["pins"][site]) for site in self.sites if site in data["pins"]}

    def verify_pin(self, site: str, pin: str) -> bool:
        site = str(site or "").strip().upper()
        if site not in self.sites or not valid_pin(pin):
            return False
        with self._lock:
            entry = self._load(persist=False)["pins"].get(site) or {}
        expected = str(entry.get("pin", ""))
        return valid_pin(expected) and hmac.compare_digest(expected, pin)

    def set_pin(self, site: str, pin: str, by: str) -> dict:
        site = str(site or "").strip().upper()
        if site not in self.sites:
            raise ValueError("unknown_site")
        if not valid_pin(pin):
            raise ValueError("invalid_pin")

        def apply(data):
            entry = {"pin": pin, "updated_at": _now_iso(), "updated_by": str(by)}
            data["pins"][site] = entry
            data["history"].append({"at": entry["updated_at"], "by": str(by), "site": site,
                                    "action": "pin_changed"})
            del data["history"][:-HISTORY_LIMIT]
            return dict(entry)

        return self._mutate(apply)

    # ── HJ_261120 - sites closed for delivery ───────────────────────────
    # manager_ui marks a site "사용 중" (people already there). The Robot UI then
    # shows it as unavailable and the backend refuses a delivery to it; a recall
    # to that site and a mission already under way are not affected.
    def disabled_sites(self) -> Dict[str, dict]:
        with self._lock:
            disabled = self._load(persist=False)["disabled"]
            return {site: dict(disabled[site]) for site in self.sites if site in disabled}

    def set_site_disabled(self, site: str, disabled: bool, by: str) -> Dict[str, dict]:
        site = str(site or "").strip().upper()
        if site not in self.sites:
            raise ValueError("unknown_site")

        def apply(data):
            current = data["disabled"]
            if bool(disabled) == (site in current):
                return  # no change, no history noise
            if disabled:
                current[site] = {"at": _now_iso(), "by": str(by)}
            else:
                current.pop(site, None)
            data["history"].append({"at": _now_iso(), "by": str(by), "site": site,
                                    "action": "site_disabled" if disabled else "site_enabled"})
            del data["history"][:-HISTORY_LIMIT]

        self._mutate(apply)
        return self.disabled_sites()

    # ── shared manager account ──────────────────────────────────────────
    def manager_username(self) -> str:
        with self._lock:
            return str(self._load(persist=False)["manager"]["username"])

    def manager_must_change(self) -> bool:
        with self._lock:
            return bool(self._load(persist=False)["manager"].get("must_change", False))

    def verify_manager(self, username: str, password: str) -> bool:
        with self._lock:
            manager = self._load(persist=False)["manager"]
        try:
            salt = bytes.fromhex(manager["salt"])
            iterations = int(manager.get("iterations", PBKDF2_ITERATIONS))
        except (KeyError, ValueError, TypeError):
            return False
        candidate = _hash_password(str(password or ""), salt, iterations)
        same_user = hmac.compare_digest(str(manager.get("username", "")), str(username or ""))
        return hmac.compare_digest(candidate, str(manager.get("hash", ""))) and same_user

    def change_manager(self, current_password: str, new_username: str, new_password: str) -> None:
        new_username = str(new_username or "").strip()
        if not (3 <= len(new_username) <= 32) or not all(c.isalnum() or c in "._-" for c in new_username):
            raise ValueError("invalid_username")
        if len(str(new_password or "")) < 4:  # HJ_261120 - 4자 이상 (운영 요청)
            raise ValueError("weak_password")
        if not self.verify_manager(self.manager_username(), current_password):
            raise PermissionError("wrong_password")

        def apply(data):
            salt = secrets.token_bytes(16)
            data["manager"] = {
                "username": new_username, "salt": salt.hex(), "iterations": PBKDF2_ITERATIONS,
                "hash": _hash_password(new_password, salt), "must_change": False,
                "updated_at": _now_iso(),
            }
            data["history"].append({"at": _now_iso(), "by": new_username, "site": "",
                                    "action": "manager_changed"})
            del data["history"][:-HISTORY_LIMIT]

        self._mutate(apply)

    def history(self, limit: int = 100) -> List[dict]:
        with self._lock:
            items = list(self._load(persist=False)["history"])
        return list(reversed(items))[: max(0, int(limit))]


class AttemptLimiter:
    """Lock a key (site or client) after repeated failures."""

    def __init__(self, max_failures: int, lockout_s: float, clock=time.monotonic) -> None:
        self.max_failures = int(max_failures)
        self.lockout_s = float(lockout_s)
        self._clock = clock
        self._lock = threading.Lock()
        self._failures: Dict[str, int] = {}
        self._locked_until: Dict[str, float] = {}

    def remaining_lock_s(self, key: str) -> float:
        with self._lock:
            until = self._locked_until.get(key, 0.0)
            left = until - self._clock()
            if left <= 0 and key in self._locked_until:
                self._locked_until.pop(key, None)
                self._failures.pop(key, None)
            return max(0.0, left)

    def record(self, key: str, ok: bool) -> None:
        with self._lock:
            if ok:
                self._failures.pop(key, None)
                self._locked_until.pop(key, None)
                return
            count = self._failures.get(key, 0) + 1
            self._failures[key] = count
            if count >= self.max_failures:
                self._locked_until[key] = self._clock() + self.lockout_s
