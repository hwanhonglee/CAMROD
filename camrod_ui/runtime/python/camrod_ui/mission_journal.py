"""HH_260915 - Independent, passive mission journal; never opens service_metrics DB.

The mode axis describes the observed platform CAN(1)/RC(0) mode, not a
certification that every CAN command originated in an autonomous planner.
One writer owns this directory. Readers use the atomically exported snapshot.
"""

from __future__ import annotations

import fcntl
import json
import math
import os
from pathlib import Path
import re
import sqlite3
import threading
import time
import uuid
from datetime import datetime
from zoneinfo import ZoneInfo


MODES = ("auto", "manual", "unknown")
INTENTS = ("delivery", "recall", "return", "other")
LIFECYCLE_EVENTS = frozenset({
    "mission_started", "phase", "return_requested", "stop_requested",
    "mission_cancelled",
})
OPEN_RESULTS = frozenset({"active", "paused"})
KST = ZoneInfo("Asia/Seoul")


def default_mission_records_root() -> Path:
    state = os.environ.get("XDG_STATE_HOME")
    return (Path(state).expanduser() if state else Path.home() / ".local/state") / "camrod/mission_records"


def _finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def _iso(stamp):
    return datetime.fromtimestamp(float(stamp), KST).isoformat(timespec="milliseconds")


def _token(value, default="other"):
    return re.sub(r"[^A-Za-z0-9_-]+", "_", str(value))[:48].strip("_") or default


def _text(value, limit=512):
    return str(value or "")[:limit]


def _json(value):
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), allow_nan=False)


def _totals():
    return {"autonomous_m": 0.0, "manual_m": 0.0, "unknown_m": 0.0, "total_m": 0.0}


def _distance_key(mode):
    return "autonomous_m" if mode == "auto" else f"{mode}_m"


class MissionJournal:
    def __init__(
        self, root, *, robot_id="unknown", environment="real", now_fn=time.time,
        minimum_speed_mps=0.03, maximum_speed_mps=3.0,
        maximum_sample_gap_s=2.0, stop_dwell_s=0.5,
        rotation_bytes=4 * 1024 * 1024, quota_bytes=256 * 1024 * 1024,
    ):
        if environment not in {"real", "simulation", "test"}:
            raise ValueError("environment must be real, simulation or test")
        parameters = (minimum_speed_mps, maximum_speed_mps, maximum_sample_gap_s, stop_dwell_s)
        if not all(_finite(v) for v in parameters) or not (
            0 <= minimum_speed_mps < maximum_speed_mps and maximum_sample_gap_s > 0
            and stop_dwell_s >= 0
        ):
            raise ValueError("invalid observation thresholds")
        if isinstance(rotation_bytes, bool) or isinstance(quota_bytes, bool) or not (
            int(rotation_bytes) >= 64 and int(quota_bytes) >= int(rotation_bytes)
        ):
            raise ValueError("invalid journal rotation/quota bounds")
        self.root = Path(root).expanduser().absolute()
        if self.root.is_symlink() or self.root == Path("/"):
            raise ValueError("journal root must be a dedicated non-symlink directory")
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.robot_id, self.environment = _text(robot_id, 128), environment
        self._now = now_fn
        self.minimum_speed_mps = float(minimum_speed_mps)
        self.maximum_speed_mps = float(maximum_speed_mps)
        self.maximum_sample_gap_s = float(maximum_sample_gap_s)
        self.stop_dwell_s = float(stop_dwell_s)
        self.rotation_bytes, self.quota_bytes = int(rotation_bytes), int(quota_bytes)
        self._mutex = threading.RLock()
        self._closed = False
        self._closing = False
        self._db = None
        self._lock_fd = os.open(str(self.root / ".writer.lock"), os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            database = self.root / "mission_journal.sqlite3"
            if database.is_symlink():
                raise ValueError("journal database must not be a symlink")
            self._db = sqlite3.connect(str(database), timeout=2, check_same_thread=False)
            self._db.execute("CREATE TABLE IF NOT EXISTS metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL)")
            stored_profile = self._get_meta("profile")
            profile = {"robot_id": self.robot_id, "environment": environment, "schema_version": 1}
            if stored_profile is not None and stored_profile != profile:
                raise ValueError("journal robot/environment/schema differs; use a separate storage root")
            self._db.execute("PRAGMA journal_mode=WAL")
            self._db.execute("PRAGMA synchronous=NORMAL")
            self._db.execute("CREATE TABLE IF NOT EXISTS missions (id TEXT PRIMARY KEY, date TEXT NOT NULL, sequence INTEGER NOT NULL, payload TEXT NOT NULL, UNIQUE(date,sequence))")
            self._db.execute("CREATE TABLE IF NOT EXISTS producers (session TEXT PRIMARY KEY, seq INTEGER NOT NULL)")
            self._db.execute("CREATE TABLE IF NOT EXISTS log_files (path TEXT PRIMARY KEY)")
            self._put_meta("profile", profile)
            self._missions = {row[0]: json.loads(row[1]) for row in self._db.execute("SELECT id,payload FROM missions ORDER BY date,sequence")}
            self._current = next((m for m in reversed(list(self._missions.values())) if m["result"] in OPEN_RESULTS), None)
            self._outside = self._get_meta("outside") or {**_totals(), "session_count": 0}
            self._errors = self._get_meta("errors") or []
            self._last_event_at = self._get_meta("last_event_at") or 0.0
            self._files = {}
            for (relative,) in self._db.execute("SELECT path FROM log_files"):
                path = self._owned_path(relative)
                self._files[relative] = path.stat().st_size if path.exists() else 0
            self._db.commit()
        except Exception:
            if self._db is not None:
                self._db.close()
            os.close(self._lock_fd)
            raise
        # HH_260915 - Never integrate from a persisted velocity across restart.
        self._previous = None
        self._last_sample_at = None
        self._mode = "unknown"
        self._gate = {}
        self._health = {}
        self._raw_can_status, self._raw_can_details = "disabled", {}
        self._outside_moving = False
        self._zero_elapsed = 0.0
        self._stop_active = False
        self._stop_reason = ""
        self._closed_snapshot = None
        if self._current is not None:
            self._current["current_mode"] = "unknown"

    def _get_meta(self, key):
        row = self._db.execute("SELECT value FROM metadata WHERE key=?", (key,)).fetchone()
        return json.loads(row[0]) if row else None

    def _put_meta(self, key, value):
        self._db.execute("INSERT INTO metadata VALUES (?,?) ON CONFLICT(key) DO UPDATE SET value=excluded.value", (key, _json(value)))

    def _owned_path(self, relative):
        part = Path(relative)
        if part.is_absolute() or ".." in part.parts:
            raise ValueError("invalid journal artifact path")
        result = self.root / part
        for component in (result, *list(result.parents)[:len(part.parts)]):
            if component.is_symlink():
                raise ValueError("journal artifact must not use symlinks")
        return result

    def _error(self, reason):
        reason = _text(reason)
        if reason and reason not in self._errors:
            self._errors.append(reason)
            self._errors = self._errors[-20:]
        if self._current is not None:
            self._current["incomplete"] = True

    def mark_incomplete(self, reason):
        with self._mutex:
            self._error(reason)
            self._previous = None
            self._zero_elapsed = 0.0
            self._mode = "unknown"
            self._stop_active = False
            if self._current is not None:
                self._current["current_mode"] = "unknown"

    def set_recorder_status(self, values):
        with self._mutex:
            protected = {"status", "error", "storage_root", "environment", "robot_id", "last_sample_at", "raw_can_status"}
            self._health.update({k: v for k, v in dict(values).items() if k not in protected})

    def set_raw_can_status(self, value):
        with self._mutex:
            self._raw_can_details = dict(value) if isinstance(value, dict) else {}
            self._raw_can_status = _text(value.get("state", "unknown") if isinstance(value, dict) else value, 128)

    def _append(self, kind, value, at):
        if self._closed:
            return False
        try:
            encoded = (_json(value) + "\n").encode("utf-8")
            if len(encoded) > self.rotation_bytes:
                raise OSError("journal record exceeds rotation_bytes; record not truncated")
            if sum(self._files.values()) + len(encoded) > self.quota_bytes:
                raise OSError("journal quota reached; existing records preserved")
            directory = self._current["_directory"] if self._current else f"{datetime.fromtimestamp(at, KST):%Y-%m-%d}/outside"
            prefix = f"{directory}/{kind}_"
            matches = sorted(path for path in self._files if path.startswith(prefix))
            relative = matches[-1] if matches else f"{prefix}000001.jsonl"
            if self._files.get(relative, 0) + len(encoded) > self.rotation_bytes:
                index = int(Path(relative).stem.rsplit("_", 1)[1]) + 1
                relative = f"{prefix}{index:06d}.jsonl"
            path = self._owned_path(relative)
            path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
            if relative not in self._files:
                self._db.execute("INSERT OR IGNORE INTO log_files VALUES (?)", (relative,))
                self._db.commit()
                self._files[relative] = 0
            fd = os.open(str(path), os.O_WRONLY | os.O_APPEND | os.O_CREAT | os.O_NOFOLLOW, 0o600)
            try:
                with os.fdopen(fd, "ab") as stream:
                    stream.write(encoded)
            finally:
                self._files[relative] = path.stat().st_size
            if self._current is not None:
                entries = self._current["files"].setdefault(kind, [])
                if relative not in entries:
                    entries.append(relative)
            return True
        except (OSError, ValueError, TypeError) as exc:
            self._error(f"{kind}: {exc}")
            return False

    def _save(self, mission=None):
        mission = self._current if mission is None else mission
        if mission is not None:
            self._db.execute("INSERT INTO missions VALUES (?,?,?,?) ON CONFLICT(id) DO UPDATE SET payload=excluded.payload", (mission["id"], mission["date"], mission["sequence"], _json(mission)))

    def _record_event(self, event, at, *, reason="", source="", **details):
        item = {"at": _iso(at), "at_unix": at, "event": event, "mode": self._mode,
                "phase": self._current["phase"] if self._current else "outside",
                "reason": _text(reason), "source": _text(source), **details}
        if self._current is not None:
            self._current["events"].append(item)
            self._current["events"] = self._current["events"][-200:]
            self._current["event_count"] += 1
        self._append("events", item, at)
        return item

    def _new_mission(self, event, at):
        identity = _text(event.get("mission_id"), 256) or f"return-{uuid.uuid4().hex}"
        date = datetime.fromtimestamp(at, KST).date().isoformat()
        # HH_260915 - Allocated by the sole DB writer, not inferred from UI sorting.
        self._db.execute("BEGIN IMMEDIATE")
        sequence = self._db.execute("SELECT COALESCE(MAX(sequence),0)+1 FROM missions WHERE date=?", (date,)).fetchone()[0]
        site = _text(event.get("site"), 64) or "DROP_ZONE"
        intent = event.get("intent") if event.get("intent") in INTENTS else "other"
        mission = {
            "id": identity, "name": f"{date} #{sequence:03d} {site} {intent}",
            "date": date, "sequence": sequence, "site": site, "intent": intent,
            "result": "active", "phase": "outbound", **_totals(),
            "current_mode": "unknown", "manual_interventions": 0,
            "started_at": _iso(at), "ended_at": None, "duration_s": 0.0,
            "stop_duration_s": 0.0, "stop_count": 0, "attempt_count": 0,
            "events": [], "event_count": 0, "files": {}, "incomplete": False,
            "_started_unix": at, "_ended_unix": None, "_attempt_ids": [],
            "_return_armed": False,
            "_directory": f"{date}/{sequence:03d}_{_token(site)}_{_token(intent)}_{uuid.uuid4().hex[:8]}",
        }
        self._current = mission
        self._missions[identity] = mission
        self._save()
        self._db.commit()
        self._reset_motion_boundary()
        return mission

    def _reset_motion_boundary(self):
        self._previous = None
        self._mode = "unknown"
        self._zero_elapsed, self._stop_active = 0.0, False
        self._outside_moving = False

    def _finish(self, result, at, reason=""):
        mission = self._current
        if mission is None:
            return
        self._record_event(result, at, reason=reason, source="lifecycle")
        mission["result"] = result
        mission["ended_at"] = _iso(at)
        mission["_ended_unix"] = at
        mission["duration_s"] = max(0.0, at - mission["_started_unix"])
        self._save()
        self._current = None
        self._reset_motion_boundary()

    @staticmethod
    def _phase(event, fallback):
        raw = str(event.get("phase", "")).lower()
        if raw in {"outbound", "site", "return", "parking"}:
            return raw
        state = event.get("state")
        if state == 10:
            return "parking"
        if state == 3 or event.get("leg_kind") == "return":
            return "return"
        if state in {5, 6, 8, 11} or (state == 9 and event.get("leg_kind") == "recall"):
            return "site"
        if state in {1, 7, 14, 15}:
            return "outbound"
        return fallback

    def observe_event(self, event, received_unix=None):
        at = event.get("at_unix")
        received = self._now() if received_unix is None else received_unix
        seq, producer = event.get("seq"), _text(event.get("producer_session"), 256)
        kind = event.get("event")
        if not (event.get("schema") == 1 and kind in LIFECYCLE_EVENTS and producer
                and isinstance(seq, int) and not isinstance(seq, bool) and seq >= 0
                and _finite(at) and at >= 0 and _finite(received)):
            return False
        with self._mutex:
            if self._closed or at < self._last_event_at:
                return False
            previous = self._db.execute("SELECT seq FROM producers WHERE session=?", (producer,)).fetchone()
            if previous is not None and seq <= previous[0]:
                return False
            if previous is not None and seq > previous[0] + 1:
                self.mark_incomplete(f"lifecycle_sequence_gap:{producer}:{previous[0]}->{seq}")
            identity = _text(event.get("mission_id"), 256)
            if kind == "stop_requested" and not identity and event.get("global_stop") is True:
                # HH_260915 - The backend explicitly observed a global STOP.
                # Pause the one open envelope after backend restart, but do not
                # adopt unrelated blank phases or create an idle zero-metre trip.
                if self._current is not None:
                    identity = self._current["id"]
                else:
                    self._record_event(kind, at, reason=event.get("reason", ""),
                                       source=producer, global_stop=True, scope="outside")
                    self._last_event_at = at
                    self._put_meta("last_event_at", at)
                    self._db.execute("INSERT INTO producers VALUES (?,?) ON CONFLICT(session) DO UPDATE SET seq=excluded.seq", (producer, seq))
                    self._db.commit()
                    return True
            if kind == "phase" and not identity and self._current is not None:
                # HH_260915 - After backend restart, only the producer whose
                # admitted blank Return bound this persisted mission may finish
                # it with blank phase messages. No uncorrelated stream adoption.
                bound = self._get_meta(f"return_binding:{producer}")
                if bound == self._current["id"] and self._current["_return_armed"]:
                    identity = bound
            if kind == "mission_started":
                if not identity:
                    return False
                existing = self._missions.get(identity)
                if existing is not None and existing is not self._current:
                    return False
                if self._current is not None and self._current["id"] != identity:
                    self._finish("interrupted", at, "replaced_by_new_mission")
                    self._db.commit()
                if self._current is None:
                    self._new_mission(event, at)
                self._current["result"] = "active"
            elif kind == "return_requested" and not identity:
                # A user-approved standalone Return may continue a paused trip.
                if self._current is None:
                    self._new_mission({**event, "intent": "return"}, at)
                identity = self._current["id"]
            elif self._current is None or identity != self._current["id"]:
                if self._current is None and identity not in self._missions:
                    self.mark_incomplete(f"orphan_lifecycle_event:{kind}:{identity or 'blank'}")
                return False
            mission = self._current
            signature = {key: event.get(key) for key in (
                "event", "mission_id", "attempt_id", "phase", "state", "state_name",
                "reason", "final_return", "global_stop", "leg_kind",
            )}
            if signature == mission.get("_last_lifecycle_signature"):
                # Same producer heartbeat cannot turn into a second timeline event.
                self._db.execute("INSERT INTO producers VALUES (?,?) ON CONFLICT(session) DO UPDATE SET seq=excluded.seq", (producer, seq))
                self._db.commit()
                return False
            mission["_last_lifecycle_signature"] = signature
            attempt = _text(event.get("attempt_id"), 256)
            if attempt and attempt not in mission["_attempt_ids"]:
                mission["_attempt_ids"].append(attempt)
                mission["attempt_count"] = len(mission["_attempt_ids"])
                self._record_event("attempt_started", at, source=producer, attempt_id=attempt)
            if not mission["attempt_count"]:
                mission["attempt_count"] = 1
            if kind == "return_requested":
                # Recall's first loading/turnaround confirmation is NOT the
                # final road return; the backend supplies this explicit boundary.
                if event.get("final_return") is not False:
                    mission["_return_armed"] = True
                    mission["phase"] = "return"
                    if not event.get("mission_id"):
                        self._put_meta(f"return_binding:{producer}", mission["id"])
                mission["result"] = "active"
            elif kind == "stop_requested":
                mission["result"] = "paused"
            else:
                mission["phase"] = self._phase(event, mission["phase"])
            state = event.get("state")
            handoff = str(event.get("state_name", "")).upper() == "ROAD_HANDOFF_READY"
            if not handoff and state in {3, 10}:
                mission["_return_armed"] = True
            self._record_event(kind, at, reason=event.get("reason", ""), source=producer,
                               received_unix=received, state=state,
                               state_name=event.get("state_name", ""), attempt_id=attempt,
                               producer_seq=seq, final_return=event.get("final_return"))
            if kind == "mission_cancelled":
                self._finish("cancelled", at, event.get("reason", ""))
            elif (kind == "phase" and not handoff and state in {0, 12, 13}
                  and mission["_return_armed"] and mission["result"] != "paused"):
                self._finish("completed", at)
            self._last_event_at = at
            self._db.execute("INSERT INTO producers VALUES (?,?) ON CONFLICT(session) DO UPDATE SET seq=excluded.seq", (producer, seq))
            self._put_meta("last_event_at", at)
            self._save()
            self._db.commit()
            return True

    @staticmethod
    def _gate_signature(gate):
        # HH_260915 - The real safety gate includes age/progress/battery samples
        # in its 2 Hz message. Their numeric churn is not a new stop reason.
        # Normalize ONLY these known numeric fields; keep unknown text, faults,
        # authority, recovery decisions and missing/invalid values observable.
        signature = dict(gate)
        message = signature.get("message")
        if isinstance(message, str):
            number = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)"
            message = re.sub(
                rf"(?<!\S)(drop_zone_auth_age_s|route_clear_s|route_episode_progress_m)={number}(?=\s|$)",
                r"\1=<sample>", message,
            )
            signature["message"] = re.sub(
                rf"(?<!\S)battery={number}%(?=\s|$)", "battery=<sample>%", message,
            )
        return signature

    def observe_gate(self, gate, received_unix=None):
        at = self._now() if received_unix is None else received_unix
        if not _finite(at):
            return False
        with self._mutex:
            new = {key: gate.get(key) for key in ("level", "operating_state", "message", "source", "reason_codes")}
            if self._closed:
                return False
            unchanged = self._gate_signature(new) == self._gate_signature(self._gate)
            # Retain the latest unmodified message for the next stopped event,
            # even when a repeated semantic state needs no timeline entry.
            self._gate = new
            if unchanged:
                return False
            self._record_event("gate_changed", at, reason=new.get("message"),
                               source=new.get("source"), evidence=new)
            if self._stop_active:
                self._record_event("stop_reason_changed", at, reason=new.get("message"),
                                   source=new.get("source"), evidence=new)
            self._save()
            self._db.commit()
            return True

    @staticmethod
    def _mode_for_sample(sample):
        value = sample.get("control_mode")
        if isinstance(value, bool):
            return "unknown"
        return "auto" if value == 1 else "manual" if value == 0 else "unknown"

    def observe_sample(self, sample, received_unix=None):
        at = self._now() if received_unix is None else received_unix
        if not _finite(at):
            return 0.0
        with self._mutex:
            if self._closed:
                return 0.0
            self._last_sample_at = _iso(at)
            self._append("telemetry", {"received_unix": at, "sample": sample}, at)
            stamp, vx, vy = sample.get("sample_time_s"), sample.get("vx"), sample.get("vy")
            quality = sample.get("source_quality") or {}
            valid = all(_finite(v) for v in (stamp, vx, vy))
            valid = valid and not (isinstance(quality, dict) and (quality.get("fresh") is False or quality.get("valid") is False))
            speed = math.hypot(vx, vy) if valid else float("nan")
            if not valid or speed > self.maximum_speed_mps:
                self.mark_incomplete("invalid_or_stale_platform_sample")
                return 0.0
            speed = speed if speed >= self.minimum_speed_mps else 0.0
            previous = self._previous
            if previous is not None:
                delta = stamp - previous[0]
                if delta == 0:
                    return 0.0
                if delta < 0 or delta > self.maximum_sample_gap_s:
                    self.mark_incomplete("platform_sample_time_discontinuity")
                    previous = None
            mode = self._mode_for_sample(sample)
            if mode != self._mode:
                previous_mode = self._mode
                if self._current is not None:
                    self._current["current_mode"] = mode
                    if previous_mode == "auto" and mode == "manual":
                        self._current["manual_interventions"] += 1
                self._mode = mode
                self._record_event("mode_changed", at, previous_mode=previous_mode,
                                   current_mode=mode, source="platform.control_mode",
                                   reason="CAN(1)/RC(0) observed mode; CAN command ownership not independently certified")
            target_id = self._current["id"] if self._current else None
            self._previous = (float(stamp), speed, mode, target_id)
            if previous is None:
                return 0.0
            old_stamp, old_speed, old_mode, old_target = previous
            dt = stamp - old_stamp
            if dt == 0:
                self._previous = previous
                return 0.0
            if dt < 0 or dt > self.maximum_sample_gap_s or old_target != target_id:
                self._zero_elapsed = 0.0
                self._error("platform_sample_time_discontinuity")
                return 0.0
            distance = 0.5 * (old_speed + speed) * dt
            bucket = mode if old_mode == mode else "unknown"
            target = self._current if self._current else self._outside
            target[_distance_key(bucket)] += distance
            target["total_m"] += distance
            if self._current is None:
                if distance > 0 and not self._outside_moving:
                    self._outside["session_count"] += 1
                    self._outside_moving = True
                if speed == 0 and old_speed == 0:
                    self._outside_moving = False
            self._observe_stop(speed, old_speed, dt, at, sample)
            return distance

    def _observe_stop(self, speed, old_speed, dt, at, sample):
        if speed == 0 and old_speed == 0:
            self._zero_elapsed += dt
            if not self._stop_active and self._zero_elapsed >= self.stop_dwell_s:
                self._stop_active = True
                self._record_event("stopped", at, reason=self._gate.get("message", ""),
                                   source=self._gate.get("source", "platform.velocity"),
                                   evidence={"gate": self._gate, "estop": sample.get("estop"),
                                             "error_code": sample.get("error_code"),
                                             "vehicle_state": sample.get("vehicle_state")})
                if self._current is not None:
                    self._current["stop_count"] += 1
                    self._current["stop_duration_s"] += self._zero_elapsed
            elif self._stop_active and self._current is not None:
                self._current["stop_duration_s"] += dt
        elif speed > 0:
            if self._stop_active:
                self._record_event("resumed", at, source="platform.velocity",
                                   stop_duration_s=self._zero_elapsed)
            self._zero_elapsed, self._stop_active = 0.0, False

    def observe_raw_frame(self, frame, received_unix=None):
        at = self._now() if received_unix is None else received_unix
        if not _finite(at):
            return False
        with self._mutex:
            return self._append("raw_can", {"received_unix": at, "frame": frame}, at)

    def _public_mission(self, mission, now):
        result = {key: value for key, value in mission.items() if not key.startswith("_")}
        result["duration_s"] = max(0.0, (mission["_ended_unix"] or now) - mission["_started_unix"])
        # HH_260915 - The snapshot is a bounded UI index, not the event archive.
        # Full event evidence stays in the mission JSONL; expose an explicit tail.
        result["events"] = [{key: item[key] for key in (
            "at", "at_unix", "event", "mode", "phase", "reason", "source",
            "previous_mode", "current_mode", "stop_duration_s",
        ) if key in item} for item in mission["events"][-50:]]
        result["events_exported_count"] = len(result["events"])
        result["events_truncated"] = mission["event_count"] > len(result["events"])
        return json.loads(_json(result))

    def snapshot(self):
        with self._mutex:
            if self._closed_snapshot is not None:
                return json.loads(_json(self._closed_snapshot))
            now = float(self._now())
            missions = list(self._missions.values())
            lifetime = {**_totals(), "mission_count": len(missions), "completed_count": 0, "manual_interventions": 0}
            sites = {}
            for mission in missions:
                key = (mission["site"], mission["intent"])
                group = sites.setdefault(key, {"site": key[0], "intent": key[1], **_totals(), "mission_count": 0, "completed_count": 0, "manual_interventions": 0})
                group["mission_count"] += 1
                for aggregate in (lifetime, group):
                    for field in _totals():
                        aggregate[field] += mission[field]
                    aggregate["completed_count"] += int(mission["result"] == "completed")
                    aggregate["manual_interventions"] += mission["manual_interventions"]
            for field in _totals():
                lifetime[field] += self._outside[field]
            recent = sorted(missions, key=lambda m: (m["_started_unix"], m["sequence"]), reverse=True)[:100]
            return {
                "schema_version": 1, "generated_at": _iso(now),
                "recorder": {**self._health, "status": "CLOSED" if self._closing else "DEGRADED" if self._errors else "READY",
                             "error": "; ".join(self._errors), "storage_root": str(self.root),
                             "last_sample_at": self._last_sample_at,
                             "raw_can_status": self._raw_can_status, "raw_can_details": dict(self._raw_can_details),
                             "environment": self.environment, "robot_id": self.robot_id,
                             "mode_basis": "observed CAN(1)/RC(0); not CAN command-owner certification",
                             "telemetry_bytes": sum(self._files.values()), "quota_bytes": self.quota_bytes},
                "lifetime": lifetime,
                "current_mission": self._public_mission(self._current, now) if self._current else None,
                "missions": [self._public_mission(m, now) for m in recent],
                "sites": list(sites.values()), "outside_missions": dict(self._outside),
            }

    def flush_snapshot(self):
        with self._mutex:
            if self._closed:
                return False
            temporary = self.root / f".snapshot-{uuid.uuid4().hex}.tmp"
            try:
                self._save()
                self._put_meta("outside", self._outside)
                self._put_meta("errors", self._errors)
                self._db.commit()
                snapshot = self.snapshot()
                encoded = _json(snapshot)
                while len(encoded.encode("utf-8")) > 7 * 1024 * 1024:
                    # Trim only already archived UI event tails, never records,
                    # measured totals, IDs or on-disk event/telemetry history.
                    candidates = snapshot["missions"] + ([snapshot["current_mission"]] if snapshot["current_mission"] else [])
                    longest = max(candidates, key=lambda m: len(m["events"]), default=None)
                    if longest is None or not longest["events"]:
                        raise ValueError("snapshot metadata exceeds bounded export size")
                    longest["events"] = longest["events"][len(longest["events"]) // 2 + 1:]
                    longest["events_exported_count"] = len(longest["events"])
                    longest["events_truncated"] = True
                    encoded = _json(snapshot)
                with temporary.open("x", encoding="utf-8") as stream:
                    stream.write(encoded)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, self.root / "snapshot.json")
                return True
            except (OSError, sqlite3.Error, ValueError, TypeError) as exc:
                self._error(f"snapshot: {exc}")
                return False
            finally:
                if temporary.exists():
                    temporary.unlink()

    def close(self):
        with self._mutex:
            if self._closed:
                return
            self._closing = True
            self.flush_snapshot()
            self._closed_snapshot = self.snapshot()
            self._closed_snapshot["recorder"]["status"] = "CLOSED"
            self._db.close()
            fcntl.flock(self._lock_fd, fcntl.LOCK_UN)
            os.close(self._lock_fd)
            self._closed = True


def read_snapshot(root, robot_id=None, environment=None):
    """Read only the independent export, never open either operational DB."""
    path = Path(root).expanduser() / "snapshot.json"
    with path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    if value.get("schema_version") != 1:
        raise ValueError("unsupported mission journal snapshot")
    recorder = value.get("recorder", {})
    if robot_id is not None and recorder.get("robot_id") != robot_id:
        raise ValueError("snapshot robot_id mismatch")
    if environment is not None and recorder.get("environment") != environment:
        raise ValueError("snapshot environment mismatch")
    return value
