"""Persistent proof-of-operation metrics for CAMROD service runs.

The tracker deliberately has no ROS dependency.  The UI backend feeds it
accepted service requests, service-state transitions, and platform velocity
samples.  This keeps evidence collection testable and prevents a storage
failure from affecting motion control.
"""

from __future__ import annotations

import json
import math
import os
import sqlite3
import threading
import time
import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError


SERVICE_METRICS_SCHEMA_VERSION = 2


def default_service_metrics_path() -> Path:
    """Return a rebuild-safe per-user state path."""
    state_root = os.environ.get("XDG_STATE_HOME", "").strip()
    root = Path(state_root).expanduser() if state_root else Path.home() / ".local" / "state"
    return root / "camrod" / "service_metrics.sqlite3"


class ServiceMetricsTracker:
    """Accumulate service distance and persist completed run evidence.

    Distance is the trapezoidal integral of planar platform speed.  Invalid,
    implausibly fast, duplicated, backwards, or widely separated samples only
    reset the integration anchor; they never add distance.
    """

    ACTIVE_RESULT = "active"
    COMPLETED_RESULT = "completed"
    INTERRUPTED_RESULT = "interrupted"
    SUPERSEDED_RESULT = "superseded"

    # The parking controller reaches WAITING_FOR_CHARGING only after the robot
    # is stationary at the station.  Charging contact is optional, so waiting
    # is a valid completed service boundary too.
    COMPLETION_STATES = frozenset({0, 12, 13})
    INTERRUPT_STATES = frozenset({16})
    CANONICAL_CAMPSITES = tuple(f"B{index}" for index in range(1, 14))
    LEG_KINDS = ("delivery", "recall", "return", "unknown")

    def __init__(
        self,
        database_path: Optional[Path | str],
        *,
        timezone_name: str = "Asia/Seoul",
        minimum_speed_mps: float = 0.03,
        maximum_speed_mps: float = 3.0,
        maximum_sample_gap_s: float = 2.0,
        checkpoint_interval_s: float = 5.0,
        now_fn: Callable[[], float] = time.time,
        monotonic_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        self._lock = threading.RLock()
        self._now_fn = now_fn
        self._monotonic_fn = monotonic_fn
        self.timezone_name = str(timezone_name).strip() or "Asia/Seoul"
        try:
            self._timezone = ZoneInfo(self.timezone_name)
        except ZoneInfoNotFoundError:
            self.timezone_name = "UTC"
            self._timezone = timezone.utc

        self.minimum_speed_mps = max(0.0, float(minimum_speed_mps))
        self.maximum_speed_mps = max(
            self.minimum_speed_mps,
            float(maximum_speed_mps),
        )
        self.maximum_sample_gap_s = max(0.05, float(maximum_sample_gap_s))
        self.checkpoint_interval_s = max(0.1, float(checkpoint_interval_s))

        self.database_path = (
            Path(database_path).expanduser() if database_path else None
        )
        self._connection: Optional[sqlite3.Connection] = None
        self._persistence_error = ""
        self._last_saved_at: Optional[float] = None
        self._last_checkpoint_monotonic = self._monotonic_fn()
        self._records: List[Dict[str, Any]] = []
        self._active: Optional[Dict[str, Any]] = None
        self._previous_velocity_sample: Optional[tuple[float, float]] = None
        self._previous_velocity_kind: Optional[str] = None

        self._open_store()

    # ------------------------------------------------------------------ store

    @property
    def persistence_enabled(self) -> bool:
        return self._connection is not None

    @property
    def persistence_error(self) -> str:
        return self._persistence_error

    @property
    def has_active_service(self) -> bool:
        """Cheap observational query; never adopt a mission from a heartbeat."""
        with self._lock:
            return self._active is not None

    def _open_store(self) -> None:
        if self.database_path is None:
            return
        try:
            self.database_path.parent.mkdir(parents=True, exist_ok=True)
            connection = sqlite3.connect(
                str(self.database_path),
                timeout=5.0,
                check_same_thread=False,
            )
            connection.row_factory = sqlite3.Row
            if connection.execute("PRAGMA user_version").fetchone()[0] > SERVICE_METRICS_SCHEMA_VERSION:
                raise sqlite3.DatabaseError("service metrics database schema is newer than this tracker")
            connection.execute("PRAGMA journal_mode=WAL")
            connection.execute("PRAGMA synchronous=NORMAL")
            connection.execute("BEGIN")
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS service_runs (
                    id TEXT PRIMARY KEY,
                    service_date TEXT NOT NULL,
                    site TEXT NOT NULL,
                    mission_key TEXT NOT NULL,
                    source TEXT NOT NULL,
                    started_at REAL NOT NULL,
                    ended_at REAL,
                    result TEXT NOT NULL,
                    distance_m REAL NOT NULL,
                    last_state INTEGER,
                    last_state_name TEXT NOT NULL,
                    updated_at REAL NOT NULL
                )
                """
            )
            # Additive migration only: old rows, dates, results and their raw
            # total distance remain untouched. Missing historical leg evidence
            # is classified as unknown at read time, never guessed from source.
            columns = {row[1] for row in connection.execute("PRAGMA table_info(service_runs)")}
            for name, declaration in (
                ("intent", "TEXT NOT NULL DEFAULT ''"),
                ("request_id", "TEXT NOT NULL DEFAULT ''"),
                ("phase", "TEXT NOT NULL DEFAULT ''"),
                ("segments_json", "TEXT NOT NULL DEFAULT '[]'"),
                ("interruption_reason", "TEXT NOT NULL DEFAULT ''"),
            ):
                if name not in columns:
                    connection.execute(f"ALTER TABLE service_runs ADD COLUMN {name} {declaration}")
            connection.execute("PRAGMA user_version=2")
            connection.execute(
                "CREATE INDEX IF NOT EXISTS service_runs_date_idx "
                "ON service_runs(service_date DESC)"
            )
            connection.execute(
                "CREATE INDEX IF NOT EXISTS service_runs_result_idx "
                "ON service_runs(result)"
            )
            connection.commit()
            rows = connection.execute(
                "SELECT * FROM service_runs ORDER BY started_at ASC"
            ).fetchall()
            self._connection = connection
            self._records = [self._row_to_record(row) for row in rows]
            active_records = [
                record
                for record in self._records
                if record["result"] == self.ACTIVE_RESULT
            ]
            if active_records:
                self._active = active_records[-1]
                # Recover deterministically if an older release left more than
                # one active row behind.
                for record in active_records[:-1]:
                    record["result"] = self.SUPERSEDED_RESULT
                    record["ended_at"] = self._active["started_at"]
                    record["updated_at"] = self._active["started_at"]
                    self._persist_record(record, commit=False)
                connection.commit()
        except (OSError, sqlite3.Error, ValueError) as exc:
            self._persistence_error = str(exc)
            connection = locals().get("connection")
            if connection is not None:
                try:
                    connection.close()
                except sqlite3.Error:
                    pass
            self._connection = None

    @classmethod
    def _row_to_record(cls, row: sqlite3.Row) -> Dict[str, Any]:
        record = {
            "id": str(row["id"]),
            "service_date": str(row["service_date"]),
            "site": str(row["site"]),
            "mission_key": str(row["mission_key"]),
            "source": str(row["source"]),
            "started_at": float(row["started_at"]),
            "ended_at": (
                float(row["ended_at"]) if row["ended_at"] is not None else None
            ),
            "result": str(row["result"]),
            "distance_m": max(0.0, float(row["distance_m"])),
            "last_state": (
                int(row["last_state"]) if row["last_state"] is not None else None
            ),
            "last_state_name": str(row["last_state_name"]),
            "updated_at": float(row["updated_at"]),
            "intent": cls._normalize_kind(row["intent"]),
            "request_id": str(row["request_id"]),
            "phase": str(row["phase"]) or "LEGACY_UNCLASSIFIED",
            "interruption_reason": str(row["interruption_reason"]),
        }
        try:
            segments = json.loads(row["segments_json"])
            valid = isinstance(segments, list) and bool(segments)
            valid = valid and all(
                isinstance(segment, dict)
                and {"kind", "phase", "distance_m", "moving_s", "waiting_s",
                     "started_at", "ended_at", "timing_complete"}.issubset(segment)
                and segment.get("kind") in cls.LEG_KINDS
                and isinstance(segment.get("phase"), str)
                and cls._nonnegative_finite(segment.get("distance_m"))
                and cls._nonnegative_finite(segment.get("started_at"))
                and (segment.get("ended_at") is None or cls._nonnegative_finite(segment["ended_at"]))
                and isinstance(segment.get("timing_complete"), bool)
                and all(value is None or cls._nonnegative_finite(value)
                        for value in (segment.get("moving_s"), segment.get("waiting_s")))
                for segment in segments
            )
            valid = valid and math.isclose(
                sum(segment["distance_m"] for segment in segments), record["distance_m"],
                rel_tol=1e-10, abs_tol=1e-8,
            )
        except (TypeError, ValueError, KeyError):
            valid = False
        record["segments"] = segments if valid else [{
            "kind": "unknown", "phase": "LEGACY_UNCLASSIFIED",
            "distance_m": record["distance_m"], "moving_s": None, "waiting_s": None,
            "started_at": record["started_at"],
            "ended_at": record["ended_at"] or record["updated_at"],
            "timing_complete": False,
        }]
        return record

    def _persist_record(self, record: Dict[str, Any], *, commit: bool = True) -> None:
        connection = self._connection
        if connection is None:
            return
        try:
            connection.execute(
                """
                INSERT INTO service_runs (
                    id, service_date, site, mission_key, source, started_at,
                    ended_at, result, distance_m, last_state, last_state_name,
                    updated_at, intent, request_id, phase, segments_json,
                    interruption_reason
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(id) DO UPDATE SET
                    service_date=excluded.service_date,
                    site=excluded.site,
                    mission_key=excluded.mission_key,
                    source=excluded.source,
                    started_at=excluded.started_at,
                    ended_at=excluded.ended_at,
                    result=excluded.result,
                    distance_m=excluded.distance_m,
                    last_state=excluded.last_state,
                    last_state_name=excluded.last_state_name,
                    updated_at=excluded.updated_at,
                    intent=excluded.intent,
                    request_id=excluded.request_id,
                    phase=excluded.phase,
                    segments_json=excluded.segments_json,
                    interruption_reason=excluded.interruption_reason
                """,
                (
                    record["id"],
                    record["service_date"],
                    record["site"],
                    record["mission_key"],
                    record["source"],
                    record["started_at"],
                    record["ended_at"],
                    record["result"],
                    record["distance_m"],
                    record["last_state"],
                    record["last_state_name"],
                    record["updated_at"],
                    record["intent"],
                    record["request_id"],
                    record["phase"],
                    json.dumps(record["segments"], separators=(",", ":"), allow_nan=False),
                    record["interruption_reason"],
                ),
            )
            if commit:
                connection.commit()
                self._last_saved_at = self._now_fn()
                self._persistence_error = ""
        except sqlite3.Error as exc:
            # Metrics are observational.  Never allow a disk/DB fault to escape
            # into the ROS control callback.
            self._persistence_error = str(exc)

    # -------------------------------------------------------------- lifecycle

    def start_service(
        self,
        site: str,
        *,
        mission_key: str = "",
        source: str = "",
        intent: str = "",
        request_id: str = "",
        now_s: Optional[float] = None,
    ) -> bool:
        """Start one accepted request; legacy callers retain same-site coalescing.

        A supplied request_id is the idempotency key, including after restart
        or completion. A new ID at the same site starts a distinct attempt.
        """
        now = self._valid_now(now_s)
        normalized_site = str(site).strip() or "미지정"
        normalized_id = str(request_id).strip()
        with self._lock:
            if normalized_id and any(
                record["request_id"] == normalized_id for record in self._records
            ):
                return False
            if self._active is not None:
                if not normalized_id and self._active["site"] == normalized_site:
                    if mission_key and not self._active["mission_key"]:
                        self._active["mission_key"] = str(mission_key)
                    if source and not self._active["source"]:
                        self._active["source"] = str(source)
                    self._active["updated_at"] = now
                    self._persist_record(self._active)
                    return False
                self._finish_active(self.SUPERSEDED_RESULT, now)

            service_date = self._date_for_timestamp(now)
            record = {
                "id": self._new_id(now),
                "service_date": service_date,
                "site": normalized_site,
                "mission_key": str(mission_key).strip(),
                "source": str(source).strip(),
                "intent": self._normalize_kind(intent),
                "request_id": normalized_id,
                "phase": "ACCEPTED",
                "segments": [],
                "interruption_reason": "",
                "started_at": now,
                "ended_at": None,
                "result": self.ACTIVE_RESULT,
                "distance_m": 0.0,
                "last_state": None,
                "last_state_name": "ACCEPTED",
                "updated_at": now,
            }
            self._records.append(record)
            self._active = record
            self._previous_velocity_sample = None
            self._previous_velocity_kind = None
            self._set_segment(record["intent"], "ACCEPTED", now)
            self._last_checkpoint_monotonic = self._monotonic_fn()
            self._persist_record(record)
            return True

    def observe_service_state(
        self,
        state: int,
        state_name: str = "",
        *,
        now_s: Optional[float] = None,
        phase: str = "",
        leg_kind: str = "",
    ) -> bool:
        """Apply an operational state transition; return True when a run ends."""
        now = self._valid_now(now_s)
        state_value = int(state)
        with self._lock:
            if self._active is None:
                return False
            self._active["last_state"] = state_value
            self._active["last_state_name"] = (
                str(state_name).strip() or f"STATE_{state_value}"
            )
            self._active["updated_at"] = now
            self._active["phase"] = str(phase).strip() or self._active["last_state_name"]
            handoff = state_value == 0 and str(state_name).strip().upper() == "ROAD_HANDOFF_READY"
            if handoff:
                # Same numeric state as parked, but this is an outbound road
                # handoff, not completed service. Preserve the integration
                # anchor and leg even when status precedes exit_complete Bool.
                self._persist_record(self._active)
                return False
            if state_value in self.COMPLETION_STATES:
                self._finish_active(self.COMPLETED_RESULT, now)
                return True
            if state_value in self.INTERRUPT_STATES:
                self._finish_active(self.INTERRUPTED_RESULT, now)
                return True
            operational_phase = str(phase).strip() or self._active["last_state_name"]
            kind = self._kind_for_state(state_value, leg_kind)
            self._set_segment(kind, operational_phase, now)
            # Service-state transitions are low frequency and valuable recovery
            # evidence even when the robot is stationary at a site.
            self._persist_record(self._active)
            return False

    def interrupt_service(
        self, reason: str, *, now_s: Optional[float] = None,
        request_id: Optional[str] = None,
    ) -> bool:
        """Close a failed/cancelled attempt without inventing a robot state.

        When an identity is supplied, an old failure cannot interrupt a newer
        request. Repeated interruption calls are harmless and return False.
        """
        now = self._valid_now(now_s)
        with self._lock:
            if self._active is None:
                return False
            if request_id is not None and str(request_id).strip() != self._active["request_id"]:
                return False
            self._active["interruption_reason"] = str(reason).strip()
            self._finish_active(self.INTERRUPTED_RESULT, now)
            return True

    def _finish_active(self, result: str, now: float) -> None:
        record = self._active
        if record is None:
            return
        record["result"] = result
        record["ended_at"] = max(float(record["started_at"]), now)
        record["updated_at"] = record["ended_at"]
        for segment in record["segments"]:
            if segment.get("ended_at") is None:
                segment["ended_at"] = max(segment["started_at"], record["ended_at"])
        self._persist_record(record)
        self._active = None
        self._previous_velocity_sample = None
        self._previous_velocity_kind = None

    # --------------------------------------------------------------- distance

    def observe_velocity(
        self,
        vx_mps: Any,
        vy_mps: Any,
        sample_time_s: Any,
    ) -> float:
        """Integrate one planar velocity sample and return added metres."""
        try:
            vx = float(vx_mps)
            vy = float(vy_mps)
            sample_time = float(sample_time_s)
        except (TypeError, ValueError):
            with self._lock:
                self._previous_velocity_sample = None
                self._previous_velocity_kind = None
                if self._active is not None:
                    self._current_segment()["timing_complete"] = False
            return 0.0
        if not all(math.isfinite(value) for value in (vx, vy, sample_time)):
            with self._lock:
                self._previous_velocity_sample = None
                self._previous_velocity_kind = None
                if self._active is not None:
                    self._current_segment()["timing_complete"] = False
            return 0.0

        speed = math.hypot(vx, vy)
        with self._lock:
            if self._active is None:
                self._previous_velocity_sample = None
                self._previous_velocity_kind = None
                return 0.0
            segment = self._current_segment()
            kind = segment["kind"]
            if speed > self.maximum_speed_mps:
                self._previous_velocity_sample = None
                self._previous_velocity_kind = None
                segment["timing_complete"] = False
                return 0.0
            if speed < self.minimum_speed_mps:
                speed = 0.0

            previous = self._previous_velocity_sample
            if previous is None:
                self._previous_velocity_sample = (sample_time, speed)
                self._previous_velocity_kind = kind
                return 0.0
            previous_time, previous_speed = previous
            delta_s = sample_time - previous_time
            if delta_s == 0.0:
                return 0.0
            if delta_s < 0.0 or delta_s > self.maximum_sample_gap_s:
                self._previous_velocity_sample = (sample_time, speed)
                self._previous_velocity_kind = kind
                segment["timing_complete"] = False
                return 0.0
            self._previous_velocity_sample = (sample_time, speed)

            added_m = 0.5 * (previous_speed + speed) * delta_s
            if not math.isfinite(added_m) or added_m < 0.0:
                return 0.0
            if self._previous_velocity_kind != kind:
                # State callbacks use wall time; velocities can use ROS/sim
                # time. Do not pretend those clocks define an exact split of
                # this crossing interval. Preserve its entire distance/time
                # once as unknown. Same-kind phase changes need no such split.
                now = self._valid_now(None)
                segment = self._new_segment("unknown", "LEG_TRANSITION_UNRESOLVED", now)
                segment["ended_at"] = now
                self._active["segments"].append(segment)
            self._previous_velocity_kind = kind
            segment["distance_m"] += added_m
            timing_key = "moving_s" if previous_speed > 0.0 or speed > 0.0 else "waiting_s"
            segment[timing_key] += delta_s
            self._active["distance_m"] += added_m
            self._active["updated_at"] = self._valid_now(None)

            checkpoint_now = self._monotonic_fn()
            if (
                checkpoint_now - self._last_checkpoint_monotonic
                >= self.checkpoint_interval_s
            ):
                self._persist_record(self._active)
                self._last_checkpoint_monotonic = checkpoint_now
            return added_m

    # --------------------------------------------------------------- snapshots

    def snapshot(self, *, days: int = 30, recent_limit: int = 50) -> Dict[str, Any]:
        now = self._valid_now(None)
        days = min(3660, max(1, int(days)))
        recent_limit = min(500, max(1, int(recent_limit)))
        with self._lock:
            today_date = self._date_for_timestamp(now)
            records = list(self._records)
            active = self._active

            groups: Dict[str, Dict[str, Any]] = {}
            for record in records:
                date = record["service_date"]
                group = groups.setdefault(
                    date,
                    {
                        "date": date,
                        "distance_m": 0.0,
                        "completed_service_count": 0,
                        "interrupted_service_count": 0,
                        "service_attempt_count": 0,
                    },
                )
                group["distance_m"] += max(0.0, float(record["distance_m"]))
                group.setdefault("records", []).append(record)
                group["service_attempt_count"] += 1
                if record["result"] == self.COMPLETED_RESULT:
                    group["completed_service_count"] += 1
                elif record["result"] in {
                    self.INTERRUPTED_RESULT,
                    self.SUPERSEDED_RESULT,
                }:
                    group["interrupted_service_count"] += 1

            today = groups.get(
                today_date,
                {
                    "date": today_date,
                    "distance_m": 0.0,
                    "completed_service_count": 0,
                    "interrupted_service_count": 0,
                    "service_attempt_count": 0,
                },
            )
            lifetime = {
                "distance_m": sum(
                    max(0.0, float(record["distance_m"])) for record in records
                ),
                "completed_service_count": sum(
                    record["result"] == self.COMPLETED_RESULT for record in records
                ),
                "interrupted_service_count": sum(
                    record["result"]
                    in {self.INTERRUPTED_RESULT, self.SUPERSEDED_RESULT}
                    for record in records
                ),
                "service_attempt_count": len(records),
                "operating_day_count": len(groups),
            }
            for group in groups.values():
                group.update(self._aggregate_leg_metrics(group.pop("records")))
            lifetime.update(self._aggregate_leg_metrics(records))

            today_local = datetime.fromtimestamp(now, timezone.utc).astimezone(
                self._timezone
            ).date()
            first_history_date = (today_local - timedelta(days=days - 1)).isoformat()
            history_dates = sorted(
                (
                    date
                    for date in groups
                    if first_history_date <= date <= today_date
                ),
                reverse=True,
            )
            daily_history = [self._format_totals(groups[date]) for date in history_dates]
            completed = [
                record
                for record in records
                if record["result"] == self.COMPLETED_RESULT
            ]
            last_completed = completed[-1] if completed else None
            recent = sorted(
                (record for record in records if record["result"] != self.ACTIVE_RESULT),
                key=lambda record: record["ended_at"] or record["started_at"],
                reverse=True,
            )[:recent_limit]
            # This is a subset already included in lifetime distance, not an
            # extra total. Count a legacy-bearing record once even when it has
            # several historical segments or has resumed with new observed legs.
            # New LEG_TRANSITION_UNRESOLVED intervals are deliberately excluded.
            historical_segments = [
                [segment for segment in record["segments"]
                 if segment["phase"] == "LEGACY_UNCLASSIFIED"]
                for record in records
            ]
            historical_distance_m = sum(
                segment["distance_m"]
                for segments in historical_segments for segment in segments
            )

            return {
                "schema_version": SERVICE_METRICS_SCHEMA_VERSION,
                "date_basis": "service_start_date",
                "timing_basis": "valid_velocity_intervals",
                "timezone": self.timezone_name,
                "generated_at": self._iso_timestamp(now),
                "historical_unclassified": {
                    "record_count": sum(bool(segments) for segments in historical_segments),
                    "distance_m": round(historical_distance_m, 2),
                    "distance_km": round(historical_distance_m / 1000.0, 3),
                    "included_in_lifetime_total": True,
                },
                "current_service": (
                    self._format_record(active, now=now) if active else None
                ),
                "last_completed_service": (
                    self._format_record(last_completed, now=now)
                    if last_completed else None
                ),
                "today": self._format_totals(today),
                "lifetime": self._format_totals(lifetime),
                "daily_history": daily_history,
                "recent_services": [
                    self._format_record(record, now=now) for record in recent
                ],
                # HH_260904 - Keep B1-B13 comparison server-side so every UI
                # client receives identical averages and active-run progress.
                # Progress is explicitly relative to completed historical
                # averages; it is not an estimate of route completion.
                "site_summaries": self._build_site_summaries(
                    records, active=active, now=now
                ),
                "persistence": {
                    "enabled": self.persistence_enabled,
                    "path": str(self.database_path) if self.database_path else "",
                    "error": self._persistence_error or None,
                    "last_saved_at": (
                        self._iso_timestamp(self._last_saved_at)
                        if self._last_saved_at is not None else None
                    ),
                },
            }

    def summary(self) -> Dict[str, Any]:
        snapshot = self.snapshot(days=1, recent_limit=1)
        snapshot.pop("daily_history", None)
        snapshot.pop("recent_services", None)
        return snapshot

    def flush(self) -> None:
        with self._lock:
            if self._active is not None:
                self._persist_record(self._active)

    def close(self) -> None:
        with self._lock:
            self.flush()
            connection = self._connection
            self._connection = None
            if connection is not None:
                try:
                    connection.close()
                except sqlite3.Error as exc:
                    self._persistence_error = str(exc)

    # ---------------------------------------------------------------- helpers

    @classmethod
    def _normalize_kind(cls, value: Any) -> str:
        normalized = str(value).strip().lower()
        return normalized if normalized in cls.LEG_KINDS else "unknown"

    @staticmethod
    def _nonnegative_finite(value: Any) -> bool:
        return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value) and value >= 0.0

    def _kind_for_state(self, state: int, override: str) -> str:
        if str(override).strip():
            return self._normalize_kind(override)
        if state == 5 and self._active["intent"] == "recall":
            return "recall"
        if state in {1, 5}:
            return "delivery"
        if state == 7:
            return "recall"
        if state in {3, 9, 10}:
            return "return"
        if state in {14, 15}:
            return self._active["intent"]
        # Waiting does not start a different distance bucket. In particular,
        # recall loading/rotation phases may be supplied explicitly by backend.
        return self._current_segment()["kind"]

    @staticmethod
    def _new_segment(kind: str, phase: str, now: float) -> Dict[str, Any]:
        return {
            "kind": kind, "phase": phase, "distance_m": 0.0,
            "moving_s": 0.0, "waiting_s": 0.0,
            "started_at": now, "ended_at": None, "timing_complete": True,
        }

    def _set_segment(self, kind: str, phase: str, now: float) -> None:
        record = self._active
        for segment in reversed(record["segments"]):
            if segment.get("ended_at") is None:
                if segment["kind"] == kind and segment["phase"] == phase:
                    record["phase"] = phase
                    return
                segment["ended_at"] = max(segment["started_at"], now)
                break
        record["segments"].append(self._new_segment(kind, phase, now))
        record["phase"] = phase

    def _current_segment(self) -> Dict[str, Any]:
        record = self._active
        for segment in reversed(record["segments"]):
            if segment.get("ended_at") is None:
                return segment
        # A recovered legacy active row has only historical unknown evidence;
        # start a fresh observed interval rather than deriving old leg/time data.
        self._set_segment(record["intent"], record["phase"], self._valid_now(None))
        return record["segments"][-1]

    @classmethod
    def _aggregate_leg_metrics(cls, records: List[Dict[str, Any]]) -> Dict[str, Any]:
        breakdown = {kind: 0.0 for kind in cls.LEG_KINDS}
        moving, waiting = [], []
        complete = True
        for record in records:
            for segment in record["segments"]:
                breakdown[segment["kind"]] += segment["distance_m"]
                if segment.get("moving_s") is not None:
                    moving.append(segment["moving_s"])
                if segment.get("waiting_s") is not None:
                    waiting.append(segment["waiting_s"])
                complete = complete and bool(segment.get("timing_complete", False))
        return {
            "distance_breakdown_m": breakdown,
            "moving_s": sum(moving) if moving else None,
            "waiting_s": sum(waiting) if waiting else None,
            "timing_complete": complete and bool(moving or waiting),
            "timing_basis": "valid_velocity_intervals",
        }

    @classmethod
    def _rounded_breakdown(cls, raw: Dict[str, float], total: float) -> Dict[str, float]:
        # Largest-remainder rounding keeps the displayed 0.01m buckets equal
        # to displayed total without changing any stored full-precision value.
        scaled = {kind: max(0.0, float(raw.get(kind, 0.0))) * 100.0 for kind in cls.LEG_KINDS}
        cents = {kind: math.floor(scaled[kind]) for kind in cls.LEG_KINDS}
        remainder = int(round(round(total, 2) * 100.0)) - sum(cents.values())
        order = sorted(cls.LEG_KINDS, key=lambda kind: scaled[kind] - cents[kind], reverse=True)
        for index in range(max(0, remainder)):
            cents[order[index % len(order)]] += 1
        return {kind: cents[kind] / 100.0 for kind in cls.LEG_KINDS}

    @classmethod
    def _formatted_leg_metrics(cls, records: List[Dict[str, Any]], total: float) -> Dict[str, Any]:
        metrics = cls._aggregate_leg_metrics(records)
        metrics["distance_breakdown_m"] = cls._rounded_breakdown(metrics["distance_breakdown_m"], total)
        for key in ("moving_s", "waiting_s"):
            if metrics[key] is not None:
                metrics[key] = round(metrics[key], 3)
        return metrics

    def _format_record(
        self, record: Dict[str, Any], *, now: float
    ) -> Dict[str, Any]:
        ended_at = record["ended_at"]
        duration_end = float(ended_at) if ended_at is not None else now
        distance_m = max(0.0, float(record["distance_m"]))
        return {
            "id": record["id"],
            "date": record["service_date"],
            "site": record["site"],
            "mission_key": record["mission_key"],
            "source": record["source"],
            "intent": record["intent"],
            "request_id": record["request_id"],
            "phase": record["phase"],
            "interruption_reason": record["interruption_reason"] or None,
            "status": record["result"],
            "result": record["result"],
            "started_at": self._iso_timestamp(record["started_at"]),
            "completed_at": (
                self._iso_timestamp(ended_at) if ended_at is not None else None
            ),
            "distance_m": round(distance_m, 2),
            "distance_km": round(distance_m / 1000.0, 3),
            "duration_s": max(0, round(duration_end - record["started_at"])),
            "state": record["last_state"],
            "state_name": record["last_state_name"],
            **self._formatted_leg_metrics([record], distance_m),
            "segments": [{
                **segment,
                "distance_m": round(segment["distance_m"], 6),
                "moving_s": None if segment["moving_s"] is None else round(segment["moving_s"], 3),
                "waiting_s": None if segment["waiting_s"] is None else round(segment["waiting_s"], 3),
                "started_at": self._iso_timestamp(segment["started_at"]),
                "ended_at": None if segment.get("ended_at") is None else self._iso_timestamp(segment["ended_at"]),
            } for segment in record["segments"]],
        }

    @classmethod
    def _canonical_site_name(cls, value: Any) -> str:
        text = str(value).strip()
        upper = text.upper()
        if upper.startswith("B") and upper[1:].isdigit():
            index = int(upper[1:])
            if 1 <= index <= 13:
                return f"B{index}"
        return text or "미지정"

    @classmethod
    def _site_sort_key(cls, value: str) -> tuple[int, int, str]:
        canonical = cls._canonical_site_name(value)
        if canonical.startswith("B") and canonical[1:].isdigit():
            return (0, int(canonical[1:]), canonical)
        return (1, 0, canonical.casefold())

    def _build_site_summaries(
        self,
        records: List[Dict[str, Any]],
        *,
        active: Optional[Dict[str, Any]],
        now: float,
    ) -> List[Dict[str, Any]]:
        grouped: Dict[str, List[Dict[str, Any]]] = {
            site: [] for site in self.CANONICAL_CAMPSITES
        }
        for record in records:
            site = self._canonical_site_name(record.get("site"))
            grouped.setdefault(site, []).append(record)

        summaries: List[Dict[str, Any]] = []
        for site in sorted(grouped, key=self._site_sort_key):
            site_records = grouped[site]
            completed = [
                record
                for record in site_records
                if record["result"] == self.COMPLETED_RESULT
                and record["ended_at"] is not None
            ]
            interrupted = [
                record
                for record in site_records
                if record["result"]
                in {self.INTERRUPTED_RESULT, self.SUPERSEDED_RESULT}
            ]
            terminal_count = len(completed) + len(interrupted)
            completed_distances = [
                max(0.0, float(record["distance_m"])) for record in completed
            ]
            completed_durations = [
                max(0.0, float(record["ended_at"]) - float(record["started_at"]))
                for record in completed
            ]
            average_distance_m = (
                sum(completed_distances) / len(completed_distances)
                if completed_distances else None
            )
            average_duration_s = (
                sum(completed_durations) / len(completed_durations)
                if completed_durations else None
            )
            terminal_records = [
                record
                for record in site_records
                if record["result"] != self.ACTIVE_RESULT
            ]
            latest = max(
                terminal_records,
                key=lambda record: record["ended_at"] or record["started_at"],
                default=None,
            )
            current = (
                active
                if active is not None
                and self._canonical_site_name(active.get("site")) == site
                else None
            )
            current_distance_m = (
                max(0.0, float(current["distance_m"])) if current else None
            )
            current_duration_s = (
                max(0.0, now - float(current["started_at"])) if current else None
            )

            def progress_percentage(
                current_value: Optional[float], average_value: Optional[float]
            ) -> Optional[float]:
                if (
                    current_value is None
                    or average_value is None
                    or average_value <= 0.0
                ):
                    return None
                return round(100.0 * current_value / average_value, 1)

            summaries.append({
                "site": site,
                "distance_m": round(sum(record["distance_m"] for record in site_records), 2),
                **self._formatted_leg_metrics(site_records, sum(record["distance_m"] for record in site_records)),
                "service_attempt_count": len(site_records),
                "completed_service_count": len(completed),
                "interrupted_service_count": len(interrupted),
                "completion_rate_percentage": (
                    round(100.0 * len(completed) / terminal_count, 1)
                    if terminal_count else None
                ),
                "average_distance_m": (
                    round(average_distance_m, 2)
                    if average_distance_m is not None else None
                ),
                "average_duration_s": (
                    round(average_duration_s)
                    if average_duration_s is not None else None
                ),
                "latest_service": (
                    self._format_record(latest, now=now) if latest else None
                ),
                "current_service": (
                    self._format_record(current, now=now) if current else None
                ),
                "current_distance_progress_percentage": progress_percentage(
                    current_distance_m, average_distance_m
                ),
                "current_duration_progress_percentage": progress_percentage(
                    current_duration_s, average_duration_s
                ),
            })
        return summaries

    @classmethod
    def _format_totals(cls, totals: Dict[str, Any]) -> Dict[str, Any]:
        formatted = dict(totals)
        distance_m = max(0.0, float(formatted.get("distance_m", 0.0)))
        formatted["distance_m"] = round(distance_m, 2)
        formatted["distance_km"] = round(distance_m / 1000.0, 3)
        raw = formatted.get("distance_breakdown_m", {"unknown": distance_m})
        formatted["distance_breakdown_m"] = cls._rounded_breakdown(raw, distance_m)
        for key in ("moving_s", "waiting_s"):
            value = formatted.get(key)
            formatted[key] = None if value is None else round(value, 3)
        formatted.setdefault("timing_complete", False)
        formatted["timing_basis"] = "valid_velocity_intervals"
        return formatted

    def _valid_now(self, candidate: Optional[float]) -> float:
        try:
            value = self._now_fn() if candidate is None else float(candidate)
        except (TypeError, ValueError):
            value = self._now_fn()
        return value if math.isfinite(value) else self._now_fn()

    def _date_for_timestamp(self, timestamp: float) -> str:
        return datetime.fromtimestamp(timestamp, timezone.utc).astimezone(
            self._timezone
        ).date().isoformat()

    def _iso_timestamp(self, timestamp: float) -> str:
        return datetime.fromtimestamp(float(timestamp), timezone.utc).astimezone(
            self._timezone
        ).isoformat(timespec="seconds")

    def _new_id(self, now: float) -> str:
        prefix = datetime.fromtimestamp(now, timezone.utc).strftime("%Y%m%dT%H%M%S")
        return f"svc-{prefix}-{uuid.uuid4().hex[:8]}"
