"""Persistent proof-of-operation metrics for CAMROD service runs.

The tracker deliberately has no ROS dependency.  The UI backend feeds it
accepted service requests, service-state transitions, and platform velocity
samples.  This keeps evidence collection testable and prevents a storage
failure from affecting motion control.
"""

from __future__ import annotations

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


SERVICE_METRICS_SCHEMA_VERSION = 1


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

        self._open_store()

    # ------------------------------------------------------------------ store

    @property
    def persistence_enabled(self) -> bool:
        return self._connection is not None

    @property
    def persistence_error(self) -> str:
        return self._persistence_error

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
            connection.execute("PRAGMA journal_mode=WAL")
            connection.execute("PRAGMA synchronous=NORMAL")
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

    @staticmethod
    def _row_to_record(row: sqlite3.Row) -> Dict[str, Any]:
        return {
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
        }

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
                    updated_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
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
                    updated_at=excluded.updated_at
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
        now_s: Optional[float] = None,
    ) -> bool:
        """Start one accepted service, coalescing retries for the same site."""
        now = self._valid_now(now_s)
        normalized_site = str(site).strip() or "미지정"
        with self._lock:
            if self._active is not None:
                if self._active["site"] == normalized_site:
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
            self._last_checkpoint_monotonic = self._monotonic_fn()
            self._persist_record(record)
            return True

    def observe_service_state(
        self,
        state: int,
        state_name: str = "",
        *,
        now_s: Optional[float] = None,
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
            if state_value in self.COMPLETION_STATES:
                self._finish_active(self.COMPLETED_RESULT, now)
                return True
            if state_value in self.INTERRUPT_STATES:
                self._finish_active(self.INTERRUPTED_RESULT, now)
                return True
            # Service-state transitions are low frequency and valuable recovery
            # evidence even when the robot is stationary at a site.
            self._persist_record(self._active)
            return False

    def _finish_active(self, result: str, now: float) -> None:
        record = self._active
        if record is None:
            return
        record["result"] = result
        record["ended_at"] = max(float(record["started_at"]), now)
        record["updated_at"] = record["ended_at"]
        self._persist_record(record)
        self._active = None
        self._previous_velocity_sample = None

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
            return 0.0
        if not all(math.isfinite(value) for value in (vx, vy, sample_time)):
            return 0.0

        speed = math.hypot(vx, vy)
        with self._lock:
            if self._active is None:
                self._previous_velocity_sample = None
                return 0.0
            if speed > self.maximum_speed_mps:
                self._previous_velocity_sample = (sample_time, 0.0)
                return 0.0
            if speed < self.minimum_speed_mps:
                speed = 0.0

            previous = self._previous_velocity_sample
            if previous is None:
                self._previous_velocity_sample = (sample_time, speed)
                return 0.0
            previous_time, previous_speed = previous
            delta_s = sample_time - previous_time
            if delta_s == 0.0:
                return 0.0
            if delta_s < 0.0 or delta_s > self.maximum_sample_gap_s:
                self._previous_velocity_sample = (sample_time, speed)
                return 0.0
            self._previous_velocity_sample = (sample_time, speed)

            added_m = 0.5 * (previous_speed + speed) * delta_s
            if not math.isfinite(added_m) or added_m <= 0.0:
                return 0.0
            self._active["distance_m"] += added_m
            self._active["updated_at"] = self._now_fn()

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

            return {
                "schema_version": SERVICE_METRICS_SCHEMA_VERSION,
                "timezone": self.timezone_name,
                "generated_at": self._iso_timestamp(now),
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
        }

    @staticmethod
    def _format_totals(totals: Dict[str, Any]) -> Dict[str, Any]:
        formatted = dict(totals)
        distance_m = max(0.0, float(formatted.get("distance_m", 0.0)))
        formatted["distance_m"] = round(distance_m, 2)
        formatted["distance_km"] = round(distance_m / 1000.0, 3)
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
