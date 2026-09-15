"""HH_260915 - Observational mission events; never grant or change motion authority."""
from __future__ import annotations

import json
import math
import os
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path


def default_mission_records_path() -> Path:
    root = os.environ.get("XDG_STATE_HOME", "").strip()
    return (Path(root).expanduser() if root else Path.home() / ".local/state") / "camrod/mission_records"


class MissionRecordingEmitter:
    """Correlate approved actions without persisting or commanding a vehicle.

    The recording ID deliberately survives an operator STOP. The controller's
    identity/authorization still follows its existing cancellation policy.
    Transport failures never escape into an authorization callback.
    """

    def __init__(self, publish, *, now_fn=time.time, session=None):
        self.publish = publish
        self.now_fn = now_fn
        self.session = session or uuid.uuid4().hex
        self.seq = 0
        self.mission_id = ""
        self.site = ""
        self.intent = ""
        self.last_phase = None
        self.return_seen = False
        self.error = ""
        self._lock = threading.RLock()

    def _emit(self, event, **fields):
        with self._lock:
            self.seq += 1
            payload = {
                "schema": 1, "event": event, "producer_session": self.session,
                "seq": self.seq, "at_unix": float(self.now_fn()),
                "mission_id": self.mission_id, "site": self.site, "intent": self.intent,
                **fields,
            }
            try:
                self.publish(json.dumps(payload, ensure_ascii=False, allow_nan=False))
                self.error = ""
                return True
            except Exception as exc:  # observational only: no control-path exception
                self.error = f"{type(exc).__name__}: {exc}"
                return False

    def start(self, site, intent, generation, attempt_id="", source=""):
        with self._lock:
            self.mission_id = f"{self.session}:mission:{int(generation)}"
            self.site = str(site)[:128]
            self.intent = str(intent)
            self.last_phase = None
            self.return_seen = False
            return self._emit("mission_started", attempt_id=str(attempt_id),
                              source=str(source).split(":", 1)[0][:128])

    def phase(self, state, name, description="", leg_kind=""):
        with self._lock:
            # After a backend restart an admitted return may have a blank ID.
            # The journal binds that producer to its ONE persisted open mission;
            # keep forwarding its parking/completion observations as well.
            if not self.mission_id and not self.return_seen:
                return False
            state = int(state)
            if state == 0 and str(name).upper() == "ROAD_HANDOFF_READY":
                # Handoff is neither finished parking nor an interruption.
                return False
            phase = str(name)
            prefix = "camping_site_maneuver_controller:"
            if str(description).startswith(prefix):
                phase = str(description)[len(prefix):].split(":", 1)[0] or phase
            signature = (state, phase, str(leg_kind), str(description))
            if signature == self.last_phase:
                return False
            result = self._emit("phase", state=state, state_name=str(name),
                                phase=phase, reason=str(description)[:2048],
                                leg_kind=str(leg_kind))
            if result:
                self.last_phase = signature
                if state in {3, 10} or (state == 9 and leg_kind == "return"):
                    self.return_seen = True
                if state in {0, 12, 13} and self.return_seen:
                    self.mission_id = self.site = self.intent = ""
                    self.return_seen = False
            return result

    def request_return(self, source="", *, final_return=True):
        # Blank identity after a backend restart means: the journal may attach
        # this approved return to its ONE persisted open mission, never guess a
        # site from an unrelated historical record.
        with self._lock:
            emitted = self._emit("return_requested", reason=str(source)[:1024],
                                 final_return=bool(final_return))
            if emitted and final_return:
                self.return_seen = True
                self.last_phase = None
            return emitted

    def stop(self, reason=""):
        # HH_260915 - A real backend STOP is global even after backend restart
        # erased its local recording identity. This never authorizes motion.
        return self._emit("stop_requested", reason=str(reason)[:1024], global_stop=True)


def load_mission_recording_snapshot(root, *, now_s=None, maximum_age_s=5.0, limit=100,
                                    emitter_error=""):
    """Read only the recorder's bounded atomic export; never open a legacy DB.

    Missing/stale data is an explicit error instead of a believable zero KPI.
    This endpoint cannot select or download arbitrary files through URL input.
    """
    now_s = time.time() if now_s is None else float(now_s)
    path = Path(root).expanduser() / "snapshot.json"
    try:
        with path.open("rb") as stream:
            raw = stream.read(8 * 1024 * 1024 + 1)
        if len(raw) > 8 * 1024 * 1024:
            raise ValueError("recorder snapshot exceeds size limit")
        data = json.loads(raw)
        if not isinstance(data, dict) or data.get("schema_version") != 1:
            raise ValueError("unsupported recorder snapshot")
        generated = data.get("generated_at")
        stamp = (float(generated) if isinstance(generated, (int, float)) else
                 datetime.fromisoformat(str(generated).replace("Z", "+00:00")).timestamp())
        age = now_s - stamp
        if not math.isfinite(stamp) or age < -2 or age > maximum_age_s:
            raise ValueError("mission recorder snapshot is stale; check recorder node")
        missions = data.get("missions")
        if not isinstance(missions, list) or not isinstance(data.get("recorder"), dict):
            raise ValueError("incomplete recorder snapshot")
        data["missions"] = missions[:max(1, min(100, int(limit)))]
        if emitter_error:
            # A failed final event may have no later sequence to reveal the
            # gap. Keep the last measurements, but never report healthy capture.
            error = f"mission lifecycle publishing failed: {str(emitter_error)[:1024]}"
            data["error"] = error
            data["recorder"]["status"] = "DEGRADED"
            data["recorder"]["error"] = "; ".join(filter(None, (
                str(data["recorder"].get("error", "")), error,
            )))
            return data, 503
        return data, 200
    except (OSError, ValueError, TypeError, OverflowError) as exc:
        error = str(exc)
        if emitter_error:
            error += f"; mission lifecycle publishing failed: {str(emitter_error)[:1024]}"
        return {"schema_version": 1, "error": error,
                "recorder": {"status": "unavailable", "error": error,
                             "storage_root": str(Path(root).expanduser())}}, 503
