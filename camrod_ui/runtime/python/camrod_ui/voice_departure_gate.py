"""ROS-free, cancellable announcement sequencing for one authorized request."""
# HH_260929 - Keep voice completion separate from ROS and the motion-authorizing caller.
# HJ_261106 - Release only once the announcer has been silent (nothing playing,
# nothing queued) for a settle window after the final cue. An interrupting or
# follow-up cue no longer counts as "finished", and the speaker's output lag is
# covered before motion starts.
from __future__ import annotations

from dataclasses import dataclass
from threading import RLock
from typing import Callable, Optional, Sequence, Tuple


@dataclass
class _PendingDispatch:
    keys: Tuple[str, ...]
    on_complete: Callable[[], None]
    label: str
    on_timeout: Optional[Callable[[str], None]]
    started_s: float


class VoiceDepartureGate:
    """Release motion after the final requested announcement stops playing."""

    DEFAULT_TIMEOUT_S = 20.0
    # Speaker/Bluetooth output trails the decoder; stay still this long after
    # the announcer goes silent so motion never overlaps the last word.
    DEFAULT_SETTLE_S = 0.5
    # The announcer starts a queued cue within ~50 ms. Silent this long
    # before the final cue played means it was dropped (debounce, missing
    # WAV, announcer restart) and will never play.
    DEFAULT_DROPPED_GRACE_S = 2.0

    def __init__(
        self,
        *,
        settle_s: float = DEFAULT_SETTLE_S,
        dropped_grace_s: float = DEFAULT_DROPPED_GRACE_S,
    ) -> None:
        self._lock = RLock()
        self._pending: Optional[_PendingDispatch] = None
        self._seen_last_key_playing = False
        self._state_seen = False
        self._deadline_s: Optional[float] = None
        self._idle_since_s: Optional[float] = None
        self._busy_until_s = 0.0
        self._settle_s = max(0.0, float(settle_s))
        self._dropped_grace_s = max(0.5, float(dropped_grace_s))

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._pending is not None

    def cancel(self) -> bool:
        with self._lock:
            had_pending = self._pending is not None
            self._pending = None
            self._seen_last_key_playing = False
            self._state_seen = False
            self._deadline_s = None
            self._idle_since_s = None
            return had_pending

    def start(
        self,
        keys: Sequence[str],
        on_complete: Callable[[], None],
        *,
        now_s: float,
        label: str = "",
        timeout_s: float = DEFAULT_TIMEOUT_S,
        on_timeout: Optional[Callable[[str], None]] = None,
    ) -> Tuple[str, ...]:
        ordered = tuple(key for key in keys if key)
        if not ordered:
            self.cancel()
            on_complete()
            return ()
        with self._lock:
            # HH_260929 - Suppress a duplicate cue sequence for the same pending request.
            if (
                self._pending is not None
                and self._pending.keys == ordered
                and self._pending.label == label
            ):
                return ()
            self.cancel()
            self._pending = _PendingDispatch(
                ordered, on_complete, label, on_timeout, float(now_s))
            self._busy_until_s = float(now_s)
            self._deadline_s = now_s + max(0.5, float(timeout_s))
        return ordered

    def on_voice_state(
        self,
        *,
        playing: bool,
        current_key: str,
        now_s: float,
        queue_size: int = 0,
    ) -> None:
        with self._lock:
            if self._pending is None:
                return
            self._state_seen = True
            silent = not playing and int(queue_size) <= 0
            if playing and current_key == self._pending.keys[-1]:
                self._seen_last_key_playing = True
            if silent:
                if self._idle_since_s is None:
                    self._idle_since_s = float(now_s)
            else:
                # Anything still speaking or queued, ours or not, holds motion.
                self._idle_since_s = None
                self._busy_until_s = float(now_s)
        self._maybe_release(now_s)

    def tick(self, now_s: float) -> None:
        # HH_260929 - Bound a missing voice-state acknowledgment so departure cannot stall forever.
        pending = None
        with self._lock:
            if (
                self._pending is not None
                and self._deadline_s is not None
                and now_s >= self._deadline_s
            ):
                pending = self._pending
                self.cancel()
        if pending is not None:
            if pending.on_timeout is not None:
                pending.on_timeout(pending.label)
            pending.on_complete()
            return
        self._maybe_release(now_s)

    def _maybe_release(self, now_s: float) -> None:
        pending = None
        ready = False
        with self._lock:
            if self._pending is None:
                return
            if not self._state_seen:
                # No announcer state at all since the request: the announcer
                # is not running (it publishes every 0.5 s).
                if now_s - self._pending.started_s >= self._dropped_grace_s:
                    pending = self._pending
                    self.cancel()
            elif self._idle_since_s is None:
                return
            elif self._seen_last_key_playing:
                ready = now_s - self._idle_since_s >= self._settle_s
            else:
                # The final cue has not played. The gap between two queued
                # cues lasts milliseconds, so a silence this long means it
                # was dropped and will never come.
                quiet_since = max(self._pending.started_s, self._busy_until_s,
                                  self._idle_since_s)
                ready = now_s - quiet_since >= self._dropped_grace_s
            if pending is None and ready:
                pending = self._pending
                self.cancel()
        if pending is not None:
            pending.on_complete()
