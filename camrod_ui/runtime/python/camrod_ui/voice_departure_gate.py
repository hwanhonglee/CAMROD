"""ROS-free, cancellable announcement sequencing for one authorized request."""
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

class VoiceDepartureGate:
    # HH_260911 - Coalesce duplicates and replace pending work without early motion.
    DEFAULT_TIMEOUT_S = 12.0

    def __init__(self) -> None:
        self._lock = RLock()
        self._pending: Optional[_PendingDispatch] = None
        self._seen_last_key_playing = False
        self._deadline_s: Optional[float] = None

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._pending is not None

    def cancel(self) -> bool:
        # HH_260911 - Stop/mission invalidation must also remove delayed callbacks.
        with self._lock:
            had_pending = self._pending is not None
            self._pending = None
            self._seen_last_key_playing = False
            self._deadline_s = None
            return had_pending

    def start(self, keys: Sequence[str], on_complete: Callable[[], None], *,
              now_s: float, label: str = "", timeout_s: float = DEFAULT_TIMEOUT_S,
              on_timeout: Optional[Callable[[str], None]] = None) -> Tuple[str, ...]:
        ordered = tuple(key for key in keys if key)
        if not ordered:
            self.cancel()
            on_complete()
            return ()
        with self._lock:
            if (self._pending is not None and self._pending.keys == ordered
                    and self._pending.label == label):
                return ()  # Do not extend the existing deadline on duplicate input.
            self._pending = _PendingDispatch(ordered, on_complete, label, on_timeout)
            self._seen_last_key_playing = False
            self._deadline_s = now_s + max(0.5, float(timeout_s))
        return ordered

    def on_voice_state(self, *, playing: bool, current_key: str, now_s: float) -> None:
        pending = None
        with self._lock:
            if self._pending is None:
                return
            if not self._seen_last_key_playing:
                if playing and current_key == self._pending.keys[-1]:
                    self._seen_last_key_playing = True
                return
            if not playing:
                pending = self._pending
                self.cancel()
        if pending is not None:
            pending.on_complete()

    def tick(self, now_s: float) -> None:
        pending = None
        with self._lock:
            if (self._pending is not None and self._deadline_s is not None
                    and now_s >= self._deadline_s):
                pending = self._pending
                self.cancel()
        if pending is not None:
            # HH_260911 - Timeout releases only the current request, never old work.
            if pending.on_timeout is not None:
                pending.on_timeout(pending.label)
            pending.on_complete()
