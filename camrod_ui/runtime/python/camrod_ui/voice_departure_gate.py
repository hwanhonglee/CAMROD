"""Defer a movement command until its voice announcement finishes playing.

Previously the UI backend published an engage/goal command the instant a
site or a return was accepted, while the matching voice cue only followed
later and reactively (once planning noticed the trip had started). Voice and
motion therefore started together. This module holds the *dispatch* — not
the acceptance/HTTP response — until the announcer confirms the cue actually
played, so bystanders hear "moving to campsite" before the robot moves.

Pure and ROS-free so it is unit-testable without rclpy. The owning node is
responsible for actually publishing the AudioRequest keys returned by
``start()`` and for feeding VoiceState updates back through
``on_voice_state()``/``tick()``.

Fail-open by design: a stuck or crashed voice_announcer must never strand a
mission. If the final announcement is never confirmed within ``timeout_s``,
the pending dispatch fires anyway (loudly logged by the caller).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Tuple


@dataclass
class _PendingDispatch:
    keys: Tuple[str, ...]
    on_complete: Callable[[], None]
    label: str
    timeout_s: float
    on_timeout: Optional[Callable[[str], None]]


class VoiceDepartureGate:
    """Holds one pending dispatch until its announcement sequence completes."""

    DEFAULT_TIMEOUT_S = 12.0

    def __init__(self) -> None:
        self._pending: Optional[_PendingDispatch] = None
        self._seen_last_key_playing = False
        self._deadline_s: Optional[float] = None

    @property
    def busy(self) -> bool:
        return self._pending is not None

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
        """Begin gating ``on_complete`` behind ``keys`` finishing playback.

        Returns the keys the caller must publish as AudioRequests, in order.
        An empty ``keys`` sequence (or a gate already busy) fires
        ``on_complete`` immediately in-line and returns an empty tuple — a
        second departure can never silently wait forever behind a first one
        that is already in flight.
        """
        ordered = tuple(key for key in keys if key)
        if not ordered or self._pending is not None:
            on_complete()
            return ()
        self._pending = _PendingDispatch(
            keys=ordered,
            on_complete=on_complete,
            label=label,
            timeout_s=max(0.5, float(timeout_s)),
            on_timeout=on_timeout,
        )
        self._seen_last_key_playing = False
        self._deadline_s = now_s + self._pending.timeout_s
        return ordered

    def on_voice_state(self, *, playing: bool, current_key: str, now_s: float) -> None:
        """Feed one ``avg_msgs/VoiceState`` sample (announcer/state topic)."""
        if self._pending is None:
            return
        last_key = self._pending.keys[-1]
        if not self._seen_last_key_playing:
            if playing and current_key == last_key:
                self._seen_last_key_playing = True
            return
        # The last cue was seen playing and is no longer: it finished (or was
        # interrupted by a higher-priority cue, which is an acceptable trade
        # — waiting forever behind a stalled interrupt is worse).
        if not playing:
            self._fire(timed_out=False)

    def tick(self, now_s: float) -> None:
        """Call periodically so a dead voice pipeline cannot block a mission."""
        if self._pending is None or self._deadline_s is None:
            return
        if now_s >= self._deadline_s:
            self._fire(timed_out=True)

    def _fire(self, *, timed_out: bool) -> None:
        pending = self._pending
        self._pending = None
        self._seen_last_key_playing = False
        self._deadline_s = None
        if pending is None:
            return
        if timed_out and pending.on_timeout is not None:
            pending.on_timeout(pending.label)
        pending.on_complete()
