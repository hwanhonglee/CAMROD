"""VoiceDepartureGate sequencing and fail-open timeout, without rclpy."""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "python"))

from camrod_ui.voice_departure_gate import VoiceDepartureGate  # noqa: E402


def test_empty_keys_dispatch_immediately():
    gate = VoiceDepartureGate()
    fired = []
    published = gate.start((), lambda: fired.append(True), now_s=0.0)
    assert published == ()
    assert fired == [True]
    assert not gate.busy


def test_dispatch_waits_for_last_key_to_finish_playing():
    gate = VoiceDepartureGate()
    fired = []
    published = gate.start(
        ("navigation.site_B4", "navigation.to_campsite"),
        lambda: fired.append(True),
        now_s=0.0,
    )
    assert published == ("navigation.site_B4", "navigation.to_campsite")
    assert gate.busy

    # First key playing must not release the dispatch.
    gate.on_voice_state(playing=True, current_key="navigation.site_B4", now_s=1.0)
    assert fired == []
    gate.on_voice_state(playing=False, current_key="", now_s=1.5)
    assert fired == []

    # Last key starts, then finishes: only then does the dispatch fire.
    gate.on_voice_state(playing=True, current_key="navigation.to_campsite", now_s=2.0)
    assert fired == []
    gate.on_voice_state(playing=True, current_key="navigation.to_campsite", now_s=2.5)
    assert fired == []
    gate.on_voice_state(playing=False, current_key="", now_s=3.0)
    assert fired == [True]
    assert not gate.busy


def test_single_key_sequence():
    gate = VoiceDepartureGate()
    fired = []
    gate.start(("navigation.to_dropzone",), lambda: fired.append(True), now_s=0.0)
    gate.on_voice_state(playing=True, current_key="navigation.to_dropzone", now_s=0.2)
    assert fired == []
    gate.on_voice_state(playing=False, current_key="", now_s=1.0)
    assert fired == [True]


def test_timeout_fires_dispatch_and_reports_when_never_confirmed():
    gate = VoiceDepartureGate()
    fired = []
    warned = []
    gate.start(
        ("navigation.to_campsite",),
        lambda: fired.append(True),
        now_s=0.0,
        timeout_s=5.0,
        label="site_departure:B4",
        on_timeout=lambda label: warned.append(label),
    )
    gate.tick(4.9)
    assert fired == []
    gate.tick(5.0)
    assert fired == [True]
    assert warned == ["site_departure:B4"]
    assert not gate.busy


def test_second_dispatch_replaces_pending_work_without_early_motion():
    # HH_260911 - Only the newest admitted request may execute, after its own cue.
    gate = VoiceDepartureGate()
    fired = []
    gate.start(("navigation.to_campsite",), lambda: fired.append("old"), now_s=0.0)
    published = gate.start(("navigation.to_dropzone",), lambda: fired.append("new"), now_s=0.1)
    assert published == ("navigation.to_dropzone",)
    assert fired == []
    gate.on_voice_state(playing=True, current_key="navigation.to_campsite", now_s=1.0)
    gate.on_voice_state(playing=False, current_key="", now_s=1.5)
    assert fired == []
    gate.on_voice_state(playing=True, current_key="navigation.to_dropzone", now_s=2.0)
    gate.on_voice_state(playing=False, current_key="", now_s=2.5)
    gate.tick(20.0)
    assert fired == ["new"]


def test_unrelated_current_key_does_not_confirm_last_key():
    gate = VoiceDepartureGate()
    fired = []
    gate.start(("navigation.to_campsite",), lambda: fired.append(True), now_s=0.0)
    # Some unrelated higher-priority cue (e.g. safety.obstacle) plays first.
    gate.on_voice_state(playing=True, current_key="safety.obstacle", now_s=0.5)
    gate.on_voice_state(playing=False, current_key="", now_s=0.8)
    assert fired == []
    gate.on_voice_state(playing=True, current_key="navigation.to_campsite", now_s=1.0)
    gate.on_voice_state(playing=False, current_key="", now_s=1.5)
    assert fired == [True]


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
