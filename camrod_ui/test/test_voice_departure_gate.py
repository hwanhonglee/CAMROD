"""VoiceDepartureGate sequencing and fail-open timeout, without rclpy."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "python"))

from camrod_ui.voice_departure_gate import VoiceDepartureGate  # noqa: E402


# HH_260929 - Preserve immediate dispatch when no announcement is requested.
def test_empty_keys_dispatch_immediately():
    gate = VoiceDepartureGate()
    fired = []
    published = gate.start((), lambda: fired.append(True), now_s=0.0)
    assert published == ()
    assert fired == [True]
    assert not gate.busy


# HH_260929 - Release departure only after the requested final cue has played.
def test_dispatch_waits_for_last_key_to_finish_playing():
    gate = VoiceDepartureGate()
    fired = []
    published = gate.start(
        ("navigation.site_B4", "navigation.to_campsite"),
        lambda: fired.append(True),
        now_s=0.0,
    )
    assert published == ("navigation.site_B4", "navigation.to_campsite")
    gate.on_voice_state(playing=True, current_key="navigation.site_B4", now_s=1.0)
    gate.on_voice_state(playing=False, current_key="", now_s=1.5)
    assert fired == []
    gate.on_voice_state(
        playing=True, current_key="navigation.to_campsite", now_s=2.0
    )
    gate.on_voice_state(playing=False, current_key="", now_s=3.0)
    assert fired == [True]
    assert not gate.busy


# HH_260929 - A missing voice-state update must not strand an authorized departure.
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
        on_timeout=warned.append,
    )
    gate.tick(4.9)
    assert fired == []
    gate.tick(5.0)
    assert fired == [True]
    assert warned == ["site_departure:B4"]


def test_replacement_and_unrelated_audio_do_not_release_old_dispatch():
    gate = VoiceDepartureGate()
    fired = []
    gate.start(
        ("navigation.to_campsite",), lambda: fired.append("old"), now_s=0.0
    )
    gate.start(
        ("navigation.site_B5",), lambda: fired.append("new"), now_s=0.1
    )
    gate.on_voice_state(
        playing=True, current_key="navigation.to_campsite", now_s=1.0
    )
    gate.on_voice_state(playing=False, current_key="", now_s=1.5)
    assert fired == []
    gate.on_voice_state(playing=True, current_key="navigation.site_B5", now_s=2.0)
    gate.on_voice_state(playing=False, current_key="", now_s=2.5)
    assert fired == ["new"]


def test_cancel_prevents_delayed_dispatch():
    gate = VoiceDepartureGate()
    fired = []
    gate.start(
        ("navigation.to_campsite",), lambda: fired.append(True), now_s=0.0
    )
    assert gate.cancel()
    gate.tick(20.0)
    gate.on_voice_state(
        playing=True, current_key="navigation.to_campsite", now_s=21.0
    )
    gate.on_voice_state(playing=False, current_key="", now_s=22.0)
    assert fired == []
