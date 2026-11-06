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
    # HJ_261106 - Silence must hold for the speaker settle window first.
    gate.tick(3.4)
    assert fired == []
    gate.tick(3.5)
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
    # HJ_261106 - Announcer alive but busy with another cue the whole time.
    gate.on_voice_state(playing=True, current_key="system.ready", now_s=0.1)
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
    gate.tick(3.0)
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


# HJ_261106 - An interrupting or follow-up cue is still speech: motion waits
# until the announcer is completely silent.
def test_interrupting_cue_after_departure_cue_still_holds_motion():
    gate = VoiceDepartureGate()
    fired = []
    gate.start(("navigation.to_campsite",), lambda: fired.append(True), now_s=0.0)
    gate.on_voice_state(playing=True, current_key="navigation.to_campsite", now_s=0.1)
    # Cut off by a priority-2 cue: briefly not playing but something queued.
    gate.on_voice_state(playing=False, current_key="", now_s=2.0, queue_size=1)
    gate.on_voice_state(playing=True, current_key="docking.failed", now_s=2.05)
    gate.tick(3.0)
    assert fired == []
    gate.on_voice_state(playing=False, current_key="", now_s=6.0)
    gate.tick(6.2)
    assert fired == []
    gate.tick(6.5)
    assert fired == [True]


def test_gap_between_site_name_and_departure_cue_does_not_release():
    gate = VoiceDepartureGate()
    fired = []
    gate.start(("navigation.site_B1", "navigation.to_campsite"),
               lambda: fired.append(True), now_s=0.0)
    gate.on_voice_state(playing=True, current_key="navigation.site_B1", now_s=0.1)
    gate.on_voice_state(playing=False, current_key="", now_s=2.5)
    gate.tick(2.52)
    gate.on_voice_state(playing=True, current_key="navigation.to_campsite", now_s=2.55)
    gate.tick(4.0)
    assert fired == []


def test_dropped_cue_releases_after_silent_grace():
    gate = VoiceDepartureGate()
    fired = []
    gate.start(("navigation.to_campsite",), lambda: fired.append(True), now_s=0.0)
    gate.on_voice_state(playing=False, current_key="", now_s=0.5)
    gate.tick(2.4)
    assert fired == []
    gate.tick(2.5)
    assert fired == [True]


def test_absent_announcer_does_not_hold_departure_for_the_full_timeout():
    gate = VoiceDepartureGate()
    fired = []
    gate.start(("navigation.to_campsite",), lambda: fired.append(True), now_s=0.0)
    gate.tick(1.9)
    assert fired == []
    gate.tick(2.0)
    assert fired == [True]
