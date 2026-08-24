from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _source(relative_path: str) -> str:
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


def test_voice_launch_pins_sdl_to_pipewire_pulse_default_sink():
    launch_source = _source("launch/voice.launch.py")

    assert "additional_env={'SDL_AUDIODRIVER': 'pulseaudio'}" in launch_source


def test_audio_player_exposes_driver_and_dummy_state():
    header = _source("include/voice_announcer/audio_player.hpp")
    implementation = _source("src/audio_player.cpp")

    assert "std::string audioDriver() const;" in header
    assert "bool isDummyMode() const" in header
    assert "std::string AudioPlayer::audioDriver() const" in implementation
    assert "SDL_GetCurrentAudioDriver()" in implementation


def test_announcer_reports_real_or_dummy_audio_driver_at_startup():
    announcer_source = _source("src/voice_announcer_node.cpp")

    assert "player_.audioDriver()" in announcer_source
    assert "player_.isDummyMode()" in announcer_source
    assert "physical speaker output is disabled" in announcer_source
    assert "AudioPlayer initialized (SDL driver=%s)" in announcer_source
