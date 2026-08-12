#pragma once

#include <string>
#include <functional>
#include <atomic>
#include <mutex>
#include <thread>
#include <unordered_map>

struct Mix_Chunk;   // forward-declare SDL2_mixer Mix_Chunk
struct _Mix_Music;  // forward-declare SDL2_mixer Mix_Music

namespace voice_announcer
{

// Wraps SDL2_mixer for simultaneous speech and background music.
// Speech plays on a reserved mixer channel (WAV chunks) while the music stream
// carries the looping BGM bed (WAV/MP3), so a cue can be heard over the bed.
// Assets are cached by file path — loaded once, reused on repeat.
// finish_cb is called from a monitor thread when speech playback ends.
class AudioPlayer
{
public:
  using FinishCallback = std::function<void()>;

  AudioPlayer();
  ~AudioPlayer();

  bool init(int sample_rate = 44100, int channels = 2);

  // ── Speech ────────────────────────────────────────────────────────────────

  // Play the speech file at path on the reserved channel. Non-blocking.
  // Stops current speech first if interrupt=true.
  bool playFile(const std::string & path, bool interrupt = false);

  void stop();
  bool isPlaying() const;
  void setFinishCallback(FinishCallback cb) { finish_cb_ = std::move(cb); }
  void setVoiceVolume(float gain);

  // ── Background music ──────────────────────────────────────────────────────

  // Start the looping bed. Idempotent while the same bed is already active.
  bool startBgm(const std::string & path, int fade_in_ms = 0);
  void stopBgm(int fade_out_ms = 0);
  bool isBgmActive() const { return bgm_active_.load(); }

  void setBgmVolumes(float normal_gain, float duck_gain);

  // Drop the bed to the duck level while speech plays; restore it afterwards.
  // Ramps over fade_ms on the calling thread, so call it from the playback
  // thread only.
  void duckBgm(bool ducked, int fade_ms = 0);

private:
  bool openDevice(int sample_rate, int channels);
  Mix_Chunk * loadChunk(const std::string & path);
  _Mix_Music * loadMusic(const std::string & path);
  void monitorPlayback();
  int targetBgmVolume() const;
  void rampBgmVolume(int target, int fade_ms);

  FinishCallback finish_cb_;
  std::atomic<bool> playing_{false};
  std::atomic<bool> dummy_mode_{false};
  std::atomic<bool> bgm_active_{false};
  std::atomic<bool> ducked_{false};
  std::thread monitor_thread_;

  int voice_channel_{0};
  int voice_volume_{128};
  int bgm_volume_{70};
  int bgm_duck_volume_{18};

  std::unordered_map<std::string, Mix_Chunk *> chunk_cache_;
  std::unordered_map<std::string, _Mix_Music *> music_cache_;
};

}  // namespace voice_announcer
