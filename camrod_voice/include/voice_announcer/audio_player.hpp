#pragma once

#include <string>
#include <functional>
#include <atomic>
#include <thread>
#include <unordered_map>

struct _Mix_Music;  // forward-declare SDL2_mixer Mix_Music

namespace voice_announcer
{

// Wraps SDL2_mixer music channel for non-blocking MP3/WAV playback.
// Music objects are cached by file path — loaded once, reused on repeat.
// finish_cb is called from a monitor thread when playback ends.
class AudioPlayer
{
public:
  using FinishCallback = std::function<void()>;

  AudioPlayer();
  ~AudioPlayer();

  bool init(int sample_rate = 44100, int channels = 2);

  // Play the audio file at path. Non-blocking.
  // Stops current playback first if interrupt=true.
  bool playFile(const std::string & path, bool interrupt = false);

  void stop();
  bool isPlaying() const;
  void setFinishCallback(FinishCallback cb) { finish_cb_ = std::move(cb); }

private:
  _Mix_Music * loadMusic(const std::string & path);
  void monitorPlayback();

  FinishCallback finish_cb_;
  std::atomic<bool> playing_{false};
  std::atomic<bool> dummy_mode_{false};
  std::thread monitor_thread_;

  std::unordered_map<std::string, _Mix_Music *> cache_;
};

}  // namespace voice_announcer
