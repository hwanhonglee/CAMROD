#include "voice_announcer/audio_player.hpp"

#include <SDL2/SDL.h>
#include <SDL2/SDL_mixer.h>
#include <string>

namespace voice_announcer
{

AudioPlayer::AudioPlayer() = default;

AudioPlayer::~AudioPlayer()
{
  stop();
  if (monitor_thread_.joinable()) {
    monitor_thread_.join();
  }
  for (auto & [path, music] : cache_) {
    Mix_FreeMusic(music);
  }
  cache_.clear();
  Mix_CloseAudio();
  SDL_QuitSubSystem(SDL_INIT_AUDIO);
}

bool AudioPlayer::init(int sample_rate, int channels)
{
  if (SDL_InitSubSystem(SDL_INIT_AUDIO) < 0) {
    SDL_setenv("SDL_AUDIODRIVER", "dummy", 1);
    if (SDL_InitSubSystem(SDL_INIT_AUDIO) < 0) {
      return false;
    }
    dummy_mode_ = true;
  }

  if (Mix_OpenAudio(sample_rate, MIX_DEFAULT_FORMAT, channels, 2048) < 0) {
    if (!dummy_mode_) {
      SDL_QuitSubSystem(SDL_INIT_AUDIO);
      SDL_setenv("SDL_AUDIODRIVER", "dummy", 1);
      if (SDL_InitSubSystem(SDL_INIT_AUDIO) < 0) {
        return false;
      }
      if (Mix_OpenAudio(sample_rate, MIX_DEFAULT_FORMAT, channels, 2048) < 0) {
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
        return false;
      }
      dummy_mode_ = true;
    } else {
      SDL_QuitSubSystem(SDL_INIT_AUDIO);
      return false;
    }
  }

  return true;
}

_Mix_Music * AudioPlayer::loadMusic(const std::string & path)
{
  auto it = cache_.find(path);
  if (it != cache_.end()) {
    return it->second;
  }
  _Mix_Music * music = Mix_LoadMUS(path.c_str());
  if (music) {
    cache_[path] = music;
  }
  return music;
}

bool AudioPlayer::playFile(const std::string & path, bool interrupt)
{
  if (!interrupt && playing_.load()) {
    return false;
  }

  stop();  // always halt before playing new music to prevent SDL audio callback collision

  _Mix_Music * music = loadMusic(path);
  if (!music) {
    return false;
  }

  if (Mix_PlayMusic(music, 0) == -1) {
    return false;
  }

  playing_.store(true);

  if (monitor_thread_.joinable()) {
    monitor_thread_.join();
  }

  if (dummy_mode_.load()) {
    playing_.store(false);
    if (finish_cb_) {
      finish_cb_();
    }
  } else {
    monitor_thread_ = std::thread(&AudioPlayer::monitorPlayback, this);
  }
  return true;
}

void AudioPlayer::stop()
{
  Mix_HaltMusic();
  playing_.store(false);
}

bool AudioPlayer::isPlaying() const
{
  return playing_.load();
}

void AudioPlayer::monitorPlayback()
{
  while (Mix_PlayingMusic()) {
    SDL_Delay(50);
  }
  // PulseAudio (WSL2 등): SDL2 디코더가 먼저 끝나도 오디오 드라이버 버퍼에
  // 수백 ms 분량의 데이터가 남아 있다. 다음 playFile()이 Mix_HaltMusic()을
  // 호출하기 전에 버퍼가 소진되도록 추가 대기한다.
  const char * drv = SDL_GetCurrentAudioDriver();
  if (drv != nullptr && std::string(drv) == "pulseaudio") {
    SDL_Delay(300);
  }
  playing_.store(false);
  if (finish_cb_) {
    finish_cb_();
  }
}

}  // namespace voice_announcer
