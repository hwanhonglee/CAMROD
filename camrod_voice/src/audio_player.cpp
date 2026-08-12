#include "voice_announcer/audio_player.hpp"

#include <SDL2/SDL.h>
#include <SDL2/SDL_mixer.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace voice_announcer
{

namespace
{
// Reserved mixer channels: 0 is speech-only, the rest stay free for future cues.
constexpr int kMixerChannels = 4;
constexpr int kReservedChannels = 1;

int toMixVolume(float gain)
{
  const float clamped = std::min(1.0f, std::max(0.0f, gain));
  return static_cast<int>(std::lround(clamped * MIX_MAX_VOLUME));
}
}  // namespace

AudioPlayer::AudioPlayer() = default;

AudioPlayer::~AudioPlayer()
{
  Mix_HaltMusic();
  stop();
  if (monitor_thread_.joinable()) {
    monitor_thread_.join();
  }
  for (auto & [path, chunk] : chunk_cache_) {
    Mix_FreeChunk(chunk);
  }
  chunk_cache_.clear();
  for (auto & [path, music] : music_cache_) {
    Mix_FreeMusic(music);
  }
  music_cache_.clear();
  Mix_CloseAudio();
  Mix_Quit();
  SDL_QuitSubSystem(SDL_INIT_AUDIO);
}

bool AudioPlayer::openDevice(int sample_rate, int channels)
{
  // MP3/OGG decoders must be loaded before the device opens, otherwise
  // Mix_LoadMUS cannot resolve a compressed background-music bed.
  Mix_Init(MIX_INIT_MP3 | MIX_INIT_OGG);
  if (Mix_OpenAudio(sample_rate, MIX_DEFAULT_FORMAT, channels, 2048) < 0) {
    return false;
  }
  Mix_AllocateChannels(kMixerChannels);
  Mix_ReserveChannels(kReservedChannels);
  voice_channel_ = 0;
  Mix_Volume(voice_channel_, voice_volume_);
  Mix_VolumeMusic(targetBgmVolume());
  return true;
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

  if (openDevice(sample_rate, channels)) {
    return true;
  }

  if (dummy_mode_) {
    SDL_QuitSubSystem(SDL_INIT_AUDIO);
    return false;
  }

  SDL_QuitSubSystem(SDL_INIT_AUDIO);
  SDL_setenv("SDL_AUDIODRIVER", "dummy", 1);
  if (SDL_InitSubSystem(SDL_INIT_AUDIO) < 0) {
    return false;
  }
  if (!openDevice(sample_rate, channels)) {
    SDL_QuitSubSystem(SDL_INIT_AUDIO);
    return false;
  }
  dummy_mode_ = true;
  return true;
}

Mix_Chunk * AudioPlayer::loadChunk(const std::string & path)
{
  auto it = chunk_cache_.find(path);
  if (it != chunk_cache_.end()) {
    return it->second;
  }
  Mix_Chunk * chunk = Mix_LoadWAV(path.c_str());
  if (chunk) {
    chunk_cache_[path] = chunk;
  }
  return chunk;
}

_Mix_Music * AudioPlayer::loadMusic(const std::string & path)
{
  auto it = music_cache_.find(path);
  if (it != music_cache_.end()) {
    return it->second;
  }
  _Mix_Music * music = Mix_LoadMUS(path.c_str());
  if (music) {
    music_cache_[path] = music;
  }
  return music;
}

bool AudioPlayer::playFile(const std::string & path, bool interrupt)
{
  if (!interrupt && playing_.load()) {
    return false;
  }

  // Halt first, then let the previous monitor thread observe the silence and
  // exit — joining after starting new audio would block for the whole clip.
  stop();
  if (monitor_thread_.joinable()) {
    monitor_thread_.join();
  }

  Mix_Chunk * chunk = loadChunk(path);
  if (!chunk) {
    return false;
  }

  if (Mix_PlayChannel(voice_channel_, chunk, 0) == -1) {
    return false;
  }

  playing_.store(true);

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
  Mix_HaltChannel(voice_channel_);
  playing_.store(false);
}

bool AudioPlayer::isPlaying() const
{
  return playing_.load();
}

void AudioPlayer::setVoiceVolume(float gain)
{
  voice_volume_ = toMixVolume(gain);
  Mix_Volume(voice_channel_, voice_volume_);
}

void AudioPlayer::monitorPlayback()
{
  while (Mix_Playing(voice_channel_)) {
    SDL_Delay(50);
  }
  // PulseAudio (WSL2 등): SDL2 디코더가 먼저 끝나도 오디오 드라이버 버퍼에
  // 수백 ms 분량의 데이터가 남아 있다. 다음 playFile()이 채널을 정지하기
  // 전에 버퍼가 소진되도록 추가 대기한다.
  const char * drv = SDL_GetCurrentAudioDriver();
  if (drv != nullptr && std::string(drv) == "pulseaudio") {
    SDL_Delay(300);
  }
  playing_.store(false);
  if (finish_cb_) {
    finish_cb_();
  }
}

bool AudioPlayer::startBgm(const std::string & path, int fade_in_ms)
{
  if (bgm_active_.load() && Mix_PlayingMusic()) {
    return true;
  }

  // A pending fade-out still counts as playing; clear it before restarting.
  if (Mix_FadingMusic() == MIX_FADING_OUT) {
    Mix_HaltMusic();
  }

  _Mix_Music * music = loadMusic(path);
  if (!music) {
    return false;
  }

  Mix_VolumeMusic(targetBgmVolume());
  const int rc = fade_in_ms > 0
    ? Mix_FadeInMusic(music, -1, fade_in_ms)
    : Mix_PlayMusic(music, -1);
  if (rc == -1) {
    return false;
  }

  bgm_active_.store(true);
  return true;
}

void AudioPlayer::stopBgm(int fade_out_ms)
{
  bgm_active_.store(false);
  if (fade_out_ms > 0) {
    Mix_FadeOutMusic(fade_out_ms);
  } else {
    Mix_HaltMusic();
  }
}

void AudioPlayer::setBgmVolumes(float normal_gain, float duck_gain)
{
  bgm_volume_ = toMixVolume(normal_gain);
  bgm_duck_volume_ = toMixVolume(duck_gain);
  Mix_VolumeMusic(targetBgmVolume());
}

int AudioPlayer::targetBgmVolume() const
{
  return ducked_.load() ? bgm_duck_volume_ : bgm_volume_;
}

void AudioPlayer::duckBgm(bool ducked, int fade_ms)
{
  if (ducked_.exchange(ducked) == ducked) {
    return;
  }
  rampBgmVolume(targetBgmVolume(), fade_ms);
}

void AudioPlayer::rampBgmVolume(int target, int fade_ms)
{
  const int start = Mix_VolumeMusic(-1);  // -1 queries without changing
  if (fade_ms <= 0 || start == target || dummy_mode_.load()) {
    Mix_VolumeMusic(target);
    return;
  }

  constexpr int kSteps = 8;
  const Uint32 step_delay = static_cast<Uint32>(std::max(1, fade_ms / kSteps));
  for (int step = 1; step <= kSteps; ++step) {
    Mix_VolumeMusic(start + (target - start) * step / kSteps);
    SDL_Delay(step_delay);
  }
}

}  // namespace voice_announcer
