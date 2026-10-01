#pragma once

#include <rclcpp/rclcpp.hpp>
#include <avg_msgs/msg/audio_request.hpp>
#include <avg_msgs/msg/avg_bool.hpp>
#include <avg_msgs/msg/voice_state.hpp>

#include "voice_announcer/priority_queue.hpp"
#include "voice_announcer/audio_player.hpp"

#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>

namespace voice_announcer
{

class VoiceAnnouncerNode : public rclcpp::Node
{
public:
  // HH_260616 - Bridge audio requests to queued playback and publish playback state.
  explicit VoiceAnnouncerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~VoiceAnnouncerNode();

  bool isIdle() const { return !player_.isPlaying() && queue_.empty(); }

private:
  void declareParameters();
  void onAudioRequest(avg_msgs::msg::AudioRequest::ConstSharedPtr msg);
  void onBgmRequest(avg_msgs::msg::AvgBool::ConstSharedPtr msg);
  void processingLoop();
  void publishState();

  // HH_260812 - Sync the trip bed only between speech cues so departure plays first.
  void syncBgm();
  void playShutdownCue();

  std::string keyToWavPath(const std::string & key, const std::string & locale) const;

  // Parameters
  std::string default_locale_;
  std::string audio_dir_;
  bool enable_bgm_{true};
  std::string bgm_key_;
  std::string bgm_path_;
  int bgm_fade_in_ms_{0};
  int bgm_fade_out_ms_{0};
  int bgm_duck_fade_ms_{0};
  bool enable_shutdown_audio_{true};
  std::string shutdown_key_;
  std::chrono::milliseconds shutdown_timeout_{0};

  PriorityQueue queue_;
  AudioPlayer player_;

  rclcpp::Subscription<avg_msgs::msg::AudioRequest>::SharedPtr say_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr bgm_sub_;
  rclcpp::Publisher<avg_msgs::msg::VoiceState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  std::thread processing_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> bgm_requested_{false};
  // HH_260812 - Prevent playback-state publishes after executor teardown.
  std::atomic<bool> state_publishable_{true};

  mutable std::mutex key_mutex_;
  std::string current_key_;
};

}  // namespace voice_announcer
