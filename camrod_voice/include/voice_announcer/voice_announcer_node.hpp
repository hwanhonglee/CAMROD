#pragma once

#include <rclcpp/rclcpp.hpp>
#include <avg_msgs/msg/audio_request.hpp>
#include <avg_msgs/msg/voice_state.hpp>

#include "voice_announcer/priority_queue.hpp"
#include "voice_announcer/audio_player.hpp"

#include <thread>
#include <atomic>
#include <mutex>
#include <string>

namespace voice_announcer
{

class VoiceAnnouncerNode : public rclcpp::Node
{
public:
  explicit VoiceAnnouncerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~VoiceAnnouncerNode();

  bool isIdle() const { return !player_.isPlaying() && queue_.empty(); }

private:
  void declareParameters();
  void onAudioRequest(avg_msgs::msg::AudioRequest::ConstSharedPtr msg);
  void processingLoop();
  void publishState();

  std::string keyToWavPath(const std::string & key, const std::string & locale) const;

  // Parameters
  std::string default_locale_;
  std::string audio_dir_;

  PriorityQueue queue_;
  AudioPlayer player_;

  rclcpp::Subscription<avg_msgs::msg::AudioRequest>::SharedPtr say_sub_;
  rclcpp::Publisher<avg_msgs::msg::VoiceState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  std::thread processing_thread_;
  std::atomic<bool> running_{false};

  mutable std::mutex key_mutex_;
  std::string current_key_;
};

}  // namespace voice_announcer
