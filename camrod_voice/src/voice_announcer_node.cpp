#include "voice_announcer/voice_announcer_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <chrono>

using namespace std::chrono_literals;

namespace voice_announcer
{

VoiceAnnouncerNode::VoiceAnnouncerNode(const rclcpp::NodeOptions & options)
: Node("voice_announcer", options)
{
  declareParameters();

  if (!player_.init()) {
    RCLCPP_ERROR(get_logger(), "AudioPlayer init failed — check SDL2 audio device");
  }

  player_.setFinishCallback([this]() {
    std::lock_guard<std::mutex> lock(key_mutex_);
    current_key_.clear();
  });

  say_sub_ = create_subscription<avg_msgs::msg::AudioRequest>(
    "~/say", rclcpp::QoS(10),
    std::bind(&VoiceAnnouncerNode::onAudioRequest, this, std::placeholders::_1));

  state_pub_ = create_publisher<avg_msgs::msg::VoiceState>(
    "~/state", rclcpp::QoS(10));

  state_timer_ = create_wall_timer(500ms, std::bind(&VoiceAnnouncerNode::publishState, this));

  running_.store(true);
  processing_thread_ = std::thread(&VoiceAnnouncerNode::processingLoop, this);

  RCLCPP_INFO(get_logger(), "VoiceAnnouncer started (locale=%s, audio_dir=%s)",
    default_locale_.c_str(), audio_dir_.c_str());
}

VoiceAnnouncerNode::~VoiceAnnouncerNode()
{
  running_.store(false);
  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
}

void VoiceAnnouncerNode::declareParameters()
{
  default_locale_ = declare_parameter<std::string>("locale", "ko-KR");
  audio_dir_      = declare_parameter<std::string>("audio_dir", "");

  if (audio_dir_.empty()) {
    try {
      auto pkg_share = ament_index_cpp::get_package_share_directory("camrod_voice");
      audio_dir_ = pkg_share + "/resource/audio";
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Could not locate camrod_voice share dir");
    }
  }
}

std::string VoiceAnnouncerNode::keyToWavPath(
  const std::string & key, const std::string & locale) const
{
  std::string rel = key;
  std::replace(rel.begin(), rel.end(), '.', '/');
  const std::string base = audio_dir_ + "/" + locale + "/" + rel;
  for (const char * ext : {".wav", ".mp3"}) {
    if (std::filesystem::exists(base + ext)) {
      return base + ext;
    }
  }
  return base + ".wav";  // default for missing-file warning message
}

void VoiceAnnouncerNode::onAudioRequest(
  avg_msgs::msg::AudioRequest::ConstSharedPtr msg)
{
  if (msg->key.empty()) {
    RCLCPP_WARN(get_logger(), "Received AudioRequest with empty key — ignoring");
    return;
  }

  const std::string locale = msg->locale.empty() ? default_locale_ : msg->locale;

  VoiceRequest req;
  req.key      = msg->key;
  req.wav_path = keyToWavPath(msg->key, locale);
  req.priority = msg->priority;
  req.interrupt = msg->interrupt;

  if (!std::filesystem::exists(req.wav_path)) {
    RCLCPP_WARN(get_logger(), "WAV file not found: %s", req.wav_path.c_str());
    return;
  }

  auto interrupt_cb = [this]() { player_.stop(); };
  if (queue_.push(req, interrupt_cb)) {
    avg_msgs::msg::VoiceState s;
    s.header.stamp = now();
    s.current_key  = req.key;
    s.queue_size   = static_cast<uint8_t>(queue_.size());
    s.state        = avg_msgs::msg::VoiceState::STATE_QUEUED;
    state_pub_->publish(s);
  }
}

void VoiceAnnouncerNode::processingLoop()
{
  while (running_.load()) {
    if (player_.isPlaying() || queue_.empty()) {
      std::this_thread::sleep_for(50ms);
      continue;
    }

    auto req_opt = queue_.pop();
    if (!req_opt) {
      continue;
    }
    const auto & req = *req_opt;

    RCLCPP_INFO(get_logger(), "Playing [%s]: %s", req.key.c_str(), req.wav_path.c_str());

    {
      std::lock_guard<std::mutex> lock(key_mutex_);
      current_key_ = req.key;
    }

    {
      avg_msgs::msg::VoiceState s;
      s.header.stamp = now();
      s.current_key  = req.key;
      s.queue_size   = static_cast<uint8_t>(queue_.size());
      s.state        = avg_msgs::msg::VoiceState::STATE_PLAYING;
      state_pub_->publish(s);
    }

    if (!player_.playFile(req.wav_path, req.interrupt)) {
      RCLCPP_WARN(get_logger(), "Failed to play: %s", req.wav_path.c_str());
    }
    publishState();
  }
}

void VoiceAnnouncerNode::publishState()
{
  avg_msgs::msg::VoiceState state;
  state.header.stamp = now();
  {
    std::lock_guard<std::mutex> lock(key_mutex_);
    state.current_key = current_key_;
  }
  state.queue_size   = static_cast<uint8_t>(queue_.size());

  if (player_.isPlaying()) {
    state.state = avg_msgs::msg::VoiceState::STATE_PLAYING;
  } else if (!queue_.empty()) {
    state.state = avg_msgs::msg::VoiceState::STATE_QUEUED;
  } else {
    state.state = avg_msgs::msg::VoiceState::STATE_IDLE;
  }

  state_pub_->publish(state);
}

}  // namespace voice_announcer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(voice_announcer::VoiceAnnouncerNode)
