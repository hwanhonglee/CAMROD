#include "voice_announcer/voice_announcer_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
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

  player_.setFinishCallback([this]() { publishState(); });

  say_sub_ = create_subscription<avg_msgs::msg::AudioRequest>(
    "~/say", rclcpp::QoS(10),
    std::bind(&VoiceAnnouncerNode::onAudioRequest, this, std::placeholders::_1));

  // Latched: the announcer must pick up an in-progress trip even if it starts
  // (or restarts) after the adapter published the current bed state.
  bgm_sub_ = create_subscription<avg_msgs::msg::AvgBool>(
    "~/bgm", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&VoiceAnnouncerNode::onBgmRequest, this, std::placeholders::_1));

  state_pub_ = create_publisher<avg_msgs::msg::VoiceState>(
    "~/state", rclcpp::QoS(10));

  state_timer_ = create_wall_timer(500ms, std::bind(&VoiceAnnouncerNode::publishState, this));

  running_.store(true);
  processing_thread_ = std::thread(&VoiceAnnouncerNode::processingLoop, this);

  RCLCPP_INFO(get_logger(), "VoiceAnnouncer started (locale=%s, audio_dir=%s, bgm=%s)",
    default_locale_.c_str(), audio_dir_.c_str(),
    enable_bgm_ ? bgm_path_.c_str() : "disabled");
}

VoiceAnnouncerNode::~VoiceAnnouncerNode()
{
  running_.store(false);
  state_publishable_.store(false);
  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
  playShutdownCue();
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

  const auto voice_volume = declare_parameter<double>("voice_volume", 1.0);
  const auto bgm_volume   = declare_parameter<double>("bgm_volume", 0.55);
  // Speech has to stay intelligible over the bed; duck deep rather than a little.
  const auto bgm_duck     = declare_parameter<double>("bgm_duck_volume", 0.12);

  enable_bgm_        = declare_parameter<bool>("enable_bgm", true);
  bgm_key_           = declare_parameter<std::string>("bgm_key", "system.bgm");
  bgm_fade_in_ms_    = declare_parameter<int>("bgm_fade_in_ms", 1500);
  bgm_fade_out_ms_   = declare_parameter<int>("bgm_fade_out_ms", 1000);
  bgm_duck_fade_ms_  = declare_parameter<int>("bgm_duck_fade_ms", 200);

  enable_shutdown_audio_ = declare_parameter<bool>("enable_shutdown_audio", true);
  shutdown_key_          = declare_parameter<std::string>("shutdown_key", "system.shutdown");
  // Launch escalates SIGINT to SIGTERM after 5 s; stay inside that window.
  shutdown_timeout_ = std::chrono::milliseconds(
    static_cast<int64_t>(declare_parameter<double>("shutdown_timeout_s", 4.5) * 1000.0));

  player_.setVoiceVolume(static_cast<float>(voice_volume));
  player_.setBgmVolumes(static_cast<float>(bgm_volume), static_cast<float>(bgm_duck));

  bgm_path_ = bgm_key_.empty() ? std::string{} : keyToWavPath(bgm_key_, default_locale_);
  if (enable_bgm_ && !std::filesystem::exists(bgm_path_)) {
    RCLCPP_WARN(get_logger(), "BGM asset not found: %s — background music disabled",
      bgm_path_.c_str());
    enable_bgm_ = false;
    bgm_path_.clear();
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

void VoiceAnnouncerNode::onBgmRequest(avg_msgs::msg::AvgBool::ConstSharedPtr msg)
{
  const bool requested = enable_bgm_ && msg->data;
  if (bgm_requested_.exchange(requested) == requested) {
    return;
  }

  // Stopping is immediate — the trip is over, so the bed must not linger under
  // the arrival cue. Starting waits for the playback thread to go idle.
  if (!requested) {
    player_.stopBgm(bgm_fade_out_ms_);
  }
  RCLCPP_INFO(get_logger(), "BGM %s", requested ? "requested" : "released");
}

void VoiceAnnouncerNode::processingLoop()
{
  while (running_.load()) {
    if (player_.isPlaying()) {
      std::this_thread::sleep_for(20ms);
      continue;
    }

    if (queue_.empty()) {
      // Nothing more to say: lift the duck and settle the bed to its request.
      player_.duckBgm(false, bgm_duck_fade_ms_);
      syncBgm();
      std::this_thread::sleep_for(50ms);
      continue;
    }

    auto req_opt = queue_.pop();
    if (!req_opt) {
      continue;
    }
    const auto & req = *req_opt;

    RCLCPP_INFO(get_logger(), "Playing [%s]: %s", req.key.c_str(), req.wav_path.c_str());

    player_.duckBgm(true, bgm_duck_fade_ms_);
    if (!player_.playFile(req.wav_path, req.interrupt)) {
      RCLCPP_WARN(get_logger(), "Failed to play: %s", req.wav_path.c_str());
    }

    {
      std::lock_guard<std::mutex> lock(key_mutex_);
      current_key_ = req.key;
    }
    publishState();
  }
}

void VoiceAnnouncerNode::syncBgm()
{
  if (!enable_bgm_) {
    return;
  }
  const bool requested = bgm_requested_.load();
  if (requested == player_.isBgmActive()) {
    return;
  }
  if (requested) {
    if (player_.startBgm(bgm_path_, bgm_fade_in_ms_)) {
      RCLCPP_INFO(get_logger(), "BGM started: %s", bgm_path_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Failed to start BGM: %s", bgm_path_.c_str());
    }
  } else {
    player_.stopBgm(bgm_fade_out_ms_);
  }
}

void VoiceAnnouncerNode::playShutdownCue()
{
  if (!enable_shutdown_audio_ || shutdown_key_.empty()) {
    return;
  }

  // The service is going away: drop the bed and anything still queued so the
  // farewell is the last thing heard.
  player_.stopBgm(0);
  player_.duckBgm(false, 0);
  queue_.clear();

  const std::string path = keyToWavPath(shutdown_key_, default_locale_);
  if (!std::filesystem::exists(path)) {
    RCLCPP_WARN(get_logger(), "Shutdown cue not found: %s", path.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Playing [%s]: %s", shutdown_key_.c_str(), path.c_str());
  if (!player_.playFile(path, true)) {
    RCLCPP_WARN(get_logger(), "Failed to play shutdown cue: %s", path.c_str());
    return;
  }

  const auto deadline = std::chrono::steady_clock::now() + shutdown_timeout_;
  while (player_.isPlaying() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }
}

void VoiceAnnouncerNode::publishState()
{
  if (!state_publishable_.load()) {
    return;
  }

  avg_msgs::msg::VoiceState state;
  state.header.stamp = now();
  const bool playing = player_.isPlaying();
  {
    std::lock_guard<std::mutex> lock(key_mutex_);
    state.current_key = playing ? current_key_ : std::string{};
  }
  state.queue_size   = static_cast<uint8_t>(queue_.size());

  if (playing) {
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
