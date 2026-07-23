#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

// HH_260720 - Use generated CAMROD interfaces for every internal localization contract.
#include <avg_msgs/msg/avg_pose_stamped.hpp>
#include <avg_msgs/msg/avg_pose_with_covariance_stamped.hpp>
#include <avg_msgs/msg/avg_odometry.hpp>
#include <avg_msgs/msg/avg_imu.hpp>
#include <avg_msgs/msg/avg_bool.hpp>
#include <avg_msgs/msg/avg_float32.hpp>

#include <avg_msgs/msg/avg_localization_status_stream.hpp>
#include <avg_msgs/msg/avg_localization_mode.hpp>
#include <avg_msgs/msg/avg_localization_msgs.hpp>
#include <avg_msgs/msg/avg_localization_status.hpp>
#include <avg_msgs/msg/module_state.hpp>

namespace camrod::localization
{
namespace
{
std::string normalizeModeToken(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}
}  // namespace

class LocalizationMonitorNode : public rclcpp::Node
{
public:
  LocalizationMonitorNode()
  : Node("localization_monitor")
  {
    // HH_260720 - Use a filter-neutral status contract; EKF operation keeps this input disabled.
    filter_status_topic_ = declare_parameter<std::string>(
      "filter_status_topic", "/localization/filter/status");
    // HH_260526: Replace use_filter_status toggle with explicit mode.
    // filter_status_mode options: stream | none.
    filter_status_mode_ = normalizeModeToken(
      declare_parameter<std::string>("filter_status_mode", "stream"));

    gnss_pose_topic_ = declare_parameter<std::string>("gnss_pose_topic", "/sensing/gnss/pose");
    gnss_pose_cov_topic_ = declare_parameter<std::string>(
      "gnss_pose_cov_topic", "/sensing/gnss/pose_with_covariance");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/sensing/imu/data");
    // HH_260720 - Monitor the canonical generated wheel topic used by localization.
    wheel_topic_ = declare_parameter<std::string>(
      "wheel_topic",
      "/localization/input/wheel_odometry");

    gnss_timeout_s_ = declare_parameter<double>("gnss_timeout_s", 1.0);
    imu_timeout_s_ = declare_parameter<double>("imu_timeout_s", 0.5);
    wheel_timeout_s_ = declare_parameter<double>("wheel_timeout_s", 0.5);

    gnss_innov_warn_ = declare_parameter<double>("gnss_innovation_warn", 3.0);
    gnss_innov_fail_ = declare_parameter<double>("gnss_innovation_fail", 6.0);
    gnss_cov_trace_fail_ = declare_parameter<double>("gnss_cov_trace_fail", 0.3);
    gnss_jump_fail_m_ = declare_parameter<double>("gnss_jump_fail_m", 1.0);
    gnss_min_hz_ = declare_parameter<double>("gnss_min_hz", 2.0);

    // HH_260507: DR timeout — stop robot if DR continues too long or covariance grows too large.
    // 0 disables each check independently.
    dr_max_duration_s_ = declare_parameter<double>("dr_max_duration_s", 0.0);
    dr_max_cov_trace_ = declare_parameter<double>("dr_max_cov_trace", 0.0);
    // HH_260626: Publish worse modes immediately, but require GNSS recovery to
    // remain stable briefly before returning to NORMAL. This avoids repeated
    // DR_ONLY->NORMAL flapping from one late GNSS sample.
    mode_degrade_debounce_s_ = declare_parameter<double>("mode_degrade_debounce_s", 0.0);
    mode_recovery_debounce_s_ = declare_parameter<double>("mode_recovery_debounce_s", 1.5);

    publish_localization_status_ =
      declare_parameter<bool>("publish_localization_status", false);
    localization_status_topic_ =
      declare_parameter<std::string>("localization_status_topic", "/localization/status");
    if (filter_status_mode_ != "stream" && filter_status_mode_ != "none") {
      RCLCPP_WARN(
        get_logger(),
        "Invalid filter_status_mode='%s'. Falling back to 'stream'.",
        filter_status_mode_.c_str());
      filter_status_mode_ = "stream";
    }

    mode_pub_ = create_publisher<avg_msgs::msg::AvgLocalizationMode>(
      "/localization/mode", rclcpp::QoS(
        10));
    status_pub_ = create_publisher<avg_msgs::msg::AvgLocalizationStatus>(
      "/localization/status", rclcpp::QoS(
        10));
    // HH_260720 - Publish generated CAMROD scalar messages instead of namespace aliases.
    confidence_pub_ = create_publisher<avg_msgs::msg::AvgFloat32>(
      "/localization/confidence", rclcpp::QoS(
        10));
    state_pub_ = create_publisher<avg_msgs::msg::AvgBool>(
      "/localization/state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
    degraded_pub_ = create_publisher<avg_msgs::msg::AvgBool>(
      "/localization/state/degraded", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
    // HH_260507: DR timeout publisher — true when DR exceeds time or covariance limit.
    dr_timeout_pub_ = create_publisher<avg_msgs::msg::AvgBool>(
      "/localization/state/dr_timeout", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

    if (publish_localization_status_) {
      avg_localization_pub_ = create_publisher<avg_msgs::msg::AvgLocalizationMsgs>(
        localization_status_topic_, rclcpp::QoS(10));
    }

    using std::placeholders::_1;

    if (filter_status_mode_ == "stream" && !filter_status_topic_.empty()) {
      diag_sub_ = create_subscription<avg_msgs::msg::AvgLocalizationStatusStream>(
        filter_status_topic_, rclcpp::QoS(20),
        std::bind(&LocalizationMonitorNode::onDiag, this, _1));
    }

    gnss_pose_sub_ = create_subscription<avg_msgs::msg::AvgPoseStamped>(
      gnss_pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationMonitorNode::onGnssPose, this, _1));

    gnss_pose_cov_sub_ = create_subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>(
      gnss_pose_cov_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationMonitorNode::onGnssPoseCov, this, _1));

    imu_sub_ = create_subscription<avg_msgs::msg::AvgImu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationMonitorNode::onImu, this, _1));

    wheel_sub_ = create_subscription<avg_msgs::msg::AvgOdometry>(
      wheel_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationMonitorNode::onWheel, this, _1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LocalizationMonitorNode::onTimer, this));
  }

private:
  void onGnssPose(const avg_msgs::msg::AvgPoseStamped::ConstSharedPtr msg)
  {
    last_gnss_pose_time_ = rclcpp::Time(msg->header.stamp);

    const rclcpp::Time stamp(msg->header.stamp);
    if (has_prev_gnss_) {
      const double dx = msg->pose.position.x - last_gnss_x_;
      const double dy = msg->pose.position.y - last_gnss_y_;
      last_gnss_jump_m_ = std::sqrt(dx * dx + dy * dy);
      const double dt = (stamp - prev_gnss_time_).seconds();
      if (dt > 1e-3) {
        last_gnss_hz_ = 1.0 / dt;
      }
    }

    last_gnss_x_ = msg->pose.position.x;
    last_gnss_y_ = msg->pose.position.y;
    prev_gnss_time_ = stamp;
    has_prev_gnss_ = true;
  }

  void onGnssPoseCov(const avg_msgs::msg::AvgPoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    last_gnss_cov_time_ = rclcpp::Time(msg->header.stamp);
    last_gnss_cov_trace_ = msg->pose.covariance[0] + msg->pose.covariance[7];

    const rclcpp::Time stamp(msg->header.stamp);
    if (has_prev_gnss_) {
      const double dx = msg->pose.pose.position.x - last_gnss_x_;
      const double dy = msg->pose.pose.position.y - last_gnss_y_;
      last_gnss_jump_m_ = std::sqrt(dx * dx + dy * dy);
      const double dt = (stamp - prev_gnss_time_).seconds();
      if (dt > 1e-3) {
        last_gnss_hz_ = 1.0 / dt;
      }
    }

    last_gnss_x_ = msg->pose.pose.position.x;
    last_gnss_y_ = msg->pose.pose.position.y;
    prev_gnss_time_ = stamp;
    has_prev_gnss_ = true;
  }

  void onImu(const avg_msgs::msg::AvgImu::ConstSharedPtr msg)
  {
    last_imu_time_ = rclcpp::Time(msg->header.stamp);
  }

  void onWheel(const avg_msgs::msg::AvgOdometry::ConstSharedPtr msg)
  {
    last_wheel_time_ = rclcpp::Time(msg->header.stamp);
  }

  void onDiag(const avg_msgs::msg::AvgLocalizationStatusStream::ConstSharedPtr msg)
  {
    last_diag_ = *msg;
    last_diag_time_ = rclcpp::Time(msg->header.stamp);
  }

  avg_msgs::msg::AvgLocalizationMode makeMode(uint8_t value, const std::string & label)
  {
    avg_msgs::msg::AvgLocalizationMode m;
    m.value = value;
    m.label = label;
    return m;
  }

  void onTimer()
  {
    const rclcpp::Time now = this->now();

    const bool imu_ok = (now - last_imu_time_).seconds() <= imu_timeout_s_;
    const bool gnss_pose_fresh = (now - last_gnss_pose_time_).seconds() <= gnss_timeout_s_;
    const bool gnss_cov_fresh = (now - last_gnss_cov_time_).seconds() <= gnss_timeout_s_;
    const bool gnss_fresh = gnss_pose_fresh || gnss_cov_fresh;
    const bool wheel_ok = (now - last_wheel_time_).seconds() <= wheel_timeout_s_;

    const bool gnss_cov_ok =
      (gnss_cov_trace_fail_ <= 0.0) || (last_gnss_cov_trace_ <= gnss_cov_trace_fail_);
    const bool gnss_jump_ok =
      !has_prev_gnss_ || (gnss_jump_fail_m_ <= 0.0) || (last_gnss_jump_m_ <= gnss_jump_fail_m_);
    const bool gnss_rate_ok =
      !has_prev_gnss_ || (gnss_min_hz_ <= 0.0) || (last_gnss_hz_ >= gnss_min_hz_);

    const bool diag_available =
      (filter_status_mode_ == "stream") && (last_diag_time_.nanoseconds() > 0);

    const bool gnss_update_accepted =
      !diag_available || last_diag_.gnss_update_accepted;
    const bool wheel_update_accepted =
      !diag_available || last_diag_.wheel_update_accepted;
    const bool gnss_innov_warn_bad =
      diag_available && (last_diag_.gnss_innovation_norm > gnss_innov_warn_);
    const bool gnss_innov_fail_bad =
      diag_available && (last_diag_.gnss_innovation_norm > gnss_innov_fail_);

    const bool gnss_good =
      gnss_fresh && gnss_update_accepted && gnss_cov_ok && gnss_jump_ok && gnss_rate_ok;
    const bool wheel_good =
      wheel_ok && wheel_update_accepted;

    double confidence = 1.0;
    if (!gnss_good) {confidence -= 0.35;}
    if (!wheel_good) {confidence -= 0.15;}
    if (!imu_ok) {confidence -= 0.40;}
    if (gnss_innov_warn_bad) {confidence -= 0.20;}
    if (!gnss_cov_ok) {confidence -= 0.15;}
    if (!gnss_jump_ok) {confidence -= 0.15;}
    if (!gnss_rate_ok) {confidence -= 0.10;}
    confidence = std::clamp(confidence, 0.0, 1.0);

    avg_msgs::msg::AvgLocalizationMode mode;
    std::string status_label = "NORMAL";
    const bool in_dr = !gnss_good && wheel_good;

    if (!imu_ok) {
      mode = makeMode(avg_msgs::msg::AvgLocalizationMode().INVALID, "INVALID");
      status_label = "IMU_TIMEOUT";
    } else if (in_dr) {
      mode = makeMode(avg_msgs::msg::AvgLocalizationMode().DR_ONLY, "DR_ONLY");
      status_label = "DEAD_RECKONING";
    } else if (!gnss_good && !wheel_good) {
      mode = makeMode(avg_msgs::msg::AvgLocalizationMode().INVALID, "INVALID");
      status_label = "GNSS_AND_WHEEL_LOST";
    } else if (gnss_innov_warn_bad) {
      mode = makeMode(avg_msgs::msg::AvgLocalizationMode().DEGRADED, "DEGRADED");
      status_label = "HIGH_GNSS_INNOVATION";
    } else {
      mode = makeMode(avg_msgs::msg::AvgLocalizationMode().NORMAL, "NORMAL");
      status_label = "OK";
    }

    mode = applyModeDebounce(mode, status_label, now);

    // HH_260507: DR duration tracking and timeout detection.
    if (in_dr) {
      if (dr_start_time_.nanoseconds() == 0) {
        dr_start_time_ = now;
      }
    } else {
      dr_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      dr_timeout_ = false;
    }
    const double cov_trace =
      diag_available ? last_diag_.covariance_trace : 0.0;
    if (in_dr && !dr_timeout_) {
      const double dr_elapsed =
        (dr_start_time_.nanoseconds() > 0) ? (now - dr_start_time_).seconds() : 0.0;
      const bool time_exceeded =
        (dr_max_duration_s_ > 0.0) && (dr_elapsed >= dr_max_duration_s_);
      const bool cov_exceeded =
        (dr_max_cov_trace_ > 0.0) && (cov_trace >= dr_max_cov_trace_);
      if (time_exceeded || cov_exceeded) {
        dr_timeout_ = true;
        RCLCPP_WARN(
          get_logger(),
          "DR timeout: elapsed=%.1fs cov_trace=%.2f (time_limit=%.1fs cov_limit=%.2f)",
          dr_elapsed, cov_trace, dr_max_duration_s_, dr_max_cov_trace_);
      }
    }
    avg_msgs::msg::AvgBool dr_timeout_msg;
    dr_timeout_msg.data = dr_timeout_;
    dr_timeout_pub_->publish(dr_timeout_msg);

    avg_msgs::msg::AvgLocalizationStatus status;
    status.header.stamp = now;
    status.header.frame_id = "map";
    status.mode = mode;
    status.confidence = static_cast<float>(confidence);
    // HH_260723 - Publish the health values used to compute mode/confidence.
    // Leaving these fields default-initialized made diagnostics report GNSS,
    // IMU, and wheel inputs missing even while the monitor was in NORMAL mode.
    status.gnss_ok = gnss_good;
    status.imu_ok = imu_ok;
    status.wheel_ok = wheel_good;
    status.gnss_innovation_norm = diag_available ?
      last_diag_.gnss_innovation_norm : 0.0F;
    status.wheel_innovation_norm = diag_available ?
      last_diag_.wheel_innovation_norm : 0.0F;

    avg_msgs::msg::AvgFloat32 confidence_msg;
    confidence_msg.data = static_cast<float>(confidence);

    avg_msgs::msg::AvgBool state_msg;
    state_msg.data = imu_ok && (gnss_good || wheel_good);

    avg_msgs::msg::AvgBool degraded_msg;
    degraded_msg.data = (mode.value >= avg_msgs::msg::AvgLocalizationMode().DEGRADED);

    mode_pub_->publish(mode);
    status_pub_->publish(status);
    confidence_pub_->publish(confidence_msg);
    state_pub_->publish(state_msg);
    degraded_pub_->publish(degraded_msg);

    if (publish_localization_status_ && avg_localization_pub_) {
      avg_msgs::msg::AvgLocalizationMsgs out;
      out.stamp = now;
      out.state.stamp = now;
      out.state.module_name = "localization";
      out.state.level = state_msg.data ?
        avg_msgs::msg::ModuleState::OK :
        avg_msgs::msg::ModuleState::WARN;
      out.state.message = status_label;
      avg_localization_pub_->publish(out);
    }
  }

  int modeSeverity(uint8_t value) const
  {
    const avg_msgs::msg::AvgLocalizationMode ref;
    if (value == ref.NORMAL) {
      return 0;
    }
    if (value == ref.DEGRADED) {
      return 1;
    }
    if (value == ref.DR_ONLY) {
      return 2;
    }
    if (value == ref.INVALID) {
      return 3;
    }
    return static_cast<int>(value);
  }

  avg_msgs::msg::AvgLocalizationMode applyModeDebounce(
    const avg_msgs::msg::AvgLocalizationMode & raw_mode,
    std::string & status_label,
    const rclcpp::Time & now)
  {
    if (!has_published_mode_) {
      acceptPublishedMode(raw_mode, status_label);
      return raw_mode;
    }

    if (raw_mode.value == published_mode_value_) {
      pending_mode_active_ = false;
      acceptPublishedMode(raw_mode, status_label);
      return raw_mode;
    }

    const bool recovering =
      modeSeverity(raw_mode.value) < modeSeverity(published_mode_value_);
    const double debounce_s =
      recovering ? mode_recovery_debounce_s_ : mode_degrade_debounce_s_;

    if (!pending_mode_active_ || pending_mode_value_ != raw_mode.value) {
      pending_mode_active_ = true;
      pending_mode_value_ = raw_mode.value;
      pending_mode_label_ = raw_mode.label;
      pending_status_label_ = status_label;
      pending_mode_since_ = now;
    }

    const double elapsed_s = (now - pending_mode_since_).seconds();
    if (debounce_s <= 0.0 || elapsed_s >= debounce_s) {
      acceptPublishedMode(raw_mode, status_label);
      pending_mode_active_ = false;
      return raw_mode;
    }

    avg_msgs::msg::AvgLocalizationMode held;
    held.value = published_mode_value_;
    held.label = published_mode_label_;
    status_label = published_status_label_;
    return held;
  }

  void acceptPublishedMode(
    const avg_msgs::msg::AvgLocalizationMode & mode,
    const std::string & status_label)
  {
    has_published_mode_ = true;
    published_mode_value_ = mode.value;
    published_mode_label_ = mode.label;
    published_status_label_ = status_label;
  }

  std::string filter_status_topic_;
  // HH_260720 - Stream mode factors active-filter acceptance flags into sensor health;
  // none mode decides health from sensor freshness and covariance only.
  std::string filter_status_mode_{"stream"};

  std::string gnss_pose_topic_;
  std::string gnss_pose_cov_topic_;
  std::string imu_topic_;
  std::string wheel_topic_;

  double gnss_timeout_s_{1.0};
  double imu_timeout_s_{0.5};
  double wheel_timeout_s_{0.5};

  double gnss_innov_warn_{3.0};
  double gnss_innov_fail_{6.0};
  double gnss_cov_trace_fail_{0.3};
  double gnss_jump_fail_m_{1.0};
  double gnss_min_hz_{2.0};

  // HH_260507: DR timeout — 0 = disabled.
  double dr_max_duration_s_{0.0};
  double dr_max_cov_trace_{0.0};
  double mode_degrade_debounce_s_{0.0};
  double mode_recovery_debounce_s_{1.5};
  rclcpp::Time dr_start_time_{0, 0, RCL_ROS_TIME};
  bool dr_timeout_{false};

  bool has_published_mode_{false};
  uint8_t published_mode_value_{0};
  std::string published_mode_label_{"NORMAL"};
  std::string published_status_label_{"OK"};
  bool pending_mode_active_{false};
  uint8_t pending_mode_value_{0};
  std::string pending_mode_label_;
  std::string pending_status_label_;
  rclcpp::Time pending_mode_since_{0, 0, RCL_ROS_TIME};

  // HH_260422: true -> also publish /localization/status (AvgLocalizationMsgs).
  bool publish_localization_status_{false};
  std::string localization_status_topic_;

  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgFloat32>::SharedPtr confidence_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgBool>::SharedPtr state_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgBool>::SharedPtr degraded_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgBool>::SharedPtr dr_timeout_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationMsgs>::SharedPtr avg_localization_pub_;

  rclcpp::Subscription<avg_msgs::msg::AvgLocalizationStatusStream>::SharedPtr diag_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseStamped>::SharedPtr gnss_pose_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>::SharedPtr gnss_pose_cov_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgImu>::SharedPtr imu_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgOdometry>::SharedPtr wheel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_gnss_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gnss_cov_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wheel_time_{0, 0, RCL_ROS_TIME};

  // HH_260422: has_prev_gnss_ becomes true after the first GNSS pose message is received.
  //   While false: jump distance and rate calculations are skipped (no previous position to compare against).
  //   Once true: last_gnss_jump_m_ and last_gnss_hz_ are updated and feed into gnss_jump_ok / gnss_rate_ok.
  bool has_prev_gnss_{false};
  double last_gnss_x_{0.0};
  double last_gnss_y_{0.0};
  double last_gnss_jump_m_{0.0};
  double last_gnss_cov_trace_{0.0};
  double last_gnss_hz_{0.0};
  rclcpp::Time prev_gnss_time_{0, 0, RCL_ROS_TIME};

  avg_msgs::msg::AvgLocalizationStatusStream last_diag_;
  rclcpp::Time last_diag_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace camrod::localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod::localization::LocalizationMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
