#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <avg_msgs/msg/pose_stamped.hpp>
#include <avg_msgs/msg/pose_with_covariance_stamped.hpp>
#include <avg_msgs/msg/odometry.hpp>
#include <avg_msgs/msg/imu.hpp>
#include <avg_msgs/msg/bool.hpp>
#include <avg_msgs/msg/float32.hpp>

#include <avg_msgs/msg/avg_localization_status_stream.hpp>
#include <avg_msgs/msg/avg_localization_mode.hpp>
#include <avg_msgs/msg/avg_localization_msgs.hpp>
#include <avg_msgs/msg/avg_localization_status.hpp>
#include <avg_msgs/msg/module_state.hpp>

namespace camrod::localization
{

class LocalizationMonitorNode : public rclcpp::Node
{
public:
  LocalizationMonitorNode()
  : Node("localization_monitor")
  {
    diag_topic_ = declare_parameter<std::string>("diag_topic", "/localization/eskf/status");
    use_filter_status_ = declare_parameter<bool>("use_filter_status", true);

    gnss_pose_topic_ = declare_parameter<std::string>("gnss_pose_topic", "/sensing/gnss/pose");
    gnss_pose_cov_topic_ = declare_parameter<std::string>(
      "gnss_pose_cov_topic", "/sensing/gnss/pose_with_covariance");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/sensing/imu/data");
    // HH_260410: Monitor uses the same unified wheel topic as EKF/ESKF.
    wheel_topic_ = declare_parameter<std::string>("wheel_topic", "/platform/status/wheel_odometry");

    gnss_timeout_s_ = declareDurationWithLegacy("gnss_timeout_s", "gnss_timeout_sec", 1.0);
    imu_timeout_s_ = declareDurationWithLegacy("imu_timeout_s", "imu_timeout_sec", 0.5);
    wheel_timeout_s_ = declareDurationWithLegacy("wheel_timeout_s", "wheel_timeout_sec", 0.5);

    gnss_innov_warn_ = declare_parameter<double>("gnss_innovation_warn", 3.0);
    gnss_innov_fail_ = declare_parameter<double>("gnss_innovation_fail", 6.0);
    gnss_cov_trace_fail_ = declare_parameter<double>("gnss_cov_trace_fail", 0.3);
    gnss_jump_fail_m_ = declare_parameter<double>("gnss_jump_fail_m", 1.0);
    gnss_min_hz_ = declare_parameter<double>("gnss_min_hz", 2.0);

    // HH_260507: DR timeout — stop robot if DR continues too long or covariance grows too large.
    // 0 disables each check independently.
    dr_max_duration_s_ = declare_parameter<double>("dr_max_duration_s", 0.0);
    dr_max_cov_trace_ = declare_parameter<double>("dr_max_cov_trace", 0.0);

    publish_localization_status_ =
      declare_parameter<bool>("publish_localization_status", false);
    localization_status_topic_ =
      declare_parameter<std::string>("localization_status_topic", "/localization/status");

    mode_pub_ = create_publisher<avg_msgs::msg::AvgLocalizationMode>("/localization/mode", rclcpp::QoS(10));
    status_pub_ = create_publisher<avg_msgs::msg::AvgLocalizationStatus>("/localization/status", rclcpp::QoS(10));
    confidence_pub_ = create_publisher<avg_msgs::msg::Float32>("/localization/confidence", rclcpp::QoS(10));
    state_pub_ = create_publisher<avg_msgs::msg::Bool>(
      "/localization/state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
    degraded_pub_ = create_publisher<avg_msgs::msg::Bool>(
      "/localization/state/degraded", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
    // HH_260507: DR timeout publisher — true when DR exceeds time or covariance limit.
    dr_timeout_pub_ = create_publisher<avg_msgs::msg::Bool>(
      "/localization/state/dr_timeout", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

    if (publish_localization_status_) {
      avg_localization_pub_ = create_publisher<avg_msgs::msg::AvgLocalizationMsgs>(
        localization_status_topic_, rclcpp::QoS(10));
    }

    using std::placeholders::_1;

    if (use_filter_status_ && !diag_topic_.empty()) {
      diag_sub_ = create_subscription<avg_msgs::msg::AvgLocalizationStatusStream>(
        diag_topic_, rclcpp::QoS(20),
        std::bind(&LocalizationMonitorNode::onDiag, this, _1));
    }

    gnss_pose_sub_ = create_subscription<avg_msgs::msg::PoseStamped>(
      gnss_pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationMonitorNode::onGnssPose, this, _1));

    gnss_pose_cov_sub_ = create_subscription<avg_msgs::msg::PoseWithCovarianceStamped>(
      gnss_pose_cov_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationMonitorNode::onGnssPoseCov, this, _1));

    imu_sub_ = create_subscription<avg_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationMonitorNode::onImu, this, _1));

    wheel_sub_ = create_subscription<avg_msgs::msg::Odometry>(
      wheel_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationMonitorNode::onWheel, this, _1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LocalizationMonitorNode::onTimer, this));
  }

private:
  double declareDurationWithLegacy(
    const std::string & canonical_name,
    const std::string & legacy_name,
    const double default_value)
  {
    const double canonical_value = declare_parameter<double>(canonical_name, default_value);
    const double legacy_value = declare_parameter<double>(legacy_name, default_value);
    if (std::abs(canonical_value - default_value) > 1e-9) {
      return canonical_value;
    }
    if (std::abs(legacy_value - default_value) > 1e-9) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter '%s' is deprecated. Use '%s' instead.",
        legacy_name.c_str(), canonical_name.c_str());
      return legacy_value;
    }
    return canonical_value;
  }

  void onGnssPose(const avg_msgs::msg::PoseStamped::ConstSharedPtr msg)
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

  void onGnssPoseCov(const avg_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
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

  void onImu(const avg_msgs::msg::Imu::ConstSharedPtr msg)
  {
    last_imu_time_ = rclcpp::Time(msg->header.stamp);
  }

  void onWheel(const avg_msgs::msg::Odometry::ConstSharedPtr msg)
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
      use_filter_status_ && (last_diag_time_.nanoseconds() > 0);

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
    if (!gnss_good) confidence -= 0.35;
    if (!wheel_good) confidence -= 0.15;
    if (!imu_ok) confidence -= 0.40;
    if (gnss_innov_warn_bad) confidence -= 0.20;
    if (!gnss_cov_ok) confidence -= 0.15;
    if (!gnss_jump_ok) confidence -= 0.15;
    if (!gnss_rate_ok) confidence -= 0.10;
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
    avg_msgs::msg::Bool dr_timeout_msg;
    dr_timeout_msg.data = dr_timeout_;
    dr_timeout_pub_->publish(dr_timeout_msg);

    avg_msgs::msg::AvgLocalizationStatus status;
    status.mode = mode;
    status.confidence = confidence;

    avg_msgs::msg::Float32 confidence_msg;
    confidence_msg.data = static_cast<float>(confidence);

    avg_msgs::msg::Bool state_msg;
    state_msg.data = imu_ok && (gnss_good || wheel_good);

    avg_msgs::msg::Bool degraded_msg;
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

  std::string diag_topic_;
  // HH_260422: true -> subscribe to diag_topic_ and factor gnss_update_accepted / wheel_update_accepted
  //   into the gnss_good / wheel_good decision.
  //   false -> decide mode from sensor freshness and covariance only (ignores ESKF internal acceptance state).
  bool use_filter_status_{true};

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
  rclcpp::Time dr_start_time_{0, 0, RCL_ROS_TIME};
  bool dr_timeout_{false};

  bool publish_localization_status_{false}; // HH_260422: true -> also publish /localization/status (AvgLocalizationMsgs)
  std::string localization_status_topic_;

  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<avg_msgs::msg::Float32>::SharedPtr confidence_pub_;
  rclcpp::Publisher<avg_msgs::msg::Bool>::SharedPtr state_pub_;
  rclcpp::Publisher<avg_msgs::msg::Bool>::SharedPtr degraded_pub_;
  rclcpp::Publisher<avg_msgs::msg::Bool>::SharedPtr dr_timeout_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationMsgs>::SharedPtr avg_localization_pub_;

  rclcpp::Subscription<avg_msgs::msg::AvgLocalizationStatusStream>::SharedPtr diag_sub_;
  rclcpp::Subscription<avg_msgs::msg::PoseStamped>::SharedPtr gnss_pose_sub_;
  rclcpp::Subscription<avg_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_cov_sub_;
  rclcpp::Subscription<avg_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<avg_msgs::msg::Odometry>::SharedPtr wheel_sub_;
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
