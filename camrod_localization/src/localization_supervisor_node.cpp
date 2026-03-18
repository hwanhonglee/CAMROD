#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include <avg_msgs/msg/pose_with_covariance_stamped.hpp>
#include <avg_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <avg_msgs/msg/imu.hpp>
#include <avg_msgs/msg/bool.hpp>
#include <avg_msgs/msg/float32.hpp>

#include <avg_msgs/msg/avg_localization_diagnostics.hpp>
#include <avg_msgs/msg/avg_localization_mode.hpp>
#include <avg_msgs/msg/avg_localization_msgs.hpp>
#include <avg_msgs/msg/avg_localization_status.hpp>
#include <avg_msgs/msg/module_health.hpp>

using avg_msgs::msg::AvgLocalizationDiagnostics;
using avg_msgs::msg::AvgLocalizationMode;
using avg_msgs::msg::AvgLocalizationStatus;

class LocalizationSupervisorNode : public rclcpp::Node
{
public:
  // Implements `LocalizationSupervisorNode` behavior.
  LocalizationSupervisorNode()
  // HH_260123 Supervisor publishes mode/confidence based on sensor health + ESKF diagnostics.
  : Node("localization_supervisor")
  {
    diag_topic_ = declare_parameter<std::string>(
      "diag_topic", "/localization/eskf/diagnostic");
    gnss_topic_ = declare_parameter<std::string>(
      "gnss_topic", "/sensing/gnss/pose_with_covariance");
    imu_topic_ = declare_parameter<std::string>(
      "imu_topic", "/sensing/imu/data");
    wheel_topic_ = declare_parameter<std::string>(
      "wheel_topic", "/platform/wheel/odometry");

    gnss_timeout_sec_ = declare_parameter<double>("gnss_timeout_sec", 1.0);
    imu_timeout_sec_ = declare_parameter<double>("imu_timeout_sec", 0.5);
    wheel_timeout_sec_ = declare_parameter<double>("wheel_timeout_sec", 0.5);
    gnss_innov_warn_ = declare_parameter<double>("gnss_innovation_warn", 3.0);
    gnss_innov_fail_ = declare_parameter<double>("gnss_innovation_fail", 6.0);
    gnss_cov_trace_fail_ = declare_parameter<double>("gnss_cov_trace_fail", 0.3);
    gnss_jump_fail_m_ = declare_parameter<double>("gnss_jump_fail_m", 1.0);
    gnss_min_hz_ = declare_parameter<double>("gnss_min_hz", 2.0);
    publish_localization_diagnostic_ = declare_parameter<bool>("publish_localization_diagnostic", false);
    localization_diagnostic_topic_ = declare_parameter<std::string>(
      "localization_diagnostic_topic", "/localization/diagnostic");

    mode_pub_ = create_publisher<AvgLocalizationMode>("/localization/mode", rclcpp::QoS(10));
    status_pub_ = create_publisher<AvgLocalizationStatus>("/localization/status", rclcpp::QoS(10));
    confidence_pub_ = create_publisher<avg_msgs::msg::Float32>(
      "/localization/confidence", rclcpp::QoS(10));
    health_pub_ = create_publisher<avg_msgs::msg::Bool>(
      "/localization/health", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
    if (publish_localization_diagnostic_) {
      avg_localization_pub_ = create_publisher<avg_msgs::msg::AvgLocalizationMsgs>(
        localization_diagnostic_topic_, rclcpp::QoS(10));
    }

    using std::placeholders::_1;
    diag_sub_ = create_subscription<AvgLocalizationDiagnostics>(
      diag_topic_, rclcpp::QoS(20),
      std::bind(&LocalizationSupervisorNode::onDiag, this, _1));
    gnss_sub_ = create_subscription<avg_msgs::msg::PoseWithCovarianceStamped>(
      gnss_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationSupervisorNode::onGnss, this, _1));
    imu_sub_ = create_subscription<avg_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationSupervisorNode::onImu, this, _1));
    wheel_sub_ = create_subscription<avg_msgs::msg::Odometry>(
      wheel_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationSupervisorNode::onWheel, this, _1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LocalizationSupervisorNode::onTimer, this));

    // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
    RCLCPP_DEBUG(get_logger(),
      "Localization supervisor ready (diag=%s)", diag_topic_.c_str());
  }

private:
  // Handles the `onDiag` callback.
  void onDiag(const AvgLocalizationDiagnostics::ConstSharedPtr msg)
  {
    last_diag_ = *msg;
    last_diag_time_ = msg->header.stamp;
  }

  // Handles the `onGnss` callback.
  void onGnss(const avg_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    if (has_prev_gnss_) {
      const double dx = msg->pose.pose.position.x - last_gnss_x_;
      const double dy = msg->pose.pose.position.y - last_gnss_y_;
      last_gnss_jump_m_ = std::sqrt(dx * dx + dy * dy);
      const double dt = (rclcpp::Time(msg->header.stamp) - prev_gnss_time_).seconds();
      if (dt > 1e-3) {
        last_gnss_hz_ = 1.0 / dt;
      }
    }
    last_gnss_x_ = msg->pose.pose.position.x;
    last_gnss_y_ = msg->pose.pose.position.y;
    prev_gnss_time_ = rclcpp::Time(msg->header.stamp);
    has_prev_gnss_ = true;

    last_gnss_time_ = msg->header.stamp;
    last_gnss_cov_trace_ = msg->pose.covariance[0] + msg->pose.covariance[7];
  }

  // Handles the `onImu` callback.
  void onImu(const avg_msgs::msg::Imu::ConstSharedPtr msg)
  {
    last_imu_time_ = msg->header.stamp;
  }

  // Handles the `onWheel` callback.
  void onWheel(const avg_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    last_wheel_time_ = msg->header.stamp;
  }

  // Implements `makeMode` behavior.
  AvgLocalizationMode makeMode(uint8_t value, const std::string & label)
  {
    AvgLocalizationMode m;
    m.value = value;
    m.label = label;
    return m;
  }

  // Handles the `onTimer` callback.
  void onTimer()
  {
    const rclcpp::Time now = this->now();
    const bool imu_ok = (now - last_imu_time_).seconds() <= imu_timeout_sec_;
    const bool gnss_fresh = (now - last_gnss_time_).seconds() <= gnss_timeout_sec_;
    const bool wheel_ok = (now - last_wheel_time_).seconds() <= wheel_timeout_sec_;

    const bool gnss_cov_ok =
      gnss_cov_trace_fail_ <= 0.0 || last_gnss_cov_trace_ <= gnss_cov_trace_fail_;
    const bool gnss_jump_ok =
      !has_prev_gnss_ || gnss_jump_fail_m_ <= 0.0 || last_gnss_jump_m_ <= gnss_jump_fail_m_;
    const bool gnss_rate_ok =
      !has_prev_gnss_ || gnss_min_hz_ <= 0.0 || last_gnss_hz_ >= gnss_min_hz_;
    const bool gnss_good =
      gnss_fresh && last_diag_.gnss_update_accepted && gnss_cov_ok && gnss_jump_ok && gnss_rate_ok;
    const bool wheel_good = wheel_ok &&
      (last_diag_time_.nanoseconds() > 0 ? last_diag_.wheel_update_accepted : true);

    double confidence = 1.0;
    if (!gnss_good) confidence -= 0.35;
    if (!wheel_good) confidence -= 0.15;
    if (!imu_ok) confidence -= 0.4;
    if (last_diag_.gnss_innovation_norm > gnss_innov_warn_) confidence -= 0.2;
    if (!gnss_cov_ok) confidence -= 0.15;
    if (!gnss_jump_ok) confidence -= 0.15;
    if (!gnss_rate_ok) confidence -= 0.1;
    confidence = std::clamp(confidence, 0.0, 1.0);

    AvgLocalizationMode mode_msg;
    if (!imu_ok) {
      mode_msg = makeMode(AvgLocalizationMode::INVALID, "INVALID");
    } else if (!gnss_good && wheel_ok) {
      mode_msg = makeMode(AvgLocalizationMode::DR_ONLY, "DR_ONLY");
    } else if (gnss_good && wheel_good &&
      last_diag_.gnss_innovation_norm <= gnss_innov_warn_) {
      mode_msg = makeMode(AvgLocalizationMode::NORMAL, "NORMAL");
    } else if (gnss_good || wheel_ok) {
      mode_msg = makeMode(AvgLocalizationMode::DEGRADED, "DEGRADED");
    } else {
      mode_msg = makeMode(AvgLocalizationMode::INVALID, "INVALID");
    }

    // If innovation too high, flag degraded even if GNSS fresh.
    if (last_diag_.gnss_innovation_norm > gnss_innov_fail_) {
      mode_msg = makeMode(AvgLocalizationMode::DEGRADED, "DEGRADED");
    }

    AvgLocalizationStatus status;
    status.header.stamp = now;
    status.mode = mode_msg;
    status.confidence = static_cast<float>(confidence);
    status.gnss_ok = gnss_good;
    status.imu_ok = imu_ok;
    status.wheel_ok = wheel_good;
    status.gnss_innovation_norm = last_diag_.gnss_innovation_norm;
    status.wheel_innovation_norm = last_diag_.wheel_innovation_norm;

    avg_msgs::msg::Bool health_msg;
    health_msg.data = (mode_msg.value == AvgLocalizationMode::NORMAL) ||
      (mode_msg.value == AvgLocalizationMode::DEGRADED);

    avg_msgs::msg::Float32 conf_msg;
    conf_msg.data = static_cast<float>(confidence);

    mode_pub_->publish(mode_msg);
    status_pub_->publish(status);
    confidence_pub_->publish(conf_msg);
    health_pub_->publish(health_msg);

    if (publish_localization_diagnostic_ && avg_localization_pub_) {
      avg_msgs::msg::AvgLocalizationMsgs avg_msg;
      avg_msg.stamp = status.header.stamp;
      avg_msg.localization_mode = mode_msg;
      avg_msg.localization_status = status;
      avg_msg.localization_diagnostics = last_diag_;
      avg_msg.gnss_update_accepted = last_diag_.gnss_update_accepted;
      avg_msg.gnss_innovation_norm = last_diag_.gnss_innovation_norm;
      avg_msg.wheel_update_accepted = last_diag_.wheel_update_accepted;
      avg_msg.wheel_innovation_norm = last_diag_.wheel_innovation_norm;
      avg_msg.covariance_trace = last_diag_.covariance_trace;
      avg_msg.health.stamp = status.header.stamp;
      avg_msg.health.module_name = "localization";
      if (mode_msg.value == AvgLocalizationMode::NORMAL) {
        avg_msg.health.level = avg_msgs::msg::ModuleHealth::OK;
      } else if (mode_msg.value == AvgLocalizationMode::DEGRADED ||
        mode_msg.value == AvgLocalizationMode::DR_ONLY)
      {
        avg_msg.health.level = avg_msgs::msg::ModuleHealth::WARN;
      } else {
        avg_msg.health.level = avg_msgs::msg::ModuleHealth::ERROR;
      }
      avg_msg.health.message = "localization_supervisor";
      avg_localization_pub_->publish(avg_msg);
    }
  }

  // Parameters
  std::string diag_topic_;
  std::string gnss_topic_;
  std::string imu_topic_;
  std::string wheel_topic_;
  double gnss_timeout_sec_{1.0};
  double imu_timeout_sec_{0.5};
  double wheel_timeout_sec_{0.5};
  double gnss_innov_warn_{3.0};
  double gnss_innov_fail_{6.0};
  double gnss_cov_trace_fail_{0.3};
  double gnss_jump_fail_m_{1.0};
  double gnss_min_hz_{2.0};
  bool publish_localization_diagnostic_{false};
  std::string localization_diagnostic_topic_{"/localization/diagnostic"};

  // State
  AvgLocalizationDiagnostics last_diag_{};
  rclcpp::Time last_diag_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gnss_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wheel_time_{0, 0, RCL_ROS_TIME};
  double last_gnss_cov_trace_{0.0};
  bool has_prev_gnss_{false};
  double last_gnss_x_{0.0};
  double last_gnss_y_{0.0};
  rclcpp::Time prev_gnss_time_{0, 0, RCL_ROS_TIME};
  double last_gnss_jump_m_{0.0};
  double last_gnss_hz_{0.0};

  // ROS interfaces
  rclcpp::Subscription<AvgLocalizationDiagnostics>::SharedPtr diag_sub_;
  rclcpp::Subscription<avg_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_sub_;
  rclcpp::Subscription<avg_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<avg_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Publisher<AvgLocalizationMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<AvgLocalizationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<avg_msgs::msg::Float32>::SharedPtr confidence_pub_;
  rclcpp::Publisher<avg_msgs::msg::Bool>::SharedPtr health_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationMsgs>::SharedPtr avg_localization_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Entry point for this executable.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationSupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
