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

#include <avg_msgs/msg/avg_localization_status_stream.hpp>
#include <avg_msgs/msg/avg_localization_mode.hpp>
#include <avg_msgs/msg/avg_localization_msgs.hpp>
#include <avg_msgs/msg/avg_localization_status.hpp>
#include <avg_msgs/msg/module_state.hpp>

using avg_msgs::msg::AvgLocalizationStatusStream;
using avg_msgs::msg::AvgLocalizationMode;
using avg_msgs::msg::AvgLocalizationStatus;

class LocalizationSupervisorNode : public rclcpp::Node
{
public:
  LocalizationSupervisorNode()
  : Node("localization_supervisor")
  {
    diag_topic_ = declare_parameter<std::string>("diag_topic", "/localization/eskf/status");
    use_filter_status_ = declare_parameter<bool>("use_filter_status", true);

    gnss_topic_ = declare_parameter<std::string>("gnss_topic", "/sensing/gnss/pose_with_covariance");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/sensing/imu/data");
    wheel_topic_ = declare_parameter<std::string>("wheel_topic", "/platform/wheel/odometry");

    gnss_timeout_sec_ = declare_parameter<double>("gnss_timeout_sec", 1.0);
    imu_timeout_sec_ = declare_parameter<double>("imu_timeout_sec", 0.5);
    wheel_timeout_sec_ = declare_parameter<double>("wheel_timeout_sec", 0.5);

    gnss_innov_warn_ = declare_parameter<double>("gnss_innovation_warn", 3.0);
    gnss_innov_fail_ = declare_parameter<double>("gnss_innovation_fail", 6.0);

    gnss_cov_trace_fail_ = declare_parameter<double>("gnss_cov_trace_fail", 0.3);
    gnss_jump_fail_m_ = declare_parameter<double>("gnss_jump_fail_m", 1.0);
    gnss_min_hz_ = declare_parameter<double>("gnss_min_hz", 2.0);

    publish_localization_status_ =
      declare_parameter<bool>("publish_localization_status", false);
    localization_status_topic_ =
      declare_parameter<std::string>("localization_status_topic", "/localization/status");

    mode_pub_ = create_publisher<AvgLocalizationMode>("/localization/mode", rclcpp::QoS(10));
    status_pub_ = create_publisher<AvgLocalizationStatus>("/localization/status", rclcpp::QoS(10));
    confidence_pub_ = create_publisher<avg_msgs::msg::Float32>("/localization/confidence", rclcpp::QoS(10));
    state_pub_ = create_publisher<avg_msgs::msg::Bool>(
      "/localization/state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

    if (publish_localization_status_) {
      avg_localization_pub_ = create_publisher<avg_msgs::msg::AvgLocalizationMsgs>(
        localization_status_topic_, rclcpp::QoS(10));
    }

    using std::placeholders::_1;

    if (use_filter_status_ && !diag_topic_.empty()) {
      diag_sub_ = create_subscription<AvgLocalizationStatusStream>(
        diag_topic_, rclcpp::QoS(20),
        std::bind(&LocalizationSupervisorNode::onDiag, this, _1));
    }

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
  }

private:
  void onDiag(const AvgLocalizationStatusStream::ConstSharedPtr msg)
  {
    last_diag_ = *msg;
    last_diag_time_ = rclcpp::Time(msg->header.stamp);
  }

  void onGnss(const avg_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
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

    last_gnss_time_ = stamp;
    last_gnss_cov_trace_ = msg->pose.covariance[0] + msg->pose.covariance[7];
  }

  void onImu(const avg_msgs::msg::Imu::ConstSharedPtr msg)
  {
    last_imu_time_ = rclcpp::Time(msg->header.stamp);
  }

  void onWheel(const avg_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    last_wheel_time_ = rclcpp::Time(msg->header.stamp);
  }

  AvgLocalizationMode makeMode(const uint8_t value, const std::string & label)
  {
    AvgLocalizationMode m;
    m.value = value;
    m.label = label;
    return m;
  }

  void onTimer()
  {
    const rclcpp::Time now = this->now();

    const bool imu_ok = (now - last_imu_time_).seconds() <= imu_timeout_sec_;
    const bool gnss_fresh = (now - last_gnss_time_).seconds() <= gnss_timeout_sec_;
    const bool wheel_ok = (now - last_wheel_time_).seconds() <= wheel_timeout_sec_;

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

    AvgLocalizationMode mode_msg;
    if (!imu_ok) {
      mode_msg = makeMode(AvgLocalizationMode::INVALID, "INVALID");
    } else if (!gnss_good && wheel_ok) {
      mode_msg = makeMode(AvgLocalizationMode::DR_ONLY, "DR_ONLY");
    } else if (gnss_good && wheel_good && !gnss_innov_warn_bad) {
      mode_msg = makeMode(AvgLocalizationMode::NORMAL, "NORMAL");
    } else if (gnss_good || wheel_ok) {
      mode_msg = makeMode(AvgLocalizationMode::DEGRADED, "DEGRADED");
    } else {
      mode_msg = makeMode(AvgLocalizationMode::INVALID, "INVALID");
    }

    if (gnss_innov_fail_bad) {
      mode_msg = makeMode(AvgLocalizationMode::DEGRADED, "DEGRADED");
    }

    AvgLocalizationStatus status;
    status.header.stamp = now;
    status.mode = mode_msg;
    status.confidence = static_cast<float>(confidence);
    status.gnss_ok = gnss_good;
    status.imu_ok = imu_ok;
    status.wheel_ok = wheel_good;
    status.gnss_innovation_norm = diag_available ? last_diag_.gnss_innovation_norm : 0.0;
    status.wheel_innovation_norm = diag_available ? last_diag_.wheel_innovation_norm : 0.0;

    avg_msgs::msg::Bool state_msg;
    state_msg.data = (mode_msg.value == AvgLocalizationMode::NORMAL) ||
                      (mode_msg.value == AvgLocalizationMode::DEGRADED);

    avg_msgs::msg::Float32 conf_msg;
    conf_msg.data = static_cast<float>(confidence);

    mode_pub_->publish(mode_msg);
    status_pub_->publish(status);
    confidence_pub_->publish(conf_msg);
    state_pub_->publish(state_msg);

    if (publish_localization_status_ && avg_localization_pub_) {
      avg_msgs::msg::AvgLocalizationMsgs avg_msg;
      avg_msg.stamp = status.header.stamp;
      avg_msg.localization_mode = mode_msg;
      avg_msg.localization_status = status;
      if (diag_available) {
        avg_msg.localization_status_stream = last_diag_;
        avg_msg.gnss_update_accepted = last_diag_.gnss_update_accepted;
        avg_msg.gnss_innovation_norm = last_diag_.gnss_innovation_norm;
        avg_msg.wheel_update_accepted = last_diag_.wheel_update_accepted;
        avg_msg.wheel_innovation_norm = last_diag_.wheel_innovation_norm;
        avg_msg.covariance_trace = last_diag_.covariance_trace;
      }
      avg_msg.state.stamp = status.header.stamp;
      avg_msg.state.module_name = "localization";
      if (mode_msg.value == AvgLocalizationMode::NORMAL) {
        avg_msg.state.level = avg_msgs::msg::ModuleState::OK;
      } else if (mode_msg.value == AvgLocalizationMode::DEGRADED ||
                 mode_msg.value == AvgLocalizationMode::DR_ONLY) {
        avg_msg.state.level = avg_msgs::msg::ModuleState::WARN;
      } else {
        avg_msg.state.level = avg_msgs::msg::ModuleState::ERROR;
      }
      avg_msg.state.message = "localization_supervisor";
      avg_localization_pub_->publish(avg_msg);
    }
  }

  std::string diag_topic_;
  bool use_filter_status_{true};
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

  bool publish_localization_status_{false};
  std::string localization_status_topic_;

  rclcpp::Time last_diag_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gnss_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wheel_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time prev_gnss_time_{0, 0, RCL_ROS_TIME};

  AvgLocalizationStatusStream last_diag_{};

  bool has_prev_gnss_{false};
  double last_gnss_x_{0.0};
  double last_gnss_y_{0.0};
  double last_gnss_jump_m_{0.0};
  double last_gnss_hz_{0.0};
  double last_gnss_cov_trace_{0.0};

  rclcpp::Subscription<AvgLocalizationStatusStream>::SharedPtr diag_sub_;
  rclcpp::Subscription<avg_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_sub_;
  rclcpp::Subscription<avg_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<avg_msgs::msg::Odometry>::SharedPtr wheel_sub_;

  rclcpp::Publisher<AvgLocalizationMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<AvgLocalizationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<avg_msgs::msg::Float32>::SharedPtr confidence_pub_;
  rclcpp::Publisher<avg_msgs::msg::Bool>::SharedPtr state_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationMsgs>::SharedPtr avg_localization_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationSupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
