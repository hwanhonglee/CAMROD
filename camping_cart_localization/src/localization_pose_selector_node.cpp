#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

#include "camping_cart_localization/msg/localization_mode.hpp"

#include <array>
#include <chrono>
#include <string>

using camping_cart_localization::msg::LocalizationMode;

namespace
{
enum class Source
{
  kPrimary = 0,
  kFallback = 1
};

std::array<double, 36> makeUnknownTwistCov()
{
  std::array<double, 36> cov{};
  cov.fill(0.0);
  cov[0] = cov[7] = cov[14] = 1e4;
  cov[21] = cov[28] = cov[35] = 1e4;
  return cov;
}

}  // namespace

class LocalizationPoseSelectorNode : public rclcpp::Node
{
public:
  LocalizationPoseSelectorNode()
  : Node("pose_selector")
  {
    // HH_260209: Primary = ESKF, fallback = Kimera-VIO bridge.
    primary_pose_cov_topic_ = declare_parameter<std::string>(
      "primary_pose_cov_topic", "/localization/eskf/pose_with_covariance");
    primary_odom_topic_ = declare_parameter<std::string>(
      "primary_odom_topic", "/localization/eskf/odometry");
    fallback_pose_cov_topic_ = declare_parameter<std::string>(
      "fallback_pose_cov_topic", "/localization/kimera_vio/pose_with_covariance");
    fallback_odom_topic_ = declare_parameter<std::string>(
      "fallback_odom_topic", "/localization/kimera_vio/odometry");
    mode_topic_ = declare_parameter<std::string>(
      "mode_topic", "/localization/mode");

    selected_pose_topic_ = declare_parameter<std::string>(
      "selected_pose_topic", "/localization/pose");
    selected_pose_cov_topic_ = declare_parameter<std::string>(
      "selected_pose_cov_topic", "/localization/pose_with_covariance");
    selected_odom_topic_ = declare_parameter<std::string>(
      "selected_odom_topic", "/localization/odometry/filtered");
    selected_source_topic_ = declare_parameter<std::string>(
      "selected_source_topic", "/localization/pose_source");

    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "robot_base_link");
    primary_timeout_sec_ = declare_parameter<double>("primary_timeout_sec", 0.5);
    fallback_timeout_sec_ = declare_parameter<double>("fallback_timeout_sec", 0.5);
    switch_hysteresis_sec_ = declare_parameter<double>("switch_hysteresis_sec", 0.5);
    fallback_on_mode_at_or_above_ = declare_parameter<int>(
      "fallback_on_mode_at_or_above",
      static_cast<int>(LocalizationMode::DR_ONLY));

    primary_timeout_sec_ = std::max(0.05, primary_timeout_sec_);
    fallback_timeout_sec_ = std::max(0.05, fallback_timeout_sec_);
    switch_hysteresis_sec_ = std::max(0.0, switch_hysteresis_sec_);
    fallback_on_mode_at_or_above_ = std::max(0, std::min(3, fallback_on_mode_at_or_above_));

    rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
    latched_qos.transient_local().reliable();
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(selected_pose_topic_, latched_qos);
    pose_cov_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      selected_pose_cov_topic_, latched_qos);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(selected_odom_topic_, latched_qos);
    source_pub_ = create_publisher<std_msgs::msg::String>(selected_source_topic_, latched_qos);

    using std::placeholders::_1;
    primary_pose_cov_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      primary_pose_cov_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationPoseSelectorNode::onPrimaryPoseCov, this, _1));
    primary_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      primary_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationPoseSelectorNode::onPrimaryOdom, this, _1));

    rclcpp::QoS fallback_qos(rclcpp::KeepLast(1));
    fallback_qos.transient_local().reliable();
    fallback_pose_cov_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      fallback_pose_cov_topic_, fallback_qos,
      std::bind(&LocalizationPoseSelectorNode::onFallbackPoseCov, this, _1));
    fallback_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      fallback_odom_topic_, fallback_qos,
      std::bind(&LocalizationPoseSelectorNode::onFallbackOdom, this, _1));
    mode_sub_ = create_subscription<LocalizationMode>(
      mode_topic_, rclcpp::QoS(20),
      std::bind(&LocalizationPoseSelectorNode::onMode, this, _1));

    monitor_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LocalizationPoseSelectorNode::onMonitorTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "pose_selector started. primary=(%s,%s) fallback=(%s,%s) output=(%s,%s,%s)",
      primary_pose_cov_topic_.c_str(), primary_odom_topic_.c_str(),
      fallback_pose_cov_topic_.c_str(), fallback_odom_topic_.c_str(),
      selected_pose_topic_.c_str(), selected_pose_cov_topic_.c_str(),
      selected_odom_topic_.c_str());
  }

private:
  static rclcpp::Time stampFromHeader(const std_msgs::msg::Header & header)
  {
    return rclcpp::Time(header.stamp);
  }

  static rclcpp::Time maxStamp(const rclcpp::Time & a, const rclcpp::Time & b)
  {
    return (a.nanoseconds() >= b.nanoseconds()) ? a : b;
  }

  void onMode(const LocalizationMode::ConstSharedPtr msg)
  {
    mode_value_ = static_cast<int>(msg->value);
    evaluateAndPublish(this->now());
  }

  void onPrimaryPoseCov(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    primary_pose_cov_ = *msg;
    primary_has_pose_cov_ = true;
    last_primary_msg_time_ = this->now();
    evaluateAndPublish(this->now());
  }

  void onPrimaryOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    primary_odom_ = *msg;
    primary_has_odom_ = true;
    last_primary_msg_time_ = this->now();
    evaluateAndPublish(this->now());
  }

  void onFallbackPoseCov(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    fallback_pose_cov_ = *msg;
    fallback_has_pose_cov_ = true;
    last_fallback_msg_time_ = this->now();
    evaluateAndPublish(this->now());
  }

  void onFallbackOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    fallback_odom_ = *msg;
    fallback_has_odom_ = true;
    last_fallback_msg_time_ = this->now();
    evaluateAndPublish(this->now());
  }

  bool sourceHasData(const Source source) const
  {
    if (source == Source::kPrimary) {
      return primary_has_pose_cov_ || primary_has_odom_;
    }
    return fallback_has_pose_cov_ || fallback_has_odom_;
  }

  rclcpp::Time sourceLatestStamp(const Source source) const
  {
    rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
    if (source == Source::kPrimary) {
      if (primary_has_pose_cov_) {
        stamp = maxStamp(stamp, stampFromHeader(primary_pose_cov_.header));
      }
      if (primary_has_odom_) {
        stamp = maxStamp(stamp, stampFromHeader(primary_odom_.header));
      }
      return stamp;
    }
    if (fallback_has_pose_cov_) {
      stamp = maxStamp(stamp, stampFromHeader(fallback_pose_cov_.header));
    }
    if (fallback_has_odom_) {
      stamp = maxStamp(stamp, stampFromHeader(fallback_odom_.header));
    }
    return stamp;
  }

  bool sourceFresh(const Source source, const rclcpp::Time & now) const
  {
    if (source == Source::kPrimary) {
      if (!sourceHasData(source)) return false;
      return (now - last_primary_msg_time_).seconds() <= primary_timeout_sec_;
    }
    if (!sourceHasData(source)) return false;
    return (now - last_fallback_msg_time_).seconds() <= fallback_timeout_sec_;
  }

  bool wantFallback(const rclcpp::Time & now) const
  {
    const bool mode_bad = mode_value_ >= fallback_on_mode_at_or_above_;
    const bool primary_bad = !sourceFresh(Source::kPrimary, now);
    return mode_bad || primary_bad;
  }

  void evaluateAndPublish(const rclcpp::Time & now)
  {
    Source desired = Source::kPrimary;
    if (wantFallback(now) && sourceFresh(Source::kFallback, now)) {
      desired = Source::kFallback;
    } else if (!sourceFresh(Source::kPrimary, now) && sourceFresh(Source::kFallback, now)) {
      desired = Source::kFallback;
    }

    bool switched = false;
    if (!selected_initialized_) {
      if (sourceFresh(desired, now)) {
        selected_source_ = desired;
      } else if (sourceFresh(Source::kPrimary, now)) {
        selected_source_ = Source::kPrimary;
      } else if (sourceFresh(Source::kFallback, now)) {
        selected_source_ = Source::kFallback;
      } else {
        return;
      }
      selected_initialized_ = true;
      last_switch_time_ = now;
      switched = true;
    } else {
      const bool selected_fresh = sourceFresh(selected_source_, now);
      if (!selected_fresh) {
        if (sourceFresh(desired, now)) {
          selected_source_ = desired;
          last_switch_time_ = now;
          switched = true;
        } else {
          return;
        }
      } else if (desired != selected_source_ && sourceFresh(desired, now)) {
        if ((now - last_switch_time_).seconds() >= switch_hysteresis_sec_) {
          selected_source_ = desired;
          last_switch_time_ = now;
          switched = true;
        }
      }
    }

    publishSelected(switched);
  }

  void publishSelected(const bool force_publish)
  {
    if (!sourceHasData(selected_source_)) {
      return;
    }

    const rclcpp::Time source_stamp = sourceLatestStamp(selected_source_);
    if (!force_publish && source_stamp.nanoseconds() <= last_published_stamp_.nanoseconds()) {
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped out_pose_cov;
    nav_msgs::msg::Odometry out_odom;
    geometry_msgs::msg::PoseStamped out_pose;

    if (selected_source_ == Source::kPrimary) {
      if (primary_has_pose_cov_) {
        out_pose_cov = primary_pose_cov_;
      }
      if (primary_has_odom_) {
        out_odom = primary_odom_;
      }
    } else {
      if (fallback_has_pose_cov_) {
        out_pose_cov = fallback_pose_cov_;
      }
      if (fallback_has_odom_) {
        out_odom = fallback_odom_;
      }
    }

    // Build missing messages from whichever source exists.
    if (out_pose_cov.header.stamp.sec == 0 && out_pose_cov.header.stamp.nanosec == 0 &&
      (out_odom.header.stamp.sec != 0 || out_odom.header.stamp.nanosec != 0))
    {
      out_pose_cov.header = out_odom.header;
      out_pose_cov.pose = out_odom.pose;
    }

    if (out_odom.header.stamp.sec == 0 && out_odom.header.stamp.nanosec == 0 &&
      (out_pose_cov.header.stamp.sec != 0 || out_pose_cov.header.stamp.nanosec != 0))
    {
      out_odom.header = out_pose_cov.header;
      out_odom.child_frame_id = base_frame_id_;
      out_odom.pose = out_pose_cov.pose;
      out_odom.twist.covariance = makeUnknownTwistCov();
    }

    out_pose.header = out_pose_cov.header;
    out_pose.pose = out_pose_cov.pose.pose;

    pose_cov_pub_->publish(out_pose_cov);
    pose_pub_->publish(out_pose);
    odom_pub_->publish(out_odom);

    std_msgs::msg::String source_msg;
    source_msg.data = (selected_source_ == Source::kPrimary) ? "primary_eskf" : "kimera_vio";
    source_pub_->publish(source_msg);

    last_published_stamp_ = source_stamp;
    if (last_source_label_ != source_msg.data) {
      RCLCPP_WARN(get_logger(), "pose selector source changed -> %s", source_msg.data.c_str());
      last_source_label_ = source_msg.data;
    }
  }

  void onMonitorTimer()
  {
    evaluateAndPublish(this->now());
  }

private:
  // Parameters
  std::string primary_pose_cov_topic_;
  std::string primary_odom_topic_;
  std::string fallback_pose_cov_topic_;
  std::string fallback_odom_topic_;
  std::string mode_topic_;
  std::string selected_pose_topic_;
  std::string selected_pose_cov_topic_;
  std::string selected_odom_topic_;
  std::string selected_source_topic_;
  std::string base_frame_id_;
  double primary_timeout_sec_{0.5};
  double fallback_timeout_sec_{0.5};
  double switch_hysteresis_sec_{0.5};
  int fallback_on_mode_at_or_above_{2};

  // State
  int mode_value_{static_cast<int>(LocalizationMode::INVALID)};
  bool selected_initialized_{false};
  Source selected_source_{Source::kPrimary};
  rclcpp::Time last_switch_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_primary_msg_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_fallback_msg_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_published_stamp_{0, 0, RCL_ROS_TIME};
  std::string last_source_label_{};

  bool primary_has_pose_cov_{false};
  bool primary_has_odom_{false};
  bool fallback_has_pose_cov_{false};
  bool fallback_has_odom_{false};
  geometry_msgs::msg::PoseWithCovarianceStamped primary_pose_cov_;
  geometry_msgs::msg::PoseWithCovarianceStamped fallback_pose_cov_;
  nav_msgs::msg::Odometry primary_odom_;
  nav_msgs::msg::Odometry fallback_odom_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr primary_pose_cov_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr primary_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr fallback_pose_cov_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fallback_odom_sub_;
  rclcpp::Subscription<LocalizationMode>::SharedPtr mode_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr source_pub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationPoseSelectorNode>());
  rclcpp::shutdown();
  return 0;
}
