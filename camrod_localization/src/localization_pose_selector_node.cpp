#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <avg_msgs/conversions.hpp>
#include <rclcpp/rclcpp.hpp>

#include <avg_msgs/msg/avg_header.hpp>
#include <avg_msgs/msg/avg_odometry.hpp>
#include <avg_msgs/msg/avg_pose_stamped.hpp>
#include <avg_msgs/msg/avg_pose_with_covariance_stamped.hpp>
#include <avg_msgs/msg/avg_string.hpp>
#include <avg_msgs/msg/avg_localization_mode.hpp>
#include <avg_msgs/msg/avg_localization_msgs.hpp>
#include <avg_msgs/msg/module_state.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.

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

bool isZeroStamp(const builtin_interfaces::msg::Time & t)
{
  return t.sec == 0 && t.nanosec == 0;
}

}  // namespace

class LocalizationPoseSelectorNode : public rclcpp::Node
{
public:
  LocalizationPoseSelectorNode()
  : Node("pose_selector")
  {
    primary_pose_cov_topic_ = declare_parameter<std::string>(
      "primary_pose_cov_topic", "/localization/primary/pose_with_covariance");
    primary_odom_topic_ = declare_parameter<std::string>(
      "primary_odom_topic", "/localization/primary/odometry");
    fallback_pose_cov_topic_ = declare_parameter<std::string>(
      "fallback_pose_cov_topic", "/localization/fallback/pose_with_covariance");
    fallback_odom_topic_ = declare_parameter<std::string>(
      "fallback_odom_topic", "/localization/fallback/odometry");
    // HH_260527: Allow empty fallback topics to run primary-only selector mode.
    fallback_enabled_ = !fallback_pose_cov_topic_.empty() || !fallback_odom_topic_.empty();
    mode_topic_ = declare_parameter<std::string>(
      "mode_topic", "/localization/mode");

    selected_pose_topic_ = declare_parameter<std::string>(
      "selected_pose_topic", "/localization/pose");
    // HH_260720 - Standard ROS mirrors are explicit external-tool boundaries.
    selected_pose_ros_topic_ = declare_parameter<std::string>(
      "selected_pose_ros_topic", "/localization/pose_ros");
    selected_pose_cov_topic_ = declare_parameter<std::string>(
      "selected_pose_cov_topic", "/localization/pose_with_covariance");
    selected_pose_cov_ros_topic_ = declare_parameter<std::string>(
      "selected_pose_cov_ros_topic", "/localization/pose_with_covariance_ros");
    // HH_260720 - Publish the selected internal odometry with a semantic public name.
    selected_odom_topic_ = declare_parameter<std::string>(
      "selected_odom_topic", "/localization/odometry");
    // HH_260720 - Publish an explicit nav_msgs mirror only for ROS ecosystem consumers.
    selected_odom_ros_topic_ = declare_parameter<std::string>(
      "selected_odom_ros_topic", "/localization/odometry_ros");
    selected_source_topic_ = declare_parameter<std::string>(
      "selected_source_topic", "/localization/pose_source");

    primary_source_label_ = declare_parameter<std::string>(
      "primary_source_label", "primary_filter");
    fallback_source_label_ = declare_parameter<std::string>(
      "fallback_source_label", "fallback_source");

    publish_localization_status_ =
      declare_parameter<bool>("publish_localization_status", false);
    localization_status_topic_ = declare_parameter<std::string>(
      "localization_status_topic", "/localization/status");

    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "robot_base_link");
    publish_selected_tf_ = declare_parameter<bool>("publish_selected_tf", true);

    primary_timeout_s_ = declare_parameter<double>("primary_timeout_s", 0.5);
    fallback_timeout_s_ = declare_parameter<double>("fallback_timeout_s", 0.5);
    switch_hysteresis_s_ = declare_parameter<double>("switch_hysteresis_s", 0.5);
    fallback_on_mode_at_or_above_ = declare_parameter<int>(
      "fallback_on_mode_at_or_above",
      static_cast<int>(avg_msgs::msg::AvgLocalizationMode::DR_ONLY));

    primary_timeout_s_ = std::max(0.05, primary_timeout_s_);
    fallback_timeout_s_ = std::max(0.05, fallback_timeout_s_);
    switch_hysteresis_s_ = std::max(0.0, switch_hysteresis_s_);
    fallback_on_mode_at_or_above_ = std::max(0, std::min(3, fallback_on_mode_at_or_above_));

    rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
    latched_qos.transient_local().reliable();

    pose_pub_ = create_publisher<avg_msgs::msg::AvgPoseStamped>(selected_pose_topic_, latched_qos);
    pose_ros_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      selected_pose_ros_topic_, latched_qos);
    pose_cov_pub_ = create_publisher<avg_msgs::msg::AvgPoseWithCovarianceStamped>(
      selected_pose_cov_topic_, latched_qos);
    pose_cov_ros_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      selected_pose_cov_ros_topic_, latched_qos);
    // HH_260720 - The canonical selected odometry is a real generated avg_msgs interface.
    odom_pub_ = create_publisher<avg_msgs::msg::AvgOdometry>(selected_odom_topic_, latched_qos);
    odom_ros_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(selected_odom_ros_topic_, latched_qos);
    source_pub_ = create_publisher<avg_msgs::msg::AvgString>(selected_source_topic_, latched_qos);

    if (publish_localization_status_) {
      avg_localization_pub_ = create_publisher<avg_msgs::msg::AvgLocalizationMsgs>(
        localization_status_topic_, rclcpp::QoS(10));
    }

    if (publish_selected_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    using std::placeholders::_1;

    primary_pose_cov_sub_ = create_subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>(
      primary_pose_cov_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationPoseSelectorNode::onPrimaryPoseCov, this, _1));
    primary_odom_sub_ = create_subscription<avg_msgs::msg::AvgOdometry>(
      primary_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationPoseSelectorNode::onPrimaryOdom, this, _1));

    rclcpp::QoS fallback_qos(rclcpp::KeepLast(1));
    fallback_qos.transient_local().reliable();

    if (!fallback_pose_cov_topic_.empty()) {
      fallback_pose_cov_sub_ = create_subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>(
        fallback_pose_cov_topic_, fallback_qos,
        std::bind(&LocalizationPoseSelectorNode::onFallbackPoseCov, this, _1));
    }
    if (!fallback_odom_topic_.empty()) {
      fallback_odom_sub_ = create_subscription<avg_msgs::msg::AvgOdometry>(
        fallback_odom_topic_, fallback_qos,
        std::bind(&LocalizationPoseSelectorNode::onFallbackOdom, this, _1));
    }
    mode_sub_ = create_subscription<avg_msgs::msg::AvgLocalizationMode>(
      mode_topic_, rclcpp::QoS(20),
      std::bind(&LocalizationPoseSelectorNode::onMode, this, _1));

    watcher_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LocalizationPoseSelectorNode::onWatcherTimer, this));

    if (fallback_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "pose_selector started. primary=(%s,%s) fallback=(%s,%s) output=(%s,%s,%s)",
        primary_pose_cov_topic_.c_str(), primary_odom_topic_.c_str(),
        fallback_pose_cov_topic_.c_str(), fallback_odom_topic_.c_str(),
        selected_pose_topic_.c_str(), selected_pose_cov_topic_.c_str(),
        selected_odom_topic_.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "pose_selector started. primary-only mode enabled. output=(%s,%s,%s)",
        selected_pose_topic_.c_str(), selected_pose_cov_topic_.c_str(),
        selected_odom_topic_.c_str());
    }
  }

private:
  static rclcpp::Time stampFromHeader(const avg_msgs::msg::AvgHeader & header)
  {
    return rclcpp::Time(header.stamp);
  }

  static rclcpp::Time maxStamp(const rclcpp::Time & a, const rclcpp::Time & b)
  {
    return (a.nanoseconds() >= b.nanoseconds()) ? a : b;
  }

  void onMode(const avg_msgs::msg::AvgLocalizationMode::ConstSharedPtr msg)
  {
    mode_value_ = static_cast<int>(msg->value);
    evaluateAndPublish(this->now());
  }

  void onPrimaryPoseCov(const avg_msgs::msg::AvgPoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    primary_pose_cov_ = *msg;
    primary_has_pose_cov_ = true;
    last_primary_msg_time_ = this->now();
    evaluateAndPublish(this->now());
  }

  void onPrimaryOdom(const avg_msgs::msg::AvgOdometry::ConstSharedPtr msg)
  {
    primary_odom_ = *msg;
    primary_has_odom_ = true;
    last_primary_msg_time_ = this->now();
    evaluateAndPublish(this->now());
  }

  void onFallbackPoseCov(const avg_msgs::msg::AvgPoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    fallback_pose_cov_ = *msg;
    fallback_has_pose_cov_ = true;
    last_fallback_msg_time_ = this->now();
    evaluateAndPublish(this->now());
  }

  void onFallbackOdom(const avg_msgs::msg::AvgOdometry::ConstSharedPtr msg)
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
    if (!fallback_enabled_) {
      return false;
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
      if (!sourceHasData(source)) {return false;}
      return (now - last_primary_msg_time_).seconds() <= primary_timeout_s_;
    }
    if (!fallback_enabled_) {return false;}
    if (!sourceHasData(source)) {return false;}
    return (now - last_fallback_msg_time_).seconds() <= fallback_timeout_s_;
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
        if ((now - last_switch_time_).seconds() >= switch_hysteresis_s_) {
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

    avg_msgs::msg::AvgPoseWithCovarianceStamped out_pose_cov;
    avg_msgs::msg::AvgOdometry out_odom;
    avg_msgs::msg::AvgPoseStamped out_pose;

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

    if (isZeroStamp(out_pose_cov.header.stamp) && !isZeroStamp(out_odom.header.stamp)) {
      out_pose_cov.header = out_odom.header;
      out_pose_cov.pose = out_odom.pose;
    }

    if (isZeroStamp(out_odom.header.stamp) && !isZeroStamp(out_pose_cov.header.stamp)) {
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
    // HH_260720 - Keep external-tool conversion visible instead of relying on aliases.
    pose_ros_pub_->publish(avg_msgs::conversions::toRos(out_pose));
    pose_cov_ros_pub_->publish(avg_msgs::conversions::toRos(out_pose_cov));
    odom_ros_pub_->publish(avg_msgs::conversions::toRos(out_odom));

    avg_msgs::msg::AvgString source_msg;
    source_msg.data =
      (selected_source_ == Source::kPrimary) ? primary_source_label_ : fallback_source_label_;
    source_pub_->publish(source_msg);

    if (publish_selected_tf_ && tf_broadcaster_) {
      publishSelectedTf(out_pose_cov, out_odom);
    }

    if (publish_localization_status_ && avg_localization_pub_) {
      avg_msgs::msg::AvgLocalizationMsgs avg_msg;
      avg_msg.stamp = out_pose.header.stamp;
      avg_msg.localization_pose = out_pose;
      avg_msg.localization_pose_cov = out_pose_cov;
      avg_msg.localization_odom = out_odom;
      avg_msg.state.stamp = out_pose.header.stamp;
      avg_msg.state.module_name = "localization";
      avg_msg.state.level = avg_msgs::msg::ModuleState::OK;
      avg_msg.state.message = std::string("pose_selector_source=") + source_msg.data;
      avg_localization_pub_->publish(avg_msg);
    }

    last_published_stamp_ = source_stamp;
    if (last_source_label_ != source_msg.data) {
      RCLCPP_WARN(get_logger(), "pose selector source changed -> %s", source_msg.data.c_str());
      last_source_label_ = source_msg.data;
    }
  }

  void publishSelectedTf(
    const avg_msgs::msg::AvgPoseWithCovarianceStamped & pose_cov_msg,
    const avg_msgs::msg::AvgOdometry & odom_msg)
  {
    geometry_msgs::msg::TransformStamped tf_msg;

    if (!isZeroStamp(pose_cov_msg.header.stamp)) {
      tf_msg.header.stamp = pose_cov_msg.header.stamp;
      tf_msg.header.frame_id = pose_cov_msg.header.frame_id;
      tf_msg.child_frame_id = base_frame_id_;
      tf_msg.transform.translation.x = pose_cov_msg.pose.pose.position.x;
      tf_msg.transform.translation.y = pose_cov_msg.pose.pose.position.y;
      tf_msg.transform.translation.z = pose_cov_msg.pose.pose.position.z;
      tf_msg.transform.rotation = avg_msgs::conversions::toRos(
        pose_cov_msg.pose.pose.orientation);
      tf_broadcaster_->sendTransform(tf_msg);
      return;
    }

    if (!isZeroStamp(odom_msg.header.stamp)) {
      tf_msg.header.stamp = odom_msg.header.stamp;
      tf_msg.header.frame_id = odom_msg.header.frame_id;
      tf_msg.child_frame_id =
        odom_msg.child_frame_id.empty() ? base_frame_id_ : odom_msg.child_frame_id;
      tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
      tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
      tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
      tf_msg.transform.rotation = avg_msgs::conversions::toRos(
        odom_msg.pose.pose.orientation);
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

  void onWatcherTimer()
  {
    evaluateAndPublish(this->now());
  }

  std::string primary_pose_cov_topic_;
  std::string primary_odom_topic_;
  std::string fallback_pose_cov_topic_;
  std::string fallback_odom_topic_;
  std::string mode_topic_;

  std::string selected_pose_topic_;
  std::string selected_pose_ros_topic_;
  std::string selected_pose_cov_topic_;
  std::string selected_pose_cov_ros_topic_;
  std::string selected_odom_topic_;
  std::string selected_odom_ros_topic_;
  std::string selected_source_topic_;

  std::string primary_source_label_;
  std::string fallback_source_label_;

  std::string localization_status_topic_;
  std::string base_frame_id_;

  double primary_timeout_s_{0.5};
  double fallback_timeout_s_{0.5};
  double switch_hysteresis_s_{0.5};
  int fallback_on_mode_at_or_above_{2};

  // HH_260422: true -> also publish /localization/status (AvgLocalizationMsgs).
  bool publish_localization_status_{false};
  bool publish_selected_tf_{true};          // HH_260422: true -> broadcast selected pose as TF transform
  bool fallback_enabled_{true};             // HH_260527: false when fallback topics are intentionally left empty

  // HH_260422: mode_value_ holds the latest enum received from /localization/mode (published by localization_monitor).
  //   When mode_value_ >= fallback_on_mode_at_or_above_ (default DR_ONLY=2), selector switches to fallback source.
  //   Data flow: localization_monitor -> /localization/mode -> pose_selector (source switch).
  int mode_value_{static_cast<int>(avg_msgs::msg::AvgLocalizationMode::INVALID)};

  // HH_260422: selected_initialized_ becomes true after the first source selection completes.
  //   While false: evaluateAndPublish() skips all source-switching logic.
  bool selected_initialized_{false};
  Source selected_source_{Source::kPrimary};
  rclcpp::Time last_switch_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_primary_msg_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_fallback_msg_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_published_stamp_{0, 0, RCL_ROS_TIME};
  std::string last_source_label_;

  // HH_260720 - Primary availability becomes true when the selected localization filter publishes.
  //   sourceHasData(kPrimary) = primary_has_pose_cov_ || primary_has_odom_.
  //   If both are false, primary cannot be selected and fallback is forced regardless of mode.
  bool primary_has_pose_cov_{false};
  bool primary_has_odom_{false};
  // HH_260422: fallback_has_pose_cov_ / fallback_has_odom_ become true once the fallback source publishes data.
  //   A mode_bad or primary_bad condition only switches to fallback if fallback data is actually available.
  bool fallback_has_pose_cov_{false};
  bool fallback_has_odom_{false};

  avg_msgs::msg::AvgPoseWithCovarianceStamped primary_pose_cov_;
  avg_msgs::msg::AvgPoseWithCovarianceStamped fallback_pose_cov_;
  avg_msgs::msg::AvgOdometry primary_odom_;
  avg_msgs::msg::AvgOdometry fallback_odom_;

  rclcpp::Subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>::SharedPtr primary_pose_cov_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgOdometry>::SharedPtr primary_odom_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>::SharedPtr
    fallback_pose_cov_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgOdometry>::SharedPtr fallback_odom_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgLocalizationMode>::SharedPtr mode_sub_;

  rclcpp::Publisher<avg_msgs::msg::AvgPoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_ros_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgPoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_ros_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgOdometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_ros_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgString>::SharedPtr source_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationMsgs>::SharedPtr avg_localization_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr watcher_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationPoseSelectorNode>());
  rclcpp::shutdown();
  return 0;
}
