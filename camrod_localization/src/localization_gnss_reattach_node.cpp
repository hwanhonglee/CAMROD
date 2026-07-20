#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

// HH_260720 - Keep localization data on generated CAMROD interfaces.
#include <avg_msgs/msg/avg_localization_mode.hpp>
#include <avg_msgs/msg/avg_odometry.hpp>
#include <avg_msgs/msg/avg_pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace camrod::localization
{
namespace
{
std::string normalizeToken(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

double normalizeYaw(double yaw)
{
  constexpr double kTwoPi = 2.0 * M_PI;
  while (yaw > M_PI) yaw -= kTwoPi;
  while (yaw < -M_PI) yaw += kTwoPi;
  return yaw;
}

double yawFromQuaternion(
  double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// HH_260720 - Populate either a CAMROD or external ROS quaternion without an alias.
template<typename QuaternionT>
void setQuaternionFromYaw(QuaternionT & q, double yaw)
{
  const double half = 0.5 * yaw;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
}
}  // namespace

class LocalizationGnssReattachNode : public rclcpp::Node
{
public:
  LocalizationGnssReattachNode()
  : Node("localization_gnss_reattach")
  {
    localization_pose_cov_topic_ = declare_parameter<std::string>(
      "localization_pose_cov_topic", "/localization/pose_with_covariance");
    gnss_pose_cov_topic_ = declare_parameter<std::string>(
      "gnss_pose_cov_topic", "/sensing/gnss/pose_with_covariance");
    wheel_odom_topic_ = declare_parameter<std::string>(
      "wheel_odom_topic", "/localization/input/wheel_odometry");
    localization_mode_topic_ = declare_parameter<std::string>(
      "localization_mode_topic", "/localization/mode");

    // HH_260528: robot_localization ekf_node reset subscriber is /localization/set_pose.
    // Publishing to /localization/ekf_filter/set_pose leaves EKF XY unchanged after RViz initialpose.
    ekf_set_pose_topic_ = declare_parameter<std::string>(
      "ekf_set_pose_topic", "/localization/set_pose");
    avg_pose_reset_topic_ = declare_parameter<std::string>(
      "avg_pose_reset_topic", "/localization/drop_zone/initial_pose");

    // HH_260527: reset_target_mode options:
    //   ekf_topic: publish geometry_msgs reset to EKF set_pose topic
    //   avg_pose_topic: publish avg_msgs reset to avg_pose_reset_topic
    //   both: publish both outputs
    reset_target_mode_ = normalizeToken(
      declare_parameter<std::string>("reset_target_mode", "ekf_topic"));
    use_localization_yaw_ = declare_parameter<bool>("use_localization_yaw", true);
    require_mode_gate_ = declare_parameter<bool>("require_mode_gate", false);
    require_mode_at_or_above_ = declare_parameter<int>("require_mode_at_or_above", 2);
    require_motion_after_detach_ = declare_parameter<bool>("require_motion_after_detach", true);

    detach_distance_m_ = declare_parameter<double>("detach_distance_m", 3.0);
    attach_distance_m_ = declare_parameter<double>("attach_distance_m", 1.5);
    detach_hold_s_ = declare_parameter<double>("detach_hold_s", 0.5);
    reattach_cooldown_s_ = declare_parameter<double>("reattach_cooldown_s", 3.0);
    motion_distance_threshold_m_ = declare_parameter<double>("motion_distance_threshold_m", 0.4);
    motion_step_clip_m_ = declare_parameter<double>("motion_step_clip_m", 5.0);
    wheel_motion_speed_threshold_mps_ = declare_parameter<double>(
      "wheel_motion_speed_threshold_mps", 0.12);

    localization_timeout_s_ = declare_parameter<double>("localization_timeout_s", 1.0);
    gnss_timeout_s_ = declare_parameter<double>("gnss_timeout_s", 2.0);

    max_gnss_cov_trace_ = declare_parameter<double>("max_gnss_cov_trace", 25.0);
    max_gnss_yaw_covariance_ = declare_parameter<double>("max_gnss_yaw_covariance", 100.0);
    min_reset_cov_xy_ = declare_parameter<double>("min_reset_cov_xy", 0.05);
    reset_cov_yaw_ = declare_parameter<double>("reset_cov_yaw", 0.5);
    // HH_260527: RViz 2D Pose Estimate bridge options.
    // initialpose_yaw_source options:
    //   input: use incoming /localization/initialpose yaw
    //   localization: use latest localization yaw
    //   gnss: use latest GNSS yaw (if available)
    enable_initialpose_reset_ = declare_parameter<bool>("enable_initialpose_reset", true);
    initialpose_topic_ = declare_parameter<std::string>(
      "initialpose_topic", "/localization/initialpose");
    // HH_260528: Keep manual RViz initialpose as operator-authoritative by default.
    // If GNSS snap is enabled here, far relocations can be pulled back to stale GNSS.
    initialpose_snap_to_gnss_ = declare_parameter<bool>("initialpose_snap_to_gnss", false);
    initialpose_snap_distance_m_ = declare_parameter<double>("initialpose_snap_distance_m", 1.0);
    // HH_260528: Also listen to RViz default /initialpose unless disabled.
    enable_initialpose_fallback_topic_ = declare_parameter<bool>(
      "enable_initialpose_fallback_topic", true);
    initialpose_topic_fallback_ = declare_parameter<std::string>(
      "initialpose_topic_fallback", "/initialpose");
    initialpose_yaw_source_ = normalizeToken(
      declare_parameter<std::string>("initialpose_yaw_source", "input"));

    if (reset_target_mode_ != "ekf_topic" &&
      reset_target_mode_ != "avg_pose_topic" &&
      reset_target_mode_ != "both")
    {
      RCLCPP_WARN(
        get_logger(),
        "Invalid reset_target_mode='%s'; fallback to 'ekf_topic'",
        reset_target_mode_.c_str());
      reset_target_mode_ = "ekf_topic";
    }
    if (attach_distance_m_ > detach_distance_m_) {
      RCLCPP_WARN(
        get_logger(),
        "attach_distance_m (%.2f) > detach_distance_m (%.2f); clamping attach distance",
        attach_distance_m_, detach_distance_m_);
      attach_distance_m_ = detach_distance_m_;
    }
    if (
      initialpose_yaw_source_ != "input" &&
      initialpose_yaw_source_ != "localization" &&
      initialpose_yaw_source_ != "gnss")
    {
      RCLCPP_WARN(
        get_logger(),
        "Invalid initialpose_yaw_source='%s'; fallback to 'input'",
        initialpose_yaw_source_.c_str());
      initialpose_yaw_source_ = "input";
    }

    using std::placeholders::_1;
    localization_pose_sub_ = create_subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>(
      localization_pose_cov_topic_, rclcpp::QoS(20),
      std::bind(&LocalizationGnssReattachNode::onLocalizationPose, this, _1));
    gnss_pose_sub_ = create_subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>(
      gnss_pose_cov_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationGnssReattachNode::onGnssPose, this, _1));
    wheel_sub_ = create_subscription<avg_msgs::msg::AvgOdometry>(
      wheel_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationGnssReattachNode::onWheelOdom, this, _1));
    mode_sub_ = create_subscription<avg_msgs::msg::AvgLocalizationMode>(
      localization_mode_topic_, rclcpp::QoS(10),
      std::bind(&LocalizationGnssReattachNode::onMode, this, _1));
    if (enable_initialpose_reset_) {
      initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initialpose_topic_, rclcpp::QoS(10),
        std::bind(&LocalizationGnssReattachNode::onInitialPose, this, _1));
      if (
        enable_initialpose_fallback_topic_ &&
        !initialpose_topic_fallback_.empty() &&
        initialpose_topic_fallback_ != initialpose_topic_)
      {
        initialpose_fallback_sub_ =
          create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          initialpose_topic_fallback_, rclcpp::QoS(10),
          std::bind(&LocalizationGnssReattachNode::onInitialPose, this, _1));
      }
    }

    ekf_set_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      ekf_set_pose_topic_, rclcpp::QoS(1));
    // HH_260720 - Publish reset requests as a real generated CAMROD message.
    avg_pose_reset_pub_ = create_publisher<avg_msgs::msg::AvgPoseWithCovarianceStamped>(
      avg_pose_reset_topic_, rclcpp::QoS(1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&LocalizationGnssReattachNode::onTimer, this));
  }

private:
  struct PoseSample
  {
    rclcpp::Time stamp;
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
    double cov_x{0.0};
    double cov_y{0.0};
    double cov_yaw{std::numeric_limits<double>::infinity()};
  };

  static double planarDistance(const PoseSample & a, const PoseSample & b)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::hypot(dx, dy);
  }

  bool isFresh(const PoseSample & sample, double timeout_s) const
  {
    return (now() - sample.stamp).seconds() <= timeout_s;
  }

  void onLocalizationPose(const avg_msgs::msg::AvgPoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    PoseSample current;
    current.stamp = rclcpp::Time(msg->header.stamp);
    current.x = msg->pose.pose.position.x;
    current.y = msg->pose.pose.position.y;
    current.z = msg->pose.pose.position.z;
    current.yaw = normalizeYaw(yawFromQuaternion(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w));
    current.cov_x = msg->pose.covariance[0];
    current.cov_y = msg->pose.covariance[7];
    current.cov_yaw = msg->pose.covariance[35];

    if (detached_ && prev_localization_pose_) {
      const double step = planarDistance(*prev_localization_pose_, current);
      if (step <= motion_step_clip_m_) {
        detached_motion_m_ += step;
      } else if (step >= motion_distance_threshold_m_) {
        detached_motion_m_ = std::max(detached_motion_m_, motion_distance_threshold_m_);
      }
      if (detached_motion_m_ >= motion_distance_threshold_m_) {
        motion_condition_met_ = true;
      }
    }

    prev_localization_pose_ = current;
    localization_pose_ = current;
  }

  void onGnssPose(const avg_msgs::msg::AvgPoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    PoseSample current;
    current.stamp = rclcpp::Time(msg->header.stamp);
    current.x = msg->pose.pose.position.x;
    current.y = msg->pose.pose.position.y;
    current.z = msg->pose.pose.position.z;
    current.yaw = normalizeYaw(yawFromQuaternion(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w));
    current.cov_x = msg->pose.covariance[0];
    current.cov_y = msg->pose.covariance[7];
    current.cov_yaw = msg->pose.covariance[35];
    gnss_pose_ = current;
  }

  void onWheelOdom(const avg_msgs::msg::AvgOdometry::ConstSharedPtr msg)
  {
    if (!detached_) {
      return;
    }
    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    const double speed = std::hypot(vx, vy);
    if (speed >= wheel_motion_speed_threshold_mps_) {
      motion_condition_met_ = true;
    }
  }

  void onMode(const avg_msgs::msg::AvgLocalizationMode::ConstSharedPtr msg)
  {
    have_mode_ = true;
    latest_mode_value_ = static_cast<int>(msg->value);
  }

  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }

    PoseSample requested;
    requested.stamp = rclcpp::Time(msg->header.stamp);
    if (requested.stamp.nanoseconds() == 0) {
      requested.stamp = now();
    }
    requested.x = msg->pose.pose.position.x;
    requested.y = msg->pose.pose.position.y;
    requested.z = msg->pose.pose.position.z;
    requested.yaw = normalizeYaw(yawFromQuaternion(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w));
    requested.cov_x = msg->pose.covariance[0];
    requested.cov_y = msg->pose.covariance[7];

    PoseSample reset = requested;
    bool snapped_to_gnss = false;
    double gap_to_gnss_m = -1.0;
    if (
      initialpose_snap_to_gnss_ &&
      gnss_pose_ &&
      isFresh(*gnss_pose_, gnss_timeout_s_) &&
      gnssCovarianceOk())
    {
      gap_to_gnss_m = planarDistance(requested, *gnss_pose_);
      if (gap_to_gnss_m >= std::max(0.0, initialpose_snap_distance_m_)) {
        reset.x = gnss_pose_->x;
        reset.y = gnss_pose_->y;
        reset.cov_x = gnss_pose_->cov_x;
        reset.cov_y = gnss_pose_->cov_y;
        snapped_to_gnss = true;
      }
    }

    if (initialpose_yaw_source_ == "localization") {
      if (localization_pose_ && isFresh(*localization_pose_, localization_timeout_s_)) {
        reset.yaw = localization_pose_->yaw;
      }
    } else if (initialpose_yaw_source_ == "gnss") {
      if (gnss_pose_ && isFresh(*gnss_pose_, gnss_timeout_s_) && gnssYawOk(*gnss_pose_)) {
        reset.yaw = gnss_pose_->yaw;
      }
    }

    publishDirectResetPose(reset, requested.stamp);

    // Reset detachment tracker state so manual reset does not immediately
    // trigger an additional forced reattach cycle before filter output updates.
    detached_ = false;
    motion_condition_met_ = false;
    detached_motion_m_ = 0.0;
    reattach_attempt_count_ = 0;
    localization_pose_ = reset;
    prev_localization_pose_ = reset;
    last_reattach_attempt_ = now();

    RCLCPP_WARN(
      get_logger(),
      "Applied manual initialpose reset: in=(%.2f, %.2f, yaw=%.1fdeg) out=(%.2f, %.2f, yaw=%.1fdeg) "
      "snap_to_gnss=%s gap_to_gnss=%.2fm yaw_source=%s",
      requested.x, requested.y, requested.yaw * 180.0 / M_PI,
      reset.x, reset.y, reset.yaw * 180.0 / M_PI,
      snapped_to_gnss ? "true" : "false",
      gap_to_gnss_m,
      initialpose_yaw_source_.c_str());
  }

  bool gnssCovarianceOk() const
  {
    if (!gnss_pose_) {
      return false;
    }
    if (max_gnss_cov_trace_ <= 0.0) {
      return true;
    }
    const double trace = std::max(0.0, gnss_pose_->cov_x) + std::max(0.0, gnss_pose_->cov_y);
    return trace <= max_gnss_cov_trace_;
  }

  bool gnssYawOk(const PoseSample & gnss) const
  {
    return std::isfinite(gnss.cov_yaw) &&
      gnss.cov_yaw > 0.0 &&
      gnss.cov_yaw <= max_gnss_yaw_covariance_;
  }

  double resetYaw(const PoseSample & localization, const PoseSample & gnss) const
  {
    // HH_260629: During dual-antenna heading loss, GNSS pose keeps position but
    // yaw covariance is intentionally huge. Do not seed EKF yaw from that placeholder.
    return (!use_localization_yaw_ && gnssYawOk(gnss)) ? gnss.yaw : localization.yaw;
  }

  void enterDetachedState(double gap_m)
  {
    detached_ = true;
    detached_since_ = now();
    detached_motion_m_ = 0.0;
    motion_condition_met_ = false;
    reattach_attempt_count_ = 0;
    RCLCPP_WARN(
      get_logger(),
      "GNSS reattach armed: localization/GNSS gap %.2f m exceeded detach_distance_m %.2f m",
      gap_m, detach_distance_m_);
  }

  void clearDetachedState(double gap_m)
  {
    detached_ = false;
    detached_motion_m_ = 0.0;
    motion_condition_met_ = false;
    reattach_attempt_count_ = 0;
    RCLCPP_INFO(
      get_logger(),
      "GNSS reattach disarmed: localization/GNSS gap back to %.2f m",
      gap_m);
  }

  void publishEkfSetPose(const PoseSample & localization, const PoseSample & gnss, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = gnss.x;
    msg.pose.pose.position.y = gnss.y;
    msg.pose.pose.position.z = localization.z;
    setQuaternionFromYaw(msg.pose.pose.orientation, resetYaw(localization, gnss));
    msg.pose.covariance.fill(0.0);
    const double cov_x = std::max(std::max(0.0, gnss.cov_x), min_reset_cov_xy_);
    const double cov_y = std::max(std::max(0.0, gnss.cov_y), min_reset_cov_xy_);
    msg.pose.covariance[0] = cov_x;
    msg.pose.covariance[7] = cov_y;
    msg.pose.covariance[14] = 1.0;
    msg.pose.covariance[21] = 999.0;
    msg.pose.covariance[28] = 999.0;
    msg.pose.covariance[35] = reset_cov_yaw_;
    ekf_set_pose_pub_->publish(msg);
  }

  void publishAvgInitialPose(const PoseSample & localization, const PoseSample & gnss, const rclcpp::Time & stamp)
  {
    avg_msgs::msg::AvgPoseWithCovarianceStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = gnss.x;
    msg.pose.pose.position.y = gnss.y;
    msg.pose.pose.position.z = localization.z;
    // HH_260720 - Fill the generated quaternion directly; no ROS-message alias is used.
    setQuaternionFromYaw(msg.pose.pose.orientation, resetYaw(localization, gnss));
    msg.pose.covariance.fill(0.0);
    const double cov_x = std::max(std::max(0.0, gnss.cov_x), min_reset_cov_xy_);
    const double cov_y = std::max(std::max(0.0, gnss.cov_y), min_reset_cov_xy_);
    msg.pose.covariance[0] = cov_x;
    msg.pose.covariance[7] = cov_y;
    msg.pose.covariance[14] = 1.0;
    msg.pose.covariance[21] = 999.0;
    msg.pose.covariance[28] = 999.0;
    msg.pose.covariance[35] = reset_cov_yaw_;
    avg_pose_reset_pub_->publish(msg);
  }

  void publishDirectResetPose(const PoseSample & target, const rclcpp::Time & stamp)
  {
    const double cov_x = std::max(std::max(0.0, target.cov_x), min_reset_cov_xy_);
    const double cov_y = std::max(std::max(0.0, target.cov_y), min_reset_cov_xy_);

    if (reset_target_mode_ == "ekf_topic" || reset_target_mode_ == "both") {
      geometry_msgs::msg::PoseWithCovarianceStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = "map";
      msg.pose.pose.position.x = target.x;
      msg.pose.pose.position.y = target.y;
      msg.pose.pose.position.z = target.z;
      setQuaternionFromYaw(msg.pose.pose.orientation, target.yaw);
      msg.pose.covariance.fill(0.0);
      msg.pose.covariance[0] = cov_x;
      msg.pose.covariance[7] = cov_y;
      msg.pose.covariance[14] = 1.0;
      msg.pose.covariance[21] = 999.0;
      msg.pose.covariance[28] = 999.0;
      msg.pose.covariance[35] = reset_cov_yaw_;
      ekf_set_pose_pub_->publish(msg);
    }

    if (reset_target_mode_ == "avg_pose_topic" || reset_target_mode_ == "both") {
      avg_msgs::msg::AvgPoseWithCovarianceStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = "map";
      msg.pose.pose.position.x = target.x;
      msg.pose.pose.position.y = target.y;
      msg.pose.pose.position.z = target.z;
      // HH_260720 - Fill the generated quaternion directly; no ROS-message alias is used.
      setQuaternionFromYaw(msg.pose.pose.orientation, target.yaw);
      msg.pose.covariance.fill(0.0);
      msg.pose.covariance[0] = cov_x;
      msg.pose.covariance[7] = cov_y;
      msg.pose.covariance[14] = 1.0;
      msg.pose.covariance[21] = 999.0;
      msg.pose.covariance[28] = 999.0;
      msg.pose.covariance[35] = reset_cov_yaw_;
      avg_pose_reset_pub_->publish(msg);
    }
  }

  void attemptReattach(double gap_m)
  {
    if (!localization_pose_ || !gnss_pose_) {
      return;
    }
    const auto stamp = now();
    if (reset_target_mode_ == "ekf_topic" || reset_target_mode_ == "both") {
      publishEkfSetPose(*localization_pose_, *gnss_pose_, stamp);
    }
    if (reset_target_mode_ == "avg_pose_topic" || reset_target_mode_ == "both") {
      publishAvgInitialPose(*localization_pose_, *gnss_pose_, stamp);
    }
    last_reattach_attempt_ = stamp;
    reattach_attempt_count_++;
    const std::string mode_label = have_mode_ ? std::to_string(latest_mode_value_) : std::string("n/a");

    RCLCPP_WARN(
      get_logger(),
      "Published GNSS reattach reset #%d (gap=%.2f m, mode=%s, target=%s, motion=%.2f m)",
      reattach_attempt_count_,
      gap_m,
      mode_label.c_str(),
      reset_target_mode_.c_str(),
      detached_motion_m_);
  }

  void onTimer()
  {
    if (!localization_pose_ || !gnss_pose_) {
      return;
    }
    if (!isFresh(*localization_pose_, localization_timeout_s_) ||
      !isFresh(*gnss_pose_, gnss_timeout_s_))
    {
      return;
    }
    if (!gnssCovarianceOk()) {
      return;
    }

    const double gap_m = planarDistance(*localization_pose_, *gnss_pose_);
    if (!detached_) {
      if (gap_m >= detach_distance_m_) {
        enterDetachedState(gap_m);
      }
      return;
    }

    if (gap_m <= attach_distance_m_) {
      clearDetachedState(gap_m);
      return;
    }
    if ((now() - detached_since_).seconds() < detach_hold_s_) {
      return;
    }
    if (require_mode_gate_) {
      if (!have_mode_ || latest_mode_value_ < require_mode_at_or_above_) {
        return;
      }
    }
    if (require_motion_after_detach_ && !motion_condition_met_) {
      return;
    }
    if ((now() - last_reattach_attempt_).seconds() < reattach_cooldown_s_) {
      return;
    }

    attemptReattach(gap_m);
  }

  std::string localization_pose_cov_topic_;
  std::string gnss_pose_cov_topic_;
  std::string wheel_odom_topic_;
  std::string localization_mode_topic_;
  std::string ekf_set_pose_topic_;
  std::string avg_pose_reset_topic_;
  std::string reset_target_mode_;
  std::string initialpose_topic_;
  std::string initialpose_topic_fallback_;
  std::string initialpose_yaw_source_;

  bool use_localization_yaw_{true};
  bool require_mode_gate_{false};
  int require_mode_at_or_above_{2};
  bool require_motion_after_detach_{true};
  bool enable_initialpose_reset_{true};
  bool enable_initialpose_fallback_topic_{true};
  bool initialpose_snap_to_gnss_{false};
  double initialpose_snap_distance_m_{1.0};
  double detach_distance_m_{3.0};
  double attach_distance_m_{1.5};
  double detach_hold_s_{0.5};
  double reattach_cooldown_s_{3.0};
  double motion_distance_threshold_m_{0.4};
  double motion_step_clip_m_{5.0};
  double wheel_motion_speed_threshold_mps_{0.12};
  double localization_timeout_s_{1.0};
  double gnss_timeout_s_{2.0};
  double max_gnss_cov_trace_{25.0};
  double max_gnss_yaw_covariance_{100.0};
  double min_reset_cov_xy_{0.05};
  double reset_cov_yaw_{0.5};

  bool detached_{false};
  bool motion_condition_met_{false};
  bool have_mode_{false};
  int latest_mode_value_{0};
  int reattach_attempt_count_{0};
  double detached_motion_m_{0.0};
  rclcpp::Time detached_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_reattach_attempt_{0, 0, RCL_ROS_TIME};
  std::optional<PoseSample> localization_pose_;
  std::optional<PoseSample> prev_localization_pose_;
  std::optional<PoseSample> gnss_pose_;

  rclcpp::Subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>::SharedPtr localization_pose_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>::SharedPtr gnss_pose_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgOdometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgLocalizationMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_fallback_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_set_pose_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgPoseWithCovarianceStamped>::SharedPtr avg_pose_reset_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace camrod::localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod::localization::LocalizationGnssReattachNode>());
  rclcpp::shutdown();
  return 0;
}
