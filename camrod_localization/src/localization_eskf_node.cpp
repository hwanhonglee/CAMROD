#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include <Eigen/Dense>

#include <builtin_interfaces/msg/time.hpp>
#include <avg_msgs/msg/pose_stamped.hpp>
#include <avg_msgs/msg/pose_with_covariance_stamped.hpp>
#include <avg_msgs/msg/transform_stamped.hpp>
#include <avg_msgs/msg/twist_stamped.hpp>
#include <avg_msgs/msg/twist_with_covariance_stamped.hpp>
#include <avg_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <avg_msgs/msg/imu.hpp>
#include <avg_msgs/msg/float32.hpp>

#include <avg_msgs/msg/avg_localization_status_stream.hpp>
#include <avg_msgs/msg/avg_localization_msgs.hpp>
#include <avg_msgs/msg/module_state.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using avg_msgs::msg::AvgLocalizationStatusStream;

namespace
{
std::string normalizeModeToken(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

// HH_260123 Helper to keep timestamped IMU measurement.
struct ImuSample
{
  rclcpp::Time stamp;
  Eigen::Vector3d acc;
  Eigen::Vector3d gyro;
};

// Implements `normalizeYaw` behavior.
double normalizeYaw(double yaw)
{
  const double two_pi = 2.0 * M_PI;
  while (yaw > M_PI) yaw -= two_pi;
  while (yaw < -M_PI) yaw += two_pi;
  return yaw;
}

// Converts radian angle to degrees.
double radToDeg(double rad)
{
  return rad * 180.0 / M_PI;
}

// Implements `yawToQuat` behavior.
tf2::Quaternion yawToQuat(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return q;
}

// Converts quaternion to yaw (rad).
double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

class LocalizationEskfNode : public rclcpp::Node
{
public:
  // Implements `LocalizationEskfNode` behavior.
  LocalizationEskfNode()
  // HH_260123: ESKF with IMU prediction + GNSS/Wheel updates (2D).
  : Node("localization_eskf")
  {
    map_frame_ = declare_parameter<std::string>("map_frame_id", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame_id", "robot_base_link");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/sensing/imu/data");
    gnss_topic_ = declare_parameter<std::string>(
      "gnss_topic", "/sensing/gnss/pose_with_covariance");
    // HH_260410: Keep wheel source naming consistent under /platform/status namespace.
    wheel_topic_ = declare_parameter<std::string>(
      "wheel_topic", "/platform/status/wheel_odometry");
    // HH_260527: Optional external pose reset topic for ESKF.
    // Empty string disables subscriber.
    set_pose_topic_ = declare_parameter<std::string>("set_pose_topic", "");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    // HH_260327: Keep map->odom TF authority in launch-level static publisher to
    // avoid duplicated map->odom broadcasters when switching filters.
    publish_map_to_odom_tf_ = declare_parameter<bool>("publish_map_to_odom_tf", true);
    publish_uninitialized_tf_ = declare_parameter<bool>("publish_uninitialized_tf", false);
    // HH_260415: Stamp TF with node ROS time to avoid TF extrapolation when sensor clocks drift.
    tf_use_node_time_ = declare_parameter<bool>("tf_use_node_time", true);
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/localization/pose");
    pose_cov_topic_ = declare_parameter<std::string>(
      "pose_cov_topic", "/localization/pose_with_covariance");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/localization/odometry/filtered");
    twist_topic_ = declare_parameter<std::string>(
      "twist_topic", "/localization/twist");
    diag_topic_ = declare_parameter<std::string>(
      "diag_topic", "/localization/eskf/status");
    publish_localization_status_ = declare_parameter<bool>("publish_localization_status", false);
    localization_status_topic_ = declare_parameter<std::string>(
      "localization_status_topic", "/localization/status");
    // HH_260413: Separate IMU mount-frame yaw alignment from visualization offset.
    // This parameter corrects filter math (not RViz marker orientation).
    // IMPORTANT: this is the PHYSICAL mount angle used only for acceleration rotation in predict().
    // Do NOT inflate this to fix heading display — use imu_yaw_init_offset_deg for that.
    imu_to_base_yaw_deg_ = declare_parameter<double>("imu_to_base_yaw_deg", 0.0);
    imu_to_base_yaw_rad_ = imu_to_base_yaw_deg_ * M_PI / 180.0;
    // HH_260506: Separate offset added ONLY to heading initialization (not acceleration rotation).
    // CV7 without magnetometer boots at arbitrary yaw=0 reference — this corrects that offset
    // so that IMU yaw + mount_angle + init_offset = correct robot world heading at first GNSS lock.
    // Tune independently: imu_to_base_yaw_deg for stable forward/backward yaw,
    //                     imu_yaw_init_offset_deg for correct initial heading display.
    {
      const double deg = declare_parameter<double>("imu_yaw_init_offset_deg", 0.0);
      imu_yaw_init_offset_rad_ = deg * M_PI / 180.0;
    }
    // HH_260416: IMU sign correction knobs for mirrored axis cases.
    // +1.0 keeps direction, -1.0 inverts direction.
    imu_accel_x_sign_ = declare_parameter<double>("imu_accel_x_sign", 1.0);
    imu_accel_y_sign_ = declare_parameter<double>("imu_accel_y_sign", 1.0);
    imu_gyro_z_sign_ = declare_parameter<double>("imu_gyro_z_sign", 1.0);
    imu_yaw_sign_ = declare_parameter<double>("imu_yaw_sign", 1.0);
    // HH_260526: Replace use_imu_orientation_for_yaw_init toggle with explicit source.
    // yaw_init_source options: imu_orientation | state.
    yaw_init_source_ = normalizeModeToken(
      declare_parameter<std::string>("yaw_init_source", "imu_orientation"));
    // HH_260507: Optional startup diagnostics for IMU yaw -> initial yaw path.
    debug_yaw_init_ = declare_parameter<bool>("debug_yaw_init", false);
    // HH_260507: Apply NHC/ZUPT also on IMU predict loop while stopped.
    // This reduces standstill drift between wheel callback intervals.
    apply_constraints_on_imu_when_stopped_ = declare_parameter<bool>(
      "apply_constraints_on_imu_when_stopped", true);

    gnss_gate_mahalanobis_ = declare_parameter<double>("gnss_gate_mahalanobis", 9.0);
    wheel_gate_mahalanobis_ = declare_parameter<double>("wheel_gate_mahalanobis", 9.0);
    // 2026-04-22: Optional wheel yaw-rate correction using odometry angular.z.
    // Ranger odometry already reflects steering geometry, so this helps keep
    // IMU yaw bias and DR heading consistent with real platform motion.
    // HH_260526: Replace use_wheel_yaw_rate_update with explicit wheel update mode.
    // wheel_yaw_rate_mode options: fused | speed_only.
    wheel_yaw_rate_mode_ = normalizeModeToken(
      declare_parameter<std::string>("wheel_yaw_rate_mode", "fused"));
    wheel_yaw_rate_noise_ = declare_parameter<double>("wheel_yaw_rate_noise", 0.25);
    wheel_yaw_rate_gate_mahalanobis_ = declare_parameter<double>(
      "wheel_yaw_rate_gate_mahalanobis", 9.0);
    wheel_yaw_rate_min_speed_mps_ = declare_parameter<double>("wheel_yaw_rate_min_speed_mps", 0.10);
    // HH_260422: NHC suppresses lateral drift using the no-sideslip constraint.
    // nhc_lateral_noise >0 allows some slip for skid-steer robots like Ranger.
    // HH_260526: Replace use_nhc/use_zupt with explicit per-constraint modes.
    // nhc_mode/zupt_mode options: enabled | disabled.
    nhc_mode_ = normalizeModeToken(declare_parameter<std::string>("nhc_mode", "enabled"));
    nhc_lateral_noise_ = declare_parameter<double>("nhc_lateral_noise", 0.05);
    nhc_gate_mahalanobis_ = declare_parameter<double>("nhc_gate_mahalanobis", 9.0);
    // HH_260422: ZUPT injects v=[0,0] when the robot is confirmed stopped,
    // preventing IMU accel bias from drifting position during standstill.
    zupt_mode_ = normalizeModeToken(declare_parameter<std::string>("zupt_mode", "enabled"));
    zupt_vel_noise_ = declare_parameter<double>("zupt_vel_noise", 0.02);
    zupt_gate_mahalanobis_ = declare_parameter<double>("zupt_gate_mahalanobis", 9.0);
    // 2026-01-30: Initialize state on first GNSS to avoid huge innovation rejection.
    init_on_first_gnss_ = declare_parameter<bool>("init_on_first_gnss", true);
    // 2026-02-02: Allow reinit if GNSS is far from current state (recover from drift).
    reinit_on_gnss_reject_ = declare_parameter<bool>("reinit_on_gnss_reject", true);
    reinit_distance_threshold_ = declare_parameter<double>("reinit_distance_threshold", 50.0);

    gyro_noise_ = declare_parameter<double>("gyro_noise", 0.015);          // rad/s/sqrt(Hz)
    accel_noise_ = declare_parameter<double>("accel_noise", 0.15);         // m/s^2/sqrt(Hz)
    gyro_bias_walk_ = declare_parameter<double>("gyro_bias_walk", 0.0005);
    accel_bias_walk_ = declare_parameter<double>("accel_bias_walk", 0.01);
    gnss_pos_noise_ = declare_parameter<double>("gnss_position_noise", 1.5);   // m 1-sigma
    wheel_speed_noise_ = declare_parameter<double>("wheel_speed_noise", 0.2);  // m/s 1-sigma

    min_imu_dt_ = declare_parameter<double>("min_imu_dt", 1e-4);
    max_imu_dt_ = declare_parameter<double>("max_imu_dt", 0.2);
    // HH_260413: Freeze yaw integration when platform is stopped to reduce gyro drift.
    freeze_yaw_when_stopped_ = declare_parameter<bool>("freeze_yaw_when_stopped", true);
    // HH_260415: When stopped, align filter yaw to IMU absolute orientation instead of freezing.
    align_yaw_to_imu_when_stopped_ = declare_parameter<bool>(
      "align_yaw_to_imu_when_stopped", true);
    stop_speed_threshold_ = declare_parameter<double>("stop_speed_threshold", 0.05);
    stop_hold_s_ = declare_parameter<double>("stop_hold_s", 1.0);

    // HH_260413: GNSS profile auto-switch (normal <-> unstable) for realtime jitter.
    gnss_profile_mode_ = declare_parameter<std::string>("gnss_profile_mode", "auto");
    gnss_profile_switch_reject_count_ = declare_parameter<int>("gnss_profile_switch_reject_count", 5);
    gnss_profile_switch_accept_count_ = declare_parameter<int>("gnss_profile_switch_accept_count", 8);
    gnss_profile_normal_gate_ = declare_parameter<double>(
      "gnss_profile_normal_gate_mahalanobis", gnss_gate_mahalanobis_);
    gnss_profile_unstable_gate_ = declare_parameter<double>(
      "gnss_profile_unstable_gate_mahalanobis", gnss_gate_mahalanobis_);
    gnss_profile_normal_noise_ = declare_parameter<double>(
      "gnss_profile_normal_position_noise", gnss_pos_noise_);
    gnss_profile_unstable_noise_ = declare_parameter<double>(
      "gnss_profile_unstable_position_noise", gnss_pos_noise_);

    initGnssProfile();

    // HH_260506: GNSS COG (Course Over Ground) heading update.
    // Subscribes to ublox fix_velocity (TwistWithCovarianceStamped in ENU) and uses
    // atan2(vel_n, vel_e) as a direct yaw measurement when speed exceeds threshold.
    // Eliminates need for manual imu_yaw_init_offset_deg calibration:
    // heading self-corrects within seconds of first motion.
    gnss_cog_topic_ = declare_parameter<std::string>("gnss_cog_topic", "");
    gnss_cog_min_speed_mps_ = declare_parameter<double>("gnss_cog_min_speed_mps", 0.5);
    gnss_cog_noise_rad_ = declare_parameter<double>("gnss_cog_noise_deg", 15.0) * M_PI / 180.0;
    gnss_cog_gate_mahalanobis_ = declare_parameter<double>("gnss_cog_gate_mahalanobis", 9.0);
    gnss_cog_yaw_offset_rad_ =
      declare_parameter<double>("gnss_cog_yaw_offset_deg", 0.0) * M_PI / 180.0;
    // HH_260508: On first valid COG measurement, hard-reset yaw instead of Kalman update.
    // Eliminates the need for imu_yaw_init_offset_deg calibration — heading self-calibrates
    // from GNSS velocity direction as soon as robot moves at gnss_cog_min_speed_mps.
    gnss_cog_force_init_ = declare_parameter<bool>("gnss_cog_force_init", true);
    // HH_260604: Dual-antenna GNSS heading arrives as yaw in the GNSS pose covariance topic.
    // Keep this separate from COG so stationary heading can be fused when RELPOSNED is valid.
    enable_gnss_pose_heading_ = declare_parameter<bool>("enable_gnss_pose_heading", true);
    gnss_pose_heading_noise_floor_rad_ =
      declare_parameter<double>("gnss_pose_heading_noise_floor_deg", 1.0) * M_PI / 180.0;
    gnss_pose_heading_gate_mahalanobis_ =
      declare_parameter<double>("gnss_pose_heading_gate_mahalanobis", 9.0);
    gnss_pose_heading_max_covariance_ =
      declare_parameter<double>("gnss_pose_heading_max_covariance", 100.0);
    gnss_pose_heading_force_init_ =
      declare_parameter<bool>("gnss_pose_heading_force_init", true);
    if (yaw_init_source_ != "imu_orientation" && yaw_init_source_ != "state") {
      RCLCPP_WARN(
        get_logger(),
        "Invalid yaw_init_source='%s'. Falling back to 'imu_orientation'.",
        yaw_init_source_.c_str());
      yaw_init_source_ = "imu_orientation";
    }
    if (wheel_yaw_rate_mode_ != "fused" && wheel_yaw_rate_mode_ != "speed_only") {
      RCLCPP_WARN(
        get_logger(),
        "Invalid wheel_yaw_rate_mode='%s'. Falling back to 'fused'.",
        wheel_yaw_rate_mode_.c_str());
      wheel_yaw_rate_mode_ = "fused";
    }
    if (nhc_mode_ != "enabled" && nhc_mode_ != "disabled") {
      RCLCPP_WARN(
        get_logger(),
        "Invalid nhc_mode='%s'. Falling back to 'enabled'.",
        nhc_mode_.c_str());
      nhc_mode_ = "enabled";
    }
    if (zupt_mode_ != "enabled" && zupt_mode_ != "disabled") {
      RCLCPP_WARN(
        get_logger(),
        "Invalid zupt_mode='%s'. Falling back to 'enabled'.",
        zupt_mode_.c_str());
      zupt_mode_ = "enabled";
    }

    // Publishers
    odom_pub_ = create_publisher<avg_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
    pose_pub_ = create_publisher<avg_msgs::msg::PoseStamped>(pose_topic_, rclcpp::QoS(10));
    pose_cov_pub_ = create_publisher<avg_msgs::msg::PoseWithCovarianceStamped>(
      pose_cov_topic_, rclcpp::QoS(10));
    twist_pub_ = create_publisher<avg_msgs::msg::TwistStamped>(twist_topic_, rclcpp::QoS(10));
    diag_pub_ = create_publisher<AvgLocalizationStatusStream>(diag_topic_, rclcpp::QoS(10));
    if (publish_localization_status_) {
      avg_localization_pub_ = create_publisher<avg_msgs::msg::AvgLocalizationMsgs>(
        localization_status_topic_, rclcpp::QoS(10));
    }

    // TF broadcaster
    if (publish_tf_) {
      // HH_260123 Avoid shared_from_this() in ctor; use node handle constructor.
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      if (publish_uninitialized_tf_) {
        using namespace std::chrono_literals;
        uninitialized_tf_timer_ = create_wall_timer(
          100ms, std::bind(&LocalizationEskfNode::publishUninitializedTf, this));
      }
    }

    using std::placeholders::_1;
    imu_sub_ = create_subscription<avg_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationEskfNode::onImu, this, _1));
    gnss_sub_ = create_subscription<avg_msgs::msg::PoseWithCovarianceStamped>(
      gnss_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationEskfNode::onGnss, this, _1));
    wheel_sub_ = create_subscription<avg_msgs::msg::Odometry>(
      wheel_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationEskfNode::onWheelOdom, this, _1));
    if (!set_pose_topic_.empty()) {
      set_pose_sub_ = create_subscription<avg_msgs::msg::PoseWithCovarianceStamped>(
        set_pose_topic_, rclcpp::QoS(10),
        std::bind(&LocalizationEskfNode::onSetPose, this, _1));
      RCLCPP_INFO(get_logger(), "ESKF external set_pose enabled: topic=%s", set_pose_topic_.c_str());
    }
    if (!gnss_cog_topic_.empty()) {
      gnss_cog_sub_ = create_subscription<avg_msgs::msg::TwistWithCovarianceStamped>(
        gnss_cog_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LocalizationEskfNode::onGnssCog, this, _1));
      RCLCPP_INFO(get_logger(), "GNSS COG heading enabled: topic=%s min_speed=%.2f m/s",
        gnss_cog_topic_.c_str(), gnss_cog_min_speed_mps_);
    }

    state_.setZero();
    covariance_.setIdentity();
    covariance_ *= 1.0;

    // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
    RCLCPP_DEBUG(get_logger(),
      "ESKF started. imu=%s gnss=%s wheel=%s",
      imu_topic_.c_str(), gnss_topic_.c_str(), wheel_topic_.c_str());
    if (debug_yaw_init_) {
      RCLCPP_WARN(
        get_logger(),
        "Yaw debug ON: yaw_init_source=%s imu_yaw_sign=%.1f "
        "imu_to_base_yaw_deg=%.3f align_yaw_to_imu_when_stopped=%s",
        yaw_init_source_.c_str(),
        imu_yaw_sign_,
        imu_to_base_yaw_deg_,
        align_yaw_to_imu_when_stopped_ ? "true" : "false");
    }
  }

private:
  using Matrix8d = Eigen::Matrix<double, 8, 8>;
  using Vector8d = Eigen::Matrix<double, 8, 1>;

  bool yawInitUsesImuOrientation() const
  {
    return yaw_init_source_ == "imu_orientation";
  }

  bool wheelYawRateFusionEnabled() const
  {
    return wheel_yaw_rate_mode_ == "fused";
  }

  bool nhcEnabled() const
  {
    return nhc_mode_ == "enabled";
  }

  bool zuptEnabled() const
  {
    return zupt_mode_ == "enabled";
  }

  void onSetPose(const avg_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    const double qn = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    const double yaw = (qn > 1e-12) ? normalizeYaw(yawFromQuat(q)) : normalizeYaw(state_(4));

    state_.setZero();
    state_(0) = p.x;
    state_(1) = p.y;
    state_(4) = yaw;

    covariance_.setIdentity();
    covariance_ *= 10.0;
    if (msg->pose.covariance[0] > 0.0) {
      covariance_(0, 0) = msg->pose.covariance[0];
    }
    if (msg->pose.covariance[7] > 0.0) {
      covariance_(1, 1) = msg->pose.covariance[7];
    }
    if (msg->pose.covariance[35] > 0.0) {
      covariance_(4, 4) = msg->pose.covariance[35];
    }

    initialized_ = true;
    wheel_initialized_ = false;
    last_imu_.reset();
    last_pub_stamp_ = rclcpp::Time(msg->header.stamp);

    AvgLocalizationStatusStream diag;
    diag.header.stamp = msg->header.stamp;
    diag.gnss_update_accepted = true;
    diag.gnss_innovation_norm = 0.0;
    diag.wheel_update_accepted = true;
    diag.wheel_innovation_norm = 0.0;
    diag.covariance_trace = covariance_.trace();
    last_diag_ = diag;
    diag_pub_->publish(diag);
    publishOutputs(msg->header.stamp);

    RCLCPP_WARN(
      get_logger(),
      "ESKF external set_pose applied: x=%.2f y=%.2f yaw=%.1f deg",
      state_(0), state_(1), radToDeg(state_(4)));
  }

  // Handles the `onImu` callback.
  void onImu(const avg_msgs::msg::Imu::ConstSharedPtr msg)
  {
    const rclcpp::Time stamp = msg->header.stamp;
    Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, 0.0);
    Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    ImuSample sample{stamp, acc, gyro};

    if (yawInitUsesImuOrientation()) {
      const auto & q = msg->orientation;
      const double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
      if (norm2 > 1e-12) {
        const double inv_norm = 1.0 / std::sqrt(norm2);
        geometry_msgs::msg::Quaternion qn;
        qn.x = q.x * inv_norm;
        qn.y = q.y * inv_norm;
        qn.z = q.z * inv_norm;
        qn.w = q.w * inv_norm;
        const double imu_yaw_raw = normalizeYaw(yawFromQuat(qn));
        last_imu_base_yaw_ = normalizeYaw(
          imu_yaw_sign_ * imu_yaw_raw + imu_to_base_yaw_rad_ + imu_yaw_init_offset_rad_);
        has_imu_base_yaw_ = true;
        if (debug_yaw_init_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Yaw debug IMU: raw=%.2f deg -> corrected=%.2f deg "
            "(sign=%.1f, mount=%.2f deg, init_offset=%.2f deg)",
            radToDeg(imu_yaw_raw),
            radToDeg(last_imu_base_yaw_),
            imu_yaw_sign_,
            imu_to_base_yaw_deg_,
            imu_yaw_init_offset_rad_ * 180.0 / M_PI);
        }
      }
    }

    if (!last_imu_) {
      last_imu_ = sample;
      last_pub_stamp_ = stamp;
      return;
    }

    double dt = (stamp - last_imu_->stamp).seconds();
    if (dt < min_imu_dt_) {
      last_imu_ = sample;
      return;
    }
    if (dt > max_imu_dt_) {
      RCLCPP_WARN(get_logger(), "IMU dt too large (%.3f s); clamping", dt);
      dt = max_imu_dt_;
    }

    predict(sample, dt);
    if (apply_constraints_on_imu_when_stopped_ && is_stopped_) {
      applyNhc();
      applyZupt();
    }
    last_imu_ = sample;
    publishOutputs(stamp);
  }

  // Handles the `onGnss` callback.
  void onGnss(const avg_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    if (!last_imu_) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for IMU before GNSS update");
      return;
    }
    Eigen::Vector2d z(msg->pose.pose.position.x, msg->pose.pose.position.y);
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * (gnss_pos_noise_ * gnss_pos_noise_);
    // Use provided covariance if valid.
    if (msg->pose.covariance[0] > 0.0 && msg->pose.covariance[7] > 0.0) {
      R(0, 0) = msg->pose.covariance[0];
      R(1, 1) = msg->pose.covariance[7];
    }

    double gnss_heading_yaw = 0.0;
    double gnss_heading_var = 0.0;
    const bool has_gnss_heading =
      gnssPoseHeadingMeasurement(*msg, gnss_heading_yaw, gnss_heading_var);

    if (init_on_first_gnss_ && !initialized_) {
      // 2026-01-30: Snap initial state to GNSS position and reset covariance.
      // HH_260604: Seed initial yaw from dual-antenna GNSS pose heading when available.
      const double yaw_seed = has_gnss_heading ? gnss_heading_yaw : yawForInitialization();
      state_.setZero();
      state_(0) = z.x();
      state_(1) = z.y();
      state_(4) = yaw_seed;
      if (debug_yaw_init_) {
        RCLCPP_WARN(
          get_logger(),
          "Yaw debug GNSS init: seeded yaw=%.2f deg from %s",
          radToDeg(yaw_seed),
          has_gnss_heading ? "GNSS pose heading" :
          ((yawInitUsesImuOrientation() && has_imu_base_yaw_) ? "IMU orientation" :
          "state yaw"));
      }

      covariance_.setIdentity();
      covariance_ *= 10.0;
      covariance_(0, 0) = R(0, 0);
      covariance_(1, 1) = R(1, 1);
      if (has_gnss_heading) {
        covariance_(4, 4) = gnss_heading_var;
        gnss_pose_heading_initialized_ = true;
        gnss_cog_heading_initialized_ = true;
      }

      initialized_ = true;
      // HH_260422: Preserve wheel status so monitor's wheel_update_accepted is not zeroed.
      AvgLocalizationStatusStream diag;
      diag.wheel_update_accepted = last_diag_.wheel_update_accepted;
      diag.wheel_innovation_norm = last_diag_.wheel_innovation_norm;
      diag.header.stamp = msg->header.stamp;
      diag.gnss_innovation_norm = 0.0;
      diag.gnss_update_accepted = true;
      diag.covariance_trace = covariance_.trace();
      last_diag_ = diag;
      diag_pub_->publish(diag);
      publishOutputs(msg->header.stamp);
      return;
    }
    Eigen::Matrix<double, 2, 8> H = Eigen::Matrix<double, 2, 8>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    Eigen::Vector2d innov = z - Eigen::Vector2d(state_(0), state_(1));
    const double pos_error = innov.norm();
    Eigen::Matrix2d S = H * covariance_ * H.transpose() + R;
    double mahal = innov.transpose() * S.inverse() * innov;
    // HH_260422: Preserve wheel status so monitor's wheel_update_accepted is not zeroed.
    AvgLocalizationStatusStream diag;
    diag.wheel_update_accepted = last_diag_.wheel_update_accepted;
    diag.wheel_innovation_norm = last_diag_.wheel_innovation_norm;
    diag.header.stamp = msg->header.stamp;
    diag.gnss_innovation_norm = std::sqrt(std::max(0.0, mahal));

    if (mahal > gnss_gate_mahalanobis_) {
      if (reinit_on_gnss_reject_ && pos_error > reinit_distance_threshold_) {
        // 2026-02-02: Hard reset to GNSS if drifted too far.
        // HH_260604: On re-init, prefer dual-antenna GNSS heading when present; otherwise preserve
        // current filter yaw because CV7 quaternion yaw has no absolute reference here.
        const double yaw_seed = has_gnss_heading ? gnss_heading_yaw : normalizeYaw(state_(4));
        state_.setZero();
        state_(0) = z.x();
        state_(1) = z.y();
        state_(4) = yaw_seed;
        if (debug_yaw_init_) {
          RCLCPP_WARN(
            get_logger(),
            "Yaw debug GNSS reinit: seeded yaw=%.2f deg from %s (pos_error=%.2fm)",
            radToDeg(yaw_seed),
            has_gnss_heading ? "GNSS pose heading" : "current filter state",
            pos_error);
        }

        covariance_.setIdentity();
        covariance_ *= 10.0;
        covariance_(0, 0) = R(0, 0);
        covariance_(1, 1) = R(1, 1);
        if (has_gnss_heading) {
          covariance_(4, 4) = gnss_heading_var;
          gnss_pose_heading_initialized_ = true;
          gnss_cog_heading_initialized_ = true;
        }

        initialized_ = true;
        diag.gnss_update_accepted = true;
        diag.covariance_trace = covariance_.trace();
        last_diag_ = diag;
        diag_pub_->publish(diag);
        publishOutputs(msg->header.stamp);
        return;
      }
      diag.gnss_update_accepted = false;
      last_diag_ = diag;
      diag_pub_->publish(diag);
      updateGnssProfile(false);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "GNSS rejected (mahal=%.2f > gate=%.2f)", mahal, gnss_gate_mahalanobis_);
      return;
    }

    Eigen::Matrix<double, 8, 2> K = covariance_ * H.transpose() * S.inverse();
    // HH_260507: Freeze yaw row so GNSS position updates don't rotate heading when stopped.
    if (freeze_yaw_when_stopped_ && is_stopped_) {
      K.row(4).setZero();
    }
    state_ += K * innov;
    state_(4) = normalizeYaw(state_(4));
    applyJosephUpdate<2>(K, H, R);
    // HH_260604: Fuse GNSS pose yaw after the accepted GNSS position update.
    applyGnssPoseHeading(*msg);

    diag.gnss_update_accepted = true;
    diag.covariance_trace = covariance_.trace();
    last_diag_ = diag;
    diag_pub_->publish(diag);
    updateGnssProfile(true);
  }

  // Handles the `onWheelOdom` callback.
  void onWheelOdom(const avg_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (!last_imu_) {
      return;
    }
    last_wheel_odom_time_ = msg->header.stamp;
    const double v_meas = msg->twist.twist.linear.x;
    updateStopState(v_meas, msg->header.stamp);
    Eigen::Matrix<double, 1, 8> H = Eigen::Matrix<double, 1, 8>::Zero();
    const double yaw = state_(4);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    // v_body_x = cos*yaw * vx + sin*yaw * vy
    H(0, 2) = cos_yaw;
    H(0, 3) = sin_yaw;
    H(0, 4) = -sin_yaw * state_(2) + cos_yaw * state_(3);

    // 2026-02-26: Initialize velocity state from first wheel measurement.
    // Without this, filter can stay near zero-velocity while GNSS moves, causing
    // repeated GNSS gate rejections and apparent localization "fly away".
    if (!wheel_initialized_) {
      state_(2) = v_meas * cos_yaw;
      state_(3) = v_meas * sin_yaw;
      covariance_(2, 2) = std::max(covariance_(2, 2), wheel_speed_noise_ * wheel_speed_noise_);
      covariance_(3, 3) = std::max(covariance_(3, 3), wheel_speed_noise_ * wheel_speed_noise_);

      // HH_260422: Preserve GNSS status so monitor's gnss_update_accepted is not zeroed.
      AvgLocalizationStatusStream init_diag;
      init_diag.gnss_update_accepted = last_diag_.gnss_update_accepted;
      init_diag.gnss_innovation_norm = last_diag_.gnss_innovation_norm;
      init_diag.header.stamp = msg->header.stamp;
      init_diag.wheel_innovation_norm = 0.0;
      init_diag.wheel_update_accepted = true;
      init_diag.covariance_trace = covariance_.trace();
      last_diag_ = init_diag;
      diag_pub_->publish(init_diag);
      wheel_initialized_ = true;
      return;
    }

    const double v_pred = cos_yaw * state_(2) + sin_yaw * state_(3);
    const bool can_use_yaw_rate_update =
      wheelYawRateFusionEnabled() &&
      std::isfinite(msg->twist.twist.angular.z) &&
      std::abs(v_meas) >= wheel_yaw_rate_min_speed_mps_;
    const double wheel_yaw_rate_meas = msg->twist.twist.angular.z;
    const double imu_yaw_rate_pred = imu_gyro_z_sign_ * (last_imu_->gyro.z() - state_(5));
    double mahal = 0.0;
    bool accepted = false;

    if (can_use_yaw_rate_update) {
      // 2026-04-22:
      // Measurement vector z = [v_body_x, yaw_rate_from_wheel_odom]
      // h(x) = [R(yaw)*v_world, imu_gyro_z - bias_gz]
      // This lets wheel yaw-rate anchor IMU yaw-bias drift without changing state size.
      Eigen::Matrix<double, 2, 8> H2 = Eigen::Matrix<double, 2, 8>::Zero();
      Eigen::Matrix2d R2 = Eigen::Matrix2d::Zero();
      Eigen::Vector2d innov2 = Eigen::Vector2d::Zero();
      H2.row(0) = H;
      H2(1, 5) = -1.0;
      innov2(0) = v_meas - v_pred;
      innov2(1) = wheel_yaw_rate_meas - imu_yaw_rate_pred;
      R2(0, 0) = wheel_speed_noise_ * wheel_speed_noise_;
      R2(1, 1) = wheel_yaw_rate_noise_ * wheel_yaw_rate_noise_;

      const Eigen::Matrix2d S2 = H2 * covariance_ * H2.transpose() + R2;
      const double det = S2.determinant();
      if (std::isfinite(det) && std::abs(det) > 1e-12) {
        mahal = innov2.transpose() * S2.inverse() * innov2;
      } else {
        mahal = std::numeric_limits<double>::infinity();
      }

      if (mahal <= wheel_yaw_rate_gate_mahalanobis_) {
        const Eigen::Matrix<double, 8, 2> K2 = covariance_ * H2.transpose() * S2.inverse();
        state_ += K2 * innov2;
        state_(4) = normalizeYaw(state_(4));
        applyJosephUpdate<2>(K2, H2, R2);
        accepted = true;
      }
    } else {
      const double innov = v_meas - v_pred;
      const double R_meas = wheel_speed_noise_ * wheel_speed_noise_;
      const double S = (H * covariance_ * H.transpose())(0, 0) + R_meas;
      mahal = (innov * innov) / std::max(1e-6, S);

      if (mahal <= wheel_gate_mahalanobis_) {
        Eigen::Matrix<double, 8, 1> K = covariance_ * H.transpose() * (1.0 / S);
        state_ += K * innov;
        state_(4) = normalizeYaw(state_(4));
        Eigen::Matrix<double, 1, 1> R1; R1 << R_meas;
        applyJosephUpdate<1>(K, H, R1);
        accepted = true;
      }
    }

    // HH_260422: Preserve GNSS status so monitor's gnss_update_accepted is not zeroed.
    AvgLocalizationStatusStream diag;
    diag.gnss_update_accepted = last_diag_.gnss_update_accepted;
    diag.gnss_innovation_norm = last_diag_.gnss_innovation_norm;
    diag.header.stamp = msg->header.stamp;
    diag.wheel_innovation_norm = std::sqrt(std::max(0.0, mahal));
    if (!accepted) {
      diag.wheel_update_accepted = false;
      last_diag_ = diag;
      diag_pub_->publish(diag);
      // HH_260422: Apply kinematic constraints even when wheel speed update is rejected.
      applyNhc();
      applyZupt();
      return;
    }

    diag.wheel_update_accepted = true;
    diag.covariance_trace = covariance_.trace();
    last_diag_ = diag;
    diag_pub_->publish(diag);
    // HH_260422: NHC and ZUPT applied after successful wheel update.
    applyNhc();
    applyZupt();
  }

  // Implements `predict` behavior.
  void predict(const ImuSample & sample, double dt)
  {
    const double yaw = state_(4);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double mount_cos = std::cos(imu_to_base_yaw_rad_);
    const double mount_sin = std::sin(imu_to_base_yaw_rad_);

    // Bias is estimated in raw-sensor space so sign changes don't corrupt the estimate.
    double gyro_z = imu_gyro_z_sign_ * (sample.gyro.z() - state_(5));
    if (freeze_yaw_when_stopped_ && is_stopped_) {
      gyro_z = 0.0;
    }
    const double imu_ax = imu_accel_x_sign_ * (sample.acc.x() - state_(6));
    const double imu_ay = imu_accel_y_sign_ * (sample.acc.y() - state_(7));
    // HH_260413: Rotate IMU XY acceleration into robot-base frame before world projection.
    const double ax = mount_cos * imu_ax - mount_sin * imu_ay;
    const double ay = mount_sin * imu_ax + mount_cos * imu_ay;

    Eigen::Vector2d acc_world;
    acc_world.x() = cos_yaw * ax - sin_yaw * ay;
    acc_world.y() = sin_yaw * ax + cos_yaw * ay;

    Eigen::Vector2d vel_prev(state_(2), state_(3));
    Eigen::Vector2d vel_new = vel_prev + acc_world * dt;
    Eigen::Vector2d pos_new = Eigen::Vector2d(state_(0), state_(1)) +
      vel_prev * dt + 0.5 * acc_world * dt * dt;

    state_(0) = pos_new.x();
    state_(1) = pos_new.y();
    state_(2) = vel_new.x();
    state_(3) = vel_new.y();
    double yaw_new = normalizeYaw(state_(4) + gyro_z * dt);
    if (freeze_yaw_when_stopped_ && is_stopped_ &&
        align_yaw_to_imu_when_stopped_ && has_imu_base_yaw_) {
      // HH_260415: Keep yaw synchronized with IMU quaternion while platform is stationary.
      yaw_new = last_imu_base_yaw_;
    }
    state_(4) = yaw_new;

    // Covariance propagation (simple linearized model).
    Matrix8d F = Matrix8d::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    // Jacobians for bias states: bias is in raw-sensor space, so signs and mount rotation apply.
    Eigen::Matrix2d R_yaw;
    R_yaw << cos_yaw, -sin_yaw,
             sin_yaw, cos_yaw;
    // Mount rotation + accel sign: d(acc_base)/d(accel_bias_raw) = -sign * R_mount
    Eigen::Matrix2d M_bias;
    M_bias << imu_accel_x_sign_ * mount_cos, imu_accel_y_sign_ * (-mount_sin),
              imu_accel_x_sign_ * mount_sin, imu_accel_y_sign_ * mount_cos;
    F.block<2, 2>(2, 6) = -R_yaw * M_bias * dt;
    F(4, 5) = -imu_gyro_z_sign_ * dt;
    F.block<2, 2>(0, 6) = -0.5 * R_yaw * M_bias * dt * dt;

    Matrix8d Q = Matrix8d::Zero();
    const double q_acc = accel_noise_ * accel_noise_;
    const double q_gyro = gyro_noise_ * gyro_noise_;
    const double q_ba = accel_bias_walk_ * accel_bias_walk_;
    const double q_bg = gyro_bias_walk_ * gyro_bias_walk_;
    Q(2, 2) = q_acc * dt;
    Q(3, 3) = q_acc * dt;
    Q(4, 4) = q_gyro * dt;
    Q(5, 5) = q_bg * dt;
    Q(6, 6) = q_ba * dt;
    Q(7, 7) = q_ba * dt;

    covariance_ = F * covariance_ * F.transpose() + Q;
  }

  // Publishes `Outputs` output.
  void publishOutputs(const rclcpp::Time & stamp)
  {
    // Do not publish a floating pose before the first GNSS-based initialization.
    if (init_on_first_gnss_ && !initialized_) {
      return;
    }

    avg_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = state_(0);
    odom.pose.pose.position.y = state_(1);
    odom.pose.pose.position.z = 0.0;
    const auto q = yawToQuat(state_(4));
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = state_(2);
    odom.twist.twist.linear.y = state_(3);
    odom.twist.twist.angular.z =
      last_imu_ ? (imu_gyro_z_sign_ * last_imu_->gyro.z() - state_(5)) : 0.0;

    // Fill 6x6 covariance (position x/y, yaw).
    for (double & c : odom.pose.covariance) c = 0.0;
    for (double & c : odom.twist.covariance) c = 0.0;
    odom.pose.covariance[0] = covariance_(0, 0);
    odom.pose.covariance[7] = covariance_(1, 1);
    odom.pose.covariance[35] = covariance_(4, 4);
    odom.twist.covariance[0] = covariance_(2, 2);
    odom.twist.covariance[7] = covariance_(3, 3);
    odom.twist.covariance[35] = covariance_(4, 4);

    odom_pub_->publish(odom);

    avg_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = odom.header;
    // HH_260125 publish pose in map frame for downstream map-aligned consumers (cost grids).
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = odom.pose.pose;
    pose_pub_->publish(pose_msg);

    avg_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
    pose_cov_msg.header = odom.header;
    // HH_260125 keep covariance pose aligned to map frame for consistency.
    pose_cov_msg.header.frame_id = map_frame_;
    pose_cov_msg.pose = odom.pose;
    pose_cov_pub_->publish(pose_cov_msg);

    avg_msgs::msg::TwistStamped twist_msg;
    twist_msg.header = odom.header;
    twist_msg.twist = odom.twist.twist;
    twist_pub_->publish(twist_msg);

    if (publish_localization_status_ && avg_localization_pub_) {
      avg_msgs::msg::AvgLocalizationMsgs avg_msg;
      avg_msg.stamp = odom.header.stamp;
      avg_msg.localization_odom = odom;
      avg_msg.localization_pose = pose_msg;
      avg_msg.localization_pose_cov = pose_cov_msg;
      avg_msg.localization_twist = twist_msg;
      avg_msg.localization_status_stream = last_diag_;
      avg_msg.gnss_update_accepted = last_diag_.gnss_update_accepted;
      avg_msg.gnss_innovation_norm = last_diag_.gnss_innovation_norm;
      avg_msg.wheel_update_accepted = last_diag_.wheel_update_accepted;
      avg_msg.wheel_innovation_norm = last_diag_.wheel_innovation_norm;
      avg_msg.covariance_trace = static_cast<float>(covariance_.trace());
      avg_msg.state.stamp = odom.header.stamp;
      avg_msg.state.module_name = "localization";
      avg_msg.state.level = initialized_ ?
        avg_msgs::msg::ModuleState::OK :
        avg_msgs::msg::ModuleState::WARN;
      avg_msg.state.message = initialized_ ? "eskf_running" : "eskf_waiting_init";
      avg_localization_pub_->publish(avg_msg);
    }

    last_pub_stamp_ = stamp;

    if (publish_tf_) {
      publishTf(odom);
    }
  }

  // Publishes `Tf` output.
  void publishTf(const avg_msgs::msg::Odometry & odom)
  {
    builtin_interfaces::msg::Time tf_stamp = odom.header.stamp;
    if (tf_use_node_time_) {
      const rclcpp::Time now_time = this->get_clock()->now();
      tf_stamp = now_time;
    }
    // Publish only odom->base by default. map->odom is handled by a single
    // launch-level static publisher for EKF/ESKF consistency.
    avg_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header = odom.header;
    odom_to_base.header.stamp = tf_stamp;
    odom_to_base.child_frame_id = odom.child_frame_id;
    odom_to_base.transform.translation.x = odom.pose.pose.position.x;
    odom_to_base.transform.translation.y = odom.pose.pose.position.y;
    odom_to_base.transform.translation.z = odom.pose.pose.position.z;
    odom_to_base.transform.rotation = odom.pose.pose.orientation;

    std::vector<avg_msgs::msg::TransformStamped> tfs;
    if (publish_map_to_odom_tf_) {
      avg_msgs::msg::TransformStamped map_to_odom;
      map_to_odom.header.stamp = tf_stamp;
      map_to_odom.header.frame_id = map_frame_;
      map_to_odom.child_frame_id = odom_frame_;
      map_to_odom.transform.translation.x = 0.0;
      map_to_odom.transform.translation.y = 0.0;
      map_to_odom.transform.translation.z = 0.0;
      map_to_odom.transform.rotation.w = 1.0;
      tfs.push_back(map_to_odom);
    }
    tfs.push_back(odom_to_base);
    tf_broadcaster_->sendTransform(tfs);
  }

  // Chooses yaw seed for GNSS-based initialization/re-initialization.
  double yawForInitialization() const
  {
    if (yawInitUsesImuOrientation() && has_imu_base_yaw_) {
      return last_imu_base_yaw_;
    }
    return normalizeYaw(state_(4));
  }

  // HH_260422: Non-Holonomic Constraint pseudo-measurement.
  // Applies lateral body velocity = 0 constraint to suppress DR sideways drift.
  // nhc_lateral_noise allows residual slip (important for skid-steer like Ranger).
  void applyNhc()
  {
    if (!nhcEnabled()) {
      return;
    }
    const double yaw = state_(4);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    // h(x) = -sin(yaw)*vx + cos(yaw)*vy  (lateral body-frame velocity)
    const double v_lat = -sin_yaw * state_(2) + cos_yaw * state_(3);
    const double innov = -v_lat;  // z - h(x) = 0 - v_lat

    Eigen::Matrix<double, 1, 8> H_nhc = Eigen::Matrix<double, 1, 8>::Zero();
    H_nhc(0, 2) = -sin_yaw;
    H_nhc(0, 3) = cos_yaw;
    H_nhc(0, 4) = -cos_yaw * state_(2) - sin_yaw * state_(3);

    const double R_nhc = nhc_lateral_noise_ * nhc_lateral_noise_;
    const double S_nhc = (H_nhc * covariance_ * H_nhc.transpose())(0, 0) + R_nhc;
    const double mahal_nhc = (innov * innov) / std::max(1e-12, S_nhc);

    if (mahal_nhc > nhc_gate_mahalanobis_) {
      return;
    }
    Eigen::Matrix<double, 8, 1> K_nhc = covariance_ * H_nhc.transpose() / S_nhc;
    // HH_260507: Don't let NHC update rotate yaw when stopped.
    if (freeze_yaw_when_stopped_ && is_stopped_) {
      K_nhc(4) = 0.0;
    }
    state_ += K_nhc * innov;
    state_(4) = normalizeYaw(state_(4));
    Eigen::Matrix<double, 1, 1> R_nhc_mat; R_nhc_mat << R_nhc;
    applyJosephUpdate<1>(K_nhc, H_nhc, R_nhc_mat);
  }

  // HH_260422: Zero Velocity Update — inject [vx, vy] = [0, 0] when robot is stopped.
  // Prevents IMU accel bias from accumulating into position drift at standstill.
  void applyZupt()
  {
    if (!zuptEnabled() || !is_stopped_) {
      return;
    }
    // h(x) = [vx, vy],  z = [0, 0]
    Eigen::Matrix<double, 2, 8> H_zupt = Eigen::Matrix<double, 2, 8>::Zero();
    H_zupt(0, 2) = 1.0;
    H_zupt(1, 3) = 1.0;

    const Eigen::Vector2d innov_zupt(-state_(2), -state_(3));
    const double R_zupt = zupt_vel_noise_ * zupt_vel_noise_;
    const Eigen::Matrix2d S_zupt =
      H_zupt * covariance_ * H_zupt.transpose() +
      Eigen::Matrix2d::Identity() * R_zupt;

    const double det = S_zupt.determinant();
    if (!std::isfinite(det) || std::abs(det) < 1e-12) {
      return;
    }
    const double mahal_zupt =
      innov_zupt.transpose() * S_zupt.inverse() * innov_zupt;
    if (mahal_zupt > zupt_gate_mahalanobis_) {
      return;
    }
    Eigen::Matrix<double, 8, 2> K_zupt =
      covariance_ * H_zupt.transpose() * S_zupt.inverse();
    // HH_260507: Freeze yaw row so ZUPT covariance cross-terms don't rotate heading.
    if (freeze_yaw_when_stopped_ && is_stopped_) {
      K_zupt.row(4).setZero();
    }
    state_ += K_zupt * innov_zupt;
    state_(4) = normalizeYaw(state_(4));
    Eigen::Matrix2d R_zupt_mat = Eigen::Matrix2d::Identity() * R_zupt;
    applyJosephUpdate<2>(K_zupt, H_zupt, R_zupt_mat);
  }

  // HH_260506: GNSS COG heading callback.
  // Uses ENU velocity (x=East, y=North) from ublox fix_velocity topic to compute
  // course-over-ground heading and apply it as a direct yaw measurement.
  // Replaces manual imu_yaw_init_offset_deg — heading self-calibrates during motion.
  void onGnssCog(
    const avg_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
  {
    if (!initialized_) {
      return;
    }
    // HH_260507: Block COG when wheel odom confirms stopped.
    // HH_260508: If wheel odom is stale (ranger disabled), skip the is_stopped_ check and rely
    // on the speed threshold below to filter standstill GNSS velocity noise.
    const bool wheel_fresh = last_wheel_odom_time_.nanoseconds() > 0 &&
      (this->now() - last_wheel_odom_time_).seconds() < kWheelOdomStaleSec;
    if (wheel_fresh && is_stopped_) {
      return;
    }
    const double vel_e = msg->twist.twist.linear.x;  // East velocity (m/s)
    const double vel_n = msg->twist.twist.linear.y;  // North velocity (m/s)
    const double speed = std::sqrt(vel_e * vel_e + vel_n * vel_n);
    if (speed < gnss_cog_min_speed_mps_) {
      return;
    }
    // COG in ENU: angle from East axis CCW. Same convention as filter yaw.
    const double cog = normalizeYaw(std::atan2(vel_n, vel_e) + gnss_cog_yaw_offset_rad_);
    applyGnssCogHeading(cog, speed, msg->twist.covariance[0]);
  }

  void publishUninitializedTf()
  {
    if (initialized_ || !publish_tf_ || !tf_broadcaster_) {
      if (uninitialized_tf_timer_) {
        uninitialized_tf_timer_->cancel();
      }
      return;
    }

    avg_msgs::msg::Odometry odom;
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = state_(0);
    odom.pose.pose.position.y = state_(1);
    odom.pose.pose.position.z = 0.0;
    const auto q = yawToQuat(yawForInitialization());
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    publishTf(odom);
  }

  // HH_260508: Joseph form P = (I-KH)P(I-KH)^T + K*R*K^T + symmetrize.
  // Maintains positive semi-definiteness even with zero rows in K (yaw freeze).
  // The simple form (I-KH)P drifts negative after repeated asymmetric updates.
  template<int MeasDim>
  void applyJosephUpdate(
    const Eigen::Matrix<double, 8, MeasDim> & K,
    const Eigen::Matrix<double, MeasDim, 8> & H,
    const Eigen::Matrix<double, MeasDim, MeasDim> & R)
  {
    const Matrix8d IKH = Matrix8d::Identity() - K * H;
    covariance_ = IKH * covariance_ * IKH.transpose() + K * R * K.transpose();
    covariance_ = (covariance_ + covariance_.transpose()) * 0.5;
    for (int i = 0; i < 8; i++) {
      covariance_(i, i) = std::max(covariance_(i, i), 1e-9);
    }
  }

  void applyGnssCogHeading(double cog, double speed, double speed_cov_xx)
  {
    // HH_260508: Hard-reset yaw from first valid COG instead of Kalman update.
    // IMU has no absolute heading reference, so the initial yaw seed is arbitrary.
    // First COG measurement gives the ground-truth heading — just set it directly.
    if (gnss_cog_force_init_ && !gnss_cog_heading_initialized_) {
      RCLCPP_INFO(get_logger(),
        "GNSS COG heading init: yaw %.1f deg -> %.1f deg (speed=%.2f m/s)",
        state_(4) * 180.0 / M_PI, cog * 180.0 / M_PI, speed);
      state_(4) = cog;
      covariance_(4, 4) = gnss_cog_noise_rad_ * gnss_cog_noise_rad_;
      gnss_cog_heading_initialized_ = true;
      return;
    }

    const double innov = normalizeYaw(cog - state_(4));

    // Heading noise from velocity noise via error propagation: σ_ψ ≈ σ_v / speed.
    // Use whichever is larger: propagated noise or the fixed minimum.
    double R_cog = gnss_cog_noise_rad_ * gnss_cog_noise_rad_;
    if (speed_cov_xx > 0.0) {
      const double sigma_heading = std::sqrt(speed_cov_xx) / speed;
      R_cog = std::max(R_cog, sigma_heading * sigma_heading);
    }

    const double S = covariance_(4, 4) + R_cog;
    const double mahal = (innov * innov) / std::max(1e-12, S);
    if (mahal > gnss_cog_gate_mahalanobis_) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
        "GNSS COG heading rejected (mahal=%.2f > gate=%.2f, cog=%.1f deg)",
        mahal, gnss_cog_gate_mahalanobis_, cog * 180.0 / M_PI);
      return;
    }

    Eigen::Matrix<double, 1, 8> H = Eigen::Matrix<double, 1, 8>::Zero();
    H(0, 4) = 1.0;
    const Eigen::Matrix<double, 8, 1> K = covariance_ * H.transpose() / S;
    state_ += K * innov;
    state_(4) = normalizeYaw(state_(4));
    Eigen::Matrix<double, 1, 1> R_cog_mat; R_cog_mat << R_cog;
    applyJosephUpdate<1>(K, H, R_cog_mat);
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
      "GNSS COG heading applied: cog=%.1f deg speed=%.2f m/s innov=%.1f deg",
      cog * 180.0 / M_PI, speed, innov * 180.0 / M_PI);
  }

  // HH_260604: Extract a valid yaw measurement from GNSS pose orientation and covariance.
  bool gnssPoseHeadingMeasurement(
    const avg_msgs::msg::PoseWithCovarianceStamped & msg,
    double & yaw,
    double & yaw_variance) const
  {
    if (!enable_gnss_pose_heading_) {
      return false;
    }
    const auto & q = msg.pose.pose.orientation;
    const double q_norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(q_norm_sq) || q_norm_sq <= 1e-12) {
      return false;
    }
    const double raw_var = msg.pose.covariance[35];
    if (!std::isfinite(raw_var) || raw_var <= 0.0 ||
        raw_var > gnss_pose_heading_max_covariance_) {
      return false;
    }
    yaw = normalizeYaw(yawFromQuat(q));
    yaw_variance = std::max(
      raw_var,
      gnss_pose_heading_noise_floor_rad_ * gnss_pose_heading_noise_floor_rad_);
    return true;
  }

  // HH_260604: Apply GNSS pose yaw as a one-dimensional heading update.
  void applyGnssPoseHeading(const avg_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    if (!initialized_) {
      return;
    }

    double yaw = 0.0;
    double yaw_variance = 0.0;
    if (!gnssPoseHeadingMeasurement(msg, yaw, yaw_variance)) {
      return;
    }

    if (gnss_pose_heading_force_init_ && !gnss_pose_heading_initialized_) {
      RCLCPP_INFO(
        get_logger(),
        "GNSS pose heading init: yaw %.1f deg -> %.1f deg",
        state_(4) * 180.0 / M_PI, yaw * 180.0 / M_PI);
      state_(4) = yaw;
      covariance_(4, 4) = yaw_variance;
      gnss_pose_heading_initialized_ = true;
      gnss_cog_heading_initialized_ = true;
      return;
    }

    const double innov = normalizeYaw(yaw - state_(4));
    const double S = covariance_(4, 4) + yaw_variance;
    const double mahal = (innov * innov) / std::max(1e-12, S);
    if (mahal > gnss_pose_heading_gate_mahalanobis_) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "GNSS pose heading rejected (mahal=%.2f > gate=%.2f, yaw=%.1f deg)",
        mahal, gnss_pose_heading_gate_mahalanobis_, yaw * 180.0 / M_PI);
      return;
    }

    Eigen::Matrix<double, 1, 8> H = Eigen::Matrix<double, 1, 8>::Zero();
    H(0, 4) = 1.0;
    const Eigen::Matrix<double, 8, 1> K = covariance_ * H.transpose() / S;
    state_ += K * innov;
    state_(4) = normalizeYaw(state_(4));
    Eigen::Matrix<double, 1, 1> R_heading; R_heading << yaw_variance;
    applyJosephUpdate<1>(K, H, R_heading);
    gnss_pose_heading_initialized_ = true;
    gnss_cog_heading_initialized_ = true;
  }

  // Updates stop-state based on wheel speed so we can clamp yaw drift at standstill.
  void updateStopState(double v_meas, const rclcpp::Time & stamp)
  {
    if (std::abs(v_meas) >= stop_speed_threshold_) {
      last_motion_time_ = stamp;
      is_stopped_ = false;
      return;
    }
    const double stopped_for = (stamp - last_motion_time_).seconds();
    if (stopped_for >= stop_hold_s_) {
      is_stopped_ = true;
    }
  }

  enum class GnssProfile
  {
    kNormal,
    kUnstable
  };

  // Initializes GNSS profile selection (normal/unstable/auto).
  void initGnssProfile()
  {
    if (gnss_profile_mode_ == "normal") {
      applyGnssProfile(GnssProfile::kNormal, "manual_normal");
      return;
    }
    if (gnss_profile_mode_ == "unstable") {
      applyGnssProfile(GnssProfile::kUnstable, "manual_unstable");
      return;
    }
    applyGnssProfile(GnssProfile::kNormal, "auto_init");
  }

  // Applies GNSS profile parameters for gate/noise tuning.
  void applyGnssProfile(GnssProfile profile, const std::string & reason)
  {
    if (gnss_profile_ == profile) {
      return;
    }
    gnss_profile_ = profile;
    if (profile == GnssProfile::kNormal) {
      gnss_gate_mahalanobis_ = gnss_profile_normal_gate_;
      gnss_pos_noise_ = gnss_profile_normal_noise_;
    } else {
      gnss_gate_mahalanobis_ = gnss_profile_unstable_gate_;
      gnss_pos_noise_ = gnss_profile_unstable_noise_;
    }
    // HH_260413: Log live GNSS profile changes for debugging in real-sensor runs.
    RCLCPP_INFO(
      get_logger(),
      "GNSS profile switched to %s (%s): gate=%.2f noise=%.2f",
      (profile == GnssProfile::kNormal) ? "normal" : "unstable",
      reason.c_str(), gnss_gate_mahalanobis_, gnss_pos_noise_);
  }

  // Updates GNSS profile based on consecutive accept/reject streaks.
  void updateGnssProfile(bool accepted)
  {
    if (gnss_profile_mode_ != "auto") {
      return;
    }
    if (accepted) {
      gnss_accept_streak_++;
      gnss_reject_streak_ = 0;
      if (gnss_profile_ == GnssProfile::kUnstable &&
          gnss_accept_streak_ >= gnss_profile_switch_accept_count_) {
        applyGnssProfile(GnssProfile::kNormal, "auto_accept_streak");
      }
      return;
    }
    gnss_reject_streak_++;
    gnss_accept_streak_ = 0;
    if (gnss_profile_ == GnssProfile::kNormal &&
        gnss_reject_streak_ >= gnss_profile_switch_reject_count_) {
      applyGnssProfile(GnssProfile::kUnstable, "auto_reject_streak");
    }
  }

  // Parameters / topics
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_topic_;
  std::string gnss_topic_;
  std::string wheel_topic_;
  std::string set_pose_topic_;
  std::string pose_topic_;
  std::string pose_cov_topic_;
  std::string odom_topic_;
  std::string twist_topic_;
  std::string diag_topic_;
  std::string localization_status_topic_;
  bool publish_tf_{true};                   // HH_260422: true -> broadcast odom->base TF (required by nav2 and rviz)
  bool publish_map_to_odom_tf_{true};       // HH_260422: true -> also broadcast map->odom TF (replaces static publisher)
  bool publish_uninitialized_tf_{false};
  bool tf_use_node_time_{true};             // HH_260422: true -> stamp TF with node ROS time, not sensor stamp (prevents TF extrapolation)
  bool publish_localization_status_{false}; // HH_260422: true -> also publish /localization/status (AvgLocalizationMsgs)
  double imu_to_base_yaw_deg_{0.0};
  double imu_to_base_yaw_rad_{0.0};
  double imu_yaw_init_offset_rad_{0.0};  // heading-init only offset (not applied in predict())
  std::string gnss_cog_topic_;
  double gnss_cog_min_speed_mps_{0.5};
  double gnss_cog_noise_rad_{0.262};  // ~15 deg default
  double gnss_cog_gate_mahalanobis_{9.0};
  double gnss_cog_yaw_offset_rad_{0.0};
  // HH_260508: force-init from first COG — eliminates imu_yaw_init_offset_deg calibration.
  bool gnss_cog_force_init_{true};
  bool gnss_cog_heading_initialized_{false};
  rclcpp::Subscription<avg_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gnss_cog_sub_;
  // HH_260604: Track dual-antenna GNSS pose heading fusion state.
  bool enable_gnss_pose_heading_{true};
  double gnss_pose_heading_noise_floor_rad_{0.017};
  double gnss_pose_heading_gate_mahalanobis_{9.0};
  double gnss_pose_heading_max_covariance_{100.0};
  bool gnss_pose_heading_force_init_{true};
  bool gnss_pose_heading_initialized_{false};
  double imu_accel_x_sign_{1.0};
  double imu_accel_y_sign_{1.0};
  double imu_gyro_z_sign_{1.0};
  double imu_yaw_sign_{1.0};
  // HH_260526: yaw_init_source controls initialization heading source (imu_orientation|state).
  std::string yaw_init_source_{"imu_orientation"};
  // HH_260507: true -> print IMU->yaw seed path for startup/debug diagnosis.
  bool debug_yaw_init_{false};
  // HH_260507: true -> run NHC/ZUPT on IMU callbacks while stop-state is active.
  bool apply_constraints_on_imu_when_stopped_{true};

  double gnss_gate_mahalanobis_{9.0};
  // HH_260422: true -> hard-reinit filter to GNSS when position error exceeds reinit_distance_threshold_
  bool reinit_on_gnss_reject_{true};
  double reinit_distance_threshold_{50.0};
  double wheel_gate_mahalanobis_{9.0};
  // HH_260526: wheel_yaw_rate_mode controls wheel update model (fused|speed_only).
  std::string wheel_yaw_rate_mode_{"fused"};
  double wheel_yaw_rate_noise_{0.25};
  double wheel_yaw_rate_gate_mahalanobis_{9.0};
  double wheel_yaw_rate_min_speed_mps_{0.10};
  // HH_260526: nhc_mode controls NHC pseudo-measurement (enabled|disabled).
  std::string nhc_mode_{"enabled"};
  double nhc_lateral_noise_{0.05};
  double nhc_gate_mahalanobis_{9.0};
  // HH_260526: zupt_mode controls zero-velocity update (enabled|disabled).
  std::string zupt_mode_{"enabled"};
  double zupt_vel_noise_{0.02};
  double zupt_gate_mahalanobis_{9.0};
  // HH_260422: true -> suppress all pose/odom/TF output until first GNSS snap completes (avoids publishing floating initial state)
  bool init_on_first_gnss_{true};
  double gyro_noise_{0.015};
  double accel_noise_{0.15};
  double gyro_bias_walk_{0.0005};
  double accel_bias_walk_{0.01};
  double gnss_pos_noise_{1.5};
  double wheel_speed_noise_{0.2};
  double min_imu_dt_{1e-4};
  double max_imu_dt_{0.2};
  // HH_260422: true -> halt gyro integration in predict() while stopped (prevents gyro drift accumulation)
  bool freeze_yaw_when_stopped_{true};
  // HH_260422: true -> pin yaw to IMU quaternion while stopped (overrides frozen integration with absolute heading)
  bool align_yaw_to_imu_when_stopped_{true};
  double stop_speed_threshold_{0.05};
  double stop_hold_s_{1.0};
  // HH_260422: is_stopped_ becomes true when wheel speed < stop_speed_threshold_ for >= stop_hold_s_ seconds.
  //   When true: freeze_yaw_when_stopped, align_yaw_to_imu_when_stopped, and applyZupt() all activate.
  //   Clears immediately when speed >= stop_speed_threshold_ (no hold-off on motion resume).
  // HH_260508: Default false — wheel odom staleness check in onGnssCog() already blocks COG
  //   when wheel odom hasn't arrived (wheel_age > kWheelOdomStaleSec). Keeping true here
  //   caused permanent yaw freeze when ranger driver is disabled (no wheel odom ever arrives).
  bool is_stopped_{false};
  rclcpp::Time last_motion_time_{0, 0, RCL_ROS_TIME};
  // HH_260507: Track last wheel odom arrival; treat as stopped if stale > 1s
  //   (covers the case where ranger driver is disabled or wheel odom drops out).
  rclcpp::Time last_wheel_odom_time_{0, 0, RCL_ROS_TIME};
  static constexpr double kWheelOdomStaleSec = 1.0;

  std::string gnss_profile_mode_{"auto"};
  int gnss_profile_switch_reject_count_{5};
  int gnss_profile_switch_accept_count_{8};
  double gnss_profile_normal_gate_{9.0};
  double gnss_profile_unstable_gate_{9.0};
  double gnss_profile_normal_noise_{1.5};
  double gnss_profile_unstable_noise_{1.5};
  int gnss_reject_streak_{0};
  int gnss_accept_streak_{0};
  GnssProfile gnss_profile_{GnssProfile::kNormal};

  // State: [px, py, vx, vy, yaw, b_gz, b_ax, b_ay]
  Vector8d state_;
  Matrix8d covariance_;
  std::optional<ImuSample> last_imu_;
  rclcpp::Time last_pub_stamp_;

  // ROS interfaces
  rclcpp::Subscription<avg_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<avg_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_sub_;
  rclcpp::Subscription<avg_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<avg_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_pose_sub_;
  rclcpp::Publisher<avg_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<avg_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<avg_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Publisher<avg_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<AvgLocalizationStatusStream>::SharedPtr diag_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationMsgs>::SharedPtr avg_localization_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr uninitialized_tf_timer_;

  AvgLocalizationStatusStream last_diag_;
  // HH_260422: initialized_ becomes true on first GNSS snap (or at startup if init_on_first_gnss_=false).
  //   While false: publishOutputs() suppresses all pose/odom/TF output.
  bool initialized_{false};
  // HH_260422: wheel_initialized_ becomes true on the first wheel odometry message.
  //   On that message vx/vy are seeded directly from wheel speed to prevent GNSS innovation
  //   gate rejections caused by a near-zero velocity state while the robot is already moving.
  bool wheel_initialized_{false};
  // HH_260422: has_imu_base_yaw_ becomes true once a valid IMU orientation quaternion is received.
  //   While false: yawForInitialization() returns current state_(4) instead of IMU heading.
  //   When true: GNSS init/reinit uses last_imu_base_yaw_, and align_yaw_to_imu_when_stopped is permitted.
  bool has_imu_base_yaw_{false};
  double last_imu_base_yaw_{0.0};
};

// Entry point for this executable.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationEskfNode>());
  rclcpp::shutdown();
  return 0;
}
