#include <chrono>
#include <cmath>
#include <optional>
#include <string>

#include <Eigen/Dense>

#include <builtin_interfaces/msg/time.hpp>
#include <avg_msgs/msg/pose_stamped.hpp>
#include <avg_msgs/msg/pose_with_covariance_stamped.hpp>
#include <avg_msgs/msg/transform_stamped.hpp>
#include <avg_msgs/msg/twist_stamped.hpp>
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
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    // HH_260327: Keep map->odom TF authority in launch-level static publisher to
    // avoid duplicated map->odom broadcasters when switching filters.
    publish_map_to_odom_tf_ = declare_parameter<bool>("publish_map_to_odom_tf", true);
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
    imu_to_base_yaw_deg_ = declare_parameter<double>("imu_to_base_yaw_deg", 0.0);
    imu_to_base_yaw_rad_ = imu_to_base_yaw_deg_ * M_PI / 180.0;
    // HH_260416: IMU sign correction knobs for mirrored axis cases.
    // +1.0 keeps direction, -1.0 inverts direction.
    imu_accel_x_sign_ = declare_parameter<double>("imu_accel_x_sign", 1.0);
    imu_accel_y_sign_ = declare_parameter<double>("imu_accel_y_sign", 1.0);
    imu_gyro_z_sign_ = declare_parameter<double>("imu_gyro_z_sign", 1.0);
    imu_yaw_sign_ = declare_parameter<double>("imu_yaw_sign", 1.0);
    use_imu_orientation_for_yaw_init_ = declare_parameter<bool>(
      "use_imu_orientation_for_yaw_init", true);

    gnss_gate_mahalanobis_ = declare_parameter<double>("gnss_gate_mahalanobis", 9.0);
    wheel_gate_mahalanobis_ = declare_parameter<double>("wheel_gate_mahalanobis", 9.0);
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
    stop_hold_sec_ = declare_parameter<double>("stop_hold_sec", 1.0);

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

    state_.setZero();
    covariance_.setIdentity();
    covariance_ *= 1.0;

    // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
    RCLCPP_DEBUG(get_logger(),
      "ESKF started. imu=%s gnss=%s wheel=%s",
      imu_topic_.c_str(), gnss_topic_.c_str(), wheel_topic_.c_str());
  }

private:
  using Matrix8d = Eigen::Matrix<double, 8, 8>;
  using Vector8d = Eigen::Matrix<double, 8, 1>;

  // Handles the `onImu` callback.
  void onImu(const avg_msgs::msg::Imu::ConstSharedPtr msg)
  {
    const rclcpp::Time stamp = msg->header.stamp;
    Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, 0.0);
    Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    ImuSample sample{stamp, acc, gyro};

    if (use_imu_orientation_for_yaw_init_) {
      const auto & q = msg->orientation;
      const double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
      if (norm2 > 1e-12) {
        const double inv_norm = 1.0 / std::sqrt(norm2);
        geometry_msgs::msg::Quaternion qn;
        qn.x = q.x * inv_norm;
        qn.y = q.y * inv_norm;
        qn.z = q.z * inv_norm;
        qn.w = q.w * inv_norm;
        last_imu_base_yaw_ = normalizeYaw(
          imu_yaw_sign_ * yawFromQuat(qn) + imu_to_base_yaw_rad_);
        has_imu_base_yaw_ = true;
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

    if (init_on_first_gnss_ && !initialized_) {
      // 2026-01-30: Snap initial state to GNSS position and reset covariance.
      state_.setZero();
      state_(0) = z.x();
      state_(1) = z.y();
      state_(4) = yawForInitialization();

      covariance_.setIdentity();
      covariance_ *= 10.0;
      covariance_(0, 0) = R(0, 0);
      covariance_(1, 1) = R(1, 1);

      initialized_ = true;
      AvgLocalizationStatusStream diag;
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
    AvgLocalizationStatusStream diag;
    diag.header.stamp = msg->header.stamp;
    diag.gnss_innovation_norm = std::sqrt(std::max(0.0, mahal));

    if (mahal > gnss_gate_mahalanobis_) {
      if (reinit_on_gnss_reject_ && pos_error > reinit_distance_threshold_) {
        // 2026-02-02: Hard reset to GNSS if drifted too far.
        state_.setZero();
        state_(0) = z.x();
        state_(1) = z.y();
        state_(4) = yawForInitialization();

        covariance_.setIdentity();
        covariance_ *= 10.0;
        covariance_(0, 0) = R(0, 0);
        covariance_(1, 1) = R(1, 1);

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
    Vector8d dx = K * innov;
    state_ += dx;
    state_(4) = normalizeYaw(state_(4));
    Eigen::Matrix2d I2 = Eigen::Matrix2d::Identity();
    covariance_ = (Matrix8d::Identity() - K * H) * covariance_;

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

      AvgLocalizationStatusStream init_diag;
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
    const double innov = v_meas - v_pred;
    const double R_meas = wheel_speed_noise_ * wheel_speed_noise_;
    const double S = (H * covariance_ * H.transpose())(0, 0) + R_meas;
    const double mahal = (innov * innov) / std::max(1e-6, S);

    AvgLocalizationStatusStream diag;
    diag.header.stamp = msg->header.stamp;
    diag.wheel_innovation_norm = std::sqrt(std::max(0.0, mahal));
    if (mahal > wheel_gate_mahalanobis_) {
      diag.wheel_update_accepted = false;
      last_diag_ = diag;
      diag_pub_->publish(diag);
      return;
    }

    Eigen::Matrix<double, 8, 1> K = covariance_ * H.transpose() * (1.0 / S);
    state_ += K * innov;
    state_(4) = normalizeYaw(state_(4));
    covariance_ = (Matrix8d::Identity() - K * H) * covariance_;

    diag.wheel_update_accepted = true;
    diag.covariance_trace = covariance_.trace();
    last_diag_ = diag;
    diag_pub_->publish(diag);
  }

  // Implements `predict` behavior.
  void predict(const ImuSample & sample, double dt)
  {
    const double yaw = state_(4);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double mount_cos = std::cos(imu_to_base_yaw_rad_);
    const double mount_sin = std::sin(imu_to_base_yaw_rad_);

    double gyro_z = imu_gyro_z_sign_ * sample.gyro.z() - state_(5);
    if (freeze_yaw_when_stopped_ && is_stopped_) {
      gyro_z = 0.0;
    }
    const double imu_ax = imu_accel_x_sign_ * sample.acc.x() - state_(6);
    const double imu_ay = imu_accel_y_sign_ * sample.acc.y() - state_(7);
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

    // Position sensitivity to accel bias
    Eigen::Matrix2d R_yaw;
    R_yaw << cos_yaw, -sin_yaw,
             sin_yaw, cos_yaw;
    F.block<2, 2>(2, 6) = -R_yaw * dt;
    F(4, 5) = -dt;
    F.block<2, 2>(0, 6) = -0.5 * R_yaw * dt * dt;

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
    if (use_imu_orientation_for_yaw_init_ && has_imu_base_yaw_) {
      return last_imu_base_yaw_;
    }
    return normalizeYaw(state_(4));
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
    if (stopped_for >= stop_hold_sec_) {
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
  std::string pose_topic_;
  std::string pose_cov_topic_;
  std::string odom_topic_;
  std::string twist_topic_;
  std::string diag_topic_;
  std::string localization_status_topic_;
  bool publish_tf_{true};
  bool publish_map_to_odom_tf_{true};
  bool tf_use_node_time_{true};
  bool publish_localization_status_{false};
  double imu_to_base_yaw_deg_{0.0};
  double imu_to_base_yaw_rad_{0.0};
  double imu_accel_x_sign_{1.0};
  double imu_accel_y_sign_{1.0};
  double imu_gyro_z_sign_{1.0};
  double imu_yaw_sign_{1.0};
  bool use_imu_orientation_for_yaw_init_{true};

  double gnss_gate_mahalanobis_{9.0};
  bool reinit_on_gnss_reject_{true};
  double reinit_distance_threshold_{50.0};
  double wheel_gate_mahalanobis_{9.0};
  bool init_on_first_gnss_{true};
  double gyro_noise_{0.015};
  double accel_noise_{0.15};
  double gyro_bias_walk_{0.0005};
  double accel_bias_walk_{0.01};
  double gnss_pos_noise_{1.5};
  double wheel_speed_noise_{0.2};
  double min_imu_dt_{1e-4};
  double max_imu_dt_{0.2};
  bool freeze_yaw_when_stopped_{true};
  bool align_yaw_to_imu_when_stopped_{true};
  double stop_speed_threshold_{0.05};
  double stop_hold_sec_{1.0};
  bool is_stopped_{false};
  rclcpp::Time last_motion_time_{0, 0, RCL_ROS_TIME};

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
  rclcpp::Publisher<avg_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<avg_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<avg_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Publisher<avg_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<AvgLocalizationStatusStream>::SharedPtr diag_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationMsgs>::SharedPtr avg_localization_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  AvgLocalizationStatusStream last_diag_;
  bool initialized_{false};
  bool wheel_initialized_{false};
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
