#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Dense>

#include "camping_cart_localization/msg/localization_diagnostics.hpp"

#include <chrono>
#include <string>
#include <optional>

using camping_cart_localization::msg::LocalizationDiagnostics;

namespace
{
// HH_260123 Helper to keep timestamped IMU measurement.
struct ImuSample
{
  rclcpp::Time stamp;
  Eigen::Vector3d acc;
  Eigen::Vector3d gyro;
};

double normalizeYaw(double yaw)
{
  const double two_pi = 2.0 * M_PI;
  while (yaw > M_PI) yaw -= two_pi;
  while (yaw < -M_PI) yaw += two_pi;
  return yaw;
}

tf2::Quaternion yawToQuat(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return q;
}
}  // namespace

class LocalizationEskfNode : public rclcpp::Node
{
public:
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
    wheel_topic_ = declare_parameter<std::string>(
      "wheel_topic", "/platform/wheel/odometry");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/localization/pose");
    pose_cov_topic_ = declare_parameter<std::string>(
      "pose_cov_topic", "/localization/pose_with_covariance");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/localization/odometry/filtered");
    twist_topic_ = declare_parameter<std::string>(
      "twist_topic", "/localization/twist");
    diag_topic_ = declare_parameter<std::string>(
      "diag_topic", "/localization/eskf/diagnostics");

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

    // Publishers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, rclcpp::QoS(10));
    pose_cov_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_cov_topic_, rclcpp::QoS(10));
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_, rclcpp::QoS(10));
    diag_pub_ = create_publisher<LocalizationDiagnostics>(diag_topic_, rclcpp::QoS(10));

    // TF broadcaster
    if (publish_tf_) {
      // HH_260123 Avoid shared_from_this() in ctor; use node handle constructor.
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    using std::placeholders::_1;
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationEskfNode::onImu, this, _1));
    gnss_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      gnss_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationEskfNode::onGnss, this, _1));
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
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

  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    const rclcpp::Time stamp = msg->header.stamp;
    Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, 0.0);
    Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    ImuSample sample{stamp, acc, gyro};

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

  void onGnss(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
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
      state_(4) = normalizeYaw(state_(4));

      covariance_.setIdentity();
      covariance_ *= 10.0;
      covariance_(0, 0) = R(0, 0);
      covariance_(1, 1) = R(1, 1);

      initialized_ = true;
      LocalizationDiagnostics diag;
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
    LocalizationDiagnostics diag;
    diag.header.stamp = msg->header.stamp;
    diag.gnss_innovation_norm = std::sqrt(std::max(0.0, mahal));

    if (mahal > gnss_gate_mahalanobis_) {
      if (reinit_on_gnss_reject_ && pos_error > reinit_distance_threshold_) {
        // 2026-02-02: Hard reset to GNSS if drifted too far.
        state_.setZero();
        state_(0) = z.x();
        state_(1) = z.y();
        state_(4) = normalizeYaw(state_(4));

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
  }

  void onWheelOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (!last_imu_) {
      return;
    }
    const double v_meas = msg->twist.twist.linear.x;
    Eigen::Matrix<double, 1, 8> H = Eigen::Matrix<double, 1, 8>::Zero();
    const double yaw = state_(4);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    // v_body_x = cos*yaw * vx + sin*yaw * vy
    H(0, 2) = cos_yaw;
    H(0, 3) = sin_yaw;
    H(0, 4) = -sin_yaw * state_(2) + cos_yaw * state_(3);

    const double v_pred = cos_yaw * state_(2) + sin_yaw * state_(3);
    const double innov = v_meas - v_pred;
    const double R_meas = wheel_speed_noise_ * wheel_speed_noise_;
    const double S = (H * covariance_ * H.transpose())(0, 0) + R_meas;
    const double mahal = (innov * innov) / std::max(1e-6, S);

    LocalizationDiagnostics diag;
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

  void predict(const ImuSample & sample, double dt)
  {
    const double yaw = state_(4);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    const double gyro_z = sample.gyro.z() - state_(5);
    const double ax = sample.acc.x() - state_(6);
    const double ay = sample.acc.y() - state_(7);

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
    state_(4) = normalizeYaw(state_(4) + gyro_z * dt);

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

  void publishOutputs(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry odom;
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
    odom.twist.twist.angular.z = last_imu_ ? (last_imu_->gyro.z() - state_(5)) : 0.0;

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

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = odom.header;
    // HH_260125 publish pose in map frame for downstream map-aligned consumers (cost grids).
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = odom.pose.pose;
    pose_pub_->publish(pose_msg);

    geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
    pose_cov_msg.header = odom.header;
    // HH_260125 keep covariance pose aligned to map frame for consistency.
    pose_cov_msg.header.frame_id = map_frame_;
    pose_cov_msg.pose = odom.pose;
    pose_cov_pub_->publish(pose_cov_msg);

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header = odom.header;
    twist_msg.twist = odom.twist.twist;
    twist_pub_->publish(twist_msg);

    last_pub_stamp_ = stamp;

    if (publish_tf_) {
      publishTf(odom);
    }
  }

  void publishTf(const nav_msgs::msg::Odometry & odom)
  {
    // map->odom (identity) and odom->base_link
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = odom.header.stamp;
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.child_frame_id = odom_frame_;
    map_to_odom.transform.translation.x = 0.0;
    map_to_odom.transform.translation.y = 0.0;
    map_to_odom.transform.translation.z = 0.0;
    map_to_odom.transform.rotation.w = 1.0;

    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header = odom.header;
    odom_to_base.child_frame_id = odom.child_frame_id;
    odom_to_base.transform.translation.x = odom.pose.pose.position.x;
    odom_to_base.transform.translation.y = odom.pose.pose.position.y;
    odom_to_base.transform.translation.z = odom.pose.pose.position.z;
    odom_to_base.transform.rotation = odom.pose.pose.orientation;

    std::vector<geometry_msgs::msg::TransformStamped> tfs{map_to_odom, odom_to_base};
    tf_broadcaster_->sendTransform(tfs);
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
  bool publish_tf_{true};

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

  // State: [px, py, vx, vy, yaw, b_gz, b_ax, b_ay]
  Vector8d state_;
  Matrix8d covariance_;
  std::optional<ImuSample> last_imu_;
  rclcpp::Time last_pub_stamp_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<LocalizationDiagnostics>::SharedPtr diag_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  LocalizationDiagnostics last_diag_;
  bool initialized_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationEskfNode>());
  rclcpp::shutdown();
  return 0;
}
