#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <ublox_msgs/msg/nav_relposned9.hpp>

#include <ublox_gps/hp_pos_rec_product.hpp>
#include <ublox_gps/hpg_ref_product.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

namespace {

// HH_260611: Convert RELPOSNED carrier flags into readable diagnostics for
// simultaneous heading and RTK-fixed validation during simpleRTK2B Heading tests.
const char * carrierSolutionLabel(uint32_t flags) {
  switch (flags & ublox_msgs::msg::NavRELPOSNED9::FLAGS_CARR_SOLN_MASK) {
    case ublox_msgs::msg::NavRELPOSNED9::FLAGS_CARR_SOLN_FIXED:
      return "fixed";
    case ublox_msgs::msg::NavRELPOSNED9::FLAGS_CARR_SOLN_FLOAT:
      return "float";
    default:
      return "none";
  }
}

}  // namespace

//
// U-Blox High Precision Positioning Receiver
//
HpPosRecProduct::HpPosRecProduct(uint16_t nav_rate, uint16_t meas_rate, const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::vector<ublox_gps::Rtcm> rtcms, rclcpp::Node* node)
  : HpgRefProduct(nav_rate, meas_rate, updater, rtcms, node), frame_id_(frame_id)
{
  if (getRosBoolean(node_, "publish.nav.relposned")) {
    nav_relposned_pub_ =
      node_->create_publisher<ublox_msgs::msg::NavRELPOSNED9>("navrelposned", 1);
  }

  if (getRosBoolean(node_, "publish.nav.heading")) {
    imu_pub_ =
      node_->create_publisher<sensor_msgs::msg::Imu>("navheading", 1);
  }
}

void HpPosRecProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Whether to publish Nav Relative Position NED
  // Subscribe to Nav Relative Position NED messages (also updates diagnostics)
  auto callback = std::bind(
     &HpPosRecProduct::callbackNavRelPosNed, this, std::placeholders::_1);
  // HH_260611: In dual-antenna mode, avoid one-shot RELPOSNED subscription so
  // heading remains continuously available during simpleRTK2B Heading operation.
  if (getRosBoolean(node_, "dual_antenna")) {
    gps->subscribe<ublox_msgs::msg::NavRELPOSNED9>(callback);
  } else {
    gps->subscribe<ublox_msgs::msg::NavRELPOSNED9>(callback, 1);
  }
}

void HpPosRecProduct::callbackNavRelPosNed(const ublox_msgs::msg::NavRELPOSNED9 &m) {
  // HH_260611: Log moving-baseline heading state explicitly to separate
  // heading-valid failures from absolute RTK pose fix failures.
  const bool rel_pos_valid =
    (m.flags & ublox_msgs::msg::NavRELPOSNED9::FLAGS_REL_POS_VALID) != 0;
  const bool heading_valid =
    (m.flags & ublox_msgs::msg::NavRELPOSNED9::FLAGS_REL_POS_HEAD_VALID) != 0;
  const bool moving_baseline =
    (m.flags & ublox_msgs::msg::NavRELPOSNED9::FLAGS_IS_MOVING) != 0;

  if (heading_valid) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Dual antenna heading valid: flags=%u carrier=%s ref_station=%u length_cm=%d heading_deg=%.5f acc_heading_deg=%.5f",
      m.flags,
      carrierSolutionLabel(m.flags),
      m.ref_station_id,
      m.rel_pos_length,
      static_cast<double>(m.rel_pos_heading) * 1e-5,
      static_cast<double>(m.acc_heading) * 1e-5);
  } else {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Dual antenna heading invalid: flags=%u carrier=%s gnss_fix_ok=%d diff_soln=%d rel_pos_valid=%d is_moving=%d ref_pos_miss=%d ref_obs_miss=%d head_valid=%d ref_station=%u length_cm=%d heading_raw=%d acc_heading_raw=%u",
      m.flags,
      carrierSolutionLabel(m.flags),
      (m.flags & ublox_msgs::msg::NavRELPOSNED9::FLAGS_GNSS_FIX_OK) != 0,
      (m.flags & ublox_msgs::msg::NavRELPOSNED9::FLAGS_DIFF_SOLN) != 0,
      rel_pos_valid,
      moving_baseline,
      (m.flags & ublox_msgs::msg::NavRELPOSNED9::FLAGS_REF_POS_MISS) != 0,
      (m.flags & ublox_msgs::msg::NavRELPOSNED9::FLAGS_REF_OBS_MISS) != 0,
      heading_valid,
      m.ref_station_id,
      m.rel_pos_length,
      m.rel_pos_heading,
      m.acc_heading);
  }

  if (getRosBoolean(node_, "publish.nav.relposned")) {
    nav_relposned_pub_->publish(m);
  }

  if (getRosBoolean(node_, "publish.nav.heading")) {
    imu_.header.stamp = node_->now();
    imu_.header.frame_id = frame_id_;

    imu_.linear_acceleration_covariance[0] = -1;
    imu_.angular_velocity_covariance[0] = -1;

    // Transform angle since ublox is representing heading as NED but ROS uses ENU as convention (REP-103).
    double heading = M_PI_2 - (static_cast<double>(m.rel_pos_heading) * 1e-5 / 180.0 * M_PI);
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, heading);
    imu_.orientation.x = orientation[0];
    imu_.orientation.y = orientation[1];
    imu_.orientation.z = orientation[2];
    imu_.orientation.w = orientation[3];
    // HH_260611: Use reported heading accuracy only when the receiver marks the
    // heading valid; otherwise publish unknown orientation covariance.
    if (heading_valid) {
      constexpr double kRollPitchCovariance = 1.0e6;
      imu_.orientation_covariance[0] = kRollPitchCovariance;
      imu_.orientation_covariance[4] = kRollPitchCovariance;
      imu_.orientation_covariance[8] = ::pow(m.acc_heading * 1e-5 / 180.0 * M_PI, 2);
    } else {
      imu_.orientation_covariance[0] = -1.0;
      imu_.orientation_covariance[4] = 0.0;
      imu_.orientation_covariance[8] = 1000.0;
    }

    imu_pub_->publish(imu_);
  }

  last_rel_pos_ = m;
}

}  // namespace ublox_node
