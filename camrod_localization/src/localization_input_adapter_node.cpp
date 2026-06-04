#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include <avg_msgs/msg/nav_sat_fix.hpp>
#include <avg_msgs/msg/pose_stamped.hpp>
#include <avg_msgs/msg/pose_with_covariance_stamped.hpp>
#include <avg_msgs/msg/odometry.hpp>
#include <avg_msgs/msg/twist_stamped.hpp>
#include <avg_msgs/msg/point.hpp>
#include <avg_msgs/msg/avg_localization_msgs.hpp>
#include <avg_msgs/msg/module_state.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace
{
constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_E2 = 6.69437999014e-3;

double deg2rad(double deg)
{
  return deg * M_PI / 180.0;
}

double normalizeYaw(double yaw)
{
  const double two_pi = 2.0 * M_PI;
  while (yaw > M_PI) yaw -= two_pi;
  while (yaw < -M_PI) yaw += two_pi;
  return yaw;
}

double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  const double half = yaw * 0.5;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

bool quaternionIsUsable(const geometry_msgs::msg::Quaternion & q)
{
  const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(norm_sq) && norm_sq > 1e-12;
}

struct Ecef
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

Ecef llhToEcef(double lat_rad, double lon_rad, double alt)
{
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);
  const double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);

  Ecef ecef;
  ecef.x = (N + alt) * cos_lat * cos_lon;
  ecef.y = (N + alt) * cos_lat * sin_lon;
  ecef.z = (N * (1.0 - WGS84_E2) + alt) * sin_lat;
  return ecef;
}

avg_msgs::msg::Point ecefToEnu(
  const Ecef & ref, const Ecef & cur, double lat_ref, double lon_ref)
{
  const double sin_lat = std::sin(lat_ref);
  const double cos_lat = std::cos(lat_ref);
  const double sin_lon = std::sin(lon_ref);
  const double cos_lon = std::cos(lon_ref);

  const double dx = cur.x - ref.x;
  const double dy = cur.y - ref.y;
  const double dz = cur.z - ref.z;

  avg_msgs::msg::Point enu;
  enu.x = -sin_lon * dx + cos_lon * dy;
  enu.y = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
  enu.z = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
  return enu;
}

avg_msgs::msg::Point rotatePointXY(const avg_msgs::msg::Point & in, double yaw_rad)
{
  avg_msgs::msg::Point out = in;
  if (std::abs(yaw_rad) < 1e-12) {
    return out;
  }
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  out.x = c * in.x - s * in.y;
  out.y = s * in.x + c * in.y;
  return out;
}

std::string normalizeWheelInputType(std::string type)
{
  std::transform(
    type.begin(), type.end(), type.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return type;
}

}  // namespace

namespace camrod::localization
{

class LocalizationInputAdapterNode : public rclcpp::Node
{
public:
  LocalizationInputAdapterNode()
  : Node("localization_input_adapter")
  {
    enable_navsat_to_pose_ = declare_parameter<bool>("enable_navsat_to_pose", true);
    // HH_260604: Add dual-antenna GNSS heading parameters for GNSS pose orientation.
    enable_gnss_heading_ = declare_parameter<bool>("enable_gnss_heading", false);
    gnss_heading_topic_ = declare_parameter<std::string>(
      "gnss_heading_topic", "/sensing/gnss/navheading");
    gnss_heading_timeout_s_ = declare_parameter<double>("gnss_heading_timeout_s", 1.0);
    gnss_heading_yaw_offset_rad_ =
      deg2rad(declare_parameter<double>("gnss_heading_yaw_offset_deg", 0.0));
    gnss_heading_covariance_floor_ = std::pow(
      deg2rad(declare_parameter<double>("gnss_heading_covariance_floor_deg", 1.0)), 2);
    gnss_heading_max_covariance_ =
      declare_parameter<double>("gnss_heading_max_covariance", 100.0);
    gnss_heading_unavailable_covariance_ =
      declare_parameter<double>("gnss_heading_unavailable_covariance", 1.0e6);
    hold_last_gnss_heading_ = declare_parameter<bool>("hold_last_gnss_heading", true);
    enable_pose_cov_bridge_ = declare_parameter<bool>("enable_pose_cov_bridge", true);
    enable_odometry_to_pose_ = declare_parameter<bool>("enable_odometry_to_pose", true);
    enable_wheel_odometry_bridge_ = declare_parameter<bool>("enable_wheel_odometry_bridge", true);

    publish_localization_status_ =
      declare_parameter<bool>("publish_localization_status", false);
    localization_status_topic_ =
      declare_parameter<std::string>("localization_status_topic", "/localization/status");

    navsat_topic_ = declare_parameter<std::string>(
      "navsat_topic", "/sensing/gnss/ublox_gps_node/fix");
    // HH_260527: Optional metric pose inputs.
    // UTM pose is treated as a metric XY source and shifted by configured offsets.
    utm_pose_topic_ = declare_parameter<std::string>("utm_pose_topic", "");
    gnss_pose_topic_ = declare_parameter<std::string>("gnss_pose_topic", "/sensing/gnss/pose");
    gnss_pose_cov_topic_ = declare_parameter<std::string>(
      "gnss_pose_cov_topic", "/sensing/gnss/pose_with_covariance");
    map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map");
    publish_gnss_covariance_ = declare_parameter<bool>("publish_gnss_covariance", true);
    // HH_260526: Replace use_navsat_position_covariance toggle with explicit source mode.
    // navsat_covariance_source options: navsat | fixed.
    navsat_covariance_source_ = normalizeWheelInputType(
      declare_parameter<std::string>("navsat_covariance_source", "navsat"));
    gnss_covariance_floor_xy_ = declare_parameter<double>("gnss_covariance_floor_xy", 0.25);
    gnss_covariance_floor_z_ = declare_parameter<double>("gnss_covariance_floor_z", 1.0);
    if (navsat_covariance_source_ != "navsat" && navsat_covariance_source_ != "fixed") {
      RCLCPP_WARN(
        get_logger(),
        "Invalid navsat_covariance_source='%s'. Falling back to 'navsat'.",
        navsat_covariance_source_.c_str());
      navsat_covariance_source_ = "navsat";
    }

    offset_lat_deg_ = declare_parameter<double>("offset_lat", 0.0);
    offset_lon_deg_ = declare_parameter<double>("offset_lon", 0.0);
    offset_alt_ = declare_parameter<double>("offset_alt", 0.0);
    yaw_offset_deg_ = declare_parameter<double>("yaw_offset_deg", 0.0);
    rotate_latlon_xy_by_yaw_offset_ = declare_parameter<bool>(
      "rotate_latlon_xy_by_yaw_offset", true);

    offset_utm_easting_ = declare_parameter<double>("offset_utm_easting", 0.0);
    offset_utm_northing_ = declare_parameter<double>("offset_utm_northing", 0.0);
    offset_utm_alt_ = declare_parameter<double>("offset_utm_alt", 0.0);
    max_position_jump_m_ = declare_parameter<double>("max_position_jump_m", 8.0);
    jump_reject_max_speed_mps_ = declare_parameter<double>("jump_reject_max_speed_mps", 20.0);
    jump_reject_reset_s_ = declare_parameter<double>("jump_reject_reset_s", 2.0);

    pose_cov_diag_ = declare_parameter<std::vector<double>>(
      "pose_covariance_diagonal",
      std::vector<double>{1.0, 1.0, 1.0, 999.0, 999.0, 1.0});

    odom_input_topic_ = declare_parameter<std::string>(
      "odom_input_topic", "/localization/odometry/filtered");
    odom_pose_topic_ = declare_parameter<std::string>(
      "odom_pose_topic", "/localization/pose");
    odom_pose_cov_topic_ = declare_parameter<std::string>(
      "odom_pose_cov_topic", "/localization/pose_with_covariance");
    odom_output_frame_id_ = declare_parameter<std::string>("odom_output_frame_id", "");

    wheel_input_topic_ = declare_parameter<std::string>(
      // HH_260410: Prefer platform status odometry as the primary wheel source.
      "wheel_input_topic", "/platform/status/odometry");
    wheel_output_topic_ = declare_parameter<std::string>(
      // HH_260410: Keep unified wheel output under /platform/status namespace.
      "wheel_output_topic", "/platform/status/wheel_odometry");
    wheel_nav_output_topic_ = declare_parameter<std::string>(
      "wheel_nav_output_topic", "/platform/wheel/nav_odometry");
    wheel_odom_frame_ = declare_parameter<std::string>("wheel_odom_frame_id", "odom");
    wheel_base_frame_ = declare_parameter<std::string>("wheel_base_frame_id", "robot_base_link");
    wheel_input_type_ = normalizeWheelInputType(
      declare_parameter<std::string>("wheel_input_type", "nav_odom"));
    // HH_260410: Runtime fallback input. Prefer primary /platform/status/*
    // and only consume fallback when primary becomes stale/missing.
    wheel_fallback_input_topic_ = declare_parameter<std::string>(
      "wheel_fallback_input_topic", "/rmp401/odom");
    wheel_fallback_input_type_ = normalizeWheelInputType(
      declare_parameter<std::string>("wheel_fallback_input_type", "nav_odom"));
    wheel_primary_timeout_s_ = declare_parameter<double>("wheel_primary_timeout_s", 0.7);

    yaw_offset_rad_ = deg2rad(yaw_offset_deg_);
    offset_lat_rad_ = deg2rad(offset_lat_deg_);
    offset_lon_rad_ = deg2rad(offset_lon_deg_);
    reference_ecef_ = llhToEcef(offset_lat_rad_, offset_lon_rad_, offset_alt_);

    pose_covariance_.fill(0.0);
    for (size_t i = 0; i < std::min<size_t>(pose_cov_diag_.size(), 6); ++i) {
      pose_covariance_[i + i * 6] = pose_cov_diag_[i];
    }

    if (enable_navsat_to_pose_) {
      gnss_pose_pub_ =
        create_publisher<avg_msgs::msg::PoseStamped>(gnss_pose_topic_, rclcpp::QoS(10));
      if (publish_gnss_covariance_) {
        gnss_pose_cov_pub_ =
          create_publisher<avg_msgs::msg::PoseWithCovarianceStamped>(gnss_pose_cov_topic_, rclcpp::QoS(10));
      }
    }

    if (enable_odometry_to_pose_) {
      odom_pose_pub_ =
        create_publisher<avg_msgs::msg::PoseStamped>(odom_pose_topic_, rclcpp::QoS(10));
      odom_pose_cov_pub_ =
        create_publisher<avg_msgs::msg::PoseWithCovarianceStamped>(odom_pose_cov_topic_, rclcpp::QoS(10));
    }

    if (enable_wheel_odometry_bridge_) {
      wheel_odom_pub_ =
        create_publisher<avg_msgs::msg::Odometry>(wheel_output_topic_, rclcpp::QoS(50));
      wheel_nav_odom_pub_ =
        create_publisher<nav_msgs::msg::Odometry>(wheel_nav_output_topic_, rclcpp::QoS(50));
    }

    if (publish_localization_status_) {
      avg_localization_pub_ =
        create_publisher<avg_msgs::msg::AvgLocalizationMsgs>(localization_status_topic_, rclcpp::QoS(10));
    }

    using std::placeholders::_1;

    if (enable_navsat_to_pose_) {
      navsat_sub_ = create_subscription<avg_msgs::msg::NavSatFix>(
        navsat_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LocalizationInputAdapterNode::onNavSatFix, this, _1));

      // HH_260604: Subscribe to dual-antenna GNSS heading when enabled.
      if (enable_gnss_heading_) {
        gnss_heading_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          gnss_heading_topic_, rclcpp::SensorDataQoS(),
          std::bind(&LocalizationInputAdapterNode::onGnssHeading, this, _1));
      }

      if (!raw_pose_topic_.empty()) {
        raw_pose_sub_ = create_subscription<avg_msgs::msg::PoseStamped>(
          raw_pose_topic_, rclcpp::QoS(10),
          std::bind(&LocalizationInputAdapterNode::onRawPose, this, _1));
      }

      if (!utm_pose_topic_.empty()) {
        utm_pose_sub_ = create_subscription<avg_msgs::msg::PoseStamped>(
          utm_pose_topic_, rclcpp::QoS(10),
          std::bind(&LocalizationInputAdapterNode::onUtmPose, this, _1));
      }
    }

    if (enable_odometry_to_pose_) {
      odom_sub_ = create_subscription<avg_msgs::msg::Odometry>(
        odom_input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LocalizationInputAdapterNode::onOdom, this, _1));
    }

    if (enable_wheel_odometry_bridge_) {
      if (wheel_input_type_ == "twist") {
        wheel_twist_sub_ = create_subscription<avg_msgs::msg::TwistStamped>(
          wheel_input_topic_, rclcpp::SensorDataQoS(),
          std::bind(&LocalizationInputAdapterNode::onWheelTwist, this, _1));
      } else if (wheel_input_type_ == "avg_odom") {
        wheel_avg_odom_sub_ = create_subscription<avg_msgs::msg::Odometry>(
          wheel_input_topic_, rclcpp::SensorDataQoS(),
          std::bind(&LocalizationInputAdapterNode::onWheelAvgOdom, this, _1));
      } else if (wheel_input_type_ == "nav_odom") {
        wheel_nav_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          wheel_input_topic_, rclcpp::SensorDataQoS(),
          std::bind(&LocalizationInputAdapterNode::onWheelNavOdom, this, _1));
      } else {
        throw std::runtime_error(
                "Unsupported wheel_input_type. Supported: twist|avg_odom|nav_odom");
      }

      if (!wheel_fallback_input_topic_.empty() && !wheel_fallback_input_type_.empty()) {
        if (wheel_fallback_input_topic_ == wheel_input_topic_ &&
            wheel_fallback_input_type_ == wheel_input_type_) {
          RCLCPP_WARN(
            get_logger(),
            "wheel_fallback_input_topic/type is same as primary. Skip fallback subscription.");
        } else if (wheel_fallback_input_type_ == "twist") {
          wheel_twist_fallback_sub_ = create_subscription<avg_msgs::msg::TwistStamped>(
            wheel_fallback_input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LocalizationInputAdapterNode::onWheelTwistFallback, this, _1));
        } else if (wheel_fallback_input_type_ == "avg_odom") {
          wheel_avg_odom_fallback_sub_ = create_subscription<avg_msgs::msg::Odometry>(
            wheel_fallback_input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LocalizationInputAdapterNode::onWheelAvgOdomFallback, this, _1));
        } else if (wheel_fallback_input_type_ == "nav_odom") {
          wheel_nav_odom_fallback_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            wheel_fallback_input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LocalizationInputAdapterNode::onWheelNavOdomFallback, this, _1));
        } else {
          throw std::runtime_error(
                  "Unsupported wheel_fallback_input_type. Supported: twist|avg_odom|nav_odom");
        }
      }

      // HH_260413: Explicit startup summary for primary/fallback wheel wiring.
      RCLCPP_INFO(
        get_logger(),
        "wheel bridge configured: primary=(%s,%s) fallback=(%s,%s) timeout=%.2fs output=%s",
        wheel_input_topic_.c_str(), wheel_input_type_.c_str(),
        wheel_fallback_input_topic_.c_str(), wheel_fallback_input_type_.c_str(),
        wheel_primary_timeout_s_, wheel_output_topic_.c_str());
    }
  }

private:
  enum class WheelSource
  {
    kNone,
    kPrimary,
    kFallback
  };

  // HH_260604: Store heading samples with covariance and timestamp for GNSS pose fusion.
  struct HeadingSample
  {
    geometry_msgs::msg::Quaternion orientation;
    double yaw_covariance{1.0e6};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool has_sample{false};
  };

  bool headingFresh(const HeadingSample & sample, const rclcpp::Time & stamp, double timeout_s) const
  {
    if (!sample.has_sample || timeout_s < 0.0) {
      return sample.has_sample;
    }
    const double age = std::abs((stamp - sample.stamp).seconds());
    return age <= timeout_s;
  }

  bool headingUsable(const HeadingSample & sample, const rclcpp::Time & stamp, double timeout_s) const
  {
    return headingFresh(sample, stamp, timeout_s) &&
      std::isfinite(sample.yaw_covariance) &&
      sample.yaw_covariance > 0.0 &&
      sample.yaw_covariance <= gnss_heading_max_covariance_;
  }

  void rememberHeading(const HeadingSample & sample)
  {
    if (!sample.has_sample) {
      return;
    }
    last_any_heading_ = sample;
    has_any_heading_ = true;
  }

  // HH_260604: Cache yaw and covariance from dual-antenna GNSS heading.
  void onGnssHeading(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if (!quaternionIsUsable(msg->orientation)) {
      return;
    }
    const double raw_cov = msg->orientation_covariance[8];
    const double yaw_cov = (std::isfinite(raw_cov) && raw_cov > 0.0)
      ? std::max(raw_cov, gnss_heading_covariance_floor_)
      : gnss_heading_unavailable_covariance_;

    const double yaw = normalizeYaw(yawFromQuat(msg->orientation) + gnss_heading_yaw_offset_rad_);
    gnss_heading_.orientation = yawToQuat(yaw);
    gnss_heading_.yaw_covariance = yaw_cov;
    gnss_heading_.stamp = rclcpp::Time(msg->header.stamp);
    gnss_heading_.has_sample = true;
    rememberHeading(gnss_heading_);
  }

  // HH_260604: Prefer dual-antenna heading; fall back to last heading for display only.
  bool selectHeading(
    const rclcpp::Time & stamp,
    geometry_msgs::msg::Quaternion & orientation,
    double & yaw_covariance) const
  {
    if (enable_gnss_heading_ &&
        headingUsable(gnss_heading_, stamp, gnss_heading_timeout_s_)) {
      orientation = gnss_heading_.orientation;
      yaw_covariance = gnss_heading_.yaw_covariance;
      return true;
    }
    if (hold_last_gnss_heading_ && has_any_heading_) {
      orientation = last_any_heading_.orientation;
    } else {
      orientation = yawToQuat(0.0);
    }
    yaw_covariance = gnss_heading_unavailable_covariance_;
    return false;
  }

  void onNavSatFix(const avg_msgs::msg::NavSatFix::ConstSharedPtr msg)
  {
    const double lat_rad = deg2rad(msg->latitude);
    const double lon_rad = deg2rad(msg->longitude);
    const double alt = msg->altitude;

    const Ecef cur_ecef = llhToEcef(lat_rad, lon_rad, alt);
    avg_msgs::msg::Point p = ecefToEnu(reference_ecef_, cur_ecef, offset_lat_rad_, offset_lon_rad_);

    if (rotate_latlon_xy_by_yaw_offset_) {
      p = rotatePointXY(p, yaw_offset_rad_);
    }

    const rclcpp::Time stamp(msg->header.stamp);

    if (has_last_position_) {
      const double dx = p.x - last_position_.x;
      const double dy = p.y - last_position_.y;
      const double dz = p.z - last_position_.z;
      const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      const double dt = (stamp - last_position_stamp_).seconds();

      if (dt > 1e-3) {
        const double speed = dist / dt;
        if (dist > max_position_jump_m_ && speed > jump_reject_max_speed_mps_ &&
            dt < jump_reject_reset_s_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Reject GNSS jump dist=%.3f speed=%.3f", dist, speed);
          return;
        }
      }
    }

    last_position_ = p;
    last_position_stamp_ = stamp;
    has_last_position_ = true;

    avg_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.header.frame_id = map_frame_id_;
    pose.pose.position = p;
    geometry_msgs::msg::Quaternion heading_orientation;
    double heading_yaw_covariance = gnss_heading_unavailable_covariance_;
    const bool has_fresh_heading =
      selectHeading(stamp, heading_orientation, heading_yaw_covariance);
    // HH_260604: Publish GNSS pose orientation from the selected heading instead of fixed yaw.
    pose.pose.orientation = heading_orientation;

    gnss_pose_pub_->publish(pose);

    if (publish_gnss_covariance_) {
      avg_msgs::msg::PoseWithCovarianceStamped pose_cov;
      pose_cov.header = pose.header;
      pose_cov.pose.pose = pose.pose;
      pose_cov.pose.covariance = pose_covariance_;
      // HH_260604: Expose only yaw covariance when a fresh GNSS heading is available.
      pose_cov.pose.covariance[21] = gnss_heading_unavailable_covariance_;
      pose_cov.pose.covariance[28] = gnss_heading_unavailable_covariance_;
      pose_cov.pose.covariance[35] = has_fresh_heading
        ? heading_yaw_covariance
        : gnss_heading_unavailable_covariance_;

      if (navsat_covariance_source_ == "navsat" && msg->position_covariance_type != 0) {
        // HH_260415: Protect filter stability by applying minimum GNSS covariance floors.
        const double nav_x = msg->position_covariance[0];
        const double nav_y = msg->position_covariance[4];
        const double nav_z = msg->position_covariance[8];

        const double var_x = (std::isfinite(nav_x) && nav_x > 0.0)
          ? std::max(nav_x, gnss_covariance_floor_xy_)
          : pose_cov.pose.covariance[0];
        const double var_y = (std::isfinite(nav_y) && nav_y > 0.0)
          ? std::max(nav_y, gnss_covariance_floor_xy_)
          : pose_cov.pose.covariance[7];
        const double var_z = (std::isfinite(nav_z) && nav_z > 0.0)
          ? std::max(nav_z, gnss_covariance_floor_z_)
          : pose_cov.pose.covariance[14];

        pose_cov.pose.covariance[0] = var_x;
        pose_cov.pose.covariance[7] = var_y;
        pose_cov.pose.covariance[14] = var_z;
      }
      gnss_pose_cov_pub_->publish(pose_cov);
    }

    publishLocalizationStatus("navsat_to_pose", msg->header.stamp);
  }

  void publishMetricPose(
    const avg_msgs::msg::PoseStamped::ConstSharedPtr msg,
    double offset_easting,
    double offset_northing,
    double offset_alt,
    const std::string & status_source)
  {
    if (!enable_navsat_to_pose_) {
      return;
    }
    avg_msgs::msg::PoseStamped out = *msg;
    out.header.frame_id = map_frame_id_;
    out.pose.position.x -= offset_easting;
    out.pose.position.y -= offset_northing;
    out.pose.position.z -= offset_alt;

    if (rotate_latlon_xy_by_yaw_offset_) {
      out.pose.position = rotatePointXY(out.pose.position, yaw_offset_rad_);
    }

    gnss_pose_pub_->publish(out);

    if (publish_gnss_covariance_) {
      avg_msgs::msg::PoseWithCovarianceStamped cov;
      cov.header = out.header;
      cov.pose.pose = out.pose;
      cov.pose.covariance = pose_covariance_;
      gnss_pose_cov_pub_->publish(cov);
    }
    publishLocalizationStatus(status_source, msg->header.stamp);
  }

  void onUtmPose(const avg_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    publishMetricPose(
      msg,
      offset_utm_easting_,
      offset_utm_northing_,
      offset_utm_alt_,
      "utm_pose_bridge");
  }

  void onRawPose(const avg_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    if (!enable_navsat_to_pose_) {
      return;
    }
    avg_msgs::msg::PoseStamped out = *msg;
    out.header.frame_id = map_frame_id_;
    gnss_pose_pub_->publish(out);
    if (publish_gnss_covariance_) {
      avg_msgs::msg::PoseWithCovarianceStamped cov;
      cov.header = out.header;
      cov.pose.pose = out.pose;
      cov.pose.covariance = pose_covariance_;
      gnss_pose_cov_pub_->publish(cov);
    }
    publishLocalizationStatus("raw_pose_bridge", msg->header.stamp);
  }

  void onOdom(const avg_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    avg_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    if (!odom_output_frame_id_.empty()) {
      pose.header.frame_id = odom_output_frame_id_;
    }
    odom_pose_pub_->publish(pose);

    avg_msgs::msg::PoseWithCovarianceStamped pose_cov;
    pose_cov.header = msg->header;
    pose_cov.pose = msg->pose;
    if (!odom_output_frame_id_.empty()) {
      pose_cov.header.frame_id = odom_output_frame_id_;
    }
    odom_pose_cov_pub_->publish(pose_cov);

    publishLocalizationStatus("odometry_to_pose", msg->header.stamp);
  }

  void onWheelTwist(const avg_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    markPrimaryWheelRx();
    setActiveWheelSource(WheelSource::kPrimary, "primary_twist");
    publishWheelFromTwist(*msg, "wheel_twist_bridge");
  }

  void onWheelTwistFallback(const avg_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    if (!shouldUseFallback()) {
      return;
    }
    markFallbackWheelRx();
    setActiveWheelSource(WheelSource::kFallback, "fallback_twist");
    publishWheelFromTwist(*msg, "wheel_twist_fallback_bridge");
  }

  void onWheelAvgOdom(const avg_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    markPrimaryWheelRx();
    setActiveWheelSource(WheelSource::kPrimary, "primary_avg_odom");
    publishWheelFromAvgOdom(*msg, "wheel_avg_odom_bridge");
  }

  void onWheelAvgOdomFallback(const avg_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (!shouldUseFallback()) {
      return;
    }
    markFallbackWheelRx();
    setActiveWheelSource(WheelSource::kFallback, "fallback_avg_odom");
    publishWheelFromAvgOdom(*msg, "wheel_avg_odom_fallback_bridge");
  }

  void onWheelNavOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    markPrimaryWheelRx();
    setActiveWheelSource(WheelSource::kPrimary, "primary_nav_odom");
    publishWheelFromNavOdom(*msg, "wheel_nav_odom_bridge");
  }

  void onWheelNavOdomFallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (!shouldUseFallback()) {
      return;
    }
    markFallbackWheelRx();
    setActiveWheelSource(WheelSource::kFallback, "fallback_nav_odom");
    publishWheelFromNavOdom(*msg, "wheel_nav_odom_fallback_bridge");
  }

  void publishWheelFromTwist(
    const avg_msgs::msg::TwistStamped & msg,
    const std::string & status_source)
  {
    avg_msgs::msg::Odometry odom;
    odom.header = msg.header;
    odom.header.frame_id = wheel_odom_frame_;
    odom.child_frame_id = wheel_base_frame_;
    odom.twist.twist = msg.twist;
    wheel_odom_pub_->publish(odom);
    wheel_nav_odom_pub_->publish(toNavOdometry(odom));
    publishLocalizationStatus(status_source, msg.header.stamp);
  }

  void publishWheelFromAvgOdom(
    const avg_msgs::msg::Odometry & msg,
    const std::string & status_source)
  {
    avg_msgs::msg::Odometry odom = msg;
    if (odom.header.frame_id.empty()) {
      odom.header.frame_id = wheel_odom_frame_;
    }
    // HH_260413: Normalize child frame to robot_base_link to avoid base_link TF duplication.
    odom.child_frame_id = wheel_base_frame_;
    wheel_odom_pub_->publish(odom);
    wheel_nav_odom_pub_->publish(toNavOdometry(odom));
    publishLocalizationStatus(status_source, msg.header.stamp);
  }

  void publishWheelFromNavOdom(
    const nav_msgs::msg::Odometry & msg,
    const std::string & status_source)
  {
    avg_msgs::msg::Odometry odom;
    odom.header = msg.header;
    // HH_260410: Keep configured fallback frame handling when incoming nav odom
    // does not carry frame IDs.
    if (odom.header.frame_id.empty()) {
      odom.header.frame_id = wheel_odom_frame_;
    }
    // HH_260413: Force child frame to configured base frame to keep TF consistent.
    odom.child_frame_id = wheel_base_frame_;
    odom.pose = msg.pose;
    odom.twist = msg.twist;
    wheel_odom_pub_->publish(odom);
    nav_msgs::msg::Odometry nav_out = msg;
    nav_out.child_frame_id = wheel_base_frame_;
    if (nav_out.header.frame_id.empty()) {
      nav_out.header.frame_id = wheel_odom_frame_;
    }
    wheel_nav_odom_pub_->publish(nav_out);
    publishLocalizationStatus(status_source, msg.header.stamp);
  }

  bool shouldUseFallback() const
  {
    if (!seen_primary_wheel_) {
      return true;
    }
    if (wheel_primary_timeout_s_ <= 0.0) {
      return false;
    }
    const double age = (this->now() - last_primary_wheel_rx_time_).seconds();
    return age > wheel_primary_timeout_s_;
  }

  void markPrimaryWheelRx()
  {
    seen_primary_wheel_ = true;
    last_primary_wheel_rx_time_ = this->now();
  }

  void markFallbackWheelRx()
  {
    seen_fallback_wheel_ = true;
    last_fallback_wheel_rx_time_ = this->now();
  }

  void setActiveWheelSource(WheelSource source, const std::string & reason)
  {
    if (active_wheel_source_ == source) {
      return;
    }
    active_wheel_source_ = source;
    last_wheel_source_switch_time_ = this->now();
    const char * label = (source == WheelSource::kPrimary) ? "primary" :
      (source == WheelSource::kFallback ? "fallback" : "none");
    // HH_260413: Log actual message-driven source switching for debugging.
    RCLCPP_INFO(
      get_logger(),
      "wheel source switched to %s (%s) primary_seen=%s fallback_seen=%s",
      label, reason.c_str(),
      seen_primary_wheel_ ? "true" : "false",
      seen_fallback_wheel_ ? "true" : "false");
  }

  nav_msgs::msg::Odometry toNavOdometry(const avg_msgs::msg::Odometry & msg) const
  {
    nav_msgs::msg::Odometry out;
    out.header = msg.header;
    out.child_frame_id = msg.child_frame_id;
    out.pose = msg.pose;
    out.twist = msg.twist;
    return out;
  }

  void publishLocalizationStatus(
    const std::string & source, const builtin_interfaces::msg::Time & stamp)
  {
    if (!publish_localization_status_ || !avg_localization_pub_) {
      return;
    }

    avg_msgs::msg::AvgLocalizationMsgs avg_msg;
    avg_msg.stamp = stamp;
    avg_msg.state.stamp = stamp;
    avg_msg.state.module_name = "localization";
    avg_msg.state.level = avg_msgs::msg::ModuleState::OK;
    avg_msg.state.message = source;
    avg_localization_pub_->publish(avg_msg);
  }

  bool enable_navsat_to_pose_{true};
  bool enable_odometry_to_pose_{true};
  bool enable_wheel_odometry_bridge_{true};
  bool publish_localization_status_{false};

  std::string localization_status_topic_;
  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationMsgs>::SharedPtr avg_localization_pub_;

  std::string navsat_topic_, utm_pose_topic_, raw_pose_topic_;
  std::string gnss_pose_topic_, gnss_pose_cov_topic_, map_frame_id_;
  bool publish_gnss_covariance_{true};
  // HH_260604: Keep GNSS heading state for dual-antenna heading selection.
  bool enable_gnss_heading_{false};
  std::string gnss_heading_topic_;
  double gnss_heading_timeout_s_{1.0};
  double gnss_heading_yaw_offset_rad_{0.0};
  double gnss_heading_covariance_floor_{0.0};
  double gnss_heading_max_covariance_{100.0};
  double gnss_heading_unavailable_covariance_{1.0e6};
  bool hold_last_gnss_heading_{true};
  bool enable_pose_cov_bridge_{true};
  HeadingSample gnss_heading_;
  HeadingSample last_any_heading_;
  bool has_any_heading_{false};
  std::string navsat_covariance_source_{"navsat"};
  double gnss_covariance_floor_xy_{0.25};
  double gnss_covariance_floor_z_{1.0};

  double offset_lat_deg_{0.0};
  double offset_lon_deg_{0.0};
  double offset_alt_{0.0};
  double yaw_offset_deg_{0.0};
  double yaw_offset_rad_{0.0};
  bool rotate_latlon_xy_by_yaw_offset_{true};

  double offset_utm_easting_{0.0};
  double offset_utm_northing_{0.0};
  double offset_utm_alt_{0.0};

  double offset_lat_rad_{0.0};
  double offset_lon_rad_{0.0};
  Ecef reference_ecef_{};

  double max_position_jump_m_{8.0};
  double jump_reject_max_speed_mps_{20.0};
  double jump_reject_reset_s_{2.0};

  bool has_last_position_{false};
  avg_msgs::msg::Point last_position_;
  rclcpp::Time last_position_stamp_{0, 0, RCL_ROS_TIME};

  std::vector<double> pose_cov_diag_;
  std::array<double, 36> pose_covariance_{};

  rclcpp::Publisher<avg_msgs::msg::PoseStamped>::SharedPtr gnss_pose_pub_;
  rclcpp::Publisher<avg_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_cov_pub_;
  rclcpp::Subscription<avg_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gnss_heading_sub_;
  rclcpp::Subscription<avg_msgs::msg::PoseStamped>::SharedPtr utm_pose_sub_;
  rclcpp::Subscription<avg_msgs::msg::PoseStamped>::SharedPtr raw_pose_sub_;

  std::string odom_input_topic_, odom_pose_topic_, odom_pose_cov_topic_, odom_output_frame_id_;
  rclcpp::Publisher<avg_msgs::msg::PoseStamped>::SharedPtr odom_pose_pub_;
  rclcpp::Publisher<avg_msgs::msg::PoseWithCovarianceStamped>::SharedPtr odom_pose_cov_pub_;
  rclcpp::Subscription<avg_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::string wheel_input_topic_, wheel_output_topic_, wheel_nav_output_topic_;
  std::string wheel_odom_frame_, wheel_base_frame_, wheel_input_type_;
  std::string wheel_fallback_input_topic_, wheel_fallback_input_type_;
  double wheel_primary_timeout_s_{0.7};
  bool seen_primary_wheel_{false};
  rclcpp::Time last_primary_wheel_rx_time_{0, 0, RCL_ROS_TIME};
  bool seen_fallback_wheel_{false};
  rclcpp::Time last_fallback_wheel_rx_time_{0, 0, RCL_ROS_TIME};
  WheelSource active_wheel_source_{WheelSource::kNone};
  rclcpp::Time last_wheel_source_switch_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<avg_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_nav_odom_pub_;
  rclcpp::Subscription<avg_msgs::msg::TwistStamped>::SharedPtr wheel_twist_sub_;
  rclcpp::Subscription<avg_msgs::msg::Odometry>::SharedPtr wheel_avg_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_nav_odom_sub_;
  rclcpp::Subscription<avg_msgs::msg::TwistStamped>::SharedPtr wheel_twist_fallback_sub_;
  rclcpp::Subscription<avg_msgs::msg::Odometry>::SharedPtr wheel_avg_odom_fallback_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_nav_odom_fallback_sub_;
};

}  // namespace camrod::localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod::localization::LocalizationInputAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
