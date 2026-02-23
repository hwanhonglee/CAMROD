#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <cmath>
#include <string>

namespace
{
constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_E2 = 6.69437999014e-3;

double deg2rad(double deg)
{
  return deg * M_PI / 180.0;
}

struct Ecef
{
  double x;
  double y;
  double z;
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

geometry_msgs::msg::Point ecefToEnu(
  const Ecef & ref, const Ecef & cur, double lat_ref, double lon_ref)
{
  const double sin_lat = std::sin(lat_ref);
  const double cos_lat = std::cos(lat_ref);
  const double sin_lon = std::sin(lon_ref);
  const double cos_lon = std::cos(lon_ref);

  const double dx = cur.x - ref.x;
  const double dy = cur.y - ref.y;
  const double dz = cur.z - ref.z;

  geometry_msgs::msg::Point enu;
  enu.x = -sin_lon * dx + cos_lon * dy;
  enu.y = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
  enu.z = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
  return enu;
}
}  // namespace

namespace camping_cart::localization
{
class NavsatToPoseNode : public rclcpp::Node
{
public:
  NavsatToPoseNode()
  // HH_260112 Use short node name; namespace applies the module prefix.
  : Node("navsat_to_pose")
  {
    // HH_260109 Use sensing-prefixed GNSS topics.
    navsat_topic_ = declare_parameter<std::string>("navsat_topic", "/sensing/gnss/navsatfix");
    // 2026-01-30: Default to no raw/UTM subscriptions unless explicitly configured.
    raw_pose_topic_ = declare_parameter<std::string>("raw_pose_topic", "");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/sensing/gnss/pose");
    utm_pose_topic_ = declare_parameter<std::string>("utm_pose_topic", "");
    pose_cov_topic_ = declare_parameter<std::string>("pose_cov_topic", "/sensing/gnss/pose_with_covariance");
    map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map");
    publish_covariance_ = declare_parameter<bool>("publish_covariance", true);
    const auto cov = declare_parameter<std::vector<double>>(
      "position_covariance_diagonal", std::vector<double>{1.0, 1.0, 1.0, 999.0, 999.0, 1.0});
    covariance_.fill(0.0);
    for (size_t i = 0; i < std::min<size_t>(cov.size(), 6); ++i) {
      covariance_[i + i * 6] = cov[i];
    }

    origin_lat_deg_ = declare_parameter<double>("origin_lat", 0.0);
    origin_lon_deg_ = declare_parameter<double>("origin_lon", 0.0);
    origin_alt_ = declare_parameter<double>("origin_alt", 0.0);
    yaw_offset_rad_ = deg2rad(declare_parameter<double>("yaw_offset_deg", 0.0));

    utm_origin_easting_ = declare_parameter<double>("utm_origin_easting", 0.0);
    utm_origin_northing_ = declare_parameter<double>("utm_origin_northing", 0.0);
    utm_origin_alt_ = declare_parameter<double>("utm_origin_alt", 0.0);

    origin_lat_rad_ = deg2rad(origin_lat_deg_);
    origin_lon_rad_ = deg2rad(origin_lon_deg_);
    reference_ecef_ = llhToEcef(origin_lat_rad_, origin_lon_rad_, origin_alt_);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, rclcpp::QoS(10));
    if (publish_covariance_) {
      pose_cov_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_cov_topic_, rclcpp::QoS(10));
    }
    navsat_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      navsat_topic_, rclcpp::SensorDataQoS(),
      std::bind(&NavsatToPoseNode::onNavSatFix, this, std::placeholders::_1));
    if (!raw_pose_topic_.empty()) {
      raw_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        raw_pose_topic_, rclcpp::QoS(10),
        std::bind(&NavsatToPoseNode::onRawPose, this, std::placeholders::_1));
    }
    if (!utm_pose_topic_.empty()) {
      utm_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        utm_pose_topic_, rclcpp::QoS(10),
        std::bind(&NavsatToPoseNode::onUtmPose, this, std::placeholders::_1));
      // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
      RCLCPP_DEBUG(
        get_logger(), "UTM pose topic enabled (%s)", utm_pose_topic_.c_str());
    }

    RCLCPP_DEBUG(
      get_logger(),
      "GNSS transform node ready - origin(lat=%.8f, lon=%.8f, alt=%.2f) raw_topic=%s",
      origin_lat_deg_, origin_lon_deg_, origin_alt_, raw_pose_topic_.c_str());
  }

private:
  enum class InputType { ENU, LAT_LON, UTM };

  void onNavSatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
  {
    if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) || !std::isfinite(msg->altitude)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "invalid NavSatFix");
      return;
    }
    const auto enu = convertLatLon(msg->latitude, msg->longitude, msg->altitude);
    publishEnuPose(msg->header.stamp, enu, yaw_offset_rad_);
  }

  void onRawPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    const auto type = deduceType(*msg);
    if (type == InputType::ENU) {
      publishEnuPose(msg->header.stamp, msg->pose.position, yawFromMsg(*msg));
      return;
    }
    if (type == InputType::LAT_LON) {
      const auto enu = convertLatLon(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
      publishEnuPose(msg->header.stamp, enu, yawFromMsg(*msg));
      return;
    }
    geometry_msgs::msg::Point enu;
    enu.x = msg->pose.position.x - utm_origin_easting_;
    enu.y = msg->pose.position.y - utm_origin_northing_;
    enu.z = msg->pose.position.z - utm_origin_alt_;
    publishEnuPose(msg->header.stamp, enu, yawFromMsg(*msg));
  }

  void onUtmPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    if (!std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y) ||
      !std::isfinite(msg->pose.position.z))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "invalid UTM pose");
      return;
    }
    geometry_msgs::msg::Point enu;
    enu.x = msg->pose.position.x - utm_origin_easting_;
    enu.y = msg->pose.position.y - utm_origin_northing_;
    enu.z = msg->pose.position.z - utm_origin_alt_;
    publishEnuPose(msg->header.stamp, enu, yawFromMsg(*msg));
  }

  InputType deduceType(const geometry_msgs::msg::PoseStamped & msg) const
  {
    const auto & frame = msg.header.frame_id;
    if (frame == "wgs84" || frame == "latlon") {
      return InputType::LAT_LON;
    }
    if (frame == "utm") {
      return InputType::UTM;
    }
    const double x = msg.pose.position.x;
    const double y = msg.pose.position.y;
    if (std::abs(x) <= 90.0 && std::abs(y) <= 180.0) {
      return InputType::LAT_LON;
    }
    if (std::abs(x) > 1e5 || std::abs(y) > 1e5) {
      return InputType::UTM;
    }
    return InputType::ENU;
  }

  geometry_msgs::msg::Point convertLatLon(double lat_deg, double lon_deg, double alt) const
  {
    const double lat_rad = deg2rad(lat_deg);
    const double lon_rad = deg2rad(lon_deg);
    const auto current_ecef = llhToEcef(lat_rad, lon_rad, alt);
    return ecefToEnu(reference_ecef_, current_ecef, origin_lat_rad_, origin_lon_rad_);
  }

  double yawFromMsg(const geometry_msgs::msg::PoseStamped & msg) const
  {
    const auto & q = msg.pose.orientation;
    return 2.0 * std::atan2(q.z, q.w);
  }

  void publishEnuPose(
    const rclcpp::Time & stamp, const geometry_msgs::msg::Point & enu, double yaw) const
  {
    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = map_frame_id_;
    out.pose.position = enu;
    const double half_yaw = yaw * 0.5;
    out.pose.orientation.x = 0.0;
    out.pose.orientation.y = 0.0;
    out.pose.orientation.z = std::sin(half_yaw);
    out.pose.orientation.w = std::cos(half_yaw);
    pose_pub_->publish(out);

    if (publish_covariance_ && pose_cov_pub_) {
      geometry_msgs::msg::PoseWithCovarianceStamped cov_msg;
      cov_msg.header = out.header;
      cov_msg.pose.pose = out.pose;
      std::copy(covariance_.begin(), covariance_.end(), cov_msg.pose.covariance.begin());
      pose_cov_pub_->publish(cov_msg);
    }
  }

  std::string navsat_topic_;
  std::string raw_pose_topic_;
  std::string pose_topic_;
  std::string utm_pose_topic_;
  std::string pose_cov_topic_;
  std::string map_frame_id_;
  bool publish_covariance_{true};
  std::array<double, 36> covariance_{};

  double origin_lat_deg_{0.0};
  double origin_lon_deg_{0.0};
  double origin_lat_rad_{0.0};
  double origin_lon_rad_{0.0};
  double origin_alt_{0.0};
  double yaw_offset_rad_{0.0};

  double utm_origin_easting_{0.0};
  double utm_origin_northing_{0.0};
  double utm_origin_alt_{0.0};

  Ecef reference_ecef_{};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr raw_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr utm_pose_sub_;
};
}  // namespace camping_cart::localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camping_cart::localization::NavsatToPoseNode>());
  rclcpp::shutdown();
  return 0;
}
