#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <avg_msgs/msg/pose_stamped.hpp>
#include <avg_msgs/msg/pose_with_covariance_stamped.hpp>
#include <avg_msgs/msg/bool.hpp>
#include <avg_msgs/msg/string.hpp>
#include <avg_msgs/msg/float32.hpp>
#include <avg_msgs/msg/avg_localization_msgs.hpp>
#include <avg_msgs/msg/module_state.hpp>
#include <avg_msgs/msg/quaternion.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/LocalCartesian.h>

#include <yaml-cpp/yaml.h>
#include "camrod_map/camrod_map/custom_regulatory_elements.hpp"  // Register custom lanelet rules (e.g., speed_bump).

namespace
{
struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct DropZone
{
  std::string id;
  std::string type;
  Point3 center;
  double yaw_deg{0.0};
  std::vector<Point3> corners;
};

struct NearestResult
{
  double sq_dist{std::numeric_limits<double>::max()};
  bool valid{false};
  lanelet::ConstPoint3d nearest_point;
  double heading{0.0};
};

avg_msgs::msg::Quaternion yawToQuat(double yaw)
{
  avg_msgs::msg::Quaternion q;
  const double half = yaw * 0.5;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

double yawFromQuat(const avg_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

Point3 averagePoints(const std::vector<Point3> & pts)
{
  Point3 out;
  if (pts.empty()) {
    return out;
  }
  for (const auto & p : pts) {
    out.x += p.x;
    out.y += p.y;
    out.z += p.z;
  }
  const double inv = 1.0 / static_cast<double>(pts.size());
  out.x *= inv;
  out.y *= inv;
  out.z *= inv;
  return out;
}

}  // namespace

namespace camrod::localization
{

class LocalizationMapHelperNode : public rclcpp::Node
{
public:
  LocalizationMapHelperNode()
  : Node("localization_map_helper")
  {
    enable_centerline_snapper_ = declare_parameter<bool>("enable_centerline_snapper", true);
    enable_drop_zone_matcher_ = declare_parameter<bool>("enable_drop_zone_matcher", true);

    publish_localization_status_ =
      declare_parameter<bool>("publish_localization_status", false);
    localization_status_topic_ =
      declare_parameter<std::string>("localization_status_topic", "/localization/status");

    map_path_ = declare_parameter<std::string>("map_path", "");
    offset_lat_ = declare_parameter<double>("offset_lat", 0.0);
    offset_lon_ = declare_parameter<double>("offset_lon", 0.0);
    offset_alt_ = declare_parameter<double>("offset_alt", 0.0);
    centerline_input_pose_topic_ = declare_parameter<std::string>(
      "centerline_input_pose_topic", "/localization/pose");
    centerline_output_pose_topic_ = declare_parameter<std::string>(
      "centerline_output_pose_topic", "/localization/lanelet_pose");
    max_search_radius_ = declare_parameter<double>("max_search_radius", 30.0);
    longitudinal_stddev_ = declare_parameter<double>("longitudinal_stddev", 0.5);
    lateral_stddev_ = declare_parameter<double>("lateral_stddev", 0.3);
    yaw_stddev_ = declare_parameter<double>("yaw_stddev", 0.2);
    use_map_z_ = declare_parameter<bool>("use_map_z", true);
    map_z_offset_ = declare_parameter<double>("map_z_offset", 0.0);
    flatten_to_ground_ = declare_parameter<bool>("flatten_to_ground", false);

    drop_zones_yaml_ = declare_parameter<std::string>("drop_zones_yaml", "");
    drop_zone_pose_topic_ = declare_parameter<std::string>(
      "drop_zone_pose_topic", "/localization/pose_with_covariance");
    match_radius_ = declare_parameter<double>("match_radius", 2.0);
    stable_count_ = declare_parameter<int>("stable_count", 10);
    publish_initialpose3d_ = declare_parameter<bool>("publish_initialpose3d", true);
    publish_once_ = declare_parameter<bool>("publish_once", true);
    use_corners_center_ = declare_parameter<bool>("use_corners_center", true);
    use_zone_yaw_ = declare_parameter<bool>("use_zone_yaw", true);
    initialpose3d_topic_ = declare_parameter<std::string>(
      "initialpose3d_topic", "/localization/initialpose3d");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/localization/initial_match_ok");
    match_id_topic_ = declare_parameter<std::string>(
      "match_id_topic", "/localization/initial_match_id");
    match_distance_topic_ = declare_parameter<std::string>(
      "match_distance_topic", "/localization/initial_match_distance");

    if (enable_centerline_snapper_) {
      if (!loadLaneletMap()) {
        RCLCPP_FATAL(get_logger(), "Failed to load lanelet map");
        throw std::runtime_error("Failed to load lanelet map");
      }
      centerline_pub_ =
        create_publisher<avg_msgs::msg::PoseWithCovarianceStamped>(centerline_output_pose_topic_, rclcpp::QoS(10));
      centerline_sub_ = create_subscription<avg_msgs::msg::PoseStamped>(
        centerline_input_pose_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LocalizationMapHelperNode::onCenterlinePose, this, std::placeholders::_1));
    }

    if (enable_drop_zone_matcher_) {
      loadDropZones();
      status_pub_ = create_publisher<avg_msgs::msg::Bool>(status_topic_, rclcpp::QoS(1));
      match_id_pub_ = create_publisher<avg_msgs::msg::String>(match_id_topic_, rclcpp::QoS(1));
      match_distance_pub_ = create_publisher<avg_msgs::msg::Float32>(match_distance_topic_, rclcpp::QoS(1));
      initialpose3d_pub_ =
        create_publisher<avg_msgs::msg::PoseWithCovarianceStamped>(initialpose3d_topic_, rclcpp::QoS(1));
      drop_zone_sub_ = create_subscription<avg_msgs::msg::PoseWithCovarianceStamped>(
        drop_zone_pose_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LocalizationMapHelperNode::onDropZonePose, this, std::placeholders::_1));
    }

    if (publish_localization_status_) {
      avg_localization_pub_ = create_publisher<avg_msgs::msg::AvgLocalizationMsgs>(
        localization_status_topic_, rclcpp::QoS(10));
    }
  }

private:
  bool loadLaneletMap()
  {
    lanelet::GPSPoint gps;
    gps.lat = offset_lat_;
    gps.lon = offset_lon_;
    gps.ele = offset_alt_;
    lanelet::Origin origin(gps);
    lanelet::projection::LocalCartesianProjector projector(origin);

    try {
      map_ = lanelet::load(map_path_, projector);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Exception loading map %s: %s", map_path_.c_str(), e.what());
      return false;
    }
    if (!map_) {
      RCLCPP_ERROR(get_logger(), "lanelet::load returned nullptr for %s", map_path_.c_str());
      return false;
    }

    std::vector<double> zs;
    zs.reserve(map_->pointLayer.size());
    for (const auto & pt : map_->pointLayer) {
      zs.push_back(pt.z());
    }
    if (zs.empty()) {
      map_ground_z_ = 0.0;
    } else {
      std::nth_element(zs.begin(), zs.begin() + zs.size() / 2, zs.end());
      map_ground_z_ = zs[zs.size() / 2];
    }
    return true;
  }

  NearestResult findNearestCenterline(double x, double y) const
  {
    NearestResult best;
    const double max_sq = max_search_radius_ * max_search_radius_;
    for (const auto & ll : map_->laneletLayer) {
      const auto & cl = ll.centerline();
      if (cl.size() < 2) {
        continue;
      }
      for (size_t i = 0; i + 1 < cl.size(); ++i) {
        const auto p0 = cl[i];
        const auto p1 = cl[i + 1];
        const double vx = p1.x() - p0.x();
        const double vy = p1.y() - p0.y();
        const double wx = x - p0.x();
        const double wy = y - p0.y();
        const double seg_len2 = vx * vx + vy * vy + 1e-6;
        double t = (vx * wx + vy * wy) / seg_len2;
        t = std::max(0.0, std::min(1.0, t));
        const double proj_x = p0.x() + t * vx;
        const double proj_y = p0.y() + t * vy;
        const double dx = x - proj_x;
        const double dy = y - proj_y;
        const double dist2 = dx * dx + dy * dy;
        if (dist2 < best.sq_dist && dist2 < max_sq) {
          best.sq_dist = dist2;
          best.valid = true;
          const double proj_z = p0.z() + t * (p1.z() - p0.z());
          best.nearest_point = lanelet::Point3d(lanelet::InvalId, proj_x, proj_y, proj_z);
          best.heading = std::atan2(vy, vx);
        }
      }
    }
    return best;
  }

  void onCenterlinePose(const avg_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    if (!map_) {
      return;
    }
    const double px = msg->pose.position.x;
    const double py = msg->pose.position.y;
    const double pz = msg->pose.position.z;

    const auto nearest = findNearestCenterline(px, py);
    if (!nearest.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No centerline within search radius %.1f m", max_search_radius_);
      return;
    }

    avg_msgs::msg::PoseWithCovarianceStamped out;
    out.header = msg->header;
    out.pose.pose.position.x = nearest.nearest_point.x();
    out.pose.pose.position.y = nearest.nearest_point.y();

    double snapped_z = use_map_z_ ? (nearest.nearest_point.z() + map_z_offset_) : pz;
    if (flatten_to_ground_) {
      snapped_z = map_ground_z_ + map_z_offset_;
    }
    out.pose.pose.position.z = snapped_z;
    out.pose.pose.orientation = yawToQuat(nearest.heading);

    for (auto & c : out.pose.covariance) {
      c = 0.0;
    }
    out.pose.covariance[0] = longitudinal_stddev_ * longitudinal_stddev_;
    out.pose.covariance[7] = lateral_stddev_ * lateral_stddev_;
    out.pose.covariance[14] = 9999.0;
    out.pose.covariance[21] = 9999.0;
    out.pose.covariance[28] = 9999.0;
    out.pose.covariance[35] = yaw_stddev_ * yaw_stddev_;

    centerline_pub_->publish(out);
  }

  void loadDropZones()
  {
    zones_.clear();
    if (drop_zones_yaml_.empty()) {
      RCLCPP_WARN(get_logger(), "drop_zones_yaml is empty");
      return;
    }

    try {
      YAML::Node root = YAML::LoadFile(drop_zones_yaml_);
      auto list = root["drop_zones"];
      if (!list || !list.IsSequence()) {
        RCLCPP_WARN(get_logger(), "drop_zones missing or invalid in %s", drop_zones_yaml_.c_str());
        return;
      }

      for (const auto & dz : list) {
        DropZone zone;
        zone.id = dz["id"] ? dz["id"].as<std::string>() : "";
        zone.type = dz["type"] ? dz["type"].as<std::string>() : "";
        zone.center.x = dz["x"] ? dz["x"].as<double>() : 0.0;
        zone.center.y = dz["y"] ? dz["y"].as<double>() : 0.0;
        zone.center.z = dz["z"] ? dz["z"].as<double>() : 0.0;
        zone.yaw_deg = dz["yaw_deg"] ? dz["yaw_deg"].as<double>() : 0.0;

        if (dz["corners"] && dz["corners"].IsSequence()) {
          for (const auto & c : dz["corners"]) {
            Point3 p;
            p.x = c["x"] ? c["x"].as<double>() : 0.0;
            p.y = c["y"] ? c["y"].as<double>() : 0.0;
            p.z = c["z"] ? c["z"].as<double>() : 0.0;
            zone.corners.push_back(p);
          }
        }

        if (use_corners_center_ && !zone.corners.empty()) {
          zone.center = averagePoints(zone.corners);
        }
        zones_.push_back(zone);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to load drop zones: %s", e.what());
    }
  }

  void onDropZonePose(const avg_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    const double px = msg->pose.pose.position.x;
    const double py = msg->pose.pose.position.y;
    const double pz = msg->pose.pose.position.z;

    double best_dist = std::numeric_limits<double>::max();
    const DropZone * best_zone = nullptr;

    for (const auto & zone : zones_) {
      const double dx = px - zone.center.x;
      const double dy = py - zone.center.y;
      const double dz = pz - zone.center.z;
      const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist < best_dist) {
        best_dist = dist;
        best_zone = &zone;
      }
    }

    avg_msgs::msg::Bool ok_msg;
    avg_msgs::msg::String id_msg;
    avg_msgs::msg::Float32 dist_msg;
    dist_msg.data = std::isfinite(best_dist) ? static_cast<float>(best_dist) : -1.0f;

    if (best_zone && best_dist <= match_radius_) {
      stable_match_count_++;
      last_match_id_ = best_zone->id;
      last_match_distance_ = best_dist;

      ok_msg.data = stable_match_count_ >= stable_count_;
      id_msg.data = best_zone->id;
    } else {
      stable_match_count_ = 0;
      ok_msg.data = false;
      id_msg.data = "";
      last_match_id_.clear();
      last_match_distance_ = std::numeric_limits<double>::infinity();
    }

    status_pub_->publish(ok_msg);
    match_id_pub_->publish(id_msg);
    match_distance_pub_->publish(dist_msg);

    if (ok_msg.data && publish_initialpose3d_) {
      if (!publish_once_ || !published_initialpose_) {
        avg_msgs::msg::PoseWithCovarianceStamped out;
        out.header = msg->header;
        out.pose.pose.position.x = best_zone->center.x;
        out.pose.pose.position.y = best_zone->center.y;
        out.pose.pose.position.z = best_zone->center.z;
        out.pose.pose.orientation = use_zone_yaw_
          ? yawToQuat(best_zone->yaw_deg * M_PI / 180.0)
          : msg->pose.pose.orientation;
        out.pose.covariance = msg->pose.covariance;
        initialpose3d_pub_->publish(out);
        published_initialpose_ = true;
      }
    }

    if (publish_localization_status_ && avg_localization_pub_) {
      avg_msgs::msg::AvgLocalizationMsgs out;
      out.stamp = msg->header.stamp;
      out.state.stamp = msg->header.stamp;
      out.state.module_name = "localization";
      out.state.level = ok_msg.data ?
        avg_msgs::msg::ModuleState::OK :
        avg_msgs::msg::ModuleState::WARN;
      out.state.message = ok_msg.data ?
        "drop_zone_match_ok" :
        "drop_zone_match_wait";
      avg_localization_pub_->publish(out);
    }
  }

  bool enable_centerline_snapper_{true};
  bool enable_drop_zone_matcher_{true};
  bool publish_localization_status_{false};
  std::string localization_status_topic_;

  std::string map_path_;
  double offset_lat_{0.0}, offset_lon_{0.0}, offset_alt_{0.0};
  std::string centerline_input_pose_topic_;
  std::string centerline_output_pose_topic_;
  double max_search_radius_{30.0};
  double longitudinal_stddev_{0.5};
  double lateral_stddev_{0.3};
  double yaw_stddev_{0.2};
  bool use_map_z_{true};
  double map_z_offset_{0.0};
  bool flatten_to_ground_{false};
  double map_ground_z_{0.0};
  lanelet::LaneletMapPtr map_;
  rclcpp::Publisher<avg_msgs::msg::PoseWithCovarianceStamped>::SharedPtr centerline_pub_;
  rclcpp::Subscription<avg_msgs::msg::PoseStamped>::SharedPtr centerline_sub_;

  std::string drop_zones_yaml_;
  std::string drop_zone_pose_topic_;
  double match_radius_{2.0};
  int stable_count_{10};
  int stable_match_count_{0};
  bool publish_initialpose3d_{true};
  bool publish_once_{true};
  bool use_corners_center_{true};
  bool use_zone_yaw_{true};
  bool published_initialpose_{false};
  std::string last_match_id_;
  double last_match_distance_{std::numeric_limits<double>::infinity()};
  std::vector<DropZone> zones_;

  std::string initialpose3d_topic_;
  std::string status_topic_;
  std::string match_id_topic_;
  std::string match_distance_topic_;
  rclcpp::Publisher<avg_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::Publisher<avg_msgs::msg::String>::SharedPtr match_id_pub_;
  rclcpp::Publisher<avg_msgs::msg::Float32>::SharedPtr match_distance_pub_;
  rclcpp::Publisher<avg_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose3d_pub_;
  rclcpp::Subscription<avg_msgs::msg::PoseWithCovarianceStamped>::SharedPtr drop_zone_sub_;

  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationMsgs>::SharedPtr avg_localization_pub_;
};

}  // namespace camrod::localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod::localization::LocalizationMapHelperNode>());
  rclcpp::shutdown();
  return 0;
}
