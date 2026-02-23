#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <string>
#include <vector>

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

class DropZoneMatcherNode : public rclcpp::Node
{
public:
  DropZoneMatcherNode()
  // 2026-02-02: Match localization pose against drop_zone centers to gate initial OK status.
  : Node("drop_zone_matcher")
  {
    drop_zones_yaml_ = declare_parameter<std::string>(
      "drop_zones_yaml", "/home/hong/cart_test_ws/src/camping_cart_map/config/drop_zones.yaml");
    pose_topic_ = declare_parameter<std::string>(
      "pose_topic", "/localization/pose_with_covariance");
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

    loadDropZones();

    using std::placeholders::_1;
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DropZoneMatcherNode::onPose, this, _1));

    status_pub_ = create_publisher<std_msgs::msg::Bool>(status_topic_, rclcpp::QoS(1));
    match_id_pub_ = create_publisher<std_msgs::msg::String>(match_id_topic_, rclcpp::QoS(1));
    match_distance_pub_ = create_publisher<std_msgs::msg::Float32>(match_distance_topic_, rclcpp::QoS(1));
    initialpose3d_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose3d_topic_, rclcpp::QoS(1));
  }

private:
  void loadDropZones()
  {
    zones_.clear();
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
        if (dz["corners"]) {
          for (const auto & c : dz["corners"]) {
            Point3 p;
            p.x = c["x"] ? c["x"].as<double>() : 0.0;
            p.y = c["y"] ? c["y"].as<double>() : 0.0;
            p.z = c["z"] ? c["z"].as<double>() : 0.0;
            zone.corners.push_back(p);
          }
        }
        zones_.push_back(zone);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "failed to load drop_zones_yaml: %s", ex.what());
    }
  }

  void onPose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    if (zones_.empty()) {
      publishStatus(false, "", std::numeric_limits<float>::infinity());
      return;
    }

    const double px = msg->pose.pose.position.x;
    const double py = msg->pose.pose.position.y;

    double best_dist = std::numeric_limits<double>::infinity();
    const DropZone * best = nullptr;

    for (const auto & z : zones_) {
      Point3 center = z.center;
      if (use_corners_center_ && !z.corners.empty()) {
        center = averagePoints(z.corners);
      }
      const double dx = px - center.x;
      const double dy = py - center.y;
      const double dist = std::hypot(dx, dy);
      if (dist < best_dist) {
        best_dist = dist;
        best = &z;
      }
    }

    const bool in_range = (best && best_dist <= match_radius_);
    if (in_range) {
      if (best_id_ == best->id) {
        stable_hits_++;
      } else {
        best_id_ = best->id;
        stable_hits_ = 1;
      }
    } else {
      best_id_.clear();
      stable_hits_ = 0;
    }

    const bool ok = (stable_hits_ >= stable_count_);
    publishStatus(ok, best ? best->id : "", static_cast<float>(best_dist));

    if (ok && publish_initialpose3d_) {
      if (!publish_once_ || !initialpose_published_) {
        publishInitialPose3d(*best, *msg);
        initialpose_published_ = true;
      }
    }
  }

  void publishInitialPose3d(
    const DropZone & zone,
    const geometry_msgs::msg::PoseWithCovarianceStamped & src)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header = src.header;
    Point3 center = zone.center;
    if (use_corners_center_ && !zone.corners.empty()) {
      center = averagePoints(zone.corners);
    }
    out.pose.pose.position.x = center.x;
    out.pose.pose.position.y = center.y;
    out.pose.pose.position.z = center.z;

    const double yaw = use_zone_yaw_ ? (zone.yaw_deg * M_PI / 180.0) : yawFromQuat(src.pose.pose.orientation);
    out.pose.pose.orientation = yawToQuat(yaw);

    out.pose.covariance = src.pose.covariance;
    initialpose3d_pub_->publish(out);
  }

  void publishStatus(bool ok, const std::string & id, float dist)
  {
    std_msgs::msg::Bool ok_msg;
    ok_msg.data = ok;
    status_pub_->publish(ok_msg);

    std_msgs::msg::String id_msg;
    id_msg.data = id;
    match_id_pub_->publish(id_msg);

    std_msgs::msg::Float32 dist_msg;
    dist_msg.data = dist;
    match_distance_pub_->publish(dist_msg);
  }

  std::string drop_zones_yaml_;
  std::string pose_topic_;
  double match_radius_{2.0};
  int stable_count_{10};
  bool publish_initialpose3d_{true};
  bool publish_once_{true};
  bool use_corners_center_{true};
  bool use_zone_yaw_{true};
  std::string initialpose3d_topic_;
  std::string status_topic_;
  std::string match_id_topic_;
  std::string match_distance_topic_;

  std::vector<DropZone> zones_;
  std::string best_id_;
  int stable_hits_{0};
  bool initialpose_published_{false};

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr match_id_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr match_distance_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose3d_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DropZoneMatcherNode>());
  rclcpp::shutdown();
  return 0;
}
