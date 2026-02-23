#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/LocalCartesian.h>

#include "camping_cart_map/custom_regulatory_elements.hpp"  // HH_260114 Register speed_bump rule.

#include <limits>
#include <string>

namespace
{
struct LoaderConfig
{
  std::string map_path;
  double offset_lat{0.0};
  double offset_lon{0.0};
  double offset_alt{0.0};
};

struct NearestResult
{
  double sq_dist{std::numeric_limits<double>::max()};
  bool valid{false};
  lanelet::ConstPoint3d nearest_point;
  double heading{0.0};
};

geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  const double half_yaw = yaw * 0.5;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half_yaw);
  q.w = std::cos(half_yaw);
  return q;
}

}  // namespace

class CenterlineSnapperNode : public rclcpp::Node
{
public:
  CenterlineSnapperNode()
  // HH_260112 Use short node name; namespace applies the module prefix.
  : rclcpp::Node("centerline_snapper")
  {
    // Parameters
    cfg_.map_path = declare_parameter<std::string>("map_path", "");
    cfg_.offset_lat = declare_parameter<double>("offset_lat", 0.0);
    cfg_.offset_lon = declare_parameter<double>("offset_lon", 0.0);
    cfg_.offset_alt = declare_parameter<double>("offset_alt", 0.0);
    // HH_260109 Default to fused localization pose and publish lanelet constraint pose.
    input_pose_topic_ = declare_parameter<std::string>("input_pose_topic", "/localization/pose");
    output_pose_topic_ = declare_parameter<std::string>(
      "output_pose_topic", "/localization/lanelet_pose");
    max_search_radius_ = declare_parameter<double>("max_search_radius", 30.0);
    longitudinal_stddev_ = declare_parameter<double>("longitudinal_stddev", 0.5);
    lateral_stddev_ = declare_parameter<double>("lateral_stddev", 0.3);
    yaw_stddev_ = declare_parameter<double>("yaw_stddev", 0.2);
    use_map_z_ = declare_parameter<bool>("use_map_z", true);           // HH_260114 Snap z to map elevation.
    map_z_offset_ = declare_parameter<double>("map_z_offset", 0.0);    // HH_260114 Extra z offset applied to map elevation.
    flatten_to_ground_ = declare_parameter<bool>("flatten_to_ground", false);  // HH_260114 Force z to ground plane.

    if (!loadMap()) {
      RCLCPP_FATAL(get_logger(), "centerline snapper: failed to load map. exiting.");
      rclcpp::shutdown();
      return;
    }

    pub_pose_cov_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_pose_topic_, rclcpp::QoS(10));
    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CenterlineSnapperNode::onPose, this, std::placeholders::_1));

    // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
    RCLCPP_DEBUG(
      get_logger(),
      "centerline snapper ready. map=%s, input=%s, output=%s",
      cfg_.map_path.c_str(), input_pose_topic_.c_str(), output_pose_topic_.c_str());
  }

private:
  bool loadMap()
  {
    lanelet::GPSPoint gps;
    gps.lat = cfg_.offset_lat;
    gps.lon = cfg_.offset_lon;
    gps.ele = cfg_.offset_alt;
    lanelet::Origin origin(gps);
    lanelet::projection::LocalCartesianProjector projector(origin);

    try {
      map_ = lanelet::load(cfg_.map_path, projector);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Exception loading map %s: %s", cfg_.map_path.c_str(), e.what());
      return false;
    }
    if (!map_) {
      RCLCPP_ERROR(get_logger(), "lanelet::load returned nullptr for %s", cfg_.map_path.c_str());
      return false;
    }
    RCLCPP_DEBUG(
      get_logger(), "Loaded lanelet map: lanelets=%zu linestrings=%zu",
      map_->laneletLayer.size(), map_->lineStringLayer.size());
    map_ground_z_ = computeGroundZ(*map_);
    RCLCPP_DEBUG(get_logger(), "map ground z (median)=%.3f", map_ground_z_);
    return true;
  }

  void onPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    const double px = msg->pose.position.x;
    const double py = msg->pose.position.y;
    const double pz = msg->pose.position.z;

    const auto nearest = findNearestCenterline(px, py);
    if (!nearest.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "No centerline within search radius %.1f m",
        max_search_radius_);
      return;
    }
    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header = msg->header;
    out.pose.pose.position.x = nearest.nearest_point.x();
    out.pose.pose.position.y = nearest.nearest_point.y();
    // HH_260114 Adjust z to lanelet centerline height or keep input z.
    double snapped_z = use_map_z_ ? (nearest.nearest_point.z() + map_z_offset_) : pz;
    if (flatten_to_ground_) {
      snapped_z = map_ground_z_ + map_z_offset_;
    }
    out.pose.pose.position.z = snapped_z;
    out.pose.pose.orientation = yawToQuat(nearest.heading);

    // Fill covariance: longitudinal, lateral, yaw
    for (auto & c : out.pose.covariance) {
      c = 0.0;
    }
    out.pose.covariance[0] = longitudinal_stddev_ * longitudinal_stddev_;
    out.pose.covariance[7] = lateral_stddev_ * lateral_stddev_;
    out.pose.covariance[14] = 9999.0;  // z not observed
    out.pose.covariance[21] = 9999.0;
    out.pose.covariance[28] = 9999.0;
    out.pose.covariance[35] = yaw_stddev_ * yaw_stddev_;

    pub_pose_cov_->publish(out);
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
          const double proj_z = p0.z() + t * (p1.z() - p0.z());  // HH_260114 Interpolate z along segment.
          best.nearest_point = lanelet::Point3d(lanelet::InvalId, proj_x, proj_y, proj_z);
          best.heading = std::atan2(vy, vx);
        }
      }
    }
    return best;
  }

  LoaderConfig cfg_;
  lanelet::LaneletMapPtr map_;

  std::string input_pose_topic_;
  std::string output_pose_topic_;
  double max_search_radius_{30.0};
  double longitudinal_stddev_{0.5};
  double lateral_stddev_{0.3};
  double yaw_stddev_{0.2};
  bool use_map_z_{true};
  double map_z_offset_{0.0};
  bool flatten_to_ground_{false};
  double map_ground_z_{0.0};

  double computeGroundZ(const lanelet::LaneletMap & map)
  {
    std::vector<double> zs;
    zs.reserve(map.pointLayer.size());
    for (const auto & pt : map.pointLayer) {
      zs.push_back(pt.z());
    }
    if (zs.empty()) {
      return 0.0;
    }
    std::nth_element(zs.begin(), zs.begin() + zs.size() / 2, zs.end());
    return zs[zs.size() / 2];
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CenterlineSnapperNode>());
  rclcpp::shutdown();
  return 0;
}
