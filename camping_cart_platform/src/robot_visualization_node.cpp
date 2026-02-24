#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <unordered_map>
#include <vector>
#include <limits>
#include <cmath>

#include "camping_cart_sensor_kit/robot_params.hpp"  // HH_260109 renamed package

namespace camping_cart
{
namespace
{
// HH_260114 Keep a single label scale so every TF text marker is consistent across sensors.
constexpr double kLabelScale = 0.22;

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

geometry_msgs::msg::Quaternion quaternionFromRPY(double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion quat;
  quat.x = q.x();
  quat.y = q.y();
  quat.z = q.z();
  quat.w = q.w();
  return quat;
}

}  // namespace

struct PoseRPY
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

class RobotVisualizationNode : public rclcpp::Node
{
public:
  RobotVisualizationNode()
  // HH_260112 Use short node name; namespace applies the module prefix.
  : Node("robot_visualization")
  {
    params_ = loadRobotParams(this);
    map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "robot_base_link");
    // 2026-02-24: Platform ownership uses /platform/robot/* topics by default.
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/platform/robot/markers");
    boundary_topic_ = declare_parameter<std::string>(
      "boundary_topic", "/platform/robot/planning_boundary");
    // 2026-02-04: Visualizer should not publish TF unless explicitly requested.
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 1.0);
    body_scale_factor_ = declare_parameter<double>("body_scale_factor", 1.0);
    planning_boundary_margin_ = declare_parameter<double>("planning_boundary_margin", 0.3);
    ground_z_offset_ = declare_parameter<double>("ground_z_offset", 0.0);
    range_ring_radii_ = declare_parameter<std::vector<double>>(
      "range_ring_radii", std::vector<double>{2.0, 4.0, 6.0, 8.0});
    use_map_ground_z_ = declare_parameter<bool>("use_map_ground_z", true);
    base_pose_.x = declare_parameter<double>("base_pose.x", 0.0);
    base_pose_.y = declare_parameter<double>("base_pose.y", 0.0);
    base_pose_.z = declare_parameter<double>("base_pose.z", ground_z_offset_);
    base_pose_.roll = declare_parameter<double>("base_pose.roll", 0.0);
    base_pose_.pitch = declare_parameter<double>("base_pose.pitch", 0.0);
    base_pose_.yaw = declare_parameter<double>("base_pose.yaw", 0.0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, qos);
    boundary_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(boundary_topic_, qos);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }
    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/initialpose", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&RobotVisualizationNode::onInitialPose, this, std::placeholders::_1));
    gnss_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/sensing/gnss/pose", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&RobotVisualizationNode::onGnssPose, this, std::placeholders::_1));
    if (use_map_ground_z_) {
      map_marker_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/map/markers", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&RobotVisualizationNode::onMapMarkers, this, std::placeholders::_1));
    }

    publishBaseTransform();
    publishMarkers();

    using namespace std::chrono_literals;
    const auto period = publish_rate_hz > 0.0
      ? std::chrono::duration<double>(1.0 / publish_rate_hz)
      : std::chrono::seconds(1);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RobotVisualizationNode::publishMarkers, this));

    // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
    RCLCPP_DEBUG(
      get_logger(),
      "robot visualization ready. Marker topic '%s', base frame '%s', map frame '%s'.",
      marker_topic_.c_str(), base_frame_id_.c_str(), map_frame_id_.c_str());
  }

private:
  void publishBaseTransform()
  {
    if (!publish_tf_ || !tf_broadcaster_) {
      return;
    }
    geometry_msgs::msg::TransformStamped base_tf;
    base_tf.header.stamp = this->now();
    base_tf.header.frame_id = map_frame_id_;
    base_tf.child_frame_id = base_frame_id_;
    base_tf.transform.translation.x = base_pose_.x;
    base_tf.transform.translation.y = base_pose_.y;
    base_tf.transform.translation.z = base_pose_.z;
    base_tf.transform.rotation = quaternionFromRPY(base_pose_.roll, base_pose_.pitch, base_pose_.yaw);
    tf_broadcaster_->sendTransform(base_tf);
  }

  void publishMarkers()
  {
    publishBaseTransform();
    visualization_msgs::msg::MarkerArray markers;
    int32_t marker_id = 0;
    const auto now = this->get_clock()->now();
    tf2::Quaternion base_tf;
    base_tf.setRPY(base_pose_.roll, base_pose_.pitch, base_pose_.yaw);
    base_tf.normalize();
    tf2::Matrix3x3 base_rot(base_tf);
    const geometry_msgs::msg::Quaternion base_orientation = tf2::toMsg(base_tf);
    const geometry_msgs::msg::Point base_translation =
      makePoint(base_pose_.x, base_pose_.y, base_pose_.z);
    // HH_260114 Reusable map->base_link transform lambda shared by sensors/bounds/rings.
    const auto transformLocal = [&](double x, double y, double z) {
      const tf2::Vector3 rotated = base_rot * tf2::Vector3(x, y, z);
      return makePoint(
        base_translation.x + rotated.x(),
        base_translation.y + rotated.y(),
        base_translation.z + rotated.z());
    };

    const geometry_msgs::msg::Quaternion identity_orientation = quaternionFromRPY(0.0, 0.0, 0.0);
    markers.markers.emplace_back(
      createAxesMarker(
        "tf/world_axes", marker_id++, makePoint(0.0, 0.0, 0.0),
        identity_orientation, 2.0, now, "world"));
    visualization_msgs::msg::Marker world_label;
    world_label.header.frame_id = "world";
    world_label.header.stamp = now;
    world_label.ns = "tf/world_label";
    world_label.id = marker_id++;
    world_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    world_label.action = visualization_msgs::msg::Marker::ADD;
    world_label.pose.position = makePoint(0.0, 0.0, 0.6);
    world_label.scale.z = kLabelScale;
    world_label.color = makeColor(0.8f, 0.8f, 0.8f, 0.9f);
    world_label.text = "world";
    markers.markers.emplace_back(world_label);

    markers.markers.emplace_back(
      createAxesMarker(
        "tf/map_axes", marker_id++, makePoint(0.0, 0.0, 0.0),
        identity_orientation, 1.5, now, map_frame_id_));
    visualization_msgs::msg::Marker map_label;
    map_label.header.frame_id = map_frame_id_;
    map_label.header.stamp = now;
    map_label.ns = "tf/map_label";
    map_label.id = marker_id++;
    map_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    map_label.action = visualization_msgs::msg::Marker::ADD;
    map_label.pose.position = makePoint(0.0, 0.0, 0.5);
    map_label.scale.z = kLabelScale;
    map_label.color = makeColor(0.9f, 0.9f, 0.6f, 0.9f);
    map_label.text = map_frame_id_;
    markers.markers.emplace_back(map_label);

    // Vehicle bounding box (assume base frame is geometric center)
    visualization_msgs::msg::Marker body_marker;
    body_marker.header.frame_id = map_frame_id_;
    body_marker.header.stamp = now;
    body_marker.ns = "robot_body";
    body_marker.id = marker_id++;
    body_marker.type = visualization_msgs::msg::Marker::CUBE;
    body_marker.action = visualization_msgs::msg::Marker::ADD;
    body_marker.pose.position = transformLocal(0.0, 0.0, params_.height * 0.5);
    body_marker.pose.orientation = base_orientation;
    body_marker.scale.x = params_.length * body_scale_factor_;
    body_marker.scale.y = params_.width * body_scale_factor_;
    body_marker.scale.z = params_.height * body_scale_factor_;
    body_marker.color = makeColor(0.1f, 0.65f, 0.9f, 0.25f);
    markers.markers.emplace_back(body_marker);

    const double axis_length = 0.8;
    geometry_msgs::msg::Point base_origin = base_translation;
    markers.markers.emplace_back(
      createAxesMarker(
        "robot_base_link/axes", marker_id++, base_origin, base_orientation, axis_length, now,
        map_frame_id_));

    visualization_msgs::msg::Marker base_label;
    base_label.header.frame_id = map_frame_id_;
    base_label.header.stamp = now;
    base_label.ns = "robot_base_link";
    base_label.id = marker_id++;
    base_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    base_label.action = visualization_msgs::msg::Marker::ADD;
    base_label.pose.position = transformLocal(0.0, 0.0, params_.height * 0.5 + 0.3);
    base_label.scale.z = kLabelScale;
    base_label.color = makeColor(1.0f, 1.0f, 1.0f, 0.9f);
    base_label.text = base_frame_id_;
    markers.markers.emplace_back(base_label);

    // Footprint outline at z = 0
    visualization_msgs::msg::Marker footprint_marker;
    footprint_marker.header.frame_id = map_frame_id_;
    footprint_marker.header.stamp = now;
    footprint_marker.ns = "robot_footprint";
    footprint_marker.id = marker_id++;
    footprint_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    footprint_marker.action = visualization_msgs::msg::Marker::ADD;
    footprint_marker.scale.x = 0.02;
    footprint_marker.color = makeColor(0.15f, 0.8f, 0.9f, 0.8f);
    footprint_marker.pose.position = base_translation;
    footprint_marker.pose.orientation = base_orientation;
    const double half_length = (params_.length * body_scale_factor_) * 0.5;
    const double half_width = (params_.width * body_scale_factor_) * 0.5;
    footprint_marker.points.push_back(makePoint(half_length, half_width, 0.0));
    footprint_marker.points.push_back(makePoint(half_length, -half_width, 0.0));
    footprint_marker.points.push_back(makePoint(-half_length, -half_width, 0.0));
    footprint_marker.points.push_back(makePoint(-half_length, half_width, 0.0));
    footprint_marker.points.push_back(makePoint(half_length, half_width, 0.0));
    markers.markers.emplace_back(footprint_marker);

    visualization_msgs::msg::Marker boundary_marker;
    boundary_marker.header.frame_id = map_frame_id_;
    boundary_marker.header.stamp = now;
    boundary_marker.ns = "robot_planning_boundary";
    boundary_marker.id = marker_id++;
    boundary_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    boundary_marker.action = visualization_msgs::msg::Marker::ADD;
    boundary_marker.scale.x = 0.03;
    boundary_marker.color = makeColor(1.0f, 0.85f, 0.0f, 0.95f);
    boundary_marker.pose.position = base_translation;
    boundary_marker.pose.orientation = base_orientation;
    const double boundary_half_length = half_length + planning_boundary_margin_;
    const double boundary_half_width = half_width + planning_boundary_margin_;
    std::vector<geometry_msgs::msg::Point> boundary_local_points{
      makePoint(boundary_half_length, boundary_half_width, 0.0),
      makePoint(boundary_half_length, -boundary_half_width, 0.0),
      makePoint(-boundary_half_length, -boundary_half_width, 0.0),
      makePoint(-boundary_half_length, boundary_half_width, 0.0),
      makePoint(boundary_half_length, boundary_half_width, 0.0)};
    boundary_marker.points = boundary_local_points;
    markers.markers.emplace_back(boundary_marker);

    geometry_msgs::msg::PolygonStamped polygon_msg;
    polygon_msg.header.frame_id = map_frame_id_;
    polygon_msg.header.stamp = now;
    for (size_t i = 0; i < 4; ++i) {
      const auto map_point =
        transformLocal(boundary_local_points[i].x, boundary_local_points[i].y, 0.0);
      geometry_msgs::msg::Point32 p32;
      p32.x = map_point.x;
      p32.y = map_point.y;
      p32.z = 0.0;
      polygon_msg.polygon.points.emplace_back(p32);
    }
    boundary_pub_->publish(polygon_msg);

    const auto sensors = getSensorDictionary();
    // HH_260114 Use only axes+label namespaces per sensor to simplify RViz toggles.
    for (const auto & [name, pose] : sensors) {
      const std::string sensor_ns = "sensor/" + name;
      const auto sensor_position = transformLocal(pose.x, pose.y, pose.z);
      const auto sensor_orientation = composeOrientation(pose.roll, pose.pitch, pose.yaw);

      // HH_260114 Keep per-sensor toggles to axes/label only.
      markers.markers.emplace_back(
        createAxesMarker(
          sensor_ns + "/axes", marker_id++, sensor_position, sensor_orientation, 0.5, now,
          map_frame_id_));

      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = map_frame_id_;
      text_marker.header.stamp = now;
      text_marker.ns = sensor_ns + "/label";
      text_marker.id = marker_id++;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position = transformLocal(pose.x, pose.y, pose.z + 0.1);
      text_marker.pose.orientation = base_orientation;
      text_marker.scale.z = kLabelScale;
      text_marker.color = makeColor(1.0f, 1.0f, 1.0f, 0.9f);
      text_marker.text = name;
      markers.markers.emplace_back(text_marker);
    }

    if (!range_ring_radii_.empty()) {
      const int segments = 96;
      for (const auto radius : range_ring_radii_) {
        if (radius <= 0.0) {
          continue;
        }
        visualization_msgs::msg::Marker ring_marker;
        ring_marker.header.frame_id = map_frame_id_;
        ring_marker.header.stamp = now;
        ring_marker.ns = "range_rings";
        ring_marker.id = marker_id++;
        ring_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        ring_marker.action = visualization_msgs::msg::Marker::ADD;
        ring_marker.pose.position = base_translation;
        ring_marker.pose.orientation = base_orientation;
        ring_marker.scale.x = 0.015;
        ring_marker.color = makeColor(0.6f, 0.6f, 0.6f, 0.4f);
        for (int i = 0; i <= segments; ++i) {
          const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(segments);
          ring_marker.points.push_back(makePoint(radius * std::cos(theta), radius * std::sin(theta), 0.0));
        }
        markers.markers.emplace_back(std::move(ring_marker));
      }
    }

    marker_pub_->publish(markers);
  }

  std::unordered_map<std::string, SensorPose> getSensorDictionary() const
  {
    return {
      {"imu", params_.imu},
      {"gnss", params_.gnss},
      {"lidar", params_.lidar},
      {"camera_front", params_.camera_front}
    };
  }

  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    const auto & pose = msg->pose.pose;
    base_pose_.x = pose.position.x;
    base_pose_.y = pose.position.y;
    base_pose_.z = pose.position.z + ground_z_offset_ + mapGroundOffset();
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    base_pose_.roll = roll;
    base_pose_.pitch = pitch;
    base_pose_.yaw = yaw;
    publishBaseTransform();
    publishMarkers();
    RCLCPP_INFO(
      get_logger(),
      "initial pose set to (%.2f, %.2f, %.2f, yaw %.1f deg)",
      base_pose_.x, base_pose_.y, base_pose_.z, yaw * 180.0 / M_PI);
  }

  void onGnssPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    auto converted = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    converted->header = msg->header;
    converted->pose.pose = msg->pose;
    onInitialPose(converted);
  }

  double mapGroundOffset() const
  {
    return use_map_ground_z_ && map_ground_ready_ ? map_ground_z_ : 0.0;
  }

  // HH_260109 Use const shared pointer to avoid deprecated subscription callback warnings.
  void onMapMarkers(const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)
  {
    if (!use_map_ground_z_ || map_ground_ready_) {
      return;
    }
    constexpr size_t sample_limit = 1000;
    for (const auto & marker : msg->markers) {
      if (marker.ns.rfind("lanelet/", 0) != 0) {
        continue;
      }
      for (const auto & point : marker.points) {
        map_ground_sum_ += point.z;
        ++map_ground_samples_;
        if (map_ground_samples_ >= sample_limit) {
          break;
        }
      }
      if (map_ground_samples_ >= sample_limit) {
        break;
      }
    }
    if (map_ground_samples_ >= sample_limit) {
      map_ground_z_ = map_ground_sum_ / static_cast<double>(map_ground_samples_);
      map_ground_ready_ = true;
      base_pose_.z = map_ground_z_ + ground_z_offset_;
      RCLCPP_INFO(
        get_logger(), "detected lanelet ground height %.3f m", map_ground_z_);
      if (map_marker_sub_) {
        map_marker_sub_.reset();
      }
      publishMarkers();
    }
  }

  RobotParams params_;
  PoseRPY base_pose_;
  double planning_boundary_margin_{0.3};
  double body_scale_factor_{1.0};
  double ground_z_offset_{0.0};
  bool use_map_ground_z_{true};
  bool publish_tf_{false};
  std::vector<double> range_ring_radii_;
  std::string map_frame_id_;
  std::string base_frame_id_;
  std::string marker_topic_;
  std::string boundary_topic_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr boundary_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr map_marker_sub_;

  double map_ground_z_{0.0};
  double map_ground_sum_{0.0};
  size_t map_ground_samples_{0};
  bool map_ground_ready_{false};

  geometry_msgs::msg::Quaternion composeOrientation(double roll, double pitch, double yaw) const
  {
    tf2::Quaternion base_q;
    base_q.setRPY(base_pose_.roll, base_pose_.pitch, base_pose_.yaw);
    tf2::Quaternion offset_q;
    offset_q.setRPY(roll, pitch, yaw);
    tf2::Quaternion q = base_q * offset_q;
    q.normalize();
    return tf2::toMsg(q);
  }

  visualization_msgs::msg::Marker createAxesMarker(
    const std::string & ns, int32_t id,
    const geometry_msgs::msg::Point & origin,
    const geometry_msgs::msg::Quaternion & orientation,
    double length, const rclcpp::Time & stamp,
    const std::string & frame_id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.04;

    tf2::Quaternion q;
    tf2::fromMsg(orientation, q);
    tf2::Matrix3x3 rot(q);
    tf2::Vector3 origin_vec(origin.x, origin.y, origin.z);

    auto tipPoint = [&](const tf2::Vector3 & axis) {
      tf2::Vector3 tip = origin_vec + rot * axis;
      geometry_msgs::msg::Point p;
      p.x = tip.x();
      p.y = tip.y();
      p.z = tip.z();
      return p;
    };

    marker.points.push_back(origin);
    marker.points.push_back(tipPoint(tf2::Vector3(length, 0.0, 0.0)));
    auto color_x = makeColor(1.0f, 0.0f, 0.0f, 0.9f);
    marker.colors.push_back(color_x);
    marker.colors.push_back(color_x);

    marker.points.push_back(origin);
    marker.points.push_back(tipPoint(tf2::Vector3(0.0, length, 0.0)));
    auto color_y = makeColor(0.0f, 1.0f, 0.0f, 0.9f);
    marker.colors.push_back(color_y);
    marker.colors.push_back(color_y);

    marker.points.push_back(origin);
    marker.points.push_back(tipPoint(tf2::Vector3(0.0, 0.0, length)));
    auto color_z = makeColor(0.0f, 0.6f, 1.0f, 0.9f);
    marker.colors.push_back(color_z);
    marker.colors.push_back(color_z);
    return marker;
  }
};

}  // namespace camping_cart

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camping_cart::RobotVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
