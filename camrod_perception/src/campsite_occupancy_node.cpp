#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <avg_msgs/msg/campsite_occupancy.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <yaml-cpp/yaml.h>

namespace camrod::perception
{

class CampsiteOccupancyNode : public rclcpp::Node
{
public:
  CampsiteOccupancyNode()
  : Node("campsite_occupancy")
  {
    detection_topic_ = declare_parameter<std::string>(
      "detection_topic", "/perception/camera_lidar/detections_3d");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/perception/camping_sites/occupancy");
    camping_sites_yaml_ = declare_parameter<std::string>("camping_sites_yaml", "");
    map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map");
    tent_class_labels_ = declare_parameter<std::vector<std::string>>(
      "tent_class_labels", {"tent", "80"});
    minimum_confidence_ = declare_parameter<double>("minimum_confidence", 0.50);
    confirm_hits_ = std::max(
      1, static_cast<int>(declare_parameter<int>("confirm_hits", 3)));
    confirm_window_s_ = std::max(0.1, declare_parameter<double>("confirm_window_s", 2.0));
    occupied_hold_s_ = std::max(0.0, declare_parameter<double>("occupied_hold_s", 3600.0));
    publish_rate_hz_ = std::max(0.1, declare_parameter<double>("publish_rate_hz", 2.0));

    for (const auto & label : tent_class_labels_) {
      tent_class_labels_normalized_.insert(normalize(label));
    }
    loadSites();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ = create_publisher<avg_msgs::msg::CampsiteOccupancy>(
      output_topic_, rclcpp::QoS(1).transient_local().reliable());
    subscription_ = create_subscription<vision_msgs::msg::Detection3DArray>(
      detection_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CampsiteOccupancyNode::onDetections, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CampsiteOccupancyNode::publishState, this));

    RCLCPP_INFO(
      get_logger(),
      "campsite_occupancy ready: detections=%s sites=%zu confirm=%d/%.1fs hold=%.0fs output=%s",
      detection_topic_.c_str(), sites_.size(), confirm_hits_, confirm_window_s_,
      occupied_hold_s_, output_topic_.c_str());
  }

private:
  struct Point2d
  {
    double x{0.0};
    double y{0.0};
  };

  struct Site
  {
    std::string mission_key;
    std::vector<Point2d> polygon;
  };

  static std::string normalize(std::string value)
  {
    std::transform(
      value.begin(), value.end(), value.begin(),
      [](const unsigned char character) {
        return static_cast<char>(std::tolower(character));
      });
    return value;
  }

  void loadSites()
  {
    sites_.clear();
    if (camping_sites_yaml_.empty()) {
      RCLCPP_ERROR(get_logger(), "camping_sites_yaml is empty; occupancy cannot be resolved");
      return;
    }
    try {
      const YAML::Node document = YAML::LoadFile(camping_sites_yaml_);
      const YAML::Node sites = document["camping_sites"];
      if (!sites || !sites.IsSequence()) {
        RCLCPP_ERROR(
          get_logger(), "invalid camping_sites_yaml format: %s", camping_sites_yaml_.c_str());
        return;
      }
      for (std::size_t index = 0; index < sites.size(); ++index) {
        const YAML::Node item = sites[index];
        Site site;
        site.mission_key = item["type"] ?
          item["type"].as<std::string>() : "camping_site_" + std::to_string(index + 1);
        const YAML::Node corners = item["corners"];
        if (!corners || !corners.IsSequence()) {
          continue;
        }
        for (const auto & corner : corners) {
          if (corner["x"] && corner["y"]) {
            site.polygon.push_back({corner["x"].as<double>(), corner["y"].as<double>()});
          }
        }
        if (!site.mission_key.empty() && site.polygon.size() >= 3U) {
          sites_.push_back(std::move(site));
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "failed to load camping_sites_yaml (%s): %s",
        camping_sites_yaml_.c_str(), error.what());
    }
  }

  static bool pointInPolygon(
    const double x, const double y, const std::vector<Point2d> & polygon)
  {
    bool inside = false;
    std::size_t previous = polygon.size() - 1U;
    for (std::size_t current = 0; current < polygon.size(); ++current) {
      const auto & a = polygon[current];
      const auto & b = polygon[previous];
      const bool crosses = (a.y > y) != (b.y > y);
      if (crosses) {
        const double denominator = b.y - a.y;
        if (std::abs(denominator) > 1.0e-9) {
          const double crossing_x = (b.x - a.x) * (y - a.y) / denominator + a.x;
          if (x < crossing_x) {
            inside = !inside;
          }
        }
      }
      previous = current;
    }
    return inside;
  }

  bool isTent(const vision_msgs::msg::Detection3D & detection) const
  {
    for (const auto & result : detection.results) {
      if (result.hypothesis.score >= minimum_confidence_ &&
        tent_class_labels_normalized_.count(normalize(result.hypothesis.class_id)) > 0U)
      {
        return true;
      }
    }
    return false;
  }

  geometry_msgs::msg::Point detectionPoint(
    const vision_msgs::msg::Detection3D & detection) const
  {
    if (detection.bbox.center.position.x != 0.0 ||
      detection.bbox.center.position.y != 0.0 ||
      detection.bbox.center.position.z != 0.0)
    {
      return detection.bbox.center.position;
    }
    if (!detection.results.empty()) {
      return detection.results.front().pose.pose.position;
    }
    return geometry_msgs::msg::Point();
  }

  void onDetections(const vision_msgs::msg::Detection3DArray::ConstSharedPtr message)
  {
    if (!message || sites_.empty()) {
      return;
    }
    std::set<std::string> sites_seen_this_frame;
    for (const auto & detection : message->detections) {
      if (!isTent(detection)) {
        continue;
      }
      geometry_msgs::msg::PointStamped source;
      source.header = detection.header;
      if (source.header.frame_id.empty()) {
        source.header = message->header;
      }
      source.point = detectionPoint(detection);

      geometry_msgs::msg::PointStamped in_map;
      try {
        in_map = tf_buffer_->transform(source, map_frame_id_, tf2::durationFromSec(0.05));
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "tent transform %s->%s failed: %s",
          source.header.frame_id.c_str(), map_frame_id_.c_str(), error.what());
        continue;
      }

      for (const auto & site : sites_) {
        if (pointInPolygon(in_map.point.x, in_map.point.y, site.polygon)) {
          sites_seen_this_frame.insert(site.mission_key);
        }
      }
    }

    const rclcpp::Time current_time = now();
    for (const auto & mission_key : sites_seen_this_frame) {
      auto & hits = confirmation_hits_[mission_key];
      hits.push_back(current_time);
      while (!hits.empty() && (current_time - hits.front()).seconds() > confirm_window_s_) {
        hits.pop_front();
      }
      if (static_cast<int>(hits.size()) >= confirm_hits_) {
        const bool newly_occupied = occupied_last_seen_.count(mission_key) == 0U;
        occupied_last_seen_[mission_key] = current_time;
        if (newly_occupied) {
          RCLCPP_WARN(
            get_logger(), "campsite occupied by confirmed tent: %s", mission_key.c_str());
        }
      }
    }
  }

  void publishState()
  {
    const rclcpp::Time current_time = now();
    avg_msgs::msg::CampsiteOccupancy message;
    message.header.stamp = current_time;
    message.header.frame_id = map_frame_id_;
    message.source = "campsite_occupancy:tent";

    for (auto iterator = occupied_last_seen_.begin(); iterator != occupied_last_seen_.end(); ) {
      const bool expired = occupied_hold_s_ > 0.0 &&
        (current_time - iterator->second).seconds() > occupied_hold_s_;
      if (expired) {
        RCLCPP_INFO(
          get_logger(), "campsite occupancy expired: %s", iterator->first.c_str());
        iterator = occupied_last_seen_.erase(iterator);
      } else {
        message.occupied_mission_keys.push_back(iterator->first);
        ++iterator;
      }
    }
    std::sort(
      message.occupied_mission_keys.begin(), message.occupied_mission_keys.end());
    publisher_->publish(message);
  }

  std::string detection_topic_;
  std::string output_topic_;
  std::string camping_sites_yaml_;
  std::string map_frame_id_;
  std::vector<std::string> tent_class_labels_;
  std::set<std::string> tent_class_labels_normalized_;
  double minimum_confidence_{0.50};
  int confirm_hits_{3};
  double confirm_window_s_{2.0};
  double occupied_hold_s_{3600.0};
  double publish_rate_hz_{2.0};
  std::vector<Site> sites_;
  std::map<std::string, std::deque<rclcpp::Time>> confirmation_hits_;
  std::map<std::string, rclcpp::Time> occupied_last_seen_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<avg_msgs::msg::CampsiteOccupancy>::SharedPtr publisher_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace camrod::perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod::perception::CampsiteOccupancyNode>());
  rclcpp::shutdown();
  return 0;
}
