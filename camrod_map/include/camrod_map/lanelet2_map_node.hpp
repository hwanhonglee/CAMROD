#ifndef CAMROD_MAP__LANELET2_MAP_NODE_HPP_
#define CAMROD_MAP__LANELET2_MAP_NODE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <avg_msgs/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <avg_msgs/msg/marker_array.hpp>

#include <avg_msgs/msg/avg_map_msgs.hpp>

#include "camrod_map/lanelet2_map_loader.hpp"
#include <avg_msgs/msg/marker.hpp>
#include <avg_msgs/msg/color_rgba.hpp>
#include <avg_msgs/msg/point.hpp>

namespace camping_cart
{
namespace map
{

struct Lanelet2MapNodeConfig
{
  std::string map_path;
  double offset_lat{0.0};
  double offset_lon{0.0};
  double offset_alt{0.0};
  std::string world_frame_id{"world"};
  std::string map_frame_id{"map"};
  // HH_260103 arrow scaling parameters
  double dir_body_scale{0.55};
  double dir_head_scale{0.35};
  double dir_width_scale{0.18};
  std::size_t dir_stride{3};
};

class Lanelet2MapNode : public rclcpp::Node
{
public:
  Lanelet2MapNode();

private:
  // HH_251215: dark-map visualization helpers
  void loadParameters();
  bool loadMap();
  void publishVisualization();
  void publishAvgMapMessage(
    const avg_msgs::msg::MarkerArray & markers,
    const rclcpp::Time & stamp);
  void publishStaticTF();
  avg_msgs::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params);
  bool reloadMapWithConfig(const Lanelet2MapNodeConfig & new_config);
  std::size_t addLaneletCenterlines(
    avg_msgs::msg::MarkerArray & markers, int32_t & id_counter,
    const rclcpp::Time & stamp) const;
  std::size_t addLaneletBounds(
    avg_msgs::msg::MarkerArray & markers, int32_t & id_counter,
    const rclcpp::Time & stamp) const;
  std::size_t addAreas(
    avg_msgs::msg::MarkerArray & markers, int32_t & id_counter,
    const rclcpp::Time & stamp) const;
  std::size_t addLineStrings(
    avg_msgs::msg::MarkerArray & markers, int32_t & id_counter,
    const rclcpp::Time & stamp) const;
  std::size_t addPoints(
    avg_msgs::msg::MarkerArray & markers, int32_t & id_counter,
    const rclcpp::Time & stamp) const;
  std::size_t addLaneletDirections(
    avg_msgs::msg::MarkerArray & markers, int32_t & id_counter,
    const rclcpp::Time & stamp) const;
  std::size_t addLaneletIds(
    avg_msgs::msg::MarkerArray & markers, int32_t & id_counter,
    const rclcpp::Time & stamp) const;
  std::size_t addSemanticMarkers(
    avg_msgs::msg::MarkerArray & markers, int32_t & id_counter,
    const rclcpp::Time & stamp) const;
  static avg_msgs::msg::Marker initLineMarker(
    const std::string & ns, int32_t id, const std::string & frame_id,
    const avg_msgs::msg::ColorRGBA & color, double width, const rclcpp::Time & stamp);
  avg_msgs::msg::ColorRGBA colorFromSubtype(
    const std::string & subtype, const avg_msgs::msg::ColorRGBA & fallback) const;
  double lineWidthFromSubtype(const std::string & subtype) const;
  static std::string sanitizeNamespace(const std::string & prefix, const std::string & subtype);
  static std::string groupedNamespace(const std::string & group, const std::string & subtype);
  static avg_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a = 1.0f);
  avg_msgs::msg::Point makePoint(double x, double y, double z) const;
  static avg_msgs::msg::Point computeCentroid(const lanelet::ConstLineString3d & line_string);  // HH_260114 Compute semantic centroid.
  bool computeFlatArrow(
    const lanelet::ConstLineString3d & centerline, std::size_t tail_idx, std::size_t head_idx,
    double lane_width,
    avg_msgs::msg::Point & tail_left,
    avg_msgs::msg::Point & tail_right,
    avg_msgs::msg::Point & head_point) const;
  double laneWidthAt(const lanelet::ConstLanelet & lanelet, std::size_t idx) const;
  void addTrafficLightBulbs(  // HH_260114 Render tri-color traffic light bulbs.
    const avg_msgs::msg::Point & base_center,
    const std::string & bulb_namespace,
    avg_msgs::msg::MarkerArray & markers,
    int32_t & id_counter,
    const rclcpp::Time & stamp) const;

  Lanelet2MapNodeConfig config_;
  Lanelet2MapLoader loader_;
  lanelet::LaneletMapPtr loaded_map_;
  bool align_z_to_ground_{true};
  double map_ground_z_{0.0};

  double computeGroundZ(const lanelet::LaneletMap & map) const;

  rclcpp::Publisher<avg_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr viz_timer_;
  bool logged_marker_stats_{false};
  bool publish_map_diagnostic_{false};
  std::string map_diagnostic_topic_{"/map/diagnostic"};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::Publisher<avg_msgs::msg::AvgMapMsgs>::SharedPtr avg_map_pub_;
};

}  // namespace map
}  // namespace camping_cart

#endif  // CAMROD_MAP__LANELET2_MAP_NODE_HPP_
