#ifndef CAMROD_MAP__LANELET2_MAP_NODE_HPP_
#define CAMROD_MAP__LANELET2_MAP_NODE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <avg_msgs/msg/pose_stamped.hpp>
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
#include <lanelet2_core/primitives/Area.h>

namespace camrod
{
namespace map
{

struct Lanelet2MapNodeConfig
{
  // Lanelet2 OSM path.
  std::string map_path;
  // WGS84 latitude origin for projector.
  double offset_lat{0.0};
  // WGS84 longitude origin for projector.
  double offset_lon{0.0};
  // WGS84 altitude origin for projector.
  double offset_alt{0.0};
  // Parent world frame for static transform publication.
  std::string world_frame_id{"world"};
  // Lanelet map frame used by all map markers.
  std::string map_frame_id{"map"};
  // HH_260103 arrow scaling parameters
  double dir_body_scale{0.55};
  double dir_head_scale{0.35};
  double dir_width_scale{0.18};
  std::size_t dir_stride{3};
};

struct VisualizationFilter
{
  double x{0.0};
  double y{0.0};
  double radius{0.0};
  double radius_sq{0.0};
};

class Lanelet2MapNode : public rclcpp::Node
{
public:
  Lanelet2MapNode();

private:
  void loadParameters();
  bool loadMap();
  void startVisualization();
  void publishVisualization();
  void publishVisualization(const VisualizationFilter * filter, const char * mode_label);
  void publishFullVisualization(bool force = false);
  void scheduleDetailedFullVisualization();
  void publishDetailedFullVisualization();
  void publishCachedVisualization();
  void publishAvgMapMessage(
    const avg_msgs::msg::MarkerArray & markers,
    const rclcpp::Time & stamp);
  void publishStaticTF();
  void onProgressivePose(const avg_msgs::msg::PoseStamped::ConstSharedPtr msg);
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
  avg_msgs::msg::Point makeMapPoint(double x, double y, double z) const;  // HH_260623 - Project OSM geometry onto the configured visualization ground plane.
  static avg_msgs::msg::Point computeCentroid(const lanelet::ConstLineString3d & line_string);  // HH_260114 Compute semantic centroid.
  bool isNearVisualizationCenter(double x, double y) const;
  bool isLineStringNear(const lanelet::ConstLineString3d & line_string) const;
  bool isLaneletNear(const lanelet::ConstLanelet & lanelet) const;
  bool isAreaNear(const lanelet::ConstArea & area) const;
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
  rclcpp::TimerBase::SharedPtr progressive_full_viz_timer_;
  rclcpp::TimerBase::SharedPtr progressive_detailed_full_viz_timer_;
  rclcpp::Subscription<avg_msgs::msg::PoseStamped>::SharedPtr progressive_pose_sub_;
  rclcpp::Subscription<avg_msgs::msg::PoseStamped>::SharedPtr progressive_fallback_pose_sub_;
  bool logged_full_marker_stats_{false};
  bool logged_detailed_full_marker_stats_{false};
  bool logged_local_marker_stats_{false};
  bool logged_empty_local_marker_warning_{false};
  bool debug_timing_{true};
  bool progressive_visualization_enable_{false};
  bool progressive_visualization_lightweight_local_{true};
  bool progressive_visualization_lightweight_full_{false};
  bool progressive_visualization_publish_detailed_full_{false};
  bool visualization_publish_raw_points_{false};
  bool progressive_local_visualization_published_{false};
  bool progressive_full_visualization_published_{false};
  bool progressive_detailed_full_visualization_published_{false};
  double progressive_visualization_radius_m_{120.0};
  double progressive_visualization_full_delay_s_{12.0};
  double progressive_visualization_detailed_full_delay_s_{30.0};
  std::string progressive_visualization_pose_topic_{"/localization/pose"};
  std::string progressive_visualization_fallback_pose_topic_{""};
  const VisualizationFilter * active_visualization_filter_{nullptr};
  // HH_260413: Optional periodic re-publish period for static map markers.
  // 0.0 disables timer and publishes once (transient_local keeps late subscribers synced).
  double visualization_republish_period_s_{0.0};
  avg_msgs::msg::MarkerArray cached_markers_;
  bool publish_map_status_{false};
  std::string map_status_topic_{"/map/status"};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::Publisher<avg_msgs::msg::AvgMapMsgs>::SharedPtr avg_map_pub_;
};

}  // namespace map
}  // namespace camrod

#endif  // CAMROD_MAP__LANELET2_MAP_NODE_HPP_
