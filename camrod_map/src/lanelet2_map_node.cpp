#include "camrod_map/lanelet2_map_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <functional>

#include <avg_msgs/msg/point.hpp>
#include <avg_msgs/msg/transform_stamped.hpp>
#include <avg_msgs/msg/color_rgba.hpp>

#include <avg_msgs/msg/module_state.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <unordered_map>
#include <unordered_set>
#include <avg_msgs/msg/marker.hpp>
#include <avg_msgs/msg/marker_array.hpp>

namespace camrod
{
namespace map
{
// HH_251215: semantic lanelet visualization utilities
namespace
{

struct SemanticTags
{
  std::string type;
  std::string subtype;
};

std::string toLowerCopy(const std::string & value)
{
  std::string lower = value;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return lower;
}

template<typename PrimitiveT>
std::string getAttributeCaseInsensitive(const PrimitiveT & primitive, const std::string & key)
{
  std::string value = primitive.attributeOr(key.c_str(), "");
  if (!value.empty()) {
    return value;
  }
  std::string lower = key;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  value = primitive.attributeOr(lower.c_str(), "");
  if (!value.empty()) {
    return value;
  }
  std::string upper = key;
  std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return primitive.attributeOr(upper.c_str(), "");
}

template<typename PrimitiveT>
std::string getSemanticType(const PrimitiveT & primitive)
{
  auto value = getAttributeCaseInsensitive(primitive, "Subtype");
  if (value.empty()) {
    value = getAttributeCaseInsensitive(primitive, "type");
  }
  return toLowerCopy(value);
}

template<typename PrimitiveT>
// Extracts normalized semantic type/subtype tags from lanelet attributes.
SemanticTags extractSemanticTags(const PrimitiveT & primitive)
{
  SemanticTags tags;
  tags.subtype = toLowerCopy(getAttributeCaseInsensitive(primitive, "Subtype"));
  tags.type = toLowerCopy(getAttributeCaseInsensitive(primitive, "type"));
  return tags;
}

}  // namespace


Lanelet2MapNode::Lanelet2MapNode()
  // HH_260112 Use short node name; namespace applies the module prefix.
  : Node("lanelet2_map"), loader_(LoaderConfig())
{
  loadParameters();

  LoaderConfig loader_cfg;
  loader_cfg.offset_lat = config_.offset_lat;
  loader_cfg.offset_lon = config_.offset_lon;
  loader_cfg.offset_alt = config_.offset_alt;
  loader_ = Lanelet2MapLoader(loader_cfg);

  if (!loadMap()) {
    RCLCPP_FATAL(
      get_logger(),
      "Lanelet2 map could not be loaded from '%s'. Shutting down node.",
      config_.map_path.c_str());
    rclcpp::shutdown();
    return;
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  // HH_260109 Publish map markers under /map prefix.
  viz_pub_ = this->create_publisher<avg_msgs::msg::MarkerArray>(
    "/map/markers", qos);
  if (publish_map_status_) {
    avg_map_pub_ = this->create_publisher<avg_msgs::msg::AvgMapMsgs>(
      map_status_topic_, rclcpp::QoS(10));
  }

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  startVisualization();
  publishStaticTF();

  // HH_260413: Keep static map marker computation one-shot by default.
  // Recompute on timer only when explicitly requested through parameter.
  if (visualization_republish_period_s_ > 0.0) {
    viz_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(visualization_republish_period_s_)),
      std::bind(&Lanelet2MapNode::publishCachedVisualization, this));
  }

  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&Lanelet2MapNode::onParameterChange, this, std::placeholders::_1));
}

void Lanelet2MapNode::loadParameters()
{
  config_.map_path = this->declare_parameter<std::string>("map_path", "");
  config_.offset_lat = this->declare_parameter<double>("offset_lat", 0.0);
  config_.offset_lon = this->declare_parameter<double>("offset_lon", 0.0);
  config_.offset_alt = this->declare_parameter<double>("offset_alt", 0.0);
  config_.world_frame_id = this->declare_parameter<std::string>("world_frame_id", "world");
  config_.map_frame_id = this->declare_parameter<std::string>("map_frame_id", "map");
  // HH_260114 Expose arrow size scaling to quickly tune RViz mismatch.
  config_.dir_body_scale = this->declare_parameter<double>("dir_body_scale", 0.55);
  config_.dir_head_scale = this->declare_parameter<double>("dir_head_scale", 0.35);
  config_.dir_width_scale = this->declare_parameter<double>("dir_width_scale", 0.18);
  // HH_260114 Arrow generation stride.
  config_.dir_stride = static_cast<std::size_t>(this->declare_parameter<int>("dir_stride", 30));
  // HH_260623 - Default RViz/planning visualization uses the 2D map ground plane.
  // Raw OSM altitude stays in the loader; marker Z is flattened unless explicitly disabled.
  align_z_to_ground_ = this->declare_parameter<bool>("align_z_to_ground", true);
  visualization_republish_period_s_ = this->declare_parameter<double>(
    "visualization_republish_period_s", 0.0);
  progressive_visualization_enable_ = this->declare_parameter<bool>(
    "progressive_visualization_enable", false);
  progressive_visualization_radius_m_ = this->declare_parameter<double>(
    "progressive_visualization_radius_m", 120.0);
  progressive_visualization_full_delay_s_ = this->declare_parameter<double>(
    "progressive_visualization_full_delay_s", 4.0);
  progressive_visualization_pose_topic_ = this->declare_parameter<std::string>(
    "progressive_visualization_pose_topic", "/sensing/gnss/pose");
  progressive_visualization_fallback_pose_topic_ = this->declare_parameter<std::string>(
    "progressive_visualization_fallback_pose_topic", "/localization/pose");
  progressive_visualization_lightweight_local_ = this->declare_parameter<bool>(
    "progressive_visualization_lightweight_local", true);
  progressive_visualization_lightweight_full_ = this->declare_parameter<bool>(
    "progressive_visualization_lightweight_full", false);
  debug_timing_ = this->declare_parameter<bool>("debug_timing", true);
  publish_map_status_ = this->declare_parameter<bool>("publish_map_status", false);
  map_status_topic_ = this->declare_parameter<std::string>("map_status_topic", "/map/status");
}

bool Lanelet2MapNode::loadMap()
{
  const auto load_t0 = std::chrono::steady_clock::now();
  loaded_map_ = loader_.load(config_.map_path);
  const auto load_t1 = std::chrono::steady_clock::now();
  if (!loaded_map_) {
    RCLCPP_ERROR(
      get_logger(), "Failed to parse Lanelet2 map file '%s'", config_.map_path.c_str());
    return false;
  }

  RCLCPP_INFO(get_logger(), "%s", loader_.getMapStats().c_str());
  if (debug_timing_) {
    const double load_ms =
      std::chrono::duration<double, std::milli>(load_t1 - load_t0).count();
    RCLCPP_INFO(
      get_logger(),
      "map load timing: %.2fms path=%s",
      load_ms, config_.map_path.c_str());
  }
  return true;
}

void Lanelet2MapNode::startVisualization()
{
  if (!progressive_visualization_enable_) {
    publishFullVisualization(true);
    return;
  }

  auto pose_callback = std::bind(&Lanelet2MapNode::onProgressivePose, this, std::placeholders::_1);
  if (!progressive_visualization_pose_topic_.empty()) {
    progressive_pose_sub_ = this->create_subscription<avg_msgs::msg::PoseStamped>(
      progressive_visualization_pose_topic_, rclcpp::QoS(10), pose_callback);
  }
  if (!progressive_visualization_fallback_pose_topic_.empty() &&
    progressive_visualization_fallback_pose_topic_ != progressive_visualization_pose_topic_)
  {
    progressive_fallback_pose_sub_ = this->create_subscription<avg_msgs::msg::PoseStamped>(
      progressive_visualization_fallback_pose_topic_, rclcpp::QoS(10), pose_callback);
  }

  const double delay_s = std::max(0.0, progressive_visualization_full_delay_s_);
  if (delay_s <= 0.0) {
    publishFullVisualization(true);
    return;
  }

  progressive_full_viz_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(delay_s)),
    [this]() { publishFullVisualization(); });

  const std::string fallback_text = progressive_visualization_fallback_pose_topic_.empty() ?
    "" :
    ", fallback " + progressive_visualization_fallback_pose_topic_;
  RCLCPP_INFO(
    get_logger(),
    "Progressive map visualization enabled: local radius %.1fm from %s%s, full map after %.1fs",
    progressive_visualization_radius_m_,
    progressive_visualization_pose_topic_.c_str(),
    fallback_text.c_str(),
    delay_s);
}

void Lanelet2MapNode::publishVisualization()
{
  publishVisualization(nullptr, "full");
}

void Lanelet2MapNode::publishVisualization(
  const VisualizationFilter * filter,
  const char * mode_label)
{
  if (!loaded_map_) {
    RCLCPP_WARN(get_logger(), "No map loaded. Visualization skipped.");
    return;
  }

  const auto * previous_filter = active_visualization_filter_;
  active_visualization_filter_ = filter;
  const auto viz_t0 = std::chrono::steady_clock::now();

  avg_msgs::msg::MarkerArray markers;
  int32_t marker_id = 0;

  const auto stamp = now();
  const bool local_mode = filter != nullptr;
  const bool lightweight_local = local_mode && progressive_visualization_lightweight_local_;
  const bool lightweight_full = !local_mode && progressive_visualization_lightweight_full_;
  const bool lightweight_mode = lightweight_local || lightweight_full;
  std::size_t centerline_count = 0U;
  std::size_t line_string_count = 0U;
  // HH_260625: The first RViz paint must not compute lazy centerlines or
  // semantic/text markers for the whole map. Bounds give the operator immediate
  // local context while planning/lifecycle nodes finish startup.
  if (!lightweight_local) {
    centerline_count = addLaneletCenterlines(markers, marker_id, stamp);
  }
  const auto bound_count = addLaneletBounds(markers, marker_id, stamp);
  const auto area_count = addAreas(markers, marker_id, stamp);
  if (!lightweight_local) {
    line_string_count = addLineStrings(markers, marker_id, stamp);
  }
  std::size_t point_count = 0U;
  std::size_t direction_count = 0U;
  std::size_t id_count = 0U;
  std::size_t semantic_count = 0U;
  if (!lightweight_mode) {
    point_count = addPoints(markers, marker_id, stamp);
    direction_count = addLaneletDirections(markers, marker_id, stamp);
    id_count = addLaneletIds(markers, marker_id, stamp);
    semantic_count = addSemanticMarkers(markers, marker_id, stamp);
  }

  const bool should_log =
    (local_mode && !logged_local_marker_stats_) ||
    (!local_mode && !logged_full_marker_stats_);
  if (should_log) {
    RCLCPP_INFO(
      get_logger(),
      "Visualization markers (%s%s) -> centerlines: %zu, bounds: %zu, areas: %zu, linestrings: %zu, points/text: %zu",
      mode_label,
      local_mode ? ", filtered" : "",
      centerline_count, bound_count, area_count,
      line_string_count + direction_count + semantic_count,
      point_count + id_count + semantic_count);
    if (local_mode) {
      logged_local_marker_stats_ = true;
    } else {
      logged_full_marker_stats_ = true;
    }
  }

  if (markers.markers.empty()) {
    RCLCPP_WARN(get_logger(), "Loaded map contains no lanelets to visualize for %s mode.", mode_label);
    if (local_mode) {
      // HH_260625: A startup fallback pose can briefly be outside the loaded map.
      // Keep waiting for the next GNSS/localization pose instead of caching an empty local map.
      logged_local_marker_stats_ = false;
      active_visualization_filter_ = previous_filter;
      return;
    }
  }

  cached_markers_ = markers;
  const auto build_t1 = std::chrono::steady_clock::now();
  viz_pub_->publish(cached_markers_);
  publishAvgMapMessage(markers, stamp);
  const auto publish_t1 = std::chrono::steady_clock::now();
  if (debug_timing_ && should_log) {
    const double build_ms =
      std::chrono::duration<double, std::milli>(build_t1 - viz_t0).count();
    const double publish_ms =
      std::chrono::duration<double, std::milli>(publish_t1 - build_t1).count();
    const double total_ms =
      std::chrono::duration<double, std::milli>(publish_t1 - viz_t0).count();
    RCLCPP_INFO(
      get_logger(),
      "map visualization timing (%s%s): markers=%zu build=%.2fms publish=%.2fms total=%.2fms",
      mode_label,
      local_mode ? ", filtered" : "",
      markers.markers.size(),
      build_ms, publish_ms, total_ms);
  }

  if (local_mode) {
    progressive_local_visualization_published_ = true;
  } else {
    progressive_full_visualization_published_ = true;
  }
  active_visualization_filter_ = previous_filter;
}

void Lanelet2MapNode::publishFullVisualization(bool force)
{
  if (progressive_full_viz_timer_) {
    progressive_full_viz_timer_->cancel();
  }
  if (!force && progressive_full_visualization_published_) {
    return;
  }
  publishVisualization(nullptr, "full");
}

void Lanelet2MapNode::publishCachedVisualization()
{
  if (cached_markers_.markers.empty()) {
    if (progressive_visualization_enable_ && !progressive_full_visualization_published_) {
      return;
    }
    publishFullVisualization(true);
    return;
  }
  viz_pub_->publish(cached_markers_);
}

void Lanelet2MapNode::onProgressivePose(const avg_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  if (!progressive_visualization_enable_ ||
    progressive_local_visualization_published_ ||
    progressive_full_visualization_published_)
  {
    return;
  }

  if (progressive_visualization_radius_m_ <= 0.0) {
    publishFullVisualization(true);
    return;
  }

  VisualizationFilter filter;
  filter.x = msg->pose.position.x;
  filter.y = msg->pose.position.y;
  filter.radius = progressive_visualization_radius_m_;
  filter.radius_sq = filter.radius * filter.radius;
  publishVisualization(&filter, "local");

  const double delay_s = std::max(0.0, progressive_visualization_full_delay_s_);
  if (delay_s <= 0.0) {
    publishFullVisualization(true);
    return;
  }
  if (progressive_full_viz_timer_) {
    progressive_full_viz_timer_->cancel();
  }
  // HH_260625: Count the full-map delay from the first local publish, not node startup.
  // Otherwise a slow local render can let the full-map timer fire immediately afterward.
  progressive_full_viz_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(delay_s)),
    [this]() { publishFullVisualization(); });
}

avg_msgs::msg::SetParametersResult Lanelet2MapNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & params)
{
  avg_msgs::msg::SetParametersResult result;
  result.successful = true;

  Lanelet2MapNodeConfig new_config = config_;
  bool requires_reload = false;
  bool requires_tf_update = false;

  auto ensure_string = [&](const rclcpp::Parameter & param, std::string & target) -> bool {
    if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      result.successful = false;
      result.reason = "Invalid type for parameter '" + param.get_name() + "'";
      return false;
    }
    target = param.as_string();
    return true;
  };

  auto ensure_double = [&](const rclcpp::Parameter & param, double & target) -> bool {
    if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
      param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      result.successful = false;
      result.reason = "Invalid type for parameter '" + param.get_name() + "'";
      return false;
    }
    target = param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ?
      param.as_double() :
      static_cast<double>(param.as_int());
    return true;
  };

  auto ensure_bool = [&](const rclcpp::Parameter & param, bool & target) -> bool {
    if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
      result.successful = false;
      result.reason = "Invalid type for parameter '" + param.get_name() + "'";
      return false;
    }
    target = param.as_bool();
    return true;
  };

  bool new_align_z_to_ground = align_z_to_ground_;
  bool requires_visualization_update = false;

  for (const auto & param : params) {
    if (param.get_name() == "map_path") {
      if (!ensure_string(param, new_config.map_path)) {
        return result;
      }
      requires_reload = true;
    } else if (param.get_name() == "offset_lat") {
      if (!ensure_double(param, new_config.offset_lat)) {
        return result;
      }
      requires_reload = true;
    } else if (param.get_name() == "offset_lon") {
      if (!ensure_double(param, new_config.offset_lon)) {
        return result;
      }
      requires_reload = true;
    } else if (param.get_name() == "offset_alt") {
      if (!ensure_double(param, new_config.offset_alt)) {
        return result;
      }
      requires_reload = true;
    } else if (param.get_name() == "world_frame_id") {
      if (!ensure_string(param, new_config.world_frame_id)) {
        return result;
      }
      requires_tf_update = true;
    } else if (param.get_name() == "map_frame_id") {
      if (!ensure_string(param, new_config.map_frame_id)) {
        return result;
      }
      requires_tf_update = true;
    } else if (param.get_name() == "dir_body_scale") {
      ensure_double(param, new_config.dir_body_scale);
    } else if (param.get_name() == "dir_head_scale") {
      ensure_double(param, new_config.dir_head_scale);
    } else if (param.get_name() == "dir_width_scale") {
      ensure_double(param, new_config.dir_width_scale);
    } else if (param.get_name() == "dir_stride") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        new_config.dir_stride = static_cast<std::size_t>(param.as_int());
      }
    } else if (param.get_name() == "align_z_to_ground") {
      if (!ensure_bool(param, new_align_z_to_ground)) {
        return result;
      }
      requires_visualization_update = true;
    } else {
      // Allow other parameters to be set without affecting the map node.
      continue;
    }
  }

  if (!requires_reload) {
    config_ = new_config;
    align_z_to_ground_ = new_align_z_to_ground;
    if (requires_tf_update) {
      publishStaticTF();
    }
    if (requires_visualization_update) {
      publishVisualization();
    }
    return result;
  }

  if (!reloadMapWithConfig(new_config)) {
    result.successful = false;
    result.reason = "Failed to reload Lanelet2 map with updated parameters";
    return result;
  }

  config_ = new_config;
  align_z_to_ground_ = new_align_z_to_ground;
  publishFullVisualization(true);
  publishStaticTF();
  return result;
}

// Rebuilds loader/map objects when projection/path parameters change.
bool Lanelet2MapNode::reloadMapWithConfig(const Lanelet2MapNodeConfig & new_config)
{
  LoaderConfig loader_cfg;
  loader_cfg.offset_lat = new_config.offset_lat;
  loader_cfg.offset_lon = new_config.offset_lon;
  loader_cfg.offset_alt = new_config.offset_alt;

  Lanelet2MapLoader new_loader(loader_cfg);
  auto new_map = new_loader.load(new_config.map_path);
  if (!new_map) {
    RCLCPP_ERROR(
      get_logger(), "Failed to reload map from '%s'", new_config.map_path.c_str());
    return false;
  }

  loader_ = new_loader;
  loaded_map_ = new_map;
  RCLCPP_INFO(get_logger(), "%s", loader_.getMapStats().c_str());
  return true;
}

std::size_t Lanelet2MapNode::addLaneletCenterlines(
  avg_msgs::msg::MarkerArray & markers, int32_t & id_counter,
  const rclcpp::Time & stamp) const
{
  // HH_260114 Light color centerlines, continuous line style.
  const auto color = makeColor(0.45f, 0.65f, 0.70f, 0.7f);
  std::size_t count = 0;
  for (const auto & lanelet : loaded_map_->laneletLayer) {
    if (!isLaneletNear(lanelet)) {
      continue;
    }
    auto marker = initLineMarker(
      "lanelet/centerline", id_counter++, config_.map_frame_id, color, 0.12, stamp);
    marker.type = avg_msgs::msg::Marker::LINE_STRIP;

    for (const auto & point : lanelet.centerline()) {
      marker.points.push_back(makeMapPoint(point.x(), point.y(), point.z()));
    }
    markers.markers.emplace_back(std::move(marker));
    ++count;
  }
  return count;
}

std::size_t Lanelet2MapNode::addLaneletBounds(
  avg_msgs::msg::MarkerArray & markers, int32_t & id_counter,
  const rclcpp::Time & stamp) const
{
  // HH_260114 Slightly thinner bounds with mid-tone gray.
  const auto left_color = makeColor(0.70f, 0.70f, 0.75f, 0.85f);
  const auto right_color = makeColor(0.70f, 0.70f, 0.75f, 0.85f);
  std::size_t count = 0;

  for (const auto & lanelet : loaded_map_->laneletLayer) {
    if (!isLaneletNear(lanelet)) {
      continue;
    }
    auto left_marker = initLineMarker(
      "lanelet/left_bound", id_counter++, config_.map_frame_id, left_color, 0.18, stamp);
    for (const auto & point : lanelet.leftBound()) {
      left_marker.points.push_back(makeMapPoint(point.x(), point.y(), point.z()));
    }
    markers.markers.emplace_back(std::move(left_marker));
    ++count;

    auto right_marker = initLineMarker(
      "lanelet/right_bound", id_counter++, config_.map_frame_id, right_color, 0.18, stamp);
    for (const auto & point : lanelet.rightBound()) {
      right_marker.points.push_back(makeMapPoint(point.x(), point.y(), point.z()));
    }
    markers.markers.emplace_back(std::move(right_marker));
    ++count;
  }
  return count;
}

std::size_t Lanelet2MapNode::addAreas(
  avg_msgs::msg::MarkerArray & markers, int32_t & id_counter,
  const rclcpp::Time & stamp) const
{
  std::size_t count = 0;
  // HH_260114 Thicken area outlines slightly so they are visible but calm.
  const auto area_color = makeColor(0.55f, 0.65f, 0.78f, 0.35f);
  for (const auto & area : loaded_map_->areaLayer) {
    if (!isAreaNear(area)) {
      continue;
    }
    auto marker = initLineMarker(
      "lanelet/area", id_counter++, config_.map_frame_id, area_color, 0.08, stamp);
    marker.type = avg_msgs::msg::Marker::LINE_STRIP;

    for (const auto & bound : area.outerBound()) {
      for (const auto & point : bound) {
        marker.points.push_back(makeMapPoint(point.x(), point.y(), point.z()));
      }
      // close each bound loop
      if (!bound.empty()) {
        const auto & first = bound.front();
        marker.points.push_back(makeMapPoint(first.x(), first.y(), first.z()));
      }
    }

    markers.markers.emplace_back(std::move(marker));
    ++count;
  }
  return count;
}

std::size_t Lanelet2MapNode::addLineStrings(
  avg_msgs::msg::MarkerArray & markers, int32_t & id_counter,
  const rclcpp::Time & stamp) const
{
  std::size_t count = 0;
  for (const auto & line_string : loaded_map_->lineStringLayer) {
    if (!isLineStringNear(line_string)) {
      continue;
    }
    auto subtype = getSemanticType(line_string);
    if (subtype.empty()) {
      subtype = "line_string";
    }
    const auto sanitized = sanitizeNamespace("line", subtype);
    if (sanitized == "line_red_yellow_green" || sanitized == "line_stop_sign" || sanitized == "line_virtual") {
      continue;
    }
    const auto color = colorFromSubtype(subtype, makeColor(0.8f, 0.8f, 0.8f, 0.4f));
    auto marker = initLineMarker(
      groupedNamespace("line", subtype), id_counter++, config_.map_frame_id,
      color, lineWidthFromSubtype(subtype), stamp);

    for (const auto & point : line_string) {
      marker.points.push_back(makeMapPoint(point.x(), point.y(), point.z()));
    }
    markers.markers.emplace_back(std::move(marker));
    if (!marker.points.empty()) {
      ++count;
    }
  }
  return count;
}

std::size_t Lanelet2MapNode::addPoints(
  avg_msgs::msg::MarkerArray & markers, int32_t & id_counter,
  const rclcpp::Time & stamp) const
{
  std::size_t count = 0;
  for (const auto & point : loaded_map_->pointLayer) {
    if (!isNearVisualizationCenter(point.x(), point.y())) {
      continue;
    }
    auto subtype = getSemanticType(point);
    if (subtype.empty()) {
      subtype = "point";
    }

    avg_msgs::msg::Marker marker;
    marker.header.frame_id = config_.map_frame_id;
    marker.header.stamp = stamp;
    const auto grouped_ns = groupedNamespace("point", subtype);
    marker.ns = grouped_ns;
    marker.id = id_counter++;
    marker.type = avg_msgs::msg::Marker::SPHERE;
    const double scale = subtype == "traffic_light" ? 0.6 : 0.4;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color = colorFromSubtype(subtype, makeColor(0.9f, 0.9f, 0.2f));
    marker.pose.position = makeMapPoint(point.x(), point.y(), point.z());
    marker.pose.orientation.w = 1.0;

    markers.markers.emplace_back(std::move(marker));
    ++count;
  }
  return count;
}

std::size_t Lanelet2MapNode::addLaneletDirections(
  avg_msgs::msg::MarkerArray & markers, int32_t & id_counter,
  const rclcpp::Time & stamp) const
{
  std::size_t count = 0;
  // HH_260114 Direction arrow color/alpha matched to Autoware tone.
  const auto color = makeColor(0.35f, 0.35f, 0.38f, 0.9f);
  for (const auto & lanelet : loaded_map_->laneletLayer) {
    if (!isLaneletNear(lanelet)) {
      continue;
    }
    const auto & centerline = lanelet.centerline();
    if (centerline.size() < 2) {
      continue;
    }

    // HH_260114 Draw arrows on short segments along the full centerline for accurate alignment.
    const std::size_t stride = std::max<std::size_t>(1, config_.dir_stride);  // HH_260114 Controlled by parameter.
    for (std::size_t i = 0; i + 1 < centerline.size(); i += stride) {
      if (!isNearVisualizationCenter(centerline[i].x(), centerline[i].y()) &&
        !isNearVisualizationCenter(centerline[i + 1].x(), centerline[i + 1].y()))
      {
        continue;
      }
      const double lane_width = laneWidthAt(lanelet, i);
      avg_msgs::msg::Point tail_left, tail_right, head_point;
      if (!computeFlatArrow(centerline, i, i + 1, lane_width, tail_left, tail_right, head_point)) {
        continue;
      }

      avg_msgs::msg::Marker marker;
      marker.header.frame_id = config_.map_frame_id;
      marker.header.stamp = stamp;
      marker.ns = "lanelet/direction";
      marker.scale.x = 1.0;  // HH_260114 TRIANGLE_LIST scale derived from points; keep 1.0.
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      marker.id = id_counter++;
      marker.type = avg_msgs::msg::Marker::TRIANGLE_LIST;
      marker.action = avg_msgs::msg::Marker::ADD;
      marker.color = color;

      marker.points.push_back(tail_left);
      marker.points.push_back(tail_right);
      marker.points.push_back(head_point);
      marker.points.push_back(tail_left);
      marker.points.push_back(head_point);
      marker.points.push_back(tail_right);

      markers.markers.emplace_back(std::move(marker));
      ++count;
    }
  }
  return count;
}

std::size_t Lanelet2MapNode::addLaneletIds(
  avg_msgs::msg::MarkerArray & markers, int32_t & id_counter,
  const rclcpp::Time & stamp) const
{
  std::size_t count = 0;
  for (const auto & lanelet : loaded_map_->laneletLayer) {
    if (!isLaneletNear(lanelet)) {
      continue;
    }
    const auto & centerline = lanelet.centerline();
    if (centerline.empty()) {
      continue;
    }
    avg_msgs::msg::Marker marker;
    marker.header.frame_id = config_.map_frame_id;
    marker.header.stamp = stamp;
    marker.ns = "lanelet/id";
    marker.id = id_counter++;
    marker.type = avg_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = avg_msgs::msg::Marker::ADD;
    // HH_260114 Make lanelet text smaller and softer.
    marker.scale.z = 1.0;
    marker.color = makeColor(0.55f, 0.55f, 0.60f, 0.45f);
    marker.text = std::to_string(lanelet.id());
    const size_t mid_index = centerline.size() / 2;
    const double label_base_z = align_z_to_ground_ ? map_ground_z_ : centerline[mid_index].z();
    avg_msgs::msg::Point p = makePoint(
      centerline[mid_index].x(),
      centerline[mid_index].y(),
      label_base_z + 1.0);
    marker.pose.position = p;
    marker.pose.orientation.w = 1.0;
    markers.markers.emplace_back(std::move(marker));
    ++count;
  }
  return count;
}

std::size_t Lanelet2MapNode::addSemanticMarkers(
  avg_msgs::msg::MarkerArray & markers, int32_t & id_counter,
  const rclcpp::Time & stamp) const
{
  std::size_t count = 0;
  struct SemanticStyle
  {
    std::string ns;
    avg_msgs::msg::ColorRGBA color;
    int marker_type;
    double sx;
    double sy;
    double sz;
    bool add_text;
  };
  // HH_251215: Autoware-like semantic style palette
  const std::unordered_map<std::string, SemanticStyle> semantic_styles = {
    {"traffic_light", {"semantic/traffic_light/body", makeColor(0.20f, 0.24f, 0.32f, 0.95f), avg_msgs::msg::Marker::CUBE, 0.35, 0.35, 2.3, true}},
    {"traffic_sign", {"semantic/traffic_sign", makeColor(0.45f, 0.70f, 0.78f, 0.92f), avg_msgs::msg::Marker::CUBE, 0.4, 2.4, 1.8, true}},
    {"speed_bump", {"semantic/speed_bump", makeColor(1.0f, 0.55f, 0.0f), avg_msgs::msg::Marker::CUBE, 1.6, 0.8, 0.25, false}}
  };

  for (const auto & line_string : loaded_map_->lineStringLayer) {
    if (!isLineStringNear(line_string)) {
      continue;
    }
    const auto tags = extractSemanticTags(line_string);
    auto it = semantic_styles.find(tags.subtype);
    if (it == semantic_styles.end() && !tags.type.empty()) {
      it = semantic_styles.find(tags.type);
    }
    if (it == semantic_styles.end()) {
      continue;
    }
    const auto & style = it->second;

    avg_msgs::msg::Marker marker;
    marker.header.frame_id = config_.map_frame_id;
    marker.header.stamp = stamp;
    marker.ns = style.ns;
    marker.id = id_counter++;
    marker.type = style.marker_type;
    marker.color = style.color;
    marker.pose.orientation.w = 1.0;

    const bool is_traffic_sign = tags.subtype == "traffic_sign" || tags.type == "traffic_sign";

    if (style.marker_type == avg_msgs::msg::Marker::LINE_STRIP) {
      marker.scale.x = style.sx;
      for (const auto & point : line_string) {
        marker.points.push_back(makeMapPoint(point.x(), point.y(), point.z()));
      }
    } else {
      marker.scale.x = style.sx;
      marker.scale.y = style.sy;
      marker.scale.z = style.sz;
      const auto centroid = computeCentroid(line_string);
      marker.pose.position = makeMapPoint(centroid.x, centroid.y, centroid.z);
      if (style.ns == "semantic/traffic_light/body") {
        marker.scale.x = 0.45;
        marker.scale.y = 0.45;
        marker.scale.z = 2.4;
        addTrafficLightBulbs(marker.pose.position, "semantic/traffic_light/bulb", markers, id_counter, stamp);
      } else if (is_traffic_sign) {
        marker.scale.x = 0.35;
        marker.scale.y = 2.8;
        marker.scale.z = 1.9;
        marker.color = makeColor(0.43f, 0.68f, 0.80f, 0.92f);
      }
    }

    markers.markers.emplace_back(marker);
    ++count;

    if (style.add_text) {
      avg_msgs::msg::Marker text_marker;
      text_marker.header = marker.header;
      text_marker.ns = "semantic/text";
      text_marker.id = id_counter++;
      text_marker.type = avg_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.scale.z = 0.9;
      text_marker.color = makeColor(1.0f, 1.0f, 1.0f);
      text_marker.pose.position = marker.type == avg_msgs::msg::Marker::LINE_STRIP
        ? marker.points.front()
        : marker.pose.position;
      text_marker.pose.position.z += 1.0;
      auto label = !tags.type.empty() ? tags.type : tags.subtype;
      text_marker.text = label;
      markers.markers.emplace_back(text_marker);
      ++count;
    }
  }

  for (const auto & point : loaded_map_->pointLayer) {
    if (!isNearVisualizationCenter(point.x(), point.y())) {
      continue;
    }
    const auto tags = extractSemanticTags(point);
    auto it = semantic_styles.find(tags.subtype);
    if (it == semantic_styles.end() && !tags.type.empty()) {
      it = semantic_styles.find(tags.type);
    }
    if (it == semantic_styles.end()) {
      continue;
    }
    const auto & style = it->second;
    avg_msgs::msg::Point centroid = makeMapPoint(point.x(), point.y(), point.z());
    const bool is_traffic_sign = tags.subtype == "traffic_sign" || tags.type == "traffic_sign";

    avg_msgs::msg::Marker marker;
    marker.header.frame_id = config_.map_frame_id;
    marker.header.stamp = stamp;
    marker.ns = style.ns;
    marker.id = id_counter++;
    marker.type = style.marker_type;
    marker.scale.x = style.sx;
    marker.scale.y = style.sy;
    marker.scale.z = style.sz;
    marker.color = style.color;
    marker.pose.position = centroid;
    marker.pose.orientation.w = 1.0;
    if (style.ns == "semantic/traffic_light/body") {
      marker.scale.x = 0.45;
      marker.scale.y = 0.45;
      marker.scale.z = 2.3;
      addTrafficLightBulbs(marker.pose.position, "semantic/traffic_light/bulb", markers, id_counter, stamp);
    } else if (is_traffic_sign) {
      marker.scale.x = 0.3;
      marker.scale.y = 2.4;
      marker.scale.z = 1.8;
      marker.color = makeColor(0.43f, 0.68f, 0.80f, 0.92f);
    }

    markers.markers.emplace_back(marker);
    ++count;

    if (style.add_text) {
      avg_msgs::msg::Marker text_marker;
      text_marker.header = marker.header;
      text_marker.ns = "semantic/text";
      text_marker.id = id_counter++;
      text_marker.type = avg_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.scale.z = 0.8;
      text_marker.color = makeColor(1.0f, 1.0f, 1.0f);
      text_marker.pose.position = marker.pose.position;
      text_marker.pose.position.z += 0.9;
      text_marker.text = !tags.type.empty() ? tags.type : tags.subtype;
      markers.markers.emplace_back(text_marker);
      ++count;
    }
  }
  return count;
}

// Initialization helper LineMarker: initializes message defaults before frame-specific fields are filled.
avg_msgs::msg::Marker Lanelet2MapNode::initLineMarker(
  const std::string & ns, int32_t id, const std::string & frame_id,
  const avg_msgs::msg::ColorRGBA & color, double width, const rclcpp::Time & stamp)
{
  avg_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = avg_msgs::msg::Marker::LINE_STRIP;
  marker.action = avg_msgs::msg::Marker::ADD;
  marker.scale.x = width;
  marker.color = color;
  return marker;
}

avg_msgs::msg::ColorRGBA Lanelet2MapNode::makeColor(float r, float g, float b, float a)
{
  avg_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

avg_msgs::msg::Point Lanelet2MapNode::makePoint(double x, double y, double z) const
{
  avg_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// makeMapPoint: Constructs a map-geometry point and optionally flattens OSM altitude for 2D RViz/planning alignment.
avg_msgs::msg::Point Lanelet2MapNode::makeMapPoint(double x, double y, double z) const
{
  return makePoint(x, y, align_z_to_ground_ ? map_ground_z_ : z);
}

// HH_260114 Semantic marker centroid for placement.
avg_msgs::msg::Point Lanelet2MapNode::computeCentroid(
  const lanelet::ConstLineString3d & line_string)
{
  avg_msgs::msg::Point centroid;
  if (line_string.empty()) {
    return centroid;
  }
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  for (const auto & point : line_string) {
    sum_x += point.x();
    sum_y += point.y();
    sum_z += point.z();
  }
  const double denom = static_cast<double>(line_string.size());
  centroid.x = sum_x / denom;
  centroid.y = sum_y / denom;
  centroid.z = sum_z / denom;
  return centroid;
}

bool Lanelet2MapNode::isNearVisualizationCenter(double x, double y) const
{
  if (!active_visualization_filter_) {
    return true;
  }
  const double dx = x - active_visualization_filter_->x;
  const double dy = y - active_visualization_filter_->y;
  return dx * dx + dy * dy <= active_visualization_filter_->radius_sq;
}

bool Lanelet2MapNode::isLineStringNear(const lanelet::ConstLineString3d & line_string) const
{
  if (!active_visualization_filter_) {
    return true;
  }
  for (const auto & point : line_string) {
    if (isNearVisualizationCenter(point.x(), point.y())) {
      return true;
    }
  }
  return false;
}

bool Lanelet2MapNode::isLaneletNear(const lanelet::ConstLanelet & lanelet) const
{
  if (!active_visualization_filter_) {
    return true;
  }
  // HH_260625: Avoid computing lazy centerlines for every far-away lanelet during
  // the first local RViz publish. Bounds are already loaded and cover the same
  // local neighborhood for an 80m startup window.
  return isLineStringNear(lanelet.leftBound()) ||
    isLineStringNear(lanelet.rightBound());
}

bool Lanelet2MapNode::isAreaNear(const lanelet::ConstArea & area) const
{
  if (!active_visualization_filter_) {
    return true;
  }
  for (const auto & bound : area.outerBound()) {
    for (const auto & point : bound) {
      if (isNearVisualizationCenter(point.x(), point.y())) {
        return true;
      }
    }
  }
  return false;
}

void Lanelet2MapNode::addTrafficLightBulbs(  // HH_260114 Autoware-style tri-color bulbs.
  const avg_msgs::msg::Point & base_center,
  const std::string & bulb_namespace,
  avg_msgs::msg::MarkerArray & markers,
  int32_t & id_counter,
  const rclcpp::Time & stamp) const
{
  // HH_251215: emulate Autoware bulb layout (red/amber/green row)
  struct BulbStyle
  {
    avg_msgs::msg::Point offset;
    avg_msgs::msg::ColorRGBA color;
  };
  const double spacing = 0.45;
  const double radius = 0.24;
  const std::array<BulbStyle, 3> bulbs = {
    BulbStyle{makePoint(-spacing, 0.0, 0.8), makeColor(0.9f, 0.25f, 0.25f, 0.95f)},
    BulbStyle{makePoint(0.0, 0.0, 0.8), makeColor(0.95f, 0.65f, 0.2f, 0.65f)},
    BulbStyle{makePoint(spacing, 0.0, 0.8), makeColor(0.2f, 0.95f, 0.35f, 0.65f)}
  };

  for (size_t idx = 0; idx < bulbs.size(); ++idx) {
    avg_msgs::msg::Marker bulb_marker;
    bulb_marker.header.frame_id = config_.map_frame_id;
    bulb_marker.header.stamp = stamp;
    bulb_marker.ns = bulb_namespace;
    bulb_marker.id = id_counter++;
    bulb_marker.type = avg_msgs::msg::Marker::SPHERE;
    bulb_marker.scale.x = radius;
    bulb_marker.scale.y = radius;
    bulb_marker.scale.z = radius;
    bulb_marker.color = bulbs[idx].color;
    bulb_marker.pose.position = makePoint(
      base_center.x + bulbs[idx].offset.x,
      base_center.y + bulbs[idx].offset.y,
      base_center.z + bulbs[idx].offset.z);
    bulb_marker.pose.orientation.w = 1.0;
    markers.markers.emplace_back(bulb_marker);
  }
}

bool Lanelet2MapNode::computeFlatArrow(
  const lanelet::ConstLineString3d & centerline, const std::size_t tail_idx, const std::size_t head_idx,
  double lane_width,
  avg_msgs::msg::Point & tail_left,
  avg_msgs::msg::Point & tail_right,
  avg_msgs::msg::Point & head_point) const
{
  if (centerline.size() < 2 || tail_idx >= centerline.size() || head_idx >= centerline.size()) {
    return false;
  }
  const auto & tail = centerline[tail_idx];
  const auto & head = centerline[head_idx];

  const double dx = head.x() - tail.x();
  const double dy = head.y() - tail.y();
  const double length = std::hypot(dx, dy);
  if (length < 1e-3) {
    return false;
  }

  const double dir_x = dx / length;
  const double dir_y = dy / length;
  const double perp_x = -dir_y;
  const double perp_y = dir_x;

  // HH_260114 Scale by lane width for centerline-relative size.
  const double clamped_width = std::clamp(lane_width, 2.0, 6.0);
  const double half_width = std::max(0.08, clamped_width * config_.dir_width_scale);
  const double body_length = std::max(0.35, clamped_width * config_.dir_body_scale);
  const double head_length = std::max(0.20, clamped_width * config_.dir_head_scale);
  // HH_260623 - Direction arrows follow the same flattened map marker plane as lane boundaries.
  const double base_z = align_z_to_ground_ ? map_ground_z_ : (tail.z() + head.z()) * 0.5;

  // Build arrow around the midpoint of the segment to avoid forward shift
  avg_msgs::msg::Point mid = makePoint(
    (tail.x() + head.x()) * 0.5,
    (tail.y() + head.y()) * 0.5,
    base_z);

  tail_left = makePoint(
    mid.x - dir_x * (body_length * 0.5) - perp_x * half_width,
    mid.y - dir_y * (body_length * 0.5) - perp_y * half_width,
    base_z);
  tail_right = makePoint(
    mid.x - dir_x * (body_length * 0.5) + perp_x * half_width,
    mid.y - dir_y * (body_length * 0.5) + perp_y * half_width,
    base_z);

  avg_msgs::msg::Point center_head = makePoint(
    mid.x + dir_x * (body_length * 0.5),
    mid.y + dir_y * (body_length * 0.5),
    base_z);

  head_point = makePoint(
    center_head.x + dir_x * head_length,
    center_head.y + dir_y * head_length,
    base_z);

  return true;
}

// Estimates local lane width from left/right bound point pairs.
double Lanelet2MapNode::laneWidthAt(const lanelet::ConstLanelet & lanelet, std::size_t idx) const
{
  // HH_260103 lane width estimation using left/right bounds at the same index
  const auto & left = lanelet.leftBound();
  const auto & right = lanelet.rightBound();
  if (left.empty() || right.empty()) {
    return 3.0;  // fallback width
  }

  const std::size_t li = std::min(idx, left.size() - 1);
  const std::size_t ri = std::min(idx, right.size() - 1);

  const auto & lp = left[li];
  const auto & rp = right[ri];
  const double dx = lp.x() - rp.x();
  const double dy = lp.y() - rp.y();
  const double width = std::hypot(dx, dy);
  return width > 0.1 ? width : 3.0;
}

avg_msgs::msg::ColorRGBA Lanelet2MapNode::colorFromSubtype(
  const std::string & subtype, const avg_msgs::msg::ColorRGBA & fallback) const
{
  if (subtype == "stop_line") {
    return makeColor(1.0f, 1.0f, 0.0f);
  }
  if (subtype == "traffic_light" || subtype == "traffic_signal") {
    return makeColor(0.20f, 0.24f, 0.32f, 0.95f);  // body
  }
  if (subtype == "traffic_sign") {
    return makeColor(0.43f, 0.79f, 0.87f, 0.92f);
  }
  if (subtype == "crosswalk") {
    return makeColor(0.9f, 0.9f, 0.0f, 0.9f);
  }
  if (subtype == "road_border" || subtype == "curbstone") {
    return makeColor(1.0f, 1.0f, 1.0f, 0.5f);
  }
  if (subtype == "virtual") {
    return makeColor(0.6f, 0.6f, 0.6f, 0.4f);
  }
  if (subtype == "speed_bump") {
    return makeColor(1.0f, 0.55f, 0.0f, 0.95f);
  }
  if (subtype == "pole") {
    return makeColor(0.7f, 0.4f, 0.1f);
  }
  if (subtype == "pedestrian_marking") {
    return makeColor(0.4f, 0.9f, 0.7f);
  }
  return fallback;
}

double Lanelet2MapNode::lineWidthFromSubtype(const std::string & subtype) const
{
  // HH_260114 Autoware-style line thickness.
  if (subtype == "center_lane" || subtype == "center_line") {
    return 0.4;
  }
  if (subtype == "right_lane_bound" || subtype == "left_lane_bound") {
    return 0.15;
  }
  if (subtype == "stop_line") {
    return 0.6;
  }
  if (subtype == "crosswalk") {
    return 0.3;
  }
  return 0.1;
}

std::string Lanelet2MapNode::sanitizeNamespace(
  const std::string & prefix, const std::string & subtype)
{
  std::string sanitized = subtype;
  if (sanitized.empty()) {
    sanitized = "unknown";
  }
  std::replace(sanitized.begin(), sanitized.end(), ' ', '_');
  return prefix + "_" + sanitized;
}

std::string Lanelet2MapNode::groupedNamespace(
  const std::string & group, const std::string & subtype)
{
  std::string sanitized = subtype;
  if (sanitized.empty()) {
    sanitized = "unknown";
  }
  std::replace(sanitized.begin(), sanitized.end(), ' ', '_');
  return group + "/" + sanitized;
}

void Lanelet2MapNode::publishStaticTF()
{
  avg_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.frame_id = config_.world_frame_id;
  tf_msg.child_frame_id = config_.map_frame_id;
  tf_msg.header.stamp = now();
  tf_msg.transform.translation.x = 0.0;
  tf_msg.transform.translation.y = 0.0;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation.x = 0.0;
  tf_msg.transform.rotation.y = 0.0;
  tf_msg.transform.rotation.z = 0.0;
  tf_msg.transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(tf_msg);
}

void Lanelet2MapNode::publishAvgMapMessage(
  const avg_msgs::msg::MarkerArray & markers,
  const rclcpp::Time & stamp)
{
  if (!publish_map_status_ || !avg_map_pub_) {
    return;
  }

  avg_msgs::msg::AvgMapMsgs msg;
  msg.stamp = stamp;
  msg.state.stamp = stamp;
  msg.state.module_name = "map";
  msg.state.level = avg_msgs::msg::ModuleState::OK;
  msg.state.message = "lanelet markers published";
  msg.lanelet_markers = markers;
  avg_map_pub_->publish(msg);
}

}  // namespace map
}  // namespace camrod

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod::map::Lanelet2MapNode>());
  rclcpp::shutdown();
  return 0;
}
