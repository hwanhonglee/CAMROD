// Multi-stream cost-field marker publisher.
// Replaces multiple cost_field_marker_node instances when several grids need
// the same conversion pipeline (OccupancyGrid -> MarkerArray).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <avg_msgs/msg/avg_map_msgs.hpp>
#include <avg_msgs/msg/color_rgba.hpp>
#include <avg_msgs/msg/header.hpp>
#include <avg_msgs/msg/marker.hpp>
#include <avg_msgs/msg/marker_array.hpp>
#include <avg_msgs/msg/module_state.hpp>
#include <avg_msgs/msg/occupancy_grid.hpp>
#include <avg_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>

namespace camrod_map
{

class MultiCostFieldMarkerNode : public rclcpp::Node
{
public:
  MultiCostFieldMarkerNode()
  : Node("multi_cost_field_marker")
  {
    publish_map_status_ = declare_parameter<bool>("publish_map_status", false);
    map_status_topic_ = declare_parameter<std::string>("map_status_topic", "/map/status");
    if (publish_map_status_) {
      avg_map_pub_ = create_publisher<avg_msgs::msg::AvgMapMsgs>(
        map_status_topic_, rclcpp::QoS(10));
    }

    const auto grid_topics =
      declare_parameter<std::vector<std::string>>("grid_topics", std::vector<std::string>{});
    const auto marker_topics =
      declare_parameter<std::vector<std::string>>("marker_topics", std::vector<std::string>{});
    if (grid_topics.empty() || marker_topics.empty() || grid_topics.size() != marker_topics.size()) {
      RCLCPP_FATAL(
        get_logger(),
        "grid_topics/marker_topics must be non-empty and same size. grid=%zu marker=%zu",
        grid_topics.size(), marker_topics.size());
      rclcpp::shutdown();
      return;
    }

    const double marker_scale_default = declare_parameter<double>("marker_scale_default", 0.1);
    const int min_value_default = declare_parameter<int>("min_value_default", 0);
    const int max_value_default = declare_parameter<int>("max_value_default", 100);
    const double alpha_default = declare_parameter<double>("alpha_default", 0.35);
    const double z_offset_default = declare_parameter<double>("z_offset_default", 0.05);
    const double cell_scale_ratio_default = declare_parameter<double>("cell_scale_ratio_default", 1.0);
    const std::string palette_default = declare_parameter<std::string>("palette_default", "safety");
    const bool show_unknown_default = declare_parameter<bool>("show_unknown_default", false);
    const bool clear_on_empty_default = declare_parameter<bool>("clear_on_empty_grid_default", true);
    const bool grid_qos_transient_local_default =
      declare_parameter<bool>("grid_qos_transient_local_default", true);
    const double stale_timeout_default =
      declare_parameter<double>("stale_timeout_s_default", 0.0);
    const double min_publish_period_default =
      declare_parameter<double>("min_publish_period_s_default", 0.0);
    const double republish_period_default =
      declare_parameter<double>("republish_period_s_default", 0.0);
    const int sample_stride_default = declare_parameter<int>("sample_stride_default", 1);

    const auto marker_scales =
      declare_parameter<std::vector<double>>("marker_scales", std::vector<double>{});
    const auto min_values =
      declare_parameter<std::vector<int64_t>>("min_values", std::vector<int64_t>{});
    const auto max_values =
      declare_parameter<std::vector<int64_t>>("max_values", std::vector<int64_t>{});
    const auto alphas =
      declare_parameter<std::vector<double>>("alphas", std::vector<double>{});
    const auto z_offsets =
      declare_parameter<std::vector<double>>("z_offsets", std::vector<double>{});
    const auto cell_scale_ratios =
      declare_parameter<std::vector<double>>("cell_scale_ratios", std::vector<double>{});
    const auto palettes =
      declare_parameter<std::vector<std::string>>("palettes", std::vector<std::string>{});
    const auto show_unknowns =
      declare_parameter<std::vector<bool>>("show_unknowns", std::vector<bool>{});
    const auto clear_on_empty_grids =
      declare_parameter<std::vector<bool>>("clear_on_empty_grids", std::vector<bool>{});
    const auto grid_qos_transient_locals =
      declare_parameter<std::vector<bool>>("grid_qos_transient_locals", std::vector<bool>{});
    const auto stale_timeouts_s =
      declare_parameter<std::vector<double>>("stale_timeouts_s", std::vector<double>{});
    const auto min_publish_periods_s =
      declare_parameter<std::vector<double>>("min_publish_periods_s", std::vector<double>{});
    const auto republish_periods_s =
      declare_parameter<std::vector<double>>("republish_periods_s", std::vector<double>{});
    const auto sample_strides =
      declare_parameter<std::vector<int64_t>>("sample_strides", std::vector<int64_t>{});

    streams_.reserve(grid_topics.size());
    for (size_t i = 0; i < grid_topics.size(); ++i) {
      StreamRuntime stream;
      stream.grid_topic = grid_topics[i];
      stream.marker_topic = marker_topics[i];
      stream.marker_scale = pickOrDefault(marker_scales, i, marker_scale_default);
      stream.min_value = static_cast<int>(pickOrDefault(min_values, i, static_cast<int64_t>(min_value_default)));
      stream.max_value = static_cast<int>(pickOrDefault(max_values, i, static_cast<int64_t>(max_value_default)));
      stream.alpha = pickOrDefault(alphas, i, alpha_default);
      stream.z_offset = pickOrDefault(z_offsets, i, z_offset_default);
      stream.cell_scale_ratio =
        std::clamp(pickOrDefault(cell_scale_ratios, i, cell_scale_ratio_default), 0.1, 1.0);
      stream.palette = pickOrDefault(palettes, i, palette_default);
      stream.show_unknown = pickOrDefault(show_unknowns, i, show_unknown_default);
      stream.clear_on_empty_grid =
        pickOrDefault(clear_on_empty_grids, i, clear_on_empty_default);
      stream.grid_qos_transient_local =
        pickOrDefault(grid_qos_transient_locals, i, grid_qos_transient_local_default);
      stream.stale_timeout_s = std::max(0.0, pickOrDefault(stale_timeouts_s, i, stale_timeout_default));
      stream.min_publish_period_s =
        std::max(0.0, pickOrDefault(min_publish_periods_s, i, min_publish_period_default));
      stream.republish_period_s =
        std::max(0.0, pickOrDefault(republish_periods_s, i, republish_period_default));
      stream.sample_stride = std::max(
        1, static_cast<int>(pickOrDefault(sample_strides, i, static_cast<int64_t>(sample_stride_default))));

      auto grid_qos = rclcpp::QoS(1).reliable();
      if (stream.grid_qos_transient_local) {
        grid_qos.transient_local();
      }
      stream.sub = create_subscription<avg_msgs::msg::OccupancyGrid>(
        stream.grid_topic,
        grid_qos,
        [this, i](avg_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
          onGrid(i, std::move(msg));
        });
      stream.pub = create_publisher<avg_msgs::msg::MarkerArray>(
        stream.marker_topic, rclcpp::QoS(1).transient_local().reliable());

      streams_.push_back(std::move(stream));
    }

    for (size_t i = 0; i < streams_.size(); ++i) {
      updateRepublishTimer(i);
    }
  }

private:
  template<typename T>
  static T pickOrDefault(const std::vector<T> & values, size_t index, const T & fallback)
  {
    if (index < values.size()) {
      return values[index];
    }
    return fallback;
  }

  // Stream runtime state.
  struct StreamRuntime
  {
    std::string grid_topic;
    std::string marker_topic;
    double marker_scale{0.1};
    int min_value{0};
    int max_value{100};
    double alpha{0.35};
    double z_offset{0.05};
    double cell_scale_ratio{1.0};
    std::string palette{"safety"};
    bool show_unknown{false};
    bool clear_on_empty_grid{true};
    bool grid_qos_transient_local{true};
    // HH_260617: Keep internal duration fields aligned with canonical `_s` naming.
    double stale_timeout_s{0.0};
    double min_publish_period_s{0.0};
    double republish_period_s{0.0};
    int sample_stride{1};

    rclcpp::Subscription<avg_msgs::msg::OccupancyGrid>::SharedPtr sub;
    rclcpp::Publisher<avg_msgs::msg::MarkerArray>::SharedPtr pub;
    avg_msgs::msg::MarkerArray last_markers;
    avg_msgs::msg::OccupancyGrid::ConstSharedPtr latest_grid;
    bool pending_grid_update{false};
    rclcpp::Time last_grid_rx{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_publish_time{0, 0, RCL_ROS_TIME};
    rclcpp::TimerBase::SharedPtr republish_timer;
  };

  bool canPublishNow(const StreamRuntime & stream, const rclcpp::Time & now_t) const
  {
    if (stream.min_publish_period_s <= 0.0 || stream.last_publish_time.nanoseconds() <= 0) {
      return true;
    }
    const double dt = (now_t - stream.last_publish_time).seconds();
    return dt >= stream.min_publish_period_s;
  }

  avg_msgs::msg::Marker initMarker(
    const avg_msgs::msg::Header & header,
    const StreamRuntime & stream) const
  {
    avg_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "inflation_cost_grid";
    marker.id = 0;
    marker.type = avg_msgs::msg::Marker::CUBE_LIST;
    marker.action = avg_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = stream.marker_scale;
    marker.scale.y = stream.marker_scale;
    marker.scale.z = stream.marker_scale;
    marker.lifetime = rclcpp::Duration(0, 0);
    return marker;
  }

  avg_msgs::msg::ColorRGBA colorFromValue(int8_t value, const StreamRuntime & stream) const
  {
    avg_msgs::msg::ColorRGBA color;
    color.a = static_cast<float>(stream.alpha);
    if (value < 0) {
      if (stream.palette == "pastel") {
        color.r = 0.85f;
        color.g = 0.85f;
        color.b = 0.88f;
      } else {
        color.r = 0.2f;
        color.g = 0.2f;
        color.b = 0.2f;
      }
      return color;
    }

    const float denom = static_cast<float>(std::max(1, stream.max_value - stream.min_value));
    const float norm = std::clamp(static_cast<float>(value - stream.min_value) / denom, 0.0f, 1.0f);
    auto lerp_gradient = [&](float low_r, float low_g, float low_b, float high_r, float high_g, float high_b) {
      color.r = high_r * norm + low_r * (1.0f - norm);
      color.g = high_g * norm + low_g * (1.0f - norm);
      color.b = high_b * norm + low_b * (1.0f - norm);
      return color;
    };

    if (stream.palette == "pastel_blue_red") {
      return lerp_gradient(0.72f, 0.86f, 0.95f, 0.95f, 0.77f, 0.77f);
    }
    if (stream.palette == "pastel_green_red") {
      return lerp_gradient(0.72f, 0.92f, 0.78f, 0.95f, 0.76f, 0.76f);
    }
    if (stream.palette == "pastel_orange_red") {
      return lerp_gradient(0.97f, 0.86f, 0.70f, 0.95f, 0.73f, 0.73f);
    }
    if (stream.palette == "pastel_purple_red") {
      return lerp_gradient(0.82f, 0.76f, 0.95f, 0.96f, 0.74f, 0.76f);
    }
    if (stream.palette == "pastel_cyan_red") {
      return lerp_gradient(0.71f, 0.92f, 0.93f, 0.95f, 0.74f, 0.76f);
    }
    if (stream.palette == "pastel") {
      return lerp_gradient(0.72f, 0.86f, 0.95f, 0.95f, 0.80f, 0.86f);
    }

    // safety palette
    const float low_r = 0.20f;
    const float low_g = 0.70f;
    const float low_b = 0.95f;
    const float mid_r = 0.20f;
    const float mid_g = 0.85f;
    const float mid_b = 0.35f;
    const float high_r = 0.92f;
    const float high_g = 0.16f;
    const float high_b = 0.14f;
    if (norm < 0.5f) {
      const float t = norm / 0.5f;
      color.r = mid_r * t + low_r * (1.0f - t);
      color.g = mid_g * t + low_g * (1.0f - t);
      color.b = mid_b * t + low_b * (1.0f - t);
    } else {
      const float t = (norm - 0.5f) / 0.5f;
      color.r = high_r * t + mid_r * (1.0f - t);
      color.g = high_g * t + mid_g * (1.0f - t);
      color.b = high_b * t + mid_b * (1.0f - t);
    }
    return color;
  }

  void publishAvgMapMessage(
    const avg_msgs::msg::MarkerArray & markers,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & message)
  {
    if (!publish_map_status_ || !avg_map_pub_) {
      return;
    }
    avg_msgs::msg::AvgMapMsgs msg;
    msg.stamp = stamp;
    msg.state.stamp = stamp;
    msg.state.module_name = "map";
    msg.state.level = avg_msgs::msg::ModuleState::OK;
    msg.state.message = message;
    msg.inflation_markers = markers;
    avg_map_pub_->publish(msg);
  }

  void publishDeleteAll(StreamRuntime & stream, const avg_msgs::msg::Header & header)
  {
    avg_msgs::msg::MarkerArray arr;
    avg_msgs::msg::Marker del;
    del.header = header;
    del.ns = "inflation_cost_grid";
    del.id = 0;
    del.action = avg_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);
    stream.pub->publish(arr);
    publishAvgMapMessage(arr, header.stamp, "multi cost markers cleared: " + stream.marker_topic);
    stream.last_markers.markers.clear();
  }

  void publishFromGrid(StreamRuntime & stream, const avg_msgs::msg::OccupancyGrid & grid_msg)
  {
    avg_msgs::msg::MarkerArray arr;
    auto marker = initMarker(grid_msg.header, stream);
    const double stride = static_cast<double>(stream.sample_stride);
    marker.scale.x = grid_msg.info.resolution * stream.cell_scale_ratio * stride;
    marker.scale.y = grid_msg.info.resolution * stream.cell_scale_ratio * stride;
    marker.scale.z = grid_msg.info.resolution * 0.5;

    const auto & info = grid_msg.info;
    const size_t width = info.width;
    const size_t height = info.height;
    const double origin_x = info.origin.position.x;
    const double origin_y = info.origin.position.y;
    const double res = info.resolution;
    const auto & q = info.origin.orientation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    const std::size_t sampled_w =
      (width + static_cast<std::size_t>(stream.sample_stride) - 1U) /
      static_cast<std::size_t>(stream.sample_stride);
    const std::size_t sampled_h =
      (height + static_cast<std::size_t>(stream.sample_stride) - 1U) /
      static_cast<std::size_t>(stream.sample_stride);
    const std::size_t sampled_cap = sampled_w * sampled_h;
    marker.points.reserve(sampled_cap);
    marker.colors.reserve(sampled_cap);

    for (size_t y = 0; y < height; y += static_cast<size_t>(stream.sample_stride)) {
      for (size_t x = 0; x < width; x += static_cast<size_t>(stream.sample_stride)) {
        const size_t idx = y * width + x;
        const int8_t value = grid_msg.data[idx];
        if (value < 0 && !stream.show_unknown) {
          continue;
        }
        avg_msgs::msg::Point point;
        const double lx = (static_cast<double>(x) + 0.5 * stride) * res;
        const double ly = (static_cast<double>(y) + 0.5 * stride) * res;
        point.x = origin_x + cy * lx - sy * ly;
        point.y = origin_y + sy * lx + cy * ly;
        point.z = info.origin.position.z + stream.z_offset;
        marker.points.push_back(point);
        marker.colors.push_back(colorFromValue(value, stream));
      }
    }

    if (marker.points.empty()) {
      if (stream.clear_on_empty_grid) {
        publishDeleteAll(stream, grid_msg.header);
      } else {
        arr.markers.push_back(marker);
        stream.last_markers = arr;
        stream.pub->publish(arr);
        publishAvgMapMessage(arr, grid_msg.header.stamp, "multi cost markers empty frame");
      }
      stream.last_publish_time = now();
      stream.pending_grid_update = false;
      return;
    }

    arr.markers.push_back(marker);
    stream.last_markers = arr;
    stream.pub->publish(arr);
    publishAvgMapMessage(arr, grid_msg.header.stamp, "multi cost markers published");
    stream.last_publish_time = now();
    stream.pending_grid_update = false;
  }

  void onGrid(size_t index, avg_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
    if (!msg || index >= streams_.size()) {
      return;
    }
    auto & stream = streams_[index];
    stream.latest_grid = msg;
    stream.pending_grid_update = true;
    stream.last_grid_rx = now();

    if (msg->data.empty()) {
      if (stream.clear_on_empty_grid) {
        publishDeleteAll(stream, msg->header);
      }
      stream.latest_grid.reset();
      stream.pending_grid_update = false;
      return;
    }

    if (!canPublishNow(stream, now())) {
      return;
    }
    publishFromGrid(stream, *msg);
  }

  void onRepublishTimer(size_t index)
  {
    if (index >= streams_.size()) {
      return;
    }
    auto & stream = streams_[index];

    if (stream.stale_timeout_s > 0.0 && stream.last_grid_rx.nanoseconds() > 0) {
      const double dt = (now() - stream.last_grid_rx).seconds();
      if (dt > stream.stale_timeout_s && !stream.last_markers.markers.empty()) {
        publishDeleteAll(stream, stream.last_markers.markers.front().header);
        stream.latest_grid.reset();
        stream.pending_grid_update = false;
        return;
      }
    }

    if (
      stream.pending_grid_update &&
      stream.latest_grid &&
      !stream.latest_grid->data.empty() &&
      canPublishNow(stream, now()))
    {
      publishFromGrid(stream, *stream.latest_grid);
      return;
    }

    if (!stream.last_markers.markers.empty()) {
      for (auto & marker : stream.last_markers.markers) {
        marker.header.stamp = now();
      }
      stream.pub->publish(stream.last_markers);
      stream.last_publish_time = now();
      return;
    }

    if (stream.latest_grid && !stream.latest_grid->data.empty() && canPublishNow(stream, now())) {
      publishFromGrid(stream, *stream.latest_grid);
    }
  }

  void updateRepublishTimer(size_t index)
  {
    if (index >= streams_.size()) {
      return;
    }
    auto & stream = streams_[index];
    stream.republish_timer.reset();
    if (stream.republish_period_s <= 0.0) {
      return;
    }
    stream.republish_timer = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(stream.republish_period_s)),
      [this, index]() {
        onRepublishTimer(index);
      });
  }

  bool publish_map_status_{false};
  std::string map_status_topic_{"/map/status"};
  rclcpp::Publisher<avg_msgs::msg::AvgMapMsgs>::SharedPtr avg_map_pub_;
  std::vector<StreamRuntime> streams_;
};

}  // namespace camrod_map

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod_map::MultiCostFieldMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
