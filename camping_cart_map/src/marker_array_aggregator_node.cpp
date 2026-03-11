// HH_260304-00:00 // Merge contributor MarkerArray topics into one combined
// debug marker topic. This is visualization-only and is distinct from the
// actual Nav2 master costmap marker topic.

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace camping_cart_map
{

class MarkerArrayAggregatorNode : public rclcpp::Node
{
public:
  MarkerArrayAggregatorNode()
  : Node("marker_array_aggregator")
  {
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/map/cost_grid/contributor_markers");
    input_topics_ = declare_parameter<std::vector<std::string>>(
      "input_topics",
      std::vector<std::string>{
        "/map/cost_grid/lanelet_markers",
        "/map/cost_grid/lidar_markers",
        "/map/cost_grid/radar_markers",
        "/planning/cost_grid/global_path_markers",
        "/planning/cost_grid/local_path_markers"});
    republish_period_sec_ = declare_parameter<double>("republish_period_sec", 0.0);
    min_publish_period_sec_ = declare_parameter<double>("min_publish_period_sec", 0.05);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.0);
    stale_timeout_topics_ = declare_parameter<std::vector<std::string>>(
      "stale_timeout_topics", std::vector<std::string>{});
    timer_mode_ = republish_period_sec_ > 0.0;

    auto marker_qos = rclcpp::QoS(1).transient_local().reliable();
    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(output_topic_, marker_qos);

    sources_.reserve(input_topics_.size());
    prev_counts_.assign(input_topics_.size(), 0);
    prev_namespaces_.assign(input_topics_.size(), {});
    const std::unordered_set<std::string> stale_topic_set(
      stale_timeout_topics_.begin(), stale_timeout_topics_.end());
    for (size_t i = 0; i < input_topics_.size(); ++i) {
      Source source;
      source.topic = input_topics_[i];
      source.watch_stale = stale_topic_set.find(source.topic) != stale_topic_set.end();
      source.sub = create_subscription<visualization_msgs::msg::MarkerArray>(
        source.topic,
        rclcpp::QoS(1).transient_local().reliable(),
        [this, i](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
          onMarkers(i, std::move(msg));
        });
      sources_.push_back(std::move(source));
    }

    if (republish_period_sec_ > 0.0) {
      republish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(republish_period_sec_)),
        std::bind(&MarkerArrayAggregatorNode::publishCombined, this));
    }
  }

private:
  struct Source
  {
    std::string topic;
    visualization_msgs::msg::MarkerArray latest;
    rclcpp::Time last_rx;
    bool watch_stale{false};
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub;
  };

  static bool containsDeleteAll(const visualization_msgs::msg::MarkerArray & msg)
  {
    return std::any_of(
      msg.markers.begin(), msg.markers.end(),
      [](const visualization_msgs::msg::Marker & marker) {
        return marker.action == visualization_msgs::msg::Marker::DELETEALL;
      });
  }

  static int32_t markerId(size_t source_index, size_t marker_index)
  {
    return static_cast<int32_t>(source_index * 10000 + marker_index);
  }

  visualization_msgs::msg::Marker makeDeleteMarker(
    const std::string & frame_id, const std::string & ns,
    size_t source_index, size_t marker_index) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id.empty() ? "map" : frame_id;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = markerId(source_index, marker_index);
    marker.action = visualization_msgs::msg::Marker::DELETE;
    return marker;
  }

  void onMarkers(size_t source_index, visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)
  {
    auto & source = sources_.at(source_index);
    source.last_rx = now();

    if (!msg || msg->markers.empty() || containsDeleteAll(*msg)) {
      source.latest.markers.clear();
      dirty_ = true;
      if (!timer_mode_) {
        publishCombined();
      }
      return;
    }

    source.latest = *msg;
    dirty_ = true;
    if (!timer_mode_) {
      publishCombined();
    }
  }

  void publishCombined()
  {
    if (expireStaleSources()) {
      dirty_ = true;
    }
    if (!dirty_) {
      return;
    }
    if (min_publish_period_sec_ > 0.0 && last_publish_time_.nanoseconds() > 0) {
      const double dt = (now() - last_publish_time_).seconds();
      if (dt < min_publish_period_sec_) {
        return;
      }
    }

    visualization_msgs::msg::MarkerArray out;

    for (size_t source_index = 0; source_index < sources_.size(); ++source_index) {
      auto & source = sources_[source_index];
      size_t marker_index = 0;
      std::vector<std::string> current_namespaces;
      current_namespaces.reserve(source.latest.markers.size());

      for (const auto & marker : source.latest.markers) {
        if (marker.action == visualization_msgs::msg::Marker::DELETEALL) {
          continue;
        }

        auto rewritten = marker;
        rewritten.id = markerId(source_index, marker_index);
        out.markers.push_back(std::move(rewritten));
        current_namespaces.push_back(marker.ns);
        ++marker_index;
      }

      const size_t prev_count = prev_counts_[source_index];
      auto & prev_ns = prev_namespaces_[source_index];

      // HH_260305-00:00 Delete stale ids when source marker count shrinks.
      if (prev_count > marker_index) {
        const std::string frame_id = !source.latest.markers.empty() ?
          source.latest.markers.front().header.frame_id : "map";
        for (size_t stale = marker_index; stale < prev_count; ++stale) {
          const std::string ns = (stale < prev_ns.size()) ? prev_ns[stale] : std::string{};
          out.markers.push_back(makeDeleteMarker(frame_id, ns, source_index, stale));
        }
      }

      // HH_260305-00:00 Delete previous id when namespace changed at same slot.
      const size_t overlap = std::min(prev_count, marker_index);
      const std::string frame_id = !source.latest.markers.empty() ?
        source.latest.markers.front().header.frame_id : "map";
      for (size_t idx = 0; idx < overlap; ++idx) {
        if (idx < prev_ns.size() && prev_ns[idx] != current_namespaces[idx]) {
          out.markers.push_back(makeDeleteMarker(frame_id, prev_ns[idx], source_index, idx));
        }
      }

      prev_counts_[source_index] = marker_index;
      prev_namespaces_[source_index] = std::move(current_namespaces);
    }

    if (out.markers.empty()) {
      dirty_ = false;
      return;
    }

    pub_->publish(out);
    last_publish_time_ = now();
    dirty_ = false;
  }

  bool expireStaleSources()
  {
    if (stale_timeout_sec_ <= 0.0) {
      return false;
    }
    const auto now_t = now();
    bool changed = false;
    for (auto & source : sources_) {
      if (!source.watch_stale) {
        continue;
      }
      if (source.latest.markers.empty() || source.last_rx.nanoseconds() <= 0) {
        continue;
      }
      const double dt = (now_t - source.last_rx).seconds();
      if (dt > stale_timeout_sec_) {
        source.latest.markers.clear();
        changed = true;
      }
    }
    return changed;
  }

  std::string output_topic_;
  std::vector<std::string> input_topics_;
  std::vector<std::string> stale_timeout_topics_;
  double republish_period_sec_{0.1};
  double stale_timeout_sec_{0.0};
  bool timer_mode_{false};
  bool dirty_{true};
  std::vector<Source> sources_;
  std::vector<size_t> prev_counts_;
  std::vector<std::vector<std::string>> prev_namespaces_;
  double min_publish_period_sec_{0.05};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
};

}  // namespace camping_cart_map

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camping_cart_map::MarkerArrayAggregatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
