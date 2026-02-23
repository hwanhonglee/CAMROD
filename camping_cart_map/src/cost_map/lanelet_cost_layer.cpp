// HH_251231: Lanelet-based cost layer implementation (OccupancyGrid input)

#include "camping_cart_map/cost_map/lanelet_cost_layer.hpp"

#include <algorithm>

namespace camping_cart_map::cost_map
{
LaneletCostLayer::LaneletCostLayer()
{
  // HH_251231: default constructor
}

void LaneletCostLayer::onInitialize()
{
  auto node = node_.lock();
  // HH_260109 Default to map-prefixed cost grid topic.
  declareParameter("source_topic", rclcpp::ParameterValue(std::string("/map/lanelet_cost_grid")));
  declareParameter("lethal_threshold", rclcpp::ParameterValue(65));
  declareParameter("unknown_value", rclcpp::ParameterValue(static_cast<int>(nav2_costmap_2d::NO_INFORMATION)));

  // 2026-01-29 21:09: Use standard ROS2 param separator (.) for layered params.
  node->get_parameter(name_ + ".source_topic", source_topic_);
  int lethal_tmp{};
  node->get_parameter(name_ + ".lethal_threshold", lethal_tmp);
  lethal_threshold_ = static_cast<unsigned char>(std::clamp(lethal_tmp, 0, 100));
  int unknown_tmp{};
  node->get_parameter(name_ + ".unknown_value", unknown_tmp);
  unknown_value_ = static_cast<unsigned char>(std::clamp(unknown_tmp, 0, 255));

  // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
  RCLCPP_DEBUG(node->get_logger(), "lanelet_cost_layer subscribing %s", source_topic_.c_str());

  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    source_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    std::bind(&LaneletCostLayer::gridCallback, this, std::placeholders::_1));

  enabled_ = true;
}

void LaneletCostLayer::gridCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  latest_grid_ = msg;
  has_data_ = true;
  // 2026-02-06 21:18: Mark layer current once grid arrives so planner doesn't stall waiting on costmap.
  current_ = true;
}

void LaneletCostLayer::updateBounds(
  double /*origin_x*/, double /*origin_y*/, double /*origin_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!has_data_ || !latest_grid_) {
    return;
  }

  // HH_251231: use full grid bounds (simple, conservative)
  const auto & info = latest_grid_->info;
  // 2026-02-06 11:16: Avoid shrinking the master bounds; only expand to prevent
  // "Illegal bounds change" warnings when the input grid window shifts.
  const double grid_min_x = info.origin.position.x;
  const double grid_min_y = info.origin.position.y;
  const double grid_max_x = info.origin.position.x + info.width * info.resolution;
  const double grid_max_y = info.origin.position.y + info.height * info.resolution;
  *min_x = std::min(*min_x, grid_min_x);
  *min_y = std::min(*min_y, grid_min_y);
  *max_x = std::max(*max_x, grid_max_x);
  *max_y = std::max(*max_y, grid_max_y);
}

void LaneletCostLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!has_data_ || !latest_grid_) {
    return;
  }

  const auto grid = latest_grid_;  // copy shared ptr
  const auto & info = grid->info;

  // HH_251231: iterate only requested window
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      // world to incoming grid index
      int gx = static_cast<int>((wx - info.origin.position.x) / info.resolution);
      int gy = static_cast<int>((wy - info.origin.position.y) / info.resolution);
      if (gx < 0 || gy < 0 || gx >= static_cast<int>(info.width) || gy >= static_cast<int>(info.height)) {
        continue;
      }
      const auto idx = gx + gy * info.width;
      const auto val = grid->data[idx];
      if (val < 0) {
        if (unknown_value_ != nav2_costmap_2d::NO_INFORMATION) {
          master_grid.setCost(i, j, unknown_value_);
        }
        continue;
      }
      unsigned char cost = nav2_costmap_2d::FREE_SPACE;
      if (val >= lethal_threshold_) {
        cost = nav2_costmap_2d::LETHAL_OBSTACLE;
      } else {
        // scale 0~lethal_threshold -> 0~254
        cost = static_cast<unsigned char>(
          (static_cast<float>(val) / static_cast<float>(lethal_threshold_)) * 254.0f);
      }
      master_grid.setCost(i, j, cost);
    }
  }
}
}  // namespace camping_cart_map::cost_map
