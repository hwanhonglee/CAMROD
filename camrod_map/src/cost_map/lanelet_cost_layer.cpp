// Implements `implementation` behavior.
// HH_251231: Lanelet-based cost layer implementation (OccupancyGrid input)

#include "camrod_map/cost_map/lanelet_cost_layer.hpp"

#include <algorithm>
#include <avg_msgs/msg/occupancy_grid.hpp>

namespace camrod_map::cost_map
{
// Implements `LaneletCostLayer` behavior.
LaneletCostLayer::LaneletCostLayer()
{
  // HH_251231: default constructor
}

// Handles the `onInitialize` callback.
void LaneletCostLayer::onInitialize()
{
  auto node = node_.lock();
  // HH_260109 Default to map-prefixed cost grid topic.
  declareParameter("source_topic", rclcpp::ParameterValue(std::string("/map/cost_grid/lanelet")));
  declareParameter("lethal_threshold", rclcpp::ParameterValue(65));
  declareParameter("unknown_value", rclcpp::ParameterValue(static_cast<int>(nav2_costmap_2d::NO_INFORMATION)));
  // HH_260316-00:00 Control whether unknown input cells overwrite master costmap.
  // false: keep existing master values on unknown cells (safer for layered lanelet/path fusion).
  // true : write unknown_value into master on unknown cells.
  declareParameter("write_unknown", rclcpp::ParameterValue(false));

  // 2026-01-29 21:09: Use standard ROS2 param separator (.) for layered params.
  node->get_parameter(name_ + ".source_topic", source_topic_);
  int lethal_tmp{};
  node->get_parameter(name_ + ".lethal_threshold", lethal_tmp);
  // HH_260315-00:00 Keep full byte range for lethal threshold.
  // Many project overlays intentionally use 101 to treat 0~100 grids as non-lethal
  // "soft cost" layers. Clamping to 100 incorrectly turned value=100 into lethal.
  lethal_threshold_ = static_cast<unsigned char>(std::clamp(lethal_tmp, 0, 255));
  int unknown_tmp{};
  node->get_parameter(name_ + ".unknown_value", unknown_tmp);
  unknown_value_ = static_cast<unsigned char>(std::clamp(unknown_tmp, 0, 255));
  node->get_parameter(name_ + ".write_unknown", write_unknown_);

  // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
  RCLCPP_DEBUG(node->get_logger(), "lanelet_cost_layer subscribing %s", source_topic_.c_str());

  sub_ = node->create_subscription<avg_msgs::msg::OccupancyGrid>(
    source_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    std::bind(&LaneletCostLayer::gridCallback, this, std::placeholders::_1));

  enabled_ = true;
}

// Implements `gridCallback` behavior.
void LaneletCostLayer::gridCallback(const avg_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  latest_grid_ = msg;
  has_data_ = true;
  // 2026-02-06 21:18: Mark layer current once grid arrives so planner doesn't stall waiting on costmap.
  current_ = true;
}

// Updates `Bounds` state.
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

// Updates `Costs` state.
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
        // HH_260316-00:00 Unknown-cell overwrite is layer-dependent.
        // Path/sensor layers commonly publish unknown outside valid regions; writing those
        // blindly can erase base lanelet costs and produce shortcut/straight paths.
        if (write_unknown_) {
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
}  // namespace camrod_map::cost_map
