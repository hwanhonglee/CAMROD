// HH_251231: Lanelet-based cost layer header for Nav2
#pragma once

#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <avg_msgs/msg/occupancy_grid.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>

namespace camrod_map::cost_map
{
// Implements `OccupancyGrid` behavior.
// HH_251230: Simple layer that consumes an OccupancyGrid (map frame) and writes costs
class LaneletCostLayer : public nav2_costmap_2d::Layer
{
public:
  LaneletCostLayer();

  void onInitialize() override;
  void updateBounds(double, double, double, double *, double *, double *, double *) override;
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override;
  bool isClearable() override {return false;}
  // Resets `reset` state.
  void reset() override {}

private:
  void gridCallback(const avg_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  rclcpp::Subscription<avg_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  std::string source_topic_;
  unsigned char lethal_threshold_{65};
  unsigned char unknown_value_{nav2_costmap_2d::NO_INFORMATION};
  bool write_unknown_{false};

  avg_msgs::msg::OccupancyGrid::ConstSharedPtr latest_grid_;
  bool has_data_{false};
};
}  // namespace camrod_map::cost_map

PLUGINLIB_EXPORT_CLASS(camrod_map::cost_map::LaneletCostLayer, nav2_costmap_2d::Layer)
