// HH_260721 - Verify forward and yaw-selected reverse routes against the active campground OSM.

#include <cmath>
#include <filesystem>
#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// HH_260721 - Compile the plugin implementation into this isolated test to exercise createPlan directly.
#include "../src/lanelet_route_planner.cpp"

namespace
{

geometry_msgs::msg::PoseStamped makePose(
  const double x, const double y, const double yaw_degrees)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  const double yaw = yaw_degrees * M_PI / 180.0;
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

double pathLength(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (std::size_t index = 1; index < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1U].pose.position;
    const auto & current = path.poses[index].pose.position;
    length += std::hypot(current.x - previous.x, current.y - previous.y);
  }
  return length;
}

class LaneletRoutePlannerTest : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("lanelet_route_planner_test");
    // HH_260721 - Resolve the active map from this source file so parentheses need no CMake escaping.
    const std::filesystem::path map_path = std::filesystem::path(__FILE__).parent_path()
      .parent_path().parent_path() / "lanelet2_maps_(copy_park_moved).osm";
    node_->declare_parameter<std::string>("LaneletRoute.map_path", map_path.string());
    node_->declare_parameter<double>("LaneletRoute.offset_lat", 36.8435737);
    node_->declare_parameter<double>("LaneletRoute.offset_lon", 128.0925646);
    node_->declare_parameter<double>("LaneletRoute.offset_alt", 0.0);
    // HH_260721 - Match the deployed planner profile while testing route direction.
    node_->declare_parameter<bool>("LaneletRoute.allow_lane_changes", false);
    node_->declare_parameter<bool>("LaneletRoute.async_initialization", false);
    node_->declare_parameter<bool>("LaneletRoute.enable_reverse_lanelet_shortest_path", true);
    node_->declare_parameter<double>(
      "LaneletRoute.reverse_lanelet_start_heading_threshold_deg", 120.0);
    planner_ = std::make_unique<camrod_planning::LaneletRoutePlanner>();
    planner_->configure(node_, "LaneletRoute", nullptr, nullptr);
    planner_->activate();
  }

  void TearDown() override
  {
    planner_->deactivate();
    planner_->cleanup();
    planner_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::unique_ptr<camrod_planning::LaneletRoutePlanner> planner_;
};

TEST_F(LaneletRoutePlannerTest, UsesShortPathFromDropZoneToRoadsideSite)
{
  const auto path = planner_->createPlan(
    makePose(-13.5777, 40.7413, 10.4),
    makePose(12.7921, 22.52, -77.0));

  ASSERT_GT(path.poses.size(), 100U);
  EXPECT_LT(pathLength(path), 80.0);
}

TEST_F(LaneletRoutePlannerTest, ReversesShortestPathAfterSiteTurnaround)
{
  const auto path = planner_->createPlan(
    makePose(12.7921, 22.52, 103.0),
    makePose(-13.5777, 40.7413, -82.2));

  ASSERT_GT(path.poses.size(), 100U);
  EXPECT_LT(pathLength(path), 80.0);
  const auto & first = path.poses.front().pose.position;
  const auto & last = path.poses.back().pose.position;
  // HH_260721 - The authored roadside service pose is 1.20 m from its centerline projection.
  EXPECT_LT(std::hypot(first.x - 12.7921, first.y - 22.52), 2.0);
  EXPECT_LT(std::hypot(last.x + 13.5777, last.y - 40.7413), 3.0);

  // HH_260721 - A transient yaw change must not switch a repeated drop-zone goal to the long loop.
  const auto replanned_path = planner_->createPlan(
    makePose(12.7921, 22.52, 23.0),
    makePose(-13.5777, 40.7413, -82.2));
  EXPECT_LT(pathLength(replanned_path), 80.0);
}

TEST_F(LaneletRoutePlannerTest, KeepsOneWayRouteBelowReverseHeadingThreshold)
{
  const auto path = planner_->createPlan(
    makePose(12.7921, 22.52, 23.0),
    makePose(-13.5777, 40.7413, -82.2));

  EXPECT_GT(pathLength(path), 120.0);
}

}  // namespace
