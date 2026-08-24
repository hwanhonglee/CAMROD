// HH_260824 - Lock signed B1-B13 geometry through the production GoalSnapperNode.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "avg_msgs/msg/avg_pose_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

// Compile only the production snapper into this small target. The existing
// route-planner test also includes the full Nav2 plugin and is intentionally
// kept independent from this static map contract.
#include "../src/goal_snapper_node.cpp"

namespace
{

struct ActiveCampsiteFixture
{
  const char * name;
  const char * service_mode;
  double snap_x;
  double snap_y;
  double snap_yaw_deg;
  double signed_lateral_m;
  double operational_offset_m;
};

constexpr std::array<ActiveCampsiteFixture, 13> kFixtures{{
  {"camping_site_1", "turnaround", 22.357515191, -6.908337803,
    -63.251744, 3.931930, 3.931930},
  {"camping_site_2", "turnaround", 22.454856615, -7.101474235,
    -63.251706, -3.235030, 3.235030},
  {"camping_site_3", "turnaround", 19.772336735, -1.222936079,
    -65.285918, 3.947756, 3.947756},
  {"camping_site_4", "turnaround", 19.307410106, -0.152900333,
    -67.428154, -3.118706, 3.118706},
  {"camping_site_5", "turnaround", 17.382903354, 4.336168303,
    -66.896141, 3.888036, 3.888036},
  {"camping_site_6", "turnaround", 16.786389776, 5.715787406,
    -66.421645, -3.092795, 3.092795},
  {"camping_site_7", "turnaround", 15.085466253, 9.670478273,
    -66.942319, 3.867722, 3.867722},
  {"camping_site_8", "turnaround", 14.147071672, 11.827906846,
    -66.924479, -3.018049, 3.018049},
  {"camping_site_9", "turnaround", 12.679983302, 15.667433959,
    -68.215752, 3.966901, 3.966901},
  {"camping_site_10", "turnaround", 11.891069361, 17.727651240,
    -69.184081, -3.063739, 3.063739},
  {"camping_site_11", "roadside_stop", 10.821530478, 20.516472190,
    -68.783648, 4.315470, 0.300000},
  {"camping_site_12", "roadside_stop", 10.103421888, 23.093735129,
    -78.439719, -3.316560, 0.300000},
  {"camping_site_13", "roadside_stop", 9.626348588, 27.418616995,
    -91.177942, -9.017805, 0.300000},
}};

std::filesystem::path activeMapPath()
{
  return std::filesystem::path(__FILE__).parent_path().parent_path()
         .parent_path() / "lanelet2_maps.osm";
}

std::filesystem::path activeCampsitesPath()
{
  return std::filesystem::path(__FILE__).parent_path().parent_path() / "config" /
         "camping_sites.yaml";
}

geometry_msgs::msg::PoseStamped makeGoal(const YAML::Node & site)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = site["x"].as<double>();
  goal.pose.position.y = site["y"].as<double>();
  const double yaw = site["yaw_deg"].as<double>() * M_PI / 180.0;
  goal.pose.orientation.z = std::sin(yaw * 0.5);
  goal.pose.orientation.w = std::cos(yaw * 0.5);
  return goal;
}

double poseYaw(const avg_msgs::msg::AvgPoseStamped & pose)
{
  const auto & orientation = pose.pose.orientation;
  return std::atan2(
    2.0 * (orientation.w * orientation.z +
    orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y +
    orientation.z * orientation.z));
}

TEST(ActiveCampsiteGeometry, LocksProductionSnapYawSignedSideAndServicePolicy)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  constexpr char kInputTopic[] = "/test/active_campsites/regulated_goal";
  constexpr char kOutputTopic[] = "/test/active_campsites/snapped_goal";

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("map_path", activeMapPath().string()),
      rclcpp::Parameter("offset_lat", 36.8435737),
      rclcpp::Parameter("offset_lon", 128.0925646),
      rclcpp::Parameter("offset_alt", 0.0),
      rclcpp::Parameter("input_goal_topic", kInputTopic),
      rclcpp::Parameter("manual_input_goal_topic", ""),
      rclcpp::Parameter("auxiliary_input_goal_topic", ""),
      rclcpp::Parameter("output_goal_topic", kOutputTopic),
      rclcpp::Parameter(
        "output_goal_topic_ros", "/test/active_campsites/snapped_goal_ros"),
      rclcpp::Parameter(
        "output_goal_source_topic", "/test/active_campsites/goal_source"),
      rclcpp::Parameter("goal_source_settle_delay_s", 0.0),
      rclcpp::Parameter("restrict_to_connected_lanelet_component", false),
      rclcpp::Parameter("restrict_to_cost_grid_component", false),
      rclcpp::Parameter("sequential_goal_release_enable", false),
      rclcpp::Parameter("reissue_active_goal_on_pose_jump", false),
      rclcpp::Parameter("reissue_active_goal_after_route_recovery", false),
    });
  auto snapper = std::make_shared<GoalSnapperNode>(options);
  auto request_node = std::make_shared<rclcpp::Node>("active_campsite_test_client");
  std::vector<avg_msgs::msg::AvgPoseStamped> outputs;
  auto output_subscription =
    request_node->create_subscription<avg_msgs::msg::AvgPoseStamped>(
    kOutputTopic, rclcpp::QoS(10),
    [&outputs](const avg_msgs::msg::AvgPoseStamped::ConstSharedPtr message) {
      outputs.push_back(*message);
    });
  auto input_publisher =
    request_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    kInputTopic, rclcpp::QoS(10));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(snapper);
  executor.add_node(request_node);

  for (int attempt = 0; attempt < 100; ++attempt) {
    if (input_publisher->get_subscription_count() > 0U &&
      output_subscription->get_publisher_count() > 0U)
    {
      break;
    }
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GT(input_publisher->get_subscription_count(), 0U);
  ASSERT_GT(output_subscription->get_publisher_count(), 0U);

  const YAML::Node sites =
    YAML::LoadFile(activeCampsitesPath().string())["camping_sites"];
  ASSERT_TRUE(sites.IsSequence());
  ASSERT_EQ(sites.size(), kFixtures.size());
  for (std::size_t index = 0; index < kFixtures.size(); ++index) {
    const auto & fixture = kFixtures[index];
    const YAML::Node site = sites[index];
    SCOPED_TRACE(fixture.name);
    ASSERT_EQ(site["type"].as<std::string>(), fixture.name);
    ASSERT_EQ(site["service_mode"].as<std::string>(), fixture.service_mode);
    input_publisher->publish(makeGoal(site));
    for (int attempt = 0; attempt < 200 && outputs.size() <= index; ++attempt) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_GT(outputs.size(), index);

    const auto & snapped = outputs[index];
    const double yaw = poseYaw(snapped);
    const double dx = site["x"].as<double>() - snapped.pose.position.x;
    const double dy = site["y"].as<double>() - snapped.pose.position.y;
    const double forward = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double lateral = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const bool roadside = index >= 10U;
    const double bounded_offset = std::clamp(std::abs(lateral), 0.20, 7.0);
    const double operational_offset =
      roadside ? std::min(bounded_offset, 0.30) : bounded_offset;

    std::cout << "CAMPSITE_GEOMETRY B" << index + 1U
              << " service=" << fixture.service_mode
              << " snap_x=" << snapped.pose.position.x
              << " snap_y=" << snapped.pose.position.y
              << " snap_yaw_deg=" << yaw * 180.0 / M_PI
              << " forward_m=" << forward << " lateral_m=" << lateral
              << " operational_offset_m=" << operational_offset << std::endl;
    EXPECT_NEAR(snapped.pose.position.x, fixture.snap_x, 0.02);
    EXPECT_NEAR(snapped.pose.position.y, fixture.snap_y, 0.02);
    EXPECT_NEAR(
      std::atan2(
        std::sin(yaw - fixture.snap_yaw_deg * M_PI / 180.0),
        std::cos(yaw - fixture.snap_yaw_deg * M_PI / 180.0)),
      0.0, 0.2 * M_PI / 180.0);
    EXPECT_NEAR(forward, 0.0, 0.01);
    EXPECT_NEAR(lateral, fixture.signed_lateral_m, 0.02);
    EXPECT_EQ(lateral >= 0.0, fixture.signed_lateral_m >= 0.0);
    EXPECT_NEAR(operational_offset, fixture.operational_offset_m, 0.02);
  }

  executor.remove_node(request_node);
  executor.remove_node(snapper);
  rclcpp::shutdown();
}

}  // namespace
