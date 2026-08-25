// HH_260723 - Verify ordinary one-way routes and explicitly authorized campsite returns.

#include <cmath>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// HH_260721 - Compile the plugin implementation into this isolated test to exercise createPlan directly.
#include "../src/lanelet_route_planner.cpp"
// HH_260730 - Exercise the source-specific goal policy before calling LaneletRoute directly.
#include "../src/goal_snapper_node.cpp"

namespace
{

// HH_260727 - Keep this unit test isolated from a concurrently running CAMROD ROS graph.
constexpr char kTestRouteLaneletIdsTopic[] =
  "/test/lanelet_route_planner/route_lanelet_ids";
constexpr char kTestRouteTurnSegmentsTopic[] =
  "/test/lanelet_route_planner/route_turn_segments";
constexpr char kTestReverseRouteRequestTopic[] =
  "/test/lanelet_route_planner/return_to_drop_zone";

std::filesystem::path activeMapPath()
{
  return std::filesystem::path(__FILE__).parent_path()
         .parent_path().parent_path() / "lanelet2_maps.osm";
}

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

double poseYaw(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & orientation = pose.pose.orientation;
  const double sin_yaw =
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cos_yaw =
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(sin_yaw, cos_yaw);
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
    // HH_260730 - Resolve the stable active-map entrypoint from this source file.
    // Versioned copies remain audit artifacts and must not make this regression test stale.
    const std::filesystem::path map_path = activeMapPath();
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
    node_->declare_parameter<bool>(
      "LaneletRoute.reverse_lanelet_request_requires_opposite_heading", false);
    // HH_260727 - Never publish test routes or requests on live planning topics.
    node_->declare_parameter<std::string>(
      "LaneletRoute.route_lanelet_ids_topic", kTestRouteLaneletIdsTopic);
    node_->declare_parameter<std::string>(
      "LaneletRoute.route_turn_segments_topic", kTestRouteTurnSegmentsTopic);
    node_->declare_parameter<std::string>(
      "LaneletRoute.reverse_lanelet_request_topic", kTestReverseRouteRequestTopic);
    planner_ = std::make_unique<camrod_planning::LaneletRoutePlanner>();
    planner_->configure(node_, "LaneletRoute", nullptr, nullptr);
    planner_->activate();

    request_node_ = std::make_shared<rclcpp::Node>("reverse_route_request_test");
    request_publisher_ =
      request_node_->create_publisher<avg_msgs::msg::PlanningRecallRequest>(
      kTestReverseRouteRequestTopic, rclcpp::QoS(10));
    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(request_node_);
  }

  void TearDown() override
  {
    planner_->deactivate();
    planner_->cleanup();
    executor_.remove_node(request_node_);
    executor_.remove_node(node_->get_node_base_interface());
    request_publisher_.reset();
    request_node_.reset();
    planner_.reset();
    node_.reset();
  }

  bool authorizeReturnRoute(const std::string & source = "test_campsite_return")
  {
    for (int attempt = 0; attempt < 100; ++attempt) {
      if (request_publisher_->get_subscription_count() > 0U) {
        break;
      }
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (request_publisher_->get_subscription_count() == 0U) {
      return false;
    }

    avg_msgs::msg::PlanningRecallRequest request;
    request.site_name = "camping_site_12";
    request.source = source;
    request_publisher_->publish(request);
    for (int attempt = 0; attempt < 20; ++attempt) {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return true;
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::unique_ptr<camrod_planning::LaneletRoutePlanner> planner_;
  std::shared_ptr<rclcpp::Node> request_node_;
  rclcpp::Publisher<avg_msgs::msg::PlanningRecallRequest>::SharedPtr request_publisher_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(LaneletRoutePlannerTest, UsesShortPathFromDropZoneToRoadsideSite)
{
  const auto goal = makePose(12.7921, 22.52, -77.0);
  const auto path = planner_->createPlan(
    makePose(-13.5777, 40.7413, 10.4),
    goal);

  ASSERT_GT(path.poses.size(), 100U);
  EXPECT_LT(pathLength(path), 80.0);
  // HH_260727 - Preserve an operator-requested final heading instead of centerline tangent yaw.
  const double final_yaw_error = std::atan2(
    std::sin(poseYaw(path.poses.back()) - poseYaw(goal)),
    std::cos(poseYaw(path.poses.back()) - poseYaw(goal)));
  EXPECT_NEAR(final_yaw_error, 0.0, 1.0e-6);
}

TEST_F(LaneletRoutePlannerTest, PlansNearbyGoalAndPreservesRequestedYaw)
{
  const auto reference_path = planner_->createPlan(
    makePose(-13.5777, 40.7413, 10.4),
    makePose(12.7921, 22.52, -77.0));
  ASSERT_GT(reference_path.poses.size(), 30U);

  auto near_start = reference_path.poses[10U];
  auto near_goal = reference_path.poses[18U];
  near_goal.pose.orientation = makePose(0.0, 0.0, 145.0).pose.orientation;
  const auto near_path = planner_->createPlan(near_start, near_goal);

  ASSERT_GE(near_path.poses.size(), 2U);
  EXPECT_GT(pathLength(near_path), 0.5);
  EXPECT_LT(pathLength(near_path), 3.0);
  const double final_yaw_error = std::atan2(
    std::sin(poseYaw(near_path.poses.back()) - poseYaw(near_goal)),
    std::cos(poseYaw(near_path.poses.back()) - poseYaw(near_goal)));
  EXPECT_NEAR(final_yaw_error, 0.0, 1.0e-6);
}

TEST_F(
  LaneletRoutePlannerTest,
  ManualGoalProjectsPositionPreservesYawAndMatchesPathEndpoint)
{
  constexpr char kManualInputTopic[] = "/test/goal_snapper/manual_goal";
  constexpr char kRegulatedInputTopic[] = "/test/goal_snapper/regulated_goal";
  constexpr char kOutputTopic[] = "/test/goal_snapper/output_goal";
  constexpr char kOutputAvgTopic[] = "/test/goal_snapper/output_goal_avg";
  constexpr char kSourceTopic[] = "/test/goal_snapper/goal_source";

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("map_path", activeMapPath().string()),
      rclcpp::Parameter("offset_lat", 36.8435737),
      rclcpp::Parameter("offset_lon", 128.0925646),
      rclcpp::Parameter("offset_alt", 0.0),
      rclcpp::Parameter("input_goal_topic", kRegulatedInputTopic),
      rclcpp::Parameter("manual_input_goal_topic", kManualInputTopic),
      rclcpp::Parameter("auxiliary_input_goal_topic", ""),
      rclcpp::Parameter("output_goal_topic", kOutputAvgTopic),
      rclcpp::Parameter("output_goal_topic_ros", kOutputTopic),
      rclcpp::Parameter("output_goal_source_topic", kSourceTopic),
      rclcpp::Parameter("goal_source_settle_delay_s", 0.0),
      rclcpp::Parameter("restrict_to_connected_lanelet_component", false),
      rclcpp::Parameter("restrict_to_cost_grid_component", false),
      rclcpp::Parameter("reissue_active_goal_on_pose_jump", false),
      rclcpp::Parameter("reissue_active_goal_after_route_recovery", false),
    });
  auto goal_snapper = std::make_shared<GoalSnapperNode>(options);

  std::vector<geometry_msgs::msg::PoseStamped> outputs;
  std::vector<std::string> source_policies;
  auto output_subscription =
    request_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    kOutputTopic, rclcpp::QoS(10),
    [&outputs](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      outputs.push_back(*msg);
    });
  auto source_subscription =
    request_node_->create_subscription<std_msgs::msg::String>(
    kSourceTopic, rclcpp::QoS(10).reliable().transient_local(),
    [&source_policies](const std_msgs::msg::String::ConstSharedPtr msg) {
      source_policies.push_back(msg->data);
    });
  auto manual_publisher =
    request_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    kManualInputTopic, rclcpp::QoS(10));
  auto regulated_publisher =
    request_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    kRegulatedInputTopic, rclcpp::QoS(10));
  executor_.add_node(goal_snapper);

  for (int attempt = 0; attempt < 100; ++attempt) {
    if (manual_publisher->get_subscription_count() > 0U &&
      regulated_publisher->get_subscription_count() > 0U &&
      output_subscription->get_publisher_count() > 0U &&
      source_subscription->get_publisher_count() > 0U)
    {
      break;
    }
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const auto clicked_goal = makePose(12.7921, 22.52, 107.8);
  manual_publisher->publish(clicked_goal);
  for (int attempt = 0; attempt < 200 && outputs.empty(); ++attempt) {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  regulated_publisher->publish(clicked_goal);
  for (int attempt = 0; attempt < 200 && outputs.size() < 2U; ++attempt) {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  for (int attempt = 0; attempt < 100 && source_policies.size() < 2U; ++attempt) {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  executor_.remove_node(goal_snapper);

  ASSERT_EQ(outputs.size(), 2U);
  ASSERT_EQ(source_policies.size(), 2U);
  EXPECT_EQ(source_policies[0], "manual");
  EXPECT_EQ(source_policies[1], "regulated");
  const auto & manual_goal = outputs[0];
  const auto & regulated_goal = outputs[1];
  const double manual_snap_distance = std::hypot(
    manual_goal.pose.position.x - clicked_goal.pose.position.x,
    manual_goal.pose.position.y - clicked_goal.pose.position.y);
  EXPECT_GT(manual_snap_distance, 0.5);
  EXPECT_LT(manual_snap_distance, 3.0);

  const double manual_yaw_error = std::atan2(
    std::sin(poseYaw(manual_goal) - poseYaw(clicked_goal)),
    std::cos(poseYaw(manual_goal) - poseYaw(clicked_goal)));
  EXPECT_NEAR(manual_yaw_error, 0.0, 1.0e-6);
  const double regulated_manual_yaw_difference = std::abs(
    std::atan2(
      std::sin(poseYaw(regulated_goal) - poseYaw(manual_goal)),
      std::cos(poseYaw(regulated_goal) - poseYaw(manual_goal))));
  EXPECT_GT(regulated_manual_yaw_difference, 0.5);

  const auto path = planner_->createPlan(
    makePose(-13.990, 43.540, 7.7), manual_goal);
  ASSERT_GT(path.poses.size(), 100U);
  const auto & endpoint = path.poses.back();
  EXPECT_NEAR(endpoint.pose.position.x, manual_goal.pose.position.x, 1.0e-6);
  EXPECT_NEAR(endpoint.pose.position.y, manual_goal.pose.position.y, 1.0e-6);
  const double endpoint_yaw_error = std::atan2(
    std::sin(poseYaw(endpoint) - poseYaw(manual_goal)),
    std::cos(poseYaw(endpoint) - poseYaw(manual_goal)));
  EXPECT_NEAR(endpoint_yaw_error, 0.0, 1.0e-6);
}

TEST(PathQualityMetrics, MeasuresLengthTurnAndCurvature)
{
  nav_msgs::msg::Path path;
  path.poses.push_back(makePose(0.0, 0.0, 0.0));
  path.poses.push_back(makePose(1.0, 0.0, 0.0));
  path.poses.push_back(makePose(1.0, 1.0, 0.0));

  const auto metrics = camrod_planning::measurePathQuality(path);

  EXPECT_EQ(metrics.segment_count, 2U);
  EXPECT_NEAR(metrics.length_m, 2.0, 1.0e-9);
  EXPECT_NEAR(metrics.total_abs_turn_rad, M_PI_2, 1.0e-9);
  EXPECT_NEAR(metrics.mean_abs_curvature_inv_m, M_PI_2 / 2.0, 1.0e-9);
  EXPECT_NEAR(metrics.max_abs_curvature_inv_m, M_PI_2, 1.0e-9);
}

TEST_F(LaneletRoutePlannerTest, ReversesShortestPathAfterSiteTurnaround)
{
  ASSERT_TRUE(authorizeReturnRoute());
  const auto path = planner_->createPlan(
    makePose(12.7921, 22.52, 103.0),
    makePose(-13.5777, 40.7413, -82.2));

  ASSERT_GT(path.poses.size(), 100U);
  EXPECT_LT(pathLength(path), 80.0);
  const auto & first = path.poses.front().pose.position;
  const auto & last = path.poses.back().pose.position;
  // HH_260730 - The stable v1.0.3 map projects this authored roadside pose by
  // 2.28 m; retain a bounded snap assertion without coupling to an old map copy.
  EXPECT_LT(std::hypot(first.x - 12.7921, first.y - 22.52), 3.0);
  EXPECT_LT(std::hypot(last.x + 13.5777, last.y - 40.7413), 3.0);

  // HH_260721 - A transient yaw change must not switch a repeated drop-zone goal to the long loop.
  const auto replanned_path = planner_->createPlan(
    makePose(12.7921, 22.52, 23.0),
    makePose(-13.5777, 40.7413, -82.2));
  EXPECT_LT(pathLength(replanned_path), 80.0);
}

TEST_F(LaneletRoutePlannerTest, ExplicitReturnReversesShortestPathBeforeTurnaround)
{
  ASSERT_TRUE(authorizeReturnRoute());
  const auto path = planner_->createPlan(
    makePose(12.7921, 22.52, 23.0),
    makePose(-13.5777, 40.7413, -82.2));

  // HH_260819 - A Return pressed during outbound travel must not retain the
  // 120+ m one-way loop merely because the stationary 180deg turn is pending.
  ASSERT_GT(path.poses.size(), 100U);
  EXPECT_LT(pathLength(path), 80.0);
}

TEST_F(LaneletRoutePlannerTest, ExplicitReturnStartsFromCurrentLanePositionNotEntryAnchor)
{
  const auto outbound = planner_->createPlan(
    makePose(-13.5777, 40.7413, 10.4),
    makePose(12.7921, 22.52, -77.0));
  ASSERT_GT(outbound.poses.size(), 30U);

  // HH_260825 - Model a diagonal campsite exit that reaches the active lane
  // several path samples before the historical arrival anchor. LaneletRoute
  // must accept that live start directly after the typed Return authorization.
  auto live_exit_pose = outbound.poses[outbound.poses.size() - 20U];
  live_exit_pose.pose.orientation = makePose(0.0, 0.0, 103.0).pose.orientation;
  ASSERT_TRUE(authorizeReturnRoute("test:done_live_lanelet"));
  const auto return_path = planner_->createPlan(
    live_exit_pose, makePose(-13.5777, 40.7413, -82.2));

  ASSERT_GT(return_path.poses.size(), 80U);
  EXPECT_LT(pathLength(return_path), 80.0);
  const auto & first = return_path.poses.front().pose.position;
  EXPECT_LT(
    std::hypot(
      first.x - live_exit_pose.pose.position.x,
      first.y - live_exit_pose.pose.position.y),
    0.5);
}

TEST_F(LaneletRoutePlannerTest, KeepsOneWayRouteForOrdinaryOppositeHeadingGoal)
{
  const auto path = planner_->createPlan(
    makePose(12.7921, 22.52, 103.0),
    makePose(-13.5777, 40.7413, -82.2));

  EXPECT_GT(pathLength(path), 120.0);
}

TEST_F(LaneletRoutePlannerTest, RoadsideReturnKeepsForwardOneWayLoop)
{
  ASSERT_TRUE(authorizeReturnRoute("test:done_roadside_forward"));
  const auto path = planner_->createPlan(
    makePose(12.7921, 22.52, 23.0),
    makePose(-13.5777, 40.7413, -82.2));

  // HH_260819 - A roadside robot leaves parallel to the road. It must follow
  // the legal forward loop instead of rotating into the physical lane edge.
  ASSERT_GT(path.poses.size(), 100U);
  EXPECT_GT(pathLength(path), 120.0);
}

TEST_F(LaneletRoutePlannerTest, KeepsOneWayRouteBelowReverseHeadingThreshold)
{
  const auto path = planner_->createPlan(
    makePose(12.7921, 22.52, 23.0),
    makePose(-13.5777, 40.7413, -82.2));

  EXPECT_GT(pathLength(path), 120.0);
}

}  // namespace
