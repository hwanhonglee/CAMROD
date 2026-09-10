// Exercise the real completion transitions without launching the robot. Topic
// remaps and a separate DDS domain prevent accidental field command injection.
#include <limits>
#include <thread>

#include "gtest/gtest.h"

#define CAMROD_CONTROL_REVERSE_PARKING_TEST
#include "../src/reverse_parking_controller_node.cpp"

class ReverseParkingControllerTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::InitOptions options;
    options.set_domain_id(188);
    rclcpp::init(0, nullptr, options);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args",
      "--remap", "/control/cmd_vel_raw:=/test/reverse_parking/cmd_vel",
      "--remap", "/service/state:=/test/reverse_parking/service_state",
      "--remap", "/parking/operation:=/test/reverse_parking/operation",
      "--remap", "/localization/pose:=/test/reverse_parking/pose",
      "--remap", "/platform/status:=/test/reverse_parking/platform_status",
      "--remap", "/planning/drop_zone_goal_raw:=/test/reverse_parking/goal",
      "--remap", "/parking/reverse_parking_controller/status:=/test/reverse_parking/status",
      "--remap", "/parking/reverse_parking_controller/request_operation:=/test/reverse_parking/request",
      "--remap", "/system/diagnostics:=/test/reverse_parking/diagnostics",
      "--remap", "/parking/reverse_parking_controller/path_ros:=/test/reverse_parking/path"});
    node_ = std::make_shared<ReverseParkingControllerNode>(options);
    node_->control_timer_->cancel();
    node_->complete_without_charging_ = true;
    node_->maximum_reverse_distance_m_ = 5.0;
    node_->station_axis_tolerance_m_ = 0.25;
  }
  void TearDown() override { node_.reset(); }
  void pose(const double x, const double y) {
    avg_msgs::msg::AvgPoseStamped message;
    message.header.frame_id = "map";
    message.header.stamp = node_->now();
    message.pose.position.x = x;
    message.pose.position.y = y;
    message.pose.orientation.z = 1.0;  // Body yaw pi: reverse toward map +X.
    message.pose.orientation.w = 0.0;
    node_->last_vehicle_pose_ = message;
    node_->last_vehicle_pose_time_ = node_->now();
  }
  bool start(const double station_x) {
    node_->station_pose_ = {station_x, 0.0, 0.0};
    pose(0.0, 0.0);
    return node_->applyOperation(avg_msgs::msg::MotionOperation::START, "test").first;
  }
  void tick() { node_->onTimer(); }
  void stale() {
    node_->last_vehicle_pose_time_ = node_->now() - rclcpp::Duration::from_seconds(10.0);
  }
  void charging() { node_->is_charging_ = true; }
  void requireCharging() { node_->complete_without_charging_ = false; }
  void travelLimit(const double limit) { node_->maximum_reverse_distance_m_ = limit; }
  void expireReverseTimeout() {
    node_->phase_start_time_ = node_->now() - rclcpp::Duration::from_seconds(31.0);
  }
  void cancel() { node_->applyOperation(avg_msgs::msg::MotionOperation::CANCEL, "test"); }
  double reversed() const { return node_->distanceReversed(); }
  double tolerance() const { return node_->station_axis_tolerance_m_; }
  double initialDistance() const { return node_->initial_station_distance_m_; }
  double axisDistance() const { return node_->stationDistanceAlongReverseAxis(); }
  bool startObservedPose() {
    // HH_260909 - Station yaw retrimmed to -88.2127 deg.
    const double station_yaw = -88.2127 * M_PI / 180.0;
    node_->station_pose_ = {-11.3585, 40.0901, station_yaw};
    pose(-11.228455380998474, 39.89074377250848);
    node_->last_vehicle_pose_->pose.orientation.z = std::sin((station_yaw + M_PI) / 2.0);
    node_->last_vehicle_pose_->pose.orientation.w = std::cos((station_yaw + M_PI) / 2.0);
    return node_->applyOperation(avg_msgs::msg::MotionOperation::START, "test").first;
  }
  auto subscribeCommands(std::vector<avg_msgs::msg::AvgTwist> & commands) {
    return node_->create_subscription<avg_msgs::msg::AvgTwist>(
      node_->command_publisher_->get_topic_name(), 100,
      [&commands](avg_msgs::msg::AvgTwist::ConstSharedPtr message) {
        commands.push_back(*message);
      });
  }
  ReverseParkingPhase phase() const { return node_->phase_; }
  std::string detail() const { return node_->phase_detail_; }
  std::shared_ptr<ReverseParkingControllerNode> node_;
};

TEST_F(ReverseParkingControllerTest, NewStationContinuesPastOldLimitAndCompletesOnlyAtGoal) {
  ASSERT_TRUE(start(3.75));
  EXPECT_NE(detail().find("initial_station_distance_m=3.750"), std::string::npos);
  EXPECT_NE(detail().find("maximum_reverse_distance_m=5.000"), std::string::npos);
  pose(1.5, 0.0);
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kReverseApproach);
  pose(3.45, 0.0);
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kReverseApproach);
  pose(3.55, 0.0);
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kParked);
}

TEST_F(ReverseParkingControllerTest, TravelLimitMissIsErrorNotParked) {
  ASSERT_TRUE(start(3.75));
  pose(5.0, 0.0);
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kError);
  EXPECT_NE(detail().find("reverse distance limit reached"), std::string::npos);
  EXPECT_NE(detail().find("xy_error_m=1.250"), std::string::npos);
}

TEST_F(ReverseParkingControllerTest, GoalOutsideFiveMeterBoundIsRejectedBeforeMotion) {
  EXPECT_FALSE(start(5.001));
  EXPECT_EQ(phase(), ReverseParkingPhase::kError);
  EXPECT_NE(detail().find("initial_station_distance_m=5.001"), std::string::npos);
  EXPECT_NE(detail().find("maximum_reverse_distance_m=5.000"), std::string::npos);
}

TEST_F(ReverseParkingControllerTest, ExplicitSmallerOrInvalidBoundCannotBeAutoExtended) {
  travelLimit(1.5);
  EXPECT_FALSE(start(3.75));
  EXPECT_EQ(phase(), ReverseParkingPhase::kError);
  EXPECT_NE(detail().find("maximum_reverse_distance_m=1.500"), std::string::npos);
  travelLimit(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(start(1.0));
  EXPECT_EQ(phase(), ReverseParkingPhase::kError);
}

TEST_F(ReverseParkingControllerTest, UnreachableLateralMissStopsInsideAxialEnvelope) {
  ASSERT_TRUE(start(1.0));
  pose(0.8, 0.4);
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kError);
  EXPECT_NE(detail().find("station reverse axis reached"), std::string::npos);
}

TEST_F(ReverseParkingControllerTest, ReachableLateralOffsetContinuesUntilActualXyGoal) {
  ASSERT_TRUE(start(1.0));
  pose(0.75, 0.128);  // axis .25, XY .2809: not arrived, but still reachable.
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kReverseApproach);
  pose(0.79, 0.128);  // axis .21, XY .2459: inside the unchanged .25 m disk.
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kParked);
  EXPECT_NE(detail().find("station XY goal reached"), std::string::npos);
}

TEST_F(ReverseParkingControllerTest, ReachableNegativeLateralOffsetAlsoContinues) {
  ASSERT_TRUE(start(1.0));
  pose(0.75, -0.128);
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kReverseApproach);
  pose(0.79, -0.128);
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kParked);
}

TEST_F(ReverseParkingControllerTest, FinalApproachCannotCrossStationPlaneOutsideGoal) {
  ASSERT_TRUE(start(1.0));
  pose(0.75, 0.128);
  tick();
  ASSERT_EQ(phase(), ReverseParkingPhase::kReverseApproach);
  pose(1.01, 0.26);
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kError);
  EXPECT_NE(detail().find("station plane passed"), std::string::npos);
}

TEST_F(ReverseParkingControllerTest, TangentialLateralMissDoesNotChaseStationPlane) {
  ASSERT_TRUE(start(1.0));
  pose(0.8, 0.25);
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kError);
  EXPECT_NE(detail().find("lateral miss outside XY disk"), std::string::npos);
}

TEST_F(ReverseParkingControllerTest, BoundedFinalApproachRetainsTimeout) {
  ASSERT_TRUE(start(1.0));
  pose(0.75, 0.128);
  expireReverseTimeout();
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kError);
  EXPECT_EQ(detail(), "reverse parking timeout");
}

TEST_F(ReverseParkingControllerTest, OvershootingAxisDoesNotCountAsArrival) {
  ASSERT_TRUE(start(1.0));
  pose(1.35, 0.0);
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kError);
}

TEST_F(ReverseParkingControllerTest, ActualGoalInsideToleranceStillCompletes) {
  ASSERT_TRUE(start(1.0));
  pose(0.8, 0.0);
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kParked);
}

TEST_F(ReverseParkingControllerTest, AlreadyInsideGoalCompletesWithoutMinimumReverseTravel) {
  for (const double station_x : {-0.25, -0.238, 0.0, 0.238, 0.25}) {
    cancel();
    ASSERT_TRUE(start(station_x));
    tick();
    EXPECT_EQ(phase(), ReverseParkingPhase::kParked) << station_x;
    EXPECT_NE(detail().find("station XY goal reached"), std::string::npos);
    EXPECT_DOUBLE_EQ(reversed(), 0.0);
    EXPECT_DOUBLE_EQ(tolerance(), 0.25);
  }
}

TEST_F(ReverseParkingControllerTest, ObservedReparkedPosePublishesOnlyZeroWithoutReversing) {
  // Measured preparation-02 pose, unchanged mapped station and tolerance.
  std::vector<avg_msgs::msg::AvgTwist> commands;
  const auto subscription = subscribeCommands(commands);
  ASSERT_TRUE(startObservedPose());
  EXPECT_NEAR(initialDistance(), .23802207538567263, .00000001);
  EXPECT_LT(axisDistance(), 0.0);
  tick();
  const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(25);
  do {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (std::chrono::steady_clock::now() < until);
  ASSERT_FALSE(commands.empty());
  for (const auto & command : commands) {
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
  }
  EXPECT_EQ(phase(), ReverseParkingPhase::kParked);
  EXPECT_DOUBLE_EQ(reversed(), 0.0);
}

TEST_F(ReverseParkingControllerTest, AlreadyCloseDoesNotWidenGoalOrBypassWrongAxis) {
  ASSERT_TRUE(start(.250001));
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kReverseApproach);
  cancel();
  ASSERT_TRUE(start(-.250001));
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kError);
  EXPECT_NE(detail().find("station is behind"), std::string::npos);
}

TEST_F(ReverseParkingControllerTest, AlreadyAtGoalKeepsFreshnessAndTimeoutGuards) {
  ASSERT_TRUE(start(.238));
  stale();
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kError);
  cancel();
  ASSERT_TRUE(start(.238));
  expireReverseTimeout();
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kError);
  EXPECT_EQ(detail(), "reverse parking timeout");
}

TEST_F(ReverseParkingControllerTest, AlreadyAtGoalStillRequiresConfiguredCharging) {
  requireCharging();
  ASSERT_TRUE(start(-.238));
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kWaitForCharging);
}

TEST_F(ReverseParkingControllerTest, ActualGoalAtTravelLimitCanComplete) {
  ASSERT_TRUE(start(5.0));
  pose(5.0, 0.0);
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kParked);
}

TEST_F(ReverseParkingControllerTest, ChargingStillStopsImmediatelyWithoutCoordinateRequirement) {
  ASSERT_TRUE(start(3.75));
  stale();
  charging();
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kParked);
  EXPECT_EQ(detail(), "charging detected");
}

TEST_F(ReverseParkingControllerTest, StaleAndNonfiniteLocalizationCannotComplete) {
  ASSERT_TRUE(start(1.0));
  pose(0.8, 0.0);
  stale();
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kError);
  cancel();
  ASSERT_TRUE(start(1.0));
  pose(std::numeric_limits<double>::quiet_NaN(), 0.0);
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kError);
}

TEST_F(ReverseParkingControllerTest, MissDoesNotBecomeChargingWaitAndCancelRemainsIdempotent) {
  requireCharging();
  ASSERT_TRUE(start(3.75));
  pose(5.0, 0.0);
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kError);
  cancel();
  cancel();
  EXPECT_EQ(phase(), ReverseParkingPhase::kIdle);
  ASSERT_TRUE(start(1.0));
  pose(0.8, 0.0);
  tick();
  EXPECT_EQ(phase(), ReverseParkingPhase::kWaitForCharging);
}
