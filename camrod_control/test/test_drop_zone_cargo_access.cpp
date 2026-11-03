// HH_261102 - Exercise the actual ROS controller with synthetic fresh feedback, never a hardware command topic.
#include <algorithm>
#include <thread>

#include <gtest/gtest.h>

#define CAMROD_CONTROL_DROP_ZONE_CARGO_TEST
#include "../src/drop_zone_maneuver_controller_node.cpp"

class DropZoneCargoAccessTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::InitOptions options;
    options.set_domain_id(219);
    rclcpp::init(0, nullptr, options);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args",
      "--remap", "/control/cmd_vel_raw:=/test/cargo_access/cmd_vel",
      "--remap", "/localization/pose:=/test/cargo_access/pose",
      "--remap", "/platform/status:=/test/cargo_access/platform",
      "--remap", "/platform/robot/planning_boundary:=/test/cargo_access/body",
      "--remap", "/planning/lanelet_pose:=/test/cargo_access/lanelet",
      "--remap", "/planning/drop_zone_goal_raw:=/test/cargo_access/goal",
      "--remap", "/planning/goal_pose_snapped:=/test/cargo_access/snapped",
      "--remap", "/planning/state_machine/state:=/test/cargo_access/planning",
      "--remap", "/control/drop_zone_maneuver_controller/operation:=/test/cargo_access/operation",
      "--remap", "/control/drop_zone_maneuver_controller/request_operation:=/test/cargo_access/request",
      "--remap", "/control/drop_zone_maneuver_controller/status:=/test/cargo_access/status",
      "--remap", "/control/drop_zone_maneuver_controller/exit_path_ros:=/test/cargo_access/path",
      "--remap", "/control/drop_zone_maneuver_controller/parking_approach_path_ros:=/test/cargo_access/approach",
      "--remap", "/control/drop_zone/exit_complete:=/test/cargo_access/complete",
      "--remap", "/parking/operation:=/test/cargo_access/parking",
      "--remap", "/service/state:=/test/cargo_access/service",
      "--remap", "/system/diagnostics:=/test/cargo_access/diagnostics"});
    options.parameter_overrides({rclcpp::Parameter("drop_zones_yaml", CAMROD_CARGO_MAP_YAML),
                                rclcpp::Parameter("exit_speed_mps", 0.20)});
    node_ = std::make_shared<DropZoneManeuverControllerNode>(options);
    node_->control_timer_->cancel();
    executor_.add_node(node_);
    pose_pub_ = node_->create_publisher<avg_msgs::msg::AvgPoseStamped>("/test/cargo_access/pose", 10);
    lanelet_pub_ = node_->create_publisher<avg_msgs::msg::AvgPoseStamped>("/test/cargo_access/lanelet", 10);
    body_pub_ = node_->create_publisher<avg_msgs::msg::AvgPolygonStamped>(
        "/test/cargo_access/body", rclcpp::QoS(1).transient_local().reliable());
    commands_sub_ = node_->create_subscription<avg_msgs::msg::AvgTwist>("/test/cargo_access/cmd_vel", 100,
        [this](avg_msgs::msg::AvgTwist::ConstSharedPtr message) { commands_.push_back(*message); });
    parking_sub_ = node_->create_subscription<avg_msgs::msg::MotionOperation>("/test/cargo_access/parking", 100,
        [this](avg_msgs::msg::MotionOperation::ConstSharedPtr message) { parking_.push_back(*message); });
    service_sub_ = node_->create_subscription<avg_msgs::msg::AvgServiceState>("/test/cargo_access/service", 100,
        [this](avg_msgs::msg::AvgServiceState::ConstSharedPtr message) { states_.push_back(*message); });
    status_sub_ = node_->create_subscription<avg_msgs::msg::ModuleState>("/test/cargo_access/status", 100,
        [this](avg_msgs::msg::ModuleState::ConstSharedPtr message) { statuses_.push_back(*message); });
    complete_sub_ = node_->create_subscription<avg_msgs::msg::AvgBool>("/test/cargo_access/complete", 100,
        [this](avg_msgs::msg::AvgBool::ConstSharedPtr message) { completions_.push_back(message->data); });
    pump(30);
    avg_msgs::msg::AvgPolygonStamped body;
    body.header.frame_id = "robot_center_link";
    for (const auto &point : std::vector<std::pair<double, double>>{
        {0.80837, 0.63505}, {0.80837, -0.63495}, {-0.78323, -0.63495}, {-0.78323, 0.63505}}) {
      avg_msgs::msg::AvgPoint32 p;
      p.x = point.first;
      p.y = point.second;
      body.polygon.points.push_back(p);
    }
    body_pub_->publish(body);
    pump(10);
    yaw_ = node_->configuredParkingBodyYaw();
    x_ = node_->station_pose_.x_m - 0.8 * std::cos(yaw_);
    y_ = node_->station_pose_.y_m - 0.8 * std::sin(yaw_);
    pose(x_, y_, yaw_);
  }
  void TearDown() override {
    executor_.remove_node(node_);
    complete_sub_.reset(); status_sub_.reset(); service_sub_.reset(); parking_sub_.reset(); commands_sub_.reset();
    body_pub_.reset(); lanelet_pub_.reset(); pose_pub_.reset();
    node_.reset();
  }
  void pump(const int cycles = 5) {
    for (int index = 0; index < cycles; ++index) {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  avg_msgs::msg::AvgPoseStamped sample(const double x, const double y, const double yaw,
                                      const std::string &frame = "map") {
    avg_msgs::msg::AvgPoseStamped pose;
    pose.header.frame_id = frame;
    pose.header.stamp = node_->now();
    pose.pose.position.x = x; pose.pose.position.y = y;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    return pose;
  }
  void pose(const double x, const double y, const double yaw, const std::string &frame = "map") {
    pose_pub_->publish(sample(x, y, yaw, frame));
    pump();
  }
  bool leave() {
    return node_->applyOperation(avg_msgs::msg::MotionOperation::EXIT,
                                "http:charging_access:leave:site_departure").first;
  }
  void tick() { node_->onTimer(); pump(); }
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<DropZoneManeuverControllerNode> node_;
  rclcpp::Publisher<avg_msgs::msg::AvgPoseStamped>::SharedPtr pose_pub_, lanelet_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgPolygonStamped>::SharedPtr body_pub_;
  rclcpp::Subscription<avg_msgs::msg::AvgTwist>::SharedPtr commands_sub_;
  rclcpp::Subscription<avg_msgs::msg::MotionOperation>::SharedPtr parking_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgServiceState>::SharedPtr service_sub_;
  rclcpp::Subscription<avg_msgs::msg::ModuleState>::SharedPtr status_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr complete_sub_;
  std::vector<avg_msgs::msg::AvgTwist> commands_;
  std::vector<avg_msgs::msg::MotionOperation> parking_;
  std::vector<avg_msgs::msg::AvgServiceState> states_;
  std::vector<avg_msgs::msg::ModuleState> statuses_;
  std::vector<bool> completions_;
  double x_{0.0}, y_{0.0}, yaw_{0.0};
};

TEST_F(DropZoneCargoAccessTest, ActualRosTrajectoryStaysInsideAuthoredZoneAndStopsWithoutTurningOrDocking) {
  ASSERT_TRUE(node_->cargo_body_radius_.has_value());
  ASSERT_TRUE(leave());
  EXPECT_EQ(node_->exit_target_.source, camrod_control::DropZoneExitTargetSource::kStationPose);
  int moving_steps = 0;
  for (int step = 0; step < 100 && !node_->cargo_access_ready_; ++step) {
    pose(x_, y_, yaw_);
    tick();
    ASSERT_FALSE(commands_.empty());
    const auto command = commands_.back();
    EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
    EXPECT_GE(command.linear.x, 0.0);
    if (command.linear.x > 0.0) { ++moving_steps; }
    x_ += (std::cos(yaw_) * command.linear.x - std::sin(yaw_) * command.linear.y) * 0.2;
    y_ += (std::sin(yaw_) * command.linear.x + std::cos(yaw_) * command.linear.y) * 0.2;
    EXPECT_TRUE(camrod_control::cargoAccessSweepInsideZone(node_->cargo_zone_corners_, x_, y_,
        node_->station_pose_.x_m, node_->station_pose_.y_m, *node_->cargo_body_radius_));
  }
  ASSERT_TRUE(node_->cargo_access_ready_);
  EXPECT_GT(moving_steps, 0);
  EXPECT_EQ(node_->phase_, DropZoneManeuverPhase::kIdle);
  EXPECT_FALSE(node_->road_handoff_ready_);
  EXPECT_TRUE(parking_.empty());
  ASSERT_FALSE(states_.empty());
  EXPECT_EQ(states_.back().state, avg_msgs::msg::AvgServiceState::DROP_ZONE_WAIT);
  EXPECT_EQ(states_.back().state_name, "CARGO_ACCESS_READY");
  ASSERT_FALSE(statuses_.empty());
  EXPECT_EQ(statuses_.back().operating_state, "CARGO_ACCESS_READY");
  ASSERT_FALSE(completions_.empty());
  EXPECT_TRUE(completions_.back());
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  RecordProperty("proof_kind", "isolated_actual_ros_controller_with_synthetic_pose_feedback");
  RecordProperty("station_x_m", fixed(node_->station_pose_.x_m, 6));
  RecordProperty("station_y_m", fixed(node_->station_pose_.y_m, 6));
  RecordProperty("terminal_position_error_m", fixed(std::hypot(
      x_ - node_->station_pose_.x_m, y_ - node_->station_pose_.y_m), 6));
  RecordProperty("body_disk_radius_m", fixed(*node_->cargo_body_radius_, 6));
  RecordProperty("moving_feedback_steps", moving_steps);
  RecordProperty("max_angular_command_radps", "0.0");
  RecordProperty("automatic_parking_starts", static_cast<int>(parking_.size()));
  RecordProperty("cargo_terminal", "CARGO_ACCESS_READY");

  avg_msgs::msg::PlanningState reached;
  reached.state = avg_msgs::msg::PlanningState::GOAL_REACHED;
  reached.scenario_id = avg_msgs::msg::PlanningScenario::RETURN_TO_DROP_ZONE;
  reached.active_mission_key = "drop_zone";
  node_->onPlanningState(reached); tick();
  EXPECT_TRUE(node_->cargo_access_ready_);
  EXPECT_TRUE(parking_.empty());

  // A later ordinary mission still selects the road target and retains its road-yaw alignment phase.
  ASSERT_TRUE(node_->applyOperation(avg_msgs::msg::MotionOperation::CANCEL,
                                   "http_ui_destination:site_departure_preempt_alignment").first);
  node_->onPlanningState(reached); tick();
  EXPECT_EQ(node_->phase_, DropZoneManeuverPhase::kIdle);
  EXPECT_TRUE(parking_.empty());
  pose(x_, y_, yaw_);
  const auto road = sample(x_ + std::cos(yaw_), y_ + std::sin(yaw_), yaw_ + M_PI_2);
  lanelet_pub_->publish(road); pump();
  ASSERT_TRUE(node_->applyOperation(avg_msgs::msg::MotionOperation::EXIT, "http_ui_destination:site_departure").first);
  EXPECT_FALSE(node_->cargo_access_ready_);
  EXPECT_FALSE(node_->cargo_access_active_);
  EXPECT_EQ(node_->exit_target_.source, camrod_control::DropZoneExitTargetSource::kLaneletPose);
  pose(road.pose.position.x, road.pose.position.y, yaw_); tick();
  EXPECT_EQ(node_->phase_, DropZoneManeuverPhase::kAlignExitYaw);
}

TEST_F(DropZoneCargoAccessTest, MissingCanonicalBoundaryRejectsImmediately) {
  node_->cargo_body_radius_.reset();
  EXPECT_FALSE(leave());
  EXPECT_EQ(node_->phase_, DropZoneManeuverPhase::kError);
  EXPECT_FALSE(node_->cargo_access_ready_);
}

TEST_F(DropZoneCargoAccessTest, RawGoalCannotReplaceTheAuthoredStation) {
  node_->station_pose_.x_m += 0.5;
  EXPECT_FALSE(leave());
  EXPECT_EQ(node_->phase_, DropZoneManeuverPhase::kError);
}

TEST_F(DropZoneCargoAccessTest, DegenerateCanonicalBoundaryAndLargeLateralCorrectionAreRejected) {
  pose(x_ - 0.25 * std::sin(yaw_), y_ + 0.25 * std::cos(yaw_), yaw_);
  EXPECT_FALSE(leave());
  pose(x_, y_, yaw_);
  avg_msgs::msg::AvgPolygonStamped body;
  body.header.frame_id = "robot_center_link";
  avg_msgs::msg::AvgPoint32 identical;
  identical.x = 1.0; identical.y = 1.0;
  body.polygon.points = {identical, identical, identical};
  body_pub_->publish(body); pump();
  EXPECT_FALSE(node_->cargo_body_radius_.has_value());
  EXPECT_FALSE(leave());
}

TEST_F(DropZoneCargoAccessTest, YawDriftStopsInsteadOfCommandingARotation) {
  ASSERT_TRUE(leave());
  pose(x_, y_, yaw_ + 0.2); tick();
  EXPECT_EQ(node_->phase_, DropZoneManeuverPhase::kError);
  ASSERT_FALSE(commands_.empty());
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(commands_.back().angular.z, 0.0);
}

TEST_F(DropZoneCargoAccessTest, WrongMapFrameAndBodyLeavingZoneStopRatherThanGuessing) {
  pose(x_, y_, yaw_, "odom");
  EXPECT_FALSE(leave());
  pose(x_, y_, yaw_);
  ASSERT_TRUE(leave());
  pose(x_ + 4.0, y_, yaw_); tick();
  EXPECT_EQ(node_->phase_, DropZoneManeuverPhase::kError);
  ASSERT_FALSE(commands_.empty());
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(commands_.back().angular.z, 0.0);
}

TEST_F(DropZoneCargoAccessTest, CancelClearsCargoTerminalAndCannotResumeOnHeartbeat) {
  ASSERT_TRUE(leave());
  EXPECT_TRUE(node_->applyOperation(avg_msgs::msg::MotionOperation::CANCEL, "test_stop").first);
  tick();
  EXPECT_FALSE(node_->cargo_access_active_);
  EXPECT_FALSE(node_->cargo_access_ready_);
  EXPECT_EQ(node_->phase_, DropZoneManeuverPhase::kIdle);
}
