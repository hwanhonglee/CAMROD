// HH_261024 - Reset is a stop-only administrative action, not permission to
// drive through an obstacle or to erase a physical/planning emergency stop.
#include <limits>
#include <thread>
#include "gtest/gtest.h"
#define CAMROD_CONTROL_RECOVERY_RESET_TEST
#include "../src/cmd_vel_safety_gate_node.cpp"

namespace camrod_control {
namespace {
RecoveryHoldResetContext stoppedContext() {
  RecoveryHoldResetContext c;
  c.now_sec = 10.0;
  c.platform_received = c.motion_received = true;
  c.platform_received_sec = c.motion_received_sec = c.motion_stamp_sec = 9.9;
  return c;
}

TEST(RecoveryHoldResetPolicy, RequiresRevokedOwnersAndUnmodifiedHardInterlocks) {
  auto c = stoppedContext();
  EXPECT_TRUE(recoveryHoldResetBlockReason(c).empty());
  c.manual_engaged = true;
  EXPECT_EQ(recoveryHoldResetBlockReason(c), "disengage_required");
  c.manual_engaged = false; c.mission_engaged = true;
  EXPECT_EQ(recoveryHoldResetBlockReason(c), "disengage_required");
  c.mission_engaged = false; c.maneuver_active = true;
  EXPECT_EQ(recoveryHoldResetBlockReason(c), "maneuver_cancel_pending");
  c.maneuver_active = false; c.estop_active = true;
  EXPECT_EQ(recoveryHoldResetBlockReason(c), "estop_active");
  c.estop_active = false; c.dr_timeout_active = true;
  EXPECT_EQ(recoveryHoldResetBlockReason(c), "dr_timeout_active");
  c.dr_timeout_active = false; c.vehicle_state = 1;
  EXPECT_EQ(recoveryHoldResetBlockReason(c), "platform_fault_active");
  c.vehicle_state = 0; c.platform_error_code = 2;
  EXPECT_EQ(recoveryHoldResetBlockReason(c), "platform_fault_active");
}

TEST(RecoveryHoldResetPolicy, MissingStaleFutureAndNonfiniteFeedbackFailClosed) {
  auto c = stoppedContext();
  c.platform_received = false;
  EXPECT_EQ(recoveryHoldResetBlockReason(c), "platform_status_missing_or_stale");
  c = stoppedContext(); c.motion_received = false;
  EXPECT_EQ(recoveryHoldResetBlockReason(c), "motion_feedback_missing_or_stale");
  for (const auto stamp : {0.0, 9.49, 10.01,
                           std::numeric_limits<double>::quiet_NaN()}) {
    c = stoppedContext(); c.motion_stamp_sec = stamp;
    EXPECT_EQ(recoveryHoldResetBlockReason(c), "motion_feedback_missing_or_stale");
    c = stoppedContext(); c.motion_received_sec = stamp;
    EXPECT_EQ(recoveryHoldResetBlockReason(c), "motion_feedback_missing_or_stale");
    c = stoppedContext(); c.platform_received_sec = stamp;
    EXPECT_EQ(recoveryHoldResetBlockReason(c), "platform_status_missing_or_stale");
  }
  for (int axis = 0; axis < 3; ++axis) {
    c = stoppedContext();
    double *component[] = {&c.vx_mps, &c.vy_mps, &c.yaw_rate_rps};
    *component[axis] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(recoveryHoldResetBlockReason(c), "invalid_motion_feedback");
  }
}

TEST(RecoveryHoldResetPolicy, CrabAndYawAreMeasuredNotOnlyForwardSpeed) {
  auto c = stoppedContext(); c.vy_mps = 0.031;
  EXPECT_EQ(recoveryHoldResetBlockReason(c), "robot_not_stationary");
  c.vx_mps = c.vy_mps = 0.025;
  EXPECT_EQ(recoveryHoldResetBlockReason(c), "robot_not_stationary");
  c = stoppedContext(); c.yaw_rate_rps = -0.031;
  EXPECT_EQ(recoveryHoldResetBlockReason(c), "robot_not_stationary");
  c = stoppedContext(); c.vx_mps = 0.03; c.yaw_rate_rps = -0.03;
  EXPECT_TRUE(recoveryHoldResetBlockReason(c).empty());
}
}  // namespace

class CmdVelSafetyGateRecoveryResetTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::InitOptions options; options.set_domain_id(214);
    rclcpp::init(0, nullptr, options);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    node_ = std::make_shared<CmdVelSafetyGateNode>();
    node_->state_timer_->cancel(); node_->command_timeout_timer_->cancel();
    node_->gate_policy_.setManualEngage(false);
    node_->gate_policy_.setMissionEngage(false);
    node_->gate_policy_.setPlatformDriveEnable(false);
    freshPlatform();
  }
  void TearDown() override { node_.reset(); }
  void freshPlatform(const double vx = 0, const double vy = 0,
                     const double yaw = 0) {
    auto p = std::make_shared<avg_msgs::msg::AvgPlatformStatus>();
    p->control_mode = 1;
    p->velocity.header.stamp = node_->now();
    p->velocity.twist.linear.x = vx;
    p->velocity.twist.linear.y = vy;
    p->velocity.twist.angular.z = yaw;
    node_->onPlatformStatus(p);
  }
  std_srvs::srv::Trigger::Response reset() {
    std_srvs::srv::Trigger::Response response;
    node_->resetRecoveryHold(response);
    return response;
  }
  void seedRouteHold() {
    MotionCostStopDecision decision;
    decision.blocked = decision.lanelet_violation = true;
    decision.reason = "test_lanelet_boundary";
    avg_msgs::msg::AvgTwist cmd; cmd.linear.x = 0.2;
    ASSERT_TRUE(node_->route_safety_recovery_.observeViolation(
        decision, cmd, node_->nowSec()));
    node_->last_input_command_sec_ = node_->nowSec();
    node_->route_safety_triggered_by_navigation_ = true;
  }
  bool holdActive() const { return node_->route_safety_recovery_.active(); }
  bool stillDisarmed() const {
    return !node_->gate_policy_.planningEngaged() &&
           !node_->gate_policy_.platformDriveEnabled() &&
           !node_->last_input_command_sec_.has_value() &&
           !node_->route_safety_triggered_by_navigation_;
  }
  void engage() { node_->gate_policy_.setMissionEngage(true); }
  void hardStop() { node_->gate_policy_.setEstopSource("physical_test", true); }
  void planningStop() {
    node_->gate_policy_.setEstopSource("/planning/state_machine/estop", true);
  }
  bool hardStopActive() const { return node_->gate_policy_.estopActive(); }
  void expireMotion() { node_->reset_platform_motion_stamp_sec_ = 0.0; }
  void seedObstacleHold(const bool stale_after_trigger = false) {
    MotionCostStopConfig config;
    config.lanelet_enabled = false;
    config.require_dynamic_source = false;
    config.use_speed_dependent_lookahead = false;
    config.dynamic_front_use_local_path = false;
    config.body_near_enabled = false;
    config.latch_enabled = true;
    node_->motion_cost_stop_config_ = config;
    node_->motion_cost_stop_.setConfig(config);
    avg_msgs::msg::AvgOccupancyGrid grid;
    grid.header.frame_id = "map"; grid.info.resolution = 0.1;
    grid.info.width = grid.info.height = 100;
    grid.info.origin.position.x = grid.info.origin.position.y = -5.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(10000, 100);
    node_->motion_cost_stop_.setMergedGrid(grid, node_->nowSec());
    node_->motion_cost_stop_.setPose({0,0,0,"map","test",node_->nowSec()});
    avg_msgs::msg::AvgTwist cmd; cmd.linear.x = 0.2;
    ASSERT_TRUE(node_->motion_cost_stop_.evaluate(cmd, node_->nowSec()).blocked);
    ASSERT_TRUE(node_->motion_cost_stop_.latched());
    if (stale_after_trigger)
      node_->motion_cost_stop_.setMergedGrid(grid, node_->nowSec() - 2.0);
  }
  bool obstacleLatched() const { return node_->motion_cost_stop_.latched(); }
  void prepareDockingGate() {
    auto config = node_->motion_cost_stop_config_;
    config.lanelet_enabled = false;
    config.dynamic_front_use_local_path = false;
    config.radar_grid_freshness_stop_enabled = true;
    node_->motion_cost_stop_.setConfig(config);
    node_->motion_cost_stop_.setPose({0,0,0,"map","test",node_->nowSec()});
    node_->enable_resume_after_hold_voice_gate_ = false;
    node_->enable_route_heading_alignment_ = false;
    node_->gate_policy_.setMissionEngage(true);
    node_->gate_policy_.setPlatformDriveEnable(true);
    reader_ = std::make_shared<rclcpp::Node>("docking_gate_test_reader");
    output_subscription_ = reader_->create_subscription<avg_msgs::msg::AvgTwist>(
        node_->output_topic_, 10, [this](avg_msgs::msg::AvgTwist::SharedPtr msg) {
          output_ = *msg;
        });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (node_->command_publisher_->get_subscription_count() == 0 &&
           std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(reader_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_GT(node_->command_publisher_->get_subscription_count(), 0U);
  }
  void dockingGrid(const double x = 1.8, const double y = 0.0,
                   const bool stale_radar = false) {
    avg_msgs::msg::AvgOccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.header.stamp = node_->now();
    grid.info.resolution = 0.1;
    grid.info.width = grid.info.height = 100;
    grid.info.origin.position.x = grid.info.origin.position.y = -5.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(10000, 0);
    grid.data[50 * 100 + 68] = 95;  // Front path stays occupied.
    grid.data[static_cast<int>((y + 5.0) / 0.1) * 100 +
              static_cast<int>((x + 5.0) / 0.1)] = 95;
    node_->motion_cost_stop_.setMergedGrid(grid, node_->nowSec());
    if (stale_radar) grid.header.stamp.sec -= 2;
    node_->motion_cost_stop_.setSourceGrid("radar", grid,
        node_->nowSec() - (stale_radar ? 2.0 : 0.0));
    freshPlatform();
  }
  void dockingPhase(const std::string & phase) {
    node_->parking_phase_ = phase;
    node_->updateManeuverPhases();
  }
  void resetDockingCost() {
    node_->motion_cost_stop_ = MotionCostStop(node_->motion_cost_stop_.config());
    node_->motion_cost_stop_.setPose({0,0,0,"map","test",node_->nowSec()});
  }
  avg_msgs::msg::AvgTwist sendDockingCommand(const double x, const double yaw = 0) {
    // Ownership transitions also publish zero. Consume those older outputs
    // before checking the response to this specific command.
    EXPECT_TRUE(node_->command_publisher_->wait_for_all_acked(std::chrono::seconds(2)));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(reader_);
    executor.spin_all(std::chrono::milliseconds(20));
    output_.reset();
    avg_msgs::msg::AvgTwist cmd;
    cmd.linear.x = x; cmd.angular.z = yaw;
    node_->onCommand(cmd, false);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!output_ && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    EXPECT_TRUE(output_.has_value());
    return output_.value_or(avg_msgs::msg::AvgTwist{});
  }
  std::shared_ptr<rclcpp::Node> reader_;
  rclcpp::Subscription<avg_msgs::msg::AvgTwist>::SharedPtr output_subscription_;
  std::optional<avg_msgs::msg::AvgTwist> output_;
  std::shared_ptr<CmdVelSafetyGateNode> node_;
};

TEST_F(CmdVelSafetyGateRecoveryResetTest, DockingReversePublishesThroughGateAfterFrontHold) {
  prepareDockingGate();
  dockingGrid();
  EXPECT_DOUBLE_EQ(sendDockingCommand(0.2).linear.x, 0.0);
  ASSERT_TRUE(obstacleLatched());
  dockingPhase("TAG_GUIDED_REVERSE");
  dockingGrid();
  EXPECT_LT(sendDockingCommand(-0.1, 0.02).linear.x, 0.0);
  EXPECT_FALSE(obstacleLatched());
  dockingGrid();
  EXPECT_DOUBLE_EQ(sendDockingCommand(0.1).linear.x, 0.0);
  EXPECT_TRUE(obstacleLatched());
}

TEST_F(CmdVelSafetyGateRecoveryResetTest, DockingReverseKeepsSideRearAndStaleRadarStops) {
  prepareDockingGate();
  dockingPhase("TAG_GUIDED_REVERSE");
  for (const auto & hit : std::vector<std::pair<double, double>>{
         {0.0, 0.8}, {0.0, -0.8}, {-0.5, 0.0}}) {
    resetDockingCost();
    dockingPhase("TAG_GUIDED_REVERSE");
    dockingGrid(hit.first, hit.second);
    EXPECT_DOUBLE_EQ(sendDockingCommand(-0.1).linear.x, 0.0);
    EXPECT_TRUE(obstacleLatched());
  }
  // A stale radar cannot release an existing front hold.
  resetDockingCost();
  dockingGrid();
  sendDockingCommand(0.2);
  ASSERT_TRUE(obstacleLatched());
  dockingGrid(1.8, 0.0, true);
  dockingPhase("TAG_GUIDED_REVERSE");
  EXPECT_DOUBLE_EQ(sendDockingCommand(-0.1).linear.x, 0.0);
  EXPECT_TRUE(obstacleLatched());
}

TEST_F(CmdVelSafetyGateRecoveryResetTest, DockingReverseCannotOverrideEmergencyStop) {
  prepareDockingGate();
  dockingGrid();
  sendDockingCommand(0.2);
  ASSERT_TRUE(obstacleLatched());
  dockingPhase("TAG_GUIDED_REVERSE");
  dockingGrid(); hardStop();
  EXPECT_DOUBLE_EQ(sendDockingCommand(-0.1).linear.x, 0.0);
  EXPECT_TRUE(hardStopActive());
}

TEST_F(CmdVelSafetyGateRecoveryResetTest, FinalYawStillStopsForFrontObstacle) {
  prepareDockingGate();
  dockingPhase("TAG_GUIDED_REVERSE");
  dockingGrid(1.1, 0.0);
  EXPECT_LT(sendDockingCommand(-0.1).linear.x, 0.0);
  dockingPhase("FINAL_YAW_ALIGNMENT");
  dockingGrid(1.1, 0.0);
  const auto output = sendDockingCommand(0.0, 0.1);
  EXPECT_DOUBLE_EQ(output.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(output.angular.z, 0.0);
  EXPECT_TRUE(obstacleLatched());
}

TEST_F(CmdVelSafetyGateRecoveryResetTest, ClearsSoftwareEpisodeWithoutEnablingMotion) {
  seedRouteHold();
  const auto response = reset();
  EXPECT_TRUE(response.success) << response.message;
  EXPECT_EQ(response.message, "recovery_hold_reset_motion_disarmed");
  EXPECT_FALSE(holdActive()); EXPECT_TRUE(stillDisarmed());
  EXPECT_TRUE(reset().success);  // Repeat is harmless, never a toggle.
}

TEST_F(CmdVelSafetyGateRecoveryResetTest, EngagedAndHardStopStatesCannotReset) {
  seedRouteHold(); engage();
  EXPECT_EQ(reset().message, "disengage_required"); EXPECT_TRUE(holdActive());
}

TEST_F(CmdVelSafetyGateRecoveryResetTest, NeverClearsPhysicalEmergencyStop) {
  seedRouteHold(); hardStop();
  EXPECT_EQ(reset().message, "estop_active");
  EXPECT_TRUE(hardStopActive()); EXPECT_TRUE(holdActive());
}

TEST_F(CmdVelSafetyGateRecoveryResetTest, NeverClearsPlanningEmergencyStop) {
  seedRouteHold(); planningStop();
  EXPECT_EQ(reset().message, "estop_active");
  EXPECT_TRUE(hardStopActive()); EXPECT_TRUE(holdActive());
}

TEST_F(CmdVelSafetyGateRecoveryResetTest, MovingAndMissingMeasurementCannotReset) {
  seedRouteHold(); freshPlatform(0,0.1);
  EXPECT_EQ(reset().message, "robot_not_stationary"); EXPECT_TRUE(holdActive());
  freshPlatform(); expireMotion();
  EXPECT_EQ(reset().message, "motion_feedback_missing_or_stale");
  EXPECT_TRUE(holdActive());
}

TEST_F(CmdVelSafetyGateRecoveryResetTest, ActiveObstacleRemainsLatched) {
  seedRouteHold(); seedObstacleHold();
  const auto response = reset();
  EXPECT_FALSE(response.success);
  EXPECT_EQ(response.message.rfind("active_or_stale_obstacle_hold:",0),0U);
  EXPECT_TRUE(obstacleLatched()); EXPECT_TRUE(holdActive());
}

TEST_F(CmdVelSafetyGateRecoveryResetTest, StaleObstacleEvidenceCannotBeAcknowledgedClear) {
  seedRouteHold(); seedObstacleHold(true);
  const auto response = reset();
  EXPECT_FALSE(response.success);
  EXPECT_EQ(response.message.rfind("active_or_stale_obstacle_hold:",0),0U);
  EXPECT_NE(response.message.find("trigger_evidence_not_fresh"),std::string::npos);
  EXPECT_TRUE(obstacleLatched()); EXPECT_TRUE(holdActive());
}

}  // namespace camrod_control
