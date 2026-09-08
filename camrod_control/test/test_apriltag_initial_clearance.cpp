// Run the actual controller on an isolated DDS domain and remapped test topics.
#include <gtest/gtest.h>
#define CAMROD_APRILTAG_INITIAL_CLEARANCE_TEST
#include "../src/apriltag_parking_controller_node.cpp"

class AprilTagInitialClearanceControllerTest : public ::testing::Test
{
protected:
  using State = AprilTagParkingControllerNode::State;
  static void SetUpTestSuite()
  {
    rclcpp::InitOptions options;
    options.set_domain_id(189);
    rclcpp::init(0, nullptr, options);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/test/initial_clearance",
      "-r", "/control/cmd_vel_raw:=/test/initial_clearance/cmd",
      "-r", "/localization/odometry:=/test/initial_clearance/odom",
      "-r", "/platform/status:=/test/initial_clearance/platform",
      "-r", "/parking/operation:=/test/initial_clearance/operation",
      "-r", "/service/state:=/test/initial_clearance/service_state",
      "-r", "/system/diagnostics:=/test/initial_clearance/diagnostics",
      "-r", "/perception/apriltag_parking_detector/tag_pose:=/test/initial_clearance/tag",
      "-r", "/parking/apriltag_parking_controller/status:=/test/initial_clearance/status"});
    node_ = std::make_shared<AprilTagParkingControllerNode>(options);
    node_->control_timer_->cancel();
    node_->initial_clearance_config_ = {true, 1.20, .25, .10};
    node_->enable_bounded_lateral_retry_ = true;
    node_->max_retries_ = 2;
    node_->retry_forward_distance_m_ = .8;
    node_->retry_forward_speed_mps_ = .2;
    node_->retry_forward_timeout_s_ = 30.;
    node_->final_lateral_tolerance_m_ = .03;
    node_->retry_maximum_lateral_error_m_ = .15;
    node_->viz_enabled_ = false;
    fresh();
  }
  void TearDown() override {node_.reset();}
  void fresh(double x = 0., double lateral = -.166, double heading = 0., double range = .741)
  {
    node_->odom_valid_ = true;
    node_->last_odometry_time_ = node_->now();
    node_->vehicle_odometry_x_m_ = x;
    node_->vehicle_odometry_y_m_ = lateral;
    node_->vehicle_odometry_yaw_rad_ = -heading;
    node_->tag_odometry_x_m_ = -1.341;
    node_->tag_odometry_y_m_ = 0.;
    node_->parking_axis_odometry_yaw_rad_ = 0.;
    node_->axis_valid_ = true;
    node_->tag_camera_distance_valid_ = true;
    node_->tag_camera_distance_m_ = range;
    node_->last_tag_time_ = node_->now();
  }
  bool start()
  {
    std::string detail;
    return node_->startParking("isolated_unit_test", detail);
  }
  void tick() {node_->controlLoop();}
  State state() {return node_->state_;}
  bool active() {return node_->initial_clearance_active_;}
  bool evaluated() {return node_->initial_clearance_evaluated_;}
  int retries() {return node_->retries_;}
  void cancel() {node_->cancelParking("isolated_unit_test");}
  void beginForward()
  {
    ASSERT_TRUE(node_->retry_progress_.begin(
      node_->vehicle_odometry_x_m_, node_->vehicle_odometry_y_m_, 0.));
    node_->retry_forward_started_ = true;
    node_->retry_forward_start_time_ = node_->now();
  }
  void staleTag() {node_->last_tag_time_ = node_->now() - rclcpp::Duration::from_seconds(1.);}
  void staleOdom() {node_->last_odometry_time_ = node_->now() - rclcpp::Duration::from_seconds(1.);}
  void charge() {node_->charging_detected_ = true;}
  void expire() {node_->retry_forward_start_time_ = node_->now() - rclcpp::Duration::from_seconds(31.);}
  void disabled() {node_->initial_clearance_config_.enabled = false;}
  std::shared_ptr<AprilTagParkingControllerNode> node_;
};

TEST_F(AprilTagInitialClearanceControllerTest, ObservedPoseSelectsClearanceWithoutUsingRetryBudget)
{
  ASSERT_TRUE(start()); fresh(); tick();
  EXPECT_EQ(state(), State::RETRY_FORWARD_EXIT);
  EXPECT_TRUE(active()); EXPECT_TRUE(evaluated()); EXPECT_EQ(retries(), 0);
}

TEST_F(AprilTagInitialClearanceControllerTest, DisabledProfileKeepsOriginalReversePhase)
{
  disabled(); ASSERT_TRUE(start()); fresh(); tick();
  EXPECT_EQ(state(), State::TAG_GUIDED_REVERSE); EXPECT_FALSE(active());
}

TEST_F(AprilTagInitialClearanceControllerTest, OutOfReverseParkingEnvelopeFailsBeforeReverse)
{
  ASSERT_TRUE(start()); fresh(0., -.250001); tick();
  EXPECT_EQ(state(), State::ERROR); EXPECT_FALSE(active()); EXPECT_EQ(retries(), 0);
}

TEST_F(AprilTagInitialClearanceControllerTest, ShortStartNeedsNearAlignedHeading)
{
  ASSERT_TRUE(start()); fresh(0., -.166, .100001); tick();
  EXPECT_EQ(state(), State::ERROR);
}

TEST_F(AprilTagInitialClearanceControllerTest, StaleWaitingInputsDoNotBeginClearance)
{
  ASSERT_TRUE(start()); fresh(); staleTag(); tick();
  EXPECT_EQ(state(), State::WAITING_FOR_TAG); EXPECT_FALSE(evaluated());
  fresh(); staleOdom(); tick();
  EXPECT_EQ(state(), State::WAITING_FOR_TAG); EXPECT_FALSE(evaluated());
}

TEST_F(AprilTagInitialClearanceControllerTest, LostTagDuringForwardFailsInsteadOfBlindMotion)
{
  ASSERT_TRUE(start()); fresh(); tick(); beginForward(); staleTag(); tick();
  EXPECT_EQ(state(), State::ERROR); EXPECT_FALSE(active());
}

TEST_F(AprilTagInitialClearanceControllerTest, LostOdometryDuringForwardFails)
{
  ASSERT_TRUE(start()); fresh(); tick(); beginForward(); staleOdom(); tick();
  EXPECT_EQ(state(), State::ERROR); EXPECT_FALSE(active());
}

TEST_F(AprilTagInitialClearanceControllerTest, ChargingPreemptsClearanceImmediately)
{
  ASSERT_TRUE(start()); fresh(); tick(); beginForward(); charge(); tick();
  EXPECT_EQ(state(), State::PARKED); EXPECT_FALSE(active());
}

TEST_F(AprilTagInitialClearanceControllerTest, CancelAndNewRequestResetOnlyInitialAttempt)
{
  ASSERT_TRUE(start()); fresh(); tick(); beginForward(); cancel();
  EXPECT_EQ(state(), State::IDLE); EXPECT_FALSE(active()); EXPECT_FALSE(evaluated());
  ASSERT_TRUE(start()); fresh(); tick();
  EXPECT_EQ(state(), State::RETRY_FORWARD_EXIT); EXPECT_TRUE(active()); EXPECT_EQ(retries(), 0);
}

TEST_F(AprilTagInitialClearanceControllerTest, ForwardCompletionReacquiresWithoutRepeatingClearance)
{
  ASSERT_TRUE(start()); fresh(); tick(); beginForward();
  for (int i = 1; i <= 16; ++i) {fresh(i * .05, -.166, 0., .741 + i * .05); tick();}
  EXPECT_EQ(state(), State::WAITING_FOR_TAG);
  EXPECT_FALSE(active()); EXPECT_TRUE(evaluated()); EXPECT_EQ(retries(), 0);
  fresh(0., -.166, 0., .741); tick();
  EXPECT_EQ(state(), State::TAG_GUIDED_REVERSE);
  EXPECT_FALSE(active()); EXPECT_EQ(retries(), 0);
}

TEST_F(AprilTagInitialClearanceControllerTest, ExistingOdometryStepAndTimeoutBoundsRemainHard)
{
  ASSERT_TRUE(start()); fresh(); tick(); beginForward(); fresh(.101); tick();
  EXPECT_EQ(state(), State::ERROR);
  cancel(); fresh(); ASSERT_TRUE(start()); fresh(); tick(); beginForward(); expire(); tick();
  EXPECT_EQ(state(), State::ERROR);
}

TEST_F(AprilTagInitialClearanceControllerTest, PointFourStopIsNotAForwardClearanceAuthorization)
{
  ASSERT_TRUE(start()); fresh(0., -.166, 0., .4); tick();
  EXPECT_EQ(state(), State::ERROR); EXPECT_FALSE(active());
}
