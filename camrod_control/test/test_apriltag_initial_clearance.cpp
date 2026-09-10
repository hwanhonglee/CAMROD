// Run the actual controller on an isolated DDS domain and remapped test topics.
#include <gtest/gtest.h>
#include <thread>
#include <optional>
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
  void TearDown() override {command_subscription_.reset(); node_.reset();}
  void fresh(double x = 0., double lateral = -.166, double heading = 0., double range = .741,
    std::optional<double> optical_depth = std::nullopt)
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
    node_->tag_camera_optical_depth_m_ = optical_depth.value_or(range * .9);
    node_->tag_observed_base_x_m_ = -.61933 - node_->tag_camera_optical_depth_m_;
    node_->last_tag_time_ = node_->now();
  }
  // HH_260911 - Geometry tests acknowledge speech without disabling its gate.
  bool start(bool confirm_voice = true)
  {
    std::string detail;
    const bool accepted = node_->startParking("isolated_unit_test", detail);
    if (accepted && confirm_voice) voiceReady();
    return accepted;
  }
  void voiceReady()
  {
    node_->docking_started_voice_gate_.onVoiceState(true, "docking.started", node_->now().seconds());
    node_->docking_started_voice_gate_.onVoiceState(false, "", node_->now().seconds());
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
  void tagBaseX(double x) {node_->tag_observed_base_x_m_ = x;}
  void captureCommands()
  {
    command_subscription_ = node_->create_subscription<avg_msgs::msg::AvgTwist>(
      node_->cmd_pub_->get_topic_name(), 100,
      [this](avg_msgs::msg::AvgTwist::ConstSharedPtr message) {commands_.push_back(*message);});
  }
  void tickAndCollect()
  {
    tick();
    const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(15);
    do {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < until);
  }
  bool stopLatched() {return node_->translation_stop_reason_ == "tag_range";}
  double stopThreshold() {return node_->translation_stop_tag_distance_m_;}
  rclcpp::Subscription<avg_msgs::msg::AvgTwist>::SharedPtr command_subscription_;
  std::vector<avg_msgs::msg::AvgTwist> commands_;
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
  // HH_260911 - Confirm reacquisition speech before checking reverse geometry.
  voiceReady(); fresh(0., -.166, 0., .741); tick();
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

TEST_F(AprilTagInitialClearanceControllerTest, BelowCalibratedOpticalDepthCannotAuthorizeClearance)
{
  ASSERT_TRUE(start()); fresh(0., -.144, .033, .31727, .199999); tick();
  EXPECT_EQ(state(), State::ERROR); EXPECT_FALSE(active());
}

TEST_F(AprilTagInitialClearanceControllerTest, ObservedClosePoseFirstTranslationIsStraightForwardOnly)
{
  captureCommands();
  ASSERT_TRUE(start()); fresh(0., -.142, .027, .31727, .292994); tickAndCollect();
  ASSERT_EQ(state(), State::RETRY_FORWARD_EXIT);
  EXPECT_TRUE(active()); EXPECT_EQ(retries(), 0);
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < until) {
    fresh(0., -.142, .027, .31727, .292994); tickAndCollect();
    if (std::any_of(commands_.begin(), commands_.end(), [](const auto & cmd) {
        return cmd.linear.x > 0.;})) {break;}
  }
  ASSERT_FALSE(commands_.empty());
  const auto translation = std::find_if(commands_.begin(), commands_.end(), [](const auto & cmd) {
    return cmd.linear.x != 0.;});
  ASSERT_NE(translation, commands_.end());
  EXPECT_DOUBLE_EQ(translation->linear.x, .2);
  for (const auto & cmd : commands_) {
    EXPECT_GE(cmd.linear.x, 0.);
    EXPECT_DOUBLE_EQ(cmd.angular.z, 0.);
  }
  EXPECT_DOUBLE_EQ(stopThreshold(), .40);
  EXPECT_FALSE(stopLatched());
}

TEST_F(AprilTagInitialClearanceControllerTest, AlreadyAlignedPoint377StillLatchesReverseStop)
{
  captureCommands();
  ASSERT_TRUE(start()); fresh(0., -.02, 0., .377); tickAndCollect();
  EXPECT_EQ(state(), State::TAG_GUIDED_REVERSE); EXPECT_FALSE(active());
  fresh(0., -.02, 0., .377); tickAndCollect();
  EXPECT_EQ(state(), State::FINAL_YAW_ALIGNMENT); EXPECT_TRUE(stopLatched());
  EXPECT_DOUBLE_EQ(stopThreshold(), .40);
  ASSERT_FALSE(commands_.empty());
  for (const auto & cmd : commands_) {EXPECT_DOUBLE_EQ(cmd.linear.x, 0.);}
}

TEST_F(AprilTagInitialClearanceControllerTest, InvalidOrNonRearTagCannotStartForwardClearance)
{
  for (const double depth : {-.3, 0., std::numeric_limits<double>::quiet_NaN()}) {
    cancel(); fresh(); ASSERT_TRUE(start());
    fresh(0., -.142, .027, .31727, depth); tick();
    EXPECT_EQ(state(), State::ERROR); EXPECT_FALSE(active());
  }
  cancel(); fresh(); ASSERT_TRUE(start()); fresh(); tagBaseX(0.1); tick();
  EXPECT_EQ(state(), State::ERROR); EXPECT_FALSE(active());
}

TEST_F(AprilTagInitialClearanceControllerTest, GeometryLossDuringInitialForwardStopsImmediately)
{
  captureCommands();
  ASSERT_TRUE(start()); fresh(0., -.142, .027, .31727, .292994); tickAndCollect();
  beginForward();
  fresh(0., -.142, .027, .31727, .199999); tickAndCollect();
  EXPECT_EQ(state(), State::ERROR); EXPECT_FALSE(active());
  ASSERT_FALSE(commands_.empty());
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.);
  EXPECT_DOUBLE_EQ(commands_.back().angular.z, 0.);
}

// HH_260911 - Prove clearance cannot start before the real announcement gate releases.
TEST_F(AprilTagInitialClearanceControllerTest, AnnouncementMustFinishBeforeClearance)
{
  ASSERT_TRUE(start(false)); fresh(); tick();
  EXPECT_EQ(state(), State::WAITING_FOR_TAG);
  EXPECT_FALSE(active());
  voiceReady(); fresh(); tick();
  EXPECT_EQ(state(), State::RETRY_FORWARD_EXIT);
  EXPECT_TRUE(active());
}

// HH_260911 - Late playback completion cannot restart a cancelled docking attempt.
TEST_F(AprilTagInitialClearanceControllerTest, CancelDuringAnnouncementStaysIdle)
{
  ASSERT_TRUE(start(false)); fresh(); tick();
  cancel(); voiceReady(); fresh(); tick();
  EXPECT_EQ(state(), State::IDLE);
  EXPECT_FALSE(active());
}
