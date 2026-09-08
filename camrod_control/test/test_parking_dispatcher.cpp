// Unit tests of the production dispatcher, not simulated mission acceptance.
// Domain 189, localhost-only and a cancelled timer prevent field interaction.
#include "gtest/gtest.h"
#define CAMROD_CONTROL_PARKING_DISPATCHER_TEST
#include "../src/parking_dispatcher_node.cpp"

class ParkingDispatcherTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::InitOptions options;
    options.set_domain_id(189);
    rclcpp::init(0, nullptr, options);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    node_ = std::make_shared<ParkingDispatcherNode>();
    node_->timer_->cancel();
    platform(0.8F);
  }
  void TearDown() override { node_.reset(); }
  void platform(const float fraction, const bool available = true,
                const bool charging = false) {
    avg_msgs::msg::AvgPlatformStatus message;
    message.battery_percentage = fraction;
    message.battery_state_available = available;
    message.control_mode = 1U;
    message.is_charging = charging;
    node_->platform_ = message;
    node_->platform_time_ = node_->now();
  }
  void start(const bool force = false) {
    ASSERT_TRUE(node_->request(avg_msgs::msg::MotionOperation::START,
        force ? "robot_ui:force_docking" : "return").first);
  }
  void acknowledge(const bool apply_start = true) {
    const auto gen = generation();
    node_->ownership_.acknowledgeCancel(ParkingMethod::kReverse, gen);
    node_->ownership_.acknowledgeCancel(ParkingMethod::kAprilTag, gen);
    node_->start_sent_ = true;
    node_->start_time_ = node_->now() - rclcpp::Duration::from_seconds(0.01);
    if (apply_start) { node_->acceptStart(selected() == ParkingMethod::kReverse ? 0 : 1, gen); }
  }
  void ackStart() { node_->acceptStart(selected() == ParkingMethod::kReverse ? 0 : 1, generation()); }
  void status(const std::string & phase, const ParkingMethod method = ParkingMethod::kReverse,
              const bool old = false) {
    auto message = camrod_control::makeModuleState(*node_, "parking", 0, "actual callback", phase);
    if (old) { message.stamp = node_->start_time_ - rclcpp::Duration::from_seconds(1.0); }
    node_->receiveStatus(method == ParkingMethod::kReverse ? 0 : 1, method, message);
  }
  avg_msgs::msg::AvgServiceState output(const uint8_t state,
                                      const ParkingMethod method = ParkingMethod::kReverse) {
    avg_msgs::msg::AvgServiceState message;
    message.state = state;
    message.description = "private actual event";
    return node_->serviceOutput(method, message);
  }
  void tick() { node_->tick(); }
  void cancel() { node_->request(avg_msgs::msg::MotionOperation::CANCEL, "test"); }
  void unsafe() { node_->platform_->estop = true; }
  void stalePlatform() { node_->platform_time_ -= rclcpp::Duration::from_seconds(3.0); }
  void staleStatus() { node_->last_controller_status_time_ -= rclcpp::Duration::from_seconds(3.0); }
  bool owns(const ParkingMethod method) const { return node_->ownership_.owns(method); }
  bool busy() const { return node_->ownership_.busy(); }
  bool freshParked() const { return node_->freshReverseParked(); }
  bool forced() const { return node_->forced_; }
  bool failed() const { return node_->failed_; }
  ParkingMethod selected() const { return node_->ownership_.selected(); }
  uint64_t generation() const { return node_->ownership_.generation(); }
  std::string phase() const { return node_->publicPhase(); }
  void staleAcknowledgements(const uint64_t generation) {
    node_->ownership_.acknowledgeCancel(ParkingMethod::kReverse, generation);
    node_->ownership_.acknowledgeCancel(ParkingMethod::kAprilTag, generation);
    node_->acceptStart(1, generation);
  }
  std::shared_ptr<ParkingDispatcherNode> node_;
};

TEST_F(ParkingDispatcherTest, EveryInitialSocAndUnknownStartsReverse) {
  for (const float soc : {0.0F, 0.249F, 0.25F, 0.349F, 0.35F, 0.8F, 1.0F}) {
    platform(soc); start(); EXPECT_EQ(selected(), ParkingMethod::kReverse); cancel();
  }
  platform(0.8F, false); start(); EXPECT_EQ(selected(), ParkingMethod::kReverse);
}

TEST_F(ParkingDispatcherTest, InitialForceDockAlsoReversesBeforeApril) {
  start(true); EXPECT_EQ(selected(), ParkingMethod::kReverse); acknowledge();
  status("PARKED"); EXPECT_TRUE(freshParked()); tick();
  EXPECT_EQ(selected(), ParkingMethod::kAprilTag); EXPECT_FALSE(owns(ParkingMethod::kAprilTag));
}

TEST_F(ParkingDispatcherTest, LowSocNeedsActualReverseParkAndNewOwnershipAcknowledgements) {
  platform(0.34F); start(); acknowledge();
  const auto reverse_generation = generation();
  tick(); EXPECT_EQ(selected(), ParkingMethod::kReverse);
  status("PARKED"); EXPECT_EQ(phase(), "REVERSE_PARKED_WAITING_FOR_DOCK");
  tick(); EXPECT_EQ(selected(), ParkingMethod::kAprilTag); EXPECT_GT(generation(), reverse_generation);
  EXPECT_FALSE(owns(ParkingMethod::kReverse)); EXPECT_FALSE(owns(ParkingMethod::kAprilTag));
  staleAcknowledgements(reverse_generation); EXPECT_FALSE(owns(ParkingMethod::kAprilTag));
  acknowledge(); EXPECT_TRUE(owns(ParkingMethod::kAprilTag));
}

TEST_F(ParkingDispatcherTest, ReverseServiceBeforeStatusCannotFinishLowBatteryMission) {
  platform(0.34F); start(); acknowledge();
  for (const auto terminal : {avg_msgs::msg::AvgServiceState::DROP_ZONE_WAIT,
       avg_msgs::msg::AvgServiceState::WAITING_FOR_CHARGING, avg_msgs::msg::AvgServiceState::CHARGING}) {
    EXPECT_EQ(output(terminal).state, avg_msgs::msg::AvgServiceState::DROP_ZONE_PARKING);
  }
  EXPECT_EQ(phase(), "REVERSE_APPROACH"); EXPECT_FALSE(freshParked());
}

TEST_F(ParkingDispatcherTest, ThirtyFiveAndAboveRemainNonChargingParkedUntilExplicitDock) {
  for (const float soc : {0.35F, 0.8F}) {
    platform(soc); start(); acknowledge(); status("PARKED"); tick();
    EXPECT_EQ(selected(), ParkingMethod::kReverse); EXPECT_EQ(phase(), "PARKED");
    EXPECT_EQ(output(avg_msgs::msg::AvgServiceState::DROP_ZONE_WAIT).state,
              avg_msgs::msg::AvgServiceState::DROP_ZONE_WAIT);
    start(true); EXPECT_EQ(selected(), ParkingMethod::kAprilTag); cancel();
  }
}

TEST_F(ParkingDispatcherTest, OrdinaryReturnDoesNotReusePreviousParkedProof) {
  start(); acknowledge(); status("PARKED"); start();
  EXPECT_EQ(selected(), ParkingMethod::kReverse); EXPECT_FALSE(freshParked());
}

TEST_F(ParkingDispatcherTest, StaleReverseProofCannotSkipReverseForExplicitDock) {
  start(); acknowledge(); status("PARKED"); staleStatus(); start(true);
  EXPECT_EQ(selected(), ParkingMethod::kReverse); EXPECT_FALSE(freshParked());
}

TEST_F(ParkingDispatcherTest, PreStartOldParkedHeartbeatIsRejected) {
  platform(0.34F); start(); acknowledge(); status("PARKED", ParkingMethod::kReverse, true);
  EXPECT_FALSE(freshParked()); tick(); EXPECT_EQ(selected(), ParkingMethod::kReverse);
}

TEST_F(ParkingDispatcherTest, ImmediateCompletionBeforeStartAckIsAppliedOnlyAfterAck) {
  platform(0.34F); start(); acknowledge(false); status("PARKED");
  EXPECT_FALSE(freshParked()); EXPECT_TRUE(busy()); ackStart();
  EXPECT_TRUE(freshParked()); EXPECT_FALSE(busy()); tick();
  EXPECT_EQ(selected(), ParkingMethod::kAprilTag);
}

TEST_F(ParkingDispatcherTest, BusyExplicitDockLatchesButDoesNotChangeMovingOwner) {
  start(); acknowledge(); const auto gen = generation(); start(true);
  EXPECT_TRUE(forced()); EXPECT_EQ(generation(), gen); EXPECT_EQ(selected(), ParkingMethod::kReverse);
  status("PARKED"); tick(); EXPECT_EQ(selected(), ParkingMethod::kAprilTag);
}

TEST_F(ParkingDispatcherTest, CancelInvalidatesProofForcedIntentAndLateOutputs) {
  start(true); acknowledge(); status("PARKED"); const auto gen = generation(); cancel();
  status("PARKED"); staleAcknowledgements(gen); tick();
  EXPECT_FALSE(forced()); EXPECT_FALSE(freshParked()); EXPECT_EQ(selected(), ParkingMethod::kNone);
}

TEST_F(ParkingDispatcherTest, ReverseErrorNeverTriggersDock) {
  platform(0.34F); start(); acknowledge(); status("ERROR"); tick();
  EXPECT_EQ(selected(), ParkingMethod::kReverse); EXPECT_EQ(phase(), "ERROR"); EXPECT_FALSE(freshParked());
}

TEST_F(ParkingDispatcherTest, FreshChargingContactStopsAutomaticDockHandoff) {
  platform(0.34F, true, true); start(); acknowledge(); status("PARKED"); tick();
  EXPECT_EQ(selected(), ParkingMethod::kReverse); EXPECT_EQ(phase(), "PARKED");
  EXPECT_EQ(output(avg_msgs::msg::AvgServiceState::CHARGING).state, avg_msgs::msg::AvgServiceState::CHARGING);
}

TEST_F(ParkingDispatcherTest, UnsafeOrStalePlatformDoesNotPermitSecondStageMotion) {
  platform(0.34F); start(); acknowledge(); status("PARKED"); unsafe(); tick();
  EXPECT_EQ(selected(), ParkingMethod::kReverse);
  platform(0.34F); stalePlatform(); tick(); EXPECT_EQ(selected(), ParkingMethod::kReverse);
}

TEST_F(ParkingDispatcherTest, UnknownSocRequiresDockButNeverSkipsReverse) {
  platform(0.8F, false); start(); acknowledge(); status("PARKED");
  EXPECT_EQ(phase(), "REVERSE_PARKED_WAITING_FOR_DOCK"); tick(); EXPECT_EQ(selected(), ParkingMethod::kAprilTag);
}

TEST_F(ParkingDispatcherTest, StaleSelectedStatusStillFailsInsteadOfHandoff) {
  platform(0.34F); start(); acknowledge(); status("PARKED"); staleStatus(); tick();
  EXPECT_TRUE(failed()); EXPECT_EQ(phase(), "ERROR"); EXPECT_FALSE(owns(ParkingMethod::kReverse));
}

TEST_F(ParkingDispatcherTest, NonOwnerAndLateReverseCannotOverwriteAprilPhase) {
  platform(0.34F); start(); acknowledge(); status("PARKED"); tick(); acknowledge();
  status("PARKED"); EXPECT_EQ(phase(), "WAITING_FOR_TAG");
  status("PARKED", ParkingMethod::kAprilTag); EXPECT_EQ(phase(), "PARKED");
}
