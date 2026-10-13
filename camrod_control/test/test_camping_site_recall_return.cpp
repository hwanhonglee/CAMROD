// Exercise the real campsite controller without spinning a robot launch. The
// friend fixture injects fresh localization/mission inputs and advances phase
// clocks deterministically; velocity safety remains owned by the final gate.
#include "gtest/gtest.h"
#include <cstdio>
#include <fstream>
#include <unistd.h>

#define CAMROD_CONTROL_CAMPING_SITE_TEST
#include "../src/camping_site_maneuver_controller_node.cpp"

class CampingSiteManeuverControllerTest : public ::testing::Test {
protected:
  using Phase = CampingSiteManeuverPhase;

  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    node_ = std::make_shared<CampingSiteManeuverControllerNode>();
    node_->control_timer_->cancel();
    node_->cancel_nav2_on_site_phase_ = false;
    node_->yaw_alignment_settling_.setConfig({4.0, 0.0, 0.0});
    node_->return_lanelet_handoff_hold_s_ = 0.0;
  }

  void TearDown() override { node_.reset(); }

  void setPose(const double x, const double y, const double yaw) {
    node_->last_pose_ = node_->makePose("map", x, y, 0.0, yaw * 180.0 / M_PI);
    node_->last_pose_time_ = node_->now();
  }

  void setAlongSite(const double lateral, const bool turned = false) {
    const auto target = camrod_control::lateralTargetFromAnchor(
        10.0, 20.0, entry_yaw_, direction_, lateral);
    setPose(target.first, target.second, entry_yaw_ + (turned ? M_PI : 0.0));
  }

  void startRecall(const int site, const bool complete_arrival = true) {
    node_->applyOperation(avg_msgs::msg::MotionOperation::CANCEL, "test_setup");
    node_->camping_site_goals_.clear();
    node_->camping_site_service_modes_.clear();
    node_->camping_site_forward_return_profiles_.clear();
    node_->last_auto_key_.clear();
    direction_ = site % 2 == 0 ? -1.0 : 1.0;
    key_ = "camping_site_" + std::to_string(site);
    const auto target = camrod_control::lateralTargetFromAnchor(
        10.0, 20.0, entry_yaw_, direction_, 3.0);
    node_->camping_site_goals_[key_] =
        node_->makePose("map", target.first, target.second, 0.0, 0.0);
    node_->camping_site_service_modes_[key_] = site <= 10
        ? CampsiteServiceMode::kTurnaround : CampsiteServiceMode::kRoadsideStop;
    node_->site_goal_.reset();
    node_->route_goal_ = node_->makePose(
        "map", 10.0, 20.0, 0.0, entry_yaw_ * 180.0 / M_PI);
    node_->route_goal_time_ = node_->now();
    node_->lanelet_pose_ = node_->route_goal_;
    node_->lanelet_pose_time_ = node_->now();
    setAlongSite(0.0);
    avg_msgs::msg::PlanningState mission;
    mission.state = avg_msgs::msg::PlanningState::GOAL_REACHED;
    mission.scenario_id = avg_msgs::msg::PlanningScenario::RECALL_TO_SITE;
    mission.active_mission_key = key_;
    mission.active_goal_source = "ui";
    node_->onPlanningState(mission);
    ASSERT_EQ(phase(), Phase::kAlignEntryYaw);
    if (!complete_arrival) {
      return;
    }
    tick();
    ASSERT_EQ(phase(), Phase::kCrabIn);
    EXPECT_DOUBLE_EQ(node_->crab_offset_m_, 0.30);
    setAlongSite(0.30);
    tick();
    ASSERT_EQ(phase(), Phase::kUnloadWait);
  }

  Phase phase() const { return node_->phase_; }
  void tick() { node_->onTimer(); }
  void elapsed(const double seconds) {
    node_->phase_start_time_ = node_->now() - rclcpp::Duration::from_seconds(seconds);
    // The voice timeout uses absolute time, independently of phase_start_time_.
    // Advance its clock by the same simulated interval without disabling it.
    node_->recall_clearance_voice_gate_.tick(node_->now().seconds() + seconds);
  }
  void completeClearanceVoice() {
    node_->recall_clearance_voice_gate_.onVoiceState(
        true, node_->recall_clearance_voice_key_, node_->now().seconds());
    node_->recall_clearance_voice_gate_.onVoiceState(
        false, "", node_->now().seconds());
  }
  void setCrabReturnTimeout(const double seconds) {
    node_->crab_return_timeout_s_ = seconds;
  }
  std::pair<bool, std::string> returnRequest() {
    return node_->requestReturn("robot_ui:loading_complete");
  }
  std::pair<bool, std::string> finalReturnRequest(
      const std::string &source = "robot_ui:recall_final_return") {
    return node_->requestReturn(source);
  }
  std::pair<bool, std::string> urgentReturnRequest(
      const std::string &source = "battery_urgent_return:platform_status") {
    return node_->requestReturn(source);
  }
  std::string finalReturnSource() const {
    return node_->returnRequestSource("done_roadside_forward_retry");
  }
  void beginDelivery(const int site) {
    startRecall(site, false);
    cancel();
    node_->last_auto_key_.clear();
    startDeliveryAfterRecall();
    ASSERT_EQ(phase(), Phase::kAlignEntryYaw);
    tick();
    ASSERT_EQ(phase(), Phase::kCrabIn);
  }
  void setPartiallyRotatedPose(const double lateral) {
    const auto target = camrod_control::lateralTargetFromAnchor(
        10.0, 20.0, entry_yaw_, direction_, lateral);
    setPose(target.first, target.second, entry_yaw_ + 0.6);
  }
  void expirePose() {
    node_->last_pose_time_ = node_->now() - rclcpp::Duration::from_seconds(3.0);
  }
  void enableAutoReturn() { node_->auto_return_after_unload_wait_ = true; }
  void setOccupied(const bool value) {
    node_->enable_campsite_occupancy_guard_ = true;
    if (value) {
      node_->occupied_mission_keys_.insert(key_);
    } else {
      node_->occupied_mission_keys_.erase(key_);
    }
  }
  bool occupancyBlocks() const { return node_->occupiedSiteBlocksCurrentMission(key_); }
  bool returnLatched() const { return node_->return_requested_; }
  bool returnPublished() const { return node_->return_published_; }
  bool turnaroundActive() const { return node_->recall_turnaround_return_active_; }
  bool internalPhase() const { return node_->isSiteInternalPhase(); }
  auto phaseStart() const { return node_->phase_start_time_; }
  double offset() const { return node_->crab_offset_m_; }
  void expectOriginalAnchor() const {
    EXPECT_DOUBLE_EQ(node_->return_anchor_x_, 10.0);
    EXPECT_DOUBLE_EQ(node_->return_anchor_y_, 20.0);
  }
  void completeClearance() {
    completeClearanceVoice();
    elapsed(8.1);
    setAlongSite(0.30);
    tick();
    ASSERT_EQ(phase(), Phase::kAlignEntryYaw);
    tick();
    ASSERT_EQ(phase(), Phase::kCrabIn);
  }
  void completeTurnaround() {
    setAlongSite(3.0);
    tick();
    ASSERT_EQ(phase(), Phase::kRotate180);
    setAlongSite(3.0, true);
    tick();
    ASSERT_EQ(phase(), Phase::kRecallReturnWait);
    EXPECT_FALSE(returnLatched());
    EXPECT_FALSE(returnPublished());
  }
  void confirmFinalReturn() {
    ASSERT_TRUE(finalReturnRequest().first);
    ASSERT_EQ(phase(), Phase::kAlignRetraceYaw);
    tick();
    ASSERT_EQ(phase(), Phase::kCrabOut);
  }
  void finishFromDifferentLiveAnchor(const bool turned) {
    // A 1 m longitudinal difference must never trigger historical-XY reverse
    // alignment once lateral exit reaches the live lanelet projection.
    setPose(10.0 + std::cos(entry_yaw_), 20.0 + std::sin(entry_yaw_),
            entry_yaw_ + (turned ? M_PI : 0.0));
    node_->lanelet_pose_ = node_->last_pose_;
    node_->lanelet_pose_time_ = node_->now();
    tick();
  }
  void cancel() {
    ASSERT_TRUE(node_->applyOperation(
        avg_msgs::msg::MotionOperation::CANCEL, "test_cancel").first);
  }
  void mutatePlanningMission() {
    avg_msgs::msg::PlanningState message;
    message.state = avg_msgs::msg::PlanningState::GOAL_REACHED;
    message.scenario_id = avg_msgs::msg::PlanningScenario::DELIVERY_TO_SITE;
    message.active_mission_key = "camping_site_10";
    node_->onPlanningState(message);
    EXPECT_EQ(node_->site_goal_key_, key_);
  }
  void removeConfiguredSite() { node_->camping_site_goals_.clear(); }
  void makeConfiguredSiteInvalid() {
    node_->camping_site_goals_[key_].pose.position.x =
        std::numeric_limits<double>::quiet_NaN();
  }
  void setReverseDeliveryMode() { node_->site_entry_mode_ = "reverse"; }
  bool adoptRecallAtRoadside() {
    node_->phase_ = Phase::kIdle;
    return node_->adoptWaitReturnState(key_, "test_recall_restart");
  }
  void startDeliveryAfterRecall() {
    node_->onSiteGoal(node_->camping_site_goals_.at(key_));
    setAlongSite(0.0);
    node_->route_goal_time_ = node_->now();
    avg_msgs::msg::PlanningState mission;
    mission.state = avg_msgs::msg::PlanningState::GOAL_REACHED;
    mission.scenario_id = avg_msgs::msg::PlanningScenario::DELIVERY_TO_SITE;
    mission.active_mission_key = key_;
    mission.active_goal_source = "ui";
    node_->onPlanningState(mission);
    EXPECT_FALSE(node_->active_recall_wait_mission_);
  }

  // HH_261012 - Exercise the optional policy through the real phase machine;
  // changing the return route must not skip turnaround or loading approval.
  void completeDeliveryService(const int site = 5) {
    node_->rotate_entry_max_position_error_m_ = 0.0;
    beginDelivery(site);
    setAlongSite(3.0);
    tick();
    ASSERT_EQ(phase(), Phase::kRotate180);
    setAlongSite(3.0, true);
    tick();
    ASSERT_EQ(phase(), Phase::kUnloadWait);
    elapsed(60.0);
    tick();
    ASSERT_EQ(phase(), Phase::kWaitReturn);
  }
  void setReturnPolicy(const std::string &policy = "forward_loop") {
    node_->camping_site_return_route_policies_[key_] = policy;
    node_->rotate_180_timeout_s_ = 90.0;
  }
  void setReturnTurnPositionBound(const double value) {
    node_->rotate_entry_max_position_error_m_ = value;
  }
  void expireForwardTurnWallClock() {
    node_->phase_start_wall_time_ = std::chrono::steady_clock::now() -
        std::chrono::seconds(120);
  }
  double targetYaw() const { return node_->target_yaw_; }
  double angularCommand() const { return node_->last_angular_command_radps_; }
  double crabCommand() const { return node_->last_crab_command_mps_; }
  bool forwardExitActive() const { return node_->site_forward_return_active_; }
  bool forwardCenteringActive() const {
    return node_->site_forward_return_centering_active_;
  }
  void useImmediateCenteringSettle() {
    node_->return_steering_settle_s_ = 0.0;
    node_->rotate_entry_centering_sequencer_.setConfig(
        camrod_control::CampsiteCrabReturnConfig{0.02, 0.10, 0.0});
  }
  void setSiteCenterBodyError(const double longitudinal, const double lateral) {
    const double yaw = entry_yaw_ + M_PI;
    setPose(node_->entry_target_x_ - std::cos(yaw) * longitudinal +
                std::sin(yaw) * lateral,
            node_->entry_target_y_ - std::sin(yaw) * longitudinal -
                std::cos(yaw) * lateral, yaw);
  }
  double preciseCenteringTolerance() const {
    return node_->siteForwardReturnCenteringToleranceM();
  }
  // HH_261012 - Exercise catalog profiles through the real departure admission,
  // never by changing the immutable pivot guard or either steady deadline.
  double recenterTrigger() const {
    return node_->siteForwardReturnRecenterTriggerM();
  }
  void setReturnProfile(const double target = 0.015, const double trigger = 0.035) {
    node_->camping_site_forward_return_profiles_[key_] =
        {target, trigger, std::nullopt, std::nullopt};
  }
  bool hasReturnProfile(const std::string &key) const {
    return node_->camping_site_forward_return_profiles_.count(key) != 0;
  }
  std::pair<double, double> returnProfile(const std::string &key) const {
    const auto &profile = node_->camping_site_forward_return_profiles_.at(key);
    return {profile.centering_tolerance_m, profile.recenter_trigger_m};
  }
  // HH_261012 - Read actual sequencer output to test the selected dwell, not
  // merely the catalog getter. No robot or executor clock is advanced here.
  void setReturnProfileDwell(const double seconds) {
    node_->camping_site_forward_return_profiles_[key_].steering_settle_s = seconds;
  }
  std::optional<double> configuredProfileDwell(const std::string &key) const {
    return node_->camping_site_forward_return_profiles_.at(key).steering_settle_s;
  }
  double selectedProfileDwell() const {
    return node_->siteForwardReturnSteeringSettleS();
  }
  // HH_261012 - Explicit recovery budgets exercise the unchanged real guards
  // and wall clocks; advancing the sequencer stage does not renew those clocks.
  void setReturnProfileLimit(const int limit) {
    node_->camping_site_forward_return_profiles_[key_].recentering_limit = limit;
  }
  int selectedProfileLimit() const {
    return node_->siteForwardReturnRecenteringLimit();
  }
  void beginLimitProfilePivot(const int limit = 20) {
    completeDeliveryService(10);
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    setReturnProfile();
    setReturnProfileDwell(0.4);
    setReturnProfileLimit(limit);
    ASSERT_TRUE(returnRequest().first);
    tick();
    ASSERT_NE(angularCommand(), 0.0);
  }
  void completeLimitProfileRecentering() {
    setSiteCenterBodyError(0.0, 0.0);
    ASSERT_EQ(centeringCommandAfterDwell(0.4).stage, CenteringStage::kLongitudinal);
    tick();
    ASSERT_FALSE(forwardCenteringActive());
    EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
    tick();
  }
  void setNativeSteeringDwell(const double seconds) {
    node_->return_steering_settle_s_ = seconds;
  }
  // HH_261012 - Use an explicit result type for earlier inline fixture callers.
  camrod_control::CampsiteCrabReturnCommand centeringCommandAfterDwell(
      const double seconds) {
    node_->rotate_entry_centering_sequencer_.reset(100.0);
    node_->rotate_entry_centering_sequencer_.update(0.03, 0.0, 100.0, 0.12, 4.0);
    return node_->rotate_entry_centering_sequencer_.update(
        0.03, 0.0, 100.0 + seconds, 0.12, 4.0);
  }
  void beginDwellProfileCentering() {
    completeDeliveryService(10);
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    setReturnProfile();
    setReturnProfileDwell(0.4);
    setSiteCenterBodyError(0.03, 0.0);
    ASSERT_TRUE(returnRequest().first);
    ASSERT_TRUE(forwardCenteringActive());
  }
  void selectReturnAlias(const std::string &key) { node_->site_goal_key_ = key; }
  void beginProfiledForwardReturn(const bool center_first = false) {
    completeDeliveryService(9);
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    setReturnProfile();
    useImmediateCenteringSettle();
    if (center_first) setSiteCenterBodyError(0.03, 0.0);
    ASSERT_TRUE(returnRequest().first);
    tick();
    if (center_first) {
      tick();
      ASSERT_EQ(centeringStage(), CenteringStage::kLongitudinal);
    } else {
      ASSERT_NE(angularCommand(), 0.0);
    }
  }
  double centeringLongitudinalCommand() {
    return node_->rotate_entry_centering_sequencer_.update(
        0.015, 0.0, node_->now().seconds(), 0.12, 1.0).linear_x_mps;
  }
  void expireCenteringWallClock() {
    node_->rotate_entry_centering_start_wall_time_ =
        std::chrono::steady_clock::now() - std::chrono::seconds(16);
  }
  auto centeringWallStart() const {
    return node_->rotate_entry_centering_start_wall_time_;
  }
  auto phaseWallStart() const { return node_->phase_start_wall_time_; }
  int forwardRecenteringCount() const {
    return node_->site_forward_return_recentering_count_;
  }
  std::string phaseDetail() const { return node_->phase_detail_; }
  void setCenteringAdmissionBound(const double value) {
    node_->rotate_entry_centering_max_initial_error_m_ = value;
  }
  void beginForwardPivot() {
    completeDeliveryService(4);
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    useImmediateCenteringSettle();
    ASSERT_TRUE(returnRequest().first);
    tick();
    ASSERT_NE(angularCommand(), 0.0);
    ASSERT_FALSE(forwardCenteringActive());
  }
  void completeForwardRecentering() {
    setSiteCenterBodyError(0.0, 0.0);
    for (int i = 0; i < 4 && forwardCenteringActive(); ++i) tick();
    ASSERT_FALSE(forwardCenteringActive());
    EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
  }
  bool nativeCenteringAcceptsOrdinaryLateralTolerance() {
    node_->rotate_entry_centering_sequencer_.reset(node_->now().seconds());
    return node_->rotate_entry_centering_sequencer_.update(
        0.03, 0.015, node_->now().seconds(), 0.12, 1.0).stage ==
        camrod_control::CampsiteCrabReturnStage::kSteeringSettle;
  }
  // HH_261012 - Reach the real centering latch without moving a simulator or
  // renewing its independently measured steady clocks.
  using CenteringStage = camrod_control::CampsiteCrabReturnStage;
  CenteringStage centeringStage() const {
    return node_->rotate_entry_centering_sequencer_.stage();
  }
  void beginForwardLongitudinalCentering(
      const bool after_pivot = false, const double turn_guard_m = 0.05) {
    node_->rotate_entry_centering_max_speed_mps_ = 0.12;
    node_->return_translation_gain_ = 4.0;
    if (after_pivot) {
      beginForwardPivot();
    } else {
      completeDeliveryService(4);
      setReturnPolicy();
      setReturnTurnPositionBound(turn_guard_m);
      useImmediateCenteringSettle();
    }
    setSiteCenterBodyError(0.03, 0.0);
    if (after_pivot) {
      tick();
    } else {
      ASSERT_TRUE(returnRequest().first);
    }
    tick();
    tick();
    ASSERT_TRUE(forwardCenteringActive());
    ASSERT_EQ(centeringStage(), CenteringStage::kLongitudinal);
  }
  void publishCenteringDirectly() { node_->publishRotationEntryCentering(); }
  void clearForwardCenteringScope() { node_->clearSiteForwardReturn(); }
  void invalidateCenteringNumber(const int input) {
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    if (input == 0) node_->rotate_entry_centering_max_speed_mps_ = invalid;
    if (input == 1) node_->return_translation_gain_ = invalid;
    if (input == 2) node_->rotate_entry_max_position_error_m_ = invalid;
  }
  bool forwardHandoffVerified() const {
    return node_->site_forward_return_handoff_verified_;
  }
  std::string ordinaryReturnSource() const {
    return node_->returnRequestSource("done_retry");
  }
  void attemptPrematureReturnPublication() {
    node_->publishReturnRequest("test_premature_handoff");
  }
  void completeForwardExitHeading() {
    setAlongSite(3.0, false);
    tick();
    ASSERT_EQ(phase(), Phase::kCrabOut);
    EXPECT_FALSE(returnPublished());
  }
  void loadReturnPolicyCatalog(const std::string &yaml) {
    char path[] = "/tmp/camrod-return-policy-XXXXXX";
    const int fd = mkstemp(path);
    ASSERT_GE(fd, 0);
    close(fd);
    { std::ofstream output(path); output << yaml; }
    node_->camping_sites_yaml_ = path;
    node_->loadCampingSites();
    EXPECT_EQ(std::remove(path), 0);
  }
  std::string policyFor(const std::string &key) const {
    return node_->returnRoutePolicyForKey(key);
  }
  CampsiteServiceMode modeFor(const std::string &key) const {
    return node_->serviceModeForKey(key);
  }

  std::shared_ptr<CampingSiteManeuverControllerNode> node_;
  double direction_{1.0};
  const double entry_yaw_{-M_PI / 3.0};
  std::string key_;
};

TEST_F(CampingSiteManeuverControllerTest, ReturnPolicyCatalogDefaultsNativeAndPreservesTurnaround) {
  loadReturnPolicyCatalog(R"(
camping_sites:
  - {id: b5, type: camping_site_5, name: B5, x: 10, y: 20, service_mode: turnaround, return_route_policy: forward_loop}
  - {id: b4, type: camping_site_4, x: 10, y: 21, service_mode: turnaround}
)");
  for (const std::string key : {"b5", "camping_site_5", "B5"}) {
    EXPECT_EQ(policyFor(key), "forward_loop");
    EXPECT_EQ(modeFor(key), CampsiteServiceMode::kTurnaround);
  }
  EXPECT_EQ(policyFor("camping_site_4"), "native");
  EXPECT_EQ(policyFor("unconfigured"), "native");
  loadReturnPolicyCatalog("camping_sites: []\n");
  EXPECT_EQ(policyFor("camping_site_5"), "native");
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopDeliveryKeepsServiceTurnThenTurnsInsideSite) {
  completeDeliveryService();
  setReturnPolicy();
  setReturnTurnPositionBound(0.05);
  EXPECT_FALSE(forwardExitActive());
  EXPECT_EQ(ordinaryReturnSource().find("roadside_forward"), std::string::npos);
  ASSERT_TRUE(returnRequest().first);
  EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
  EXPECT_NEAR(targetYaw(), entry_yaw_, 1e-9);
  tick();
  EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
  EXPECT_NE(angularCommand(), 0.0);
  EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
  EXPECT_FALSE(returnPublished());
  const auto started = phaseStart();
  ASSERT_TRUE(returnRequest().first);
  EXPECT_EQ(phaseStart(), started);
  completeForwardExitHeading();
  finishFromDifferentLiveAnchor(false);
  EXPECT_EQ(phase(), Phase::kDone);
  EXPECT_TRUE(returnPublished());
  EXPECT_TRUE(forwardHandoffVerified());
  EXPECT_NE(ordinaryReturnSource().find(":site_policy_roadside_forward"),
            std::string::npos);
  tick();
  EXPECT_NE(ordinaryReturnSource().find(":site_policy_roadside_forward"),
            std::string::npos);
  expectOriginalAnchor();
}

TEST_F(CampingSiteManeuverControllerTest, NativeDeliveryStillExitsTurnedWithoutForwardMarker) {
  completeDeliveryService();
  ASSERT_TRUE(returnRequest().first);
  EXPECT_NEAR(camrod_control::normalizeAngle(targetYaw() - entry_yaw_ - M_PI),
              0.0, 1e-9);
  tick();
  EXPECT_EQ(phase(), Phase::kCrabOut);
  finishFromDifferentLiveAnchor(true);
  EXPECT_TRUE(returnPublished());
  EXPECT_FALSE(forwardHandoffVerified());
  EXPECT_EQ(ordinaryReturnSource().find("roadside_forward"), std::string::npos);
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopRecallStillRequiresBothLoadingConfirmations) {
  startRecall(5);
  setReturnPolicy();
  EXPECT_FALSE(forwardExitActive());
  ASSERT_TRUE(returnRequest().first);
  EXPECT_EQ(phase(), Phase::kRecallClearanceWait);
  completeClearance();
  completeTurnaround();
  EXPECT_FALSE(returnRequest().first);
  EXPECT_FALSE(urgentReturnRequest().first);
  EXPECT_EQ(phase(), Phase::kRecallReturnWait);
  EXPECT_FALSE(forwardExitActive());
  ASSERT_TRUE(finalReturnRequest().first);
  EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
  EXPECT_NEAR(targetYaw(), entry_yaw_, 1e-9);
  tick();
  EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
  completeForwardExitHeading();
  finishFromDifferentLiveAnchor(false);
  EXPECT_TRUE(returnPublished());
  EXPECT_NE(ordinaryReturnSource().find("roadside_forward"), std::string::npos);
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopTurnRejectsWrongSitePositionOrUnsupportedPolicy) {
  completeDeliveryService();
  setReturnPolicy();
  setReturnTurnPositionBound(0.05);
  setAlongSite(0.1, true);
  EXPECT_FALSE(returnRequest().first);
  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_FALSE(returnPublished());
  completeDeliveryService();
  setReturnPolicy("forward_loop_typo");
  EXPECT_FALSE(returnRequest().first);
  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_FALSE(returnPublished());
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopTurnFailsClosedOnDriftPoseLossAndSteadyTimeout) {
  for (int failure = 0; failure < 3; ++failure) {
    SCOPED_TRACE(failure);
    completeDeliveryService();
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    ASSERT_TRUE(returnRequest().first);
    if (failure == 0) setAlongSite(2.9, true);
    if (failure == 1) expirePose();
    if (failure == 2) expireForwardTurnWallClock();
    tick();
    EXPECT_EQ(phase(), Phase::kError);
    EXPECT_FALSE(returnPublished());
    EXPECT_FALSE(forwardExitActive());
    EXPECT_EQ(ordinaryReturnSource().find("roadside_forward"), std::string::npos);
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopRecentersServiceSettlingErrorBeforeTurning) {
  completeDeliveryService();
  setReturnPolicy();
  setReturnTurnPositionBound(0.05);
  useImmediateCenteringSettle();
  setAlongSite(3.06, true);
  ASSERT_TRUE(returnRequest().first);
  EXPECT_TRUE(forwardCenteringActive());
  EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
  tick();
  EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
  EXPECT_FALSE(returnPublished());
  setAlongSite(3.0, true);
  for (int i = 0; i < 4 && forwardCenteringActive(); ++i) tick();
  EXPECT_FALSE(forwardCenteringActive());
  EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
  tick();
  EXPECT_NE(angularCommand(), 0.0);
  completeForwardExitHeading();
  finishFromDifferentLiveAnchor(false);
  EXPECT_TRUE(forwardHandoffVerified());
}

// HH_261012 - V2 B4 exhausted the pivot envelope after ordinary centering
// stopped at 0.0426 m. Admission inside 0.05 m is not sufficient turn margin.
TEST_F(CampingSiteManeuverControllerTest, ForwardLoopRecentersEvenInsideUnchangedTurnBound) {
  for (const double initial_error : {0.03, 0.04, 0.05}) {
    SCOPED_TRACE(initial_error);
    completeDeliveryService(4);
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    useImmediateCenteringSettle();
    setSiteCenterBodyError(0.0, initial_error);
    ASSERT_TRUE(returnRequest().first);
    EXPECT_NEAR(preciseCenteringTolerance(), 0.01, 1e-12);
    EXPECT_TRUE(forwardCenteringActive());
    tick();
    EXPECT_NE(crabCommand(), 0.0);
    EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
    EXPECT_FALSE(returnPublished());
    const auto phase_started = phaseStart();
    const auto phase_wall_started = phaseWallStart();
    const auto centering_started = centeringWallStart();
    EXPECT_TRUE(returnRequest().first);
    EXPECT_EQ(phaseStart(), phase_started);
    EXPECT_EQ(phaseWallStart(), phase_wall_started);
    EXPECT_EQ(centeringWallStart(), centering_started);
    EXPECT_TRUE(forwardCenteringActive());
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopPreciseCenteringRequiresTightLateralAndRadialCompletion) {
  completeDeliveryService(4);
  setReturnPolicy();
  setReturnTurnPositionBound(0.05);
  useImmediateCenteringSettle();
  setSiteCenterBodyError(0.0263, -0.0136);
  ASSERT_TRUE(returnRequest().first);
  tick();
  // HH_261012 - B4's measured lateral residual would already be latched by
  // native 0.02 m centering, preventing a strict 0.01 m radial completion.
  EXPECT_NE(crabCommand(), 0.0);
  EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
  setSiteCenterBodyError(0.015, 0.004);
  for (int i = 0; i < 4; ++i) tick();
  EXPECT_TRUE(forwardCenteringActive());
  EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
  EXPECT_GT(centeringLongitudinalCommand(), 0.0);
  setSiteCenterBodyError(0.008, 0.004);
  tick();
  EXPECT_FALSE(forwardCenteringActive());
  EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
  tick();
  EXPECT_NE(angularCommand(), 0.0);
  // A completed precision stage never licenses movement beyond the old guard.
  setSiteCenterBodyError(0.0502, 0.0);
  tick();
  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_FALSE(returnPublished());
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopPreciseCenteringScalesDownWithSmallerGuard) {
  completeDeliveryService();
  setReturnPolicy();
  setReturnTurnPositionBound(0.02);
  useImmediateCenteringSettle();
  setSiteCenterBodyError(0.0, 0.015);
  ASSERT_TRUE(returnRequest().first);
  EXPECT_NEAR(preciseCenteringTolerance(), 0.004, 1e-12);
  setSiteCenterBodyError(0.005, 0.001);
  for (int i = 0; i < 4; ++i) tick();
  EXPECT_TRUE(forwardCenteringActive());
  setSiteCenterBodyError(0.003, 0.001);
  tick();
  EXPECT_FALSE(forwardCenteringActive());
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopPreciseCenteringRetainsAllRecoveryBounds) {
  for (int failure = 0; failure < 4; ++failure) {
    SCOPED_TRACE(failure);
    completeDeliveryService();
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    setSiteCenterBodyError(0.0, 0.04);
    ASSERT_TRUE(returnRequest().first);
    ASSERT_TRUE(forwardCenteringActive());
    if (failure == 0) setSiteCenterBodyError(0.31, 0.0);
    if (failure == 1) expirePose();
    if (failure == 2) {
      // HH_261012 - A late exact-center pose cannot evade the 15 s deadline
      // by entering the success branch before the next centering command.
      useImmediateCenteringSettle();
      setSiteCenterBodyError(0.015, 0.0);
      for (int i = 0; i < 3; ++i) tick();
      setSiteCenterBodyError(0.0, 0.0);
      expireCenteringWallClock();
    }
    if (failure == 3) expireForwardTurnWallClock();
    tick();
    EXPECT_EQ(phase(), Phase::kError);
    EXPECT_FALSE(returnPublished());
    EXPECT_FALSE(forwardCenteringActive());
    EXPECT_TRUE(nativeCenteringAcceptsOrdinaryLateralTolerance());
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopPreciseCenteringRestoresNativeConfigOnCancelAndUrgentReturn) {
  for (const bool urgent : {false, true}) {
    completeDeliveryService();
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    setSiteCenterBodyError(0.0, 0.04);
    ASSERT_TRUE(returnRequest().first);
    ASSERT_TRUE(forwardCenteringActive());
    EXPECT_FALSE(nativeCenteringAcceptsOrdinaryLateralTolerance());
    if (urgent) {
      ASSERT_TRUE(urgentReturnRequest().first);
    } else {
      cancel();
    }
    EXPECT_FALSE(forwardExitActive());
    EXPECT_FALSE(forwardCenteringActive());
    EXPECT_TRUE(nativeCenteringAcceptsOrdinaryLateralTolerance());
  }
}

// HH_261012 - B9/B10 opt into a different convergence margin, not a larger
// safety envelope. Native and omitted-profile sites keep their exact behavior.
TEST_F(CampingSiteManeuverControllerTest, ForwardProfileCatalogAliasesAndReloadAreIsolated) {
  for (const std::string alias : {"b9", "camping_site_9", "B9"}) {
    completeDeliveryService(9);
    // HH_261012 - Supply the existing bounded-turn fixture before the loader
    // replaces its policy map; native construction intentionally defaults to 0 s.
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    loadReturnPolicyCatalog(R"(
camping_sites:
  - {id: b9, type: camping_site_9, name: B9, x: 10, y: 20, service_mode: turnaround, return_route_policy: forward_loop, forward_return_centering_tolerance_m: 0.015, forward_return_recenter_trigger_m: 0.035}
  - {id: b8, type: camping_site_8, x: 10, y: 21, service_mode: turnaround, return_route_policy: forward_loop}
  - {id: b11, type: camping_site_11, x: 10, y: 22, service_mode: roadside_stop}
)");
    ASSERT_TRUE(hasReturnProfile(alias));
    EXPECT_EQ(returnProfile(alias), std::make_pair(0.015, 0.035));
    EXPECT_FALSE(hasReturnProfile("b8"));
    EXPECT_FALSE(hasReturnProfile("camping_site_11"));
    EXPECT_EQ(modeFor(alias), CampsiteServiceMode::kTurnaround);
    selectReturnAlias(alias);
    const auto request = returnRequest();
    ASSERT_TRUE(request.first) << request.second;
    EXPECT_DOUBLE_EQ(preciseCenteringTolerance(), 0.015);
    EXPECT_DOUBLE_EQ(recenterTrigger(), 0.035);
    cancel();
    EXPECT_DOUBLE_EQ(preciseCenteringTolerance(), 0.01);
    EXPECT_DOUBLE_EQ(recenterTrigger(), 0.025);
  }
  loadReturnPolicyCatalog("camping_sites: []\n");
  EXPECT_FALSE(hasReturnProfile("camping_site_9"));
  EXPECT_FALSE(hasReturnProfile("b9"));
  EXPECT_FALSE(hasReturnProfile("B9"));
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileOmissionKeepsExactGuardScaledDefaults) {
  for (const double guard : {0.02, 0.05, 0.10}) {
    completeDeliveryService(8);
    setReturnPolicy();
    setReturnTurnPositionBound(guard);
    ASSERT_TRUE(returnRequest().first);
    EXPECT_DOUBLE_EQ(preciseCenteringTolerance(), std::min(0.01, 0.2 * guard));
    EXPECT_DOUBLE_EQ(recenterTrigger(), 0.5 * guard);
    EXPECT_FALSE(hasReturnProfile(key_));
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileMalformedPairsFailClosedWithoutDroppingLaterRows) {
  for (const std::string fields : {
           "forward_return_centering_tolerance_m: 0.015",
           "forward_return_recenter_trigger_m: 0.035",
           "forward_return_centering_tolerance_m: bad, forward_return_recenter_trigger_m: 0.035",
           "forward_return_centering_tolerance_m: null, forward_return_recenter_trigger_m: 0.035",
           "forward_return_centering_tolerance_m: .nan, forward_return_recenter_trigger_m: 0.035",
           "forward_return_centering_tolerance_m: 0.015, forward_return_recenter_trigger_m: .inf"}) {
    SCOPED_TRACE(fields);
    completeDeliveryService(9);
    // HH_261012 - Reach profile validation, not the independent 0 s timeout
    // admission guard, so malformed catalog rejection cannot pass vacuously.
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    loadReturnPolicyCatalog(
        "camping_sites:\n  - {type: camping_site_9, x: 10, y: 20, "
        "return_route_policy: forward_loop, " + fields + "}\n"
        "  - {type: camping_site_10, x: 11, y: 20, return_route_policy: forward_loop, "
        "forward_return_centering_tolerance_m: 0.015, forward_return_recenter_trigger_m: 0.035}\n");
    EXPECT_TRUE(hasReturnProfile(key_));
    EXPECT_EQ(returnProfile("camping_site_10"), std::make_pair(0.015, 0.035));
    EXPECT_FALSE(returnRequest().first);
    EXPECT_EQ(phase(), Phase::kError);
    EXPECT_NE(phaseDetail().find("invalid forward_loop centering profile"),
              std::string::npos);
    EXPECT_FALSE(forwardExitActive());
    EXPECT_FALSE(returnPublished());
    EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
    EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileRejectsInvalidOrderValuesAndActualGuard) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const auto &values : std::vector<std::tuple<double, double, double>>{
           {0.0, 0.035, 0.05}, {-0.01, 0.035, 0.05}, {0.015, 0.0, 0.05},
           {0.035, 0.035, 0.05}, {0.04, 0.035, 0.05}, {0.015, 0.05, 0.05},
           {0.015, 0.06, 0.05}, {0.015, 0.035, 0.03}, {nan, 0.035, 0.05},
           {0.015, nan, 0.05}, {inf, 0.035, 0.05}, {0.015, inf, 0.05},
           {0.015, 0.035, nan}, {0.015, 0.035, inf}}) {
    completeDeliveryService(9);
    setReturnPolicy();
    setReturnProfile(std::get<0>(values), std::get<1>(values));
    setReturnTurnPositionBound(std::get<2>(values));
    EXPECT_FALSE(returnRequest().first);
    EXPECT_EQ(phase(), Phase::kError);
    EXPECT_FALSE(forwardExitActive());
    EXPECT_FALSE(returnPublished());
    EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileDoesNotAlterNativeOrUrgentDeparture) {
  for (const bool urgent : {false, true}) {
    completeDeliveryService(9);
    setReturnTurnPositionBound(0.05);
    setReturnPolicy(urgent ? "forward_loop" : "native");
    setReturnProfile(-1.0, std::numeric_limits<double>::quiet_NaN());
    ASSERT_TRUE((urgent ? urgentReturnRequest() : returnRequest()).first);
    EXPECT_FALSE(forwardExitActive());
    EXPECT_DOUBLE_EQ(preciseCenteringTolerance(), 0.01);
    EXPECT_DOUBLE_EQ(recenterTrigger(), 0.025);
    EXPECT_NE(phase(), Phase::kError);
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileTriggerReservesHardGuardAndPreservesYawAndClock) {
  beginProfiledForwardReturn();
  const auto started = phaseWallStart();
  const double yaw = targetYaw();
  setSiteCenterBodyError(0.0, 0.0349);
  tick();
  EXPECT_FALSE(forwardCenteringActive());
  EXPECT_NE(angularCommand(), 0.0);
  setSiteCenterBodyError(0.0, 0.0351);
  tick();
  ASSERT_TRUE(forwardCenteringActive());
  EXPECT_EQ(forwardRecenteringCount(), 1);
  EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
  EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
  EXPECT_EQ(phaseWallStart(), started);
  EXPECT_DOUBLE_EQ(targetYaw(), yaw);
  setSiteCenterBodyError(0.0, 0.0501);
  tick();
  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_FALSE(returnPublished());
  EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
  EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileCompletesMeasuredB9B10ResidualsBeforeRealignment) {
  for (const auto &error : std::vector<std::pair<double, double>>{
           {-0.005134, 0.010857}, {0.009060, 0.010451}, {-0.008793, -0.010859}}) {
    beginProfiledForwardReturn(true);
    const auto started = phaseWallStart();
    const auto center_started = centeringWallStart();
    setSiteCenterBodyError(error.first, error.second);
    tick();
    EXPECT_FALSE(forwardCenteringActive());
    EXPECT_EQ(centeringStage(), CenteringStage::kLongitudinal);
    EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
    EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
    EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
    EXPECT_EQ(phaseWallStart(), started);
    EXPECT_EQ(centeringWallStart(), center_started);
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileScalesFullTargetRealignmentAndLatchCeiling) {
  for (const double lateral : {0.0149, 0.0151, 0.1074, 0.1076}) {
    beginProfiledForwardReturn(true);
    setSiteCenterBodyError(0.003, lateral);
    tick();
    if (lateral < 0.015) {
      EXPECT_TRUE(forwardCenteringActive());  // radial > .015, reachable by x
      EXPECT_EQ(centeringStage(), CenteringStage::kLongitudinal);
    } else if (lateral <= 0.1075) {
      EXPECT_EQ(centeringStage(), CenteringStage::kLateral);
      EXPECT_GT(crabCommand(), 0.0);
      EXPECT_LE(crabCommand(), 0.12);
    } else {
      EXPECT_EQ(phase(), Phase::kError);
      EXPECT_EQ(centeringStage(), CenteringStage::kLongitudinal);
      EXPECT_NE(phaseDetail().find("lateral latch exceeded"), std::string::npos);
    }
    EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileKeepsFreshnessAndBothSteadyDeadlines) {
  for (int failure = 0; failure < 3; ++failure) {
    beginProfiledForwardReturn();
    setSiteCenterBodyError(0.0, 0.0351);
    tick();
    ASSERT_TRUE(forwardCenteringActive());
    if (failure == 0) expirePose();
    if (failure == 1) expireCenteringWallClock();
    if (failure == 2) expireForwardTurnWallClock();
    tick();
    EXPECT_EQ(phase(), Phase::kError);
    EXPECT_FALSE(forwardExitActive());
    EXPECT_FALSE(returnPublished());
    EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
    EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileRetainsTenRecoveryCapWithoutRenewingPhase) {
  beginProfiledForwardReturn();
  const auto started = phaseWallStart();
  const double yaw = targetYaw();
  for (int recovery = 1; recovery <= 10; ++recovery) {
    setSiteCenterBodyError(0.0, 0.0351);
    tick();
    ASSERT_TRUE(forwardCenteringActive());
    EXPECT_EQ(forwardRecenteringCount(), recovery);
    EXPECT_EQ(phaseWallStart(), started);
    completeForwardRecentering();
    tick();
    EXPECT_DOUBLE_EQ(targetYaw(), yaw);
  }
  setSiteCenterBodyError(0.0, 0.0351);
  tick();
  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_NE(phaseDetail().find("recovery limit reached: 10/10"), std::string::npos);
  EXPECT_FALSE(returnPublished());
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileCannotLeakAfterCancelUrgentOrNextSite) {
  for (const bool urgent : {false, true}) {
    beginProfiledForwardReturn();
    setSiteCenterBodyError(0.0, 0.0351);
    tick();
    ASSERT_TRUE(forwardCenteringActive());
    if (urgent) ASSERT_TRUE(urgentReturnRequest().first);
    else cancel();
    EXPECT_FALSE(forwardExitActive());
    EXPECT_EQ(forwardRecenteringCount(), 0);
    EXPECT_DOUBLE_EQ(preciseCenteringTolerance(), 0.01);
    EXPECT_DOUBLE_EQ(recenterTrigger(), 0.025);
    EXPECT_TRUE(nativeCenteringAcceptsOrdinaryLateralTolerance());
    completeDeliveryService(8);
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    ASSERT_TRUE(returnRequest().first);
    EXPECT_DOUBLE_EQ(preciseCenteringTolerance(), 0.01);
    EXPECT_DOUBLE_EQ(recenterTrigger(), 0.025);
  }
}

// HH_261012 - Only an explicit departure-centering profile may shorten the
// native steering dwell; guard/deadline limits and default sites are unchanged.
TEST_F(CampingSiteManeuverControllerTest, ForwardProfileSteeringDwellAliasesAndReloadAreIsolated) {
  for (const std::string alias : {"b10", "camping_site_10", "B10"}) {
    completeDeliveryService(10);
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    loadReturnPolicyCatalog(R"(
camping_sites:
  - {id: b10, type: camping_site_10, name: B10, x: 10, y: 20, return_route_policy: forward_loop, forward_return_centering_tolerance_m: 0.015, forward_return_recenter_trigger_m: 0.035, forward_return_steering_settle_s: 0.4}
  - {id: b9, type: camping_site_9, x: 11, y: 20, return_route_policy: forward_loop, forward_return_centering_tolerance_m: 0.015, forward_return_recenter_trigger_m: 0.035}
)");
    ASSERT_TRUE(configuredProfileDwell(alias).has_value());
    EXPECT_DOUBLE_EQ(*configuredProfileDwell(alias), 0.4);
    EXPECT_FALSE(configuredProfileDwell("b9").has_value());
    selectReturnAlias(alias);
    setSiteCenterBodyError(0.03, 0.0);
    const auto request = returnRequest();
    ASSERT_TRUE(request.first) << request.second;
    EXPECT_DOUBLE_EQ(selectedProfileDwell(), 0.4);
    EXPECT_EQ(centeringCommandAfterDwell(0.4).stage, CenteringStage::kLongitudinal);
    cancel();
    EXPECT_DOUBLE_EQ(selectedProfileDwell(), 1.2);
  }
  loadReturnPolicyCatalog("camping_sites: []\n");
  EXPECT_FALSE(hasReturnProfile("b10"));
  EXPECT_FALSE(hasReturnProfile("camping_site_10"));
  EXPECT_FALSE(hasReturnProfile("B10"));
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileSteeringDwellOmissionKeepsB9AndOtherSitesNative) {
  for (const int site : {8, 9}) {
    completeDeliveryService(site);
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    if (site == 9) setReturnProfile();
    setSiteCenterBodyError(0.03, 0.0);
    ASSERT_TRUE(returnRequest().first);
    EXPECT_DOUBLE_EQ(selectedProfileDwell(), 1.2);
    EXPECT_EQ(centeringCommandAfterDwell(0.4).stage, CenteringStage::kSteeringSettle);
    EXPECT_EQ(centeringCommandAfterDwell(1.2).stage, CenteringStage::kLongitudinal);
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileSteeringDwellRejectsMalformedOrIncompleteYaml) {
  for (const std::string fields : {
           "forward_return_steering_settle_s: 0.4",
           "forward_return_centering_tolerance_m: 0.015, forward_return_steering_settle_s: 0.4",
           "forward_return_centering_tolerance_m: 0.015, forward_return_recenter_trigger_m: 0.035, forward_return_steering_settle_s: bad",
           "forward_return_centering_tolerance_m: 0.015, forward_return_recenter_trigger_m: 0.035, forward_return_steering_settle_s: null",
           "forward_return_centering_tolerance_m: 0.015, forward_return_recenter_trigger_m: 0.035, forward_return_steering_settle_s: .nan",
           "forward_return_centering_tolerance_m: 0.015, forward_return_recenter_trigger_m: 0.035, forward_return_steering_settle_s: .inf",
           "forward_return_centering_tolerance_m: 0.015, forward_return_recenter_trigger_m: 0.035, forward_return_steering_settle_s: -0.4",
           "forward_return_centering_tolerance_m: 0.015, forward_return_recenter_trigger_m: 0.035, forward_return_steering_settle_s: 0",
           "forward_return_centering_tolerance_m: 0.015, forward_return_recenter_trigger_m: 0.035, forward_return_steering_settle_s: 1.21"}) {
    SCOPED_TRACE(fields);
    completeDeliveryService(10);
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    loadReturnPolicyCatalog(
        "camping_sites:\n  - {type: camping_site_10, x: 10, y: 20, "
        "return_route_policy: forward_loop, " + fields + "}\n");
    EXPECT_FALSE(returnRequest().first);
    EXPECT_EQ(phase(), Phase::kError);
    EXPECT_NE(phaseDetail().find("invalid forward_loop centering profile"),
              std::string::npos);
    EXPECT_FALSE(forwardExitActive());
    EXPECT_FALSE(returnPublished());
    EXPECT_DOUBLE_EQ(selectedProfileDwell(), 1.2);
    EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
    EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileSteeringDwellRespectsConfiguredNativeUpperBound) {
  for (const double native_dwell : {0.3, 0.0, -0.1,
           std::numeric_limits<double>::quiet_NaN()}) {
    completeDeliveryService(10);
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    setReturnProfile();
    setReturnProfileDwell(0.4);
    setNativeSteeringDwell(native_dwell);
    EXPECT_FALSE(returnRequest().first);
    EXPECT_EQ(phase(), Phase::kError);
    EXPECT_FALSE(forwardExitActive());
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileSteeringDwellHoldsZeroThenAllowsLongitudinalOnly) {
  beginDwellProfileCentering();
  const auto started = phaseWallStart();
  const auto center_started = centeringWallStart();
  const auto before = centeringCommandAfterDwell(0.399);
  EXPECT_EQ(before.stage, CenteringStage::kSteeringSettle);
  EXPECT_DOUBLE_EQ(before.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(before.linear_y_mps, 0.0);
  const auto ready = centeringCommandAfterDwell(0.4);
  EXPECT_EQ(ready.stage, CenteringStage::kLongitudinal);
  EXPECT_GT(ready.linear_x_mps, 0.0);
  EXPECT_LE(ready.linear_x_mps, 0.12);
  EXPECT_DOUBLE_EQ(ready.linear_y_mps, 0.0);
  EXPECT_EQ(phaseWallStart(), started);
  EXPECT_EQ(centeringWallStart(), center_started);
  EXPECT_EQ(forwardRecenteringCount(), 0);
  EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileSteeringDwellRestoresNativeOnCancelAndUrgentReturn) {
  for (const bool urgent : {false, true}) {
    beginDwellProfileCentering();
    if (urgent) ASSERT_TRUE(urgentReturnRequest().first);
    else cancel();
    EXPECT_FALSE(forwardExitActive());
    EXPECT_DOUBLE_EQ(selectedProfileDwell(), 1.2);
    EXPECT_EQ(centeringCommandAfterDwell(0.4).stage, CenteringStage::kSteeringSettle);
    EXPECT_EQ(centeringCommandAfterDwell(1.2).stage, CenteringStage::kLongitudinal);
  }
}

// HH_261012 - The measured B10 candidate may budget up to twenty recoveries
// inside the original guards. All omitted/native profiles still use ten.
TEST_F(CampingSiteManeuverControllerTest, ForwardProfileRecoveryLimitAliasesAndDefaultsStayIsolated) {
  for (const std::string alias : {"b10", "camping_site_10", "B10"}) {
    completeDeliveryService(10);
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    loadReturnPolicyCatalog(R"(
camping_sites:
  - {id: b10, type: camping_site_10, name: B10, x: 10, y: 20, return_route_policy: forward_loop, forward_return_centering_tolerance_m: 0.015, forward_return_recenter_trigger_m: 0.035, forward_return_steering_settle_s: 0.4, forward_return_recentering_limit: 20}
  - {id: b9, type: camping_site_9, x: 11, y: 20, return_route_policy: forward_loop, forward_return_centering_tolerance_m: 0.015, forward_return_recenter_trigger_m: 0.035}
)");
    selectReturnAlias(alias);
    const auto request = returnRequest();
    ASSERT_TRUE(request.first) << request.second;
    EXPECT_EQ(selectedProfileLimit(), 20);
    cancel();
    EXPECT_EQ(selectedProfileLimit(), 10);
  }
  loadReturnPolicyCatalog("camping_sites: []\n");
  EXPECT_FALSE(hasReturnProfile("b10"));
  EXPECT_EQ(selectedProfileLimit(), 10);
  beginProfiledForwardReturn();  // Existing B9 pair omits the optional limit.
  EXPECT_EQ(selectedProfileLimit(), 10);
  beginForwardPivot();  // Existing profile omission keeps ten as well.
  EXPECT_EQ(selectedProfileLimit(), 10);
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileRecoveryLimitRejectsMalformedAndOutOfRangeYaml) {
  for (const std::string limit : {"bad", "null", ".nan", ".inf", "20.0",
                                  "20.5", "0", "-1", "21", "99999999999999999999"}) {
    SCOPED_TRACE(limit);
    completeDeliveryService(10);
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    loadReturnPolicyCatalog(
        "camping_sites:\n  - {type: camping_site_10, x: 10, y: 20, "
        "return_route_policy: forward_loop, forward_return_centering_tolerance_m: 0.015, "
        "forward_return_recenter_trigger_m: 0.035, forward_return_recentering_limit: " +
        limit + "}\n");
    EXPECT_FALSE(returnRequest().first);
    EXPECT_EQ(phase(), Phase::kError);
    EXPECT_NE(phaseDetail().find("invalid forward_loop centering profile"),
              std::string::npos);
    EXPECT_FALSE(forwardExitActive());
    EXPECT_EQ(selectedProfileLimit(), 10);
    EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
    EXPECT_FALSE(returnPublished());
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileRecoveryLimitAloneCannotBypassRequiredPair) {
  for (const std::string fields : {
           "forward_return_recentering_limit: 20",
           "forward_return_centering_tolerance_m: 0.015, forward_return_recentering_limit: 20",
           "forward_return_recenter_trigger_m: 0.035, forward_return_recentering_limit: 20"}) {
    completeDeliveryService(10);
    setReturnPolicy();
    setReturnTurnPositionBound(0.05);
    loadReturnPolicyCatalog(
        "camping_sites:\n  - {type: camping_site_10, x: 10, y: 20, "
        "return_route_policy: forward_loop, " + fields + "}\n");
    EXPECT_FALSE(returnRequest().first);
    EXPECT_EQ(phase(), Phase::kError);
    EXPECT_EQ(selectedProfileLimit(), 10);
    EXPECT_FALSE(forwardExitActive());
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileAllowsTwentiethRecoveryButRejectsTwentyFirst) {
  beginLimitProfilePivot();
  const auto started = phaseWallStart();
  const double target = targetYaw();
  for (int recovery = 1; recovery <= 20; ++recovery) {
    setSiteCenterBodyError(0.0, 0.0351);
    tick();
    ASSERT_TRUE(forwardCenteringActive());
    ASSERT_EQ(phase(), Phase::kAlignRetraceYaw);
    EXPECT_EQ(forwardRecenteringCount(), recovery);
    EXPECT_EQ(selectedProfileLimit(), 20);
    EXPECT_EQ(phaseWallStart(), started);
    EXPECT_DOUBLE_EQ(targetYaw(), target);
    EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
    EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
    completeLimitProfileRecentering();
  }
  setSiteCenterBodyError(0.0, 0.0351);
  tick();
  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_NE(phaseDetail().find("recovery limit reached: 20/20"), std::string::npos);
  EXPECT_EQ(selectedProfileLimit(), 10);
  EXPECT_FALSE(returnPublished());
  EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
  EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileRecoveryLimitRetainsGuardsBeyondTen) {
  for (int failure = 0; failure < 4; ++failure) {
    SCOPED_TRACE(failure);
    beginLimitProfilePivot();
    const auto started = phaseWallStart();
    for (int recovery = 1; recovery <= 11; ++recovery) {
      setSiteCenterBodyError(0.0, 0.0351);
      tick();
      ASSERT_TRUE(forwardCenteringActive());
      ASSERT_EQ(forwardRecenteringCount(), recovery);
      EXPECT_EQ(phaseWallStart(), started);
      if (recovery < 11) completeLimitProfileRecentering();
    }
    if (failure == 0) expireCenteringWallClock();
    if (failure == 1) expireForwardTurnWallClock();
    if (failure == 2) expirePose();
    if (failure == 3) setSiteCenterBodyError(0.0, 0.0501);
    tick();
    EXPECT_EQ(phase(), Phase::kError);
    EXPECT_FALSE(forwardExitActive());
    EXPECT_EQ(selectedProfileLimit(), 10);
    EXPECT_FALSE(returnPublished());
    EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
    EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileRecoveryLimitRestoresTenOnCancelUrgentAndNative) {
  for (const bool urgent : {false, true}) {
    beginLimitProfilePivot();
    setSiteCenterBodyError(0.0, 0.0351);
    tick();
    ASSERT_EQ(forwardRecenteringCount(), 1);
    if (urgent) ASSERT_TRUE(urgentReturnRequest().first);
    else cancel();
    EXPECT_EQ(selectedProfileLimit(), 10);
    EXPECT_EQ(forwardRecenteringCount(), 0);
    EXPECT_FALSE(forwardExitActive());
    completeDeliveryService(10);
    setReturnPolicy("native");
    setReturnProfile();
    setReturnProfileLimit(20);
    ASSERT_TRUE(returnRequest().first);
    EXPECT_EQ(selectedProfileLimit(), 10);
    EXPECT_FALSE(forwardExitActive());
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardProfileRecoveryLimitAcceptsBoundedLowerEndpoint) {
  beginLimitProfilePivot(1);
  EXPECT_EQ(selectedProfileLimit(), 1);
  setSiteCenterBodyError(0.0, 0.0351);
  tick();
  ASSERT_EQ(forwardRecenteringCount(), 1);
  completeLimitProfileRecentering();
  setSiteCenterBodyError(0.0, 0.0351);
  tick();
  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_NE(phaseDetail().find("recovery limit reached: 1/1"), std::string::npos);
}

// HH_261012 - V5 B3/B9 had an unreachable lateral residual after the
// longitudinal latch; B10 developed it during guarded pivot recentering.
TEST_F(CampingSiteManeuverControllerTest, ForwardLoopCenteringRealignsMeasuredUnreachableLateralResiduals) {
  for (const auto &error : std::vector<std::pair<double, double>>{
           {0.00155, 0.0206}, {-0.00138758, 0.02013151},
           {-0.00235826, -0.01152098}}) {
    beginForwardLongitudinalCentering();
    setSiteCenterBodyError(error.first, error.second);
    const auto phase_started = phaseWallStart();
    const auto centering_started = centeringWallStart();
    const double yaw_target = targetYaw();
    tick();
    EXPECT_EQ(centeringStage(), CenteringStage::kLateral);
    EXPECT_GT(crabCommand() * error.second, 0.0);
    EXPECT_LE(std::abs(crabCommand()), 0.12);
    EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
    EXPECT_EQ(phaseWallStart(), phase_started);
    EXPECT_EQ(centeringWallStart(), centering_started);
    EXPECT_DOUBLE_EQ(targetYaw(), yaw_target);
    EXPECT_EQ(forwardRecenteringCount(), 0);
    EXPECT_FALSE(returnPublished());
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopCenteringKeepsReachableLateralAndLongitudinalProgressLatched) {
  for (const auto &error : std::vector<std::pair<double, double>>{
           {0.0101, 0.0206}, {0.00814279, 0.00842004},
           {-0.00839804, 0.00585321}, {0.009, 0.0099}}) {
    beginForwardLongitudinalCentering();
    setSiteCenterBodyError(error.first, error.second);
    tick();
    EXPECT_TRUE(forwardCenteringActive());
    EXPECT_EQ(centeringStage(), CenteringStage::kLongitudinal);
    EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
    EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopCenteringRetainsRadialSuccessBeforeAnyRealignment) {
  beginForwardLongitudinalCentering();
  setSiteCenterBodyError(0.005, 0.007);
  tick();
  EXPECT_FALSE(forwardCenteringActive());
  EXPECT_EQ(centeringStage(), CenteringStage::kLongitudinal);
  EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
  EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
  EXPECT_EQ(forwardRecenteringCount(), 0);
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopLateralRealignmentCannotRenewEitherSteadyDeadline) {
  for (const bool expire_whole_phase : {false, true}) {
    beginForwardLongitudinalCentering();
    const auto phase_started = phaseWallStart();
    const auto centering_started = centeringWallStart();
    for (int correction = 0; correction < 3; ++correction) {
      setSiteCenterBodyError(0.001, 0.02);
      tick();
      ASSERT_EQ(centeringStage(), CenteringStage::kLateral);
      EXPECT_EQ(phaseWallStart(), phase_started);
      EXPECT_EQ(centeringWallStart(), centering_started);
      EXPECT_EQ(forwardRecenteringCount(), 0);
      setSiteCenterBodyError(0.02, 0.004);
      tick();
      EXPECT_EQ(centeringStage(), CenteringStage::kSteeringSettle);
      EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
      tick();
      ASSERT_EQ(centeringStage(), CenteringStage::kLongitudinal);
    }
    if (expire_whole_phase) expireForwardTurnWallClock();
    else expireCenteringWallClock();
    setSiteCenterBodyError(0.0, 0.0);
    tick();
    EXPECT_EQ(phase(), Phase::kError);
    EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
    EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
    EXPECT_FALSE(returnPublished());
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopActiveRealignmentPreservesPivotCountAndPositionGuard) {
  beginForwardLongitudinalCentering(true);
  ASSERT_EQ(forwardRecenteringCount(), 1);
  const auto phase_started = phaseWallStart();
  const auto centering_started = centeringWallStart();
  setSiteCenterBodyError(-0.00235826, -0.01152098);
  tick();
  EXPECT_EQ(centeringStage(), CenteringStage::kLateral);
  EXPECT_EQ(forwardRecenteringCount(), 1);
  EXPECT_EQ(phaseWallStart(), phase_started);
  EXPECT_EQ(centeringWallStart(), centering_started);
  setSiteCenterBodyError(0.0, 0.0502);
  tick();
  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
  EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopLateralRealignmentCannotEraseExistingLatchFault) {
  beginForwardLongitudinalCentering();
  // HH_261012 - Inside initial admission but beyond the configured precision
  // latch (0.005 + 0.10 m): reset must not erase the original fault.
  setSiteCenterBodyError(0.001, 0.106);
  tick();
  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_EQ(centeringStage(), CenteringStage::kLongitudinal);
  EXPECT_NE(phaseDetail().find("lateral latch exceeded"), std::string::npos);
  EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
  EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopInvalidCenteringNumbersNeverResetTheSequencer) {
  for (int input = 0; input < 3; ++input) {
    SCOPED_TRACE(input);
    beginForwardLongitudinalCentering();
    setSiteCenterBodyError(0.00155, 0.0206);
    invalidateCenteringNumber(input);
    publishCenteringDirectly();
    EXPECT_EQ(centeringStage(), CenteringStage::kLongitudinal);
    EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
    if (input < 2) {
      EXPECT_EQ(phase(), Phase::kError);
    } else {
      // HH_261012 - A non-finite optional guard grants no reset authority;
      // its existing fallback outside this correction remains unchanged.
      EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
    }
  }
}

TEST_F(CampingSiteManeuverControllerTest, NativeCenteringRetainsItsLongitudinalLatch) {
  beginForwardLongitudinalCentering();
  clearForwardCenteringScope();
  setSiteCenterBodyError(0.00155, 0.0206);
  publishCenteringDirectly();
  EXPECT_FALSE(forwardExitActive());
  EXPECT_EQ(centeringStage(), CenteringStage::kLongitudinal);
  EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
  EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopLateralRealignmentScalesWithThePreciseTarget) {
  beginForwardLongitudinalCentering(false, 0.02);
  EXPECT_NEAR(preciseCenteringTolerance(), 0.004, 1e-12);
  setSiteCenterBodyError(0.001, -0.005);
  tick();
  EXPECT_EQ(centeringStage(), CenteringStage::kLateral);
  EXPECT_LT(crabCommand(), 0.0);
  EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
}

// HH_261012 - Segmented departure pivots recover margin without broadening
// their position envelope, replaying the service turn, or renewing authority.
TEST_F(CampingSiteManeuverControllerTest, ForwardLoopPivotPausesAtHalfGuardAndResumesSameTarget) {
  beginForwardPivot();
  const auto phase_started = phaseStart();
  const auto wall_started = phaseWallStart();
  const double target = targetYaw();
  setSiteCenterBodyError(0.0, 0.0249);
  tick();
  EXPECT_FALSE(forwardCenteringActive());
  EXPECT_NE(angularCommand(), 0.0);
  setSiteCenterBodyError(0.0, 0.0251);
  tick();
  ASSERT_TRUE(forwardCenteringActive());
  EXPECT_EQ(forwardRecenteringCount(), 1);
  // The crossing tick only publishes zero; translation belongs to later ticks.
  EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
  EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
  EXPECT_EQ(phaseStart(), phase_started);
  EXPECT_EQ(phaseWallStart(), wall_started);
  EXPECT_DOUBLE_EQ(targetYaw(), target);
  tick();
  EXPECT_NE(crabCommand(), 0.0);
  EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
  EXPECT_FALSE(returnPublished());
  completeForwardRecentering();
  tick();
  EXPECT_NE(angularCommand(), 0.0);
  EXPECT_DOUBLE_EQ(targetYaw(), target);
  EXPECT_EQ(phaseWallStart(), wall_started);
  EXPECT_EQ(forwardRecenteringCount(), 1);
  completeForwardExitHeading();
  finishFromDifferentLiveAnchor(false);
  EXPECT_TRUE(forwardHandoffVerified());
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopActiveRecenteringCannotUseInitialAdmissionEnvelope) {
  completeDeliveryService();
  setReturnPolicy();
  setReturnTurnPositionBound(0.05);
  setCenteringAdmissionBound(0.65);
  useImmediateCenteringSettle();
  setSiteCenterBodyError(0.0, 0.10);
  ASSERT_TRUE(returnRequest().first);
  EXPECT_TRUE(forwardCenteringActive());
  EXPECT_EQ(forwardRecenteringCount(), 0);
  tick();
  EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
  completeForwardRecentering();
  tick();
  setSiteCenterBodyError(0.0, 0.03);
  tick();
  ASSERT_EQ(forwardRecenteringCount(), 1);
  ASSERT_TRUE(forwardCenteringActive());
  // Less than initial 0.65 m, but already beyond the immutable pivot bound.
  setSiteCenterBodyError(0.0, 0.0501);
  tick();
  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
  EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
  EXPECT_FALSE(returnPublished());
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopGuardBreachCannotStartRecovery) {
  beginForwardPivot();
  setSiteCenterBodyError(0.0, 0.0501);
  tick();
  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_EQ(forwardRecenteringCount(), 0);
  EXPECT_FALSE(forwardCenteringActive());
  EXPECT_FALSE(returnPublished());
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopPivotAllowsAtMostTenRecoveriesWithoutRenewingClock) {
  beginForwardPivot();
  const auto wall_started = phaseWallStart();
  const double target = targetYaw();
  // HH_261012 - Exhaust the measured candidate's budget, not a shorter loop
  // that would miss an off-by-one at the new tenth recovery boundary.
  for (int recovery = 1; recovery <= 10; ++recovery) {
    setSiteCenterBodyError(0.0, 0.03);
    tick();
    ASSERT_EQ(phase(), Phase::kAlignRetraceYaw);
    EXPECT_EQ(forwardRecenteringCount(), recovery);
    EXPECT_TRUE(forwardCenteringActive());
    EXPECT_EQ(phaseWallStart(), wall_started);
    EXPECT_TRUE(returnRequest().first);
    EXPECT_EQ(phaseWallStart(), wall_started);
    completeForwardRecentering();
    tick();
    EXPECT_DOUBLE_EQ(targetYaw(), target);
  }
  setSiteCenterBodyError(0.0, 0.03);
  tick();
  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_NE(phaseDetail().find("recovery limit reached: 10/10"), std::string::npos);
  EXPECT_DOUBLE_EQ(angularCommand(), 0.0);
  EXPECT_DOUBLE_EQ(crabCommand(), 0.0);
  EXPECT_FALSE(returnPublished());
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopRecenteringKeepsFreshnessAndBothSteadyDeadlines) {
  for (int failure = 0; failure < 3; ++failure) {
    SCOPED_TRACE(failure);
    beginForwardPivot();
    setSiteCenterBodyError(0.0, 0.03);
    tick();
    ASSERT_TRUE(forwardCenteringActive());
    if (failure == 0) expirePose();
    if (failure == 1) expireCenteringWallClock();
    if (failure == 2) expireForwardTurnWallClock();
    tick();
    EXPECT_EQ(phase(), Phase::kError);
    EXPECT_FALSE(returnPublished());
    EXPECT_FALSE(forwardCenteringActive());
    EXPECT_EQ(forwardRecenteringCount(), 0);
    EXPECT_TRUE(nativeCenteringAcceptsOrdinaryLateralTolerance());
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopRecenteringCounterCannotLeakToUrgentOrNativeMission) {
  for (const bool urgent : {false, true}) {
    beginForwardPivot();
    setSiteCenterBodyError(0.0, 0.03);
    tick();
    ASSERT_EQ(forwardRecenteringCount(), 1);
    if (urgent) {
      ASSERT_TRUE(urgentReturnRequest().first);
    } else {
      cancel();
    }
    EXPECT_EQ(forwardRecenteringCount(), 0);
    EXPECT_FALSE(forwardCenteringActive());
    EXPECT_TRUE(nativeCenteringAcceptsOrdinaryLateralTolerance());
    completeDeliveryService(3);
    EXPECT_EQ(policyFor(key_), "native");
    EXPECT_EQ(forwardRecenteringCount(), 0);
    ASSERT_TRUE(returnRequest().first);
    tick();
    EXPECT_EQ(phase(), Phase::kCrabOut);
  }
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopMarkerRequiresCompletedHeadingAndHandoff) {
  completeDeliveryService();
  setReturnPolicy();
  ASSERT_TRUE(returnRequest().first);
  attemptPrematureReturnPublication();
  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_FALSE(returnPublished());
  completeDeliveryService();
  setReturnPolicy();
  ASSERT_TRUE(returnRequest().first);
  completeForwardExitHeading();
  finishFromDifferentLiveAnchor(true);
  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_FALSE(returnPublished());
  EXPECT_EQ(ordinaryReturnSource().find("roadside_forward"), std::string::npos);
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopCancelAndNextNativeMissionCannotInheritProof) {
  completeDeliveryService();
  setReturnPolicy();
  ASSERT_TRUE(returnRequest().first);
  cancel();
  EXPECT_FALSE(forwardExitActive());
  EXPECT_FALSE(forwardHandoffVerified());
  completeDeliveryService();
  setReturnPolicy();
  ASSERT_TRUE(returnRequest().first);
  completeForwardExitHeading();
  finishFromDifferentLiveAnchor(false);
  ASSERT_TRUE(forwardHandoffVerified());
  completeDeliveryService(4);
  EXPECT_FALSE(forwardHandoffVerified());
  EXPECT_EQ(policyFor(key_), "native");
  ASSERT_TRUE(returnRequest().first);
  tick();
  finishFromDifferentLiveAnchor(true);
  EXPECT_TRUE(returnPublished());
  EXPECT_EQ(ordinaryReturnSource().find("roadside_forward"), std::string::npos);
}

TEST_F(CampingSiteManeuverControllerTest, ForwardLoopPreferenceCannotChangeUrgentBatteryRouting) {
  completeDeliveryService();
  setReturnPolicy();
  ASSERT_TRUE(returnRequest().first);
  ASSERT_TRUE(forwardExitActive());
  ASSERT_TRUE(urgentReturnRequest().first);
  EXPECT_FALSE(forwardExitActive());
  completeForwardExitHeading();
  finishFromDifferentLiveAnchor(false);
  EXPECT_EQ(phase(), Phase::kAlignReturnRouteYaw);
  EXPECT_FALSE(returnPublished());
  finishFromDifferentLiveAnchor(true);
  EXPECT_EQ(phase(), Phase::kDone);
  EXPECT_TRUE(returnPublished());
  EXPECT_EQ(ordinaryReturnSource().find("roadside_forward"), std::string::npos);
  EXPECT_NE(ordinaryReturnSource().find("battery_urgent_return"), std::string::npos);
}

TEST_F(CampingSiteManeuverControllerTest, FinishedVoiceCannotShortenClearanceFloor) {
  startRecall(1);
  ASSERT_TRUE(returnRequest().first);
  completeClearanceVoice();
  elapsed(7.9);
  setAlongSite(0.30);
  tick();
  EXPECT_EQ(phase(), Phase::kRecallClearanceWait);
  elapsed(8.1);
  tick();
  EXPECT_EQ(phase(), Phase::kAlignEntryYaw);
}

TEST_F(CampingSiteManeuverControllerTest, MissingVoiceWaitsForItsOwnTimeout) {
  startRecall(1);
  ASSERT_TRUE(returnRequest().first);
  elapsed(8.1);
  setAlongSite(0.30);
  tick();
  EXPECT_EQ(phase(), Phase::kRecallClearanceWait);
  elapsed(14.1);
  tick();
  EXPECT_EQ(phase(), Phase::kAlignEntryYaw);
}

TEST_F(CampingSiteManeuverControllerTest, B1ThroughB10TurnInsideSiteThenReturnFromCurrentLanePose) {
  for (int site = 1; site <= 10; ++site) {
    SCOPED_TRACE(site);
    startRecall(site);
    ASSERT_TRUE(returnRequest().first);
    EXPECT_EQ(phase(), Phase::kRecallClearanceWait);
    EXPECT_TRUE(internalPhase());
    EXPECT_TRUE(turnaroundActive());
    tick();
    EXPECT_EQ(phase(), Phase::kRecallClearanceWait);
    mutatePlanningMission();
    completeClearance();
    EXPECT_DOUBLE_EQ(offset(), 3.0);
    expectOriginalAnchor();
    completeTurnaround();
    confirmFinalReturn();
    EXPECT_TRUE(returnLatched());
    finishFromDifferentLiveAnchor(true);
    EXPECT_EQ(phase(), Phase::kDone);
    EXPECT_TRUE(returnPublished());
    expectOriginalAnchor();
  }
}

TEST_F(CampingSiteManeuverControllerTest, B11ThroughB13KeepRoadsideForwardLoopWithoutReentry) {
  for (int site = 11; site <= 13; ++site) {
    SCOPED_TRACE(site);
    startRecall(site);
    ASSERT_TRUE(returnRequest().first);
    EXPECT_EQ(phase(), Phase::kCrabOut);
    EXPECT_FALSE(turnaroundActive());
    EXPECT_DOUBLE_EQ(offset(), 0.30);
    finishFromDifferentLiveAnchor(false);
    EXPECT_EQ(phase(), Phase::kDone);
    EXPECT_TRUE(returnPublished());
  }
}

TEST_F(CampingSiteManeuverControllerTest, RecallNeverAutoReturnsAfterLoadingDwell) {
  enableAutoReturn();
  startRecall(1);
  elapsed(60.0);
  tick();
  EXPECT_EQ(phase(), Phase::kWaitReturn);
  EXPECT_FALSE(returnLatched());
  EXPECT_FALSE(returnPublished());
}

TEST_F(CampingSiteManeuverControllerTest, ReturnBeforeArrivalCannotLatchLoadingCompletion) {
  startRecall(1, false);
  EXPECT_FALSE(returnRequest().first);
  EXPECT_FALSE(returnLatched());
  tick();
  EXPECT_EQ(phase(), Phase::kCrabIn);
  EXPECT_FALSE(returnRequest().first);
  EXPECT_FALSE(returnLatched());
  cancel();
  EXPECT_FALSE(returnRequest().first);
  EXPECT_FALSE(returnLatched());
}

TEST_F(CampingSiteManeuverControllerTest, DuplicateReturnNeverRestartsAcceptedPhases) {
  startRecall(1);
  ASSERT_TRUE(returnRequest().first);
  const auto clearance_started = phaseStart();
  ASSERT_TRUE(returnRequest().first);
  EXPECT_EQ(phaseStart(), clearance_started);
  completeClearance();
  const auto entry_started = phaseStart();
  ASSERT_TRUE(returnRequest().first);
  EXPECT_EQ(phase(), Phase::kCrabIn);
  EXPECT_EQ(phaseStart(), entry_started);
  completeTurnaround();
  EXPECT_FALSE(returnRequest().first);
  EXPECT_EQ(phase(), Phase::kRecallReturnWait);
  confirmFinalReturn();
  ASSERT_TRUE(finalReturnRequest().first);
  EXPECT_EQ(phase(), Phase::kCrabOut);
  finishFromDifferentLiveAnchor(true);
  ASSERT_TRUE(returnRequest().first);
  EXPECT_EQ(phase(), Phase::kDone);
}

TEST_F(CampingSiteManeuverControllerTest, OccupancyExemptionEndsBeforeFullSiteReentry) {
  startRecall(1);
  setOccupied(true);
  EXPECT_FALSE(occupancyBlocks());
  ASSERT_TRUE(returnRequest().first);
  EXPECT_TRUE(occupancyBlocks());
  elapsed(60.0);
  tick();
  EXPECT_EQ(phase(), Phase::kRecallClearanceWait);
  setOccupied(false);
  tick();
  EXPECT_EQ(phase(), Phase::kAlignEntryYaw);
}

TEST_F(CampingSiteManeuverControllerTest, PoseMustBeFreshBeforeAndAfterClearance) {
  startRecall(1);
  expirePose();
  ASSERT_TRUE(returnRequest().first);
  EXPECT_EQ(phase(), Phase::kRecallClearanceWait);
  elapsed(60.0);
  expirePose();
  tick();
  EXPECT_EQ(phase(), Phase::kRecallClearanceWait);
  setAlongSite(0.30);
  tick();
  EXPECT_EQ(phase(), Phase::kAlignEntryYaw);
  expirePose();
  tick();
  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_FALSE(returnRequest().first);
}

TEST_F(CampingSiteManeuverControllerTest, CancelStopsClearanceAndEveryReentryPhase) {
  for (int stage = 0; stage < 5; ++stage) {
    SCOPED_TRACE(stage);
    startRecall(1);
    ASSERT_TRUE(returnRequest().first);
    if (stage >= 1) {
      completeClearance();
    }
    if (stage == 2) {
      setAlongSite(3.0);
      tick();
      ASSERT_EQ(phase(), Phase::kRotate180);
    }
    if (stage >= 3) {
      completeTurnaround();
    }
    if (stage == 4) {
      confirmFinalReturn();
    }
    cancel();
    elapsed(120.0);
    tick();
    EXPECT_EQ(phase(), Phase::kIdle);
    EXPECT_FALSE(returnLatched());
    EXPECT_FALSE(turnaroundActive());
    EXPECT_FALSE(returnPublished());
  }
}

TEST_F(CampingSiteManeuverControllerTest, ReentryRequiresConfiguredFiniteSite) {
  startRecall(1);
  makeConfiguredSiteInvalid();
  EXPECT_FALSE(returnRequest().first);
  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_FALSE(returnLatched());
  startRecall(1);
  removeConfiguredSite();
  EXPECT_FALSE(returnRequest().first);
  EXPECT_EQ(phase(), Phase::kError);
}

TEST_F(CampingSiteManeuverControllerTest, RoadsideReturnAcceptsOneClickWhilePoseRecovers) {
  startRecall(11);
  expirePose();
  EXPECT_TRUE(returnRequest().first);
  EXPECT_EQ(phase(), Phase::kWaitReturn);
  tick();
  EXPECT_EQ(phase(), Phase::kWaitReturn);
  setAlongSite(0.30);
  tick();
  EXPECT_EQ(phase(), Phase::kCrabOut);
  EXPECT_FALSE(turnaroundActive());
}

TEST_F(CampingSiteManeuverControllerTest, RecallUsesCrabTurnaroundEvenIfDeliveryUsesReverseEntry) {
  setReverseDeliveryMode();
  startRecall(1);
  ASSERT_TRUE(returnRequest().first);
  completeClearance();
  completeTurnaround();
  confirmFinalReturn();
  EXPECT_EQ(phase(), Phase::kCrabOut);
}

TEST_F(CampingSiteManeuverControllerTest, RecallAdoptionPreservesRoadsideWaitAtOccupiedSite) {
  startRecall(1);
  setOccupied(true);
  EXPECT_TRUE(adoptRecallAtRoadside());
  EXPECT_EQ(phase(), Phase::kWaitReturn);
  EXPECT_NEAR(offset(), 0.30, 1.0e-9);
}

TEST_F(CampingSiteManeuverControllerTest, NextDeliveryCannotInheritRecallReturnAuthorization) {
  startRecall(1);
  ASSERT_TRUE(returnRequest().first);
  completeClearance();
  completeTurnaround();
  confirmFinalReturn();
  finishFromDifferentLiveAnchor(true);
  ASSERT_EQ(phase(), Phase::kDone);
  startDeliveryAfterRecall();
  EXPECT_EQ(phase(), Phase::kAlignEntryYaw);
  EXPECT_FALSE(turnaroundActive());
  EXPECT_FALSE(returnLatched());
  EXPECT_FALSE(returnPublished());
  tick();
  EXPECT_EQ(phase(), Phase::kCrabIn);
  EXPECT_DOUBLE_EQ(offset(), 3.0);
}

TEST_F(CampingSiteManeuverControllerTest, BatteryUrgentReturnStopsIncompleteRecallEntryForEverySite) {
  for (int site = 1; site <= 13; ++site) {
    SCOPED_TRACE(site);
    startRecall(site, false);
    tick();
    ASSERT_EQ(phase(), Phase::kCrabIn);
    setAlongSite(0.15);
    ASSERT_TRUE(urgentReturnRequest().first);
    EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
    EXPECT_TRUE(returnLatched());
    EXPECT_FALSE(turnaroundActive());
    EXPECT_FALSE(returnPublished());
    tick();
    ASSERT_EQ(phase(), Phase::kCrabOut);
    finishFromDifferentLiveAnchor(false);
    if (site <= 10) {
      EXPECT_EQ(phase(), Phase::kAlignReturnRouteYaw);
      EXPECT_FALSE(returnPublished());
      finishFromDifferentLiveAnchor(true);
      EXPECT_EQ(finalReturnSource().find("roadside_forward"), std::string::npos);
    } else {
      EXPECT_NE(finalReturnSource().find("roadside_forward"), std::string::npos);
    }
    EXPECT_EQ(phase(), Phase::kDone);
    EXPECT_TRUE(returnPublished());
    EXPECT_NE(finalReturnSource().find("battery_urgent_return:platform_status"),
              std::string::npos);
    expectOriginalAnchor();
  }
}

TEST_F(CampingSiteManeuverControllerTest, BatteryUrgentReturnSkipsLoadingConfirmationAndSiteReentry) {
  startRecall(1);
  setOccupied(true);
  ASSERT_TRUE(urgentReturnRequest().first);
  EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
  EXPECT_FALSE(turnaroundActive());
  EXPECT_DOUBLE_EQ(offset(), 0.30);
  tick();
  EXPECT_EQ(phase(), Phase::kCrabOut);
  EXPECT_FALSE(returnPublished());
}

TEST_F(CampingSiteManeuverControllerTest, BatteryUrgentYawAlignmentTimesOutFailClosed) {
  startRecall(1, false);
  ASSERT_TRUE(urgentReturnRequest().first);
  ASSERT_EQ(phase(), Phase::kAlignRetraceYaw);
  setCrabReturnTimeout(1.0);
  elapsed(1.1);

  tick();

  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_FALSE(returnPublished());
}

TEST_F(CampingSiteManeuverControllerTest, SimilarSourceCannotAuthorizeUrgentRecallReturn) {
  startRecall(1, false);
  EXPECT_FALSE(urgentReturnRequest("ui:not_battery_urgent_return").first);
  EXPECT_FALSE(urgentReturnRequest("ui:battery_urgent_return_suffix").first);
  EXPECT_FALSE(returnLatched());
  EXPECT_EQ(phase(), Phase::kAlignEntryYaw);
  EXPECT_TRUE(urgentReturnRequest("backend:battery_urgent_return:site=1").first);
  EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
}

TEST_F(CampingSiteManeuverControllerTest, BatteryUrgentReturnPreemptsClearanceAndPartialRotation) {
  startRecall(1);
  ASSERT_TRUE(returnRequest().first);
  ASSERT_EQ(phase(), Phase::kRecallClearanceWait);
  ASSERT_TRUE(urgentReturnRequest().first);
  EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
  EXPECT_FALSE(turnaroundActive());

  startRecall(1);
  ASSERT_TRUE(returnRequest().first);
  completeClearance();
  setAlongSite(3.0);
  tick();
  ASSERT_EQ(phase(), Phase::kRotate180);
  setPartiallyRotatedPose(3.0);
  ASSERT_TRUE(urgentReturnRequest().first);
  EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
  tick();
  EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
  EXPECT_FALSE(returnPublished());
  setAlongSite(3.0);
  tick();
  EXPECT_EQ(phase(), Phase::kCrabOut);
}

TEST_F(CampingSiteManeuverControllerTest, BatteryUrgentReturnStopsPartialDeliveryWithoutFinishingEntry) {
  beginDelivery(2);
  setAlongSite(1.0);
  ASSERT_TRUE(urgentReturnRequest().first);
  EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
  tick();
  ASSERT_EQ(phase(), Phase::kCrabOut);
  finishFromDifferentLiveAnchor(false);
  EXPECT_EQ(phase(), Phase::kAlignReturnRouteYaw);
  EXPECT_FALSE(returnPublished());
  finishFromDifferentLiveAnchor(true);
  EXPECT_EQ(phase(), Phase::kDone);
  EXPECT_TRUE(returnPublished());
}

TEST_F(CampingSiteManeuverControllerTest, BatteryUrgentDuplicateCancelAndStalePoseStayBounded) {
  startRecall(3);
  ASSERT_TRUE(urgentReturnRequest().first);
  const auto started = phaseStart();
  ASSERT_TRUE(urgentReturnRequest().first);
  EXPECT_EQ(phaseStart(), started);
  cancel();
  EXPECT_FALSE(urgentReturnRequest().first);
  EXPECT_EQ(phase(), Phase::kIdle);
  EXPECT_FALSE(returnLatched());

  startRecall(3);
  expirePose();
  EXPECT_FALSE(urgentReturnRequest().first);
  EXPECT_EQ(phase(), Phase::kError);
  EXPECT_FALSE(returnPublished());
  EXPECT_FALSE(urgentReturnRequest().first);
}

TEST_F(CampingSiteManeuverControllerTest, TurnedRecallWaitCannotBeBypassedByTimerOldClickOrLowBattery) {
  startRecall(1);
  EXPECT_FALSE(finalReturnRequest().first);
  ASSERT_TRUE(returnRequest().first);
  EXPECT_FALSE(finalReturnRequest().first);
  completeClearance();
  completeTurnaround();
  enableAutoReturn();
  for (int heartbeat = 0; heartbeat < 3; ++heartbeat) {
    elapsed(600.0);
    tick();
    EXPECT_EQ(phase(), Phase::kRecallReturnWait);
    EXPECT_FALSE(returnPublished());
    EXPECT_FALSE(returnRequest().first);
    EXPECT_FALSE(urgentReturnRequest().first);
    EXPECT_FALSE(finalReturnRequest("robot_ui:not_recall_final_return").first);
    EXPECT_FALSE(finalReturnRequest("robot_ui:recall_final_return_suffix").first);
    EXPECT_EQ(phase(), Phase::kRecallReturnWait);
    EXPECT_FALSE(returnLatched());
  }
  setAlongSite(3.0, true);
  confirmFinalReturn();
  finishFromDifferentLiveAnchor(true);
  EXPECT_EQ(phase(), Phase::kDone);
  EXPECT_TRUE(returnPublished());
}

TEST_F(CampingSiteManeuverControllerTest, FinalLoadingConfirmationWaitsForFreshPoseWithoutRepeatingTurn) {
  startRecall(3);
  ASSERT_TRUE(returnRequest().first);
  completeClearance();
  completeTurnaround();
  expirePose();
  ASSERT_TRUE(finalReturnRequest().first);
  EXPECT_EQ(phase(), Phase::kRecallReturnWait);
  tick();
  EXPECT_EQ(phase(), Phase::kRecallReturnWait);
  EXPECT_FALSE(returnPublished());
  setAlongSite(3.0, true);
  tick();
  EXPECT_EQ(phase(), Phase::kAlignRetraceYaw);
  tick();
  EXPECT_EQ(phase(), Phase::kCrabOut);
}
