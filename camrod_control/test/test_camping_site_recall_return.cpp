// Exercise the real campsite controller without spinning a robot launch. The
// friend fixture injects fresh localization/mission inputs and advances phase
// clocks deterministically; velocity safety remains owned by the final gate.
#include "gtest/gtest.h"

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

  std::shared_ptr<CampingSiteManeuverControllerNode> node_;
  double direction_{1.0};
  const double entry_yaw_{-M_PI / 3.0};
  std::string key_;
};

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
