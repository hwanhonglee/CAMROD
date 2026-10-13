// HH_261007 - Reproduce the manual-trigger/path-relative deadlock through the
// production gate callbacks. DDS domain 213 is never the live CARLA domain.
#include "gtest/gtest.h"
#include <cstdlib>
#include <iostream>
#include <thread>
#define CAMROD_CONTROL_MANUAL_RECOVERY_TEST
#include "../src/cmd_vel_safety_gate_node.cpp"

namespace camrod_control {

namespace {
YAML::Node exampleAccessCatalog() {
  return YAML::Load(R"(
frame_id: map
service_access:
  - site: camping_site_1
    lanelet_ids: ['2193', '2219']
    corners: [{x: -1, y: 0}, {x: 0, y: 0}, {x: 0, y: 1}, {x: -1, y: 1}]
)");
}
std::vector<ManualServiceArea> exampleAccessSites() {
  return {{"camping_site_1", {{0,0},{2,0},{2,2},{0,2}}}};
}
}

TEST(ManualServiceAccessCatalog, LoadsExactUnionAndRejectsInvalidCatalogsAtomically) {
  auto areas = exampleAccessSites();
  EXPECT_NO_THROW(loadManualServiceAccess(exampleAccessCatalog(),"map",areas));
  ASSERT_EQ(areas.front().access_polygons.size(),1U);
  EXPECT_TRUE(manualServiceAreaContains(areas.front(),-.5,.5));
  EXPECT_FALSE(manualServiceAreaContains(areas.front(),-.5,1.5));
  for (int failure = 0; failure < 10; ++failure) {
    SCOPED_TRACE(failure);
    auto catalog = exampleAccessCatalog();
    auto unchanged = exampleAccessSites();
    if (failure == 0) catalog["frame_id"] = "odom";
    if (failure == 1) catalog.remove("frame_id");
    if (failure == 2) catalog["service_access"][0]["site"] = "unknown";
    if (failure == 3) catalog["service_access"].push_back(YAML::Clone(catalog["service_access"][0]));
    if (failure == 4) catalog["service_access"][0]["lanelet_ids"] = YAML::Load("[1, 1]");
    if (failure == 5) catalog["service_access"][0]["lanelet_ids"] = YAML::Load("[-1]");
    if (failure == 6) catalog["service_access"][0]["corners"][0]["x"] = ".nan";
    if (failure == 7) catalog["service_access"][0]["frame_id"] = "odom";
    if (failure == 8) {
      for (auto point : catalog["service_access"][0]["corners"])
        point["x"] = point["x"].as<double>() - 1;
    }
    if (failure == 9)
      unchanged.push_back({"other_site", {{-2,0},{-1,0},{-1,1},{-2,1}}});
    EXPECT_THROW(loadManualServiceAccess(catalog,"map",unchanged),std::exception);
    for (const auto &area : unchanged) EXPECT_TRUE(area.access_polygons.empty());
  }
}

TEST(ManualServiceAccessCatalog, OptionalAuthoredAccessFileReplay) {
  const auto sites_path = std::getenv("CAMROD_MANUAL_SITE_CATALOG");
  const auto access_path = std::getenv("CAMROD_MANUAL_ACCESS_CATALOG");
  if (!sites_path || !access_path) GTEST_SKIP() << "Set authored site/access catalog paths";
  const auto catalog = YAML::LoadFile(sites_path);
  std::vector<ManualServiceArea> areas;
  for (const auto &site : catalog["camping_sites"]) {
    ManualServiceArea area; area.id = site["type"].as<std::string>();
    for (const auto &point : site["corners"])
      area.polygon.emplace_back(point["x"].as<double>(),point["y"].as<double>());
    ASSERT_TRUE(validManualServiceArea(area));
    areas.push_back(std::move(area));
  }
  ASSERT_FALSE(areas.empty());
  const auto access = YAML::LoadFile(access_path);
  ASSERT_NO_THROW(loadManualServiceAccess(access,"map",areas));
  std::size_t loaded = 0;
  for (const auto &area : areas) loaded += area.access_polygons.size();
  EXPECT_EQ(loaded,access["service_access"].size());
  std::cout << "authored_manual_access loaded_sites=" << areas.size()
            << " connectors=" << loaded << " file=" << access_path << '\n';
}

class CmdVelSafetyGateManualRecoveryTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::InitOptions options;
    options.set_domain_id(213);
    rclcpp::init(0, nullptr, options);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  static avg_msgs::msg::AvgTwist twist(double x, double y = 0, double w = 0) {
    avg_msgs::msg::AvgTwist cmd;
    cmd.linear.x = x; cmd.linear.y = y; cmd.angular.z = w;
    return cmd;
  }
  static avg_msgs::msg::AvgOccupancyGrid grid(bool contact = false,
                                             bool obstacle = false) {
    avg_msgs::msg::AvgOccupancyGrid g;
    g.header.frame_id = "map";
    g.info.resolution = 0.1;
    g.info.width = 100; g.info.height = 100;
    g.info.origin.position.x = -5; g.info.origin.position.y = -5;
    g.info.origin.orientation.w = 1;
    g.data.assign(10000, 0);
    if (contact) g.data[51 * 100 + 50] = 100;  // Left body contact at (0.05, 0.15).
    if (obstacle) g.data[46 * 100 + 50] = 90;  // Radar obstacle in right escape.
    return g;
  }

  void SetUp() override {
    node_ = std::make_shared<CmdVelSafetyGateNode>();
    node_->state_timer_->cancel();
    node_->command_timeout_timer_->cancel();
    node_->manual_input_topic_ = "/control/manual_cmd_vel_ros";
    node_->command_source_arbiter_config_.manual_input_enabled = true;
    node_->command_source_arbiter_.setConfig(node_->command_source_arbiter_config_);
    node_->command_source_arbiter_.setEngagement(true, false);
    node_->gate_config_.platform_safety_enabled = false;
    node_->gate_policy_.setConfig(node_->gate_config_);
    node_->gate_policy_.setManualEngage(true);
    node_->gate_policy_.setPlatformDriveEnable(true);
    node_->enable_yaw_alignment_zone_ = false;
    node_->enable_route_heading_alignment_ = false;
    node_->enable_resume_after_hold_voice_gate_ = false;
    node_->speed_scale_ = 1.0;
    node_->path_relative_recovery_config_.enabled = true;
    node_->route_safety_recovery_config_.opposite_recovery_probe_distance_m = 0.3;
    node_->route_safety_recovery_.setConfig(node_->route_safety_recovery_config_);
    MotionCostStopConfig c;
    c.stale_stop_enabled = true;
    c.require_dynamic_source = true;
    c.dynamic_source_labels = {"radar"};
    c.latch_enabled = false;
    c.use_speed_dependent_lookahead = false;
    c.body_near_enabled = false;
    c.dynamic_front_use_local_path = false;
    c.tapered_rounded_boundary_enabled = false;
    c.lanelet_enabled = true;
    c.lanelet_body_hard_stop_enabled = true;
    c.lanelet_footprint_enabled = true;
    c.body_front_m = c.body_rear_m = c.body_left_m = c.body_right_m = 0.2;
    c.footprint_front_m = c.footprint_rear_m = 0.3;
    c.footprint_left_m = c.footprint_right_m = 0.3;
    node_->motion_cost_stop_config_ = c;
    node_->motion_cost_stop_.setConfig(c);
    node_->pose_source_preference_ = "pose_topic";
    node_->merged_grid_frame_ = node_->lanelet_grid_frame_ = "map";
    evidence();
    reader_ = std::make_shared<rclcpp::Node>("manual_recovery_test_reader");
    sub_ = reader_->create_subscription<avg_msgs::msg::AvgTwist>(
        "/control/cmd_vel", 100,
        [this](avg_msgs::msg::AvgTwist::ConstSharedPtr m) { outputs_.push_back(*m); });
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (node_->command_publisher_->get_subscription_count() == 0 &&
           std::chrono::steady_clock::now() < end) drain();
    ASSERT_GT(node_->command_publisher_->get_subscription_count(), 0U);
    outputs_.clear();
  }
  void TearDown() override { sub_.reset(); reader_.reset(); node_.reset(); }
  void drain() {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
    while (std::chrono::steady_clock::now() < end) rclcpp::spin_some(reader_);
  }
  void evidence(bool contact = true, bool obstacle = false, double y = 0) {
    const double now = node_->nowSec();
    avg_msgs::msg::AvgPoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1;
    node_->latest_pose_ = pose;
    node_->latest_pose_receive_sec_ = now;
    node_->motion_cost_stop_.setLaneletGrid(grid(contact), now);
    node_->motion_cost_stop_.setMergedGrid(grid(false, obstacle), now);
    node_->motion_cost_stop_.setSourceGrid("radar", grid(false, obstacle), now);
    node_->refreshMotionCostStopPose();
  }
  void command(const avg_msgs::msg::AvgTwist &cmd,
               CommandInputSource source = CommandInputSource::kManual) {
    outputs_.clear();
    node_->onCommand(cmd, source);
    drain();
  }
  void trigger() {
    command(twist(0, 0, -0.2));
    ASSERT_TRUE(node_->route_safety_recovery_.active());
    ASSERT_FALSE(node_->route_safety_triggered_by_navigation_);
    EXPECT_EQ(node_->route_safety_recovery_.triggerReason(), "lanelet_physical_body_cost");
    expectZero();
  }
  void expectZero() {
    ASSERT_FALSE(outputs_.empty());
    for (const auto &o : outputs_) {
      EXPECT_DOUBLE_EQ(o.linear.x, 0); EXPECT_DOUBLE_EQ(o.linear.y, 0);
      EXPECT_DOUBLE_EQ(o.angular.z, 0);
    }
  }
  void expectCrab() {
    ASSERT_FALSE(outputs_.empty());
    EXPECT_DOUBLE_EQ(outputs_.back().linear.y, -0.1);
    EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0);
    EXPECT_DOUBLE_EQ(outputs_.back().angular.z, 0);
  }
  void timer() { outputs_.clear(); node_->publishState(); drain(); }
  void expireCommandWatchdog() {
    // HH_261007 - A manual route exception must never keep a stale command alive.
    outputs_.clear();
    node_->last_input_command_sec_ = node_->nowSec() - 2.0;
    node_->command_timeout_timer_->reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(110));
    rclcpp::spin_some(node_);
    node_->command_timeout_timer_->cancel();
    drain();
  }
  bool hold() const { return node_->route_safety_recovery_.active(); }
  void estop() { node_->gate_policy_.setEstopSource("test", true); }
  void stalePose() { node_->latest_pose_receive_sec_ -= 5; }
  void staleLanelet() { node_->motion_cost_stop_.setLaneletGrid(grid(true), node_->nowSec() - 10); }
  void platformEnable(bool enabled) { node_->gate_policy_.setPlatformDriveEnable(enabled); }
  void releaseOwner() {
    node_->gate_policy_.setManualEngage(false);
    node_->command_source_arbiter_.setEngagement(false, false);
    node_->onAuthorizationChanged("test_manual_release");
  }
  void genericMode() { node_->path_relative_recovery_config_.enabled = false; }
  void navigationOwner() {
    node_->command_source_arbiter_.setEngagement(false, true);
    node_->gate_policy_.setManualEngage(false);
    node_->gate_policy_.setMissionEngage(true);
  }
  bool handoffReady() { return node_->pathRelativeHandoffReady(node_->nowSec()); }
  // HH_261007 - Keep manual geography independent from the cancelled phase.
  void serviceArea() {
    node_->manual_service_area_enabled_ = true;
    node_->manual_service_areas_ = {{"camping_site_1", {{-0.15,0.1},{0.2,0.1},{0.2,0.4},{-0.15,0.4}}}};
    node_->campsite_phase_ = "IDLE";
    node_->updateManeuverPhases();
  }
  void serviceFrameMismatch() { node_->manual_service_area_frame_ = "wrong_map"; }
  void secondServiceArea() { node_->manual_service_areas_.push_back(node_->manual_service_areas_.front()); }
  void serviceSweepHazard(int value) {
    auto g = grid(true); g.data[50 * 100 + 54] = value;
    node_->motion_cost_stop_.setLaneletGrid(g, node_->nowSec());
  }
  void disableServiceArea() { node_->manual_service_area_enabled_ = false; }
  void rearLookaheadHazard() {
    // HH_261007 - Rear directional stop with no present/swept rotation contact.
    node_->motion_cost_stop_config_.lanelet_check_reverse = true;
    node_->motion_cost_stop_.setConfig(node_->motion_cost_stop_config_);
    evidence(false);
    auto lanelet = grid();
    lanelet.data[50 * 100 + 42] = 100;
    node_->motion_cost_stop_.setLaneletGrid(lanelet, node_->nowSec());
  }
  void moveServiceAreaAway() {
    // HH_261007 - Road-only completion must not depend on cached CRAB_IN state
    // or continued body overlap with an already exited authored site.
    node_->manual_service_areas_ = {{"distant_site", {{3,3},{4,3},{4,4},{3,4}}}};
  }
  void accessApproach(const int direction, const bool with_access = true,
                      const double along = 0.0) {
    // Road ends at s=.4; the authored site starts at s=.8: a .4 m gap.
    // Direction 0/1/2 exercises forward/reverse/CRAB without changing yaw.
    const double ux = direction == 0 ? 1.0 : direction == 1 ? -1.0 : 0.0;
    const double uy = direction == 2 ? 1.0 : 0.0;
    const auto rectangle = [&](double a, double b, double left, double right) {
      ManualServicePolygon polygon;
      for (const auto &point : {std::make_pair(a,left), std::make_pair(b,left),
                                std::make_pair(b,right), std::make_pair(a,right)})
        polygon.emplace_back(ux * point.first - uy * point.second,
                             uy * point.first + ux * point.second);
      return polygon;
    };
    node_->manual_service_area_enabled_ = true;
    node_->manual_service_areas_ = {{"camping_site_1", rectangle(.8,2.5,-.8,.8)}};
    if (with_access)
      node_->manual_service_areas_.front().access_polygons = {rectangle(.35,.85,-.8,.8)};
    node_->motion_cost_stop_config_.lanelet_footprint_enabled = false;
    node_->motion_cost_stop_.setConfig(node_->motion_cost_stop_config_);
    evidence(false);
    auto lanelet = grid();
    for (unsigned y = 0; y < lanelet.info.height; ++y)
      for (unsigned x = 0; x < lanelet.info.width; ++x) {
        const double wx = -5.0 + (x + .5) * .1;
        const double wy = -5.0 + (y + .5) * .1;
        if (ux * wx + uy * wy >= .4)
          lanelet.data[y * lanelet.info.width + x] = 100;
      }
    node_->motion_cost_stop_.setLaneletGrid(lanelet, node_->nowSec());
    node_->latest_pose_->pose.position.x = ux * along;
    node_->latest_pose_->pose.position.y = uy * along;
    node_->refreshMotionCostStopPose();
  }
  static avg_msgs::msg::AvgTwist accessCommand(const int direction) {
    return direction == 0 ? twist(2) : direction == 1 ? twist(-2) : twist(0,2);
  }
  void accessUnknown() {
    auto lanelet = grid();
    lanelet.data[50 * 100 + 56] = -1; // (.65,.05), inside forward connector.
    node_->motion_cost_stop_.setLaneletGrid(lanelet, node_->nowSec());
  }
  void accessRadar() {
    auto radar = grid(); radar.data[50 * 100 + 56] = 100;
    node_->motion_cost_stop_.setSourceGrid("radar", radar, node_->nowSec());
    node_->motion_cost_stop_.setMergedGrid(radar, node_->nowSec());
  }
  std::string serviceReason() const { return node_->manual_service_area_last_reason_; }
  std::string campsitePhase() const { return node_->campsite_phase_; }
  bool scopedAccessDecision(const avg_msgs::msg::AvgTwist &command) {
    return node_->motion_cost_stop_.evaluateManualServiceAreaCommand(
      command, node_->nowSec(), node_->manual_service_areas_).has_value();
  }
  void narrowAccess() {
    node_->manual_service_areas_.front().access_polygons = {
      {{.35,-.05},{.85,-.05},{.85,.05},{.35,.05}}};
  }
  void secondDisjointAccessSite() {
    node_->manual_service_areas_.front().polygon = {{.8,-.8},{.9,-.8},{.9,.8},{.8,.8}};
    node_->manual_service_areas_.push_back(
      {"camping_site_2", {{1,-.8},{2.5,-.8},{2.5,.8},{1,.8}}});
  }
  void restoreServiceFrame() { node_->manual_service_area_frame_ = "map"; }
  void fixedYawLock() {
    // HH_261007 - The existing fixed-zone policy converts translation into
    // an in-place +0.2 rad/s command at this pose; service permission must
    // evaluate and publish that exact result, not the original forward input.
    YawAlignmentZone zone;
    zone.id = "test_fixed_yaw_lock";
    zone.target_yaw = 0.5;
    zone.angular_kp = 1.0;
    zone.max_angular_z = 0.2;
    node_->yaw_zones_ = {zone};
    node_->yaw_alignment_frame_id_ = "map";
    node_->enable_yaw_alignment_zone_ = true;
    node_->active_yaw_zone_.reset();
    node_->yaw_zone_hold_start_sec_.reset();
    node_->yaw_zone_satisfied_ = false;
  }
  void rotationOnlyServiceSweepHazard() {
    // HH_261007 - The tiny authored region covers only the original contact.
    // A cell above the front-right corner is clear for straight translation
    // but touched by positive rotation. Disable only the optional planning
    // margin in this fixture so the cell is not already in its larger extent.
    node_->manual_service_areas_ = {
      {"camping_site_1", {{0.0,0.1},{0.1,0.1},{0.1,0.2},{0.0,0.2}}}};
    node_->motion_cost_stop_config_.lanelet_footprint_enabled = false;
    node_->motion_cost_stop_.setConfig(node_->motion_cost_stop_config_);
    auto g = grid(true);
    g.data[52 * 100 + 51] = 100;
    node_->motion_cost_stop_.setLaneletGrid(g, node_->nowSec());
  }
  void unrelatedMergedStaticObstacle() {
    // HH_261007 - No radar attribution: this independently enabled merged
    // static obstacle is outside the selected authored site and must survive.
    node_->motion_cost_stop_config_.require_dynamic_source = false;
    node_->motion_cost_stop_.setConfig(node_->motion_cost_stop_config_);
    auto merged = grid();
    merged.data[50 * 100 + 58] = 90;
    node_->motion_cost_stop_.setMergedGrid(merged, node_->nowSec());
  }
  void serviceAreaWithInitialOutsideContact(const int new_contact_grid_x = -1) {
    // HH_261007 - One touched cell is inside the authored site; another lies
    // outside it in the front-right half of the body. Reverse can shed that exact
    // outside cell without adding contact, while service cells remain masked.
    serviceArea();
    // HH_261007 - Match the active CARLA physical-only boundary setting;
    // other fixtures continue exercising the optional planning margin.
    node_->motion_cost_stop_config_.lanelet_footprint_enabled = false;
    node_->motion_cost_stop_.setConfig(node_->motion_cost_stop_config_);
    auto lanelet = grid(true);
    lanelet.data[48 * 100 + 50] = 100;  // (0.05, -0.15), outside the site.
    if (new_contact_grid_x == 45)
      lanelet.data[48 * 100 + 49] = 100;  // A second initial cell survives the replacement.
    if (new_contact_grid_x >= 0)
      lanelet.data[48 * 100 + new_contact_grid_x] = 100;
    node_->motion_cost_stop_.setLaneletGrid(lanelet, node_->nowSec());
  }
  void initialServiceUnknown(const bool inside_site) {
    serviceArea();
    auto lanelet = grid(true);
    // Preserve the known site contact so this tests the geographic exception,
    // not merely fallback to an ordinary on-road evaluator.
    lanelet.data[(inside_site ? 51 : 48) * 100 + 51] = -1;
    node_->motion_cost_stop_.setLaneletGrid(lanelet, node_->nowSec());
  }
  void enableDynamicLatch() {
    node_->motion_cost_stop_config_.latch_enabled = true;
    node_->motion_cost_stop_config_.clear_required_s = 60.0;
    node_->motion_cost_stop_.setConfig(node_->motion_cost_stop_config_);
  }
  bool dynamicLatchActive() const { return node_->motion_cost_stop_.latched(); }
  std::string dynamicLatchReason() const { return node_->motion_cost_stop_.latchReason(); }
  std::string manualServiceBlockReason() const { return node_->manual_service_area_block_reason_; }
  void replayServiceAreaSnapshot(const std::string &path) {
    // HH_261007 - Explicit offline geometry replay, never a motion command or
    // live-obstacle qualification. Every candidate receives an isolated copy.
    const auto snapshot = YAML::LoadFile(path);
    const auto source = snapshot["grid"];
    const auto saved_pose = snapshot["pose"];
    avg_msgs::msg::AvgOccupancyGrid lanelet;
    lanelet.header.frame_id = source["frame"].as<std::string>();
    lanelet.info.resolution = source["resolution"].as<double>();
    lanelet.info.width = source["width"].as<unsigned>();
    lanelet.info.height = source["height"].as<unsigned>();
    lanelet.info.origin.position.x = source["origin"]["x"].as<double>();
    lanelet.info.origin.position.y = source["origin"]["y"].as<double>();
    const double origin_yaw = source["origin"]["yaw"].as<double>();
    lanelet.info.origin.orientation.z = std::sin(origin_yaw * 0.5);
    lanelet.info.origin.orientation.w = std::cos(origin_yaw * 0.5);
    for (const auto &value : source["data"]) lanelet.data.push_back(value.as<int>());
    ASSERT_GT(lanelet.info.resolution, 0.0);
    ASSERT_GT(lanelet.info.width, 0U);
    ASSERT_GT(lanelet.info.height, 0U);
    ASSERT_EQ(lanelet.data.size(),
              static_cast<std::size_t>(lanelet.info.width) * lanelet.info.height);
    std::vector<ManualServiceArea> areas;
    const auto authored = YAML::LoadFile(snapshot["areas_file"].as<std::string>());
    ASSERT_TRUE(authored["camping_sites"].IsSequence());
    std::set<std::string> ids;
    for (const auto &site : authored["camping_sites"]) {
      ManualServiceArea area;
      area.id = site["type"].as<std::string>();
      for (const auto &point : site["corners"])
        area.polygon.emplace_back(point["x"].as<double>(), point["y"].as<double>());
      ASSERT_TRUE(validManualServiceArea(area)) << area.id;
      ASSERT_TRUE(ids.insert(area.id).second) << area.id;
      if (site["frame_id"]) {
        ASSERT_EQ(site["frame_id"].as<std::string>(), lanelet.header.frame_id);
      }
      areas.push_back(std::move(area));
    }
    ASSERT_FALSE(areas.empty());
    if (authored["frame_id"]) {
      ASSERT_EQ(authored["frame_id"].as<std::string>(), lanelet.header.frame_id);
    }
    const std::string pose_frame = saved_pose["frame"].as<std::string>();
    ASSERT_EQ(pose_frame, lanelet.header.frame_id);
    auto config = node_->motion_cost_stop_config_;
    config.tapered_rounded_boundary_enabled = true;
    config.body_front_m = 0.70837;
    config.body_rear_m = 0.68323;
    config.body_left_m = 0.53505;
    config.body_right_m = 0.53495;
    config.lanelet_footprint_enabled = false;
    config.require_dynamic_source = true;
    config.lanelet_check_reverse = true;
    config.latch_enabled = true;
    const double now = node_->nowSec();
    MotionCostStop baseline(config);
    baseline.setPose(PlanarPose{saved_pose["x"].as<double>(),
      saved_pose["y"].as<double>(), saved_pose["yaw"].as<double>(),
      pose_frame, "offline_manual_service_snapshot", now});
    baseline.setLaneletGrid(lanelet, now);
    auto empty_dynamic = lanelet;
    std::fill(empty_dynamic.data.begin(), empty_dynamic.data.end(), 0);
    baseline.setMergedGrid(empty_dynamic, now);
    baseline.setSourceGrid("radar", empty_dynamic, now);
    const std::vector<std::pair<std::string, avg_msgs::msg::AvgTwist>> candidates = {
      {"forward", twist(0.2)}, {"reverse", twist(-0.2)},
      {"left", twist(0, 0.2)}, {"right", twist(0, -0.2)},
      {"yaw_left", twist(0, 0, 0.2)}, {"yaw_right", twist(0, 0, -0.2)}};
    int accepted = 0;
    std::cout << "HH_261007 manual_service_snapshot geometric_only=true "
                 "live_obstacles=not_sampled command_not_published=true path="
              << path << '\n';
    for (const auto &candidate : candidates) {
      MotionCostStop probe = baseline;
      const auto service = probe.evaluateManualServiceAreaCommand(
          candidate.second, now, areas, pose_frame);
      const auto decision = service ? *service : probe.evaluate(candidate.second, now);
      if (!decision.blocked) ++accepted;
      std::cout << candidate.first << " blocked=" << decision.blocked
                << " reason=" << decision.reason << '\n';
    }
    EXPECT_GT(accepted, 0) << "The captured geometry admits no tested direction";
  }
  void finishClearWindow() {
    // HH_261007 - Use real elapsed time and continuously refreshed evidence;
    // a fabricated future probe would accidentally test stale-pose rejection.
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (hold() && std::chrono::steady_clock::now() < end) {
      evidence(false, false, -0.3);
      node_->refreshRouteSafetyRecovery(node_->nowSec());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  std::shared_ptr<CmdVelSafetyGateNode> node_;
  rclcpp::Node::SharedPtr reader_;
  rclcpp::Subscription<avg_msgs::msg::AvgTwist>::SharedPtr sub_;
  std::vector<avg_msgs::msg::AvgTwist> outputs_;
};

TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualTriggerEscapesWithoutAutonomousPath) {
  trigger(); command(twist(0, -0.1)); expectCrab();
  EXPECT_TRUE(hold()); EXPECT_TRUE(handoffReady());
  timer(); EXPECT_TRUE(outputs_.empty());  // No asynchronous final zero.
  command(twist(0, -0.1)); expectCrab();
}

TEST_F(CmdVelSafetyGateManualRecoveryTest, CancelledSiteManualForwardReleasesOnlyMapHold) {
  trigger(); serviceArea(); command(twist(2));
  ASSERT_FALSE(outputs_.empty()); EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 2);
  EXPECT_FALSE(hold()); timer(); EXPECT_TRUE(outputs_.empty());
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, RestartIdleManualCrabAndRotationUseAuthoredArea) {
  serviceArea(); command(twist(0,-0.1)); expectCrab();
  command(twist(0,0,0.2)); ASSERT_FALSE(outputs_.empty());
  EXPECT_DOUBLE_EQ(outputs_.back().angular.z, 0.2);
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServicePreservesFixedYawLockOutput) {
  serviceArea(); fixedYawLock(); command(twist(2));
  ASSERT_FALSE(outputs_.empty());
  for (const auto &output : outputs_) {
    EXPECT_DOUBLE_EQ(output.linear.x, 0);
    EXPECT_DOUBLE_EQ(output.linear.y, 0);
    EXPECT_DOUBLE_EQ(output.angular.z, 0.2);
  }
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceProjectsFixedYawTransformedCommand) {
  serviceArea(); rotationOnlyServiceSweepHazard(); command(twist(2));
  ASSERT_FALSE(outputs_.empty());
  ASSERT_DOUBLE_EQ(outputs_.back().linear.x, 2);
  // The same original input is no longer safe after the fixed-zone transform.
  fixedYawLock(); command(twist(2)); expectZero();
  EXPECT_EQ(manualServiceBlockReason(),
            "manual_service_area_sweep_outside:camping_site_1");
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ServiceLimitDoesNotCapClearRoadManualRotation) {
  serviceArea(); evidence(false); command(twist(0, 0, 0.3));
  ASSERT_FALSE(outputs_.empty());
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0);
  EXPECT_DOUBLE_EQ(outputs_.back().linear.y, 0);
  EXPECT_DOUBLE_EQ(outputs_.back().angular.z, 0.3);
  EXPECT_TRUE(manualServiceBlockReason().empty());
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ClearBodyRotationReleasesRearHoldButNotReverse) {
  serviceArea(); rearLookaheadHazard(); command(twist(-0.2));
  ASSERT_TRUE(hold()); expectZero();
  command(twist(-0.2)); expectZero(); EXPECT_TRUE(hold());
  command(twist(0, 0, -0.2));
  ASSERT_FALSE(outputs_.empty());
  EXPECT_DOUBLE_EQ(outputs_.back().angular.z, -0.2);
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0);
  EXPECT_FALSE(hold());
  command(twist(-0.2)); expectZero(); EXPECT_TRUE(hold());
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceKeepsUnrelatedMergedStaticObstacle) {
  serviceArea(); unrelatedMergedStaticObstacle(); command(twist(2)); expectZero();
  EXPECT_EQ(manualServiceBlockReason(), "merged_front");
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceCannotReleaseActualDynamicLatch) {
  serviceArea(); enableDynamicLatch(); evidence(true, true);
  command(twist(0, -0.1)); expectZero();
  ASSERT_TRUE(dynamicLatchActive());
  const auto original_reason = dynamicLatchReason();
  ASSERT_EQ(original_reason, "dynamic_right:radar");
  // HH_261007 - Neither a different permitted direction nor one fresh clear
  // frame may replace the saved hazard's ordinary continuous-clear proof.
  command(twist(2)); expectZero();
  EXPECT_TRUE(dynamicLatchActive());
  EXPECT_EQ(dynamicLatchReason(), original_reason);
  evidence(true, false); command(twist(2)); expectZero();
  EXPECT_TRUE(dynamicLatchActive());
  EXPECT_EQ(dynamicLatchReason(), original_reason);
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceAllowsMonotonicInitialOutsideEscape) {
  serviceAreaWithInitialOutsideContact(); command(twist(-0.2));
  ASSERT_FALSE(outputs_.empty());
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, -0.2);
  EXPECT_DOUBLE_EQ(outputs_.back().linear.y, 0);
  EXPECT_DOUBLE_EQ(outputs_.back().angular.z, 0);
  EXPECT_TRUE(manualServiceBlockReason().empty());
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualEscapeContinuesAfterLeavingSitePolygon) {
  serviceAreaWithInitialOutsideContact(); moveServiceAreaAway();
  command(twist(-0.2)); ASSERT_FALSE(outputs_.empty());
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, -0.2);
  EXPECT_TRUE(manualServiceBlockReason().empty());
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceRejectsIncreasingOutsideContact) {
  // At x=-0.35 the rear starts touching a second outside cell before the
  // original front-right contact is shed; an unchanged direction is not proof.
  serviceAreaWithInitialOutsideContact(46); command(twist(-0.2)); expectZero();
  EXPECT_EQ(manualServiceBlockReason(),
            "manual_service_area_sweep_outside:camping_site_1");
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceRejectsReplacementOutsideContact) {
  // HH_261007 - At x=-0.45 a new cell replaces the frontmost original cell
  // while a second initial cell remains touched: no first-clear endpoint has
  // occurred. Count need not grow, but subset identity must still reject it.
  serviceAreaWithInitialOutsideContact(45); command(twist(-0.2)); expectZero();
  EXPECT_EQ(manualServiceBlockReason(),
            "manual_service_area_sweep_outside:camping_site_1");
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceRejectsInitialUnknownOutsideSite) {
  initialServiceUnknown(false); command(twist(-0.2)); expectZero();
  EXPECT_EQ(manualServiceBlockReason().find("manual_service_area_"), 0U);
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceNeverMasksInitialUnknownInsideSite) {
  initialServiceUnknown(true); command(twist(-0.2)); expectZero();
  EXPECT_EQ(manualServiceBlockReason().find("manual_service_area_"), 0U);
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, OptionalAuthoredMapSnapshotGeometryReplay) {
  const char *snapshot = std::getenv("CAMROD_MANUAL_AREA_SNAPSHOT");
  if (snapshot == nullptr || *snapshot == '\0') {
    GTEST_SKIP() << "Set CAMROD_MANUAL_AREA_SNAPSHOT for read-only captured-map replay";
  }
  replayServiceAreaSnapshot(snapshot);
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceSweepCannotCrossAnotherLethalCell) {
  serviceArea(); serviceSweepHazard(100); command(twist(2)); expectZero();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceSweepCannotCrossUnknownCell) {
  serviceArea(); serviceSweepHazard(-1); command(twist(2)); expectZero();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServicePermissionDoesNotChangeRawMap) {
  serviceArea(); command(twist(2)); ASSERT_FALSE(outputs_.empty());
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 2);
  disableServiceArea(); command(twist(2)); expectZero(); EXPECT_TRUE(hold());
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceStillStopsOnRadar) {
  serviceArea(); evidence(true,true); command(twist(0,-0.1)); expectZero();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceRejectsStalePoseAndGrid) {
  serviceArea(); stalePose(); command(twist(2)); expectZero();
  evidence(); staleLanelet(); command(twist(2)); expectZero();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceRejectsFrameMismatchAndAmbiguity) {
  serviceArea(); serviceFrameMismatch(); command(twist(2)); expectZero();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceRejectsAmbiguousAreas) {
  serviceArea(); secondServiceArea(); command(twist(2)); expectZero();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceCannotOverrideEstop) {
  serviceArea(); estop(); command(twist(2)); expectZero();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceCannotOverridePlatformDisable) {
  serviceArea(); platformEnable(false); command(twist(2)); expectZero();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceNotAvailableToAutonomousOwner) {
  serviceArea(); navigationOwner(); command(twist(2),CommandInputSource::kNavigation); expectZero();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, AccessSelectsForwardReverseAndCrabFromClearRoad) {
  for (const int direction : {0,1,2}) {
    SCOPED_TRACE(direction);
    accessApproach(direction);
    const auto requested = accessCommand(direction);
    command(requested);
    ASSERT_FALSE(outputs_.empty());
    EXPECT_DOUBLE_EQ(outputs_.back().linear.x, requested.linear.x);
    EXPECT_DOUBLE_EQ(outputs_.back().linear.y, requested.linear.y);
    EXPECT_EQ(serviceReason(), "manual_service_area:camping_site_1");
    EXPECT_EQ(campsitePhase(), "");  // No fabricated maneuver phase.
    EXPECT_FALSE(hold());
  }
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, WithoutAccessGapContactStillStopsAllDirections) {
  for (const int direction : {0,1,2}) {
    SCOPED_TRACE(direction);
    accessApproach(direction, false);
    // Legacy no-access behavior does not claim a clear-road service permit.
    EXPECT_FALSE(scopedAccessDecision(accessCommand(direction)));
    accessApproach(direction, false, .35);
    command(accessCommand(direction)); expectZero();
  }
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, AccessRejectsSweptBodyOutsideExactUnion) {
  accessApproach(0);
  narrowAccess();
  command(twist(2)); expectZero();
  EXPECT_NE(manualServiceBlockReason().find("sweep_outside"), std::string::npos);
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, AccessRejectsUnknownAndAmbiguousSweep) {
  accessApproach(0); accessUnknown(); command(twist(2)); expectZero();
  EXPECT_NE(manualServiceBlockReason().find("unknown"), std::string::npos);
  accessApproach(0); secondServiceArea(); command(twist(2)); expectZero();
  EXPECT_EQ(manualServiceBlockReason(), "manual_service_area_ambiguous");
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, AccessCannotJumpToSecondDisjointSite) {
  accessApproach(0);
  secondDisjointAccessSite();
  command(twist(2)); expectZero();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, AccessKeepsFreshnessFrameAndInterlocks) {
  accessApproach(0); stalePose(); command(twist(2)); expectZero();
  accessApproach(0); staleLanelet(); command(twist(2)); expectZero();
  accessApproach(0); serviceFrameMismatch(); command(twist(2)); expectZero();
  restoreServiceFrame();
  accessApproach(0); platformEnable(false); command(twist(2)); expectZero();
  platformEnable(true); estop(); command(twist(2)); expectZero();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, AccessCannotClearLiveDynamicLatch) {
  accessApproach(0); enableDynamicLatch(); accessRadar();
  command(twist(2)); expectZero(); ASSERT_TRUE(dynamicLatchActive());
  accessApproach(0); command(twist(2)); expectZero(); EXPECT_TRUE(dynamicLatchActive());
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, AccessDoesNotAuthorizeNavigationOrRawOwner) {
  accessApproach(0, true, .35); navigationOwner();
  command(twist(2),CommandInputSource::kNavigation); expectZero();
  command(twist(2),CommandInputSource::kRaw); expectZero();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualServiceDisarmAndWatchdogKeepZero) {
  serviceArea(); command(twist(2)); expireCommandWatchdog(); expectZero();
  // HH_261007 - Observe the mandatory disarm zero before testing that a later
  // unauthorized command produces no new output; DDS delivery is asynchronous.
  releaseOwner(); drain(); expectZero();
  command(twist(2)); EXPECT_TRUE(outputs_.empty());
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, OutwardAndRotationOnlyRemainBlocked) {
  trigger(); command(twist(0, 0.1)); expectZero();
  command(twist(0, 0, 0.2)); expectZero();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ActualRadarObstacleStillStopsEscape) {
  trigger(); evidence(true, true); command(twist(0, -0.1)); expectZero();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, StalePoseStillStopsEscape) {
  trigger(); stalePose(); command(twist(0, -0.1)); expectZero();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, StaleLaneletStillStopsEscape) {
  trigger(); staleLanelet(); command(twist(0, -0.1)); expectZero();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, EstopAndPlatformPermissionRemainAuthoritative) {
  trigger(); platformEnable(false); command(twist(0, -0.1)); expectZero();
  platformEnable(true); command(twist(0, -0.1)); expectCrab();
  estop(); command(twist(0, -0.1)); expectZero();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, RawControllerCannotOverwriteManualOwner) {
  trigger(); command(twist(0, -0.1)); expectCrab();
  command(twist(0, 0.1), CommandInputSource::kRaw); EXPECT_TRUE(outputs_.empty());
  releaseOwner(); drain(); expectZero();
  command(twist(0, -0.1)); EXPECT_TRUE(outputs_.empty());
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, AutonomousRawStillRequiresExactCandidate) {
  navigationOwner(); command(twist(0, 0, -0.2), CommandInputSource::kNavigation);
  EXPECT_TRUE(hold());
  command(twist(0, -0.1), CommandInputSource::kRaw); expectZero();
  EXPECT_FALSE(handoffReady());
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, GenericManualRecoveryRemainsCompatible) {
  genericMode(); trigger(); command(twist(0, -0.1)); expectCrab();
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ClearSpaceRetainsTwoMetersPerSecond) {
  evidence(false); command(twist(2.0));
  ASSERT_FALSE(outputs_.empty()); EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 2.0);
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, ManualHandoffNeedsMotionThenFreshContinuousClear) {
  trigger(); command(twist(0, -0.1)); expectCrab();
  evidence(false, false, -0.3); finishClearWindow(); EXPECT_FALSE(hold());
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, FreshClearWithoutAcceptedMotionCannotReleaseHold) {
  trigger(); finishClearWindow(); EXPECT_TRUE(hold());
}
TEST_F(CmdVelSafetyGateManualRecoveryTest, StaleManualHeartbeatStillPublishesZero) {
  trigger(); command(twist(0, -0.1)); expectCrab();
  expireCommandWatchdog(); expectZero();
}
}  // namespace camrod_control
