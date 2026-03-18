#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <avg_msgs/msg/goal_status_array.hpp>
#include <avg_msgs/msg/avg_planning_msgs.hpp>
#include <avg_msgs/msg/module_health.hpp>
#include <avg_msgs/msg/pose_stamped.hpp>
#include <avg_msgs/msg/path.hpp>
#include <avg_msgs/action/compute_path_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace camrod_planning
{

class GoalReplannerNode : public rclcpp::Node
{
public:
  using AvgPlanningMsgs = avg_msgs::msg::AvgPlanningMsgs;
  using ModuleHealth = avg_msgs::msg::ModuleHealth;
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandleComputePathToPose = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

  // Implements `GoalReplannerNode` behavior.
  GoalReplannerNode()
  : rclcpp::Node("goal_replanner")
  {
    enabled_ = declare_parameter<bool>("enabled", true);
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/planning/goal_pose");
    start_topic_ = declare_parameter<std::string>("start_topic", "/planning/lanelet_pose");
    // 2026-02-25: clearer start-source selector with legacy compatibility.
    // Preferred:
    //   planning_start_source:
    //     - "robot_base_link"  : use map->robot_base_link TF as start
    //     - "lanelet_pose"     : use /planning/lanelet_pose as explicit start
    //     - "localization_pose": use /localization/pose as explicit start
    //     - "start_topic"      : use start_topic param as explicit start
    //   start_topic_fallback_to_tf:
    //     - true: if topic start is missing, fallback to TF start
    //     - false: wait until topic start is received
    //
    // Compatibility:
    //   start_reference, start_mode, start_source, use_start_pose, use_topic_start_pose
    // Legacy:
    //   start_source: "tf_robot_base_link" | "topic_pose"
    //   use_start_pose / use_topic_start_pose (bool)
    const bool legacy_use_start_pose = declare_parameter<bool>("use_start_pose", false);
    const bool legacy_use_topic_start_pose =
      declare_parameter<bool>("use_topic_start_pose", legacy_use_start_pose);
    const std::string legacy_start_source = declare_parameter<std::string>(
      "start_source",
      legacy_use_topic_start_pose ? "topic_pose" : "tf_robot_base_link");
    const std::string start_mode = declare_parameter<std::string>(
      "start_mode",
      (legacy_start_source == "topic_pose" || legacy_use_topic_start_pose) ?
      "start_topic_pose" : "robot_base_link_tf");
    const std::string start_reference = declare_parameter<std::string>("start_reference", "");
    const std::string planning_start_source = declare_parameter<std::string>(
      "planning_start_source", "");
    const std::string selected_mode = !planning_start_source.empty() ?
      planning_start_source :
      (start_reference.empty() ? start_mode : start_reference);
    start_topic_fallback_to_tf_ =
      declare_parameter<bool>("start_topic_fallback_to_tf", true);

    if (
      selected_mode == "start_topic_pose" || selected_mode == "topic_pose" ||
      selected_mode == "topic_start_pose" || selected_mode == "start_topic" ||
      selected_mode == "lanelet_pose_topic" || selected_mode == "localization_pose_topic" ||
      selected_mode == "lanelet_pose" || selected_mode == "localization_pose")
    {
      use_topic_start_pose_ = true;
      if (selected_mode == "lanelet_pose") {
        start_topic_ = "/planning/lanelet_pose";
      } else if (selected_mode == "localization_pose") {
        start_topic_ = "/localization/pose";
      }
    } else if (
      selected_mode == "robot_base_link_tf" || selected_mode == "tf_robot_base_link" ||
      selected_mode == "robot_tf" || selected_mode == "tf" ||
      selected_mode == "robot_base_link" || selected_mode == "start_from_tf")
    {
      use_topic_start_pose_ = false;
    } else {
      if (legacy_start_source == "topic_pose") {
        use_topic_start_pose_ = true;
      } else if (legacy_start_source == "tf_robot_base_link") {
        use_topic_start_pose_ = false;
      } else {
        use_topic_start_pose_ = legacy_use_topic_start_pose;
      }
      RCLCPP_WARN(get_logger(),
        "Unknown start mode '%s'. Use one of: "
        "robot_base_link_tf | robot_base_link | start_topic_pose | start_topic. "
        "Fallback legacy start_source='%s' -> topic_start=%s.",
        selected_mode.c_str(), legacy_start_source.c_str(),
        use_topic_start_pose_ ? "true" : "false");
    }
    action_name_ = declare_parameter<std::string>("action_name", "/planning/compute_path_to_pose");
    planner_id_ = declare_parameter<std::string>("planner_id", "Smac2D");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    output_path_topic_ = declare_parameter<std::string>(
      "output_path_topic", "/planning/global_path");
    publish_result_path_ = declare_parameter<bool>("publish_result_path", true);
    publish_planning_diagnostic_ = declare_parameter<bool>("publish_planning_diagnostic", false);
    planning_diagnostic_topic_ =
      declare_parameter<std::string>("planning_diagnostic_topic", "/planning/diagnostic");
    enable_periodic_replan_ = declare_parameter<bool>("enable_periodic_replan", false);
    // HH_260309-00:00 Prevent compute-path request storms.
    min_request_interval_sec_ = declare_parameter<double>("min_request_interval_sec", 0.25);
    // HH_260309-00:00 When NavigateToPose BT is running, planner_server already computes paths.
    // Pause this helper to avoid action contention/abort storms.
    pause_when_navigate_active_ = declare_parameter<bool>("pause_when_navigate_active", true);
    navigate_status_topic_ = declare_parameter<std::string>(
      "navigate_status_topic", "/planning/navigate_to_pose/_action/status");
    // HH_260306-00:00 Default to event-driven mode.
    replan_rate_hz_ = declare_parameter<double>("replan_rate_hz", 0.0);
    // HH_260306-00:00 Set <=0 to disable timeout-based cancellation.
    request_timeout_sec_ = declare_parameter<double>("request_timeout_sec", 0.0);
    // HH_260306-00:00 Backoff interval after failed/canceled planner result.
    retry_after_failure_sec_ = declare_parameter<double>("retry_after_failure_sec", 0.8);
    start_replan_distance_ = declare_parameter<double>("start_replan_distance", 0.4);
    goal_replan_distance_ = declare_parameter<double>("goal_replan_distance", 0.1);
    // HH_260309-00:00 Ignore repeated equivalent goal messages (same XY/yaw) to avoid
    // action churn when upstream goal relays republish identical snapped goals.
    ignore_duplicate_goal_messages_ =
      declare_parameter<bool>("ignore_duplicate_goal_messages", true);
    duplicate_goal_xy_epsilon_m_ =
      declare_parameter<double>("duplicate_goal_xy_epsilon_m", 0.05);
    duplicate_goal_yaw_epsilon_deg_ =
      declare_parameter<double>("duplicate_goal_yaw_epsilon_deg", 4.0);
    // HH_260309-00:00 Debounce transient inactive blips from navigate status topic.
    navigate_inactive_grace_sec_ =
      declare_parameter<double>("navigate_inactive_grace_sec", 0.8);
    start_replan_yaw_deg_ = declare_parameter<double>("start_replan_yaw_deg", 10.0);
    replan_on_start_change_ = declare_parameter<bool>("replan_on_start_change", false);
    // HH_260306-00:00 Stop replanning after reaching goal until a new goal is received.
    stop_replan_after_goal_reached_ =
      declare_parameter<bool>("stop_replan_after_goal_reached", true);
    goal_reached_distance_m_ = declare_parameter<double>("goal_reached_distance_m", 0.8);
    // 2026-02-27: Reduce goal->path latency by triggering immediate request on new goal/start.
    immediate_replan_on_goal_ = declare_parameter<bool>("immediate_replan_on_goal", true);
    immediate_replan_on_start_ = declare_parameter<bool>("immediate_replan_on_start", true);

    action_client_ = rclcpp_action::create_client<ComputePathToPose>(this, action_name_);
    if (publish_result_path_) {
      path_pub_ = create_publisher<avg_msgs::msg::Path>(
        output_path_topic_, rclcpp::QoS(1).transient_local().reliable());
    }
    if (publish_planning_diagnostic_) {
      pub_avg_planning_ = create_publisher<AvgPlanningMsgs>(
        planning_diagnostic_topic_, rclcpp::QoS(10));
    }

    sub_goal_ = create_subscription<avg_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&GoalReplannerNode::onGoal, this, std::placeholders::_1));
    if (use_topic_start_pose_ || replan_on_start_change_) {
      sub_start_ = create_subscription<avg_msgs::msg::PoseStamped>(
        start_topic_, rclcpp::SystemDefaultsQoS(),
        std::bind(&GoalReplannerNode::onStart, this, std::placeholders::_1));
    }
    if (pause_when_navigate_active_) {
      sub_nav_status_ = create_subscription<avg_msgs::msg::GoalStatusArray>(
        navigate_status_topic_, rclcpp::QoS(10).reliable(),
        std::bind(&GoalReplannerNode::onNavigateStatus, this, std::placeholders::_1));
    }

    if (enable_periodic_replan_ && replan_rate_hz_ > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / replan_rate_hz_);
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&GoalReplannerNode::onTimer, this));
    } else if (!enable_periodic_replan_) {
      RCLCPP_INFO(get_logger(), "periodic replanning disabled (goal/start event driven mode)");
    } else {
      RCLCPP_WARN(get_logger(), "replan_rate_hz <= 0.0, auto replanning timer disabled");
    }

    RCLCPP_INFO(
      get_logger(),
      "Goal replanner: enabled=%s, goal=%s, start=%s, action=%s, planner_id=%s",
      enabled_ ? "true" : "false", goal_topic_.c_str(), start_topic_.c_str(),
      action_name_.c_str(), planner_id_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "replan config: rate=%.2fHz timeout=%.2fs retry_after_failure=%.2fs "
      "replan_on_start_change=%s immediate(goal/start)=%s/%s pause_on_nav=%s min_req_int=%.2fs",
      replan_rate_hz_, request_timeout_sec_, retry_after_failure_sec_,
      replan_on_start_change_ ? "true" : "false",
      immediate_replan_on_goal_ ? "true" : "false",
      immediate_replan_on_start_ ? "true" : "false",
      pause_when_navigate_active_ ? "true" : "false",
      min_request_interval_sec_);
    if (publish_result_path_) {
      RCLCPP_INFO(
        get_logger(),
        "ComputePath result path publish enabled: %s",
        output_path_topic_.c_str());
    }
    if (!use_topic_start_pose_) {
      RCLCPP_INFO(
        get_logger(),
        "start_mode=robot_base_link_tf: ComputePathToPose uses map->robot_base_link as start");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "start_mode=start_topic_pose: ComputePathToPose uses %s as explicit start pose%s",
        start_topic_.c_str(),
        start_topic_fallback_to_tf_ ? " (fallback-to-tf enabled)" : "");
    }
  }

private:
  // Implements `normalizeAngle` behavior.
  static double normalizeAngle(double a)
  {
    while (a > M_PI) {
      a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
      a += 2.0 * M_PI;
    }
    return a;
  }

  // Implements `distance2D` behavior.
  static double distance2D(
    const avg_msgs::msg::PoseStamped & a,
    const avg_msgs::msg::PoseStamped & b)
  {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    return std::hypot(dx, dy);
  }

  // Implements `yawDiffRad` behavior.
  static double yawDiffRad(
    const avg_msgs::msg::PoseStamped & a,
    const avg_msgs::msg::PoseStamped & b)
  {
    const auto & qa = a.pose.orientation;
    const auto & qb = b.pose.orientation;
    const double ya = std::atan2(
      2.0 * (qa.w * qa.z + qa.x * qa.y),
      1.0 - 2.0 * (qa.y * qa.y + qa.z * qa.z));
    const double yb = std::atan2(
      2.0 * (qb.w * qb.z + qb.x * qb.y),
      1.0 - 2.0 * (qb.y * qb.y + qb.z * qb.z));
    return std::abs(normalizeAngle(ya - yb));
  }

  // Handles the `onNavigateStatus` callback.
  void onNavigateStatus(const avg_msgs::msg::GoalStatusArray::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }
    publishAvgPlanning(std::nullopt, *msg, std::nullopt);
    bool active = false;
    for (const auto & st : msg->status_list) {
      // action_msgs/GoalStatus:
      // 1 ACCEPTED, 2 EXECUTING, 3 CANCELING
      if (st.status == 1 || st.status == 2 || st.status == 3) {
        active = true;
        break;
      }
    }
    if (active) {
      last_navigate_active_time_ = now();
    } else if (
      navigate_active_ && navigate_inactive_grace_sec_ > 0.0 &&
      last_navigate_active_time_.nanoseconds() > 0 &&
      (now() - last_navigate_active_time_).seconds() < navigate_inactive_grace_sec_)
    {
      return;
    }
    if (navigate_active_ == active) {
      return;
    }
    navigate_active_ = active;
    if (navigate_active_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "navigate_to_pose active -> goal_replanner paused");
      return;
    }
    RCLCPP_INFO(get_logger(), "navigate_to_pose inactive -> goal_replanner resumed");
  }

  // Checks `shouldReplan` condition.
  bool shouldReplan() const
  {
    if (goal_reached_latched_) {
      return false;
    }
    if (!has_goal_) {
      return false;
    }
    if ((use_topic_start_pose_ || replan_on_start_change_) &&
      !has_start_ && !start_topic_fallback_to_tf_)
    {
      return false;
    }
    if (!has_last_submitted_) {
      return true;
    }
    if (goal_msg_counter_ != last_submitted_goal_counter_) {
      return true;
    }

    if (distance2D(latest_goal_, last_submitted_goal_) > goal_replan_distance_) {
      return true;
    }
    if (replan_on_start_change_) {
      if (distance2D(latest_start_, last_submitted_start_) > start_replan_distance_) {
        return true;
      }
      const double yaw_thresh = start_replan_yaw_deg_ * M_PI / 180.0;
      if (yawDiffRad(latest_start_, last_submitted_start_) > yaw_thresh) {
        return true;
      }
    }
    return false;
  }

  // Checks `isGoalReachedByStartPose` condition.
  bool isGoalReachedByStartPose() const
  {
    if (!use_topic_start_pose_ || !has_start_ || !has_goal_) {
      return false;
    }
    return distance2D(latest_start_, latest_goal_) <= goal_reached_distance_m_;
  }

  // Updates `GoalReachedLatch` state.
  void updateGoalReachedLatch()
  {
    if (!stop_replan_after_goal_reached_) {
      return;
    }
    if (goal_reached_latched_) {
      return;
    }
    if (!isGoalReachedByStartPose()) {
      return;
    }
    goal_reached_latched_ = true;
    if (in_flight_) {
      if (active_goal_handle_) {
        (void)action_client_->async_cancel_goal(active_goal_handle_);
      } else {
        (void)action_client_->async_cancel_all_goals();
      }
      in_flight_ = false;
      active_goal_handle_.reset();
      active_request_seq_ = 0;
    }
    has_last_submitted_ = false;
    RCLCPP_INFO(
      get_logger(),
      "Goal reached latch enabled (dist<=%.2fm). Replan paused until new goal.",
      goal_reached_distance_m_);
  }

  // Handles the `onGoal` callback.
  void onGoal(const avg_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }

    avg_msgs::msg::PoseStamped incoming = *msg;
    if (incoming.header.frame_id.empty()) {
      incoming.header.frame_id = frame_id_;
    }

    if (ignore_duplicate_goal_messages_ && has_goal_) {
      const bool same_frame =
        latest_goal_.header.frame_id.empty() ||
        incoming.header.frame_id.empty() ||
        latest_goal_.header.frame_id == incoming.header.frame_id;
      if (same_frame) {
        const double dxy = distance2D(incoming, latest_goal_);
        const double dyaw_deg = yawDiffRad(incoming, latest_goal_) * 180.0 / M_PI;
        if (
          dxy <= duplicate_goal_xy_epsilon_m_ &&
          dyaw_deg <= duplicate_goal_yaw_epsilon_deg_)
        {
          RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Duplicate goal ignored: dxy=%.3f m dyaw=%.2f deg", dxy, dyaw_deg);
          return;
        }
      }
    }

    latest_goal_ = incoming;
    has_goal_ = true;
    ++goal_msg_counter_;
    publishAvgPlanning(std::nullopt, std::nullopt, latest_goal_);
    RCLCPP_INFO(
      get_logger(),
      "New goal received #%lu: (%.2f, %.2f) frame=%s",
      static_cast<unsigned long>(goal_msg_counter_),
      latest_goal_.pose.position.x, latest_goal_.pose.position.y,
      latest_goal_.header.frame_id.c_str());
    // HH_260306-00:00 New goal always re-enables replanning.
    goal_reached_latched_ = false;
    force_tf_start_once_ = false;
    tf_start_fallback_used_for_goal_ = false;
    next_allowed_request_time_ = now();
    // HH_260306-00:00 New goal must force a fresh request immediately.
    if (in_flight_) {
      if (active_goal_handle_) {
        (void)action_client_->async_cancel_goal(active_goal_handle_);
      } else {
        (void)action_client_->async_cancel_all_goals();
      }
      in_flight_ = false;
      active_goal_handle_.reset();
      active_request_seq_ = 0;
    }
    has_last_submitted_ = false;
    if (pause_when_navigate_active_ && navigate_active_) {
      return;
    }
    if (immediate_replan_on_goal_) {
      onTimer();
    }
  }

  // Handles the `onStart` callback.
  void onStart(const avg_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }
    latest_start_ = *msg;
    if (latest_start_.header.frame_id.empty()) {
      latest_start_.header.frame_id = frame_id_;
    }
    has_start_ = true;
    updateGoalReachedLatch();
    if (pause_when_navigate_active_ && navigate_active_) {
      return;
    }
    if (immediate_replan_on_start_) {
      // HH_260307-00:00 Avoid request storms on high-rate start pose updates.
      // - Always trigger once after startup (first valid start pose)
      // - Trigger on every start update only when start-change replanning is enabled
      // - Do not trigger repeatedly just because last request failed/cleared
      const bool should_trigger_now =
        !has_sent_any_request_ ||
        (replan_on_start_change_ && !in_flight_);
      if (should_trigger_now) {
        onTimer();
      }
    }
  }

  // Handles the `onTimer` callback.
  void onTimer()
  {
    if (!enabled_) {
      return;
    }
    if (!has_goal_) {
      return;
    }
    if (pause_when_navigate_active_ && navigate_active_) {
      return;
    }
    if (use_topic_start_pose_ && !has_start_ && !start_topic_fallback_to_tf_) {
      return;
    }

    updateGoalReachedLatch();
    if (goal_reached_latched_) {
      return;
    }

    if (
      next_allowed_request_time_.nanoseconds() > 0 &&
      now() < next_allowed_request_time_)
    {
      return;
    }
    if (min_request_interval_sec_ > 0.0 && last_request_sent_time_.nanoseconds() > 0) {
      const double dt_req = (now() - last_request_sent_time_).seconds();
      if (dt_req < min_request_interval_sec_) {
        return;
      }
    }

    if (in_flight_) {
      if (shouldReplan()) {
        if (active_goal_handle_) {
          (void)action_client_->async_cancel_goal(active_goal_handle_);
        } else {
          (void)action_client_->async_cancel_all_goals();
        }
        in_flight_ = false;
        active_goal_handle_.reset();
        active_request_seq_ = 0;
        has_last_submitted_ = false;
        RCLCPP_INFO(get_logger(), "Canceled in-flight plan request due to updated goal/start");
      } else {
        if (request_timeout_sec_ <= 0.0) {
          return;
        }
        const auto dt = (now() - request_sent_time_).seconds();
        if (dt <= request_timeout_sec_) {
          return;
        }

        if (active_goal_handle_) {
          (void)action_client_->async_cancel_goal(active_goal_handle_);
        } else {
          (void)action_client_->async_cancel_all_goals();
        }
        in_flight_ = false;
        active_goal_handle_.reset();
        active_request_seq_ = 0;
        has_last_submitted_ = false;
        RCLCPP_WARN(get_logger(), "Replan request timed out (%.2fs), canceled pending goal", dt);
      }
    }

    if (!shouldReplan()) {
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(0))) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for action server %s", action_name_.c_str());
      return;
    }

    ComputePathToPose::Goal goal_msg;
    goal_msg.use_start = use_topic_start_pose_ && has_start_;
    if (goal_msg.use_start && force_tf_start_once_) {
      goal_msg.use_start = false;
    }
    if (use_topic_start_pose_ && !has_start_ && start_topic_fallback_to_tf_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "start_topic has no message yet; fallback to TF start (map->robot_base_link)");
    }
    if (goal_msg.use_start) {
      goal_msg.start = latest_start_;
    }
    goal_msg.goal = latest_goal_;
    goal_msg.planner_id = planner_id_;
    if (goal_msg.use_start) {
      goal_msg.start.header.frame_id = goal_msg.start.header.frame_id.empty()
        ? frame_id_ : goal_msg.start.header.frame_id;
    }
    goal_msg.goal.header.frame_id = goal_msg.goal.header.frame_id.empty()
      ? frame_id_ : goal_msg.goal.header.frame_id;

    const uint64_t req_seq = ++request_seq_counter_;
    active_request_seq_ = req_seq;

    auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this, req_seq](GoalHandleComputePathToPose::SharedPtr goal_handle) {
        if (req_seq != active_request_seq_) {
          return;
        }
        if (!goal_handle) {
          in_flight_ = false;
          active_goal_handle_.reset();
          active_request_seq_ = 0;
          has_last_submitted_ = false;
          RCLCPP_WARN(get_logger(), "ComputePathToPose goal rejected");
          return;
        }
        active_goal_handle_ = goal_handle;
      };
    send_goal_options.result_callback =
      [this, req_seq](const GoalHandleComputePathToPose::WrappedResult & result) {
        if (req_seq != active_request_seq_) {
          return;
        }
        in_flight_ = false;
        active_goal_handle_.reset();
        active_request_seq_ = 0;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          if (!result.result || result.result->path.poses.empty()) {
            has_last_submitted_ = false;
            RCLCPP_WARN(
              get_logger(),
              "ComputePathToPose returned success but empty path. Will retry on next cycle.");
            return;
          }
          if (publish_result_path_ && path_pub_) {
            auto out_path = result.result->path;
            if (out_path.header.frame_id.empty()) {
              out_path.header.frame_id = frame_id_;
            }
            out_path.header.stamp = now();
            path_pub_->publish(out_path);
            publishAvgPlanning(out_path, std::nullopt, std::nullopt);
          }
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "ComputePathToPose success: path size=%zu",
            result.result->path.poses.size());
          force_tf_start_once_ = false;
          tf_start_fallback_used_for_goal_ = false;
          return;
        }
        // HH_260306-00:00 If explicit topic-start is rejected (often due lethal/unknown start cell),
        // retry once using TF start (map->robot_base_link) for this goal.
        if (use_topic_start_pose_ && !last_request_used_tf_start_ &&
          !tf_start_fallback_used_for_goal_ && has_goal_)
        {
          force_tf_start_once_ = true;
          tf_start_fallback_used_for_goal_ = true;
          has_last_submitted_ = false;
          next_allowed_request_time_ = now();
          RCLCPP_WARN(
            get_logger(),
            "ComputePathToPose failed with topic-start (code=%d). Retrying once with TF start.",
            static_cast<int>(result.code));
          onTimer();
          return;
        }
        // HH_260309-00:00 In goal-event mode, do not auto-retry forever on the same failed goal.
        // Wait for a truly new goal (or explicit start-change replanning mode) to avoid action storms.
        const bool allow_automatic_retry = enable_periodic_replan_ || replan_on_start_change_;
        if (!allow_automatic_retry) {
          has_last_submitted_ = true;
          RCLCPP_WARN(
            get_logger(),
            "ComputePathToPose finished with code %d. Waiting for a new goal to replan.",
            static_cast<int>(result.code));
          return;
        }
        has_last_submitted_ = false;
        if (retry_after_failure_sec_ > 0.0) {
          next_allowed_request_time_ = now() + rclcpp::Duration::from_seconds(retry_after_failure_sec_);
        }
        RCLCPP_WARN(
          get_logger(), "ComputePathToPose finished with code %d",
          static_cast<int>(result.code));
      };

    request_sent_time_ = now();
    last_request_sent_time_ = request_sent_time_;
    in_flight_ = true;
    if (goal_msg.use_start) {
      last_submitted_start_ = latest_start_;
    }
    last_request_used_tf_start_ = !goal_msg.use_start;
    force_tf_start_once_ = false;
    last_submitted_goal_ = latest_goal_;
    has_last_submitted_ = true;
    has_sent_any_request_ = true;
    last_submitted_goal_counter_ = goal_msg_counter_;
    if (goal_msg.use_start) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Replan request: start(%.2f, %.2f) -> goal(%.2f, %.2f), planner=%s",
        goal_msg.start.pose.position.x, goal_msg.start.pose.position.y,
        goal_msg.goal.pose.position.x, goal_msg.goal.pose.position.y,
        goal_msg.planner_id.c_str());
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Replan request: start(tf:map->robot_base_link) -> goal(%.2f, %.2f), planner=%s",
        goal_msg.goal.pose.position.x, goal_msg.goal.pose.position.y,
        goal_msg.planner_id.c_str());
    }
    (void)action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  // Publishes `AvgPlanning` output.
  void publishAvgPlanning(
    const std::optional<avg_msgs::msg::Path> & global_path,
    const std::optional<avg_msgs::msg::GoalStatusArray> & navigate_status,
    const std::optional<avg_msgs::msg::PoseStamped> & goal_pose)
  {
    if (!publish_planning_diagnostic_ || !pub_avg_planning_) {
      return;
    }
    AvgPlanningMsgs msg;
    msg.stamp = now();
    msg.health.stamp = msg.stamp;
    msg.health.module_name = "planning";
    msg.health.level = ModuleHealth::OK;
    msg.health.message = "goal_replanner";
    if (global_path.has_value()) {
      msg.global_path = global_path.value();
    }
    if (navigate_status.has_value()) {
      msg.navigate_to_pose_status = navigate_status.value();
    }
    if (goal_pose.has_value()) {
      msg.goal_pose = goal_pose.value();
    }
    pub_avg_planning_->publish(msg);
  }

  bool enabled_{true};
  bool has_goal_{false};
  bool has_start_{false};
  bool has_last_submitted_{false};
  bool in_flight_{false};
  uint64_t goal_msg_counter_{0};
  uint64_t last_submitted_goal_counter_{0};

  std::string goal_topic_;
  std::string start_topic_;
  bool use_topic_start_pose_{false};
  bool start_topic_fallback_to_tf_{true};
  bool pause_when_navigate_active_{true};
  bool navigate_active_{false};
  rclcpp::Time last_navigate_active_time_{0, 0, RCL_ROS_TIME};
  std::string action_name_;
  std::string planner_id_;
  std::string frame_id_;
  std::string output_path_topic_;
  std::string planning_diagnostic_topic_;
  std::string navigate_status_topic_;
  bool publish_result_path_{true};
  bool publish_planning_diagnostic_{false};
  bool enable_periodic_replan_{false};

  double min_request_interval_sec_{0.25};
  double replan_rate_hz_{0.0};
  double request_timeout_sec_{0.0};
  double retry_after_failure_sec_{0.8};
  double start_replan_distance_{0.4};
  double goal_replan_distance_{0.1};
  bool ignore_duplicate_goal_messages_{true};
  double duplicate_goal_xy_epsilon_m_{0.05};
  double duplicate_goal_yaw_epsilon_deg_{4.0};
  double navigate_inactive_grace_sec_{0.8};
  double start_replan_yaw_deg_{10.0};
  bool replan_on_start_change_{false};
  bool stop_replan_after_goal_reached_{true};
  bool goal_reached_latched_{false};
  bool immediate_replan_on_goal_{true};
  bool immediate_replan_on_start_{true};
  bool has_sent_any_request_{false};
  bool last_request_used_tf_start_{false};
  bool force_tf_start_once_{false};
  bool tf_start_fallback_used_for_goal_{false};
  double goal_reached_distance_m_{0.8};
  uint64_t request_seq_counter_{0};
  uint64_t active_request_seq_{0};

  rclcpp::Subscription<avg_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<avg_msgs::msg::PoseStamped>::SharedPtr sub_start_;
  rclcpp::Subscription<avg_msgs::msg::GoalStatusArray>::SharedPtr sub_nav_status_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr action_client_;
  GoalHandleComputePathToPose::SharedPtr active_goal_handle_;
  rclcpp::Publisher<avg_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<AvgPlanningMsgs>::SharedPtr pub_avg_planning_;

  rclcpp::Time request_sent_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_request_sent_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_allowed_request_time_{0, 0, RCL_ROS_TIME};
  avg_msgs::msg::PoseStamped latest_goal_;
  avg_msgs::msg::PoseStamped latest_start_;
  avg_msgs::msg::PoseStamped last_submitted_goal_;
  avg_msgs::msg::PoseStamped last_submitted_start_;
};

}  // namespace camrod_planning

// Entry point for this executable.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod_planning::GoalReplannerNode>());
  rclcpp::shutdown();
  return 0;
}
