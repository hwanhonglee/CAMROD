#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <avg_msgs/msg/avg_planning_msgs.hpp>
#include <avg_msgs/msg/module_health.hpp>
#include <avg_msgs/msg/point.hpp>
#include <avg_msgs/msg/pose_stamped.hpp>
#include <avg_msgs/msg/quaternion.hpp>
#include <avg_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace camrod_planning
{

class LocalPathExtractorNode : public rclcpp::Node
{
public:
  using AvgPlanningMsgs = avg_msgs::msg::AvgPlanningMsgs;
  using ModuleHealth = avg_msgs::msg::ModuleHealth;

  // Implements `LocalPathExtractorNode` behavior.
  LocalPathExtractorNode()
  : rclcpp::Node("local_path_extractor")
  {
    enabled_ = declare_parameter<bool>("enabled", true);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    global_path_topic_ =
      declare_parameter<std::string>("global_path_topic", "/planning/global_path");
    // HH_260316-00:00 Default to lanelet-snapped pose to keep local path aligned to centerline.
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/planning/lanelet_pose");
    output_topic_ = declare_parameter<std::string>("output_topic", "/planning/local_path");
    publish_planning_diagnostic_ = declare_parameter<bool>("publish_planning_diagnostic", false);
    planning_diagnostic_topic_ =
      declare_parameter<std::string>("planning_diagnostic_topic", "/planning/diagnostic");
    // HH_260305-00:00 QoS durability for /planning/global_path subscriber.
    // Keep false by default because planner_server publishes VOLATILE.
    global_path_qos_transient_local_ =
      declare_parameter<bool>("global_path_qos_transient_local", false);

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 15.0);
    publish_on_input_update_ = declare_parameter<bool>("publish_on_input_update", true);
    lookahead_distance_m_ = declare_parameter<double>("lookahead_distance_m", 25.0);
    lookbehind_distance_m_ = declare_parameter<double>("lookbehind_distance_m", 2.0);
    min_points_ = declare_parameter<int>("min_points", 8);
    max_points_ = declare_parameter<int>("max_points", 300);
    nearest_search_window_ = declare_parameter<int>("nearest_search_window", 200);
    nearest_backtrack_window_ = declare_parameter<int>("nearest_backtrack_window", 20);
    full_reacquire_distance_m_ = declare_parameter<double>("full_reacquire_distance_m", 3.0);
    full_reacquire_heading_deg_ =
      declare_parameter<double>("full_reacquire_heading_deg", 85.0);
    // HH_260316-00:00 Guard against occasional discontinuous segments
    // in upstream paths (prevents straight-line bridge artifacts in RViz/local path).
    max_segment_jump_m_ = declare_parameter<double>("max_segment_jump_m", 3.0);
    // HH_260306-00:00 Latch local-path stop when goal is reached.
    stop_after_goal_reached_ = declare_parameter<bool>("stop_after_goal_reached", true);
    goal_reached_distance_m_ = declare_parameter<double>("goal_reached_distance_m", 0.8);
    goal_reached_index_margin_ = declare_parameter<int>("goal_reached_index_margin", 2);
    pose_timeout_sec_ = declare_parameter<double>("pose_timeout_sec", 1.0);
    empty_republish_period_sec_ =
      declare_parameter<double>("empty_republish_period_sec", 0.5);
    // HH_260305-00:00 Clear stale local-path consumers immediately when inputs go invalid.
    publish_empty_on_invalid_ = declare_parameter<bool>("publish_empty_on_invalid", true);

    auto global_path_qos = rclcpp::QoS(1).reliable();
    if (global_path_qos_transient_local_) {
      global_path_qos.transient_local();
    }
    // HH_260305-00:00 Use reliable/latest-only pose QoS.
    // SensorDataQoS(best_effort) can drop updates under load, which makes local path look late.
    auto pose_qos = rclcpp::QoS(1).reliable();
    auto local_path_qos = rclcpp::QoS(1).reliable();
    sub_global_path_ = create_subscription<avg_msgs::msg::Path>(
      global_path_topic_, global_path_qos,
      std::bind(&LocalPathExtractorNode::onGlobalPath, this, std::placeholders::_1));
    sub_pose_ = create_subscription<avg_msgs::msg::PoseStamped>(
      pose_topic_, pose_qos,
      std::bind(&LocalPathExtractorNode::onPose, this, std::placeholders::_1));
    pub_local_path_ = create_publisher<avg_msgs::msg::Path>(output_topic_, local_path_qos);
    if (publish_planning_diagnostic_) {
      pub_avg_planning_ = create_publisher<AvgPlanningMsgs>(
        planning_diagnostic_topic_, rclcpp::QoS(10));
    }

    if (publish_rate_hz_ > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&LocalPathExtractorNode::onTimer, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "local_path_extractor: global=%s pose=%s output=%s frame=%s lookahead=%.1fm lookbehind=%.1fm",
      global_path_topic_.c_str(), pose_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str(),
      lookahead_distance_m_, lookbehind_distance_m_);
  }

private:
  struct SearchResult
  {
    size_t idx{0};
    double score{std::numeric_limits<double>::infinity()};
    double d2{std::numeric_limits<double>::infinity()};
    double yaw_err{0.0};
    bool valid{false};
  };

  // Implements `dist2` behavior.
  static double dist2(
    const avg_msgs::msg::Point & a,
    const avg_msgs::msg::Point & b)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return dx * dx + dy * dy;
  }

  // Implements `segmentLen2D` behavior.
  static double segmentLen2D(
    const avg_msgs::msg::PoseStamped & a,
    const avg_msgs::msg::PoseStamped & b)
  {
    return std::hypot(
      b.pose.position.x - a.pose.position.x,
      b.pose.position.y - a.pose.position.y);
  }

  // Implements `normalizeAngle` behavior.
  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  // Implements `yawFromQuaternion` behavior.
  static double yawFromQuaternion(const avg_msgs::msg::Quaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  // Implements `degToRad` behavior.
  static double degToRad(double deg)
  {
    return deg * M_PI / 180.0;
  }

  // Implements `tangentYawAt` behavior.
  double tangentYawAt(size_t idx) const
  {
    const auto & poses = global_path_.poses;
    if (poses.empty()) {
      return 0.0;
    }
    if (poses.size() == 1) {
      return yawFromQuaternion(poses.front().pose.orientation);
    }

    const size_t prev = (idx > 0) ? (idx - 1) : idx;
    const size_t next = (idx + 1 < poses.size()) ? (idx + 1) : idx;
    if (prev == next) {
      return yawFromQuaternion(poses[idx].pose.orientation);
    }

    const auto & a = poses[prev].pose.position;
    const auto & b = poses[next].pose.position;
    return std::atan2(b.y - a.y, b.x - a.x);
  }

  // Handles the `onGlobalPath` callback.
  void onGlobalPath(const avg_msgs::msg::Path::ConstSharedPtr msg)
  {
    if (!msg || msg->poses.empty()) {
      has_global_path_ = false;
      publishEmptyPath();
      return;
    }

    bool route_changed = true;
    if (has_global_path_ && !global_path_.poses.empty() && !msg->poses.empty()) {
      const auto & old_first = global_path_.poses.front().pose.position;
      const auto & old_last = global_path_.poses.back().pose.position;
      const auto & new_first = msg->poses.front().pose.position;
      const auto & new_last = msg->poses.back().pose.position;
      const double first_delta = std::hypot(old_first.x - new_first.x, old_first.y - new_first.y);
      const double last_delta = std::hypot(old_last.x - new_last.x, old_last.y - new_last.y);
      route_changed = (first_delta > 2.0) || (last_delta > 2.0);
    }

    global_path_ = *msg;
    if (global_path_.header.frame_id.empty()) {
      global_path_.header.frame_id = frame_id_;
    }
    has_global_path_ = true;
    // HH_260306-00:00 Any valid new global path re-enables local path publishing.
    route_completed_latched_ = false;
    // HH_260305-00:00 Keep continuity across frequent replans on the same route.
    // Only force full reacquire when the route geometry changed significantly.
    if (route_changed || last_closest_idx_ >= global_path_.poses.size()) {
      last_closest_idx_ = global_path_.poses.size();
      force_full_reacquire_ = true;
    } else {
      force_full_reacquire_ = false;
    }
    if (publish_on_input_update_ && has_pose_) {
      onTimer();
    }
  }

  // Handles the `onPose` callback.
  void onPose(const avg_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }
    latest_pose_ = *msg;
    if (latest_pose_.header.frame_id.empty()) {
      latest_pose_.header.frame_id = frame_id_;
    }
    has_pose_ = true;
    last_pose_rx_ = now();
    if (publish_on_input_update_ && has_global_path_) {
      onTimer();
    }
  }

  // Implements `findBestIndexInRange` behavior.
  SearchResult findBestIndexInRange(
    size_t begin, size_t end, bool continuity_bias) const
  {
    const auto & poses = global_path_.poses;
    const size_t n = poses.size();
    if (n == 0) {
      return SearchResult{};
    }
    if (begin >= end || begin >= n) {
      return SearchResult{};
    }
    end = std::min(end, n);

    const auto & p = latest_pose_.pose.position;
    const double pose_yaw = yawFromQuaternion(latest_pose_.pose.orientation);
    SearchResult best;
    constexpr double kHeadingWeight = 8.0;
    constexpr double kBacktrackPenalty = 1.5;
    constexpr double kJumpPenalty = 0.02;
    for (size_t i = begin; i < end; ++i) {
      const double d2 = dist2(poses[i].pose.position, p);
      const double tangent_yaw = tangentYawAt(i);
      const double yaw_err = normalizeAngle(tangent_yaw - pose_yaw);
      double continuity_penalty = 0.0;
      if (continuity_bias && last_closest_idx_ < n) {
        if (i < last_closest_idx_) {
          const double backtrack = static_cast<double>(last_closest_idx_ - i);
          continuity_penalty += kBacktrackPenalty * backtrack * backtrack;
        } else {
          const double jump = static_cast<double>(i - last_closest_idx_);
          continuity_penalty += kJumpPenalty * jump * jump;
        }
      }
      const double score = d2 + kHeadingWeight * yaw_err * yaw_err + continuity_penalty;
      if (score < best.score) {
        best.idx = i;
        best.score = score;
        best.d2 = d2;
        best.yaw_err = yaw_err;
        best.valid = true;
      }
    }
    return best;
  }

  // Implements `findClosestPathIndex` behavior.
  size_t findClosestPathIndex() const
  {
    const auto & poses = global_path_.poses;
    const size_t n = poses.size();
    if (n == 0) {
      return 0;
    }

    if (!force_full_reacquire_ && last_closest_idx_ < n && nearest_search_window_ > 0) {
      const size_t fwd_win = static_cast<size_t>(nearest_search_window_);
      const size_t back_win = static_cast<size_t>(std::max(0, nearest_backtrack_window_));
      const size_t begin = (last_closest_idx_ > back_win) ? (last_closest_idx_ - back_win) : 0;
      const size_t end = std::min(n, last_closest_idx_ + fwd_win + 1);
      const auto local = findBestIndexInRange(begin, end, true);
      if (local.valid) {
        const double local_dist = std::sqrt(std::max(0.0, local.d2));
        const double local_yaw_deg = std::abs(local.yaw_err) * 180.0 / M_PI;
        const bool local_is_reliable =
          local_dist <= full_reacquire_distance_m_ &&
          local_yaw_deg <= full_reacquire_heading_deg_;
        if (local_is_reliable) {
          return local.idx;
        }
      }
    }

    const auto global = findBestIndexInRange(0, n, false);
    return global.valid ? global.idx : 0;
  }

  // Implements `growBackward` behavior.
  size_t growBackward(size_t anchor) const
  {
    if (anchor == 0 || lookbehind_distance_m_ <= 0.0) {
      return anchor;
    }
    double acc = 0.0;
    size_t i = anchor;
    while (i > 0) {
      const double ds = segmentLen2D(global_path_.poses[i - 1], global_path_.poses[i]);
      if (acc + ds > lookbehind_distance_m_) {
        break;
      }
      acc += ds;
      --i;
    }
    return i;
  }

  // Implements `growForward` behavior.
  size_t growForward(size_t anchor) const
  {
    const size_t n = global_path_.poses.size();
    if (n <= 1 || lookahead_distance_m_ <= 0.0) {
      return anchor;
    }
    double acc = 0.0;
    size_t i = anchor;
    while (i + 1 < n) {
      const double ds = segmentLen2D(global_path_.poses[i], global_path_.poses[i + 1]);
      acc += ds;
      ++i;
      if (acc >= lookahead_distance_m_) {
        break;
      }
    }
    return i;
  }

  // Handles the `onTimer` callback.
  void onTimer()
  {
    if (!enabled_ || !has_pose_ || !has_global_path_) {
      publishEmptyPath();
      return;
    }
    if (pose_timeout_sec_ > 0.0) {
      const double dt = (now() - last_pose_rx_).seconds();
      if (dt > pose_timeout_sec_) {
        publishEmptyPath();
        return;
      }
    }

    if (global_path_.poses.empty()) {
      publishEmptyPath();
      return;
    }
    if (
      !latest_pose_.header.frame_id.empty() &&
      !global_path_.header.frame_id.empty() &&
      latest_pose_.header.frame_id != global_path_.header.frame_id)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "frame mismatch pose=%s global_path=%s (expected same frame)",
        latest_pose_.header.frame_id.c_str(), global_path_.header.frame_id.c_str());
      publishEmptyPath();
      return;
    }

    const size_t closest = findClosestPathIndex();
    if (stop_after_goal_reached_) {
      const size_t n = global_path_.poses.size();
      const size_t margin = static_cast<size_t>(std::max(0, goal_reached_index_margin_));
      const size_t end_threshold = (n > 1) ? (n - 1 > margin ? (n - 1 - margin) : 0) : 0;
      const auto & goal_pos = global_path_.poses.back().pose.position;
      const auto & cur_pos = latest_pose_.pose.position;
      const double goal_dist = std::hypot(cur_pos.x - goal_pos.x, cur_pos.y - goal_pos.y);
      if (!route_completed_latched_ && closest >= end_threshold && goal_dist <= goal_reached_distance_m_) {
        route_completed_latched_ = true;
      }
      if (route_completed_latched_) {
        publishEmptyPath();
        return;
      }
    }

    const size_t begin = growBackward(closest);
    size_t end = growForward(closest);
    if (end < begin) {
      end = begin;
    }

    const int min_pts = std::max(2, min_points_);
    const int max_pts = std::max(min_pts, max_points_);
    const size_t cur_pts = end - begin + 1;
    if (static_cast<int>(cur_pts) < min_pts) {
      end = std::min(global_path_.poses.size() - 1, begin + static_cast<size_t>(min_pts - 1));
    }
    const size_t capped_end = std::min(
      end, begin + static_cast<size_t>(max_pts - 1));

    avg_msgs::msg::Path out;
    out.header.stamp = now();
    out.header.frame_id = global_path_.header.frame_id.empty()
      ? frame_id_ : global_path_.header.frame_id;
    out.poses.reserve(capped_end - begin + 1);
    out.poses.push_back(global_path_.poses[begin]);
    for (size_t i = begin + 1; i <= capped_end; ++i) {
      const double ds = segmentLen2D(global_path_.poses[i - 1], global_path_.poses[i]);
      if (max_segment_jump_m_ > 0.0 && ds > max_segment_jump_m_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "local_path_extractor: discontinuity detected (%.2fm > %.2fm), truncating local path",
          ds, max_segment_jump_m_);
        break;
      }
      out.poses.push_back(global_path_.poses[i]);
    }
    if (out.poses.size() < 2) {
      publishEmptyPath();
      return;
    }
    pub_local_path_->publish(out);
    publishAvgPlanning(out, false);
    last_output_empty_ = false;
    last_empty_publish_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_closest_idx_ = closest;
    force_full_reacquire_ = false;
  }

  // Publishes `EmptyPath` output.
  void publishEmptyPath()
  {
    if (!publish_empty_on_invalid_) {
      return;
    }
    const auto stamp = now();
    if (
      last_output_empty_ && empty_republish_period_sec_ > 0.0 &&
      last_empty_publish_time_.nanoseconds() > 0 &&
      (stamp - last_empty_publish_time_).seconds() < empty_republish_period_sec_)
    {
      return;
    }
    avg_msgs::msg::Path out;
    out.header.stamp = stamp;
    out.header.frame_id = global_path_.header.frame_id.empty()
      ? frame_id_ : global_path_.header.frame_id;
    pub_local_path_->publish(out);
    publishAvgPlanning(out, true);
    last_output_empty_ = true;
    last_empty_publish_time_ = stamp;
  }

  // Publishes `AvgPlanning` output.
  void publishAvgPlanning(const avg_msgs::msg::Path & local_path, bool is_empty)
  {
    if (!publish_planning_diagnostic_ || !pub_avg_planning_) {
      return;
    }
    AvgPlanningMsgs msg;
    msg.stamp = now();
    msg.health.stamp = msg.stamp;
    msg.health.module_name = "planning";
    msg.health.level = ModuleHealth::OK;
    msg.health.message = is_empty ? "local_path_extractor.empty" : "local_path_extractor";
    msg.local_path = local_path;
    pub_avg_planning_->publish(msg);
  }

  bool enabled_{true};
  std::string frame_id_{"map"};
  std::string global_path_topic_;
  std::string pose_topic_;
  std::string output_topic_;
  std::string planning_diagnostic_topic_;
  double publish_rate_hz_{15.0};
  bool publish_on_input_update_{true};
  double lookahead_distance_m_{25.0};
  double lookbehind_distance_m_{2.0};
  int min_points_{8};
  int max_points_{300};
  int nearest_search_window_{200};
  int nearest_backtrack_window_{20};
  double full_reacquire_distance_m_{3.0};
  double full_reacquire_heading_deg_{85.0};
  double max_segment_jump_m_{3.0};
  bool stop_after_goal_reached_{true};
  double goal_reached_distance_m_{0.8};
  int goal_reached_index_margin_{2};
  double pose_timeout_sec_{1.0};
  double empty_republish_period_sec_{0.5};
  bool publish_empty_on_invalid_{true};
  bool global_path_qos_transient_local_{false};

  bool has_pose_{false};
  bool has_global_path_{false};
  avg_msgs::msg::PoseStamped latest_pose_;
  avg_msgs::msg::Path global_path_;
  size_t last_closest_idx_{0};
  bool force_full_reacquire_{true};
  bool route_completed_latched_{false};
  bool last_output_empty_{true};
  rclcpp::Time last_pose_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_empty_publish_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<avg_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<avg_msgs::msg::Path>::SharedPtr sub_global_path_;
  rclcpp::Publisher<avg_msgs::msg::Path>::SharedPtr pub_local_path_;
  rclcpp::Publisher<AvgPlanningMsgs>::SharedPtr pub_avg_planning_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool publish_planning_diagnostic_{false};
};

}  // namespace camrod_planning

// Entry point for this executable.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod_planning::LocalPathExtractorNode>());
  rclcpp::shutdown();
  return 0;
}
