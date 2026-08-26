// HH_260720 - Keep every control executable in the package's single source directory.
// HH_260720 - Rename the bench controller as the final AprilTag parking controller.
//
// HH_260720 - The controller waits for a rear-camera parking tag, estimates the
// station axis in odometry, reverses with heading/lateral feedback, and latches
// translation off at the camera-range stop before a bounded yaw-only alignment.
// It never bypasses /control/cmd_vel_safety_gate.
//
// State machine:
//   WAITING_FOR_TAG -> TAG_GUIDED_REVERSE -> FINAL_YAW_ALIGNMENT
//                              |                       |
//                              +-> ERROR       WAITING_FOR_CHARGING -> PARKED

#include <memory>
#include <string>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <avg_msgs/conversions.hpp>
#include <avg_msgs/msg/avg_april_tag_pose.hpp>
#include <avg_msgs/msg/avg_odometry.hpp>
#include <avg_msgs/msg/avg_platform_status.hpp>
#include <avg_msgs/msg/avg_twist.hpp>
#include <avg_msgs/msg/motion_operation.hpp>
#include <avg_msgs/srv/request_motion_operation.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <avg_msgs/msg/avg_service_state.hpp>
#include <avg_msgs/msg/module_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "camrod_control/parking_speed_profile.hpp"
#include "camrod_control/yaw_alignment_settling.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// HH_260720 - Keep parking controller implementations under the control parking source layer.
namespace
{
double clamp(double v, double lo, double hi) {return std::max(lo, std::min(hi, v));}
double normalizeAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}
}  // namespace

class AprilTagParkingControllerNode : public rclcpp::Node
{
public:
  // HH_260720 - State names describe the physical motion performed in each phase.
  enum class State { IDLE, WAITING_FOR_TAG, TAG_GUIDED_REVERSE,
    FINAL_YAW_ALIGNMENT, WAITING_FOR_CHARGING, PARKED, ERROR };

  AprilTagParkingControllerNode()
  : Node("apriltag_parking_controller"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    // HH_260720 - Expose descriptive parking-controller parameters and canonical topics.
    // HH_260720 - Use canonical CAMROD frames and supervised raw-command interfaces.
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "robot_center_link");
    cmd_vel_topic_ = declare_parameter<std::string>("command_topic", "/control/cmd_vel_raw");
    tag_pose_topic_ = declare_parameter<std::string>(
      "tag_pose_topic",
      "/perception/apriltag_parking_detector/tag_pose");
    odom_topic_ = declare_parameter<std::string>(
      "odometry_topic", "/localization/odometry");
    require_charging_for_completion_ = declare_parameter<bool>(
      "require_charging_for_completion", true);
    stop_when_charging_ = declare_parameter<bool>("stop_when_charging", true);
    platform_status_topic_ = declare_parameter<std::string>(
      "platform_status_topic", "/platform/status");
    operation_topic_ = declare_parameter<std::string>(
      "operation_topic", "/parking/operation");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/parking/apriltag_parking_controller/status");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/system/diagnostics");
    service_state_topic_ = declare_parameter<std::string>(
      "service_state_topic", "/service/state");

    // HH_260720 - Keep the imported Ranger gains explicit until hardware tuning is repeated.
    heading_gain_ = declare_parameter<double>("heading_gain", 1.5);
    lateral_to_heading_gain_ = declare_parameter<double>("lateral_to_heading_gain", 2.5);
    maximum_angular_speed_radps_ =
      declare_parameter<double>("maximum_angular_speed_radps", 1.0);
    maximum_approach_angle_rad_ =
      declare_parameter<double>("maximum_approach_angle_rad", 0.5);
    reverse_approach_speed_mps_ =
      declare_parameter<double>("reverse_approach_speed_mps", 0.2);
    // HH_260720 - Default to REP-103 angular response; enable inversion only after hardware proof.
    invert_wz_in_reverse_ = declare_parameter<bool>(
      "invert_angular_command_while_reversing", false);
    final_insertion_speed_mps_ =
      declare_parameter<double>("final_insertion_speed_mps", 0.05);
    // HH_260819 - These thresholds use the unmodified camera-frame 3D range,
    // exactly matching the operator UI's Tag distance value.
    slowdown_start_tag_distance_m_ = std::abs(declare_parameter<double>(
      "slowdown_start_tag_distance_m", 0.80));
    translation_stop_tag_distance_m_ = std::abs(declare_parameter<double>(
      "translation_stop_tag_distance_m", 0.40));
    minimum_approach_turn_radius_m_ = std::abs(declare_parameter<double>(
      "minimum_approach_turn_radius_m", 0.85));
    final_yaw_angular_speed_radps_ = std::abs(declare_parameter<double>(
      "final_yaw_angular_speed_radps", 0.20));
    slowdown_start_distance_m_ =
      declare_parameter<double>("slowdown_start_distance_m", 1.443);
    slowdown_start_remaining_distance_m_ = std::abs(declare_parameter<double>(
      "slowdown_start_remaining_distance_m", 0.60));

    // HH_260720 - Final insertion starts only inside the configured alignment tolerances.
    final_heading_tolerance_rad_ =
      declare_parameter<double>("final_heading_tolerance_rad", 0.10);
    final_lateral_tolerance_m_ =
      declare_parameter<double>("final_lateral_tolerance_m", 0.03);
    final_yaw_settle_hold_s_ = std::abs(declare_parameter<double>(
      "final_yaw_settle_hold_s", 0.8));
    final_yaw_settle_max_rate_degps_ = std::abs(declare_parameter<double>(
      "final_yaw_settle_max_rate_degps", 3.0));
    yaw_alignment_settling_.setConfig(
      camrod_control::YawAlignmentSettlingConfig{
        std::abs(final_heading_tolerance_rad_) * 180.0 / M_PI,
        final_yaw_settle_hold_s_, final_yaw_settle_max_rate_degps_});

    // HH_260720 - Distances are measured along the parking-tag normal axis.
    final_insertion_start_distance_m_ = declare_parameter<double>(
      "final_insertion_start_distance_m", 0.993);
    parked_distance_from_tag_m_ = declare_parameter<double>(
      "parked_distance_from_tag_m", 0.943);
    tag_timeout_s_ = declare_parameter<double>("tag_timeout_s", 0.5);
    odometry_timeout_s_ = std::abs(declare_parameter<double>("odometry_timeout_s", 0.5));
    tag_wait_timeout_s_ = declare_parameter<double>("tag_wait_timeout_s", 60.0);
    retry_forward_distance_m_ =
      declare_parameter<double>("retry_forward_distance_m", 1.0);
    max_retries_ = declare_parameter<int>("maximum_retries", 5);

    pose_filter_gain_ = declare_parameter<double>("pose_filter_gain", 0.3);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 30.0);
    status_rate_hz_ = declare_parameter<double>("status_rate_hz", 2.0);

    // HH_260720 - Weight the odometry-frame parking axis by observed tag distance.
    axis_full_trust_distance_m_ =
      declare_parameter<double>("axis_full_trust_distance_m", 0.743);
    axis_minimum_trust_distance_m_ =
      declare_parameter<double>("axis_minimum_trust_distance_m", 4.443);
    axis_minimum_filter_weight_ =
      declare_parameter<double>("axis_minimum_filter_weight", 0.05);

    // HH_260720 - Connect shared parking commands, status, and platform charging feedback.
    tag_sub_ = create_subscription<avg_msgs::msg::AvgAprilTagPose>(
      tag_pose_topic_, 10,
      std::bind(&AprilTagParkingControllerNode::tagCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<avg_msgs::msg::AvgOdometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AprilTagParkingControllerNode::odomCallback, this, std::placeholders::_1));

    // HH_260720 - Consume charging from the one canonical generated platform status.
    platform_status_sub_ = create_subscription<avg_msgs::msg::AvgPlatformStatus>(
      platform_status_topic_, 10,
      [this](const avg_msgs::msg::AvgPlatformStatus::SharedPtr message) {
        const bool charging_changed = charging_detected_ != message->is_charging;
        charging_detected_ = message->is_charging;
        if (charging_changed && state_ == State::PARKED) {
          // HH_260721 - Refresh the shared parked state when CAN charging changes.
          publishServiceState(true);
        }
      });

    cmd_pub_ = create_publisher<avg_msgs::msg::AvgTwist>(cmd_vel_topic_, 10);
    status_pub_ = create_publisher<avg_msgs::msg::ModuleState>(status_topic_, 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, 10);
    service_state_pub_ = create_publisher<avg_msgs::msg::AvgServiceState>(
      service_state_topic_, 10);

    // HH_260720 - Publish parking geometry and traveled path for RViz inspection.
    odom_frame_id_ = declare_parameter<std::string>("odometry_frame_id", "odom");
    viz_enabled_ = declare_parameter<bool>("publish_visualization", true);
    if (viz_enabled_) {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/parking/apriltag_parking_controller/markers_ros", 5);
      path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/parking/apriltag_parking_controller/path_ros", 5);
      robot_path_.header.frame_id = odom_frame_id_;
    }

    // HH_260720 - Use one typed operation topic instead of Bool start/cancel aliases.
    operation_sub_ = create_subscription<avg_msgs::msg::MotionOperation>(
      operation_topic_, 10,
      [this](const avg_msgs::msg::MotionOperation::SharedPtr message) {
        const std::string source = message->source.empty() ? "topic" : message->source;
        if (message->operation == avg_msgs::msg::MotionOperation::START) {
          std::string detail;
          if (!startParking(source, detail)) {
            RCLCPP_ERROR(get_logger(), "%s", detail.c_str());
          }
        } else if (message->operation == avg_msgs::msg::MotionOperation::CANCEL) {
          cancelParking(source);
        }
      });
    operation_service_ = create_service<avg_msgs::srv::RequestMotionOperation>(
      "/parking/apriltag_parking_controller/request_operation",
      std::bind(
        &AprilTagParkingControllerNode::operationService, this,
        std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AprilTagParkingControllerNode::controlLoop, this));

    if (final_insertion_start_distance_m_ <= parked_distance_from_tag_m_) {
      RCLCPP_WARN(
        get_logger(),
        "final insertion start %.2fm must exceed parked distance %.2fm",
        final_insertion_start_distance_m_, parked_distance_from_tag_m_);
    }
    if (slowdown_start_tag_distance_m_ <= translation_stop_tag_distance_m_) {
      RCLCPP_WARN(
        get_logger(),
        "tag slowdown start %.2fm must exceed translation stop %.2fm",
        slowdown_start_tag_distance_m_, translation_stop_tag_distance_m_);
    }
    RCLCPP_INFO(
      get_logger(),
      "apriltag_parking_controller ready: command=%s odometry=%s tag=%s "
      "slowdown_tag=%.3fm translation_stop_tag=%.3fm",
      cmd_vel_topic_.c_str(), odom_topic_.c_str(), tag_pose_topic_.c_str(),
      slowdown_start_tag_distance_m_, translation_stop_tag_distance_m_);
    publishStatus(true);
  }

private:
  // HH_260720 - Transform tag observations and estimate a fixed odometry-frame parking axis.
  void tagCallback(const avg_msgs::msg::AvgAprilTagPose::SharedPtr msg)
  {
    const auto observation_time = now();
    // HH_260819 - Do not freeze a new tag/axis estimate against an old vehicle
    // pose. WAITING_FOR_TAG remains stopped until tag and odometry overlap in
    // the configured freshness window.
    if (!odometryIsFresh(observation_time)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "fresh odometry unavailable; rejecting parking tag observation");
      return;
    }
    // HH_260720 - TF remains a standard ROS boundary; convert only for that call.
    const geometry_msgs::msg::PoseStamped tag_pose =
      avg_msgs::conversions::toRos(msg->pose);
    // HH_260819 - Preserve the detector's camera-frame 3D norm before TF.  The
    // UI computes Tag distance from these exact three coordinates.
    const double measured_tag_camera_distance_m = std::sqrt(
      tag_pose.pose.position.x * tag_pose.pose.position.x +
      tag_pose.pose.position.y * tag_pose.pose.position.y +
      tag_pose.pose.position.z * tag_pose.pose.position.z);
    if (!std::isfinite(measured_tag_camera_distance_m)) {
      return;
    }
    // HH_260720 - Transform the optical-frame observation into the robot base frame.
    geometry_msgs::msg::PoseStamped pose_base;
    try {
      // HH_260720 - Use the latest static transform for the fixed rear camera mount.
      auto tf = tf_buffer_.lookupTransform(
        base_frame_id_, tag_pose.header.frame_id,
        tf2::TimePointZero);
      tf2::doTransform(tag_pose, pose_base, tf);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "tag TF failed (%s -> %s): %s; verify the rear-camera static transform",
        tag_pose.header.frame_id.c_str(), base_frame_id_.c_str(), e.what());
      return;
    }

    // HH_260720 - Read tag position in the robot base frame.
    const double px = pose_base.pose.position.x;
    const double py = pose_base.pose.position.y;

    // HH_260720 - Project the tag normal onto the ground as the parking axis.
    tf2::Quaternion q(pose_base.pose.orientation.x, pose_base.pose.orientation.y,
      pose_base.pose.orientation.z, pose_base.pose.orientation.w);
    tf2::Matrix3x3 R(q);
    // HH_260720 - The third rotation-matrix column is the tag normal in base frame.
    double ax = R[0][2];
    double ay = R[1][2];
    const double norm = std::hypot(ax, ay);
    if (norm < 1e-3) {return;}
    ax /= norm; ay /= norm;
    // HH_260720 - Orient the normal from the parking tag toward the reversing robot.
    if (ax < 0.0) {ax = -ax; ay = -ay;}

    // HH_260720 - Filter the stationary tag and axis in odometry so brief camera loss is tolerable.
    const double cy = std::cos(vehicle_odometry_yaw_rad_);
    const double sy = std::sin(vehicle_odometry_yaw_rad_);
    const double measured_tag_odometry_x_m = vehicle_odometry_x_m_ + cy * px - sy * py;
    const double measured_tag_odometry_y_m = vehicle_odometry_y_m_ + sy * px + cy * py;
    const double measured_axis_odometry_yaw_rad =
      normalizeAngle(vehicle_odometry_yaw_rad_ + std::atan2(ay, ax));

    // HH_260720 - Down-weight far tag-normal yaw because perspective creates systematic bias.
    const double dist = std::hypot(px, py);
    double axis_observation_weight = (axis_minimum_trust_distance_m_ - dist) /
      std::max(axis_minimum_trust_distance_m_ - axis_full_trust_distance_m_, 1e-3);
    axis_observation_weight = clamp(
      axis_observation_weight, axis_minimum_filter_weight_, 1.0);

    if (!axis_valid_) {
      tag_odometry_x_m_ = measured_tag_odometry_x_m;
      tag_odometry_y_m_ = measured_tag_odometry_y_m;
      parking_axis_odometry_yaw_rad_ = measured_axis_odometry_yaw_rad;
      axis_valid_ = true;
    } else {
      const double position_filter_gain = pose_filter_gain_;
      tag_odometry_x_m_ += position_filter_gain *
        (measured_tag_odometry_x_m - tag_odometry_x_m_);
      tag_odometry_y_m_ += position_filter_gain *
        (measured_tag_odometry_y_m - tag_odometry_y_m_);
      const double axis_filter_gain = pose_filter_gain_ * axis_observation_weight;
      parking_axis_odometry_yaw_rad_ = normalizeAngle(
        parking_axis_odometry_yaw_rad_ + axis_filter_gain * normalizeAngle(
          measured_axis_odometry_yaw_rad - parking_axis_odometry_yaw_rad_));
    }
    tag_camera_distance_m_ = measured_tag_camera_distance_m;
    tag_camera_distance_valid_ = true;
    last_tag_time_ = observation_time;
  }

  // HH_260720 - Update longitudinal, lateral, and heading errors from current odometry.
  void updateParkingErrorsFromOdometry()
  {
    const double axis_cos = std::cos(parking_axis_odometry_yaw_rad_);
    const double axis_sin = std::sin(parking_axis_odometry_yaw_rad_);
    const double tag_to_vehicle_x_m = vehicle_odometry_x_m_ - tag_odometry_x_m_;
    const double tag_to_vehicle_y_m = vehicle_odometry_y_m_ - tag_odometry_y_m_;
    distance_along_parking_axis_m_ =
      tag_to_vehicle_x_m * axis_cos + tag_to_vehicle_y_m * axis_sin;
    lateral_error_m_ = -tag_to_vehicle_x_m * axis_sin + tag_to_vehicle_y_m * axis_cos;
    heading_error_rad_ = normalizeAngle(
      parking_axis_odometry_yaw_rad_ - vehicle_odometry_yaw_rad_);
  }

  void odomCallback(const avg_msgs::msg::AvgOdometry::SharedPtr msg)
  {
    vehicle_odometry_x_m_ = msg->pose.pose.position.x;
    vehicle_odometry_y_m_ = msg->pose.pose.position.y;
    const auto & q = msg->pose.pose.orientation;
    vehicle_odometry_yaw_rad_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    odom_valid_ = true;
    last_odometry_time_ = now();

    // HH_260720 - Record one path point per 2 cm only while parking is active.
    if (viz_enabled_ && path_pub_ && state_ != State::IDLE &&
      state_ != State::PARKED && state_ != State::ERROR)
    {
      bool add = robot_path_.poses.empty();
      if (!add) {
        const auto & last = robot_path_.poses.back().pose.position;
        add = std::hypot(
          vehicle_odometry_x_m_ - last.x, vehicle_odometry_y_m_ - last.y) > 0.02;
      }
      if (add) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = msg->header.stamp;
        ps.header.frame_id = odom_frame_id_;
        // HH_260720 - Convert the internal CAMROD pose explicitly at the RViz boundary.
        ps.pose = avg_msgs::conversions::toRos(msg->pose.pose);
        robot_path_.poses.push_back(ps);
        if (robot_path_.poses.size() > 2000) {
          robot_path_.poses.erase(robot_path_.poses.begin());
        }
        robot_path_.header.stamp = msg->header.stamp;
        path_pub_->publish(robot_path_);
      }
    }
  }

  bool odometryIsFresh(const rclcpp::Time & current_time) const
  {
    if (!odom_valid_ || last_odometry_time_.nanoseconds() <= 0) {
      return false;
    }
    const double age_s = (current_time - last_odometry_time_).seconds();
    return age_s >= 0.0 && age_s <= odometry_timeout_s_;
  }

  // HH_260720 - Start and cancel are shared by topic and service interfaces.
  bool startParking(const std::string & source, std::string & detail)
  {
    if (state_ != State::IDLE && state_ != State::ERROR && state_ != State::PARKED) {
      detail = "AprilTag parking is already active";
      return false;
    }
    if (!odometryIsFresh(now())) {
      detail = "fresh odometry is unavailable";
      transitionTo(State::ERROR);
      return false;
    }
    retries_ = 0;
    axis_valid_ = false;
    tag_camera_distance_valid_ = false;
    translation_stop_reason_ = "none";
    translation_stop_trigger_tag_distance_m_ = -1.0;
    yaw_alignment_settling_.reset();
    yaw_alignment_settled_logged_ = false;
    robot_path_.poses.clear();
    transitionTo(State::WAITING_FOR_TAG);
    detail = "AprilTag parking started from " + source;
    return true;
  }

  void cancelParking(const std::string & source)
  {
    publishStop();
    transitionTo(State::IDLE);
    RCLCPP_INFO(get_logger(), "AprilTag parking cancelled from %s", source.c_str());
  }

  void operationService(
    const avg_msgs::srv::RequestMotionOperation::Request::SharedPtr request,
    avg_msgs::srv::RequestMotionOperation::Response::SharedPtr response)
  {
    // HH_260720 - One generated service handles all supported parking operations.
    const std::string source = request->source.empty() ? "service" : request->source;
    if (request->operation == avg_msgs::msg::MotionOperation::START) {
      response->accepted = startParking(source, response->message);
      return;
    }
    if (request->operation == avg_msgs::msg::MotionOperation::CANCEL) {
      cancelParking(source);
      response->accepted = true;
      response->message = "AprilTag parking cancelled";
      return;
    }
    response->accepted = false;
    response->message = "unsupported AprilTag parking operation";
  }

  // HH_260720 - Run the explicit AprilTag parking state machine.
  void controlLoop()
  {
    const auto current_time = now();
    const bool odometry_is_fresh = odometryIsFresh(current_time);
    const bool tag_fresh = axis_valid_ && tag_camera_distance_valid_ &&
      (current_time - last_tag_time_).seconds() < tag_timeout_s_;

    // HH_260720 - Refresh errors only from current odometry after the first valid tag.
    if (axis_valid_ && odometry_is_fresh) {
      updateParkingErrorsFromOdometry();
      if (viz_enabled_) {updateVisualizationGeometry();}
    }

    avg_msgs::msg::AvgTwist command;

    // HH_260818 - Charger current is authoritative contact evidence. Stop in
    // the same control tick from every active insertion phase, including tag
    // guidance and tag waiting, instead of waiting for a distance threshold.
    if (stop_when_charging_ && charging_detected_ && state_ != State::IDLE &&
      state_ != State::PARKED && state_ != State::ERROR)
    {
      translation_stop_reason_ = "charging";
      translation_stop_trigger_tag_distance_m_ =
        tag_camera_distance_valid_ ? tag_camera_distance_m_ : -1.0;
      RCLCPP_INFO(
        get_logger(),
        "parking translation stopped by charging contact: tag=%.3fm "
        "configured_stop=%.3fm phase=%s",
        translation_stop_trigger_tag_distance_m_, translation_stop_tag_distance_m_,
        stateName(state_));
      publishStop();
      transitionTo(State::PARKED);
      publishStatus();
      return;
    }

    switch (state_) {
      case State::IDLE:
      case State::PARKED:
      case State::ERROR:
        publishStatus();
        return;

      // HH_260720 - Wait stopped until the configured parking tag is visible.
      case State::WAITING_FOR_TAG: {
          if (!odometry_is_fresh) {
            RCLCPP_ERROR_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "odometry stale while waiting for parking tag; holding zero");
            publishStop();
            publishStatus();
            return;
          }
          if (tag_fresh) {
            transitionTo(State::TAG_GUIDED_REVERSE);
            break;
          }
          if (stateElapsed() > tag_wait_timeout_s_) {
            RCLCPP_ERROR(
              get_logger(),
              "parking tag wait timed out; tag is outside rear camera view");
            fail();
            break;
          }
          break;
        }

      // HH_260819 - Reverse from the first valid tag with simultaneous lateral
      // and heading correction. Camera range owns slowdown and the one-way
      // translation stop; robot-center thresholds below remain compatibility-only.
      case State::TAG_GUIDED_REVERSE: {
          if (!odometry_is_fresh) {
            // HH_260819 - A current tag must never drive reverse/steer using
            // the last heading or lateral error from stale odometry.
            RCLCPP_ERROR_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "odometry stale during tag-guided reverse; holding zero");
            publishStop();
            publishStatus();
            return;
          }
          if (!tag_fresh) {
            // HH_260819 - Never turn a near-tag camera loss into blind reverse
            // insertion. The default command is zero until reacquisition.
            handleTagDetectionLoss();
            break;
          }
          const double correction_heading_rad = calculateLateralCorrectionHeading();
          if (tag_camera_distance_m_ <= translation_stop_tag_distance_m_) {
            // HH_260824 - The configured 0.40 m camera-range crossing is a
            // one-way latch. Record the exact trigger sample separately from
            // later tag observations so final yaw motion or chassis settling
            // cannot make the operator-visible stop cause ambiguous.
            translation_stop_reason_ = "tag_range";
            translation_stop_trigger_tag_distance_m_ = tag_camera_distance_m_;
            RCLCPP_INFO(
              get_logger(),
              "parking translation stop threshold crossed: tag=%.3fm "
              "configured_stop=%.3fm lateral=%.3fm heading=%.3frad",
              translation_stop_trigger_tag_distance_m_, translation_stop_tag_distance_m_,
              lateral_error_m_, heading_error_rad_);
            // After this point no tag loss, yaw error, or missing charge may
            // restart translation. Lateral error cannot be corrected by yaw-only motion.
            if (std::fabs(lateral_error_m_) > final_lateral_tolerance_m_) {
              RCLCPP_ERROR(
                get_logger(),
                "tag translation stop reached with unsafe lateral error: "
                "tag=%.3fm lateral=%.3fm tolerance=%.3fm",
                tag_camera_distance_m_, lateral_error_m_, final_lateral_tolerance_m_);
              fail();
              break;
            }
            yaw_alignment_settling_.reset();
            yaw_alignment_settled_logged_ = false;
            publishStop();
            transitionTo(State::FINAL_YAW_ALIGNMENT);
            break;
          }
          const double remaining_tag_distance_m =
            camrod_control::tagApproachRemainingDistance(
              tag_camera_distance_m_, translation_stop_tag_distance_m_);
          const double slowdown_window_m =
            camrod_control::tagApproachSlowdownWindow(
              slowdown_start_tag_distance_m_, translation_stop_tag_distance_m_);
          const double reverse_speed_mps = camrod_control::parkingApproachSpeed(
            remaining_tag_distance_m, slowdown_window_m,
            reverse_approach_speed_mps_, final_insertion_speed_mps_);
          double angular_speed_radps = clamp(
            heading_gain_ * normalizeAngle(heading_error_rad_ - correction_heading_rad),
            -maximum_angular_speed_radps_, maximum_angular_speed_radps_);
          if (invert_wz_in_reverse_) {angular_speed_radps = -angular_speed_radps;}
          angular_speed_radps = camrod_control::limitApproachAngularSpeedForTurnRadius(
            angular_speed_radps, reverse_speed_mps, minimum_approach_turn_radius_m_);
          command.linear.x = -reverse_speed_mps;
          command.angular.z = angular_speed_radps;
          break;
        }

      // HH_260819 - At the camera-range stop, rotate only around the chassis
      // center. A fresh odometry sample must remain aligned and slow for the
      // configured hold; charging confirmation may arrive later while stopped.
      case State::FINAL_YAW_ALIGNMENT: {
          if (!odometry_is_fresh) {
            // HH_260819 - Never continue rotating from a stale heading error.
            // Stay in this exact phase with zero so the charging fast-path can
            // still preempt to PARKED as soon as current is confirmed.
            yaw_alignment_settling_.reset();
            RCLCPP_ERROR_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "odometry stale during final yaw alignment; holding zero");
            break;
          }
          const bool settled = yaw_alignment_settling_.observe(
            heading_error_rad_, vehicle_odometry_yaw_rad_, last_odometry_time_.seconds());
          if (!yaw_alignment_settling_.withinTolerance()) {
            command.angular.z = clamp(
              heading_gain_ * heading_error_rad_,
              -final_yaw_angular_speed_radps_, final_yaw_angular_speed_radps_);
          } else if (settled && !yaw_alignment_settled_logged_) {
            RCLCPP_INFO(
              get_logger(),
              "final yaw aligned at tag %.3fm; stopped while waiting for charging",
              tag_camera_distance_m_);
            yaw_alignment_settled_logged_ = true;
          }
          if (settled && require_charging_for_completion_) {
            publishStop();
            transitionTo(State::WAITING_FOR_CHARGING);
          } else if (settled) {
            publishStop();
            transitionTo(State::PARKED);
          }
          break;
        }

      // HH_260824 - Publish an explicit fresh phase for the charging fast-path
      // while continuing to command zero at the configured 0.40 m stop.
      case State::WAITING_FOR_CHARGING:
        break;

    }

    cmd_pub_->publish(command);
    publishStatus();
  }

  // HH_260720 - Convert lateral offset into a bounded correction heading while reversing.
  double calculateLateralCorrectionHeading() const
  {
    return clamp(
      -lateral_to_heading_gain_ * lateral_error_m_,
      -maximum_approach_angle_rad_, maximum_approach_angle_rad_);
  }

  void handleTagDetectionLoss()
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "parking tag detection lost");
    // HH_260720 - Return to stopped tag waiting after a sustained early loss.
    if (stateElapsed() > tag_timeout_s_ * 4.0) {
      transitionTo(State::WAITING_FOR_TAG);
    }
  }

  void fail()
  {
    publishStop();
    transitionTo(State::ERROR);
  }

  void publishStop()
  {
    avg_msgs::msg::AvgTwist zero;
    cmd_pub_->publish(zero);
  }

  // HH_260720 - Convert odometry estimates into base-frame parking visualization geometry.
  void updateVisualizationGeometry()
  {
    const double cy = std::cos(vehicle_odometry_yaw_rad_);
    const double sy = std::sin(vehicle_odometry_yaw_rad_);
    const double vehicle_to_tag_x_m = tag_odometry_x_m_ - vehicle_odometry_x_m_;
    const double vehicle_to_tag_y_m = tag_odometry_y_m_ - vehicle_odometry_y_m_;
    tag_base_x_m_ = cy * vehicle_to_tag_x_m + sy * vehicle_to_tag_y_m;
    tag_base_y_m_ = -sy * vehicle_to_tag_x_m + cy * vehicle_to_tag_y_m;
    const double axis_base_yaw_rad =
      parking_axis_odometry_yaw_rad_ - vehicle_odometry_yaw_rad_;
    parking_axis_base_x_ = std::cos(axis_base_yaw_rad);
    parking_axis_base_y_ = std::sin(axis_base_yaw_rad);
    publishMarkers();
  }

  void publishMarkers()
  {
    if (!marker_pub_ || marker_pub_->get_subscription_count() == 0) {return;}

    visualization_msgs::msg::MarkerArray arr;
    const auto stamp = now();

    auto makeBase = [&](int id, int type) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = base_frame_id_;
        m.header.stamp = stamp;
        m.ns = "apriltag_parking_controller";
        m.id = id;
        m.type = type;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.lifetime = rclcpp::Duration::from_seconds(0.5);
        m.pose.orientation.w = 1.0;
        return m;
      };

    // HH_260720 - Draw the parking axis from the tag toward the robot.
    {
      auto m = makeBase(0, visualization_msgs::msg::Marker::LINE_STRIP);
      m.scale.x = 0.02;
      m.color.g = 1.0; m.color.a = 0.8;
      geometry_msgs::msg::Point p0, p1;
      p0.x = tag_base_x_m_; p0.y = tag_base_y_m_; p0.z = 0.02;
      p1.x = tag_base_x_m_ + parking_axis_base_x_ * 2.0;
      p1.y = tag_base_y_m_ + parking_axis_base_y_ * 2.0;
      p1.z = 0.02;
      m.points = {p0, p1};
      arr.markers.push_back(m);
    }

    // HH_260720 - Draw the observed parking-tag position.
    {
      auto m = makeBase(1, visualization_msgs::msg::Marker::SPHERE);
      m.scale.x = m.scale.y = m.scale.z = 0.06;
      m.color.r = 1.0; m.color.a = 0.9;
      m.pose.position.x = tag_base_x_m_;
      m.pose.position.y = tag_base_y_m_;
      m.pose.position.z = 0.02;
      arr.markers.push_back(m);
    }

    // HH_260720 - Draw lateral error from the robot origin to the parking axis.
    {
      auto m = makeBase(2, visualization_msgs::msg::Marker::LINE_STRIP);
      m.scale.x = 0.015;
      m.color.r = 1.0; m.color.g = 1.0; m.color.a = 0.9;
      geometry_msgs::msg::Point foot, origin;
      foot.x = tag_base_x_m_ +
        parking_axis_base_x_ * distance_along_parking_axis_m_;
      foot.y = tag_base_y_m_ +
        parking_axis_base_y_ * distance_along_parking_axis_m_;
      foot.z = 0.02;
      origin.x = 0.0; origin.y = 0.0; origin.z = 0.02;
      m.points = {origin, foot};
      arr.markers.push_back(m);
    }

    // HH_260720 - Draw the current controller phase and error values.
    {
      auto m = makeBase(3, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      m.scale.z = 0.12;
      m.color.r = m.color.g = m.color.b = 1.0; m.color.a = 1.0;
      m.pose.position.z = 0.5;
      char buf[96];
      snprintf(
        buf, sizeof(buf), "%s\nd=%.2f ey=%.3f eth=%.1fdeg",
        stateName(state_), distance_along_parking_axis_m_, lateral_error_m_,
        heading_error_rad_ * 180.0 / M_PI);
      m.text = buf;
      arr.markers.push_back(m);
    }

    marker_pub_->publish(arr);
  }

  void transitionTo(State s)
  {
    if (state_ != s) {
      RCLCPP_INFO(
        get_logger(), "AprilTag parking state: %s -> %s",
        stateName(state_), stateName(s));
      state_ = s;
      state_enter_time_ = now();
      publishServiceState(true);
      publishStatus(true);
    }
  }

  double stateElapsed() const
  {
    return (const_cast<AprilTagParkingControllerNode *>(this)->now() - state_enter_time_).seconds();
  }

  static const char * stateName(State s)
  {
    switch (s) {
      case State::IDLE:           return "IDLE";
      case State::WAITING_FOR_TAG: return "WAITING_FOR_TAG";
      case State::TAG_GUIDED_REVERSE: return "TAG_GUIDED_REVERSE";
      case State::FINAL_YAW_ALIGNMENT: return "FINAL_YAW_ALIGNMENT";
      case State::WAITING_FOR_CHARGING: return "WAITING_FOR_CHARGING";
      case State::PARKED:          return "PARKED";
      case State::ERROR:           return "ERROR";
    }
    return "?";
  }

  void publishServiceState(bool force = false)
  {
    if (state_ == State::IDLE || state_ == State::ERROR) {
      return;
    }
    const auto current_time = now();
    if (!force && last_service_state_time_.nanoseconds() > 0 &&
      (current_time - last_service_state_time_).seconds() <
      1.0 / std::max(status_rate_hz_, 0.1))
    {
      return;
    }
    avg_msgs::msg::AvgServiceState message;
    if (state_ == State::PARKED) {
      // HH_260721 - Keep AprilTag and default parking completion semantics identical.
      message.state = charging_detected_ ?
        avg_msgs::msg::AvgServiceState::CHARGING :
        avg_msgs::msg::AvgServiceState::DROP_ZONE_WAIT;
      message.state_name = charging_detected_ ? "CHARGING" : "DROP_ZONE_WAIT";
    } else if (state_ == State::WAITING_FOR_CHARGING) {
      message.state = avg_msgs::msg::AvgServiceState::WAITING_FOR_CHARGING;
      message.state_name = "WAITING_FOR_CHARGING";
    } else {
      message.state = avg_msgs::msg::AvgServiceState::DROP_ZONE_PARKING;
      message.state_name = "DROP_ZONE_PARKING";
    }
    message.description = std::string("apriltag_parking_controller:") + stateName(state_);
    service_state_pub_->publish(message);
    last_service_state_time_ = current_time;
  }

  void publishStatus(bool force = false)
  {
    const auto current_time = now();
    if (!force && last_status_time_.nanoseconds() > 0 &&
      (current_time - last_status_time_).seconds() < 1.0 / std::max(status_rate_hz_, 0.1))
    {
      return;
    }
    char buf[320];
    snprintf(
      buf, sizeof(buf), "phase=%s tag_distance_m=%.3f axis_distance_m=%.3f "
      "lateral_m=%.3f heading_rad=%.3f remaining_tag_m=%.3f "
      "configured_stop_tag_m=%.3f stop_reason=%s stop_trigger_tag_m=%.3f "
      "retry=%d charging=%s",
      stateName(state_), tag_camera_distance_valid_ ? tag_camera_distance_m_ : -1.0,
      distance_along_parking_axis_m_, lateral_error_m_, heading_error_rad_,
      tag_camera_distance_valid_ ? camrod_control::tagApproachRemainingDistance(
        tag_camera_distance_m_, translation_stop_tag_distance_m_) : -1.0,
      translation_stop_tag_distance_m_, translation_stop_reason_.c_str(),
      translation_stop_trigger_tag_distance_m_, retries_,
      charging_detected_ ? "true" : "false");

    uint8_t level = avg_msgs::msg::ModuleState::OK;
    if (state_ == State::ERROR) {
      level = avg_msgs::msg::ModuleState::ERROR;
    }

    avg_msgs::msg::ModuleState module_state;
    module_state.stamp = current_time;
    // HH_260720 - Identify the selected implementation directly in shared status streams.
    module_state.module_name = "apriltag_parking_controller";
    module_state.level = level;
    // HH_260721 - Expose parking progress without overloading the health level.
    module_state.operating_state = stateName(state_);
    module_state.message = buf;
    status_pub_->publish(module_state);

    diagnostic_msgs::msg::DiagnosticArray diagnostic_array;
    diagnostic_array.header.stamp = current_time;
    diagnostic_msgs::msg::DiagnosticStatus diagnostic_status;
    diagnostic_status.name = "parking/apriltag_parking_controller";
    diagnostic_status.hardware_id = "camrod_control";
    diagnostic_status.level = level == avg_msgs::msg::ModuleState::ERROR ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      level == avg_msgs::msg::ModuleState::WARN ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostic_status.message = buf;
    diagnostic_msgs::msg::KeyValue category;
    category.key = "category";
    category.value = "parking";
    diagnostic_status.values.push_back(category);
    diagnostic_array.status.push_back(diagnostic_status);
    diagnostics_pub_->publish(diagnostic_array);
    // HH_260819 - Re-announce recoverable parked phases on the same 2 Hz
    // heartbeat so a restarted UI does not depend on a past transition edge.
    if (state_ == State::PARKED || state_ == State::WAITING_FOR_CHARGING) {
      publishServiceState();
    }
    last_status_time_ = current_time;
  }

  // HH_260720 - Descriptive parameters and runtime state for maintainability.
  std::string base_frame_id_, cmd_vel_topic_, tag_pose_topic_, odom_topic_;
  std::string platform_status_topic_, operation_topic_, status_topic_;
  std::string diagnostics_topic_, service_state_topic_;
  bool require_charging_for_completion_{true};
  bool stop_when_charging_{true};
  double heading_gain_, lateral_to_heading_gain_;
  double maximum_angular_speed_radps_, maximum_approach_angle_rad_;
  double reverse_approach_speed_mps_, final_insertion_speed_mps_;
  double slowdown_start_tag_distance_m_{0.80};
  double translation_stop_tag_distance_m_{0.40};
  double minimum_approach_turn_radius_m_{0.85};
  double final_yaw_angular_speed_radps_{0.20};
  // HH_260819 - Deprecated robot-center distance parameters remain declared so
  // older override files and frame-contract tests continue to load unchanged.
  double slowdown_start_distance_m_;
  double slowdown_start_remaining_distance_m_{0.60};
  bool invert_wz_in_reverse_{false};
  double final_heading_tolerance_rad_, final_lateral_tolerance_m_;
  double final_yaw_settle_hold_s_{0.8};
  double final_yaw_settle_max_rate_degps_{3.0};
  double final_insertion_start_distance_m_, parked_distance_from_tag_m_;
  double tag_timeout_s_, odometry_timeout_s_, tag_wait_timeout_s_;
  double retry_forward_distance_m_, pose_filter_gain_, control_rate_hz_, status_rate_hz_;
  int max_retries_{5};

  // HH_260720 - State-machine bookkeeping.
  State state_{State::IDLE};
  rclcpp::Time state_enter_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_service_state_time_{0, 0, RCL_ROS_TIME};
  int retries_{0};

  // HH_260720 - Fixed tag and parking-axis estimate in the odometry frame.
  double tag_odometry_x_m_{0.0}, tag_odometry_y_m_{0.0};
  double parking_axis_odometry_yaw_rad_{0.0};
  bool axis_valid_{false};
  double axis_full_trust_distance_m_, axis_minimum_trust_distance_m_;
  double axis_minimum_filter_weight_;

  // HH_260720 - Parking errors recomputed from odometry on every control tick.
  double distance_along_parking_axis_m_{0.0};
  double lateral_error_m_{0.0}, heading_error_rad_{0.0};
  double tag_camera_distance_m_{0.0};
  bool tag_camera_distance_valid_{false};
  // HH_260824 - Preserve the exact one-way stop cause and trigger sample for
  // field diagnosis; the live Tag distance may continue changing after zero.
  std::string translation_stop_reason_{"none"};
  double translation_stop_trigger_tag_distance_m_{-1.0};
  rclcpp::Time last_tag_time_{0, 0, RCL_ROS_TIME};

  // HH_260819 - Vehicle odometry drives final yaw after camera-range translation latches off.
  double vehicle_odometry_x_m_{0.0}, vehicle_odometry_y_m_{0.0};
  double vehicle_odometry_yaw_rad_{0.0};
  bool odom_valid_{false};
  rclcpp::Time last_odometry_time_{0, 0, RCL_ROS_TIME};
  camrod_control::YawAlignmentSettling yaw_alignment_settling_;
  bool yaw_alignment_settled_logged_{false};

  bool charging_detected_{false};

  // HH_260720 - Base-frame visualization geometry.
  std::string odom_frame_id_;
  bool viz_enabled_{true};
  double tag_base_x_m_{0.0}, tag_base_y_m_{0.0};
  double parking_axis_base_x_{1.0}, parking_axis_base_y_{0.0};
  nav_msgs::msg::Path robot_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // HH_260720 - ROS interfaces.
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<avg_msgs::msg::AvgAprilTagPose>::SharedPtr tag_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgPlatformStatus>::SharedPtr platform_status_sub_;
  rclcpp::Subscription<avg_msgs::msg::MotionOperation>::SharedPtr operation_sub_;
  rclcpp::Publisher<avg_msgs::msg::AvgTwist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<avg_msgs::msg::ModuleState>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgServiceState>::SharedPtr service_state_pub_;
  rclcpp::Service<avg_msgs::srv::RequestMotionOperation>::SharedPtr operation_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagParkingControllerNode>());
  rclcpp::shutdown();
  return 0;
}
