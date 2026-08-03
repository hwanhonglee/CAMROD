// HH_260720 - Keep every control executable in the package's single source directory.
// HH_260720 - Rename the bench controller as the final AprilTag parking controller.
//
// HH_260720 - The controller waits for a rear-camera parking tag, estimates the
// station axis in odometry, reverses with heading/lateral feedback, and uses a
// short odometry-only final insertion when the tag leaves the camera field of view.
// It never bypasses /control/cmd_vel_safety_gate.
//
// State machine:
//   WAITING_FOR_TAG -> TAG_GUIDED_REVERSE -> FINAL_REVERSE_INSERTION -> PARKED
//                              |                         |
//                              +-> RETRY_FORWARD_EXIT <--+
//                                           |
//                                         ERROR

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
    FINAL_REVERSE_INSERTION, PARKED, RETRY_FORWARD_EXIT, ERROR };

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
    slowdown_start_distance_m_ =
      declare_parameter<double>("slowdown_start_distance_m", 1.443);

    // HH_260720 - Final insertion starts only inside the configured alignment tolerances.
    final_heading_tolerance_rad_ =
      declare_parameter<double>("final_heading_tolerance_rad", 0.10);
    final_lateral_tolerance_m_ =
      declare_parameter<double>("final_lateral_tolerance_m", 0.03);

    // HH_260720 - Distances are measured along the parking-tag normal axis.
    final_insertion_start_distance_m_ = declare_parameter<double>(
      "final_insertion_start_distance_m", 0.993);
    parked_distance_from_tag_m_ = declare_parameter<double>(
      "parked_distance_from_tag_m", 0.943);
    tag_timeout_s_ = declare_parameter<double>("tag_timeout_s", 0.5);
    tag_wait_timeout_s_ = declare_parameter<double>("tag_wait_timeout_s", 10.0);
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
          publishServiceState();
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
    RCLCPP_INFO(
      get_logger(),
      "apriltag_parking_controller ready: command=%s odometry=%s tag=%s",
      cmd_vel_topic_.c_str(), odom_topic_.c_str(), tag_pose_topic_.c_str());
    publishStatus(true);
  }

private:
  // HH_260720 - Transform tag observations and estimate a fixed odometry-frame parking axis.
  void tagCallback(const avg_msgs::msg::AvgAprilTagPose::SharedPtr msg)
  {
    // HH_260720 - TF remains a standard ROS boundary; convert only for that call.
    const geometry_msgs::msg::PoseStamped tag_pose =
      avg_msgs::conversions::toRos(msg->pose);
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
    if (!odom_valid_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "odometry unavailable; cannot estimate the parking axis");
      return;
    }
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
    last_tag_time_ = now();
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

  // HH_260720 - Start and cancel are shared by topic and service interfaces.
  bool startParking(const std::string & source, std::string & detail)
  {
    if (state_ != State::IDLE && state_ != State::ERROR && state_ != State::PARKED) {
      detail = "AprilTag parking is already active";
      return false;
    }
    if (!odom_valid_) {
      detail = "fresh odometry is unavailable";
      transitionTo(State::ERROR);
      return false;
    }
    retries_ = 0;
    axis_valid_ = false;
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
    const bool tag_fresh = axis_valid_ &&
      (now() - last_tag_time_).seconds() < tag_timeout_s_;

    // HH_260720 - Refresh errors from odometry every tick after the first valid tag.
    if (axis_valid_ && odom_valid_) {
      updateParkingErrorsFromOdometry();
      if (viz_enabled_) {updateVisualizationGeometry();}
    }

    avg_msgs::msg::AvgTwist command;

    switch (state_) {
      case State::IDLE:
      case State::PARKED:
      case State::ERROR:
        publishStatus();
        return;

      // HH_260720 - Wait stopped until the configured parking tag is visible.
      case State::WAITING_FOR_TAG: {
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

      // HH_260720 - Reverse along the observed tag axis with lateral and heading correction.
      case State::TAG_GUIDED_REVERSE: {
          if (!tag_fresh) {
            // HH_260720 - Near the tag, continue with bounded odometry-only final insertion.
            if (axis_valid_ &&
              distance_along_parking_axis_m_ < final_insertion_start_distance_m_ * 1.5)
            {
              startFinalReverseInsertion();
            } else {
              handleTagDetectionLoss();
            }
            break;
          }
          // HH_260720 - Stop before passing the configured parking distance.
          if (distance_along_parking_axis_m_ < parked_distance_from_tag_m_) {
            publishStop();
            if (!require_charging_for_completion_ || charging_detected_) {
              transitionTo(State::PARKED);
            } else {
              RCLCPP_WARN(
                get_logger(),
                "parking distance reached without charging feedback; retrying");
              startRetryForwardExit();
            }
            break;
          }
          const double correction_heading_rad = calculateLateralCorrectionHeading();
          if (distance_along_parking_axis_m_ < final_insertion_start_distance_m_) {
            // HH_260720 - Reject straight final insertion while outside alignment tolerances.
            if (std::fabs(lateral_error_m_) > final_lateral_tolerance_m_ ||
              std::fabs(heading_error_rad_) > final_heading_tolerance_rad_)
            {
              RCLCPP_WARN(
                get_logger(),
                "final insertion alignment failed: lateral=%.3fm heading=%.3frad; retrying",
                lateral_error_m_, heading_error_rad_);
              startRetryForwardExit();
            } else {
              startFinalReverseInsertion();
            }
            break;
          }
          // HH_260720 - Slow proportionally while correcting lateral and heading errors.
          double reverse_speed_mps = reverse_approach_speed_mps_;
          if (distance_along_parking_axis_m_ < slowdown_start_distance_m_) {
            reverse_speed_mps = final_insertion_speed_mps_ +
              (reverse_approach_speed_mps_ - final_insertion_speed_mps_) *
              (distance_along_parking_axis_m_ - final_insertion_start_distance_m_) /
              std::max(slowdown_start_distance_m_ - final_insertion_start_distance_m_, 1e-3);
            reverse_speed_mps = clamp(
              reverse_speed_mps, final_insertion_speed_mps_, reverse_approach_speed_mps_);
          }
          double angular_speed_radps = clamp(
            heading_gain_ * normalizeAngle(heading_error_rad_ - correction_heading_rad),
            -maximum_angular_speed_radps_, maximum_angular_speed_radps_);
          if (invert_wz_in_reverse_) {angular_speed_radps = -angular_speed_radps;}
          command.linear.x = -reverse_speed_mps;
          command.angular.z = angular_speed_radps;
          break;
        }

      // HH_260720 - Finish with a bounded straight reverse using odometry and charging feedback.
      case State::FINAL_REVERSE_INSERTION: {
          const double traveled = std::hypot(
            vehicle_odometry_x_m_ - final_insertion_start_odometry_x_m_,
            vehicle_odometry_y_m_ - final_insertion_start_odometry_y_m_);
          const bool distance_done = traveled >= final_insertion_target_distance_m_;
          const bool charging_done =
            require_charging_for_completion_ && charging_detected_;

          if (charging_done || (!require_charging_for_completion_ && distance_done)) {
            publishStop();
            transitionTo(State::PARKED);
            RCLCPP_INFO(get_logger(), "AprilTag parking completed after final %.3fm", traveled);
            break;
          }
          // HH_260720 - Retry if the charger is still absent beyond the bounded overtravel margin.
          if (require_charging_for_completion_ &&
            traveled > final_insertion_target_distance_m_ * 1.2 + 0.03)
          {
            RCLCPP_WARN(get_logger(), "final insertion completed without charging; retrying");
            startRetryForwardExit();
            break;
          }
          command.linear.x = -final_insertion_speed_mps_;
          break;
        }

      // HH_260720 - Drive forward a bounded distance before waiting for the tag again.
      case State::RETRY_FORWARD_EXIT: {
          const double traveled = std::hypot(
            vehicle_odometry_x_m_ - retry_start_odometry_x_m_,
            vehicle_odometry_y_m_ - retry_start_odometry_y_m_);
          if (traveled >= retry_forward_distance_m_) {
            // HH_260720 - Retain the odometry-frame axis for immediate reuse after reacquisition.
            transitionTo(State::WAITING_FOR_TAG);
            break;
          }
          command.linear.x = reverse_approach_speed_mps_;
          break;
        }
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

  void startFinalReverseInsertion()
  {
    if (!odom_valid_) {
      RCLCPP_ERROR(get_logger(), "odometry unavailable; final reverse insertion is blocked");
      fail();
      return;
    }
    final_insertion_start_odometry_x_m_ = vehicle_odometry_x_m_;
    final_insertion_start_odometry_y_m_ = vehicle_odometry_y_m_;
    // HH_260720 - Reverse only the remaining tag-axis distance to the configured stop point.
    final_insertion_target_distance_m_ = std::max(
      distance_along_parking_axis_m_ - parked_distance_from_tag_m_, 0.0);
    RCLCPP_INFO(
      get_logger(), "final insertion started: remaining=%.3fm",
      final_insertion_target_distance_m_);
    transitionTo(State::FINAL_REVERSE_INSERTION);
  }

  void startRetryForwardExit()
  {
    if (++retries_ > max_retries_) {
      RCLCPP_ERROR(get_logger(), "maximum AprilTag parking retries exceeded");
      fail();
      return;
    }
    retry_start_odometry_x_m_ = vehicle_odometry_x_m_;
    retry_start_odometry_y_m_ = vehicle_odometry_y_m_;
    RCLCPP_WARN(get_logger(), "AprilTag parking retry %d/%d", retries_, max_retries_);
    transitionTo(State::RETRY_FORWARD_EXIT);
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
      publishServiceState();
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
      case State::FINAL_REVERSE_INSERTION: return "FINAL_REVERSE_INSERTION";
      case State::PARKED:          return "PARKED";
      case State::RETRY_FORWARD_EXIT: return "RETRY_FORWARD_EXIT";
      case State::ERROR:           return "ERROR";
    }
    return "?";
  }

  void publishServiceState()
  {
    if (state_ == State::IDLE || state_ == State::ERROR) {
      return;
    }
    avg_msgs::msg::AvgServiceState message;
    if (state_ == State::PARKED) {
      // HH_260721 - Keep AprilTag and default parking completion semantics identical.
      message.state = charging_detected_ ?
        avg_msgs::msg::AvgServiceState::CHARGING :
        avg_msgs::msg::AvgServiceState::DROP_ZONE_WAIT;
      message.state_name = charging_detected_ ? "CHARGING" : "DROP_ZONE_WAIT";
    } else {
      message.state = avg_msgs::msg::AvgServiceState::DROP_ZONE_PARKING;
      message.state_name = "DROP_ZONE_PARKING";
    }
    message.description = std::string("apriltag_parking_controller:") + stateName(state_);
    service_state_pub_->publish(message);
  }

  void publishStatus(bool force = false)
  {
    const auto current_time = now();
    if (!force && last_status_time_.nanoseconds() > 0 &&
      (current_time - last_status_time_).seconds() < 1.0 / std::max(status_rate_hz_, 0.1))
    {
      return;
    }
    char buf[128];
    snprintf(
      buf, sizeof(buf), "phase=%s distance_m=%.3f lateral_m=%.3f heading_rad=%.3f "
      "retry=%d charging=%s",
      stateName(state_), distance_along_parking_axis_m_, lateral_error_m_,
      heading_error_rad_, retries_, charging_detected_ ? "true" : "false");

    uint8_t level = avg_msgs::msg::ModuleState::OK;
    if (state_ == State::ERROR) {
      level = avg_msgs::msg::ModuleState::ERROR;
    } else if (state_ == State::RETRY_FORWARD_EXIT) {
      // HH_260721 - A retry is degraded but recoverable; normal parking progress remains OK.
      level = avg_msgs::msg::ModuleState::WARN;
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
    last_status_time_ = current_time;
  }

  // HH_260720 - Descriptive parameters and runtime state for maintainability.
  std::string base_frame_id_, cmd_vel_topic_, tag_pose_topic_, odom_topic_;
  std::string platform_status_topic_, operation_topic_, status_topic_;
  std::string diagnostics_topic_, service_state_topic_;
  bool require_charging_for_completion_{true};
  double heading_gain_, lateral_to_heading_gain_;
  double maximum_angular_speed_radps_, maximum_approach_angle_rad_;
  double reverse_approach_speed_mps_, final_insertion_speed_mps_;
  double slowdown_start_distance_m_;
  bool invert_wz_in_reverse_{false};
  double final_heading_tolerance_rad_, final_lateral_tolerance_m_;
  double final_insertion_start_distance_m_, parked_distance_from_tag_m_;
  double tag_timeout_s_, tag_wait_timeout_s_;
  double retry_forward_distance_m_, pose_filter_gain_, control_rate_hz_, status_rate_hz_;
  int max_retries_{5};

  // HH_260720 - State-machine bookkeeping.
  State state_{State::IDLE};
  rclcpp::Time state_enter_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
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
  rclcpp::Time last_tag_time_{0, 0, RCL_ROS_TIME};

  // HH_260720 - Vehicle odometry and bounded insertion/retry start positions.
  double vehicle_odometry_x_m_{0.0}, vehicle_odometry_y_m_{0.0};
  double vehicle_odometry_yaw_rad_{0.0};
  bool odom_valid_{false};
  double final_insertion_start_odometry_x_m_{0.0};
  double final_insertion_start_odometry_y_m_{0.0};
  double final_insertion_target_distance_m_{0.0};
  double retry_start_odometry_x_m_{0.0}, retry_start_odometry_y_m_{0.0};

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
