// HH_260428: Ranger platform bridge — normalises ranger-specific CAN-derived topics to the
// /platform/status/* interface consumed by the CAMROD stack.
//
// Individual status topics (functional — consumed by other nodes):
//   /platform/status/odometry  (nav_msgs/Odometry)          <- localization_input_adapter
//   /platform/status/velocity  (geometry_msgs/TwistStamped) <- platform_velocity_converter
//   /platform/status/wheel     (geometry_msgs/TwistStamped) <- diagnostics
//   /platform/status/estop     (std_msgs/Bool)               <- cmd_vel_gate
//   /platform/status/is_charging (std_msgs/Bool)              <- docking/mission
//   /docking/is_charging       (std_msgs/Bool)                <- existing docking contract
//
// Comprehensive status topic (all DBC data aggregated):
//   /platform/status           (avg_msgs/AvgPlatformStatus)  <- monitoring / external tools
//     vehicle_state, control_mode, error_code, battery_voltage  (CAN 0x211)
//     battery_state                                             (CAN 0x361 via /battery_state)
//     motion_mode                                               (CAN 0x291)
//     motor_rpm[8]   (all motors,     CAN 0x251-0x258)
//     motor_speed[4] (drive motors,   CAN 0x281)
//     motor_angle[4] (steering motors,CAN 0x271)
//     odometry, velocity, wheel, estop  (aggregated from above)
//
// Odom fallback:
//   Primary  : odom_topic_name     (default /odom,        ranger_base CAN output)
//   Fallback : odom_fallback_topic (default /rmp401/odom, substitute RMP401 platform)
//   Falls back when primary is silent > odom_fallback_timeout_s seconds.

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <avg_msgs/msg/avg_platform_status.hpp>
#include <avg_msgs/msg/module_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <ranger_msgs/msg/actuator_state_array.hpp>
#include <ranger_msgs/msg/system_state.hpp>

namespace camrod_platform
{

class RangerPlatformBridgeNode : public rclcpp::Node
{
public:
  RangerPlatformBridgeNode()
  : Node("ranger_platform_bridge")
  {
    // HH_260428: Odom input parameters — primary (ranger) and fallback (substitute platform).
    odom_input_topic_      = declare_parameter<std::string>("odom_topic_name",      "/odom");
    odom_fallback_topic_   = declare_parameter<std::string>("odom_fallback_topic",  "/rmp401/odom");
    // HH_260617: Use canonical `_s` suffix for duration parameters.
    odom_fallback_timeout_s_ = declare_parameter<double>("odom_fallback_timeout_s", 1.0);

    actuator_state_topic_  = declare_parameter<std::string>("actuator_state_topic", "/actuator_state");
    system_state_topic_    = declare_parameter<std::string>("system_state_topic",   "/system_state");
    publish_topics_        = declare_parameter<bool>("publish_platform_status_topics", true);
    status_odom_topic_     = declare_parameter<std::string>(
      "status_odom_topic",      "/platform/status/odometry");
    status_velocity_topic_ = declare_parameter<std::string>(
      "status_velocity_topic",  "/platform/status/velocity");
    status_wheel_topic_    = declare_parameter<std::string>(
      "status_wheel_topic",     "/platform/status/wheel");
    status_estop_topic_    = declare_parameter<std::string>(
      "status_estop_topic",     "/platform/status/estop");
    // HH_260617: Normalize Ranger BMS CAN 0x361 into platform and docking status topics.
    battery_state_topic_   = declare_parameter<std::string>(
      "battery_state_topic",    "/battery_state");
    status_battery_topic_  = declare_parameter<std::string>(
      "status_battery_topic",   "/platform/status/battery_state");
    status_charging_topic_ = declare_parameter<std::string>(
      "status_charging_topic",  "/platform/status/is_charging");
    docking_charging_topic_ = declare_parameter<std::string>(
      "docking_charging_topic", "/docking/is_charging");
    publish_docking_charging_ = declare_parameter<bool>(
      "publish_docking_charging", true);
    charging_current_threshold_a_ = declare_parameter<double>(
      "charging_current_threshold_a", 0.3);
    charging_current_positive_is_charging_ = declare_parameter<bool>(
      "charging_current_positive_is_charging", true);
    charging_min_consecutive_samples_ = declare_parameter<int>(
      "charging_min_consecutive_samples", 2);
    // HH_260428: Comprehensive aggregated DBC status topic (AvgPlatformStatus).
    platform_status_topic_ = declare_parameter<std::string>(
      "platform_status_topic",  "/platform/status");
    // HH_260506: Frame id used by aggregated AvgPlatformStatus header.
    status_frame_id_      = declare_parameter<std::string>(
      "status_frame_id", "robot_base_link");
    estop_on_exception_    = declare_parameter<bool>("estop_on_exception_state", true);
    estop_on_error_code_   = declare_parameter<bool>("estop_on_error_code",      false);

    if (!publish_topics_) {
      RCLCPP_INFO(get_logger(), "platform status topics disabled");
      return;
    }

    // HH_260428: Initialize last_primary_odom_time_ via node clock to match
    // the clock type used in subsequent this->now() calls (sim vs wall time).
    last_primary_odom_time_ = this->now();

    using std::placeholders::_1;

    // Individual functional publishers
    odom_pub_     = create_publisher<nav_msgs::msg::Odometry>(
      status_odom_topic_,     rclcpp::QoS(50));
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      status_velocity_topic_, rclcpp::QoS(50));
    wheel_pub_    = create_publisher<geometry_msgs::msg::TwistStamped>(
      status_wheel_topic_,    rclcpp::QoS(50));
    estop_pub_    = create_publisher<std_msgs::msg::Bool>(
      status_estop_topic_,    rclcpp::QoS(10));
    battery_pub_  = create_publisher<sensor_msgs::msg::BatteryState>(
      status_battery_topic_,  rclcpp::QoS(10));
    charging_pub_ = create_publisher<std_msgs::msg::Bool>(
      status_charging_topic_, rclcpp::QoS(10));
    if (publish_docking_charging_) {
      docking_charging_pub_ = create_publisher<std_msgs::msg::Bool>(
        docking_charging_topic_, rclcpp::QoS(10));
    }

    // HH_260428: Comprehensive DBC status publisher — aggregates all available CAN data
    // (0x211, 0x251-0x268, 0x271, 0x281, 0x291) into a single AvgPlatformStatus message.
    platform_status_pub_ = create_publisher<avg_msgs::msg::AvgPlatformStatus>(
      platform_status_topic_, rclcpp::QoS(10));

    // HH_260428: Primary odom subscription (ranger_base /odom, CAN 0x221/0x311/0x312).
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RangerPlatformBridgeNode::onPrimaryOdom, this, _1));

    // HH_260428: Fallback odom subscription (/rmp401/odom from substitute platform).
    // Only created when fallback topic is non-empty and different from primary.
    if (!odom_fallback_topic_.empty() && odom_fallback_topic_ != odom_input_topic_) {
      odom_fallback_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_fallback_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RangerPlatformBridgeNode::onFallbackOdom, this, _1));
    }

    // HH_260428: Actuator state (CAN 0x251-0x258 RPM/current + 0x271 angles + 0x281 speeds).
    actuator_state_sub_ = create_subscription<ranger_msgs::msg::ActuatorStateArray>(
      actuator_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RangerPlatformBridgeNode::onActuatorState, this, _1));

    // HH_260428: System state (CAN 0x211 vehicle_state/control_mode/error_code/battery + 0x291 motion_mode).
    system_state_sub_ = create_subscription<ranger_msgs::msg::SystemState>(
      system_state_topic_, rclcpp::QoS(10),
      std::bind(&RangerPlatformBridgeNode::onSystemState, this, _1));

    // HH_260617: ranger_base publishes BMS basic feedback (CAN 0x361) as BatteryState.
    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      battery_state_topic_, rclcpp::QoS(10),
      std::bind(&RangerPlatformBridgeNode::onBatteryState, this, _1));

    RCLCPP_INFO(
      get_logger(),
      "ranger_platform_bridge ready: "
      "odom=%s fallback=%s (%.1fs) actuator=%s system_state=%s battery=%s"
      " -> odom=%s vel=%s wheel=%s estop=%s battery=%s charging=%s docking_charging=%s status=%s frame=%s",
      odom_input_topic_.c_str(), odom_fallback_topic_.c_str(), odom_fallback_timeout_s_,
      actuator_state_topic_.c_str(), system_state_topic_.c_str(), battery_state_topic_.c_str(),
      status_odom_topic_.c_str(), status_velocity_topic_.c_str(),
      status_wheel_topic_.c_str(), status_estop_topic_.c_str(),
      status_battery_topic_.c_str(), status_charging_topic_.c_str(),
      publish_docking_charging_ ? docking_charging_topic_.c_str() : "(disabled)",
      platform_status_topic_.c_str(), status_frame_id_.c_str());
  }

private:
  // HH_260428: Publish individual functional topics from odom source.
  // Called by both primary and fallback odom callbacks.
  void publishOdomStatus(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
  {
    odom_pub_->publish(*msg);

    geometry_msgs::msg::TwistStamped vel;
    vel.header = msg->header;
    vel.twist  = msg->twist.twist;
    velocity_pub_->publish(vel);

    // Cache latest odom for AvgPlatformStatus aggregation
    latest_odom_ = msg;
    publishAggregatedStatus(msg->header.stamp);
  }

  // HH_260428: Primary odom handler (ranger_base /odom, CAN 0x221/0x311/0x312).
  // Updates last_primary_odom_time_ and clears fallback flag on each message.
  void onPrimaryOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    last_primary_odom_time_ = this->now();
    if (!primary_odom_active_) {
      primary_odom_active_ = true;
      RCLCPP_INFO(get_logger(), "primary odom active: %s", odom_input_topic_.c_str());
    }
    if (using_fallback_) {
      using_fallback_ = false;
      RCLCPP_INFO(get_logger(), "primary odom restored, leaving fallback (%s)",
        odom_input_topic_.c_str());
    }
    publishOdomStatus(msg);
  }

  // HH_260428: Fallback odom handler (/rmp401/odom substitute platform).
  // Activates when primary has never published OR has been silent > odom_fallback_timeout_s.
  void onFallbackOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    bool should_fallback = !primary_odom_active_;

    if (primary_odom_active_) {
      const double elapsed = (this->now() - last_primary_odom_time_).seconds();
      should_fallback = (elapsed > odom_fallback_timeout_s_);
      if (should_fallback && !using_fallback_) {
        RCLCPP_WARN(
          get_logger(),
          "primary odom %s timed out (%.1f s > %.1f s), switching to fallback %s",
          odom_input_topic_.c_str(), elapsed, odom_fallback_timeout_s_,
          odom_fallback_topic_.c_str());
      }
    }

    if (should_fallback) {
      using_fallback_ = true;
      publishOdomStatus(msg);
    }
  }

  // HH_260428: Actuator state handler — id-based motor data extraction.
  //
  // Motor layout (ranger CAN DBC):
  //   id 0-3  = drive motors No.1-4 (CAN 0x281):
  //             motor_speeds [m/s]  -> wheel.linear.x (mean) + motor_speed[] array
  //             rpm, current        -> motor_rpm[] array
  //   id 4-7  = steering motors No.5-8 (CAN 0x271):
  //             motor_angles [rad]  -> wheel.angular.z (mean) + motor_angle[] array
  //
  // Note: upstream ranger_messenger.cpp fills all states with speed_1/angle_5 due to
  // a loop bug — id-based extraction is correct and handles future upstream SDK fixes.
  void onActuatorState(const ranger_msgs::msg::ActuatorStateArray::ConstSharedPtr msg)
  {
    if (msg->states.empty()) {
      return;
    }

    double drive_speed_sum = 0.0;
    double steer_angle_sum = 0.0;
    int    drive_count     = 0;
    int    steer_count     = 0;

    // HH_260428: Populate motor arrays for AvgPlatformStatus (CAN 0x251-0x258/0x271/0x281).
    std::vector<float> motor_rpm(msg->states.size());
    std::vector<float> motor_speed;
    std::vector<float> motor_angle;

    for (const auto & s : msg->states) {
      motor_rpm[s.id] = static_cast<float>(s.motor.rpm);

      if (s.id < 4) {
        // HH_260428: id 0-3 = drive motors (CAN 0x281), motor_speeds in m/s
        const double spd = static_cast<double>(s.motor.motor_speeds);
        drive_speed_sum += spd;
        ++drive_count;
        motor_speed.push_back(static_cast<float>(spd));
      } else {
        // HH_260428: id 4-7 = steering motors (CAN 0x271), motor_angles in rad
        const double ang = static_cast<double>(s.motor.motor_angles);
        steer_angle_sum += ang;
        ++steer_count;
        motor_angle.push_back(static_cast<float>(ang));
      }
    }

    geometry_msgs::msg::TwistStamped wheel;
    wheel.header = msg->header;
    if (drive_count > 0) {
      wheel.twist.linear.x  = drive_speed_sum / drive_count;
    }
    if (steer_count > 0) {
      wheel.twist.angular.z = steer_angle_sum / steer_count;
    }
    wheel_pub_->publish(wheel);

    // Cache for AvgPlatformStatus
    latest_wheel_       = wheel;
    latest_motor_rpm_   = std::move(motor_rpm);
    latest_motor_speed_ = std::move(motor_speed);
    latest_motor_angle_ = std::move(motor_angle);
  }

  // HH_260428: System state handler — derives estop from CAN 0x211 and caches all fields.
  // vehicle_state, control_mode, error_code, battery_voltage, motion_mode all saved for
  // AvgPlatformStatus aggregation.
  void onSystemState(const ranger_msgs::msg::SystemState::ConstSharedPtr msg)
  {
    // HH_260428: estop logic from CAN 0x211 vehicle_state + optional error_code.
    // VEHICLE_STATE_ESTOP always triggers; EXCEPTION triggers when estop_on_exception_=true.
    const bool is_estop =
      (msg->vehicle_state == ranger_msgs::msg::SystemState::VEHICLE_STATE_ESTOP);
    const bool is_exception =
      (msg->vehicle_state == ranger_msgs::msg::SystemState::VEHICLE_STATE_EXCEPTION);

    bool estop = is_estop;
    if (estop_on_exception_) {
      estop = estop || is_exception;
    }
    if (estop_on_error_code_) {
      estop = estop || (msg->error_code != 0);
    }

    std_msgs::msg::Bool estop_msg;
    estop_msg.data = estop;
    estop_pub_->publish(estop_msg);

    if (estop != last_estop_) {
      last_estop_ = estop;
      RCLCPP_WARN(
        get_logger(), "estop %s (vehicle_state=%d error_code=0x%04X)",
        estop ? "ACTIVE" : "clear",
        static_cast<int>(msg->vehicle_state),
        static_cast<unsigned>(msg->error_code));
    }

    // Cache all CAN 0x211 + 0x291 fields for AvgPlatformStatus
    latest_system_state_ = msg;
    publishAggregatedStatus(msg->header.stamp);
  }

  bool inferChargingFromBattery(const sensor_msgs::msg::BatteryState & msg) const
  {
    // HH_260617: Prefer explicit BatteryState status when a driver provides it.
    // The current upstream ranger_base leaves status UNKNOWN, so fall back to the
    // signed BMS current from CAN 0x361. ROS BatteryState convention is negative
    // while discharging; keep the sign configurable because AgileX manuals do not
    // state current sign direction consistently across models.
    if (msg.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING) {
      return true;
    }
    if (
      msg.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING ||
      msg.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING)
    {
      return false;
    }

    const double current = static_cast<double>(msg.current);
    if (!std::isfinite(current)) {
      return false;
    }
    if (charging_current_positive_is_charging_) {
      return current > charging_current_threshold_a_;
    }
    return current < -charging_current_threshold_a_;
  }

  void updateChargingDebounce(bool charging_sample)
  {
    // HH_260617: Debounce BMS current spikes before declaring charger contact.
    const int required = std::max(1, charging_min_consecutive_samples_);
    if (charging_sample == charging_debounced_) {
      charging_candidate_count_ = 0;
      charging_candidate_ = charging_sample;
      return;
    }
    if (charging_sample != charging_candidate_) {
      charging_candidate_ = charging_sample;
      charging_candidate_count_ = 1;
    } else {
      ++charging_candidate_count_;
    }
    if (charging_candidate_count_ >= required) {
      charging_debounced_ = charging_sample;
      charging_candidate_count_ = 0;
      RCLCPP_INFO(
        get_logger(), "charging status -> %s (threshold=%.2f A, positive_is_charging=%s)",
        charging_debounced_ ? "true" : "false",
        charging_current_threshold_a_,
        charging_current_positive_is_charging_ ? "true" : "false");
    }
  }

  void onBatteryState(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg)
  {
    battery_pub_->publish(*msg);

    const bool charging_sample = inferChargingFromBattery(*msg);
    updateChargingDebounce(charging_sample);

    std_msgs::msg::Bool charging_msg;
    charging_msg.data = charging_debounced_;
    charging_pub_->publish(charging_msg);
    if (docking_charging_pub_) {
      docking_charging_pub_->publish(charging_msg);
    }
  }

  // HH_260428: Publish AvgPlatformStatus aggregating all available CAN data.
  // Fields populated:
  //   odometry, velocity (from odom/fallback — CAN 0x221/0x311/0x312)
  //   wheel              (from actuator_state — CAN 0x281/0x271)
  //   estop              (derived from system_state — CAN 0x211)
  //   vehicle_state, control_mode, error_code, battery_voltage (CAN 0x211)
  //   motion_mode        (CAN 0x291, stored in SystemState.motion_mode)
  //   motor_rpm[8]       (all motors — CAN 0x251-0x258)
  //   motor_speed[4]     (drive motors No.1-4 — CAN 0x281)
  //   motor_angle[4]     (steering motors No.5-8 — CAN 0x271)
  //   state.level        (OK/WARN/ERROR based on vehicle_state/control_mode/error_code)
  void publishAggregatedStatus(const rclcpp::Time & stamp)
  {
    avg_msgs::msg::AvgPlatformStatus s;
    s.header.stamp = stamp;
    s.header.frame_id = status_frame_id_;

    // Odom and velocity from latest odom source
    if (latest_odom_) {
      s.odometry = *latest_odom_;
      s.velocity.header = latest_odom_->header;
      s.velocity.twist  = latest_odom_->twist.twist;
    }

    // Wheel from actuator state
    s.wheel = latest_wheel_;

    // Estop
    s.estop = last_estop_;

    // HH_260428: All CAN 0x211 fields and motion_mode (0x291) from SystemState.
    if (latest_system_state_) {
      s.vehicle_state    = latest_system_state_->vehicle_state;
      s.control_mode     = latest_system_state_->control_mode;
      s.error_code       = static_cast<uint16_t>(latest_system_state_->error_code);
      s.battery_voltage  = static_cast<double>(latest_system_state_->battery_voltage);
      s.motion_mode      = latest_system_state_->motion_mode;
    }

    // HH_260428: Motor arrays from actuator_state (CAN 0x251-0x258/0x271/0x281).
    s.motor_rpm   = latest_motor_rpm_;
    s.motor_speed = latest_motor_speed_;
    s.motor_angle = latest_motor_angle_;

    // HH_260428: ModuleState — OK when CAN control and no faults, WARN for RC/STANDBY,
    // ERROR for estop or exception or non-zero error_code.
    s.state.module_name = "platform";
    if (last_estop_ || (latest_system_state_ &&
      latest_system_state_->vehicle_state != ranger_msgs::msg::SystemState::VEHICLE_STATE_NORMAL))
    {
      s.state.level   = avg_msgs::msg::ModuleState::ERROR;
      s.state.message = last_estop_ ? "ESTOP active" : "vehicle_state EXCEPTION";
    } else if (latest_system_state_ && latest_system_state_->control_mode != 0x01 /* CAN */) {
      s.state.level   = avg_msgs::msg::ModuleState::WARN;
      s.state.message = "control_mode is not CAN";
    } else {
      s.state.level   = avg_msgs::msg::ModuleState::OK;
      s.state.message = "";
    }

    platform_status_pub_->publish(s);
  }

  // Odom source parameters and fallback state
  std::string  odom_input_topic_;
  std::string  odom_fallback_topic_;
  double       odom_fallback_timeout_s_{1.0};
  bool         primary_odom_active_{false};
  bool         using_fallback_{false};
  rclcpp::Time last_primary_odom_time_;

  // Topic parameters
  std::string  actuator_state_topic_;
  std::string  system_state_topic_;
  std::string  status_odom_topic_;
  std::string  status_velocity_topic_;
  std::string  status_wheel_topic_;
  std::string  status_estop_topic_;
  std::string  battery_state_topic_;
  std::string  status_battery_topic_;
  std::string  status_charging_topic_;
  std::string  docking_charging_topic_;
  std::string  platform_status_topic_;
  std::string  status_frame_id_;
  bool         publish_topics_{true};
  bool         estop_on_exception_{true};
  bool         estop_on_error_code_{false};
  bool         publish_docking_charging_{true};
  bool         charging_current_positive_is_charging_{true};
  double       charging_current_threshold_a_{0.3};
  int          charging_min_consecutive_samples_{2};
  bool         charging_debounced_{false};
  bool         charging_candidate_{false};
  int          charging_candidate_count_{0};

  // Cached latest state for AvgPlatformStatus aggregation
  nav_msgs::msg::Odometry::ConstSharedPtr          latest_odom_;
  ranger_msgs::msg::SystemState::ConstSharedPtr     latest_system_state_;
  geometry_msgs::msg::TwistStamped                  latest_wheel_;
  bool                                              last_estop_{false};
  std::vector<float>                                latest_motor_rpm_;
  std::vector<float>                                latest_motor_speed_;
  std::vector<float>                                latest_motor_angle_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr       velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr       wheel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                    estop_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr         battery_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                    charging_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                    docking_charging_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgPlatformStatus>::SharedPtr       platform_status_pub_;

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr             odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr             odom_fallback_sub_;
  rclcpp::Subscription<ranger_msgs::msg::ActuatorStateArray>::SharedPtr actuator_state_sub_;
  rclcpp::Subscription<ranger_msgs::msg::SystemState>::SharedPtr       system_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr      battery_sub_;
};

}  // namespace camrod_platform

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod_platform::RangerPlatformBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
