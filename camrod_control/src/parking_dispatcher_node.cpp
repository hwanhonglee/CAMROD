// One runtime owner sequences reverse-first, SOC-aware final parking and forwards only that
// controller's private outputs. Both physical controllers remain isolated even
// during cancellation, late charging feedback, and forced redocking.
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "avg_msgs/msg/avg_platform_status.hpp"
#include "avg_msgs/msg/avg_service_state.hpp"
#include "avg_msgs/msg/avg_twist.hpp"
#include "avg_msgs/msg/module_state.hpp"
#include "avg_msgs/msg/motion_operation.hpp"
#include "avg_msgs/srv/request_motion_operation.hpp"
#include "camrod_control/control_diagnostics.hpp"
#include "camrod_control/parking_selection_policy.hpp"
#include "rclcpp/rclcpp.hpp"

using camrod_control::ParkingMethod;
using MotionService = avg_msgs::srv::RequestMotionOperation;

class ParkingDispatcherNode : public rclcpp::Node {
public:
  explicit ParkingDispatcherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("parking_dispatcher", options) {
    threshold_ = declare_parameter<double>("charging_threshold_percent", 35.0);
    platform_timeout_s_ = declare_parameter<double>("platform_status_timeout_s", 2.0);
    handoff_hold_s_ = declare_parameter<double>("controller_handoff_hold_s", 0.5);
    controller_timeout_s_ = declare_parameter<double>("controller_operation_timeout_s", 8.0);
    command_timeout_s_ = declare_parameter<double>("controller_command_timeout_s", 0.5);
    status_timeout_s_ = declare_parameter<double>("controller_status_timeout_s", 2.0);
    if (!std::isfinite(threshold_) || threshold_ <= 0.0 || threshold_ > 100.0 ||
        !std::isfinite(platform_timeout_s_) || platform_timeout_s_ <= 0.0 ||
        !std::isfinite(handoff_hold_s_) || handoff_hold_s_ < 0.0 ||
        !std::isfinite(controller_timeout_s_) || controller_timeout_s_ <= 0.0 ||
        !std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0.0 ||
        !std::isfinite(status_timeout_s_) || status_timeout_s_ <= 0.0) {
      throw std::invalid_argument("invalid parking-dispatcher safety configuration");
    }
    command_pub_ = create_publisher<avg_msgs::msg::AvgTwist>(
        declare_parameter<std::string>("command_topic", "/control/cmd_vel_raw"), 10);
    service_pub_ = create_publisher<avg_msgs::msg::AvgServiceState>(
        declare_parameter<std::string>("service_state_topic", "/service/state"), 10);
    status_pub_ = create_publisher<avg_msgs::msg::ModuleState>(
        declare_parameter<std::string>("status_topic", "/parking/status"), 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/system/diagnostics", 10);
    platform_sub_ = create_subscription<avg_msgs::msg::AvgPlatformStatus>(
        declare_parameter<std::string>("platform_status_topic", "/platform/status"), 10,
        [this](avg_msgs::msg::AvgPlatformStatus::ConstSharedPtr message) {
          platform_ = *message;
          platform_time_ = now();
        });
    operation_sub_ = create_subscription<avg_msgs::msg::MotionOperation>(
        declare_parameter<std::string>("operation_topic", "/parking/operation"), 10,
        [this](avg_msgs::msg::MotionOperation::ConstSharedPtr message) {
          if (message->operation == avg_msgs::msg::MotionOperation::START &&
              (message->header.stamp.sec != 0 || message->header.stamp.nanosec != 0U)) {
            const std::string identity = message->source + ":" +
                std::to_string(message->header.stamp.sec) + ":" +
                std::to_string(message->header.stamp.nanosec);
            if (identity == last_topic_start_identity_) { return; }
            last_topic_start_identity_ = identity;
          }
          request(message->operation, message->source);
        });
    operation_service_ = create_service<MotionService>(
        "/parking/request_operation",
        [this](MotionService::Request::SharedPtr request_message,
               MotionService::Response::SharedPtr response) {
          const auto result = request(request_message->operation, request_message->source);
          response->accepted = result.first;
          response->message = result.second;
        });
    for (std::size_t index = 0; index < methods_.size(); ++index) {
      const auto method = methods_[index];
      const std::string prefix = std::string("/parking/private/") +
          camrod_control::parkingMethodName(method);
      clients_[index] = create_client<MotionService>(prefix + "/request_operation");
      command_subs_[index] = create_subscription<avg_msgs::msg::AvgTwist>(
          prefix + "/cmd_vel", 10,
          [this, method](avg_msgs::msg::AvgTwist::ConstSharedPtr message) {
            if (!ownership_.owns(method) || failed_) { return; }
            last_command_time_ = now();
            if (!platformAllowsMotion() || selected_phase_ == "PARKED" ||
                selected_phase_ == "ERROR" || selected_phase_ == "IDLE") {
              publishZero();
              return;
            }
            command_pub_->publish(*message);
          });
      status_subs_[index] = create_subscription<avg_msgs::msg::ModuleState>(
          prefix + "/status", 10,
          [this, index, method](avg_msgs::msg::ModuleState::ConstSharedPtr message) {
            receiveStatus(index, method, *message);
          });
      service_subs_[index] = create_subscription<avg_msgs::msg::AvgServiceState>(
          prefix + "/service_state", 10,
          [this, index, method](avg_msgs::msg::AvgServiceState::ConstSharedPtr message) {
            if ((pending_start_ && start_sent_ && ownership_.selected() == method) ||
                ownership_.owns(method)) {
              cached_service_[index] = *message;
            }
            if (ownership_.owns(method)) { forwardService(method, *message); }
          });
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&ParkingDispatcherNode::tick, this));
    publishStatus();
  }

private:
#ifdef CAMROD_CONTROL_PARKING_DISPATCHER_TEST
  friend class ParkingDispatcherTest;
#endif
  std::optional<double> battery() const {
    return camrod_control::usableParkingBatteryPercent(
        platform_.has_value() && platform_->battery_state_available,
        platform_.has_value() ? platform_->battery_percentage : -1.0F,
        platform_.has_value() ? (now() - platform_time_).seconds() : -1.0,
        platform_timeout_s_);
  }
  bool platformAllowsMotion() const {
    if (!platform_.has_value()) { return false; }
    const double age = (now() - platform_time_).seconds();
    return age >= 0.0 && age <= platform_timeout_s_ &&
        platform_->control_mode == 1U && !platform_->estop &&
        platform_->vehicle_state == 0U && platform_->error_code == 0U;
  }
  bool chargingConfirmed() const {
    if (!platform_.has_value()) { return false; }
    const double age = (now() - platform_time_).seconds();
    return age >= 0.0 && age <= platform_timeout_s_ && platform_->is_charging;
  }
  bool dockingRequired() const {
    return camrod_control::selectParkingMethod(battery(), threshold_, forced_) ==
        ParkingMethod::kAprilTag;
  }
  bool freshReverseParked() const {
    const double age = (now() - last_controller_status_time_).seconds();
    return ownership_.owns(ParkingMethod::kReverse) && reverse_parked_verified_ &&
        !failed_ && selected_phase_ == "PARKED" && age >= 0.0 && age <= status_timeout_s_;
  }
  bool reverseDockPending() const {
    return ownership_.selected() == ParkingMethod::kReverse &&
        dockingRequired() && !chargingConfirmed();
  }
  std::string publicPhase() const {
    return selected_phase_ == "PARKED" && reverseDockPending()
        ? "REVERSE_PARKED_WAITING_FOR_DOCK" : selected_phase_;
  }
  avg_msgs::msg::AvgServiceState serviceOutput(
      const ParkingMethod method, avg_msgs::msg::AvgServiceState output) const {
    // Reverse emits its service state BEFORE its ModuleState. Do not expose a
    // transient final WAIT/PARKED (which disarms the mission) between stages.
    // Charger contact remains authoritative; reverse completion alone is not.
    if (method == ParkingMethod::kReverse && reverseDockPending() &&
        (output.state == avg_msgs::msg::AvgServiceState::DROP_ZONE_WAIT ||
         output.state == avg_msgs::msg::AvgServiceState::WAITING_FOR_CHARGING ||
         output.state == avg_msgs::msg::AvgServiceState::CHARGING)) {
      output.state = avg_msgs::msg::AvgServiceState::DROP_ZONE_PARKING;
      output.state_name = "DROP_ZONE_PARKING";
      output.description = "parking_dispatcher:REVERSE_PARKED_WAITING_FOR_DOCK; " +
          output.description;
    }
    output.description += "; " + selectionDescription();
    return output;
  }
  void forwardService(const ParkingMethod method,
                      const avg_msgs::msg::AvgServiceState & message) {
    const auto output = serviceOutput(method, message);
    if (method == ParkingMethod::kReverse) {
      reverse_terminal_service_deferred_ = output.state != message.state;
    }
    service_pub_->publish(output);
  }
  void receiveStatus(const std::size_t index, const ParkingMethod method,
                     const avg_msgs::msg::ModuleState & message) {
    if (!((pending_start_ && start_sent_ && ownership_.selected() == method) ||
          ownership_.owns(method))) { return; }
    // A pre-START heartbeat, including an old PARKED, cannot prove completion
    // for this generation. Private controllers stamp status with their clock.
    if (rclcpp::Time(message.stamp, get_clock()->get_clock_type()) < start_time_) {
      return;
    }
    cached_status_[index] = message;
    if (!ownership_.owns(method)) { return; }
    last_controller_status_time_ = now();
    selected_phase_ = message.operating_state;
    reverse_parked_verified_ = method == ParkingMethod::kReverse && selected_phase_ == "PARKED";
    if (selected_phase_ == "PARKED" || selected_phase_ == "ERROR") {
      ownership_.complete();
      publishZero();
    }
    // Actual charger contact can still recover a selected controller ERROR.
    if (selected_phase_ == "PARKED") { failed_ = false; error_detail_.clear(); }
    if (selected_phase_ == "ERROR") { error_detail_ = message.message; }
    publishStatus(message.level);
  }
  void acceptStart(const std::size_t index, const uint64_t generation) {
    if (!ownership_.acknowledgeStart(generation)) { return; }
    pending_start_ = false;
    last_command_time_ = now();
    // Grant the first heartbeat interval, but do not manufacture PARKED proof.
    last_controller_status_time_ = now();
    selected_phase_ = index == 0 ? "REVERSE_APPROACH" : "WAITING_FOR_TAG";
    if (cached_status_[index].has_value() &&
        cached_status_[index]->operating_state != "IDLE") {
      const auto cached = *cached_status_[index];
      receiveStatus(index, methods_[index], cached);
    }
    publishStatus();
    if (cached_service_[index].has_value()) {
      forwardService(methods_[index], *cached_service_[index]);
    }
  }
  std::string selectionDescription() const {
    const auto soc = battery();
    return std::string("parking_method=") +
        camrod_control::parkingMethodName(ownership_.selected()) +
        " battery_percent=" + (soc ? std::to_string(*soc) : "unknown") +
        " charging_required=" +
        (ownership_.selected() == ParkingMethod::kAprilTag || reverseDockPending() ? "true" : "false") +
        " forced=" + (forced_ ? "true" : "false") +
        " attempt=" + std::to_string(ownership_.generation());
  }
  void publishZero() { command_pub_->publish(avg_msgs::msg::AvgTwist()); }
  void publishStatus(const uint8_t level = avg_msgs::msg::ModuleState::OK) {
    const std::string detail = selectionDescription() + " phase=" + publicPhase() +
        (error_detail_.empty() ? "" : " error=" + error_detail_);
    status_pub_->publish(camrod_control::makeModuleState(
        *this, "parking", level, detail, publicPhase()));
    diagnostics_pub_->publish(camrod_control::makeDiagnostics(
        *this, "parking/parking_dispatcher", "parking", level, detail));
  }
  void requestControllerStops(const std::string &source) {
    // Use the same ordered service request channel as handoff/start. A late
    // CANCEL from a separate topic must not overtake a newly accepted START.
    for (const auto &client : clients_) {
      if (!client->service_is_ready()) { continue; }
      auto message = std::make_shared<MotionService::Request>();
      message->operation = avg_msgs::msg::MotionOperation::CANCEL;
      message->source = "parking_dispatcher:" + source;
      client->async_send_request(message);
    }
  }
  std::pair<bool, std::string> request(const uint8_t operation,
                                      const std::string &source) {
    if (operation == avg_msgs::msg::MotionOperation::CANCEL) {
      ownership_.cancel();
      pending_start_ = false;
      forced_ = false;
      reverse_parked_verified_ = false;
      reverse_terminal_service_deferred_ = false;
      failed_ = false;
      error_detail_.clear();
      selected_phase_ = "IDLE";
      publishZero();
      requestControllerStops(source);
      publishStatus();
      return {true, "parking dispatcher cancelled"};
    }
    if (operation != avg_msgs::msg::MotionOperation::START) {
      return {false, "unsupported parking operation"};
    }
    if (ownership_.busy()) {
      // An explicit Dock during reverse latches only the final destination;
      // it cannot switch the moving owner or bypass the reverse completion.
      forced_ = forced_ || camrod_control::hasForceDockingToken(source);
      return {true, "parking attempt already accepted; " + selectionDescription()};
    }
    forced_ = camrod_control::hasForceDockingToken(source);
    const auto method = camrod_control::initialParkingMethod(forced_, freshReverseParked());
    beginAttempt(method, source);
    return {true, "parking selection accepted; " + selectionDescription()};
  }
  void beginAttempt(const ParkingMethod method, const std::string & source) {
    ownership_.begin(method);
    reverse_parked_verified_ = false;
    reverse_terminal_service_deferred_ = false;
    source_ = source;
    pending_start_ = true;
    start_sent_ = false;
    failed_ = false;
    error_detail_.clear();
    cancel_sent_.fill(false);
    cached_status_.fill(std::nullopt);
    cached_service_.fill(std::nullopt);
    attempt_time_ = now();
    cancelled_time_.reset();
    selected_phase_ = "WAITING_FOR_PARKING_OWNER";
    publishZero();
    publishStatus();
    avg_msgs::msg::AvgServiceState handoff;
    handoff.state = avg_msgs::msg::AvgServiceState::DROP_ZONE_PARKING;
    handoff.state_name = "DROP_ZONE_PARKING";
    handoff.description = "parking_dispatcher:WAITING_FOR_PARKING_OWNER:" +
        selectionDescription();
    service_pub_->publish(handoff);
  }
  void fail(const std::string &detail) {
    failed_ = true;
    pending_start_ = false;
    ownership_.abort();
    selected_phase_ = "ERROR";
    error_detail_ = detail;
    publishZero();
    requestControllerStops("handoff_failed");
    publishStatus(avg_msgs::msg::ModuleState::ERROR);
    // AvgServiceState has no FAILED enum. The authoritative ModuleState ERROR
    // carries the failure to UI/safety without inventing a service-state ID.
  }
  void tick() {
    if (pending_start_) {
      publishZero();
      const auto generation = ownership_.generation();
      for (std::size_t index = 0; index < methods_.size(); ++index) {
        if (cancel_sent_[index] || !clients_[index]->service_is_ready()) { continue; }
        cancel_sent_[index] = true;
        auto message = std::make_shared<MotionService::Request>();
        message->operation = avg_msgs::msg::MotionOperation::CANCEL;
        message->source = "parking_dispatcher:handoff:" + std::to_string(generation);
        clients_[index]->async_send_request(message,
            [this, index, generation](rclcpp::Client<MotionService>::SharedFuture future) {
              if (generation != ownership_.generation()) { return; }
              if (!future.get()->accepted) { fail("controller cancellation rejected"); return; }
              ownership_.acknowledgeCancel(methods_[index], generation);
            });
      }
      if (!ownership_.cancellationsAcknowledged()) {
        if ((now() - attempt_time_).seconds() > controller_timeout_s_) {
          fail("controller cancellation acknowledgement timeout");
        }
        return;
      }
      if (!cancelled_time_.has_value()) { cancelled_time_ = now(); }
      if ((now() - *cancelled_time_).seconds() < handoff_hold_s_ ||
          !platformAllowsMotion()) {
        publishStatus();
        return;
      }
      if (!start_sent_) {
        start_sent_ = true;
        start_time_ = now();
        const std::size_t index = ownership_.selected() == ParkingMethod::kReverse ? 0 : 1;
        auto message = std::make_shared<MotionService::Request>();
        message->operation = avg_msgs::msg::MotionOperation::START;
        message->source = "parking_dispatcher:attempt=" + std::to_string(generation) + ":" + source_;
        clients_[index]->async_send_request(message,
            [this, index, generation](rclcpp::Client<MotionService>::SharedFuture future) {
              if (generation != ownership_.generation()) { return; }
              const auto result = future.get();
              if (!result->accepted) { fail("selected controller START rejected: " + result->message); return; }
              acceptStart(index, generation);
            });
      } else if ((now() - start_time_).seconds() > controller_timeout_s_) {
        fail("selected controller START acknowledgement timeout");
      }
      return;
    }
    if (freshReverseParked() && !reverseDockPending() &&
        reverse_terminal_service_deferred_ && cached_service_[0].has_value()) {
      // A fresh high SOC/contact update may resolve a previously unknown SOC
      // while stopped. Release the real cached terminal event, never invent it.
      forwardService(ParkingMethod::kReverse, *cached_service_[0]);
    }
    if (ownership_.started()) {
      const auto health = camrod_control::parkingTelemetryHealth(
          selected_phase_, (now() - last_command_time_).seconds(),
          (now() - last_controller_status_time_).seconds(), command_timeout_s_, status_timeout_s_);
      if (health != camrod_control::ParkingTelemetryHealth::kHealthy) {
        fail(health == camrod_control::ParkingTelemetryHealth::kStatusTimeout
            ? "selected controller status timeout" : "selected controller command timeout");
        return;
      }
    }
    // Only an actual completed reverse permits the second-stage owner. Its
    // separate generation still requires BOTH CANCEL ACKs and selected START
    // ACK; never move in RC/EStop or after CAN already confirms contact.
    if (freshReverseParked() && dockingRequired() &&
        platformAllowsMotion() && !platform_->is_charging) {
      beginAttempt(ParkingMethod::kAprilTag, source_ + ":reverse_first_complete");
      return;
    }
    if (ownership_.started() && (failed_ || selected_phase_ == "PARKED" ||
        selected_phase_ == "ERROR" || !platformAllowsMotion() ||
        (now() - last_command_time_).seconds() > command_timeout_s_)) {
      publishZero();
    }
    publishStatus(failed_ || selected_phase_ == "ERROR"
        ? avg_msgs::msg::ModuleState::ERROR : avg_msgs::msg::ModuleState::OK);
  }

  const std::array<ParkingMethod, 2> methods_{{ParkingMethod::kReverse, ParkingMethod::kAprilTag}};
  camrod_control::ParkingOwnershipPolicy ownership_;
  double threshold_{35.0}, platform_timeout_s_{2.0}, handoff_hold_s_{0.5};
  double controller_timeout_s_{8.0}, command_timeout_s_{0.5}, status_timeout_s_{2.0};
  bool pending_start_{false}, start_sent_{false}, failed_{false}, forced_{false};
  bool reverse_parked_verified_{false};
  bool reverse_terminal_service_deferred_{false};
  std::array<bool, 2> cancel_sent_{{false, false}};
  std::string selected_phase_{"IDLE"}, source_, last_topic_start_identity_, error_detail_;
  std::optional<avg_msgs::msg::AvgPlatformStatus> platform_;
  std::array<std::optional<avg_msgs::msg::ModuleState>, 2> cached_status_;
  std::array<std::optional<avg_msgs::msg::AvgServiceState>, 2> cached_service_;
  rclcpp::Time platform_time_{0, 0, RCL_ROS_TIME}, attempt_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME}, last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_controller_status_time_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> cancelled_time_;
  rclcpp::Publisher<avg_msgs::msg::AvgTwist>::SharedPtr command_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgServiceState>::SharedPtr service_pub_;
  rclcpp::Publisher<avg_msgs::msg::ModuleState>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::array<rclcpp::Client<MotionService>::SharedPtr, 2> clients_;
  std::array<rclcpp::Subscription<avg_msgs::msg::AvgTwist>::SharedPtr, 2> command_subs_;
  std::array<rclcpp::Subscription<avg_msgs::msg::ModuleState>::SharedPtr, 2> status_subs_;
  std::array<rclcpp::Subscription<avg_msgs::msg::AvgServiceState>::SharedPtr, 2> service_subs_;
  rclcpp::Subscription<avg_msgs::msg::AvgPlatformStatus>::SharedPtr platform_sub_;
  rclcpp::Subscription<avg_msgs::msg::MotionOperation>::SharedPtr operation_sub_;
  rclcpp::Service<MotionService>::SharedPtr operation_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#ifndef CAMROD_CONTROL_PARKING_DISPATCHER_TEST
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParkingDispatcherNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
