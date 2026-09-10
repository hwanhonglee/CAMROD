#pragma once

#include <cctype>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

namespace camrod_control {

enum class ParkingMethod { kNone, kReverse, kAprilTag };
enum class ParkingTelemetryHealth { kHealthy, kCommandTimeout, kStatusTimeout };

inline ParkingTelemetryHealth parkingTelemetryHealth(
    const std::string &phase, const double command_age_s, const double status_age_s,
    const double command_timeout_s, const double status_timeout_s) {
  if (!std::isfinite(status_age_s) || status_age_s < 0.0 ||
      status_age_s > status_timeout_s) {
    return ParkingTelemetryHealth::kStatusTimeout;
  }
  // A completed park or stopped contact/tag wait need not keep publishing
  // velocity, but every selected controller must still report fresh status.
  const bool stationary = phase == "PARKED" || phase == "WAIT_FOR_CHARGING" ||
      phase == "WAITING_FOR_CHARGING" || phase == "WAITING_FOR_TAG" || phase == "ERROR";
  if (!stationary && (!std::isfinite(command_age_s) || command_age_s < 0.0 ||
                     command_age_s > command_timeout_s)) {
    return ParkingTelemetryHealth::kCommandTimeout;
  }
  return ParkingTelemetryHealth::kHealthy;
}

inline const char *parkingMethodName(const ParkingMethod method) {
  return method == ParkingMethod::kReverse ? "reverse" :
      method == ParkingMethod::kAprilTag ? "apriltag" : "none";
}

inline bool hasForceDockingToken(const std::string &source) {
  std::string token;
  for (std::size_t index = 0; index <= source.size(); ++index) {
    const unsigned char value = index < source.size() ? source[index] : ':';
    if (std::isalnum(value) || value == '_') {
      token.push_back(static_cast<char>(value));
    } else {
      if (token == "force_docking") {
        return true;
      }
      token.clear();
    }
  }
  return false;
}

inline std::optional<double> usableParkingBatteryPercent(
    const bool available, const float fraction, const double age_s,
    const double timeout_s) {
  // HH_260907 - AvgPlatformStatus carries BatteryState.percentage as float32
  // in [0, 1], not [0, 100]. Validate that wire contract before conversion.
  if (!available || !std::isfinite(fraction) || fraction < 0.0F || fraction > 1.0F ||
      !std::isfinite(age_s) || age_s < 0.0 || age_s > timeout_s) {
    return std::nullopt;
  }
  // Convert at the message's float32 precision before widening: 0.35F must
  // become 35%, not 34.9999994%. The adjacent lower float remains below 35%.
  return static_cast<double>(fraction * 100.0F);
}

inline ParkingMethod selectParkingMethod(const std::optional<double> &percent,
                                         const double threshold,
                                         const bool force_docking) {
  // The final destination AFTER reverse parking. Unknown/stale SOC cannot
  // authorize a non-charging completion; it never skips the reverse approach.
  return force_docking || !percent.has_value() || !std::isfinite(*percent) ||
                 *percent < threshold
      ? ParkingMethod::kAprilTag : ParkingMethod::kReverse;
}

inline ParkingMethod initialParkingMethod(const bool force_docking,
                                          const bool fresh_reverse_parked) {
  // A normal Return always establishes the near-station reverse pose first.
  // Only an explicit Dock may reuse an actual, fresh, owned reverse completion.
  return force_docking && fresh_reverse_parked
      ? ParkingMethod::kAprilTag : ParkingMethod::kReverse;
}

// The chosen method is immutable from START through its terminal state. Every
// ownership transfer first waits for both CANCEL acknowledgements; only a
// successful selected START acknowledgement enables velocity forwarding.
class ParkingOwnershipPolicy {
public:
  bool begin(const ParkingMethod method) {
    if (busy_ || method == ParkingMethod::kNone) {
      return false;
    }
    ++generation_;
    selected_ = method;
    busy_ = true;
    started_ = false;
    reverse_cancelled_ = false;
    apriltag_cancelled_ = false;
    return true;
  }
  void cancel() {
    ++generation_;
    selected_ = ParkingMethod::kNone;
    busy_ = false;
    started_ = false;
    reverse_cancelled_ = false;
    apriltag_cancelled_ = false;
  }
  void acknowledgeCancel(const ParkingMethod method, const uint64_t generation) {
    if (generation != generation_ || !busy_ || started_) {
      return;
    }
    if (method == ParkingMethod::kReverse) { reverse_cancelled_ = true; }
    if (method == ParkingMethod::kAprilTag) { apriltag_cancelled_ = true; }
  }
  bool cancellationsAcknowledged() const {
    return reverse_cancelled_ && apriltag_cancelled_;
  }
  bool acknowledgeStart(const uint64_t generation) {
    if (generation != generation_ || !busy_ || !cancellationsAcknowledged()) {
      return false;
    }
    started_ = true;
    return true;
  }
  void complete() { busy_ = false; }
  void abort() {
    // Preserve the failed method for diagnostics, but invalidate its output and
    // every in-flight ACK. CANCEL-induced IDLE must not overwrite this ERROR.
    ++generation_;
    busy_ = false;
    started_ = false;
    reverse_cancelled_ = false;
    apriltag_cancelled_ = false;
  }
  bool owns(const ParkingMethod method) const {
    return started_ && selected_ == method;
  }
  ParkingMethod selected() const { return selected_; }
  bool busy() const { return busy_; }
  bool started() const { return started_; }
  uint64_t generation() const { return generation_; }

private:
  ParkingMethod selected_{ParkingMethod::kNone};
  bool busy_{false};
  bool started_{false};
  bool reverse_cancelled_{false};
  bool apriltag_cancelled_{false};
  uint64_t generation_{0};
};

} // namespace camrod_control
