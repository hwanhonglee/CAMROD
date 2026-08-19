// Copyright 2026 hwanhonglee
//
// HH_260819 - Pure charging confirmation policy used by the Ranger platform
// bridge.  The normal 10 s path rejects regenerative-current bursts; a tightly
// scoped AprilTag docking arm may use a shorter confirmation without weakening
// charging detection everywhere else.
#ifndef CAMROD_PLATFORM__CHARGING_DETECTION_POLICY_HPP_
#define CAMROD_PLATFORM__CHARGING_DETECTION_POLICY_HPP_

#include <algorithm>
#include <cmath>
#include <string>

namespace camrod_platform
{

enum class ChargingTransitionReason
{
  kNone,
  kGlobalConfirm,
  kDockingFastConfirm,
  kRelease,
};

struct ChargingDetectionConfig
{
  int minimum_samples{2};
  double global_confirm_s{10.0};
  double docking_fast_confirm_s{1.5};
  double release_s{3.0};
  double maximum_sample_gap_s{1.0};
};

struct ChargingDetectionResult
{
  bool charging{false};
  bool changed{false};
  ChargingTransitionReason reason{ChargingTransitionReason::kNone};
  double held_s{0.0};
  int sample_count{0};
};

// A ModuleState heartbeat may arm the fast path only when it comes from the
// selected AprilTag controller, reports one of the two terminal docking phases,
// is healthy, and both its source stamp and its receipt are still fresh.
class AprilTagChargingFastArm
{
public:
  explicit AprilTagChargingFastArm(double ttl_s = 1.5)
  : ttl_s_(std::max(0.0, ttl_s))
  {
  }

  void setTtl(double ttl_s)
  {
    ttl_s_ = std::max(0.0, ttl_s);
    reset();
  }

  bool observeStatus(
    const std::string & module_name,
    const std::string & operating_state,
    bool health_ok,
    double source_stamp_s,
    double receipt_time_s)
  {
    const bool exact_module = module_name == "apriltag_parking_controller";
    const bool allowed_phase =
      operating_state == "FINAL_YAW_ALIGNMENT" ||
      operating_state == "WAITING_FOR_CHARGING";
    const double source_age_s = receipt_time_s - source_stamp_s;
    const bool valid_time =
      std::isfinite(source_stamp_s) && source_stamp_s > 0.0 &&
      std::isfinite(receipt_time_s) &&
      source_age_s >= -kFutureToleranceS && source_age_s <= ttl_s_;

    if (!exact_module || !allowed_phase || !health_ok || !valid_time) {
      reset();
      return false;
    }

    armed_ = true;
    source_stamp_s_ = source_stamp_s;
    receipt_time_s_ = receipt_time_s;
    phase_ = operating_state;
    return true;
  }

  bool eligible(double now_s) const
  {
    if (!armed_ || !std::isfinite(now_s)) {
      return false;
    }
    const double receipt_age_s = now_s - receipt_time_s_;
    const double source_age_s = now_s - source_stamp_s_;
    return receipt_age_s >= -kFutureToleranceS && receipt_age_s <= ttl_s_ &&
           source_age_s >= -kFutureToleranceS && source_age_s <= ttl_s_;
  }

  void reset()
  {
    armed_ = false;
    source_stamp_s_ = 0.0;
    receipt_time_s_ = 0.0;
    phase_.clear();
  }

  const std::string & phase() const {return phase_;}

private:
  // The publisher and bridge use the same ROS clock.  This tolerance only
  // absorbs callback scheduling/rounding, not a genuinely future-dated status.
  static constexpr double kFutureToleranceS = 0.1;

  double ttl_s_{1.5};
  bool armed_{false};
  double source_stamp_s_{0.0};
  double receipt_time_s_{0.0};
  std::string phase_;
};

class ChargingDetectionPolicy
{
public:
  explicit ChargingDetectionPolicy(
    const ChargingDetectionConfig & config = ChargingDetectionConfig())
  {
    setConfig(config);
  }

  void setConfig(const ChargingDetectionConfig & config)
  {
    config_ = config;
    config_.minimum_samples = std::max(1, config.minimum_samples);
    config_.global_confirm_s = std::max(0.0, config.global_confirm_s);
    config_.docking_fast_confirm_s = std::max(0.0, config.docking_fast_confirm_s);
    config_.release_s = std::max(0.0, config.release_s);
    config_.maximum_sample_gap_s = std::max(0.0, config.maximum_sample_gap_s);
    reset(false);
  }

  ChargingDetectionResult observe(
    bool charging_sample, bool docking_fast_armed, double sample_time_s)
  {
    ChargingDetectionResult result;
    result.charging = charging_;
    if (!std::isfinite(sample_time_s)) {
      resetCandidates();
      return result;
    }
    if (has_last_sample_time_ && sample_time_s < last_sample_time_s_) {
      // A ROS-time reset must not carry an old dwell interval into a new epoch.
      resetCandidates();
    }
    last_sample_time_s_ = sample_time_s;
    has_last_sample_time_ = true;

    if (charging_) {
      resetRisingCandidates();
      if (charging_sample) {
        release_candidate_.reset();
        return result;
      }
      release_candidate_.observe(sample_time_s, config_.maximum_sample_gap_s);
      if (candidateReady(
          release_candidate_, config_.minimum_samples, config_.release_s,
          sample_time_s))
      {
        return transition(false, ChargingTransitionReason::kRelease,
          release_candidate_, sample_time_s);
      }
      result.held_s = release_candidate_.held(sample_time_s);
      result.sample_count = release_candidate_.samples;
      return result;
    }

    release_candidate_.reset();
    if (!charging_sample) {
      resetRisingCandidates();
      return result;
    }

    global_candidate_.observe(sample_time_s, config_.maximum_sample_gap_s);
    if (docking_fast_armed) {
      fast_candidate_.observe(sample_time_s, config_.maximum_sample_gap_s);
    } else {
      fast_candidate_.reset();
    }

    // Fast confirmation is never allowed from a single CAN frame, even if an
    // operator lowers the general minimum-sample parameter.
    const int fast_minimum_samples = std::max(2, config_.minimum_samples);
    if (docking_fast_armed && candidateReady(
        fast_candidate_, fast_minimum_samples,
        config_.docking_fast_confirm_s, sample_time_s))
    {
      return transition(true, ChargingTransitionReason::kDockingFastConfirm,
        fast_candidate_, sample_time_s);
    }
    if (candidateReady(
        global_candidate_, config_.minimum_samples,
        config_.global_confirm_s, sample_time_s))
    {
      return transition(true, ChargingTransitionReason::kGlobalConfirm,
        global_candidate_, sample_time_s);
    }

    const Candidate & reported = docking_fast_armed ? fast_candidate_ : global_candidate_;
    result.held_s = reported.held(sample_time_s);
    result.sample_count = reported.samples;
    return result;
  }

  void resetFastCandidate()
  {
    fast_candidate_.reset();
  }

  void reset(bool charging = false)
  {
    charging_ = charging;
    has_last_sample_time_ = false;
    last_sample_time_s_ = 0.0;
    resetCandidates();
  }

  bool charging() const {return charging_;}

private:
  struct Candidate
  {
    bool active{false};
    double since_s{0.0};
    double last_sample_s{0.0};
    int samples{0};

    void observe(double sample_time_s, double maximum_gap_s)
    {
      // A dwell represents continuous evidence.  If physical BMS updates stop
      // beyond the configured cadence allowance, this frame starts a new
      // candidate instead of completing an old global, fast, or release hold.
      if (!active || sample_time_s - last_sample_s > maximum_gap_s) {
        active = true;
        since_s = sample_time_s;
        last_sample_s = sample_time_s;
        samples = 1;
        return;
      }
      last_sample_s = sample_time_s;
      ++samples;
    }

    double held(double sample_time_s) const
    {
      return active ? std::max(0.0, sample_time_s - since_s) : 0.0;
    }

    void reset()
    {
      active = false;
      since_s = 0.0;
      last_sample_s = 0.0;
      samples = 0;
    }
  };

  static bool candidateReady(
    const Candidate & candidate, int minimum_samples, double hold_s,
    double sample_time_s)
  {
    return candidate.active && candidate.samples >= minimum_samples &&
           candidate.held(sample_time_s) >= hold_s;
  }

  ChargingDetectionResult transition(
    bool charging, ChargingTransitionReason reason,
    const Candidate & candidate, double sample_time_s)
  {
    ChargingDetectionResult result;
    result.charging = charging;
    result.changed = charging_ != charging;
    result.reason = reason;
    result.held_s = candidate.held(sample_time_s);
    result.sample_count = candidate.samples;
    charging_ = charging;
    resetCandidates();
    return result;
  }

  void resetRisingCandidates()
  {
    global_candidate_.reset();
    fast_candidate_.reset();
  }

  void resetCandidates()
  {
    resetRisingCandidates();
    release_candidate_.reset();
  }

  ChargingDetectionConfig config_;
  bool charging_{false};
  bool has_last_sample_time_{false};
  double last_sample_time_s_{0.0};
  Candidate global_candidate_;
  Candidate fast_candidate_;
  Candidate release_candidate_;
};

}  // namespace camrod_platform

#endif  // CAMROD_PLATFORM__CHARGING_DETECTION_POLICY_HPP_
