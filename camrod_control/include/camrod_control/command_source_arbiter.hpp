#pragma once

#include <algorithm>
#include <cctype>
#include <limits>
#include <string>

namespace camrod_control
{

// HH_260806 - Give an explicit maneuver exclusive ownership of the final
// command stream and require a short stationary handoff before Nav2 translates.
struct CommandSourceArbiterConfig
{
  double maneuver_release_hold_s{0.5};
  // The dedicated manual source is opt-in so ordinary CAMROD launches retain
  // their existing raw/Nav2 arbitration exactly.
  bool manual_input_enabled{false};
};

enum class CommandInputSource
{
  kRaw,
  kNavigation,
  kManual,
};

enum class CommandSourceDecision
{
  kAllow,
  kIgnore,
  kHoldZero,
};

struct ManeuverOwnershipTransition
{
  bool maneuver_started{false};
  bool drop_zone_started{false};
  bool campsite_started{false};
  bool parking_started{false};
  bool maneuver_finished{false};
};

class CommandSourceArbiter
{
public:
  explicit CommandSourceArbiter(CommandSourceArbiterConfig config = {})
  : config_(config)
  {
  }

  void setConfig(const CommandSourceArbiterConfig & config)
  {
    config_ = config;
    if (!config_.manual_input_enabled) {
      manual_source_active_ = false;
    }
  }

  // Returns true when an active manual owner was released.  Engagement is
  // updated independently from command arrival so a mission takeover or
  // operator disengage can synchronously publish a zero command.
  bool setEngagement(const bool manual_engaged, const bool mission_engaged)
  {
    manual_engaged_ = manual_engaged;
    mission_engaged_ = mission_engaged;
    if (manual_source_active_ && (!manual_engaged_ || mission_engaged_)) {
      manual_source_active_ = false;
      return true;
    }
    return false;
  }

  ManeuverOwnershipTransition setManeuverPhases(
    const std::string & drop_zone_phase,
    const std::string & campsite_phase,
    const std::string & parking_phase,
    const double now_sec)
  {
    const bool next_drop_zone_active = dropZoneOwnsCommand(drop_zone_phase);
    const bool next_campsite_active = campsiteOwnsCommand(campsite_phase);
    const bool next_parking_active = parkingOwnsCommand(parking_phase);
    const bool next_active = next_campsite_active || next_parking_active ||
      next_drop_zone_active;
    ManeuverOwnershipTransition transition;
    transition.maneuver_started = !maneuver_active_ && next_active;
    transition.drop_zone_started = !drop_zone_active_ && next_drop_zone_active;
    transition.campsite_started = !campsite_active_ && next_campsite_active;
    transition.parking_started = !parking_active_ && next_parking_active;
    transition.maneuver_finished = maneuver_active_ && !next_active;

    if (transition.maneuver_finished) {
      release_hold_until_sec_ = std::max(
        release_hold_until_sec_, now_sec + std::max(0.0, config_.maneuver_release_hold_s));
    } else if (next_active) {
      release_hold_until_sec_ = -std::numeric_limits<double>::infinity();
    }
    maneuver_active_ = next_active;
    drop_zone_active_ = next_drop_zone_active;
    campsite_active_ = next_campsite_active;
    parking_active_ = next_parking_active;
    return transition;
  }

  CommandSourceDecision evaluate(
    const CommandInputSource source,
    const double now_sec)
  {
    // Explicit campsite/drop-zone/parking controllers are the highest-priority
    // owners.  Their raw stream remains available even if manual ownership was
    // previously latched.
    if (maneuver_active_) {
      return source == CommandInputSource::kRaw ?
             CommandSourceDecision::kAllow : CommandSourceDecision::kIgnore;
    }
    // An unauthorized manual sample is ignored even during a maneuver-release
    // hold. It must not count as a fresh input or influence watchdog state.
    if (source == CommandInputSource::kManual &&
      (!config_.manual_input_enabled || !manual_engaged_ || mission_engaged_))
    {
      return CommandSourceDecision::kIgnore;
    }
    if (now_sec < release_hold_until_sec_) {
      return CommandSourceDecision::kHoldZero;
    }

    if (source == CommandInputSource::kManual) {
      // The first authorized, fresh manual Twist takes ownership.  Before this
      // latch, UI manual Nav2 goals remain compatible with ordinary CAMROD.
      manual_source_active_ = true;
      return CommandSourceDecision::kAllow;
    }

    if (manual_source_active_) {
      return CommandSourceDecision::kIgnore;
    }
    // HH_260806 - RotationShim/RPP is one Nav2 owner. Its normal rotation and
    // translation commands must pass continuously; re-arbitrating those
    // commands here produced a repeated 0.5 s stop-go cycle on curved routes.
    return CommandSourceDecision::kAllow;
  }

  // Compatibility overload for existing raw/Nav2 users and tests.
  CommandSourceDecision evaluate(
    const bool navigation_source,
    const double now_sec)
  {
    return evaluate(
      navigation_source ? CommandInputSource::kNavigation : CommandInputSource::kRaw,
      now_sec);
  }

  bool maneuverActive() const
  {
    return maneuver_active_;
  }

  bool campsiteActive() const
  {
    return campsite_active_;
  }

  bool manualSourceActive() const
  {
    return manual_source_active_;
  }

private:
  static std::string normalize(std::string value)
  {
    value.erase(
      std::remove_if(value.begin(), value.end(), [](const unsigned char character) {
        return std::isspace(character) != 0;
      }),
      value.end());
    std::transform(
      value.begin(), value.end(), value.begin(), [](const unsigned char character) {
        return static_cast<char>(std::tolower(character));
      });
    return value;
  }

  static bool campsiteOwnsCommand(const std::string & phase)
  {
    const std::string value = normalize(phase);
    return value == "align_entry_yaw" || value == "reverse_in" || value == "crab_in" ||
           value == "rotate_180" || value == "unload_wait" || value == "wait_return" ||
           value == "align_retrace_yaw" || value == "align_return_route_yaw" ||
           value == "reverse_out" || value == "crab_out";
  }

  static bool dropZoneOwnsCommand(const std::string & phase)
  {
    const std::string value = normalize(phase);
    return value == "exit_straight" || value == "align_exit_yaw" ||
           value == "align_parking_yaw";
  }

  static bool parkingOwnsCommand(const std::string & phase)
  {
    const std::string value = normalize(phase);
    return value == "reverse_approach" || value == "wait_for_charging" ||
           value == "waiting_for_charging" || value == "final_yaw_alignment" ||
           value == "waiting_for_tag" || value == "tag_guided_reverse" ||
           value == "final_reverse_insertion" || value == "retry_forward_exit" ||
           value == "parked";
  }

  CommandSourceArbiterConfig config_;
  bool maneuver_active_{false};
  bool drop_zone_active_{false};
  bool campsite_active_{false};
  bool parking_active_{false};
  bool manual_engaged_{false};
  bool mission_engaged_{false};
  bool manual_source_active_{false};
  double release_hold_until_sec_{-std::numeric_limits<double>::infinity()};
};

}  // namespace camrod_control
