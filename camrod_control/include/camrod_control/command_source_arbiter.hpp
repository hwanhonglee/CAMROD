#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>

namespace camrod_control
{

// HH_260806 - Give an explicit maneuver exclusive ownership of the final
// command stream and require a short stationary handoff before Nav2 translates.
struct CommandSourceArbiterConfig
{
  double maneuver_release_hold_s{0.5};
  double navigation_rotation_settle_s{0.5};
  double navigation_translation_epsilon_mps{0.01};
  double navigation_rotation_min_radps{0.05};
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
  bool campsite_started{false};
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
  }

  ManeuverOwnershipTransition setManeuverPhases(
    const std::string & drop_zone_phase,
    const std::string & campsite_phase,
    const double now_sec)
  {
    const bool next_campsite_active = campsiteOwnsCommand(campsite_phase);
    const bool next_active = next_campsite_active || dropZoneOwnsCommand(drop_zone_phase);
    ManeuverOwnershipTransition transition;
    transition.maneuver_started = !maneuver_active_ && next_active;
    transition.campsite_started = !campsite_active_ && next_campsite_active;
    transition.maneuver_finished = maneuver_active_ && !next_active;

    if (transition.maneuver_finished) {
      release_hold_until_sec_ = std::max(
        release_hold_until_sec_, now_sec + std::max(0.0, config_.maneuver_release_hold_s));
    } else if (next_active) {
      release_hold_until_sec_ = -std::numeric_limits<double>::infinity();
    }
    maneuver_active_ = next_active;
    campsite_active_ = next_campsite_active;
    return transition;
  }

  CommandSourceDecision evaluate(
    const bool navigation_source,
    const double linear_x,
    const double linear_y,
    const double angular_z,
    const double now_sec)
  {
    if (maneuver_active_) {
      return navigation_source ? CommandSourceDecision::kIgnore : CommandSourceDecision::kAllow;
    }
    if (now_sec < release_hold_until_sec_) {
      return CommandSourceDecision::kHoldZero;
    }
    if (!navigation_source) {
      return CommandSourceDecision::kAllow;
    }

    const double translation = std::hypot(linear_x, linear_y);
    const double translation_epsilon = std::max(
      0.0, config_.navigation_translation_epsilon_mps);
    const bool pure_rotation = translation <= translation_epsilon &&
      std::abs(angular_z) >= std::max(0.0, config_.navigation_rotation_min_radps);
    if (pure_rotation) {
      last_navigation_rotation_sec_ = now_sec;
      return CommandSourceDecision::kAllow;
    }
    if (translation > translation_epsilon &&
      now_sec - last_navigation_rotation_sec_ <
      std::max(0.0, config_.navigation_rotation_settle_s))
    {
      return CommandSourceDecision::kHoldZero;
    }
    return CommandSourceDecision::kAllow;
  }

  bool maneuverActive() const
  {
    return maneuver_active_;
  }

  bool campsiteActive() const
  {
    return campsite_active_;
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

  CommandSourceArbiterConfig config_;
  bool maneuver_active_{false};
  bool campsite_active_{false};
  double release_hold_until_sec_{-std::numeric_limits<double>::infinity()};
  double last_navigation_rotation_sec_{-std::numeric_limits<double>::infinity()};
};

}  // namespace camrod_control
