#pragma once

// HH_260729 / TODOLIST 11-12 - Reissue a retained Nav2 goal after an explicit
// route-safety hold clears. The shared develop policy requires ABORTED; an
// external-simulator profile may explicitly allow ACCEPTED/EXECUTING to rebuild
// the controller/path context from the corrected robot pose.

#include <algorithm>
#include <optional>

namespace camrod_planning
{

struct RouteGoalRecoveryConfig
{
  bool enabled{true};
  // Develop reissues only an action that aborted during the route hold. The
  // CARLA tuned profile may explicitly allow refreshing an action that remains
  // ACCEPTED/EXECUTING while control performs the recovery.
  bool reissue_while_nav_active{false};
  double clear_delay_s{0.5};
  double minimum_reissue_interval_s{2.0};
  int maximum_reissues_per_goal{2};
};

class RouteGoalRecovery
{
public:
  explicit RouteGoalRecovery(RouteGoalRecoveryConfig config = {})
  : config_(config)
  {
  }

  void setConfig(const RouteGoalRecoveryConfig & config)
  {
    config_ = config;
  }

  void resetForGoal()
  {
    route_hold_seen_ = false;
    nav_active_ = false;
    nav_aborted_ = false;
    nav_terminal_ = false;
    gate_enabled_ = false;
    clear_since_sec_.reset();
    reissue_count_ = 0;
    last_reissue_sec_.reset();
  }

  void observeRouteHold(const bool active, const bool gate_enabled, const double now_sec)
  {
    gate_enabled_ = gate_enabled;
    if (active) {
      route_hold_seen_ = true;
      clear_since_sec_.reset();
      return;
    }
    if (!route_hold_seen_ || !gate_enabled_) {
      clear_since_sec_.reset();
      return;
    }
    if (!clear_since_sec_.has_value()) {
      clear_since_sec_ = now_sec;
    }
  }

  void observeNavAborted()
  {
    if (terminalSuppressionEnabled() && nav_terminal_) {
      return;
    }
    nav_active_ = false;
    nav_aborted_ = true;
  }

  void observeNavActive()
  {
    if (terminalSuppressionEnabled() && nav_terminal_) {
      return;
    }
    nav_active_ = true;
    nav_aborted_ = false;
  }

  void observeNavSucceeded()
  {
    if (terminalSuppressionEnabled()) {
      suppressTerminalGoal();
      return;
    }
    // origin/develop grouped ACTIVE and SUCCEEDED: either status only cleared
    // an earlier ABORTED observation.  Keep that exact default transition;
    // terminal suppression is required only by the opt-in active-goal reissue
    // policy.
    nav_active_ = false;
    nav_aborted_ = false;
  }

  void observeNavCanceled()
  {
    // An operator cancellation is authoritative for the retained goal.  Do
    // not let an earlier route hold/active pair restart it after a later
    // engage transition.  A newly published goal calls resetForGoal().
    suppressTerminalGoal();
    if (!terminalSuppressionEnabled()) {
      // The develop policy clears this event but does not latch it. A later
      // action status can therefore establish a new eligible state for the
      // same retained goal, exactly as before the simulator extension.
      nav_terminal_ = false;
    }
  }

  bool ready(const double now_sec) const
  {
    const bool eligible_nav_state =
      nav_aborted_ || (config_.reissue_while_nav_active && nav_active_);
    if (!config_.enabled || !route_hold_seen_ || !eligible_nav_state ||
      (terminalSuppressionEnabled() && nav_terminal_) || !gate_enabled_ ||
      !clear_since_sec_.has_value())
    {
      return false;
    }
    if (reissue_count_ >= std::max(0, config_.maximum_reissues_per_goal)) {
      return false;
    }
    if (now_sec - *clear_since_sec_ < std::max(0.0, config_.clear_delay_s)) {
      return false;
    }
    return !last_reissue_sec_.has_value() ||
           now_sec - *last_reissue_sec_ >=
           std::max(0.0, config_.minimum_reissue_interval_s);
  }

  void markReissued(const double now_sec)
  {
    ++reissue_count_;
    last_reissue_sec_ = now_sec;
    route_hold_seen_ = false;
    nav_active_ = false;
    nav_aborted_ = false;
    clear_since_sec_.reset();
  }

  int reissueCount() const {return reissue_count_;}
  bool routeHoldSeen() const {return route_hold_seen_;}
  bool navActive() const {return nav_active_;}
  bool navAborted() const {return nav_aborted_;}

private:
  bool terminalSuppressionEnabled() const
  {
    return config_.reissue_while_nav_active;
  }

  void suppressTerminalGoal()
  {
    route_hold_seen_ = false;
    nav_active_ = false;
    nav_aborted_ = false;
    nav_terminal_ = true;
    gate_enabled_ = false;
    clear_since_sec_.reset();
  }

  RouteGoalRecoveryConfig config_;
  bool route_hold_seen_{false};
  bool nav_active_{false};
  bool nav_aborted_{false};
  bool nav_terminal_{false};
  bool gate_enabled_{false};
  std::optional<double> clear_since_sec_;
  int reissue_count_{0};
  std::optional<double> last_reissue_sec_;
};

}  // namespace camrod_planning
