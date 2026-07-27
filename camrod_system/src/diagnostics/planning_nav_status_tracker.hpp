#pragma once

#include <cstdint>
#include <deque>
#include <map>

#include <action_msgs/msg/goal_status.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

namespace camrod_system::diagnostics
{

// HH_260727: Track terminal aborts by Nav2 goal UUID so the same retained
// GoalStatusArray entry cannot be counted again on every status publication.
class PlanningNavStatusTracker
{
public:
  using GoalUuid = unique_identifier_msgs::msg::UUID::_uuid_type;

  bool observe(const GoalUuid & goal_uuid, int8_t status, int64_t now_nanoseconds)
  {
    auto [it, inserted] = goals_.try_emplace(goal_uuid);
    auto & goal = it->second;

    // HH_260727: Count the first transition into ABORTED for each UUID. Nav2
    // terminal states are retained in later status arrays, and an invalid
    // terminal-state re-entry must not create another abort for the same goal.
    const bool new_abort =
      status == action_msgs::msg::GoalStatus::STATUS_ABORTED &&
      !goal.abort_counted &&
      (inserted || goal.last_status != action_msgs::msg::GoalStatus::STATUS_ABORTED);

    goal.last_status = status;
    if (new_abort) {
      goal.abort_counted = true;
      abort_times_nanoseconds_.push_back(now_nanoseconds);
    }

    prune(now_nanoseconds);
    return new_abort;
  }

  void prune(int64_t now_nanoseconds)
  {
    // HH_260727: Preserve the original strict 60-second rolling-window
    // semantics while allowing diagnostics to expire history without waiting
    // for a new action-status message.
    while (
      !abort_times_nanoseconds_.empty() &&
      now_nanoseconds >= abort_times_nanoseconds_.front() &&
      now_nanoseconds - abort_times_nanoseconds_.front() > kAbortWindowNanoseconds)
    {
      abort_times_nanoseconds_.pop_front();
    }
  }

  std::size_t abortCount() const
  {
    return abort_times_nanoseconds_.size();
  }

private:
  struct GoalState
  {
    int8_t last_status{action_msgs::msg::GoalStatus::STATUS_UNKNOWN};
    bool abort_counted{false};
  };

  static constexpr int64_t kAbortWindowNanoseconds = 60'000'000'000LL;

  // HH_260727: UUIDs remain recorded for this checker process lifetime. This
  // guarantees an old terminal goal can never be misclassified as a new abort
  // if Nav2 republishes it after a quiet interval.
  std::map<GoalUuid, GoalState> goals_;
  std::deque<int64_t> abort_times_nanoseconds_;
};

}  // namespace camrod_system::diagnostics
