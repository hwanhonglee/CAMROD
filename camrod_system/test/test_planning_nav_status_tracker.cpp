#include <cstdint>
#include <iostream>
#include <string>

#include <action_msgs/msg/goal_status.hpp>
#include <avg_msgs/msg/avg_service_state.hpp>

#include "planning_nav_status_policy.hpp"
#include "planning_nav_status_tracker.hpp"

namespace
{

using action_msgs::msg::GoalStatus;
using Tracker = camrod_system::diagnostics::PlanningNavStatusTracker;

constexpr int64_t kSecond = 1'000'000'000LL;

Tracker::GoalUuid makeUuid(uint8_t suffix)
{
  Tracker::GoalUuid uuid{};
  uuid.back() = suffix;
  return uuid;
}

bool expect(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    return false;
  }
  return true;
}

// HH_260727: Reproduce the field failure where one retained ABORTED entry was
// republished and incorrectly reported as many independent aborts.
bool testRepeatedAbortedEntryCountsOnce()
{
  Tracker tracker;
  const auto uuid = makeUuid(1);

  bool ok = expect(
    !tracker.observe(uuid, GoalStatus::STATUS_EXECUTING, 0),
    "EXECUTING must not count as an abort");
  ok &= expect(
    tracker.observe(uuid, GoalStatus::STATUS_ABORTED, kSecond),
    "first ABORTED transition must be counted");

  for (int64_t second = 2; second <= 9; ++second) {
    ok &= expect(
      !tracker.observe(uuid, GoalStatus::STATUS_ABORTED, second * kSecond),
      "retained ABORTED entry must not be counted again");
  }

  ok &= expect(tracker.abortCount() == 1, "one UUID must contribute one abort");
  return ok;
}

// HH_260727: Independent goal UUIDs must remain independent abort samples.
bool testDistinctAbortedGoalsAreCounted()
{
  Tracker tracker;

  bool ok = expect(
    tracker.observe(makeUuid(1), GoalStatus::STATUS_ABORTED, kSecond),
    "first UUID must be counted");
  ok &= expect(
    tracker.observe(makeUuid(2), GoalStatus::STATUS_ABORTED, 2 * kSecond),
    "second UUID must be counted");
  ok &= expect(tracker.abortCount() == 2, "two UUIDs must contribute two aborts");
  return ok;
}

// HH_260727: A malformed terminal-state re-entry for one UUID must not inflate
// the rolling abort count.
bool testSameGoalCannotReenterAbortHistory()
{
  Tracker tracker;
  const auto uuid = makeUuid(3);

  bool ok = expect(
    tracker.observe(uuid, GoalStatus::STATUS_ABORTED, kSecond),
    "initial ABORTED must be counted");
  ok &= expect(
    !tracker.observe(uuid, GoalStatus::STATUS_CANCELED, 2 * kSecond),
    "CANCELED must not count as an abort");
  ok &= expect(
    !tracker.observe(uuid, GoalStatus::STATUS_ABORTED, 3 * kSecond),
    "same UUID must not re-enter abort history");
  ok &= expect(tracker.abortCount() == 1, "same UUID must remain one abort");
  return ok;
}

// HH_260727: Preserve the checker's existing strict greater-than-60-seconds
// rolling-window boundary.
bool testAbortWindowExpiresWithoutNewStatus()
{
  Tracker tracker;
  tracker.observe(makeUuid(4), GoalStatus::STATUS_ABORTED, 0);

  tracker.prune(60 * kSecond);
  bool ok = expect(tracker.abortCount() == 1, "abort must remain at exactly 60 seconds");

  tracker.prune(60 * kSecond + 1);
  ok &= expect(tracker.abortCount() == 0, "abort must expire after 60 seconds");
  return ok;
}

// HH_260810 - Reproduce the field handoff: the first return transition may
// terminate the site-route Nav2 goal, but a later return-route abort is real.
bool testReturnTransitionGraceIsBounded()
{
  using avg_msgs::msg::AvgServiceState;
  using camrod_system::diagnostics::shouldSuppressNavAbort;

  bool ok = expect(
    shouldSuppressNavAbort(
      AvgServiceState::RETURNING_TO_DROP_ZONE, 0.5, 3.0, true),
    "pre-transition return handoff abort inside grace must be suppressed");
  ok &= expect(
    shouldSuppressNavAbort(
      AvgServiceState::RETURNING_TO_DROP_ZONE, 3.0, 3.0, true),
    "return handoff grace includes its configured boundary");
  ok &= expect(
    !shouldSuppressNavAbort(
      AvgServiceState::RETURNING_TO_DROP_ZONE, 3.01, 3.0, true),
    "later return-route abort must remain visible");
  ok &= expect(
    !shouldSuppressNavAbort(
      AvgServiceState::RETURNING_TO_DROP_ZONE, 0.5, 3.0, false),
    "new return-route goal abort must remain visible inside handoff grace");
  ok &= expect(
    shouldSuppressNavAbort(AvgServiceState::SITE_ENTRY, 30.0, 3.0, false),
    "controller-owned site entry remains suppressed");
  ok &= expect(
    !shouldSuppressNavAbort(AvgServiceState::MOVING_TO_SITE, 0.1, 3.0, true),
    "normal route navigation abort must remain visible");
  return ok;
}

}  // namespace

int main()
{
  const bool ok =
    testRepeatedAbortedEntryCountsOnce() &&
    testDistinctAbortedGoalsAreCounted() &&
    testSameGoalCannotReenterAbortHistory() &&
    testAbortWindowExpiresWithoutNewStatus() &&
    testReturnTransitionGraceIsBounded();

  if (ok) {
    std::cout << "planning_nav_status_tracker tests passed\n";
    return 0;
  }
  return 1;
}
