# Bringup Smoke Raw Logs

<!-- HH_260804 - Preserve the exact ROS node logs behind the normalized
campsite smoke evidence instead of leaving them only under ~/.ros/log. -->

The six mission-role files are unchanged copies from the two full-bringup
simulations summarized by
[`campsite-smoke-20260804.json`](../campsite-smoke-20260804.json). The runtime
visual file is a deliberately concise excerpt that also preserves the matching
CLI stop result; all copied ROS timestamps and message text remain intact.

| Scenario | Node role | File | Key lines |
|---|---|---|---|
| B6 `turnaround` | campsite maneuver | [`b6-turnaround-maneuver.log`](b6-turnaround-maneuver.log) | 13 `CRAB_IN`, 28 timeout |
| B6 `turnaround` | safety gate | [`b6-turnaround-safety-gate.log`](b6-turnaround-safety-gate.log) | 49 `lanelet_footprint_cost` hold |
| B6 `turnaround` | system diagnostic | [`b6-turnaround-system.log`](b6-turnaround-system.log) | 57 `[SYSTEM] OK` |
| B12 `roadside_stop` | campsite maneuver | [`b12-roadside-maneuver.log`](b12-roadside-maneuver.log) | 13 `CRAB_IN`, 30 timeout |
| B12 `roadside_stop` | safety gate | [`b12-roadside-safety-gate.log`](b12-roadside-safety-gate.log) | 43 `lanelet_footprint_cost` hold |
| B12 `roadside_stop` | system diagnostic | [`b12-roadside-system.log`](b12-roadside-system.log) | 62 `[SYSTEM] OK` |
| B6 map-v14 live capture | full-stack retry and stop | [`runtime-visual-capture-20260804.log`](runtime-visual-capture-20260804.log) | one release, 0.276 s rapid recontact latch, zero Twist, HTTP 200 stop, state 16 |

The safety-gate logs intentionally retain repeated hold/status lines. They show
that the complete-footprint violation remained latched rather than being hidden
by a shortened excerpt.
