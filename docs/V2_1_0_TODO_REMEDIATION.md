# CAMROD post-v2.1.0 TODO remediation

<!-- HH_260729 - Keep this implementation record separate from the immutable
     v2.1.0 release notes and from field acceptance evidence. -->

Date: 2026-07-29 (Asia/Seoul)

## Baseline identity

After `git fetch origin --prune --tags`, the latest remote `develop` and the
dereferenced annotated tag `v2.1.0^{}` both resolved to:

```text
4734a6e78ac56713dade0bf2e4c35c57da04cd12
```

`git diff origin/develop v2.1.0^{}` and the reverse comparison were empty. The
annotated tag object itself is `617b1d635...`; that object is tag metadata, not a
different source tree. Local `develop` was fast-forwarded to the common commit
before implementation began.

The changes below are a `develop` delta after that baseline. They do not
retroactively belong to the existing `v2.1.0` tag.

## Requirement-to-code traceability

<!-- HH_260729 - Map each field TODO to its implementation, configuration, and
     regression owner so later tuning cannot silently bypass the safety intent. -->

| TODO | Implemented responsibility | Primary code | Configuration | Regression evidence |
|---|---|---|---|---|
| 11 | Retain the first lanelet violation and require continuous fresh route-clear evidence | `route_safety_recovery.hpp`, `motion_cost_stop.cpp`, `cmd_vel_safety_gate_node.cpp` | package/bringup `cmd_vel_safety_gate.yaml` | route direction, continuous clear, stale grid and stale pose tests |
| 11 | Reissue only the same goal that aborted during that hold | `route_goal_recovery.hpp`, `goal_snapper_node.cpp` | package/bringup `goal_snapper.yaml` | hold+abort prerequisite, delay, reset, and retry-bound tests |
| 12 | Permit only a constrained opposite escape while preserving every other interlock | `route_safety_recovery.hpp`, `motion_cost_stop.cpp` | package/bringup `cmd_vel_safety_gate.yaml` | projected footprint clear/blocked and rear dynamic-obstacle tests |
| 12 | Preserve explicit operator cancel and first re-engage semantics | `cmd_vel_safety_gate_node.cpp`, `goal_snapper_node.cpp` | no separate state override | canceled/unrelated failure cannot trigger reissue |
| 13 | Reduce translation while real wheel direction lags the new target | `steering_transition_policy.hpp`, `ranger_messenger.cpp` | package/bringup `ranger_driver.yaml`, `ranger.launch.py` | stop/interpolation/disabled-compatibility tests |

The `HH_260729 / TODOLIST 11-12` and `HH_260729 / TODOLIST 13` comments at the
policy and test entry points identify this change directly in source. Detailed
comments remain around fail-closed evidence, operator cancel, projected
footprint checks, exact-goal reissue, steering sign ordering, and parameter
validation.

## Problem analysis

### 1. Route stops were invisible to Nav2 progress ownership

The command gate blocked a lanelet full-footprint or route-corridor violation at
the final output, but `/control/command_enabled` did not retain that route stop
as an explicit condition. The engage-aware progress checker could therefore
continue measuring progress while the robot was deliberately held at zero.
Nav2 could eventually report `ABORTED`, and no owner was responsible for
reissuing the still-valid mission goal after the robot returned to the lane.

### 2. A later command could not define route-clear evidence safely

After a stop, Nav2 commonly emits zero or a command with changed geometry.
Using that new command to clear the original route check would prove the wrong
corridor. The trigger direction and reason must remain immutable until the
original full-footprint route is continuously clear.

### 3. A strict current-footprint check prevented any supervised escape

Identifying an opposite command was not sufficient. If the current footprint
already touched lanelet cost 100, applying the ordinary current-footprint check
again blocked the opposite command too. The safe exception is not to lower the
threshold. It is to require a short projected full footprint to be clear while
all present dynamic-obstacle and hardware interlocks remain active.

### 4. Ranger translation could precede wheel-direction convergence

Ranger limited the steering-angle transition but still sent full translational
speed while the limited wheel angle represented the previous motion direction.
Parallel steering also finalized a sign after rate limiting in one branch,
which could bypass the intended slew envelope. This contributes to lateral
overshoot even when the higher-level controller command is already shrinking.

## Implemented behavior

### Control-side route hold

`cmd_vel_safety_gate` now owns a `RouteSafetyRecovery` policy.

- A lanelet violation saves the original `AvgTwist` translation and reason.
- `/control/command_enabled` becomes false and module operating state becomes
  `ROUTE_SAFETY_HOLD`.
- A timer continues checking the saved direction even when Nav2 stops publishing
  useful commands.
- Missing, invalid, stale, or frame-mismatched lanelet grid/pose evidence is
  blocked, never clear. Pose observations older than 0.5 s are stale; a timer
  cannot make a previously received sample fresh.
- Release requires 1.0 s of continuously fresh clear evidence for the original
  direction.
- Zero, rotation, same-direction, or insufficiently opposite translation cannot
  replace the trigger or clear the hold.

### Constrained opposite-direction escape

An opposite command requires translation-vector cosine `<= -0.5`. It is then
evaluated with a 0.25 m projected pose:

1. The complete projected footprint must avoid lanelet cost 100 and
   unknown/out-of-grid cells.
2. The current requested escape corridor must pass fresh LiDAR, radar, and
   merged-grid checks.
3. Any retained dynamic-obstacle latch remains authoritative.
4. Engage, operator arm, ESTOP, CAN, charging, critical battery, localization
   hold, and command timeout remain unchanged.

Only present lanelet contact receives this bounded escape treatment. Ordinary
motion still checks the current full footprint exactly as v2.1.0 did.

### Planning-side retained-goal recovery

`goal_snapper` observes the gate's transient-local module status and the Nav2
action status. The exact active snapped goal and its manual/regulated source are
reissued only when:

1. `ROUTE_SAFETY_HOLD` was observed for the current goal;
2. Nav2 reported `ABORTED`;
3. the gate returned to `ENABLED`; and
4. enabled remained clear for 0.5 s.

Reissue has a 2.0 s minimum interval and a maximum of two attempts per goal.
A newly released goal resets recovery state. `SUCCEEDED`, `ACCEPTED`, or
`EXECUTING` clears the pending abort. Generic planning failure, a canceled
action without route hold, and operator stop do not independently trigger an
automatic restart.

### Ranger steering-transition velocity envelope

Dual-Ackermann and parallel steering now compute translation scale from:

```text
steering_error = abs(target_steering - rate_limited_steering)

error <= 0.05 rad  -> scale 1.0
error >= 0.35 rad  -> scale 0.0
otherwise          -> linear interpolation
```

The lower scale is configurable and defaults to zero. Parallel sign and maximum
steering clamp are applied before rate limiting. Startup and dynamic-parameter
validation enforce finite values, `0 <= full_speed_error < stop_error`, minimum
scale in `[0, 1]`, and steering rate in `[0.05, 2.0] rad/s`.

## State timeline

| Event | Gate operating state | Command output | Planning action |
|---|---|---|---|
| Healthy engaged route | `ENABLED` | Ordinary guarded command | Current goal active |
| Lanelet violation | `ROUTE_SAFETY_HOLD` | Zero | Progress checker paused by command-enabled=false |
| Same/zero/rotation command during hold | `ROUTE_SAFETY_HOLD` | Zero | Trigger remains unchanged |
| Opposite command, projected footprint unsafe | `ROUTE_SAFETY_HOLD` | Zero | No goal change |
| Opposite command, projection clear, obstacle clear | `ROUTE_SAFETY_HOLD` | Bounded escape command | No goal change |
| Original route clear for 1.0 s | `ENABLED` | Ordinary guarded command | Wait 0.5 s if ABORTED |
| Prior hold plus Nav2 ABORTED | `ENABLED` | From rebuilt Nav2 action | Same goal/source reissued, max 2 |
| Operator cancel | `STANDBY` or another normal hold | Zero | No automatic reissue |

`ROUTE_SAFETY_HOLD` is a WARN because operator attention and repositioning may
be required. It is distinct from a system ERROR and from expected charging or
disengaged standby.

## Configuration ownership

These package defaults and bringup mirrors are byte-identical:

- `camrod_control/config/cmd_vel_safety_gate.yaml`
  and `camrod_bringup/config/control/cmd_vel_safety_gate.yaml`;
- `camrod_planning/config/goal_snapper.yaml`
  and `camrod_bringup/config/planning/goal_snapper.yaml`;
- `camrod_platform/config/ranger_driver.yaml`
  and `camrod_bringup/config/platform/ranger_driver.yaml`.

The Ranger launch explicitly forwards every new driver parameter. String-like
boolean values use the launch file's `_truthy()` parser rather than Python
object truthiness.

## Safety invariants

- Lanelet footprint cost threshold remains 100.
- Full planning-boundary geometry and unknown/out-of-grid stop behavior remain.
- Dynamic obstacle latch source, probe geometry, freshness, clear window, and
  post-clear hold remain.
- Runtime recovery tuning cannot disable freshness/continuous-clear proof or
  enlarge the opposite projection beyond 0.5 m; an invalid update is rejected
  atomically and the previous values remain active.
- No automatic goal reissue occurs solely because planning failed.
- Reissue is bounded and preserves the existing goal rather than inventing a
  new mission.
- Steering velocity scaling can only reduce translation; it cannot increase the
  requested velocity or steering rate.

## Validation

Completed without robot hardware:

- package build: `camrod_control`, `camrod_planning`, `ranger_base`,
  `camrod_platform`, and `camrod_bringup`;
- control policy tests, including route trigger retention, fail-closed stale
  evidence, projected full-footprint escape, and dynamic rear-obstacle stop
  (`37/37` policy cases plus `2/2` reverse-parking-axis cases);
- planning tests for hold+abort prerequisites, clear delay, status reset, and
  retry bound (`3/3` route-recovery cases; all eight package CTest entries
  passed);
- Ranger tests for large-lag stop, interpolation/minimum scale, and disabled
  compatibility (`3/3`);
- all `camrod_platform` and `camrod_bringup` CTest entries, including their
  Python launch/config tests;
- package/bringup YAML byte comparisons, installed-config comparison, Python
  launch syntax check, and `field_test_tool.sh config`;
- runtime parameter callback: a valid `0.4 s` pose freshness update was
  accepted; unsafe `0.0 s` freshness and `1.0 m` projection updates were
  rejected without changing the active values.

An isolated ordinary-simulation smoke run (`ROS_DOMAIN_ID=92`,
`sim_platform_status_enable:=false`) started
`/control/cmd_vel_safety_gate`, `/planning/goal_snapper`,
`/planning/planning_state_machine`, and `/ranger_platform_bridge`.
`/platform/status` was live, the gate reported normal `STANDBY` rather than
`FAULT_HOLD`, `/control/command_enabled` was false only because engage and
drive-enable were false, and the installed nodes exposed recovery enable,
`0.25 m` projection, and two bounded reissues. No fatal error occurred before
the test shutdown signal.

The bringup's existing respawn behavior restarts several planning processes
during forced process-group shutdown; those restarted processes emitted
`ExternalShutdownException`/termination diagnostics after the test's SIGINT.
That shutdown-only behavior is outside this route-recovery change and is not
being counted as a clean-shutdown validation.

### 2026-07-30 indoor planning, latency, CPU, and state follow-up

<!-- HH_260730 - Distinguish measured software progress from the physical
     acceptance that remains deliberately open. -->

A fresh isolated `bringup.launch.py sim:=true` run completed the software side
of TODO 6-9 and part of the diagnosis for TODO 13:

<!-- HH_260730 - Replace the stale v1.0.1/moved-map cause with the active
     empty-profile v1.0.3 stable-map evidence. -->
- the final run loaded the empty-profile `lanelet2_maps.osm`, byte-identical to
  the `copy_park_v1.0.3` snapshot;
- manual and UI near goals both generated 22-pose, approximately 4.042 m
  global paths plus nonzero `/control/nav2_cmd_vel_ros` and final
  `/control/cmd_vel`;
- the regulated far UI goal applied its separate position/heading policy,
  generated a 249-pose, 49.083 m route, and produced nonzero commands;
- direct calls from `(-13.958, 43.540, 7.73 deg)` to the clicked goal
  `(12.173, 20.458, 107.8 deg)` showed NavFn, ThetaStar, SmacHybrid, and
  SmacLattice returning zero poses. The exact clicked cell had cost 254 and was
  nontraversable, rather than being separated by the old v1.0.1 0.860 m
  corridor cut;
- LaneletRoute succeeded in approximately 19.6 ms with 249 poses/49.047 m and
  a reachable cost-186 endpoint at
  `(11.1604526307, 20.1120538289, 107.8 deg)`. The route accumulated 4.892 rad,
  reached maximum cost 218, and used no cost >=252 cell;
- the pre-safe-snap far manual baseline released the exact clicked action goal
  even though the safe route endpoint was 1.07 m away. Manual position
  projection must be rebuilt and the four cases rerun before final arrival
  behavior is claimed; clicked yaw remains the manual orientation contract;
- the localization selector's odometry-before-pose callback order was fixed,
  reducing final-pose header age p50/p95 from 55.9/63.9 ms to 6.3/10.9 ms
  while preserving 20 Hz sim output;
- latest-only raw path/grid coalescing reduced the active
  `path_visualizer` sample from about 20% to about 9--11% of one core and the
  obstacle monitor from about 26--28% to about 19--20%, without reducing source
  path, monitor, threshold, lookahead, or safety timing;
- UI and voice now use one source-neutral sequence from initialization through
  goal, path, driving, safety stop, and arrival. Required inputs, graph,
  localization, TF, gate, and platform state remain strict; an intentional
  sensor `DUMMY/WARN` is degraded-ready while `ERROR` and
  startup/fault/inactive states remain blocking.

The complete conditions, measurements, and limits are recorded in
`camrod_bringup/docs/post_v2_1_0_indoor_validation.md`. No footprint padding,
lanelet cost, unknown-cell, dynamic-obstacle, or command-gate rule was weakened
to reach the off-corridor clicked cell.

### Evidence availability

The pre-fix real-robot launch logs referenced by the v2.1.0 release record live
under the Orin path `/home/nvidia/.ros/log/...`; they were not present on the
current `/home/hong` development host. Their extracted findings remain useful
for root cause: 46 lanelet-footprint stops, no radar stop reason in that
interval, and the camera/YOLO ownership failure.

The current host retains nominal simulation logs under `/tmp`, but those runs
did not create `ROUTE_SAFETY_HOLD`, opposite escape, retained-goal reissue, or
real Ranger steering lag. They prove startup/configuration only. No relevant
post-fix field rosbag was found, so TODO 11-13 remain field-pending.

`field_test_tool.sh record-recovery <log_dir>` was added to close this evidence
gap. It captures the required route, action, goal, gate, footprint, cost-grid,
command, platform, wheel, actuator, TF, and `/rosout` streams and creates a
parameter snapshot plus a structured `FIELD_RESULT.txt`.

The recorder was smoke-tested in isolated `ROS_DOMAIN_ID=94` with the real
control-gate executable. It requested all 27 recovery topics, retained eight
initially missing topics for later ROS discovery, recorded the five available
gate/command/log topics for 7.17 s (41 messages), shut down its rosbag on SIGINT,
and produced valid metadata, parameter evidence, missing/recorded-topic lists,
and all three TODO result sections. Real planning, cost-grid, and CAN/wheel
publishers were intentionally absent from this tool-only smoke test.

The vendored `ranger_base` functional test passes, but its pre-existing upstream
ament lint suite still reports repository-wide copyright, cpplint, flake8,
CMake whitespace, uncrustify, and package.xml schema debt. Those failures include
unchanged launch and header files and are not evidence of a steering-policy
functional failure.

## Deliberately pending

No real robot, CAN, camera, radar, LiDAR, GNSS, or IMU was started for this
implementation. Production-entry simulation now covers the software behavior
described above, but TODO items 1-10 still retain their hardware/field
acceptance. Items 11 and 12 have software coverage but still require the
runbook's physical exit/reentry, opposite escape, first re-engage, and timing
acceptance.

For item 13, driver-layer lag mitigation is implemented. Controller-side
lateral prediction, crossing hysteresis, gain/ratio changes, and command-delay
compensation remain pending until localization, controller, gate, and platform
timestamps are measured on the robot. Guessing those values could trade
overshoot for oscillation or delayed stopping, so no full-footprint guard or
controller threshold was weakened.
