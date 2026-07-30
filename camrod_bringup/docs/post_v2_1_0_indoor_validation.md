# Post-v2.1.0 indoor validation

<!-- HH_260730 - Preserve the measured indoor evidence separately from the
     still-pending physical field acceptance in TODOLIST.txt. -->

Date: 2026-07-30 (Asia/Seoul)

## Scope and safety

The runtime checks used the production entry point,
`camrod_bringup/bringup.launch.py`, with `sim:=true` on isolated ROS domains.
The fake platform was used, physical sensor acquisition was disabled, and the
software gates were returned to false after each command-generation case. No
physical robot movement occurred. These results validate software behavior;
they do not replace the outdoor low-speed acceptance items in `TODOLIST.txt`.

The operator-owned formatting/geometry changes in
`camrod_platform/config/robot_visualization.yaml` and its bringup mirror were
not modified by this audit.

<!-- HH_260730 - Tie the evidence to the active stable map instead of the
     superseded v1.0.1 moved-map measurement. -->
The final planning run loaded `lanelet2_maps.osm` with an empty `map_profile`.
That file is byte-identical to the `copy_park_v1.0.3` audit snapshot. Earlier
v1.0.1/moved-map measurements are not used as the current planner-failure
explanation below.

## Goal policy and command generation

The same simulated localization start was used for source-policy comparisons.
The ordinary Nav2 command source is `/control/nav2_cmd_vel_ros`;
`/control/cmd_vel_raw` belongs to campsite/drop-zone maneuver control and is
expected to remain zero during an ordinary Nav2 route.

| Input | Policy result | Global path | Command result |
|---|---|---:|---|
| manual near, yaw -172.66 deg | pre-safe-snap baseline retained clicked position/yaw; `RotationShim` + `manual_goal_checker` | 22 poses, 4.042 m | Nav2 raw max L1 1.0446; final max L1 0.5223; gate `ENABLED` |
| UI near, input yaw 90 deg | position moved 0.0943 m; yaw snapped to 5.202 deg | 22 poses, 4.042 m | Nav2 raw max L1 2.9350; final max L1 1.4675; gate `ENABLED` |
| manual far, yaw 107.8 deg | action goal retained clicked pose, but route endpoint was 1.07 m away | 249 poses, 49.082 m; local 154 poses/30.18 m | Nav2 raw max L1 1.044; final max L1 0.522; gate `ENABLED` |
| UI far, input yaw 90 deg | position moved about 1.07 m; yaw snapped to -71.137 deg | 249 poses, 49.083 m | Nav2 raw max L1 2.5669; final max L1 1.2834; gate `ENABLED` |

All four baseline cases reached path and nonzero command generation. They did
not close manual arrival semantics: the far manual action goal and the safe
LaneletRoute endpoint differed by 1.07 m, which exceeds the 0.25 m manual goal
checker tolerance. The code now projects manual x/y to the routable endpoint
while preserving the clicked final yaw. A fresh four-case runtime run is still
required before final manual behavior is claimed. Regulated UI input continues
to apply both lane position and heading rules before Nav2.

## Long-range planner audit

One direct `/planning/compute_path_to_pose` comparison used:

```text
start = (-13.958, 43.540, 7.73 deg)
goal  = ( 12.173, 20.458, 107.8 deg)
```

| Planner | Status | Poses | Length | Planning time |
|---|---|---:|---:|---:|
| NavFn | ABORTED | 0 | 0 m | no path |
| ThetaStar | ABORTED | 0 | 0 m | no path |
| SmacHybrid | ABORTED | 0 | 0 m | no path |
| SmacLattice | ABORTED | 0 | 0 m | no path |
| LaneletRoute | SUCCEEDED | 249 | 49.047 m | approximately 19.6 ms |

The raw global costmap was 2400 by 2400 cells at 0.1 m resolution. Its start
cell had cost 56 in traversable component 13247. The exact clicked goal cell
had cost 254 and was nontraversable, so NavFn, ThetaStar, SmacHybrid, and
SmacLattice correctly could not connect to that exact cell. This replaces the
superseded v1.0.1 explanation based on a 0.860 m corridor and cost-253 cut.

LaneletRoute projected the drive endpoint to
`(11.1604526307, 20.1120538289, 107.8 deg)`. That cell had cost 186 and was
reachable from the start. The 249-pose route was 49.047 m long, accumulated
4.892 rad of turn, reached a maximum path cost of 218, and contained no cells
at or above the lethal threshold of 252. It preserved the requested 107.8 deg
orientation at the safe endpoint.

The clicked lethal cell must not be appended to the route and no footprint,
padding, lethal/unknown-cost, or final command-gate protection is weakened.
The implemented manual-safe-snap change makes the released manual action goal
use the same reachable position as LaneletRoute while retaining the operator's
yaw. Its final four-case result remains pending a fresh run.

## Localization latency

The selector previously reacted to a new odometry callback while the matching
pose-covariance callback had not yet arrived. It emitted the previous pose with
the new source stamp and skipped the later matching pose. Selection now uses
the freshest header-stamped payload, independent of callback order.

| Topic | Rate | Before age p50/p95 | After age p50/p95 |
|---|---:|---:|---:|
| adapter primary pose | 20.01 Hz | 6.1 / 13.7 ms | 5.9 / 10.0 ms |
| final selected pose | 20.00 Hz | 55.9 / 63.9 ms | 6.3 / 10.9 ms |

The final pose retained its 20 Hz simulated output. The production
dual-antenna overlay still runs GNSS at 1 Hz; v2.1.1 raises the real EKF from
10 Hz to 15 Hz to match the controller. Moving absolute-correction behavior
remains an outdoor measurement because increasing EKF publication does not
create additional GNSS epochs.

## Planning CPU

The active local path was approximately 30.6--35.2 Hz, 10.3 KB, and 155 poses.
Deserializing one generated path in Python cost about 7.9 ms. The visualization
and obstacle-monitor callbacks previously paid that cost for every intermediate
path even though their useful work rates were only 2 Hz and 5 Hz.

The revised nodes keep only a depth-1 serialized sample and deserialize the
latest sample at their existing work cadence. Grid transforms and lateral
sample offsets are cached. No controller/safety source rate, lookahead,
threshold, or clear/latch timing was reduced.

| Process | Before snapshot | Post-change active-path window |
|---|---:|---:|
| `path_visualizer` | 19.7--21.3% of one core | approximately 9--11% |
| `obstacle_replan_monitor` | 26.2--27.5% of one core | approximately 19--20% |
| `planner_server` | approximately 46.7% of one core | approximately 40--49% |

The marker subscriber observed approximately 2.14 Hz, matching the configured
2 Hz rebuild cap. The desktop was not a clean embedded baseline: VS Code GPU
and renderer processes each consumed roughly 60% or more of one core. A
camera/YOLO-enabled five-minute field comparison with RViz, WebKit-only, and
window-off conditions remains required.

## UI and voice state contract

UI and voice now use the source-neutral sequence:

```text
INITIALIZING -> READY -> GOAL_RECEIVED -> PATH_PREPARING
             -> DRIVING -> SAFETY_STOP -> ARRIVED
```

Readiness requires all configured map, sensing, localization, planning,
control, platform, and system modules to be present and free of `ERROR` or
startup/fault/inactive states; NORMAL localization; `map -> robot_base_link`
TF; the Nav2 action server; a valid command-gate state; a non-faulted platform;
and the canonical planning engage state. `WARN`, including intentional sensor
`DUMMY DATA`, remains visible as degraded health but does not hold the UI and
voice in initialization. Required graph gaps remain blocking through explicit
`STARTING`, then `FAULT/ERROR` after the configured startup grace.

The first startup event is `system.startup`. `system.ready` is emitted once,
only after every prerequisite first becomes ready. `WAIT_DZ` is silent, a
generic planning warning is not called an obstacle, and navigation audio
requires a valid goal, engage, and an enabled gate. Manual and regulated UI
goals expose the same mission-phase words.

Pure-policy regression coverage passes for UI startup/mission state and voice
startup, degraded-ready versus error, manual/UI motion, return, hold, arrival,
and deduplication.
The v2.1.1 production-entry smoke test additionally reached `SYSTEM OK` and
played `system.startup` followed by `system.ready` after readiness completed.
The actual manual/UI speaker and WebKit mission sequences remain part of
outdoor acceptance.

## Test artifacts on the development machine

The temporary evidence directories are:

- `/tmp/camrod_sim_planning_theta`
- `/tmp/camrod_manual_far_audit.9hBbQF`
- `/tmp/camrod_final_sim.mPpuM7`

They include goal-policy JSON, direct-planner JSON, pose-latency JSON, costmap
connectivity JSON, process samples, and bringup logs. The active-map planning
reports are `goal_policy_final.json` and `costmap_connectivity_final.json` in
the final directory. `/tmp` is not release storage; the measured conclusions
above are the maintained project record.
