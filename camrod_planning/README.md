# camrod_planning

<!-- HH_260805 - Record the scoped Nav2 planner/controller production topology
and retain standalone field-isolation fallbacks. -->
<!-- HH_260806 - Synchronize the fabrication-inclusive planning polygon and
the B1-B10 turnaround / B11-B13 roadside service policy. -->

Nav2 lifecycle servers, Lanelet routing, goal snapping, local paths, fallback
planners/controllers, and semantic mission state.

![Planning runtime](../docs/assets/module-guides/planning/nav2-servers-and-mission-states.png)

## Actual Simulation Runtime

![Live B6 global and local paths](../docs/assets/module-guides/planning/runtime-b6-global-local-path-20260804.png)

`SIM RUNTIME CAPTURE`: historical 2026-08-04 B6 goal, LaneletRoute global path,
local path, and robot pose. Current map-v16 campsite sequencing evidence is
shown separately below.

## At A Glance

| Uses | Function | Main outputs |
|---|---|---|
| Nav2 planner/controller/BT/smoother servers | Plans and follows map-constrained routes | `/planning/global_path`, `/control/nav2_cmd_vel_ros` |
| Lanelet2 route graph and goal snapper | Converts UI/RViz goals to reachable lanelet routes | Route lanelet IDs and snapped goals |
| Planning state machine | Coordinates Nav2, local maneuvers, return, and parking | `/planning/state`, `/service/state` handoffs |

## Active Selection

| Item | Value |
|---|---|
| Full-bringup planner | `LaneletRoute` |
| Full-bringup controller | `RPP` |
| Production planner plugins loaded | `LaneletRoute`, `SmacLattice` |
| Implemented but dormant planners | `Smac2D`, `NavFn`, `ThetaStar`, `SmacHybrid` |
| Production controller plugins loaded | `RPP`, `RotationShim` |
| Implemented but dormant controllers | `DWB`, `MPPI`, `Graceful` |
| Controller frequency | `15 Hz` |
| RPP desired speed | `0.4 m/s` |
| RPP lookahead | `1.1..2.0 m` |
| RPP reverse / rotate-to-heading | disabled / enabled at `2 deg` |
| Physical body boundary | `1.39160 x 1.07000 m` |
| Nav2 planning footprint | `1.49160 x 1.17000 m` (body plus `0.05 m` each side) |
| Obstacle fallback | `SmacLattice` only on a proven `>= 2.50 m` lane, then restore `LaneletRoute` |

Normal missions construct only policy-reachable planner/controller instances.
Definitions for every implementation remain in `nav2_base.yaml`; the
development-only `nav2_{planner,controller}_profiles/all.yaml` files load every
option when comparison work is needed.

## Nav2 Process Boundary

`use_nav2_container:=true` is the production default. The vendored planner and
controller share `scoped_component_container_mt`; smoother, behavior, BT
navigator, and lifecycle manager remain standalone because their Humble vendor
binaries own private default-context executors. Transient-local Nav2 publishers
use DDS rather than unsupported Humble intra-process transport.

The scoped path reached all managed nodes active, used `robot_center_link`,
reached `[SYSTEM] OK`, and cleanly stopped the Nav2 plus five system containers
in 3/3 final amd64 runs. `use_nav2_container:=false` remains the field-isolation
fallback. Jetson still requires ten mission/cancel/restart cycles and resource
measurement; amd64 lifecycle success is not a physical navigation claim.

## Route Flow

```text
UI mission key + site goal
  -> goal snapper
  -> LaneletRoute planner
  -> BT/smoother/controller selectors
  -> RPP
  -> /control/nav2_cmd_vel_ros
  -> camrod_control safety gate
```

| Cost source | Role |
|---|---|
| `/map/cost_grid/lanelet` | Lane-centered global planning base |
| `/sensing/cost_grid/lidar` | Dynamic LiDAR obstacles |
| `/sensing/cost_grid/radar` | Near-field radar obstacles |
| `/planning/cost_grid/inflation` | Merged planning/safety cost |

## Dynamic Obstacle Replanning

| Requirement | Active value |
|---|---:|
| Dynamic blockage persistence before replan | `>= 20.0 s` |
| Lanelet grid freshness | `<= 2.5 s` |
| Contiguous lane width | `>= 2.50 m` |
| Clearance from path reference | `>= 0.60 m` on each side |
| Fallback / restore | `SmacLattice / LaneletRoute` |

LiDAR/radar blockage still reports status and reaches the control safety gate
immediately on every lane. The monitor preempts Nav2 only after the blockage
persists for 20 seconds and when the current path, goal,
pose, and fresh lanelet grid are available and the measured cross-section
passes all width checks. Missing/stale geometry, an outside-lane center, or a
narrow corridor publishes `BLOCKED_REPLAN_DENIED`; it keeps `LaneletRoute` and
stops fail-closed instead of requesting an insufficiently bounded detour. The
fallback remains constrained by the global lanelet cost layer, Smac footprint
checks, and the final control gate; the width test alone is not a traffic-rule
or lane-direction proof.

## State Surfaces

| Surface | Meaning | Example |
|---|---|---|
| `/planning/state` | Planner lifecycle | running, goal reached, recovery, error |
| `/service/state` | User-visible operation | moving to site, entering, waiting, returning, charging |
| gate status | Final motion authorization | enabled, route safety hold, battery stop |
| `/system/status` | Health only | OK, WARN, ERROR |

An expected Nav2 cancel during `SITE_ENTRY` is a handoff to
`camrod_control`, not a persistent health warning. A planning abort while
`MOVING_TO_SITE` remains a real warning/error condition. Operator cancel ends
in explicit `OPERATOR_STOPPED` state.

## Ownership Handoffs

| Phase | Motion owner |
|---|---|
| Drop-zone wait | none |
| Route to campsite | Nav2/RPP |
| B1-B10 site entry, zero-turn, unload wait, site exit | campsite maneuver controller |
| B11-B13 roadside arrival and wait | campsite maneuver controller; return geometry field-pending |
| Return route | Nav2/RPP |
| Drop-zone alignment and station exit | drop-zone maneuver controller |
| Final parking | selected reverse or AprilTag controller |
| Boundary contact recovery | bounded recovery controller after gate proof |

## Current Evidence

| Check | Result |
|---|---|
| Nav2 server/plugin topology | Source-derived and test-covered |
| RPP center-frame route A/B | Common segment cross-track RMS improved `0.0588 -> 0.0549 m` |
| Oscillation in compared run | `0` yaw-step sign reversals in both A/B runs |
| Historical map-v14 B6 route | Stops at boundary near `(4.3688, 45.0583)` and latches after one rapid retry |
| Active map-v16 source | Synchronized SHA `fd9c18...d0cf`; 55 lanelets/14 areas/1658 nodes; LaneletRoute contracts pass |
| Reduced-boundary route evidence | Historical `1.29160 x 0.87000 m` run: `10.0403 m`, goal error `0.2932 m`, bounded margin recovery, physical hard stop |
| B1-B10 site maneuver round trip | `CRAB_IN -> ROTATE_180 -> ... -> CRAB_OUT -> DONE`; all 10/10 PASS |
| B11-B13 roadside arrival | `CRAB_IN -> UNLOAD_WAIT -> WAIT_RETURN`; all PASS with no zero-turn and no RETURN command |
| B11-B13 return | Physical-body lanelet stop observed during the prior on-lane alignment; field geometry decision pending |

The A/B run supports the current `1.1 m` RPP lookahead and center-frame choice
for the compared route. It does not prove every lane width or physical vehicle
behavior.

![Historical reduced-boundary policy](../docs/assets/test_result/robot-boundary-adjustment-20260806/02-runtime-boundary-policy.png)

![Current campsite sequencing policy](../docs/assets/test_result/camping-site-sequencing-20260806/campsite-policy-validation.png)

![Current campsite phase order](../docs/assets/test_result/camping-site-sequencing-20260806/campsite-phase-sequence.gif)

Nav2 uses the larger planning polygon for both local and global footprint
checks. `camrod_control` independently samples the physical body first:
body contact cannot be reclassified as a recoverable planning-margin contact.

![Robot-center narrow-route risk](../docs/assets/module-guides/planning/robot-center-narrow-route-risk-map.png)

The highlighted samples are planning-footprint failures on the current map.
Control recovery can release an individual contact, but it cannot make an
undersized mapped corridor mission-capable.

## Run And Validate

```bash
ros2 launch camrod_planning planning.launch.py
ros2 launch camrod_planning planning.launch.py use_nav2_container:=true  # bench only

ros2 topic echo /planning/state
ros2 topic echo /planning/route_lanelet_ids
ros2 topic hz /planning/global_path
ros2 action info /planning/navigate_to_pose
```

| Config | Purpose |
|---|---|
| `config/nav2_base.yaml` | Servers, plugins, costmaps, RPP, and fallback controllers |
| `config/nav2_planner_profiles/production.yaml` | Loads only normal and reachable fallback planners |
| `config/nav2_planner_profiles/all.yaml` | Opt-in six-planner development profile |
| `config/nav2_controller_profiles/production.yaml` | Loads mission `RPP` and manual-goal `RotationShim` only |
| `config/nav2_controller_profiles/all.yaml` | Opt-in five-controller development profile |
| `config/obstacle_replan_monitor.yaml` | Dynamic blockage and wide-lane preemption policy |
| `config/camping_sites.yaml` | Site mission keys, service modes, and goal poses |
| `config/planning_state_machine.yaml` | Mission and ownership transitions |
| `config/goal_snapper.yaml` | Lanelet-aware goal release policy |
