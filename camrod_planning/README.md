# camrod_planning

<!-- HH_260804 - Make active Nav2 plugins, state surfaces, handoffs, and
simulation limits visible without a long chronological narrative. -->

Nav2 lifecycle servers, Lanelet routing, goal snapping, local paths, fallback
planners/controllers, and semantic mission state.

![Planning runtime](../docs/assets/module-guides/planning/nav2-servers-and-mission-states.png)

## Actual Simulation Runtime

![Live B6 global and local paths](../docs/assets/module-guides/planning/runtime-b6-global-local-path-20260804.png)

`SIM RUNTIME CAPTURE`: live B6 goal, LaneletRoute global path, local path, and
robot pose. The route is generated correctly but later fails closed at the
unsurveyed service-access boundary.

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
| Planner plugins loaded | `LaneletRoute`, `Smac2D`, `NavFn`, `ThetaStar`, `SmacHybrid`, `SmacLattice` |
| Controller plugins loaded | `RPP`, `DWB`, `MPPI`, `Graceful`, `RotationShim` |
| Controller frequency | `15 Hz` |
| RPP desired speed | `0.4 m/s` |
| RPP lookahead | `1.1..2.0 m` |
| RPP reverse / rotate-to-heading | disabled / disabled |
| Obstacle fallback | `SmacLattice`, then restore `LaneletRoute` |

Loaded plugins are available options; the selector values above identify the
ordinary full-bringup path.

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
| Site entry, turnaround, unload wait, site exit | campsite maneuver controller |
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
| Map-v14 B6 route | Stops at boundary near `(4.3688, 45.0583)` and latches after one rapid retry |
| Full campsite round trip | Not demonstrated; map boundary still blocks campsite entry |

The A/B run supports the current `1.1 m` RPP lookahead and center-frame choice
for the compared route. It does not prove every lane width or physical vehicle
behavior.

![Robot-center narrow-route risk](../docs/assets/module-guides/planning/robot-center-narrow-route-risk-map.png)

The highlighted samples are planning-footprint failures on the current map.
Control recovery can release an individual contact, but it cannot make an
undersized mapped corridor mission-capable.

## Run And Validate

```bash
ros2 launch camrod_planning planning.launch.py

ros2 topic echo /planning/state
ros2 topic echo /planning/route_lanelet_ids
ros2 topic hz /planning/global_path
ros2 action info /planning/navigate_to_pose
```

| Config | Purpose |
|---|---|
| `config/nav2_base.yaml` | Servers, plugins, costmaps, RPP, and fallback controllers |
| `config/camping_sites.yaml` | Site mission keys, service modes, and goal poses |
| `config/planning_state_machine.yaml` | Mission and ownership transitions |
| `config/goal_snapper.yaml` | Lanelet-aware goal release policy |
