# CAMROD v2.1.4 Release Notes

<!-- HH_260805 - Record exact active behavior, its evidence level, and the
remaining Jetson/real-robot work for the v2.1.4 release. -->

## Scope

v2.1.4 synchronizes the current `develop` runtime after the `for_merge` UI and
bringup review, user-authored Lanelet map revision 15, robot-center footprint
adjustment, planner/runtime resource work, and package visual documentation.
It is a source, configuration, unit-test, and workstation-simulation baseline.
It is not a new full-route or Jetson field acceptance result.

## Active Values

| Contract | v2.1.4 value | Operational meaning |
|---|---:|---|
| Navigation frame | `robot_center_link` | Axle midpoint for planning/control; rear-axle compatibility remains `robot_base_link` at X `-0.443 m` |
| Measured body | `1.49160 x 1.07000 m` | Physical hard-stop geometry, unchanged |
| Planning boundary | `1.59160 x 1.17000 m` | Static body plus `0.05 m` on front, rear, left, and right |
| Center-frame extents | `0.80837 / 0.78323 / 0.58505 / 0.58495 m` | Front / rear / left / right |
| Active map | `map_version=15` | User-authored active/copy OSM files are synchronized |
| Production planners | `LaneletRoute`, `SmacLattice` | Normal route plus reachable wide-lane fallback |
| Production controllers | `RPP`, `RotationShim` | Mission tracking plus manual clicked-yaw handling |
| Persistent block | `20.0 s` | Delay before planner preemption, not before safety stop |
| Fallback width | `>= 2.50 m` | Minimum contiguous mapped width |
| Fallback side clearance | `>= 0.60 m` each | Left and right clearance requirement |
| RPP minimum lookahead | `1.1 m` | Unchanged low-speed tracking value |
| Route recovery | `0.10 m/s`, `0.40 m`, `10 s` | Bounded crab/reverse; contact angular command remains zero |
| Retry containment | `1 release`, `5.0 s` | Rapid same-route recontact latches final zero output |
| Mission admission | SOC `>= 35%` | New campsite mission can depart |
| Hard battery stop | SOC `<= 20%` | Final motion output is stopped |

## Runtime Changes

### Planning and Control

- The physical body and cost-100 stop contract are unchanged. Only the static
  planning clearance changed from 10 cm front/rear and 5 cm sides to 5 cm on
  all four sides. LiDAR/radar dynamic obstacle corridors were not reduced.
- Package defaults, bringup mirrors, Nav2 costmaps, platform visualization,
  sensor-kit geometry, safety gate config, and C++ defaults use the same polygon.
- Dynamic obstacle evidence closes the command gate immediately. Only the
  `LaneletRoute` to `SmacLattice` preemption waits for 20 seconds of continuous
  blockage and requires fresh, sufficiently wide lanelet evidence.
- `Smac2D`, `NavFn`, `ThetaStar`, and `SmacHybrid` implementations remain in
  `nav2_planner_profiles/all.yaml` for explicit experiments; production does
  not construct dormant plugins.
- `DWB`, `MPPI`, and `Graceful` controller definitions remain available through
  `nav2_controller_profiles/all.yaml`; production does not construct them.
- The engage-aware progress checker no longer owns its parent lifecycle node.
  It releases parameter/subscription callbacks in its destructor, and the
  vendored controller server destroys the progress plugin during cleanup. This
  removes the full-stack shutdown ownership cycle without changing tracking.
- A turnaround campsite completes crab entry and the 180-degree zero-turn
  inside the site before waiting for unload/RETURN. RETURN does not move the
  zero-turn to the lanelet departure phase.

### UI and System Runtime

- Robot UI restores the B-site verification keypad and Guest-driven return
  flow while keeping operator stop/cancel propagation and lifecycle display.
- Guest WebSocket sends are serialized, ROS work does not block the async event
  loop, and heartbeat/stale-client handling releases slots deterministically.
- Robot and Guest uvicorn servers receive an orderly stop request and join their
  event-loop threads before ROS entities are destroyed. Normal Ctrl+C shutdown
  no longer runs a concurrent post-launch process-kill pass by default.
- The default local operator surface is WebKitGTK. It probes backend readiness,
  caches static resources, enables smooth touch scrolling, requests
  `HardwareAccelerationPolicy.ALWAYS`, and disables only unused WebGL.
- Twenty-four low-rate system checkers run in three component containers:
  hardware/sensing, localization, and autonomy. Standalone executables remain
  available for fault-isolation tests. Each category uses one serialized
  executor; this keeps the three fault boundaries while avoiding the observed
  Cyclone DDS library-unload race from multi-threaded container teardown.

### Map, Geometry, and Evidence

- Both active OSM files carry user-authored map revision 15 and remain byte
  identical. The release process does not rewrite their geometry.
- Historical map-v14 retry JSON/PNG/GIF remains tied to its original map SHA.
  It demonstrates fail-closed retry containment, not map-v15 route completion.
- Package architecture images now use distinct paired palettes. Safety red is
  common across packages, preserving stop/failure semantics.
- PNG/GIF files are derived views. Referenced JSON and concise logs under
  `docs/evidence` remain as measured-label provenance; redundant unreferenced
  raw logs and unused manual timelines were removed.

## Validation Boundary

- The selected release build/test scope covered the eight top-level CAMROD
  packages plus the bundled `nav2_controller`. Their emitted xUnit results
  totalled 380 tests, 0 errors, 0 failures, and 24 skips. The `camrod_ui`
  setuptools test command emits no pytest cases, so its 27 contract tests were
  also run directly with `python3 -m pytest`; all passed. The optimized React
  production build compiled and synchronized successfully.
- Source/config contract tests cover the 20-second transition boundary,
  wide/narrow/stale map gating, planner profile synchronization, four-sided
  footprint synchronization, campsite turnaround ordering, system component
  grouping, WebKit policy, Robot/Guest UI behavior, and visual regeneration.
- An offline map-v15 sweep sampled 58 poses on lanelets `754/2751/2720` with
  lateral adjustment up to `+/-0.4 m` and yaw adjustment up to `+/-20 deg`.
  The previous asymmetric envelope had seven center-pose failures and the
  current all-side 5 cm envelope had six; both had zero samples without a tested
  adjustment. This is bounded geometry evidence, not a dynamic completion run.
- Workstation WebKit, Nav2, system-component, and simulation checks do not
  establish Jetson GPU usage, real wheel response, sensor timing, or charging.
- Full `sim:=true` bringup loaded map v15, reached `[SYSTEM] OK` with 87 ROS
  nodes, exposed exactly `LaneletRoute/SmacLattice` and `RPP/RotationShim`, and
  returned HTTP 200 from Robot and Guest UI in about 6 ms on this workstation.
- Five isolated system shutdown repetitions loaded 120 checker components in
  total and produced 15 clean container exits. A subsequent full-stack Ctrl+C
  run cleanly stopped the controller server, both UI servers, and all three
  checker containers with no `-11` process exit.

## Field Work Still Required

1. Re-run normal-route, one-side boundary contact, body contact, crab/reverse,
   zero-turn, campsite round trip, parking, and charging on the real robot.
2. Profile `RViz + WebKit`, `WebKit only`, and local-window-off on Jetson using
   process CPU/RAM, `tegrastats`, frame pacing, first-load time, and sensor rates.
3. Verify immediate obstacle stop plus no planner preemption at 19.9 seconds and
   width-gated SmacLattice preemption at 20 seconds or later.
4. Run all seven physical radar channels in a cleared area before adding any
   fixed-return exclusion. No new radar exclusion is part of v2.1.4.
5. Complete the rear-camera 10 Hz, dual-GNSS Fixed, IMU startup, speaker/state,
   and charging feedback checks in `TODOLIST.txt`.

Detailed procedures and pass criteria are in
[`TODOLIST.txt`](../TODOLIST.txt) and
[`field_test_runbook.md`](../camrod_bringup/docs/field_test_runbook.md).
