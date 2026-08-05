# CAMROD v2.1.4 Release Notes

<!-- HH_260805 - Record the final Chromium, transport, optional LiDAR-grid,
and staged recovery behavior without overstating workstation validation. -->

## Scope

v2.1.4 synchronizes the current `develop` runtime after the `for_merge` UI and
bringup review, user-authored Lanelet map revision 15 (`v1.0.5` copy),
robot-center footprint adjustment, planner/runtime resource work, composed
sensor transport, and package visual documentation.
It is a source, configuration, unit-test, and workstation-simulation baseline.
It is not a new full-route or Jetson field acceptance result.

## Commit Structure

| Commit | Scope |
|---|---|
| `7d22eb944` | Initial runtime, synchronized configuration, map v15, WebKit UI, Nav2 policy, and diagnostic composition |
| `1b6ee1603` | Safety, map, campsite, Nav2, system, and Robot/Guest UI regression contracts |
| `47323dc58` | Package guides, release records, reproducible renderers, PNG/GIF assets, and evidence cleanup |
| `dd4658b52` | Preserve the user-authored active map v15 and synchronize park copy `v1.0.5` |
| `b91e7818e` | Project and stage crab, reverse, and reverse-yaw boundary-recovery candidates |
| `337d2706a` | Chromium default, ROS intra-process paths, guarded DDS-SHM, and optional LiDAR cost-grid diagnostics |
| `49a8a96bd` | Map-v15 recovery traces, regenerated PNG/GIF assets, package guides, and field-test boundaries |

## Active Values

| Contract | v2.1.4 value | Operational meaning |
|---|---:|---|
| Navigation frame | `robot_center_link` | Axle midpoint for planning/control; rear-axle compatibility remains `robot_base_link` at X `-0.443 m` |
| Measured body | `1.49160 x 1.07000 m` | Physical hard-stop geometry, unchanged |
| Planning boundary | `1.59160 x 1.17000 m` | Static body plus `0.05 m` on front, rear, left, and right |
| Center-frame extents | `0.80837 / 0.78323 / 0.58505 / 0.58495 m` | Front / rear / left / right |
| Active map | `map_version=15`, park `v1.0.5` | User-authored active/current-copy OSM files are synchronized; `v1.0.4` remains historical |
| Production planners | `LaneletRoute`, `SmacLattice` | Normal route plus reachable wide-lane fallback |
| Production controllers | `RPP`, `RotationShim` | Mission tracking plus manual clicked-yaw handling |
| Persistent block | `20.0 s` | Delay before planner preemption, not before safety stop |
| Fallback width | `>= 2.50 m` | Minimum contiguous mapped width |
| Fallback side clearance | `>= 0.60 m` each | Left and right clearance requirement |
| RPP minimum lookahead | `1.1 m` | Unchanged low-speed tracking value |
| Route recovery | `0.10 m/s`, `0.10 rad/s`, `12 deg`, `0.40 m`, `10 s` | Projected staged crab/reverse/reverse-yaw |
| Retry containment | `1 release`, `5.0 s` | Rapid same-route recontact latches final zero output |
| LiDAR cost grid | default `OFF` | Optional component, graph node/topic, and checker contract share one toggle |
| Operator window | Chromium | GPU-enabled kiosk default; WebKit remains explicit/auto fallback |
| Process transport | ROS intra-process; DDS SHM opt-in | Camera/LiDAR hot paths use ownership transfer; global iceoryx stays OFF on Humble after full-graph capacity failure |
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
- Boundary recovery no longer latches a straight reverse command forever.
  Every update projects left/right crab, reverse, and both reverse-yaw arcs.
  It can reverse to create space, select the unique safe crab/yaw side, or use
  the original RPP turn sign when both short yaw arcs are clear. All stages
  share one distance/time/yaw budget and retain hard body/dynamic-obstacle stops.

### UI and System Runtime

- Robot UI restores the B-site verification keypad and Guest-driven return
  flow while keeping operator stop/cancel propagation and lifecycle display.
- Guest WebSocket sends are serialized, ROS work does not block the async event
  loop, and heartbeat/stale-client handling releases slots deterministically.
- Robot and Guest uvicorn servers receive an orderly stop request and join their
  event-loop threads before ROS entities are destroyed. Normal Ctrl+C shutdown
  no longer runs a concurrent post-launch process-kill pass by default.
- The default local operator surface is Chromium with backend-readiness wait,
  an isolated profile, GPU rasterization, zero-copy compositor, and disabled
  background throttling. WebKitGTK remains an explicit fallback; `auto` tries
  Chromium first and then WebKit.
- Twenty-four low-rate system checkers run as standalone processes by default.
  Component registrations and an opt-in partial composition remain for bench
  work, but repeated complete-graph Humble shutdowns intermittently produced
  `-11` exits from localization and autonomy component processes. The release
  therefore does not trade reliable shutdown for an unproven process reduction.
- LiDAR preprocessing and ground segmentation run in one intra-process
  component container. The LiDAR cost-grid component is independently
  default-off; when off, only its node/topic are removed from graph readiness
  and cost-grid diagnostics. When enabled, its transient-local publisher uses
  DDS inside the container because ROS 2 Humble rejects that QoS with
  intra-process transport. Physical LiDAR stream checks remain active.
- `enable_dds_shared_memory:=true` starts one RouDi daemon and selects the
  central CycloneDDS SHM profile before ROS modules. It is default-off because
  the full graph exceeded Humble iceoryx 2.0.5 static publisher/history limits
  and aborted nodes; the profile remains for bounded bench experiments.

### Map, Geometry, and Evidence

- The active OSM and `lanelet2_maps_(copy_park_v1.0.5).osm` carry user-authored
  map revision 15 and remain byte-identical. The release process does not
  rewrite their geometry; `v1.0.4` remains a historical copy.
- Historical map-v14 retry JSON/PNG/GIF remains tied to its original map SHA.
  It demonstrates fail-closed retry containment, not map-v15 route completion.
- Current map-v15 JSON/PNG/GIF records the staged owner dynamically selecting
  `REVERSE_YAW_RIGHT` and `CRAB_LEFT`. Both route cases still fail closed; this
  is current simulation evidence, not physical mission completion.
- Package architecture images now use distinct paired palettes. Safety red is
  common across packages, preserving stop/failure semantics.
- PNG/GIF files are derived views. Referenced JSON and concise logs under
  `docs/evidence` remain as measured-label provenance; redundant unreferenced
  raw logs and unused manual timelines were removed.

## Validation Boundary

- The preceding v2.1.4 baseline validation covered eight top-level CAMROD
  packages plus bundled `nav2_controller`: 380 emitted xUnit cases, zero
  errors/failures, and 24 skips. After the final transport/recovery delta, the
  five changed CAMROD packages were rebuilt and fresh CMake/xUnit suites
  reported control 60, sensing 97, system 5, and bringup 166 cases: 328 total,
  zero errors/failures/skips. The separately discovered bundled
  `ground_segmentation_ros2` package also rebuilt successfully.
- `camrod_ui` setuptools still emits no pytest cases, so its 27 contract tests
  were run directly and all passed. The optimized React production build also
  compiled and synchronized successfully.
- Source/config contract tests cover the 20-second transition boundary,
  wide/narrow/stale map gating, planner profile synchronization, four-sided
  footprint synchronization, campsite turnaround ordering, system component
  grouping, Chromium policy, optional LiDAR diagnostics, ROS transport,
  Robot/Guest UI behavior, and visual regeneration.
- An offline map-v15 sweep sampled 58 poses on lanelets `754/2751/2720` with
  lateral adjustment up to `+/-0.4 m` and yaw adjustment up to `+/-20 deg`.
  The previous asymmetric envelope had seven center-pose failures and the
  current all-side 5 cm envelope had six; both had zero samples without a tested
  adjustment. A separate full-simulation dynamic run selected reverse-yaw at
  maximum `0.05 rad/s`, released after `0.0582/0.0405 m`, latched recontact at
  `0.335/0.400 s`, and crabbed left `0.3378 m`; final commands were zero.
- Workstation Chromium/WebKit, Nav2, system-component, and simulation checks do not
  establish Jetson GPU usage, real wheel response, sensor timing, or charging.
- RouDi accepted the checked-in 212.6 MB application segment and two isolated
  ROS 2 processes exchanged a message. A full graph then reproduced publisher
  port exhaustion and `TOO_MANY_CHUNKS_HELD_IN_PARALLEL`; this is why the
  profile is opt-in rather than a production claim.
- Full `sim:=true` bringup loaded map v15, reached `[SYSTEM] OK` with 79 steady-state ROS
  nodes, exposed exactly `LaneletRoute/SmacLattice` and `RPP/RotationShim`, and
  returned HTTP 200 from Robot and Guest UI in about 6 ms on this workstation.
- An isolated LiDAR launch with the default OFF switch exited without creating
  a container or rasterizer. With only the switch enabled, it loaded exactly
  `/lidar/lidar_cost_grid`, exposed one transient-local publisher on
  `/sensing/cost_grid/lidar`, and shut down cleanly.
- Isolated checker component tests could stop cleanly, but repeated full-stack
  tests later reproduced intermittent component `-11` exits. The final stable
  run used the production standalone default and cleanly stopped all 24
  checkers, the controller server, and both UI servers.

## Field Work Still Required

1. Re-run normal-route, one-side boundary contact, body contact,
   crab/reverse/reverse-yaw,
   zero-turn, campsite round trip, parking, and charging on the real robot.
2. Profile `RViz + Chromium`, `Chromium only`, WebKit fallback, and
   local-window-off on Jetson using
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
