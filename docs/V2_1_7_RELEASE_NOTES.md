# CAMROD v2.1.7 Release Notes

<!-- HH_260810 - Release the shared tapered/rounded boundary, goal-independent
UI readiness, current ROS road evidence, and maintainable visualization tools. -->

Release date: 2026-08-10 (Asia/Seoul)

Previous baseline: `v2.1.6`

## Release Summary

v2.1.7 replaces independent rectangular boundary consumers with one canonical
tapered-front, rounded-corner contour while preserving the measured maximum
body extents. The physical contour is used for final lanelet hard-stop checks;
its exact `0.10 m` outward offset is used for planning and projected recovery.
Platform visualization and both Nav2 costmaps consume the same geometry.

The Robot/Guest UI readiness decision no longer depends on a goal event or an
idle callback ordering accident. Sensors, localization/TF, Nav2 availability,
the command gate, platform feedback, and system health are the prerequisites.
An authoritative `0.5 s` state heartbeat repairs an initial WebSocket snapshot
race without changing mission ownership.

The administrator UI now provides six leased live telemetry views for GNSS/IMU,
radar/LiDAR, cameras, trajectory, map/perception, and safety/control. This is the
daily operational replacement for the corresponding RViz/Tk inspection tools;
offline renderers remain evidence generators. Each view creates only its own
`4-11` ROS subscriptions and releases them on close or after a 12-second lease.

This release also makes package evidence reproducible. Fourteen CAMROD packages
are classified by measured ROS simulation, measured AMD64 simulation, runtime
capture, source inventory, or remaining field work. Nine offline cross-package
renderers now have one owner directory; live operational visualization remains
inside the package that publishes or consumes the relevant ROS data.

## Active Boundary Contract

| Item | v2.1.7 value | Runtime meaning |
|---|---:|---|
| Navigation origin | `robot_center_link` | Dual-Ackermann/crab/zero-turn transforms rotate about the axle midpoint |
| Physical extents | `1.39160 x 1.07000 m` | Cost-100 contact stops ordinary motion |
| Front shape | taper `0.12 m`, shoulder depth `0.12 m` | Short front face with sloped left/right shoulders |
| Physical corner | `R0.05 m`, 4 arc segments/corner | Six rounded vertices produce a 30-point contour |
| Planning contour | exact `0.10 m` outward offset | `1.59160 x 1.27000 m` extrema and `R0.15 m` corners |
| Recovery eligibility | monotonic inward overlap reduction | Swept physical body and endpoint planning contour must both be clear |
| Dynamic/interlock policy | fail closed | Geometry changes do not bypass live obstacles, stale evidence, CAN, E-stop, or SOC holds |

Canonical shape parameters live in
`camrod_sensor_kit/config/robot_params.yaml`. Package-owned and bringup-mirrored
control, platform, sensor-kit, and Nav2 YAML remain synchronized. A source
regression regenerates the polygons and compares both deployed Nav2 footprint
strings to six decimal places.

## Historical Measured ROS Simulation

The preserved map-v17 B2 run contains 511 ROS pose samples over `29.700 s`:

| Observation | Result |
|---|---:|
| Physical cost-100 contact | no |
| Planning cost-100 contact | yes |
| Recovery stage | `REVERSE_YAW_RIGHT` |
| Recovery translation | `0.0972 m` |
| Recovery yaw | `-4.545 deg` |
| Second hold / retry latch | no / no |
| Original route completion | yes |
| Final command | zero |

Raw timeline, stable summary, historical geometry/map hashes, PNG, 80-frame GIF, and the
regeneration test are under
`docs/assets/module-guides/control/test-results/tapered-rounded-boundary-road-sim-20260810/`. This is
AMD64 ROS simulation evidence, not physical clearance or actuator acceptance.

## UI Initialization

- Initialization can latch as soon as all real prerequisites are healthy, even
  if manual engage or a campsite request arrived before the final prerequisite.
- Engage without a goal does not falsely display driving.
- The backend publishes unchanged authoritative readiness/mission state every
  `0.5 s`, so a newly connected browser does not remain on `INITIALIZING`.
- A final no-goal full graph reached `[SYSTEM] OK`, `ready=true`,
  `mission_phase=READY`, diagnostics error count `0`, and shut down cleanly.

## Operator Telemetry And Camera Contract

| Item | Verified result |
|---|---:|
| Dynamic subscriptions | GNSS `6`, proximity `11`, camera `4`, trajectory `7`, perception `9`, safety `7` versus former `32` always-on |
| View re-entry rate | GNSS `10.01 Hz`, IMU `10.01 Hz`, selected pose `20.02 Hz` after GNSS -> proximity -> GNSS |
| Browser layout | Six actual `1600x1000` captures; document/workspace/text overflow `0` |
| AMD64 backend idle | CPU `7.58%`, PSS `79.32 MiB`, JSON `2.53 KiB` |
| AMD64 largest view | Perception CPU `14.45%`, PSS `80.00 MiB`, JSON `36.14 KiB` |
| Camera contract | Front/rear raw target `10 Hz`; rear monitoring JPEG `2 Hz`; ordinary sim has no camera publisher and reports `NO FRAME` |

The CPU/PSS data measures only the UI backend on an i5-12400F workstation at
2 Hz API polling. It validates bounded implementation behavior, not the final
ARM64 8-core/16-GB deployment. The last physical record remains front
`9.167 Hz` PASS and rear raw `3.633 Hz` FAIL; Jetson frame pacing and rear-rate
acceptance remain open.

## Runtime Shutdown

The Nav2 lifecycle manager remains standalone because its Humble bond/private
executor path assumes the default ROS context. A composition experiment was
rejected after planner bond activation timed out. The standalone executable now
releases the node and ROS entities before process-exit containment. Shutdown
selects transitions from each node's observed state, and startup/reset/shutdown
cancel obsolete bond-respawn timers. This avoids both the shutdown-only DSO/DDS
race and the former invalid-transition/next-run-active race.
Three fresh controlled AMD64 full-graph runs reached `[SYSTEM] OK`; every
component and standalone process exited cleanly with no `-11`, `-9`, forced
kill, or descendant. A post-fix no-goal rerun additionally reached UI READY and
SYSTEM OK, then cleanly finished 44/44 processes after parent-only SIGINT. Its
raw log and structured assertions are preserved under
`docs/assets/module-guides/runtime/test-results/nav2-lifecycle-shutdown-20260810/`.

## Evidence And Tool Ownership

- `docs/assets/module-guides/bringup/guide/package-technology-evidence.{png,gif}`
  summarizes all 14 package claims and their evidence limits.
- `docs/assets/module-guides/runtime/guide/scoped-component-lifecycle.{png,gif}`
  documents scoped context setup and measured AMD64 shutdown behavior.
- `camrod_bringup/scripts/visualization/` owns the maintained offline report renderers.
  Their installed executable names remain unchanged for `ros2 run` users.
- Live path, radar, and speed displays remain in `camrod_planning`,
  `camrod_sensing`, and `camrod_platform` respectively.
- `util/`, user copy files, unrelated external vendor sources, and disabled
  experiments were not reorganized. The only vendored runtime edits are the
  documented Nav2 lifecycle shutdown/bond corrections; the Lanelet2 OSM remains
  the user's active map input.

## Build And Verification

The canonical `colcon_build.sh` rebuilt 13 affected ROS packages in Release
mode, including runtime, lifecycle, control, localization, map, perception,
planning, platform, sensing, sensor-kit, system, UI, and bringup.

| Verification | Result |
|---|---:|
| Affected package CTest targets | `88/88` passed |
| Aggregate package xUnit records | `683`, errors/failures `0`, static-analysis skips `24` |
| Direct Robot/Guest UI policy/transport/telemetry tests | `48/48` passed |
| Bringup source/config/map/evidence cases | `249`, errors/failures `0` |
| Control policies | `77/77` passed |
| Isolated Nav2 lifecycle/bond/lint CTest | `10/10` passed |
| Maintained shell syntax/setup contract | passed |
| Installed offline renderers | `9/9`, executable |
| Final isolated full graph | Recorded `3/3`, plus post-fix `1/1`: SYSTEM OK/READY, 44/44 clean exits, lifecycle exit `0` |

The UI frontend built successfully. npm reports existing dependency audit
findings (`40` total); this release does not perform a breaking dependency
upgrade. The host also lacks `libsdl2-mixer-dev`, so the wrapper continued its
documented behavior of skipping `camrod_voice`; voice source was unchanged.

## Remaining Field Acceptance

The user will measure and update road width later. The active user-authored
`lanelet2_maps.osm` is map v15 SHA `689c49...1b39` (55 lanelets, 14 areas,
1592 points); named copy files remain user snapshots and are not synchronized
automatically. Jetson resource/frame pacing,
physical body and swept clearance, dual-GNSS epochs, seven radar channels,
rear-camera rate, real CAN charging, repeated service, and physical boundary
recovery remain open in [`TODOLIST.txt`](../TODOLIST.txt).
