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

## Measured ROS Simulation

The current map-v17 B2 run contains 511 ROS pose samples over `29.700 s`:

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

Raw timeline, stable summary, geometry/map hashes, PNG, 80-frame GIF, and the
regeneration test are under
`docs/assets/test_result/tapered-rounded-boundary-road-sim-20260810/`. This is
AMD64 ROS simulation evidence, not physical clearance or actuator acceptance.

## UI Initialization

- Initialization can latch as soon as all real prerequisites are healthy, even
  if manual engage or a campsite request arrived before the final prerequisite.
- Engage without a goal does not falsely display driving.
- The backend publishes unchanged authoritative readiness/mission state every
  `0.5 s`, so a newly connected browser does not remain on `INITIALIZING`.
- A final no-goal full graph reached `[SYSTEM] OK`, `ready=true`,
  `mission_phase=READY`, diagnostics error count `0`, and shut down cleanly.

## Evidence And Tool Ownership

- `docs/assets/module-guides/bringup/package-technology-evidence.{png,gif}`
  summarizes all 14 package claims and their evidence limits.
- `docs/assets/module-guides/runtime/scoped-component-lifecycle.{png,gif}`
  documents scoped context setup and measured AMD64 shutdown behavior.
- `camrod_bringup/scripts/visualization/` owns nine offline report renderers.
  Their installed executable names remain unchanged for `ros2 run` users.
- Live path, radar, and speed displays remain in `camrod_planning`,
  `camrod_sensing`, and `camrod_platform` respectively.
- `util/`, user copy files, external vendor sources, disabled experiments, and
  the Lanelet2 OSM were intentionally not reorganized or edited.

## Build And Verification

The canonical `colcon_build.sh` rebuilt `camrod_sensor_kit`, `camrod_control`,
`camrod_platform`, `camrod_planning`, `camrod_ui`, `camrod_runtime`, and
`camrod_bringup` in Release mode.

| Verification | Result |
|---|---:|
| Affected package CTest targets | `42/42` passed |
| Direct Robot/Guest UI policy/transport tests | `32/32` passed |
| Boundary/module/service evidence pytest | `27/27` passed |
| Control policies | `71/71` passed |
| Maintained shell syntax/setup contract | passed |
| Installed offline renderers | `9/9`, executable |
| Final isolated full graph | SYSTEM OK, UI READY, clean SIGINT |

The UI frontend built successfully. npm reports existing dependency audit
findings (`40` total); this release does not perform a breaking dependency
upgrade. The host also lacks `libsdl2-mixer-dev`, so the wrapper continued its
documented behavior of skipping `camrod_voice`; voice source was unchanged.

## Remaining Field Acceptance

The user will measure and update road width later. v2.1.7 deliberately leaves
`lanelet2_maps.osm` and its named copy unchanged. Jetson resource/frame pacing,
physical body and swept clearance, dual-GNSS epochs, seven radar channels,
rear-camera rate, real CAN charging, repeated service, and physical boundary
recovery remain open in [`TODOLIST.txt`](../TODOLIST.txt).
