# camrod_system

<!-- HH_260720 - Align diagnostics documentation with control and selectable parking nodes. -->

`camrod_system` observes runtime graph health and aggregates diagnostics. It
does not produce vehicle commands.

## Components

| Component | Responsibility |
|---|---|
| `system_checker` | Required node/topic/type/publisher manifest checks |
| `system_diagnostic` | Module-state summary and missing-update/error reporting |
| diagnostics checkers | Sensor, localization, planning, platform and hardware health |
| diagnostics aggregator | Stable grouped diagnostics output |

<!-- HH_260721 - Document health severity separately from normal runtime operation. -->
Diagnostic health uses `OK`, `WARN`, and `ERROR`. `WARN` means a recoverable
degraded condition that can become a failure if it persists. A ROS diagnostic
`STALE` input or a checker update timeout is normalized to `ERROR`, with the
stale source retained in the detail message. Normal preparation, standby,
waiting, driving, maneuvering, and parking remain `OK` and are reported through
`ModuleState.operating_state` or `AvgServiceState` on `/service/state`.
<!-- HH_260721 - Give normal wait and charging phases explicit service examples. -->
Examples include `WAITING_FOR_RETURN_REQUEST`, `WAITING_FOR_CHARGING`,
`CHARGING`, `DEPARTING_CHARGER`, `DEPARTING_DROP_ZONE`, and
`OPERATOR_STOPPED`; these are lifecycle states, not `WARN` or `ERROR`
conditions.
Required modules may report `STARTING + OK` during the configured 10-second
startup grace; no first diagnostic after that grace becomes `FAULT + ERROR`.

<!-- HH_260728 - Preserve physical sensor identity through aggregation. -->
## Sensor Fault Location Detail

The diagnostics registry now attaches `component_id`, `sensor_location`,
`sensor_frame`, `mount_xyz_m`, `mount_rpy_deg`, and `pose_verified` to physical
sensor statuses. These values survive STALE conversion and remain visible on
`/system/diagnostics_agg`, in UI diagnostic detail, and in the terminal
`[SYSTEM]` summary. They describe sensor-health diagnostics; an obstacle stop
from the already-merged radar cost grid remains source/region-level and is not
mislabelled as one channel when several radar disks may overlap.

The terminal summary lists every simultaneous non-OK checker rather than only
the last worst item in a module. For example, FRONT1 and LEFT2 can appear as
separate lines with `front_right` and `left_rear`, their TF frames, ranges, and
actual/expected rates. Output uses one global 24-detail-line cap across ERROR
and WARN. Live measurement changes are shown in the five-second periodic report
but cannot bypass that throttle; changed fault membership or severity still
reports immediately. This also prevents changing CPU percentages or sensor
rates in message text from recreating one-Hz multi-line log spam. Mount
coordinates come from the sensor-kit robot parameters. The GNSS mount is
deliberately reported as `unverified` with `pose_verified=false` because its
current `0,0,0` configuration has not been surveyed.

<!-- HH_260724 - Site entry hands motion ownership from Nav2 to the campsite maneuver controller. -->
During campsite entry and unload phases, Nav2 cancel/abort status is expected
because `camrod_control` owns the local maneuver. `planning_nav_status_checker`
subscribes to `/service/state` and suppresses repeated ABORTED health warnings
while the service state is `SITE_ENTRY`, `UNLOAD_WAIT`,
`WAITING_FOR_RETURN_REQUEST`, return/parking maneuver states, or
`OPERATOR_STOPPED`. A planning abort during `MOVING_TO_SITE` is still reported
as WARN/ERROR.

## Control And Parking Manifests

The control manifest requires these intuitive runtime nodes:

- `/control/cmd_vel_safety_gate`
- `/control/camping_site_maneuver_controller`
- `/control/drop_zone_maneuver_controller`

The `final_parking` alternative group accepts exactly one of:

- `/parking/reverse_parking_controller`
- `/parking/apriltag_parking`

This allows parking implementation selection without mixing crab/zero-turn
health into the parking category.

## Key Topics

| Topic | Purpose |
|---|---|
| `/system/diagnostics` | Source diagnostics |
| `/system/diagnostics_agg` | Filtered diagnostics with component/location/frame/mount metadata |
| `/system/status` | Aggregated system state |
| `/control/cmd_vel_safety_gate/status` | Command policy state |
| `/control/camping_site_maneuver_controller/status` | Campsite local maneuver state |
| `/control/drop_zone_maneuver_controller/status` | Drop-zone local maneuver state |
| `/parking/reverse_parking_controller/status` | Reverse parking state |
| `/parking/apriltag_parking/status` | AprilTag parking state when selected |

Configuration is in `config/system_checker.yaml`,
`config/system_checker_sim.yaml`, and `config/diagnostics/`.

<!-- HH_260721 - Explain event-driven map diagnostics and method-selected parking discovery. -->
`/map/cost_grid/lanelet` is event-driven by pose/path changes, so its checker
does not enforce a minimum publish frequency. It still reports a missing first
grid, stale data after 12 seconds, and excessive unknown cells.
<!-- HH_260721 - Dynamic groups now accept their first real OK result. -->
The `final_parking` category starts as a neutral dynamic group and becomes OK
when exactly one reverse or AprilTag implementation is healthy. It no longer
remains in WARN because of its discovery seed. A real parking failure is
reported separately as `parking: phase=ERROR ...`.
