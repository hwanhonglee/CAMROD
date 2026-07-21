# camrod_system

<!-- HH_260720 - Align diagnostics documentation with control and selectable parking nodes. -->

`camrod_system` observes runtime graph health and aggregates diagnostics. It
does not produce vehicle commands.

## Components

| Component | Responsibility |
|---|---|
| `system_checker` | Required node/topic/type/publisher manifest checks |
| `system_diagnostic` | Module-state summary and missing/stale/error reporting |
| diagnostics checkers | Sensor, localization, planning, platform and hardware health |
| diagnostics aggregator | Stable grouped diagnostics output |

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
