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
