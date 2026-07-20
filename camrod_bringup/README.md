# camrod_bringup

<!-- HH_260720 - Describe the current control and parking launch topology. -->

`camrod_bringup` starts the CAMROD stack in dependency order and provides the
field/simulation configuration tree.

## Launch Order

```text
platform -> sensor_kit -> map -> sensing -> perception -> localization
         -> planning -> control gate/maneuvers -> selected control parking node -> system -> UI
```

Control starts after planning because it consumes Nav2 commands and mission
state. The selected parking controller is also owned by `camrod_control` and
starts after the maneuver controllers publish the parking operation handoff.

## Common Launches

Simulation:

```bash
ros2 launch camrod_bringup bringup.launch.py sim:=true rviz:=false parking_method:=reverse
```

Real hardware:

```bash
ros2 launch camrod_bringup bringup.launch.py sim:=false parking_method:=reverse
```

`parking_method` accepts only `reverse` or `apriltag`. Exactly one parking node
is launched and checked by `camrod_system`.

## Relevant Configuration

| Path | Purpose |
|---|---|
| `config/bringup/launch_defaults.yaml` | Module enable flags and launch defaults |
| `config/control/cmd_vel_safety_gate.yaml` | Bringup mirror of command authorization and motion-safety policy |
| `config/control/control.yaml` | Bringup mirror of campsite/drop-zone maneuver tuning |
| `config/control/parking.yaml` | Bringup mirror of reverse and AprilTag parking tuning |
| `config/control/yaw_alignment_zones.yaml` | Optional command-gate yaw zones |
| `config/planning/` | Nav2 and mission-state configuration |
| `config/map/drop_zones.yaml` | Drop-zone station position and reverse-axis yaw |
| `config/platform/ranger_driver.yaml` | Ranger CAN/SDK bridge |
| `config/system/system_checker*.yaml` | Runtime graph manifests |

<!-- HH_260721 - Define the package-to-bringup control configuration mirror contract. -->
Package-owned defaults remain canonical under `camrod_control/config/`. The
four files under `camrod_bringup/config/control/` are byte-identical deployment
mirrors; bringup additionally supplies resolved map/config paths.

## Simulation Validation

The validation runner checks sensor rates, directional gate stops, manual Nav2
movement, and the complete campsite round trip.

```bash
ros2 run camrod_bringup sim_validation_runner.py --ros-args \
  -p quick:=true \
  -p run_gate_matrix:=false \
  -p skip_manual_goal:=true \
  -p run_camping:=true \
  -p camping_wait_drop_zone:=true \
  -p camping_timeout_s:=420.0 \
  -p report_file:=/tmp/camrod_control_sim_full_camping.json
```

The full camping check requires campsite crab/zero-turn/exit, return navigation,
drop-zone yaw alignment, reverse parking start, and final `PARKED` state.
