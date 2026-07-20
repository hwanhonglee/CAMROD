# camrod_platform

<!-- HH_260720 - Document platform as CAN/driver integration after removing the duplicate gate. -->

`camrod_platform` connects Ranger CAN/SDK interfaces to generated CAMROD status
contracts. All motion authorization policy belongs to `camrod_control`.

## Main Nodes

| Node | Responsibility |
|---|---|
| `ranger_platform_bridge` | Normalize Ranger SDK/CAN odometry, state, battery and charging feedback |
| `light_controller` | Vehicle light/indicator behavior |
| `robot_visualization` | Robot footprint and platform visualization |

## Command Path

```text
/control/cmd_vel_ros (geometry_msgs/Twist boundary)
       |
       v
Ranger driver /cmd_vel
```

The Ranger launch remaps its standard `/cmd_vel` subscription directly to
`/control/cmd_vel_ros`. `cmd_vel_safety_gate` has already applied
`/platform/drive_enable`, `/platform/status`, and all other command conditions.

## Normalized Status

| Topic | Meaning |
|---|---|
| `/platform/status` | Generated odometry, velocity, wheel, e-stop, vehicle/CAN mode, errors and BMS/charging status |
| `/platform/status/odometry` | Platform odometry |
| `/platform/status/velocity` | Generated platform velocity used by sensing |
| `/platform/status/wheel` | Generated actuator/wheel telemetry |

<!-- HH_260720 - The normalized wheel stream is produced by localization, not platform. -->
`camrod_localization` converts `/platform/status/odometry` into
`/localization/input/wheel_odometry` and its explicit `_ros` EKF boundary.

The bridge derives status from Ranger system-state and BMS feedback. A charging
state does not itself initiate motion; `camrod_control/cmd_vel_safety_gate`
decides whether a bounded campsite departure request may move the robot.

## Launch

```bash
ros2 launch camrod_platform platform.launch.py
```

Ranger driver command input is intentionally the only standard `Twist` boundary in this package.
