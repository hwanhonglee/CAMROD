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

<!-- HH_260804 - Document the axle-midpoint platform contract. -->
## Canonical Robot Frame

Ranger odometry, normalized platform status, dummy feedback, and platform
visualization now use `robot_center_link` at the front/rear axle midpoint.
`robot_base_link` remains a static child 0.443 m behind it for legacy consumers.
The physical 1.49160 x 1.07000 m body and 0.10 m-per-side planning margin are
unchanged; only frame-relative X coordinates changed. The visualization and
control gate publish/check the same 1.69160 x 1.27000 m planning boundary.

The production boundary-recovery controller may request pure crab or reverse,
but every command still reaches Ranger only through `/control/cmd_vel_ros`
after the gate's footprint, obstacle, engage, e-stop, CAN, charging, battery,
freshness, and bounded-travel checks. Recovery intentionally holds yaw at zero
until the route is clear; normal dual-Ackermann yaw resumes with RPP afterward.
The center-frame boundary sweep and risk map are preserved in
[the frame migration ledger](../docs/ROBOT_CENTER_LINK_MIGRATION.md).

## Launch

<!-- HH_260721 - Document the real-hardware CAN prerequisite separately from simulation. -->
For real Ranger hardware, first confirm that the powered SocketCAN adapter has
created `can0`:

```bash
ip link show can0
```

Install the boot-time CAN setup once. This avoids requiring an interactive sudo
credential inside `ros2 launch`:

```bash
sudo /home/hong/camrod_ws/src/camrod_platform/scripts/install_can0_service.sh
systemctl --no-pager --full status camrod-can0.service
```

Then start the platform normally:

```bash
ros2 launch camrod_platform platform.launch.py
```

If `can0` does not exist, connect or power the CAN adapter before using real
hardware mode. Use `sim:=true` from `camrod_bringup` only for a non-hardware run.

### Disabled Ranger driver

<!-- HH_260729 - Keep a deliberate CAN disable distinct from a crashed driver,
while preserving a fail-closed platform state. -->

With `ranger_driver_enable:=false` and `ranger_dummy_when_disabled:=true`,
`platform_dummy_publisher.py` opens no CAN device and publishes the raw Ranger
topic schema at 5 Hz. Odometry and actuator velocities are zero with high
uncertainty, battery data is unknown/not-present, and system state is forced to
RC + ESTOP with an explicit error code. `/platform/dummy_active=true` makes
wheel/velocity diagnostics report DUMMY/WARN. The normalized platform status
therefore remains non-drivable and cannot be mistaken for healthy CAN.

The real Ranger node and dummy are mutually exclusive. `platform_type:=rmp401`
and top-level `sim:=true` suppress the Ranger dummy because their simulator owns
the platform topics.

Ranger driver command input is intentionally the only standard `Twist` boundary in this package.

## Steering-mode transition rate

<!-- HH_260727 - Document the field-adjustable longitudinal/lateral wheel transition. -->

`ranger_base_node.steering_transition_rate_radps` limits the wheel-angle
transition used by dual-Ackermann and parallel steering. The current
conservative field default is `0.25 rad/s` (approximately 6.3 s for a
0-to-90-degree transition); accepted runtime values are `0.05` through
`2.0 rad/s`.

It can be changed without restarting:

```bash
ros2 param set /ranger_base_node steering_transition_rate_radps 0.25
```

The same value is available from the operator UI's platform-tuning slider and
is applied through the Ranger node's standard dynamic-parameter service.

<!-- HH_260729 - Prevent translational motion from outrunning rate-limited wheel direction. -->
Ranger now also scales commanded translational velocity from the error between
the target steering angle and the rate-limited angle actually sent to the SDK.
With the deployment defaults, error at or below `0.05 rad` keeps full velocity,
error at or above `0.35 rad` commands zero translation, and the interval is
linear. `steering_transition_min_velocity_scale` defaults to `0.0`.

The same envelope applies to dual-Ackermann and parallel steering. Parallel
steering sign and hardware-angle clamp are finalized before rate limiting, so a
longitudinal-to-lateral sign change cannot bypass the slew limit. All four
parameters are dynamically validated:

```bash
ros2 param set /ranger_base_node steering_transition_velocity_scale_enabled true
ros2 param set /ranger_base_node steering_transition_full_speed_error_rad 0.05
ros2 param set /ranger_base_node steering_transition_stop_error_rad 0.35
ros2 param set /ranger_base_node steering_transition_min_velocity_scale 0.0
```

<!-- HH_260801 / TODOLIST 13 - Keep unthrottled controller-to-wheel evidence. -->
Every incoming command also produces a timestamped
`ranger_msgs/msg/SteeringTransitionState` on
`/platform/steering_transition_state`. It records the selected motion mode,
original Twist, target and rate-limited steering angles, signed steering
error, translation scale, and signed speed sent to the Ranger SDK. Compare it
with `/actuator_state` in the recovery bag to separate controller target
reversals from driver slew limiting and physical wheel lag; `/rosout` remains
operator feedback, not the measurement source.

<!-- HH_260731 - Record the real crab-yaw corrections at the Ranger boundary. -->
Parallel XY commands now use `atan2` and an explicit signed-speed
representation for all four quadrants. Pure left/right crab no longer depends
on whether the preceding command was forward or reverse. Odometry uses the
motion mode from the same CAN feedback snapshot, publishes measured parallel
`angular_velocity`, and carries non-zero configurable velocity covariance
(`0.05 m/s` linear, `0.10 rad/s` angular by default). The eight actuator states
also expose each drive speed and steering angle instead of repeating
speed-1/angle-5, so a single wheel that lags its crab angle is visible.

The production localization EKF fuses Ranger `vx/vy/wz` with non-zero measured
covariance alongside IMU `angular_velocity.z`; neither source is treated as
mathematically exact. Dual-GNSS remains the absolute yaw correction.

## Indicator MCU serial servicing

<!-- HH_260728 - Document the WS2815 refresh/UART starvation correction. -->
The external light MCU refreshes both WS2815 strips only when the computed
left/right illuminated state changes. Repeated `FastLED.show()` calls disable
interrupts long enough to starve the two-byte UART RX buffer at 115200 baud;
state-change-only refresh preserves incoming light commands while keeping both
sides blink-synchronized.
