# camrod_sensor_kit

<!-- HH_260806 - Distinguish the provisional reduced collision boundary from
the archived surveyed envelope and unchanged sensor mount coordinates. -->

Canonical robot geometry, URDF/xacro, static sensor TF, and the RobotParams
library shared by localization, planning, control, platform, and diagnostics.

![Reference frame before and after](../docs/assets/module-guides/sensor-kit/reference-frame-before-after.png)

## Actual Simulation Runtime

![Live sensor TF geometry](../docs/assets/module-guides/sensor-kit/runtime-sensor-tf-20260804.png)

`2026-08-04 SIM RUNTIME CAPTURE`: the then-running 3D sensor TF graph. It is
historical runtime evidence; the source-derived tables and diagrams below are
regenerated from the current v2.1.5 config, including centered `gnss_link`.

## At A Glance

| Uses | Function | Main outputs |
|---|---|---|
| `robot_params.yaml` | Single source for provisional body, wheelbase, margin, and sensor mounts | RobotParams API and xacro properties |
| URDF/xacro + `robot_state_publisher` | Publishes body, axle, wheel, and sensor frame tree | `/robot_description`, `/tf_static` |
| Axle-midpoint reference | Keeps Dual-Ackermann, crab, zero-turn, EKF, and footprint geometry consistent | `robot_center_link` |

## Reference Frame Migration

| Item | Before | Current |
|---|---|---|
| Runtime origin | rear axle `robot_base_link` | axle midpoint `robot_center_link` |
| Coordinate conversion | rear-axle X | non-GNSS mounts use `current X = previous X - 0.443 m` |
| GNSS position reference | converted placeholder `-0.443 m` | temporary robot-center assumption `0.000 m` |
| Compatibility | primary frame | fixed child retained at rear axle |
| Physical mounts | unchanged | unchanged; collision boundary is a separate provisional candidate |

`robot_base_link` remains available for compatibility. It is no longer the
navigation/control reference.

## Robot Geometry

| Value | Current |
|---|---:|
| Wheelbase | `0.886 m` |
| Rear axle to center | `0.443 m` |
| Provisional body boundary | `1.29160 x 0.87000 x 1.09463 m` |
| Body extents from center | front `0.65837`, rear `0.63323`, left `0.43505`, right `0.43495 m` |
| Planning margin | front/rear/left/right `0.05 m` |
| Planning rectangle | `1.39160 x 0.97000 m` |
| Wheel radius | `0.15275 m` |

The previous `1.49160 x 1.07000 m` envelope and the exact old/new conversion
are retained in the [2026-08-06 boundary adjustment record](../docs/assets/test_result/robot-boundary-adjustment-20260806/README.md).
The reduced values are not a completed physical survey. Wheels, body corners,
sensor housings, brackets, and carried cargo must all fit inside the confirmed
field envelope before real-robot acceptance.

The two rectangles have different runtime meanings. A cost-100 cell inside
the provisional body is an unrecoverable hard stop. Contact only in the outer
`0.05 m` planning margin stops the ordinary command but may admit a separately
projected, speed/distance/time-bounded escape. The [fresh simulation summary](../docs/assets/test_result/robot-boundary-adjustment-20260806/02-runtime-boundary-policy.png)
shows both cases; it does not validate the reduced physical dimensions.

## Physical Side View

![Sensor mount side view](../docs/assets/module-guides/sensor-kit/sensor-mount-side-view.png)

## Sensor Mounts

All XYZ values are metres from `sensor_kit_base_link`, coincident with
`robot_center_link`. Roll/pitch are zero for the entries below.

| Sensor | X | Y | Z | Yaw |
|---|---:|---:|---:|---:|
| IMU | `+0.68800` | `0.00000` | `0.75600` | `0 deg` |
| GNSS center assumption* | `0.00000` | `0.00000` | `0.00000` | `0 deg` |
| LiDAR | `+0.76336` | `0.00000` | `0.59538` | `0 deg` |
| Front camera | `+0.76337` | `0.00000` | `0.49568` | `0 deg` |
| Rear camera | `-0.61933` | `0.00000` | `0.30013` | `180 deg` |
| Radar front 1 / 2 | `+0.62787` | `-0.11005 / +0.11005` | `0.33378` | `0 deg` |
| Radar left 1 / 2 | `+0.29188 / -0.28334` | `+0.41005` | `0.29013` | `90 deg` |
| Radar right 1 / 2 | `+0.29188 / -0.28334` | `-0.41005` | `0.29013` | `-90 deg` |
| Radar rear | `-0.61733` | `0.00000` | `0.33978` | `180 deg` |

![Sensor X conversion table](../docs/assets/module-guides/sensor-kit/sensor-x-before-after.png)

The GNSS position solution is treated as `robot_center_link` because the
localization adapter currently applies no antenna lever-arm correction. This
removes the former TF/localization mismatch, but it is still an assumption:
`pose_verified=false` remains until both dual-GNSS antenna mounts are surveyed.

## Measured A/B Simulation

![Rear-axle versus robot-center drive](../docs/assets/module-guides/sensor-kit/rear-axle-vs-robot-center-drive.gif)

| Common route metric | Rear-axle origin | Center origin | Delta |
|---|---:|---:|---:|
| Cross-track RMS | `0.0588 m` | `0.0549 m` | `-0.0039 m` |
| Cross-track p95 | `0.1215 m` | `0.1212 m` | `-0.0003 m` |
| Yaw-error RMS | `2.901 deg` | `2.713 deg` | `-0.188 deg` |
| Yaw-error p95 | `5.209 deg` | `5.153 deg` | `-0.056 deg` |
| Yaw-step sign reversals | `0` | `0` | `0` |

The center frame improved the compared segment and advanced `0.8566 m` farther
before the first boundary hold in the full runs. Both runs still encountered a
narrow mapped boundary, so this is not an all-route or physical-drive PASS.

## Frame Tree

```text
robot_center_link
  |-- robot_base_link              (rear-axle compatibility child, x=-0.443)
  |-- sensor_kit_base_link         (coincident)
  |   |-- imu_link / gnss_link / lidar_link
  |   |-- camera_front / camera_rear
  |   `-- radar_*_link
  |-- front_axle_link              (x=+0.443)
  `-- rear_axle_link               (x=-0.443)
```

## Run And Validate

```bash
ros2 launch camrod_sensor_kit sensor_kit.launch.py

ros2 run tf2_ros tf2_echo robot_center_link lidar_link
ros2 run tf2_ros tf2_echo robot_center_link gnss_link
ros2 run tf2_ros tf2_echo robot_center_link camera_rear
ros2 run tf2_ros tf2_echo robot_center_link robot_base_link
```

| Source | Purpose |
|---|---|
| `config/robot_params.yaml` | Canonical geometry and mounts |
| `urdf/camrod_sensor_kit.xacro` | Frame/joint/visual/collision model |
| RobotParams library | Typed access for C++ consumers |

Exact migration details are in the
[`robot_center_link` ledger](../docs/V2_1_3_ROBOT_CENTER_MIGRATION.md).
