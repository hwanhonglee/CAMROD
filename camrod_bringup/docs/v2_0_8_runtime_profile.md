# v2.0.8 Runtime Profile and Navigation Validation

<!-- HH_260727 - Preserve the measured real-bringup load and the exact safety state used for v2.0.8. -->

This report records the 2026-07-27 Jetson Orin NX 16 GB validation performed
with the production entry point:

```bash
ros2 launch camrod_bringup bringup.launch.py \
  sim:=false parking_method:=reverse \
  control_cmd_vel_gate_allow_on_start:=false
```

The command gate remained closed throughout the hardware test. Raw Nav2
commands were observed, but `/control/command_enabled=false` and the final
`/control/cmd_vel_ros` output remained zero, so the Ranger was not driven.

## Planning and command-chain result

The live localization pose was approximately `(51.9, -52.3)`, about 31.75 m
from the nearest drivable map lane and closest to crosswalk lanelet 1413. A
single ordinary `NavigateToPose` request could therefore not form a valid
production Lanelet route. This is a field localization/map-alignment issue,
not evidence of a planner or controller failure.

The two stages were consequently verified independently while keeping the
hardware output disarmed:

- `ComputePathToPose` to nearby road lanelet 2305 succeeded in 90.9 ms and
  published a 31-pose, approximately 6 m `/planning/global_path`.
- A safe 2 m local `FollowPath` request was accepted. RPP produced 291 raw
  `/control/nav2_cmd_vel_ros` samples, 290 of them nonzero, with a maximum
  absolute component of 1.4.
- After the CPU-related changes, a repeated 1 m `FollowPath` request was
  accepted and produced 122 raw samples, 121 nonzero, again with maximum 1.4.
- In both runs every final `/control/cmd_vel_ros` sample was zero because the
  gate was deliberately disarmed.

These measurements prove the planner and controller execution paths under
full `bringup.launch.py`, but they are not represented as a single
Goal-to-motion success. A true end-to-end hardware goal requires correcting
the localization/map offset first.

## CPU and GPU observations

Linux process CPU percentages below use one logical CPU as 100%; the Jetson
has eight logical CPUs. The machine percentage is the process sum divided by
eight.

| Condition | System CPU | ROS process sum | Machine-equivalent ROS |
|---|---:|---:|---:|
| Full stack with active planning/control | 97.6% | 545.4% | 68.2% |
| Stable stack before UI/status throttling | 96.7% | 523.1% | 65.4% |
| Stable stack after UI/status throttling | 88.3% | 507.9% | 63.5% |

The last comparison also had Brave closed, so the 8.4 percentage-point system
drop must not be attributed entirely to the code changes. In the comparable
ROS process list, `ui_backend` fell from 19.1% to 8.0%.

The initial high-load process sample was:

| Component | One-core CPU |
|---|---:|
| Front camera + YOLO container | 42.8% |
| Ground segmentation | 35.4% |
| Planner server | 32.8% |
| Rear camera | 24.6% |
| RViz | 24.1% |
| UI backend | 21.3% |
| LiDAR driver/pipeline | 20.7% |
| Controller server | 9.2% |

After the implemented throttles, a second sample showed ground segmentation
55.9%, planner server 47.7%, front camera + YOLO 41.7%, RViz 25.6%, rear
camera 24.0%, LiDAR 20.6%, UI backend 8.0%, and controller server 6.9%.
Instantaneous planner and perception values vary with route/cost-grid work, so
these figures identify priorities rather than deterministic benchmarks.

`tegrastats` showed active CPU cores commonly at 94-100%, GPU samples averaging
about 36.7%, input power around 17 W, and temperature around 59-60 °C.

## Changes applied

<!-- HH_260727 - Explain the bounded optimizations without weakening sensing or command safety. -->

- Stable `/platform/status` publication is limited to 10 Hz instead of 50 Hz.
  Safety-relevant vehicle state, control mode, error, motion mode, and charging
  transitions bypass the throttle immediately.
- Battery websocket updates are sent only when the displayed integer
  percentage changes.
- Occupancy updates are sent only when the site list changes, and no websocket
  work is scheduled when no client is connected.
- The frontend ignores identical occupancy lists.
- A GTK/WebKit operator window replaces a full local browser when desired.
  Its measured steady CPU was about 5.2% of one core with approximately
  415 MB RSS. Rendering on another device at
  `http://<robot-ip>:8010` removes that display load from the robot entirely.
- HH_260727 - Route-ID updates now rebuild only the 600×600 active-route
  sensor mask. The prior combined rebuild unnecessarily regenerated the
  independent 960×960 all-lane base and took 8.75 s and 6.99 s in the field
  log, saturating CPU during the exact interval when the mask was needed.

The live status rate after throttling was 9.309 Hz over 12 seconds, with a
maximum observed interval of 0.130 s, safely below the command gate's 0.5 s
platform-status stale limit.

## Next optimization priorities

The remaining dominant costs are perception and planning rather than the UI.
Changes to camera resolution/rate, YOLO frequency, ground segmentation, or the
global costmap size should be tested against detection distance and stopping
performance before field deployment. The current global costmap is
`240 m x 240 m` at `0.10 m` resolution (5.76 million cells), so reducing its
window or resolution remains the clearest larger planning-side experiment.
That geometry change is not enabled by default because route coverage and
safety validation are still required; the route-mask-only rebuild optimization
above removes the measured redundant work without changing planning geometry.

## Field-log follow-up

<!-- HH_260727 - Record the post-validation manual-planning and radar diagnosis. -->

A later full-bringup field log explained three messages that initially appeared
to be one failure:

- RViz goals had selected `Smac2D`; one long goal exhausted the grid-planner
  retries with `no valid path found`, while a successful straight-lane grid
  path showed raster stair-steps. Manual defaults now use `LaneletRoute`, with
  the requested final yaw retained.
- `dynamic_left_near:radar` came directly from the radar cost grid. Map
  boundary costs are not copied into that grid. The route mask was unavailable,
  so the configured fail-open policy correctly passed real radar returns
  through until a route existed.
- `Repeated ABORTED (9 in 60s)` was a diagnostic counting artifact: one
  terminal UUID remained in repeated action-status messages. UUID-based
  deduplication now reports one abort for that action.

The same session also exposed that a planner unit test was using the live ROS
topic names. Its transient route IDs caused an unrelated mask rebuild and made
the robot appear outside the test route. Those test publishers are now
isolated on test-only topics.

Post-fix validation used the real top-level launch in an isolated DDS domain:
`bringup.launch.py sim:=true` with only RViz and UI windows disabled. A manual
48.44 m route completed planning in 144 ms with 247 poses, retained the
requested -77 degree final yaw, and produced a live Nav2 command of
`linear.x=0.500`, `angular.z=0.369`. The safety gate intentionally kept the
vehicle output in standby because engage was false.
