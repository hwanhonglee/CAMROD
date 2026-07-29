# CAMROD v2.1.0 Release Notes

<!-- HH_260729 - Release record for fail-visible disabled-hardware contracts,
     radar cost isolation, and camera/YOLO crash containment. -->

Release date: 2026-07-29

## Scope

v2.1.0 is based directly on the published v2.0.9 release. It preserves the
source-aware goal policy, dynamic-obstacle latch, complete-footprint map
boundary guard, lightweight WebKit operator window, battery/charging mission
policy, and 90% WARN / 95% ERROR disk thresholds.

This release changes disabled-hardware behavior, sensor diagnostics, radar
hardware configuration and cost attribution, front-camera/YOLO ownership,
camera input crash containment, synchronized deployment configuration, tests,
and operator documentation.

## Disabled hardware remains explicit and fail-safe

<!-- HH_260729 - A dummy heartbeat keeps a hardware-free test observable
without converting deliberately absent hardware into an unexplained graph
failure or a false healthy state. -->

- `publish_sensor_dummies_when_disabled` is the common real-hardware policy for
  deliberately disabled front/rear cameras, GNSS, IMU, LiDAR, radar, and Ranger
  feedback. Top-level `sim:=true` forces these auxiliary publishers off because
  `fake_sensor_publisher.py` already owns the simulation schemas.
- Each replacement publishes the normal typed input contract plus a fresh
  `dummy_active=true` heartbeat. Diagnostics report `DUMMY DATA / WARN` with
  component, logical location, TF frame, topic, and mounting information. A
  false or stale marker returns immediately to ordinary missing/stale ERROR
  handling.
- GNSS dummy data uses `STATUS_NO_FIX` with non-finite LLH and unusable heading
  covariance. `localization_input_adapter_node` rejects it before geographic
  conversion, so a disabled GNSS cannot invent a pose.
- IMU dummy data is stationary with deliberately high covariance. The velocity
  converter remains present so downstream graph contracts stay inspectable.
- LiDAR dummy data is a valid empty XYZ cloud. The perception obstacle node
  publishes a `DELETEALL` heartbeat for an empty or fully filtered frame rather
  than silently retaining stale markers.
- Camera dummies use a black 1×1 raw image, a decode-tested 1×1 JPEG, and
  matching `CameraInfo`. They preserve transport only and are never diagnosed
  as working physical cameras.
- Ranger dummy feedback keeps zero motion, unknown battery state, RC mode,
  ESTOP, and an explicit error. It cannot open the physical command path.
- `system_checker` now verifies shared typed contracts and processing nodes
  instead of requiring a physical acquisition executable that is intentionally
  absent during a hardware-free test.

The implementation uses one generic sensing dummy process, a dedicated
seven-channel radar dummy process, and a dedicated fail-closed platform dummy.
Real and dummy publishers are mutually exclusive at their leaf launches.

## Radar hardware and channel mapping

The SEN0592 configuration is grouped by serial communication, physical
registers, software acceptance, ROS output, and channel wiring. All topology
and physical/software range fields are startup-only.

- Startup reads the current registers, writes only mismatched values, and uses
  exact FC03 readback as the authority. A sometimes-missing FC06 write echo
  alone no longer marks a verified setting as failed.
- `hardware_angle_levels` is `[1, 1, 1, 1, 1, 1, 1]`. Register `0x0208` is one
  combined angle/sensitivity level; it does not provide independent horizontal
  and vertical controls. Level 1 is the narrowest available setting.
- `hardware_range_levels` is `[2, 2, 2, 2, 2, 2, 1]`, corresponding to the
  near-field profile of approximately 1.5 m for front/side sensors and 0.5 m
  for rear.
- Exact software acceptance remains 1.50 m for FRONT1/FRONT2, 0.80 m for the
  four side sensors, and 0.50 m for REAR.
- `sensor_enabled[i]: false` opens no serial port and replaces only that
  channel with 2 Hz `max_range + 0.001 m` no-target and per-channel
  `dummy_active` heartbeats.
- All seven channels remain enabled in the v2.1.0 deployment profile. The
  crossed field-harness mapping remains FRONT1=USB0, FRONT2=USB1, LEFT1=USB4,
  LEFT2=USB5, RIGHT1=USB2, RIGHT2=USB3, and REAR=USB6.

GNSS and IMU parameter/port YAML files were not changed by v2.1.0.

## Radar fixed returns, dummy barrier, and stop evidence

<!-- HH_260729 - Replace index arithmetic and broad minimum-distance floors
with named, reviewable intervals and an independent disabled-source barrier. -->

The active cost-grid profile uses named inclusive
`SENSOR:min_range:max_range` fixed-return intervals:

| Radar | Excluded fixed intervals (m) |
|---|---|
| FRONT1 | 0.099–0.123; 0.152–0.220 |
| FRONT2 | 0.097–0.117 |
| LEFT1 | 0.182–0.226; 0.234–0.258 |
| LEFT2 | 0.210–0.280 |
| RIGHT1 | 0.055–0.080; 0.253–0.277 |
| RIGHT2 | 0.248–0.278 |
| REAR | 0.090–0.190 |

All other fresh, valid returns remain obstacle candidates. Automatic startup
return learning is disabled in the driving profile because an object present at
startup is indistinguishable from a body return to a scalar range sensor.
Supervised learning remains available only as an explicit stationary,
clear-area calibration followed by a restart and restoration of the disabled
field default.

Fresh global or per-channel radar dummy markers form a second barrier in
`radar_cost_grid_node`, independent of the numeric no-target convention. A
dummy-marked channel cannot paint obstacle cost. When the global marker is
fresh, `enable_radar:=false` therefore suppresses FRONT1, FRONT2, LEFT1, LEFT2,
RIGHT1, RIGHT2, and REAR together.

`/sensing/radar/obstacle_evidence` publishes `clear` or the exact fresh,
unfiltered hits that remain in the final route-clipped radar grid. Active
entries carry sensor name, source frame, range, map point, and cost. The command
gate appends this fresh provenance to radar stop logs, but occupancy-grid
evaluation remains the sole stop and release authority. Missing provenance can
never release or weaken a stop.

The v2.0.9 dynamic-obstacle latch remains unchanged: a stop-induced zero or
changed-direction command cannot clear the original hazard, and release still
requires continuously fresh clear evidence plus the configured hold.

## Camera and YOLO routing/crash containment

<!-- HH_260729 - Record both the log-confirmed failure and the source-level
hardening without claiming a post-fix physical-camera acceptance run. -->

The pre-fix 2026-07-29 logs showed the physical `/dev/video0` front camera and a
front dummy publishing together while the front camera and YOLO shared
`/camera_yolo_container`. The then-current dummy JPEG separately failed an
OpenCV 4.8 decode check. The captured core proves that
`compressed_image_callback -> cv_bridge::toCvCopy -> cv::cvtColor` received an
empty decoded image, raised an OpenCV assertion, and terminated the component
container with SIGABRT (`-6`); because both publishers shared one topic, the
core alone cannot attribute that exact frame to one publisher. The malformed
concurrent dummy is the strongest directly reproduced source.

Two independent corrections are included:

1. Bringup evaluates camera+YOLO container ownership once in the parent launch
   scope, stores the resolved value, and forwards that stable value into the
   scoped sensing include. A child-local `enable_front_camera=false` can no
   longer be reinterpreted as physical-camera disablement and start a duplicate
   front dummy.
2. The dummy publisher now carries a valid black 1×1 JPEG verified through
   OpenCV decoding. The YOLO compressed callback rejects null, empty,
   undecodable, empty-image, and unexpected-type inputs; it catches cv_bridge,
   OpenCV, standard, and unknown exceptions and emits throttled frame metadata
   instead of terminating the shared component container.

Camera diagnostics also retain the highest-priority reason so a low rear-camera
rate remains an `FPS low` result rather than being overwritten by a later
equal-severity encoding check.

## Retained map-boundary and storage safety

- The raw lanelet safety path continues to evaluate the complete
  `/platform/robot/planning_boundary`, not only `robot_base_link`.
- Footprint cost 100 remains the off-lane stop threshold. The softer rasterized
  edge is not treated as equivalent to leaving the mapped corridor.
- The dynamic-obstacle stop latch and fresh-clear release policy remain active.
- Filesystem usage remains WARN at 90% and ERROR at 95%; an 80% disk is not a
  system warning.

## Reviewed ROS log evidence

The following pre-fix full-bringup logs were reviewed:

- `/home/nvidia/.ros/log/2026-07-29-17-01-52-538414-EAC6k-Orin-174724/launch.log`
- `/home/nvidia/.ros/log/2026-07-29-16-58-15-985404-EAC6k-Orin-167240/launch.log`

In the latest `enable_radar:=false` run:

- no physical `sen0592_radar_node` was active;
- all seven radar checkers explicitly reported global+channel dummy data;
- the radar cost grid did not report active radar obstacle evidence;
- the observed command-gate stops were `lanelet_footprint_cost`, not LEFT2 or
  another radar source;
- the physical rear camera remained active, but its observed rate was below the
  configured 10 Hz target; and
- the front camera/YOLO component initialized, then hit the malformed dummy
  compressed-frame failure described above.

These observations identify the corrected source boundaries but are not a
post-fix real-robot acceptance run.

## Configuration synchronization and automated coverage

Package-owned configuration and full-bringup deployment mirrors are kept
byte-identical for:

- radar driver and radar cost grid;
- command safety gate;
- system graph manifest;
- camera, GNSS, IMU, LiDAR, radar, velocity-converter, wheel-odometry, and
  localization-GNSS diagnostic checkers.

During release documentation preparation,
`field_test_tool.sh config` reported package/bringup/source/install
configuration synchronization as OK. Regression coverage was added for:

- generic sensing, radar, and Ranger dummy message contracts and launch
  exclusivity;
- fresh/stale dummy-state diagnostic behavior;
- radar hardware arrays, named fixed-return configuration, dummy cost
  suppression, and deployment mirror equality;
- parent-to-scoped-child camera ownership resolution;
- decodeable dummy JPEG generation and malformed YOLO compressed-frame
  containment;
- GNSS dummy rejection before localization conversion;
- empty LiDAR/perception heartbeat behavior; and
- camera diagnostic severity/message retention.

Camera hardening validation completed with 13/13 sensing-dummy pytest cases,
including real OpenCV decoding of the replacement 1×1 JPEG; a successful
`camrod_system` package build and its new camera-diagnostic CTest; and a
successful `yolov9mit_ros` package build. An isolated `ROS_DOMAIN_ID=88` smoke
test injected both an empty payload and a corrupt four-byte JPEG. YOLO logged
the input/OpenCV stage and frame metadata, remained alive, then processed three
valid replacement JPEG frames before normal SIGINT shutdown.

Final integration completed without hardware:

- `camrod_sensing`, `camrod_bringup`, `camrod_platform`,
  `camrod_localization`, `camrod_perception`, `camrod_system`, and
  `camrod_control` built successfully;
- the separately discovered vendored `yolov9mit_ros` package built
  successfully;
- all 38 CTest suite entries across the seven affected CAMROD packages passed
  with zero failures; and
- `field_test_tool.sh config` reported final package/bringup/source/install
  configuration synchronization as OK.

## Physical validation status

No post-fix full real-robot bringup, engage, or driving test was performed while
finalizing v2.1.0. This tag is the synchronized source/configuration/test
baseline for the next supervised field run, not a claim that all physical
acceptance work is complete.

The outstanding radar-off soak, seven-channel clear-area measurement,
front/rear camera and YOLO rate/lifetime checks, lanelet-footprint alignment,
off-lane/reverse mission recovery, centerline lateral-control latency,
goal/path/cmd_vel driving, planning quality, CPU profiling, voice sequence,
OpenCV ABI cleanup, and final GNSS/IMU ownership checks are maintained in
[`TODOLIST.txt`](../TODOLIST.txt).
