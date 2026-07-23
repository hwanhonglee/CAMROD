# CAMROD v2.0.6 Release Notes

<!-- HH_260723 - Record the localization, routing, perception, and occupied-campsite field release. -->

Release date: 2026-07-23
Target branch: `develop`
Target remote: `hwanhonglee/CAMROD`

## Scope

- Restored contiguous robot_localization pose inputs so GNSS position and
  moving-baseline heading both reach the EKF.
- Reduced Lanelet centerline lookup work, kept throttled outputs continuous,
  and made the diagnostic compare snapped output with its live localization
  input instead of blaming system-wide CPU slowdown on snapping.
- Restricted reverse Lanelet routing to an explicit campsite-return request;
  ordinary operator goals always retain the normal one-way shortest path.
- Isolated child-launch configurations so the sensing camera ownership flag
  cannot disable the following perception include.
- Verified the front camera and TensorRT YOLO path and preserved semantic class
  labels through 2D and 3D detections.
- Added tent-based campsite occupancy, control-side entry rejection, and UI
  selection blocking for occupied sites.
- Changed the compatibility LiDAR cost-grid rasterizer to consume
  `/perception/obstacles` by default. Raw filtered LiDAR inputs remain available
  through `raw_lidar_cost_enabled`, while radar cost remains unchanged.
- Synchronized the active dual-GNSS field ports (`/dev/ttyACM0` rover output,
  `/dev/ttyUSB4` moving-base correction input), selected `JECH-RTCM32` in both
  NTRIP configuration mirrors, and provided a live GNSS status GUI for NAV-PVT,
  NAV-COV, NAV-RELPOSNED, and NavSatFix inspection.

## Localization And Diagnostics

<!-- HH_260723 - Explain why the old Lanelet warning did not prove bad GNSS coordinates. -->

`/localization/centerline_pose` is a map-helper output derived from the selected
EKF pose; it is not the source used to create `/localization/pose`. The old
Lanelet checker compared this helper output with a fixed 10 Hz target. During
the recorded warning, the system diagnostic also reported 100% CPU, so an
upstream scheduling slowdown could be mislabeled as “intermittent snapping
failure.”

The map helper now republishes its last valid snap when a nearest-lanelet search
is intentionally throttled. The checker also subscribes to
`/localization/pose`, evaluates output against `min(10 Hz, live input rate)`,
and reports a rate warning only when the helper drops input poses. EKF rate and
system CPU pressure remain visible through their owning diagnostics.

GNSS position and heading are fused as contiguous `pose0` and `pose1` inputs.
This matters because robot_localization stops discovering numbered pose inputs
at the first missing index.

## Route Direction

<!-- HH_260723 - Preserve the explicit authorization boundary for reverse campsite return. -->

Vehicle yaw alone no longer reverses an ordinary route. A reverse route is
authorized only by the typed campsite return request, consumed once for the
matching goal and latched only for replans of that same goal. This preserves the
same-lane return behavior without making unrelated destinations appear
backward.

## Camera, YOLO, And Occupancy

<!-- HH_260723 - Record the fixed launch ownership and semantic occupancy chain. -->

Each child launch now runs in a scoped launch-configuration group. The sensing
include may therefore receive `enable_front_camera=false` when the composable
camera+YOLO container owns `/dev/video0` without leaking that value into
perception and disabling `obstacle_fusion_node`.

The verified pipeline is:

```text
/dev/video0
  -> /sensing/camera/econ_front/image_rect/compressed
  -> yolov9mit
  -> /perception/camera/detections_2d
  -> obstacle_fusion_node
  -> /perception/obstacles
  -> /perception/camera_lidar/detections_3d
  -> campsite_occupancy_node
  -> /perception/camping_sites/occupancy
```

YOLO publishes the configured semantic label, including `tent`, instead of only
the TensorRT class-array index. A campsite becomes occupied after three
qualifying tent hits within two seconds and remains occupied for the configured
field-session hold. Control refuses entry or aborts an approach that becomes
occupied, and the backend/frontend reject or disable duplicate site selection.

## Dynamic Cost Profile

<!-- HH_260723 - Document the reversible raw-LiDAR cost switch. -->

The compatibility grid on `/sensing/cost_grid/lidar` now rasterizes
`/perception/obstacles` and perception markers. Direct
`/sensing/lidar/points_filtered` and legacy filtered-cloud inputs are excluded
while `raw_lidar_cost_enabled: false`. Set the switch to `true` and restart the
node to restore those raw inputs. Radar continues to publish and merge through
its independent grid.

## Verification

<!-- HH_260723 - Record the release checks performed before tagging. -->

- Camera input measured approximately 10.0 Hz in the integrated bringup path.
- `/perception/camera/detections_2d` measured approximately 3.9 Hz with the
  configured 5 Hz inference throttle.
- `obstacle_fusion_node`, `campsite_occupancy_node`, and the
  `/perception/obstacles` publisher were present in the integrated graph.
- A synthetic tent Detection3D inside a configured campsite produced the
  corresponding occupied mission key.
- Package builds, unit/lint tests, configuration mirror checks, Python syntax,
  frontend production build, and `git diff --check` are required to pass in the
  final release commit.

## Known Limits

<!-- HH_260723 - Separate verified software behavior from remaining field observations. -->

- A published YOLO detection array may legitimately contain no objects when the
  scene has no class above `min_confidence: 0.5`; topic rate and publisher
  presence are the health checks.
- `/perception/obstacles` requires live filtered LiDAR input as well as the
  camera/detection pipeline.
- CPU saturation remains a separate system fault. The Lanelet checker no longer
  misattributes a matching upstream slowdown, but sustained 100% CPU can still
  delay sensing, planning, and localization callbacks.
- The raw-LiDAR cost switch is read at node startup and requires a restart after
  changing its value.
