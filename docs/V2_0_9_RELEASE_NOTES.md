# CAMROD v2.0.9 Release Notes

<!-- HH_260728 - Release record for bounded radar self-return filtering and
     direction-specific side protection. -->

Release date: 2026-07-28

## Scope

v2.0.9 is based directly on the published v2.0.8 release. It preserves the
v2.0.8 battery admission, charging departure, source-aware planning,
full-footprint map-boundary enforcement, WebKit operator window, diagnostics,
and runtime-load changes.

This release changes radar self-return handling, the normal-forward side-near
guard, sensor-location diagnostics, their synchronized deployment
configuration, tests, and operator documentation.

## Radar behavior

- The old common 0.30 m and LEFT2 0.75 m one-sided ignore floors are disabled.
  A valid return is ignored only when it lies inside a configured narrow
  `(sensor index, center, half width)` notch.
- Each radar receives a complete disengaged 8-second collection window
  beginning with that channel's first valid sample. A channel with no timely
  first sample, including one first seen at/after the absolute 15-second
  deadline, is rejected. The node may append one dominant, tight startup notch
  per sensor; it rejects insufficient, weak, or wider-than-0.03 m clusters and
  freezes accepted notches for the node lifetime.
- Startup candidates are capped per sensor at 0.20-0.30 m. Observed side
  returns around 0.68-0.80 m therefore remain obstacle costs.
- The command gate publishes transient-local `/control/planning_engaged`,
  combining manual and mission engage independently of effective cost/CAN
  holds. Calibration waits for a known false state and cancels immediately on
  true, including after a radar-node restart during an active mission. A
  malformed fixed profile disables both fixed and startup filtering so every
  valid range remains available to the obstacle grid.
- No-target heartbeats remain invalid obstacle inputs. Generated `AvgRange`
  topics are used internally and `_ros` `sensor_msgs/Range` mirrors remain
  available for RViz and ROS-native inspection.

## Sensor diagnostic detail

- The central diagnostic registry attaches a stable component ID, logical
  vehicle location, TF frame, and configured mount pose to GNSS, IMU, LiDAR,
  seven Radar channels, front/rear cameras, wheel odometry, and derived sensor
  pipelines.
- STALE conversion preserves the original checker values and mount metadata.
- `[SYSTEM]` terminal output lists every simultaneous non-OK snapshot instead
  of collapsing equal-severity sensors into one `sensing` line. Live range,
  actual/expected rate, topic, and other checker-specific values are retained.
- Detail output remains on the existing five-second periodic throttle and uses
  one global 24-line cap across ERROR and WARN. Changing live range/rate values
  or numeric message text does not bypass the throttle; fault membership or
  severity changes still report immediately. Health levels and planning stop
  policy are unchanged.
- GNSS mount position is marked `unverified` because the current configured
  `0,0,0` pose is not a surveyed physical antenna location.
- This location registry identifies sensor-health faults. A normal valid return
  that triggers `dynamic_*:radar` is already merged into one radar cost grid;
  v2.0.9 keeps that stop fail-safe and does not guess a single originating
  channel when multiple radar disks may overlap.

## Direction-specific safety

- Normal forward travel uses `body_near_side_lookahead_m: 0.75`.
- Crab and reverse retain
  `body_near_maneuver_side_lookahead_m: 1.20`.
- A farther side return therefore does not immobilize straight travel merely
  because it is inside the old 1.20 m base-centred window, but a command toward
  that side remains protected by the wider maneuver corridor.
- The v2.0.8 complete-footprint cost-100 map-boundary check is unchanged.

## Hardware-port preservation

v2.0.9 does not change IMU or GNSS port routing. The 2026-07-28 runtime udev
inventory matched the v2.0.8 configuration:

- `/dev/ttyACM0`: u-blox GNSS rover.
- MicroStrain CV7: stable Lord MicroStrain by-id path, currently enumerated as
  `/dev/ttyACM1`.
- Moving-base RTCM writer: FTDI `DN03DF8V` stable by-id path, currently
  enumerated as `/dev/ttyUSB0`.
- `/dev/ttyUSB2` through `/dev/ttyUSB5`: Quectel modem interfaces, not GNSS or
  RTCM writer ports.

Source and bringup IMU/GNSS parameter files remain byte-identical to v2.0.8.

## Validation and field observation

- Radar self-echo helper tests cover validated multi-notch construction,
  narrow matching, invalid-profile fail-safe behavior, dominant startup
  learning, weak/broad-cluster rejection, delayed per-sensor startup windows,
  disconnected-channel timeout, and late-first-packet rejection.
- Diagnostic-detail tests cover metadata upsert, STALE preservation,
  location/frame/range/rate formatting, and the no-invented-location fallback.
- Control policy tests cover a side return that is clear for normal forward
  travel but blocking for a lateral command, plus a closer forward-side stop.
- Isolated ROS integration verifies transient `/control/planning_engaged`
  remains true independently of an effective command hold and cancels radar
  learning after a subscriber restart.
- Isolated diagnostics integration verifies simultaneous FRONT1/LEFT2 detail,
  registry injection, STALE value preservation, the global line cap, and that
  changing live measurements cannot bypass the summary throttle.
- Source and bringup radar/control mirrors are checked byte-for-byte, with YAML
  parsing and whitespace validation included in the release checks.
- The selected sensing/control/system/bringup build completes and all 83
  registered tests pass with zero failures, errors, or skips.

One live bringup observation intentionally remains fail-safe for continued
operator testing. LEFT2 `0.221-0.225 m` fell just below the configured
`0.227 m` notch edge and FRONT1 `0.178-0.179 m` fell between two accepted
notches; both produced radar cost 95. RIGHT1 `0.063 m` was not the cause
because startup calibration had already learned `0.062 +/- 0.012 m`.

Those FRONT1/LEFT2 hits project outside the measured physical body but inside
the extra 0.10 m planning margin. Treating the complete planning footprint as
self-body would therefore hide real obstacles within the requested clearance
zone. The fixed notches were not widened. Instead, the zero-sample startup
failure was corrected with per-sensor timing and the operator now receives
exact sensor location detail for checker faults. Final driving acceptance, any
new fixed notch, and exact merged-cost channel provenance still require a
clear-bay, repeated-cold-boot target sweep and a separate observation-only
validation path.
