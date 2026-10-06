# Rear-camera / AprilTag live CPU A/B/A — 2026-10-06

Measured 14:35:26–14:36:30 KST in the live CARLA/CAMROD stack. This is an
x86_64 CARLA sensor-relay and standalone AprilTag test, not the Jetson physical
rear-camera capture/VIC/composed-container path. No motion was commanded.

CPU is process user+system time divided by elapsed wall time; 100% is one logical
CPU. The five-second image metadata subscriber was destroyed before measuring
CPU. UI telemetry requests changed only diagnostic subscriptions. The telemetry
lease was closed at the end and the final API state confirmed inactive.

| Process / PID | Closed before (15.02 s) | Docking open (20.05 s) | Closed after (15.02 s) |
|---|---:|---:|---:|
| AprilTag detector / 1149078 | 30.96% | 41.40% | 30.82% |
| CARLA sensor relay / 1148303 | 32.02% | 32.13% | 32.69% |
| UI backend / 1149289 | 27.36% | 29.83% | 27.09% |

Mean RSS MiB, in the same order: detector 104.25 / 105.16 / 105.38;
relay 94.12 / 94.32 / 94.20; UI 97.51 / 99.34 / 99.69.

Opening the docking telemetry subscription increased the detector from the two
closed-window mean of 30.89% to 41.40%, a 10.52 one-core percentage-point delta
(about 34%). The detector code enables full-frame JPEG debug encoding when this
subscription is present. This A/B/A is consistent with that code path; it is a
single short run, not a complete causal performance benchmark. No tag was detected
in the sampled docking view. This run did not reproduce 200% detector CPU.

Observed rear input: 960×720 BGRA8, frame `camera_rear`, 27 image messages over the
five-second metadata window (inter-arrival rate 5.38 Hz); CameraInfo 9.56 Hz.
Both canonical topics had exactly one publisher, `/carla_sensor_relay`.

Read-back parameters:

- Detector: `quad_decimate=1`, `n_threads=2`, `roi_full_search_interval=30`,
  `publish_debug_image=true`, `debug_jpeg_quality=80`; input canonical
  `/sensing/camera/econ_rear/image_rect` and `/camera_info`.
- Relay: input `/carla/ego_vehicle/rgb_rear/image` and `/camera_info`,
  `raw_image_max_rate_hz=10`, `compressed_image_max_rate_hz=5`,
  `publish_compressed_images=true`. These are maximum rates, not observed rates.
- UI: `telemetry_session_timeout_s=12`,
  `telemetry_camera_raw_fallback_enabled=true`,
  `telemetry_docking_rear_camera_fallback_enabled=false`.

The UI request was `POST /api/telemetry/session?active=true&view=docking`, renewed
every five seconds; close was `active=false&view=all`. No camera/control parameters
were changed. Raw data, command lines, publisher names and telemetry snapshots are
in `rear_cpu_probe_results.json`; `rear_cpu_probe.py` is the diagnostic script.
