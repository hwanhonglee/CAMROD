---
name: field-baseline-and-conventions
description: "CAMROD v2.0.0 field-tuning baseline, hardware port map, and repo conventions"
metadata: 
  node_type: memory
  type: project
  originSessionId: 874cb916-2ddd-4e00-a4fe-2c0616836bfd
---

**Repo conventions:**
- Git: work on `develop` branch (also `main`). Commit messages are lowercase `area: summary` style (e.g. `planning: damp route alignment and speed campsite crab`, `bringup: expose field safety gate parameters`).
- Code/README annotations use dated tags `HH_YYMMDD - <note>` (HH = author Hwanhong Lee) to mark field-change provenance. Grep these to trace why a value was set.
- Docs are first-class: root README + per-package READMEs + `docs/templates/` style guides + `docs/DOCS_CHANGELOG.md` + `PARAMETER_NAMING_STANDARD.md`. Keep docs synced with code changes.
- `avg_msgs` is the internal message surface; use its `conversions.hpp` helpers at std-ROS boundaries rather than passing std msgs through the stack.

**Current field hardware port map (v2.0.0, may drift per robot):**
- GNSS ZED-F9P rover → `/dev/ttyACM1` (synced in camrod_sensing + camrod_bringup)
- CV7 IMU → `/dev/ttyACM0`
- Radar SEN0592 on CH9344 USB (crossed LEFT/RIGHT harness): FRONT1=USB0, FRONT2=USB1, LEFT1=USB4, LEFT2=USB5, RIGHT1=USB2, RIGHT2=USB3, REAR=USB6
- SEN0592 no-target = value near 65535 mm → filtered (heartbeat above max_range), not an obstacle
- Ranger CAN via `can0` / `setup_can0.sh`

**v2.0.0 field-tuning values (kept synced across configs):**
- cmd_vel gate route-heading guard: lookahead 2.0 m, angular gain 0.8, angular clamp 0.35 rad/s, release threshold 35°
- Live LiDAR/Radar cost-stop latch: clear-for 2.0 s before release
- Inflation grid fail-closed: stale/missing >1.0 s closes the gate
- Side radar self-echo filter 0.05 m (front/rear 0.15 m)
- Campsite crab lateral speed 0.24 m/s (raised from 0.18); `crab_timeout_speed_scale: 0.4`
- LiDAR ground filter `downsample_resolution: 0.10`; filtered obstacle stream ~6 Hz target under field load
- LiDAR cost grid 180×180 @ 0.10 m, 0.80 s input freshness; perception markers capped 1.2 m radius at cost 90
- GNSS diagnostics accept 1 Hz field-rate floor (no false ERROR_STOP)
- Diagnostics policy: map/perception/lidar/radar diag errors show in `/system/status` but no longer force planning ERROR_STOP by themselves; costmap diagnostics demoted to WARN. Filtered LiDAR/Radar cost grids + gate checks are the motion-safety authority.

Full-stack field runs with RViz/UI/voice/cameras/YOLO/docking are load probes — on Jetson Orin they can saturate CPU/GPU and delay LiDAR/cost-grid/radar diagnostics. Prefer the lighter outdoor profile for drive validation after sim checks pass. See [[project-overview]], [[safety-critical-path]], [[build-and-run]].
