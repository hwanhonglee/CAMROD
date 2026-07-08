---
name: project-overview
description: What CAMROD is — ROS 2 Humble autonomous camping delivery robot workspace
metadata: 
  node_type: memory
  type: project
  originSessionId: 874cb916-2ddd-4e00-a4fe-2c0616836bfd
---

CAMROD = "Autonomous Camping Delivery Robot". A **ROS 2 Humble** workspace (Ubuntu 22.04) for a supervised-autonomy outdoor delivery robot built on the **Agilex Ranger** 4WD skid-steer base (CAN bus). It navigates pre-surveyed **Lanelet2** maps of campgrounds/parks, delivers goods to named camping sites, then returns to a fixed "drop zone".

Mission loop: deliver → dwell → recall → return. Operator drives it via a web UI (site dropdown) or RViz 2D Nav Goal.

Core capabilities: point-to-point Nav2 navigation on Lanelet2 maps; multi-sensor obstacle detection (LiDAR + camera + mmWave radar); GNSS/RTK localization with IMU + wheel dead-reckoning fallback; AprilTag docking; FastAPI+React operator UI (port 8010).

Key hardware: Agilex Ranger base, SparkFun ZED-F9P GNSS + ArduSimple simpleRTK2B dual-antenna heading, Microstrain CV7/GQ7 IMU, Vanjee 3D LiDAR, dual ECON ISX031 cameras (front compressed / rear raw AprilTag), SEN0592 7-channel near-range radar, Jetson Orin field target (also runs x86_64).

Repo root: `/home/avg/camrod_develop`. The actual git repo + all source lives in `src/` (upstream: github.com/hwanhonglee/CAMROD). Current release **v2.0.0** (2026-07-06 field safety/tuning baseline). See [[workspace-layout]], [[safety-critical-path]], [[build-and-run]], [[field-baseline-and-conventions]].
