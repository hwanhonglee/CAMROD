# Package Work Log

<!-- HH_260109 Initialize localization work log. -->

## 2026-02-03 10:20
- 2026-02-03: Reinitialize ESKF from GNSS when drift exceeds threshold (reinit_on_gnss_reject / reinit_distance_threshold).

## 2026-02-09 15:50
- 2026-02-09: Add `kimera_csv_bridge_node` to import Kimera `traj_abs.csv` / `traj_local.csv` into ROS topics.
- 2026-02-09: Place bridge source in `external/kimera_vio_bridge/kimera_csv_bridge_node.cpp` (externalized module layout).
- 2026-02-09: Add launch args `kimera_bridge_enable` and `kimera_bridge_param_file` in `launch/localization.launch.py`.
- 2026-02-09: Add default bridge config `config/kimera_bridge.yaml` with fallback topics:
  `/localization/kimera_vio/pose`, `/localization/kimera_vio/pose_with_covariance`,
  `/localization/kimera_vio/odometry`, `/localization/kimera_vio/local_pose`, `/localization/kimera_vio/local_odometry`.
- 2026-02-09: Bringup override config added at `camping_cart_bringup/config/localization/kimera_bridge.yaml`.
- 2026-02-09: Add `localization_pose_selector_node` for automatic ESKF(primary)/Kimera(fallback) switching into canonical `/localization/pose*` and `/localization/odometry/filtered`.
- 2026-02-09: Extend supervisor GNSS instability checks (covariance trace, jump distance, update rate) for fallback gating.
- 2026-02-09: Example launch:
  `ros2 launch camping_cart_localization localization.launch.py kimera_bridge_enable:=true kimera_bridge_param_file:=/home/hong/cart_test_ws/src/camping_cart_bringup/config/localization/kimera_bridge.yaml`

## 2026-02-02 10:35
- 2026-02-02: Add drop zone matcher node to gate initial localization OK and publish /localization/initialpose3d.

## 2026-01-30 16:05
- 2026-01-30: Initialize ESKF state on first GNSS measurement to prevent large innovation rejection.
- 2026-01-30: Disable raw/UTM GNSS pose subscriptions by default (enable only when configured).
- 2026-01-30: Update localization map config to new_lanelet2_maps.osm origin.

## 2026-01-27 17:45
- HH_260127: Remove HH tags from runtime logs and demote startup logs to DEBUG (ESKF/navsat/bridges/snapper/supervisor).

## 2026-01-26 16:15
- HH_260126: Removed OpenVINS configs/docs; planning to vendor Kimera-VIO under localization/external (ESKF remains default, EKF optional).

## 2026-01-25 14:25
- HH_260125: ESKF publishes pose/pose_with_covariance with map frame headers (map->odom TF still identity) to keep cost grids/nav2 inputs aligned without frame drift.

## 2026-01-23 13:07
- HH_260123: Add ESKF node (IMU prediction + GNSS gate + wheel assist) with diagnostics and TF output.
- HH_260123: Add localization supervisor (mode/confidence/health topics) and wheel odom bridge for /platform/status/wheel.
- HH_260123: Add localization status/diagnostics messages and launch toggle (use_eskf) with new configs.
- HH_260123: Merge navsat pose + covariance publishing (pose_cov_bridge removed from launch; navsat_to_pose publishes PoseWithCovariance directly).

## 2026-01-19
- HH_260119: Review README ordering/architecture; note random_centerline_pose_node removal reflected; documentation-only update.

## 2026-01-14 14:30
- HH_260114: Translate localization comments/logs to English and remove Korean strings.
- HH_260114: Add HH_260114 parameter comments for centerline snapper z handling and ENU projection notes.
- HH_260114: Remove unused random_centerline_pose_node from build/install.

## 2026-01-12 12:45
- HH_260112: Rename localization helper nodes to short defaults for namespaced execution.
- HH_260109: Scope EKF parameters to /localization namespace for consistent loading.
- HH_260109: Switch EKF configs to node-agnostic format to avoid namespace mismatch.

## 2026-01-12 10:40
- HH_260112: Namespace localization nodes under /localization and align health monitor parameters.

## 2026-01-11 19:10
- HH_260109: Fix robot_localization message_filters subscriber constructor and sync order for ROS2 Humble.
- HH_260109: Remove NodeOptions clock_type usage for Humble compatibility in robot_localization nodes.

## 2026-01-09 20:10
- HH_260109: Move robot_localization vendor path to camping_cart_localization/external per request.

## 2026-01-09 19:52
- HH_260109: Vendor robot_localization under camping_cart_localization/third_party.

## 2026-01-09 19:45
- HH_260109: Add EKF fusion explanation document and note sensor weighting behavior.
- HH_260109: Add robot_localization source into workspace for local build.

## 2026-01-09 19:22
- HH_260109: Remove legacy GNSS+IMU localizer and A/B fallback nodes.

## 2026-01-09 18:37
- HH_260109: Update localization topics to /sensing, /localization, and /platform prefixes.

## 2026-01-09 15:46
- HH_260109: Add EKF fusion configuration (GNSS + IMU + Wheel + VIO + lanelet constraint).
- HH_260109: Add odometry-to-pose bridge and lanelet constraint defaults.
