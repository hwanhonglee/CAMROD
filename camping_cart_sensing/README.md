# Package Work Log

<!-- HH_260109 Initialize sensing package work log. -->

## 2026-02-02 10:32
- Use `camping_cart_bringup/config/sensing/sensing_params.yaml` as the default launch param (centralized config).

## 2026-02-20 14:20
- HH_260220: Removed `sensor_calibration_broadcaster_node`; static sensor TF is now owned by `camping_cart_sensor_kit`.
- HH_260220: Updated sensing frame overrides to `camera_front_link` (camera) and `lidar_link` (LiDAR).
- HH_260220: Updated MicroStrain mount frame to `sensor_kit_base_link` and disabled duplicate mount-to-frame static TF publishing.

## 2026-01-27 17:45
- HH_260127: Remove HH tags from runtime logs and demote startup logs to DEBUG (LiDAR/camera preprocessing, calibration, velocity converter).

## 2026-01-19
- HH_260119: Review README ordering/architecture; no functional changes (doc only).

## 2026-01-14 14:45
- HH_260114: Set maintainer email to hwanhong57@gmail.com and license to Apache-2.0.

## 2026-01-12 12:45
- HH_260112: Rename sensing nodes to short names for namespace-friendly defaults.

## 2026-01-12 10:40
- HH_260112: Namespace sensing nodes under /sensing and align parameter keys to match.

## 2026-01-09 20:30
- HH_260109: Keep sensing for raw/preprocessed data only (LiDAR/camera preprocessing + calibration).
- HH_260109: Add platform velocity converter for twist_with_covariance output.

## 2026-01-09 18:37
- HH_260109: Move sensing topics under /sensing prefix.

## 2026-01-09 16:48
- HH_260109: Add robust center access for Pose2D variants with position fields.

## 2026-01-09 16:40
- HH_260109: Fix Detection2D center access to handle pointer/value variants.

## 2026-01-09 15:46
- HH_260109: Add LiDAR preprocessing node and camera-guided obstacle fusion node.
