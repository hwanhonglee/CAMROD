# Package Work Log

<!-- HH_260109 Initialize perception package work log. -->

## 2026-02-02 10:32
- Use `camping_cart_bringup/config/perception/perception_params.yaml` as the default launch param (centralized config).

## 2026-02-20 14:20
- HH_260220: Default camera TF frame changed to `camera_front_link` to match sensor kit URDF static TF ownership.

## 2026-01-27 17:45
- HH_260127: Remove HH tags from runtime logs and demote startup logs to DEBUG (obstacle fusion).

## 2026-01-19
- HH_260119: Review README ordering/architecture; no change to perception pipeline (doc only).

## 2026-01-14 14:45
- HH_260114: Set maintainer email to hwanhong57@gmail.com and license to Apache-2.0.

## 2026-01-12 12:45
- HH_260112: Rename obstacle fusion node to a short default name for namespaced runs.

## 2026-01-12 10:40
- HH_260112: Namespace perception nodes under /perception and align parameter keys to match.

## 2026-01-09 20:30
- HH_260109: Add obstacle fusion node (camera detections + LiDAR) and perception launch/config.

## 2026-01-09 15:46
- HH_260109: Initialized work log; perception implementation pending.
