# Package Work Log

<!-- HH_260109 Initialize visualizer package work log. -->

## 2026-02-04 16:05
- Add `publish_tf` parameter to robot_visualization; default off to avoid duplicate map->base_link TF.

## 2026-02-02 10:32
- `visualizer.launch.py` defaults now use bringup config for visualizer/robot/map_info.

## 2026-02-02 11:10
- Cost field marker defaults now visualize combined inflation costmap output.

## 2026-01-27 17:45
- HH_260127: Remove HH tags from runtime logs and demote startup logs to DEBUG (robot_visualization, cost_field_marker).

## 2026-01-19
- HH_260119: Review README ordering/architecture; no visualizer behavior changes (doc only).

## 2026-01-14 14:45
- HH_260114: Set maintainer email to hwanhong57@gmail.com and license to Apache-2.0.

## 2026-01-12 12:45
- HH_260112: Rename visualizer nodes to short defaults for namespaced execution.

## 2026-01-12 10:40
- HH_260112: Namespace visualizer nodes under /visualizer and align parameter keys to match.

## 2026-01-09 21:45
- HH_260109: Drop ament_target_dependencies on sensor kit to avoid legacy link flags.

## 2026-01-09 21:25
- HH_260109: Link visualizer against namespaced sensor kit target.

## 2026-01-09 20:50
- HH_260109: Drop bringup dependency to avoid circular build ordering.

## 2026-01-09 20:20
- HH_260109: Link visualizer to shared robot params library in sensor kit.

## 2026-01-09 18:37
- HH_260109: Standardize visualizer output topics under /visualizer prefix.

## 2026-01-09 17:55
- HH_260109: Create camping_cart_visualizer and move visualization nodes here.
