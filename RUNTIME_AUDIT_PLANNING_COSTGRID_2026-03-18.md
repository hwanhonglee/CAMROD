# CAMROD Runtime Audit (Planning + Cost Grid)
Date: 2026-03-18

## 1) Nav2 Lanelet Plugin Reflection Check

### Result
Cost-grid layers are reflected in Nav2 runtime, with one intentional exception.

### Local costmap (`/planning/local_costmap`)
Configured plugin chain:
1. `lanelet_cost_layer_base`
2. `lanelet_cost_layer_local_path`
3. `lidar_cost_layer`
4. `radar_cost_layer`
5. `inflation_layer`

Source files:
- `camrod_bringup/config/planning/nav2_base.yaml`
- `camrod_bringup/config/planning/nav2_lanelet_overlay.yaml`

### Global costmap (`/planning/global_costmap`)
Configured plugin chain:
1. `lanelet_cost_layer_base`
2. `lidar_cost_layer`
3. `radar_cost_layer`
4. `inflation_layer`

Intentional design:
- `lanelet_cost_layer_global_path` is **not** injected into global planner master.
- Reason: avoid circular dependency  
  `planner -> global_path -> global_path_cost_grid -> planner master`.

Source files:
- `camrod_bringup/config/planning/nav2_base.yaml`
- `camrod_bringup/config/planning/nav2_lanelet_overlay.yaml`

## 2) Where to Tune When Path Generation Is Poor

## A. If planner fails with `no valid path` or `start in lethal space`
Primary tuning points:
- `camrod_map/launch/map.launch.py` (planning-base grid policy)
  - `centerline_half_width`
  - `centerline_lanelet_fill_value`
  - `outside_value`
  - `resolution`
- `camrod_bringup/config/planning/nav2_lanelet_overlay.yaml`
  - `lanelet_cost_layer_base.lethal_threshold`
  - `unknown_value`
  - `write_unknown`
- `camrod_bringup/config/planning/nav2_lanelet_overlay.yaml` (Smac2D)
  - `cost_penalty`
  - `tolerance`
  - `angle_quantization_bins`
  - `smoother.w_data`, `smoother.w_smooth`

## B. If path leaves lane center too aggressively
Primary tuning points:
- `camrod_bringup/config/planning/path_cost_grids.yaml`
  - `path_use_lateral_gradient: true`
  - `path_lateral_cost_weight` (increase)
  - `path_pose_cost_weight` (decrease)
  - `path_lanelet_match_max_dist`
  - `path_lanelet_sample_step`
- `camrod_bringup/config/planning/nav2_lanelet_overlay.yaml`
  - `Smac2D.cost_penalty` (increase for stronger low-cost preference)

## C. If global/local cost markers blink or stop after new goals
Primary tuning points:
- `camrod_planning/launch/path_cost_grids.launch.py`
  - marker nodes: `min_publish_period_sec=0.0`, `republish_period_sec=0.0`
- `camrod_bringup/config/planning/path_cost_grids.yaml`
  - `ignore_empty_path: true`
  - `clear_path_on_goal` (global: false, local: true)
  - `drop_stale_path_after_goal`
  - `rebuild_on_timer`, `min_rebuild_period_sec`

## 3) Cost Grid Color/Value Mapping

Color conversion is implemented in:
- `camrod_map/src/cost_field_marker_node.cpp` (`colorFromValue`)

Normalization:
- `norm = clamp((value - min_value) / (max_value - min_value), 0..1)`
- `value < 0` is treated as unknown

Unknown color:
- pastel palettes: light gray (`~0.85, 0.85, 0.88`)
- safety palette: dark gray (`~0.2, 0.2, 0.2`)

### Active marker palettes by topic
- `/map/cost_grid/lanelet_markers`: `pastel_blue_red`
- `/map/cost_grid/lidar_markers`: `pastel_green_red`
- `/map/cost_grid/radar_markers`: `pastel_orange_red`
- `/planning/cost_grid/global_path_markers`: `pastel_purple_red`
- `/planning/cost_grid/local_path_markers`: `pastel_cyan_red`
- `/map/cost_grid/inflation_nav2_markers` (debug): `safety`

### Why sparse red cells appear
Typical causes:
- Local high-cost peaks from sensor grids (`lidar/radar`)
- Marker decimation (`sample_stride > 1`)
- `min_value/max_value` window compressing dynamic range

`/map/cost_grid/inflation_markers` is an aggregated stream from contributor markers, so colors come from each contributor topic (not a single recolor pass).

## 4) Parameter Matching / Source-of-Truth Audit

## Runtime source-of-truth
- `camrod_bringup/config/*` is the runtime default authority.

## Synced during this audit
- `camrod_planning/config/path_cost_grids.yaml` <= `camrod_bringup/config/planning/path_cost_grids.yaml`
- `camrod_map/config/lanelet_cost_grid.yaml` <= `camrod_bringup/config/map/lanelet_cost_grid.yaml`
- `camrod_perception/config/perception_params.yaml` <= `camrod_bringup/config/perception/perception_params.yaml`

## Intentional non-strict sync
- `camrod_planning/config/nav2_lanelet.yaml`
  - legacy reference file (runtime uses layered overlays)
- `camrod_localization/config/kimera_bridge.yaml`
  - environment/site-specific CSV paths may intentionally differ from bringup defaults

## Sync checker command
```bash
bash ~/camrod_ws/src/camrod_bringup/scripts/check_config_sync.sh
```

## 5) Node-Level Change Notes (Current Focus)

- `camrod_map::LaneletCostLayer`  
  Layer fusion behavior controlled by: `lethal_threshold`, `unknown_value`, `write_unknown`.
- `lanelet_cost_grid_node`  
  Core path/lanelet raster logic and realtime rebuild policy.
- `cost_field_marker_node`  
  Value-to-color mapping and marker publish behavior.
- `marker_array_aggregator_node`  
  Combines contributor marker streams into final inflation marker view.
- `planning/nav2_lanelet.launch.py`  
  Layered Nav2 parameter chain and fixed `robot_base_link` overrides.

