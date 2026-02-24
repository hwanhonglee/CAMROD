# Lanelet Cost Layer (HH_251231)

Nav2 costmap2d plugin that consumes an OccupancyGrid (map frame) and writes costs.

## Plugin registration
- Class: `camping_cart_map::cost_map::LaneletCostLayer`
- Export: `plugin_cost_map.xml`
- Built inside `camping_cart_map` package.

## Parameters
- `source_topic` (string, default `/map/cost_grid/lanelet`): OccupancyGrid input in `map` frame.
- `lethal_threshold` (int 0~100, default 65): values >= threshold become LETHAL_OBSTACLE.
- `unknown_value` (int 0~255, default 255): if incoming value < 0, write this (255 to mark unknown). Set to -1 to skip writing unknown.

## Nav2 example (merge into your nav2_params.yaml)
See `config/nav2_params_costlayer_example.yaml` for global/local costmap snippet.

## Topics / Frames
- Input: OccupancyGrid in `map` frame (`source_topic`).
- TF: `map -> robot_base_link` is provided by localization; layer does not require sensor frames.

## RViz check
- Add Map display: `local_costmap/costmap` or `global_costmap/costmap` to see blended result.
- (Optional) Also visualize source grids:
  - `/map/cost_grid/lanelet`
  - `/planning/cost_grid/global_path`
  - `/planning/cost_grid/local_path`

## Notes
- updateBounds/updateCosts touch only the requested window from Nav2, avoiding full-map scans.
- No marking/clearing split; this is a marking-style preference map.
