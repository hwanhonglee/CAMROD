# camrod_map

Map module for Lanelet2 loading, base/planning cost grids, and marker visualization.

## Purpose
- Load Lanelet2 map and map metadata
- Generate lanelet-constrained base grids
- Generate planning-base grids used by planning overlays
- Publish visualization markers (including inflation marker streams)

## Entry Point
```bash
ros2 launch camrod_map map.launch.py
```

## Runtime Structure
```text
map.launch.py (top-level orchestrator)
  -> lanelet2_map.launch.py   (Lanelet map + static TF)
  -> cost_grid.launch.py      (single node publishes lanelet + planning_base OccupancyGrid)
  -> visualization.launch.py  (multi-grid marker publisher + aggregator + lanelet cost field)
```

## Main Launch Arguments
- `map_info_file`
- `map_path`
- `origin_lat`, `origin_lon`, `origin_alt`
- `map_param_file`
- `lanelet_cost_grid_param_file`
- `map_visualization_param_file`
- `enable_nav2_inflation_debug_marker`
- `enable_cost_grids`
- `enable_map_cost_markers`
- `enable_inflation_markers`
- `enable_cost_field` (default `false`)
- `module_namespace` (default `map`)
- `system_namespace` (default `system`)

## Key Topics
- Map and grid outputs:
  - `/map/cost_grid/lanelet`
  - `/map/cost_grid/planning_base`
  - `/map/cost_grid/inflation_markers`
- Marker topics:
  - `/map/cost_grid/lanelet_markers`
  - `/map/cost_grid/lidar_markers`
  - `/map/cost_grid/radar_markers`
- Map visualization:
  - `/map/lanelet2_map/*` (centerlines, bounds, points, areas)

## Utility Launches
- `lanelet2_map.launch.py`: map-only launcher
- `cost_grid.launch.py`: lanelet/planning base cost-grid publishers
- `visualization.launch.py`: marker and cost-field publishers
- `area_export.launch.py`: exports semantic area metadata (drop_zone, camping_site_*) from Lanelet2 map
- `map_top.launch.py` is removed. Use `map.launch.py` directly.

## Configuration
- `camrod_map/config/map_info.yaml` (shared map/localization reference source)
- `camrod_map/config/lanelet_cost_grid.yaml` (cost-grid behavior only; map/origin injected from launch)
- `camrod_map/config/map_visualization.yaml`

## StatusStream
- Module-local topic: `/map/status`
- Aggregated topic: `/status_stream`
