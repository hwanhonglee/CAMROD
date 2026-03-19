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
lanelet2_map.launch
  -> lanelet2_map node
  -> lanelet cost grid (map base)
  -> lanelet planning base grid
  -> cost field + marker aggregation
```

## Main Launch Arguments
- `map_path`
- `origin_lat`, `origin_lon`, `origin_alt`
- `map_param_file`
- `map_visualization_param_file`
- `enable_nav2_inflation_debug_marker`
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
- `area_export.launch.py`: exports semantic area metadata (drop_zone, camping_site_*) from Lanelet2 map

## Configuration
- `camrod_bringup/config/map/map_info.yaml`
- `camrod_bringup/config/map/lanelet_cost_grid.yaml`
- `camrod_bringup/config/map/map_visualization.yaml`

## Diagnostics
- Module-local topic: `/map/diagnostic`
- Aggregated topic: `/diagnostics`
