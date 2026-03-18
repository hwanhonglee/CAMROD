# Package Work Log

<!-- HH_260109 Initialize map package work log. -->

## 2026-02-06 21:18
- Mark lanelet cost layer as current after receiving a grid so planner costmap can become current (prevents compute_path from stalling).

## 2026-02-06 11:16
- Prevent lanelet cost layer from shrinking costmap update bounds (avoids illegal bounds change warnings when path grids move).

## 2026-02-04 16:25
- Keep path-mode cost grids free until a path is received, so planners can compute the first global path.

## 2026-02-02 10:32
- Use bringup `config/map/map_info.yaml` as the default map_info for `lanelet2_map.launch.py`.
- `drop_zone_export.launch.py` now defaults to bringup map_info and writes YAML under bringup localization config.

## 2026-02-02 11:55
- Lanelet cost grid supports bounds-only and path-only cost modes (static boundaries vs. path corridor).

## 2026-01-30 16:05
- 2026-01-30: Add map-bounds option for lanelet cost grids (use_map_bbox) to avoid hardcoded global window sizing.
- 2026-01-30: Add drop zone exporter node to generate YAML from Lanelet types (drop_zone/parking_*).
- 2026-01-30: Drop zone export launch now pulls origin/map_path defaults from map_info.yaml.
- 2026-01-30: Fix drop zone exporter area-boundary point extraction for Lanelet2 Area types.
- 2026-01-30: Use polygon centroid for linestrings tagged area=yes (drop_zone ways).
- 2026-01-30: Update map config to new_lanelet2_maps.osm and new origin coordinates.
- 2026-02-02: Add OSM fallback parser for drop_zone ways (type=drop_zone, area=yes) using local_x/local_y tags.
- 2026-02-02: Export drop_zone corners (4-point center) and default dropzone_types to drop_zone only.

## 2026-01-29 19:29
- HH_260129: Fix lanelet cost layer plugin library name in plugin XML (lanelet_cost_layer).
- HH_260129: Fix lanelet cost layer parameter lookup to use '.' separator (source_topic now loads).

## 2026-01-27 17:45
- HH_260127: Remove HH tags from runtime logs and demote lanelet cost layer/grid logs to DEBUG.

## 2026-01-26 10:20
- HH_260126: Add lock_window parameter to lanelet_cost_grid_node to keep grid window fixed at configured origin/size (used by planning global path grid to prevent origin drift).

## 2026-01-25 14:10
- HH_260125: lanelet_cost_grid_node now transforms incoming pose/path to map frame (fix drifting path grids when pose frame != map); added TF listener dependency note.

## 2026-01-23 15:30
- HH_260123: Make lanelet cost grid output topic configurable; base grid now follows fused localization pose for consistent map/path alignment.

## 2026-01-19
- HH_260119: Review README ordering/architecture; no functional map pipeline changes (doc only).

## 2026-01-14 14:35
- HH_260114: Translate remaining comments/docs to English; keep HH tags aligned.
- HH_260114: Clarify map_info origin comment and lanelet visualization styling notes.

## 2026-01-12 15:41
- HH_260112: Update RViz planning display topics to match Nav2 outputs.

## 2026-01-12 12:45
- HH_260112: Rename map nodes to short defaults for namespaced execution.

## 2026-01-12 10:40
- HH_260112: Namespace map nodes under /map and align map/cost grid parameter keys.

## 2026-01-09 20:20
- HH_260109: Remove unused map visualization launch helper.

## 2026-01-09 19:22
- HH_260109: Remove test-only cost_grid_publisher node.

## 2026-01-09 18:37
- HH_260109: Move visualization node out and prefix map topics (/map/*, /visualizer/*).

## 2026-01-09 17:17
- HH_260109: Fix CMake install rule for plugin_cost_map.xml file.

## 2026-01-09 15:46
- HH_260109: Set lanelet cost grid to use fused localization pose by default.
