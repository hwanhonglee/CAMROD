# camrod_map

## Role
`camrod_map` loads Lanelet2 map data, generates map/planning cost grids, and publishes map/cost visualization markers.

## Package Diagram
```mermaid
graph TD
  MAPREF[(map_info config and map path)] --> MAPNODE[lanelet2_map_node]
  MAPNODE --> MAPMARK((Map Marker Topic))
  MAPNODE --> MAPSTAT((Map Status Topic))

  LOCALPOSE((Localization Pose Topic)) --> GRID[lanelet_cost_grid_node]
  GLOBALPATH((Planning Global Path Topic)) --> GRID
  GRID --> BASEGRID((Lanelet Cost Grid Topic))
  GRID --> PLANBASE((Planning Base Grid Topic))
  GRID --> MAPSTAT

  BASEGRID --> MULTI[multi_cost_field_marker_node]
  MULTI --> COSTMARK((Cost Marker Topics))

  COSTSRC((Cost Marker Inputs)) --> AGGR[marker_array_aggregator_node]
  AGGR --> INFLATE((Inflation Marker Topic))

  MAPREF --> FIELD[cost_field_node optional]
  FIELD --> FIELDMARK((Lanelet Field Marker Topic))
```

Diagram legend: `[node/process]`, `((topic stream))`, `[(config or interface)]`, `[[external package or launch]]`.

## Node Data Flow
| Node | Main Inputs | Main Outputs |
|---|---|---|
| `lanelet2_map_node` | `map_info.yaml`, `map_path`, origin params | `/map/markers`, `/map/status` |
| `lanelet_cost_grid_node` (single integrated node) | `/localization/pose`, `/planning/global_path` (and optional fallback/goal), map/origin params | `/map/cost_grid/lanelet`, `/map/cost_grid/planning_base`, `/map/status` |
| `multi_cost_field_marker_node` | configured grid topics (`OccupancyGrid`) | marker topics such as `/map/cost_grid/lanelet_markers`, `/map/cost_grid/lidar_markers`, `/map/cost_grid/radar_markers` |
| `marker_array_aggregator_node` | multiple marker array contributors | `/map/cost_grid/inflation_markers` |
| `cost_field_marker_node` | one occupancy grid (default `/planning/global_costmap/costmap`) | one marker array topic (debug/visualization) |
| `cost_field_node` (optional) | map+origin+viz params | `/map/cost_grid/lanelet_field_markers` |
| `area_exporter_node` (utility launch) | lanelet map file | exported area YAML (drop zones / camping sites) |

## Inter-Package Connections
```mermaid
graph LR
  MAP[camrod_map] --> PLANNING[camrod_planning]
  MAP --> LOCALIZATION[camrod_localization]
  SENSING[camrod_sensing] --> MAP
  PERCEPTION[camrod_perception] --> MAP
  MAP --> SYSTEM[camrod_system]
```

## Topic Summary
### Input Topics
| Input Topic | Purpose |
|---|---|
| `/localization/pose` | Center pose for dynamic lanelet grid focus |
| `/planning/global_path` | Path-aware cost shaping |

### Output Topics
| Output Topic | Purpose |
|---|---|
| `/map/cost_grid/lanelet` | Base lanelet occupancy/cost grid |
| `/map/cost_grid/planning_base` | Planning-oriented secondary grid |
| `/map/cost_grid/inflation_markers` | Merged visualization markers |
| `/map/status` | Module status payload |

## Practical Usage
```bash
ros2 launch camrod_map map.launch.py
```

Sub-launches:
```bash
ros2 launch camrod_map lanelet2_map.launch.py
ros2 launch camrod_map cost_grid.launch.py
ros2 launch camrod_map visualization.launch.py
ros2 launch camrod_map area_export.launch.py
```

Map override example:
```bash
ros2 launch camrod_map map.launch.py map_path:=/absolute/path/lanelet2_maps.osm
```

## Config Files
- `config/map_info.yaml` (shared source-of-truth for map path/origin/frame reference)
- `config/lanelet_cost_grid.yaml`
- `config/map_visualization.yaml`
- `config/drop_zones.yaml`
- `config/nav2_params_costlayer_example.yaml` (example)

## Path Handling Rule
- Default map/origin values are read from `config/map_info.yaml`.
- Launch arguments (`map_path`, `origin_lat/lon/alt`) override YAML values.
- bringup can pass the same `map_path` into map/localization/planning for unified runtime behavior.
