# camrod_map

<!-- HH_260804 - Summarize the map source, dual-grid values, semantic products,
and known route limitation without duplicating every launch parameter. -->

Lanelet2 map loading, WGS84 projection, semantic-area export, route masks,
planning cost grids, and RViz visualization.

![Lanelet map and cost grids](../docs/assets/module-guides/map/lanelet-map-and-cost-grids.png)

## At A Glance

| Uses | Function | Main outputs |
|---|---|---|
| Lanelet2 OSM + LocalCartesian projector | Loads road geometry and directional lanelets | `/map/markers`, map metadata |
| Active route lanelet IDs | Builds a binary route mask for obstacle filtering | `/map/cost_grid/route_lanelet_mask` |
| All lanelets + centerline costs | Builds the Nav2 planning base | `/map/cost_grid/lanelet` |
| OSM semantic Areas | Exports drop zones and service metadata | `drop_zones.yaml` and related generated YAML |

## Active Map Reference

| Item | Value |
|---|---|
| Runtime map | `/home/nvidia/camrod_ws/src/lanelet2_maps.osm` |
| Projector | `local_cartesian` |
| WGS84 origin | `36.8435737, 128.0925646, 0.0` |
| Map yaw offset | `0.0 deg` |
| Frames | `world -> map` |
| Immediate visualization radius | `120 m` |

The absolute map path is a Jetson deployment path. Workstation testing may
override it, but must not rewrite the production path solely for local use.

## Cost Grids

| Product | Geometry | Mode | Key costs |
|---|---|---|---|
| Active-route mask | `600 x 600 @ 0.20 m` (`120 m`) | full route lanelets | inside `0`, outside `100` |
| Nav2 planning base | `960 x 960 @ 0.25 m` (`240 m`) | centerline gradient | center `0`, fill `70`, edge `98`, outside `100` |

| Rule | Current behavior |
|---|---|
| Hard body boundary | Full footprint stops at cost `100` or unknown |
| Soft mapped edge | Cost `98` biases planning but remains traversable |
| Lane change | `lane_change=yes` can clear configured crossing cells |
| Grid placement | Robot-centered window |
| Rebuild | Route/path change and pose-exits-grid; not every pose update |
| Republish | Static grid every `1.0 s` for late lifecycle consumers |

## Boundary Evidence

![Robot-center narrow-route risk](../docs/assets/module-guides/planning/robot-center-narrow-route-risk-map.png)

The current B6/B12 smoke run reaches campsite entry but the complete footprint
contacts the mapped boundary. Existing campsite Areas do not provide surveyed
road-to-service `service_access` geometry. This is a map-data limitation; the
`0.10 m` safety margin and hard footprint threshold remain unchanged.

## Nodes And Products

| Node/tool | Responsibility |
|---|---|
| `lanelet_map_provider` | Loads OSM, publishes map markers and projection context |
| `lanelet_boundary_cost_grid` | Publishes route mask and all-lane planning grid |
| `cost_grid_multi_marker` | Converts map/LiDAR/radar grids to throttled RViz markers |
| `lanelet_cost_field_visualizer` | Displays distance/curvature cost structure |
| `area_exporter` | Converts semantic OSM Areas into deployment YAML |

## Run And Validate

```bash
ros2 launch camrod_map map.launch.py
ros2 launch camrod_map area_export.launch.py

ros2 topic echo --once /map/markers
ros2 topic echo --once /map/cost_grid/lanelet
ros2 topic echo --once /map/cost_grid/route_lanelet_mask
ros2 run tf2_ros tf2_echo world map
```

| Config | Purpose |
|---|---|
| `config/map_info.yaml` | Single map path, origin, frames, and visualization source |
| `config/lanelet_cost_grid.yaml` | Route mask and planning-grid geometry/costs |
| `config/map_visualization.yaml` | RViz marker palettes, rates, and sampling |
| `config/drop_zones.yaml` | Generated semantic drop-zone geometry |

Grid dimensions and thresholds are source configuration, not measured map-build
latency. Field route coverage remains pending where service-access geometry is
missing.
