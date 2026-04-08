# camrod_system

## Role
`camrod_system` runs diagnostics checkers and aggregation for the full CAMROD stack.

## Package Diagram
```mermaid
graph TD
  CHECK[Component Checker Nodes] --> DIAG[Diagnostics Stream]
  DIAG --> AGG[Aggregator Node]
  AGG --> DIAGAGG[Diagnostics Aggregated]
```

## Node Data Flow
| Node / Group | Main Inputs | Main Outputs |
|---|---|---|
| Hardware checkers (`hw_checker`, `gpu_checker`, `network_checker`) | HW/network probes | `/system/diagnostics` statuses |
| Sensing checkers (`gnss`, `imu`, `lidar`, `radar`, `camera`, `wheel_odometry`, `cost_grid`, `velocity_converter`) | sensing topics | `/system/diagnostics` statuses |
| Localization checkers (`localization_*_checker`) | localization topics/signals | `/system/diagnostics` statuses |
| Map checker (`map_cost_grid_checker`) | `/map/cost_grid/lanelet` | `/system/diagnostics` status |
| Perception checker (`perception_obstacle_checker`) | perception obstacle topic | `/system/diagnostics` status |
| Planning checkers (`planning_lifecycle/costmap/nav_status/path`) | planning topics/services | `/system/diagnostics` statuses |
| Platform checker (`ranger_platform_checker`, optional build/runtime) | platform-specific topics | `/system/diagnostics` status |
| `aggregator_node` | `/system/diagnostics` (via remap from `/diagnostics`) | `/system/diagnostics_agg` |

## Inter-Package Connections
```mermaid
graph LR
  MAP[camrod_map] --> SYS[camrod_system]
  SENSING[camrod_sensing] --> SYS
  LOC[camrod_localization] --> SYS
  PER[camrod_perception] --> SYS
  PLAN[camrod_planning] --> SYS
  PLATFORM[camrod_platform] --> SYS
  API[camrod_api] --> SYS
```

## Topic Summary
| Direction | Topic | Purpose |
|---|---|---|
| In | module runtime topics (map/sensing/localization/perception/planning/platform) | health checks |
| Out | `/system/diagnostics` | raw diagnostics stream |
| Out | `/system/diagnostics_agg` | aggregated diagnostics stream |

## Practical Usage
```bash
ros2 launch camrod_system system.launch.py
```

Examples:
```bash
ros2 launch camrod_system system.launch.py config_profile:=default
ros2 launch camrod_system system.launch.py enable_checkers:=true enable_platform:=false
```

## Config Files
- `config/diagnostics/default/*` (component checker YAMLs)
- `config/diagnostics/default/aggregator/diagnostics_config.yaml`
- backward-compatible wrapper launch: `launch/module_checkers.launch.py`
