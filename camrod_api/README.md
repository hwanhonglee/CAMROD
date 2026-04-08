# camrod_api

## Role
`camrod_api` exposes ROS-friendly control/state endpoints for external UI or plugin clients.

## Package Diagram
```mermaid
graph TD
  BRIDGE[Plugin Api Bridge]
  TOPICS[Api State Topics]
  ENGAGE[Planning Engage]
  DRIVE[Platform Drive Enabled]
  DIAG[System Diagnostics]
  UI[Ui Backend]
  HTTP[Http Api]
  CLIENT[External Client]

  BRIDGE --> TOPICS
  BRIDGE --> ENGAGE
  DRIVE --> BRIDGE
  DIAG --> BRIDGE
  TOPICS --> UI
  DIAG --> UI
  UI --> HTTP
  HTTP --> CLIENT
```

## Node Data Flow
| Node | Main Inputs | Main Outputs |
|---|---|---|
| `plugin_api_bridge_node.py` | `/platform/drive_enabled`, `/diagnostics`, service calls to `/api/plugin/set/*` | `/planning/engage`, `/api/plugin/get/engage`, `/api/plugin/get/ready`, `/api/plugin/get/operation_mode`, `/api/plugin/get/module_states`, `/api/plugin/get/ready_message` |
| `ui_backend_node.py` | `/api/plugin/get/*`, `/diagnostics_agg`, HTTP requests | HTTP responses (`/api/state`, `/api/engage`, `/api/operation_mode/*`) and service client calls to `/api/plugin/set/*` |

## Inter-Package Connections
```mermaid
graph LR
  PLATFORM[camrod_platform] --> API[camrod_api]
  SYSTEM[camrod_system] --> API
  API --> PLANNING[camrod_planning]
  BRINGUP[camrod_bringup] --> API
```

## Topic Summary
| Direction | Topic / Endpoint | Purpose |
|---|---|---|
| In | `/platform/drive_enabled` | Current drive-enabled state from platform gate |
| In | `/diagnostics`, `/diagnostics_agg` | Module/system health used for API readiness |
| Out | `/planning/engage` | Engage command forwarded to planning gate |
| Out | `/api/plugin/get/*` | Plugin-compatible state topics |
| In/Out | `/api/plugin/set/*` (services) | External control path (engage/mode) |
| HTTP | `/api/*` | UI/API access for web client |

## Practical Usage
```bash
ros2 launch camrod_api api.launch.py
```

Common overrides:
```bash
ros2 launch camrod_api api.launch.py enable_ui_backend:=false
ros2 launch camrod_api api.launch.py ui_host:=0.0.0.0 ui_port:=8010
ros2 launch camrod_api api.launch.py frontend_dir:=/absolute/path/to/frontend/build
```

## Config / Parameters
- Launch-only parameters are declared in `launch/api.launch.py`.
- Shared topic/service names are centralized in `_plugin_api_topics()` to keep bridge/UI consistent.
