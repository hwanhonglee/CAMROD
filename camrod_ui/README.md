# camrod_ui

## Role
`camrod_ui` is the unified UI/API package for CAMROD.
It replaces `camrod_api` and provides:
- `ui_backend_node` for HTTP UI serving and direct destination bridge endpoints

## Node Overview
| Node | Inputs | Outputs |
|---|---|---|
| `ui_backend_node` | `/diagnostics_agg`, `/planning/engaged`, `/ui/selected_destination`, HTTP requests | HTTP responses, `/planning/engage`, `/planning/state_machine/goal_key`, `/goal_pose`, `/ui/selected_destination` |

## Launch
Primary launch:
```bash
ros2 launch camrod_ui ui.launch.py
```

Useful overrides:
```bash
ros2 launch camrod_ui ui.launch.py ui_host:=0.0.0.0 ui_port:=8010
ros2 launch camrod_ui ui.launch.py frontend_dir:=/absolute/path/to/frontend/build
ros2 launch camrod_ui ui.launch.py camping_sites_yaml:=/absolute/path/to/camping_sites.yaml
```

## Frontend path resolution
`ui.launch.py` resolves frontend in this order:
1. `CAMROD_UI_FRONTEND_DIR`
2. `CAMROD_API_FRONTEND_DIR` (legacy compatibility)
3. Source path `camrod_ui/runtime/assets/frontend/build`
4. Installed package path `share/camrod_ui/assets/frontend/build`
5. Fallback `share/camrod_ui/assets/web`
