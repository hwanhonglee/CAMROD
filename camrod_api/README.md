# camrod_api

Runtime API package for CAMROD:

- `plugin_api_bridge_node.py`
  - Exposes plugin-style ROS API topics/services.
  - Bridges `/planning/engage` and `/platform/drive_enabled`.
  - Converts `/diagnostics` into API-ready module states.
- `ui_backend_node.py`
  - Serves lightweight HTTP endpoints (`/api/state`, `/api/engage`, etc.).
  - Optionally serves static web UI files from `web/` or an override directory.
- `launch/api.launch.py`
  - Starts bridge/UI nodes with shared API topic/service configuration.

## Package layout

The nested path `camrod_api/camrod_api` is intentional for ROS 2 `ament_python`:

- outer `camrod_api/`: package root (launch, web assets, package metadata)
- inner `camrod_api/`: importable Python module namespace

## Notes

- Shared helper functions are centralized in `camrod_api/api_common.py`.
- No IMU-related runtime logic is included in this package.
