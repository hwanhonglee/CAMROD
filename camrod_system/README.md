# camrod_system

System health and diagnostic aggregation module.

## Purpose
- Run per-module checker nodes
- Verify required nodes/topics/lifecycle states
- Publish unified diagnostic stream for the full stack

## Entry Points
- Full checker set:
  ```bash
  ros2 launch camrod_system module_checkers.launch.py
  ```
- System checker only:
  ```bash
  ros2 launch camrod_system system_checker.launch.py
  ```

## Runtime Structure
```text
module_checker_node.py (map/sensing/localization/planning/platform/perception/sensor_kit/bringup/system)
    -> status entries
    -> /diagnostics

system_diagnostic_node.py
    -> aggregates source diagnostics
    -> /diagnostics
```

## Main Launch Arguments
- `enable_checkers` (module checker launcher)
- `param_file` (system checker config)
- `system_namespace` (default `system`)

## Key Topics
- Aggregated diagnostics: `/diagnostics`
- Checker status names (diagnostic status keys):
  - `map/checker`
  - `sensing/checker`
  - `localization/checker`
  - `planning/checker`
  - `platform/checker`
  - `perception/checker`
  - `sensor_kit/checker`
  - `bringup/checker`
  - `system/checker`

## Configuration
- `camrod_bringup/config/system/system_checker.yaml`

