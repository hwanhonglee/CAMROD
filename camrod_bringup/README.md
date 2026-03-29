# camrod_bringup

Top-level runtime orchestrator for CAMROD modules.

## Role
- Includes all module launches in a controlled order
- Applies cross-module runtime toggles and shared arguments
- Loads defaults from a single config file
- Optionally runs pre-launch cleanup for stale processes

## Entry Point
```bash
ros2 launch camrod_bringup bringup.launch.py
```

## Architecture (Orchestration Layer)
```text
bringup.launch.py
  -> camrod_platform
  -> camrod_map
  -> camrod_sensing
  -> camrod_localization
  -> camrod_perception
  -> camrod_planning
  -> camrod_system (module validators / status_stream)
  -> RViz (optional)
```

## Main Launch Arguments
- Runtime: `clean_before_launch`, `sim`, `rviz`
- Localization toggles: `use_eskf`, `wheel_bridge_enable`, `kimera_bridge_enable`, `pose_selector_enable`
- Planning toggles: `enable_path_cost_grids`, `enable_goal_replanner`, `enable_nav2_lifecycle_retry`, `enable_state_machine`, `use_dwb_controller`
- Sensing toggles: `enable_lidar_driver`, `enable_lidar_cost_grid`, `enable_radar`, `enable_radar_cost_grid`, `enable_gnss`, `enable_ntrip`
- Shared map/origin: `map_path`, `origin_lat`, `origin_lon`, `origin_alt`, `yaw_offset_deg`
- Namespaces: `map_namespace`, `sensing_namespace`, `localization_namespace`, `planning_namespace`, `platform_namespace`, `perception_namespace`, `sensor_kit_namespace`, `bringup_namespace`, `system_namespace`, `gnss_namespace`

## Default Configuration
- `config/bringup/launch_defaults.yaml`: global defaults
- `config/bringup/cleanup_patterns.yaml`: process cleanup patterns

## StatusStream
- Module-local status publishers remain namespaced (for example `/map/status`)
- Aggregated system stream: `/status_stream`
- Bringup status node: `/bringup/bringup_status`

## Example Overrides
```bash
ros2 launch camrod_bringup bringup.launch.py sim:=true rviz:=true
ros2 launch camrod_bringup bringup.launch.py enable_gnss:=true enable_ntrip:=true
ros2 launch camrod_bringup bringup.launch.py map_path:=/home/camrod_ws/lanelet2_maps.osm
```

