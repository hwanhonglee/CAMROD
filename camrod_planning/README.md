# camrod_planning

Planning module built around Nav2, with lanelet-aware goal/pose snapping and path-cost overlays.

## Purpose
- Run Nav2 planner/controller/behavior/navigator in `/planning`
- Keep lanelet-aware goal ingestion and centerline pose snapping
- Publish canonical path topics:
  - global: `/planning/global_path`
  - local: `/planning/local_path`

## Entry Point
```bash
ros2 launch camrod_planning planning.launch.py
```

## Planning Flow
```text
/goal_pose
  -> goal_snapper
  -> /planning/goal_pose_snapped_ros
  -> Nav2 (bt_navigator)
  -> /planning/global_path
  -> local_path_extractor
  -> /planning/local_path
```

Pose alignment flow:
```text
/localization/pose -> centerline_snapper -> /planning/lanelet_pose
```

## Main Launch Arguments
- Nav2 overlays:
  - `nav2_base_param_file`
  - `nav2_vehicle_param_file`
  - `nav2_lanelet_param_file`
  - `nav2_behavior_param_file`
- Feature toggles:
  - `enable_path_cost_grids`
  - `enable_goal_replanner`
  - `enable_nav2_lifecycle_retry`
  - `enable_state_machine`
- Map/origin:
  - `map_path`, `origin_lat`, `origin_lon`, `origin_alt`
- Frame:
  - `nav2_robot_base_frame` (default `robot_base_link`)
- Namespaces:
  - `module_namespace` (default `planning`)
  - `system_namespace` (default `system`)

## Key Topics
- Inputs:
  - `/goal_pose`
  - `/localization/pose`
  - `/planning/state_machine/goal_key` (`std_msgs/String`, e.g. `camping_site_3`)
- Main outputs:
  - `/planning/global_path`
  - `/planning/local_path`
- Cost-grid marker outputs:
  - `/planning/cost_grid/global_path_markers`
  - `/planning/cost_grid/local_path_markers`

## Keypoint Goal Request (Camping Sites)
- Topic request:
  - publish `std_msgs/String` to `/planning/state_machine/goal_key`
  - payload example: `camping_site_7`
- Service request:
  - call `/planning/state_machine/request_goal`
  - type: `avg_msgs/srv/RequestGoalByKey`
  - request field: `key` (e.g. `camping_site_7`)
- Keypoint sources:
  - `keypoints_yaml` (static keypoints like `drop_zone`, `garage`)
  - `camping_sites_yaml` (exported by map area exporter)

## Related Launch Files
- `nav2_lanelet.launch.py`: Nav2 wrapper with overlay chain
- `path_cost_grids.launch.py`: path-grid marker helpers
- `planning_top.launch.py`: compatibility launcher (not required by bringup)

## Configuration
- `camrod_bringup/config/planning/nav2_base.yaml`
- `camrod_bringup/config/planning/nav2_vehicle.yaml`
- `camrod_bringup/config/planning/nav2_lanelet_overlay.yaml`
- `camrod_bringup/config/planning/nav2_behavior.yaml`
- `camrod_bringup/config/planning/local_path_extractor.yaml`
- `camrod_bringup/config/planning/path_cost_grids.yaml`

## Diagnostics
- Module-local topic: `/planning/diagnostic`
- Aggregated topic: `/diagnostics`
