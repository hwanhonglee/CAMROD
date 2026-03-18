# Node Naming Audit

HH_260317-00:00 launch-based node name readability audit.

Scope: `camrod_*/launch/*.py` Node actions only.


## Summary
- total nodes scanned: **77**
- heuristic flagged: **9**

## Flagged Candidates
- `rviz2` (`rviz2`) in `camrod_bringup/launch/bringup.launch.py`
- `planner_server` (`planner_server`) in `camrod_planning/launch/nav2_lanelet.launch.py`
- `controller_server` (`controller_server`) in `camrod_planning/launch/nav2_lanelet.launch.py`
- `behavior_server` (`behavior_server`) in `camrod_planning/launch/nav2_lanelet.launch.py`
- `bt_navigator` (`bt_navigator`) in `camrod_planning/launch/nav2_lanelet.launch.py`
- `ublox_gps_node` (`ublox_gps_node`) in `camrod_sensing/launch/gnss.launch.py`
- `sen0592_radar_node` (`sen0592_radar_node`) in `camrod_sensing/launch/radar.launch.py`
- `sen0592_radar_node` (`sen0592_radar_node`) in `camrod_sensing/launch/sensing.launch.py`
- `robot_state_publisher` (`robot_state_publisher`) in `camrod_sensor_kit/launch/sensor_kit.launch.py`

## All Nodes
- `bringup_diagnostic` -> `bringup_diagnostic_node.py` (camrod_bringup/launch/bringup.launch.py)
- `rviz2` -> `rviz2` (camrod_bringup/launch/bringup.launch.py)
- `fake_sensor_publisher` -> `fake_sensor_publisher.py` (camrod_bringup/launch/fake_sensors.launch.py)
- `lidar_cost_grid` -> `lidar_cost_grid_node` (camrod_bringup/launch/fake_sensors.launch.py)
- `radar_cost_grid` -> `radar_cost_grid_node` (camrod_bringup/launch/fake_sensors.launch.py)
- `navsat_to_pose` -> `navsat_to_pose_node` (camrod_localization/launch/localization.launch.py)
- `ekf_filter_main` -> `ekf_node` (camrod_localization/launch/localization.launch.py)
- `eskf_filter` -> `localization_eskf_node` (camrod_localization/launch/localization.launch.py)
- `eskf_filter` -> `localization_eskf_node` (camrod_localization/launch/localization.launch.py)
- `odom_to_pose_bridge` -> `odometry_to_pose_node` (camrod_localization/launch/localization.launch.py)
- `supervisor` -> `localization_supervisor_node` (camrod_localization/launch/localization.launch.py)
- `wheel_odometry_bridge` -> `wheel_odometry_bridge_node` (camrod_localization/launch/localization.launch.py)
- `drop_zone_matcher` -> `drop_zone_matcher_node` (camrod_localization/launch/localization.launch.py)
- `kimera_csv_bridge` -> `kimera_csv_bridge_node` (camrod_localization/launch/localization.launch.py)
- `pose_selector` -> `localization_pose_selector_node` (camrod_localization/launch/localization.launch.py)
- `health_monitor` -> `localization_health_monitor_node` (camrod_localization/launch/localization.launch.py)
- `localization_checker` -> `module_checker_node.py` (camrod_localization/launch/localization.launch.py)
- `localization_diagnostic` -> `localization_diagnostic_node.py` (camrod_localization/launch/localization.launch.py)
- `drop_zone_exporter` -> `drop_zone_exporter_node` (camrod_map/launch/drop_zone_export.launch.py)
- `lanelet2_map` -> `lanelet2_map_node` (camrod_map/launch/lanelet2_map.launch.py)
- `cost_grid_map` -> `lanelet_cost_grid_node` (camrod_map/launch/map.launch.py)
- `cost_grid_planning_base` -> `lanelet_cost_grid_node` (camrod_map/launch/map.launch.py)
- `inflation_nav2_cost_marker` -> `cost_field_marker_node` (camrod_map/launch/map.launch.py)
- `lanelet_cost_marker` -> `cost_field_marker_node` (camrod_map/launch/map.launch.py)
- `radar_cost_marker` -> `cost_field_marker_node` (camrod_map/launch/map.launch.py)
- `lidar_cost_marker` -> `cost_field_marker_node` (camrod_map/launch/map.launch.py)
- `inflation_marker_aggregator` -> `marker_array_aggregator_node` (camrod_map/launch/map.launch.py)
- `lanelet_cost_field` -> `cost_field_node` (camrod_map/launch/map.launch.py)
- `map_checker` -> `module_checker_node.py` (camrod_map/launch/map.launch.py)
- `map_diagnostic` -> `map_diagnostic_node.py` (camrod_map/launch/map.launch.py)
- `obstacle_fusion` -> `obstacle_fusion_node` (camrod_perception/launch/perception.launch.py)
- `obstacle_lidar` -> `obstacle_lidar_node` (camrod_perception/launch/perception.launch.py)
- `perception_checker` -> `module_checker_node.py` (camrod_perception/launch/perception.launch.py)
- `perception_diagnostic` -> `perception_diagnostic_node.py` (camrod_perception/launch/perception.launch.py)
- `planner_server` -> `planner_server` (camrod_planning/launch/nav2_lanelet.launch.py)
- `controller_server` -> `controller_server` (camrod_planning/launch/nav2_lanelet.launch.py)
- `behavior_server` -> `behavior_server` (camrod_planning/launch/nav2_lanelet.launch.py)
- `bt_navigator` -> `bt_navigator` (camrod_planning/launch/nav2_lanelet.launch.py)
- `lifecycle_manager_planning` -> `lifecycle_manager` (camrod_planning/launch/nav2_lanelet.launch.py)
- `lanelet_cost_grid_global_path` -> `lanelet_cost_grid_node` (camrod_planning/launch/path_cost_grids.launch.py)
- `lanelet_cost_grid_local_path` -> `lanelet_cost_grid_node` (camrod_planning/launch/path_cost_grids.launch.py)
- `global_path_cost_marker` -> `cost_field_marker_node` (camrod_planning/launch/path_cost_grids.launch.py)
- `local_path_cost_marker` -> `cost_field_marker_node` (camrod_planning/launch/path_cost_grids.launch.py)
- `goal_snapper` -> `goal_snapper_node` (camrod_planning/launch/planning.launch.py)
- `centerline_snapper` -> `centerline_snapper_node` (camrod_planning/launch/planning.launch.py)
- `local_path_extractor` -> `local_path_extractor_node` (camrod_planning/launch/planning.launch.py)
- `goal_replanner` -> `goal_replanner_node` (camrod_planning/launch/planning.launch.py)
- `planning_state_machine` -> `planning_state_machine_node.py` (camrod_planning/launch/planning.launch.py)
- `planning_checker` -> `module_checker_node.py` (camrod_planning/launch/planning.launch.py)
- `planning_diagnostic` -> `planning_diagnostic_node.py` (camrod_planning/launch/planning.launch.py)
- `nav2_lifecycle_startup_retry` -> `nav2_lifecycle_startup_retry_node.py` (camrod_planning/launch/planning.launch.py)
- `robot_visualization` -> `robot_visualization_node` (camrod_platform/launch/platform.launch.py)
- `base_link_alias_publisher` -> `static_transform_publisher` (camrod_platform/launch/platform.launch.py)
- `platform_checker` -> `module_checker_node.py` (camrod_platform/launch/platform.launch.py)
- `platform_diagnostic` -> `platform_diagnostic_node.py` (camrod_platform/launch/platform.launch.py)
- `camera_preprocessor` -> `camera_preprocessor_node` (camrod_sensing/launch/camera.launch.py)
- `ublox_gps_node` -> `ublox_gps_node` (camrod_sensing/launch/gnss.launch.py)
- `ntrip_client` -> `ntrip_ros.py` (camrod_sensing/launch/gnss.launch.py)
- `ntrip_client` -> `ntrip_ros.py` (camrod_sensing/launch/imu_ntrip.launch.py)
- `vanjee_driver` -> `vanjee_lidar_sdk_node` (camrod_sensing/launch/lidar.launch.py)
- `lidar_preprocessor` -> `lidar_preprocessor_node` (camrod_sensing/launch/lidar.launch.py)
- `static_tf_base_to_vanjee` -> `static_transform_publisher` (camrod_sensing/launch/lidar.launch.py)
- `platform_velocity_converter` -> `platform_velocity_converter_node` (camrod_sensing/launch/platform_velocity_converter.launch.py)
- `sen0592_radar_node` -> `sen0592_radar_node` (camrod_sensing/launch/radar.launch.py)
- `camera_preprocessor` -> `camera_preprocessor_node` (camrod_sensing/launch/sensing.launch.py)
- `platform_velocity_converter` -> `platform_velocity_converter_node` (camrod_sensing/launch/sensing.launch.py)
- `sen0592_radar_node` -> `sen0592_radar_node` (camrod_sensing/launch/sensing.launch.py)
- `radar_cost_grid` -> `radar_cost_grid_node` (camrod_sensing/launch/sensing.launch.py)
- `lidar_cost_grid` -> `lidar_cost_grid_node` (camrod_sensing/launch/sensing.launch.py)
- `sensing_checker` -> `module_checker_node.py` (camrod_sensing/launch/sensing.launch.py)
- `sensing_diagnostic` -> `sensing_diagnostic_node.py` (camrod_sensing/launch/sensing.launch.py)
- `robot_state_publisher` -> `robot_state_publisher` (camrod_sensor_kit/launch/sensor_kit.launch.py)
- `sensor_kit_diagnostic` -> `sensor_kit_diagnostic_node.py` (camrod_sensor_kit/launch/sensor_kit.launch.py)
- `system_diagnostic` -> `system_diagnostic_node.py` (camrod_system/launch/module_checkers.launch.py)
- `system_checker` -> `system_checker_node.py` (camrod_system/launch/system_checker.launch.py)
- `system_diagnostic` -> `system_diagnostic_node.py` (camrod_system/launch/system_checker.launch.py)
