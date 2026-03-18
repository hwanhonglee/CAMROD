# Camrod Node Execution Audit

Auto-generated static audit from CMake + launch files.

## camrod_localization

- C++ executables in CMake: 9
- Referenced by any launch: 9
- Not referenced by launch: 0

### Launched C++ executables
- `drop_zone_matcher_node`
  - from `camrod_localization/launch/localization.launch.py`
- `kimera_csv_bridge_node`
  - from `camrod_localization/launch/localization.launch.py`
- `localization_eskf_node`
  - from `camrod_localization/launch/localization.launch.py`
- `localization_health_monitor_node`
  - from `camrod_localization/launch/localization.launch.py`
- `localization_pose_selector_node`
  - from `camrod_localization/launch/localization.launch.py`
- `localization_supervisor_node`
  - from `camrod_localization/launch/localization.launch.py`
- `navsat_to_pose_node`
  - from `camrod_localization/launch/localization.launch.py`
- `odometry_to_pose_node`
  - from `camrod_localization/launch/localization.launch.py`
- `wheel_odometry_bridge_node`
  - from `camrod_localization/launch/localization.launch.py`

### C++ executables not launched
- (none)

## camrod_map

- C++ executables in CMake: 6
- Referenced by any launch: 6
- Not referenced by launch: 0

### Launched C++ executables
- `cost_field_marker_node`
  - from `camrod_map/launch/map.launch.py`
  - from `camrod_planning/launch/path_cost_grids.launch.py`
- `cost_field_node`
  - from `camrod_map/launch/map.launch.py`
- `drop_zone_exporter_node`
  - from `camrod_map/launch/drop_zone_export.launch.py`
- `lanelet2_map_node`
  - from `camrod_map/launch/lanelet2_map.launch.py`
- `lanelet_cost_grid_node`
  - from `camrod_map/launch/map.launch.py`
  - from `camrod_planning/launch/path_cost_grids.launch.py`
- `marker_array_aggregator_node`
  - from `camrod_map/launch/map.launch.py`

### C++ executables not launched
- (none)

## camrod_perception

- C++ executables in CMake: 2
- Referenced by any launch: 2
- Not referenced by launch: 0

### Launched C++ executables
- `obstacle_fusion_node`
  - from `camrod_perception/launch/perception.launch.py`
- `obstacle_lidar_node`
  - from `camrod_perception/launch/perception.launch.py`

### C++ executables not launched
- (none)

## camrod_planning

- C++ executables in CMake: 4
- Referenced by any launch: 4
- Not referenced by launch: 0

### Launched C++ executables
- `centerline_snapper_node`
  - from `camrod_planning/launch/planning.launch.py`
- `goal_replanner_node`
  - from `camrod_planning/launch/planning.launch.py`
- `goal_snapper_node`
  - from `camrod_planning/launch/planning.launch.py`
- `local_path_extractor_node`
  - from `camrod_planning/launch/planning.launch.py`

### C++ executables not launched
- (none)

## camrod_platform

- C++ executables in CMake: 1
- Referenced by any launch: 1
- Not referenced by launch: 0

### Launched C++ executables
- `robot_visualization_node`
  - from `camrod_platform/launch/platform.launch.py`

### C++ executables not launched
- (none)

## camrod_sensing

- C++ executables in CMake: 6
- Referenced by any launch: 6
- Not referenced by launch: 0

### Launched C++ executables
- `camera_preprocessor_node`
  - from `camrod_sensing/launch/sensing.launch.py`
- `lidar_cost_grid_node`
  - from `camrod_bringup/launch/fake_sensors.launch.py`
  - from `camrod_sensing/launch/sensing.launch.py`
- `lidar_preprocessor_node`
  - from `camrod_sensing/launch/lidar.launch.py`
- `platform_velocity_converter_node`
  - from `camrod_sensing/launch/platform_velocity_converter.launch.py`
  - from `camrod_sensing/launch/sensing.launch.py`
- `radar_cost_grid_node`
  - from `camrod_bringup/launch/fake_sensors.launch.py`
  - from `camrod_sensing/launch/sensing.launch.py`
- `sen0592_radar_node`
  - from `camrod_sensing/launch/radar.launch.py`
  - from `camrod_sensing/launch/sensing.launch.py`

### C++ executables not launched
- (none)

## Python Nodes Referenced By Launch

### camrod_bringup
- `bringup_diagnostic_node.py`
  - from `camrod_bringup/launch/bringup.launch.py`
- `fake_sensor_publisher.py`
  - from `camrod_bringup/launch/fake_sensors.launch.py`

### camrod_localization
- `localization_diagnostic_node.py`
  - from `camrod_localization/launch/localization.launch.py`

### camrod_map
- `map_diagnostic_node.py`
  - from `camrod_map/launch/map.launch.py`

### camrod_perception
- `perception_diagnostic_node.py`
  - from `camrod_perception/launch/perception.launch.py`

### camrod_planning
- `planning_diagnostic_node.py`
  - from `camrod_planning/launch/planning.launch.py`
- `planning_state_machine_node.py`
  - from `camrod_planning/launch/planning.launch.py`

### camrod_platform
- `platform_diagnostic_node.py`
  - from `camrod_platform/launch/platform.launch.py`

### camrod_sensing
- `sensing_diagnostic_node.py`
  - from `camrod_sensing/launch/sensing.launch.py`

### camrod_sensor_kit
- `sensor_kit_diagnostic_node.py`
  - from `camrod_sensor_kit/launch/sensor_kit.launch.py`

### camrod_system
- `module_checker_node.py`
  - from `camrod_localization/launch/localization.launch.py`
  - from `camrod_map/launch/map.launch.py`
  - from `camrod_perception/launch/perception.launch.py`
  - from `camrod_planning/launch/planning.launch.py`
  - from `camrod_platform/launch/platform.launch.py`
  - from `camrod_sensing/launch/sensing.launch.py`
  - from `camrod_system/launch/module_checkers.launch.py`
- `system_checker_node.py`
  - from `camrod_system/launch/system_checker.launch.py`
- `system_diagnostic_node.py`
  - from `camrod_system/launch/module_checkers.launch.py`
  - from `camrod_system/launch/system_checker.launch.py`

## Topic Multiplicity (candidate review list)

Topics repeated in many files are not automatically wrong, but are good cleanup targets.

- `/diagnostic`: 38 references
- `/planning/lanelet_pose`: 23 references
- `/localization/pose`: 20 references
- `/localization/messages`: 19 references
- `/planning/goal_pose`: 19 references
- `/map/messages`: 17 references
- `/planning/global_path`: 17 references
- `/sensing/lidar/points_filtered`: 15 references
- `/sensing/imu/data`: 14 references
- `/sensing/radar/front/range`: 14 references
- `/sensing/radar/left1/range`: 14 references
- `/sensing/radar/left2/range`: 14 references
- `/sensing/radar/rear/range`: 14 references
- `/sensing/radar/right1/range`: 14 references
- `/sensing/radar/right2/range`: 14 references
- `/localization/pose_with_covariance`: 13 references
- `/perception/obstacles`: 13 references
- `/map/cost_grid/lanelet`: 12 references
- `/sensing/camera/processed/camera_info`: 12 references
- `/map/cost_grid/planning_base`: 11 references
- `/localization/odometry/filtered`: 10 references
- `/sensing/gnss/pose_with_covariance`: 10 references
- `/sensing/lidar/near_cost_grid`: 10 references
- `/sensing/radar/near_cost_grid`: 10 references
- `/dev/ttyACM0`: 9 references
- `/planning/local_path`: 9 references
- `/planning/messages`: 9 references
- `/platform/wheel/odometry`: 9 references
- `/sensing/messages`: 9 references
- `/sensing/lidar/vanjee/points_raw`: 8 references
- `/vanjee_device_ctrl_state`: 8 references
- `/vanjee_packet`: 8 references
- `/perception/camera/detections_2d`: 7 references
- `/planning/cost_grid/global_path_markers`: 7 references
- `/planning/cost_grid/local_path_markers`: 7 references
- `/sensing/camera/processed/image`: 7 references
- `/sensing/gnss/pose`: 7 references
- `/sensing/platform_velocity_converter/twist_with_covariance`: 7 references
- `/system/messages`: 7 references
- `/dev/ttyACM1`: 6 references
- `/localization/eskf/diagnostics`: 6 references
- `/map/cost_grid/inflation_markers`: 6 references
- `/map/cost_grid/lidar_markers`: 6 references
- `/map/cost_grid/radar_markers`: 6 references
- `/map/lanelet2_map`: 6 references
- `/perception/lidar/bboxes`: 6 references
- `/perception/messages`: 6 references
- `/planning/navigate_to_pose/_action/status`: 6 references
- `/platform/messages`: 6 references
- `/sensing/gnss/navsatfix`: 6 references
- `/sensor_kit/messages`: 6 references
- `/goal_pose`: 5 references
- `/localization/kimera_vio/odometry`: 5 references
- `/localization/kimera_vio/pose_with_covariance`: 5 references
- `/map/cost_grid/lanelet_markers`: 5 references
- `/platform/status/velocity`: 5 references
- `/sensing/camera/camera_info`: 5 references
- `/sensing/camera/image_raw`: 5 references
- `/tf_static`: 5 references
- `/dev/ttyCH9344USB2`: 4 references
- `/dev/ttyCH9344USB3`: 4 references
- `/dev/ttyCH9344USB4`: 4 references
- `/dev/ttyCH9344USB5`: 4 references
- `/dev/ttyCH9344USB6`: 4 references
- `/dev/ttyCH9344USB7`: 4 references
- `/localization/eskf/odometry`: 4 references
- `/localization/eskf/pose_with_covariance`: 4 references
- `/localization/mode`: 4 references
- `/localization/twist`: 4 references
- `/perception/obstacles/lidar`: 4 references
- `/planning/behavior_server`: 4 references
- `/planning/bt_navigator`: 4 references
- `/planning/controller_server`: 4 references
- `/planning/cost_grid/global_path`: 4 references
- `/planning/cost_grid/local_path`: 4 references
- `/planning/global_costmap/costmap`: 4 references
- `/planning/planner_server`: 4 references
- `/platform/robot/markers`: 4 references
- `/platform/robot/planning_boundary`: 4 references
- `/tf`: 4 references
- `/vanjee_lidar_parameter_pub`: 4 references
- `/vanjee_lidar_parameter_sub`: 4 references