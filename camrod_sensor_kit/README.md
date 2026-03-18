# camrod_sensor_kit

Sensor-kit module that publishes robot description and core TF tree.

## Purpose
- Build URDF from xacro and runtime parameters
- Publish static robot/sensor frames via `robot_state_publisher`
- Provide canonical anchor chain for all sensing/localization/planning modules

## Entry Point
```bash
ros2 launch camrod_sensor_kit sensor_kit.launch.py
```

## TF Layout
```text
robot_base_link
  -> sensor_kit_base_link
      -> imu_link
      -> gnss_link
      -> lidar_link
      -> camera_front_link
```

## Main Launch Arguments
- `params_file` (robot dimensions + sensor offsets)
- `module_namespace` (default `sensor_kit`)
- `base_frame_id` (default `robot_base_link`)
- `sensor_kit_base_frame_id` (default `sensor_kit_base_link`)
- `map_frame_id` (default `map`)
- `enable_diagnostic`

## Configuration
- `camrod_bringup/config/sensor_kit/robot_params.yaml`

## Diagnostics
- Module-local topic: `/sensor_kit/diagnostic`
- Aggregated topic: `/diagnostics`

