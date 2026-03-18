# camrod_platform

Platform module for robot-side visualization and frame compatibility helpers.

## Purpose
- Launch platform visualization nodes
- Include `camrod_sensor_kit` for URDF/TF publication
- Optionally publish `robot_base_link -> base_link` static alias for legacy consumers

## Entry Point
```bash
ros2 launch camrod_platform platform.launch.py
```

## Runtime Structure
```text
platform.launch.py
  -> include camrod_sensor_kit/sensor_kit.launch.py
  -> robot_visualization_node
  -> static_transform_publisher (optional base_link alias)
```

## Main Launch Arguments
- Frames:
  - `map_frame_id` (default `map`)
  - `base_frame_id` (default `robot_base_link`)
  - `sensor_kit_base_frame_id` (default `sensor_kit_base_link`)
- Params:
  - `params_file`
  - `robot_visualization_param_file`
- Toggles:
  - `publish_base_link_alias`
  - `enable_module_checker`
- Namespaces:
  - `module_namespace` (default `platform`)
  - `sensor_kit_namespace` (default `sensor_kit`)
  - `system_namespace` (default `system`)

## Key Topics
- Visualization:
  - `/platform/robot/markers`
  - `/platform/robot/planning_boundary`
- TF:
  - `/tf`
  - `/tf_static`

## Diagnostics
- Module-local topic: `/platform/diagnostic`
- Aggregated topic: `/diagnostics`

