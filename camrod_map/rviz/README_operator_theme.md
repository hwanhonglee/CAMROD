## Camrod Operator RViz Preset

This preset is tuned toward an operations-console visual style with camrod-oriented naming.

### Run

```bash
source /opt/ros/humble/setup.bash
source ~/camrod_ws/install/setup.bash

rviz2 \
  -d ~/camrod_ws/src/camrod_map/rviz/camrod_operator.rviz \
  -stylesheet ~/camrod_ws/src/camrod_map/rviz/operator_theme.qss \
  --ros-args -p use_sim_time:=false
```

### What changed

- Dark blue background and high-contrast UI theme
- Stronger lane/path readability
- Tuned lidar/perception point cloud rendering
- Reduced visual clutter (duplicate empty groups removed)
