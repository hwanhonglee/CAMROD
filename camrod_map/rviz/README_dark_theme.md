## HH_260114 How to use dark RViz theme

```
source /opt/ros/humble/setup.bash
source ~/camrod_ws/install/setup.bash

rviz2 -d ~/camrod_ws/src/camrod_map/rviz/camrod_dark.rviz \
  --ros-args -p use_sim_time:=false
```

## Operator style preset

```
rviz2 -d ~/camrod_ws/src/camrod_map/rviz/camrod_operator.rviz \
  -stylesheet ~/camrod_ws/src/camrod_map/rviz/operator_theme.qss \
  --ros-args -p use_sim_time:=false
```
