## HH_260114 How to use dark RViz theme

```
source /opt/ros/humble/setup.bash
source ~/camping_cart_ws/install/setup.bash

rviz2 -d ~/camping_cart_ws/src/camping_cart_map/rviz/camping_cart_dark.rviz \
  --ros-args -p use_sim_time:=false
```
