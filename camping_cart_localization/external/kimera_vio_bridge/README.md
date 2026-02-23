# Kimera VIO CSV Bridge (External)

This external module hosts `kimera_csv_bridge_node.cpp`, which tails Kimera-VIO
CSV outputs (`traj_abs.csv`, `traj_local.csv`) and republishes them as ROS topics
for fallback localization.

Build integration is defined in:
- `camping_cart_localization/CMakeLists.txt`

Runtime launch integration is defined in:
- `camping_cart_localization/launch/localization.launch.py`

