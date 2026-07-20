# Command Gate Conditions

<!-- HH_260720 - Replace the removed two-gate architecture with the single control-owned gate. -->

`cmd_vel_safety_gate` is the only final command authorization node.

```text
Nav2 /control/nav2_cmd_vel_ros -----> explicit ROS-to-Avg conversion --+
camping/drop-zone/parking ----------> /control/cmd_vel_raw ------------+--> gate
                                                                       |     |
                                                                       |     +--> /control/cmd_vel
                                                                       |     +--> /control/cmd_vel_ros --> Ranger
```

The gate publishes motion only when all applicable conditions pass:

| Condition | Canonical input |
|---|---|
| Manual or mission engage | `/planning/engage`, `/planning/mission_engage` |
| Operator drive arm | `/platform/drive_enable` |
| Hardware e-stop clear, CAN mode valid, no platform error | `/platform/status` (`AvgPlatformStatus`) |
| Not charging, or active campsite departure grace window | `/platform/status`, `/planning/mission_key` |
| Battery above critical threshold | `/platform/status` |
| Localization recovery hold and DR timeout clear | `/localization/mode`, `/localization/dr_timeout` |
| Cost grid fresh and travel corridor clear | `/planning/cost_grid/inflation` |
| Input command fresh | `/control/cmd_vel_raw` or `/control/nav2_cmd_vel_ros` |

For diagnosis, inspect these topics in order:

```bash
ros2 topic echo --once /control/command_enabled
ros2 topic echo --once /control/cmd_vel_safety_gate/status
ros2 topic echo --once /platform/status
ros2 topic echo --once /localization/mode
ros2 topic echo --once /planning/cost_grid/inflation
ros2 topic echo --once /control/cmd_vel
ros2 topic echo --once /control/cmd_vel_ros
```

There is no `/platform/cmd_vel`, platform command gate, or compatibility alias.
