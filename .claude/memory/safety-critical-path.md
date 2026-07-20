---
name: safety-critical-path
description: CAMROD runtime data flow and the cmd_vel safety gate chain
metadata: 
  node_type: memory
  type: project
  originSessionId: 874cb916-2ddd-4e00-a4fe-2c0616836bfd
---

<!-- HH_260720 - Record the one-gate control path and charging departure behavior. -->
Safety-critical control path (the `==>` chain in README diagrams):
**sensing -> localization -> planning/control candidates -> control safety gate -> Ranger CAN**.

Key topic chain for velocity:
Nav2's standard `Twist` output is remapped to `/control/nav2_cmd_vel_ros`; CAMROD controllers publish generated `AvgTwist` candidates on `/control/cmd_vel_raw`. `cmd_vel_safety_gate_node` publishes generated `/control/cmd_vel` and the explicit Ranger boundary `/control/cmd_vel_ros`. There is no second platform gate or `/platform/cmd_vel` alias.

`cmd_vel_safety_gate_node` (`camrod_control/camrod_control/safety`) is the only command authority. It enforces engage/drive latches, unified `/platform/status` e-stop/CAN/error/charging/battery state, GNSS recovery hold, lanelet static safety, and directional live LiDAR/Radar cost stops. It fails closed on stale required status, cost grid, pose, or command input. A new campsite mission can open a bounded charging-departure window so control input disengages charging and starts the site route.

Localization modes on `/localization/mode` (avg_msgs/AvgLocalizationMode): NORMAL / DEGRADED / DR_ONLY / INVALID. On GNSS outage → DR_ONLY (IMU+wheel). On recovery → `gnss_recovery_hold_s` = 2 s stop lets Nav2/costmap settle.

Planning: Nav2 defaults to connected-lanelet `LaneletRoute` planner; grid planners remain selectable diagnostics/recovery fallbacks. Internal snapped goals use generated `AvgPoseStamped` on `/planning/goal_pose_snapped`; `/planning/goal_pose_snapped_ros` is the explicit Nav2 boundary.

State machine (`planning_state_machine_node.py`) publishes `avg_msgs/PlanningState` on `/planning/state_machine/state` (INIT/RUNNING/GOAL_REACHED/RETURNING/...). Mission selection via `/planning/mission_key` (avg_msgs/PlanningMissionKey, e.g. `camping_site_3`). Nav2 planner×controller combos are YAML overrides in `config/nav2_combo_profiles/` on top of `nav2_base.yaml`, selected via `nav2_combo_profile:=` launch arg.

Control maneuvers: `camping_site_maneuver_controller` handles crab entry, 180-degree zero-turn, unload wait, crab exit, and return handoff. `drop_zone_maneuver_controller` handles drop-zone exit and yaw alignment. Parking itself is either `reverse_parking_controller` or the `apriltag_parking_controller` placeholder selected by `parking_method`; both stop when `/platform/status.is_charging` becomes true. See [[field-baseline-and-conventions]], [[workspace-layout]].
