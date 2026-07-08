---
name: safety-critical-path
description: CAMROD runtime data flow and the cmd_vel safety gate chain
metadata: 
  node_type: memory
  type: project
  originSessionId: 874cb916-2ddd-4e00-a4fe-2c0616836bfd
---

Safety-critical control path (the `==>` chain in README diagrams):
**sensing → localization → planning → cmd_vel_gate → Ranger CAN**.

Key topic chain for velocity:
`/planning/cmd_vel_raw` (Nav2 controller out) → `planning_cmd_vel_gate_node.py` → `/planning/cmd_vel` (gated) → platform gate → `/platform/cmd_vel` (final to Ranger).

`planning_cmd_vel_gate_node` (Python, camrod_planning/scripts) is the primary motion-safety authority. It enforces, in order: manual/mission engage latches (`/planning/engage`, `/planning/mission_engage`), e-stop (hardware `/platform/status/estop` ORed with soft `/planning/state_machine/estop`), GNSS recovery hold (2 s), **lanelet static safety** (checks raw `/map/cost_grid/lanelet` BEFORE ego-cleared inflation grid — blocks forward translation on off-lane/boundary cost but allows in-place rotation), and **live LiDAR/Radar cost stops** (latched until corridor stays clear 2.0 s to prevent stop/go flicker). **Fails closed** if `/planning/cost_grid/inflation` is missing/stale >1.0 s. Has a deterministic unit test: `test/test_cmd_vel_gate_logic.py` (51 assertions as of v2.0.0).

Localization modes on `/localization/mode` (avg_msgs/AvgLocalizationMode): NORMAL / DEGRADED / DR_ONLY / INVALID. On GNSS outage → DR_ONLY (IMU+wheel). On recovery → `gnss_recovery_hold_s` = 2 s stop lets Nav2/costmap settle.

Planning: Nav2 defaults to connected-lanelet `LaneletRoute` planner; grid planners (Smac2D, NavFn, ThetaStar) are selectable diagnostics/recovery fallbacks. `goal_snapper` converts raw site goal → lanelet route goal on `/planning/goal_pose_snapped_ros`. `smoother_server` uses `robot_base_link` for collision checks (avoids Nav2 default `base_link` lookup failure). `local_path_extractor` publishes a `map`-fixed slice of `/planning/global_path`, clearing stale markers on route change.

State machine (`planning_state_machine_node.py`) publishes `avg_msgs/PlanningState` on `/planning/state_machine/state` (INIT/RUNNING/GOAL_REACHED/RETURNING/...). Mission selection via `/planning/mission_key` (avg_msgs/PlanningMissionKey, e.g. `camping_site_3`). Nav2 planner×controller combos are YAML overrides in `config/nav2_combo_profiles/` on top of `nav2_base.yaml`, selected via `nav2_combo_profile:=` launch arg.

Parking: `camrod_parking/site_maneuver` does campsite crab-entry (commands `Twist.linear.y` on `/planning/cmd_vel_raw`) → 180° rotate → unload wait → crab-exit → return handoff; stays silent on cmd_vel_raw when idle so it doesn't race Nav2. `drop_zone_parking` does yaw/lateral-feedback reverse parking, stops on `/platform/status/is_charging`. See [[field-baseline-and-conventions]], [[workspace-layout]].
