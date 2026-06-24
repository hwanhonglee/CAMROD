# cmd_vel Output Gate Conditions
<!-- Reference for all boolean states that must hold for /platform/cmd_vel to be published. -->

## Full Pipeline

```
/sensing/gnss  +  /sensing/imu  +  /platform/status/wheel_odometry
        |
        v
localization_eskf_node      ->  /localization/odometry/filtered
        |
        v
localization_monitor_node   ->  /localization/mode
        |
        v
localization_pose_selector  ->  /localization/selected_pose  (feeds nav2)
        |
        v
nav2 controller             ->  /planning/cmd_vel_raw
        |
        v
planning_cmd_vel_gate_node  ->  /planning/cmd_vel
        |
        v
cmd_vel_gate_node           ->  /platform/cmd_vel   <-- final output
```

---

## Conditions Required for /platform/cmd_vel to Flow

All of the following must be satisfied simultaneously.

### 1. ESKF initialized
| Variable | Node | Condition |
|---|---|---|
| `initialized_` | localization_eskf_node | First GNSS snap received |
| `wheel_initialized_` | localization_eskf_node | First wheel odometry received |

Until both are true, ESKF publishes no pose — nav2 has nothing to plan with.

### 2. Localization mode acceptable
| Topic | Publisher | Required value |
|---|---|---|
| `/localization/mode` | localization_monitor_node | `NORMAL (0)` or `DEGRADED (1)` |

`DR_ONLY (2)` or `INVALID (3)` → pose_selector may switch to fallback or stop publishing.
Threshold is `fallback_on_mode_at_or_above` parameter (default: `2 = DR_ONLY`).

**Monitor evaluates mode from:**
```
gnss_good  = gnss_fresh && gnss_cov_ok && gnss_jump_ok && gnss_rate_ok
             [&& gnss_update_accepted  if filter_status_mode=stream]

wheel_good = wheel_fresh
             [&& wheel_update_accepted  if filter_status_mode=stream]

imu_ok     = imu_fresh

NORMAL     = imu_ok && gnss_good && wheel_good
DEGRADED   = imu_ok && gnss_good (high innovation warning)
DR_ONLY    = imu_ok && !gnss_good && wheel_good
INVALID    = !imu_ok  OR  (!gnss_good && !wheel_good)
```

### 3. Planning gate open
| Variable | Node | Set by topic | Blocks when |
|---|---|---|---|
| `_enabled` | planning_cmd_vel_gate_node | `/planning/engage` OR `/planning/mission_engage` | `False` → /planning/cmd_vel_raw blocked |
| `_estop` | planning_cmd_vel_gate_node | `/platform/status/estop` | `True` → overrides _enabled |
| `_cost_blocked_until` | planning_cmd_vel_gate_node | internal (costmap check) | current time < value |

Effective pass condition: `(manual_engage OR mission_engage) AND NOT _estop AND NOT cost_blocked`

HHL_260624 - Site crab/reverse and in-place parking rotations may bypass static
lanelet/global-path cost only for their mission-owned maneuver. Live LiDAR/Radar
dynamic cost still sets `_cost_blocked_until` and stops `/planning/cmd_vel`.

### 4. Platform gate open
| Variable | Node | Set by topic | Blocks when |
|---|---|---|---|
| `_enabled` | cmd_vel_gate_node | `/planning/engaged` by default; `/platform/drive_enable` remains an optional bench override | `False` → /planning/cmd_vel blocked |
| `_estop` | cmd_vel_gate_node | `/platform/status/estop` | `True` → overrides _enabled |

Effective pass condition: `_enabled AND NOT _estop`

---

## Quick Debug Checklist

When `/platform/cmd_vel` is silent, check in order:

```
1. ros2 topic echo /localization/mode
       NORMAL=0  DEGRADED=1  DR_ONLY=2  INVALID=3
       -> If DR_ONLY or INVALID: check GNSS/IMU/wheel topics for timeouts

2. ros2 topic echo /planning/cmd_vel_raw
       -> If silent: nav2 is not planning (localization not ready or no goal)

3. ros2 topic echo /planning/engaged
       -> If False: for RViz manual goal publish /planning/engage true;
          for UI campsite mission verify /planning/mission_engage true

4. ros2 topic echo /platform/drive_enabled
       -> If False while /planning/engaged is True, check platform engage_topic wiring

5. ros2 topic echo /platform/status/estop
       -> If True: hardware e-stop is active — check physical platform
```

---

## Topic → Effect Map

| Topic | Value | Effect |
|---|---|---|
| `/planning/engage` | `true` | Opens the manual 2D-goal latch in planning gate |
| `/planning/engage` | `false` | Closes only the manual latch; UI mission latch can keep `/planning/engaged=true` |
| `/planning/mission_engage` | `true` | Opens the accepted UI campsite/drop-zone mission latch |
| `/planning/engaged` | `true` | Opens the platform final gate by default |
| `/platform/drive_enable` | `true` | Optional platform bench override |
| `/platform/status/estop` | `true` | Closes ALL gates immediately, overrides engage |
| `/localization/mode` | `>= DR_ONLY(2)` | pose_selector may stop publishing primary pose |
| `/localization/state` | `false` | Overall localization unhealthy (imu_ok && (gnss_good || wheel_good)) |
| `/localization/state/degraded` | `true` | mode >= DEGRADED(1) |
