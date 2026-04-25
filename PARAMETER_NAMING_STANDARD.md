# CAMROD Parameter Naming Standard

This document defines the canonical parameter naming rules for custom CAMROD packages
and records the legacy-to-canonical aliases currently supported in code.

## 1) Canonical Naming Rules

- Time duration in seconds: `*_s`
  - Examples: `stale_timeout_s`, `grace_period_s`, `cost_stop_hold_s`
- Frequency in Hz: `*_hz` or `*_rate_hz`
  - Examples: `publish_rate_hz`, `poll_rate_hz`, `expected_hz`
- ROS topic names: `*_topic` or `*_topics`
- Frame names: `*_frame_id`
- Warning/error thresholds: `*_warn_*`, `*_error_*` (existing keys preserved)

## 2) Migration Policy

- Canonical key is preferred.
- Legacy key is still accepted for backward compatibility.
- When only a legacy key is provided, runtime falls back to the legacy value and prints a deprecation warning.
- If canonical and legacy keys are both set with different values, canonical value wins.

## 3) Implemented Alias Migrations (This Change)

### camrod_system (C++ diagnostics)

- `publish_rate` -> `publish_rate_hz`
- `poll_rate` -> `poll_rate_hz`
- `stale_timeout` -> `stale_timeout_s`
- `<name>.stale_timeout` -> `<name>.stale_timeout_s`
- `global_stale_timeout` -> `global_stale_timeout_s`
- `local_stale_timeout` -> `local_stale_timeout_s`
- `odom_stale_timeout` -> `odom_stale_timeout_s`
- `velocity_stale_timeout` -> `velocity_stale_timeout_s`
- `imu_stale_timeout` -> `imu_stale_timeout_s`
- `output_stale_timeout` -> `output_stale_timeout_s`
- `grace_period_sec` -> `grace_period_s`
- `fallback_warn_sec` -> `fallback_warn_s`
- `fallback_error_sec` -> `fallback_error_s`

### camrod_system (Python)

- `startup_grace_sec` -> `startup_grace_s`

### camrod_planning (Python)

- `cost_stop_hold_sec` -> `cost_stop_hold_s`

### camrod_localization (C++)

- `jump_reject_reset_sec` -> `jump_reject_reset_s`
- `wheel_primary_timeout_sec` -> `wheel_primary_timeout_s`
- `primary_timeout_sec` -> `primary_timeout_s`
- `fallback_timeout_sec` -> `fallback_timeout_s`
- `switch_hysteresis_sec` -> `switch_hysteresis_s`
- `gnss_timeout_sec` -> `gnss_timeout_s`
- `imu_timeout_sec` -> `imu_timeout_s`
- `wheel_timeout_sec` -> `wheel_timeout_s`
- `stop_hold_sec` -> `stop_hold_s`
- `centerline_min_update_period_sec` -> `centerline_min_update_period_s`

### camrod_map (C++)

- `visualization_republish_period_sec` -> `visualization_republish_period_s`
- `stale_timeout_sec` -> `stale_timeout_s`
- `min_publish_period_sec` -> `min_publish_period_s`
- `republish_period_sec` -> `republish_period_s`
- `primary_path_timeout_sec` -> `primary_path_timeout_s`
- `goal_fallback_holdoff_sec` -> `goal_fallback_holdoff_s`
- `stale_path_timeout_sec` -> `stale_path_timeout_s`
- `path_goal_stamp_slack_sec` -> `path_goal_stamp_slack_s`
- `min_rebuild_period_sec` -> `min_rebuild_period_s`
- `stale_timeout_secs` -> `stale_timeouts_s`
- `min_publish_period_secs` -> `min_publish_periods_s`
- `republish_period_secs` -> `republish_periods_s`

### camrod_sensing (C++)

- `max_message_age_sec` -> `max_message_age_s`
- `input_max_ages_sec` -> `input_max_ages_s`

### camrod_platform (C++)

- `localization_pose_timeout_sec` -> `localization_pose_timeout_s`

### camrod_perception (C++)

- `marker_lifetime_sec` -> `marker_lifetime_s`

## 4) Config Alignment Updated

The following trees were updated to canonical keys:

- `camrod_system/config/diagnostics/default/**`
- `camrod_bringup/config/system/diagnostics/default/**`
- `camrod_localization/config/**`
- `camrod_map/config/**`
- `camrod_sensing/config/**`
- `camrod_platform/config/**`
- `camrod_perception/config/**`
- `camrod_bringup/config/localization/**`
- `camrod_bringup/config/map/**`
- `camrod_bringup/config/sensing/**`
- `camrod_bringup/config/platform/**`
- `camrod_bringup/config/perception/**`

And planning launch now passes canonical gate parameter key:

- `camrod_planning/launch/cmd_vel_gate.launch.py`

## 5) Remaining Legacy Aliases (Intentional)

The following legacy `_sec` keys are intentionally still declared as compatibility aliases:

- `goal_delay_sec`, `engage_delay_sec`, `test_duration_sec`
- `step_timeout_sec`, `wait_for_topics_sec`
- `cmd_vel_timeout_sec`, `startup_hold_sec`
- `gnss_failure_after_sec`, `gnss_recovery_after_sec`
- `startup_grace_sec`

Canonical runtime keys for new configs are:

- `goal_delay_s`, `engage_delay_s`, `test_duration_s`
- `step_timeout_s`, `wait_for_topics_s`
- `cmd_vel_timeout_s`, `startup_hold_s`
- `gnss_failure_after_s`, `gnss_recovery_after_s`
- `startup_grace_s`
