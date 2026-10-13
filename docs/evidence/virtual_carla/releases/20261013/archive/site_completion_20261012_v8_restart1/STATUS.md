# B1–B13 diagnostic status — 2026-10-12

<!-- HH_261012 - Generated from trial/supervisor files; source evidence is never changed. -->

Updated UTC: 2026-10-12T06:54:46.090463+00:00

Verified current-attempt observed roundtrips: 0/13 unique sites. Formal acceptance: false.

Recognized attempt directories: 1. Attempt statuses: {'FAILED': 1}. Earlier attempts are not extra sites.

| Site | Status | Roundtrip verified | Stage / site phase | Observed s / m | Outbound s / m | Return s / m | Assets |
| --- | --- | --- | --- | --- | --- | --- | --- |
| B1 | NOT_STARTED | false | — / — | — / — | — / — | — / — | — |
| B2 | NOT_STARTED | false | — / — | — / — | — / — | — / — | — |
| B3 | NOT_STARTED | false | — / — | — / — | — / — | — / — | — |
| B4 | NOT_STARTED | false | — / — | — / — | — / — | — / — | — |
| B5 | NOT_STARTED | false | — / — | — / — | — / — | — / — | — |
| B6 | NOT_STARTED | false | — / — | — / — | — / — | — / — | — |
| B7 | NOT_STARTED | false | — / — | — / — | — / — | — / — | — |
| B8 | NOT_STARTED | false | — / — | — / — | — / — | — / — | — |
| B9 | NOT_STARTED | false | — / — | — / — | — / — | — / — | — |
| B10 | FAILED | false | return / ERROR | 190.25 / 54.22 | 137.78 / 52.91 | — / — | PNG (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v8_restart1/B10/visual/representative_contact_sheet.png`), GIF (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v8_restart1/B10/visual/representative_motion.gif`) |
| B11 | NOT_STARTED | false | — / — | — / — | — / — | — / — | — |
| B12 | NOT_STARTED | false | — / — | — / — | — / — | — / — | — |
| B13 | NOT_STARTED | false | — / — | — / — | — / — | — / — | — |

## Recorded diagnostics

### B10

Trial: FINALIZED_RESULT (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v8_restart1/B10/trial.json`).

Recorded status: FAILED

Error: camping-site controller ERROR: phase=ERROR site=camping_site_10 service_mode=turnaround return_route_policy=forward_loop site_forward_return_active=false site_forward_return_centering=false site_forward_return_recenter_count=0 site_forward_return_recenter_limit=10 site_forward_return_centering_tolerance_m=0.010 site_forward_return_recenter_trigger_m=0.025 site_forward_return_steering_settle_s=1.200 roadside_return=turnaround recall_turnaround_return=False battery_urgent_return=False return_source= crab_speed_mps=0.67 crab_command_mps=0.00 anchor_centering=inactive phase_wall_elapsed_s=0.0 current_yaw_deg=-3.87 target_yaw_deg=-69.18 yaw_error_deg=-65.32 yaw_rate_degps=0.00 last_angular_cmd_radps=0.000 crab_out_yaw_recovery_active=false crab_body_yaw_alignment=inactive crab_body_yaw_compensation_state=active_automatic_turnaround detail=forward_loop recenter recovery limit reached: 10/10; immutable turn guard retained return_published=False return_ack=False

Exception: MatrixError

Site message: phase=ERROR site=camping_site_10 service_mode=turnaround return_route_policy=forward_loop site_forward_return_active=false site_forward_return_centering=false site_forward_return_recenter_count=0 site_forward_return_recenter_limit=10 site_forward_return_centering_tolerance_m=0.010 site_forward_return_recenter_trigger_m=0.025 site_forward_return_steering_settle_s=1.200 roadside_return=turnaround recall_turnaround_return=False battery_urgent_return=False return_source= crab_speed_mps=0.67 crab_command_mps=0.00 anchor_centering=inactive phase_wall_elapsed_s=0.0 current_yaw_deg=-3.87 target_yaw_deg=-69.18 yaw_error_deg=-65.32 yaw_rate_degps=0.00 last_angular_cmd_radps=0.000 crab_out_yaw_recovery_active=false crab_body_yaw_alignment=inactive crab_body_yaw_compensation_state=active_automatic_turnaround detail=forward_loop recenter recovery limit reached: 10/10; immutable turn guard retained return_published=False return_ack=False

Gate state: ENABLED

Gate message: state=ENABLED reasons=none charging=false charging_departure_auth=none drop_zone_auth_phase=none drop_zone_auth_age_s=-1.00 manual_departure_remaining_s=0.00 battery=80.0% route_clear_s=0.00 route_releases=0/50 route_episode_progress_m=0.000 recovery_motion_observed=false recovery_candidate=none recovery_reason=route_hold_inactive

Descriptive tags: NONZERO_PROCESS_EXIT_RECORDED, MISSION_PHASE_ERROR_OBSERVED

Supervisor (recorded):

```json
{
  "site": "B10",
  "started_utc": "2026-10-12T06:51:02.249343+00:00",
  "status": "finished",
  "diagnostic_stop_reason": null,
  "actor_id": 12,
  "trial_exit_code": 1,
  "capture_exit_code": 0,
  "result_status": "FAILED",
  "finished_utc": "2026-10-12T06:54:45.774758+00:00",
  "fixed_profile": {
    "yaw_pi": true,
    "parking_approach_tolerance_m": 0.05,
    "basis": "configured profile, not a live parameter read",
    "freeze_manifest": "/home/hong/camrod_ws/src/docs/evidence/virtual_carla/site_completion_20261012_v8_restart1/frozen_inputs.json",
    "old_evidence_modified": false
  }
}
```

## Limits

- File-only snapshot; RUNNING means an open/incomplete trial, not verified process liveness.
- SETUP_ERROR means a supervisor infrastructure error without trial.json, not a failed site mission.
- Historical preflight incidents remain linked separately and do not override a later trial outcome.
- Attempt count counts existing recognized attempt directories, including setup-only or not-yet-started folders; preflight incidents are not extra attempts.
- Earlier attempt statuses, errors, evidence links, and complete finalized retry-history results remain separate; later success never changes an earlier failure.
- No success is inferred from a phase, screenshot, absent error, or supervisor exit alone.
- Elapsed times and distances are copied only from recorded fields; null means not recorded.
- Sampled physical-body XY distance is secondary diagnostic evidence, not certified distance.
- Error tags are descriptive, not root-cause findings. Original messages are preserved.
- A completed mission does not validate capture provenance; asset links mean files exist only.
- No guest/recall, physical-vehicle, historical-run, or deployment acceptance is claimed.
- The highest explicit attempt number is displayed; every previous result remains unchanged.
