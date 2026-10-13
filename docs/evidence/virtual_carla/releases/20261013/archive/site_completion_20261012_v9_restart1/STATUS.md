# B1–B13 diagnostic status — 2026-10-12

<!-- HH_261012 - Generated from trial/supervisor files; source evidence is never changed. -->

Updated UTC: 2026-10-12T08:02:32.495164+00:00

Verified current-attempt observed roundtrips: 1/13 unique sites. Formal acceptance: false.

Recognized attempt directories: 1. Attempt statuses: {'COMPLETED_OBSERVED': 1}. Earlier attempts are not extra sites.

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
| B10 | COMPLETED_OBSERVED | true | return / DONE | 409.82 / 217.43 | 137.41 / 52.74 | 272.26 / 164.68 | [PNG](../../B10/visual/representative_contact_sheet.png), [GIF](../../B10/visual/representative_motion.gif) |
| B11 | NOT_STARTED | false | — / — | — / — | — / — | — / — | — |
| B12 | NOT_STARTED | false | — / — | — / — | — / — | — / — | — |
| B13 | NOT_STARTED | false | — / — | — / — | — / — | — / — | — |

## Recorded diagnostics

### B10

Trial: [FINALIZED_RESULT](../../B10/trial.json.gz).

Recorded status: COMPLETED_OBSERVED

Site message: phase=DONE site=camping_site_10 service_mode=turnaround return_route_policy=forward_loop site_forward_return_active=true site_forward_return_centering=false site_forward_return_recenter_count=13 site_forward_return_recenter_limit=20 site_forward_return_centering_tolerance_m=0.015 site_forward_return_recenter_trigger_m=0.035 site_forward_return_steering_settle_s=0.400 roadside_return=turnaround recall_turnaround_return=False battery_urgent_return=False return_source= crab_speed_mps=0.67 crab_command_mps=0.00 anchor_centering=inactive phase_wall_elapsed_s=203.7 current_yaw_deg=95.49 target_yaw_deg=-69.18 yaw_error_deg=-164.68 yaw_rate_degps=0.00 last_angular_cmd_radps=0.000 crab_out_yaw_recovery_active=false crab_body_yaw_alignment=inactive crab_body_yaw_compensation_state=active_automatic_turnaround detail=live lanelet handoff current=(11.82,17.81) projection=(11.92,17.85) lateral_error=0.11m original_anchor_error=0.27m return_published=True return_ack=True

Gate state: STANDBY

Gate message: state=STANDBY reasons=engage=false(manual=false,mission=false),platform_drive_enable=false charging=false charging_departure_auth=none drop_zone_auth_phase=none drop_zone_auth_age_s=-1.00 manual_departure_remaining_s=0.00 battery=80.0% route_clear_s=0.00 route_releases=0/50 route_episode_progress_m=0.000 recovery_motion_observed=false recovery_candidate=none recovery_reason=route_hold_inactive

Supervisor (recorded):

```json
{
  "site": "B10",
  "started_utc": "2026-10-12T07:54:34.771651+00:00",
  "status": "finished",
  "diagnostic_stop_reason": null,
  "actor_id": 12,
  "trial_exit_code": 0,
  "capture_exit_code": 0,
  "result_status": "COMPLETED_OBSERVED",
  "finished_utc": "2026-10-12T08:02:32.049438+00:00",
  "fixed_profile": {
    "yaw_pi": true,
    "parking_approach_tolerance_m": 0.05,
    "basis": "configured profile, not a live parameter read",
    "freeze_manifest": "/home/hong/camrod_ws/src/docs/evidence/virtual_carla/site_completion_20261012_v9_restart1/frozen_inputs.json",
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
