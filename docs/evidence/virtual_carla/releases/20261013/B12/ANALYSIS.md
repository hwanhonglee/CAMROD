# B12 attempt02 — verified return and reverse parking

<!-- HH_261012 - Finalized evidence under the same frozen v5 profile. -->

[trial.json](trial.json.gz) finalized `COMPLETED_OBSERVED`, `roundtrip_verified=true`:
actor **92**, operator-delivery generation **1791777942795001**, **2026-10-12
04:05:50.689–04:11:12.315 UTC**. Reverse parking ended `PARKED: station XY goal
reached`, charging false, final command zero, recorded collision events **0**.
[Supervisor](supervisor.json) finalized with probe/capture exit codes **0/0**.

| Recorded metric | Value |
| --- | ---: |
| Entire roundtrip | 321.670 s / 209.833 m |
| Outbound | 106.426 s / 43.756 m |
| Return including parking | 215.082 s / 166.077 m |
| Final localization XY error | **0.233476 m < unchanged 0.250 m** |
| Remaining parking margin | 0.016524 m |
| Live-lanelet handoff lateral / original-anchor error | 0.13 m / 0.37 m |

Travel is sampled physical-body XY distance, not UI wall-time-integrated
distance. Final localization `(-8.657851593,40.534473082)` is compared with the
unchanged station `(-8.47366,40.391)`.

## Parking geometry

The published snapped-approach residual reduced **0.180→0.037 m** before yaw
alignment. This residual is not the final station XY error. Reconstructed
station-axis lateral error changed from **−0.402552 m** before correction to
**−0.256828 m** at ALIGN, then **−0.301314 m** at reverse entry. The sampled
ALIGN-to-REVERSE physical body shift was **−0.045331 m laterally**. Projection
uses reverse-axis yaw −88.2127° and CARLA Y converted to ROS convention; this
is sampled reconstruction, not a continuously measured controller bound.

The original B12 failure (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_regression_20261008/B12_attempt02/ANALYSIS.md`)
accepted about 0.197 m approach error and ended near **0.367 m** final XY error.
Here, actual pre-turn correction removed about **0.146 m** lateral error and
parking succeeded despite the adverse pivot shift. The
October 8 candidate (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_fixes_20261008/B12/ANALYSIS.md`) passed at
0.217279 m; this run does not demonstrate improved precision over that pass.
The remaining 16.5 mm margin is modest and is not a general robustness bound.

## Handoff and final approach

The bounded `camrod-sites-20261012-v5-camrod.service` journal from
04:05:50–04:11:14 UTC records:

- **04:07:41.197:** authenticated `done_roadside_forward_live_lanelet` handoff.
- **04:07:41.318:** `mode=one_way reverse_source=roadside_forward`, heading error **4.1°**, **23 lanelets / 164.28 m** planned route.
- **04:10:54.459:** POSITION target `(-8.757,42.404)`, residual **0.180 m**.
- **04:10:56.630:** exact approach point settled; ALIGN begins.
- **04:11:04.845:** reverse approach starts at station distance **2.080 m**, unchanged maximum reverse travel **5.000 m**.
- **04:11:11.955:** `PARKED: station XY goal reached`.

Between sampled DONE and parking-position correction (**110.587–303.672 s**),
all **831** road samples had gate `ENABLED`, command vx **0–1.111111 m/s**, and
**0 negative-vx samples**; sampled physical travel was **163.839 m**. Native
`roadside_stop`/forward-loop policy remains separate from local reverse parking.

The same [live profile](../archive/site_completion_20261012_v5/live_profile.json) and [frozen inputs](../archive/site_completion_20261012_v5/frozen_inputs.json)
apply: CARLA approach 0.05 m and PI enabled; native approach 0.20 m and final
goal 0.25 m are unchanged. Actual 5 fps capture artifacts are
[PNG](visual/representative_contact_sheet.png), [GIF](visual/representative_motion.gif),
and [PASS manifest](visual/capture_manifest.json); retained derivative hashes
were independently validated. The source MP4 was pruned after derivation.
The inspected contact sheet shows B12 selection, outbound/return UI states,
and the final stationary robot at the drop zone with the UI back at its menu;
the precise parking result above comes from telemetry, not image estimation.
Trial SHA-256: `4abf5781878a38066418d885d7e44b56b8d533437d0558e8ec6f5b6c788ae08c`.

This successful simulator attempt does not constitute continuous-bound,
charging, or real-robot acceptance.
