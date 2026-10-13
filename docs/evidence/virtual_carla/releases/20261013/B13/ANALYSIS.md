# B13 attempt02 — verified return and reverse parking

<!-- HH_261012 - Finalized evidence under the same frozen v5 profile. -->

[trial.json](trial.json.gz) finalized `COMPLETED_OBSERVED`, `roundtrip_verified=true`:
actor **108**, operator-delivery generation **1791778373066001**, **2026-10-12
04:13:00.297–04:18:21.236 UTC**. Reverse parking ended `PARKED: station XY goal
reached`, charging false, final command zero, recorded collision events **0**.
[Supervisor](supervisor.json) finalized with probe/capture exit codes **0/0**.

| Recorded metric | Value |
| --- | ---: |
| Entire roundtrip | 320.968 s / 209.764 m |
| Outbound | 99.544 s / 38.980 m |
| Return including parking | 221.275 s / 170.785 m |
| Final localization XY error | **0.227488 m < unchanged 0.250 m** |
| Remaining parking margin | 0.022512 m |
| Live-lanelet handoff lateral / original-anchor error | 0.14 m / 0.40 m |

Travel is sampled physical-body XY distance, not UI wall-time-integrated
distance. Final localization `(-8.656821711,40.525917315)` is compared with the
unchanged station `(-8.47366,40.391)`.

## Parking geometry

The published snapped-approach residual reduced **0.198→0.045 m** before yaw
alignment. This residual is separate from the final station XY error.
Reconstructed station-axis lateral error changed from **−0.421260 m** before
correction to **−0.265277 m** at ALIGN, then **−0.307973 m** at reverse entry.
The sampled ALIGN-to-REVERSE physical body shift was **−0.045956 m laterally**.
Projection uses reverse-axis yaw −88.2127° and CARLA Y converted to ROS
convention; it is sampled reconstruction, not a continuous controller bound.

The original B13 failure (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_regression_20261008/B13/ANALYSIS.md`)
accepted about 0.192 m approach error and ended near **0.382 m** final XY error.
Today's adverse pivot shift is essentially comparable to that original run;
actual pre-turn correction removes about **0.156 m** lateral error. The
October 8 candidate (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_fixes_20261008/B13/ANALYSIS.md`) also passed at
0.228436 m. Neither result establishes a general robustness bound.

## Handoff and final approach

The bounded `camrod-sites-20261012-v5-camrod.service` journal from
04:12:08–04:19:17 UTC records:

- **04:14:43.748:** authenticated `done_roadside_forward_live_lanelet` handoff.
- **04:14:43.869:** `mode=one_way reverse_source=roadside_forward`, heading error **2.4°**, **24 lanelets / 168.99 m** planned route.
- **04:18:02.683:** POSITION target `(-8.757,42.404)`, residual **0.198 m**.
- **04:18:05.384:** exact approach point settled; ALIGN begins.
- **04:18:13.648:** reverse approach starts at station distance **2.079 m**, unchanged maximum reverse travel **5.000 m**.
- **04:18:20.805:** `PARKED: station XY goal reached`.

Between sampled DONE and parking-position correction (**103.548–302.284 s**),
all **853** road samples had gate `ENABLED`, command vx **0–1.111111 m/s**, and
**0 negative-vx samples**; sampled physical travel was **168.525 m**. Native
`roadside_stop`/forward-loop policy remains separate from local reverse parking.

The same [live profile](../archive/site_completion_20261012_v5/live_profile.json) and [frozen inputs](../archive/site_completion_20261012_v5/frozen_inputs.json)
apply: CARLA approach 0.05 m and PI enabled; native approach 0.20 m and final
goal 0.25 m are unchanged. Actual 5 fps capture artifacts are
[PNG](visual/representative_contact_sheet.png), [GIF](visual/representative_motion.gif),
and [PASS manifest](visual/capture_manifest.json); retained derivative hashes
were independently validated. The source MP4 was pruned after derivation.
Trial SHA-256: `d86dc31abbedb2176cf86238fa908f80a822417ae902012dd02fafe882d677a5`.

Together with [B11](../B11/ANALYSIS.md) and
[B12](../B12/ANALYSIS.md), all three formerly failing long-return
parking cases now have verified v5 closures at **0.221056 / 0.233476 / 0.227488 m**
without widening the final tolerance. These simulator attempts do not constitute
all-site, continuous-bound, charging, or real-robot acceptance.
