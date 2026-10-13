# B11 attempt02 — verified return and reverse parking

<!-- HH_261012 - Same frozen v5 profile; prior interrupted evidence is not reclassified. -->

[trial.json](trial.json.gz) finalized `COMPLETED_OBSERVED`, `roundtrip_verified=true`:
actor **76**, operator-delivery generation **1791777509670001**, **2026-10-12
03:58:37.672–04:04:00.943 UTC**. Native reverse parking ended `PARKED: station
XY goal reached` without charging. Recorded collision events were **0**.
[Supervisor](supervisor.json) probe/capture exit codes are **0/0**.

| Recorded metric | Value |
| --- | ---: |
| Entire roundtrip | 323.298 s / 209.620 m |
| Outbound | 107.750 s / 45.769 m |
| Return including parking | 215.404 s / 163.851 m |
| Final localization XY error | **0.221056 m < unchanged 0.250 m** |
| Remaining parking margin | 0.028944 m |
| Live-lanelet handoff lateral / original-anchor error | 0.15 m / 0.41 m |

Travel is sampled physical-body XY distance, not UI wall-time-integrated
distance. Final localization `(-8.657394655,40.513911078)` is compared with the
unchanged station `(-8.47366,40.391)`.

## Parking geometry

The published snapped-approach residual reduced **0.190→0.043 m** before yaw
alignment. This is separate from final station XY error. Reconstructed
station-axis lateral error changed from **−0.410300 m** before correction to
**−0.263267 m** at ALIGN, then **−0.305672 m** at reverse entry. The sampled
ALIGN-to-REVERSE physical body shift was **−0.045069 m laterally**. Projection
uses reverse-axis yaw −88.2127° and CARLA Y converted to ROS convention; it is
sample-derived evidence, not a continuous controller measurement.

The original B11 failure (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_regression_20261008/B11/ANALYSIS.md`)
accepted about 0.190 m approach error and ended near **0.372 m** final XY error.
Today's adverse pivot shift is comparable to that original run, while actual
pre-turn position correction removes about **0.147 m** lateral error. The
October 8 candidate (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_fixes_20261008/B11/ANALYSIS.md`) also passed at
0.205813 m; this run is not claimed to improve that earlier precision.

## Handoff and final approach

The bounded `camrod-sites-20261012-v5-camrod.service` journal from
03:58:37–04:04:02 UTC records:

- **04:00:32.420:** authenticated `done_roadside_forward_live_lanelet` handoff.
- **04:00:32.546:** `mode=one_way reverse_source=roadside_forward`, heading error **6.6°**, **23 lanelets / 162.02 m** planned route.
- **04:03:42.837:** POSITION starts at target `(-8.757,42.404)`, residual **0.190 m**.
- **04:03:45.269:** exact approach point settled; ALIGN begins.
- **04:03:53.399:** reverse approach begins at station distance **2.079 m**, unchanged maximum reverse travel **5.000 m**.
- **04:04:00.609:** `PARKED: station XY goal reached`.

Between sampled DONE and parking-position correction (**115.009–305.003 s**),
all **816** road samples had gate `ENABLED`, command vx **0–1.111111 m/s**, and
**0 negative-vx samples**; sampled physical travel was **161.572 m**. Native
`roadside_stop`/forward-loop behavior remains separate from local reverse parking.

The same [live profile](../archive/site_completion_20261012_v5/live_profile.json) and [frozen inputs](../archive/site_completion_20261012_v5/frozen_inputs.json)
apply: CARLA approach 0.05 m, PI enabled; native approach 0.20 m and final goal
0.25 m are unchanged. Actual 5 fps capture artifacts are [PNG](visual/representative_contact_sheet.png),
[GIF](visual/representative_motion.gif), and [PASS manifest](visual/capture_manifest.json).
The source MP4 was pruned after derivation. Trial SHA-256:
`a1d008b4970676646a046609ce3af8f9f0916510d0cf84d44a40efa3fb73e1a3`.

The [earlier interrupted v5 attempt](../archive/site_completion_20261012_v5/B11/ANALYSIS.md) remains incomplete;
its SIGTERM origin is unproved. This successful simulator attempt does not
constitute all-site, continuous-bound, charging, or real-robot acceptance.
