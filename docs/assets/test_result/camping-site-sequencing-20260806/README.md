# Campsite Maneuver Sequencing - 2026-08-06

<!-- HH_260806 - Separate validated turnaround behavior from the unresolved
constrained-roadside return geometry. -->

![Campsite policy validation](campsite-policy-validation.png)

![Campsite phase sequence](campsite-phase-sequence.gif)

## Scope

| Sites | Active policy | Runtime validation | Result |
|---|---|---|---|
| B1-B10 | Full lateral entry, 180-degree zero-turn inside the campsite, unload wait, explicit RETURN, lateral exit | All ten sites ran their map-derived lateral distance through the complete controller round trip. | PASS 10/10 (code/config/sim) |
| B11-B13 | At most `0.60 m` roadside crab, unload wait, no on-site zero-turn | Each site ran only through `WAIT_RETURN`; the test deliberately issued no RETURN. | PASS (arrival only) |
| B11-B13 return | Undecided pending safe return geometry | An exploratory B11 RETURN reached the lane snap pose, then `lanelet_physical_body_cost` blocked the earlier on-lane 180-degree alignment. | FIELD-PENDING; not a PASS |

The active map is revision 16. Both deployed OSM copies have SHA-256
`fd9c1855573784e4e4e952f931c87e3b2c2858fa20f9c8ae5c2ad9adfc32d0cf`.
Both active campsite YAML copies have SHA-256
`0b4b288a553eae562301eb06f79d5b46d8a9f4e6a2c04fd34105b493c41150ce`.

## Observed Sequences

All B1-B10 reports directly observed the same ordered sequence:

`CRAB_IN -> ROTATE_180 -> UNLOAD_WAIT -> WAIT_RETURN -> ALIGN_RETRACE_YAW -> CRAB_OUT -> DONE`

| Report | Map-derived lateral entry | Observed phases | Final command maximum | Overall |
|---|---:|---:|---:|---|
| `b1-turnaround.json` | `5.22 m` | `7/7` | `0.175` | PASS |
| `b2-turnaround.json` | `2.16 m` | `7/7` | `0.175` | PASS |
| `b3-turnaround.json` | `5.26 m` | `7/7` | `0.175` | PASS |
| `b4-turnaround.json` | `1.95 m` | `7/7` | `0.175` | PASS |
| `b5-turnaround.json` | `5.25 m` | `7/7` | `0.175` | PASS |
| `b6-turnaround.json` | `1.91 m` | `7/7` | `0.175` | PASS |
| `b7-turnaround.json` | `5.23 m` | `7/7` | `0.175` | PASS |
| `b8-turnaround.json` | `1.79 m` | `7/7` | `0.175` | PASS |
| `b9-turnaround.json` | `5.31 m` | `7/7` | `0.175` | PASS |
| `b10-turnaround.json` | `1.80 m` | `7/7` | `0.175` | PASS |
| `b11-roadside-arrival.json` | `0.60 m cap` | `3/3` | `0.120` | PASS |
| `b12-roadside-arrival.json` | `0.60 m cap` | `3/3` | `0.120` | PASS |
| `b13-roadside-arrival.json` | `0.60 m cap` | `3/3` | `0.120` | PASS |

B11-B13 directly observed `CRAB_IN -> UNLOAD_WAIT -> WAIT_RETURN` and ended
before any RETURN command.

`campsite-policy-summary.json` is the machine-readable 13-site aggregate used
to render the PNG and GIF. Every B1-B10 report uses the same ordered-sequence
field; no turnaround sequence is inferred from unordered flags.

## Safety Contract

- Explicit campsite phases exclusively own final `cmd_vel`; Nav2 commands are ignored until the maneuver ends.
- The gate publishes zero for `0.5 s` at maneuver handoff and between Nav2 pure rotation and first translation.
- Static road-lanelet cost is bypassed only during configured campsite motion phases because campsite areas are outside road lanelets.
- Dynamic LiDAR/radar obstacle checks remain active during that map-only bypass.
- Physical-body lanelet contact remains a hard stop during ordinary navigation and the unresolved B11-B13 on-lane return alignment.
- LiDAR cost-grid was intentionally OFF for these runs. The validation runner omitted only that explicitly optional baseline rate; radar, inflation, filtered points, localization, wheel odometry, and IMU rates remained required.

## Reproduction

Launch full simulation on an isolated ROS domain, then run the validation runner
with `run_camping:=true`. Use `camping_stop_at_wait_return:=true` only for
B11-B13 arrival checks. Regenerate the visual files with:

```bash
python3 camrod_bringup/scripts/render_camping_site_sequence_results.py \
  --result-dir docs/assets/test_result/camping-site-sequencing-20260806 \
  --camping-sites camrod_planning/config/camping_sites.yaml
```

These results validate software sequencing on AMD64 simulation. They do not
approve B11-B13 return geometry or replace a controlled real-robot test.
