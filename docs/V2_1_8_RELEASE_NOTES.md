# CAMROD v2.1.8 Release Notes

<!-- HH_260818 - Record the worak-test fast-forward, exact campsite motion,
bounded repeated recovery, semantic obstacle policy, and occupancy toggle. -->

## Scope

`v2.1.8` fast-forwards the previous `develop` baseline through all 20
`worak-test` commits, then applies the reviewed campsite, recovery, perception,
radar, configuration, and documentation changes described below. No
`develop`-only commit was discarded and the user-authored map OSM was not
rewritten.

## Runtime Changes

| Area | v2.1.8 behavior |
|---|---|
| Campsite crab | Entry remains pure `linear.y`; Ranger longitudinal/parallel mode changes hold translation until steering is within `0.05 rad`; exit removes lateral error before longitudinal drift |
| Boundary recovery | Retries projected crab/reverse/yaw stages instead of permanently holding after one attempt; each stage is limited to `0.40 m/10 s`, and the complete episode to 50 attempts, `1.50 m/90 s` with `0.5 s` zero pauses |
| Camera-LiDAR stop | `/perception/obstacles` accepts only current classified detections; unknown or LiDAR-only clusters cannot create the early stop; the final gate checks the active route's first `2.0 m` |
| Radar | FRONT1/FRONT2 and four side channels remain enabled with their measured self-return windows; rear channel 7 remains quarantined pending field measurement |
| Tent occupancy | `enable_campsite_occupancy_guard` controls both UI admission and the control start gate; default `false` preserves the current workflow, while `true` blocks pre-entry dispatch without interrupting committed site motion |
| Integrated contracts | Package/bringup GNSS uses `-90 deg` and a `0.5 s` fallback anchor; simulation applies the matching raw bias; rear-camera parking retains the current two-component rectify/AprilTag container |
| Active map | `map_version=22`, SHA `8fa13157b8e956559ad29b1bf49b4357ec6d252b0259debfb40a946b29f24e59`; 55 lanelets, 14 areas, 1,652 nodes, 236 ways |

## Verification

| Check | Result |
|---|---:|
| Canonical `colcon_build.sh` affected packages | 8 passed |
| Ranger steering transition | `1/1` passed |
| Control policy tests | `2/2` passed |
| Perception class filter and lint | `7/7` passed |
| Robot/Guest UI backend tests | `61/61` passed |
| Bringup launch/config/evidence contracts | `194/194` passed |
| Map-v22 B1 site entry/exit simulation | PASS through `CRAB_IN`, `ROTATE_180`, `CRAB_OUT`, and `DONE` |
| Fusion/radar directional gate matrix | PASS; fusion blocks route-front only, synthetic radar blocks all four commanded directions |

The full simulation evidence and its SHA manifest are under
[`worak-crab-fusion-safety-20260818`](assets/module-guides/control/test-results/worak-crab-fusion-safety-20260818/README.md).

## Field Acceptance Still Required

- Confirm physical wheel-angle settling before traction on the Ranger CAN
  platform.
- Measure camera-LiDAR calibration and classified-object stopping distance.
- Recheck front-radar false returns and decide whether rear channel 7 can be
  enabled.
- Validate the occupancy guard with real tent detections before changing its
  deployment default to `true`.
