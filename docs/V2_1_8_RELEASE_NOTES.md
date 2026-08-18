# CAMROD v2.1.8 Release Notes

<!-- HH_260818 - Record the worak-test fast-forward, exact campsite motion,
bounded repeated recovery, semantic obstacle policy, and occupancy toggle. -->
<!-- HH_260818 - Extend the tag with normal/crab separation, one-anchor site
return, restart heading recovery, manual Return, and docking observability. -->

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
| Normal driving | A `0.02 m/s` lateral deadband prevents normal Nav2 residue from selecting parallel mode; explicit campsite/recovery lateral commands still select crab |
| Campsite anchor/restart | `CRAB_IN` and `CRAB_OUT` use one snap anchor; current heading selects the body crab side after a 180-degree restart; reboot near the site restores `WAIT_RETURN` |
| Boundary recovery | Retries projected crab/reverse/yaw stages instead of permanently holding after one attempt; each stage is limited to `0.40 m/10 s`, and the complete episode to 50 attempts, `1.50 m/90 s` with `0.5 s` zero pauses |
| Camera-LiDAR stop | `/perception/obstacles` accepts only current classified detections; unknown or LiDAR-only clusters cannot create the early stop; the final gate checks the active route's first `2.0 m` |
| Radar | FRONT1/FRONT2 and four side channels remain enabled with their measured self-return windows; rear channel 7 remains quarantined pending field measurement |
| Tent occupancy | `enable_campsite_occupancy_guard` controls both UI admission and the control start gate; default `false` preserves the current workflow, while `true` blocks pre-entry dispatch without interrupting committed site motion |
| Manual Return | Robot UI can request return from any active service; it preserves campsite crab-out ownership, aligns before parking at the drop zone, and does not move an already charging robot |
| Docking/parking | UI exposes tag debug/pose/detection, paths, controller states, battery, and charging; reverse slows over the final `0.30 m`, AprilTag over `0.60 m`, and charging immediately commands zero |
| Integrated contracts | Package/bringup GNSS uses `-90 deg` and a `0.5 s` fallback anchor; simulation applies the matching raw bias; rear-camera parking retains the current two-component rectify/AprilTag container |
| Active map | `map_version=22`, SHA `8fa13157b8e956559ad29b1bf49b4357ec6d252b0259debfb40a946b29f24e59`; 55 lanelets, 14 areas, 1,652 nodes, 236 ways |

## Verification

| Check | Result |
|---|---:|
| Canonical `colcon_build.sh` affected packages | 5 passed |
| Ranger motion policy GTest | `9/9` passed |
| Control policy/package result | 83 policy cases; `85` xUnit, zero failures |
| Platform package result | `36` xUnit, zero failures |
| Robot/Guest UI backend tests | `69/69` passed |
| Bringup launch/config/evidence contracts | 29 CTest targets; `265` xUnit, zero failures |
| Map-v22 B1 site entry/exit simulation | PASS through `CRAB_IN`, `ROTATE_180`, `CRAB_OUT`, and `DONE` |
| Current B8 same-anchor simulation | PASS through `CRAB_IN -> ROTATE_180 -> UNLOAD_WAIT -> WAIT_RETURN -> ALIGN_RETRACE_YAW -> CRAB_OUT -> DONE`; IN/OUT paths `2/2`, mixed-axis commands `0` |
| Normal route before B8 | PASS; `3.73 m`, maximum `linear.y=0.000 m/s`, no crab-mode command |
| Manual Return API / docking telemetry | PASS; site action `site_exit_then_return`, no drop-zone plan before `DONE`, schema v3, seven selected-view subscriptions |
| Parking speed/restart geometry policies | Focused C++ and Python tests PASS |
| Fusion/radar directional gate matrix | PASS; fusion blocks route-front only, synthetic radar blocks all four commanded directions |

The vendored `ranger_base` functional GTest is green. Its full upstream lint
bundle still reports the pre-existing copyright/style/package.xml debt across
the vendor tree; those six lint targets are not presented as functional test
failures and were not mass-reformatted in this release.

The full simulation evidence and its SHA manifest are under
[`worak-crab-fusion-safety-20260818`](assets/module-guides/control/test-results/worak-crab-fusion-safety-20260818/README.md).

The same-anchor B8 JSON, generated PNG/GIF, parking profile, production UI
capture, and SHA manifests are under
[`b8-return-docking-20260819`](assets/module-guides/bringup/test-results/b8-return-docking-20260819/README.md).

## Field Acceptance Still Required

- Confirm physical wheel-angle settling before traction on the Ranger CAN
  platform.
- Measure camera-LiDAR calibration and classified-object stopping distance.
- Recheck front-radar false returns and decide whether rear channel 7 can be
  enabled.
- Validate the occupancy guard with real tent detections before changing its
  deployment default to `true`.
- Measure the `0.60/0.30 m` final-approach windows against the physical tag,
  station geometry, CAN charger contact, and braking delay.
- Re-run the reversed-heading campsite restart and manual Return on B1-B10 with
  the physical steering and payload envelope.
