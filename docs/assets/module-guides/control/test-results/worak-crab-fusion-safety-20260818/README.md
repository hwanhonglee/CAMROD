# Worak Crab And Semantic Safety Simulation

<!-- HH_260818 - Preserve the raw AMD64 simulation boundary for the exact
camping-site crab, repeated route recovery, and semantic/radar stop changes. -->

## Scope

| Item | Value |
|---|---|
| Date | 2026-08-18 |
| Platform | AMD64 ROS 2 simulation |
| Active map | `map_version=22`, SHA `8fa13157b8e956559ad29b1bf49b4357ec6d252b0259debfb40a946b29f24e59` |
| Field claim | `false` |

## Results

| Record | Result | Meaning |
|---|---|---|
| `b1-site-entry-exit.json` | PASS | B1 completed `CRAB_IN -> ROTATE_180 -> UNLOAD_WAIT -> WAIT_RETURN -> ALIGN_RETRACE_YAW -> CRAB_OUT -> DONE` |
| `gate-matrix.json` | PASS | Classified-fusion fixture blocked forward and passed crab/reverse; radar fixtures blocked front/left/right/rear; both grids ran at 10 Hz |
| `b1-full-return-timeout.json` | PARTIAL | Site entry/exit and return dispatch succeeded; the 180 s runner ended 20.05 m into the approximately 80 m drop-zone return, before parking |

The gate matrix publishes directly to the post-fusion obstacle topic, so the
class-label admission rule is verified separately by
`camrod_perception/test/test_classified_detection.cpp`. The radar directions
are synthetic policy fixtures; production FRONT1/FRONT2 are enabled, while
REAR/channel 7 remains quarantined in the `worak-test` hardware profile.

Physical Ranger wheel-angle settling, camera-LiDAR calibration, and radar
false-return acceptance require supervised field tests.
