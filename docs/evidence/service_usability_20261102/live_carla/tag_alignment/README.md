# CARLA-only station tag alignment — HH_261102

The V224 tag center maps to ROS `(-11.220385, 39.080170)`. At the authored
dz7144 station `(-8.47366, 40.391)`, body yaw `+91.7873°`, its bearing is
approximately `77.75°` from the rear camera axis, outside the 90° camera HFOV.
The actual before-images are retained in the sibling `tag_alignment_before/`.

This tool clones the accepted V224 map to `Woraksan_v27_service_tag_20261102`.
It reuses the existing exact-46-tag stage/invariant helpers. Only the tag's 46
rigid locations/rotations change: approximately +6° UE yaw (−6° ROS yaw), then
translation to the authored station's rear camera axis. The original map is
hash-pinned, its source disk contracts reloaded, all non-tag actor/component
contracts preserved, referenced mesh/material bytes protected, and sidecars
copied unchanged. No terrain, engine, vehicle, tag ID, size, materials, scale,
collision, tilt, detector threshold, or robot station is changed.

Target tag ROS XY is `(-8.441868, 39.372166)`, 0.4 m along the rear optical
axis from camera XY `(-8.454344, 39.771971)`. The absolute tag Z stays
`−0.636465912 m`; camera Z depends on the settled body pose. Consequently 0.4 m
is optical/horizontal depth, not a claimed exact Euclidean distance. The tag
remains tag36h11 ID 3, whole white plane 0.20 m, black-border outer size 0.16 m,
and normal tilt +10° toward world up.

PLAN is inert except for its new evidence report. APPLY requires a fresh
target namespace, fresh report, the exact token, and the existing UE editor.
Wait for the root agent to stop the owned runtime/server before APPLY. Do not
use `EditorLevelLibrary` against the live `-game` world: current remote
discovery found no Python node; CARLA's environment-object API cannot move
these baked static planes.

```bash
export RANGER_CARLA_ROOT=/home/hong/Downloads/ranger-carla-4ws-pipeline
export CARLA_ROOT=/home/hong/carla
export RANGER_WORAKSAN_V224_DROPZONE_PROJECT_ROOT=$CARLA_ROOT/Unreal/CarlaUE4
export RANGER_SERVICE_TAG_MODE=PLAN
unset RANGER_SERVICE_TAG_APPLY
python3 /home/hong/camrod_service_test_ws/src/docs/evidence/service_usability_20261102/live_carla/tag_alignment/stage_tag_alignment.py

# Only after root confirms the owned world/server is stopped:
export RANGER_SERVICE_TAG_MODE=APPLY
export RANGER_SERVICE_TAG_APPLY=APPLY_CLONED_EXACT46_DZ7144_TAG_ALIGNMENT
/home/hong/UnrealEngine_4.26/Engine/Binaries/Linux/UE4Editor \
  "$CARLA_ROOT/Unreal/CarlaUE4/CarlaUE4.uproject" \
  -NullRHI -unattended -nop4 -nosplash -nosound -stdout \
  -ExecutePythonScript=/home/hong/camrod_service_test_ws/src/docs/evidence/service_usability_20261102/live_carla/tag_alignment/stage_tag_alignment.py
```

The new map requires its explicit new URI and honest runtime map selection;
it is not the old V224 scene. An APPLIED report alone is not camera proof:
retain an actual rear raw/rectified PNG and detector debug/ID3 pose at the
parked station, then test physical docking/charging separately.

## Applied result

The scoped editor operation completed successfully and exited 0. The new map
SHA-256 is `0a8b39d286ae61607ce98e88401e33d5b9220551212219d1b21baff23171df9e`.
All 522 protected source/shared files remained unchanged; exact46 tag and
non-tag contracts passed both in-memory and disk-reload checks. Evidence is
`/home/hong/Downloads/ranger-carla-4ws-pipeline/.work/evidence/service_usability_tag_alignment_20261102/apply.json`.
This staging result does not itself establish actual camera detection.
