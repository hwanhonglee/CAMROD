# CAMROD v2.2.2 Release Notes

## Scope

`v2.2.2` advances the CARLA-free `v2.2.1` field baseline by extending only the
AprilTag initial/reacquisition wait. GNSS, campsite return, charging departure,
radar policy, and post-latch yaw alignment are unchanged.

## Runtime Change

| Item | v2.2.2 behavior |
|---|---|
| Tag freshness safety stop | Unchanged at `0.5 s`; stale input commands zero |
| Initial/reacquisition wait | `tag_wait_timeout_s: 10.0 -> 60.0 s` |
| Valid target returns | `WAITING_FOR_TAG -> TAG_GUIDED_REVERSE` and parking resumes |
| Wait expires | Controller enters terminal `ERROR`; a new parking START is required |
| Translation already latched at `0.40 m` | Final yaw remains odometry-driven and does not re-enter Tag waiting |

The `60.0 s` budget starts on each `WAITING_FOR_TAG` entry. Existing fail-closed
odometry behavior is retained: stale odometry keeps zero command, and timeout
evaluation resumes when odometry is fresh again.

## Owned Values

- `camrod_control/src/apriltag_parking_controller_node.cpp`: standalone default
- `camrod_control/config/parking.yaml`: package runtime configuration
- `camrod_bringup/config/control/parking.yaml`: deployment mirror

All three use `tag_wait_timeout_s=60.0`; `tag_timeout_s=0.5` remains unchanged.

## Field Acceptance Still Required

- Occlude target ID 3 for more than `0.5 s` and verify continuous zero command.
- Restore a valid target before `60 s` and verify one safe guided-reverse resume.
- Keep the target absent past `60 s` and verify terminal `ERROR` with no motion.
- Repeat with a wrong tag ID and failed reprojection validation; neither may
  reset the wait or resume motion.
