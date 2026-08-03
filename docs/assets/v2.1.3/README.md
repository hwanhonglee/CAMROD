# v2.1.3 simulation evidence index

<!-- HH_260804 - Give every release artifact one discoverable purpose and
     distinguish manual pre-owner probes from automatic-owner behavior. -->

This directory contains review visuals. Raw measurements are stored in the
matching `docs/evidence/v2.1.3` categories. All results are simulation or host
integration evidence, not real-robot FIELD-PASS.

## Reference frame

| Visual | Meaning | Source |
|---|---|---|
| [rear-axle-vs-robot-center-drive.gif](reference-frame/rear-axle-vs-robot-center-drive.gif) | Side-by-side replay showing the axle-midpoint reference reaching 0.8566 m farther before the first boundary hold | [summary JSON](../../evidence/v2.1.3/reference-frame/rear-axle-vs-robot-center-summary.json) |

## Boundary geometry

| Visual | Meaning | Source |
|---|---|---|
| [first-route-boundary-stop-location.png](boundary-geometry/first-route-boundary-stop-location.png) | Highlighted first deployed-footprint stop location | [route samples](../../evidence/v2.1.3/boundary-geometry/robot-center-route-samples.json) |
| [robot-center-narrow-route-risk-map.png](boundary-geometry/robot-center-narrow-route-risk-map.png) | Route sections where the planning rectangle cannot be placed safely | [planning sweep](../../evidence/v2.1.3/boundary-geometry/planning-footprint-envelope-sweep.json) |

Raw geometry sweeps compare the physical body, body plus 0.05 m per side, and
the deployed 0.10 m-per-side planning boundary under
[`docs/evidence/v2.1.3/boundary-geometry`](../../evidence/v2.1.3/boundary-geometry/).

## Boundary recovery

| Visual | Command ownership | Meaning |
|---|---|---|
| [pre-owner-manual-no-yaw.gif](boundary-recovery/pre-owner-manual-no-yaw.gif) | Manually injected candidates | Initial gate admission test with fixed-yaw display |
| [pre-owner-manual-yaw-aware.gif](boundary-recovery/pre-owner-manual-yaw-aware.gif) | Manually injected candidates | Normal route yaw plus manual reverse/right-crab candidate test |
| [pre-owner-robot-center-recovery.gif](boundary-recovery/pre-owner-robot-center-recovery.gif) | Manually injected candidates | Same gate experiment after moving the reference to `robot_center_link` |
| [pre-owner-robot-center-contact-sheet.png](boundary-recovery/pre-owner-robot-center-contact-sheet.png) | Manually injected candidates | Center-reference stop, reverse release, retry, and second hold |
| [automatic-owner-policy.png](boundary-recovery/automatic-owner-policy.png) | `route_safety_recovery_controller` | Current candidate-selection and bounded-motion policy |
| [automatic-owner-route-retry.gif](boundary-recovery/automatic-owner-route-retry.gif) | `route_safety_recovery_controller` | Current automatic crab/reverse and retained-goal retry timeline |
| [automatic-owner-route-retry-contact-sheet.png](boundary-recovery/automatic-owner-route-retry-contact-sheet.png) | `route_safety_recovery_controller` | Key automatic-owner milestones for static and moving route contact |

Raw timelines are under
[`docs/evidence/v2.1.3/boundary-recovery`](../../evidence/v2.1.3/boundary-recovery/).
The `pre-owner-*` prefix is a safety distinction: those runs prove gate
admission only and must not be cited as automatic command-generation evidence.

## UI

| Visual | Meaning | Source |
|---|---|---|
| [guest-mission-dispatch-ready.png](ui/guest-mission-dispatch-ready.png) | Guest UI stationary and above the shared 35% mission threshold | [lifecycle JSON](../../evidence/v2.1.3/ui/guest-mission-lifecycle.json) |
| [guest-route-safety-hold.png](ui/guest-route-safety-hold.png) | Service lifecycle retained while command safety hold is shown as an overlay | [lifecycle JSON](../../evidence/v2.1.3/ui/guest-mission-lifecycle.json) |
