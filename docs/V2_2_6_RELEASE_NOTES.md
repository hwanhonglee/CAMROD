# CAMROD v2.2.6

<!-- HH_260911 - Separate common algorithm changes from CARLA adapters and test evidence. -->

## Source and branch boundaries

The common baseline is merged `worak-test` commit
`914c3ed1d64f75ae3893e754f39078193421691f` (PR #1).
The local `develop` worktree was updated in place; its branch was not renamed
or checked out as `worak-test`. CARLA-specific adapters remain in `virtual/carla`.
This release is prepared locally. No push or GitHub release publication is performed.

## Common algorithm and UI corrections

- Bind delayed movement to the current mission and command epoch. Stop,
  cancellation, mission clearing, and node shutdown invalidate pending voice work.
- Coalesce identical announcement requests without extending their deadline.
  A newer request waits instead of executing immediately behind an older request.
- Treat PLAYING followed by IDLE as playback confirmation. QUEUED and ERROR
  are not successful completion. The existing bounded fail-open timeout remains.
- Use the common dispatch helpers consistently in runtime and isolated tests.
  A missing optional site catalogue does not break a lightweight backend fixture.
- Admit explicit docking from OPERATOR_STOPPED only with fresh map-frame
  localization strictly inside the authored drop-zone polygon.
- Route explicit charging through the existing serialized owner cancellation,
  CAN/re-arm hold, drop-zone alignment, and parking dispatcher sequence.
  Preserve `force_docking` through that sequence; an HTTP acknowledgement is
  never reported as observed charging or as the selected controller.
- Protect an active parking owner from repeated requests while permitting a
  new explicit attempt after ERROR/IDLE/PARKED. Already-charging requests are no-ops.

## Build and inheritance

Both `yolov9mit` and `yolov9mit_ros` remain discoverable: their package-level
`COLCON_IGNORE` markers are removed. Other vendor/platform exclusions are unchanged.
The selected source includes the prior charging-completion UI, service chooser,
parking dispatcher, station calibration, and UI-only test simulator updates.
Field calibration values inherited from worak-test are not new measurement evidence.

## CARLA-only integration

The simulator keeps its Ranger asset, physics, map, sensor actors, ROS bridge,
and dedicated runtime overlays. A generated calibration uses the GNSS mount
from the selected spawn JSON for both input-adapter compensation and sensor TF.
It does not change the common production calibration, EKF, safety thresholds,
parking tolerances, controller gains, or robot asset. Ground-truth pose is not
substituted for the production localization path.
The CARLA shell restores only the already-provisioned local SDL2_mixer prefix
after ROS environment cleanup, so the updated voice adapter is actually installed.

## Validation scope

See `V2_2_6_VALIDATION.md` for measured counts, source boundaries, and remaining
limitations. CARLA charging is a simulator contact signal, not a real BMS test.
No all-campsite endurance, physical charging, or field-safety certification is claimed.
