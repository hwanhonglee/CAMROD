# v2.2.6 validation record — 2026-09-11

<!-- HH_260911 - Report executed scope, not a blanket safety or all-site PASS. -->

## Environment and provenance

Host `htop`: Ubuntu 22.04.5, ROS 2 Humble, Python 3.10.12, RTX 3060.
The rendered CARLA world, Ranger actor, ROS bridge, and pacer were real processes.
The tested algorithm source was `/home/hong/camrod_ws/src` (`virtual/carla`),
with the explicit `camrod-site-geometry` simulator profile and ROS domain 188,
localhost-only. Unit tests used isolated domain 189. No physical vehicle was driven.
The common changes were also applied to the existing `develop` worktree at
`/home/hong/camrod_ws/.codex-develop-20260904`; neither branch was renamed.

## Source-level and native regression results

| Scope | Result |
|---|---|
| develop: five UI/voice/battery regression files | 207 passed, 0 failed |
| virtual/carla: the same regression scope plus existing extensions | 258 passed, 0 failed |
| New station/cancellation tests, each branch independently | 18 passed, 0 failed per branch |
| Native control policies | 159 passed, 0 failed |
| Native parking dispatcher | 17 passed, 0 failed |
| Native reverse parking controller | 21 passed, 0 failed |
| Native reverse completion policy | 4 passed, 0 failed |
| Native initial-clearance controller, including voice preconditions | 18 passed, 0 failed |
| CARLA sensor-mount calibration checks | 5 passed, 0 failed |

The original ten UI failures were resolved without turning off voice gating.
Geometry fixtures explicitly acknowledge playback; additional cases prove zero
progress before acknowledgement and continued idle after cancellation.
Native tests were compiled from the CARLA integration worktree. They are not
claimed as a separate clean build of every develop package.

## Executed rendered CARLA sequences

| Sequence | Actual observation |
|---|---|
| Before correction: Dock from startup STOPPED at the station | HTTP 409; request rejected |
| Before correction: Return, then Dock retry | New attempts accepted, but reverse-axis ERROR repeated |
| Corrected direct Dock | Alignment -> reverse -> AprilTag -> CHARGING/PARKED |
| Dock while already charging | `already_charging`; no new maneuver |
| Return while contact remains active | `waiting_for_disconnect`, then bounded expiry; no movement |
| B1 departure from charging, followed by Stop | Actual localization displacement 0.661 m; Stop acknowledged |
| Return after that Stop | DROP_ZONE_WAIT/PARKED observed about 7.03 s after request |
| Controlled detector outage | Real detector paused; parking ERROR observed after about 64.21 s |
| Detector restored, Dock retried without restarting the stack | New attempt -> clearance -> reverse -> CHARGING after about 40.13 s |

Times are observer samples, not hard real-time guarantees. The corrected first
Dock run reached CHARGING in about 18.06 s; the final installed voice adapter
was verified byte-identical to source before the later continuous sequence.
The detector was resumed in a finally block. No tag/charge success was fabricated.

## Evidence and limitations

Full raw snapshots, build logs, and failure/retry records remain under:
`/home/hong/camrod_ws/_sync_backups/v226_sequence_20260911/`.
The earlier source-import and build checkpoint is under:
`/home/hong/camrod_ws/_sync_backups/algorithm_regression_20260910T175434Z/`.
Committed compact evidence contains test reports and timestamped phase traces.

The commands used the same REST endpoints as the UI; this is not a claim that
all browser click/reconnect paths were exercised end to end.
The existing Ranger model, map, wheel physics, camera, and sensor actors were
retained. The 0.0 m CARLA GNSS mount and 0.65 m hardware mount are deliberately
separate calibrations, not a localization algorithm change.
The site-geometry profile contains pre-existing simulator-only opt-ins; this
run is not byte-identical to a physical-vehicle parameter configuration.
The test did not complete a B1 service round trip or B1-B13 endurance matrix.
Charging confirmation was the existing simulator contact emulator, not hardware.
Selected packages, including YOLO, control, UI, adapter and voice, were built;
no complete clean rebuild or real-vehicle acceptance is claimed.
The production announcement timeout remains fail-open and requires deployment review.
