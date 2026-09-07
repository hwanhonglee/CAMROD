# CAMROD v2.2.4 Release Notes

<!-- HH_260907 - Preserve the full field-change rationale and distinguish
source fixes, installed builds, isolated tests, and physical acceptance. -->

## Scope

The 2026-09-07 develop release combines the day's Recall/loading workflow,
Robot/Guest mission authority fixes, battery-aware parking dispatcher, relocated
shared drop zone, field parking corrections, voice guards, and regression tests.
Detailed investigation evidence is recorded in
[the dated validation record](../camrod_bringup/docs/recall_return_validation_20260907.md).
This release is not a claim that all physical B1-B13 service cycles passed.

## Battery and parking policy

| Fresh battery SOC | Behavior |
| --- | --- |
| At least 35% | New mission allowed; ordinary non-charging reverse parking on return |
| At least 25%, below 35% | Reject a new mission; finish the current service and return through its normal confirmation workflow |
| Below 25% | Request urgent return through the existing safe site exit and road route |
| Below 35% at parking selection | AprilTag charging docking at the same station |
| Unknown/stale SOC at parking selection | Require charging docking, never assume non-charging completion |
| Explicit Dock request | Select AprilTag docking even at high SOC |

The old SOC-only <=20% motion stop is disabled so it cannot prevent urgent
return. EStop, CAN mode, platform error masks, actual voltage/temperature/BMS
faults, localization freshness, and live obstacle protections remain authoritative.
The final Recall loading confirmation cannot be bypassed by low battery.

The parking dispatcher converts the platform's float32 SOC fraction [0,1] to
percent before comparing it with 35. Exact 0.35F selects reverse; the adjacent
lower float selects docking. A moving attempt never changes controller merely
because SOC crosses the threshold. A completed reverse park can later request
charging when SOC falls below the threshold.

Auto mode uses private reverse/AprilTag command, status, service and operation
channels. Both old owners must acknowledge cancellation before the selected
owner starts. Only the selected owner publishes the shared `/parking/status`
and public motion/service stream. Handoff, telemetry freshness, cancellation,
retry, and late confirmed-charge recovery retain explicit failure handling.

## Recall and UI authority

- Initial Recall waits up to 0.30 m toward the campsite from its lanelet snap.
- B1-B10: first loading confirmation, spoken clearance notice, an 8 s stopped
  interval, site entry, 180-degree turnaround, then `RECALL_RETURN_WAIT`.
- A second explicit same-site/same-generation confirmation is required before
  site exit and the normal forward return route. The robot remains stopped
  while people may still be loading. Duplicate or early confirmations do not
  skip this barrier.
- B11-B13 retain the opposite-direction roadside loop without a site turnaround.
- Robot and Guest screens follow real controller phases and share typed
  mission ownership, cancellation, and final-confirmation handling.
- The backend broadcasts accepted mission identity to already-connected Robot
  WebSocket clients. A stale pre-dispatch identity must not suppress Return.
- Normal return routes start from the current road-exit pose, without returning
  to the historical campsite entry coordinate.

The 8 s clearance interval is a timed announcement wait, not proof that
perception has verified the area empty. Existing occupancy/safety guards apply.

## One relocated drop zone

Active `lanelet2_maps.osm` is map version 23. Only area 7019 remains a drop zone;
former relation 2320 is removed because that space is a vehicle entrance.
Its underlying way/nodes and the operator's named map snapshots are preserved.

The shared station is x=-11.3585 m, y=40.0901 m, yaw=-82.2127 degrees. Both reverse
parking and AprilTag docking use it; there is no separate battery-selected bay.
The existing area exporter preserves `parking_method=auto`, explicit site yaw,
service modes and stable ordering. Map/localization/bringup drop-zone mirrors,
planning/bringup B1-B13 configs and return keypoints are regenerated together.

Bringup uses `camrod_bringup/config/map/drop_zones.yaml` as the station source.
Historical map-v22 PNG/JSON reports are identified as historical evidence, not
regenerated physical proof of the v23 map. Named 1.0.12/1.0.13 OSM snapshots are
archives; the active map remains `lanelet2_maps.osm`. The unrelated, untracked
July point-cloud copy is not part of this day's release.

## Departure after UI/backend restart

The 18:12 field session exposed a separate departure bug: startup recovery
published OPERATOR_STOPPED, clearing the remembered parked/charging service
state. A Robot B1 selection then published a normal site goal without an EXIT
operation. The gate correctly held `lanelet_physical_body_cost` because the
vehicle still occupied the off-road station. No parking-controller cancellation
failure or longer EXIT-distance limit was demonstrated by that trace.

Robot and Guest now use the same authored drop-zone polygon before mission
admission. The UI receives the exact bringup station YAML used by maneuver and
parking nodes. A fresh finite map-frame pose inside it requires the existing
parking CANCEL, bounded straight EXIT, yaw alignment and road-handoff sequence
before the site goal is published, including after restart/OPERATOR_STOPPED.
An outside pose must not initiate an arbitrary station EXIT. Missing/invalid
station geometry, stale/future/missing pose timestamps, wrong frames and
ambiguous boundary positions refuse new dispatch instead of guessing. These
checks do not release parking ownership for a rejected request.

The existing departure controller targets a fresh lanelet point, not the old
1.2 m maintenance fallback. Its 8 m approach bound, 1 m lateral bound, 0.20 m
arrival tolerance and 30 s timeout remain unchanged; live obstacles still stop
motion. Physical successful departure after this fix remains to be verified.

## Reverse parking, docking and voice

The observed false parking success occurred at the old 1.5 m reverse limit,
although the new station was about 3.75 m from the road approach. The limit is
now explicitly 5.0 m; a farther/invalid station is rejected before departure.
It is not expanded automatically. Success requires fresh finite station XY
within the existing 0.25 m tolerance, or confirmed charging contact. Travel-limit
and signed-axis misses stop with ERROR; stale pose and the 30 s timeout still
stop motion. Logs expose initial distance, maximum bound and remaining XY error.

Parking already has a phase-bounded lanelet-boundary exception. No radar range,
physical obstacle threshold, EStop, or lanelet exception was widened to implement
the longer mapped approach. Ordinary reverse parking no longer emits misleading
`docking.started/succeeded/failed` speech. These cues belong to the selected
AprilTag attempt; unverified ordinary-parking audio is not invented or reused.

A saved visible tag36h11 ID 3 frame failed detection at runtime decimation 1.75
and passed at 1.5 during diagnosis. Final release inspection found the current
operator profiles at 2.0, which is preserved unchanged. A fresh isolated replay
of the installed detector confirmed decimation 2.0, ID 3 and finite pose with
the unchanged reprojection gate. The constructor-loaded parameter requires
restart; one-frame replay is not proof of all-distance recognition or physical
docking success.

## Field tuning and GNSS limits

Preserve the operator's current values: Nav2 route XY tolerance 0.10 m in both
planning profiles; AprilTag heading/lateral gains 1.2/2.0 in both parking profiles;
bringup GNSS heading trim -92 degrees. These are not silently reset to upstream
defaults. The local snap correction remains 0.05 m, and final reverse parking
remains 0.25 m: they are distinct arrival checks.

The operator confirmed that the LEFT GNSS antenna produces NavSatFix. Existing
TF and explicit lever-arm parameters use x=0, y=+0.45 m from robot_center_link.
No mounting offset was guessed or changed. The custom adapter subtracts the
rotated offset numerically; changing TF alone does not update those parameters.
Its direct receiver-heading path admits up to 1 s timestamp difference from
the fix, a potential rotation-error source still needing synchronized field
evidence. New tests cover fixed-center antenna rotation and synthetic yaw lag;
they do not establish centimeter-level real-world accuracy.

## Build and validation boundary

- The day's functional UI/control/planning/system/voice/map/localization and
  bringup revisions have separate successful selected-package build records.
- The latest reverse executable compiled successfully; its installed path and
  both installed parking YAML paths resolve to the updated build/source files.
- The final bounded-reverse follow-up passed 122 control cases, 12 parking
  battery/ownership cases and 5 GNSS lever-arm cases in isolated tests.
- Final departure/release validation: all 284 UI cases, 47 bringup/map/routing
  cases, the separate 98-case voice/system/battery batch, and all eight native
  control targets passed. These suites overlap; their counts are not summed.
- After the departure fix, the canonical wrapper rebuilt the Robot frontend
  (`main.daacedf0.js`) and built/installed UI, voice and bringup successfully.
  The imported installed backend matches the final source byte for byte.
- Earlier map/UI/Recall revisions passed the Python and native batches listed
  in the dated validation record. Those are per-revision results, not a claim
  that every workspace test passes on the final release.
- The combined full-perception build was interrupted in unrelated
  `obstacle_lidar_node` compilation. The AprilTag target built and replay passed,
  but a complete full-workspace/perception build is not claimed.
- No robot motion, process restart, or live safety-parameter change was issued
  by the release workflow. Source installation does not update a running Python
  process; restart under operator control while safely stopped.

## Physical acceptance still required

Verify departure from the actual parked station after backend restart, forward
road handoff, the 3.75 m reverse approach, final pose error, charging contact,
tag reacquisition, and both Recall confirmations with the real robot. Record
current pose, phase and final safety-gate reason together when movement stops.
