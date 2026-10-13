# B11 — interrupted during road return; parking untested

<!-- HH_261012 - Preserve the interrupted result; do not infer a controller failure or parking pass. -->

trial.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B11/trial.json`) finalized `FAILED`, `roundtrip_verified=false`, with
`SIGTERM / KeyboardInterrupt`. supervisor.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B11/supervisor.json`) records
`infrastructure_error: completion supervisor interrupted`. The file evidence
does not establish why the supervisor was interrupted.

Actor **60**, operator-delivery generation **1791777060409001** ran from
**2026-10-12 03:51:09.540 to 03:54:50.217 UTC**. Observed travel was
**220.707 s / 125.077 m**; completed outbound service was
**108.299 s / 45.802 m**. No completed return-leg metric exists. Distances are
sampled physical-body XY travel, not UI wall-time-integrated distance.

The last pre-cleanup observation was site `DONE`, drop-zone
`ROAD_HANDOFF_READY`, reverse parking `IDLE`, and safety gate `ENABLED`.
Live-lanelet handoff reported **0.14 m** lateral error and **0.40 m** original
anchor error, with the forward return published and acknowledged. The last
command was forward **1.111111 m/s**; it is a pre-cleanup sample, not evidence
of motion after stopping. The probe subsequently recorded its exact mission
inactive through visible-UI cleanup. Recorded collision events were **0**.

Final reverse parking was never reached. There is **no final-parking XY error
or comparison against the unchanged 0.25 m goal tolerance** for this attempt.
Earlier B11 passes belong to their own profiles and cannot make this v5
attempt successful.

The finalized capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B11/visual/capture_manifest.json`) is `PASS`, with
actual paired-desktop capture and derived GIF at **5 fps**:
PNG (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B11/visual/representative_contact_sheet.png`),
GIF (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B11/visual/representative_motion.gif`). Source MP4 was pruned after derivation.
Supervisor probe/capture exit-code fields are null because execution was
interrupted; they are not reclassified as zero. This analysis changes no
source, runtime state, or original trial record.
