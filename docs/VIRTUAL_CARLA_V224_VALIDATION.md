# CAMROD v2.2.4 CARLA validation — 2026-09-08

This is a historical investigation log. Current source/result distinctions,
actual B1 Guest PASS, optional Dock FAIL, PNG/GIF links and the corrected
GNSS profile scope are in [the v2.2.5 validation summary](VIRTUAL_CARLA_V2_2_5_VERIFICATION.md).

Status: runtime validation in progress. The historical B1–B13 PNG/GIF bundles
under `docs/evidence/virtual_carla/current` predate this update and do not prove
the v2.2.4 reverse-first parking workflow.

## Source and pure changes

The pure branch starts at develop `f3a177038`. Independent verification and
heading/config/test synchronization are `32597828d`; the nested parking launch
parameter scope fix is `078ca6010`. Both are incorporated into virtual/carla
at `e836ca358`. See [pure verification](V2_2_4_VERIFICATION_20260908.md).

The scope fix prevents the AprilTag detector's YAML from replacing parking.yaml.
Live pre-fix inspection found `complete_without_charging=false` despite the
configured true value. Correct controller-file delivery also restores the
configured reverse/final speeds and charging wait timeout.

## Virtual integration changes

- The evidence runner observes `/parking/status` and both private controller
  status topics. Reverse acceptance requires the current mission's reverse
  approach, PARKED, dispatcher reverse ownership, DROP_ZONE_WAIT and stopped
  command gate. `charging_confirmed=false` is retained rather than relabelled.
- The charger contact emulator follows dispatcher ownership. A reverse park
  cannot manufacture a charging contact from an inactive AprilTag heartbeat.
- Initial spawn is the current dz_area_7019 center (-11.3585, 40.0901), with
  body yaw +97.7873 degrees (station reverse axis plus 180 degrees). The vehicle
  is spawned 0.50 m above ray-cast road height and settles through physics.
- The new `woraksan-camrod-site-geometry-v224` map copies v15, translating only
  the 46 existing tag objects by (+287.620024738, -30.379765796, 0) UE cm.
  The other 581 objects, terrain, collisions, materials and source assets are
  unchanged; OpenDrive/Nav/TM sidecars are byte-identical. This aligns the
  simulator's existing charger target with develop's changed station position.

The staging report is in
`$RANGER_EVIDENCE_ROOT/v224_validation/map_dropzone_stage.json`. The verified
output map SHA-256 is
`1af1a6d160f8ee24a97294b104e5be9ca6aec767fc456abcf51f43a0ee4b1343`.
The editor reported APPLIED and verified disk reload before an exit-time Slate
assertion; the execution note records this separately from runtime acceptance.

## Evidence in progress

The first real Robot UI B1 delivery departed the simulated charger, reached
the site, crabbed in and completed the 180-degree rotation. Outbound time was
269.106 seconds and odometry distance 84.236863 m; no collision event occurred.
It then failed before Return: a duplicate battery handler present only in
virtual/carla cleared the arrival state on `battery_return_pending=false`.
Removing that duplicate restores develop's existing incremental handler.
Actual WebSocket-handler replay reproduces the failure before the fix and
passes afterward. The failed attempt's PNG/GIF are labelled as outbound-only.

The browser runner now accepts the actual authorized arrival modal or panel
button (both use the production handler), and clears its observation probe
before Return so a previous mission's frame cannot count as a fresh request.
Desktop capture also includes a sample from the last second of each recording;
the former fixed 90-percent sample could miss the final parking state entirely.

The next actual B1 run verified that the restored button emitted Return and
the vehicle completed crab-out to the live lanelet handoff. Its old assertion
then rejected the new generation-bound `robot_ui:usage_complete` source.
The updated runner/offline validator now bind owner, site, intent, generation
and a fresh post-click ROS ACK; old transport labels are not acceptance proof.
B1–B10 recall includes separate clearance and final loading confirmations.
Guest missions default to real Guest first confirmation and real Robot UI
final confirmation, preserving Guest ownership and recording both visible UI
identities/screenshots. `--guest-final-return-authority guest` explicitly tests
the Guest-only alternative. B11–B13 retain the single-confirmation route.

### Reverse goal stop-condition fix

A separately labelled real return-only recovery reached the new station but
failed at XY error 0.280 m against the unchanged 0.250 m tolerance. The axial
stop threshold was also 0.250 m, so a lateral offset of about 0.128 m caused
premature failure before the vehicle entered the circular XY acceptance area.
Pure commit `ddd599672` permits the existing bounded final approach while the
station remains ahead and the reverse line can reach that area. It still stops
on a lateral miss, crossing the station plane outside the goal, invalid/stale
pose, maximum travel, timeout or charger detection. No tolerance was increased.
Native controller/completion tests: 20 passed; rebuilt main control CTest:
9 suites passed. A fresh full B1 round trip subsequently passed on virtual
commit `78f4f0b64102fdb5ba708dee33806835158ed189`, actor 18:

| Segment | Elapsed time | Actual odometry distance |
| --- | ---: | ---: |
| Delivery and site arrival | 264.157 s | 83.598284 m |
| Return and reverse parking | 483.751 s | 85.697460 m |
| Total | 747.908 s | 169.295744 m |

Final XY error was 0.216393 m, collision events were zero, both reverse
controller and dispatcher were PARKED, service was DROP_ZONE_WAIT and
charging remained false. This is B1 evidence, not B2–B13 or recall acceptance.
Its completed PNG/GIF, physical-wheel summary and native report references are
under `v224_validation/operator_delivery_current/B1/`.

### Interrupted B2 and standalone Return completion

B2 genuinely departed the non-charging parked position and received its road
navigation goal. The test/recording process group then exited with signal 15
(exit 143); CARLA and CAMROD remained alive. A production Stop was issued.
That incomplete attempt is not accepted as a B2 round trip. Subsequent runners
use independent user-systemd services with this runtime's explicit ROS domain
5; `env.sh` otherwise defaults to 188 in a fresh service environment.

During supervised Return-only recovery, the physical controller reached
reverse PARKED again. The virtual-only standalone-Return UI guard rejected
its DROP_ZONE_WAIT heartbeat: it required observing parking start inside the
station polygon, although reverse parking starts at the road approach point
outside that polygon. The same guard is absent from the pure develop branch.
The virtual-only correction binds a new current dispatcher attempt, real
reverse approach, matching reverse PARKED and fresh in-station pose; stale or
uncorrelated terminal heartbeats remain rejected. Replaying the previous
actual callback fails the new regression; the corrected callback passes.
Related backend Stop, manual-drive and frontend tests: 168 passed. This is
deterministic callback evidence, not a claim of completed fresh live Return.

### Optional legacy generated-map compatibility

The active map remains the exact latest develop `lanelet2_maps.osm`, SHA-256
`2c96514fa788e46ab5061a0ebc130a732557045d0baa3b67bb9f9dbcb132fef7`.
The unused optional generated artifact was refreshed separately: develop
replaced old B12 centerline 6975 with authored 6998, so its obsolete four-point
override is retired, while the three existing connector adjustments remain.
Generated IDs now start after every existing OSM primitive ID. No active
lanelet map, UE terrain or mesh is changed by this compatibility fix.
Adapter offline tests after regeneration and clean map environment: 577 passed.

### GNSS/yaw verification boundary

Pure localization implements `center = antenna - R(yaw) * offset`, with the
current left-antenna offset `(0, +0.45 m)` and heading trim `-92 degrees`.
Five native tests pass, including invariant center over cardinal rotations
and the error from a lagged heading. This mount must match the actual robot.
The separate metric-pose CARLA profile disables that GNSS correction path.
The active site-geometry profile instead uses production raw GNSS and lever-arm
correction, with simulated side-mounted antennas and IMU-derived heading.
Neither profile proves physical GNSS timestamp synchronization or a real
front-mounted antenna calibration. There is currently no post-yaw XY correction after
parking-point alignment. A passive observer records fresh localization pose
against the latched approach target; cached status error is not final XY proof.

Current logs and new artifacts are under
`$RANGER_EVIDENCE_ROOT/v224_validation/`. Build logs cover eight main packages,
the canonical Robot UI bundle and camrod_voice. A separate PulseAudio monitor
WAV proves playback of return/recall clips, not automatic mission triggering.

Pending runtime completion: B2–B13 Operator delivery/Return, Operator recall,
Guest recall/usage complete, reverse parking, optional docking, and Manual 4WS.
Each accepted mission must retain PNG/GIF, timing/distance, wheel telemetry
summaries and source/runtime hashes. A failed run remains labelled FAIL.

## Execution

```bash
cd /home/hong/camrod_ws/src
unset CARLA_ROOT CARLA_PYTHON_EGG
export RANGER_CARLA_ROOT=/home/hong/Downloads/ranger-carla-4ws-pipeline
export RANGER_EVIDENCE_ROOT="$RANGER_CARLA_ROOT/.work/evidence"
export CARLA_RENDER_MODE=onscreen
export CAMROD_CARLA_PARKING_COMPLETION=reverse
```

Start the documented `site_access.sh` lifecycle in order: server, bridge,
pacer, spawn, camrod-site-geometry, operator-ui and spectator. For visible
Operator delivery recording:

```bash
./scripts/virtual_carla/run_site_evidence_matrix.sh run \
  --authority operator-browser --mission-intent delivery \
  --sites B1,B2,B3,B4,B5,B6,B7,B8,B9,B10,B11,B12,B13 \
  --output-root "$RANGER_EVIDENCE_ROOT/v224_validation/operator_delivery" \
  --capture-fps 5 --derived-width 1920
```

Use a fresh output directory for each independent authority/intent. Select
`CAMROD_CARLA_PARKING_COMPLETION=charging` only for an actual docking test;
that setting changes the acceptance requirement, not the robot's SOC or policy.
