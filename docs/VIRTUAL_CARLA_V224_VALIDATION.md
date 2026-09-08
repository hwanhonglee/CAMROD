# CAMROD v2.2.4 CARLA validation — 2026-09-08

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

### GNSS/yaw verification boundary

Pure localization implements `center = antenna - R(yaw) * offset`, with the
current left-antenna offset `(0, +0.45 m)` and heading trim `-92 degrees`.
Five native tests pass, including invariant center over cardinal rotations
and the error from a lagged heading. This mount must match the actual robot.
The CARLA profile uses measured simulator center pose and disables that GNSS
correction path, so a successful driving test does not prove physical GNSS
timestamp synchronization. There is currently no post-yaw XY correction after
parking-point alignment. A passive observer records fresh localization pose
against the latched approach target; cached status error is not final XY proof.

Current logs and new artifacts are under
`$RANGER_EVIDENCE_ROOT/v224_validation/`. Build logs cover eight main packages,
the canonical Robot UI bundle and camrod_voice. A separate PulseAudio monitor
WAV proves playback of return/recall clips, not automatic mission triggering.

Pending runtime completion: B1–B13 Operator delivery/Return, Operator recall,
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
