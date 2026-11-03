# Real CARLA sensor follow-up — stationary first

HH_261103 — Reuse the unchanged November 2 actor probe and reducer; add only a read-only fresh-per-case camera watcher and a quantitative counts panel. No production sensor/C++/YAML changes, fake sensor publications, ego teleport/control, map editing, or runtime startup is authorized by these tools. Raw runs 01/02 stay frozen.

## What the actual measurements mean

Run 02 placed a pedestrian by **bbox-to-bbox body gap**, not by sensor range or closest visible mesh skin. Its measured Ranger front bbox was .763471 m ahead of the actor origin; the native guard front was .708370 m: a .055101 m difference. This is not the whole radar offset.

Across 10/20/30/40 cm placements, raw FRONT1 frame-minimum range minus measured bbox gap was .2974–.3116 m; FRONT2 was .2856–.3030 m. The inspected authored radar mount is x=.627870 m, giving only .135601 m sensor-to-front-bbox inset. The remaining difference cannot honestly be assigned to calibration from saved extrema alone: slant range, ray height/azimuth, actual sensor transform and the pedestrian's ray-visible mesh differ from its enclosing bbox. Never subtract a guessed constant from every radar return. Record actual CARLA sensor transforms and paired raw xyz/range rays before proposing any CARLA-only calibration; root must approve it separately.

The LiDAR filter did **not** discard every frontal obstacle point: actual filtered counts were 1246–1304 / 938–971 / 750–771 / 622–641 at 10/20/30/40 cm. The 10 cm guard result was 24 HIT / 0 NO_HIT / 1 unavailable; farther cases had no high-cost guard hits. A guard checks measured return points against its native body corridor, not the obstacle's unseen bbox surface. Authored LiDAR x=.76336 m and recorded filtered x minima provide context, not a verified live TF or closest-skin measurement. Its 16-channel −25°..10°/120° view, 8 cm nonground threshold and existing self-return mask also limit near/low-object observability.

No-nuisance reference: the actual 60 s baseline had 292 observations: radar 0H/269N/23U, guard 0H/284N/8U, both 0H/263N/29U. Both steady roadside controls had zero high-cost front hits in valid samples. Preserve unavailable counts and the one residual guard hit immediately after the 10 cm actor was removed. NO_HIT is not proof of free space. The gate was already FAULT_HOLD/estop, so run 02 was **not a braking test**.

## Authorized stationary sequence after root reserves the world

1. Root confirms exact ego ID, standstill, native guard/radar freshness and clock/TF health. Raw CARLA cloud stamps use `/clock`; reception-restamped canonical camera/ranges/grids use wall time. Do not infer a clock from timestamp magnitude.
2. Start `sensor_camera_watch.py --run` **before** the existing actor probe, even if its input JSONL does not yet exist. Wait for `watcher_ready`. It drains all complete receipts per spin, waits .5 s after each measured placement, and saves only an unmodified fresh actual front JPEG. Three-second expiry, stale image or early actor removal produces explicit unavailable evidence. A photo outside the camera FOV is not object-detection proof.
3. Run the unchanged old `sensor_probe.py --run --place-obstacles --ego-id ID --baseline-seconds 60 --case-seconds 5 --output NEW.jsonl`. It owns only its unique role/type/IDs; root does not move/retry/dock during the reservation. Six stationary placements and exact-owned cleanup remain its existing behavior. No world tick or ego movement is added.
4. After both tools exit and owned cleanup is acknowledged, run `sensor_compare.py OLD02.jsonl NEW.jsonl --output-prefix NEW`. It imports the original reducer, retains H/N/U, raw range/counts, hashes, baseline availability and residual transitions. Its panel always says **STATIONARY / NOT A BRAKING TEST**; motion tests need separate evidence.

## Real stop test — root owns motion and further authorization

Reuse the visible manual-UI bounded pulse pattern (`tools/probe_manual_reverse_once.py`, `scripts/virtual_carla/manual_4ws_evidence.py`) only after the docking fault is cleared and root confirms healthy final arbitration. Observe manual input, native raw command, final `/control/cmd_vel_ros`, adapter command, actual odometry, collision events, grids and gate reasons together. Final Twist feeds the existing `twist_to_4ws_node.py`, not a probe publisher.

First prove a stationary blocked-command case and an unobstructed bounded crawl. A subsequent approach starts at ample distance, at the lowest supported already-approved speed, with a root-owned independent timeout/distance stop and measured stopping margin. Do **not** approach a 10 cm obstacle at 3 km/h: .833 m/s × the configured .15 s reaction term already travels .125 m before braking. Preserve the native 3 cm docking limit and never remove a live interlock to obtain motion.

Record first fresh sensor return, first occupied grid, exact gate reason, command-to-zero delay, actual speed-to-standstill delay, displacement and minimum independently measured gap. A test is not a successful safety stop if another FAULT_HOLD, timeout or operator stop caused the zero command. Do not claim a numerical safe speed/deceleration before observing them.

Radar-only / guard-only / both are currently **input shadow comparisons**, not three isolated final-gate braking runs. The guard-enable parameter is startup-read-only. Do not toggle sensors or publish artificial clear grids on the driving graph. Any separately approved evaluator/gate instance must isolate **all** output/status/service endpoints from the driver; otherwise duplicate gates can contend. Combined real-gate braking can be measured without weakening either source.

Not checked here: live sensor transforms, collision geometry/closest mesh skin, fault-cleared braking, and new per-case photos. No live commands are implied by this plan.

## Exact commands (only after root reserves this stationary world)

From `/home/hong/camrod_service_test_ws/src`, use the **November 3** runtime wrapper and root-confirmed cargo-release gate revision below; the old probe source is reused without extending it. Start the watcher in one terminal. `watcher_ready` means the read-only subscription exists, not that the camera is healthy. The 60 s baseline permits discovery; freshness is independently checked for each photo. If root selects a different accepted gate revision, update this explicit value before running.

```bash
CAMROD_SERVICE_GATE_REVISION=20261103_cargo_release CAMROD_SERVICE_TAG_SCENE=1 bash docs/evidence/service_usability_20261103/live_carla/runtime.sh ros python3 docs/evidence/service_usability_20261103/live_carla/sensors/sensor_camera_watch.py --run --input docs/evidence/service_usability_20261103/live_carla/sensors/nearfield_actual_03.jsonl --output docs/evidence/service_usability_20261103/live_carla/sensors/nearfield_actual_03_camera.jsonl --seconds 180
```

After ready, run the existing actor probe in another terminal. This adds the CARLA egg **after** the wrapper has sourced ROS, without replacing ROS's Python path. Actor ID 2 must be re-confirmed by root before executing.

```bash
sensor_probe_egg_cache=$(mktemp -d /tmp/camrod_sensor_20261103.XXXXXX)
CAMROD_SERVICE_GATE_REVISION=20261103_cargo_release CAMROD_SERVICE_TAG_SCENE=1 bash docs/evidence/service_usability_20261103/live_carla/runtime.sh ros env PYTHON_EGG_CACHE="$sensor_probe_egg_cache" python3 -c 'import runpy,sys; sys.path.insert(0,"/home/hong/carla/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"); runpy.run_path("docs/evidence/service_usability_20261102/live_carla/sensor_probe.py",run_name="__main__")' --run --place-obstacles --ego-id 2 --baseline-seconds 60 --case-seconds 5 --output docs/evidence/service_usability_20261103/live_carla/sensors/nearfield_actual_03.jsonl
```

Offline comparison, after the actual trace is complete:

```bash
python3 docs/evidence/service_usability_20261103/live_carla/sensors/sensor_compare.py docs/evidence/service_usability_20261102/live_carla/nearfield_actual_02.jsonl docs/evidence/service_usability_20261103/live_carla/sensors/nearfield_actual_03.jsonl --output-prefix docs/evidence/service_usability_20261103/live_carla/sensors/sensor_actual02_vs_actual03
python3 -m pytest -q docs/evidence/service_usability_20261103/live_carla/sensors/sensor_tools_test.py docs/evidence/service_usability_20261102/live_carla/sensor_probe_test.py
```

Local validation: 26 pure tests passed; `sensor_actual02_reference.{json,csv,png}` is an offline replay of unchanged actual run 02 only. It is not a new trial or a new photograph. The plotting environment reports unavailable optional Matplotlib 3D support; this 2D counts panel renders successfully.
