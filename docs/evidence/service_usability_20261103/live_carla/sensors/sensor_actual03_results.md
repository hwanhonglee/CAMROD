# Actual CARLA stationary near-field trial 03

HH_261103 — Actual world `/Game/map_package/Maps/Woraksan_v27_charge_anchor_20261103/Woraksan_v27_charge_anchor_20261103`, ego actor 2 (`vehicle.ranger.default`, role `ego_vehicle`). Root reserved the stopped world after the SOC35 cargo departure. Preflight measured speed was 0.000000785 m/s. No ego control, pose reset, world tick, sensor/control publication, map edit or parameter change was performed.

**This is a stationary sensor/grid comparison, NOT a braking test or safety PASS.** The native gate remained `STANDBY`, unengaged, with platform drive disabled. H/N/U below means fresh measured high-cost HIT / NO_HIT / UNAVAILABLE; NO_HIT is not proof of free space.

The 60 s baseline began 2026-11-03 08:16:00 UTC. All cases completed by 08:18:00 UTC. Each case has 25 observations. Gaps are actor-bbox-to-vehicle-bbox distances, **not closest visible mesh-skin gaps or raw sensor ranges**.

| Actual case | Measured bbox gap (m) | Raw FRONT1 minimum range (m) | Raw FRONT2 minimum range (m) | Filtered LiDAR points | Radar H/N/U | Guard H/N/U | Both H/N/U |
| --- | ---: | --- | --- | ---: | --- | --- | --- |
| Front 10 cm | .10000183 | .400–.411 | .390–.400 | 1250–1305 | 0/24/1 | 25/0/0 | 24/0/1 |
| Front 20 cm | .20000226 | .498–.512 | .488–.501 | 938–971 | 0/24/1 | 0/25/0 | 0/24/1 |
| Front 30 cm | .30000269 | .598–.608 | .587–.603 | 750–771 | 0/23/2 | 0/23/2 | 0/21/4 |
| Front 40 cm | .40000118 | .696–.708 | .686–.699 | 622–641 | 0/25/0 | 0/25/0 | 0/25/0 |
| Roadside left | .20000013 forward; .20 lateral | no return | no return | 0 | 0/25/0 | 0/25/0 | 0/25/0 |
| Roadside right | .20000147 forward; .20 lateral | no return | no return | 0 | 0/23/2 | 0/25/0 | 0/23/2 |

Baseline: 292 observations, radar **0/271/21**, guard **0/273/19**, both **0/255/37**. Steady roadside controls had no observed high-cost front hits. One guard residual HIT 0.003663 s after the 10 cm actor's removal remains in the report; it was not erased or described as a persistent background false positive. Availability gaps are retained, not converted to clear measurements.

## Actual photographs and source records

The new read-only watcher saved **7/7 fresh original camera JPEGs**, with source ages .009–.020 s. Each case image's source stamp is at least .5 s after its measured-placement receipt. The camera's field of view limits what is visible; roadside images are not proof of front-object detection. These are actual CARLA camera data, not UI fixtures or generated scenes.

- [No-owned-actor baseline](nearfield_actual_03_camera_baseline_end_front.jpg)
- [Front 10 cm](nearfield_actual_03_camera_front_10cm_front.jpg)
- [Front 20 cm](nearfield_actual_03_camera_front_20cm_front.jpg)
- [Front 30 cm](nearfield_actual_03_camera_front_30cm_front.jpg)
- [Front 40 cm](nearfield_actual_03_camera_front_40cm_front.jpg)
- [Roadside left](nearfield_actual_03_camera_roadside_left_front.jpg)
- [Roadside right](nearfield_actual_03_camera_roadside_right_front.jpg)
- [Original photo/source-stamp/SHA manifest](nearfield_actual_03_camera.jsonl.gz)
- [Actual 02-versus-03 counts panel](sensor_actual02_vs_actual03.png), [CSV](sensor_actual02_vs_actual03.csv), [full JSON](sensor_actual02_vs_actual03.json)

The [actual sensor geometry snapshot](nearfield_actual_03_sensor_geometry.json) records 15 sensors at CARLA frame 37361. Actual ego-relative mount x was .62786963 m FRONT1, .62786703 m FRONT2 and .76335984 m LiDAR. This snapshot is **not aligned to each raw-cloud source timestamp** and does not establish closest mesh skin or justify a range calibration. The observed physical/native front-edge difference was .05511681 m. More geometry/surface claims remain unverified; no sensor calibration was changed.

## Exact-owned cleanup and limits

The original probe acknowledged removal of characterization actor **18** and case actors **19–24**, all under unique role `camrod_sensor_probe_55b76e85fa37`, exact actor type/ID checks. Probe, camera watcher and optional read-only geometry observer exited normally. No such observer/probe process remained at handoff. Root was told **WORLD FREE** immediately after completion. No subsequent world queries or retakes were performed.

Raw [actual03 JSONL](nearfield_actual_03.jsonl.gz) SHA-256: `3d1dc8113e8f728bde574d7573d4760e8ba131a65b5ac080a2e9957b758976fa`. Earlier actual01/02 raw files are unchanged. The comparison generator imports the original reducer; 26 pure harness tests passed before the run. Exact-range arrays, raw high-cost cell counts, missing observations, publisher identities and UTC/source clocks are preserved in the JSON files.

Not checked: fault-cleared moving braking, three isolated final-gate modes, true nearest surface clearance, paired per-ray source-stamp geometry, or production/physical-robot certification. No PASS is inferred for these untested items.

## Offline actual-photo overview

HH_261103 — [Seven-case PNG contact sheet](nearfield_actual_03_photos.png) (1280×1390, 2.00 MB) and [small GIF screenshot sequence](nearfield_actual_03_photos.gif) (960×726, seven 1.5 s stills, 2.61 MB). Every scene has its case, measured bbox gap and capture UTC from the original camera manifest. Both explicitly say **not continuous / not a braking test**.

All seven original JPEGs are unchanged. PNG thumbnails use nearest-neighbor sampling only; the GIF keeps each source's native 800×600 image pixels before GIF palette quantization. Labels are outside the scene pixels. No interpolation between frames, generated imagery, object drawing or retakes were used. [SHA-256/source manifest](nearfield_actual_03_photos_manifest.json) records every original and both outputs. The offline generator is [sensor_photo_sheet.py](sensor_photo_sheet.py); 27 pure harness tests passed after this addition. No live world queries were made for these visuals.
