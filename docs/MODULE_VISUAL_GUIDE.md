# CAMROD Module Visual Guide

<!-- HH_260804 - Separate architecture diagrams, measured simulation data,
and pending physical evidence so package READMEs cannot imply false passes. -->

This index explains what each package image proves, where its values come from,
and how to replace simulation-only material with Jetson/robot evidence. The
images are documentation artifacts; runtime decisions continue to come from
ROS topics, diagnostics, controller state, and safety gates.

## Evidence Classes

| Label | Meaning | Valid claim |
|---|---|---|
| `SOURCE-DERIVED CONTRACT` | Generated from checked-in messages, launch defaults, and package YAML | The configured topology, state names, selected plugins, and thresholds match source |
| `MEASURED SIM EVIDENCE` | Generated from a committed JSON report captured from a running ROS graph | The listed topics/events were observed in that simulation run |
| `ALGORITHM SCHEMATIC` | Deterministic synthetic data illustrates a configured algorithm | The processing concept and displayed thresholds are reproducible; the points are not sensor data |
| `FIELD PENDING` | Hardware-dependent behavior has no matching committed capture | No physical accuracy, throughput, calibration, or detection pass is claimed |

## Asset Index

| Package | Visual | Source | Current verdict |
|---|---|---|---|
| `camrod_bringup` | `full-stack-mission-contract.png`, `mission-lifecycle-contract.gif` | Launch/state/battery/parking contracts | Expected full scenario only; the GIF is not runtime footage |
| `camrod_bringup` | `simulation-evidence-20260804.png` | `campsite-smoke-20260804.json` | Stack startup and pose chain pass; B6/B12 round trips fail closed during campsite entry |
| `camrod_sensor_kit` | `reference-frame-before-after.png`, `sensor-x-before-after.png` | Canonical geometry plus committed A/B simulation summary | Rear-axle and axle-midpoint values are compared directly; GNSS lever-arm calibration remains pending |
| `camrod_localization` | `pose-generation-and-timing.png` | EKF YAML plus 30-second pose probe JSON | Sim input/output cadence and freshness pass; field accuracy remains pending |
| `camrod_planning` | `nav2-servers-and-mission-states.png` | Nav2 config, bringup selectors, and state contracts | Configured route/control ownership is documented; it does not override the campsite failure |
| `camrod_perception` | `yolo-lidar-and-parking-pipelines.png` | Perception and AprilTag YAML | Source topology only; physical YOLO/fusion/AprilTag evidence is pending |
| `camrod_sensing` | `sensor-processing-and-cost-fusion.png` | Sensor/cost-grid YAML | Source topology only; hardware quality remains pending |
| `camrod_sensing` | `ground-segmentation-schematic.png` | Ground-filter thresholds plus seeded synthetic points | Algorithm illustration only, explicitly not a field point cloud |

## What Each Module Shows

### Bringup

The bringup contract follows dependency order from platform and sensor frames
through sensing, perception, localization, planning, control, system, and both
UIs. Its lifecycle distinguishes parking-controller `PARKED` from the public
service state: no charge feedback produces `DROP_ZONE_WAIT`, contact-pending
logic produces `WAITING_FOR_CHARGING`, and positive CAN charge feedback
produces `CHARGING`.

The adjacent runtime image is authoritative for the 2026-08-04 smoke run. The
81-node stack reached `[SYSTEM] OK`, but both B6 turnaround and B12 roadside
entry issued `CRAB_IN`, hit `lanelet_footprint_cost`, and timed out. The map has
campsite Areas but no surveyed `service_access` polygon joining each road
lanelet to the maneuver area. Existing Areas were tested temporarily and were
insufficient, so that experiment was reverted. The 0.10 m planning margin and
complete-footprint gate were not weakened. The normalized report points to the
six committed raw ROS logs in
[`docs/evidence/module-guides/bringup/raw`](evidence/module-guides/bringup/raw/README.md).

### Sensor Kit

| | Before | Current |
|---|---|---|
| Origin | rear axle, `robot_base_link` | axle midpoint, `robot_center_link` |
| 4WS role | former runtime reference | canonical Dual-Ackermann/crab/zero-turn reference |
| Sensor X | rear-axle coordinates | `previous X - 0.443 m` |

The physical robot is unchanged. `robot_base_link` remains a fixed compatibility
child; the GNSS value remains an unmeasured placeholder.

### Localization

GNSS contributes absolute position and covariance; valid dual-antenna heading
contributes yaw. The field profile adds IMU roll/pitch and angular rates plus
wheel longitudinal/lateral velocity, then `robot_localization` predicts a
`robot_center_link` state and publishes an odometry/pose stream through the
selector. The sim profile additionally accepts its map-consistent fake IMU yaw.

The committed stationary probe measured 10.0 Hz GNSS/IMU/wheel inputs, 20.0 Hz
EKF odometry, and 20.0 Hz selected pose. Selected-pose header age p95 was
1.83 ms. This demonstrates the fusion chain and prediction cadence, not a
position-error reduction: stationary noiseless simulation has no independent
ground truth and cannot measure field multipath, antenna lever arm, vibration,
or wheel slip.

### Planning

The planning image enumerates every loaded planner and controller server plugin.
Full bringup selects `LaneletRoute + RPP`; manual RViz goals use
`LaneletRoute + RotationShim(RPP)`, and obstacle fallback can select
`SmacLattice`. It also separates `/planning/state`, user-visible
`/service/state`, and health-only `/system/status`, then shows handoffs from
Nav2 to campsite maneuver, return routing, parking, and charging feedback.

### Perception

Simulation exercises the LiDAR-only path: filtered nonground points enter
Euclidean clustering and obstacle publication. The field path adds front-camera
TensorRT YOLOv9-MIT at the configured 5 fps throttle, 2D detections,
camera-bbox LiDAR fusion, campsite tent occupancy, and rear-camera AprilTag
parking. Because the ordinary simulator has no physical camera scene, the
visual intentionally contains no fabricated YOLO boxes or AprilTag pass.

### Sensing

The sensor matrix traces each physical source to its canonical output and
consumer: Vanjee LiDAR and DFKI ground segmentation, seven scalar radars and
fixed-return filtering, front/rear cameras, dual F9P GNSS, IMU, and Ranger wheel
feedback. The final cost grid merges lanelet, LiDAR, radar, and global-path
inputs in the `robot_center_link` window. The ground image uses seeded synthetic
points only to make voxel/cell/slope/inlier behavior readable.

## Regeneration

Run from the repository root after changing any referenced YAML, message, or
evidence JSON:

```bash
python3 camrod_bringup/scripts/render_module_readme_assets.py
```

Render one package while iterating on its README, or repeat `--module` for a
subset:

```bash
python3 camrod_bringup/scripts/render_module_readme_assets.py --module sensing
python3 camrod_bringup/scripts/render_module_readme_assets.py --module sensor-kit
python3 camrod_bringup/scripts/render_module_readme_assets.py \
  --module planning --module localization
```

The renderer writes only under `docs/assets/module-guides/`. Its regression test
renders to a temporary directory and checks all nine PNG dimensions plus the
10-frame GIF:

```bash
pytest -q camrod_bringup/test/test_module_readme_assets.py
```

Required host Python packages are NumPy, PyYAML, Matplotlib, and Pillow. The
renderer reads package-owned configuration as the source of truth; it does not
copy or mutate bringup/package configuration mirrors.

## Replacing Pending Evidence

Capture evidence on the Jetson with the production configuration unchanged.
Store raw JSON/bag/log data first, then add a derived image with its command,
commit, duration, and pass criteria.

Localization timing and pose comparison:

```bash
ros2 run camrod_bringup pose_latency_probe.py \
  --duration 60 \
  --output-json "$HOME/camrod_field_logs/pose-chain.json"
```

Physical front-camera payload and YOLO lifetime:

```bash
ros2 run camrod_bringup camera_payload_probe.py \
  --duration 300 --min-rate-hz 5
```

Full sensor/status snapshot and recovery timeline:

```bash
camrod_bringup/scripts/field_test_tool.sh snapshot
camrod_bringup/scripts/field_test_tool.sh record-recovery
```

A full mission PASS requires campsite arrival, safe local entry, explicit user
return, return route, selected parking method, `PARKED`, and charging feedback.
Until surveyed service-access geometry lets B6 and B12 complete with the full
1.6916 x 1.2700 m safety footprint, the contract animation must remain paired
with the red `ROUND TRIP: NOT DEMONSTRATED` runtime result.
