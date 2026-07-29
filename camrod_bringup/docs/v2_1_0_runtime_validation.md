# CAMROD v2.1.0 Runtime Validation

<!-- HH_260729 - Separate evidence from existing robot logs, offline
     code/build checks, and field validation that has not yet been run. -->

Date: 2026-07-29
Timezone: KST (`Asia/Seoul`)

## 1. Validation boundary

No robot bringup or motion test was rerun while preparing this document.
Runtime findings below are an audit of existing ROS logs only. Source/config
changes and tests performed without robot hardware are recorded separately in
Section 5. Remaining robot checks are in Section 6.

Raw ROS logs and the core dump are not copied into this repository. This page
records their original paths, timestamps, and summary counts so that the
evidence remains auditable on the test computer.

Current conclusion:

- `enable_radar:=false` was effective in both audited runs. LEFT2 `0.801 m` is
  the deliberate no-target heartbeat above its `0.800 m` maximum, not a radar
  obstacle.
- The 46 stops in the latest run were
  `lanelet_footprint_cost`, not radar stops.
- The front physical camera did start, but a front dummy publisher was also
  started on the same compressed topic. YOLO then aborted the shared component
  container while decoding an empty image, so the front/YOLO path was not
  healthy.
- The rear physical camera remained alive until operator shutdown, but its
  measured raw-image rate was below the configured 10 FPS.

## 2. Audited ROS runs

The ROS log base is:

```text
/home/nvidia/.ros/log
```

| Run | ROS launch-log directory | Launch start | Operator SIGINT | Last teardown record |
|---|---|---:|---:|---:|
| Latest | `2026-07-29-17-01-52-538414-EAC6k-Orin-174724` | 17:01:52.540 | 17:06:43.900 | 17:06:52.502 |
| Previous | `2026-07-29-16-58-15-985404-EAC6k-Orin-167240` | 16:58:15.987 | 17:00:19.380 | 17:00:26.129 |

The component logs used for the counts are:

| Evidence | Latest | Previous |
|---|---|---|
| Radar dummy | `python3_175238_1785312123677.log` | `python3_167753_1785311907006.log` |
| Radar cost grid | `radar_cost_grid_node_175241_1785312122754.log` | `radar_cost_grid_node_167769_1785311906284.log` |
| Command safety gate | `cmd_vel_safety_gate_node_175700_1785312127611.log` | `cmd_vel_safety_gate_node_168214_1785311910830.log` |
| Front camera + YOLO container | `component_container_mt_175152_1785312121441.log` | container output is represented by the previous `launch.log` |
| Front sensing dummy | `python3_175173_1785312123592.log` | `python3_167690_1785311906865.log` |
| Rear camera | `camera_rear_publisher_node_175175_1785312122923.log` | `camera_rear_publisher_node_167692_1785311906274.log` |
| System diagnostic | `system_diagnostic_node_176155_1785312138079.log` | `system_diagnostic_node_168686_1785311921001.log` |

## 3. Existing-log audit: radar and stop source

<!-- HH_260729 - A dummy Range marker is transport evidence, not obstacle
     evidence. Attribute a stop only from the command-gate reason. -->

### 3.1 `enable_radar:=false` was active

Both launch logs started `radar_dummy_publisher.py` and
`radar_cost_grid_node`; neither contains a `sen0592_radar_node` process.
Each radar dummy log states that physical SEN0592 polling is disabled and that
seven no-target heartbeats are published at 10 Hz.

The system diagnostic provides a second independent check:

| Run | LEFT2 `DUMMY DATA` records | All seven-channel dummy records |
|---|---:|---:|
| Latest | 75 | 525 (`75 x 7`) |
| Previous | 32 | 224 (`32 x 7`) |

Therefore a visible LEFT2 range/diagnostic in these runs does not mean that
USB5 was polled or that LEFT2 detected a nearby obstacle.

### 3.2 Why LEFT2 reports `0.801 m`

The configured LEFT2 software maximum is `0.800 m`, and the dummy no-target
epsilon is `0.001 m`:

```text
LEFT2 dummy range = max_range + epsilon
                  = 0.800 m + 0.001 m
                  = 0.801 m
```

The message retains `max_range=0.800 m`; the cost grid rejects
`range > max_range`. Thus `0.801 m` is intentionally outside the valid obstacle
interval. It keeps topic freshness visible without painting radar cost.

### 3.3 The actual latest stop source

`cmd_vel_safety_gate_node_175700_1785312127611.log` contains exactly 46:

```text
cmd_vel cost stop: lanelet_footprint_cost
```

They span 17:04:21.222 through 17:06:03.625 KST. There is no radar stop reason
in that file. The previous run's command-gate log has no cost-stop record.

The latest motion block must therefore be investigated as a raw lanelet/map
footprint boundary condition. Nearby real obstacles could not have entered
through the disabled radar path in these two runs.

## 4. Existing-log audit: front camera, YOLO, and rear camera

<!-- HH_260729 - Distinguish a live topic from a healthy physical source.
     A dummy image can keep a topic visible after the physical container dies. -->

### 4.1 Front physical and dummy publishers overlapped

Both runs show the same incorrect ownership sequence:

| Event | Latest | Previous |
|---|---:|---:|
| Physical front component loaded | 17:02:03.408 | 16:58:26.877 |
| Front dummy announced at 2 Hz | 17:02:03.998 | 16:58:27.224 |
| YOLO component loaded | 17:02:04.083 | 16:58:27.506 |
| Shared container exited with `-6` (`SIGABRT`) | 17:02:08.846 | 16:58:31.049 |

The latest component log additionally confirms `/dev/video0` opened through
GStreamer, the front publisher started with a 10 FPS target, and TensorRT YOLO
initialized with compressed transport. The launch then also enabled the front
dummy publisher on the same canonical compressed topic:

```text
/sensing/camera/econ_front/image_rect/compressed
```

Consequently, topic presence alone could not distinguish a physical front
frame from the dummy stream. After the shared container aborted, the physical
front publisher and YOLO were both gone while the separate dummy process could
continue publishing.

### 4.2 Direct abort mechanism and attribution limit

The existing crash report is:

```text
/var/crash/_opt_ros_humble_lib_rclcpp_components_component_container_mt.1000.crash
```

Its metadata is dated 2026-07-29 16:28:15 KST and records signal 6. Core-stack
inspection reaches:

```text
YOLO compressed_image_callback
  -> cv_bridge::toCvCopy(CompressedImage)
  -> cv::cvtColor
  -> uncaught termination
  -> SIGABRT
```

OpenCV reports an empty-source assertion (`!_src.empty()`) in `cvtColor`.
Therefore the direct abort mechanism is confirmed: compressed-image decode
produced an empty matrix and the old YOLO callback did not validate the payload,
validate the decoded image, or contain the exception.

The core cannot identify which publisher supplied that exact frame. Attribution
must remain explicit:

- The physical front and dummy publishers were both active on the same topic.
- The dummy JPEG payload deployed for these runs reproduces an empty result
  with the deployed OpenCV 4.8 decoder, making it the strongest direct trigger
  found offline.
- The process also loads OpenCV 4.8 and 4.5 libraries together, and the physical
  NVJPEG path did not yet provide field evidence that every payload was
  non-empty and decodable. Those remain separate risks to verify.

The system diagnostic recorded no
`/perception/camera/detections_2d` messages 74 times in the latest run and
32 times in the previous run. YOLO initialization alone was therefore not a
successful perception test.

### 4.3 Rear camera survived but was slow

The rear publisher opened `/dev/video1`, reported
`1920x1080 @10fps` with a 2 Hz compressed stream, and remained alive until the
operator SIGINT in both runs:

- latest: ready at 17:02:04.759, SIGINT at 17:06:43.902;
- previous: ready at 16:58:28.181, SIGINT at 17:00:19.380.

The diagnostic checker nevertheless observed:

| Run | Recorded raw-image FPS values, target 10 FPS |
|---|---|
| Latest | `1.1` to `7.0` FPS (7 records) |
| Previous | `2.7`, `3.1`, and `6.1` FPS |

The old checker mislabeled the intermediate `7.0` and `6.1` cases as
`Encoding mismatch`; their recorded encoding/camera-info fields were otherwise
valid, so these are rate failures. Logs prove publisher survival and message
arrival, but do not prove that the displayed rear image content was visually
correct.

## 5. Offline code and build validation

<!-- HH_260729 - These checks do not claim that cameras, radar, or robot motion
     were validated against real hardware after the fixes. -->

Worktree remediation being integrated for this finding includes:

- freezing the parent-scope camera+YOLO ownership result and passing an explicit
  external-front-source flag so sensing cannot start a duplicate front dummy;
- replacing the damaged dummy JPEG with a decoder-validated image;
- rejecting null/empty compressed payloads and empty decode results in YOLO,
  with contained/throttled decode exceptions instead of process termination;
- reporting rear rate degradation as an FPS warning rather than an encoding
  mismatch;
- publishing fresh global/per-channel radar dummy markers and applying the
  one-second dummy cost barrier.

Completed non-hardware regression command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  camrod_bringup/test/test_sensor_dummy_launch_routing.py \
  camrod_bringup/test/test_radar_defaults_sync.py \
  camrod_sensing/test/test_radar_dummy_publisher.py
```

Result: **46 passed**.

Final non-hardware integration also completed:

- seven affected CAMROD packages built successfully:
  `camrod_sensing`, `camrod_bringup`, `camrod_platform`,
  `camrod_localization`, `camrod_perception`, `camrod_system`, and
  `camrod_control`;
- the separately discovered vendored `yolov9mit_ros` package built
  successfully;
- all 38 CTest suite entries from those seven packages passed with zero
  failures; and
- `field_test_tool.sh config` reported final package/bringup/source/install
  configuration synchronization as OK.

The isolated camera callback smoke also used `ROS_DOMAIN_ID=88`: corrupt and
empty JPEG inputs produced detailed throttled errors without terminating YOLO,
then three valid replacement 1×1 JPEG frames were accepted before normal
shutdown.

## 6. Real-robot TODO — not executed today

<!-- HH_260729 - Keep motion disabled until source ownership and decoded-image
     health are proven. -->

1. Before field bringup, remove the OpenCV 4.8/4.5 ABI mixture from the shared
   camera+YOLO process, or demonstrate with an isolated sustained decode test
   that the selected library set is safe.
2. Validate every physical front NVJPEG status, returned byte count, and JPEG
   decode result; a zero/invalid payload must be dropped and diagnosed.
3. Start with motion gates closed and verify exactly one publisher owns
   `/sensing/camera/econ_front/image_rect/compressed`. The front dummy-active
   marker must not be fresh while the physical component owns the camera.
4. Keep the front camera+YOLO container alive for at least five minutes.
   Confirm front compressed images are visually valid, decode errors remain
   zero, and `/perception/camera/detections_2d` publishes for a known target.
5. Confirm `/dev/video1` rear raw images are visually valid and near 10 Hz, with
   the rear compressed monitoring stream near 2 Hz.
6. Repeat the radar-disabled check: no `sen0592_radar_node`, all seven channels
   explicitly dummy, LEFT2 `range=0.801 m > max_range=0.800 m`, clear radar
   obstacle evidence, and no radar-derived command stop.
7. If `lanelet_footprint_cost` returns in a verified clear route corridor,
   capture robot pose, the planning-boundary polygon, and the exact raw lanelet
   grid cell before changing thresholds.
8. Only after the checks above pass, perform supervised obstacle and motion
   tests with the physical emergency stop available.

## 7. Release interpretation

This audit diagnoses the two existing radar-disabled runs; it does not certify
post-fix real-robot operation. Source/build/config integration is complete, but
v2.1.0 runtime acceptance remains pending until the Section 6 field checks are
recorded.
