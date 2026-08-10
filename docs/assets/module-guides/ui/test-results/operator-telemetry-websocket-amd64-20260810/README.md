# Operator Telemetry WebSocket AMD64 Check

<!-- HH_260810 - Preserve the post-v2.1.7 transport and lease measurement
without presenting a sensorless x86_64 backend as Jetson acceptance. -->

![Operator telemetry WebSocket AMD64 summary](operator-telemetry-websocket-amd64.png)

![Operator telemetry WebSocket and lease summary](operator-telemetry-websocket-amd64.gif)

## Scope

This record measures the Release-installed `ui_backend_node` with the
`trajectory` view selected, `telemetry_stream_rate_hz=10.0`, and no ROS sensor
publishers. It checks the WebSocket scheduler, payload bound, process overhead,
normal disconnect cleanup, and silent-client lease timeout.

This is not ARM64 acceptance.

It does not measure camera JPEG work, LiDAR/point-cloud processing, live DDS
traffic, WebKit/Chromium rendering, GPU frame pacing, or the final ARM64
8-core/16-GB platform.

## Result

| Check | Measured value |
|---|---:|
| Frames / duration | `201 / 20.126 s` |
| Effective rate | `9.938 Hz` |
| Mean / p95 / max interval | `100.628 / 100.792 / 101.295 ms` |
| Mean / max JSON | `1192.8 / 1193 bytes` |
| Backend CPU, idle / active | `1.00 / 1.12%` of one logical CPU |
| Backend RSS, idle / active | `76,696 / 77,592 KiB` |
| Normal close release detection | `83.3 ms` |
| No-heartbeat lease expiration | `12.078 s` |

The source values are in [`measurement.json`](measurement.json).

The PNG is an exact aggregate summary. The GIF is labelled as a summary
animation and does not reconstruct unrecorded per-frame samples.

## Reproduction

```bash
source /opt/ros/humble/setup.bash
source ~/camrod_ws/install/setup.bash
ros2 launch camrod_ui ui.launch.py \
  enable_operator_ui_window:=false \
  enable_ui_guest:=false \
  ui_port:=18110
```

Connect `ws://127.0.0.1:18110/ws/telemetry?view=trajectory`, send the text
heartbeat `lease` every four seconds, and collect 201 frames. Sample the backend
with `pidstat -r -u -p <ui_backend_pid> 1 8` after subscription discovery has
settled. Close the socket and poll `/api/telemetry` for `session_active=false`.

For the timeout path, connect without sending a heartbeat and verify that
`session_active` returns to false near the configured 12-second lease.

## ARM64 Acceptance Boundary

Repeat the same procedure on the 8-core/16-GB Jetson with physical camera,
LiDAR, radar, GNSS, localization, and control publishers active. Cycle all six
views for 30 minutes and record backend/whole-system CPU, GPU, PSS/RSS, payload,
camera frame pacing, topic rates, disconnect release, and memory growth. That
field result, not this record, decides the deployment stream rate.
