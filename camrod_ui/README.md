# camrod_ui

<!-- HH_260807 - Make the documented renderer match the WebKit field default
while retaining the tested Chromium and auto alternatives. -->
<!-- HH_260807 - Preserve charger-departure authorization and deduplicate destination commands. -->

Robot operator UI, Guest campsite UI, HTTP/WebSocket backends, ROS mission
bridge, diagnostics display, and managed local kiosk.

![Robot and Guest mission/state contract](../docs/assets/module-guides/ui/robot-and-guest-mission-state.png)

## At A Glance

| Surface | User actions | Shared feedback |
|---|---|---|
| Robot UI | Manual engage/stop, campsite dispatch, return, tuning, diagnostics | Service state, command gate, battery, health |
| Guest UI | Campsite selection and return request | Same service state, SOC admission, and safety overlay |
| Backend | Resolves mission key/goal and publishes typed ROS commands | WebSocket state snapshots and arrival events |

## Active Deployment Values

| Item | Full-bringup value |
|---|---:|
| Robot UI | `0.0.0.0:8010` |
| Guest UI | `0.0.0.0:8012` |
| Campsites | `B1..B13` |
| New mission battery | `>= 35%` |
| Battery feedback required | `true` |
| Low-battery current-mission latch | `< 35%` |
| Hard stop authority | control gate at `<= 20%`, not the UI |
| Guest disconnect lock grace | `60 s` |
| Guest heartbeat / stale close | `10 s` / `45 s` |
| Local operator window | fullscreen WebKit by default; Chromium and `auto` explicit alternatives |

## Destination Dispatch

```text
site B<N>
  -> mission key camping_site_<N>
  -> service/operational goal from camping_sites.yaml
  -> /ui/selected_destination
  -> /planning/mission_key + /goal_pose
  -> /platform/drive_enable + /planning/mission_engage
  -> /service/state = MOVING_TO_SITE
```

Unknown battery or SOC below 35% blocks a new destination. If SOC falls below
35% during an active campsite mission, the current site phase finishes and the
UI waits for the normal return request; it does not command immediate motion
while users may be unloading.

## State Display

| Layer | UI label examples | Source |
|---|---|---|
| Operation | Moving to site, Entering site, Waiting for return, Charging | `/service/state` (17 states) |
| Motion authorization | Driving, Standby, Route safety hold | command-gate status |
| Health | System OK, warning, error | `/system/status` and aggregated diagnostics |
| Battery | Ready/block/charging and SOC | `/platform/status` |

Manual engage uses the same command-gate state and therefore displays driving
even when no campsite destination was selected. Operator stop publishes
`OPERATOR_STOPPED`; a previous planning warning is not the operation label.
The active campsite ID is retained separately from the transient destination
ack, so arrival notifications still identify the selected site after departure.

## Actual Browser Runtime

![Robot UI site verification keypad](../docs/assets/module-guides/ui/robot-ui-site-verification-keypad.png)

| Mission-ready dispatch | Route safety overlay |
|---|---|
| ![Guest dispatch ready](../docs/assets/module-guides/ui/guest-mission-dispatch-ready.png) | ![Guest safety hold](../docs/assets/module-guides/ui/guest-route-safety-hold.png) |

`SIM BROWSER CAPTURE`: these are rendered Guest UI screens connected to the
running ROS backend, not generated UI mockups.

| Integration check | Result |
|---|---|
| Initial state | `80%` SOC, ready, 13 sites |
| Destination | Mission key and departure/moving state observed |
| Return | Return state and `MotionOperation.RETURN` observed |
| Safety | `ROUTE_SAFETY_HOLD` overlay observed |
| Cancel | `POST /ui/stop` HTTP 200; all local owners and Nav2 canceled; state `16 OPERATOR_STOPPED` observed |
| Robot site verification | `B6` virtual-key input at 1920x1080 and lowercase physical-key normalization at 1280x800; both produced the same destination frame |
| Robot UI negative paths | Wrong `B5` sends nothing, correction sends `B6`, cancel sends nothing, Guest return exits idle, and diagnostic keypad login remains valid |
| Current production bundle | At 1280x800, destination screen appeared in 1.1 ms and the 40-key verification keypad in 17.6 ms; queued same-frame `B`+`6` remained `B6`, cancel sent zero frames, and confirmation sent one frame |
| WebKit fallback smoke | WebKitGTK 2.50.4 waited for backend readiness, loaded on the first render attempt, and shut down cleanly |
| Historical renderer sample | Before static status cues, forced WebKit was near 97% of one workstation CPU; a later Chromium sample was 0.5% combined |
| Chromium alternative contract | Private kiosk profile, GPU rasterization, zero-copy compositor, backend readiness wait, and process-group shutdown are unit-tested |
| Robot/Guest backend shutdown | Full-graph SIGINT smoke: both backends exited cleanly, with no traceback, failed process, or remaining child |
| Charging recall | Active charger contact no longer overwrites `DEPARTING_CHARGER`; B2/B3 departure completed in the three-cycle service soak |
| Duplicate destination | A repeated WebSocket destination while station exit is pending is idempotent; one maneuver owner and state transition remain |

This validates browser/backend/ROS integration. It is not a physical mission or
collision-safety test. Neither historical renderer sample proves current
Jetson CPU/GPU use; a production Jetson profile is still required.

![Three-cycle UI/service integration](../docs/assets/test_result/v2-1-5-service-validation-20260807/repeated-service-summary.png)

The map-v17 simulation completed charging recall and next-site departure twice
after the initial cycle. Direct backend regression tests pass `30/30`; actual
CAN timing, touchscreen latency, and Jetson kiosk performance remain field work.

<!-- HH_260807 - Record the Humble shutdown-only conversion race separately
from operational backend failures. -->
On ROS 2 Humble, context teardown can raise a `RuntimeError` while converting a
queued message instead of `ExternalShutdownException`. Robot and Guest backends
now treat that exception as a normal shutdown only when `rclpy.ok()` is already
false. A live `RuntimeError` is still re-raised. The full Robot+Guest UI smoke
ended both processes with exit code 0 and left no backend process behind.

The filtered [shutdown and repeated-service record](../docs/assets/test_result/b1-b10-service-endurance-20260807/README.md)
also covers nine successive charger recalls after the seeded B1 handoff. Each
recall reached `DEPARTING_CHARGER`, disconnected simulated charge feedback, and
accepted the next campsite without duplicating the destination or motion owner.

## Key ROS Interfaces

| Direction | Topic | Purpose |
|---|---|---|
| Publish | `/ui/selected_destination` | Typed destination command |
| Publish | `/planning/mission_key` | Semantic mission selection |
| Publish | `/goal_pose` | Site operational goal |
| Publish | `/planning/mission_engage` | Mission motion authorization request |
| Publish | `/platform/drive_enable` | Platform drive-enable request |
| Publish | `/ui/camping_site_operation_request` | Return/adopt operation request |
| Subscribe | `/service/state` | Public lifecycle |
| Subscribe | `/control/cmd_vel_safety_gate/status` | Motion/safety overlay |
| Subscribe | `/platform/status` | SOC, charging, and platform state |
| Subscribe | `/system/diagnostics_agg` | Detailed health display |

## HTTP And WebSocket

| Method/path | Purpose |
|---|---|
| `GET /ui/health` | Backend liveness |
| `GET /ui/state` | Full UI state snapshot |
| `GET /ui/destination` | Current/valid destinations |
| `GET /ui/diagnostics` | Aggregated diagnostic detail |
| `POST /ui/destination?site=B6&run=true` | Select and dispatch a campsite |
| `POST /ui/engage?value=true|false` | Manual engage/disengage |
| `POST /ui/stop` | Operator stop |
| `WS /ws` | Real-time state updates |

Guest frames use one serialized writer. ROS publish work runs outside the
uvicorn event loop, and three missed 15-second receive windows release a stale
single-client slot. The browser heartbeat keeps a healthy idle session active.

## Measured amd64 Renderer A/B

`MEASURED WORKSTATION`: three headless-RViz full-sim runs per renderer on an
Intel i5-12400F/RTX 3060 desktop, with 15 one-second steady-state samples per
run. Only `operator_ui_window_engine` changed.

| Mean | WebKit | Chromium | Chromium - WebKit |
|---|---:|---:|---:|
| Processes | `69.1` | `80.4` | `+11.3` |
| CPU, one-core basis | `89.3%` | `90.8%` | `+1.44 points` |
| PSS | `1926.2 MiB` | `2253.5 MiB` | `+327.3 MiB` |
| Startup to `[SYSTEM] OK` | `26.77 s` | `25.44 s` | `-1.33 s` |
| Controlled stop / no descendants | `3/3` | `3/3` | equal |

On this workstation WebKit is clearly lighter. Chromium's full stack reached
`[SYSTEM] OK` sooner on average, but this is not a browser first-paint/load
measurement. Host GPU utilization is not used because `nvidia-smi` also
included the desktop and personal browser. The field default is WebKit because
the robot image's Snap Chromium does not start; the Jetson
same-scene frame-pacing/GPU/rate comparison remains TODO 8/14. Exact runs are
in [`amd64-container-ab-20260805.json`](../docs/evidence/v2.1.5/runtime-topology/amd64-container-ab-20260805.json).

## Build And Run

```bash
cd ~/camrod_ws/src/camrod_ui/camrod_ui_robot/assets/frontend
npm ci
npm run build

cd ~/camrod_ws
colcon build --packages-select camrod_ui --symlink-install
source install/setup.bash

ros2 launch camrod_ui ui.launch.py
curl http://127.0.0.1:8010/ui/health
curl http://127.0.0.1:8010/ui/state
```

`setup.py` installs the generated React `build/` tree; it does not run npm.
Regenerate that tree before colcon whenever frontend sources or public assets
change. Set `enable_operator_ui_window:=false` on headless hosts, or use
`operator_ui_window_fullscreen:=false` for a resizable maintenance window.
WebKit is the standalone and full-bringup default because the robot's Snap
Chromium currently exits in `snap-confine` before opening a window. The launcher
waits for the backend before opening the first page.

Use `operator_ui_window_engine:=chromium` only on a host where Chromium can
start normally. That mode retains its isolated kiosk profile, GPU rasterization,
zero-copy compositor, and disabled background throttling. `auto` tries Chromium
first and then WebKit; `CAMROD_UI_BROWSER` overrides Chromium executable
selection. Actual Jetson renderer utilization and frame pacing remain field
measurements.

## Network Boundary

Standalone UI and full bringup bind ports 8010 and 8012 to all interfaces for
robot-network access. The current backend has no authentication and permissive
CORS. Do not expose either port to a public or untrusted network; use a trusted
robot LAN, firewalling, or an authenticated tunnel.

Evidence JSON: [`guest-mission-lifecycle.json`](../docs/evidence/v2.1.3/ui/guest-mission-lifecycle.json).
