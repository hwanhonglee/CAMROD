# virtual/carla — Unit 2 driving UI sync, 2026-11-26

<!-- HH_261126 - Distinguish display regression checks from closed-loop scenario acceptance. -->
Base: `virtual/carla` at `6c415e9bb`. Reviewed source: `unit2/main` at
`3d459d4a0b32f6c5892f3df5c0c35c3d18037c27`.

| Moved from Unit 2 | Adaptation in CARLA |
| --- | --- |
| `3b25bb005` — keep frequent driving samples out of the kiosk's top-level state, reuse unchanged geometry buffers, and skip unchanged draws | Preserve the virtual manual/resume controls and measured pose/wheel rendering. Keep a 1 s idle refresh and a roughly 30 fps limit when moving. |
| `3d459d4a0` — read-only monitor | Open `/?view=driving-monitor` on the simulator PC (optional `&theme=dark`). Reuse the kiosk's bounded GET/timeout path. Do not mount kiosk controls, take focus from the host, start a WebSocket, or add a service listener. |

Not imported: the Unit 2 site/PIN manager and new mission admission flow; the
C-site map and field NDT/RTK defaults; field LiDAR and docking policy changes.
The existing pinned service-test overlay and CARLA map, sensors, 4WS, and
control defaults are unchanged.

Verification: 322 frontend tests, 233 UI/Python checks, 199 CARLA source
contracts and 21 portable overlay/charging geometry checks passed. The
production frontend verified 58 public assets; the isolated `camrod_ui`
package built and served its installed bundle. Chrome mounted the passive view
and the original kiosk separately and received the model and three wrap
textures. Tests also exercised stale/missing telemetry, dispatch-bound
visibility, unchanged-render suppression, and monitor request cleanup.

The live smoke used restored Woraksan with the exact Ranger and 15 attached
CARLA sensors. In 8 seconds, the bridge delivered 152 clock messages and 75–76
messages per sampled RGB/LiDAR/GNSS/IMU stream. It ran the new UI, not a new
closed-loop mission; the monitor correctly displayed unavailable driving
telemetry without the full localization/control graph. No conclusions about
CPU savings on Jetson, navigation success, or HWP pass/fail follow from this
smoke.
