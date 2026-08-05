# for_merge Selective Integration

<!-- HH_260804 - Record why only selected imported changes were applied. -->

## 2026-08-05 Operational Follow-up

The imported browser selector is retained. The initial integration checkpoint
selected `operator_ui_window_engine=webkit`, but the final v2.1.4 runtime now
selects `chromium` for touch/IME compatibility and keeps WebKit as an explicit
fallback. Backend readiness and static-bundle caching apply to both paths;
WebKit keeps smooth touch scrolling and its acceleration request. The older CPU
comparison below is historical and has not been repeated on Jetson; browser
flags alone are not a Jetson GPU-use measurement.

The active user-authored map is now revision 15. All map-v14 JSON, PNG, GIF, and
timing results in this document remain immutable historical evidence and are no
longer compared to the active map SHA.

## Comparison Scope

Generated `node_modules`, frontend `build`, Python caches, and test caches were
excluded from semantic comparison.

| Package | Initial comparison | Interpretation |
|---|---:|---|
| `camrod_bringup` | 152 identical, 6 different | Only fullscreen launch propagation was newer in `for_merge` |
| `camrod_ui` | 63 identical, 7 different, 2 imported assets | Kiosk, transport, arrival fallback, endpoint, and icon changes required review |

The review snapshot occupied about 607 MB, mostly imported frontend dependencies
and build output. It was removed from the workspace on 2026-08-05 after the
selected changes, tests, and this comparison record were complete.

## Integrated

| Area | Selected behavior |
|---|---|
| bringup/UI launch | `operator_ui_window_fullscreen=true` flows from the central YAML to the managed browser process; maintenance opt-out remains |
| local window | Renderer selection, isolated Chromium kiosk profile, clean process-group shutdown, and managed WebKit support |
| Guest transport | Serialized WebSocket writes, no await under thread lock, ROS work off the event loop, 10 s heartbeat, 45 s stale close, guaranteed slot release |
| mission arrival | Accepted campsite identity survives transient destination clearing; battery-rejected sites cannot overwrite it |
| HTTP backend | Simple ROS/state endpoints run as synchronous FastAPI handlers outside the event loop |
| frontend | Imported facility/trail PNG icons; restored site-verification virtual keyboard and Guest-driven return idle-screen exit after a second semantic diff audit |

## Current Behavior Retained

- rclpy `call_async()` operator stop and Nav2 cancellation.
- Rapid route-recontact latch and synchronized control/bringup configuration.
- Historical map-v14 evidence, current concise READMEs, and runtime evidence.
  Older `for_merge` copies were not allowed to overwrite these.

## 2026-08-04 Validation Snapshot

- React production build and postbuild install sync: pass.
- UI tests: 24 pass; fullscreen propagation, Chromium selection/command, REST
  handler mode, icon assets, Guest heartbeat, and frontend flow contracts pass.
- Runtime regression check: the imported forced WebKit path held one workstation
  CPU core near 97%. Chromium with the legacy infinite motion cues averaged
  15.8% across browser/GPU/renderer; replacing only those decorative cues with
  static equivalents reduced the settled ten-second average to 0.5%. This is a
  workstation renderer check, not a Jetson performance claim.
- Current production-bundle browser check at 1280x800: destination screen in
  1.1 ms, 40-key verification keypad in 17.6 ms, queued same-frame virtual input
  retained as `B6`, cancel with zero frames, and confirmation with exactly one
  `B6=true` frame.
- Robot UI browser flow: `B6` keypad entry and destination frame pass at
  1920x1080; lowercase physical input normalizes to `B6` at 1280x800. Both
  layouts keep the keypad and confirmation controls inside the viewport.
- Extended browser audit: wrong `B5` input sends nothing and shows feedback;
  correction sends only `B6`; cancel sends nothing; Guest return exits the
  idle screen; the existing `admin`/`1234` diagnostic keypad still authenticates.
- Bringup tests: 141 test results, zero failures.
- Control tests: 56 test results, zero failures.
- Full-stack map-v14 recovery: `[SYSTEM] OK`, three scenarios recorded, final
  Twist zero in every record.

Measured JSON and reproduction details:
[`map-v14-boundary-recovery/`](evidence/v2.1.3/map-v14-boundary-recovery/).

![Map-v14 recovery result](assets/module-guides/control/map-v14-boundary-recovery-contact-sheet.png)

[Open the recovery GIF](assets/module-guides/control/map-v14-boundary-recovery.gif).

![Robot UI site verification keypad](assets/module-guides/ui/robot-ui-site-verification-keypad.png)
