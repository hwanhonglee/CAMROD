# Committed UI portability verification — 2026-10-06

This evidence is a browser render of synthetic UI fixtures, **not real driving**,
not CARLA driving, and not proof of target-machine GPU performance.

## Source and isolation

- Source: committed develop/v2.3.0 revision `0387c5668`; full SHA is in
  `portability_report.json`.
- Only `git archive 0387c5668 camrod_ui` was extracted into
  `/tmp/camrod-committed-ui.BRJDk4/workspace/src`.
- No existing frontend `build/`, installed assets, local public assets, or
  `node_modules/` were copied into that workspace.
- Node `v18.20.8`, npm `10.8.2`, Google Chrome `154.0.8037.57`.
- The browser had its own profile `/tmp/camrod-committed-chrome.hl2xAU`, local
  preview port `8030`, and debugging port `9231`. SwiftShader WebGL2 was used
  explicitly to avoid contention with the live CARLA GPU session.
- Both test-owned server/browser processes were stopped after capture. The
  isolated source/build/profile directories remain available for inspection.

## Commands executed

```bash
git archive 0387c5668 camrod_ui | tar -x -C /tmp/camrod-committed-ui.BRJDk4/workspace/src
cd /tmp/camrod-committed-ui.BRJDk4/workspace/src/camrod_ui/camrod_ui_robot/assets/frontend
npm ci --prefer-offline --no-audit --no-fund
npm run build
```

Dependency installation completed with 1,297 packages. It emitted deprecated
dependency warnings, but did not fail. Production compilation succeeded with
`main.73f3317d.js` and `main.0afe7b1b.css`. This does not claim a security audit:
`npm ci` was intentionally run with `--no-audit`.

```bash
python3 -m http.server 8030 --bind 127.0.0.1 --directory /tmp/camrod-committed-ui.BRJDk4/workspace/src/camrod_ui/camrod_ui_robot/assets/frontend/build
/usr/bin/google-chrome-stable --headless=new --no-sandbox --no-first-run --no-default-browser-check --remote-debugging-address=127.0.0.1 --remote-debugging-port=9231 --user-data-dir=/tmp/camrod-committed-chrome.hl2xAU --window-size=1920,1080 --use-gl=angle --use-angle=swiftshader --enable-unsafe-swiftshader 'http://127.0.0.1:8030/?driving_preview=1'
python3 /tmp/camrod-committed-ui.BRJDk4/verify_committed_ui.py
cd /home/hong/camrod_ws/src
python3 camrod_ui/tools/capture_navigation_smoothing.py --animate --debug-port 9231 --output /home/hong/camrod_ws/src/docs/evidence/runtime_portability_20261006/committed_ui
```

`verify_committed_ui.py` remains beside the isolated workspace. It loads the
existing DevTools transport helper from the main source checkout only as test
tooling; application source/assets are served exclusively from the fresh archive.

## Results

- All 58 public files other than the transformed React `index.html` returned
  HTTP 200 and matched byte-for-byte across source, compiled build, and HTTP.
- GLB status reached `ready`; `wrap_front`, `wrap_rear`, `wrap_left`, and
  `wrap_right` all loaded in light/dark themes and the four detail views.
- Existing home logo and four menu illustrations loaded with nonzero image sizes.
- No JavaScript runtime exception, console API error, failed captured network
  request, control/API request, or WebSocket connection occurred in the preview.
- **One minor omission was observed:** browser-requested `/favicon.ico` is not
  present and returns 404. It affects the browser tab icon, not navigation/home
  rendering. Browser log and explicit HTTP probe are preserved in the report.

## Evidence

- `portability_report.json`: source SHA, 58 file checksums, HTTP checks, model/wrap
  state, browser logs, four view checks, and home image checks.
- `capture.json`: synthetic light/dark navigation state and API/runtime checks.
- `verified_navigation.png`, `light.png`, `dark.png`: navigation screenshots.
- `detail_view_0.png` through `detail_view_3.png`: rear/right, front/right,
  front/left, rear/left views.
- `existing_home.png`: existing home menu with original image assets.
- `navigation_preview.gif`: 48-frame, 4 fps, 960-pixel-wide synthetic preview;
  originals remain in `frames/`.

The main checkout's tracked application source was not edited. This validates
frontend build and image/model portability, not a fresh ROS installation or
actual ROS sensor/route behavior; those require separate runtime verification.
