---
name: build-and-run
description: "How to bootstrap, build, and run/test the CAMROD workspace"
metadata: 
  node_type: memory
  type: project
  originSessionId: 874cb916-2ddd-4e00-a4fe-2c0616836bfd
---

Work from the workspace src dir (README uses `~/camrod_ws/src` or `/home/nvidia/camrod_ws/src`; here it's `/home/avg/camrod_develop/src`). Two root wrapper scripts:

- `./setup_camrod.sh` — idempotent bootstrap: clones/keeps `external/` repos, installs system deps incl. SocketCAN tools, skips Jetson-only docking pkgs on x86_64, runs rosdep (`--no-rosdep` to skip, `--update` to update externals).
- `./colcon_build.sh` — builds source pkgs + all `external/` roots; runs `npm install`/`npm run build` for the robot UI **before** packaging `camrod_ui`; keeps `camrod_parking` in normal source graph. `--packages-up-to camrod_bringup` for targeted build.

Then `source install/setup.bash`.

Run:
- Full stack: `ros2 launch camrod_bringup bringup.launch.py`
- Sim + RViz: `ros2 launch camrod_bringup bringup.launch.py sim:=true rviz:=true` (sim uses fake sensors + a sim graph manifest so no hardware nodes required)
- Real HW: `sim:=false`; override map with `map_path:=/path/to.osm`
- Individual modules: `ros2 launch camrod_<pkg> <pkg>.launch.py`
- UI only: `ros2 launch camrod_ui ui.launch.py ui_host:=0.0.0.0 ui_port:=8010` → http://<ip>:8010
- Parking method (mutually exclusive): `parking_method:=rule_based` (camrod_parking) or `docking` (camrod_docking)

Docker: `Dockerfile.camrod.amd64` / `.arm64` in src root.

Sim validation runner: `ros2 run camrod_bringup sim_validation_runner.py --ros-args -p ...` — validates topic Hz, radar direction topics, directional LiDAR/Radar cost-stops, manual goal nav, and camping-site flow (crab/rotate/unload/return + drop-zone parking). Params: `run_obstacle_replan`, `skip_manual_goal`, `run_camping`, `camping_wait_drop_zone`, `camping_timeout_s`, `report_file`.

Gate unit test: `camrod_planning/test/test_cmd_vel_gate_logic.py`. Note: `colcon test --packages-select camrod_planning` also runs package-wide ament lint; lint failures from vendored `external/nav2_*` are lint-scope, not runtime failures.

Config values are kept **synchronized** across `setup_camrod.sh`, `colcon_build.sh`, READMEs, bringup defaults (`camrod_bringup/config/<pkg>/`), and each package's own launch/node defaults — so a wrapper build installs the same field-tuned values as package-level launches. Change tuning in all synced locations. See [[build-and-run]] partner [[field-baseline-and-conventions]].
