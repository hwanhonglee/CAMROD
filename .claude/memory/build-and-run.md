---
name: build-and-run
description: "How to bootstrap, build, and run/test the CAMROD workspace"
metadata: 
  node_type: memory
  type: project
  originSessionId: 874cb916-2ddd-4e00-a4fe-2c0616836bfd
---

<!-- HH_260720 - Update build and launch instructions for the consolidated control package. -->
Work from `/home/hong/camrod_ws/src`. Two root wrapper scripts are provided:

- `./setup_camrod.sh` — idempotent bootstrap for package dependencies, source dependencies, and SocketCAN tools (`--no-rosdep` skips rosdep; `--update` updates external repositories).
- `./colcon_build.sh` — builds source packages and required `external/` roots; it builds the robot UI before packaging `camrod_ui`. Use `--packages-up-to camrod_bringup` for a targeted stack build.

Then `source install/setup.bash`.

Run:
- Full stack: `ros2 launch camrod_bringup bringup.launch.py`
- Sim + RViz: `ros2 launch camrod_bringup bringup.launch.py sim:=true rviz:=true` (sim uses fake sensors + a sim graph manifest so no hardware nodes required)
- Real HW: `sim:=false`; override map with `map_path:=/path/to.osm`
- Individual modules: `ros2 launch camrod_<pkg> <pkg>.launch.py`
- UI only: `ros2 launch camrod_ui ui.launch.py ui_host:=0.0.0.0 ui_port:=8010` → http://<ip>:8010
- Parking method: `parking_method:=reverse` or `parking_method:=apriltag`; exactly one controller is launched from `camrod_control`.

Docker: `Dockerfile.camrod.amd64` / `.arm64` in src root.

Sim validation runner: `ros2 run camrod_bringup sim_validation_runner.py --ros-args -p ...` validates topic rates, directional obstacle gates, manual navigation, campsite crab/rotate/unload/return, and drop-zone parking.

Gate unit tests: `camrod_control/test/test_cmd_vel_safety_gate_logic.py` and `test_parking_geometry.py`.

Package configs are the ownership source; `camrod_bringup/config/<pkg>/` contains deployment overrides. Keep intentional overrides aligned when changing a runtime contract. See [[field-baseline-and-conventions]].
