# CAMROD VIO External Workspace

This directory contains non-ROS external assets used to run Kimera-VIO based localization fallback.

## Contents

- `Kimera-VIO/`: upstream Kimera-VIO source tree (shared library + live executables).
- `third_party/DBoW2/`: bundled DBoW2 source required by `find_package(DBoW2)` in Kimera-VIO.
- `sensor_inspection_tools/`: camera inspection and YAML export utilities.
- `sdk_installers/`: optional vendor SDK installers (Orbbec/ZED).
- `docker/`: container helper scripts and Dockerfiles.

## Why DBoW2 is required

`Kimera-VIO/CMakeLists.txt` declares:

- `find_package(DBoW2 REQUIRED)`

Without a valid `DBoW2Config.cmake`, Kimera-VIO configuration fails.

## Build

From this directory:

```bash
./build_kimera_stack.sh
```

Options:

```bash
./build_kimera_stack.sh --configure-only
./build_kimera_stack.sh --clean
./build_kimera_stack.sh --with-tests
```

Default build excludes `testKimeraVIO` target because OpenCV major-version differences can break upstream test sources.

## Runtime Environment

Use ROS workspace environment for CAMROD nodes, and add this workspace only when running standalone Kimera binaries/tools:

```bash
# CAMROD ROS workspace
source /home/hong/camrod_ws/install/setup.bash

# Optional: standalone Kimera/DBoW2 runtime path setup
source /home/hong/camrod_ws/src/camrod_localization/external/vio_bridge/setup_env.sh
```

## Notes for CAMROD workspace build

- This tree is marked with `COLCON_IGNORE` on purpose.
- CAMROD ROS 2 packages are built with `colcon`.
- Kimera/DBoW2 in this folder are built by `build_kimera_stack.sh`, not by `colcon`.
