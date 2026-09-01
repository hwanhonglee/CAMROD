# CAMROD VIO External Workspace

This directory contains the optional, non-ROS build/runtime helpers used for the
Kimera-VIO localization fallback. It is excluded from the normal CAMROD colcon
build by `COLCON_IGNORE`.

## Contents

- `Kimera-VIO/`: local, untracked checkout of `MIT-SPARK/Kimera-VIO` at
  `ce8c59b7b273ab5ac29db7e5572e1623760e19c7`.
- `third_party/DBoW2/`: local, untracked checkout of `dorian3d/DBoW2` at
  `3924753db6145f12618e7de09b7e6b258db93c6e`.
- `sensor_inspection_tools/`: camera inspection and YAML export utilities.
- `sdk_installers/`: optional vendor SDK installers (Orbbec/ZED).
- `docker/`: container helper scripts and Dockerfiles.

Neither upstream checkout is a Git submodule or vendored source in this
repository. A fresh CAMROD clone therefore has neither directory.

## Why DBoW2 is required

`Kimera-VIO/CMakeLists.txt` declares:

- `find_package(DBoW2 REQUIRED)`

Without a valid `DBoW2Config.cmake`, Kimera-VIO configuration fails. The helper
builds the pinned DBoW2 checkout first and passes its local install prefix to
Kimera-VIO.

## Build

From this directory:

```bash
./build_kimera_stack.sh
```

This explicit command is the only host-side step here that downloads the two
upstream source trees. When a checkout is missing, it fetches the exact commits
listed above into an ordinary nested Git checkout. If an existing checkout has
a different origin, a different commit, local/untracked changes, or is on a
branch instead of detached at the pin, the command stops without rewriting it.

Options:

```bash
./build_kimera_stack.sh --prepare-sources-only
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
- CAMROD ROS 2 packages are built with `colcon`; that normal build performs no
  Kimera/DBoW2 clone or fetch.
- Kimera/DBoW2 in this folder are prepared and built only by the explicitly
  invoked `build_kimera_stack.sh` helper.
- Because the source checkouts are intentionally untracked, do not add them to
  CAMROD commits. Remove the directories directly if the optional VIO stack is
  no longer needed.
