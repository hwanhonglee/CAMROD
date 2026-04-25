# Disabled VIO Stack (Not Built by colcon)

This folder archives Kimera/VIO-related components that are intentionally excluded
from normal CAMROD ROS 2 workspace builds.

## Contents
- `kimera_vio_bridge/`: legacy ROS-side CSV bridge source.
- `vio_bridge/`: standalone Kimera-VIO build/runtime toolchain.
- `config_archive/`: archived Kimera-related YAML files moved from active packages.

## Why moved here
- Current project decision: do not use Kimera-VIO in production path for now.
- Keep source history/assets together, but avoid accidental compile/link/runtime coupling
  with `camrod_localization` and bringup.

## Build behavior
- `camrod_localization` launch/config no longer references Kimera bridge paths.
- This directory is data/archive-only for now.
