# CAMROD Docker Guide

This folder is now intentionally minimal and focused on one production path:
**build and run the full CAMROD bringup image**.

## Why There Were Many Similar Files

Historically, this folder had mixed experiments:

- full-stack image drafts
- per-module image drafts
- local run helper scripts
- compose variants

Those overlapped in responsibility and made maintenance confusing.

This has been consolidated into one supported flow:

- one main Dockerfile (`Dockerfile.camrod`)
- one entrypoint (`entrypoint.camrod.sh`)
- one build/push helper (`buildx_camrod.sh`)

## Removed Legacy Files

The following files were removed because they were no longer part of the active build/deploy path:

- `Dockerfile.base`
- `Dockerfile.module`
- `build_module.sh`
- `run_module.sh`
- `compose.modules.yaml`

## Files and Roles

- `Dockerfile.camrod`
  - Multi-stage Dockerfile for the full workspace runtime image.
  - Builds on ROS 2 Humble (`ros:humble`) and supports amd64/arm64 via Buildx.
  - Installs dependencies with `rosdep`, builds CAMROD packages, and sets default command:
    - `ros2 launch camrod_bringup bringup.launch.py`
  - `rosdep` is constrained to runtime/build dependency types only:
    - `build, buildtool, exec` (test/doc dependencies excluded for faster CI/container builds)

- `buildx_camrod.sh`
  - Build/push helper script for Docker Buildx.
  - Supports single-arch (`linux/arm64`) or multi-arch (`linux/amd64,linux/arm64`) publishing.
  - Default image repo: `lehong/camrod`.

- `entrypoint.camrod.sh`
  - Container entrypoint.
  - Sources ROS/workspace setup and executes the container command.

## Build and Push

### A) Push arm64 only (quick)

```bash
cd /home/camrod_ws/src/docker
IMAGE_REPO=lehong/camrod \
IMAGE_TAG=v1.1-arm64 \
PLATFORMS=linux/arm64 \
WORKSPACE_ROOT=/home/camrod_ws \
./buildx_camrod.sh
```

### B) Push multi-arch (amd64 + arm64)

```bash
cd /home/camrod_ws/src/docker
IMAGE_REPO=lehong/camrod \
IMAGE_TAG=v1.1 \
PLATFORMS=linux/amd64,linux/arm64 \
WORKSPACE_ROOT=/home/camrod_ws \
./buildx_camrod.sh
```

## Run

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  --gpus all \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID=0 \
  lehong/camrod:v1.1
```

## Notes

- Build context must be workspace root (`/home/camrod_ws`).
- Dockerfile path is `src/docker/Dockerfile.camrod`.
- If arm64 build fails with binfmt/qemu issues, retry with:

```bash
RESET_BUILDER=1 ./buildx_camrod.sh
```
