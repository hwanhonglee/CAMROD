# CAMROD Docker (Bringup, Multi-Arch)

This folder provides a full-workspace Docker image for:
- `ros2 launch camrod_bringup bringup.launch.py`
- amd64 + arm64 multi-arch publish to Docker Hub: `lehong/camrod`

## 1) Build and Push Multi-Arch Image

```bash
cd /home/hong/camrod_ws/src/docker
chmod +x buildx_camrod.sh
./buildx_camrod.sh
```

Default target:
- image: `lehong/camrod`
- tags: `latest` and `YYYYMMDD`
- platforms: `linux/amd64,linux/arm64`

## 2) Optional Custom Build Parameters

```bash
IMAGE_REPO=lehong/camrod \
IMAGE_TAG=v1.0.0 \
PLATFORMS=linux/amd64,linux/arm64 \
WORKSPACE_ROOT=/home/hong/camrod_ws \
./buildx_camrod.sh
```

## 3) Run Bringup Container

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  --gpus all \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID=0 \
  lehong/camrod:latest
```

Default container command:
```bash
ros2 launch camrod_bringup bringup.launch.py
```

## 4) Login Requirement

Before push:
```bash
docker login
```

## 5) Notes

- Build context is workspace root: `/home/hong/camrod_ws`.
- Dockerfile path is: `src/docker/Dockerfile.camrod`.
- Runtime dependencies are installed via `rosdep` during image build.
- The script auto-installs `binfmt` (`tonistiigi/binfmt`) for arm64 emulation.

## 6) If you see `Invalid ELF image for this architecture`

Symptom:
```text
.buildkit_qemu_emulator: /bin/bash: Invalid ELF image for this architecture
```

Fix:
```bash
docker run --privileged --rm tonistiigi/binfmt --install all
```

Then rebuild with a clean builder:
```bash
RESET_BUILDER=1 ./buildx_camrod.sh
```
