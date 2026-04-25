# Kimera-VIO Live Image (arm64)

Image: `lehong/kimera-vio:live-arm64-with-docs`

## Purpose
This image is for live sensor execution on arm64 systems.
It is intended especially for Jetson-based deployment.

## Included files inside the image
- Dockerfile: `/opt/dockerfiles/Dockerfile.vio_live.arm64`
- README: `/opt/readmes/README.live.arm64.md`

## Pull
```bash
docker pull lehong/kimera-vio:live-arm64-with-docs
```

## Basic Run
```bash
docker run --rm -it \
  --name kimera_live_arm64 \
  --network host \
  --privileged \
  -v /dev:/dev \
  lehong/kimera-vio:live-arm64-with-docs \
  /bin/bash
```

## Jetson Run with NVIDIA runtime
```bash
docker run --rm -it \
  --name kimera_live_arm64 \
  --runtime nvidia \
  --network host \
  --privileged \
  -v /dev:/dev \
  -v /tmp:/tmp \
  lehong/kimera-vio:live-arm64-with-docs \
  /bin/bash
```

## GMSL / ZED X Mini Run (Jetson)
```bash
docker run --rm -it \
  --name kimera_live_arm64 \
  --runtime nvidia \
  --network host \
  --privileged \
  -v /dev:/dev \
  -v /tmp:/tmp \
  -v /var/nvidia/nvcam/settings:/var/nvidia/nvcam/settings \
  -v /etc/systemd/system/zed_x_daemon.service:/etc/systemd/system/zed_x_daemon.service \
  -v /usr/local/zed/resources:/usr/local/zed/resources \
  lehong/kimera-vio:live-arm64-with-docs \
  /bin/bash
```

## Run with current workspace mounted
```bash
docker run --rm -it \
  --name kimera_live_arm64 \
  --runtime nvidia \
  --network host \
  --privileged \
  -v /dev:/dev \
  -v /tmp:/tmp \
  -v $(pwd):/workspace \
  lehong/kimera-vio:live-arm64-with-docs \
  /bin/bash
```

## Check embedded Dockerfile
```bash
cat /opt/dockerfiles/Dockerfile.vio_live.arm64
```

## Check embedded README
```bash
cat /opt/readmes/README.live.arm64.md
```

## Notes
- This image is for arm64, especially Jetson environments.
- ZED X Mini is a GMSL camera, so host-side Jetson/ZED Link driver setup is required.
- GMSL driver components should be prepared on the host side, not installed only inside the container.
- If camera detection fails, check JetPack/L4T compatibility, device mounts, and daemon/service status on the host.
