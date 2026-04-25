# Kimera-VIO Live Image (amd64)

Image: `lehong/kimera-vio:live-amd64-with-docs`

## Purpose
This image is for live sensor execution on amd64 systems.
It is intended for x86_64 hosts using cameras/sensors in a live environment.

## Included files inside the image
- Dockerfile: `/opt/dockerfiles/Dockerfile.vio_live.amd64`
- README: `/opt/readmes/README.live.amd64.md`

## Pull
```bash
docker pull lehong/kimera-vio:live-amd64-with-docs
```

## Basic Run
```bash
docker run --rm -it \
  --name kimera_live_amd64 \
  --network host \
  --privileged \
  -v /dev:/dev \
  lehong/kimera-vio:live-amd64-with-docs \
  /bin/bash
```

## Run with NVIDIA GPU support
```bash
docker run --rm -it \
  --name kimera_live_amd64 \
  --network host \
  --privileged \
  --gpus all \
  -v /dev:/dev \
  -v /tmp:/tmp \
  lehong/kimera-vio:live-amd64-with-docs \
  /bin/bash
```

## Run with current workspace mounted
```bash
docker run --rm -it \
  --name kimera_live_amd64 \
  --network host \
  --privileged \
  --gpus all \
  -v /dev:/dev \
  -v /tmp:/tmp \
  -v $(pwd):/workspace \
  lehong/kimera-vio:live-amd64-with-docs \
  /bin/bash
```

## Check embedded Dockerfile
```bash
cat /opt/dockerfiles/Dockerfile.vio_live.amd64
```

## Check embedded README
```bash
cat /opt/readmes/README.live.amd64.md
```

## Notes
- This image is for amd64/x86_64 hosts.
- For Jetson or GMSL ZED X Mini usage, use the arm64 live image instead.
- USB ZED/general live sensors usually need `/dev` access and often NVIDIA GPU runtime access.
