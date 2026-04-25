# Docker Live Guide

This setup is intended to be portable across different PCs.

## Prerequisites

- Docker installed
- NVIDIA driver + NVIDIA Container Toolkit (for `--gpus all`)
- Internet access (if local SDK installer files are not provided)

## Local SDK installers (optional but recommended)

Place files under `./sdk_installers`:

- `ZED_SDK_Ubuntu22_cuda12.8_tensorrt10.9_v5.1.2.zstd.run`
- `OrbbecSDK_v2.7.6_amd64.deb`

If files are missing, Dockerfile tries online fallback:

- Orbbec: direct GitHub release URL
- ZED: direct URL from `ZED_SDK_RUN_URL` or auto-discovery from release page

## Build

```bash
cd /home/hong/camrod_ws/src/camrod_localization/external/vio_bridge
chmod +x docker/build_live.sh docker/run_live.sh
./docker/build_live.sh
```

`build_live.sh`는 host 아키텍처를 자동 감지해서 아래 파일 중 하나를 사용합니다.
- `Dockerfile.vio_live.amd64`
- `Dockerfile.vio_live.arm64`

Optional build-time env vars:

- `IMAGE_NAME` (default: `kimera-vio:live`)
- `BASE_IMAGE` (default: `nvidia/cuda:12.8.0-devel-ubuntu22.04`)
- `INSTALL_ZED_SDK` (`ON|OFF`)
- `INSTALL_ORBBEC_SDK` (`ON|OFF`)
- `KIMERA_VIO_USE_ZED` (`ON|OFF`)
- `KIMERA_VIO_USE_ORBBEC` (`ON|OFF`)
- `INSTALL_ROS2` (`ON|OFF`)
- `KIMERA_VIO_USE_ROS2` (`ON|OFF`)
- `KIMERA_BUILD_JOBS` (arm64 default: `2`, amd64 default: `nproc`)
- `KIMERA_DO_INSTALL` (arm64 default: `OFF`, amd64 default: `ON`)
- `ZED_SDK_RUN_URL` (direct `.run` URL, recommended when online fallback is needed)
- `ORBBEC_SDK_DEB_URL` (override Orbbec `.deb` URL)

Example:

```bash
IMAGE_NAME=kimera-vio:live \
ZED_SDK_RUN_URL="https://download.stereolabs.com/..." \
./docker/build_live.sh
```

arm64를 x86 host에서 buildx로 빌드할 때는 아래 스크립트를 권장합니다.
(`buildkit EOF/GOAWAY` 완화를 위해 arm64 빌드 시 amd64 ZED installer를 임시 숨김 후 자동 복구)

```bash
cd /home/hong/camrod_ws/src/camrod_localization/external/vio_bridge
chmod +x docker/build_live_arm64_buildx.sh
./docker/build_live_arm64_buildx.sh
```

## Build + Push (amd64 + arm64)

```bash
cd /home/hong/camrod_ws/src/camrod_localization/external/vio_bridge
docker login
chmod +x docker/buildx_push_live_multiarch.sh
IMAGE_REPO=lehong/kimera-vio TAG=live ./docker/buildx_push_live_multiarch.sh
```

이 스크립트는:
- `Dockerfile.vio_live.amd64`로 `:live-amd64` 빌드/푸시
- `Dockerfile.vio_live.arm64`로 `:live-arm64` 빌드/푸시
- 최종 manifest 태그 `:live` 갱신

## Run

```bash
cd /home/hong/camrod_ws/src/camrod_localization/external/vio_bridge
./docker/run_live.sh
```

Optional run-time env vars:

- `CONTAINER_NAME` (default: `kimera_live`)
- `WORKDIR_IN_CONTAINER` (default: `/kimera_ws`)
- `USE_GUI` (`1|0`, default: `1`)
- `NETWORK_MODE` (default: `host`)
- `NVIDIA_VISIBLE_DEVICES` (default: `all`)
- `NVIDIA_DRIVER_CAPABILITIES` (default: `all`)
- `AUTO_REMOVE` (`1|0`, default: `1`)
- `MOUNT_WORKSPACE` (`1|0`, default: `0`)
- `OUTPUT_VOLUME_NAME` (default: `${CONTAINER_NAME}_output`)
- `OUTPUT_BIND_DIR` (optional host path for `Kimera-VIO/output`)

## Run without local workspace

If another PC has only the image and not this repository, run it directly:

```bash
docker run -it --rm \
  --name kimera_live \
  --network host \
  --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  --privileged \
  -v /run/udev:/run/udev:ro \
  -v kimera_live_output:/kimera_ws/Kimera-VIO/output \
  -w /kimera_ws \
  kimera-vio:live \
  /bin/bash
```

Inside the container:

```bash
cd /kimera_ws/Kimera-VIO/build
./orbbecDumpCalibration --output_folder_path ../params/Orbbec_live --template_folder_path ../params/Orbbec --orbbec_address 192.168.1.10 --orbbec_port 8090
./orbbecLiveVIO --params_folder_path ../params/Orbbec_live --orbbec_address 192.168.1.10 --orbbec_port 8090 --orbbec_enable_visualization=false --orbbec_enable_csv_log=false
```

## Notes

- Default mode does not mount your host workspace.
- The image already contains `Kimera-VIO`, `sensor_inspection_tools`, docs, and built `build/` folders.
- This keeps local `/home/hong/camrod_ws/src/camrod_localization/external/vio_bridge/.../build` and container `/kimera_ws/.../build` completely separate while preserving the same folder names.
- By default only `Kimera-VIO/output` is persisted through a Docker named volume.
- If you want host-side CSV files directly, set `OUTPUT_BIND_DIR=/some/host/path`.
- If you explicitly want developer-style host workspace shadowing, set `MOUNT_WORKSPACE=1`. This is off by default because it couples host and container build trees again.
- Orbbec `.deb` installs to `/opt/OrbbecSDK_v*`; Dockerfile auto-links it to `/usr/local/OrbbecSDK` for CMake compatibility.
- ZED build requires CUDA headers (`cuda.h`). Default base image already provides CUDA 12.8 devel headers.
- ZED runtime needs NVIDIA video libs (`libnvcuvid.so.1`, `libnvidia-encode.so.1`), so run script sets `NVIDIA_DRIVER_CAPABILITIES=all` by default.
- Ethernet Orbbec discovery uses GVCP/broadcast, so run script defaults to `--network host`.
- If GUI permission is denied, run once on host:
  - `xhost +local:root`
