#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

IMAGE_NAME="${IMAGE_NAME:-kimera-vio:live}"
CONTAINER_NAME="${CONTAINER_NAME:-kimera_live}"
WORKDIR_IN_CONTAINER="${WORKDIR_IN_CONTAINER:-/kimera_ws}"
USE_GUI="${USE_GUI:-1}"
NETWORK_MODE="${NETWORK_MODE:-host}"
NVIDIA_VISIBLE_DEVICES="${NVIDIA_VISIBLE_DEVICES:-all}"
NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES:-all}"
AUTO_REMOVE="${AUTO_REMOVE:-1}"
MOUNT_WORKSPACE="${MOUNT_WORKSPACE:-0}"
OUTPUT_VOLUME_NAME="${OUTPUT_VOLUME_NAME:-${CONTAINER_NAME}_output}"
OUTPUT_BIND_DIR="${OUTPUT_BIND_DIR:-}"

if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
  echo "[ERROR] Image not found locally: ${IMAGE_NAME}"
  echo "        Build first: ./docker/build_live.sh"
  exit 1
fi

docker_args=(
  run
  -it
  --name "${CONTAINER_NAME}"
  --network "${NETWORK_MODE}"
  --gpus all
  -e "NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES}"
  -e "NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES}"
  --privileged
  -v /run/udev:/run/udev:ro
  -w "${WORKDIR_IN_CONTAINER}"
)

if [[ "${AUTO_REMOVE}" == "1" ]]; then
  docker_args+=(--rm)
fi

if [[ -n "${OUTPUT_BIND_DIR}" ]]; then
  mkdir -p "${OUTPUT_BIND_DIR}"
  docker_args+=(
    -v "${OUTPUT_BIND_DIR}:${WORKDIR_IN_CONTAINER}/Kimera-VIO/output"
  )
else
  docker_args+=(
    -v "${OUTPUT_VOLUME_NAME}:${WORKDIR_IN_CONTAINER}/Kimera-VIO/output"
  )
fi

if [[ "${MOUNT_WORKSPACE}" == "1" ]]; then
  docker_args+=(
    -v "${ROOT_DIR}:${WORKDIR_IN_CONTAINER}"
  )
fi

if [[ "${USE_GUI}" == "1" ]]; then
  docker_args+=(
    -e DISPLAY
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  )
fi

docker_args+=("${IMAGE_NAME}" /bin/bash)

echo "[INFO] Starting container: ${CONTAINER_NAME}"
echo "[INFO] Workspace mode: $([[ "${MOUNT_WORKSPACE}" == "1" ]] && echo "host-mounted" || echo "image-internal")"
if [[ -n "${OUTPUT_BIND_DIR}" ]]; then
  echo "[INFO] Output bind mount: ${OUTPUT_BIND_DIR} -> ${WORKDIR_IN_CONTAINER}/Kimera-VIO/output"
else
  echo "[INFO] Output docker volume: ${OUTPUT_VOLUME_NAME} -> ${WORKDIR_IN_CONTAINER}/Kimera-VIO/output"
fi
if [[ "${USE_GUI}" == "1" ]]; then
  echo "[INFO] If GUI permission error occurs, run once on host: xhost +local:root"
fi

docker "${docker_args[@]}"
