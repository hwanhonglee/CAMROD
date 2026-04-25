#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

BUILDER_NAME="${BUILDER_NAME:-camrod-multiarch}"
IMAGE_NAME="${IMAGE_NAME:-kimera-vio:live-arm64}"
DOCKERFILE="${DOCKERFILE:-Dockerfile.vio_live.arm64}"

BASE_IMAGE="${BASE_IMAGE:-nvidia/cuda:12.8.0-devel-ubuntu22.04}"
INSTALL_ZED_SDK="${INSTALL_ZED_SDK:-ON}"
INSTALL_ORBBEC_SDK="${INSTALL_ORBBEC_SDK:-ON}"
INSTALL_ROS2="${INSTALL_ROS2:-ON}"
KIMERA_VIO_USE_ZED="${KIMERA_VIO_USE_ZED:-ON}"
KIMERA_VIO_USE_ORBBEC="${KIMERA_VIO_USE_ORBBEC:-ON}"
KIMERA_VIO_USE_ROS2="${KIMERA_VIO_USE_ROS2:-ON}"

ZED_SDK_RUN_URL="${ZED_SDK_RUN_URL:-}"
ORBBEC_SDK_DEB_URL="${ORBBEC_SDK_DEB_URL:-}"

AMD64_ZED_INSTALLER="sdk_installers/ZED_SDK_Ubuntu22_cuda12.8_tensorrt10.9_v5.1.2.zstd.run"
HIDDEN_AMD64_ZED=""

cleanup_hidden_installer() {
  if [[ -n "${HIDDEN_AMD64_ZED}" && -f "${HIDDEN_AMD64_ZED}" ]]; then
    mv -f "${HIDDEN_AMD64_ZED}" "${ROOT_DIR}/${AMD64_ZED_INSTALLER}"
    echo "[INFO] Restored hidden amd64 ZED installer."
  fi
}
trap cleanup_hidden_installer EXIT

if [[ ! -f "${DOCKERFILE}" ]]; then
  echo "[ERROR] Dockerfile not found: ${DOCKERFILE}"
  exit 1
fi

echo "[INFO] Dockerfile: ${DOCKERFILE}"
echo "[INFO] Dockerfile sha256: $(sha256sum "${DOCKERFILE}" | awk '{print $1}')"
echo "[INFO] Dockerfile key lines:"
grep -nE "set -euo pipefail|Kimera-VIO CMake configure failed|ros-\\$\\{ROS_DISTRO\\}-rclcpp" "${DOCKERFILE}" || true

if ! docker buildx inspect "${BUILDER_NAME}" >/dev/null 2>&1; then
  docker buildx create --name "${BUILDER_NAME}" --driver docker-container --use
else
  docker buildx use "${BUILDER_NAME}"
fi
docker buildx inspect --bootstrap >/dev/null

# Reduce context size and buildkit transport instability on arm64 buildx:
# temporarily hide huge amd64 ZED installer from build context.
if [[ -f "${ROOT_DIR}/${AMD64_ZED_INSTALLER}" ]]; then
  HIDDEN_AMD64_ZED="${ROOT_DIR}/sdk_installers/.ZED_SDK_Ubuntu22_cuda12.8_tensorrt10.9_v5.1.2.zstd.run.arm64_hide.$$"
  mv -f "${ROOT_DIR}/${AMD64_ZED_INSTALLER}" "${HIDDEN_AMD64_ZED}"
  echo "[INFO] Temporarily hid amd64 ZED installer for arm64 build context shrink."
fi

build_args=(
  --build-arg "BASE_IMAGE=${BASE_IMAGE}"
  --build-arg "INSTALL_ZED_SDK=${INSTALL_ZED_SDK}"
  --build-arg "INSTALL_ORBBEC_SDK=${INSTALL_ORBBEC_SDK}"
  --build-arg "INSTALL_ROS2=${INSTALL_ROS2}"
  --build-arg "KIMERA_VIO_USE_ZED=${KIMERA_VIO_USE_ZED}"
  --build-arg "KIMERA_VIO_USE_ORBBEC=${KIMERA_VIO_USE_ORBBEC}"
  --build-arg "KIMERA_VIO_USE_ROS2=${KIMERA_VIO_USE_ROS2}"
)

if [[ -n "${ZED_SDK_RUN_URL}" ]]; then
  build_args+=(--build-arg "ZED_SDK_RUN_URL=${ZED_SDK_RUN_URL}")
fi
if [[ -n "${ORBBEC_SDK_DEB_URL}" ]]; then
  build_args+=(--build-arg "ORBBEC_SDK_DEB_URL=${ORBBEC_SDK_DEB_URL}")
fi

docker buildx build \
  --builder "${BUILDER_NAME}" \
  --platform linux/arm64 \
  -f "${DOCKERFILE}" \
  -t "${IMAGE_NAME}" \
  --progress=plain \
  --load \
  "${build_args[@]}" \
  .

echo "[OK] Built image: ${IMAGE_NAME}"
