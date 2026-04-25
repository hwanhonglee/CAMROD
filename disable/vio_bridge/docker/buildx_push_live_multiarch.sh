#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

IMAGE_REPO="${IMAGE_REPO:-lehong/kimera-vio}"
TAG="${TAG:-live}"
BASE_IMAGE="${BASE_IMAGE:-nvidia/cuda:12.8.0-devel-ubuntu22.04}"

INSTALL_ZED_SDK="${INSTALL_ZED_SDK:-ON}"
INSTALL_ORBBEC_SDK="${INSTALL_ORBBEC_SDK:-ON}"
INSTALL_ROS2="${INSTALL_ROS2:-ON}"
KIMERA_VIO_USE_ZED="${KIMERA_VIO_USE_ZED:-ON}"
KIMERA_VIO_USE_ORBBEC="${KIMERA_VIO_USE_ORBBEC:-ON}"
KIMERA_VIO_USE_ROS2="${KIMERA_VIO_USE_ROS2:-ON}"

ZED_SDK_RUN_URL="${ZED_SDK_RUN_URL:-}"
ORBBEC_SDK_DEB_URL="${ORBBEC_SDK_DEB_URL:-}"

BUILDER_NAME="${BUILDER_NAME:-multiarch_builder}"
ROOT_SDK_DIR="${ROOT_DIR}/sdk_installers"
AMD64_ZED_INSTALLER="ZED_SDK_Ubuntu22_cuda12.8_tensorrt10.9_v5.1.2.zstd.run"
HIDDEN_AMD64_ZED=""

# Recover stale hidden amd64 ZED installers left by interrupted runs.
shopt -s nullglob
for stale_hidden in "${ROOT_SDK_DIR}/.${AMD64_ZED_INSTALLER}.arm64_hide."*; do
  if [[ -f "${ROOT_SDK_DIR}/${AMD64_ZED_INSTALLER}" ]]; then
    rm -f "${stale_hidden}"
    echo "[INFO] Removed stale hidden installer: ${stale_hidden}"
  else
    mv -f "${stale_hidden}" "${ROOT_SDK_DIR}/${AMD64_ZED_INSTALLER}"
    echo "[INFO] Restored stale hidden amd64 ZED installer before build."
  fi
done
shopt -u nullglob

cleanup_hidden_installer() {
  if [[ -n "${HIDDEN_AMD64_ZED}" && -f "${HIDDEN_AMD64_ZED}" ]]; then
    mv -f "${HIDDEN_AMD64_ZED}" "${ROOT_SDK_DIR}/${AMD64_ZED_INSTALLER}"
    echo "[INFO] Restored hidden amd64 ZED installer."
  fi
}
trap cleanup_hidden_installer EXIT

if ! docker buildx inspect "${BUILDER_NAME}" >/dev/null 2>&1; then
  docker buildx create --name "${BUILDER_NAME}" --use
else
  docker buildx use "${BUILDER_NAME}"
fi

docker buildx inspect --bootstrap >/dev/null

echo "[INFO] Dockerfile amd64: Dockerfile.vio_live.amd64"
echo "[INFO] Dockerfile amd64 sha256: $(sha256sum Dockerfile.vio_live.amd64 | awk '{print $1}')"
grep -nE "set -euo pipefail|Kimera-VIO CMake configure failed|ros-\\$\\{ROS_DISTRO\\}-rclcpp" Dockerfile.vio_live.amd64 || true
echo "[INFO] Dockerfile arm64: Dockerfile.vio_live.arm64"
echo "[INFO] Dockerfile arm64 sha256: $(sha256sum Dockerfile.vio_live.arm64 | awk '{print $1}')"
grep -nE "set -euo pipefail|Kimera-VIO CMake configure failed|ros-\\$\\{ROS_DISTRO\\}-rclcpp" Dockerfile.vio_live.arm64 || true

common_build_args=(
  --build-arg "BASE_IMAGE=${BASE_IMAGE}"
  --build-arg "INSTALL_ZED_SDK=${INSTALL_ZED_SDK}"
  --build-arg "INSTALL_ORBBEC_SDK=${INSTALL_ORBBEC_SDK}"
  --build-arg "INSTALL_ROS2=${INSTALL_ROS2}"
  --build-arg "KIMERA_VIO_USE_ZED=${KIMERA_VIO_USE_ZED}"
  --build-arg "KIMERA_VIO_USE_ORBBEC=${KIMERA_VIO_USE_ORBBEC}"
  --build-arg "KIMERA_VIO_USE_ROS2=${KIMERA_VIO_USE_ROS2}"
)

if [[ -n "${ZED_SDK_RUN_URL}" ]]; then
  common_build_args+=(--build-arg "ZED_SDK_RUN_URL=${ZED_SDK_RUN_URL}")
fi
if [[ -n "${ORBBEC_SDK_DEB_URL}" ]]; then
  common_build_args+=(--build-arg "ORBBEC_SDK_DEB_URL=${ORBBEC_SDK_DEB_URL}")
fi

AMD64_TAG="${IMAGE_REPO}:${TAG}-amd64"
ARM64_TAG="${IMAGE_REPO}:${TAG}-arm64"
FINAL_TAG="${IMAGE_REPO}:${TAG}"

echo "[INFO] Building amd64 image: ${AMD64_TAG}"
docker buildx build \
  --platform linux/amd64 \
  -f Dockerfile.vio_live.amd64 \
  -t "${AMD64_TAG}" \
  "${common_build_args[@]}" \
  --push \
  .

echo "[INFO] Building arm64 image: ${ARM64_TAG}"
# Reduce arm64 context size by hiding the huge amd64 installer before arm64 build.
if [[ -f "${ROOT_SDK_DIR}/${AMD64_ZED_INSTALLER}" ]]; then
  HIDDEN_AMD64_ZED="${ROOT_SDK_DIR}/.${AMD64_ZED_INSTALLER}.arm64_hide.$$"
  mv -f "${ROOT_SDK_DIR}/${AMD64_ZED_INSTALLER}" "${HIDDEN_AMD64_ZED}"
  echo "[INFO] Temporarily hid amd64 ZED installer for arm64 build context shrink."
fi

docker buildx build \
  --platform linux/arm64 \
  -f Dockerfile.vio_live.arm64 \
  -t "${ARM64_TAG}" \
  "${common_build_args[@]}" \
  --push \
  .

echo "[INFO] Creating multi-arch manifest: ${FINAL_TAG}"
docker buildx imagetools create -t "${FINAL_TAG}" "${AMD64_TAG}" "${ARM64_TAG}"

echo "[INFO] Inspect manifest: ${FINAL_TAG}"
docker buildx imagetools inspect "${FINAL_TAG}"

echo "[OK] Pushed multi-arch tag: ${FINAL_TAG}"
