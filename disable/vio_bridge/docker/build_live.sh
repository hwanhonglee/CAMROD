#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

IMAGE_NAME="${IMAGE_NAME:-kimera-vio:live}"
BASE_IMAGE="${BASE_IMAGE:-nvidia/cuda:12.8.0-devel-ubuntu22.04}"
INSTALL_ZED_SDK="${INSTALL_ZED_SDK:-ON}"
INSTALL_ORBBEC_SDK="${INSTALL_ORBBEC_SDK:-ON}"
INSTALL_ROS2="${INSTALL_ROS2:-ON}"
KIMERA_VIO_USE_ZED="${KIMERA_VIO_USE_ZED:-ON}"
KIMERA_VIO_USE_ORBBEC="${KIMERA_VIO_USE_ORBBEC:-ON}"
KIMERA_VIO_USE_ROS2="${KIMERA_VIO_USE_ROS2:-ON}"
ZED_SDK_RUN_URL="${ZED_SDK_RUN_URL:-}"
ORBBEC_SDK_DEB_URL="${ORBBEC_SDK_DEB_URL:-}"
AMD64_ZED_INSTALLER="ZED_SDK_Ubuntu22_cuda12.8_tensorrt10.9_v5.1.2.zstd.run"
HIDDEN_AMD64_ZED=""

cleanup_hidden_installer() {
  if [[ -n "${HIDDEN_AMD64_ZED}" && -f "${HIDDEN_AMD64_ZED}" ]]; then
    mv -f "${HIDDEN_AMD64_ZED}" "${ROOT_DIR}/sdk_installers/${AMD64_ZED_INSTALLER}"
    echo "[INFO] Restored hidden amd64 ZED installer."
  fi
}
trap cleanup_hidden_installer EXIT

TARGET_ARCH="${TARGET_ARCH:-}"
if [[ -z "${TARGET_ARCH}" ]]; then
  case "$(uname -m)" in
    x86_64) TARGET_ARCH="amd64" ;;
    aarch64|arm64) TARGET_ARCH="arm64" ;;
    *)
      echo "[ERROR] Unsupported host architecture: $(uname -m)"
      echo "        Set TARGET_ARCH=amd64 or TARGET_ARCH=arm64 explicitly."
      exit 1
      ;;
  esac
fi

DOCKERFILE="${DOCKERFILE:-Dockerfile.vio_live.${TARGET_ARCH}}"
if [[ ! -f "${DOCKERFILE}" ]]; then
  echo "[ERROR] Dockerfile not found: ${DOCKERFILE}"
  exit 1
fi

echo "[INFO] Dockerfile: ${DOCKERFILE}"
echo "[INFO] Dockerfile sha256: $(sha256sum "${DOCKERFILE}" | awk '{print $1}')"
echo "[INFO] Dockerfile key lines:"
grep -nE "set -euo pipefail|Kimera-VIO CMake configure failed|ros-\\$\\{ROS_DISTRO\\}-rclcpp" "${DOCKERFILE}" || true

mkdir -p sdk_installers

if [[ "${TARGET_ARCH}" == "arm64" ]]; then
  EXPECTED_ZED_RUN="ZED_SDK_Tegra_L4T36.4_v5.2.1.zstd.run"
  EXPECTED_ORBBEC_DEB="OrbbecSDK_v2.7.6_arm64.deb"
  # Reduce build context for arm64 buildx/qemu stability:
  # temporarily hide the huge amd64 ZED installer from context transfer.
  if [[ -f "sdk_installers/${AMD64_ZED_INSTALLER}" ]]; then
    HIDDEN_AMD64_ZED="sdk_installers/.${AMD64_ZED_INSTALLER}.arm64_hide.$$"
    mv -f "sdk_installers/${AMD64_ZED_INSTALLER}" "${HIDDEN_AMD64_ZED}"
    echo "[INFO] Temporarily hid amd64 ZED installer for arm64 build context shrink."
  fi
else
  EXPECTED_ZED_RUN="ZED_SDK_Ubuntu22_cuda12.8_tensorrt10.9_v5.1.2.zstd.run"
  EXPECTED_ORBBEC_DEB="OrbbecSDK_v2.7.6_amd64.deb"
fi

if [[ -f "sdk_installers/${EXPECTED_ZED_RUN}" ]]; then
  echo "[INFO] Using local ZED installer from ./sdk_installers"
else
  echo "[INFO] Local ZED installer not found (${EXPECTED_ZED_RUN}). Dockerfile will try online fallback."
fi

if [[ -f "sdk_installers/${EXPECTED_ORBBEC_DEB}" ]]; then
  echo "[INFO] Using local Orbbec installer from ./sdk_installers"
else
  echo "[INFO] Local Orbbec installer not found (${EXPECTED_ORBBEC_DEB}). Dockerfile will try online fallback."
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

docker build \
  -f "${DOCKERFILE}" \
  -t "${IMAGE_NAME}" \
  "${build_args[@]}" \
  .

echo "[OK] Built image: ${IMAGE_NAME}"
echo "[INFO] Dockerfile: ${DOCKERFILE}"
