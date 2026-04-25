#!/usr/bin/env bash
set -euo pipefail

# Builds external Kimera dependencies used by CAMROD localization fallback.
# - Builds DBoW2 from bundled source into a local install prefix.
# - Configures (and optionally builds) Kimera-VIO against that local DBoW2.
#
# Usage:
#   ./build_kimera_stack.sh
#   ./build_kimera_stack.sh --configure-only
#   ./build_kimera_stack.sh --clean
#   ./build_kimera_stack.sh --jobs 8

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
ROOT_DIR="$(cd "$(dirname "${SCRIPT_PATH}")" && pwd)"
DBOW2_SRC="${ROOT_DIR}/third_party/DBoW2"
DBOW2_BUILD="${ROOT_DIR}/build/DBoW2"
DBOW2_INSTALL="${ROOT_DIR}/install/dbow2"

KIMERA_SRC="${ROOT_DIR}/Kimera-VIO"
KIMERA_BUILD="${ROOT_DIR}/build/Kimera-VIO"
KIMERA_INSTALL="${ROOT_DIR}/install/kimera_vio"

CONFIGURE_ONLY=0
DO_CLEAN=0
WITH_TESTS=0
JOBS="${JOBS:-$(nproc)}"

usage() {
  cat <<'EOF'
Usage: ./build_kimera_stack.sh [options]

Options:
  --configure-only    Configure Kimera-VIO only (skip compile step).
  --clean             Remove local build/install outputs before building.
  --jobs N            Parallel build jobs (default: nproc).
  --with-tests        Build full tree including testKimeraVIO target.
  -h, --help          Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --configure-only)
      CONFIGURE_ONLY=1
      shift
      ;;
    --clean)
      DO_CLEAN=1
      shift
      ;;
    --jobs)
      JOBS="$2"
      shift 2
      ;;
    --with-tests)
      WITH_TESTS=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[build_kimera_stack] Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ ! -d "${DBOW2_SRC}" ]]; then
  echo "[build_kimera_stack] Missing DBoW2 source: ${DBOW2_SRC}" >&2
  exit 1
fi
if [[ ! -d "${KIMERA_SRC}" ]]; then
  echo "[build_kimera_stack] Missing Kimera-VIO source: ${KIMERA_SRC}" >&2
  exit 1
fi

if [[ "${DO_CLEAN}" -eq 1 ]]; then
  echo "[build_kimera_stack] Cleaning local build/install outputs..."
  rm -rf "${ROOT_DIR}/build" "${ROOT_DIR}/install"
fi

mkdir -p "${DBOW2_BUILD}" "${DBOW2_INSTALL}" "${KIMERA_BUILD}" "${KIMERA_INSTALL}"

echo "[build_kimera_stack] Step 1/3: build DBoW2"
cmake -S "${DBOW2_SRC}" -B "${DBOW2_BUILD}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${DBOW2_INSTALL}" \
  -DBUILD_Demo=OFF
cmake --build "${DBOW2_BUILD}" -j"${JOBS}"
cmake --install "${DBOW2_BUILD}"

DBOW2_DIR_CMAKE="${DBOW2_INSTALL}/lib/cmake/DBoW2"
if [[ ! -f "${DBOW2_DIR_CMAKE}/DBoW2Config.cmake" ]]; then
  echo "[build_kimera_stack] DBoW2Config.cmake not found at ${DBOW2_DIR_CMAKE}" >&2
  exit 1
fi

echo "[build_kimera_stack] Step 2/3: configure Kimera-VIO"
cmake -S "${KIMERA_SRC}" -B "${KIMERA_BUILD}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${KIMERA_INSTALL}" \
  -DDBoW2_DIR="${DBOW2_DIR_CMAKE}" \
  -DCMAKE_PREFIX_PATH="${DBOW2_INSTALL};${CMAKE_PREFIX_PATH:-}" \
  -DKIMERA_VIO_USE_ROS2=ON

if [[ "${CONFIGURE_ONLY}" -eq 1 ]]; then
  echo "[build_kimera_stack] Step 3/3: skipped (--configure-only)"
  exit 0
fi

echo "[build_kimera_stack] Step 3/3: build Kimera-VIO"
if [[ "${WITH_TESTS}" -eq 1 ]]; then
  cmake --build "${KIMERA_BUILD}" -j"${JOBS}"
else
  # Build runtime/library targets only. testKimeraVIO is excluded because
  # OpenCV major-version differences can break upstream test sources.
  cmake --build "${KIMERA_BUILD}" -j"${JOBS}" --target \
    kimera_vio \
    stereoVIOEuroc \
    zedDumpCalibration \
    zedLiveVIO \
    orbbecDumpCalibration \
    orbbecLiveVIO
fi
echo "[build_kimera_stack] Done."
