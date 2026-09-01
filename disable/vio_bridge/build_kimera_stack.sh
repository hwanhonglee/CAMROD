#!/usr/bin/env bash
set -euo pipefail

# Builds the optional Kimera dependencies used by CAMROD localization fallback.
# The upstream sources are deliberately NOT git submodules.  This explicit
# helper prepares pinned, local Git checkouts before it starts the build.  The
# normal CAMROD/colcon build never invokes this script and never downloads them.
#
# Usage:
#   ./build_kimera_stack.sh
#   ./build_kimera_stack.sh --prepare-sources-only
#   ./build_kimera_stack.sh --configure-only
#   ./build_kimera_stack.sh --clean
#   ./build_kimera_stack.sh --jobs 8

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
ROOT_DIR="$(cd "$(dirname "${SCRIPT_PATH}")" && pwd)"
DBOW2_URL="https://github.com/dorian3d/DBoW2.git"
DBOW2_COMMIT="3924753db6145f12618e7de09b7e6b258db93c6e"
DBOW2_SRC="${ROOT_DIR}/third_party/DBoW2"
DBOW2_BUILD="${ROOT_DIR}/build/DBoW2"
DBOW2_INSTALL="${ROOT_DIR}/install/dbow2"

KIMERA_URL="https://github.com/MIT-SPARK/Kimera-VIO.git"
KIMERA_COMMIT="ce8c59b7b273ab5ac29db7e5572e1623760e19c7"
KIMERA_SRC="${ROOT_DIR}/Kimera-VIO"
KIMERA_BUILD="${ROOT_DIR}/build/Kimera-VIO"
KIMERA_INSTALL="${ROOT_DIR}/install/kimera_vio"

CONFIGURE_ONLY=0
PREPARE_SOURCES_ONLY=0
DO_CLEAN=0
WITH_TESTS=0
JOBS="${JOBS:-$(nproc)}"

usage() {
  cat <<'EOF'
Usage: ./build_kimera_stack.sh [options]

Options:
  --prepare-sources-only
                      Fetch/verify pinned source checkouts, then stop.
  --configure-only    Configure Kimera-VIO only (skip compile step).
  --clean             Remove local build/install outputs before building.
  --jobs N            Parallel build jobs (default: nproc).
  --with-tests        Build full tree including testKimeraVIO target.
  -h, --help          Show this help.
EOF
}

die() {
  echo "[build_kimera_stack] ERROR: $*" >&2
  exit 1
}

validate_pinned_checkout() {
  local label="$1"
  local expected_url="$2"
  local expected_commit="$3"
  local source_dir="$4"
  local source_real git_top remote_url head_commit dirty

  # A normal clone has its own .git directory.  In particular, do not accept a
  # submodule-style .git file or a plain directory that happens to sit below the
  # CAMROD repository and therefore resolves to CAMROD's own .git directory.
  [[ ! -L "${source_dir}" ]] || \
    die "${label} source path must not be a symlink: ${source_dir}"
  [[ -d "${source_dir}/.git" ]] || \
    die "${label} source is not an independent ordinary Git checkout: ${source_dir}"

  source_real="$(readlink -f "${source_dir}")"
  git_top="$(git -C "${source_dir}" rev-parse --show-toplevel 2>/dev/null)" || \
    die "cannot inspect ${label} checkout: ${source_dir}"
  git_top="$(readlink -f "${git_top}")"
  [[ "${git_top}" == "${source_real}" ]] || \
    die "${label} Git root mismatch: expected ${source_real}, got ${git_top}"

  remote_url="$(git -C "${source_dir}" remote get-url origin 2>/dev/null)" || \
    die "${label} checkout has no origin remote"
  [[ "${remote_url}" == "${expected_url}" ]] || \
    die "${label} origin mismatch: expected ${expected_url}, got ${remote_url}"

  dirty="$(git -C "${source_dir}" status --porcelain=v1 --untracked-files=all)" || \
    die "cannot inspect ${label} checkout status"
  [[ -z "${dirty}" ]] || \
    die "${label} checkout is dirty; preserve or discard its changes explicitly before retrying"

  head_commit="$(git -C "${source_dir}" rev-parse HEAD 2>/dev/null)" || \
    die "cannot resolve ${label} HEAD"
  [[ "${head_commit}" == "${expected_commit}" ]] || \
    die "${label} commit mismatch: expected ${expected_commit}, got ${head_commit}"

  if git -C "${source_dir}" symbolic-ref -q HEAD >/dev/null 2>&1; then
    die "${label} checkout must be detached at pinned commit ${expected_commit}"
  fi

  echo "[build_kimera_stack] ${label} source verified: ${expected_commit}"
}

prepare_pinned_checkout() {
  local label="$1"
  local url="$2"
  local commit="$3"
  local source_dir="$4"
  local parent_dir checkout_name temp_dir fetched_commit

  if [[ ! -e "${source_dir}" && ! -L "${source_dir}" ]]; then
    command -v git >/dev/null 2>&1 || die "git is required to prepare ${label}"
    parent_dir="$(dirname "${source_dir}")"
    checkout_name="$(basename "${source_dir}")"
    mkdir -p "${parent_dir}"
    temp_dir="$(mktemp -d "${parent_dir}/.${checkout_name}.clone.XXXXXX")"

    echo "[build_kimera_stack] Fetching pinned ${label} source: ${url}@${commit}"
    if ! git -C "${temp_dir}" init --quiet ||
       ! git -C "${temp_dir}" remote add origin "${url}" ||
       ! git -C "${temp_dir}" fetch --quiet --depth 1 origin "${commit}"; then
      rm -rf -- "${temp_dir}"
      die "failed to fetch pinned ${label} source"
    fi
    fetched_commit="$(git -C "${temp_dir}" rev-parse FETCH_HEAD 2>/dev/null)" || {
      rm -rf -- "${temp_dir}"
      die "cannot resolve fetched ${label} commit"
    }
    if [[ "${fetched_commit}" != "${commit}" ]]; then
      rm -rf -- "${temp_dir}"
      die "fetched ${label} commit mismatch: expected ${commit}, got ${fetched_commit}"
    fi
    if ! git -C "${temp_dir}" -c advice.detachedHead=false \
        checkout --quiet --detach "${commit}"; then
      rm -rf -- "${temp_dir}"
      die "failed to check out pinned ${label} commit ${commit}"
    fi
    if ! mv -- "${temp_dir}" "${source_dir}"; then
      rm -rf -- "${temp_dir}"
      die "failed to install ${label} checkout at ${source_dir}"
    fi
  fi

  validate_pinned_checkout "${label}" "${url}" "${commit}" "${source_dir}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prepare-sources-only)
      PREPARE_SOURCES_ONLY=1
      shift
      ;;
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

prepare_pinned_checkout "DBoW2" "${DBOW2_URL}" "${DBOW2_COMMIT}" "${DBOW2_SRC}"
prepare_pinned_checkout "Kimera-VIO" "${KIMERA_URL}" "${KIMERA_COMMIT}" "${KIMERA_SRC}"

if [[ "${PREPARE_SOURCES_ONLY}" -eq 1 ]]; then
  echo "[build_kimera_stack] Pinned source preparation complete."
  exit 0
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
