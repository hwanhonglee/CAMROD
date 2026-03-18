#!/usr/bin/env bash
set -euo pipefail

# Optional KEY=VALUE style arguments are supported:
#   ./buildx_camrod.sh PUSH_IMAGE=0 IMAGE_TAG=test
for kv in "$@"; do
  if [[ "${kv}" == *=* ]]; then
    export "${kv}"
  else
    echo "[buildx_camrod] ERROR: unsupported argument '${kv}'. Use KEY=VALUE form."
    exit 1
  fi
done

WORKSPACE_ROOT="${WORKSPACE_ROOT:-/home/hong/camrod_ws}"
DOCKERFILE_REL="${DOCKERFILE_REL:-src/docker/Dockerfile.camrod}"
IMAGE_REPO="${IMAGE_REPO:-lehong/camrod}"
IMAGE_TAG="${IMAGE_TAG:-latest}"
DATE_TAG="$(date +%Y%m%d)"
PLATFORMS="${PLATFORMS:-linux/amd64,linux/arm64}"
BUILDER_NAME="${BUILDER_NAME:-camrod-multiarch}"
PUSH_IMAGE="${PUSH_IMAGE:-1}"
RESET_BUILDER="${RESET_BUILDER:-0}"
INSTALL_BINFMT="${INSTALL_BINFMT:-1}"

if [[ ! -f "${WORKSPACE_ROOT}/${DOCKERFILE_REL}" ]]; then
  echo "[buildx_camrod] ERROR: Dockerfile not found: ${WORKSPACE_ROOT}/${DOCKERFILE_REL}"
  exit 1
fi

# Install binfmt handlers when multi-arch includes non-host architecture.
# This prevents ".buildkit_qemu_emulator: Invalid ELF image for this architecture".
if [[ "${INSTALL_BINFMT}" == "1" && "${PLATFORMS}" == *"arm64"* ]]; then
  echo "[buildx_camrod] Installing binfmt emulators (requires docker --privileged)..."
  docker run --privileged --rm tonistiigi/binfmt --install all >/dev/null
fi

if [[ "${RESET_BUILDER}" == "1" ]]; then
  docker buildx rm "${BUILDER_NAME}" >/dev/null 2>&1 || true
fi

if ! docker buildx inspect "${BUILDER_NAME}" >/dev/null 2>&1; then
  docker buildx create --name "${BUILDER_NAME}" --driver docker-container --use
else
  docker buildx use "${BUILDER_NAME}"
fi

INSPECT_OUTPUT="$(docker buildx inspect "${BUILDER_NAME}" --bootstrap)"
if [[ "${PLATFORMS}" == *"arm64"* ]] && ! grep -q "linux/arm64" <<<"${INSPECT_OUTPUT}"; then
  echo "[buildx_camrod] arm64 platform still unavailable on builder '${BUILDER_NAME}'."
  echo "[buildx_camrod] Recreating builder after binfmt install..."
  docker buildx rm "${BUILDER_NAME}" >/dev/null 2>&1 || true
  docker buildx create --name "${BUILDER_NAME}" --driver docker-container --use
  INSPECT_OUTPUT="$(docker buildx inspect "${BUILDER_NAME}" --bootstrap)"
  if ! grep -q "linux/arm64" <<<"${INSPECT_OUTPUT}"; then
    echo "[buildx_camrod] ERROR: builder does not support linux/arm64."
    echo "[buildx_camrod] Check Docker Desktop/Engine virtualization and binfmt support."
    exit 1
  fi
fi

BUILD_CMD=(
  docker buildx build
  --platform "${PLATFORMS}"
  -f "${WORKSPACE_ROOT}/${DOCKERFILE_REL}"
  -t "${IMAGE_REPO}:${IMAGE_TAG}"
  -t "${IMAGE_REPO}:${DATE_TAG}"
  "${WORKSPACE_ROOT}"
)

if [[ "${PUSH_IMAGE}" == "1" ]]; then
  BUILD_CMD+=(--push)
else
  # docker buildx --load supports a single platform only.
  if [[ "${PLATFORMS}" == *","* ]]; then
    PLATFORMS="${PLATFORMS%%,*}"
    BUILD_CMD=(
      docker buildx build
      --platform "${PLATFORMS}"
      -f "${WORKSPACE_ROOT}/${DOCKERFILE_REL}"
      -t "${IMAGE_REPO}:${IMAGE_TAG}"
      -t "${IMAGE_REPO}:${DATE_TAG}"
      "${WORKSPACE_ROOT}"
    )
  fi
  BUILD_CMD+=(--load)
fi

echo "[buildx_camrod] Building image: ${IMAGE_REPO}:${IMAGE_TAG}"
echo "[buildx_camrod] Extra date tag: ${IMAGE_REPO}:${DATE_TAG}"
echo "[buildx_camrod] Platforms: ${PLATFORMS}"
echo "[buildx_camrod] Push: ${PUSH_IMAGE}"

"${BUILD_CMD[@]}"

echo "[buildx_camrod] Done."
