#!/usr/bin/env bash
# Prepare the official YOLOv9MIT model as a host-local TensorRT engine.

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: prepare_yolo_engine.sh [--verify-only] [--print-path]

Download the pinned upstream YOLOv9MIT v1.0.0 ONNX model into the untracked
Ranger work cache, verify its size and SHA-256, and build an FP16 TensorRT
engine for the selected local GPU. The engine is never committed because a
TensorRT plan is tied to the TensorRT runtime and GPU that built it.

  --verify-only  download/build nothing; validate cache manifest and engine
  --print-path   print the selected stable engine path after success

Configuration comes from scripts/virtual_carla/env.sh:
  RANGER_WORK_ROOT
  CAMROD_CARLA_YOLO_MODEL_PATH
  CAMROD_CARLA_YOLO_DEVICE
  CAMROD_CARLA_YOLO_WORKSPACE_MIB
EOF
}

verify_only=false
print_path=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --verify-only) verify_only=true; shift ;;
    --print-path) print_path=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *)
      printf '[virtual_carla] ERROR: unknown argument: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${script_dir}/env.sh"

readonly model_release="v1.0.0"
readonly model_filename="v9-s.vec2box.sim.onnx"
readonly engine_filename="v9-s.vec2box.sim.fp16.engine"
readonly model_url="https://github.com/fateshelled/YOLOv9MIT-ROS/releases/download/v1.0.0/v9-s.vec2box.sim.onnx"
readonly model_size_bytes="29226807"
readonly model_sha256="22bfd3d91b8b1fc69586803f676b72e52766116eca644ef56c94e2e344b569dc"

virtual_carla_require_var RANGER_WORK_ROOT
virtual_carla_require_var CAMROD_CARLA_YOLO_MODEL_PATH

if [[ ! "${CAMROD_CARLA_YOLO_DEVICE}" =~ ^[0-9]+$ ]]; then
  virtual_carla_die \
    "CAMROD_CARLA_YOLO_DEVICE must be a non-negative integer (got ${CAMROD_CARLA_YOLO_DEVICE})"
  exit 1
fi
if [[ ! "${CAMROD_CARLA_YOLO_WORKSPACE_MIB}" =~ ^[1-9][0-9]*$ ]]; then
  virtual_carla_die \
    "CAMROD_CARLA_YOLO_WORKSPACE_MIB must be a positive integer (got ${CAMROD_CARLA_YOLO_WORKSPACE_MIB})"
  exit 1
fi

cache_dir="${RANGER_WORK_ROOT}/camrod/model_cache/yolov9mit/${model_release}"
managed_engine_path="${cache_dir}/${engine_filename}"
model_path="${cache_dir}/${model_filename}"
manifest_path="${managed_engine_path}.manifest.json"
builder="${CAMROD_YOLO_ENGINE_BUILDER:-${CAMROD_WS_ROOT}/install/yolov9mit/lib/yolov9mit/yolov9mit_build_engine}"

if [[ "$(readlink -m "${CAMROD_CARLA_YOLO_MODEL_PATH}")" != \
      "$(readlink -m "${managed_engine_path}")" ]]; then
  virtual_carla_die \
    "prepare_yolo_engine.sh manages ${managed_engine_path}, but CAMROD_CARLA_YOLO_MODEL_PATH selects ${CAMROD_CARLA_YOLO_MODEL_PATH}; unset the override to prepare the official cache, or use build.sh --skip-yolo-engine"
  exit 1
fi

virtual_carla_require_executable "${builder}" "YOLOv9MIT TensorRT engine builder"
command -v sha256sum >/dev/null 2>&1 || {
  virtual_carla_die "sha256sum is required to verify model artifacts"
  exit 1
}
command -v python3 >/dev/null 2>&1 || {
  virtual_carla_die "python3 is required to verify the deterministic model manifest"
  exit 1
}

runtime_key="$(${builder} --print-runtime-key --device "${CAMROD_CARLA_YOLO_DEVICE}")"
if [[ ! "${runtime_key}" =~ ^trt[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+-sm[0-9]+-[a-z0-9-]+$ ]]; then
  virtual_carla_die \
    "engine builder returned an invalid runtime key: ${runtime_key}"
  exit 1
fi

verify_model() {
  local actual_size actual_sha
  virtual_carla_require_file "${model_path}" "pinned YOLOv9MIT ONNX" || return 1
  actual_size="$(stat -c %s "${model_path}")"
  actual_sha="$(sha256sum "${model_path}" | awk '{print $1}')"
  if [[ "${actual_size}" != "${model_size_bytes}" || \
        "${actual_sha}" != "${model_sha256}" ]]; then
    virtual_carla_die \
      "pinned ONNX verification failed: ${model_path} (size=${actual_size}, sha256=${actual_sha}); expected size=${model_size_bytes}, sha256=${model_sha256}; the file was preserved"
    return 1
  fi
}

verify_engine_cache() {
  local engine_size engine_sha
  virtual_carla_require_file \
    "${managed_engine_path}" "CARLA YOLO TensorRT engine" || return 1
  virtual_carla_require_file \
    "${manifest_path}" "CARLA YOLO TensorRT manifest" || return 1
  [[ -s "${managed_engine_path}" ]] || {
    virtual_carla_die "CARLA YOLO TensorRT engine is empty: ${managed_engine_path}"
    return 1
  }
  engine_size="$(stat -c %s "${managed_engine_path}")"
  engine_sha="$(sha256sum "${managed_engine_path}" | awk '{print $1}')"
  python3 - \
    "${manifest_path}" "${model_url}" "${model_size_bytes}" \
    "${model_sha256}" "${engine_size}" "${engine_sha}" \
    "${runtime_key}" "${CAMROD_CARLA_YOLO_WORKSPACE_MIB}" <<'PY'
import json
from pathlib import Path
import sys

manifest_path = Path(sys.argv[1])
expected = {
    "schema_version": 1,
    "source_url": sys.argv[2],
    "onnx_size_bytes": int(sys.argv[3]),
    "onnx_sha256": sys.argv[4],
    "engine_size_bytes": int(sys.argv[5]),
    "engine_sha256": sys.argv[6],
    "runtime_key": sys.argv[7],
    "precision": "fp16",
    "workspace_mib": int(sys.argv[8]),
}
try:
    actual = json.loads(manifest_path.read_text(encoding="utf-8"))
except (OSError, json.JSONDecodeError) as error:
    raise SystemExit(f"invalid YOLO TensorRT manifest {manifest_path}: {error}")
if actual != expected:
    differing = sorted(
        key for key in set(actual) | set(expected)
        if actual.get(key) != expected.get(key)
    )
    raise SystemExit(
        "YOLO TensorRT manifest does not match this ONNX/runtime/engine: "
        + ", ".join(differing)
    )
PY
  "${builder}" --validate-engine "${managed_engine_path}" \
    --device "${CAMROD_CARLA_YOLO_DEVICE}"
}

if [[ "${verify_only}" == "true" ]]; then
  verify_model
  verify_engine_cache
  virtual_carla_log \
    "CARLA YOLO engine VERIFIED: ${managed_engine_path} (${runtime_key})"
  [[ "${print_path}" == "true" ]] && printf '%s\n' "${managed_engine_path}"
  exit 0
fi

mkdir -p "${cache_dir}"
if [[ -e "${model_path}" ]]; then
  verify_model
else
  command -v curl >/dev/null 2>&1 || {
    virtual_carla_die "curl is required to download the pinned YOLOv9MIT ONNX"
    exit 1
  }
  download_path="$(mktemp "${cache_dir}/.${model_filename}.download.XXXXXX")"
  cleanup_download() { rm -f -- "${download_path}"; }
  trap cleanup_download EXIT
  virtual_carla_log "downloading pinned YOLOv9MIT ONNX: ${model_url}"
  curl --fail --location --retry 3 --retry-delay 2 \
    --output "${download_path}" "${model_url}"
  download_size="$(stat -c %s "${download_path}")"
  download_sha="$(sha256sum "${download_path}" | awk '{print $1}')"
  if [[ "${download_size}" != "${model_size_bytes}" || \
        "${download_sha}" != "${model_sha256}" ]]; then
    virtual_carla_die \
      "downloaded ONNX failed verification (size=${download_size}, sha256=${download_sha}); expected size=${model_size_bytes}, sha256=${model_sha256}"
    exit 1
  fi
  mv -- "${download_path}" "${model_path}"
  trap - EXIT
  verify_model
fi

if [[ -e "${managed_engine_path}" || -e "${manifest_path}" ]]; then
  if [[ -f "${managed_engine_path}" && -f "${manifest_path}" ]]; then
    verify_engine_cache
    virtual_carla_log \
      "reusing verified CARLA YOLO engine: ${managed_engine_path} (${runtime_key})"
    [[ "${print_path}" == "true" ]] && printf '%s\n' "${managed_engine_path}"
    exit 0
  fi
  virtual_carla_die \
    "incomplete CARLA YOLO cache: engine and manifest must either both exist or both be absent (${managed_engine_path}, ${manifest_path}); preserved for diagnosis"
  exit 1
fi

build_dir="$(mktemp -d "${cache_dir}/.engine-build.XXXXXX")"
temporary_engine="${build_dir}/${engine_filename}"
temporary_manifest="${build_dir}/${engine_filename}.manifest.json"
cleanup_build() { rm -rf -- "${build_dir}"; }
trap cleanup_build EXIT

virtual_carla_log \
  "building CARLA YOLO TensorRT engine: runtime=${runtime_key} precision=fp16 workspace_mib=${CAMROD_CARLA_YOLO_WORKSPACE_MIB}"
"${builder}" \
  --onnx "${model_path}" \
  --engine "${temporary_engine}" \
  --workspace-mib "${CAMROD_CARLA_YOLO_WORKSPACE_MIB}" \
  --device "${CAMROD_CARLA_YOLO_DEVICE}" \
  --fp16
"${builder}" --validate-engine "${temporary_engine}" \
  --device "${CAMROD_CARLA_YOLO_DEVICE}"

engine_size="$(stat -c %s "${temporary_engine}")"
engine_sha="$(sha256sum "${temporary_engine}" | awk '{print $1}')"
python3 - \
  "${temporary_manifest}" "${model_url}" "${model_size_bytes}" \
  "${model_sha256}" "${engine_size}" "${engine_sha}" \
  "${runtime_key}" "${CAMROD_CARLA_YOLO_WORKSPACE_MIB}" <<'PY'
import json
from pathlib import Path
import sys

manifest = {
    "schema_version": 1,
    "source_url": sys.argv[2],
    "onnx_size_bytes": int(sys.argv[3]),
    "onnx_sha256": sys.argv[4],
    "engine_size_bytes": int(sys.argv[5]),
    "engine_sha256": sys.argv[6],
    "runtime_key": sys.argv[7],
    "precision": "fp16",
    "workspace_mib": int(sys.argv[8]),
}
Path(sys.argv[1]).write_text(
    json.dumps(manifest, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
PY

mv -- "${temporary_engine}" "${managed_engine_path}"
mv -- "${temporary_manifest}" "${manifest_path}"
trap - EXIT
rmdir "${build_dir}"

verify_engine_cache
virtual_carla_log \
  "CARLA YOLO engine ready: ${managed_engine_path} sha256=${engine_sha} runtime=${runtime_key}"
[[ "${print_path}" == "true" ]] && printf '%s\n' "${managed_engine_path}"
