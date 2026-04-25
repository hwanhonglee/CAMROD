#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
SENSOR=""
OUTPUT_DIR=""
COMPOSE_PARAMS_DIR=""
TEMPLATE_PARAMS_DIR=""
EXTRA_ARGS=()

usage() {
  cat <<'EOF'
Usage:
  ./save_sensor_yaml.sh --sensor <zed|orbbec> [--output_dir <path>] [--build_dir <path>] \
      [--compose_params_dir <path>] [--template_params_dir <path>] [-- <extra_args>]

Purpose:
  1) Save Kimera sensor YAML files (Left/Right/IMU) from a live sensor.
  2) Optionally compose a full Kimera params folder (Pipeline/Frontend/Backend/Lcd/Display + sensor YAML).

Examples:
  # Save sensor YAML only
  ./save_sensor_yaml.sh --sensor zed --output_dir ./Kimera-VIO/params/ZED_live_sensor
  ./save_sensor_yaml.sh --sensor orbbec --output_dir ./Kimera-VIO/params/Orbbec_live_sensor

  # Compose full params folder (recommended for direct Kimera run)
  ./save_sensor_yaml.sh --sensor zed --compose_params_dir ./Kimera-VIO/params/ZED_live
  ./save_sensor_yaml.sh --sensor orbbec --compose_params_dir ./Kimera-VIO/params/Orbbec_live

  # Compose with custom template folder
  ./save_sensor_yaml.sh --sensor zed \
      --compose_params_dir ./Kimera-VIO/params/ZED_custom \
      --template_params_dir ./Kimera-VIO/params/Euroc

  ./save_sensor_yaml.sh --sensor zed -- --rectified --resolution HD720 --fps 30
  ./save_sensor_yaml.sh --sensor orbbec -- --serial <SN> --fps 30
  ./save_sensor_yaml.sh --sensor orbbec -- --address 192.168.1.10 --port 8090 --fps 30
EOF
}

ensure_dump_files() {
  local dir="$1"
  local required=(
    "LeftCameraParams.yaml"
    "RightCameraParams.yaml"
    "ImuParams.yaml"
  )
  local file
  for file in "${required[@]}"; do
    if [[ ! -f "${dir}/${file}" ]]; then
      echo "[ERR] Missing generated file: ${dir}/${file}"
      exit 1
    fi
  done
}

ensure_template_files() {
  local dir="$1"
  local required=(
    "PipelineParams.yaml"
    "FrontendParams.yaml"
    "BackendParams.yaml"
    "LcdParams.yaml"
    "DisplayParams.yaml"
  )
  local file
  for file in "${required[@]}"; do
    if [[ ! -f "${dir}/${file}" ]]; then
      echo "[ERR] Template is missing required file: ${dir}/${file}"
      exit 1
    fi
  done
}

resolve_workspace_path() {
  local path="$1"
  if [[ -z "${path}" ]]; then
    echo ""
    return 0
  fi
  if [[ "${path}" == /* ]]; then
    echo "${path}"
    return 0
  fi

  case "${path}" in
    Kimera-VIO/*|./Kimera-VIO/*|sensor_inspection_tools/*|./sensor_inspection_tools/*)
      echo "${WORKSPACE_ROOT}/${path#./}"
      ;;
    *)
      # Keep generic relative paths as-is (relative to caller cwd).
      echo "${path}"
      ;;
  esac
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sensor)
      [[ $# -ge 2 ]] || { echo "[ERR] --sensor requires a value."; usage; exit 1; }
      SENSOR="$2"
      shift 2
      ;;
    --output_dir)
      [[ $# -ge 2 ]] || { echo "[ERR] --output_dir requires a value."; usage; exit 1; }
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --build_dir)
      [[ $# -ge 2 ]] || { echo "[ERR] --build_dir requires a value."; usage; exit 1; }
      BUILD_DIR="$2"
      shift 2
      ;;
    --compose_params_dir)
      [[ $# -ge 2 ]] || { echo "[ERR] --compose_params_dir requires a value."; usage; exit 1; }
      COMPOSE_PARAMS_DIR="$2"
      shift 2
      ;;
    --template_params_dir)
      [[ $# -ge 2 ]] || { echo "[ERR] --template_params_dir requires a value."; usage; exit 1; }
      TEMPLATE_PARAMS_DIR="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      EXTRA_ARGS=("$@")
      break
      ;;
    *)
      echo "[ERR] Unknown argument: $1"
      usage
      exit 1
      ;;
  esac
done

if [[ -z "${SENSOR}" ]]; then
  echo "[ERR] --sensor <zed|orbbec> is required."
  usage
  exit 1
fi

if [[ "${SENSOR}" != "zed" && "${SENSOR}" != "orbbec" ]]; then
  echo "[ERR] --sensor must be 'zed' or 'orbbec'."
  exit 1
fi

DEFAULT_TEMPLATE_PARAMS_DIR=""
if [[ -z "${OUTPUT_DIR}" ]]; then
  if [[ "${SENSOR}" == "zed" ]]; then
    OUTPUT_DIR="${SCRIPT_DIR}/zed_sensor_yaml"
    DEFAULT_TEMPLATE_PARAMS_DIR="${SCRIPT_DIR}/../Kimera-VIO/params/ZED"
  else
    OUTPUT_DIR="${SCRIPT_DIR}/orbbec_sensor_yaml"
    DEFAULT_TEMPLATE_PARAMS_DIR="${SCRIPT_DIR}/../Kimera-VIO/params/Orbbec"
  fi
fi

if [[ "${SENSOR}" == "zed" ]]; then
  TOOL="${BUILD_DIR}/zed_sensor_inspector"
  if [[ -d "${SCRIPT_DIR}/../Kimera-VIO/params/ZED" ]]; then
    DEFAULT_TEMPLATE_PARAMS_DIR="${SCRIPT_DIR}/../Kimera-VIO/params/ZED"
  elif [[ -d "${SCRIPT_DIR}/../Kimera-VIO/params/ZED_inverted" ]]; then
    DEFAULT_TEMPLATE_PARAMS_DIR="${SCRIPT_DIR}/../Kimera-VIO/params/ZED_inverted"
  else
    DEFAULT_TEMPLATE_PARAMS_DIR="${SCRIPT_DIR}/../Kimera-VIO/params/Euroc"
  fi
else
  TOOL="${BUILD_DIR}/orbbec_sensor_inspector"
  if [[ -d "${SCRIPT_DIR}/../Kimera-VIO/params/Orbbec" ]]; then
    DEFAULT_TEMPLATE_PARAMS_DIR="${SCRIPT_DIR}/../Kimera-VIO/params/Orbbec"
  else
    DEFAULT_TEMPLATE_PARAMS_DIR="${SCRIPT_DIR}/../Kimera-VIO/params/Euroc"
  fi
fi

if [[ -z "${TEMPLATE_PARAMS_DIR}" ]]; then
  TEMPLATE_PARAMS_DIR="${DEFAULT_TEMPLATE_PARAMS_DIR}"
fi

OUTPUT_DIR="$(resolve_workspace_path "${OUTPUT_DIR}")"
COMPOSE_PARAMS_DIR="$(resolve_workspace_path "${COMPOSE_PARAMS_DIR}")"
TEMPLATE_PARAMS_DIR="$(resolve_workspace_path "${TEMPLATE_PARAMS_DIR}")"

if [[ "${OUTPUT_DIR}" != /* ]]; then
  OUTPUT_DIR="$(pwd)/${OUTPUT_DIR}"
fi
if [[ -n "${COMPOSE_PARAMS_DIR}" && "${COMPOSE_PARAMS_DIR}" != /* ]]; then
  COMPOSE_PARAMS_DIR="$(pwd)/${COMPOSE_PARAMS_DIR}"
fi
if [[ "${TEMPLATE_PARAMS_DIR}" != /* ]]; then
  TEMPLATE_PARAMS_DIR="$(pwd)/${TEMPLATE_PARAMS_DIR}"
fi

if [[ ! -x "${TOOL}" ]]; then
  echo "[ERR] Missing executable: ${TOOL}"
  echo "Build first:"
  echo "  cmake -S ${SCRIPT_DIR} -B ${BUILD_DIR}"
  echo "  cmake --build ${BUILD_DIR} -j"
  exit 1
fi

TMP_SENSOR_DIR="$(mktemp -d "${SCRIPT_DIR}/.tmp_sensor_yaml.XXXXXX")"
trap 'rm -rf "${TMP_SENSOR_DIR}"' EXIT

if [[ "${SENSOR}" == "orbbec" ]]; then
  echo "[INFO] Inspecting Orbbec device summary before YAML export..."
  "${TOOL}" --describe "${EXTRA_ARGS[@]}"
fi

echo "[INFO] Reading ${SENSOR} sensor and generating Kimera sensor YAMLs..."
"${TOOL}" --dump_kimera_yaml --output_dir "${TMP_SENSOR_DIR}" "${EXTRA_ARGS[@]}"
ensure_dump_files "${TMP_SENSOR_DIR}"

mkdir -p "${OUTPUT_DIR}"
echo "[INFO] Saving ${SENSOR} Kimera sensor YAMLs to: ${OUTPUT_DIR}"
cp -f "${TMP_SENSOR_DIR}/LeftCameraParams.yaml" "${OUTPUT_DIR}/"
cp -f "${TMP_SENSOR_DIR}/RightCameraParams.yaml" "${OUTPUT_DIR}/"
cp -f "${TMP_SENSOR_DIR}/ImuParams.yaml" "${OUTPUT_DIR}/"
if [[ -f "${TMP_SENSOR_DIR}/README.txt" ]]; then
  cp -f "${TMP_SENSOR_DIR}/README.txt" "${OUTPUT_DIR}/"
fi

if [[ -n "${COMPOSE_PARAMS_DIR}" ]]; then
  if [[ ! -d "${TEMPLATE_PARAMS_DIR}" ]]; then
    echo "[ERR] --template_params_dir does not exist: ${TEMPLATE_PARAMS_DIR}"
    exit 1
  fi

  ensure_template_files "${TEMPLATE_PARAMS_DIR}"

  mkdir -p "${COMPOSE_PARAMS_DIR}"
  cp -a "${TEMPLATE_PARAMS_DIR}/." "${COMPOSE_PARAMS_DIR}/"
  cp -f "${TMP_SENSOR_DIR}/LeftCameraParams.yaml" "${COMPOSE_PARAMS_DIR}/"
  cp -f "${TMP_SENSOR_DIR}/RightCameraParams.yaml" "${COMPOSE_PARAMS_DIR}/"
  cp -f "${TMP_SENSOR_DIR}/ImuParams.yaml" "${COMPOSE_PARAMS_DIR}/"
  if [[ -f "${TMP_SENSOR_DIR}/README.txt" ]]; then
    cp -f "${TMP_SENSOR_DIR}/README.txt" "${COMPOSE_PARAMS_DIR}/SensorDumpREADME.txt"
  fi
  echo "[INFO] Composed full Kimera params folder: ${COMPOSE_PARAMS_DIR}"
  echo "[INFO] Template source: ${TEMPLATE_PARAMS_DIR}"
fi

echo "[INFO] Done."
