#!/usr/bin/env bash
# Run selected camping sites one at a time with site-scoped visual/wheel evidence.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SCRIPT_VERSION="2"
DEFAULT_SITES="B1,B2,B3,B4,B5,B6,B7,B8,B9,B10,B11,B12,B13"
CAPTURE_DURATION_SECONDS="86400"
CAPTURE_FPS="${CAMROD_SITE_EVIDENCE_CAPTURE_FPS:-1}"
GIF_FPS="${CAMROD_SITE_EVIDENCE_GIF_FPS:-8}"
DERIVED_WIDTH="${CAMROD_SITE_EVIDENCE_DERIVED_WIDTH:-1280}"
WHEEL_RATE_HZ="${CAMROD_SITE_EVIDENCE_WHEEL_RATE_HZ:-10.0}"
MINIMUM_CAPTURE_SECONDS=12

usage() {
  cat <<'EOF'
Usage:
  run_site_evidence_matrix.sh [plan] [--sites B1,B2,...] [--output-root PATH]
  run_site_evidence_matrix.sh run [--authority operator|guest] \
    --sites B1,B2,... --output-root /absolute/new/path
  run_site_evidence_matrix.sh --dry-run [--sites B1,B2,...] [--output-root PATH]

Actions:
  plan       print the exact passive-recording and camping-sites workflow;
             create nothing and send no motion (default)
  run        run each selected site independently and stop after the first
             matrix or evidence failure
  --dry-run  alias for plan; it always overrides a preceding run action

Options:
  --sites CSV        ordered, unique subset of B1..B13 (default: all sites)
  --authority MODE   frontend authority: operator or guest (default: operator)
  --output-root PATH new or empty absolute evidence directory (required by run)
  --display DISPLAY  X11 display passed to capture_ui_evidence.sh
  --xauthority PATH  Xauthority passed to capture_ui_evidence.sh
  -h, --help         show this help

For every site the runner starts the read-only physical-wheel recorder and the
already-visible CarlaUE4 + selected CAMROD UI capture before invoking exactly
one of these production matrices:

  operator: CAMROD_CARLA_CAMPING_SITES=<site> run.sh camping-sites
  guest:    CAMROD_CARLA_CAMPING_SITES=<site> run.sh camping-sites-guest

The capture is finalized with ffmpeg's interactive `q`; the wheel recorder is
stopped with SIGINT. PNG/GIF, telemetry, logs, hashes and manifests are kept,
while the temporary source MP4 is removed by capture_ui_evidence.sh only after
its derivatives validate. The script does not start/stop CARLA or CAMROD,
publish commands, click the UI, or invoke any motion path other than the
selected existing run.sh matrix. Guest mode requires the visible browser from
`run.sh guest-ui`; that matrix drives its real navigate -> usage_complete page
functions and verifies the exact guest:usage_complete ROS return source.
EOF
}

die() {
  printf '[site-evidence-matrix] ERROR: %s\n' "$*" >&2
  exit 1
}

log() {
  printf '[site-evidence-matrix] %s\n' "$*"
}

render_command() {
  local argument
  for argument in "$@"; do
    printf '%q ' "${argument}"
  done
  printf '\n'
}

ACTION="plan"
SITES_CSV="${DEFAULT_SITES}"
AUTHORITY="operator"
OUTPUT_ROOT=""
DISPLAY_VALUE="${DISPLAY:-}"
XAUTHORITY_VALUE="${XAUTHORITY:-}"

if [[ $# -gt 0 ]]; then
  case "$1" in
    plan|run) ACTION="$1"; shift ;;
    help|-h|--help) usage; exit 0 ;;
  esac
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sites|--authority|--output-root|--display|--xauthority)
      [[ $# -ge 2 ]] || die "missing value for $1"
      option="$1"
      value="$2"
      shift 2
      case "${option}" in
        --sites) SITES_CSV="${value}" ;;
        --authority) AUTHORITY="${value}" ;;
        --output-root) OUTPUT_ROOT="${value}" ;;
        --display) DISPLAY_VALUE="${value}" ;;
        --xauthority) XAUTHORITY_VALUE="${value}" ;;
      esac
      ;;
    --dry-run) ACTION="plan"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

# env.sh is the single source of the CARLA endpoint, gate-bound Python API,
# native evidence root and lifecycle entrypoint used by both observers and the
# existing matrix. Sourcing it launches nothing and sends no command.
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

validate_sites() {
  [[ -n "${SITES_CSV}" ]] || die "--sites cannot be empty"
  local raw site
  local -A seen=()
  IFS=',' read -r -a SELECTED_SITES <<<"${SITES_CSV}"
  (( ${#SELECTED_SITES[@]} > 0 )) || die "--sites selected no sites"
  for raw in "${SELECTED_SITES[@]}"; do
    site="${raw}"
    [[ "${site}" =~ ^B([1-9]|1[0-3])$ ]] || \
      die "invalid site ${site@Q}; expected an uppercase site in B1..B13"
    [[ ! -v "seen[${site}]" ]] || die "duplicate site: ${site}"
    seen["${site}"]=1
  done
  SITES_CSV="$(IFS=,; printf '%s' "${SELECTED_SITES[*]}")"
}

validate_number_settings() {
  python3 - "${CAPTURE_FPS}" "${GIF_FPS}" "${DERIVED_WIDTH}" \
      "${WHEEL_RATE_HZ}" <<'PY'
import math
import sys

capture_fps, gif_fps, width, wheel_rate = sys.argv[1:]
try:
    capture_fps_i = int(capture_fps)
    gif_fps_i = int(gif_fps)
    width_i = int(width)
    wheel_rate_f = float(wheel_rate)
except ValueError as error:
    raise SystemExit(f"[site-evidence-matrix] ERROR: invalid numeric setting: {error}")
if not 1 <= capture_fps_i <= 30:
    raise SystemExit("[site-evidence-matrix] ERROR: capture FPS must be in [1, 30]")
if not 1 <= gif_fps_i <= 20:
    raise SystemExit("[site-evidence-matrix] ERROR: GIF FPS must be in [1, 20]")
if not 320 <= width_i <= 1920:
    raise SystemExit("[site-evidence-matrix] ERROR: derived width must be in [320, 1920]")
if not math.isfinite(wheel_rate_f) or not 0.1 <= wheel_rate_f <= 100.0:
    raise SystemExit("[site-evidence-matrix] ERROR: wheel rate must be in [0.1, 100] Hz")
PY
}

validate_sites
case "${AUTHORITY}" in
  operator|guest) ;;
  *) die "--authority must be operator or guest: ${AUTHORITY}" ;;
esac
command -v python3 >/dev/null 2>&1 || die "required tool not found: python3"
validate_number_settings

RUNNER="${CAMROD_VIRTUAL_CARLA_ENTRYPOINT:-${SCRIPT_DIR}/run.sh}"
CAPTURE_SCRIPT="${SCRIPT_DIR}/capture_ui_evidence.sh"
WHEEL_RECORDER="${SCRIPT_DIR}/record_physical_wheel_telemetry.py"

case "${AUTHORITY}" in
  operator)
    MATRIX_SUBCOMMAND="camping-sites"
    MATRIX_ROOT="${RANGER_EVIDENCE_ROOT}/camrod_camping_site_matrix"
    MATRIX_RETURN_AUTHORITY="operator_rest"
    EXPECTED_RETURN_SOURCE=""
    CAPTURE_UI_TITLE="CAMROD Operator UI"
    CAPTURE_UI_KIND="operator"
    ;;
  guest)
    MATRIX_SUBCOMMAND="camping-sites-guest"
    MATRIX_ROOT="${RANGER_EVIDENCE_ROOT}/camrod_camping_site_matrix_guest_usage_complete"
    MATRIX_RETURN_AUTHORITY="guest_browser"
    EXPECTED_RETURN_SOURCE="guest:usage_complete"
    CAPTURE_UI_TITLE="국립공원 로봇 서비스"
    CAPTURE_UI_KIND="guest"
    ;;
esac

print_plan() {
  local shown_root="${OUTPUT_ROOT:-/absolute/path/to/new-site-evidence-run}"
  cat <<EOF
[site-evidence-matrix] PLAN ONLY -- no directory, CARLA/ROS/UI query, capture,
[site-evidence-matrix] recorder or vehicle command was started.

Selected sites (strict order): ${SITES_CSV}
Frontend authority: ${AUTHORITY} (${MATRIX_RETURN_AUTHORITY})
Expected return source: ${EXPECTED_RETURN_SOURCE:-operator REST /ui/manual_return}
Output root: ${shown_root}
Native matrix root: ${MATRIX_ROOT}

Prerequisites already running and healthy:
  server -> bridge -> pacer -> spawn -> camrod, with the visible CarlaUE4
  window left of the visible ${CAPTURE_UI_TITLE} window.

EOF
  if [[ "${AUTHORITY}" == "guest" ]]; then
    cat <<EOF
  Keep a separate '${RUNNER} guest-ui' terminal open so the one visible Guest
  page owns its WebSocket and local-only CDP endpoint for usage_complete.

EOF
  fi
  cat <<EOF
For each site, independently and stopping on the first failure:
  1. Start ${WHEEL_RECORDER} at ${WHEEL_RATE_HZ} Hz (read-only).
  2. Start ${CAPTURE_SCRIPT} at ${CAPTURE_FPS} FPS (read-only pixels).
  3. Run the only motion-authorized command:
EOF
  local site
  for site in "${SELECTED_SITES[@]}"; do
    printf '       CAMROD_CARLA_CAMPING_SITES=%q %q %q\n' \
      "${site}" "${RUNNER}" "${MATRIX_SUBCOMMAND}"
  done
  cat <<EOF
  4. Send the letter q to the capture and SIGINT to the wheel recorder.
  5. Keep PNG/GIF and JSONL; remove MP4 after validated derivation.
  6. Bind the native camping_site_matrix.json path, bytes and SHA-256 into
     <output>/<site>/site_manifest.json and the run_manifest.json index.

Live command:
  ${0@Q} run --authority ${AUTHORITY@Q} --sites ${SITES_CSV@Q} --output-root ${shown_root@Q}
EOF
}

if [[ "${ACTION}" == "plan" ]]; then
  print_plan
  exit 0
fi

[[ "${ACTION}" == "run" ]] || die "internal unsupported action: ${ACTION}"

validate_output_root() {
  [[ -n "${OUTPUT_ROOT}" ]] || die "run requires --output-root /absolute/new/path"
  [[ "${OUTPUT_ROOT}" == /* ]] || die "--output-root must be absolute: ${OUTPUT_ROOT}"
  [[ "${OUTPUT_ROOT}" != "/" ]] || die "--output-root cannot be /"
  [[ ! -L "${OUTPUT_ROOT}" ]] || die "--output-root must not be a symlink: ${OUTPUT_ROOT}"
  if [[ -e "${OUTPUT_ROOT}" && ! -d "${OUTPUT_ROOT}" ]]; then
    die "--output-root exists but is not a directory: ${OUTPUT_ROOT}"
  fi
  if [[ -d "${OUTPUT_ROOT}" && \
        -n "$(find "${OUTPUT_ROOT}" -mindepth 1 -print -quit)" ]]; then
    die "--output-root must be empty; evidence is never overwritten: ${OUTPUT_ROOT}"
  fi
}

require_live_prerequisites() {
  local tool
  for tool in bash find mkfifo python3 sha256sum tee; do
    command -v "${tool}" >/dev/null 2>&1 || die "required tool not found: ${tool}"
  done
  [[ -x "${RUNNER}" ]] || die "lifecycle runner is not executable: ${RUNNER}"
  [[ -x "${CAPTURE_SCRIPT}" ]] || die "capture script is not executable: ${CAPTURE_SCRIPT}"
  [[ -f "${WHEEL_RECORDER}" ]] || die "wheel recorder not found: ${WHEEL_RECORDER}"
  [[ -n "${DISPLAY_VALUE}" ]] || die "DISPLAY is empty; use --display from the graphical session"
  if [[ -n "${XAUTHORITY_VALUE}" && ! -r "${XAUTHORITY_VALUE}" ]]; then
    die "XAUTHORITY is not readable: ${XAUTHORITY_VALUE}"
  fi
  [[ -n "${RANGER_EVIDENCE_ROOT:-}" && "${RANGER_EVIDENCE_ROOT}" == /* ]] || \
    die "env.sh must provide an absolute RANGER_EVIDENCE_ROOT"
  [[ "${CARLA_RENDER_MODE}" != "nullrhi" ]] || \
    die "visual evidence requires a rendered CARLA mode"
  virtual_carla_use_python_egg

  local -a validate_command=(
    "${CAPTURE_SCRIPT}" validate
    --display "${DISPLAY_VALUE}"
    --ui-window-title "${CAPTURE_UI_TITLE}"
    --ui-kind "${CAPTURE_UI_KIND}"
  )
  if [[ -n "${XAUTHORITY_VALUE}" ]]; then
    validate_command+=(--xauthority "${XAUTHORITY_VALUE}")
  fi
  log "preflight: validating already-visible CARLA/UI windows before creating evidence"
  "${validate_command[@]}"
}

validate_output_root
require_live_prerequisites
mkdir -p -- "${OUTPUT_ROOT}"
OUTPUT_ROOT="$(readlink -m "${OUTPUT_ROOT}")"

RUN_STARTED_AT_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
TEMP_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/camrod-site-evidence.XXXXXX")"
CURRENT_CAPTURE_PID=""
CURRENT_RECORDER_PID=""
CURRENT_CAPTURE_FD=""
CURRENT_CAPTURE_FIFO=""

cleanup_processes() {
  set +e
  if [[ -n "${CURRENT_CAPTURE_PID}" ]] && kill -0 "${CURRENT_CAPTURE_PID}" 2>/dev/null; then
    if [[ -n "${CURRENT_CAPTURE_FD}" ]]; then
      printf 'q' >&"${CURRENT_CAPTURE_FD}" 2>/dev/null
    fi
  fi
  if [[ -n "${CURRENT_RECORDER_PID}" ]] && kill -0 "${CURRENT_RECORDER_PID}" 2>/dev/null; then
    kill -INT "${CURRENT_RECORDER_PID}" 2>/dev/null
  fi
  [[ -z "${CURRENT_RECORDER_PID}" ]] || wait "${CURRENT_RECORDER_PID}" 2>/dev/null
  [[ -z "${CURRENT_CAPTURE_PID}" ]] || wait "${CURRENT_CAPTURE_PID}" 2>/dev/null
  if [[ -n "${CURRENT_CAPTURE_FD}" ]]; then
    exec {CURRENT_CAPTURE_FD}>&-
    CURRENT_CAPTURE_FD=""
  fi
  [[ -z "${CURRENT_CAPTURE_FIFO}" ]] || rm -f -- "${CURRENT_CAPTURE_FIFO}"
  if [[ -n "${TEMP_ROOT:-}" && "${TEMP_ROOT}" != "/" && -d "${TEMP_ROOT}" ]]; then
    rm -rf -- "${TEMP_ROOT}"
  fi
}

trap cleanup_processes EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

write_run_manifest() {
  local status="$1" failure_site="$2" reason="$3" exit_code="$4"
  python3 - "${OUTPUT_ROOT}" "${SITES_CSV}" "${status}" "${failure_site}" \
      "${reason}" "${exit_code}" "${RUN_STARTED_AT_UTC}" "${SCRIPT_VERSION}" \
      "${SOURCE_ROOT}" "${BASH_SOURCE[0]}" "${AUTHORITY}" \
      "${MATRIX_SUBCOMMAND}" "${MATRIX_ROOT}" "${MATRIX_RETURN_AUTHORITY}" \
      "${EXPECTED_RETURN_SOURCE}" "${CAPTURE_UI_KIND}" <<'PY'
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

(
    root_raw,
    sites_raw,
    status,
    failure_site,
    reason,
    exit_code_raw,
    started_at,
    version_raw,
    source_root_raw,
    script_raw,
    authority,
    matrix_subcommand,
    matrix_root_raw,
    matrix_return_authority,
    expected_return_source,
    capture_ui_kind,
) = sys.argv[1:]
root = Path(root_raw)
source_root = Path(source_root_raw)
script = Path(script_raw)
sites = sites_raw.split(",")

def artifact(path: Path) -> dict:
    return {
        "path": str(path),
        "bytes": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }

site_results = []
for site in sites:
    path = root / site / "site_manifest.json"
    if path.is_file():
        site_results.append(json.loads(path.read_text(encoding="utf-8")))

try:
    branch = subprocess.check_output(
        ["git", "-C", str(source_root), "branch", "--show-current"], text=True
    ).strip()
    head = subprocess.check_output(
        ["git", "-C", str(source_root), "rev-parse", "HEAD"], text=True
    ).strip()
except (OSError, subprocess.CalledProcessError):
    branch = head = ""

document = {
    "schema": "camrod.virtual_carla.site_evidence_matrix.v1",
    "status": status,
    "created_at_utc": started_at,
    "updated_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
    "script_version": int(version_raw),
    "selected_sites": sites,
    "completed_site_manifests": len(site_results),
    "stop_on_first_failure": True,
    "failure_site": failure_site or None,
    "failure_reason": reason or None,
    "exit_code": int(exit_code_raw),
    "source": {
        "git_root": str(source_root),
        "branch": branch,
        "head": head,
        "runner": artifact(script),
    },
    "motion_authority": {
        "frontend": authority,
        "matrix_return_authority": matrix_return_authority,
        "expected_return_source": expected_return_source,
        "only_command": f"run.sh {matrix_subcommand}",
        "native_matrix_root": matrix_root_raw,
        "captured_ui_kind": capture_ui_kind,
        "sites_run_independently": True,
        "observers_publish_or_control_vehicle": False,
    },
    "sites": site_results,
}
temporary = root / ".run_manifest.json.tmp"
target = root / "run_manifest.json"
temporary.write_text(
    json.dumps(document, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
)
os.replace(temporary, target)
PY
}

write_run_manifest "RUNNING" "" "" 0

wait_for_file_from_process() {
  local pid="$1" path="$2" label="$3" attempts="${4:-300}"
  local attempt
  for ((attempt = 0; attempt < attempts; attempt += 1)); do
    [[ -s "${path}" ]] && return 0
    if ! kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}" 2>/dev/null || true
      log "${label} exited before its readiness file appeared: ${path}"
      return 1
    fi
    sleep 0.1
  done
  log "${label} did not become ready within $((attempts / 10)) seconds"
  return 1
}

discover_matrix_report() {
  local matrix_log="$1" site="$2" marker="$3" matrix_root="$4"
  python3 - "${matrix_log}" "${matrix_root}" \
      "${site}" "${marker}" <<'PY'
import json
from pathlib import Path
import sys

log_path, root_raw, expected_site, marker_raw = sys.argv[1:]
root = Path(root_raw).resolve()
marker_ns = Path(marker_raw).stat().st_mtime_ns
candidates = []
for line in Path(log_path).read_text(encoding="utf-8", errors="replace").splitlines():
    if "report=" not in line:
        continue
    raw = line.split("report=", 1)[1].strip()
    if raw.endswith("camping_site_matrix.json"):
        candidates.append(Path(raw))

accepted = []
for candidate in dict.fromkeys(candidates):
    try:
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(root)
        if resolved.stat().st_mtime_ns < marker_ns:
            continue
        report = json.loads(resolved.read_text(encoding="utf-8"))
        report_sites = [item.get("site") for item in report.get("sites", [])]
        if report_sites != [expected_site]:
            continue
    except (OSError, ValueError, json.JSONDecodeError):
        continue
    accepted.append(resolved)

if len(accepted) > 1:
    raise SystemExit(
        "multiple fresh, site-matched matrix reports were printed: "
        + ", ".join(map(str, accepted))
    )
if accepted:
    print(accepted[0])
PY
}

finalize_site_manifest() {
  local site="$1" site_dir="$2" matrix_report="$3" matrix_status="$4"
  local tee_status="$5" recorder_status="$6" capture_status="$7" q_sent="$8"
  python3 - "${site}" "${site_dir}" "${matrix_report}" "${matrix_status}" \
      "${tee_status}" "${recorder_status}" "${capture_status}" "${q_sent}" \
      "${SCRIPT_VERSION}" "${AUTHORITY}" "${MATRIX_SUBCOMMAND}" \
      "${MATRIX_RETURN_AUTHORITY}" "${EXPECTED_RETURN_SOURCE}" \
      "${CAPTURE_UI_KIND}" <<'PY'
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import sys

(
    site,
    site_dir_raw,
    matrix_report_raw,
    matrix_exit_raw,
    tee_exit_raw,
    recorder_exit_raw,
    capture_exit_raw,
    q_sent_raw,
    version_raw,
    authority,
    matrix_subcommand,
    matrix_return_authority,
    expected_return_source,
    capture_ui_kind,
) = sys.argv[1:]
site_dir = Path(site_dir_raw)
matrix_exit = int(matrix_exit_raw)
tee_exit = int(tee_exit_raw)
recorder_exit = int(recorder_exit_raw)
capture_exit = int(capture_exit_raw)
reasons = []

def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()

def artifact(path: Path) -> dict:
    return {"path": str(path), "bytes": path.stat().st_size, "sha256": sha256(path)}

def load_json(path: Path, label: str):
    if not path.is_file():
        reasons.append(f"missing {label}: {path}")
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        reasons.append(f"invalid {label}: {error}")
        return None

if matrix_exit != 0:
    reasons.append(f"run.sh {matrix_subcommand} exited {matrix_exit}")
if tee_exit != 0:
    reasons.append(f"matrix log tee exited {tee_exit}")
if recorder_exit != 0:
    reasons.append(f"wheel recorder exited {recorder_exit}")
if capture_exit != 0:
    reasons.append(f"UI capture exited {capture_exit}")
if q_sent_raw != "true":
    reasons.append("capture was not finalized with q")

matrix_record = None
if matrix_report_raw:
    matrix_path = Path(matrix_report_raw)
    matrix = load_json(matrix_path, "camping-site matrix report")
    if matrix is not None:
        matrix_sites = [item.get("site") for item in matrix.get("sites", [])]
        matrix_item = matrix.get("sites", [{}])[0] if matrix.get("sites") else {}
        matrix_scope = matrix.get("scope", {})
        if matrix.get("schema") != "camrod.virtual_carla.camping_site_matrix.v1":
            reasons.append("unexpected camping-site matrix schema")
        if matrix.get("status") != "PASS":
            reasons.append(f"camping-site matrix status is {matrix.get('status')!r}")
        if matrix_sites != [site]:
            reasons.append(f"camping-site matrix selection is {matrix_sites!r}, expected {[site]!r}")
        if matrix_scope.get("return_authority") != matrix_return_authority:
            reasons.append(
                "camping-site matrix return authority is "
                f"{matrix_scope.get('return_authority')!r}, expected "
                f"{matrix_return_authority!r}"
            )
        if matrix_scope.get("expected_return_source", "") != expected_return_source:
            reasons.append(
                "camping-site matrix expected return source is "
                f"{matrix_scope.get('expected_return_source')!r}, expected "
                f"{expected_return_source!r}"
            )
        if authority == "guest":
            response = matrix_item.get("return_response") or {}
            if response.get("action") != "usage_complete":
                reasons.append("guest matrix did not record action=usage_complete")
            if response.get("transport") != "visible_guest_page_websocket_via_cdp":
                reasons.append("guest return did not use the visible page WebSocket via CDP")
            requests = matrix_item.get("ui_operation_request_sequence") or []
            if not any(
                isinstance(request, dict)
                and request.get("operation") == 3
                and request.get("source") == expected_return_source
                for request in requests
            ):
                reasons.append(
                    "guest matrix did not observe RETURN operation source "
                    f"{expected_return_source!r}"
                )
        matrix_record = artifact(matrix_path)
        matrix_record["status"] = matrix.get("status")
        matrix_record["site_status"] = (
            matrix.get("sites", [{}])[0].get("status") if matrix.get("sites") else None
        )
        link = site_dir / "camping_site_matrix.json"
        if not link.is_symlink() or link.resolve() != matrix_path.resolve():
            reasons.append("site matrix symlink does not resolve to the native report")
        matrix_record["evidence_link"] = str(link)
else:
    reasons.append("run.sh did not print a fresh site-matched camping_site_matrix.json")

wheel_path = site_dir / "physical_wheels.jsonl"
wheel_manifest_path = site_dir / "physical_wheels.manifest.json"
wheel_manifest = load_json(wheel_manifest_path, "physical-wheel manifest")
wheel_record = None
if wheel_manifest is not None:
    if wheel_manifest.get("schema") != "camrod.virtual_carla.physical_wheel_telemetry_manifest.v1":
        reasons.append("unexpected physical-wheel manifest schema")
    if wheel_manifest.get("status") != "STOPPED" or wheel_manifest.get("stop_reason") != "signal":
        reasons.append("physical-wheel recorder did not finalize through its signal path")
    if wheel_manifest.get("read_only") is not True:
        reasons.append("physical-wheel manifest is not marked read-only")
    if int(wheel_manifest.get("sample_count") or 0) <= 0:
        reasons.append("physical-wheel recorder contains no samples")
    if wheel_path.is_file():
        expected = wheel_manifest.get("output", {}).get("sha256")
        actual = sha256(wheel_path)
        if expected != actual:
            reasons.append("physical-wheel JSONL SHA-256 does not match its manifest")
        wheel_record = {
            "jsonl": artifact(wheel_path),
            "manifest": artifact(wheel_manifest_path),
            "status": wheel_manifest.get("status"),
            "sample_count": wheel_manifest.get("sample_count"),
        }
    else:
        reasons.append(f"missing physical-wheel JSONL: {wheel_path}")

capture_root = site_dir / "visual"
capture_manifest_path = capture_root / "capture_manifest.json"
capture_manifest = load_json(capture_manifest_path, "visual capture manifest")
visual_record = None
if capture_manifest is not None:
    if capture_manifest.get("schema") != "camrod.virtual_carla.desktop_ui_capture.v4":
        reasons.append("unexpected visual capture manifest schema")
    if capture_manifest.get("status") != "PASS":
        reasons.append(f"visual capture status is {capture_manifest.get('status')!r}")
    scope = capture_manifest.get("scope", {})
    if scope.get("vehicle_motion_or_ui_input_sent_by_capture") is not False:
        reasons.append("visual capture does not prove passive operation")
    ui_key = "camrod_guest_ui" if capture_ui_kind == "guest" else "camrod_operator_ui"
    captured_ui = capture_manifest.get("x11", {}).get("windows", {}).get(ui_key)
    if not isinstance(captured_ui, dict) or captured_ui.get("kind") != capture_ui_kind:
        reasons.append(
            f"visual capture does not contain the expected {capture_ui_kind} UI window"
        )
    video = capture_manifest.get("recording", {}).get("source_video", {})
    if video.get("retained") is not False or video.get("removed_after_derivation") is not True:
        reasons.append("source MP4 was not removed after derivative validation")
    mp4 = capture_root / "carla_camrod_desktop.mp4"
    if mp4.exists() or mp4.is_symlink():
        reasons.append("source MP4 still exists")
    visual_record = {
        "manifest": artifact(capture_manifest_path),
        "source_mp4_retained": False,
        "capture_finalized_with_q": q_sent_raw == "true",
    }
    for manifest_key, filename, output_key in (
        ("contact_sheet_png", "representative_contact_sheet.png", "png"),
        ("representative_gif", "representative_motion.gif", "gif"),
    ):
        path = capture_root / filename
        if not path.is_file():
            reasons.append(f"missing visual derivative: {path}")
            continue
        expected = capture_manifest.get("artifacts", {}).get(manifest_key, {}).get("sha256")
        actual = sha256(path)
        if expected != actual:
            reasons.append(f"visual derivative SHA-256 mismatch: {filename}")
        visual_record[output_key] = artifact(path)

logs = {}
for filename in ("matrix.log", "physical_wheels.log", "capture.log"):
    path = site_dir / filename
    if path.is_file():
        logs[filename] = artifact(path)

matrix_metrics = None
if matrix_report_raw and matrix_record is not None:
    item = json.loads(Path(matrix_report_raw).read_text(encoding="utf-8"))["sites"][0]
    matrix_metrics = {
        key: item.get(key)
        for key in (
            "elapsed_s",
            "outbound_duration_s",
            "return_duration_s",
            "outbound_distance_m",
            "return_distance_m",
            "total_odom_distance_m",
        )
    }

document = {
    "schema": "camrod.virtual_carla.site_evidence.v1",
    "status": "PASS" if not reasons else "FAIL",
    "created_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
    "script_version": int(version_raw),
    "site": site,
    "stop_on_failure": True,
    "authority": {
        "frontend": authority,
        "matrix_return_authority": matrix_return_authority,
        "expected_return_source": expected_return_source,
        "captured_ui_kind": capture_ui_kind,
    },
    "matrix_command": {
        "argv": ["run.sh", matrix_subcommand],
        "environment": {"CAMROD_CARLA_CAMPING_SITES": site},
        "exit_code": matrix_exit,
        "tee_exit_code": tee_exit,
        "only_motion_authority": True,
    },
    "matrix_report": matrix_record,
    "motion_metrics": matrix_metrics,
    "physical_wheels": wheel_record,
    "visual": visual_record,
    "logs": logs,
    "failure_reasons": reasons,
}
temporary = site_dir / ".site_manifest.json.tmp"
target = site_dir / "site_manifest.json"
temporary.write_text(
    json.dumps(document, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
)
os.replace(temporary, target)
raise SystemExit(0 if not reasons else 1)
PY
}

OVERALL_STATUS=0
FAILURE_SITE=""
FAILURE_REASON=""

for site in "${SELECTED_SITES[@]}"; do
  site_dir="${OUTPUT_ROOT}/${site}"
  mkdir -- "${site_dir}"
  marker="${site_dir}/.matrix-start-marker"
  : >"${marker}"
  matrix_log="${site_dir}/matrix.log"
  recorder_log="${site_dir}/physical_wheels.log"
  capture_log="${site_dir}/capture.log"
  wheel_output="${site_dir}/physical_wheels.jsonl"
  wheel_manifest="${site_dir}/physical_wheels.manifest.json"
  visual_dir="${site_dir}/visual"
  egg_cache="${TEMP_ROOT}/${site}-python-egg-cache"
  capture_fifo="${TEMP_ROOT}/${site}-capture-control.fifo"
  mkdir -- "${egg_cache}"
  mkfifo -- "${capture_fifo}"

  log "${site}: starting passive physical-wheel recorder"
  (
    export PYTHON_EGG_CACHE="${egg_cache}"
    exec python3 "${WHEEL_RECORDER}" \
      --output "${wheel_output}" \
      --manifest "${wheel_manifest}" \
      --host "${CARLA_HOST}" \
      --port "${CARLA_PORT}" \
      --rate-hz "${WHEEL_RATE_HZ}"
  ) >"${recorder_log}" 2>&1 &
  CURRENT_RECORDER_PID=$!
  if ! wait_for_file_from_process \
      "${CURRENT_RECORDER_PID}" "${wheel_output}" "${site} wheel recorder" 150; then
    OVERALL_STATUS=1
    FAILURE_SITE="${site}"
    FAILURE_REASON="physical-wheel recorder failed before motion"
    write_run_manifest "FAIL" "${FAILURE_SITE}" "${FAILURE_REASON}" "${OVERALL_STATUS}"
    exit "${OVERALL_STATUS}"
  fi

  # Open both ends in this parent so neither the capture shell nor this runner
  # blocks while ffmpeg is starting. The only byte sent through it is `q`.
  CURRENT_CAPTURE_FIFO="${capture_fifo}"
  exec {CURRENT_CAPTURE_FD}<>"${capture_fifo}"
  capture_command=(
    "${CAPTURE_SCRIPT}" capture
    --output-dir "${visual_dir}"
    --duration-seconds "${CAPTURE_DURATION_SECONDS}"
    --capture-fps "${CAPTURE_FPS}"
    --gif-fps "${GIF_FPS}"
    --derived-width "${DERIVED_WIDTH}"
    --retain-source-video false
    --allow-short-capture true
    --display "${DISPLAY_VALUE}"
    --ui-window-title "${CAPTURE_UI_TITLE}"
    --ui-kind "${CAPTURE_UI_KIND}"
  )
  if [[ -n "${XAUTHORITY_VALUE}" ]]; then
    capture_command+=(--xauthority "${XAUTHORITY_VALUE}")
  fi
  log "${site}: starting passive side-by-side UI capture"
  "${capture_command[@]}" <"${capture_fifo}" >"${capture_log}" 2>&1 &
  CURRENT_CAPTURE_PID=$!
  if ! wait_for_file_from_process "${CURRENT_CAPTURE_PID}" \
      "${visual_dir}/carla_camrod_desktop.mp4" "${site} UI capture" 300; then
    OVERALL_STATUS=1
    FAILURE_SITE="${site}"
    FAILURE_REASON="UI capture failed before motion"
    write_run_manifest "FAIL" "${FAILURE_SITE}" "${FAILURE_REASON}" "${OVERALL_STATUS}"
    exit "${OVERALL_STATUS}"
  fi
  capture_started_seconds="${SECONDS}"

  log "${site}: invoking the existing ${MATRIX_SUBCOMMAND} matrix (only motion authority)"
  log "RUN CAMROD_CARLA_CAMPING_SITES=${site@Q} $(render_command "${RUNNER}" "${MATRIX_SUBCOMMAND}")"
  set +e
  CAMROD_CARLA_CAMPING_SITES="${site}" \
    "${RUNNER}" "${MATRIX_SUBCOMMAND}" 2>&1 | tee "${matrix_log}"
  pipeline_status=("${PIPESTATUS[@]}")
  set -e
  matrix_status="${pipeline_status[0]}"
  tee_status="${pipeline_status[1]}"

  # capture_ui_evidence.sh accepts interactive finalization only after twelve
  # seconds. A very fast preflight failure is held just long enough to retain a
  # valid diagnostic PNG/GIF; no later site is started in that interval.
  capture_elapsed=$((SECONDS - capture_started_seconds))
  if (( capture_elapsed < MINIMUM_CAPTURE_SECONDS )) && \
      kill -0 "${CURRENT_CAPTURE_PID}" 2>/dev/null; then
    sleep "$((MINIMUM_CAPTURE_SECONDS - capture_elapsed))"
  fi

  q_sent=false
  if kill -0 "${CURRENT_CAPTURE_PID}" 2>/dev/null; then
    printf 'q' >&"${CURRENT_CAPTURE_FD}"
    q_sent=true
  fi
  exec {CURRENT_CAPTURE_FD}>&-
  CURRENT_CAPTURE_FD=""

  if kill -0 "${CURRENT_RECORDER_PID}" 2>/dev/null; then
    kill -INT "${CURRENT_RECORDER_PID}"
  fi
  set +e
  wait "${CURRENT_RECORDER_PID}"
  recorder_status=$?
  CURRENT_RECORDER_PID=""
  wait "${CURRENT_CAPTURE_PID}"
  capture_status=$?
  CURRENT_CAPTURE_PID=""
  set -e
  rm -f -- "${capture_fifo}"
  CURRENT_CAPTURE_FIFO=""

  matrix_report=""
  if discovered="$(discover_matrix_report \
      "${matrix_log}" "${site}" "${marker}" "${MATRIX_ROOT}" \
      2>>"${matrix_log}")"; then
    matrix_report="${discovered}"
  fi
  if [[ -n "${matrix_report}" ]]; then
    ln -s -- "${matrix_report}" "${site_dir}/camping_site_matrix.json"
  fi
  rm -f -- "${marker}"

  site_manifest_status=0
  if ! finalize_site_manifest "${site}" "${site_dir}" "${matrix_report}" \
      "${matrix_status}" "${tee_status}" "${recorder_status}" \
      "${capture_status}" "${q_sent}"; then
    site_manifest_status=1
  fi

  if (( matrix_status != 0 || tee_status != 0 || recorder_status != 0 || \
        capture_status != 0 || site_manifest_status != 0 )); then
    OVERALL_STATUS="${matrix_status}"
    (( OVERALL_STATUS != 0 )) || OVERALL_STATUS=1
    FAILURE_SITE="${site}"
    FAILURE_REASON="site matrix or its bound evidence failed; inspect ${site_dir}/site_manifest.json"
    write_run_manifest "FAIL" "${FAILURE_SITE}" "${FAILURE_REASON}" "${OVERALL_STATUS}"
    log "${site}: FAIL; no later site will be started"
    exit "${OVERALL_STATUS}"
  fi

  log "${site}: PASS; PNG/GIF, wheel telemetry and native matrix link verified"
  write_run_manifest "RUNNING" "" "" 0
done

write_run_manifest "PASS" "" "" 0
log "PASS: ${#SELECTED_SITES[@]} site(s); evidence index=${OUTPUT_ROOT}/run_manifest.json"
