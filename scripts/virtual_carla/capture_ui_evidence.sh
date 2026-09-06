#!/usr/bin/env bash
# Reproducible, non-controlling CARLA + CAMROD UI evidence capture.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SCRIPT_VERSION="4"
DEFAULT_CARLA_TITLE="CarlaUE4"
DEFAULT_UI_TITLE="CAMROD Operator UI"
DEFAULT_DURATION_SECONDS="60"
DEFAULT_CAPTURE_FPS="5"
DEFAULT_GIF_FPS="8"
DEFAULT_DERIVED_WIDTH="960"
DEFAULT_CLIP_SECONDS="1.25"
DEFAULT_GEOMETRY_TOLERANCE_PX="48"

usage() {
  cat <<'EOF'
Usage:
  capture_ui_evidence.sh [plan]
  capture_ui_evidence.sh validate [options]
  capture_ui_evidence.sh capture --output-dir /absolute/empty/path [options]

Actions:
  plan      print the workflow and example commands; read/write nothing (default)
  validate  find the two live X11 windows and validate side-by-side geometry;
            write nothing
  capture   validate the windows, record one desktop region, then derive a PNG
            contact sheet, GIF, ffprobe metadata, exact commands and SHA-256 list

Options:
  --output-dir PATH           new or empty absolute directory (capture requires it)
  --duration-seconds N        recording duration, 12..86400 (default: 60)
  --capture-fps N             X11 MP4 capture rate, 1..30 (default: 5)
  --gif-fps N                 representative GIF rate, 1..20 (default: 8)
  --derived-width PX          width of each PNG/GIF panel, 320..1920 (default: 960)
  --clip-seconds N            GIF seconds around each of six samples, 0.25..5
  --geometry-tolerance-px PX  accepted frame/decorator gap, 0..256 (default: 48)
  --retain-source-video BOOL  keep the source MP4 after deriving PNG/GIF;
                              true or false (default: true)
  --allow-short-capture BOOL  accept an interactive ffmpeg 'q' after at least
                              12 seconds and derive from the actual duration;
                              true or false (default: false)
  --display DISPLAY           X11 display (default: current DISPLAY)
  --xauthority PATH           readable Xauthority file (default: current XAUTHORITY)
  --carla-window-title TEXT   literal title substring (default: CarlaUE4)
  --ui-window-title TEXT      literal title substring; use Robot UI for a browser
                              (default: CAMROD Operator UI)
  --ui-kind KIND              manifest identity: operator or guest
                              (default: operator)
  --carla-window-id HEX       explicit X11 id if the title is not unique
  --ui-window-id HEX          explicit X11 id if the title is not unique
  -h, --help                  show this help

The script never starts/stops CARLA or ROS, publishes motion, clicks the UI,
repositions windows, or overwrites evidence. It records the already-visible
desktop pixels only. Arrange CarlaUE4 on the left and the selected CAMROD UI on
the right before validate/capture. The resulting visual is not CARLA sensor-camera,
physical-hardware, or AI-generated evidence; pair it with runtime gate/topic
reports for functional acceptance. Window id, title and geometry are checked
before and after recording; X11 cannot prove that another window never occluded
part of the selected desktop region.
EOF
}

die() {
  printf '[virtual_carla_capture] ERROR: %s\n' "$*" >&2
  exit 1
}

log() {
  printf '[virtual_carla_capture] %s\n' "$*"
}

render_command() {
  local argument
  for argument in "$@"; do
    printf '%q ' "${argument}"
  done
  printf '\n'
}

absolute_difference() {
  local left="$1" right="$2"
  if (( left >= right )); then
    printf '%d\n' "$((left - right))"
  else
    printf '%d\n' "$((right - left))"
  fi
}

require_integer_range() {
  local label="$1" value="$2" minimum="$3" maximum="$4"
  [[ "${value}" =~ ^[0-9]+$ ]] || die "${label} must be an integer: ${value}"
  (( value >= minimum && value <= maximum )) || \
    die "${label} must be in [${minimum}, ${maximum}]: ${value}"
}

require_float_range() {
  local label="$1" value="$2" minimum="$3" maximum="$4"
  python3 - "${label}" "${value}" "${minimum}" "${maximum}" <<'PY' || exit 1
import math
import sys

label, raw, low_raw, high_raw = sys.argv[1:]
try:
    value = float(raw)
    low = float(low_raw)
    high = float(high_raw)
except ValueError:
    print(f"[virtual_carla_capture] ERROR: {label} must be numeric: {raw}", file=sys.stderr)
    raise SystemExit(1)
if not math.isfinite(value) or not low <= value <= high:
    print(
        f"[virtual_carla_capture] ERROR: {label} must be in [{low:g}, {high:g}]: {raw}",
        file=sys.stderr,
    )
    raise SystemExit(1)
PY
}

print_plan() {
  local shown_output="${OUTPUT_DIR:-/absolute/path/to/new-capture-directory}"
  local shown_ui_label="CAMROD Operator UI"
  if [[ "${UI_KIND}" == "guest" ]]; then
    shown_ui_label="CAMROD Guest UI"
  fi
  cat <<EOF
[virtual_carla_capture] PLAN ONLY -- no window query, directory creation, or capture occurred.

Prerequisites:
  1. Start server -> bridge -> pacer -> spawn -> camrod with CARLA_RENDER_MODE=onscreen.
  2. Keep one visible CarlaUE4 window on the left and one visible
     ${shown_ui_label} window on the right.
  3. Run the read-only geometry check:

  ${0@Q} validate

Explicit recording command (${DEFAULT_DURATION_SECONDS} s default):

  ${0@Q} capture --output-dir ${shown_output@Q} \\
    --duration-seconds ${DURATION_SECONDS} --capture-fps ${CAPTURE_FPS}

Capture creates only files below that new/empty directory. It uses ffmpeg
x11grab once for a single already-tiled desktop region, then derives six-sample
PNG/GIF assets from that MP4 and records exact commands, ffprobe JSON, metadata,
and SHA-256 hashes. Source-video retention is ${RETAIN_SOURCE_VIDEO}; when false,
the MP4 is removed only after the derivatives and metadata have been validated,
and the manifest records its pre-removal size/hash. Short-capture acceptance is
${ALLOW_SHORT_CAPTURE}; when true, ffmpeg may be ended with its interactive 'q'
after 12 seconds and derivatives use the measured duration. No runtime, ROS
topic, UI control, or vehicle command changes.
EOF
}

require_capture_tools() {
  local tool
  for tool in xwininfo ffmpeg ffprobe python3 sha256sum; do
    command -v "${tool}" >/dev/null 2>&1 || die "required tool not found: ${tool}"
  done
  [[ -n "${DISPLAY_VALUE}" ]] || die "DISPLAY is empty; use --display from the graphical session"
  if [[ -n "${XAUTHORITY_VALUE}" && ! -r "${XAUTHORITY_VALUE}" ]]; then
    die "XAUTHORITY is not readable: ${XAUTHORITY_VALUE}"
  fi
}

x11_command_prefix() {
  # Keep xwininfo's field labels deterministic while preserving UTF-8 window
  # titles such as the Korean Guest UI.  Plain LC_ALL=C converts the UTF-8
  # _NET_WM_NAME to ASCII and makes exact title discovery fail closed.
  X11_PREFIX=(env "DISPLAY=${DISPLAY_VALUE}" LC_ALL=C.UTF-8)
  if [[ -n "${XAUTHORITY_VALUE}" ]]; then
    X11_PREFIX+=("XAUTHORITY=${XAUTHORITY_VALUE}")
  fi
}

discover_window_id() {
  local label="$1" needle="$2" explicit_id="$3"
  local tree candidate
  local -a candidates=()
  if [[ -n "${explicit_id}" ]]; then
    [[ "${explicit_id}" =~ ^0x[0-9a-fA-F]+$ ]] || \
      die "${label} window id must be hexadecimal (0x...): ${explicit_id}"
    printf '%s\n' "${explicit_id}"
    return 0
  fi

  tree="$("${X11_PREFIX[@]}" xwininfo -root -tree 2>/dev/null)" || \
    die "cannot query X11 window tree on ${DISPLAY_VALUE}"
  while IFS= read -r candidate; do
    [[ -n "${candidate}" ]] && candidates+=("${candidate}")
  done < <(
    awk -v needle="${needle}" '
      index($0, needle) > 0 && $1 ~ /^0x[0-9a-fA-F]+$/ { print $1 }
    ' <<<"${tree}" | sort -u
  )
  if (( ${#candidates[@]} == 0 )); then
    die "no ${label} X11 window contains title text: ${needle@Q}"
  fi
  if (( ${#candidates[@]} != 1 )); then
    printf '[virtual_carla_capture] ERROR: %s title is ambiguous (%d windows):' \
      "${label}" "${#candidates[@]}" >&2
    printf ' %s' "${candidates[@]}" >&2
    printf '\nUse --%s-window-id 0x... explicitly.\n' "${label}" >&2
    exit 1
  fi
  printf '%s\n' "${candidates[0]}"
}

read_window_geometry() {
  local label="$1" window_id="$2" info title expected_title x y width height map_state
  info="$("${X11_PREFIX[@]}" xwininfo -id "${window_id}" 2>/dev/null)" || \
    die "cannot inspect ${label} window id ${window_id}"
  title="$(sed -n 's/^xwininfo: Window id: [^ ]* "\(.*\)"$/\1/p' <<<"${info}" | head -n1)"
  case "${label}" in
    carla) expected_title="${CARLA_TITLE}" ;;
    ui) expected_title="${UI_TITLE}" ;;
    *) die "internal unknown window label: ${label}" ;;
  esac
  [[ -n "${title}" && "${title}" == *"${expected_title}"* ]] || \
    die "${label} window ${window_id} title ${title@Q} does not contain expected ${expected_title@Q}"
  x="$(awk -F: '/Absolute upper-left X:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}' <<<"${info}")"
  y="$(awk -F: '/Absolute upper-left Y:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}' <<<"${info}")"
  width="$(awk -F: '/^[[:space:]]*Width:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}' <<<"${info}")"
  height="$(awk -F: '/^[[:space:]]*Height:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}' <<<"${info}")"
  map_state="$(awk -F: '/^[[:space:]]*Map State:/ {sub(/^[[:space:]]+/, "", $2); print $2; exit}' <<<"${info}")"
  [[ "${x}" =~ ^-?[0-9]+$ && "${y}" =~ ^-?[0-9]+$ ]] || \
    die "cannot parse ${label} absolute X11 position"
  [[ "${width}" =~ ^[0-9]+$ && "${height}" =~ ^[0-9]+$ ]] || \
    die "cannot parse ${label} X11 size"
  (( width >= 320 && height >= 240 )) || \
    die "${label} window is too small for evidence: ${width}x${height}"
  [[ "${map_state}" == "IsViewable" ]] || \
    die "${label} window is not visible (Map State=${map_state:-unknown})"

  case "${label}" in
    carla)
      CARLA_ID="${window_id}"; CARLA_TITLE_RESOLVED="${title}"
      CARLA_X="${x}"; CARLA_Y="${y}"; CARLA_WIDTH="${width}"; CARLA_HEIGHT="${height}"
      ;;
    ui)
      UI_ID="${window_id}"; UI_TITLE_RESOLVED="${title}"
      UI_X="${x}"; UI_Y="${y}"; UI_WIDTH="${width}"; UI_HEIGHT="${height}"
      ;;
    *) die "internal unknown window label: ${label}" ;;
  esac
}

validate_side_by_side_geometry() {
  local carla_right gap top_delta height_delta union_right union_bottom
  (( CARLA_X >= 0 && CARLA_Y >= 0 && UI_X >= 0 && UI_Y >= 0 )) || \
    die "windows must be on the non-negative X11 capture plane"
  (( CARLA_X < UI_X )) || \
    die "CarlaUE4 must be left of the selected UI window"
  carla_right=$((CARLA_X + CARLA_WIDTH))
  gap=$((UI_X - carla_right))
  (( gap >= -GEOMETRY_TOLERANCE_PX && gap <= GEOMETRY_TOLERANCE_PX )) || \
    die "windows are not contiguous side-by-side: horizontal gap=${gap}px, tolerance=${GEOMETRY_TOLERANCE_PX}px"
  top_delta="$(absolute_difference "${CARLA_Y}" "${UI_Y}")"
  height_delta="$(absolute_difference "${CARLA_HEIGHT}" "${UI_HEIGHT}")"
  (( top_delta <= GEOMETRY_TOLERANCE_PX )) || \
    die "window tops differ by ${top_delta}px (tolerance=${GEOMETRY_TOLERANCE_PX}px)"
  (( height_delta <= GEOMETRY_TOLERANCE_PX * 2 )) || \
    die "window heights differ by ${height_delta}px (limit=$((GEOMETRY_TOLERANCE_PX * 2))px)"

  CAPTURE_X=$((CARLA_X < UI_X ? CARLA_X : UI_X))
  CAPTURE_Y=$((CARLA_Y < UI_Y ? CARLA_Y : UI_Y))
  union_right=$((carla_right > UI_X + UI_WIDTH ? carla_right : UI_X + UI_WIDTH))
  union_bottom=$((CARLA_Y + CARLA_HEIGHT > UI_Y + UI_HEIGHT ? CARLA_Y + CARLA_HEIGHT : UI_Y + UI_HEIGHT))
  CAPTURE_WIDTH=$((union_right - CAPTURE_X))
  CAPTURE_HEIGHT=$((union_bottom - CAPTURE_Y))
  (( CARLA_WIDTH * 4 >= CAPTURE_WIDTH && UI_WIDTH * 4 >= CAPTURE_WIDTH )) || \
    die "one window occupies less than 25% of the capture region"

  log "geometry VERIFIED: carla=${CARLA_ID} ${CARLA_WIDTH}x${CARLA_HEIGHT}+${CARLA_X}+${CARLA_Y} title=${CARLA_TITLE_RESOLVED@Q}"
  log "geometry VERIFIED: ui=${UI_ID} ${UI_WIDTH}x${UI_HEIGHT}+${UI_X}+${UI_Y} title=${UI_TITLE_RESOLVED@Q}"
  log "single desktop capture region=${CAPTURE_WIDTH}x${CAPTURE_HEIGHT}+${CAPTURE_X}+${CAPTURE_Y} gap=${gap}px"
}

inspect_windows() {
  require_capture_tools
  x11_command_prefix
  local resolved_carla_id resolved_ui_id
  resolved_carla_id="$(discover_window_id carla "${CARLA_TITLE}" "${CARLA_WINDOW_ID}")"
  resolved_ui_id="$(discover_window_id ui "${UI_TITLE}" "${UI_WINDOW_ID}")"
  [[ "${resolved_carla_id,,}" != "${resolved_ui_id,,}" ]] || \
    die "CARLA and UI resolved to the same X11 window: ${resolved_carla_id}"
  read_window_geometry carla "${resolved_carla_id}"
  read_window_geometry ui "${resolved_ui_id}"
  validate_side_by_side_geometry
}

validate_output_directory_request() {
  [[ -n "${OUTPUT_DIR}" ]] || die "capture requires --output-dir /absolute/empty/path"
  [[ "${OUTPUT_DIR}" == /* ]] || die "--output-dir must be absolute: ${OUTPUT_DIR}"
  [[ "${OUTPUT_DIR}" != "/" ]] || die "--output-dir cannot be /"
  [[ ! -L "${OUTPUT_DIR}" ]] || die "--output-dir must not be a symlink: ${OUTPUT_DIR}"
  if [[ -e "${OUTPUT_DIR}" && ! -d "${OUTPUT_DIR}" ]]; then
    die "--output-dir exists but is not a directory: ${OUTPUT_DIR}"
  fi
  if [[ -d "${OUTPUT_DIR}" && -n "$(find "${OUTPUT_DIR}" -mindepth 1 -print -quit)" ]]; then
    die "--output-dir must be empty; no existing evidence is overwritten: ${OUTPUT_DIR}"
  fi
}

prepare_output_directory() {
  validate_output_directory_request
  mkdir -p -- "${OUTPUT_DIR}"
}

append_exact_command() {
  local command_file="$1"
  shift
  render_command "$@" >>"${command_file}"
}

run_and_log_command() {
  local command_file="$1"
  shift
  append_exact_command "${command_file}" "$@"
  log "RUN $(render_command "$@")"
  "$@"
}

write_rejection_marker() {
  local reason="$1"
  python3 - "${OUTPUT_DIR}" "${SCRIPT_VERSION}" "${reason}" <<'PY'
import datetime as dt
import hashlib
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
artifacts = {}
for path in sorted(root.iterdir()):
    if not path.is_file() or path.name == "capture_rejection.json":
        continue
    artifacts[path.name] = {
        "bytes": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }
document = {
    "schema": "camrod.virtual_carla.desktop_ui_capture_rejection.v1",
    "status": "REJECTED",
    "created_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
    "script_version": int(sys.argv[2]),
    "reason": sys.argv[3],
    "accepted_visual_evidence": False,
    "replacement_required": True,
    "retained_unaccepted_artifacts": artifacts,
}
(root / "capture_rejection.json").write_text(
    json.dumps(document, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
)
PY
}

capture_evidence() {
  prepare_output_directory
  local carla_geometry_before="${CARLA_WIDTH}x${CARLA_HEIGHT}+${CARLA_X}+${CARLA_Y}"
  local ui_geometry_before="${UI_WIDTH}x${UI_HEIGHT}+${UI_X}+${UI_Y}"
  local region_before="${CAPTURE_WIDTH}x${CAPTURE_HEIGHT}+${CAPTURE_X}+${CAPTURE_Y}"
  local carla_title_before="${CARLA_TITLE_RESOLVED}"
  local ui_title_before="${UI_TITLE_RESOLVED}"
  local video="${OUTPUT_DIR}/carla_camrod_desktop.mp4"
  local sheet="${OUTPUT_DIR}/representative_contact_sheet.png"
  local gif="${OUTPUT_DIR}/representative_motion.gif"
  local probe="${OUTPUT_DIR}/ffprobe.json"
  local commands="${OUTPUT_DIR}/exact_commands.txt"
  local manifest="${OUTPUT_DIR}/capture_manifest.json"
  local hashes="${OUTPUT_DIR}/sha256sums.txt"
  : >"${commands}"

  local -a capture_command=(
    "${X11_PREFIX[@]}" ffmpeg -n -hide_banner -loglevel warning
    -f x11grab -draw_mouse 0 -framerate "${CAPTURE_FPS}"
    -video_size "${CAPTURE_WIDTH}x${CAPTURE_HEIGHT}"
    -i "${DISPLAY_VALUE}+${CAPTURE_X},${CAPTURE_Y}"
    -t "${DURATION_SECONDS}" -an
    -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2:color=black"
    -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p
    -movflags +faststart "${video}"
  )
  if ! run_and_log_command "${commands}" "${capture_command[@]}"; then
    write_rejection_marker "ffmpeg desktop recording failed"
    die "ffmpeg desktop recording failed; evidence is not accepted"
  fi

  # Fail closed if either captured window was replaced, moved, resized or
  # unmapped during recording. X11 cannot prove that no other window occluded
  # these pixels, so the manifest states that limitation explicitly.
  read_window_geometry carla "${CARLA_ID}"
  read_window_geometry ui "${UI_ID}"
  validate_side_by_side_geometry
  local carla_geometry_after="${CARLA_WIDTH}x${CARLA_HEIGHT}+${CARLA_X}+${CARLA_Y}"
  local ui_geometry_after="${UI_WIDTH}x${UI_HEIGHT}+${UI_X}+${UI_Y}"
  local region_after="${CAPTURE_WIDTH}x${CAPTURE_HEIGHT}+${CAPTURE_X}+${CAPTURE_Y}"
  if [[ "${CARLA_TITLE_RESOLVED}" != "${carla_title_before}" \
      || "${UI_TITLE_RESOLVED}" != "${ui_title_before}" \
      || "${carla_geometry_after}" != "${carla_geometry_before}" \
      || "${ui_geometry_after}" != "${ui_geometry_before}" \
      || "${region_after}" != "${region_before}" ]]; then
    write_rejection_marker \
      "window title/geometry changed during capture; evidence is not accepted"
    die "window title/geometry changed during capture; evidence is not accepted"
  fi

  local -a probe_command=(
    ffprobe -v error -show_format -show_streams -of json "${video}"
  )
  append_exact_command "${commands}" "${probe_command[@]}"
  "${probe_command[@]}" >"${probe}"

  local -a duration_probe_command=(
    ffprobe -v error -show_entries format=duration
    -of default=nw=1:nk=1 "${video}"
  )
  append_exact_command "${commands}" "${duration_probe_command[@]}"
  local actual_duration duration_policy duration_tolerance early_finalized
  actual_duration="$("${duration_probe_command[@]}")"
  require_float_range "captured duration" "${actual_duration}" "1" "86401"
  if ! duration_policy="$(python3 - \
      "${actual_duration}" "${DURATION_SECONDS}" "${CAPTURE_FPS}" \
      "${ALLOW_SHORT_CAPTURE}" <<'PY'
import sys

actual, requested, fps = map(float, sys.argv[1:4])
allow_short = sys.argv[4] == "true"
tolerance = max(1.0, 2.0 / fps)
if allow_short:
    valid = 12.0 <= actual <= requested + tolerance
    expectation = (
        f"12.000s <= actual <= {requested + tolerance:.3f}s "
        "(--allow-short-capture=true)"
    )
else:
    valid = requested - tolerance <= actual <= requested + tolerance
    expectation = (
        f"{requested - tolerance:.3f}s <= actual <= "
        f"{requested + tolerance:.3f}s (strict default)"
    )
if not valid:
    print(
        "[virtual_carla_capture] ERROR: captured duration violates the selected "
        f"policy: actual={actual:.3f}s requested={requested:.3f}s "
        f"tolerance={tolerance:.3f}s expected={expectation}",
        file=sys.stderr,
    )
    raise SystemExit(1)
early_finalized = allow_short and actual < requested - tolerance
print(f"{tolerance:.6f}")
print(str(early_finalized).lower())
PY
  )"; then
    write_rejection_marker \
      "captured duration violates the selected short-capture policy"
    die "captured duration validation failed; evidence is not accepted"
  fi
  duration_tolerance="$(sed -n '1p' <<<"${duration_policy}")"
  early_finalized="$(sed -n '2p' <<<"${duration_policy}")"
  [[ "${duration_tolerance}" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
    die "internal duration tolerance parse failed: ${duration_tolerance}"
  case "${early_finalized}" in
    true|false) ;;
    *) die "internal early-finalized parse failed: ${early_finalized}" ;;
  esac
  printf '# capture_duration_policy allow_short_capture=%s requested_duration_s=%s actual_duration_s=%s tolerance_s=%s early_finalized=%s\n' \
    "${ALLOW_SHORT_CAPTURE}" "${DURATION_SECONDS}" "${actual_duration}" \
    "${duration_tolerance}" "${early_finalized}" >>"${commands}"

  local sample_data
  sample_data="$(python3 - "${actual_duration}" "${CAPTURE_FPS}" "${CLIP_SECONDS}" <<'PY'
import math
import sys

duration, fps, clip = map(float, sys.argv[1:])
fractions = (0.05, 0.22, 0.39, 0.56, 0.73, 0.90)
times = [min(max(duration * fraction, 0.0), max(0.0, duration - 0.001)) for fraction in fractions]
frames = [max(0, int(math.floor(value * fps))) for value in times]
intervals = []
for value in times:
    start = max(0.0, value - clip / 2.0)
    end = min(duration, start + clip)
    start = max(0.0, end - clip)
    intervals.append((start, end))
print(",".join(f"{value:.3f}" for value in times))
print(",".join(str(value) for value in frames))
print(",".join(f"{start:.3f}:{end:.3f}" for start, end in intervals))
PY
)"
  local sample_times sample_frames sample_intervals
  sample_times="$(sed -n '1p' <<<"${sample_data}")"
  sample_frames="$(sed -n '2p' <<<"${sample_data}")"
  sample_intervals="$(sed -n '3p' <<<"${sample_data}")"

  local contact_select="" gif_select="" value start end
  IFS=',' read -r -a frame_values <<<"${sample_frames}"
  for value in "${frame_values[@]}"; do
    [[ -z "${contact_select}" ]] || contact_select+="+"
    contact_select+="eq(n\\,${value})"
  done
  IFS=',' read -r -a interval_values <<<"${sample_intervals}"
  for value in "${interval_values[@]}"; do
    start="${value%%:*}"
    end="${value##*:}"
    [[ -z "${gif_select}" ]] || gif_select+="+"
    gif_select+="between(t\\,${start}\\,${end})"
  done

  local contact_filter
  contact_filter="select='${contact_select}',scale=${DERIVED_WIDTH}:-2:flags=lanczos,tile=2x3:padding=0:margin=0:color=black"
  local -a contact_command=(
    ffmpeg -n -hide_banner -loglevel warning -i "${video}"
    -vf "${contact_filter}" -frames:v 1 -vsync vfr "${sheet}"
  )
  run_and_log_command "${commands}" "${contact_command[@]}"

  local gif_filter
  gif_filter="[0:v]select='${gif_select}',setpts=N/(${CAPTURE_FPS}*TB),scale=${DERIVED_WIDTH}:-2:flags=lanczos,fps=${GIF_FPS},split[gif_a][gif_b];[gif_a]palettegen=stats_mode=diff:max_colors=192[palette];[gif_b][palette]paletteuse=dither=bayer:bayer_scale=3[gif_out]"
  local -a gif_command=(
    ffmpeg -n -hide_banner -loglevel warning -i "${video}"
    -filter_complex "${gif_filter}" -map "[gif_out]" -loop 0 "${gif}"
  )
  run_and_log_command "${commands}" "${gif_command[@]}"
  [[ -s "${sheet}" && -s "${gif}" ]] || \
    die "representative PNG/GIF derivative is empty"

  local source_video_bytes source_video_sha256 source_video_removed="false"
  source_video_bytes="$(stat -c '%s' -- "${video}")"
  source_video_sha256="$(sha256sum -- "${video}" | awk '{print $1}')"
  [[ "${source_video_bytes}" =~ ^[0-9]+$ && ${#source_video_sha256} -eq 64 ]] || \
    die "cannot record source MP4 provenance before retention handling"

  if [[ "${RETAIN_SOURCE_VIDEO}" == "false" ]]; then
    [[ "${video}" == "${OUTPUT_DIR}/carla_camrod_desktop.mp4" && -f "${video}" ]] || \
      die "refusing to remove an unexpected source-video path: ${video}"
    run_and_log_command "${commands}" rm -- "${video}"
    [[ ! -e "${video}" ]] || die "source MP4 removal did not complete: ${video}"
    source_video_removed="true"
  fi

  local -a retained_hash_targets=(
    "$(basename "${sheet}")"
    "$(basename "${gif}")"
    "$(basename "${probe}")"
    "$(basename "${commands}")"
    "$(basename "${manifest}")"
  )
  if [[ "${RETAIN_SOURCE_VIDEO}" == "true" ]]; then
    retained_hash_targets=("$(basename "${video}")" "${retained_hash_targets[@]}")
  fi
  printf '(cd %q && sha256sum' "${OUTPUT_DIR}" >>"${commands}"
  printf ' %q' "${retained_hash_targets[@]}" >>"${commands}"
  printf ' > %q)\n' "$(basename "${hashes}")" >>"${commands}"

  export CAPTURE_META_OUTPUT_DIR="${OUTPUT_DIR}"
  export CAPTURE_META_SOURCE_ROOT="${SOURCE_ROOT}"
  export CAPTURE_META_SCRIPT="${SCRIPT_DIR}/capture_ui_evidence.sh"
  export CAPTURE_META_VERSION="${SCRIPT_VERSION}"
  export CAPTURE_META_DISPLAY="${DISPLAY_VALUE}"
  export CAPTURE_META_XAUTHORITY="${XAUTHORITY_VALUE}"
  export CAPTURE_META_REQUESTED_DURATION="${DURATION_SECONDS}"
  export CAPTURE_META_ACTUAL_DURATION="${actual_duration}"
  export CAPTURE_META_ALLOW_SHORT_CAPTURE="${ALLOW_SHORT_CAPTURE}"
  export CAPTURE_META_EARLY_FINALIZED="${early_finalized}"
  export CAPTURE_META_DURATION_TOLERANCE="${duration_tolerance}"
  export CAPTURE_META_CAPTURE_FPS="${CAPTURE_FPS}"
  export CAPTURE_META_GIF_FPS="${GIF_FPS}"
  export CAPTURE_META_DERIVED_WIDTH="${DERIVED_WIDTH}"
  export CAPTURE_META_SAMPLE_TIMES="${sample_times}"
  export CAPTURE_META_SAMPLE_INTERVALS="${sample_intervals}"
  export CAPTURE_META_CARLA_ID="${CARLA_ID}"
  export CAPTURE_META_CARLA_TITLE="${CARLA_TITLE_RESOLVED}"
  export CAPTURE_META_CARLA_GEOMETRY_BEFORE="${carla_geometry_before}"
  export CAPTURE_META_CARLA_GEOMETRY_AFTER="${carla_geometry_after}"
  export CAPTURE_META_UI_ID="${UI_ID}"
  export CAPTURE_META_UI_KIND="${UI_KIND}"
  export CAPTURE_META_UI_TITLE="${UI_TITLE_RESOLVED}"
  export CAPTURE_META_UI_GEOMETRY_BEFORE="${ui_geometry_before}"
  export CAPTURE_META_UI_GEOMETRY_AFTER="${ui_geometry_after}"
  export CAPTURE_META_REGION="${region_before}"
  export CAPTURE_META_COMMANDS="${commands}"
  export CAPTURE_META_PROBE="${probe}"
  export CAPTURE_META_VIDEO="${video}"
  export CAPTURE_META_RETAIN_SOURCE_VIDEO="${RETAIN_SOURCE_VIDEO}"
  export CAPTURE_META_SOURCE_VIDEO_REMOVED="${source_video_removed}"
  export CAPTURE_META_SOURCE_VIDEO_BYTES="${source_video_bytes}"
  export CAPTURE_META_SOURCE_VIDEO_SHA256="${source_video_sha256}"
  export CAPTURE_META_SHEET="${sheet}"
  export CAPTURE_META_GIF="${gif}"
  python3 - <<'PY'
import datetime as dt
import hashlib
import json
import os
import subprocess
from pathlib import Path

root = Path(os.environ["CAPTURE_META_OUTPUT_DIR"])

def artifact(path_value):
    path = Path(path_value)
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return {"path": path.name, "bytes": path.stat().st_size, "sha256": digest}

def split_csv(name):
    return [float(value) for value in os.environ[name].split(",") if value]

def parse_intervals():
    result = []
    for value in os.environ["CAPTURE_META_SAMPLE_INTERVALS"].split(","):
        if not value:
            continue
        start, end = value.split(":", 1)
        result.append({"start_s": float(start), "end_s": float(end)})
    return result

source_root = Path(os.environ["CAPTURE_META_SOURCE_ROOT"])
try:
    git_root = subprocess.check_output(
        ["git", "-C", str(source_root), "rev-parse", "--show-toplevel"],
        text=True,
        stderr=subprocess.DEVNULL,
    ).strip()
    git_head = subprocess.check_output(
        ["git", "-C", git_root, "rev-parse", "HEAD"], text=True
    ).strip()
    git_branch = subprocess.check_output(
        ["git", "-C", git_root, "branch", "--show-current"], text=True
    ).strip()
    git_status = subprocess.check_output(
        ["git", "-C", git_root, "status", "--short", "--untracked-files=all"],
        text=True,
    ).splitlines()
except (OSError, subprocess.CalledProcessError):
    git_root = git_head = git_branch = ""
    git_status = []

probe = json.loads(Path(os.environ["CAPTURE_META_PROBE"]).read_text(encoding="utf-8"))
ui_kind = os.environ["CAPTURE_META_UI_KIND"]
ui_key = {
    "operator": "camrod_operator_ui",
    "guest": "camrod_guest_ui",
}[ui_kind]
ui_label = {
    "operator": "CAMROD Operator UI",
    "guest": "CAMROD Guest UI",
}[ui_kind]
retain_source_video = os.environ["CAPTURE_META_RETAIN_SOURCE_VIDEO"] == "true"
source_video_removed = os.environ["CAPTURE_META_SOURCE_VIDEO_REMOVED"] == "true"
source_video = {
    "path": Path(os.environ["CAPTURE_META_VIDEO"]).name,
    "retained": retain_source_video,
    "removed_after_derivation": source_video_removed,
    "bytes_before_retention_action": int(
        os.environ["CAPTURE_META_SOURCE_VIDEO_BYTES"]
    ),
    "sha256_before_retention_action": os.environ[
        "CAPTURE_META_SOURCE_VIDEO_SHA256"
    ],
}
artifacts = {
    "contact_sheet_png": artifact(os.environ["CAPTURE_META_SHEET"]),
    "representative_gif": artifact(os.environ["CAPTURE_META_GIF"]),
}
if retain_source_video:
    artifacts["source_mp4"] = artifact(os.environ["CAPTURE_META_VIDEO"])

document = {
    "schema": "camrod.virtual_carla.desktop_ui_capture.v4",
    "status": "PASS",
    "created_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
    "script_version": int(os.environ["CAPTURE_META_VERSION"]),
    "source": {
        "git_root": git_root,
        "branch": git_branch,
        "head": git_head,
        "worktree_clean": not git_status,
        "worktree_status": git_status,
        "capture_script": artifact(os.environ["CAPTURE_META_SCRIPT"]),
    },
    "scope": {
        "kind": "actual X11 desktop-region recording",
        "single_region_capture": True,
        "windows_post_composited": False,
        "carla_sensor_camera_evidence": False,
        "ai_generated_or_enhanced": False,
        "vehicle_motion_or_ui_input_sent_by_capture": False,
        "unoccluded_window_pixels_validated": False,
        "statement": (
            f"The script recorded already-visible CarlaUE4 and {ui_label} "
            "pixels. It did not command CARLA, ROS, the UI, or the vehicle."
        ),
    },
    "x11": {
        "display": os.environ["CAPTURE_META_DISPLAY"],
        "xauthority": os.environ["CAPTURE_META_XAUTHORITY"],
        "geometry_validated_side_by_side": True,
        "same_titles_and_geometry_before_after_capture": True,
        "unoccluded_pixels_validated": False,
        "capture_region": os.environ["CAPTURE_META_REGION"],
        "windows": {
            "carla": {
                "id": os.environ["CAPTURE_META_CARLA_ID"],
                "title": os.environ["CAPTURE_META_CARLA_TITLE"],
                "geometry_before": os.environ[
                    "CAPTURE_META_CARLA_GEOMETRY_BEFORE"],
                "geometry_after": os.environ[
                    "CAPTURE_META_CARLA_GEOMETRY_AFTER"],
            },
            ui_key: {
                "kind": ui_kind,
                "id": os.environ["CAPTURE_META_UI_ID"],
                "title": os.environ["CAPTURE_META_UI_TITLE"],
                "geometry_before": os.environ[
                    "CAPTURE_META_UI_GEOMETRY_BEFORE"],
                "geometry_after": os.environ[
                    "CAPTURE_META_UI_GEOMETRY_AFTER"],
            },
        },
    },
    "recording": {
        "requested_duration_s": float(os.environ["CAPTURE_META_REQUESTED_DURATION"]),
        "actual_duration_s": float(os.environ["CAPTURE_META_ACTUAL_DURATION"]),
        "allow_short_capture": (
            os.environ["CAPTURE_META_ALLOW_SHORT_CAPTURE"] == "true"
        ),
        "early_finalized": os.environ["CAPTURE_META_EARLY_FINALIZED"] == "true",
        "duration_validation_tolerance_s": float(
            os.environ["CAPTURE_META_DURATION_TOLERANCE"]
        ),
        "minimum_short_capture_duration_s": 12.0,
        "capture_fps": int(os.environ["CAPTURE_META_CAPTURE_FPS"]),
        "ffprobe": probe,
        "source_video": source_video,
    },
    "derivatives": {
        "panel_width_px": int(os.environ["CAPTURE_META_DERIVED_WIDTH"]),
        "gif_fps": int(os.environ["CAPTURE_META_GIF_FPS"]),
        "contact_sheet_sample_times_s": split_csv("CAPTURE_META_SAMPLE_TIMES"),
        "gif_intervals": parse_intervals(),
    },
    "exact_commands": Path(os.environ["CAPTURE_META_COMMANDS"]).read_text(
        encoding="utf-8"
    ).splitlines(),
    "artifacts": artifacts,
    "sha256sums_scope": (
        "retained files only; when source_video.retained is false, its "
        "pre-removal provenance is recorded under recording.source_video"
    ),
    "acceptance_limit": (
        "Visual evidence alone does not prove CARLA actor identity, sensor-source "
        "ownership, physical wheel telemetry, safety-gate state, mission success, "
        "or that no third-party X11 window occluded part of the recorded region."
    ),
}
(root / "capture_manifest.json").write_text(
    json.dumps(document, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
)
PY

  (
    cd "${OUTPUT_DIR}"
    sha256sum "${retained_hash_targets[@]}" >"$(basename "${hashes}")"
  )
  log "capture complete: ${OUTPUT_DIR}"
  log "verify with: (cd ${OUTPUT_DIR@Q} && sha256sum -c ${hashes##*/})"
}

ACTION="plan"
if [[ $# -gt 0 ]]; then
  case "$1" in
    plan|validate|capture) ACTION="$1"; shift ;;
    -h|--help|help) usage; exit 0 ;;
  esac
fi

OUTPUT_DIR=""
DURATION_SECONDS="${DEFAULT_DURATION_SECONDS}"
CAPTURE_FPS="${DEFAULT_CAPTURE_FPS}"
GIF_FPS="${DEFAULT_GIF_FPS}"
DERIVED_WIDTH="${DEFAULT_DERIVED_WIDTH}"
CLIP_SECONDS="${DEFAULT_CLIP_SECONDS}"
GEOMETRY_TOLERANCE_PX="${DEFAULT_GEOMETRY_TOLERANCE_PX}"
DISPLAY_VALUE="${DISPLAY:-}"
XAUTHORITY_VALUE="${XAUTHORITY:-}"
CARLA_TITLE="${DEFAULT_CARLA_TITLE}"
UI_TITLE="${DEFAULT_UI_TITLE}"
UI_KIND="operator"
RETAIN_SOURCE_VIDEO="true"
ALLOW_SHORT_CAPTURE="false"
CARLA_WINDOW_ID=""
UI_WINDOW_ID=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir|--duration-seconds|--capture-fps|--gif-fps|--derived-width|\
    --clip-seconds|--geometry-tolerance-px|--retain-source-video|--allow-short-capture|\
    --display|--xauthority|\
    --carla-window-title|--ui-window-title|--ui-kind|--carla-window-id|--ui-window-id)
      [[ $# -ge 2 ]] || die "missing value for $1"
      option="$1"; value="$2"; shift 2
      case "${option}" in
        --output-dir) OUTPUT_DIR="${value}" ;;
        --duration-seconds) DURATION_SECONDS="${value}" ;;
        --capture-fps) CAPTURE_FPS="${value}" ;;
        --gif-fps) GIF_FPS="${value}" ;;
        --derived-width) DERIVED_WIDTH="${value}" ;;
        --clip-seconds) CLIP_SECONDS="${value}" ;;
        --geometry-tolerance-px) GEOMETRY_TOLERANCE_PX="${value}" ;;
        --retain-source-video) RETAIN_SOURCE_VIDEO="${value}" ;;
        --allow-short-capture) ALLOW_SHORT_CAPTURE="${value}" ;;
        --display) DISPLAY_VALUE="${value}" ;;
        --xauthority) XAUTHORITY_VALUE="${value}" ;;
        --carla-window-title) CARLA_TITLE="${value}" ;;
        --ui-window-title) UI_TITLE="${value}" ;;
        --ui-kind) UI_KIND="${value}" ;;
        --carla-window-id) CARLA_WINDOW_ID="${value}" ;;
        --ui-window-id) UI_WINDOW_ID="${value}" ;;
      esac
      ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

command -v python3 >/dev/null 2>&1 || die "required tool not found: python3"
require_float_range "--duration-seconds" "${DURATION_SECONDS}" "12" "86400"
require_integer_range "--capture-fps" "${CAPTURE_FPS}" 1 30
require_integer_range "--gif-fps" "${GIF_FPS}" 1 20
require_integer_range "--derived-width" "${DERIVED_WIDTH}" 320 1920
require_float_range "--clip-seconds" "${CLIP_SECONDS}" "0.25" "5"
require_integer_range "--geometry-tolerance-px" "${GEOMETRY_TOLERANCE_PX}" 0 256
python3 - "${DURATION_SECONDS}" "${CLIP_SECONDS}" <<'PY' || exit 1
import sys

duration, clip = map(float, sys.argv[1:])
if clip * 8.0 > duration:
    print(
        "[virtual_carla_capture] ERROR: --duration-seconds must be at least "
        "8 times --clip-seconds so six representative GIF intervals do not overlap",
        file=sys.stderr,
    )
    raise SystemExit(1)
PY
[[ -n "${CARLA_TITLE}" ]] || die "--carla-window-title cannot be empty"
[[ -n "${UI_TITLE}" ]] || die "--ui-window-title cannot be empty"
case "${UI_KIND}" in
  operator|guest) ;;
  *) die "--ui-kind must be operator or guest: ${UI_KIND}" ;;
esac
case "${RETAIN_SOURCE_VIDEO}" in
  true|false) ;;
  *) die "--retain-source-video must be true or false: ${RETAIN_SOURCE_VIDEO}" ;;
esac
case "${ALLOW_SHORT_CAPTURE}" in
  true|false) ;;
  *) die "--allow-short-capture must be true or false: ${ALLOW_SHORT_CAPTURE}" ;;
esac

case "${ACTION}" in
  plan)
    print_plan
    ;;
  validate)
    inspect_windows
    log "validation only; no files were written"
    ;;
  capture)
    validate_output_directory_request
    inspect_windows
    capture_evidence
    ;;
  *) die "internal unknown action: ${ACTION}" ;;
esac
