#!/bin/bash
# Syncs React build output to colcon build/install paths after npm run build.
# Called automatically via package.json postbuild.

set -euo pipefail

FRONTEND_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../camrod_ui_robot/assets/frontend" && pwd)"
WS_ROOT="$(realpath "$FRONTEND_DIR/../../../../..")"

SRC="$FRONTEND_DIR/build"
COLCON="$WS_ROOT/build/camrod_ui/camrod_ui_robot/assets/frontend/build"
INSTALL="$WS_ROOT/install/camrod_ui/share/camrod_ui/camrod_ui_robot/assets/frontend/build"

copy_if_needed() {
  local src_file="$1"
  local dst_file="$2"
  mkdir -p "$(dirname "$dst_file")"
  if [ -e "$dst_file" ] && [ "$(realpath "$src_file")" = "$(realpath "$dst_file")" ]; then
    return 0
  fi
  if [ ! -e "$dst_file" ] || [ "$(stat -Lc '%d:%i' "$src_file")" != "$(stat -Lc '%d:%i' "$dst_file" 2>/dev/null)" ]; then
    cp "$src_file" "$dst_file"
  fi
}

publish_index() {
  local dst="$1"
  local src_real
  local dst_real
  src_real="$(realpath "$SRC/index.html")"
  dst_real="$(realpath "$dst/index.html" 2>/dev/null || true)"
  if [ "$src_real" = "$dst_real" ]; then
    return 0
  fi

  local temporary_index="$dst/.index.html.tmp.$$"
  cp "$SRC/index.html" "$temporary_index"
  mv -f "$temporary_index" "$dst/index.html"
}

sync_build_tree() {
  local dst="$1"
  local src_real
  local dst_real
  src_real="$(realpath "$SRC")"
  dst_real="$(realpath "$dst")"

  if [ "$src_real" = "$dst_real" ]; then
    return 0
  fi

  mkdir -p "$dst/static"

  # Publish every asset before exposing the index that names the new build.
  # Public files live at the build root while compiled bundles live below
  # static/, so both trees must be refreshed.
  while IFS= read -r -d '' src_file; do
    rel_path="${src_file#"$SRC/"}"
    copy_if_needed "$src_file" "$dst/$rel_path"
  done < <(
    find "$SRC" -type f \
      ! -path "$SRC/index.html" \
      ! -path "$SRC/static/*" \
      -print0
  )

  while IFS= read -r -d '' src_file; do
    rel_path="${src_file#"$SRC/static/"}"
    copy_if_needed "$src_file" "$dst/static/$rel_path"
  done < <(find "$SRC/static" -type f -print0)

  publish_index "$dst"

  # HH_260619 - Remove stale hashed React files only after the new index is
  # atomically visible, so in-flight requests can finish during publication.
  while IFS= read -r -d '' dst_file; do
    rel_path="${dst_file#"$dst/static/"}"
    if [ ! -f "$SRC/static/$rel_path" ]; then
      rm -f "$dst_file"
    fi
  done < <(find "$dst/static" \( -type f -o -type l \) -print0)
}

synced=0
if [ -d "$COLCON" ]; then
  sync_build_tree "$COLCON"
  synced=1
fi
if [ -d "$INSTALL" ]; then
  sync_build_tree "$INSTALL"
  synced=1
fi

if [ "$synced" -eq 0 ]; then
  echo "[sync_frontend_build] sync pending; colcon will install the new bundle"
else
  echo "[sync_frontend_build] done → $(grep -o 'main\.[a-z0-9]*\.js' "$SRC/index.html")"
fi
