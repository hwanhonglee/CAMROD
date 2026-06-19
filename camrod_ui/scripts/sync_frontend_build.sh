#!/bin/bash
# Syncs React build output to colcon build/install paths after npm run build.
# Called automatically via package.json postbuild.

FRONTEND_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../camrod_ui_robot/assets/frontend" && pwd)"
WS_ROOT="$(realpath "$FRONTEND_DIR/../../../../..")"

SRC="$FRONTEND_DIR/build"
COLCON="$WS_ROOT/build/camrod_ui/camrod_ui_robot/assets/frontend/build"
INSTALL="$WS_ROOT/install/camrod_ui/share/camrod_ui/camrod_ui_robot/assets/frontend/build"

if [ ! -d "$COLCON" ] || [ ! -d "$INSTALL" ]; then
  echo "[sync_frontend_build] colcon paths not found, skipping (run colcon build first)"
  exit 0
fi

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

sync_build_tree() {
  local dst="$1"
  local src_real
  local dst_real
  src_real="$(realpath "$SRC")"
  dst_real="$(realpath "$dst")"

  if [ "$src_real" = "$dst_real" ]; then
    return 0
  fi

  copy_if_needed "$SRC/index.html" "$dst/index.html"
  mkdir -p "$dst/static"

  # HH_260619 - Remove stale hashed React files so colcon never installs deleted bundle names.
  while IFS= read -r -d '' dst_file; do
    rel_path="${dst_file#"$dst/static/"}"
    if [ ! -f "$SRC/static/$rel_path" ]; then
      rm -f "$dst_file"
    fi
  done < <(find "$dst/static" \( -type f -o -type l \) -print0)

  while IFS= read -r -d '' src_file; do
    rel_path="${src_file#"$SRC/static/"}"
    copy_if_needed "$src_file" "$dst/static/$rel_path"
  done < <(find "$SRC/static" -type f -print0)
}

sync_build_tree "$COLCON"
sync_build_tree "$INSTALL"

echo "[sync_frontend_build] done → $(grep -o 'main\.[a-z0-9]*\.js' "$INSTALL/index.html")"
