#!/bin/bash
# Syncs React build output to colcon build/install paths after npm run build.
# HH_261002 - Called after a staged build succeeds; standalone use remains valid.

set -euo pipefail

FRONTEND_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../camrod_ui_robot/assets/frontend" && pwd)"
WS_ROOT="$(realpath "$FRONTEND_DIR/../../../../..")"

SRC="${CAMROD_FRONTEND_BUILD_SOURCE:-$FRONTEND_DIR/build}"
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
    # HH_261002 - Replace complete assets atomically, including old install
    # symlinks, so concurrent image requests cannot read half-written bytes.
    local temporary_asset
    temporary_asset="$(mktemp "$(dirname "$dst_file")/.asset-publish.XXXXXX")"
    cp "$src_file" "$temporary_asset"
    chmod --reference="$src_file" "$temporary_asset"
    mv -f "$temporary_asset" "$dst_file"
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

  # HH_261002 - Retain previous content-hashed bundles for already-open tabs.
  # Pruning belongs to offline maintenance, not live publication.
}

if [[ ! -s "$SRC/index.html" || ! -d "$SRC/static" ]]; then
  echo "[sync_frontend_build] refusing incomplete frontend output: $SRC" >&2
  exit 1
fi

# HH_261002 - A staged build updates the source build as another publication
# target; failed compilation never reaches this point or touches served files.
if [[ "$(realpath "$SRC")" != "$(realpath -m "$FRONTEND_DIR/build")" ]]; then
  mkdir -p "$FRONTEND_DIR/build"
  sync_build_tree "$FRONTEND_DIR/build"
fi

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
