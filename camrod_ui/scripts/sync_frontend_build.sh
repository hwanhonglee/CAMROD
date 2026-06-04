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

# src와 colcon은 하드링크로 연결되어 있으므로 inode가 다를 때만 복사
if [ "$(stat -c %i "$SRC/index.html")" != "$(stat -c %i "$COLCON/index.html" 2>/dev/null)" ]; then
  cp "$SRC/index.html" "$COLCON/index.html"
fi

# install 경로는 항상 덮어쓰기 (별도 inode)
cp "$SRC/index.html" "$INSTALL/index.html"

# static 파일 동기화 (same-file 오류 무시)
cp -r "$SRC/static/." "$COLCON/static/" 2>/dev/null || true
cp -r "$SRC/static/." "$INSTALL/static/"

echo "[sync_frontend_build] done → $(grep -o 'main\.[a-z0-9]*\.js' "$INSTALL/index.html")"
