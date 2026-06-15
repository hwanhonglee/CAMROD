#!/usr/bin/env bash
# setup_deps.sh — CAMROD DOCKING 외부 의존성 설치 스크립트
#
# 기능:
#   1. vcs (python3-vcstool) 사용해 git 의존성 클론
#   2. NVIDIA Isaac ROS apt 패키지 설치
#   3. rosdep로 표준 ROS 의존성 설치 (nav2_msgs, vision_msgs 등)
#
# 사용법:
#   ./setup_deps.sh                     # 기본 (shallow 클론 + Isaac ROS apt + rosdep)
#   ./setup_deps.sh --full              # full 클론 (git history 포함)
#   ./setup_deps.sh --update            # 이미 클론된 패키지 업데이트
#   ./setup_deps.sh --skip-apt          # Isaac ROS apt 설치 건너뜀
#   ./setup_deps.sh --skip-rosdep       # rosdep 설치 건너뜀
#
# 최초 실행 후 버전 고정:
#   vcs export src/external --exact > deps.repos.lock

set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOS_FILE="${WORKSPACE_ROOT}/deps.repos"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*" >&2; }
step()  { echo -e "${CYAN}[STEP]${NC} $*"; }

SHALLOW=true
UPDATE=false
SKIP_APT=false
SKIP_ROSDEP=false

for arg in "$@"; do
  case "$arg" in
    --full)         SHALLOW=false ;;
    --update)       UPDATE=true ;;
    --skip-apt)     SKIP_APT=true ;;
    --skip-rosdep)  SKIP_ROSDEP=true ;;
    --help|-h)
      echo "Usage: $0 [--full] [--update] [--skip-apt] [--skip-rosdep]"
      echo "  --full          git history 포함 full 클론 (기본: shallow)"
      echo "  --update        이미 클론된 패키지 git pull 업데이트"
      echo "  --skip-apt      Isaac ROS apt 패키지 설치 건너뜀"
      echo "  --skip-rosdep   rosdep 의존성 설치 건너뜀"
      exit 0
      ;;
  esac
done

if [[ ! -f "$REPOS_FILE" ]]; then
  error "deps.repos 파일을 찾을 수 없습니다: $REPOS_FILE"
  exit 1
fi

cd "$WORKSPACE_ROOT"

# ─────────────────────────────────────────────────────────────────────────────
# 1단계: git 의존성 클론 (vcs 우선, Python3 fallback)
# ─────────────────────────────────────────────────────────────────────────────
clone_deps_vcs() {
  step "vcs로 의존성 임포트 중..."

  local vcs_args=()
  [[ "$SHALLOW" == true ]] && vcs_args+=(--shallow)

  if [[ "$UPDATE" == true ]]; then
    vcs pull .
  else
    vcs import "${vcs_args[@]}" . < "$REPOS_FILE"
  fi

  info "git 의존성 준비 완료."
  info "현재 커밋 SHA로 버전 고정하려면: vcs export external --exact > deps.repos.lock"
}

clone_deps_python() {
  warn "vcs 설치 불가. Python3 fallback으로 진행합니다."

  python3 - "$WORKSPACE_ROOT" "$REPOS_FILE" "$SHALLOW" "$UPDATE" <<'PYEOF'
import sys
import subprocess
from pathlib import Path

workspace   = Path(sys.argv[1])
repos_file  = Path(sys.argv[2])
shallow     = sys.argv[3] == "true"
update_mode = sys.argv[4] == "true"

try:
    import yaml
except ImportError:
    print("[INFO] pyyaml 설치 중...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "--quiet", "pyyaml"])
    import yaml

with open(repos_file) as f:
    repos = yaml.safe_load(f).get("repositories", {})

errors = []
skipped = []

for rel_path, info in repos.items():
    target  = workspace / rel_path
    url     = info["url"]
    version = info.get("version", "main")

    if target.exists():
        if update_mode:
            print(f"[UPDATE] {rel_path}")
            result = subprocess.run(["git", "-C", str(target), "pull"], capture_output=True, text=True)
            if result.returncode != 0:
                print(f"  WARN: {result.stderr.strip()}")
        else:
            skipped.append(rel_path)
        continue

    target.parent.mkdir(parents=True, exist_ok=True)

    clone_cmd = ["git", "clone"]
    if shallow:
        clone_cmd += ["--depth", "1"]
    clone_cmd += ["-b", version, url, str(target)]

    print(f"[CLONE] {rel_path} @ {version}" + (" (shallow)" if shallow else ""))
    result = subprocess.run(clone_cmd, capture_output=True, text=True)

    if result.returncode != 0:
        print(f"  ERROR: {result.stderr.strip()}", file=sys.stderr)
        errors.append(rel_path)
    else:
        print(f"  OK")

if skipped:
    print(f"\n[SKIP] 이미 존재하는 패키지 ({len(skipped)}개) — 업데이트하려면 --update 옵션 사용:")
    for s in skipped:
        print(f"  {s}")

if errors:
    print(f"\n[FAIL] 실패한 패키지:", file=sys.stderr)
    for e in errors:
        print(f"  {e}", file=sys.stderr)
    sys.exit(1)

print("\n[DONE] 모든 의존성 패키지 준비 완료.")
PYEOF
}

if command -v vcs &>/dev/null; then
  info "vcs 감지됨: $(vcs --version 2>&1 | head -1)"
  clone_deps_vcs
elif sudo apt-get install -y python3-vcstool 2>/dev/null && command -v vcs &>/dev/null; then
  info "python3-vcstool 설치 완료."
  clone_deps_vcs
else
  clone_deps_python
fi

# ─────────────────────────────────────────────────────────────────────────────
# 2단계: Isaac ROS apt 패키지 설치
# ─────────────────────────────────────────────────────────────────────────────
install_isaac_ros_apt() {
  step "Isaac ROS apt 패키지 설치 중..."

  if ! apt-cache show ros-humble-isaac-ros-apriltag &>/dev/null; then
    info "NVIDIA Isaac ROS apt 저장소 등록 중..."

    sudo apt-get install -y curl gnupg lsb-release

    curl -sSL https://isaac.download.nvidia.com/isaac-ros/repos.key -o /tmp/isaac-ros.key
    gpg --dearmor < /tmp/isaac-ros.key > /tmp/isaac-ros-keyring.gpg
    sudo cp /tmp/isaac-ros-keyring.gpg /usr/share/keyrings/isaac-ros-archive-keyring.gpg

    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/isaac-ros-archive-keyring.gpg] \
https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" \
      > /tmp/isaac-ros.list
    sudo cp /tmp/isaac-ros.list /etc/apt/sources.list.d/isaac-ros.list

    sudo apt-get update
  fi

  sudo apt-get install -y \
    ros-humble-isaac-ros-apriltag \
    ros-humble-isaac-ros-image-proc \
    ros-humble-isaac-ros-nitros

  info "Isaac ROS apt 패키지 설치 완료."
}

if [[ "$SKIP_APT" == "true" ]]; then
  warn "Isaac ROS apt 설치 건너뜀 (--skip-apt)"
else
  install_isaac_ros_apt
fi

# ─────────────────────────────────────────────────────────────────────────────
# 3단계: rosdep — 표준 ROS 의존성 설치
#   Isaac ROS apt 저장소 등록 이후 실행해야 isaac_ros_* 키도 해석 가능
# ─────────────────────────────────────────────────────────────────────────────
install_rosdeps() {
  step "rosdep로 ROS 의존성 설치 중..."

  if ! command -v rosdep &>/dev/null; then
    warn "rosdep 없음 — 건너뜀 (sudo apt-get install python3-rosdep)"
    return 0
  fi

  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init 2>/dev/null || true
    rosdep update 2>/dev/null || true
  fi

  if ! rosdep install --from-paths src --ignore-src -r -y; then
    warn "rosdep 일부 의존성 미해결 — 빌드 시 오류 발생 가능"
  fi

  info "rosdep 설치 완료."
}

if [[ "$SKIP_ROSDEP" == "true" ]]; then
  warn "rosdep 설치 건너뜀 (--skip-rosdep)"
else
  install_rosdeps
fi
