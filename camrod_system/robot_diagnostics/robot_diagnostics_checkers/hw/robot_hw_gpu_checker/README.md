# hw_gpu_checker

ROS 2 패키지로, 로봇 컴퓨터의 **CPU / 메모리 / 디스크 / CPU 온도 / GPU** 상태를
주기적으로 수집해 `/diagnostics` 토픽으로 발행합니다.

**컨테이너(Docker) 배포 시에도 호스트 리소스를 올바르게 감시**할 수 있도록
컨테이너 환경 자동 감지 및 호스트 마운트 경로 지원이 내장되어 있습니다.

---

## 목차

1. [패키지 구성](#1-패키지-구성)
2. [의존성](#2-의존성)
3. [빌드 방법](#3-빌드-방법)
4. [실행 방법](#4-실행-방법)
5. [발행 토픽](#5-발행-토픽)
6. [파라미터 설정](#6-파라미터-설정)
7. [컨테이너(Docker) 환경에서 사용하기](#7-컨테이너docker-환경에서-사용하기)
   - [Docker 이미지 빌드](#docker-이미지-빌드)
   - [NVIDIA GPU + 호스트 디바이스 마운트 테스트](#nvidia-gpu--호스트-디바이스-마운트-테스트)
8. [진단 상태 확인 방법](#8-진단-상태-확인-방법)
9. [자주 묻는 질문 (FAQ)](#9-자주-묻는-질문-faq)

---

## 1. 패키지 구성

```
hw_gpu_checker/
├── src/
│   ├── hw_checker_node.cpp   # CPU / 메모리 / 디스크 / CPU 온도 감시
│   └── gpu_checker_node.cpp  # NVIDIA GPU 감시 (nvidia-smi 사용)
├── config/
│   └── hw_gpu_checker.yaml   # 임계값 및 경로 설정
├── launch/
│   └── hw_gpu_checker.launch.py  # 두 노드를 한 번에 실행
└── README.md
```

---

## 2. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 클라이언트 라이브러리 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | 베이스 체커 클래스 (워크스페이스 내 패키지) |

GPU 감시 기능을 사용하려면 호스트에 **NVIDIA 드라이버**와 **nvidia-smi**가
설치되어 있어야 합니다.

---

## 3. 빌드 방법

```bash
# 워크스페이스 루트로 이동
cd ~/ros2_ws

# 빌드 (이 패키지만 빌드)
colcon build --packages-select hw_gpu_checker

# 빌드 결과를 현재 쉘에 적용
source install/setup.bash
```

> **빌드가 처음이라면?** 의존 패키지도 함께 빌드해야 합니다.
> ```bash
> colcon build
> source install/setup.bash
> ```

---

## 4. 실행 방법

### launch 파일로 실행 (권장)

hw_checker + gpu_checker 두 노드를 동시에 시작합니다.

```bash
ros2 launch hw_gpu_checker hw_gpu_checker.launch.py
```

### 노드를 개별 실행

```bash
# HW 체커만 실행
ros2 run hw_gpu_checker hw_checker_node

# GPU 체커만 실행
ros2 run hw_gpu_checker gpu_checker_node
```

---

## 5. 발행 토픽

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 모든 진단 결과 묶음 |

`/diagnostics` 안에 포함되는 항목(name):

| name | 내용 |
|------|------|
| `/hardware/cpu` | CPU 사용률 (%) |
| `/hardware/memory` | 메모리 사용률 (%) |
| `/hardware/disk` | 디스크 사용률 (%) |
| `/hardware/cpu_temp` | CPU 온도 (°C) |
| `/hardware/gpu0` | GPU 0번 상태 (util / VRAM / 온도) |
| `/hardware/gpu1` | GPU 1번 (2개 이상일 때 추가됨) |

각 항목의 상태 레벨:
- **OK** (초록): 정상
- **WARN** (노랑): 임계값 초과 주의
- **ERROR** (빨강): 임계값 초과 위험
- **STALE** (회색): 센서 없음 또는 읽기 실패

---

## 6. 파라미터 설정

설정 파일: `config/hw_gpu_checker.yaml`

### hw_checker 파라미터

```yaml
hw_checker:
  ros__parameters:
    publish_rate: 1.0           # 발행 주기 (Hz) — 1.0 = 1초마다

    cpu:
      warn_threshold: 70.0      # CPU 사용률 70% 이상이면 WARN
      error_threshold: 90.0     # CPU 사용률 90% 이상이면 ERROR

    memory:
      warn_threshold: 75.0      # 메모리 사용률 75% 이상이면 WARN
      error_threshold: 90.0     # 메모리 사용률 90% 이상이면 ERROR

    disk:
      warn_threshold: 80.0      # 디스크 사용률 80% 이상이면 WARN
      error_threshold: 95.0     # 디스크 사용률 95% 이상이면 ERROR
      path: "/"                 # 감시할 디렉터리 경로

    cpu_temp:
      warn_threshold: 75.0      # CPU 온도 75°C 이상이면 WARN
      error_threshold: 90.0     # CPU 온도 90°C 이상이면 ERROR
```

### gpu_checker 파라미터

```yaml
gpu_checker:
  ros__parameters:
    publish_rate: 1.0

    gpu:
      util_warn_threshold: 85.0   # GPU 사용률 85% 이상이면 WARN
      util_error_threshold: 95.0
      mem_warn_threshold: 80.0    # VRAM 사용률 80% 이상이면 WARN
      mem_error_threshold: 95.0
      temp_warn_threshold: 75.0   # GPU 온도 75°C 이상이면 WARN
      temp_error_threshold: 85.0
```

### 실행 시 파라미터 덮어쓰기

설정 파일을 수정하지 않고 실행 시점에 값을 바꿀 수 있습니다.

```bash
ros2 run hw_gpu_checker hw_checker_node \
  --ros-args -p cpu.warn_threshold:=60.0 -p disk.path:=/data
```

---

## 7. 컨테이너(Docker) 환경에서 사용하기

컨테이너 내부에서 `/`를 감시하면 실제 호스트 디스크가 아닌
**컨테이너 overlay 파일시스템**을 보게 됩니다.
또한 `/sys/class/thermal`이 마운트되지 않아 CPU 온도를 읽지 못할 수 있습니다.

이를 해결하는 방법을 소개합니다.

### Docker 이미지 빌드

패키지 내 `Dockerfile`을 사용합니다. 빌드 컨텍스트는 `src/` 디렉토리여야 합니다.

```bash
cd ~/ros2_ws/src
docker build \
  -f robot_diagnostics_checkers/hw/robot_hw_gpu_checker/Dockerfile \
  -t robot_hw_gpu_checker:test \
  .
```

### NVIDIA GPU + 호스트 디바이스 마운트 테스트

NVIDIA Container Toolkit이 설치된 환경에서 GPU 접근과 호스트 리소스를 동시에 테스트합니다.

```bash
docker run --rm \
  --gpus all \
  -v /proc:/host/proc:ro \
  -v /sys:/host/sys:ro \
  -v /:/host/rootfs:ro \
  robot_hw_gpu_checker:test
```

| 옵션 | 설명 |
|------|------|
| `--gpus all` | 모든 NVIDIA GPU를 컨테이너에 노출 |
| `-v /proc:/host/proc:ro` | 호스트 CPU / 메모리 정보 읽기 |
| `-v /sys:/host/sys:ro` | 호스트 CPU 온도 센서 읽기 |
| `-v /:/host/rootfs:ro` | 호스트 디스크 사용량 읽기 |

> **NVIDIA Container Toolkit 설치:**
> ```bash
> curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
> curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
>   sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
>   sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
> sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
> sudo nvidia-ctk runtime configure --runtime=docker
> sudo systemctl restart docker
> ```

### 방법 A: 호스트 경로를 컨테이너에 마운트 (권장)

Docker 실행 시 호스트의 `/proc`, `/sys`, `/`를 컨테이너 안으로 마운트합니다.

```bash
docker run \
  -v /proc:/host/proc:ro \
  -v /sys:/host/sys:ro   \
  -v /:/host/rootfs:ro   \
  <이미지_이름>
```

노드가 시작되면 `/host/proc`, `/host/sys`, `/host/rootfs`를 **자동으로 감지**해
호스트 리소스를 읽습니다. 별도 파라미터 설정이 필요 없습니다.

### 방법 B: 파라미터로 경로 지정

자동 감지 경로가 아닌 다른 경로를 사용한다면 파라미터로 직접 지정합니다.

```yaml
hw_checker:
  ros__parameters:
    container:
      host_proc_path: "/mnt/host/proc"    # 호스트 /proc 마운트 위치
      host_sys_path: "/mnt/host/sys"      # 호스트 /sys 마운트 위치
      host_rootfs_path: "/mnt/host"       # 호스트 루트 FS 마운트 위치
```

### 방법 C: 환경 변수로 경로 지정

파라미터보다 환경 변수가 우선 적용됩니다.

```bash
docker run \
  -e HOST_PROC=/host/proc \
  -e HOST_SYS=/host/sys   \
  -e HOST_ROOTFS=/host/rootfs \
  -v /proc:/host/proc:ro  \
  -v /sys:/host/sys:ro    \
  -v /:/host/rootfs:ro    \
  <이미지_이름>
```

### 우선순위 요약

```
환경 변수 > 파라미터 > 자동 감지(/host/proc, /host/sys ...) > 기본값(/proc, /sys)
```

### GPU (컨테이너)

**NVIDIA Container Toolkit**을 사용하는 환경에서는 별도 설정 없이 동작합니다.

```bash
# nvidia-docker2 또는 --gpus 플래그 사용
docker run --gpus all <이미지_이름>
```

Toolkit 없이 GPU 디바이스만 마운트된 경우:

```bash
docker run \
  -v /usr/bin/nvidia-smi:/host/usr/bin/nvidia-smi \
  <이미지_이름>
```

또는 파라미터/환경 변수로 명시:

```bash
docker run -e NVIDIA_SMI_PATH=/usr/local/bin/nvidia-smi <이미지_이름>
```

### 컨테이너 환경의 추가 진단 정보

컨테이너 환경이 감지되면 `/diagnostics` 에 아래 항목이 추가됩니다.

| key | 설명 |
|-----|------|
| `cgroup_cpu_quota_vcpus` | 컨테이너에 할당된 가상 CPU 코어 수 |
| `cgroup_mem_limit_MB` | 컨테이너 메모리 제한 (MB) |
| `cgroup_usage_%` | 컨테이너 메모리 제한 기준 사용률 |
| `proc_source` | 실제로 읽은 `/proc` 경로 |

---

## 8. 진단 상태 확인 방법

### 터미널에서 실시간 확인

```bash
# 원시 메시지 출력
ros2 topic echo /diagnostics

# rqt_runtime_monitor GUI (권장)
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

### 특정 항목만 필터링

```bash
# CPU 사용률만 출력 (grep 활용)
ros2 topic echo /diagnostics | grep -A 5 "hardware/cpu"
```

### 주요 필드 설명

```
name:   "/hardware/cpu"      ← 항목 이름
level:  0                    ← 0=OK, 1=WARN, 2=ERROR, 3=STALE
message: "CPU 45.2% "        ← 요약 메시지
values:
  - key: "usage_%"
    value: "45.2"            ← 상세 수치
```

---

## 9. 자주 묻는 질문 (FAQ)

**Q. CPU 온도가 항상 STALE로 나옵니다.**

WSL2 또는 일부 가상 환경에서는 `/sys/class/thermal`이 없어 온도를 읽을 수 없습니다. 정상 동작입니다. 네이티브 Linux 환경에서는 자동으로 감지됩니다.

---

**Q. GPU가 STALE로 나옵니다.**

`nvidia-smi` 명령을 터미널에서 직접 실행해 보세요.
```bash
nvidia-smi
```
동작하면 노드 실행 시 로그에서 탐색 경로를 확인하세요.
동작하지 않으면 NVIDIA 드라이버가 설치되지 않은 것입니다.

---

**Q. 컨테이너에서 Disk 사용량이 이상합니다.**

컨테이너의 `disk.path: "/"` 는 호스트 디스크가 아닌 overlay FS를 가리킵니다.
[방법 A](#방법-a-호스트-경로를-컨테이너에-마운트-권장)를 참고해 호스트 루트 FS를 마운트하세요.

---

**Q. 발행 주기를 바꾸고 싶습니다.**

`publish_rate` 파라미터(단위: Hz)를 변경하세요.
```yaml
publish_rate: 0.5   # 2초마다 발행
```

---

**Q. 특정 디렉터리의 디스크 사용량을 감시하고 싶습니다.**

`disk.path` 파라미터를 원하는 경로로 변경하세요.
```yaml
disk:
  path: "/data"   # /data 파티션 감시
```
