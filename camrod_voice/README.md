# camrod_voice

CAMROD 음성 안내 모듈. 사전 녹음된 한국어 WAV 파일을 우선순위 큐 기반으로 재생한다.

## 패키지 구조

```
camrod_voice/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── voice_event_adapter.yaml        # TODO: 생성 필요
├── include/voice_announcer/
│   ├── audio_player.hpp
│   ├── priority_queue.hpp
│   └── voice_announcer_node.hpp        # avg_msgs 기반
├── launch/
│   └── voice.launch.py                 # TODO: 생성 필요
├── resource/audio/ko-KR/               # 23개 WAV 파일
│   ├── battery/   (low, critical, charging, full)
│   ├── docking/   (started, approaching, succeeded, failed, cancelled)
│   ├── navigation/(to_campsite, arrived_campsite, to_dropzone, return_to_dropzone, please_step_aside)
│   ├── safety/    (estop, estop_released, obstacle)
│   ├── system/    (startup, ready, shutdown)
│   └── undocking/ (started, succeeded, failed)
└── src/
    ├── audio_player.cpp
    ├── main.cpp
    ├── priority_queue.cpp
    ├── voice_announcer_node.cpp        # avg_msgs 기반
    └── voice_event_adapter_node.py     # TODO: 구현 필요 (현재 placeholder)
```

## 인터페이스

| 방향 | 토픽 | 타입 |
|------|------|------|
| Subscribe | `~/say` | `avg_msgs/AudioRequest` |
| Publish   | `~/state` | `avg_msgs/VoiceState` |

**AudioRequest.key** 형식: `"카테고리.파일명"` → `resource/audio/{locale}/{카테고리}/{파일명}.wav`  
예) `"navigation.arrived_campsite"` → `ko-KR/navigation/arrived_campsite.wav`

**우선순위**: 0=info, 1=notice, 2=warning, 3=critical (3은 큐 클리어 후 즉시 재생)

## 빌드

```bash
colcon build --packages-select camrod_voice --cmake-args -DCMAKE_BUILD_TYPE=Release
```

의존 시스템 패키지: `libsdl2-dev`, `libsdl2-mixer-dev` (setup_camrod.sh에 이미 포함)

---

## TODO

### 1. voice_event_adapter_node.py 구현

`src/voice_event_adapter_node.py` (현재 placeholder) — CAMROD 시스템 이벤트를 AudioRequest로 변환하는 Python 노드.

**구독 토픽:**

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/planning/state_machine/state` | `std_msgs/String` | 내비게이션 상태 전환 감지 |
| `/platform/status/estop` | `std_msgs/Bool` | 비상정지 engage/release |
| `/battery_percentage` | `std_msgs/Int32` | 배터리 임계값 경고 |
| `/docking/is_charging` | `std_msgs/Bool` | 도킹/충전 감지 |

**상태머신 → 오디오 키 매핑 (edge-triggered):**

| 전환 | 키 | 우선순위 |
|------|----|----------|
| `→ READY` | `system.ready` | 1 |
| `→ GOAL_REACHED` | `navigation.arrived_campsite` | 1 |
| `→ RETURNING` / `WAIT_DZ` | `navigation.return_to_dropzone` | 1 |
| `→ WARN_RECOVERY` | `safety.obstacle` | 2 |
| `→ RUNNING` (from WAIT_DZ) | `undocking.started` | 1 |
| 노드 시작 후 3초 | `system.startup` | 0 |

**기타 이벤트:**

| 이벤트 | 키 | 우선순위 |
|--------|----|----------|
| estop `false→true` | `safety.estop` | 3 |
| estop `true→false` | `safety.estop_released` | 3 |
| 배터리 ≤ 20% | `battery.low` | 1 |
| 배터리 ≤ 10% | `battery.critical` | 2 |
| is_charging `false→true` | `docking.succeeded` | 1 |

---

### 2. voice.launch.py 생성

`launch/voice.launch.py` — `voice` 네임스페이스 아래 두 노드를 함께 기동.

```python
# 기동 노드:
# - voice_announcer_node  (C++ component)
# - voice_event_adapter_node  (Python, enable_voice_adapter 조건부)
#
# 파라미터:
#   locale             (default: "ko-KR")
#   voice_namespace    (default: "voice")
#   enable_voice_adapter (default: true)
```

---

### 3. voice_event_adapter.yaml 생성

`config/voice_event_adapter.yaml` — 이벤트 어댑터 파라미터.

```yaml
/voice/voice_event_adapter:
  ros__parameters:
    startup_delay_s: 3.0
    enable_nav_audio: true
    enable_estop_audio: true
    enable_battery_audio: true
    enable_docking_audio: true
    battery_low_threshold: 20
    battery_critical_threshold: 10
```

---

### 4. bringup 통합

**`camrod_bringup/launch/_bringup_impl.py`** 수정 3곳:

1. `arg_specs` 리스트에 추가 (docking 항목 뒤):
```python
('enable_voice',         cfg_get(launch_cfg, 'voice/enable_voice',         True),  'Enable voice announcer module'),
('enable_voice_adapter', cfg_get(launch_cfg, 'voice/enable_voice_adapter', True),  'Enable voice event adapter node'),
('voice_namespace',      cfg_get(launch_cfg, 'namespaces/voice',           'voice'), 'Voice module namespace'),
```

2. `voice_args` dict 추가 (docking_args 블록 뒤):
```python
voice_args = {
    'voice_namespace': lc['voice_namespace'],
    'enable_voice_adapter': lc['enable_voice_adapter'],
}
```

3. `module_specs` 리스트에 추가 (docking 뒤, system 앞):
```python
('camrod_voice', 'voice.launch.py', voice_args, IfCondition(lc['enable_voice'])),
```

---

### 5. bringup config 수정

**`camrod_bringup/config/bringup/launch_defaults.yaml`** — `voice:` 섹션 추가:
```yaml
  voice:
    enable_voice: true
    enable_voice_adapter: true
```

**`camrod_bringup/config/bringup/cleanup_patterns.yaml`** — voice 노드 패턴 추가:
```yaml
  # [voice]
  - __node:=voice_announcer
  - __node:=voice_event_adapter
```
