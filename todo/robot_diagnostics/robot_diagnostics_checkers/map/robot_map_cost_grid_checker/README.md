# robot_map_cost_grid_checker

ROS 2 패키지로, `lanelet_cost_grid_node`가 발행하는 `/map/cost_grid/lanelet`을 구독해
**Planning에 공급되는 레인 제약 격자의 상태**를 `/diagnostics` 토픽으로 발행합니다.

> **핵심 감시 포인트**: `lanelet_cost_grid_node`는 맵 파일 로드 실패 시 노드가 살아있지만
> 토픽을 **발행하지 않습니다(silent failure)**. 이 체커는 그 침묵을 STALE로 드러냅니다.

---

## 목차

1. [어떻게 동작하나요?](#1-어떻게-동작하나요)
2. [패키지 구성](#2-패키지-구성)
3. [의존성](#3-의존성)
4. [빌드 방법](#4-빌드-방법)
5. [실행 방법](#5-실행-방법)
6. [파라미터 설명](#6-파라미터-설명)
7. [진단 결과 확인하기](#7-진단-결과-확인하기)
8. [진단 상태 레벨이란?](#8-진단-상태-레벨이란)
9. [미구현 / 우선순위 낮은 항목](#9-미구현--우선순위-낮은-항목)
10. [자주 묻는 질문 (FAQ)](#10-자주-묻는-질문-faq)

---

## 1. 어떻게 동작하나요?

```
map.osm ──► lanelet_cost_grid_node
                  ▲  ▲
                  │  └── /planning/global_path
                  └───── /localization/pose
                  │
          정상: /map/cost_grid/lanelet 발행 (OccupancyGrid)
          실패: 발행 중단 (silent failure)
                  │
          map_cost_grid_checker_node ──► /diagnostics
                  ▲
            1초마다 체크
```

### 언제 STALE이 되는가?

| 원인 | 설명 |
|------|------|
| 맵 파일 없음 / OSM 파싱 실패 | 노드가 살아있어도 토픽을 발행하지 않음 |
| `/localization/pose` 미수신 | 격자 재생성 트리거 없음 |
| `/planning/global_path` 미수신 | `allow_build_without_path=false` 설정 시 발행 중단 |
| 경로 timeout 초과 | `stale_path_timeout_sec` 초과 시 격자 클리어 |

### unknown 비율이 높아지는 경우

| 상황 | 의미 |
|------|------|
| > 80% (`unknown_ratio_warn`) | 경로 없거나 맵 커버리지 밖 |
| > 98% (`unknown_ratio_error`) | 사실상 빈 격자 — 맵 로드 실패 가능성 |

---

## 2. 패키지 구성

```
robot_map_cost_grid_checker/
├── src/
│   ├── map_cost_grid_checker_node.cpp       # 메인 노드 코드
│   └── map_cost_grid_dummy_publisher.cpp    # 테스트용 더미 퍼블리셔
├── config/
│   └── map_cost_grid_checker.yaml           # 설정 파일
├── launch/
│   ├── map_cost_grid_checker.launch.py      # 운영용
│   └── map_cost_grid_test.launch.py         # 테스트용
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 기본 라이브러리 |
| `nav_msgs` | `OccupancyGrid` 메시지 타입 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | BaseChecker 공통 클래스 |

---

## 4. 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_map_cost_grid_checker
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 운영용
ros2 launch robot_map_cost_grid_checker map_cost_grid_checker.launch.py

# 테스트용
ros2 launch robot_map_cost_grid_checker map_cost_grid_test.launch.py
ros2 launch robot_map_cost_grid_checker map_cost_grid_test.launch.py scenario:=stale
ros2 launch robot_map_cost_grid_checker map_cost_grid_test.launch.py scenario:=high_unknown
ros2 launch robot_map_cost_grid_checker map_cost_grid_test.launch.py scenario:=full_unknown
```

### 전체 시나리오 목록

| 시나리오 | 동작 | 예상 결과 |
|----------|------|-----------|
| `ok` | 정상 격자 발행 (unknown 10%, 1Hz) | OK |
| `hz_warn` | 발행 속도 저하 (expected × 0.25) | WARN |
| `hz_error` | 발행 속도 심각 저하 (expected × 0.05) | ERROR |
| `stale` | 퍼블리시 중단 (맵 로드 실패 시뮬레이션) | STALE |
| `high_unknown` | unknown 85% (경로 없거나 맵 커버리지 밖) | WARN |
| `full_unknown` | unknown 100% (격자 완전 무효) | ERROR |

---

## 6. 파라미터 설명

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 발행 주기 (Hz) |
| `cost_grid_topic` | `/map/cost_grid/lanelet` | 감시할 토픽 (transient_local QoS) |
| `expected_hz` | `1.0` | 기대 발행 주파수 (이벤트 기반이므로 낮게 설정) |
| `hz_warn_ratio` | `0.4` | actual/expected < 이 값 → WARN |
| `hz_error_ratio` | `0.1` | actual/expected < 이 값 → ERROR |
| `stale_timeout` | `3.0` | 이 시간(초) 이상 메시지 없으면 STALE |
| `unknown_ratio_warn` | `0.8` | unknown 셀 비율 > 이 값 → WARN |
| `unknown_ratio_error` | `0.98` | unknown 셀 비율 > 이 값 → ERROR |

> `expected_hz`가 낮은 이유: `lanelet_cost_grid_node`는 고정 주기가 아닌
> pose/path 콜백 이벤트 시 격자를 재생성합니다.
> 로봇이 정지해 있거나 경로가 없으면 발행 빈도가 낮아집니다.

---

## 7. 진단 결과 확인하기

```bash
ros2 topic echo /diagnostics | grep -A 15 "/map/cost_grid"
```

### 출력 예시 (정상)

```
name:    "/map/cost_grid"
level:   0                          ← OK
message: "OK (1.0 Hz, unknown=10%)"
values:
  - key: "actual_hz"            value: "1.00"
  - key: "expected_hz"          value: "1.0"
  - key: "unknown_ratio_pct"    value: "10.0"
  - key: "grid_size"            value: "400 x 400"
  - key: "grid_resolution_m"    value: "0.50"
  - key: "last_msg_sec_ago"     value: "0.50"
```

### 출력 예시 (STALE — 맵 로드 실패)

```
name:    "/map/cost_grid"
level:   3                          ← STALE
message: "토픽 수신 없음: /map/cost_grid/lanelet (맵 로드 실패 또는 pose/path 입력 없음)"
```

### 출력 예시 (WARN — unknown 높음)

```
name:    "/map/cost_grid"
level:   1                          ← WARN
message: "레인 격자 unknown 높음 — 경로 없거나 맵 커버리지 밖"
values:
  - key: "unknown_ratio_pct"    value: "85.0"
```

---

## 8. 진단 상태 레벨이란?

| 레벨 | 숫자 | 색상 | 의미 |
|------|------|------|------|
| OK | 0 | 초록 | 레인 격자 정상 생성 중 |
| WARN | 1 | 노랑 | 격자 발행 저하 또는 unknown 비율 높음 |
| ERROR | 2 | 빨강 | 격자 사실상 무효 (unknown 98% 이상) |
| STALE | 3 | 회색 | 맵 로드 실패 또는 입력 토픽 없음 |

---

## 9. 미구현 / 우선순위 낮은 항목

Map 컴포넌트에서 현재 구현하지 않은 항목입니다.

### 구현하지 않은 이유

| 토픽 | 노드 | 이유 |
|------|------|------|
| `/map/markers` | `lanelet2_map_node` | `visualization_msgs/MarkerArray` — RViz 시각화 전용, 기능적 진단 가치 없음 |
| `/map/cost_grid/lanelet_field_markers` | `cost_field_node` | visualization only, Planning에 직접 영향 없음 |
| `/map/cost_grid/inflation_markers` | `cost_field_marker_node` | visualization adapter, 내부에 staleness 자체 처리 있음 |
| `/map/cost_grid/contributor_markers` | `marker_array_aggregator_node` | visualization 집계 노드 |
| `/planning/global_costmap/costmap` | Nav2 | Nav2 내부 costmap — Planning 도메인 별도 구현 필요 |

### 향후 추가 가능 항목

| 항목 | 설명 | 조건 |
|------|------|------|
| `robot_map_lanelet_node_checker` | `lanelet2_map_node` 노드 생존 감시 | heartbeat 토픽 추가 또는 rosnode 기반 구현 필요 |
| `robot_planning_path_checker` | `/planning/global_path` 발행 상태 | Planning 도메인 구현 시 |
| `robot_planning_costmap_checker` | Nav2 global/local costmap 상태 | Nav2 도메인 구현 시 |

---

## 10. 자주 묻는 질문 (FAQ)

**Q. STALE이 됐는데 `lanelet_cost_grid_node`는 살아있습니다. 왜 그런가요?**

`lanelet_cost_grid_node`는 맵 파일 로드 실패 시 노드 프로세스가 살아있지만
토픽을 발행하지 않습니다. `map_path` 파라미터와 파일 존재 여부를 확인하세요.

---

**Q. 로봇이 정지해 있으면 항상 WARN이 되나요?**

`expected_hz`가 1.0Hz이고 로봇이 정지 중이어도 `lanelet_cost_grid_node`의
`republish_period_sec` 설정에 따라 주기적으로 재발행됩니다.
WARN이 된다면 `republish_period_sec`을 확인하세요.

---

**Q. unknown 비율이 항상 80% 이상입니다.**

아래 순서로 확인하세요:
1. `/planning/global_path` 토픽이 발행 중인지 확인
2. `lanelet_cost_grid_node`의 `allow_build_without_path` 파라미터 확인
3. 로봇 위치가 맵 경계(`max_search_radius`) 안에 있는지 확인
