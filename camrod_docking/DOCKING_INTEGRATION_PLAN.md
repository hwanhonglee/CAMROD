# CAMROD 자율 도킹 통합 마스터 플랜

> Phase 1(Nav2 자율 주행) + Phase 2(AprilTag 시각 도킹) 연계부터 Planning State Machine 개편까지
> 기준 브랜치: `feature_YH_260625-docking-success-rate-tuning` · ROS 2 Humble

---

## 0. 큰 그림 — 무엇을, 왜, 어떤 순서로

### 0-1. 최종 목표

UI 버튼 하나 또는 배터리 임계값 하나로, 로봇이 **임의 위치에서 → Nav2로 도킹 스테이션 앞까지 자율 주행(Phase 1) → AprilTag 시각 도킹으로 밀착(Phase 2) → 충전 대기 → 완료 후 미션 복귀**까지 전 과정을 스스로 수행.

```
[현재]  👤 사람이 수동 배치  →  Phase 2 시각 도킹 ✓  →  밀착 완료
[목표]  임의 위치 → Phase 1(Nav2 자율주행) → Phase 2(AprilTag 시각도킹) → 충전 → 미션 복귀
        └───────────────── 전 과정 완전 자율 (사람 개입 0) ─────────────────┘
```

### 0-2. 왜 단계를 이렇게 나누는가 (전체 전략)

가장 큰 위험은 **"파이프라인이 되는지 검증"** 과 **"상태를 어떻게 관리할지 설계"** 를 동시에 하면 문제가 터졌을 때 원인을 못 가린다는 것.

- **Stage 1~3**: 하드웨어·설정 레벨에서 도킹 파이프라인이 **물리적으로 동작하는지만** 검증 (상태머신은 손대지 않음)
- **Stage 4~5**: 파이프라인이 확실해진 뒤, Planning State Machine이 도킹 생명주기를 **소유**하도록 개편

이 경계선(Stage 3 → 4)이 이 문서 전체의 척추.

---

## 1. 먼저 알아야 할 3가지 구조 (모든 설계의 전제)

### ① cmd_vel(속도 명령)이 흐르는 길이 Phase마다 완전히 다르다 — 가장 중요

```
Phase 1 (Nav2 스테이징):
  bt_navigator → controller_server → /planning/cmd_vel_raw
       → [cmd_vel_gate] → /planning/cmd_vel → 플랫폼 → 모터
                ↑
         engage=True 필수 (게이트 열려야 통과)

Phase 2 (AprilTag EgoPolar):
  docking_server(SmoothControlLaw) → /platform/cmd_vel (직접 발행!)
                                          ↑
                          cmd_vel_gate를 완전히 우회함
```

**여기서 파생되는 문제:** Phase 1은 게이트가 열려야(engage=True) 움직이고, Phase 2는 게이트를 우회하므로 이때 게이트가 열려있으면 Nav2 잔여 명령과 EgoPolar 명령이 **동시에** 플랫폼에 도달해 로봇이 예측 불가능하게 움직인다. → **Phase 전환 순간 engage를 정반대로 뒤집어야 한다.** 이것이 통합의 핵심 난제. (Stage 2에서 해결)

### ② opennav_docking은 Nav2를 "엉뚱한 주소"로 찾는다 (네임스페이스 하드코딩)

`navigator.cpp` 소스 확인 결과:

```cpp
// docking_server는 /docking 네임스페이스에서 실행됨
nav_to_pose_client_ = create_client<Nav2Pose>(..., "navigate_to_pose", ...);
// → 실제 탐색 경로: /docking/navigate_to_pose  ✗
// → CAMROD Nav2 서버 위치: /planning/navigate_to_pose  ✓
```

remapping 없이는 Phase 1에서 Nav2 서버를 못 찾음 → `FailedToStage` 예외로 즉시 실패. (Stage 1-3에서 해결)

### ③ 도킹은 bt_navigator를 거치지 않고 액션을 직접 부른다

현재 UI는 `manual_dock_server`를 통해 opennav_docking의 `DockRobot` 액션을 **직접** 호출.
`camrod_docking/config/bt/dock_robot.xml`은 존재하지만 어디에도 연결 안 됨 (미사용).

→ **Phase 1용 BT를 새로 만들 필요 없음.** bt_navigator가 이미 쓰는 기본 주행 BT(`navigate_to_pose_w_planner_selector.xml`, LaneletRoute+MPPI)를 `navigator_bt_xml: ""` 설정으로 그대로 재사용.

---

## 2. 현재 상태 정밀 진단

| 구성요소 | 현재 상태 | 통합 시 문제 | 조치 Stage |
|---------|----------|------------|-----------|
| `docking_server.yaml` `fixed_frame` | `"odom"` | ESKF 리셋 시 좌표 드리프트 | S1 |
| `docks.yaml` `frame`/`pose` | odom / `[1,0,0]` 임시 | 실제 스테이션 map 좌표 없음 | S1 |
| `docking.launch.py` navigate_to_pose remap | 없음 | Nav2 서버 못 찾음 | S1 |
| `manual_dock_server.yaml` `navigate_to_staging_pose` | `false` | Phase 1 비활성 | S1 |
| `docking_test.launch.py` | Nav2 미포함 | Phase 1 테스트 불가 | S1 |
| cmd_vel 경합 방지 (engage) | 없음 | Phase 전환 시 명령 충돌 🔴 | S2 |
| `odom_yaw_corrector` vs ESKF TF | 이중 발행 가능 | odom→base_link 충돌 | S2 |
| Planning SM 도킹 인식 | 전혀 모름 | 자동화/미션연동 불가 | S4 |
| `auto_dock_enabled` 소비처 | 없음 (발행만) | 자동 도킹 미동작 | S4 |
| `manual_dock_server` lifecycle | 미지원 | 크래시 복구 불가 | S5 |
| `bridge_params.yaml` `target_tag_id` | `3` 단일 고정 | 다중 스테이션 불가 | S5 |

---

## 3. 단계별 상세 설계 (TODO)

### STAGE 1 — Phase 1 인프라 활성화 🔴 최우선

> **목표:** Nav2가 로봇을 도킹 스테이션 앞 staging 위치까지 데려다 놓을 수 있게 만든다.
> **성격:** 설정 변경 + 필드 실측. 상태머신은 절대 건드리지 않음.

#### 1-1. fixed_frame 변경 (odom → map)
- **파일:** `camrod_docking/config/docking_server.yaml`
- **변경:** `fixed_frame: "odom"` → `"map"`
- **왜?** Phase 1은 Nav2가 `map` 프레임 기준으로 목표를 계산·추종. `odom`을 쓰면 GNSS 재수렴 등으로 ESKF 리셋 시 목표 좌표가 통째로 틀어짐.
- **⚠ 우려:** `map` 프레임은 ESKF(camrod_localization)가 발행 → **풀스택 실행 필수**. Phase 2 단독 테스트는 여전히 odom 필요할 수 있으니 **런치 인수로 전환 가능하게** 파라미터화 권장.

#### 1-2. docks.yaml 실측 좌표 등록 (필드 작업)
- **파일:** `camrod_docking/config/docks.yaml`
- **반드시 이해할 것:** 여기 넣는 `pose`는 **도킹 스테이션(AprilTag 마커) 자체의 위치**다. staging 위치가 아니다. opennav_docking이 이 좌표에서 `staging_x_offset(-0.7m)`만큼 자동으로 뒤로 빼서 Nav2 목표(staging)를 계산한다. (소스 `simple_charging_dock.cpp` `getStagingPose()` 검증 완료)

```
[staging pose]  ← 0.7m (staging_x_offset) →  [docks.yaml pose]
 🤖 Nav2 목표(자동계산)                          🏷️ 스테이션 위치(실측)
```

```yaml
docks:
  home_dock:
    type: apriltag_dock
    frame: map              # odom → map
    pose: [x, y, yaw]      # 스테이션의 map 좌표 (실측)
```

- **실측 방법:** 로봇을 스테이션에 정확히 밀착시킨 뒤 `/localization/pose`(map)를 읽어 접촉점 기준 역산.
- **병목 주의:** 실물 로봇 측정이 필요해 소프트웨어 작업과 병행 어려움. 전체에서 **가장 먼저 착수**해야 일정이 밀리지 않음.

#### 1-3. navigate_to_pose 액션 remapping 추가
- **파일:** `camrod_docking/launch/docking.launch.py`
- **왜?** §1-② 네임스페이스 하드코딩 우회 (없으면 Phase 1 100% 실패)

```python
Node(
  package='opennav_docking', executable='docking_server',
  name='docking_server', namespace=docking_ns,
  remappings=[('navigate_to_pose', '/planning/navigate_to_pose')],
  parameters=[cfg('docking_server.yaml'), ...],
)
```

#### 1-4. navigate_to_staging_pose 활성화
- **파일:** `camrod_docking/config/manual_dock_server.yaml`
- **변경:** `navigate_to_staging_pose: false` → `true`
- `manual_dock_server_node.cpp` 라인 129가 이미 이 파라미터를 DockRobot goal로 전달 중 → **코드 수정 불필요, yaml만 변경.** `navigator_bt_xml`은 `""`로 두면 기본 주행 BT 자동 사용(§1-③).

#### 1-5. Phase 1+2 풀스택 테스트 런치 작성
- **파일:** `camrod_bringup/launch/docking_full_test.launch.py` (신규)
- **포함 모듈:** `sensor_kit` → `localization`(map→odom) → `sensing` → **`planning`(Nav2, 기존 테스트엔 없던 것)** → `docking`(enable_odom_corrector=false)
- **추가 런치 인수:** `navigate_to_staging_pose`, `dock_id`
- **⚠ 우려:** `odom_yaw_corrector`가 켜지면 ESKF의 `odom→base_link`와 TF 충돌. 풀스택에서는 반드시 `enable_odom_corrector: false`.

**✓ Stage 1 완료 판정:** `ros2 action send_goal /docking/manual_dock ...` 실행 시 로봇이 **임의 위치에서 스테이션 앞 0.7m staging 위치까지 Nav2로 자율 주행**하면 성공.

---

### STAGE 2 — cmd_vel 경합 안전장치 🔴 최우선 · 안전

> **목표:** Phase 전환 순간 Nav2 명령과 EgoPolar 명령이 충돌하지 않게 한다.
> **성격:** Stage 1과 거의 동시 진행. 여기서 넣는 engage 제어는 **임시**이며 Stage 4에서 SM으로 이관.

#### 2-1. Phase별 engage 전환 로직 (임시 구현)
- **파일:** `camrod_docking/src/manual_dock_server_node.cpp`

```
engage 게이트 타임라인:
  PHASE_NAV_TO_STAGING ──────┐ TRUE (게이트 열림)
                             │
  ⚡전환점 ──────────────────┤
                             │
  INITIAL_PERCEPTION ────────┴─── CONTROLLING ─── WAIT_CHARGE
                                 FALSE (게이트 닫힘 · EgoPolar 직접제어)
```

```cpp
// Feedback relay 콜백 내부
switch (feedback.state) {
  case NAV_TO_STAGING_POSE: publishEngage(true);  break; // Phase1: 열림
  case INITIAL_PERCEPTION:  publishEngage(false); break; // Phase2 진입: 닫힘
  case CONTROLLING:         publishEngage(false); break;
  case WAIT_FOR_CHARGE:     publishEngage(false); break;
}
// TODO(Phase-SM): Stage 4 SM 재정의 후 planning_state_machine으로 이관
```

#### 2-2. /docking/is_docking 상태 토픽 발행
- `_execute()` 진입 시 `True`, 종료(성공/실패/취소) 시 `False`.
- **왜 지금?** Stage 4에서 SM이 "도킹 중"을 인지하고 외부 goal을 막기 위한 사전 배선.

#### 2-3. odom_yaw_corrector 비활성화 강제 (풀스택)
- 풀스택 런치에서 `enable_odom_corrector: false`. Phase 2 단독 테스트에서만 `true`. ESKF와 `odom→base_link` TF 이중 발행 방지.

**✓ Stage 2 완료 판정:** Phase 1→2 전환 순간 `ros2 topic echo /platform/cmd_vel` 관찰 시 두 소스 명령이 겹치지 않고 깔끔히 전환되면 성공.

---

### STAGE 3 — Phase 1+2 통합 실주행 검증 🟠 높음

> **목표:** 전 구간(임의위치→staging→밀착)을 반복 성공시키고 실패 케이스를 문서화.
> **성격:** 검증·튜닝 전용, 코드 변경 최소.

#### 3-1. Feedback relay 완성도 검증
- `PHASE_NAV_TO_STAGING(1)` feedback이 Phase 1 실주행 중 UI까지 정상 전달되는지, `phase_label / num_retries / docking_time` 정확한지 확인.

#### 3-2. Phase 전환 타임아웃·허용오차 튜닝
```yaml
# docking_server.yaml — 실주행 기반 재조정
dock_prestaging_tolerance: 0.5    # staging 도달 판정
initial_perception_timeout: 10.0  # Phase2 첫 태그검출 여유
dock_approach_timeout: 120.0      # Phase2 전체 접근
```
- **우려:** staging 도달 후 AprilTag가 즉시 안 잡히면 `initial_perception_timeout` 안에 재정렬 필요할 수 있음.

#### 3-3. 실패 케이스 카탈로그 작성
- staging 실패 / AprilTag 미검출 / EgoPolar 발산 / TF extrapolation 재발 등을 표로 기록.
- **이 실측 데이터가 Stage 4 SM의 상태 전환·재시도 정책의 근거가 된다.**

**✓ Stage 3 완료 판정:** 서로 다른 시작 위치 **10회 중 8회 이상** 전 구간 자율 도킹 성공 + 실패 모드 문서화 완료.

---

> ★ 여기서부터 상태머신 개편 ★ — Stage 1~3으로 파이프라인이 물리적으로 검증되었다. 이제부터 Planning State Machine이 도킹을 **소유**한다.

---

### STAGE 4 — Planning State Machine 재정의 / 확장 🟠 높음 · 핵심 개편

> **목표:** SM이 도킹 생명주기를 소유하고, 배터리 자동 트리거·미션 중단/재개·engage 단일 관리를 구현.
> **성격:** 대규모 개편. 검증된 파이프라인 위에서만.

```
확장된 Planning State Machine (신규 = DOCKING, CHARGING):

  ERROR_STOP(8) ← 진단 ERROR 시 어디서든

  INIT(0) → READY(1) ──미션키──→ RUNNING(3) → GOAL_REACHED(4) → RETURNING(6) → WAIT_DZ(2)
              │                      │
     수동/배터리낮음│              배터리임계│(미션 저장)
              ↓                      ↓
          DOCKING(9) ◄───────────────┘
              │
           성공│                  실패/취소 → READY(1)
              ↓
          CHARGING(10) ──충전완료──→ (저장 미션 있으면 RUNNING 복귀) → READY(1)
```

#### 4-A. 상태·시나리오 확장
- **파일:** `avg_msgs/msg/PlanningState.msg`, `PlanningScenario.msg`
- 상태 `DOCKING=9`, `CHARGING=10` / 시나리오 `DOCKING_TO_STATION=10`, `MISSION_PAUSED_DOCKING=11` 추가.
- **⚠ R9:** `PlanningState.msg`는 UI·진단·파킹 노드가 모두 구독 → 상수 추가는 **append-only**(기존 값 불변). 빌드 후 전 패키지 재빌드 필수.

#### 4-B. SM에 ManualDock 액션 클라이언트 추가
- **파일:** `camrod_planning/scripts/planning_state_machine_node.py`
- SM이 `/docking/manual_dock`을 직접 호출·관리.
- **⚠ R5 — 이중 클라이언트 경합:** UI 백엔드도 같은 액션 client 보유. 택일:
  - **(A·권장)** UI는 요청을 SM에 이벤트로 전달, 실제 전송은 SM 단독
  - (B) UI 직접 전송 유지, SM은 `is_docking`·feedback만 구독
  - 자동 도킹까지 SM이 관장해야 하므로 **A가 정합** (단 UI 리팩터 공수).

#### 4-C. engage 단일 관리 이관
- Stage 2-1의 임시 engage 제어를 SM으로 이관.
```
RUNNING / RECALLED / RETURNING / WARN_RECOVERY   → True
INIT / READY / WAIT_DZ / GOAL_REACHED / CHARGING → False
DOCKING + PHASE_NAV_TO_STAGING                   → True
DOCKING + PHASE_INITIAL_PERCEPTION 이후          → False
```
- **⚠ R6:** 현재 UI 백엔드가 `/planning/engage` 독립 발행 중 → 이관 후 UI 직접 발행 제거(A) 또는 SM 우선순위 규칙 필요.

#### 4-D. 배터리 자동 도킹 트리거
```
battery < critical(15%) & state∈{RUNNING,RETURNING,RECALLED} → 미션중단 후 DOCKING
battery < low(30%)      & state∈{READY,WAIT_DZ}              → DOCKING
```
```yaml
# planning_state_machine.yaml (신규 섹션)
auto_dock_enabled: false
dock_id: "home_dock"
dock_low_battery_threshold: 30.0
dock_critical_battery_threshold: 15.0
resume_battery_threshold: 80.0
max_dock_retries: 3
auto_resume_after_charge: true
max_charge_wait_sec: 3600.0
```
- **⚠ R7:** 배터리 토픽명 확인 필요 — UI는 `/battery_percentage` 구독. 통일할 것.

#### 4-E. 미션 중단/재개 (SavedMission)
- RUNNING→DOCKING 진입 시 `(mission_key, goal_pose, scenario_id)` 저장 → CHARGING 완료 후 복원 → RUNNING 재개.
- **⚠ R8:** 도킹 중 리콜/복귀 요청 시 우선순위 규칙 필요: **ERROR > 도킹완료 > 리콜 > 복귀**. 실패 시 저장 미션 폐기 정책도 정의.

#### 4-F. 도킹 중 외부 goal 인터록
- `state ∈ {DOCKING, CHARGING}`이면 `/planning/request_mission` 서비스·UI destination 요청 거부(409). 새 미션 goal이 Nav2/EgoPolar 명령과 충돌하는 것을 막는다.

**✓ Stage 4 완료 판정:** 배터리 임계 → 미션 자동 중단 → 도킹 → 충전 → 미션 자동 재개 전 사이클을 **SM 단독 제어**로 성공. UI는 상태만 표시.

**완전 자율 사이클 시퀀스:**
```
Battery         Planning SM        Docking          Gate/Nav2       Robot
  │ battery<15% →   │                 │                 │             │
  │             [미션 저장]           │                 │             │
  │                 │─ManualDock goal→│                 │             │
  │                 │─engage=True(Phase1 게이트 열림)──→│─staging주행→│
  │                 │←feedback:INITIAL_PERCEPTION───────│             │
  │                 │─engage=False(Phase2 게이트 닫힘)─→│             │
  │                 │                 │─EgoPolar 직접제어──────────→밀착│
  │                 │←result:success──│                 │             │
  │             [CHARGING]            │                 │             │
  │ battery≥80% →   │                 │                 │             │
  │             [저장 미션 복귀 → RUNNING]              │             │
```

---

### STAGE 5 — 안정성 · 운영 강화 🟡 중간 / 🟢 낮음

- **5-1** 🟡 `/docking/is_charging` 발행 + CHARGING 정합 — `WAIT_FOR_CHARGE` feedback 시 `True`, 종료 시 `False`. `use_battery_status: true` 전환 후 실충전 하드웨어 연동.
- **5-2** 🟢 `manual_dock_server` LifecycleNode 전환 — `rclcpp_lifecycle::LifecycleNode`로 전환, `lifecycle_manager.yaml`에 등록. opennav_docking 크래시 후 자동 재연결.
- **5-3** 🟢 opennav_docking 헬스 모니터링 — `camrod_system` 진단에 `docking_server_checker` 추가. lifecycle 상태 폴링 → UI 표시.
- **5-4** 🟢 다중 도킹 스테이션 지원 — `bridge_params.yaml`의 `target_tag_id: 3` 단일 고정 해제. `dock_id → docks.yaml 조회 → 태그 ID 동적 설정` 서비스화.

---

## 4. 우려사항 총정리 (리스크 레지스터)

| # | 우려사항 | 심각도 | 영향 | 완화책 | Stage |
|---|---------|-------|------|-------|-------|
| R1 | Phase 전환 시 cmd_vel 이중 명령 | 치명 | 로봇 예측불가 거동 | engage Phase별 전환 | 2 |
| R2 | navigate_to_pose 네임스페이스 불일치 | 치명 | Phase 1 즉시 실패 | remapping | 1 |
| R3 | ESKF vs odom_yaw_corrector TF 충돌 | 높음 | 좌표 이중 발행 | 풀스택 시 corrector off | 2 |
| R4 | fixed_frame odom → ESKF 리셋 드리프트 | 높음 | staging 좌표 틀어짐 | map 전환 | 1 |
| R5 | SM/UI 이중 ManualDock 클라이언트 | 높음 | goal 중복/취소 혼선 | 옵션 A (SM 단독) | 4 |
| R6 | SM/UI engage 이중 발행 | 높음 | 게이트 상태 진동 | UI 직접발행 제거 | 4 |
| R7 | 배터리 토픽명 불일치 | 중 | 자동 트리거 미동작 | 토픽명 통일 | 4 |
| R8 | 도킹 중 리콜/복귀 요청 충돌 | 중 | 상태 전환 모호 | 우선순위 규칙 | 4 |
| R9 | PlanningState.msg 변경 전파 | 중 | 구독 노드 빌드 깨짐 | append-only + 전체 재빌드 | 4 |
| R10 | manual_dock_server 크래시 복구 불가 | 낮 | 재시작 필요 | LifecycleNode | 5 |

---

## 5. 착수 순서 & 우선순위 (전체 로드맵)

```
┌ STAGE 1 🔴 Phase 1 인프라 (1-2 실측부터, 병목) ┐
│                                                │──→ STAGE 3 🟠 통합 실주행 검증
└ STAGE 2 🔴 cmd_vel 안전장치 (동시 진행)        ┘         (10회 중 8회 성공)
                                                              │
                       ★ 파이프라인 검증 완료 → SM 개편 시작 ★
                                                              ↓
                                          STAGE 4 🟠 SM 재정의/확장
                                          (DOCKING/CHARGING, 배터리·미션재개)
                                                              ↓
                                          STAGE 5 🟡🟢 안정성·운영 마감
                                                              ↓
                              Voice 어댑터 · 인디케이터 연계 (SM 상태 구독, 별도 프로젝트)
```

**착수 순서 한 줄 요약:** **1-2 docks.yaml 실측**(필드 병목이라 제일 먼저) 착수와 동시에 → **Stage 1 나머지 + Stage 2** 설정/코드 병행 → **Stage 3**로 파이프라인 검증 완료 → 그 검증 데이터 위에서 **Stage 4 SM 개편** → **Stage 5** 마감 → Voice/인디케이터는 SM 완성 후 상태 구독으로 연결.
