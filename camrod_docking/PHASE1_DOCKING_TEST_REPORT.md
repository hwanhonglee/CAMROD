# CAMROD 도킹 Phase1 / Phase1→2 전환 시뮬레이션 테스트 리포트

작성일: 2026-07-07 · 브랜치: `docking-phase1-phase2-integration-test` · 환경: Jetson(aarch64), ROS2 Humble, 캠핑장(park) 맵, RViz 헤드리스

---

## 1. 목적과 범위

- **목적**: 캠핑장(park) 맵 시뮬레이션에서 자율 도킹의
  - (a) **Phase 1** (Nav2 staging 주행) 동작 확인
  - (b) **Phase 1 → Phase 2 전환** (`INITIAL_PERCEPTION` 진입) 확인
- **범위 밖(=sim 검증 불가)**: Phase 2의 실제 AprilTag 검출 이후(마커 추종·접촉·충전). 헤드리스 sim에는 후방 카메라/실물 마커가 없으므로 검출 단계는 실차에서만 검증 가능.
- **후면 도킹**: 로봇 후면에 충전 장치가 있어 후진 도킹. 시스템은 이미 후면 도킹 구성(`dock_backwards: true` + 후방 카메라 `econ_rear`로 AprilTag 감지).

---

## 2. 도킹 시스템 구조 (데이터 흐름)

```
[UI / CLI] → DockRobot 액션(/docking/dock_robot) → [opennav_docking docking_server]
   Phase 1: navigate_to_pose 로 staging pose 까지 자율주행 (CAMROD Nav2)
   Phase 2: 후방 카메라 AprilTag 검출 → 마커 추종 후진 도킹
```

DockRobot 액션의 상태(feedback.state)와 결과(result.error_code)가 판정의 핵심 신호:

| feedback.state | 단계 | 의미 |
|---|---|---|
| 1 | NAV_TO_STAGING_POSE | **Phase 1** — staging 지점으로 주행 |
| 2 | INITIAL_PERCEPTION | **Phase 2 진입** — 마커 초기 검출 |
| 3 | CONTROLLING | Phase 2 — 마커 추종 |
| 4 | WAIT_FOR_CHARGE | 접촉/충전 대기 |

| result.error_code | 의미 | 어느 단계에서 발생 |
|---|---|---|
| 0 (NONE) | 성공 | — |
| 903 (FAILED_TO_STAGE) | **Phase 1(staging) 실패** | staging 미도달 → 전환 못 함 |
| 904 (FAILED_TO_DETECT_DOCK) | **Phase 2 진입 후 마커 미검출** | staging 통과 → 전환 성공 후 검출 실패 |

> **핵심**: `903`이면 Phase1에서 막힌 것(전환 실패), `904`가 나오면 반드시 `INITIAL_PERCEPTION`(Phase2)에 진입했다는 뜻(전환 성공). 즉 error_code 자체가 전환 여부의 강력한 증거.

---

## 3. 테스트 전에 발견·수정한 선행 문제 (왜 필요했나)

Phase1이 처음엔 goal 전송 즉시 `error_code 903`으로 실패했다. 원인을 순차적으로 추적하며 아래를 수정. **모두 "Nav2 planning 스택이 활성화되어 navigate_to_pose 서버가 떠야 한다"로 수렴**한다.

| # | 문제 | 원인 | 수정 |
|---|---|---|---|
| 1 | SHM 전송 에러, 통신 불안정 | `~/.ros/fastdds.xml` SHM segment_size=256MB × ~30노드 → `/dev/shm`(7.7G) 포화 | segment_size 256MB→**64MB** |
| 2 | planner_server configure 실패(롤백) | `nav2-navfn-planner`, `nav2-theta-star-planner` 미설치 (planner_plugins가 요구) | apt 설치 + `setup_camrod.sh` 반영 |
| 3 | planner_server configure 실패(롤백) | `SmacLattice.lattice_filepath`가 `/opt`(없음) 지정. nav2_smac_planner는 repo에 vendored 소스빌드 | 워크스페이스 install 경로로 수정 |
| 4 | controller_server configure 실패(롤백) | `nav2-dwb/mppi/rotation-shim-controller` 미설치 (controller_plugins가 요구) | apt 설치 + setup 반영 |
| 5 | bt_navigator activate 실패(inactive) | `nav2-behaviors` 미설치 → behavior_server 미기동 → BT의 `<Spin>/<BackUp>` 참조 불가 | apt 설치 + setup 반영 |
| 6 | bt_navigator activate 실패(inactive) | `nav2-smoother` 미설치 → smoother_server 미기동 → BT의 `<SmoothPath>` 참조 불가 | apt 설치 + setup 반영 |
| 7 | Phase1 즉시 903 (planning은 정상인데) | **opennav 내부 navigator가 노드 remap을 무시**, `/docking/navigate_to_pose`(서버 없음) 호출 | `navigate_to_pose_relay` 노드 신규 추가(서버 제공 → `/planning/navigate_to_pose` 중계) |

정리: **누락 nav2 플러그인 패키지(2 planner + 3 controller + behaviors + smoother)** 를 모두 설치하니 planning lifecycle(planner/controller/bt_navigator/behavior/smoother) 5개가 전부 `active`가 되었고, **릴레이 노드**로 opennav이 실제 Nav2 서버에 연결됨.

---

## 4. 전환을 위해 조정한 파라미터와 그 의미

Phase1이 목적지 근처까지 가는데도 **staging 최종 도달 판정**을 못 해 전환이 안 되는 문제가 남았다. 이를 위해 조정한 값과 의미:

### 4-1. `goal_checker.xy_goal_tolerance` : 0.15 → 0.3  (nav2_base.yaml)
- **의미**: Nav2가 목표(staging) 도달로 인정하는 위치 허용 반경(m). 로봇 base_link 중심이 목표에서 이 거리 안이면 "도착".
- **왜 조정**: 로봇이 lanelet 경로 종단에서 staging 목표까지 **~0.2m를 남기고** 멈추는데(경로가 그 지점에서 끝남), 0.15m로는 도착 미인정 → nav 미완료 → Phase2 전환 실패(903).
- **정당성**: Phase1 staging은 "dock 앞에 대략 정렬"이 목적이고 **정밀 정렬은 Phase2(후방 카메라+마커)** 가 담당하므로, staging 위치를 0.15m로 빡빡하게 볼 필요가 없다. 0.3m는 과거 0.8m처럼 느슨하지 않으면서 종단 오차를 흡수.
- **판정**: **유의미(유지)**. Test B에서 0.15로 되돌리니 전환 0/3.

### 4-2. `yaw_goal_tolerance` : 3.14 (기존값, 미변경)
- **의미**: 목표 도달 시 방향(yaw) 허용 오차(rad). 3.14=180°=사실상 방향 무제한.
- **관련 발견**: 로봇이 차선 방향(-65°)으로 도착해 staging 요구 방향(24°)과 ~89° 차이가 났지만, yaw 허용이 180°라 **방향은 전환의 blocker가 아니었다**. (후면 도킹의 정밀 방향은 Phase2가 맞춤)

### 4-3. dock 좌표 (docks.yaml `home_dock.pose`)
- **의미**: 도킹 스테이션(마커/접촉점) 자체의 map 좌표. opennav이 여기서 `staging_x_offset(-0.7m)`만큼 뺀 지점을 Phase1 목표(staging)로 계산.
- **왜 건드림**: park `camping_site_1` 중심(37.14,-47.49)은 staging이 **lanelet 경계에서 ~2.7m** 떨어져 있어, 중심선 추종 플래너(LaneletRoute)가 최종 접근을 못 함.
- **판정**: 재배치는 "전환 신뢰성"에는 유의미하나(아래 Test A vs C), 신뢰성을 만든 좌표(34.221,-48.608)는 **"로봇이 실제 닿는 lanelet 지점"을 역산한 임의값**이라 실제 충전기 위치가 아님 → **camping_site_1 중심으로 원복**하고 실험 결과만 주석에 기록. 실차 신뢰성은 "실 충전기 위치 + 그 앞 staging이 차선망에서 도달 가능"하도록 배치하는 **맵/배치 설계 이슈**로 남김.

### 4-4. (부수) `dock_prestaging_tolerance` : 0.5 (실차값 유지)
- 초기에 sim 전환을 억지로 보려고 5.0으로 올려봤으나, "sim에서만 되고 실차와 괴리"라는 지적에 따라 **0.5(실차값)로 원복**.

---

## 5. 테스트 방법론

### 5-1. 환경 구성
- **헤드리스 실행**: `docking_phase1_sim.launch.py rviz:=false`
  - 이유: RViz+풀 Nav2 부하로 **gnome 데스크톱 세션이 크래시**(터미널·앱 종료)한 사례가 있어, GUI 부하를 제거하고 안정적으로 반복 테스트.
- **경량 sim 스택**: bringup(sim=true) — EKF 로컬라이제이션 · map · Nav2(planner/controller/bt/behavior/smoother) · cost grid. 진단/UI/state_machine 등 고부하 보조노드는 OFF.
- **게이트 확인**: goal 전송 전 `planner_server / controller_server / bt_navigator / behavior_server / smoother_server` 5개 `active [3]` + `/docking/navigate_to_pose`에 릴레이 서버 1개 존재를 매번 확인.

### 5-2. Phase 1 동작 확인 방법
- `/planning/engage`, `/platform/drive_enable`를 true로 발행(주행 활성화)
- DockRobot goal 전송(`navigate_to_staging_pose: true`)
- **관찰**: `feedback.state==1(NAV_TO_STAGING)` 수신 + `map→robot_base_link` TF로 로봇이 시작점에서 dock 구역으로 **실제 이동**하는지 좌표 추적.
- 관측 예: `(-3,1) → (14,2) → (21,-12) → (31,-43) → (33.5,-48.7)` 로 캠핑장 맵을 가로질러 dock 근처까지 주행 확인.

### 5-3. Phase 1 → 2 전환 확인 방법 (핵심)
- **rclpy 액션 클라이언트(`dock_probe.py` / `dock_trials.py`)** 로 DockRobot goal을 보내고 feedback·result를 **직접 수신**.
  - `ros2 topic echo`는 **출력 버퍼링 + 짧게 스쳐가는 state 2**를 놓쳐, 초기엔 "전환 안 됨"으로 오판했음. → rclpy 직접 수신으로 해결.
- **fresh run 보장**: 매 시험 전 `/initialpose`로 로봇을 고정 시작점(≈`(21.5,-19)`, staging에서 ~30m)으로 **리셋**(teleport). 리셋 없이 반복하면 로봇이 이미 staging 근처(pre-staged)라 항상 전환되어 결과가 왜곡됨.
- **반복 시험**: 동일 조건에서 N회 반복해 전환 성공 횟수 집계.

### 5-4. 성공/실패 판정 근거 (2개 독립 신호)
전환 "성공"은 **추측이 아니라 opennav 액션 인터페이스의 실제 값**으로 판정:
1. **feedback.state ≥ 2 수신** = `INITIAL_PERCEPTION` 도달 관측
2. **result.error_code == 904 (FAILED_TO_DETECT_DOCK)** = staging 통과 후 검출 단계 진입(903이면 staging 실패였을 것)

두 신호가 일치할 때만 "전환 성공"으로 집계. (904는 sim에 마커가 없어 나는 **예상된 결과** — 전환 자체는 성공)

---

## 6. 실험 결과

각 조건에서 fresh run 반복. 판정 = `state 2 도달 AND error_code 904`.

| 시험 | dock 좌표 | xy_goal_tol | 전환 성공 | error_code | 해석 |
|---|---|---|---|---|---|
| **A** (조정 적용) | 34.221,-48.608 (도달점 역산) | **0.3** | **10 / 10** | 전부 904 | staging 완주 → 100% 전환 |
| **B** (xy 원복) | 34.221,-48.608 | 0.15 | **0 / 3** | 결과 없음/None | ~0.2m 미달 → 전환 실패 |
| **C** (dock 원복) | 37.1358,-47.4945 (camping_site_1) | 0.3 | **flaky (1~2 / 3)** | 904/None 혼재 | staging이 차선 밖 ~2.7m → 접근 불안정 |

- **Test A**: 10회 모두 `state 1→2` 전환 + 904. 시작점 리셋(21.5,-18.9) 로그로 fresh run 확인. → **현재 조정 조합에서 전환 100% 재현.**
- **Test B**: xy만 0.15로 되돌림 → `max_state=0~1`, 결과 미반환(staging에서 못 끝남). → **xy 0.3 필요성 입증.**
- **Test C**: dock만 원래값으로 → 시작점이 깔끔했던 시행만 전환, 아니면 실패 → **dock 배치가 전환 신뢰성에 직접 영향.**

---

## 7. 결론

1. **Phase 1 동작**: ✅ 정상 — 로봇이 캠핑장 맵을 가로질러 staging 구역까지 자율주행.
2. **Phase 1 → Phase 2(INITIAL_PERCEPTION) 전환**: ✅ **확인됨** — 적절한 조건(staging 도달 가능 + xy_tol 0.3)에서 **10/10 재현**. 근거: feedback state 2 + result 904.
3. **Phase 2 이후(마커 검출~도킹 완료)**: sim 검증 불가(마커/후방카메라 부재, 904에서 정지). 실차 필요.
4. **조정 파라미터**:
   - `xy_goal_tolerance 0.3`: **유의미 → 유지**.
   - dock 재배치: 신뢰성엔 유효하나 값은 임의 → **camping_site_1로 원복**, 실차는 배치 설계로 해결.

## 8. 남은 과제 (실차/후속)
- **dock/staging 배치**: 실제 충전기 위치 + 그 앞 staging이 lanelet 도달 가능하도록 맵/배치 설계 (전환 신뢰성의 근본).
- **Phase 2 실검증**: 후방 카메라 + 실물 AprilTag 환경에서 INITIAL_PERCEPTION → CONTROLLING → 접촉/충전.
- **후면 도킹 정밀 방향**: staging 종단 방향은 Phase2가 맞추나, 실차에서 후방 카메라 시야에 마커가 들어오는 staging 방향인지 확인 필요.

## 부록: 커밋된 수정 (브랜치 `docking-phase1-phase2-integration-test`)
- 캠핑장 맵 전환(map_info×2, docks.yaml)
- 누락 nav2 패키지 설치 자동화(setup_camrod.sh: navfn/theta-star/mppi/dwb/rotation-shim/behaviors/smoother)
- SmacLattice lattice_filepath 수정(nav2_base.yaml×2)
- `navigate_to_pose_relay` 노드 신규(+CMakeLists/package.xml/런치 2개)
- (미커밋 예정) `xy_goal_tolerance 0.3`
