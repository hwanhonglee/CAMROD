# B3 정밀 centering 재정렬 후보와 회귀 테스트 설계 — 미적용

<!-- HH_261012 - Planning artifact only. V5 source and runtime remain frozen. -->

이 문서는 [실제 B3 실패](ANALYSIS.md)에 대한 후보 설계다. 현재 V5에는 적용하지 않는다. 남은 B6–B10 실행과 안전한 suite 종료 이후 별도 승인·빌드·source-bound gate·실제 재시험이 필요하다. V5와 향후 V6 결과를 섞어 동일 profile의 13/13 성공이라고 표기하지 않는다.

## 수정 범위

예상 수정: [controller](../../../../../../../../camrod_control/src/camping_site_maneuver_controller_node.cpp)의 `publishRotationEntryCentering()` 약 **18–28줄**, [focused tests](../../../../../../../../camrod_control/test/test_camping_site_recall_return.cpp) 약 **130–190줄**, 새 테스트 **6–8개**. Native [sequencer helper](../../../../../../../../camrod_control/include/camrod_control/motion_geometry.hpp), catalog geometry, 속도·torque 한도, 장애물 및 주차 설정은 바꾸지 않는다. 새 parameter나 per-site field는 추가하지 않는 설계다.

이미 `forward_loop`로 명시 활성화된 정밀 centering에서만 다음 조건을 모두 만족하면 기존 sequencer의 lateral 단계로 재시작한다.

1. `site_forward_return_active_ && site_forward_return_centering_active_`.
2. 현재 sequencer stage가 `longitudinal`.
3. Fresh finite pose를 사용하며 아직 **radial ≤ precise target** 성공 조건을 만족하지 않음.
4. `abs(body_longitudinal_error) <= precise_target`.
5. `abs(body_lateral_error) > precise_target`.
6. `abs(body_lateral_error) <= min(native_lateral_transition_tolerance, 0.5 * precise_target) + native_lateral_hysteresis` — 기존 실제 sequencer의 lateral latch limit 안에 있음.

현재 precise target은 `min(0.01, 0.2 * turn_guard)`이다. 즉 기본 후보는 longitudinal이 **0.01 m 이내**까지 가까워졌지만 radial이 아직 **0.01 m 밖**이고 lateral 잔차가 **0.01 m 초과**일 때만 재정렬한다. Lateral이 radial 목표보다 큰 경우에는 longitudinal을 0으로 만들어도 목표에 들어갈 수 없으므로 횡방향 보정이 필요하다. 반대로 lateral이 목표 이내이면 먼저 기존 longitudinal 보정을 계속하여 불필요한 wheel 전환을 피한다. 초기 제안의 `0.5 * target` trigger는 아래 기록 비교 후 채택하지 않았다. 이것은 재정렬 trigger를 더 엄격하게 한 것이며 radial 성공 기준·position guard·시간 한도를 완화하지 않는다.

오류, guard/tolerance, lateral 설정, gain/speed가 non-finite인 경우에는 기존 fail-closed 경로를 보존하며, 잘못된 수치로 sequencer 상태를 먼저 재시작하지 않는다. 양의 유효 tolerance를 확인한다. 모든 실제 이동은 기존 bounded command를 그대로 사용한다. 기본 정밀 episode의 lateral latch limit는 `0.005 + 0.10 = 0.105 m`이다. 이보다 큰 횡방향 잔차는, 초기 radial admission `0.65 m` 안이어도, 재시작으로 fault를 지우지 않고 기존 `update().lateral_latch_exceeded`가 ERROR로 처리하게 둔다.

재시작 때 바꾸는 것은 **sequencer stage와 그 내부 stage timestamp만**이다. 같은 fresh pose로 기존 `update()`를 호출한다. 두 축 동시 명령이나 angular 명령은 추가하지 않고, lateral 완료 뒤 기존 **1.2 s zero/steering settle**을 유지한다. 다음 내용은 절대 초기화하거나 확장하지 않는다.

- 최초 centering steady start와 **15 s** deadline.
- 전체 `ALIGN_RETRACE_YAW` phase start와 **90 s** deadline.
- 기존 target XY/yaw, phase, mission/Return 승인.
- Pivot recenter count 및 **10회** cap.
- 초기 precenter admission envelope와 pivot 시작 후 recenter의 **0.05 m** position guard.

이미 **stage=longitudinal AND radial≤0.01 m**이면 현재 `onTimer()`의 성공 shortcut을 그대로 먼저 실행한다. 이때 별도의 `abs(lateral)≤0.005 m` 성공 조건을 추가하지 않는다. 예를 들어 radial `0.008 m`, lateral `0.006 m`인 성공 pose를 불필요하게 재정렬하지 않는다.

## 기록 기반 후보 trigger 비교

Trial의 sample pose, status에 기록된 반올림 yaw, 해당 run의 bounded journal centering-stage 전이를 연결하여 계산했다. 센서/상태/journal이 같은 controller tick에 완전히 동기화된 기록은 아니므로 **정확한 controller replay가 아니라 회귀 후보 선정 근거**다. B1은 centroid가 아니라 기존 `service_x/y` target을 사용했다.

| V5 실행 | 관측 centering episode 수 | 후보 predicate가 성립한 episode | 최초 성립 및 15 s 잔여 시간 |
| --- | ---: | --- | --- |
| B3_attempt02 실패 | 1 | 최초 precenter count 0; 13개 샘플 | 13:39:31.041604 KST, 시작 후 **5.168271 s**, **9.831729 s** 남음 |
| B4 성공 | 8 | 0 | 관측되지 않음 |
| B2_attempt02 성공 | 8 | 0 | 관측되지 않음 |
| B1_attempt02 성공 | 3 | 0 | 관측되지 않음 |
| B5 성공 | 6 | 0 | 관측되지 않음 |
| B6_attempt02 성공 | 2 | 0 | 관측되지 않음; 13:41:04–13:47:12 KST journal/trial 확인 |
| B7_attempt02 성공 | 6 | 0 | 관측되지 않음; 13:49:05–13:55:24 KST journal/trial 확인 |
| B8_attempt02 성공 | 4 | 0 | 관측되지 않음 |
| B9_attempt02 실패 | 1 | 최초 precenter count 0; 4개 샘플 | 14:07:49.693918 KST, 시작 후 **6.140112 s**, **8.859888 s** 남음 |
| B10_attempt02 실패 | 3 | recovery count 1/2; 총 5개 샘플 | recovery 1 시작 후 **3.782403 s**, **11.217597 s** 남음 |

B3 최초 성립 시 radial **0.014195108 m**, body-longitudinal **+0.009805626 m**, body-lateral **+0.010264054 m**였다. 실제 오류 때까지 기다리지 않고 약 9.8 s의 기존 예산 안에서 횡방향 보정을 다시 시도할 여지가 있다. 성공은 실제 재시험으로 확인해야 한다.

B1/B2/B4/B5/B6/B7/B8 성공 실행의 **37개 centering episode**에서 최종 `abs(lateral)>target` predicate가 성립한 샘플은 **0개**이다. **B3/B9/B10 실제 실패 재시험과 B2 실제 회귀 재시험은 필수**로 유지한다. Trigger가 관측되지 않았다는 사실만으로 연속시간 비활성 또는 새 binary의 실제 성공을 보증하지 않는다.

B9는 초기 precenter가 약 `0.300 m`에서 시작하여 **15.062724 s**에 timeout이 났다. 최종 조건이 처음 성립할 때 radial **0.020179276 m**, longitudinal **-0.001387579 m**, lateral **+0.020131513 m**로 횡방향 재보정 기회가 있었다. 그러나 이후에는 longitudinal 왕복/발산도 관측되었다. 마지막 샘플 radial **0.090355336 m**, longitudinal **-0.075703966 m**, lateral **+0.049325411 m**, 실제 body longitudinal 속도 **-0.283222 m/s** 대 명령 **-0.12 m/s**였다. 최종 radial **0.078673168 m**는 최초 precenter의 기존 `0.65 m` admission envelope 안이며, pivot의 `0.05 m` guard를 늘릴 사유가 아니다. Longitudinal 구간에서 실제 actor 횡이동도 **0.039218235 m**였다. **초기에 같은 후보 조건이 존재한다는 뜻이지, 이 수정만으로 B9의 물리 수렴이 입증되었다는 뜻은 아니다.**

B10의 최종 오류는 B3/B9와 구분한다. 최초 precenter 후 실제 회전을 시작했고, **두 번째 active recenter에서 약 4.300 s 만에 0.05 m guard를 넘었다**. 최종 radial은 **0.055753181 m**, 전체 두 번째 회전 단계는 약 **18.507 s**로 15 s centering/90 s 전체 timeout이나 count 10 cap이 원인이 아니다. Recovery 2의 **14:11:53.583036 KST** 샘플은 radial **0.011759863 m**, longitudinal **-0.002358257 m**, lateral **-0.011520981 m**로 최종 후보 조건을 만족한다. 이는 ERROR **14:11:54.527531 KST**보다 약 **0.9445 s** 이르다. 당시 실제 body longitudinal 속도는 **-0.057659 m/s**였고 이후 **+0.116480 m/s**까지 반전했다. 횡방향 잔차가 남은 상태의 longitudinal 왕복이 position guard 위반으로 끝났다. 후보는 그 전에 axis 재정렬을 선택할 수 있지만, 남은 물리 운동량과 steering 전환까지 포함한 실제 B10 수렴은 재시험 전에는 입증되지 않는다. **0.05 m guard는 그대로 유지한다.**

판단 이력: 폐기한 `abs(lateral)>0.5*target` 조건에서는 B2의 count 0/1/4(총 4개 샘플), B8의 count 3(1개 샘플)이 재정렬 대상으로 잡혔다. B2 예시는 radial **0.011713333 m**, longitudinal **+0.008142794 m**, lateral **+0.008420041 m**; B8 예시는 radial **0.010236554 m**, longitudinal **-0.008398037 m**, lateral **+0.005853205 m**였다. 둘 다 lateral이 0.01 m보다 작아 기존 longitudinal 보정만으로 목표에 도달할 수 있으므로 최종 predicate에서는 제외된다. **B8은 최종 조건에서 관측된 trigger 후보가 아니다**; 추가 비교 실행을 한다면 이 사실과 구분한다.

## focused-test 설계

1. **B3/B9 측정 잔차**: forward-only centering의 longitudinal stage에 B3 `long=0.00155`, `lat=0.0206 m`와 B9 최초 trigger `long=-0.00138758`, `lat=0.02013151 m`를 넣으면 lateral로 돌아가 `vx=0`, bounded `vy`, `wz=0`. 남은 target/yaw/phase/count는 동일.
2. **근접 longitudinal 조건 및 경계**: `abs(long)>0.01` 또는 `abs(lat)<=0.01`이면 기존 longitudinal stage 유지. 특히 B2/B8의 위 측정 예시를 넣어 불필요한 재정렬이 생기지 않음을 확인한다. 양·음 lateral 방향을 모두 확인하고 작은 guard에 비례한 precise target도 확인.
3. **기존 radial 성공 유지**: stage longitudinal이며 radial≤0.01이면 lateral이 0.005를 넘더라도 기존 성공 shortcut으로 회전 준비를 완료한다. 반대로 radial>0.01이면 완료하지 않는다.
4. **예산 갱신 금지**: 재정렬을 여러 번 유도해도 centering steady start/phase steady start/target/recenter count가 그대로인지 확인. 15 s 만료 시 중심에 도달했더라도 기존 deadline 우선 순서로 ERROR, zero output. 전체 90 s도 동일.
5. **위치·pose/latch 안전 조건**: pivot 중 recenter에서 0.05 m 초과나 stale/non-finite pose이면 재정렬로 구제하지 않고 ERROR. 최초 precenter의 radial admission `0.65 m` 안이지만 lateral이 실제 latch limit `0.105 m`보다 큰 경우에도 재시작하지 않고 기존 `lateral_latch_exceeded`로 ERROR임을 확인한다. 초기 precenter의 별도 envelope와 혼동하지 않는다.
6. **유효 수치 및 출력**: non-finite 입력/guard/gain/speed는 sequencer를 재시작하지 않는지, 기존 fail-closed 동작과 bounded speed, centering 중 angular=0, lateral→settle→longitudinal 순서 및 기존 settle 시간이 유지되는지 확인한다.
7. **Native/urgent 불변**: 동일 잔차라도 opt-in forward episode가 아니면 기존 longitudinal latch를 유지한다. Cancel/urgent/new mission 후 optional 동작이 남지 않는다.
8. **기존 회귀 전체**: 기존 focused 45개 테스트와 추가 테스트, 기존 정책/Python/catalog 검사, 새 source-bound gate를 재수행하고 B2/B3/B9/B10의 실제 주행·주차·캡처를 확인한다. B10의 active-guard 실패는 timeout 실패와 별도로 판정한다.

본 문서 작성 시점에는 controller/test/runtime를 변경하지 않았다. 수정 후에도 이전 V5 실패는 삭제하거나 PASS로 덮어쓰지 않는다.
