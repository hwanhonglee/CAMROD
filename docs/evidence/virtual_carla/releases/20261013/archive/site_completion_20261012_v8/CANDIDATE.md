# V8 후보: B10 forward-return steering settle 0.4 s

<!-- HH_261012 - Read-only V7 evidence analysis; this document is not a V8 driving result or latency certification. -->

**아직 실제 주행 성공을 주장하지 않는 후보**다. [V7 B10](../site_completion_20261012_v7/B10/ANALYSIS.md)은 활성 center/trigger **0.015/0.035 m**에서 네 차례 lateral realign 후 recovery 4의 **15.000064 s** deadline에 도달했다. 원시 trial (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v7/B10/trial.json`), 실제 wheel/body telemetry와 `camrod-sites-20261012-v7-camrod.service`의 **2026-10-12 15:16:14–15:16:51 KST** journal을 읽어 아래 수치를 확인했다.

## 제안 범위

B10의 명시적 `forward_loop` profile에만 **`forward_return_steering_settle_s: 0.4`**를 선택한다. 이는 lateral centering 후 wheel을 0°로 돌리며 zero command를 유지하는 dwell이다. B9 및 나머지 사이트의 omitted-field 동작과 native controller 기본값 **1.2 s**는 유지한다. Profile은 mission-local이어야 하며 cancel/urgent/error/다음 mission에 넘겨지지 않아야 한다.

다음 값은 변경하지 않는다.

- B10 convergence target **0.015 m**, proactive recenter trigger **0.035 m**.
- 실제 active position hard guard **0.05 m**, 각 centering **15 s**, 전체 heading **90 s**, recenter 최대 **10회**.
- 기존 속도·gain·torque cap, fresh-pose/steering/backend acceptance interlock, native 최종 station parking **0.25 m**.
- 다른 사이트 geometry·서비스 회전·주행 경로와 기존 terrain/objects.

Explicit dwell은 유한한 양수이며 native dwell보다 길지 않아야 하고, 기존 target/trigger pair와 hard-guard admission을 통과해야 한다. 이 문서는 후보의 요구 범위이며 구현·빌드·gate 및 실제 실행 결과를 대신하지 않는다.

## 왜 dwell을 검토하는가

V7 recovery 4는 **15:16:35.732267–15:16:50.732331**이었다. 실제 lateral 보정은 수 mm 잔차까지 도달했지만, **1.2 s zero-command steering dwell 동안 lateral 잔차가 다시 커졌고**, 이후 longitudinal 보정 중 full-target lateral realign 조건이 재발했다. 단순히 큰 gain으로 목표를 고속 통과하는 longitudinal overshoot만으로 설명되는 기록은 아니다.

같은 episode의 다섯 steering-settle 구간에서, 기록된 zero command 샘플만 선택하고 첫/마지막 sample의 localization residual을 body 축으로 재계산했다. 각 구간의 실제 표본 간격은 **0.920–0.968 s**이며, 이는 전체 1.2 s dwell과 동일한 길이라고 주장하지 않는다.

| Settle 시작 KST | 첫 → 마지막 lateral 잔차 (mm) | 첫 → 마지막 longitudinal 잔차 (mm) |
| --- | ---: | ---: |
| 15:16:37.131759 | -1.718 → -11.165 | -22.750 → -21.551 |
| 15:16:40.031782 | -3.425 → -11.799 | -19.265 → -19.293 |
| 15:16:42.931841 | -2.595 → -11.495 | -23.831 → -23.516 |
| 15:16:45.931881 | -3.418 → -9.751 | -18.520 → -19.714 |
| 15:16:48.931954 | -3.765 → -11.027 | -19.724 → -19.681 |

이 표의 body residual은 localization XY와 상태에 기록된 반올림 yaw를 사용한 근사값이다. 별도 CARLA atomic body-velocity를 실제 body yaw로 투영하면 dwell 중 여러 sample에서 횡속도 **약 +0.007–+0.010 m/s**가 직접 관측된다. Localization 변화만을 물리 움직임으로 오인한 설명은 아니다. Recovery 4의 실제 body attitude는 pitch **약 -2.156…-2.093°**, roll **약 -3.747…-3.277°**였다. 경사진 자세에서 발생한 물리 drift와 양립하지만, terrain 경사만이 유일한 원인이라는 인과 인증은 아니다.

가설은 **wheel이 이미 0°이고 속도가 작아진 뒤에도 남는 dwell을 줄여**, longitudinal 보정 전에 횡잔차가 다시 증가하는 시간을 줄이는 것이다. 매번 lateral → dwell → longitudinal → lateral로 되돌아가는 cycle과 deadline 소비를 줄일 가능성이 있다. 실제로 0.015 m radial 안에 수렴할지는 재시험 대상이다.

## 아홉 실제 전환의 관측 시간

V7 B10 return-heading의 steering-settle 로그는 **9개**였다. 각 로그 시각 이후 물리 sample에서 네 wheel 모두 `steer_angle_degrees≈0` 및 `external_drive_torque_nm≈0`을 처음 관측한 시간, 같은 조건에서 body-XY speed **≤0.015 m/s**도 처음 만족한 시간을 계산했다. 계산상 zero 기준은 각각 **1e-4° / 1e-6 Nm**였고 해당 기록 값은 0이었다.

| Settle 시작 KST | 첫 four-wheel zero 관측 Δs | zero + speed≤0.015 관측 Δs |
| --- | ---: | ---: |
| 15:16:15.531380 | 0.187193 | 0.187193 |
| 15:16:23.231510 | 0.150765 | 0.150765 |
| 15:16:27.931886 | 0.101617 | 0.101617 |
| 15:16:32.631671 | 0.043223 | 0.268440 |
| 15:16:37.131759 | 0.198494 | 0.198494 |
| 15:16:40.031782 | 0.073684 | 0.073684 |
| 15:16:42.931841 | 0.214299 | 0.214299 |
| 15:16:45.931881 | 0.063815 | 0.288317 |
| 15:16:48.931954 | 0.084130 | 0.084130 |

Four-wheel zero 첫 관측의 최대는 **0.214299 s**이고, 속도 조건까지 포함한 최대는 **0.288317 s**였다. Wheel zero가 곧 body 정지를 의미하지는 않는다. 예를 들어 15:16:32 전환의 최초 zero-wheel sample에도 speed **0.047268 m/s**가 남았다.

후보 **0.4 s**는 관측된 두 번째 최대값보다 **약 0.111683 s** 길다. **0.3 s**라면 같은 관측 여유가 약 **0.011683 s**뿐이다. 이는 **이 한 실행의 경험적 간격**이며, 제어/통신 latency의 상한, 모든 terrain의 제동 시간, 연속적인 settling 유지, hardware 안전 인증이 아니다. 최대 5 Hz sample·ROS/journal/물리 telemetry의 서로 다른 관측 시각을 포함하므로 수치를 정밀한 실시간 latency 보증처럼 사용하지 않는다.

## 실제 V8에서 확인할 것

새 binary/gate/profile이 일치하는 상태에서 B10의 실제 active dwell **0.4 s**, B9·기타/native의 **1.2 s** 기본값 보존을 확인해야 한다. Wheel/torque/body telemetry, phase별 실제 시간, sampled radial 최대, realign/recenter 횟수, 전체 도로 return 및 native station parking 완료를 함께 확인한다. Step 통과만으로 왕복 PASS를 선언하지 않는다.

V7 실패 이미지와 기록은 유지한다. V8이 실패하면 **0.05 m / 15 s / 90 s / 10회** 한도를 늘려 성공으로 재분류하지 않고 실제 종료 조건을 별도로 기록한다. 이 후보 문서 작성은 읽기 전용 분석과 Markdown 추가만으로 수행했으며 runtime·vehicle API·source 변경은 하지 않았다.
