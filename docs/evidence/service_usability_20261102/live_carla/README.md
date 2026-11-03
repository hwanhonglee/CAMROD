# 2026-11-02 CARLA 서비스 유용성 재시험

HH_261102 - Record actual simulator observations separately from earlier UI fixtures and synthetic sensor tests.

## 시험 범위

- 별도 작업 공간의 수정본 control·sensing·UI·ground segmentation 설치를 사용한다.
- CARLA 전용 bringup·adapter는 `/home/hong/camrod_ws`의 기존 설치를 재사용한다. 순수 unit2 bringup은 외부 CARLA 실행기를 포함하지 않아 전체 overlay로 덮어쓰지 않았다.
- 최초 두 실행은 기존 Woraksan V224 맵을 사용했다. 이후 사용자의 태그 위치 수정 요청에 따라 **태그 46조각만 강체 이동·회전한 별도 사본** `Woraksan_v27_service_tag_20261102`를 만들었다. 원본과 보호 자산 522개는 불변이다. Ranger 모델·20 Hz 물리/스텝 페이싱·10 Hz 센서·지형은 유지했다. 차량 순간이동·가짜 PointCloud/Range/충전 신호 발행은 하지 않는다.
- 배터리만 CARLA 전용 `/camrod_carla/platform_heartbeat/soc`에 0–1 비율로 주입한다. 제어 대상은 CARLA heartbeat이며 실제 BMS 변화가 아니다.
- 저장은 `/home/hong/camrod_service_test_ws/state/service_usability_live_20261102`로 격리했다. 실물 운행 DB와 기존 가상시험 DB에는 합산하지 않는다.
- `runtime.sh`는 domain 188·localhost, 새 control/sensing/UI와 기존 virtual bringup/adapter의 패키지 경로를 검사한다.

## 실행 전 확인

새 물리 게이트 `c7b2325c0dbb1af4c2e689b1cd445e5496c47f3a1802727db6704a7ff3558025`를 공식 도구로 생성·검사했다.
기존 게이트 파일이나 source hash를 고쳐 검사를 우회하지 않았다.
공식 새 CARLA actor 물리 점검(8개 모드/부호 + 2개 전환), control CTest 13/13, sensing 9/9, source contract 93 통과·과거 fixture 3 skip을 확인했다.
원본 보고서는 `/home/hong/Downloads/ranger-carla-4ws-pipeline/.work/evidence/service_usability_live_20261102`에 있다.

이 점검 자체는 도킹·근거리 장애물 통합시험 PASS를 의미하지 않는다.

## 현재 관측

| 조건 | 관측 / 판정 |
| --- | --- |
| 34% 새 호출 | Robot backend가 HTTP 400 `battery_below_mission_minimum` 반환. 미션 generation=0·비활성 유지. [기록](recall_soc34_backend.json) |
| 34% Guest 화면 | 기존 연결에서 SOC34, B1~B13·호출 버튼 disabled 확인. 단, 초기 운행 정지 화면이 선택 창을 덮으므로 정상 대기 화면에서의 단독 배터리 차단 증빙과 구별. [DOM](guest_soc34.json), [PNG](guest_soc34.png) |
| 34% 미션 중 | 첫 실행에서 이미 수락된 B7 배송 지속·복귀 필요 상태 확인. 34%는 즉시 긴급 복귀 임계값이 아님. |
| 34% 복귀 후 자동 도킹 선택 | B7 실제 도착·완료·복귀 → reverse PARKED → `forced=false` AprilTag 자동 선택을 두 번 관측. 선택 정책은 확인됐지만 최종 도킹 완료와 구분. |
| 태그 수정 전 최종 도킹 | FAIL: WAITING_FOR_TAG 시간 초과. 실제 영상 23프레임/5초, 태그 위치 메시지 0. [결과](battery_step/result.json), [GIF](battery_step/phase_progress.gif) |
| 태그 수정 후 검출 | 실제 카메라 중앙에서 ID3·약0.399m, 태그 위치 메시지 26개/5초. [수정 전후 GIF](tag_alignment/tag_camera_before_after.gif), [검출 PNG](tag_alignment_after/detector_debug.png) |
| 태그 수정 후 최종 도킹 | FAIL: 거리0.392m에서 횡오차0.045m가 유지한 허용0.030m를 초과해 안전정지. 실제 충전 신호 false. [결과](battery_aligned_step/result.json), [단계별 GIF](battery_aligned_step/phase_progress.gif) |
| 35% 이상 충전 위치 → 기존 대기 위치 정차 | **미검증 / 도킹 실패로 차단.** 충전 신호를 강제로 만들어 성공 처리하지 않았다. |
| 해당 출차 후 미션 수락·수행 | **미검증 / 앞 단계 차단.** 일반 SOC80 B7 배차·실주행은 별도로 관측했지만 충전 출차 후 성공으로 대체하지 않는다. |
| Radar/LiDAR 근거리·과민반응 | 실제 actor의10/20/30/40cm 배치와 도로변2조건 완료. 현재30cm 설정에서10cm만 가드 점유 감지,20/30/40cm는 두 격자 모두 점유 감지 없음. 이미 FAULT_HOLD 상태였으므로 물리 제동 시험 PASS는 아님. |

34%의 의미는 정상 미션 완료·이용객 복귀 확인 후, 복귀 주차에서 자동 도킹하는 것이다.
즉시 강제 복귀 임계값 25%와 혼동하지 않는다. 초기 dispatcher IDLE에서 SOC만 낮춘 것은 주차 시작 증빙이 아니다.

## 재현된 문제와 보존 자료

1. **첫 실행 센서 스트림 중단:** B7 도착 후 복귀 CRAB_OUT 중 CARLA tick/센서가 진행되지 않았다. 브리지의 30초 tick timeout, 페이서의 20초 STEP_ONCE timeout, 센서 stale과 gate 정지를 관측했다. `battery_live_01.jsonl`에 전후 상태를 보존했다. 도킹 성공으로 집계하지 않는다.
2. **재실행:** 소유한 runtime을 모두 종료하고 같은 모델/맵으로 새 server·bridge·spawn 토큰을 사용했다. 서버 출력은 계속 읽되 파일 크기가 폭증하지 않도록 첫 4,000줄과 이후 백만 줄당 한 줄을 보존했다. 출력 backpressure가 원인이라는 확정 판정은 하지 않는다.
3. **UI 안내 겹침:** 도착·복귀 창(z400)이 저배터리 안내(z300)를 덮어 낮은 안내 확인 버튼이 실제 hit-test에 실패했다. 강제 DOM 클릭은 하지 않았다. 위에 보이는 정상 도착·복귀 버튼은 실제 pointer로 눌렀고, 현재 미션의 usage_complete 프레임을 확인했다. [화면](battery_step/02_notice_click_blocked.png), [원인](battery_step/ui_blocked.json), [복귀 명령](battery_step/return_action.json).
4. **태그 공간 계약:** 기존 태그는 현재 주차 기준의 후방카메라 화각 밖이었다. 별도 사본에서 목표 ROS XY `(-8.441868,39.372166)`, UE yaw +6° 회전으로 교정했다. 태그 ID·크기·재질·높이·기울기는 유지했다. [정확한 변경과 해시](tag_alignment/README.md).
5. **CARLA 주차 연결 미호환:** 새로운 순수 `camrod_control/launch/parking.launch.py`는 기존 virtual 런치의 `runtime_override_parameter_file`을 받지 않아 CARLA 전용 튜닝이 누락된다. 새로운 native AprilTag 제어기에는 기존 virtual의 `RETRY_FORWARD_EXIT` 구현도 없다. 실제 파라미터는 [저장본](tag_alignment_after/live_apriltag_parameters.yaml) 참조. 실제 로봇용 코드의3cm 기준을 높여 우회하지 않았다.
6. **충전/짐 작업 위치 충돌:** 현재 태그의 최종 도킹 목표가 정상 station과 거의 같아, 기록된 station 거리0.151m는 cargo 이동의 최소0.30m 조건에 못 미친다. 충전 위치만0.5m 뒤로 옮기면 기존 contact emulator의 같은 station·반경0.35m 조건과 충돌한다. 두 앵커를 일관되게 분리하는 시험용 연결 수정이 필요하며 아직 적용하지 않았다.
7. **초기 정지 상태 수동 복귀:** 별도 미션 없이 station에서 `/ui/manual_return`을 요청하면 lanelet/차량 거리2.13m가 한계1.50m를 넘어 경로 대기에 머물렀다. 이 요청을 정상 Stop으로 종료하고 실제 B7 왕복을 사용했다. 경계 한계를 늘리거나 가짜 도착 상태를 발행하지 않았다.

## 자료의 의미

- `battery_live_01.jsonl`, `battery_live_02.jsonl`: 실제 ROS 입력·원본 stamp·수신 경과시간·읽기 전용 API 관측. recorder는 PASS를 추정하지 않는다.
- `battery_step/`: 실제 UI pointer 명령·단계별 화면·관측. 내부 상태를 화면으로 재구성한 fixture가 아니다.
- `clock_tf_check_02.json`: 원시 CARLA 센서는 sim clock, canonical relay/grid는 wall clock이라는 실제 시계·TF 계약 확인.
- `sensor_probe.py`: 실제 actor 거리별 센서 입력의 **shadow grid 비교** 도구. 그 결과만으로 물리 제동거리나 독립 final-gate 모드 시험 PASS를 선언하지 않는다.
- `nearfield_actual_01.jsonl`: 60초 기준 관찰 후 지면 확인 실패로 근접 배치 전 중단된 원본. [기준 관찰 PNG](baseline_results.png)는 유효/관찰불가 표본을 모두 구분한다.
- `nearfield_actual_02.jsonl`: 실제10/20/30/40cm·도로변 배치 원본. bbox 간격과 실제 센서 반사 거리, native guard 경계를 구분한다. native body-front0.70837m와 CARLA bbox-front약0.76347m 차이도 숨기지 않는다.
- [거리별 실제 관측 비교 PNG](nearfield_actual_02_comparison.png), [CSV](nearfield_actual_02_comparison.csv), [전체 수치·해시 JSON](nearfield_actual_02_results.json). H/N/U는 점유비용 HIT / NO_HIT / 관찰불가이며, NO_HIT를 물리적 안전·자유공간으로 해석하지 않는다.
- 거리별25표본에서10cm는 radar H0/N23/U2, guard H24/N0/U1, both H22/N0/U3이었다.20cm·30cm·40cm에서는 유효 표본의 점유 HIT가 모두0이었다. 실제 전방radar 반사 거리는 bbox10cm 배치에서도 약0.39–0.41m로, 설정30cm와 차체 기준30cm가 같지 않음을 관측했다.
- 무시험 actor 기준60초와 도로변2조건은 정상 유지 구간에서 점유 HIT0이었다.10cm actor 제거 직후 가드의 일시 HIT1표본도 결과에서 지우지 않았다. 그리드 원래 unknown 배경과 TF 대기/관찰불가는 별도 집계했다.
- 가까운 장애물을 놓은 동안 ego는 도킹 오류로 이미 FAULT_HOLD였으므로 실제 감속·제동 시험으로 집계하지 않는다. 실제 카메라 사진은 [도로변 오른쪽 1장](nearfield_actual_02_camera_roadside_right_front.png)만 신선하게 보존됐고, 이 사진의 인물은 카메라 화각 밖이다. 다른 거리 사진이나 눈에 보이는 인물 검출 증빙으로 재사용하지 않는다.
- GIF는 저장된 실제 단계 PNG의 시간 압축 몽타주다. 프레임 간 시간 간격이 다르며 연속 실시간 녹화·물리 이동 궤적 증빙으로 바꿔 말하지 않는다.
- 실행 로그는 `/home/hong/camrod_service_test_ws/log/service_usability_*`에 보존한다.

추가 CARLA 전용 도킹 재정렬·충전/대기 앵커 분리 수정은 사용자에게 별도 진행 방향을 질문했다. 현재 unit1/main·unit2/main으로 커밋·push하지 않았다.
실물 충전 전원 차단·케이블 분리·실물 제동 안전성은 이 시험 범위가 아니다. 앞 단계가 실패한 후속 항목은 미검증으로 유지한다.

## 시각자료 바로 보기

| 시험 | 실제 화면/비교 | 정량 근거 |
| --- | --- | --- |
| 34% 호출 차단 | [Guest PNG](guest_soc34.png) — 초기 정지 overlay 조건 주의 | [backend 응답·미션 번호 불변](recall_soc34_backend.json), [Guest DOM](guest_soc34.json) |
| B7 복귀·자동 도킹 선택, 태그 없음 | [단계 GIF](battery_step/phase_progress.gif), [실패 PNG](battery_step/04_apriltag_timeout.png) | [결과](battery_step/result.json) |
| 태그 위치 교정 | [후방영상 수정 전후 GIF](tag_alignment/tag_camera_before_after.gif), [실제 ID3 검출 PNG](tag_alignment_after/detector_debug.png) | [APPLIED·원본 불변 설명](tag_alignment/README.md), [검출 메시지](tag_alignment_after/capture.json) |
| 교정 후 실제 자동 도킹 | [단계 GIF](battery_aligned_step/phase_progress.gif), [최종 실패 PNG](battery_aligned_step/failure.png) | [0.045m > 0.030m 오차 판정](battery_aligned_step/result.json) |
| 근거리·도로변 센서 처리 | [실제 관측 비교 PNG](nearfield_actual_02_comparison.png), [무시험 actor 기준 PNG](baseline_results.png) | [거리별 CSV](nearfield_actual_02_comparison.csv), [전체 JSON](nearfield_actual_02_results.json) |
| 35% 충전 출차·후속 미션 | **앞 단계 실패로 실제 영상 없음.** 기존 UI fixture를 실제 주행 증빙으로 넣지 않는다. | 미검증 |

자료 경로는 커밋·push 전의 로컬 시험 작업 공간이다. 원본/수정 후·실패/성공·실제 주행/소프트웨어 fixture 범위를 서로 바꾸어 표시하지 않는다.
