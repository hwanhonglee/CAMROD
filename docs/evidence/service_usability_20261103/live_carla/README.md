# 2026-11-03 CARLA 서비스 재현·증빙 목록

> 게시본 실행 안내: 이 문서의 `runtime.sh`와 호스트 경로는 실제 시험 당시의
> 기록이다. 새 실행은 [고정 태그 시험 구성](../../../../camrod_carla_service_test/README.md)을
> 사용한다. [Git에 포함된 자료·압축 원본 범위](../PUBLICATION.md)도 구분해서 확인한다.

<!-- HH_261103 - Separate frozen build/scene proof from live outcomes; never infer a completed mission from fixtures. -->

## 범위와 고정 기준

이 자료는 `/home/hong/camrod_service_test_ws`의 명시적 CARLA 시험이다. 실물
unit1/unit2 배포·충전기·CAN·제동거리 검증으로 집계하지 않는다. B7 04 주기의
자동 도킹은 통과했고, 이어진 짐 작업 출차에서는 실제 오류를 재현했다.
그 오류를 수정한 05 재시험과 이후 35% B7 호출·복귀를 완료했다.
근거리 센서 비교는 수행했지만 20–30 cm 기대 반응은 미해결이며 제동 PASS가 아니다.
커밋·push하지 않았다.

- 물리 게이트: `fc276c77a3129da445b0670b91199a29af214c198d0ac96adf0580ac6240a7d1`.
  [게이트·결합 파일 영수증](provenance/physical_gate_initial.json).
  게이트 통과는 새 장면의 도킹·짐 작업 성공 판정이 아니다.
- 새 맵 URI: `/Game/map_package/Maps/Woraksan_v27_charge_anchor_20261103/Woraksan_v27_charge_anchor_20261103`.
  `.umap` SHA256: `7e9b655b9bff3bda581ea04bd321b442df26fa61f77915151259038b50574e87`.
- 장면 영수증: [APPLIED 원본](provenance/tag_scene_apply.json),
  스키마 `camrod.service_charging_geometry.stage.v1`. 태그 46조각만 평행이동했다.
  기존 V224·Nov2 사본, 비태그 계약·지형·522개 보호 자산과 native station 파일은 불변이다.

## 수정 범위와 위치 계약

[시험 패키지](../../../../camrod_carla_service_test/README.md)는 기존 virtual AprilTag
제어기와 필요한 helper를 별도로 보존하며, [출처 해시](../../../../camrod_carla_service_test/PROVENANCE.json)를 기록한다.
`service_test.launch.py`에서만 시험 제어기·재시도 YAML·별도 contact YAML을 선택한다.
native dispatcher·reverse parking·cargo 제어기는 유지한다.

native `camrod_control/launch/parking.launch.py`에는 선택적인 제어기 package/executable와
마지막 sparse YAML 연결만 추가했다. 기본 선택은 기존 native 제어기와 빈 override다.
시험은 최대 2회·0.8 m 전진 재획득을 명시적으로 사용하되, 최종 횡오차 **0.03 m**와
태그 정지 거리 **0.40 m**, freshness·진행량·편류·경로·시간 한계는 완화하지 않는다.
실제 연결값은 [live 파라미터](live_apriltag_parameters.yaml)에서 확인한다.

| 기준 | ROS map XY (m) | 용도 |
| --- | --- | --- |
| 기존 정상 station | `(-8.47366, 40.391)` | native 주차·cargo 목표, 변경 없음 |
| 시험 충전 body anchor | `(-8.4549464735, 39.7912919011)` | body yaw `+91.7873°` 기준 0.60 m 뒤 |

[형상·접촉 계약](../../../../camrod_carla_service_test/CHARGING_GEOMETRY.md): 실제 canonical
planning boundary 30점의 반경은 `0.9465604801 m`; native swept-body 검사가 통과했고
drop zone 경계의 최소 잔여 여유는 `0.36992 m`다. cargo의 0.20 m 조기 정차 허용까지
고려해도 명목 충전 anchor와 최소 0.40 m 떨어진다. contact emulator의 기존 반경
0.35 m·정지속도 0.05 m/s·신선한 선택 AprilTag 완료 상태·1초 dwell은 그대로다.
이 신호는 시뮬레이터의 접촉 판단이며 실물 전원 차단·기계적 분리 증거가 아니다.

## 재현 진입점과 저장 분리

[runtime.sh](runtime.sh)는 domain 188·localhost, 별도 state/mission 저장소,
NEW control/sensing/UI/ground segmentation/시험 패키지와 OLD virtual bringup/adapter
경로를 검사한다. `CAMROD_SERVICE_TAG_SCENE=1`일 때 위 영수증과 맵 해시를 검사하고
새 URI를 선택한다. Nov2 맵으로 이름만 바꿔 실행하지 않는다.

서버·bridge·pacer·spawn·주행은 루트 에이전트가 단일 소유권으로 관리한다.
실제 주행과 CARLA SOC 입력은 아래 각 결과의 UTC·배차 generation으로 구분한다.
기록은 이 디렉터리, state는
`/home/hong/camrod_service_test_ws/state/service_usability_live_20261103`로 분리한다.
SOC 시험 입력은 CARLA heartbeat 대상이며 실제 BMS 변화가 아니다.

## 완료된 소프트웨어·형상 확인

| 확인 | 결과 | 원본 |
| --- | --- | --- |
| native control CTest | 13/13 통과 | [XML](native_control_ctest.xml) |
| 시험 도킹 호환·형상 CTest | 5/5 통과 | [XML](docking_compat_ctest.xml) |
| 새 tag-only clone | APPLIED, editor exit 0, 원본 불변 | 위 장면 영수증 |

이 표는 실제 왕복·도킹·접촉·짐 작업 PASS를 대신하지 않는다.

## 실제 시험 행렬

| 시나리오 | 상태 | 실제 자료·남은 확인 |
| --- | --- | --- |
| B7 배차·현장 진입·왕복 | PASS (04) | [배차 PNG](01_b7_dispatch.png), [실제 화면](02_desktop_b7.png), [원본 관측](battery_live_04.jsonl.gz) |
| 도착 후 SOC34 → 정상 복귀·자동 AprilTag 선택 | PASS (04) | [SOC 입력](battery_cycle_04/soc_stimulus.json), [복귀 요청](battery_cycle_04/return_action.json), [관측](battery_cycle_04/observations.jsonl.gz) |
| 새 tag 장면에서 최종 도킹·접촉 | PASS (04) | [결과](battery_cycle_04/result.json), [충전 화면](03_charging_soc34.png), 실제 녹화（원본 MP4는 실행 호스트에 보존; PUBLICATION.md 참조） |
| SOC34 호출 차단 | PASS | [HTTP400·배차 불변](recall_soc34_after_dock.json), [Guest 화면](guest_soc34_after_dock.png), [13개 사이트·호출 비활성 관측](guest_soc34_after_dock.json) |
| SOC35 충전 위치 → 정상 station에서 짐 작업 정차 | FAIL (04) → PASS (05) | [수정 전 실패](cargo_exit_04/result.json), [수정 후 실측](cargo_exit_05/result.json), [정차 PNG](cargo_exit_05/02_ready.png), [단계 GIF](cargo_exit_05/phase_progress.gif) |
| 짐 작업 후 SOC35 새 호출·복귀·후진 주차 | PASS (06) | [실제 배차](b7_recall_dispatch_06.json), [완료 확인 2회·최종 결과](recall_cycle_06/result.json), [최종 PNG](recall_cycle_06/14_final_reverse_parked.png), [실제 단계 GIF](recall_cycle_06/phase_progress.gif) |
| 실제 센서 근거리·도로변 비교 | 수행 완료, 기대 반응 일부 미해결 | [실측 표·사진·한계](sensors/sensor_actual03_results.md), [비교 PNG](sensors/sensor_actual02_vs_actual03.png), [각 배치 실제 PNG](sensors/nearfield_actual_03_photos.png), [사진 GIF](sensors/nearfield_actual_03_photos.gif). Radar/Guard/Both는 입력 shadow 비교이며 주행 제동 비교가 아님 |

[Nov2 실패·수정 전후 자료](../../service_usability_20261102/live_carla/README.md)는 삭제하지 않는다.
태그 화각 밖 timeout, 위치 교정 후 `0.045 m > 0.030 m` 안전정지, overlay/retry
미연결, 충전/cargo anchor 충돌을 새 주기의 성공으로 덮어쓰지 않는다. PNG/GIF와
원본 stamp·관측 로그를 연결해 실제 영상·소프트웨어 fixture·추정 도식을 구분한다.

## 실제 결과 — 루트 에이전트 작성 영역

<!-- ROOT_ACTUAL_RESULTS_BEGIN -->
04 자동 도킹: 정상 복귀 → reverse 주차 → 강제 Dock 없이 AprilTag →
WAITING_FOR_CHARGING → 정차한 CHARGING을 2초 이상 관측했다. 실제 정지 시
태그 거리 0.392 m, 횡오차 −0.011 m로 기존 0.030 m 한계를 통과했다.
최종 body pose는 약 `(-8.4676545,39.6776213)`이고, contact emulator의 충전
anchor 거리 0.115 m·실측 정지·선택 AprilTag 완료·dwell 조건으로 신호가 발생했다.
재시도 기능은 복원·단위시험했으나 이 주기에서는 실제 재시도가 필요하지 않았다.

04 출차 실패: 35% 요청 수락 뒤 7초 대기 → 자기 parking CANCEL → 시험 contact
신호 해제 → 출발 음성 완료/timeout callback이 초기 충전 조건을 다시 요구했다.
그 결과 EXIT 명령 전 취소됐으며 **로봇 이동은 없었다**. 실패 자료는 보존한다.
시험 작업공간의 backend만 수정해 초기 admission은 충전 중을 요구하되,
자기 handoff 이후 callback은 정상적인 contact 해제를 허용한다. 신선한 차량 상태,
35%·정차·영역·소유권·취소 검사는 유지한다. [실제 회귀시험 XML](cargo_fix_tests.xml)
245개 통과, source/install 일치, frontend 변경·재빌드 없음.

05 수정본 게이트:
`0c28760c6694d61b51e8378919b8b1ed1a2ed04c0fdddf661dfdcdc752201b82`.
[수정본 영수증](provenance/physical_gate_cargo_release.json).
원래 04 영수증을 덮어쓰지 않았다. 실행 시
`CAMROD_SERVICE_GATE_REVISION=20261103_cargo_release CAMROD_SERVICE_TAG_SCENE=1`
환경으로 이 폴더의 runtime.sh를 사용한다. 실제 로봇 branch·배포 파일은 변경하지 않았다.

05 재시험: B7 배송 도착 → SOC34 입력 → 정상 복귀 → 후진 주차 → 자동 AprilTag
도킹 → 실제 contact emulator 충전 신호를 재관측했다. 태그 정지 거리 0.398 m,
횡오차 +0.004 m. 이후 SOC35를 한 번 입력하고 실제 충전 화면의 앞으로 이동·확인
버튼을 눌렀다. 29.37초 전체 절차(안전 대기·출발 안내 대기 포함) 후, 초기 body
방향으로 0.51517 m 전진·측방 편차 0.00143 m·yaw 변화 −0.000848 rad로 정차했다.
원래 station까지 잔여 0.19140 m는 그대로 둔 0.20 m 완료 허용범위 안이다.
`cargo_access_ready=true`, fresh platform feedback, 충전 false·실제 정지를 2초 이상
확인했으며 도로로 나가거나 새 미션을 자동 생성하지 않았다.

06 후속 실제 호출: 짐 작업 정차 후 배터리 35%를 유지하고 Robot UI에서 B7 recall을
새 배차했다(generation `1793693264276002`, owner `robot`). 사이트 도로변 도착의
첫 완료 → 사이트 진입·180° 회전 → 최종 완료 → 복귀 → reverse PARKED가 이어졌다.
완료 요청은 실제 화면 클릭 2회이며 SOC 변경·강제 Dock·워프·상태 주입은 없었다.
08:23:35 UTC에 새 미션 없음·DROP_ZONE_WAIT·35%·충전 false·정차를 최종 확인했다.

센서 03: 60초 무장애 구간과 도로변 좌/우 배치에서 지속 HIT는 없었고, 10 cm
보행자 bbox 배치의 LiDAR guard는 25/25 유효 관측에서 HIT였다. Radar는 10 cm에도
HIT가 없었으며, 20/30/40 cm에서는 guard와 radar 모두 정지용 HIT가 없었다.
누락/지연은 별도 U로 유지했다. bbox 간격은 실제 표면·센서 거리와 같지 않으므로
이것을 실물의 30 cm 미검출 확정 또는 정지거리 인증으로 바꾸지 않는다.
**기대한 20–30 cm 근접 반응과 실제 제동은 미검증/미해결이다.** 임의로 임계값을
확대하거나 가짜 clear/HIT를 만들지 않았다. 실제 사진 7장과 원본 센서·TF·격자
관측, 시험 객체 18–24 제거 확인을 남겼다.

종료 상태: CARLA·bridge·CAMROD/UI는 대기 상태로 남겨 두었고, 미션·SOC 주입기·
관측기·시험 장애물·녹화 프로세스는 종료됐다. 현재 실행은 시험 작업공간뿐이며
unit1/main·unit2/main에 commit/push/배포하지 않았다.
<!-- ROOT_ACTUAL_RESULTS_END -->

## 시각자료 — 실제 04번 기록만 사용

| 자료 | 내용과 한계 |
| --- | --- |
| [단계별 실제 PNG 요약 GIF](battery_cycle_04/phase_progress.gif) | 실제 PNG 15장, 촬영 UTC·phase·충전 feedback 표시. 원본 간격은 불균일하며 사진당 1.5초로 압축한 24초 요약이다. **연속 영상이 아니다.** 960px, 약1.4MB. |
| [실제 자동 도킹 연속 GIF](battery_cycle_04/autodock_continuous_100_160_768.gif) | 완료된 원본 MP4의 **+100~+160초**를 자른 60초·5fps·1배속 화면. 주차 접근→태그 후진→충전 화면 구간이며 전체 미션 영상이 아니다. 768px, 약10.6MB. |
| 원본 연속 MP4（원본 MP4는 실행 호스트에 보존; PUBLICATION.md 참조） | 1920×1080, 5fps, 360초. GIF는 축소·색상 양자화·반복 재생하므로 작은 글자나 세부 형상은 원본과 PNG에서 확인한다. |
| [생성 명령·원본/결과 해시 JSON](battery_cycle_04/visuals_manifest.json) | MP4 probe, 선택 offset, PNG 촬영 시각, ffmpeg 명령, 결과 프레임 수·용량·SHA256을 기록했다. |

MP4에는 creation-time 메타데이터가 없어 GIF의 상대 offset을 절대 UTC로
억지 환산하지 않았다. 단계 GIF의 UTC는 `result.json`의 실제 촬영 기록이다.
실제 UI·카메라 내용을 새로 그리거나 AI 이미지·05번 주기 자료를 섞지 않았다.
용량 제한을 넘은 첫 960px 파생 GIF는 최종 768px 파일 검증 후 제거했으며,
원본 MP4와 모든 PNG는 보존했다. 이 영상은 CARLA의 도킹·feedback 관측이며
실물 충전기·전원 차단·케이블 분리 또는 물리 제동거리 증빙이 아니다.

## 시각자료 — 실제 05번 짐 작업

[실제 전진·정차 단계 GIF](cargo_exit_05/phase_progress.gif)는 05번의
`01_before` → `phase_departing` → `phase_ready` → `02_ready` 원본 PNG 4장만
사용한다. 960px·약147KB·8.6초 요약이며 **연속 영상이 아니다**. 상태 관측 UTC와
원본 간 시간 공백을 표시했다. 시각은 screenshot 호출 직전 API 관측 기록이지
정밀한 카메라 노출 시각으로 간주하지 않는다.

[정확한 결과 원본](cargo_exit_05/result.json) ·
[생성 명령·PNG/GIF 해시·시각 매핑](cargo_exit_05/visuals_manifest.json).
`retest_05_desktop.mp4`가 최종 ready까지 모두 담았다고 주장하지 않는다.
이 GIF의 종료 상태는 실제 ready PNG와 결과·관측 로그로 연결한다.

## 시각자료 — 실제 06번 호출·복귀

[호출·복귀 GIF](recall_cycle_06/phase_progress.gif)는 실제 PNG 14장 중 핵심 6장을
사용한 비연속 단계 요약이다. 사진 UTC와 공백을 표시하며 두 완료 클릭·정차 판정은
[원본 결과](recall_cycle_06/result.json), [시각자료 해시](recall_cycle_06/visuals_manifest.json)로 확인한다.

## 시험 helper 재사용 조건

동작을 요청하는 watcher와 cargo helper는 이 날짜의 `runtime.sh ros`를 통해 실행한다.
domain188·localhost·simulation 표식을 확인하기 전에는 명령을 허용하지 않는다.
04/05 실제 출차 실행 당시에도 root가 그 조건과 실제 UI endpoint를 확인했으며,
재사용 시 실수 방지를 위해 이후 cargo helper에도 같은 환경 검사를 추가했다.
재실행은 새 output 디렉터리를 요구한다. 실행 중인 미션을 모르고 자동 재배차하거나,
기존 결과를 덮어쓰는 도구가 아니다.
