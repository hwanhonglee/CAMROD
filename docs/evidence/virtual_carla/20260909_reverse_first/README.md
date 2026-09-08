# 2026-09-09 후진 주차·도킹·충전 출차 실제 증거

실제 실행 runtime은 **`f02c2fb512a25565c676b5103d780635ae0892f9`**, CARLA 차량은 actor **98**입니다. 이후 순수 CAMROD 승격·문서·증거 커밋을 이미 실행한 runtime으로 재표기하지 않습니다. 아래 성공은 지정한 실제 실행에 한정하며 전 구역·모든 recall·실차 시험 완료가 아닙니다.

## 실제 결과와 바로 볼 자료

| 실제 시험 | 확인 결과 | PNG / GIF | 수치·판정 원본 |
| --- | --- | --- | --- |
| 충전 상태 → B1 배송 → UI 복귀 → 비충전 후진 주차 | native 및 독립 strict PASS. 715.044초, 168.762416m | [PNG](b1_charging_departure/visual/representative_contact_sheet.png) · [GIF](b1_charging_departure/visual/representative_motion.gif) | [native](b1_charging_departure/native.json) · [strict](b1_charging_departure/strict.json) · [metrics](b1_charging_departure/metrics.json) · [run](b1_charging_departure/run_manifest.json) · [site](b1_charging_departure/site_manifest.json) |
| 실제 후진 PARKED → 선택 Dock → 실제 태그 접근 → 충전 피드백 | 기능 PASS, 29.95초. 실제 UI 서비스 메뉴·확인을 통해 요청 | [PNG](optional_docking/desktop/representative_contact_sheet.png) · [GIF](optional_docking/desktop/representative_motion.gif) | [기능 결과](optional_docking/functional/result.json) · [사용자 확인 화면](optional_docking/functional/01b_service_docking_confirmation.png) · [실제 태그](optional_docking/functional/03_actual_rear_tag_detected.png) · [충전 UI](optional_docking/functional/05_ui_charging_complete.png) |
| B1 출차 중 SOC 24% 시험 입력 → 자발적 긴급 Return → 후진 주차 → 자동 Dock | 기능 및 독립 시간·세대·소유권 검증 PASS. 110.283995초, 16.339463m. 안전 정지 확인 후 SOC 80% 복원 | [검토된 PNG](low_soc_auto_return/reviewed_visuals/actual_contact_sheet_5_panels.png) · [검토된 GIF](low_soc_auto_return/reviewed_visuals/actual_motion_preparation_excluded.gif) | [기능 결과](low_soc_auto_return/functional/result.json) · [독립 검증](low_soc_auto_return/independent_validation.json) · [파생 영상 기록](low_soc_auto_return/reviewed_visuals/derivation_manifest.json) |

GIF는 실제 녹화의 선택 구간 요약이며 전체 주행을 실시간 길이로 재생하는 영상은 아닙니다. 수치의 기능 시간과 녹화 정리까지 포함한 wrapper 시간은 다릅니다. [집중 시험 상태 원본](focused_status.original.json)에는 단계별 완료 이력이 남아 있습니다.

### 실제 센서와 충전의 범위

- 선택 Dock [충전 후 실제 후방 RGB](optional_docking/functional/06_rear_camera_charging_complete.png).
- 저SOC 시험 [실제 태그 후방 RGB](low_soc_auto_return/functional/05_actual_tag_rgb_rear_rgb.png), [충전 후 후방 RGB](low_soc_auto_return/functional/06_actual_charging_complete_rear_rgb.png), [배터리 24%에서 Charging UI](low_soc_auto_return/functional/06_actual_charging_complete_robot.png).
- 충전은 CARLA의 실제 위치·속도·도킹 상태·유지시간을 보는 **contact emulator 피드백**입니다. 실물 충전기의 전력 전달을 검증한 것은 아닙니다. SOC 0.24는 승인된 CARLA 전용 시험 입력이며 자연 방전 시험이 아닙니다. 카메라·태그·차량 위치·충전 완료를 시험 코드가 직접 주입한 결과가 아닙니다.

### 저SOC 증거의 시간 기준 정정

원래 helper에는 CARLA odometry의 sim-time stamp와 제어 wall-clock stamp를 직접 비교한 한계가 있습니다. 원본을 수정하지 않고 [독립 검증](low_soc_auto_return/independent_validation.json)에서 동일 수신 monotonic 기준의 SOC 입력 → 0.901645초 뒤 긴급 Return, 신선한 mission generation/token, 동일 제어 clock의 reverse attempt 13 → 비충전 PARKED → April attempt 14 → CHARGING 순서를 다시 확인했습니다.

`04_actual_reverse_parked_*`는 실제 새 주차 완료보다 이르게 찍힌 사진이므로 이번 묶음에 **포함하지 않았습니다**. 실제 새 제어 이벤트를 완료 증거로 사용합니다. reviewed PNG/GIF는 원래 녹화에서 VS Code에 가린 준비 구간만 제외한 기존 파생본이며 차량·UI·문구를 합성하거나 고친 영상이 아닙니다. [설명 원본](low_soc_auto_return/README.original.md), [영상 설명 원본](low_soc_auto_return/reviewed_visuals/README.original.md), [당시 독립 검증기 사본](tools/validate_low_soc_run.py)은 출처 보존용이며 생략된 전체 원시 자료 없이 독립 재실행 가능한 패키지라는 뜻은 아닙니다.

## GNSS 중심 보정: 실제 측정과 아직 안 한 시험

[실제 측정 PNG](gnss/first90/first90_of_178deg_actual.png) · [측정 JSON](gnss/first90/first90_measurement.json) · [선정 방법·해석](gnss/first90/README.md)

기존 −178.176376° 연속 회전의 첫 **−90.416908° / 4.509528초** 부분구간입니다. 원시 안테나 변위 62.438cm, 실제 차체 중심 변위 4.179cm, 보정 중심 변위 3.191cm, 보정−truth 오차벡터 변화 1.352cm를 구분했습니다. **새로운 90°/180° 명령 후 정지 시험은 아직 실행하지 않았습니다.** 현재 측면 장착 `(X≈0, Y=+0.45m)`의 관측이며 실차 전방 안테나 치수·성능 인증도 아닙니다.

[GNSS·TF 설정 휴대용 설명](gnss/GNSS_TF_CONFIGURATION.md)은 저장소 상대 링크를 사용합니다. [원본 문서](gnss/GNSS_TF_CONFIGURATION.original.md)는 바이트 그대로 보존하여 당시 절대 경로·파라미터·runtime 출처를 유지했습니다. 외부 CARLA ROS bridge 설치 경로는 저장소에 없는 의존성이므로 휴대용 문서에서 원래 설치 경로로 명시합니다.

## 무결성과 배포 범위

[manifest.json](manifest.json)에 36개 원본 파일의 source 절대 경로·복사 대상 상대 경로·bytes·SHA256 및 14개 PNG/GIF의 전체 프레임 decode 결과를 기록했습니다. 원본 복사량은 99,589,107 bytes(약 95MiB), 최대 단일 파일은 약 22MB입니다. 요청한 세 PNG/GIF 쌍만으로 50MB를 넘으므로 원본 보존을 우선했습니다. 복사본의 bytes/SHA가 원본과 모두 같고 선택된 원본 JSON의 파일 참조 26건도 일치했습니다.

단, strict 검증은 run manifest의 `VALIDATING` 시점 해시를 기록하고 runner가 나중에 같은 run을 `PASS`로 마감합니다. 이 기존 생명주기에 따른 **run hash 1건 차이**는 manifest에 expected/current 해시를 모두 보존했으며, 과거 파일을 만들어 같은 해시인 것처럼 꾸미지 않았습니다. native/site/미디어를 임의로 바꾸거나 이 차이로 주행 결과를 다시 판정하지 않았습니다.

대용량 raw wheel JSONL·오디오·원본 비디오, 이전 실패의 중복 영상은 제외했습니다. 따라서 원본 JSON이 가리키는 모든 외부 파일이 이 작은 묶음에 있다는 뜻은 아닙니다. 25~35% 임무 완료 후 복귀 확인, 전 구역 B1~B13, 최신 모든 Guest/Robot recall, 실차 배터리·충전기 및 별도 90°/180° 정지 시험은 추가 검증 범위입니다.
