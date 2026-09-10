# v2.2.5 순수 코드 집중 검증 — 2026-09-08

제품 소스 기준 `91158175102a2bbf86624f5c5b726e5e272f8b90`.
이후 릴리스 정리 커밋은 문서·검사 원본 추가이며 아래 제품 소스의 동작을 바꾸지 않는다.
Ubuntu 22.04 / ROS 2 Humble / Python 3.10 / Node 18.20.8 / GCC 11.4에서 실행했다.
기존 workspace install은 메시지·라이브러리 의존성으로 사용했다. 모든 의존성을 새로 빌드한 결과가 아니다.

| 검사 | 결과 | 원본 |
| --- | --- | --- |
| PURE UI 전체 | 364 PASS, 4.353s | [ui.junit.xml](ui.junit.xml) |
| 지정 bringup/localization/planning 계약 12개 파일 | 119 PASS, 16.498s | [contracts.junit.xml](contracts.junit.xml) |
| 새로 빌드한 실제 후진 주차 controller | 16 PASS | [native_reverse_parking_controller.junit.xml](native_reverse_parking_controller.junit.xml) |
| 새로 빌드한 reverse XY 완료 helper | 4 PASS | [native_reverse_parking_completion.junit.xml](native_reverse_parking_completion.junit.xml) |
| 새로 빌드한 GNSS/heading/주차 접근 수학 | 15 PASS | [native_gnss_center_scope.junit.xml](native_gnss_center_scope.junit.xml) |
| 소스·설정·바이너리 동일성을 확인한 종점 경로 재실행 | 17 PASS, 14.789s | [native_terminal_path_runtime.junit.xml](native_terminal_path_runtime.junit.xml) |

모든 원본의 failure/error/skip/disabled는 0이다. **실행 범위가 겹치므로 고유 535개 기능 성공으로 합산하지 않는다.**
추가 집중 검사 52개 중 C++ GoogleTest는 35개이며, 나머지 17개는 실제 extractor 8개와 Python 관측 회귀 9개다.

## 출처와 격리

- UI는 PURE `camrod_ui/runtime/python`을 우선 사용하며 `camrod_ui/test` 전체를 실행했다.
- 계약 119개는 주차 launch scope, 설정 mirror, drop-zone departure, GNSS mount,
  robot center, AprilTag docking contract, Guest wait/turnaround, Nav2 profile,
  GNSS heading cache, recall policy, planning coalescing/terminal-path 검사다.
- 계약에 포함된 extractor 8개는 기존 설치 바이너리
  `933e52bf79d92e888d146d5468e93602ef707a94fd2478634af236a6d8a4429f`를 사용했다.
  이를 새 PURE 빌드로 표시하지 않는다.
- 별도 17개 재실행은 보존된 순수 소스 빌드 바이너리와 C++/YAML/검사 SHA가 일치하는지 확인했다.
  새 reverse/GNSS 빌드와 이 재사용의 정확한 차이는 [native_verification_summary.json](native_verification_summary.json)에 있다.
- 쉘 기본 DDS domain 189 / localhost 전용. 기존 fixture는 reverse 188,
  planning 226 및 별도 test 토픽을 명시한다. 운영 도메인에 연결하거나 UI/주행 명령을 보내지 않았다.

## 재실행

저장소 루트에서 기존 ROS 의존성을 준비하고 다음과 같이 UI를 검사할 수 있다.

```bash
source /opt/ros/humble/setup.bash
# 필요한 avg_msgs 등은 대상 workspace의 install/local_setup.bash로 준비한다.
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=189
export PYTHONPATH="$PWD/camrod_ui/runtime/python:${PYTHONPATH:-}"
python3 -m pytest -q camrod_ui/test
```

네이티브 정확한 빌드·검사 명령, 실행 파일 및 원본 JUnit SHA는
[native_verification_summary.json](native_verification_summary.json)의 `executed_commands`,
`sources`, `binaries`, `artifacts`에 기록했다. 다른 장비에서는 체크아웃·빌드 경로를 그 장비에 맞춘다.

## 이 자료로 증명하지 않는 것

전체 B1–B13 실주행, 선택 도킹 완료, 모든 출차 초기 상태, 실차 전방 GNSS 장착 보정,
90° 회전 후 XY 재정렬, 모든 lanelet 경계 조건, 센서·음성·충전기·ARM64 현장 인증은 아니다.
검사 입력·수학 회귀에 대한 결과를 실제 주행 PNG/GIF로 대체하지 않는다.
