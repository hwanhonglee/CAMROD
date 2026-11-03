# 게시 구성 재검증 — 2026-11-03

<!-- HH_261103 - Distinguish a freshly built publication overlay from the earlier actual CARLA mission recordings. -->

순수 릴리스 `unit2-v1.0.1`의 commit
`9c261f9df15abffb86fbff12d334efbb4c2663f3`과 CARLA 구현 commit
`1552d551a242d973ae26450feb58c221bcd303f6`를 사용했다.
검증한 `camrod_carla_service_test` Git tree는
`4259f91660c2c0060e7394275d9305e000e48707`이다.

실제 `setup_overlay.sh`로 고정 태그의 추적 파일을 새 작업공간에 추출한 뒤,
parking 연결·허용된 dependency underlay 패치를 검사·적용했다. 실제
`overlay.sh ... build`가 5개 패키지를 새로 빌드·설치했고 3분 39초에 완료됐다.
순수 태그나 기존 가상환경 작업공간을 checkout/reset/덮어쓰기 하지 않았다.

| 확인 | 결과·자료 |
| --- | --- |
| CARLA 도킹 호환 패키지 | 6/6 CTest 통과 [XML](compat_ctest.xml) |
| native cargo·35% 경계 | 3/3 CTest 통과 [XML](native_cargo_ctest.xml) |
| 실제 모델·도색 등 UI 자산 | 58개 설치 확인 [기록](ui_assets.log) |
| helper 환경 검사 | 5개 새 prefix, 기존 virtual bringup·adapter 유지 [경로 기록](package_prefixes.txt) |
| 전체 launch 인자 구성 | `ros2 launch ... --show-args` 통과, 서버·차량 동작 없음 |

도킹 제어기 fixture는 localhost domain215를 사용했다. 운영 중 CARLA domain188에
명령을 발행하거나 미션을 실행하지 않았다. 테스트 수 세부는 [원본 집계](test_counts.txt)에 있다.

이 검사는 설치·컴파일·제어기 fixture·연결 구성의 검증이다. 앞서 기록한 실제
B7 주행·충전·출차·호출 영상과 구분한다. 새 overlay 바이너리의 물리 게이트를
만들었다고 주장하지 않으며, 새 호스트의 실행 전에는 공식 pipeline 도구로
그 호스트·소스·바이너리에 결합된 게이트를 다시 만들어야 한다.
ARM/실물·충전 전기적 분리·실제 제동·20–30cm 보호 성능은 이 검사의 범위 밖이다.
