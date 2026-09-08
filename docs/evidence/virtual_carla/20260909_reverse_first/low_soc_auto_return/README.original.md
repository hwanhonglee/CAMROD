# 저SOC 자동 복귀 시험 — 독립 검증

결과: **PASS**. 원본 결과·실행 helper·PNG/GIF는 수정하거나 다시 실행하지 않았습니다.
독립 판정과 원본 해시는 [independent_validation.json](independent_validation.json)에 있습니다.

- 실제 UI B1 배송 출발 후 DropZone에서 5.008802 m 떨어진 주행 중 CARLA SOC만 24%로 한 번 입력했습니다.
- 입력 monotonic 1081859.592049248 → 자동 `battery_urgent_return` 수신 1081860.493693799: 0.901645초 뒤입니다. 출처의 B1·임무 generation·Return token을 검증했습니다.
- 제어 토픽의 같은 wall-clock 영역 안에서 reverse attempt 13 → 실제 비충전 PARKED → April attempt 14 → 태그 유도 → PARKED/CHARGING을 확인했습니다.
- 시간 110.283995초, CARLA odometry 거리 16.339463 m. 충전 완료 후 정지(3.10161465058e-08 m/s)·임무 비활성·engage 해제를 확인하고 SOC 80%를 복원했으며 실제 피드백을 받았습니다.
- 원본 PNG/GIF 16개와 desktop checksum 파일을 대조했습니다.

## 원본 검증기 한계와 사진 선택

원본 helper의 low_ns는 CARLA simulation odometry(46516597887316)이고, 제어 토픽은 wall-clock(1788887646590279071)입니다. 서로 직접 비교한 원본 cutoff는 유효하지 않으므로 이 독립 검증에서는 사용하지 않았습니다. 입력→요청 인과관계는 동일 프로세스 monotonic으로, 제어 순서는 제어 토픽끼리만 비교했습니다. 태그/RGB는 실제 bridge-restamped 센서 source stamp끼리 비교했으며 raw CARLA odometry와 비교하지 않았습니다.

`functional/04_actual_reverse_parked_*`는 17:14:07 UTC에 촬영돼, 이번 임무의 실제 reverse PARKED(17:14:55 UTC)보다 이릅니다. **그 이름을 신뢰해 주차 완료 사진으로 사용하면 안 됩니다.** 원본은 보존했으며 완료 입증에는 새로운 제어 이벤트와 이후 화면만 사용합니다.

확인할 자료: [전체 실제 화면 PNG](desktop/representative_contact_sheet.png), [실제 주행 GIF](desktop/representative_motion.gif), [April 유도 중 Robot UI](functional/05_apriltag_approach_robot.png), [해당 후방 RGB](functional/05_apriltag_approach_rear_rgb.png), [충전 완료 Robot UI](functional/06_actual_charging_complete_robot.png), [충전 완료 후방 RGB](functional/06_actual_charging_complete_rear_rgb.png).

dummy flag는 미수신(null)을 그대로 유지했습니다. 충전은 CARLA contact emulation이며 실물 충전기 검증이 아닙니다. 25–35% 구간의 사용자 확인 및 전 사이트 시나리오 완료를 주장하지 않습니다. SOC 복원 직전의 독립 odometry receipt 값은 별도로 직렬화되지 않았으므로, 가까운 실제 로그의 증가한 sample count·정지 속도·당시 UI 상태를 대조했습니다.

재검증 코드: [validate_low_soc_run.py](../validate_low_soc_run.py). 결과 원본: [functional/result.json](functional/result.json).
