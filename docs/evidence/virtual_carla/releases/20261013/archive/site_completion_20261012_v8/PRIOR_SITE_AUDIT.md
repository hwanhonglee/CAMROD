# V8 이전 완료 12개 사이트의 독립 증거 재검증

<!-- HH_261012 - Read-only audit of finalized prior-version evidence; not a V8 all-site acceptance claim. -->

2026-10-12, V8 build 진행 중 지정된 **12개 완료 사례**를 실제 저장 파일에서 재검증했다. 요청된 검사에서 누락·불일치는 없었다. 구성은 **V5 9개 + V6 2개 + V7 1개**이며, **동일 V8 profile의 13개 전체 통과가 아니다. B10 V8 실제 결과는 이 감사 범위 밖이며 작성 당시 대기 중**이다. 과거 실패·중단 기록은 수정하지 않았다.

## 실제 수행한 검사

- 각 `trial.json`: `COMPLETED_OBSERVED`, `roundtrip_verified=true`, outbound/return 물리 거리 양수. 모든 `formal_acceptance=false`를 그대로 유지했다.
- 각 `supervisor.json`: `finished`, trial/capture 종료 코드 **0/0**.
- 최종 reverse 및 dispatcher 모두 `PARKED`, gate `STANDBY`. 최종 localization XY에서 station **(-8.47366, 40.391)**까지 `hypot(dx,dy)`를 직접 다시 계산했다. 모두 변경하지 않은 **0.25 m** 이하이며 범위는 **0.206369652–0.233476055 m**다.
- 각 최종 collision 기록: publisher **1**, subscriber/publisher 발견 확인, `event_count`, `sampled_event_count`, `omitted_event_count`, `stale_queued_event_count` 모두 **0**. 이는 기록된 collision 증거에 대한 결과이지 무한 주파수 충돌 부재 인증이 아니다.
- supervisor/trial/최종 physical identity 및 저장된 전체 physical sample의 actor ID가 사례별로 일치했다. 최종 physical gate accepted도 모두 true다.
- [V8 runner](run_all.py)를 import하여 재사용되는 실제 `validate_visual(folder)`만 호출했다. `main()` 또는 runtime API는 호출하지 않았다. 12개 모두 실제 X11 화면, capture/GIF **5 fps**, capture 전후 동일 창 geometry, 비어 있지 않은 PNG/GIF 및 manifest SHA-256 검사를 통과했다.
- 추가로 사례별 `visual/sha256sums.txt`의 **5개 retained file × 12 = 60개** 해시를 전부 재계산·대조했다. PNG, GIF, `ffprobe.json`, `exact_commands.txt`, `capture_manifest.json`이 모두 일치했다. 제거된 원본 MP4를 필수 잔존 파일로 오인하지 않았다.

이 감사는 자동 구조·수치·해시 검증이며, 12개 PNG/GIF를 새로 모두 육안 판독했다고 주장하지 않는다. 기존 실제 캡처와 각 사이트 분석을 대체하지 않는다. 현재 소스는 V8 수정 중이므로 과거 frozen source를 현재 소스와 같다고 검증하지도 않았다.

## 사례별 실제 완료 수치

시간은 wall seconds, 거리는 저장된 physical body-XY 표본 경로 길이(m)다. 출발·복귀 시간 합계와 전체 시간의 작은 차이는 구간 경계 정의 때문이며 UI 속도 적분 값으로 대체하지 않았다. 표의 trial 링크와 같은 폴더에 supervisor 및 `visual/capture_manifest.json`이 있다.

| 실제 profile / 사례 | actor / generation | 전체 s / m | outbound s / m | return s / m | 최종 station 거리 m |
| --- | --- | ---: | ---: | ---: | ---: |
| [V5 B4](../../B4/trial.json.gz) | 28 / 1791775921124001 | 394.763985 / 216.713311 | 165.848208 / 72.112116 | 228.743867 / 144.601195 | 0.225581043 |
| [V5 B5](../../B5/trial.json.gz) | 44 / 1791776563087001 | 380.745712 / 215.389545 | 154.228515 / 66.533315 | 226.313628 / 148.856230 | 0.214906527 |
| [V5 B1_attempt02](../../B1/trial.json.gz) | 124 / 1791778802475001 | 369.236993 / 215.385603 | 170.901555 / 79.037441 | 198.184522 / 136.348162 | 0.225141205 |
| [V5 B6_attempt02](../../B6/trial.json.gz) | 172 / 1791780055444001 | 367.014864 / 215.693355 | 153.548430 / 65.388667 | 213.248010 / 150.304688 | 0.225386285 |
| [V5 B7_attempt02](../../B7/trial.json.gz) | 188 / 1791780536997001 | 378.606991 / 215.203380 | 146.459080 / 60.612064 | 231.890415 / 154.591317 | 0.232555458 |
| [V5 B8_attempt02](../../B8/trial.json.gz) | 204 / 1791781029252001 | 373.202094 / 215.554549 | 143.311175 / 58.557997 | 229.735069 / 156.996552 | 0.231756783 |
| [V5 B11_attempt02](../../B11/trial.json.gz) | 76 / 1791777509670001 | 323.298424 / 209.620022 | 107.750034 / 45.769330 | 215.403576 / 163.850692 | 0.221055551 |
| [V5 B12_attempt02](../../B12/trial.json.gz) | 92 / 1791777942795001 | 321.669866 / 209.832739 | 106.426403 / 43.755611 | 215.081543 / 166.077128 | 0.233476055 |
| [V5 B13_attempt02](../../B13/trial.json.gz) | 108 / 1791778373066001 | 320.967977 / 209.764368 | 99.543717 / 38.979830 | 221.274624 / 170.784539 | 0.227488228 |
| [V6 B2](../../B2/trial.json.gz) | 300 / 1791783794049001 | 389.133823 / 216.251340 | 172.031062 / 78.687330 | 216.962828 / 137.564010 | 0.220897669 |
| [V6 B3](../../B3/trial.json.gz) | 252 / 1791782600687001 | 397.796144 / 215.957646 | 163.111274 / 72.771374 | 234.482943 / 143.186272 | 0.227134771 |
| [V7 B9](../../B9/trial.json.gz) | 316 / 1791785015292001 | 401.002762 / 215.956555 | 139.218789 / 53.720730 | 261.566569 / 162.235825 | 0.206369652 |

## 감사 시점 trial SHA-256

아래는 실제 읽은 trial 파일의 재계산 fingerprint다. 과거 별도 서명과 대조한 것이라는 뜻은 아니다. Visual 파일은 위에서 설명한 기존 `sha256sums.txt` 및 capture manifest와 실제 대조했다.

| 사례 | trial.json SHA-256 |
| --- | --- |
| V5 B4 | `b8c718004148e046cb5509f4c1e7e91b3973f8cc10660998e8c025e360024ac1` |
| V5 B5 | `fd011f4d083b7a907e93b3393cb8d203c8688556fb589b30668d260292a955b4` |
| V5 B1_attempt02 | `5f2a49942d90382fbb4a8b7a33a7eaaf1060986c9939fbd13b94b54d2a649cd3` |
| V5 B6_attempt02 | `540d8275fd7ba99f102977099d551075e50e41c18000420f157c0e9feb660a9c` |
| V5 B7_attempt02 | `41ac3e3918b27e7b0b95c2359dece3de7cb1e62337c91932fe6b42add9eb7996` |
| V5 B8_attempt02 | `3319943ad1322d2e2ea45dc3d5a7b1db90f63004426d0224b07e4e6e28c4d5c1` |
| V5 B11_attempt02 | `a1d008b4970676646a046609ce3af8f9f0916510d0cf84d44a40efa3fb73e1a3` |
| V5 B12_attempt02 | `4abf5781878a38066418d885d7e44b56b8d533437d0558e8ec6f5b6c788ae08c` |
| V5 B13_attempt02 | `d86dc31abbedb2176cf86238fa908f80a822417ae902012dd02fafe882d677a5` |
| V6 B2 | `366d3e896e2b2b7688d709155a8711fb64a8c25d40f878b425e957776f59e49e` |
| V6 B3 | `c76730e742a4a2914f615bf8d32048467781999f986a8f6df2118694af766b2f` |
| V7 B9 | `68709142aecf0b662fc84637ef987e1fc26e63537d2f94b6b332b52f74e4c0c7` |

최종 physical identity가 기록한 gate fingerprint도 버전별로 구분된다.

- V5 9개: `155733157023793752263a8201894ff55b7ccba4d45c3aa67fa6239c4009d727`.
- V6 2개: `07d38ccc60e4920340107021b0a2c97a849e6b4919c4c7d03ec7e28fcfe10c16`.
- V7 B9: `feb3de0673729a54b3272492222e345d9302071a7503f59496e9e047ac7ed26b`.

따라서 결론은 **요청된 과거 완료 12개 사례의 보존 증거가 재검증되었다**는 것이다. B10 해결 여부, 최신 profile 전 사이트 회귀, 실차 승인까지 확장하지 않는다. 이 감사에서는 source/config/runtime를 수정하지 않았고 이 Markdown만 새로 작성했다.
