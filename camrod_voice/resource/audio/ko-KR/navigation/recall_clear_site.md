# Recall site-clearance announcement

Key: `navigation.recall_clear_site`

Text: 해당 캠핑 사이트를 잠시 비워 주세요. 로봇이 들어가 방향을 바꾼 뒤 복귀합니다.

Generated on 2026-09-07 with `edge-tts 7.2.8`, voice `ko-KR-SunHiNeural`,
rate `+10%`, default pitch and volume. Generation used a temporary environment;
there is no TTS package, network request, or credential dependency at runtime.

The generated MP3 was decoded with GStreamer to `recall_clear_site.wav`:
24,000 Hz, mono, signed 16-bit PCM; 157,248 frames, 6.552 seconds.
The controller's default clearance pause is 8.0 seconds. When replacing this
recording, keep the full duration below 7.0 seconds to retain dispatch margin,
or update the controller pause and its mirrored configuration accordingly.

The controller announces `RECALL_CLEARANCE_WAIT` before B1–B10 site re-entry.
Voice emits this cue once per clearance phase, independently of whether the
shared `RETURN_WITH_CARGO` service-state ID has changed. B11–B13 keep their
existing exit behavior and do not request this recording.

This recording accompanies only the first site-clearance confirmation. After
the 180-degree turn, `RECALL_RETURN_WAIT` holds the robot stopped for loading;
a second explicit `recall_final_return` confirmation is still required to exit.
The cue itself is not a motion authorization or a promise of automatic exit.
