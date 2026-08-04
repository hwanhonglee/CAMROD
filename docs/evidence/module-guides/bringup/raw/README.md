# Bringup Runtime Excerpt

<!-- HH_260805 - Keep one concise, directly referenced runtime excerpt and
remove duplicated mission-role logs after their results were normalized. -->

The duplicated B6/B12 node logs were removed after their outcomes were captured
in [`campsite-smoke-20260804.json`](../campsite-smoke-20260804.json) and the
module-guide figures. The remaining excerpt is directly referenced by the
validation document and preserves the retry/stop sequence without repeated
status lines.

| Scenario | File | Preserved result |
|---|---|---|
| B6 map-v14 live capture | [`runtime-visual-capture-20260804.log`](runtime-visual-capture-20260804.log) | one release, 0.276 s rapid recontact latch, zero Twist, HTTP 200 stop, state 16 |
