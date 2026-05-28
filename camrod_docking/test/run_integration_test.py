#!/usr/bin/env python3
"""Manual Docking 통합 테스트 실행기.

ManualDockServerNode + MockDockRobotServer를 모두 subprocess로 기동하고
ros2 action send_goal CLI로 각 TC를 검증한다.

사용법:
    bash -c 'source /opt/ros/humble/setup.bash && ... && python3 run_integration_test.py'
"""

import json
import os
import signal
import subprocess
import sys
import time
import threading
import urllib.request
import urllib.error

# ── ANSI 색상 ─────────────────────────────────────────────────────────────────
GREEN  = '\033[92m'
RED    = '\033[91m'
YELLOW = '\033[93m'
CYAN   = '\033[96m'
RESET  = '\033[0m'

def ok(msg):   print(f"  {GREEN}✓ PASS{RESET}  {msg}")
def fail(msg): print(f"  {RED}✗ FAIL{RESET}  {msg}")
def info(msg): print(f"  {CYAN}  ···{RESET}  {msg}")


SOURCE_CMD = (
    'source /opt/ros/humble/setup.bash && '
    'source /home/avg/ros2_ws/install/setup.bash && '
    'source /home/avg/CAMROD/install/setup.bash && '
    'export ROS_LOCALHOST_ONLY=1 && '
)

MDS_BIN = '/home/avg/CAMROD/install/camrod_docking/lib/camrod_docking/manual_dock_server_node'
MOCK_PY  = '/home/avg/CAMROD/camrod_docking/test/mock_dock_robot_server.py'
UI_BIN          = '/home/avg/CAMROD/install/camrod_api/lib/camrod_api/ui_backend_node'
UI_PORT         = 8010
API_STATUS_PATH = '/api/docking/status'


def bash(cmd: str, timeout: float = 5.0) -> tuple[int, str]:
    """bash 명령 실행 후 (returncode, stdout+stderr) 반환."""
    result = subprocess.run(
        ['bash', '-c', SOURCE_CMD + cmd],
        capture_output=True, text=True, timeout=timeout
    )
    return result.returncode, result.stdout + result.stderr


def start_proc(cmd: str, log_prefix: str) -> subprocess.Popen:
    """bash subprocess 시작 + 로그 스레드."""
    proc = subprocess.Popen(
        ['bash', '-c', SOURCE_CMD + cmd],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )
    def _log():
        for line in proc.stdout:
            print(f"  [{log_prefix}] {line.rstrip()}")
    threading.Thread(target=_log, daemon=True).start()
    return proc


def http_get(path: str, timeout: float = 3.0) -> dict:
    """UiBackendNode HTTP GET 요청. 실패 시 빈 dict 반환."""
    try:
        url = f'http://localhost:{UI_PORT}{path}'
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return json.loads(resp.read().decode())
    except Exception:
        return {}


def http_post(path: str, timeout: float = 3.0) -> dict:
    """UiBackendNode HTTP POST 요청. 실패 시 빈 dict 반환."""
    try:
        url = f'http://localhost:{UI_PORT}{path}'
        req = urllib.request.Request(url, method='POST', data=b'')
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode())
    except urllib.error.HTTPError as e:
        try:
            return json.loads(e.read().decode())
        except Exception:
            return {'http_error': e.code}
    except Exception:
        return {}


def send_goal(dock_id: str, feedback: bool = False, timeout: float = 25.0) -> str:
    """ros2 action send_goal 실행 후 출력 반환."""
    fb_flag = '--feedback' if feedback else ''
    _, out = bash(
        f'ros2 action send_goal /docking/manual_dock '
        f'avg_msgs/action/ManualDock "{{dock_id: \'{dock_id}\'}}" {fb_flag}',
        timeout=timeout
    )
    return out


def parse_result(out: str) -> dict:
    """send_goal 출력에서 Result 파싱."""
    result = {}
    in_result = False
    for line in out.splitlines():
        if line.strip().startswith('Result:'):
            in_result = True
            continue
        if in_result:
            line = line.strip()
            if not line or line.startswith('Goal finished'):
                break
            if ':' in line:
                k, v = line.split(':', 1)
                result[k.strip()] = v.strip()
    # status
    for line in out.splitlines():
        if 'Goal finished with status:' in line:
            result['__status__'] = line.split(':')[-1].strip()
    return result


def parse_feedbacks(out: str) -> list:
    """Feedback의 phase_label 순서 추출 (중복 제거)."""
    phases = []
    for line in out.splitlines():
        if 'phase_label:' in line:
            p = line.split(':', 1)[1].strip()
            if not phases or phases[-1] != p:
                phases.append(p)
    return phases


# ── 테스트 케이스 ─────────────────────────────────────────────────────────────

def tc01(mock_proc) -> bool:
    print(f"\n{YELLOW}[TC-01] 정상 도킹 성공{RESET}")
    out = send_goal('home_dock', feedback=True, timeout=25.0)
    r   = parse_result(out)
    fbs = parse_feedbacks(out)

    expected = ['NAVIGATING_TO_STAGING', 'INITIAL_PERCEPTION', 'DOCKING', 'WAIT_FOR_CHARGE']
    phase_ok   = all(p in fbs for p in expected)
    success_ok = r.get('success') == 'true'
    code_ok    = r.get('error_code') == '0'
    status_ok  = r.get('__status__') == 'SUCCEEDED'

    if success_ok:  ok(f"Result.success=true")
    else:           fail(f"Result.success={r.get('success')}")
    if code_ok:     ok("Result.error_code=0")
    else:           fail(f"Result.error_code={r.get('error_code')}")
    if phase_ok:    ok(f"Feedback phases: {fbs}")
    else:           fail(f"Feedback phases 누락 — 수신={fbs}, 기대={expected}")
    if status_ok:   ok(f"Goal status=SUCCEEDED")
    else:           fail(f"Goal status={r.get('__status__')}")

    return all([success_ok, code_ok, phase_ok, status_ok])


def tc02() -> bool:
    print(f"\n{YELLOW}[TC-02] cancel 정상 전파{RESET}")
    # goal 전송 후 4초 뒤 SIGINT → CLI가 cancel request를 서버로 전달
    proc = subprocess.Popen(
        ['bash', '-c',
         SOURCE_CMD +
         'ros2 action send_goal /docking/manual_dock '
         "avg_msgs/action/ManualDock \"{dock_id: 'home_dock'}\" --feedback"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True  # 독립 세션 — SIGINT가 부모로 전파되지 않음
    )
    time.sleep(4.0)
    # SIGINT: ros2 CLI가 cancel_goal() 호출 후 종료
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        out, _ = proc.communicate(timeout=8.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        out, _ = proc.communicate()

    cancelled_in_output = (
        'Canceling' in out or 'canceled' in out.lower()
        or 'CANCELED' in out or 'cancel' in out.lower()
    )
    info(f"send_goal 출력 (앞 300자): {out[:300].strip()}")
    if cancelled_in_output:
        info("CLI 출력에서 cancel 확인됨")
    else:
        info("CLI 출력에 cancel 미확인 — MDS 로그로 확인 필요")

    # MDS 상태 리셋 대기 후, dock_id='' → dock_id 오류 REJECT인지 확인
    # (active 상태면 "already in progress" REJECT)
    time.sleep(3.0)
    _, out2 = bash(
        'ros2 action send_goal /docking/manual_dock '
        "avg_msgs/action/ManualDock \"{dock_id: ''}\"",
        timeout=8.0
    )
    already = 'already in progress' in out2
    if not already:
        ok("cancel 후 MDS 상태 정상 (active=false)")
        return True
    else:
        fail("이전 goal이 아직 active — cancel 전파 실패")
        return False


def tc03() -> bool:
    print(f"\n{YELLOW}[TC-03] 중복 요청 차단{RESET}")
    # 첫 번째 goal 백그라운드
    proc1 = subprocess.Popen(
        ['bash', '-c',
         SOURCE_CMD +
         'ros2 action send_goal /docking/manual_dock '
         "avg_msgs/action/ManualDock \"{dock_id: 'home_dock'}\""],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )
    time.sleep(1.5)

    # 두 번째 goal 전송
    _, out2 = bash(
        'ros2 action send_goal /docking/manual_dock '
        "avg_msgs/action/ManualDock \"{dock_id: 'home_dock'}\"",
        timeout=8.0
    )

    rejected = 'Goal was rejected' in out2 or 'REJECTED' in out2
    if rejected:    ok(f"두 번째 goal REJECT됨")
    else:           fail(f"두 번째 goal 수락됨 — 중복 차단 실패\n    출력: {out2[:200]}")

    # 첫 번째 goal 완료 대기 (success 시나리오 ~10s)
    try:
        proc1.wait(timeout=20.0)
    except subprocess.TimeoutExpired:
        proc1.kill()
        proc1.wait()

    return rejected


def tc04(mock_proc) -> bool:
    print(f"\n{YELLOW}[TC-04] DockRobot 실패 → error_code 전파{RESET}")
    out = send_goal('home_dock', timeout=15.0)
    r   = parse_result(out)

    success_ok = r.get('success') == 'false'
    ec_ok      = r.get('error_code') == '903'
    status_ok  = r.get('__status__') in ('ABORTED', 'ABORT')

    if success_ok:  ok("Result.success=false")
    else:           fail(f"Result.success={r.get('success')}")
    if ec_ok:       ok("error_code=903 (FAILED_TO_STAGE)")
    else:           fail(f"error_code={r.get('error_code')} (903 기대)")
    if status_ok:   ok(f"Goal status={r.get('__status__')}")
    else:           fail(f"Goal status={r.get('__status__')}")

    return success_ok and ec_ok


def tc05() -> bool:
    print(f"\n{YELLOW}[TC-05] action server 미기동 → 5초 타임아웃{RESET}")
    info("MockDockRobotServer 없이 goal 전송")
    t_start = time.time()
    out = send_goal('home_dock', timeout=25.0)
    elapsed = time.time() - t_start
    r = parse_result(out)

    timed_out = r.get('success') == 'false' and 'not available' in out
    elapsed_ok = 4.5 <= elapsed <= 15.0  # CLI sourcing ~3s + MDS 5s wait

    if timed_out:   ok(f"타임아웃 abort 확인 (message에 'not available')")
    else:           fail(f"타임아웃 미확인 — 출력:\n{out[:300]}")
    if elapsed_ok:  ok(f"소요시간={elapsed:.1f}s (5초 타임아웃 + CLI 오버헤드)")
    else:           fail(f"소요시간={elapsed:.1f}s (4.5~15.0s 기대)")

    return timed_out and elapsed_ok


def tc06() -> bool:
    print(f"\n{YELLOW}[TC-06] goal 수락 타임아웃 (slow_accept 12s → MDS 10s 초과){RESET}")
    # slow_accept 시나리오: _goal_callback에서 12초 지연 → MDS goal_accept_future 10초 초과
    t_start = time.time()
    out = send_goal('home_dock', timeout=25.0)
    elapsed = time.time() - t_start
    r = parse_result(out)

    timed_out = r.get('success') == 'false' and 'timed out' in out
    elapsed_ok = 9.0 <= elapsed <= 20.0  # CLI sourcing + MDS 10s wait

    if timed_out:   ok("goal 수락 타임아웃 abort 확인 (message에 'timed out')")
    else:           fail(f"타임아웃 미확인 — 출력:\n{out[:300]}")
    if elapsed_ok:  ok(f"소요시간={elapsed:.1f}s (10초 타임아웃 + CLI 오버헤드)")
    else:           fail(f"소요시간={elapsed:.1f}s (9.0~20.0s 기대)")

    return timed_out and elapsed_ok


def tc07() -> bool:
    print(f"\n{YELLOW}[TC-07] dock_id 빈값 → REJECT{RESET}")
    _, out = bash(
        'ros2 action send_goal /docking/manual_dock '
        "avg_msgs/action/ManualDock \"{dock_id: ''}\"",
        timeout=8.0
    )
    rejected = 'Goal was rejected' in out or 'REJECTED' in out
    if rejected:    ok("dock_id='' → REJECT됨")
    else:           fail(f"REJECT 미확인 — 출력: {out[:200]}")
    return rejected


def tc08() -> bool:
    print(f"\n{YELLOW}[TC-08] HTTP 상태 실시간 갱신{RESET}")
    # POST /api/docking/start 후 0.5초 간격 폴링으로 phase/elapsed_sec 변화 확인

    resp = http_post(f'/api/docking/start?dock_id=home_dock')
    if not resp.get('success'):
        fail(f"POST /api/docking/start 실패: {resp}")
        return False
    info("POST /api/docking/start 성공 — 폴링 시작")

    phases_seen = []
    elapsed_prev = -1.0
    elapsed_increasing = True
    t_start = time.time()

    while time.time() - t_start < 20.0:
        st = http_get(API_STATUS_PATH)
        if not st:
            time.sleep(0.5)
            continue

        # /api/docking/status 는 docking 딕셔너리를 직접 반환
        phase   = st.get('phase', '')
        elapsed = st.get('elapsed_sec', 0.0)
        active  = st.get('active', False)

        if phase and phase != 'IDLE' and (not phases_seen or phases_seen[-1] != phase):
            phases_seen.append(phase)
            info(f"phase={phase} elapsed={elapsed:.1f}s")

        if elapsed < elapsed_prev:
            elapsed_increasing = False
        elapsed_prev = elapsed

        if not active and len(phases_seen) > 0:
            break
        time.sleep(0.5)

    # SENDING_GOAL: UiBackendNode가 goal 전송 직후 설정하는 내부 phase
    expected = ['SENDING_GOAL', 'NAVIGATING_TO_STAGING', 'INITIAL_PERCEPTION', 'DOCKING', 'WAIT_FOR_CHARGE']
    phase_ok    = phases_seen == expected
    elapsed_ok  = elapsed_increasing
    success_ok  = http_get(API_STATUS_PATH).get('success') is True

    if phase_ok:    ok(f"phase 순서 정상: {phases_seen}")
    else:           fail(f"phase 순서 불일치 — 기대:{expected}\n    실제:{phases_seen}")
    if elapsed_ok:  ok("elapsed_sec 단조 증가 확인")
    else:           fail("elapsed_sec 감소 발생 — 단조 증가 실패")
    if success_ok:  ok("최종 success=true 확인")
    else:           fail("최종 success 미확인")

    return phase_ok and elapsed_ok and success_ok


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    results = {}

    # ROS2 daemon 재시작 (WSL2 DDS discovery 안정화)
    info("ROS2 daemon 재시작...")
    bash('ros2 daemon stop', timeout=5.0)
    time.sleep(1.0)
    bash('ros2 daemon start', timeout=5.0)
    time.sleep(2.0)

    # ManualDockServerNode 기동
    mds_proc = start_proc(
        f'exec {MDS_BIN} --ros-args '
        '-p action_server_name:=/docking/manual_dock '
        '-p dock_action_name:=/docking/dock_robot',
        'MDS'
    )
    info(f"ManualDockServerNode PID={mds_proc.pid}")

    # ManualDock action server 탐색 대기
    info("action server 탐색 대기...")
    for i in range(15):
        rc, out = bash('ros2 action list', timeout=10.0)
        if '/docking/manual_dock' in out:
            ok("ManualDockServerNode 연결됨")
            break
        time.sleep(1.0)
    else:
        print(f"{RED}ERROR: ManualDockServerNode 탐색 실패{RESET}")
        mds_proc.terminate()
        sys.exit(1)

    print(f"\n{'='*55}")
    print(f"  Manual Docking 통합 테스트")
    print(f"{'='*55}")

    # ── TC-01: success 시나리오 ───────────────────────────────────────────────
    mock = start_proc(f'python3 {MOCK_PY} --ros-args -p scenario:=success', 'MOCK')
    time.sleep(2.0)
    results['TC-01'] = tc01(mock)
    mock.terminate(); mock.wait()
    time.sleep(2.0)

    # ── TC-02: cancel ─────────────────────────────────────────────────────────
    mock = start_proc(f'python3 {MOCK_PY} --ros-args -p scenario:=success', 'MOCK')
    time.sleep(2.0)
    results['TC-02'] = tc02()
    mock.terminate(); mock.wait()
    time.sleep(3.0)  # MDS cancel 처리 + 상태 리셋 대기

    # ── TC-03: 중복 요청 차단 ─────────────────────────────────────────────────
    mock = start_proc(f'python3 {MOCK_PY} --ros-args -p scenario:=success', 'MOCK')
    time.sleep(2.0)
    results['TC-03'] = tc03()
    mock.terminate(); mock.wait()
    time.sleep(3.0)  # TC-03 첫 번째 goal 완전 종료 대기

    # ── TC-04: fail 시나리오 ──────────────────────────────────────────────────
    mock = start_proc(f'python3 {MOCK_PY} --ros-args -p scenario:=fail', 'MOCK')
    time.sleep(2.0)
    results['TC-04'] = tc04(mock)
    mock.terminate(); mock.wait()
    time.sleep(2.0)

    # ── TC-05: Mock 없이 타임아웃 ─────────────────────────────────────────────
    results['TC-05'] = tc05()
    time.sleep(2.0)

    # ── TC-06: goal 수락 타임아웃 ─────────────────────────────────────────────
    mock = start_proc(f'python3 {MOCK_PY} --ros-args -p scenario:=slow_accept', 'MOCK')
    time.sleep(2.0)
    results['TC-06'] = tc06()
    mock.terminate(); mock.wait()
    time.sleep(2.0)

    # ── TC-07: dock_id 빈값 ───────────────────────────────────────────────────
    mock = start_proc(f'python3 {MOCK_PY} --ros-args -p scenario:=success', 'MOCK')
    time.sleep(2.0)
    results['TC-07'] = tc07()
    mock.terminate(); mock.wait()
    time.sleep(2.0)

    # ── TC-08: HTTP 상태 실시간 갱신 ─────────────────────────────────────────
    # UiBackendNode 기동 (ManualDock action client + HTTP :8010)
    ui_proc = start_proc(
        f'exec {UI_BIN} --ros-args '
        '-p dock_action_name:=/docking/manual_dock '
        f'-p port:={UI_PORT}',
        'UI'
    )
    # HTTP 서버 준비 대기
    info("UiBackendNode HTTP 준비 대기...")
    for _ in range(15):
        st = http_get(API_STATUS_PATH)
        if st:
            ok("UiBackendNode HTTP 응답 확인됨")
            break
        time.sleep(1.0)
    else:
        fail("UiBackendNode HTTP 응답 없음 — TC-08 스킵")
        results['TC-08'] = False
        ui_proc.terminate(); ui_proc.wait()

    if 'TC-08' not in results:
        mock = start_proc(f'python3 {MOCK_PY} --ros-args -p scenario:=success', 'MOCK')
        time.sleep(2.0)
        results['TC-08'] = tc08()
        mock.terminate(); mock.wait()
        ui_proc.terminate(); ui_proc.wait()

    # ── 결과 요약 ─────────────────────────────────────────────────────────────
    labels = {
        'TC-01': '정상 도킹 성공',
        'TC-02': 'cancel 정상 전파',
        'TC-03': '중복 요청 차단',
        'TC-04': 'DockRobot 실패 전파',
        'TC-05': 'action server 미기동 타임아웃',
        'TC-06': 'goal 수락 타임아웃',
        'TC-07': 'dock_id 빈값 거부',
        'TC-08': 'HTTP 상태 실시간 갱신',
    }
    passed = failed = 0
    print(f"\n{'='*55}")
    print(f"  테스트 결과 요약")
    print(f"{'='*55}")
    for tc, label in labels.items():
        r = results.get(tc)
        if r:
            print(f"  {GREEN}✓ PASS{RESET}  {tc}: {label}"); passed += 1
        else:
            print(f"  {RED}✗ FAIL{RESET}  {tc}: {label}"); failed += 1
    print(f"\n  결과: {GREEN}{passed} passed{RESET} / {RED}{failed} failed{RESET}")
    print(f"{'='*55}\n")

    mds_proc.terminate()
    mds_proc.wait()
    sys.exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()
