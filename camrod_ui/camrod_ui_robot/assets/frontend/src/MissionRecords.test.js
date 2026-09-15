import React from 'react';
import { createRoot } from 'react-dom/client';
import { act as legacyAct } from 'react-dom/test-utils';
import MissionRecords, { missionDate } from './MissionRecords';
import { ServiceEvidenceDashboard } from './ServiceEvidence';

const act = React.act || legacyAct;
const flush = async () => { for (let i = 0; i < 12; i += 1) await Promise.resolve(); };
const stamp = '2026-09-15T10:00:00+09:00';
const distances = { autonomous_m: 100, manual_m: 5, unknown_m: 2, total_m: 107 };
const mission = (id, site = 'B7', intent = 'delivery') => ({
  id, name: `${site} 왕복`, date: '2026-09-15', sequence: 1, site, intent,
  result: 'active', phase: 'MOVING_TO_SITE', current_mode: 'auto',
  ...distances, manual_interventions: 1, started_at: stamp, ended_at: null,
  duration_s: 80, stop_duration_s: 10, stop_count: 1, attempt_count: 2,
  events: [{ at: stamp, event: 'mode_changed', mode: 'manual', phase: 'OPERATOR_STOPPED',
    reason: 'operator_stop', source: 'operator_ui' }],
  files: { events: '/state/robot-1/mission/events.jsonl', raw_can: '/state/robot-1/raw.log' },
});
const snapshot = () => ({
  schema_version: 1, generated_at: stamp,
  recorder: { status: 'READY', error: null, storage_root: '/state/robot-1', last_sample_at: stamp,
    raw_can_status: 'disabled', environment: 'test', robot_id: 'robot-1' },
  lifetime: { ...distances, mission_count: 2, completed_count: 1, manual_interventions: 1 },
  current_mission: mission('one'), missions: [mission('one'), { ...mission('two', 'B8', 'recall'), result: 'completed' }],
  sites: [{ site: 'B7', intent: 'delivery', ...distances, mission_count: 1, completed_count: 0, manual_interventions: 1 }],
  outside_missions: { autonomous_m: 0, manual_m: 0, unknown_m: 0, total_m: 0 },
});
const response = (data, status = 200) => ({ ok: status === 200, status, json: async () => data });

describe('mission timestamp provenance', () => {
  test.each([null, undefined, 1789370202697, '1789370202697', '09/15/2026', 'not-a-date'])('%p cannot become a guessed date', value => {
    expect(missionDate(value)).toBe('미수신');
  });
  test('ISO day and timezone-bearing timestamp are retained as dates', () => {
    expect(missionDate('2026-09-15')).toBe('2026-09-15');
    expect(missionDate(stamp)).toContain('2026');
    expect(missionDate('2026-09-15Tnonsense')).toBe('미수신');
  });
});

describe('read-only actual MissionRecords component', () => {
  let container;
  let root;
  beforeEach(() => {
    global.IS_REACT_ACT_ENVIRONMENT = true;
    jest.useFakeTimers();
    jest.setSystemTime(new Date(stamp));
    global.fetch = jest.fn();
    container = document.createElement('div');
    document.body.appendChild(container);
    root = createRoot(container);
  });
  afterEach(async () => {
    await act(async () => { root.unmount(); await flush(); });
    container.remove();
    jest.useRealTimers();
    delete global.fetch;
  });
  const render = async (element = <MissionRecords />) => {
    await act(async () => { root.render(element); await flush(); });
  };
  const click = async element => {
    await act(async () => { element.dispatchEvent(new MouseEvent('click', { bubbles: true })); await flush(); });
  };
  const advance = async ms => {
    await act(async () => { jest.advanceTimersByTime(ms); await flush(); });
  };
  const open = async () => {
    await render();
    await click(container.querySelector('[aria-expanded]'));
  };

  test('closed panel makes no requests and labels totals separately', async () => {
    await render();
    expect(fetch).not.toHaveBeenCalled();
    expect(container.textContent).toContain('기존 누적과 별도 상세기록이며 서로 더하지 않습니다');
    expect(container.textContent).toContain('새 기록기 시작 이후');
  });

  test('real journal fields render without double-counting active mission', async () => {
    fetch.mockResolvedValue(response(snapshot()));
    await open();
    expect(fetch.mock.calls[0][0]).toBe('/api/mission-records?limit=100');
    expect(fetch.mock.calls[0][1].method).toBe('GET');
    expect(container.querySelectorAll('.mission-records-table tbody tr')).toHaveLength(2);
    const totals = container.querySelector('[aria-label="별도 미션 누적 지표"]');
    expect(totals.textContent).toContain('미션 2건 · 완료 1건 · 수동 개입 1회');
    expect(totals.textContent).toContain('107 m');
    expect(totals.textContent).toContain('0.107 km');
    expect(container.textContent).toContain('원시 CAN 저장 꺼짐 · 디코딩된 주행 정보와 별도');
    expect(container.textContent).not.toContain('원시 CAN 저장 활성');
    expect(container.textContent).toContain('현재 모드 자율');
    expect(container.querySelector('.mission-records-headline').textContent).toContain('총 이동 거리');
    const bar = totals.querySelector('.mission-records-distance-bar');
    expect(bar.getAttribute('aria-label')).toContain('자율 100 m, 수동 5 m, 모드 미확인 2 m');
    expect(parseFloat(bar.querySelector('.distance-auto').style.width)).toBeCloseTo(100 / 107 * 100);
  });

  test('polls every three seconds and updates actual supplied totals', async () => {
    const updated = snapshot();
    updated.lifetime.total_m = 120;
    fetch.mockResolvedValueOnce(response(snapshot())).mockResolvedValue(response(updated));
    await open();
    await advance(2999);
    expect(fetch).toHaveBeenCalledTimes(1);
    await advance(1);
    expect(fetch).toHaveBeenCalledTimes(2);
    expect(container.querySelector('.mission-records-totals').textContent).toContain('120 m');
  });

  test('HTTP 503 is unavailable rather than zero successful missions', async () => {
    fetch.mockResolvedValue(response({ error: 'recorder absent' }, 503));
    await open();
    expect(container.querySelector('[role="alert"]').textContent).toContain('HTTP 503');
    expect(container.textContent).toContain('0건으로 표시하지 않습니다');
    expect(container.querySelector('.mission-records-totals')).toBeNull();
  });

  test('8 second timeout aborts and never accepts the timed-out late response', async () => {
    let resolveOld;
    fetch.mockImplementationOnce(() => new Promise(resolve => { resolveOld = resolve; }))
      .mockResolvedValue(response(snapshot()));
    await open();
    const signal = fetch.mock.calls[0][1].signal;
    await advance(8000);
    expect(signal.aborted).toBe(true);
    expect(container.textContent).toContain('응답 시간 초과 (8초)');
    const late = snapshot(); late.lifetime.total_m = 99999;
    await act(async () => { resolveOld(response(late)); await flush(); });
    expect(container.querySelector('.mission-records-totals')).toBeNull();
    await advance(3000);
    expect(container.querySelector('.mission-records-totals').textContent).toContain('107 m');
    expect(container.textContent).not.toContain('99,999');
  });

  test('closing stops polling and aborts pending fetch; reopening ignores old connection', async () => {
    let resolveOld;
    fetch.mockImplementationOnce(() => new Promise(resolve => { resolveOld = resolve; }))
      .mockResolvedValue(response(snapshot()));
    await open();
    const oldSignal = fetch.mock.calls[0][1].signal;
    await click(container.querySelector('[aria-expanded]'));
    expect(oldSignal.aborted).toBe(true);
    await advance(12000);
    expect(fetch).toHaveBeenCalledTimes(1);
    await click(container.querySelector('[aria-expanded]'));
    const late = snapshot(); late.lifetime.total_m = 99999;
    await act(async () => { resolveOld(response(late)); await flush(); });
    expect(container.querySelector('.mission-records-totals').textContent).toContain('107 m');
    expect(container.textContent).not.toContain('99,999');
  });

  test('failed refresh retains last data with explicit stale warning', async () => {
    fetch.mockResolvedValueOnce(response(snapshot())).mockRejectedValue(new Error('network lost'));
    await open();
    await advance(3000);
    expect(container.querySelector('.mission-records-totals').textContent).toContain('107 m');
    expect(container.querySelector('[role="alert"]').textContent).toContain('최신 기록으로 확정하지 않습니다');
    expect(container.textContent).toContain('network lost');
  });

  test.each([{}, { schema_version: 2 }, { ...snapshot(), recorder: {} }])('malformed response cannot display zero success', async body => {
    fetch.mockResolvedValue(response(body));
    await open();
    expect(container.querySelector('[role="alert"]').textContent).toContain('응답 형식 오류');
    expect(container.querySelector('.mission-records-totals')).toBeNull();
  });

  test.each(['NOT_STARTED', 'ERROR'])('%s never displays totals as a successful recorder', async status => {
    const data = snapshot(); data.recorder.status = status;
    fetch.mockResolvedValue(response(data));
    await open();
    expect(container.querySelector('[role="alert"]')).not.toBeNull();
    expect(container.querySelector('.mission-records-totals')).toBeNull();
  });

  test('DEGRADED retains recorded data and error; CLOSED is explicitly historical', async () => {
    const degraded = snapshot(); degraded.recorder.status = 'DEGRADED'; degraded.recorder.error = 'disk quota exceeded';
    fetch.mockResolvedValueOnce(response(degraded));
    await open();
    expect(container.textContent).toContain('disk quota exceeded');
    expect(container.querySelector('.mission-records-totals')).not.toBeNull();
    const closed = snapshot(); closed.recorder.status = 'CLOSED';
    fetch.mockResolvedValue(response(closed));
    await advance(3000);
    expect(container.textContent).toContain('저장된 마지막 기록');
  });

  test('filters change only displayed records; selecting exposes reasons and paths as text', async () => {
    fetch.mockResolvedValue(response(snapshot()));
    await open();
    const site = container.querySelector('[aria-label="미션 사이트 필터"]');
    await act(async () => { site.value = 'B8'; site.dispatchEvent(new Event('change', { bubbles: true })); await flush(); });
    expect(container.querySelectorAll('.mission-records-table tbody tr')).toHaveLength(1);
    expect(container.querySelector('.mission-records-detail').textContent).toContain('B8 왕복');
    expect(container.querySelector('.mission-records-detail').textContent).toContain('operator_stop');
    expect(container.querySelector('.mission-records-detail').textContent).toContain('마지막 관측 모드 자율');
    expect(container.querySelector('.mission-records-detail').textContent).not.toContain('현재 모드 자율');
    expect(container.querySelector('.mission-records-events').textContent).toContain('수동');
    expect(container.querySelector('.mission-records-files').textContent).toContain('/state/robot-1/mission/events.jsonl');
    expect(container.querySelectorAll('.mission-records a')).toHaveLength(0);
    const intent = container.querySelector('[aria-label="미션 요청 유형 필터"]');
    await act(async () => { intent.value = 'delivery'; intent.dispatchEvent(new Event('change', { bubbles: true })); await flush(); });
    expect(container.querySelectorAll('.mission-records-table tbody tr')).toHaveLength(0);
    expect(fetch).toHaveBeenCalledTimes(1);
  });

  test('missing distances remain unknown, not fabricated zeros', async () => {
    const data = snapshot(); data.lifetime = {};
    fetch.mockResolvedValue(response(data));
    await open();
    const totals = container.querySelector('.mission-records-totals');
    expect(totals.textContent).toContain('미션 —건');
    expect(totals.textContent).not.toContain('0 m');
    expect(totals.querySelector('.mission-records-distance-bar')).toBeNull();
    expect(totals.textContent).toContain('일부 값 누락 또는 합계 불일치');
  });

  test.each([
    [{ autonomous_m: 0, manual_m: 0, unknown_m: 0, total_m: 0 }, '이동 비율 없음'],
    [{ autonomous_m: 10, manual_m: 1, unknown_m: 0, total_m: 100 }, '합계 불일치'],
  ])('zero or inconsistent distance never renders a misleading complete bar', async (values, label) => {
    const data = snapshot(); data.lifetime = { ...data.lifetime, ...values };
    fetch.mockResolvedValue(response(data));
    await open();
    const totals = container.querySelector('.mission-records-totals');
    expect(totals.querySelector('.mission-records-distance-bar')).toBeNull();
    expect(totals.textContent).toContain(label);
  });

  test('observed phases, Korean stop reason and important timeline remain evidence based', async () => {
    const data = snapshot();
    data.current_mission = { ...mission('one'), phase: 'return', events: [
      { at: stamp, event: 'mission_started', phase: 'outbound', mode: 'unknown' },
      { at: stamp, event: 'gate_changed', phase: 'outbound', reason: 'ready' },
      { at: stamp, event: 'stopped', phase: 'outbound', mode: 'auto', reason: 'reasons=obstacle_stop', source: 'safety_gate' },
      { at: stamp, event: 'mode_changed', phase: 'outbound', previous_mode: 'auto', current_mode: 'manual' },
      { at: stamp, event: 'resumed', phase: 'return', mode: 'manual', stop_duration_s: 10 },
    ] };
    fetch.mockResolvedValue(response(data));
    await open();
    const journey = container.querySelector('.mission-records-journey');
    expect([...journey.querySelectorAll('[data-observed-stage]')].map(node => node.dataset.observedStage)).toEqual(['outbound', 'return']);
    expect(journey.querySelector('[data-observed-stage="parking"]')).toBeNull();
    expect(journey.textContent).not.toContain('100%');
    expect(container.querySelector('.mission-records-stop-summary').textContent).toContain('장애물·충돌 관련 정지 보고');
    const stopped = container.querySelector('[data-event-kind="stopped"]');
    expect(stopped.textContent).toContain('정지 감지');
    expect(stopped.querySelector('details').open).toBe(false);
    expect(stopped.querySelector('details').textContent).toContain('reasons=obstacle_stop');
    expect(container.querySelector('[data-event-kind="gate_changed"]')).toBeNull();
    expect(container.querySelector('[data-event-kind="mode_changed"]').textContent).toContain('자율 → 수동');
    await click(container.querySelector('[aria-label="전체 이벤트 표시"]'));
    expect(container.querySelector('[data-event-kind="gate_changed"]')).not.toBeNull();
    expect(fetch).toHaveBeenCalledTimes(1);
  });

  test('unknown events and phases remain visible; truncated or incomplete evidence is explicit', async () => {
    const data = snapshot(); data.current_mission = { ...mission('one'), phase: 'NEW_PHASE',
      incomplete: true, events_truncated: true, event_count: 200,
      events: [{ event: 'vendor_event', phase: 'NEW_PHASE', reason: 'reason_vendor_42', at: stamp }] };
    fetch.mockResolvedValue(response(data));
    await open();
    const detail = container.querySelector('.mission-records-detail');
    expect(detail.textContent).toContain('일부 기록이 누락된 미션');
    expect(detail.textContent).toContain('전체 200건 중 최근 1건');
    expect(detail.querySelector('.mission-records-journey').textContent).toContain('최근 관측 단계만');
    expect(detail.querySelector('[data-event-kind="vendor_event"]').textContent).toContain('reason_vendor_42');
    expect(detail.querySelector('[data-observed-stage="NEW_PHASE"]')).not.toBeNull();
  });

  test('file evidence shows decoded, state and optional raw CAN separately without JSON array strings', async () => {
    const data = snapshot(); data.current_mission.files = { telemetry: ['mission/telemetry.0001.jsonl', 'mission/telemetry.0002.jsonl'],
      events: ['mission/events.0001.jsonl'], raw_can: ['mission/raw_can.0001.jsonl'] };
    fetch.mockResolvedValue(response(data));
    await open();
    const technical = container.querySelector('.mission-records-technical');
    expect(technical.open).toBe(false);
    expect(technical.textContent).toContain('snapshot.json의 요약');
    expect(technical.textContent).toContain('CAN 해독 표본');
    expect(technical.textContent).toContain('상태·이벤트 이력');
    expect(technical.textContent).toContain('원시 CAN 프레임');
    expect(technical.querySelectorAll('.mission-records-files code')).toHaveLength(4);
    expect(technical.textContent).not.toContain('["');
  });

  test('standalone return is not labelled as a second outbound-and-return trip', async () => {
    const data = snapshot(); data.current_mission = { ...mission('one', 'DROP_ZONE', 'return'),
      events: [{ event: 'stopped', phase: 'return', reason: 'reasons=none', at: stamp }] };
    fetch.mockResolvedValue(response(data));
    await open();
    expect(container.querySelector('.mission-records-detail-heading').textContent).toContain('단독 복귀');
    expect(container.querySelector('.mission-records-detail-heading').textContent).not.toContain('복귀 → 복귀');
    expect(container.querySelector('.mission-records-stop-summary').textContent).toContain('구체적인 정지 사유 없음으로 보고됨');
    expect(container.querySelector('[data-event-kind="stopped"] details').textContent).toContain('reasons=none');
  });

  test('latest sample age updates without extra requests and distinguishes absent timestamp', async () => {
    const data = snapshot(); data.recorder.last_sample_at = null;
    fetch.mockResolvedValue(response(data));
    await open();
    expect(container.textContent).toContain('마지막 주행 표본 미수신 · 시각 미수신');
    await advance(2000);
    expect(container.textContent).toContain('2초 전');
    expect(fetch).toHaveBeenCalledTimes(1);
  });

  test.each([[0.034955, '0.035 m', '0.000035 km'], [0.00001234, '0.0000123 m', '0.0000000123 km']])(
    'small measured %p is visible in both metres and kilometres', async (value, shownM, shownKm) => {
      const data = snapshot();
      data.lifetime = { ...data.lifetime, autonomous_m: 0, manual_m: 0, unknown_m: value, total_m: value };
      fetch.mockResolvedValue(response(data));
      await open();
      const totals = container.querySelector('.mission-records-totals');
      expect(totals.textContent).toContain(shownM);
      expect(totals.textContent).toContain(shownKm);
      expect(totals.querySelectorAll('.mission-records-distances dd')[2].textContent).not.toBe('0 m');
    },
  );

  test('legacy dashboard remains 243 records and 25.689 km with no mission fetch before opening', async () => {
    const old = { generated_at: stamp, persistence: { enabled: true },
      historical_unclassified: { record_count: 243, distance_m: 25689.44, included_in_lifetime_total: true },
      lifetime: { distance_m: 25689.44, completed_service_count: 123 }, today: { distance_m: 0, completed_service_count: 0 },
      recent_services: [], daily_history: [], site_summaries: [] };
    fetch.mockResolvedValue(response(old));
    await render(<ServiceEvidenceDashboard summaryData={old} />);
    expect(fetch.mock.calls.every(([url]) => url.startsWith('/api/service-metrics'))).toBe(true);
    expect(container.querySelector('.evidence-historical-records').textContent).toContain('243건');
    expect(container.querySelector('.evidence-dashboard-kpis').textContent).toContain('25.689 km');
    expect(container.querySelector('.evidence-dashboard-kpis').textContent).toContain('123회');
  });
});
