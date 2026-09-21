import React from 'react';
import { createRoot } from 'react-dom/client';
import { act as legacyAct } from 'react-dom/test-utils';
import {
  ServiceEvidenceDashboard, ServiceEvidenceSummary, serviceDistanceBreakdown,
  useServiceMetricsSummary,
} from './ServiceEvidence';

const act = React.act || legacyAct;

const record = (id, site = 'B1') => ({
  id, site, intent: 'recall', phase: 'RETURNING_TO_DROP_ZONE',
  result: 'COMPLETED', distance_m: 123.4, duration_s: 250,
  completed_at: '2026-09-14T01:00:00Z',
  distance_breakdown_m: { delivery: 0, recall: 60, return: 63.4, unknown: 0 },
});
const snapshot = (id = 'first', site = 'B1') => ({
  generated_at: '2026-09-14T01:00:00Z', current_service: null,
  last_completed_service: record(id, site),
  today: { ...record(id, site), completed_service_count: 1 },
  lifetime: { ...record(id, site), completed_service_count: 1 },
  recent_services: [record(id, site)],
  daily_history: [{ ...record(id, site), date: '2026-09-14', completed_service_count: 1 }],
  site_summaries: [{ ...record(id, site), completed_service_count: 1,
    service_attempt_count: 1, completion_rate_percentage: 100,
    average_distance_m: 123.4, average_duration_s: 250 }],
  persistence: { enabled: true },
});
const response = body => ({ ok: true, json: async () => body });
const flush = async () => { for (let index = 0; index < 8; index += 1) await Promise.resolve(); };

describe('distance provenance', () => {
  test('legacy distance is entirely unknown, even with a recall source or destination', () => {
    expect(serviceDistanceBreakdown({ distance_m: 3.82, source: 'robot_ui:recall', site: 'B1' }))
      .toEqual({ delivery: 0, recall: 0, return: 0, unknown: 3.82 });
  });
  test('preserves explicit measured categories and assigns only the residual to unknown', () => {
    expect(serviceDistanceBreakdown({ distance_m: 100,
      distance_breakdown_m: { delivery: 20, recall: 10, return: 30 } }))
      .toEqual({ delivery: 20, recall: 10, return: 30, unknown: 40 });
  });
  test('floating-point addition does not erase valid measured categories', () => {
    expect(serviceDistanceBreakdown({ distance_m: 0.3,
      distance_breakdown_m: { delivery: 0.1, return: 0.2, recall: 0, unknown: 0 } }))
      .toEqual({ delivery: 0.1, return: 0.2, recall: 0, unknown: 0 });
  });
  test.each([null, {}, { distance_m: null }, { distance_m: NaN }, { distance_m: -1 }])(
    'missing or invalid total does not fabricate zero: %p', value => {
      expect(serviceDistanceBreakdown(value)).toBeNull();
    },
  );
  test.each([
    { delivery: -1 }, { delivery: 11 }, { delivery: 9, unknown: 9 },
  ])('inconsistent categories cannot exceed measured total: %p', breakdown => {
    expect(serviceDistanceBreakdown({ distance_m: 10, distance_breakdown_m: breakdown }))
      .toEqual({ delivery: 0, recall: 0, return: 0, unknown: 10 });
  });
});

describe('actual service evidence components', () => {
  let container;
  let root;
  beforeEach(() => {
    global.IS_REACT_ACT_ENVIRONMENT = true;
    jest.useFakeTimers();
    global.fetch = jest.fn();
    container = document.createElement('div');
    document.body.appendChild(container);
    root = createRoot(container);
  });
  afterEach(async () => {
    if (root) await act(async () => { root.unmount(); await flush(); });
    container.remove();
    jest.useRealTimers();
    delete global.fetch;
  });
  const render = async element => {
    await act(async () => { root.render(element); await flush(); });
  };
  const advance = async ms => {
    await act(async () => { jest.advanceTimersByTime(ms); await flush(); });
  };

  test('summary exposes four kinds and small movement in metres without changing the total', async () => {
    const data = snapshot();
    data.current_service = { distance_m: 3.82 };
    await render(<ServiceEvidenceSummary data={data} loading={false} error="" />);
    expect(container.textContent).toContain('3.8 m');
    expect(container.textContent).not.toContain('0.00 km');
    expect(container.textContent).toContain('배송(가는 길)');
    expect(container.textContent).toContain('호출(가는 길)');
    expect(container.textContent).toContain('구간 미확인');
    expect(container.querySelector('[data-distance-kind="unknown"]').textContent).toBe('3.8 m');
  });

  test('dashboard displays intent, phase, cumulative site breakdown and explicit service basis', async () => {
    fetch.mockResolvedValue(response(snapshot()));
    await render(<ServiceEvidenceDashboard summaryData={snapshot()} />);
    expect(container.textContent).toContain('호출 · 대기 장소로 복귀');
    expect(container.textContent).toContain('누적 구간거리(진행·중단 포함)');
    expect(container.textContent).toContain('순수 자율주행 누적거리와는 다르며');
    expect(container.textContent).toContain('서비스 경과시간(대기 포함)');
    expect(container.textContent).toContain('서비스 시작일 기준 · 최근 30일');
    expect(container.querySelector('.evidence-site-table [data-distance-kind="recall"]').textContent).toBe('60 m');
    expect(container.querySelector('.evidence-refresh-button').disabled).toBe(false);
  });

  test('today and lifetime retain metres and expose summed kilometres in both views', async () => {
    const data = snapshot();
    data.today.distance_m = 64;
    data.lifetime.distance_m = 264;
    await render(<ServiceEvidenceSummary data={data} loading={false} error="" />);
    expect(container.textContent).toContain('64 m');
    expect(container.textContent).toContain('합산 0.064 km');
    expect(container.textContent).toContain('합산 0.264 km');
    fetch.mockResolvedValue(response(data));
    await render(<ServiceEvidenceDashboard summaryData={data} />);
    expect(container.textContent).toContain('64 m');
    expect(container.textContent).toContain('합산 0.064 km');
    expect(container.textContent).toContain('합산 0.264 km');
  });

  test('legacy history is visibly preserved in total without allocating or adding it twice', async () => {
    const data = snapshot();
    data.lifetime.distance_m = 25689.44;
    data.lifetime.distance_breakdown_m = { delivery: 0, recall: 0, return: 0, unknown: 25689.44 };
    data.historical_unclassified = { record_count: 243, distance_m: 25689.44, included_in_lifetime_total: true };
    fetch.mockResolvedValue(response(data));
    await render(<ServiceEvidenceDashboard summaryData={data} />);
    const notice = container.querySelector('[aria-label="기존 운행 기록 보존 안내"]');
    expect(notice.textContent).toContain('기존 기록 243건');
    expect(notice.textContent).toContain('25.689 km가 전체 누적에 포함');
    expect(notice.textContent).toContain('삭제되거나 호출 거리로 바뀐 것이 아닙니다');
    const total = container.querySelectorAll('.evidence-dashboard-kpis [data-distance-kind="unknown"]')[1];
    expect(total.textContent).toBe('25.689 km');
    expect(container.textContent).not.toContain('51.379 km');
  });

  test.each([undefined, { record_count: 0, distance_m: 0, included_in_lifetime_total: true },
    { record_count: 243, distance_m: 25689.44 },
    { record_count: 243, distance_m: -1, included_in_lifetime_total: true }])(
    'does not claim historical preservation without explicit valid evidence: %p', async history => {
      const data = { ...snapshot(), historical_unclassified: history };
      fetch.mockResolvedValue(response(data));
      await render(<ServiceEvidenceDashboard summaryData={data} />);
      expect(container.querySelector('[aria-label="기존 운행 기록 보존 안내"]')).toBeNull();
    },
  );

  test('zero-distance historical rows are preserved and shown too', async () => {
    const data = { ...snapshot(), historical_unclassified: {
      record_count: 2, distance_m: 0, included_in_lifetime_total: true,
    } };
    fetch.mockResolvedValue(response(data));
    await render(<ServiceEvidenceDashboard summaryData={data} />);
    expect(container.querySelector('.evidence-historical-records').textContent).toContain('기존 기록 2건');
    expect(container.querySelector('.evidence-historical-records').textContent).toContain('0.0 m');
  });

  test.each([null, -1])('missing or negative total has no invented kilometre value: %p', total => {
    const data = snapshot();
    data.today.distance_m = total;
    data.lifetime.distance_m = total;
    return render(<ServiceEvidenceSummary data={data} loading={false} error="" />).then(() => {
      const lines = [...container.querySelectorAll('small')].map(element => element.textContent);
      expect(lines.filter(line => line === '합산 —')).toHaveLength(2);
      expect(lines.some(line => /합산 -[0-9]/.test(line))).toBe(false);
      expect(container.textContent).not.toContain('-1.0 m');
    });
  });

  test('an actual measured zero remains a valid total kilometre value', async () => {
    const data = snapshot();
    data.today.distance_m = 0;
    data.lifetime.distance_m = 0;
    await render(<ServiceEvidenceSummary data={data} loading={false} error="" />);
    expect(container.textContent).toContain('합산 0.000 km');
    expect(container.textContent).not.toContain('합산 —');
  });

  test('open history refreshes every four seconds and shows a completed new row without reopening', async () => {
    fetch.mockResolvedValueOnce(response(snapshot('first', 'B1')))
      .mockResolvedValueOnce(response(snapshot('second', 'B2')));
    await render(<ServiceEvidenceDashboard summaryData={snapshot()} />);
    expect(container.querySelector('.evidence-recent-table').textContent).toContain('B1');
    await advance(3999);
    expect(fetch).toHaveBeenCalledTimes(1);
    await advance(1);
    expect(fetch).toHaveBeenCalledTimes(2);
    expect(fetch.mock.calls[1][0]).toBe('/api/service-metrics?days=30');
    expect(container.querySelector('.evidence-recent-table').textContent).toContain('B2');
    expect(container.querySelector('.evidence-recent-table').textContent).not.toContain('B1');
    expect(container.textContent).toContain('목록 갱신');
  });

  test('manual refresh aborts pending request and ignores its late stale response', async () => {
    let resolveOld;
    fetch.mockImplementationOnce(() => new Promise(resolve => { resolveOld = resolve; }))
      .mockResolvedValueOnce(response(snapshot('new', 'B2')));
    await render(<ServiceEvidenceDashboard summaryData={snapshot()} />);
    const oldSignal = fetch.mock.calls[0][1].signal;
    await act(async () => {
      container.querySelector('.evidence-refresh-button').click();
      await flush();
    });
    expect(oldSignal.aborted).toBe(true);
    expect(container.querySelector('.evidence-recent-table').textContent).toContain('B2');
    await act(async () => { resolveOld(response(snapshot('old', 'B1'))); await flush(); });
    expect(container.querySelector('.evidence-recent-table').textContent).not.toContain('B1');
  });

  test('in-flight requests do not overlap and closing the modal aborts and cancels polling', async () => {
    fetch.mockImplementation(() => new Promise(() => {}));
    await render(<ServiceEvidenceDashboard />);
    const signal = fetch.mock.calls[0][1].signal;
    await advance(7000);
    expect(fetch).toHaveBeenCalledTimes(1);
    await act(async () => { root.unmount(); root = null; await flush(); });
    expect(signal.aborted).toBe(true);
    await advance(20000);
    expect(fetch).toHaveBeenCalledTimes(1);
    expect(jest.getTimerCount()).toBe(0);
  });

  test('transport error retains old history with a warning and recovers on next poll', async () => {
    fetch.mockResolvedValueOnce(response(snapshot('first', 'B1')))
      .mockRejectedValueOnce(new Error('offline'))
      .mockResolvedValueOnce(response(snapshot('second', 'B2')));
    await render(<ServiceEvidenceDashboard summaryData={snapshot()} />);
    await advance(4000);
    expect(container.querySelector('.evidence-recent-table').textContent).toContain('B1');
    expect(container.textContent).toContain('최신 기록 갱신에 실패했습니다');
    await advance(4000);
    expect(container.querySelector('.evidence-recent-table').textContent).toContain('B2');
    expect(container.textContent).not.toContain('최신 기록 갱신에 실패했습니다');
  });

  test('an unresponsive fetch is aborted after eight seconds and polling recovers', async () => {
    fetch.mockImplementationOnce((url, { signal }) => new Promise((resolve, reject) => {
      signal.addEventListener('abort', () => reject(new DOMException('aborted', 'AbortError')));
    })).mockResolvedValueOnce(response(snapshot('recovered', 'B2')));
    await render(<ServiceEvidenceDashboard summaryData={snapshot()} />);
    await advance(8000);
    expect(fetch.mock.calls[0][1].signal.aborted).toBe(true);
    expect(container.querySelector('[role="alert"]')).not.toBeNull();
    await advance(4000);
    expect(fetch).toHaveBeenCalledTimes(2);
    expect(container.querySelector('.evidence-recent-table').textContent).toContain('B2');
    expect(container.querySelector('[role="alert"]')).toBeNull();
  });

  test('summary keeps its separate three-second cadence and completion clears current service', async () => {
    function SummaryHost() {
      const state = useServiceMetricsSummary();
      return <ServiceEvidenceSummary {...state} />;
    }
    const active = snapshot();
    active.current_service = { ...record('active'), result: 'ACTIVE' };
    fetch.mockResolvedValueOnce(response(active)).mockResolvedValueOnce(response(snapshot()));
    await render(<SummaryHost />);
    expect(container.textContent).toContain('이번 서비스');
    await advance(3000);
    expect(fetch.mock.calls[1][0]).toBe('/api/service-metrics/summary');
    expect(container.textContent).toContain('최근 서비스');
    expect(container.textContent).not.toContain('이번 서비스');
  });
});
