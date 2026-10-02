import React from 'react';
import { createRoot } from 'react-dom/client';
import { act as legacyAct } from 'react-dom/test-utils';
import {
  ServiceEvidenceDashboard, ServiceEvidenceSummary, serviceDistanceBreakdown,
} from './ServiceEvidence';

const act = React.act || legacyAct;
const flush = async () => { for (let index = 0; index < 8; index += 1) await Promise.resolve(); };
const record = (id = 'run-1', site = 'B7') => ({
  id, site, intent: 'recall', phase: 'RETURNING_TO_DROP_ZONE',
  result: 'COMPLETED', distance_m: 123.4, duration_s: 250,
  completed_at: '2026-10-02T01:00:00+09:00',
  distance_breakdown_m: { delivery: 0, recall: 60, return: 63.4, unknown: 0 },
});
const snapshot = () => ({
  generated_at: '2026-10-02T01:00:00+09:00', current_service: null,
  last_completed_service: record(),
  today: { ...record(), completed_service_count: 1 },
  lifetime: { ...record(), completed_service_count: 1 },
  recent_services: [record()],
  daily_history: [{ ...record(), date: '2026-10-02', completed_service_count: 1 }],
  site_summaries: [{ ...record(), completed_service_count: 1,
    service_attempt_count: 1, completion_rate_percentage: 100,
    average_distance_m: 123.4, average_duration_s: 250 }],
  persistence: { enabled: true },
});

describe('service distance provenance', () => {
  // HH_261002 - Legacy data has no trustworthy leg classification.
  test('an old recall-looking record stays unknown without v2 breakdown', () => {
    expect(serviceDistanceBreakdown({ distance_m: 3.82, source: 'robot_ui:recall', site: 'B1' }))
      .toEqual({ delivery: 0, recall: 0, return: 0, unknown: 3.82 });
  });

  test('v2 categories retain measured legs and allocate only the residual to unknown', () => {
    expect(serviceDistanceBreakdown({ distance_m: 100,
      distance_breakdown_m: { delivery: 20, recall: 10, return: 30 } }))
      .toEqual({ delivery: 20, recall: 10, return: 30, unknown: 40 });
  });

  test('floating-point roundoff does not erase measured distance', () => {
    expect(serviceDistanceBreakdown({ distance_m: 0.3,
      distance_breakdown_m: { delivery: 0.1, recall: 0, return: 0.2, unknown: 0 } }))
      .toEqual({ delivery: 0.1, recall: 0, return: 0.2, unknown: 0 });
  });

  test.each([null, {}, { distance_m: null }, { distance_m: NaN }, { distance_m: -1 }])(
    'invalid total never becomes a fabricated zero: %p', value => {
      expect(serviceDistanceBreakdown(value)).toBeNull();
    },
  );

  test.each([{ delivery: -1 }, { delivery: 11 }, { delivery: 9, unknown: 9 }])(
    'inconsistent categories fall back to unknown: %p', breakdown => {
      expect(serviceDistanceBreakdown({ distance_m: 10, distance_breakdown_m: breakdown }))
        .toEqual({ delivery: 0, recall: 0, return: 0, unknown: 10 });
    },
  );
});

describe('service evidence dashboard with v2 distances', () => {
  let container;
  let root;
  beforeEach(() => {
    global.IS_REACT_ACT_ENVIRONMENT = true;
    global.fetch = jest.fn();
    container = document.createElement('div');
    document.body.appendChild(container);
    root = createRoot(container);
  });
  afterEach(async () => {
    await act(async () => { root.unmount(); await flush(); });
    container.remove();
    delete global.fetch;
  });
  const render = async element => {
    await act(async () => { root.render(element); await flush(); });
  };

  test('summary keeps existing KPIs and shows a short trip in metres', async () => {
    const data = snapshot();
    data.current_service = { distance_m: 3.82 };
    await render(<ServiceEvidenceSummary data={data} loading={false} error="" />);
    expect(container.querySelectorAll('.evidence-kpi')).toHaveLength(4);
    expect(container.textContent).toContain('3.8 m');
    expect(container.textContent).toContain('오늘 완료');
    expect(container.textContent).not.toContain('0.00 km');
  });

  test('today, lifetime, site, daily and recent distances coexist with existing B1-B13 chart', async () => {
    const data = snapshot();
    fetch.mockResolvedValue({ ok: true, json: async () => data });
    await render(<ServiceEvidenceDashboard summaryData={data} />);
    expect(fetch.mock.calls.map(([url]) => url)).toEqual(['/api/service-metrics?days=30']);
    expect(container.querySelector('.evidence-site-chart')).not.toBeNull();
    expect(container.querySelector('.evidence-site-trend')).not.toBeNull();
    const panels = container.querySelectorAll('.evidence-distance-panel');
    expect(panels).toHaveLength(2);
    for (const panel of panels) {
      expect(panel.textContent).toContain('합계 0.123 km');
      expect(panel.querySelector('[data-distance-kind="recall"]').textContent).toBe('60 m');
      expect(panel.querySelector('[data-distance-kind="return"]').textContent).toBe('63.4 m');
    }
    expect(container.querySelector('.evidence-current-panel [data-distance-kind="recall"]')).not.toBeNull();
    expect(container.querySelector('.evidence-site-table [data-distance-kind="recall"]')).not.toBeNull();
    expect(container.querySelector('.evidence-history-layout [data-distance-kind="return"]')).not.toBeNull();
    expect(container.querySelector('.mission-records')).not.toBeNull();
    expect(container.querySelector('.evidence-dashboard-kpis').textContent).toContain('123.4 m');
  });

  test('historical unclassified distance is included in total once, never relabelled as recall', async () => {
    const data = snapshot();
    data.lifetime = { distance_m: 25689.44, completed_service_count: 243 };
    data.historical_unclassified = {
      record_count: 243, distance_m: 25689.44, included_in_lifetime_total: true,
    };
    fetch.mockResolvedValue({ ok: true, json: async () => data });
    await render(<ServiceEvidenceDashboard summaryData={data} />);
    expect(container.querySelector('.evidence-historical-records').textContent).toContain('기존 기록 243건');
    const lifetime = container.querySelectorAll('.evidence-distance-panel')[1];
    expect(lifetime.textContent).toContain('합계 25.689 km');
    expect(lifetime.querySelector('[data-distance-kind="unknown"]').textContent).toBe('25.689 km');
    expect(lifetime.querySelector('[data-distance-kind="recall"]').textContent).toBe('0.0 m');
    expect(container.textContent).not.toContain('51.379 km');
  });

  test('missing measurements remain unavailable rather than a zero-kilometre success', async () => {
    const data = snapshot();
    data.lifetime = {};
    fetch.mockResolvedValue({ ok: true, json: async () => data });
    await render(<ServiceEvidenceDashboard summaryData={data} />);
    const lifetime = container.querySelectorAll('.evidence-distance-panel')[1];
    expect(lifetime.textContent).toContain('합계 확인 불가');
    expect(lifetime.querySelector('[data-distance-kind="unknown"]').textContent).toBe('—');
  });
});
