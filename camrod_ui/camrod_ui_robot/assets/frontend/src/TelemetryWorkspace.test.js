import React, { act } from 'react';
import { createRoot } from 'react-dom/client';
import TelemetryWorkspace from './TelemetryWorkspace';

// HH_261026 - Exercise the real reset control and existing telemetry transport;
// HH_261026 - Native telemetry keeps its existing transport and no simulator control.

let host, root, sockets, originalFetch, originalWebSocket;
class TestSocket {
  static OPEN = 1;
  constructor(url) {
    this.url = url;
    this.readyState = TestSocket.OPEN;
    this.send = jest.fn();
    this.close = jest.fn();
    sockets.push(this);
  }
}

const response = body => ({ ok: true, json: async () => body });
const postCalls = () => fetch.mock.calls.filter(([, options]) => options?.method === 'POST');
const render = async props => {
  await act(async () => root.render(<div className="admin-workspace">
    <TelemetryWorkspace activeTab="safety" adminLoginId="operator" adminLoginPw="secret" {...props} />
  </div>));
};
const emit = async telemetry => {
  await act(async () => sockets[sockets.length - 1].onmessage({ data: JSON.stringify(telemetry) }));
};
const source = { age_s: 0.1, rate_hz: 10 };
const snapshot = (changes = {}) => ({
  session_active: true,
  mission: { phase: 'DRIVING', source: 'manual', service_state_name: 'MOVING_TO_SITE',
    system_health: 'OK', ready: true, engaged: false },
  sources: { 'control.safety_gate': source },
  safety: { gate: { level: 1, operating_state: 'ROUTE_SAFETY_HOLD',
    message: 'state=ROUTE_SAFETY_HOLD reasons=lanelet_physical_body_cost,engage=false charging=false battery=78.00 recovery_reason=manual_projected_recovery',
    missing_nodes: [], missing_topics: [] },
  controllers: {}, radar_evidence: 'active SENSOR=FRONT1 range=1.23', obstacle_replan: 'waiting_for_clear_path' },
  ...changes,
});

beforeEach(() => {
  global.IS_REACT_ACT_ENVIRONMENT = true;
  jest.useFakeTimers();
  sockets = [];
  originalFetch = global.fetch;
  originalWebSocket = global.WebSocket;
  global.WebSocket = TestSocket;
  global.fetch = jest.fn(async url => response(url === '/api/admin/safety/reset'
    ? { success: true, available: true, message: '정지 원인을 확인하세요' }
    : url === '/api/telemetry/map' ? { polylines: [], point_count: 0 } : {}));
  host = document.createElement('div');
  document.body.appendChild(host);
  root = createRoot(host);
});

afterEach(() => {
  act(() => root.unmount());
  host.remove();
  global.fetch = originalFetch;
  global.WebSocket = originalWebSocket;
  jest.useRealTimers();
  delete global.IS_REACT_ACT_ENVIRONMENT;
});

test('three named sections keep safety controls visible and missing status explicit', async () => {
  await render();
  for (const name of ['명령 권한', '미션 상태', '진단 준비 상태']) {
    const section = host.querySelector(`section[aria-label="${name}"]`);
    expect(section).not.toBeNull();
    expect(section.querySelector('h3').textContent).toBe(name);
  }
  const reset = host.querySelector('[data-ui="admin-safety-reset"]');
  expect(reset).not.toBeNull();
  expect(reset.closest('details')).toBeNull();
  expect(host.querySelector('[data-ui="safety-gate-reason"]').closest('details')).toBeNull();
  expect(host.querySelector('.safety-gate-banner').classList.contains('safety-gate-unknown')).toBe(true);
  expect(host.querySelector('.safety-gate-banner strong').textContent).toBe('현재 상태 미확인');
  expect(host.querySelector('[aria-label="진단 준비 상태"]').textContent).toContain('미확인');
  expect(fetch).toHaveBeenCalledTimes(1);
  expect(fetch.mock.calls[0][0]).toBe('/api/admin/safety/reset');
  expect(postCalls()).toHaveLength(0);
});

test('blocking reasons remain visible while long raw diagnostics are folded but available', async () => {
  await render();
  const telemetry = snapshot();
  await emit(telemetry);
  const reason = host.querySelector('[data-ui="safety-gate-reason"]');
  expect(reason.textContent).toContain('lanelet_physical_body_cost,engage=false');
  expect(reason.textContent).not.toContain('battery=78');
  expect(reason.closest('details')).toBeNull();
  expect(host.querySelector('.safety-gate-banner strong').textContent).toBe('경로 안전 정지');
  const details = [...host.querySelectorAll('details')];
  expect(details.every(detail => !detail.open)).toBe(true);
  expect(details[0].textContent).toContain(telemetry.safety.gate.message);
  expect(details[2].textContent).toContain('active SENSOR=FRONT1 range=1.23');
  expect(details[2].textContent).toContain('waiting_for_clear_path');
  expect(host.querySelector('[aria-label="미션 상태"]').textContent).toContain('캠핑장 이동 중');
  await act(async () => details[0].querySelector('summary').click());
  expect(details[0].open).toBe(true);
  expect(fetch).toHaveBeenCalledTimes(1);
  await emit(telemetry);
  expect(fetch).toHaveBeenCalledTimes(1);
  expect(postCalls()).toHaveLength(0);
});

test('unstructured reasons and missing components are not hidden or truncated', async () => {
  await render();
  const message = '비상정지 입력 유지: 운영자가 주변 안전과 장치를 직접 확인해야 합니다.';
  await emit(snapshot({
    safety: { gate: { level: 2, operating_state: 'FAULT_HOLD', message,
      missing_nodes: ['/platform'], missing_topics: ['/emergency_stop'] } },
    mission: { phase: 'FUTURE_PHASE', system_health: 'NEW_HEALTH' },
  }));
  expect(host.querySelector('[data-ui="safety-gate-reason"]').textContent).toContain(message);
  const missing = host.querySelector('.safety-missing-list');
  expect(missing.closest('details')).toBeNull();
  expect(missing.textContent).toContain('/platform');
  expect(missing.textContent).toContain('/emergency_stop');
  expect(host.textContent).toContain('미확인 상태 (FUTURE_PHASE)');
  expect(host.textContent).toContain('미확인 상태 (NEW_HEALTH)');
  expect(host.querySelector('.safety-gate-banner').classList.contains('safety-gate-error')).toBe(true);
});

test('stale status cannot appear permitted and retains the last reported stop reason', async () => {
  await render();
  await emit(snapshot({ sources: { 'control.safety_gate': { age_s: 12, rate_hz: 0 } } }));
  expect(host.querySelector('.safety-gate-banner').classList.contains('safety-gate-unknown')).toBe(true);
  expect(host.querySelector('.safety-gate-banner strong').textContent).toBe('현재 상태 미확인');
  expect(host.querySelector('[data-ui="safety-gate-reason"]').textContent).toContain('마지막 수신값');
  expect(host.querySelector('[data-ui="safety-gate-reason"]').textContent).toContain('lanelet_physical_body_cost');
});

test('engage keeps its handler and disabled policy; reset still needs two explicit clicks', async () => {
  const toggle = jest.fn();
  await render({ engageState: true, engageDisabled: true, onToggleEngage: toggle });
  let engage = host.querySelector('.safety-engage-btn');
  expect(engage.textContent).toBe('ENGAGE OFF');
  await act(async () => engage.click());
  expect(toggle).not.toHaveBeenCalled();
  await render({ engageState: false, engageDisabled: false, onToggleEngage: toggle });
  engage = host.querySelector('.safety-engage-btn');
  expect(engage.textContent).toBe('ENGAGE ON');
  await act(async () => engage.click());
  expect(toggle).toHaveBeenCalledTimes(1);
  await act(async () => host.querySelector('[data-ui="admin-safety-reset-request"]').click());
  expect(postCalls()).toHaveLength(0);
  expect(host.querySelector('[role="alertdialog"]')).not.toBeNull();
  await act(async () => host.querySelector('[data-ui="admin-safety-reset-confirm"]').click());
  expect(postCalls()).toEqual([['/api/admin/safety/reset', {
    method: 'POST', headers: { Authorization: `Basic ${btoa('operator:secret')}`, 'Content-Type': 'application/json' },
    body: '{"confirm":true}',
  }]]);
});

test('switching tabs retains one active telemetry socket and map fetches only for map views', async () => {
  await render();
  expect(sockets).toHaveLength(1);
  expect(sockets[0].url).toContain('/ws/telemetry?view=safety');
  await act(async () => sockets[0].onopen());
  expect(sockets[0].send).toHaveBeenCalledWith('lease');
  await act(async () => jest.advanceTimersByTime(4000));
  expect(sockets[0].send).toHaveBeenCalledTimes(2);
  expect(fetch.mock.calls.some(([url]) => url === '/api/telemetry/map' || url === '/api/telemetry')).toBe(false);
  await render({ activeTab: 'gnss' });
  expect(sockets[0].close).toHaveBeenCalledTimes(1);
  expect(sockets).toHaveLength(2);
  expect(sockets[1].url).toContain('/ws/telemetry?view=gnss');
  expect(host.querySelector('[data-ui="admin-safety-reset"]')).toBeNull();
  expect(fetch).toHaveBeenCalledWith('/api/telemetry/session?active=false&view=safety', { method: 'POST', keepalive: true });
  expect(fetch.mock.calls.some(([url]) => url === '/api/telemetry/map')).toBe(false);
  await render({ activeTab: 'trajectory' });
  expect(sockets[1].close).toHaveBeenCalledTimes(1);
  expect(sockets).toHaveLength(3);
  expect(sockets[2].url).toContain('/ws/telemetry?view=trajectory');
  expect(fetch.mock.calls.filter(([url]) => url === '/api/telemetry/map')).toHaveLength(1);
  expect(postCalls().every(([url]) => url.startsWith('/api/telemetry/session?active=false'))).toBe(true);
});
