import React, { act } from 'react';
import { createRoot } from 'react-dom/client';
import ManualMissionResume, { manualResumeFromSnapshot } from './ManualMissionResume';
import App from './App';

jest.mock('./RangerNavigationScene', () => function MockScene() { return <div />; });

// HH_261002 - Exercise token-bound explicit admission, not synthetic robot
// motion. Network reads, disarm and reconnect must never issue resume commands.
const pending = (changes = {}) => ({ pending: true, token: 'pause-9', site: 'B9',
  intent: 'delivery', stage: 'return', can_resume: true, reason: '', message: '', ...changes });
let host, root, originalFetch, originalWebSocket, sockets;
class TestSocket {
  static OPEN = 1;
  constructor() { this.readyState = 1; this.send = jest.fn(); this.close = jest.fn(); sockets.push(this); }
}
const render = async element => { await act(async () => root.render(element)); };
const button = () => host.querySelector('[data-ui="manual-mission-resume-confirm"]');
const click = async () => { await act(async () => button().click()); };
const emit = async snapshot => { await act(async () => sockets[sockets.length - 1].onmessage({ data: JSON.stringify(snapshot) })); };
const posts = () => fetch.mock.calls.filter(([, options]) => options?.method === 'POST');

beforeEach(() => {
  global.IS_REACT_ACT_ENVIRONMENT = true;
  jest.useFakeTimers();
  originalFetch = global.fetch;
  originalWebSocket = global.WebSocket;
  sockets = [];
  global.WebSocket = TestSocket;
  global.fetch = jest.fn(async () => ({ ok: true, json: async () => ({ success: true, accepted: true }) }));
  host = document.createElement('div'); document.body.appendChild(host);
  root = createRoot(host);
});
afterEach(() => {
  act(() => root.unmount()); host.remove();
  global.fetch = originalFetch; global.WebSocket = originalWebSocket;
  delete global.IS_REACT_ACT_ENVIRONMENT; jest.useRealTimers();
});

test('optional/cleared/malformed backend contracts never produce authority', () => {
  expect(manualResumeFromSnapshot(null, {})).toBeNull();
  for (const value of [null, {}, { pending: false }, { pending: true, token: 9 }, { pending: true, token: ' ' }]) {
    expect(manualResumeFromSnapshot(pending(), { manual_resume: value })).toBeNull();
  }
  expect(manualResumeFromSnapshot(null, { manual_resume: pending({ can_resume: 'true' }) }).can_resume).toBe(false);
});

test('minimal broadcasts preserve pending; replacement mission clears stale token', () => {
  const value = pending();
  expect(manualResumeFromSnapshot(value, { battery: 80 })).toBe(value);
  expect(manualResumeFromSnapshot(value, { mission_dispatch_active: true, manual_resume: value })).toBeNull();
});

test('shows paused return destination and never posts on mounting or status updates', async () => {
  await render(<ManualMissionResume resume={pending({ can_resume: false, reason: 'manual_active', message: '수동 제어를 먼저 종료해주세요.' })} connected />);
  expect(host.textContent).toContain('수동 개입으로 자율주행 일시정지');
  expect(host.textContent).toContain('B9에서 대기·충전 장소로 복귀');
  expect(host.textContent).toContain('manual_active'); expect(button().disabled).toBe(true);
  await render(<ManualMissionResume resume={pending()} connected />);
  expect(button().disabled).toBe(false); expect(fetch).not.toHaveBeenCalled();
});

test('outbound recall shows its original site and intent', async () => {
  await render(<ManualMissionResume resume={pending({ stage: 'outbound', intent: 'recall', site: 'B7' })} connected />);
  expect(host.textContent).toContain('B7 호출');
});

test('disconnection disables the explicit button; reconnect alone sends nothing', async () => {
  await render(<ManualMissionResume resume={pending()} connected={false} />);
  expect(button().disabled).toBe(true); await click();
  await render(<ManualMissionResume resume={pending()} connected />);
  expect(button().disabled).toBe(false); expect(fetch).not.toHaveBeenCalled();
});

test('one explicit click posts exactly the captured token and suppresses double clicks', async () => {
  let complete;
  fetch.mockImplementation(() => new Promise(resolve => { complete = resolve; }));
  await render(<ManualMissionResume resume={pending()} connected />);
  await act(async () => { button().click(); button().click(); });
  expect(fetch).toHaveBeenCalledTimes(1);
  expect(fetch).toHaveBeenCalledWith('/ui/manual_resume', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{"token":"pause-9"}' });
  await act(async () => complete({ ok: true, json: async () => ({ success: true, accepted: true }) }));
  expect(button().disabled).toBe(true); expect(host.textContent).toContain('재개 요청이 수락');
  await render(<ManualMissionResume resume={pending()} connected />); await click();
  expect(fetch).toHaveBeenCalledTimes(1);
});

test('backend rejection is visible and a retry still requires another click', async () => {
  fetch.mockResolvedValue({ ok: false, json: async () => ({ accepted: false, message: '주변 안전 조건을 확인해주세요.' }) });
  await render(<ManualMissionResume resume={pending()} connected />); await click();
  expect(host.querySelector('[role="alert"]').textContent).toContain('주변 안전');
  expect(button().disabled).toBe(false); expect(fetch).toHaveBeenCalledTimes(1);
});

test('old in-flight response cannot affect a replacement token; clear removes prompt', async () => {
  let complete;
  fetch.mockImplementation(() => new Promise(resolve => { complete = resolve; }));
  await render(<ManualMissionResume resume={pending()} connected />); await click();
  await render(<ManualMissionResume resume={pending({ token: 'pause-10', site: 'B8' })} connected />);
  await act(async () => complete({ ok: false, json: async () => ({ message: 'old request failed' }) }));
  expect(host.textContent).toContain('B8'); expect(host.textContent).not.toContain('old request failed');
  expect(button().disabled).toBe(false);
  await render(<ManualMissionResume resume={null} connected />); expect(button()).toBeNull();
});

test('real App restores optional pause via HTTP on initial connection without auto commands', async () => {
  fetch.mockImplementation(async url => ({ ok: true, json: async () => url === '/ui/state'
    ? { mission_dispatch_active: false, manual_resume: pending() } : {} }));
  await render(<App />); await act(async () => sockets[0].onopen());
  expect(button()).not.toBeNull(); expect(posts()).toHaveLength(0);
  expect(sockets[0].send).not.toHaveBeenCalled();
  await emit({ manual_resume: { pending: false } }); expect(button()).toBeNull();
});

test('real App reconnect reads new state and ignores late HTTP state from old socket', async () => {
  let firstComplete;
  fetch.mockImplementation(url => url === '/ui/state'
    ? new Promise(resolve => { firstComplete = resolve; })
    : Promise.resolve({ ok: true, json: async () => ({}) }));
  await render(<App />); await act(async () => sockets[0].onopen());
  await emit({ manual_resume: pending({ token: 'latest-ws', site: 'B8' }) });
  await act(async () => firstComplete({ ok: true, json: async () => ({ manual_resume: pending({ token: 'stale-http', site: 'B1' }) }) }));
  expect(host.querySelector('[data-ui="manual-mission-resume"]').textContent).toContain('B8');
  expect(host.querySelector('[data-ui="manual-mission-resume"]').textContent).not.toContain('B1');
  await act(async () => sockets[0].onclose());
  expect(button().disabled).toBe(true);
  fetch.mockImplementation(async url => ({ ok: true, json: async () => url === '/ui/state'
    ? { manual_resume: pending({ token: 'reconnected' }) } : {} }));
  await act(async () => jest.advanceTimersByTime(2000));
  await act(async () => sockets[1].onopen());
  expect(button().disabled).toBe(false); expect(posts()).toHaveLength(0);
  await click(); expect(JSON.parse(posts()[0][1].body).token).toBe('reconnected');
});

test('real App does not claim driving while suspended and hides prompt for a new mission', async () => {
  await render(<App />); await act(async () => sockets[0].onopen());
  await emit({ mission_dispatch_active: false, manual_resume: pending(),
    mission_phase: 'DRIVING', service_state: 9, service_state_name: 'RETURN_WITH_CARGO' });
  expect(button()).not.toBeNull();
  expect(host.querySelector('.waiting-runtime-item.mission').textContent).not.toContain('주행 중');
  expect(host.querySelector('.preview-returning')?.textContent || '').not.toContain('복귀 중입니다');
  await emit({ mission_dispatch_active: true, mission_dispatch_generation: 99,
    mission_dispatch_site: 'B1', mission_dispatch_intent: 'delivery', mission_dispatch_owner: 'operator' });
  expect(button()).toBeNull(); expect(posts()).toHaveLength(0);
});
