import React from 'react';
import { createRoot } from 'react-dom/client';
import { act as legacyAct } from 'react-dom/test-utils';
import App from './App';

// HH_261001 - Actual WebGL rendering is browser-tested; DOM integration must not require a GPU.
jest.mock('./RangerNavigationScene', () => function MockNavigationScene() {
  return <div data-testid="ranger-navigation-scene" />;
});

const act = React.act || legacyAct;
const mission = {
  active: true, generation: 42, site: 'B2', owner: 'robot', intent: 'delivery',
  service_state_name: 'MOVING_TO_SITE', phase: 'DRIVING', description: '배송 이동 중',
};
const snapshot = () => ({
  schema_version: 1, connected: true, mission: { ...mission },
  pose: { x: 0, y: 0, yaw: 0, frame_id: 'map', age_s: 0.1 },
  route: { points: [[0, 0], [5, 0], [12, 2]], frame_id: 'map', age_s: 0.1, valid: true },
  perception: { frame_id: 'map', age_s: 0.1, points: [], objects: [] },
  motion: { speed_mps: 0.5 }, battery: { percentage: 85 },
  progress: { valid: true, remaining_distance_m: 12, remaining_time_s: 24 },
  sensors: {},
});
const missionFrame = (overrides = {}) => ({
  states: Object.fromEntries(Array.from({ length: 13 }, (_, i) => [`B${i + 1}`, i === 1])),
  mission_dispatch_active: true, mission_dispatch_generation: 42,
  mission_dispatch_site: 'B2', mission_dispatch_owner: 'robot', mission_dispatch_intent: 'delivery',
  service_state: 1, service_state_name: 'MOVING_TO_SITE', mission_phase: 'DRIVING',
  mission_source: 'campsite', system_health: 'OK', battery: 85, engage: true,
  ...overrides,
});

let host, root, originalFetch, originalWebSocket, sockets;
class TestSocket {
  static OPEN = 1;
  constructor(url) {
    this.url = url;
    this.readyState = 1;
    this.send = jest.fn();
    this.close = jest.fn(() => { this.readyState = 3; });
    sockets.push(this);
  }
}
const emit = frame => sockets[0].onmessage({ data: JSON.stringify(frame) });
async function renderApp(props = {}) {
  await act(async () => root.render(<App {...props} />));
}
async function startMission() {
  await renderApp();
  await act(async () => {
    sockets[0].onopen();
    emit(missionFrame());
  });
}
async function tap(element) {
  expect(element).not.toBeNull();
  await act(async () => {
    element.dispatchEvent(new MouseEvent('pointerdown', { bubbles: true, clientX: 40, clientY: 40 }));
    element.dispatchEvent(new MouseEvent('pointerup', { bubbles: true, clientX: 40, clientY: 40 }));
    element.dispatchEvent(new MouseEvent('click', { bubbles: true, cancelable: true, clientX: 40, clientY: 40 }));
  });
}
function expectNoCommands() {
  expect(fetch.mock.calls.every(([, options]) => !options?.method || options.method === 'GET')).toBe(true);
  sockets.forEach(socket => expect(socket.send).not.toHaveBeenCalled());
}

beforeEach(() => {
  global.IS_REACT_ACT_ENVIRONMENT = true;
  jest.useFakeTimers();
  originalFetch = global.fetch;
  originalWebSocket = global.WebSocket;
  sockets = [];
  global.WebSocket = TestSocket;
  global.fetch = jest.fn(async url => ({
    ok: true, status: 200,
    json: async () => url === '/api/driving' ? snapshot() : {},
  }));
  host = document.createElement('div');
  document.body.appendChild(host);
  root = createRoot(host);
});

afterEach(() => {
  act(() => root.unmount());
  host.remove();
  global.fetch = originalFetch;
  global.WebSocket = originalWebSocket;
  delete global.IS_REACT_ACT_ENVIRONMENT;
  jest.useRealTimers();
});

test('actual App driving overlay dismisses onto the same existing header and site controls', async () => {
  await startMission();
  const display = host.querySelector('[data-testid="driving-display"]');
  const controlScreen = host.querySelector('[data-ui="operator-control-screen"]');
  const originalLogo = host.querySelector('.control-header .ch-logo img');
  const originalSiteButton = host.querySelector('[data-ui="operator-site-B2"]');
  const originalSiteGrid = host.querySelector('.toggle-grid');
  expect(display).not.toBeNull();
  expect(controlScreen).not.toBeNull();
  expect(originalLogo.getAttribute('src')).toBe('/월악산_국립공원_로고.jpg');
  expect(originalSiteButton.textContent).toContain('B2');
  expect(originalSiteButton.textContent).toContain('ON');
  expect(originalSiteButton.disabled).toBe(true);
  expect(originalSiteGrid.querySelectorAll('.toggle-card')).toHaveLength(6);
  expectNoCommands();

  await tap(display);
  expect(host.querySelector('[data-testid="driving-display"]')).toBeNull();
  expect(host.querySelector('[data-ui="operator-control-screen"]')).toBe(controlScreen);
  expect(host.querySelector('.control-header .ch-logo img')).toBe(originalLogo);
  expect(host.querySelector('[data-ui="operator-site-B2"]')).toBe(originalSiteButton);
  expect(host.querySelector('.toggle-grid')).toBe(originalSiteGrid);
  expect(host.textContent).toContain('배송 목적지 선택');
  expect(originalSiteButton.disabled).toBe(true);
  expect(host.querySelector('[data-ui="open-driving-display"]')).not.toBeNull();
  expectNoCommands();

  await act(async () => emit(missionFrame()));
  expect(host.querySelector('[data-testid="driving-display"]')).toBeNull();
  expect(host.querySelector('[data-ui="operator-site-B2"]')).toBe(originalSiteButton);
  expectNoCommands();
});

test('idle home opens the real map only by its button and returns home without a command', async () => {
  // HH_261002 - Map inspection does not create a delivery, recall, or manual drive.
  const idle = { active: false, generation: 0, site: '', intent: '', owner: '' };
  global.fetch = jest.fn(async url => ({ ok: true, status: 200, json: async () =>
    url === '/api/driving' ? { ...snapshot(), mission: idle, route: { valid: false,
      points: [], frame_id: 'map' }, base_map: { valid: true, frame_id: 'map',
      source: '/map/markers', polylines: [{ namespace: 'lanelet/centerline',
        points: [[0, 0], [5, 0]] }] } } : {} }));
  await renderApp();
  await act(async () => {
    sockets[0].onopen();
    emit(missionFrame({ states: Object.fromEntries(Array.from({ length: 13 }, (_, i) => [`B${i + 1}`, false])),
      mission_dispatch_active: false, mission_dispatch_generation: 0,
      mission_dispatch_site: '', mission_dispatch_owner: '', mission_dispatch_intent: '',
      service_state: 0, service_state_name: 'DROP_ZONE_WAIT', mission_phase: 'READY' }));
  });
  expect(host.querySelector('[data-ui="operator-waiting-screen"]')).not.toBeNull();
  expect(host.querySelector('[data-ui="idle-navigation-map"]')).toBeNull();
  await tap(host.querySelector('[data-ui="open-idle-navigation-map"]'));
  const map = host.querySelector('[data-ui="idle-navigation-map"]');
  expect(map).not.toBeNull();
  expect(map.querySelector('[data-map-source="/map/markers"]')).not.toBeNull();
  expect(map.querySelector('.dd-path-core')).toBeNull();
  expect(map.querySelector('.dd-back-action').textContent).toBe('홈으로');
  expectNoCommands();
  await tap(map.querySelector('.dd-back-action'));
  expect(host.querySelector('[data-ui="idle-navigation-map"]')).toBeNull();
  expect(host.querySelector('[data-ui="operator-waiting-screen"]')).not.toBeNull();
  expectNoCommands();
});

test('existing reopen button changes only presentation and preserves active site', async () => {
  await startMission();
  await tap(host.querySelector('[data-testid="driving-display"]'));
  const originalSiteButton = host.querySelector('[data-ui="operator-site-B2"]');
  await tap(host.querySelector('[data-ui="open-driving-display"]'));
  expect(host.querySelector('[data-testid="driving-display"]')).not.toBeNull();
  expect(host.querySelector('[data-ui="operator-site-B2"]')).toBe(originalSiteButton);
  expect(originalSiteButton.textContent).toContain('ON');
  expectNoCommands();
});

test('driving controls stay in view and confirmed stop uses the existing stop endpoint', async () => {
  await startMission();
  await tap(host.querySelector('.dd-theme-action'));
  expect(host.querySelector('[data-testid="driving-display"]').classList.contains('dd-theme-dark')).toBe(true);
  expect(host.querySelector('[data-testid="driving-display"]')).not.toBeNull();
  await tap(host.querySelector('.dd-stop-action'));
  expect(host.querySelector('.dd-stop-dialog')).not.toBeNull();
  expect(fetch.mock.calls.some(([url, options]) => url === '/ui/stop' && options?.method === 'POST')).toBe(false);
  await tap(host.querySelector('.dd-stop-confirm'));
  expect(fetch.mock.calls.filter(([url, options]) => url === '/ui/stop' && options?.method === 'POST')).toHaveLength(1);
  expect(host.querySelector('[data-testid="driving-display"]')).not.toBeNull();
  expect(sockets.every(socket => socket.send.mock.calls.length === 0)).toBe(true);
});

test('diagnostic loss during SAFETY_STOP keeps the active mission view and manual dismissal latch', async () => {
  await startMission();
  const allSitesOff = Object.fromEntries(Array.from({ length: 13 }, (_, i) => [`B${i + 1}`, false]));
  await act(async () => emit(missionFrame({
    states: allSitesOff, engage: false, mission_phase: 'SAFETY_STOP', system_health: 'ERROR',
  })));
  const display = host.querySelector('[data-testid="driving-display"]');
  expect(display).not.toBeNull();
  expect(display.textContent).toContain('안전 정지');
  expect(display.querySelector('[role="alert"]').textContent).toContain('시스템 오류');
  expectNoCommands();

  // HH_261002 - An accepted mission cannot fall into standby merely because
  // its site toggle and diagnostic health dropped during a safety hold.
  await act(async () => { jest.advanceTimersByTime(11_000); });
  expect(host.querySelector('[data-ui="operator-waiting-screen"]')).toBeNull();
  expect(host.querySelector('[data-testid="driving-display"]')).not.toBeNull();
  expectNoCommands();

  await tap(display);
  expect(host.querySelector('[data-testid="driving-display"]')).toBeNull();
  await act(async () => emit(missionFrame({
    states: allSitesOff, engage: false, mission_phase: 'SAFETY_STOP', system_health: 'ERROR',
  })));
  expect(host.querySelector('[data-testid="driving-display"]')).toBeNull();
  expectNoCommands();
});

test('the driving view closes when mission authority ends after a safety hold', async () => {
  await startMission();
  await act(async () => emit(missionFrame({ mission_phase: 'SAFETY_STOP', system_health: 'ERROR' })));
  expect(host.querySelector('[data-testid="driving-display"]')).not.toBeNull();
  await act(async () => emit(missionFrame({
    states: Object.fromEntries(Array.from({ length: 13 }, (_, i) => [`B${i + 1}`, false])),
    mission_dispatch_active: false, mission_dispatch_generation: 0,
    mission_dispatch_site: '', mission_dispatch_owner: '', mission_dispatch_intent: '',
    service_state: 0, service_state_name: 'DROP_ZONE_WAIT', mission_phase: 'READY', system_health: 'OK',
  })));
  expect(host.querySelector('[data-testid="driving-display"]')).toBeNull();
  expectNoCommands();
});

test.each([
  ['delivery', null],
  ['return', {
    states: Object.fromEntries(Array.from({ length: 13 }, (_, i) => [`B${i + 1}`, false])),
    service_state: 9, service_state_name: 'RETURNING_TO_DROP_ZONE', returning: true,
  }],
  ['recall', {
    mission_dispatch_intent: 'recall', robot_recall_site: 'B2',
  }],
  ['manual', {
    states: Object.fromEntries(Array.from({ length: 13 }, (_, i) => [`B${i + 1}`, false])),
    mission_dispatch_active: false, mission_dispatch_generation: 0,
    mission_dispatch_site: '', mission_dispatch_owner: '', mission_dispatch_intent: '',
    mission_source: 'manual',
  }],
])('existing %s stop button requires an explicit yes', async (_, frame) => {
  await startMission();
  await tap(host.querySelector('[data-testid="driving-display"]'));
  if (frame) await act(async () => emit(missionFrame(frame)));
  await tap(host.querySelector('.preview-stop-btn'));
  expect(host.querySelector('[data-ui="operator-stop-confirm-dialog"]')).not.toBeNull();
  expect(fetch.mock.calls.some(([url, options]) => url === '/ui/stop' && options?.method === 'POST')).toBe(false);

  await tap(host.querySelector('[data-ui="operator-stop-confirm-no"]'));
  expect(host.querySelector('[data-ui="operator-stop-confirm-dialog"]')).toBeNull();
  expect(fetch.mock.calls.some(([url, options]) => url === '/ui/stop' && options?.method === 'POST')).toBe(false);

  await tap(host.querySelector('.preview-stop-btn'));
  await tap(host.querySelector('[data-ui="operator-stop-confirm-yes"]'));
  expect(fetch.mock.calls.filter(([url, options]) => url === '/ui/stop' && options?.method === 'POST')).toHaveLength(1);
});

test('stale stop confirmation cannot stop a newly admitted mission', async () => {
  await startMission();
  await tap(host.querySelector('[data-testid="driving-display"]'));
  await tap(host.querySelector('.preview-stop-btn'));
  await act(async () => emit(missionFrame({ mission_dispatch_generation: 43 })));
  await tap(host.querySelector('[data-ui="operator-stop-confirm-yes"]'));
  expect(fetch.mock.calls.some(([url, options]) => url === '/ui/stop' && options?.method === 'POST')).toBe(false);
});

test('paging the existing destination controls survives reopen and touch return', async () => {
  await startMission();
  await tap(host.querySelector('[data-testid="driving-display"]'));
  await tap(host.querySelector('.page-arrow.right'));
  const pageTwoSite = host.querySelector('[data-ui="operator-site-B7"]');
  expect(pageTwoSite).not.toBeNull();
  expect(host.querySelector('[data-ui="operator-site-B2"]')).toBeNull();
  await tap(host.querySelector('[data-ui="open-driving-display"]'));
  await tap(host.querySelector('[data-testid="driving-display"]'));
  expect(host.querySelector('[data-ui="operator-site-B7"]')).toBe(pageTwoSite);
  expect(host.querySelector('[data-ui="operator-site-B2"]')).toBeNull();
  expectNoCommands();
});

test('real App exposes unloading confirmation instead of covering it with driving display', async () => {
  await startMission();
  await act(async () => emit(missionFrame({
    service_state: 6, service_state_name: 'UNLOAD_WAIT', mission_phase: 'ARRIVED', site: 'B2',
  })));
  expect(host.querySelector('[data-testid="driving-display"]')).toBeNull();
  expect(host.querySelector('[data-ui="operator-arrival-return-confirm"]')).not.toBeNull();
  expectNoCommands();
});

test('Recall first completion reveals moving navigation, then final loading confirmation returns', async () => {
  await startMission();
  const recallWait = missionFrame({
    mission_dispatch_intent: 'recall', robot_recall_site: 'B2',
    service_state: 8, service_state_name: 'GUEST_LOADING_WAIT',
    mission_phase: 'ARRIVED', site: 'B2', arrived: 'B2',
    recall_final_return_ready: false,
  });
  await act(async () => emit(recallWait));
  const firstConfirmation = host.querySelector('[data-ui="operator-arrival-return-confirm"]');
  expect(firstConfirmation).not.toBeNull();
  expect(firstConfirmation.textContent).toContain('정리 완료 · 사이트 재진입');
  expect(host.querySelector('[data-testid="driving-display"]')).toBeNull();

  await tap(firstConfirmation);
  expect(sockets[0].send).toHaveBeenCalledTimes(1);
  expect(JSON.parse(sockets[0].send.mock.calls[0][0])).toMatchObject({
    usage_complete: true, site: 'B2', mission_generation: 42,
    recall_final_return: false,
  });

  // HH_261002 - Arrival identity remains latched for the second confirmation,
  // but the accepted controller turnaround owns the visible moving phase.
  await act(async () => emit(missionFrame({
    mission_dispatch_intent: 'recall', robot_recall_site: 'B2',
    service_state: 9, service_state_name: 'RETURN_WITH_CARGO',
    service_state_description: 'camping_site_maneuver_controller:CRAB_IN',
    mission_phase: 'DRIVING', recall_final_return_ready: false,
    returning: true,
  })));
  expect(host.querySelector('[data-testid="driving-display"]')).not.toBeNull();
  expect(host.querySelector('[data-ui="operator-arrival-return-confirm"]')).toBeNull();
  expect(host.querySelector('[data-ui="operator-arrival-return"]')).toBeNull();
  expect(sockets[0].send).toHaveBeenCalledTimes(1);

  await act(async () => emit(missionFrame({
    mission_dispatch_intent: 'recall', robot_recall_site: 'B2',
    service_state: 9, service_state_name: 'RETURN_WITH_CARGO',
    service_state_description: 'camping_site_maneuver_controller:RECALL_RETURN_WAIT',
    mission_phase: 'ARRIVED', recall_final_return_ready: true,
    returning: true,
  })));
  const finalConfirmation = host.querySelector('[data-ui="operator-arrival-return-confirm"]');
  expect(host.querySelector('[data-testid="driving-display"]')).toBeNull();
  expect(finalConfirmation).not.toBeNull();
  expect(finalConfirmation.textContent).toContain('짐 싣기 완료 · 복귀');
  expect(host.querySelector('[data-ui="operator-arrival-return"]')).not.toBeNull();
  expect(sockets[0].send).toHaveBeenCalledTimes(1);
});

test('injected local preview renders the original App and dismisses without any transport', async () => {
  await renderApp({ drivingPreviewSnapshot: snapshot() });
  expect(sockets).toHaveLength(0);
  expect(fetch).not.toHaveBeenCalled();
  const originalLogo = host.querySelector('.control-header .ch-logo img');
  const originalGrid = host.querySelector('.toggle-grid');
  const originalSite = host.querySelector('[data-ui="operator-site-B2"]');
  expect(originalLogo).not.toBeNull();
  expect(originalGrid).not.toBeNull();
  expect(originalSite.textContent).toContain('ON');
  await tap(host.querySelector('[data-testid="driving-display"]'));
  expect(host.querySelector('[data-testid="driving-display"]')).toBeNull();
  expect(host.querySelector('.control-header .ch-logo img')).toBe(originalLogo);
  expect(host.querySelector('.toggle-grid')).toBe(originalGrid);
  expect(host.querySelector('[data-ui="operator-site-B2"]')).toBe(originalSite);
  expect(host.querySelector('.driving-preview-returned')).toBeNull();
  expect(host.querySelector('[data-preview="returned"] .toggle-grid')).toBe(originalGrid);
  expect(host.textContent).toContain('배송 목적지 선택');
  expect(sockets).toHaveLength(0);
  expect(fetch).not.toHaveBeenCalled();
});

test('preview controls are inert and fixture pulses preserve dismissal and selected page', async () => {
  await renderApp({ drivingPreviewSnapshot: snapshot() });
  await tap(host.querySelector('[data-testid="driving-display"]'));
  await tap(host.querySelector('.preview-stop-btn'));
  await tap(host.querySelector('.page-arrow.right'));
  const originalPageTwo = host.querySelector('[data-ui="operator-site-B7"]');
  expect(originalPageTwo).not.toBeNull();
  await renderApp({ drivingPreviewSnapshot: { ...snapshot(), preview_sequence: 2 } });
  expect(host.querySelector('[data-testid="driving-display"]')).toBeNull();
  expect(host.querySelector('[data-ui="operator-site-B7"]')).toBe(originalPageTwo);
  expect(host.querySelector('[data-ui="operator-site-B2"]')).toBeNull();
  expect(sockets).toHaveLength(0);
  expect(fetch).not.toHaveBeenCalled();
});
