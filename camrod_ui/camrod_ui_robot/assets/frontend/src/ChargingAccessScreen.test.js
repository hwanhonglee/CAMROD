import React, { act } from 'react';
import { createRoot } from 'react-dom/client';
import ChargingAccessScreen, { useChargingAccess } from './ChargingAccessScreen';
import App from './App';

jest.mock('./RangerNavigationScene', () => function MockNavigationScene() { return <div />; });

// HH_261102 - Browser fixtures verify confirmation and presentation, never send robot commands.
const status = (overrides = {}) => ({
  phase: 'inactive', message: '충전 위치에서 대기 중입니다.',
  can_leave: true, cargo_access_ready: false,
  battery_percentage: 35, minimum_battery_percentage: 35,
  platform_is_charging: true, electrical_stop_available: false,
  platform_feedback_fresh: true,
  physical_disconnect_verified: false, target_label: 'drop_zone 내 대기 위치',
  ...overrides,
});
let host, root, originalFetch, originalWebSocket, sockets, liveStatus, onStop, onServices;
class TestSocket {
  static OPEN = 1;
  constructor() { this.readyState = 1; this.send = jest.fn(); this.close = jest.fn(); sockets.push(this); }
}
const response = body => ({ ok: true, json: async () => body });
const button = key => host.querySelector(`[data-ui="${key}"]`);
const posts = () => fetch.mock.calls.filter(([, options]) => options?.method === 'POST');
const tap = async element => { expect(element).not.toBeNull(); await act(async () => element.click()); };
function Harness({ connected = true, complete = false }) {
  const access = useChargingAccess(connected);
  return <ChargingAccessScreen access={access} batteryChargeComplete={complete} onStop={onStop} onServices={onServices} />;
}
async function render(props = {}) { await act(async () => root.render(<Harness {...props} />)); }
async function poll(next) { liveStatus = next; await act(async () => jest.advanceTimersByTime(1000)); }

beforeEach(() => {
  global.IS_REACT_ACT_ENVIRONMENT = true;
  jest.useFakeTimers();
  originalFetch = global.fetch;
  originalWebSocket = global.WebSocket;
  sockets = [];
  global.WebSocket = TestSocket;
  liveStatus = status();
  global.fetch = jest.fn(async (url, options) => response(url === '/api/charging-access' ? liveStatus
    : options?.method === 'POST' ? { ...liveStatus, success: true } : {}));
  onStop = jest.fn(); onServices = jest.fn();
  host = document.createElement('div'); document.body.appendChild(host); root = createRoot(host);
});
afterEach(() => {
  act(() => root.unmount()); host.remove();
  global.fetch = originalFetch; global.WebSocket = originalWebSocket;
  delete global.IS_REACT_ACT_ENVIRONMENT; jest.useRealTimers();
});

test.each([[-1, false], [34, false], [35, true], [36, true]])('SOC %s obeys the 35%% departure threshold', async (battery, allowed) => {
  liveStatus = status({ battery_percentage: battery });
  await render();
  expect(button('charging-access-leave').disabled).toBe(!allowed);
  expect(button('charging-open-services').disabled).toBe(!allowed);
  expect(posts()).toHaveLength(0);
});

// HH_261102 - Unknown battery feedback must not be presented as a confirmed low-SOC reading.
test.each([null, -1])('unknown SOC %s keeps a short checking message and blocks departure', async battery => {
  liveStatus = status({ battery_percentage: battery });
  await render();
  expect(button('charging-access-message').textContent).toContain('배터리 확인 중');
  expect(button('charging-access-message').textContent).not.toContain('충전이 더 필요');
  expect(button('charging-access-leave').disabled).toBe(true);
  expect(posts()).toHaveLength(0);
});

test('opening and cancelling confirmation never moves or claims an electrical disconnect', async () => {
  await render();
  expect(button('charging-contact-state').textContent).toContain('충전 중');
  expect(host.textContent).toContain('충전 전원을 끄는 기능은 아닙니다');
  expect(host.textContent).not.toContain('충전 완료 신호가 확인');
  await tap(button('charging-access-leave'));
  expect(host.querySelector('[role="dialog"]')).not.toBeNull();
  expect(posts()).toHaveLength(0);
  await tap(button('charging-confirm-cancel'));
  expect(host.querySelector('[role="dialog"]')).toBeNull();
  expect(posts()).toHaveLength(0);
});

test('explicit leave posts once and pending request disables competing services', async () => {
  await render();
  await tap(button('charging-access-leave'));
  let finish;
  fetch.mockImplementation(async (url, options) => options?.method === 'POST'
    ? new Promise(resolve => { finish = resolve; }) : response(liveStatus));
  await tap(button('charging-confirm-move'));
  expect(button('charging-access-leave').disabled).toBe(true);
  expect(button('charging-open-services').disabled).toBe(true);
  await tap(button('charging-access-leave'));
  expect(posts()).toHaveLength(1);
  expect(posts()[0][0]).toBe('/ui/charging_access?action=leave&confirm=true');
  await act(async () => finish(response({ ...status({ phase: 'departing', can_leave: false }), success: true })));
  expect(host.textContent).toContain('앞으로 이동 중');
  expect(button('charging-open-services')).toBeNull();
  expect(button('charging-access-redock')).toBeNull();
});

test('ready stays stopped without an extra completion workflow and allows existing services', async () => {
  // HH_261102 - Staging ends at standstill; selecting an existing service is a separate user action.
  liveStatus = status({ phase: 'ready', platform_is_charging: false, can_leave: false, cargo_access_ready: true });
  await render();
  expect(host.textContent).toContain('정차 완료');
  expect(host.textContent).not.toContain('짐 작업 완료');
  expect(button('charging-contact-state').textContent).toContain('충전 신호 없음');
  await act(async () => jest.advanceTimersByTime(3600000));
  expect(posts()).toHaveLength(0);
  expect(button('charging-access-leave')).toBeNull();
  expect(button('charging-access-redock')).toBeNull();
  expect(button('charging-open-services').disabled).toBe(false);
  await tap(button('charging-open-services'));
  expect(onServices).toHaveBeenCalledTimes(1);
  expect(posts()).toHaveLength(0);
});

test('a ready phase without measured readiness never enables a service movement', async () => {
  liveStatus = status({ phase: 'ready', can_leave: false, cargo_access_ready: false });
  await render();
  expect(host.textContent).toContain('정차 상태 확인 중');
  expect(button('charging-open-services').disabled).toBe(true);
  expect(posts()).toHaveLength(0);
});

test('fresh HTTP with stale vehicle feedback never claims charging or enables a move', async () => {
  liveStatus = status({ platform_feedback_fresh: false });
  await render({ complete: true });
  expect(button('charging-contact-state').textContent).toBe('충전 상태 확인 중');
  expect(host.textContent).not.toContain('충전 완료 신호가 확인');
  expect(button('charging-eligibility').classList.contains('charging-eligible')).toBe(false);
  expect(button('charging-access-leave').disabled).toBe(true);
  expect(button('charging-open-services').disabled).toBe(true);
  expect(posts()).toHaveLength(0);
});

test('phase changes invalidate an open confirmation and missing status blocks movement, not public stop', async () => {
  await render();
  await tap(button('charging-access-leave'));
  await poll(status({ phase: 'departing', can_leave: false }));
  expect(host.querySelector('[role="dialog"]')).toBeNull();
  await render({ connected: false });
  expect(button('charging-contact-state').textContent).toBe('충전 상태 확인 중');
  await tap(button('charging-safety-stop'));
  expect(onStop).not.toHaveBeenCalled();
  expect(button('charging-confirm-stop').disabled).toBe(false);
  await tap(button('charging-confirm-stop'));
  expect(onStop).toHaveBeenCalledTimes(1);
  expect(posts()).toHaveLength(0);
});

test('cancelling mission Stop confirmation never sends a command, even during cargo access', async () => {
  // HH_261102 - Ordinary mission Stop is not an immediate physical emergency-stop input.
  liveStatus = status({ phase: 'departing', can_leave: false });
  await render();
  await tap(button('charging-safety-stop'));
  expect(onStop).not.toHaveBeenCalled();
  await poll(status({ phase: 'ready', platform_feedback_fresh: false }));
  expect(button('charging-confirm-stop').disabled).toBe(false);
  await tap(button('charging-confirm-cancel'));
  expect(onStop).not.toHaveBeenCalled();
  expect(posts()).toHaveLength(0);
});

test('refused POST remains visible without claiming successful departure', async () => {
  await render(); await tap(button('charging-access-leave'));
  fetch.mockImplementation(async (url, options) => options?.method === 'POST'
    ? { ok: false, json: async () => ({ ...status(), success: false, message: '차량 정차 신호가 없습니다.' }) }
    : response(liveStatus));
  await tap(button('charging-confirm-move'));
  expect(host.querySelector('[role="alert"]').textContent).toContain('요청을 확인하지 못했어요');
  expect(button('charging-details').textContent).toContain('차량 정차 신호가 없습니다');
  expect(button('charging-access-leave').disabled).toBe(true);
  expect(button('charging-screen').dataset.phase).toBe('inactive');
});

test('technical reasons stay available behind a closed disclosure without starting movement', async () => {
  // HH_261102 - Short public copy must preserve diagnostic evidence and visible contact-safety advice.
  liveStatus = status({ can_leave: false, message: 'phase=ERROR source=CAN readiness=false' });
  await render();
  const details = button('charging-details');
  expect(details.open).toBe(false);
  expect(details.textContent).toContain('phase=ERROR');
  expect(button('charging-access-message').textContent).not.toContain('phase=');
  expect(host.querySelector('.charging-safety-note').textContent).toContain('충전 접점은 만지지 마세요');
  await tap(details.querySelector('summary'));
  expect(details.open).toBe(true);
  expect(posts()).toHaveLength(0);
});

test('confirmed full charge is distinct from a normal charging contact', async () => {
  await render();
  expect(button('charging-contact-state').textContent).toBe('충전 중');
  await render({ complete: true });
  expect(button('charging-contact-state').textContent).toBe('충전 완료');
});

test.each(['delivery', 'recall'])('actual App allows existing %s selection after stopping inside drop_zone', async intent => {
  await act(async () => root.render(<App />));
  await act(async () => {
    sockets[0].onopen();
    sockets[0].onmessage({ data: JSON.stringify({ states: {}, mission_dispatch_active: false,
      mission_dispatch_generation: 0, service_state: 13, service_state_name: 'CHARGING',
      system_health: 'OK', mission_phase: 'READY', battery: 35 }) });
  });
  expect(button('charging-screen')).not.toBeNull();
  expect(button('charging-access-leave').disabled).toBe(false);
  await poll(status({ phase: 'ready', cargo_access_ready: true, can_leave: false, platform_is_charging: false }));
  await act(async () => sockets[0].onmessage({ data: JSON.stringify({ service_state: 0, service_state_name: 'DROP_ZONE_WAIT' }) }));
  expect(button('charging-screen').dataset.phase).toBe('ready');
  expect(button('operator-open-destination')).toBeNull();
  expect(button('operator-intent-delivery')).toBeNull();
  expect(button('operator-site-B1')).toBeNull();
  await tap(button('charging-open-services'));
  expect(button('service-safety-gate')).not.toBeNull();
  await tap(button('usage-safety-confirm'));
  expect(button('operator-service-selection-screen')).not.toBeNull();
  await tap(button(`operator-intent-${intent}`));
  await tap(button(`operator-service-${intent}-confirm`));
  expect(button('operator-site-B1').disabled).toBe(false);
  expect(sockets[0].send).not.toHaveBeenCalled();
  expect(posts()).toHaveLength(0);
});

// HJ_261105 - Standby chooses the charging screen from the backend charging contact, not the service-state name.
async function bootApp(serviceState, serviceStateName, next) {
  liveStatus = next;
  await act(async () => root.render(<App />));
  await act(async () => {
    sockets[0].onopen();
    sockets[0].onmessage({ data: JSON.stringify({ states: {}, mission_dispatch_active: false,
      mission_dispatch_generation: 0, service_state: serviceState, service_state_name: serviceStateName,
      system_health: 'OK', mission_phase: 'READY', battery: 80 }) });
  });
  await poll(next);
}
const backToStandby = async () => {
  const back = [...host.querySelectorAll('button')].find(b => /(대기|충전) 화면으로/.test(b.textContent));
  await tap(back);
};
const stoppedFrame = () => act(async () => sockets[0].onmessage({ data: JSON.stringify({
  service_state: 16, service_state_name: 'OPERATOR_STOPPED' }) }));

test('charging-screen Stop keeps cargo access while the robot stays on the charger', async () => {
  await bootApp(13, 'CHARGING', status({ battery_percentage: 80 }));
  fetch.mockImplementation(async url => response(url === '/api/charging-access' ? liveStatus : { success: true }));
  await tap(button('charging-safety-stop'));
  await tap(button('charging-confirm-stop'));
  await stoppedFrame();
  await poll(status({ battery_percentage: 80 }));
  expect(button('charging-screen')).not.toBeNull();
  expect(button('charging-access-leave').disabled).toBe(false);
  expect(button('operator-waiting-screen')).toBeNull();
});

test('OPERATOR_STOPPED during service selection returns to cargo access while charging', async () => {
  await bootApp(13, 'CHARGING', status({ battery_percentage: 80 }));
  await tap(button('charging-open-services'));
  await tap(button('usage-safety-confirm'));
  await stoppedFrame();
  expect(button('operator-service-selection-screen')).not.toBeNull();
  await backToStandby();
  expect(button('charging-screen')).not.toBeNull();
  expect(sockets[0].send).not.toHaveBeenCalled();
  expect(posts()).toHaveLength(0);
});

test('after the staging move leaves the charger, a stop shows ordinary standby (no cargo screen)', async () => {
  await bootApp(0, 'DROP_ZONE_WAIT', status({ phase: 'ready', cargo_access_ready: true,
    can_leave: false, platform_is_charging: false, battery_percentage: 80 }));
  expect(button('charging-screen')).not.toBeNull();
  await stoppedFrame();
  await poll(status({ phase: 'inactive', can_leave: false, platform_is_charging: false, battery_percentage: 80 }));
  expect(button('charging-screen')).toBeNull();
  expect(button('operator-waiting-screen')).not.toBeNull();
});

// HJ_261105 - The measured stop is a short loading cue; ordinary standby owns destination selection after it.
const readyStatus = (overrides = {}) => status({ phase: 'ready', cargo_access_ready: true, can_leave: false,
  platform_is_charging: false, battery_percentage: 80, ...overrides });
const backButtonText = () => [...host.querySelectorAll('button')]
  .find(b => /(대기|충전) 화면으로/.test(b.textContent))?.textContent.trim();
const advance = ms => act(async () => jest.advanceTimersByTime(ms));

test('measured stop shows the loading cue for 3 s, then ordinary standby', async () => {
  await bootApp(0, 'DROP_ZONE_WAIT', readyStatus());
  expect(button('charging-screen').dataset.phase).toBe('ready');
  expect(button('charging-access-message').textContent).toBe('로봇에 짐을 실어주세요.');
  await advance(1500);
  expect(button('charging-screen')).not.toBeNull();
  await advance(1600);
  expect(button('charging-screen')).toBeNull();
  expect(button('operator-waiting-screen')).not.toBeNull();
  await tap(button('operator-open-destination'));
  await tap(button('usage-safety-confirm'));
  expect(backButtonText()).toBe('← 대기 화면으로');
  await backToStandby();
  expect(button('operator-waiting-screen')).not.toBeNull();
  expect(posts()).toHaveLength(0);
});

test('an unconfirmed stop keeps the charging screen; the 3 s cue starts only once stop is measured', async () => {
  await bootApp(0, 'DROP_ZONE_WAIT', readyStatus({ cargo_access_ready: false }));
  await advance(10000);
  expect(button('charging-screen')).not.toBeNull();
  expect(host.textContent).toContain('정차 상태 확인 중');
  await poll(readyStatus());
  await advance(1500);
  expect(button('charging-screen')).not.toBeNull();
  // A lost measurement during the cue restarts it from zero once regained.
  await poll(readyStatus({ cargo_access_ready: false }));
  await poll(readyStatus());
  await advance(2500);
  expect(button('charging-screen')).not.toBeNull();
  await advance(1000);
  expect(button('charging-screen')).toBeNull();
  expect(button('operator-waiting-screen')).not.toBeNull();
});

test('during the cue the back button names the charging screen and returns there', async () => {
  await bootApp(0, 'DROP_ZONE_WAIT', readyStatus());
  await tap(button('charging-open-services'));
  await tap(button('usage-safety-confirm'));
  expect(backButtonText()).toBe('← 충전 화면으로');
  await backToStandby();
  expect(button('charging-screen')).not.toBeNull();
});

test('while charging the back button reads "충전 화면으로"; without charging it reads "대기 화면으로"', async () => {
  await bootApp(13, 'CHARGING', status({ battery_percentage: 80 }));
  await tap(button('charging-open-services'));
  await tap(button('usage-safety-confirm'));
  expect(backButtonText()).toBe('← 충전 화면으로');
  await act(async () => root.unmount());
  root = createRoot(host); sockets = [];
  await bootApp(0, 'DROP_ZONE_WAIT', status({ platform_is_charging: false, can_leave: false, battery_percentage: 80 }));
  expect(button('operator-waiting-screen')).not.toBeNull();
  await tap(button('operator-open-destination'));
  await tap(button('usage-safety-confirm'));
  expect(backButtonText()).toBe('← 대기 화면으로');
});
