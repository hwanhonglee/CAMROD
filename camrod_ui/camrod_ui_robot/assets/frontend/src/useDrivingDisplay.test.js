import React from 'react';
import { createRoot } from 'react-dom/client';
import { act as legacyAct } from 'react-dom/test-utils';
import useDrivingDisplay from './useDrivingDisplay';

const act = React.act || legacyAct;

const dispatch = { active: true, generation: 42, site: 'B2', intent: 'delivery', owner: 'robot' };
const initialProps = {
  missionDispatch: dispatch, serviceStateName: 'MOVING_TO_SITE',
  missionPhase: 'DRIVING', connected: true, blocked: false,
};
const payload = (mission = dispatch) => ({
  mission: { ...mission, phase: 'DRIVING' },
  pose: { x: 1, y: 2, yaw: 0, age_s: 0.25, frame_id: 'map' },
  route: { points: [[1, 2], [3, 4]], age_s: 8, frame_id: 'map', valid: true },
  motion: { speed_mps: 0.4, age_s: 0.1 },
  progress: { valid: true, remaining_distance_m: 2.8, remaining_time_s: 7 },
});
const response = body => ({ ok: true, json: async () => body });

let root;
let container;
let latest;
let props;
let originalFetch;
function Probe(input) {
  latest = useDrivingDisplay(input);
  return null;
}
async function render(overrides = {}) {
  props = { ...props, ...overrides };
  await act(async () => { root.render(<Probe {...props} />); });
}
async function tick(ms) {
  await act(async () => { jest.advanceTimersByTime(ms); });
}

beforeEach(() => {
  jest.useFakeTimers();
  global.IS_REACT_ACT_ENVIRONMENT = true;
  originalFetch = global.fetch;
  global.fetch = jest.fn().mockResolvedValue(response(payload()));
  container = document.createElement('div');
  document.body.appendChild(container);
  root = createRoot(container);
  props = initialProps;
});

afterEach(() => {
  act(() => root.unmount());
  container.remove();
  global.fetch = originalFetch;
  delete global.IS_REACT_ACT_ENVIRONMENT;
  jest.useRealTimers();
});

test('auto opens; dismiss is presentation-only and stays dismissed on heartbeat', async () => {
  await render();
  expect(latest.visible).toBe(true);
  expect(latest.snapshot.motion.speed_mps).toBe(0.4);
  const firstSignal = fetch.mock.calls[0][1].signal;
  act(() => latest.dismiss());
  expect(firstSignal.aborted).toBe(true);
  expect(latest.visible).toBe(false);
  await render({ missionDispatch: { ...dispatch } });
  await tick(5000);
  expect(latest.visible).toBe(false);
  expect(fetch).toHaveBeenCalledTimes(1);
  expect(fetch.mock.calls[0][0]).toBe('/api/driving');
  expect(fetch.mock.calls[0][1].method).toBe('GET');
  expect(latest.snapshot.mission.generation).toBe(42);
});

test('new return leg opens once, while parking keeps the return dismissal latch', async () => {
  await render();
  act(() => latest.dismiss());
  await render({ serviceStateName: 'SITE_ENTRY' });
  expect(latest.visible).toBe(false);
  await render({ serviceStateName: 'UNLOAD_WAIT', missionPhase: 'ARRIVED' });
  expect(latest.canOpen).toBe(false);
  await render({ serviceStateName: 'RETURNING_TO_DROP_ZONE', missionPhase: 'DRIVING' });
  expect(latest.visible).toBe(true);
  act(() => latest.dismiss());
  await render({ serviceStateName: 'DROP_ZONE_PARKING' });
  expect(latest.visible).toBe(false);
  await act(async () => latest.open());
  expect(latest.visible).toBe(true);
});

test.each(['SITE_ARRIVED', 'UNLOAD_WAIT', 'GUEST_LOADING_WAIT',
  'WAITING_FOR_RETURN_REQUEST', 'CHARGING', 'WAITING_FOR_CHARGING',
  'DROP_ZONE_WAIT', 'OPERATOR_STOPPED'])('does not cover required/stationary state %s', async state => {
  await render({ serviceStateName: state });
  expect(latest.visible).toBe(false);
  expect(latest.canOpen).toBe(false);
  expect(fetch).not.toHaveBeenCalled();
});

test('Guest recall is eligible and safety hold retains actual measured speed and App phase', async () => {
  const recall = { ...dispatch, owner: 'guest', intent: 'recall' };
  fetch.mockResolvedValue(response(payload(recall)));
  await render({ missionDispatch: recall, serviceStateName: 'RECALL_TO_SITE_ROAD' });
  await render({ missionPhase: 'SAFETY_STOP' });
  expect(latest.visible).toBe(true);
  expect(latest.snapshot.mission.phase).toBe('SAFETY_STOP');
  expect(latest.snapshot.motion.speed_mps).toBe(0.4);
});

test('SAFETY_STOP retains the same leg; dismissal survives recovery and repeated hold frames', async () => {
  await render();
  await render({ missionPhase: 'SAFETY_STOP' });
  expect(latest.visible).toBe(true);
  expect(latest.snapshot.mission.phase).toBe('SAFETY_STOP');
  act(() => latest.dismiss());
  await render({ missionPhase: 'DRIVING' });
  await render({ missionPhase: 'SAFETY_STOP' });
  expect(latest.visible).toBe(false);
  expect(latest.canOpen).toBe(true);
  expect(fetch.mock.calls.every(([, options]) => options?.method === 'GET')).toBe(true);
});

test('admin/modal blocking hides and releases telemetry without changing mission', async () => {
  await render();
  const signal = fetch.mock.calls[0][1].signal;
  await render({ blocked: true });
  expect(latest.visible).toBe(false);
  expect(signal.aborted).toBe(true);
  expect(latest.snapshot.pose).toBeNull();
  expect(latest.snapshot.mission).toMatchObject(dispatch);
  await tick(1000);
  expect(fetch).toHaveBeenCalledTimes(1);
  await render({ blocked: false });
  expect(latest.visible).toBe(true);
});

test('polls at most 10 Hz and never overlaps a request', async () => {
  await render();
  await tick(99);
  expect(fetch).toHaveBeenCalledTimes(1);
  await tick(1);
  expect(fetch).toHaveBeenCalledTimes(2);
  fetch.mockImplementation(() => new Promise(() => {}));
  await tick(100);
  expect(fetch).toHaveBeenCalledTimes(3);
  await tick(2000);
  expect(fetch).toHaveBeenCalledTimes(3);
});

test('disconnect keeps the open display, clears geometry and aborts polling', async () => {
  await render();
  const signal = fetch.mock.calls[0][1].signal;
  await render({ connected: false });
  expect(latest.visible).toBe(true);
  expect(latest.canOpen).toBe(false);
  expect(latest.snapshot.connected).toBe(false);
  expect(latest.snapshot.pose).toBeNull();
  expect(latest.snapshot.route).toBeNull();
  expect(latest.snapshot.motion.speed_mps).toBeNull();
  expect(signal.aborted).toBe(true);
  await tick(2000);
  expect(fetch).toHaveBeenCalledTimes(1);
  await render({ connected: true });
  expect(latest.visible).toBe(true);
  expect(latest.snapshot.connected).toBe(true);
  expect(latest.snapshot.motion.speed_mps).toBe(0.4);
});

test('does not auto-open while disconnected or without an accepted mission', async () => {
  await render({ connected: false });
  expect(latest.visible).toBe(false);
  expect(fetch).not.toHaveBeenCalled();
  await render({ connected: true, missionDispatch: { ...dispatch, active: false } });
  expect(latest.visible).toBe(false);
  expect(fetch).not.toHaveBeenCalled();
  await render({ missionDispatch: dispatch });
  expect(latest.visible).toBe(true);
});

test('idle map opens only by request, accepts only inactive GET, and does not dispatch', async () => {
  // HH_261002 - The existing home stays visible until the operator opens the map.
  const idle = { active: false, generation: 0, site: '', intent: '', owner: '' };
  fetch.mockResolvedValue(response({ ...payload(idle), connected: true,
    base_map: { valid: true, frame_id: 'map', source: '/map/markers',
      polylines: [{ namespace: 'lanelet/centerline', points: [[1, 2], [3, 4]] }] } }));
  await render({ missionDispatch: idle, serviceStateName: 'DROP_ZONE_WAIT', missionPhase: 'READY' });
  expect(latest.visible).toBe(false);
  expect(latest.canOpen).toBe(true);
  expect(fetch).not.toHaveBeenCalled();
  await act(async () => latest.open());
  expect(latest.visible).toBe(true);
  expect(latest.snapshot.mission.active).toBe(false);
  expect(latest.snapshot.base_map.valid).toBe(true);
  expect(fetch.mock.calls.every(([url, options]) => url === '/api/driving' && options.method === 'GET')).toBe(true);
  act(() => latest.dismiss());
  expect(latest.visible).toBe(false);
  await tick(500);
  expect(fetch).toHaveBeenCalledTimes(1);
});

test('idle map rejects stale active response and yields to a new mission identity', async () => {
  const idle = { active: false, generation: 0, site: '', intent: '', owner: '' };
  fetch.mockResolvedValue(response(payload(dispatch)));
  await render({ missionDispatch: idle, serviceStateName: 'DROP_ZONE_WAIT', missionPhase: 'READY' });
  await act(async () => latest.open());
  expect(latest.snapshot.connected).toBe(false);
  expect(latest.snapshot.pose).toBeNull();
  fetch.mockResolvedValue(response(payload(dispatch)));
  await render({ missionDispatch: dispatch, serviceStateName: 'MOVING_TO_SITE', missionPhase: 'DRIVING' });
  expect(latest.visible).toBe(true);
  expect(latest.snapshot.mission).toMatchObject(dispatch);
  act(() => latest.dismiss());
  await render({ missionDispatch: idle, serviceStateName: 'DROP_ZONE_WAIT', missionPhase: 'READY' });
  expect(latest.visible).toBe(false);
  expect(fetch.mock.calls.every(([, options]) => options.method === 'GET')).toBe(true);
});

test('dismissal while disconnected remains latched after reconnect', async () => {
  await render();
  await render({ connected: false });
  act(() => latest.dismiss());
  await render({ connected: true });
  expect(latest.visible).toBe(false);
  expect(fetch).toHaveBeenCalledTimes(1);
});

test('HTTP failure clears previously received geometry without inventing zero speed', async () => {
  await render();
  fetch.mockResolvedValue({ ok: false, status: 503 });
  await tick(500);
  expect(latest.visible).toBe(true);
  expect(latest.snapshot.connected).toBe(false);
  expect(latest.snapshot.pose).toBeNull();
  expect(latest.snapshot.route).toBeNull();
  expect(latest.snapshot.motion.speed_mps).toBeNull();
  expect(latest.snapshot.progress.valid).toBe(false);
});

test('an unavailable backend response never retains its attached geometry', async () => {
  fetch.mockResolvedValue(response({ ...payload(), connected: false }));
  await render();
  expect(latest.snapshot.connected).toBe(false);
  expect(latest.snapshot.pose).toBeNull();
  expect(latest.snapshot.route).toBeNull();
});

test.each([
  { generation: 41 }, { site: 'B3' }, { intent: 'recall' }, { active: false },
])('rejects response identity mismatch %j', async mismatch => {
  fetch.mockResolvedValue(response(payload({ ...dispatch, ...mismatch })));
  await render();
  expect(latest.snapshot.connected).toBe(false);
  expect(latest.snapshot.pose).toBeNull();
  expect(latest.snapshot.mission).toMatchObject(dispatch);
});

test('a new mission generation opens even after the old mission was dismissed', async () => {
  await render();
  act(() => latest.dismiss());
  const next = { ...dispatch, generation: 43, site: 'B3' };
  fetch.mockResolvedValue(response(payload(next)));
  await render({ missionDispatch: next });
  expect(latest.visible).toBe(true);
  expect(latest.snapshot.mission.generation).toBe(43);
});

test('source ages advance monotonically while a delayed request cannot refresh them', async () => {
  await render();
  fetch.mockImplementation(() => new Promise(() => {}));
  await tick(500);
  expect(latest.snapshot.route.age_s).toBeGreaterThanOrEqual(8.5);
  await tick(500);
  expect(latest.snapshot.pose.age_s).toBeGreaterThanOrEqual(1.25);
  await tick(2500);
  expect(latest.snapshot.connected).toBe(false);
  expect(latest.snapshot.pose).toBeNull();
  expect(fetch.mock.calls[1][1].signal.aborted).toBe(true);
});

test('late response after dismiss cannot restore hidden geometry', async () => {
  let resolve;
  fetch.mockImplementation(() => new Promise(done => { resolve = done; }));
  await render();
  act(() => latest.dismiss());
  await act(async () => { resolve(response(payload())); });
  expect(latest.visible).toBe(false);
  expect(latest.snapshot.connected).toBe(false);
  expect(latest.snapshot.pose).toBeNull();
});

test('cloud and classified-object ages both advance while a request is delayed', async () => {
  fetch.mockResolvedValue(response({ ...payload(), perception: {
    age_s: 0.1, points_age_s: 2.4, objects_age_s: 0.1, points: [], objects: [],
  } }));
  await render();
  fetch.mockImplementation(() => new Promise(() => {}));
  await tick(500);
  expect(latest.snapshot.perception.points_age_s).toBeGreaterThanOrEqual(2.9);
  expect(latest.snapshot.perception.objects_age_s).toBeGreaterThanOrEqual(0.6);
});

test('injected preview uses the real dismissal latch without any network fetch', async () => {
  await render({ injectedSnapshot: payload() });
  expect(latest.visible).toBe(true);
  expect(latest.snapshot.motion.speed_mps).toBe(0.4);
  expect(latest.snapshot.mission.generation).toBe(42);
  expect(fetch).not.toHaveBeenCalled();
  await tick(500);
  expect(latest.snapshot.pose.age_s).toBeGreaterThanOrEqual(0.75);
  act(() => latest.dismiss());
  await render({ injectedSnapshot: { ...payload(), preview_sequence: 2 } });
  expect(latest.visible).toBe(false);
  expect(fetch).not.toHaveBeenCalled();
  await render({ serviceStateName: 'RETURNING_TO_DROP_ZONE', injectedSnapshot: {
    ...payload(), mission: { ...dispatch, service_state_name: 'RETURNING_TO_DROP_ZONE' },
  } });
  expect(latest.visible).toBe(true);
  expect(latest.snapshot.mission.service_state_name).toBe('RETURNING_TO_DROP_ZONE');
  expect(fetch).not.toHaveBeenCalled();
});

test('injected preview still rejects mismatched identity and respects disconnected state', async () => {
  await render({ injectedSnapshot: payload({ ...dispatch, generation: 43 }) });
  expect(latest.snapshot.connected).toBe(false);
  expect(latest.snapshot.pose).toBeNull();
  expect(latest.snapshot.mission.generation).toBe(42);
  await render({ injectedSnapshot: payload(), connected: false });
  expect(latest.visible).toBe(true);
  expect(latest.snapshot.connected).toBe(false);
  expect(latest.snapshot.pose).toBeNull();
  expect(fetch).not.toHaveBeenCalled();
});
