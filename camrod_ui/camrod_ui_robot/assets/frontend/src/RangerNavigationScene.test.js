import React, { act } from 'react';
import { Color, WebGLRenderer } from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { createRoot } from 'react-dom/client';
import RangerNavigationScene, { configureCargoPreview, hasIllustrativeRoute,
  navigationCameraSideOffset, baseMapLinePositions, baseMapLineColors, syncBaseMapVisibility,
  projectVisibleObjectLabel, navigationIllustrationInput, illustratedRoadPositions,
  LANE_BOUNDARY_WIDTH_M, boundaryPaintPositions } from './RangerNavigationScene';

jest.mock('three', () => ({ ...jest.requireActual('three'),
  WebGLRenderer: jest.fn().mockImplementation(() => { throw new Error('WebGL unavailable in test'); }),
}));
jest.mock('three/examples/jsm/loaders/GLTFLoader.js', () => ({ GLTFLoader: jest.fn() }));

const data = { connected: true, pose: { x: 0, y: 0, yaw: 0, age_s: 0.1 }, route: [], points: [], objects: [], mission: {} };
let host, root;
beforeEach(() => {
  global.IS_REACT_ACT_ENVIRONMENT = true;
  host = document.createElement('div'); document.body.appendChild(host); root = createRoot(host);
});
afterEach(() => { act(() => root.unmount()); host.remove(); });

test('WebGL failure is explicit and never replaced with a static robot image', () => {
  act(() => root.render(<RangerNavigationScene data={data} />));
  expect(host.textContent).toContain('이 환경에서 3D 렌더링을 사용할 수 없습니다');
  expect(host.querySelector('img, image')).toBeNull();
  expect(host.querySelector('[data-model-status]').getAttribute('data-model-status')).toBe('webgl-unavailable');
  expect(host.textContent).not.toContain('Ranger 3D 모델');
  expect(host.textContent).toContain('전방 인지 수신 대기');
  expect(host.querySelector('[data-navigation="zoom"]').textContent).toBe('외관 보기');
});

test('frame-start time cannot hide telemetry accepted later in the same frame', () => {
  // HH_261002 - Reproduce RAF's older frame timestamp, then retain the real
  // stale timeout when no further telemetry arrives. No fake fresh-data bypass.
  const renderer = { domElement: document.createElement('canvas'), shadowMap: {},
    info: { render: { calls: 0, triangles: 0 } },
    setPixelRatio: jest.fn(), getPixelRatio: () => 1, setClearColor: jest.fn(),
    setSize: jest.fn(), render: jest.fn(), dispose: jest.fn(), forceContextLoss: jest.fn() };
  WebGLRenderer.mockImplementationOnce(() => renderer);
  GLTFLoader.mockImplementationOnce(() => ({ load: jest.fn() }));
  let callback, now = 1000;
  const clock = jest.spyOn(performance, 'now').mockImplementation(() => now);
  const raf = jest.spyOn(window, 'requestAnimationFrame').mockImplementation(next => { callback = next; return 1; });
  const cancel = jest.spyOn(window, 'cancelAnimationFrame').mockImplementation(() => {});
  try {
    act(() => root.render(<RangerNavigationScene data={data} />));
    now = 1010;
    act(() => root.render(<RangerNavigationScene data={{ ...data }} />));
    now = 1015;
    act(() => callback(1005));
    expect(JSON.parse(renderer.domElement.dataset.navigationState).poseFresh).toBe(true);
    now = 2200;
    act(() => callback(2190));
    expect(JSON.parse(renderer.domElement.dataset.navigationState).poseFresh).toBe(false);
  } finally {
    act(() => root.unmount());
    clock.mockRestore(); raf.mockRestore(); cancel.mockRestore();
  }
});

test('model detail pointer/click gestures do not bubble into display dismissal', () => {
  const outer = jest.fn();
  act(() => root.render(<section onClick={outer} onPointerDown={outer} onPointerUp={outer}>
    <RangerNavigationScene data={data} /></section>));
  const button = host.querySelector('[data-testid="ranger-model-detail-toggle"]');
  act(() => button.dispatchEvent(new MouseEvent('pointerdown', { bubbles: true })));
  act(() => button.dispatchEvent(new MouseEvent('pointerup', { bubbles: true })));
  act(() => button.dispatchEvent(new MouseEvent('click', { bubbles: true, cancelable: true })));
  expect(outer).not.toHaveBeenCalled();
  expect(button.getAttribute('aria-pressed')).toBe('true');
  expect(button.textContent).toBe('주행 시점');
});

test('Enter on the model-detail button is isolated from the parent Enter-to-dismiss shortcut', () => {
  const outer = jest.fn();
  act(() => root.render(<section onKeyDown={outer}><RangerNavigationScene data={data} /></section>));
  act(() => host.querySelector('button').dispatchEvent(new KeyboardEvent('keydown', { key: 'Enter', bubbles: true })));
  expect(outer).not.toHaveBeenCalled();
});

test('cached shelf is always hidden and replacement cargo is explicitly display-only', () => {
  const shelf = { visible: true }, cargo = { visible: false };
  const model = { getObjectByName: (name) => ({ accessory_shelf_preview: shelf, accessory_cargo_preview: cargo })[name] };
  expect(configureCargoPreview(model, true)).toBe(true);
  expect(shelf.visible).toBe(false);
  expect(cargo.visible).toBe(true);
  expect(configureCargoPreview(model, false)).toBe(false);
  expect(cargo.visible).toBe(false);
});

test('live route receives the illustrated roadside scene only while pose and route are valid', () => {
  const onRoute = { ...data, route: [[0, 0], [2, 0]] };
  expect(hasIllustrativeRoute(onRoute)).toBe(true);
  expect(hasIllustrativeRoute({ ...onRoute, route: [] })).toBe(false);
  expect(hasIllustrativeRoute({ ...onRoute, connected: false })).toBe(false);
  expect(hasIllustrativeRoute({ ...onRoute, pose: { ...onRoute.pose, age_s: 5 } })).toBe(false);
  act(() => root.render(<RangerNavigationScene data={onRoute} />));
  expect(host.textContent).toContain('실제 경로 · 도로변 예시 배경 (실제 지형 아님)');
  expect(host.querySelector('[data-demo]').getAttribute('data-demo')).toBe('false');
});

test('navigation camera centers the robot and lateral offset belongs only to exterior view', () => {
  // HH_261002 - Route/normal driving must not inherit the exterior orbit's side offset.
  expect(navigationCameraSideOffset(0)).toBe(0);
  expect(navigationCameraSideOffset(0.5)).toBeCloseTo(1.375);
  expect(navigationCameraSideOffset(1)).toBeCloseTo(2.75);
  expect(navigationCameraSideOffset(-1)).toBe(0);
  expect(navigationCameraSideOffset(2)).toBeCloseTo(2.75);
});

test('received base map has metre-space geometry even when there is no route', () => {
  // HH_261002 - The map layer is independent of the mission-bound route artwork.
  const baseMap = { valid: true, polylines: [{ namespace: 'lanelet/left_bound',
    points: [[10, 20], [12, 20], [12, 22]] }] };
  expect(hasIllustrativeRoute({ ...data, route: [] })).toBe(false);
  // HH_261002 - Curved display samples retain metre scale, exact endpoints, and map height.
  const positions = baseMapLinePositions(baseMap, { x: 10, y: 20 }).map(value => value + 0);
  expect(positions.length).toBeGreaterThan(12);
  expect(positions.slice(0, 3)).toEqual([0, 0.025, 0]);
  expect(positions.slice(-3)).toEqual([2, 0.025, -2]);
  expect(positions.filter((_, i) => i % 3 === 1).every(y => y === 0.025)).toBe(true);
  expect(positions.filter((_, i) => i % 3 === 0).every(x => x >= 0 && x <= 2)).toBe(true);
  expect(positions.filter((_, i) => i % 3 === 2).every(z => z >= -2 && z <= 0)).toBe(true);
  expect(baseMapLinePositions({ ...baseMap, valid: false }, { x: 10, y: 20 })).toEqual([]);
  const lines = { visible: false, geometry: { attributes: { position: { count: 4 } } } };
  expect(syncBaseMapVisibility(lines, true)).toBe(true);
  expect(lines.visible).toBe(true);
  expect(syncBaseMapVisibility(lines, false)).toBe(false);
  act(() => root.render(<RangerNavigationScene data={{ ...data, baseMap, route: [] }} />));
  expect(host.textContent).toContain('수신 Lanelet 지도 · 경로 대기');
  expect(host.textContent).not.toContain('수신 Lanelet 지도 · 실제 경로');
});

// HH_261002 - Static road surfaces survive mission cancellation, stale pose, and
// connection loss. Only the independent active route artwork is freshness-bound.
test('map road illustration remains independent of route and pose freshness', () => {
  const baseMap = { valid: true, polylines: [{ namespace: 'lanelet/centerline',
    points: [[10, 20], [30, 20]] }] };
  const idle = navigationIllustrationInput({ ...data, baseMap });
  expect(idle.source).toBe('base_map');
  expect(idle.roads).toEqual([[[10, 20], [30, 20]]]);
  expect(idle.route).toEqual([]);
  const active = navigationIllustrationInput({ ...data, baseMap, route: [[10, 20], [11, 20]] });
  expect(active.roads).toEqual(idle.roads);
  expect(active.route).toHaveLength(2);
  expect(navigationIllustrationInput({ ...data, baseMap, connected: false, pose: null })).toEqual(idle);
  expect(navigationIllustrationInput(data)).toBeNull();
  expect(navigationIllustrationInput({ ...data, route: [[0, 0], [2, 0]] }).source).toBe('route');
});

test('curved boundaries are green in both themes while centerlines and source remain unchanged', () => {
  // HH_261002 - Colors follow curved display vertices; the neutral centerline stays uncurved.
  const baseMap = { valid: true, polylines: ['lanelet/left_bound', 'lanelet/right_bound',
    'lanelet/centerline'].map(namespace => ({ namespace, points: [[0, 0], [5, 0], [7, 1]] })) };
  const vertices = baseMapLinePositions(baseMap, { x: 0, y: 0 });
  const sourceBefore = JSON.stringify(baseMap);
  const greenComponents = vertices.length - 12;
  expect(vertices.slice(-12).map(value => value + 0)).toEqual([
    0, 0.025, 0, 5, 0.025, 0, 5, 0.025, 0, 7, 0.025, -1,
  ]);
  for (const [dark, green, neutral] of [[false, '#1ea65a', '#506c60'], [true, '#4ade80', '#91ada8']]) {
    const colors = baseMapLineColors(baseMap, dark);
    expect(colors).toHaveLength(vertices.length);
    const rgb = new Color(green).toArray();
    expect(colors.slice(0, greenComponents)).toEqual(Array.from({ length: greenComponents / 3 }, () => rgb).flat());
    expect(colors.slice(greenComponents)).toEqual(Array.from({ length: 4 }, () => new Color(neutral).toArray()).flat());
  }
  expect(baseMapLineColors({ ...baseMap, valid: false })).toEqual([]);
  expect(baseMapLinePositions(baseMap, { x: 0, y: 0 })).toEqual(vertices);
  expect(JSON.stringify(baseMap)).toBe(sourceBefore);
});

test('filled road geometry never joins disconnected lines or rejects sparse map segments', () => {
  const origin = { x: 10, y: 20 };
  const paths = [[[10, 20], [30, 20]], [[110, 20], [114, 20]]];
  const positions = illustratedRoadPositions(paths, origin, 2, 0.008);
  expect(positions.length).toBeGreaterThan(36);
  expect(positions.filter((_, index) => index % 3 === 1).every(y => y === 0.008)).toBe(true);
  const xValues = positions.filter((_, index) => index % 3 === 0);
  // HH_261002 - Round caps extend by half-width, never across separate roads.
  expect(xValues.every(x => x <= 21 || x >= 99)).toBe(true);
  const shortRoad = illustratedRoadPositions(paths, origin, 2, 0.008, 15);
  expect(shortRoad.length).toBeGreaterThan(18);
  expect(shortRoad.filter((_, i) => i % 3 === 0).every(x => x >= 99)).toBe(true);
  expect(illustratedRoadPositions([[[10, 20], [10, 20]]], origin, 2, 0.008)).toEqual([]);
});

test('received boundary paint keeps physical 0.10 m width with round end caps', () => {
  // HH_261002 - Road paint follows actual Lanelet bounds, not an invented centerline.
  const map = { valid: true, polylines: [{ namespace: 'lanelet/left_bound',
    points: [[10, 20], [30, 20]] }] };
  const positions = boundaryPaintPositions(map, { x: 10, y: 20 });
  expect(LANE_BOUNDARY_WIDTH_M).toBe(0.1);
  expect(positions.length).toBeGreaterThan(6 * 3);
  const xs = positions.filter((_, index) => index % 3 === 0);
  const heights = positions.filter((_, index) => index % 3 === 1);
  const zs = positions.filter((_, index) => index % 3 === 2);
  expect(Math.min(...xs)).toBeCloseTo(-0.05);
  expect(Math.max(...xs)).toBeCloseTo(20.05);
  expect(heights.every((height) => height === 0.027)).toBe(true);
  expect(Math.min(...zs)).toBeCloseTo(-0.05);
  expect(Math.max(...zs)).toBeCloseTo(0.05);
});

test('paint treats two boundaries independently, excludes centerline, and fails closed on invalid map', () => {
  // HH_261002 - No triangle may bridge separate received boundary polylines.
  const map = { valid: true, polylines: [
    { namespace: 'lanelet/left_bound', points: [[10, 20], [30, 20]] },
    { namespace: 'lanelet/right_bound', points: [[10, 24], [30, 24]] },
    { namespace: 'lanelet/centerline', points: [[10, 22], [30, 22]] },
  ] };
  const positions = boundaryPaintPositions(map, { x: 10, y: 20 });
  const first = boundaryPaintPositions({ ...map, polylines: [map.polylines[0]] }, { x: 10, y: 20 });
  const second = boundaryPaintPositions({ ...map, polylines: [map.polylines[1]] }, { x: 10, y: 20 });
  expect(positions).toEqual([...first, ...second]);
  const zFirst = first.filter((_, index) => index % 3 === 2);
  const zSecond = second.filter((_, index) => index % 3 === 2);
  expect(zFirst.every((z) => Math.abs(z) <= 0.05 + 1e-8)).toBe(true);
  expect(zSecond.every((z) => Math.abs(z + 4) <= 0.05 + 1e-8)).toBe(true);
  expect(boundaryPaintPositions({ ...map, valid: false }, { x: 10, y: 20 })).toEqual([]);
  expect(boundaryPaintPositions({ valid: true, polylines: map.polylines.slice(2) },
    { x: 10, y: 20 })).toEqual([]);
});

test('boundary paint rebases map coordinates without changing width or height', () => {
  const map = { valid: true, polylines: [{ namespace: 'lanelet/right_bound',
    points: [[10, 20], [30, 20]] }] };
  const original = boundaryPaintPositions(map, { x: 10, y: 20 });
  const rebased = boundaryPaintPositions(map, { x: 5, y: 15 });
  expect(rebased).toHaveLength(original.length);
  for (let index = 0; index < original.length; index += 3) {
    expect(rebased[index]).toBeCloseTo(original[index] + 5);
    expect(rebased[index + 1]).toBeCloseTo(original[index + 1]);
    expect(rebased[index + 2]).toBeCloseTo(original[index + 2] - 5);
  }
});

test('side boundary curves feed both the thin line and paint without editing the map', () => {
  // HH_261002 - The former angular center hairline must not remain under curved paint.
  const points = [[0, 0], [4, 0], [4, 4]];
  const map = { valid: true, polylines: [{ namespace: 'lanelet/left_bound', points }] };
  const before = JSON.stringify(map);
  const line = baseMapLinePositions(map, { x: 0, y: 0 });
  expect(line.length).toBeGreaterThan(12);
  expect(baseMapLineColors(map).length).toBe(line.length);
  expect(boundaryPaintPositions(map, { x: 0, y: 0 }).length).toBeGreaterThan(line.length);
  expect(JSON.stringify(map)).toBe(before);
  // There are visual samples rounding the corner, not just rounded strip end caps.
  const xy = [];
  for (let index = 0; index < line.length; index += 3) xy.push([line[index], -line[index + 2]]);
  expect(xy.some(([x, y]) => x < 4 && x > 3.5 && y > 0 && y < 0.5)).toBe(true);
});

test('a measured forward object at the side of the camera keeps a readable edge label', () => {
  // HH_261002 - Side-of-route sensor objects must not disappear at narrow UI aspect ratios.
  expect(projectVisibleObjectLabel({ x: 1.4, y: 0, z: 0.5 }, 800, 600))
    .toEqual({ x: 728, y: 300, edge: 'right' });
  expect(projectVisibleObjectLabel({ x: -1.2, y: 0, z: 0.5 }, 800, 600))
    .toEqual({ x: 72, y: 300, edge: 'left' });
  expect(projectVisibleObjectLabel({ x: 0, y: 0, z: 0.5 }, 800, 600))
    .toEqual({ x: 400, y: 300, edge: '' });
  expect(projectVisibleObjectLabel({ x: 1.4, y: 0, z: 2 }, 800, 600)).toBeNull();
});

test('detail direction cycles four faces without triggering dismissal', () => {
  const outer = jest.fn();
  act(() => root.render(<section onClick={outer} onPointerDown={outer} onPointerUp={outer}>
    <RangerNavigationScene data={data} /></section>));
  expect(host.querySelector('[data-navigation="angle"]')).toBeNull();
  act(() => host.querySelector('[data-navigation="zoom"]').dispatchEvent(new MouseEvent('click', { bubbles: true, cancelable: true })));
  const button = host.querySelector('[data-navigation="angle"]');
  for (const expected of [1, 2, 3, 0]) {
    act(() => button.dispatchEvent(new MouseEvent('pointerdown', { bubbles: true })));
    act(() => button.dispatchEvent(new MouseEvent('pointerup', { bubbles: true })));
    act(() => button.dispatchEvent(new MouseEvent('click', { bubbles: true, cancelable: true })));
    expect(button.getAttribute('data-view-index')).toBe(String(expected));
  }
  expect(outer).not.toHaveBeenCalled();
  expect(host.textContent).not.toContain('임시 선반');
});
