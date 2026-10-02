import React, { act } from 'react';
import { Color } from 'three';
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
  expect(baseMapLinePositions(baseMap, { x: 10, y: 20 }).map((value) => value + 0)).toEqual([
    0, 0.025, 0, 2, 0.025, 0,
    2, 0.025, 0, 2, 0.025, -2,
  ]);
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

test('map boundaries are green in both themes without recoloring centerlines or changing geometry', () => {
  // HH_261002 - Every colored endpoint corresponds to an unchanged received map vertex.
  const baseMap = { valid: true, polylines: ['lanelet/left_bound', 'lanelet/right_bound',
    'lanelet/centerline'].map(namespace => ({ namespace, points: [[0, 0], [5, 0], [7, 1]] })) };
  const vertices = baseMapLinePositions(baseMap, { x: 0, y: 0 });
  for (const [dark, green, neutral] of [[false, '#1ea65a', '#506c60'], [true, '#4ade80', '#91ada8']]) {
    const colors = baseMapLineColors(baseMap, dark);
    expect(colors).toHaveLength(vertices.length);
    const rgb = new Color(green).toArray();
    expect(colors.slice(0, 24)).toEqual(Array.from({ length: 8 }, () => rgb).flat());
    expect(colors.slice(24)).toEqual(Array.from({ length: 4 }, () => new Color(neutral).toArray()).flat());
  }
  expect(baseMapLineColors({ ...baseMap, valid: false })).toEqual([]);
  expect(baseMapLinePositions(baseMap, { x: 0, y: 0 })).toEqual(vertices);
});

test('filled road geometry never joins disconnected lines or rejects sparse map segments', () => {
  const origin = { x: 10, y: 20 };
  const paths = [[[10, 20], [30, 20]], [[110, 20], [114, 20]]];
  const positions = illustratedRoadPositions(paths, origin, 2, 0.008);
  expect(positions).toHaveLength(36);
  expect(positions.filter((_, index) => index % 3 === 1).every(y => y === 0.008)).toBe(true);
  const xValues = positions.filter((_, index) => index % 3 === 0);
  expect(xValues.every(x => x <= 20 || x >= 100)).toBe(true);
  expect(illustratedRoadPositions(paths, origin, 2, 0.008, 15)).toHaveLength(18);
  expect(illustratedRoadPositions([[[10, 20], [10, 20]]], origin, 2, 0.008)).toEqual([]);
});

test('received boundary paint has physical 0.10 m width and six vertices per segment', () => {
  // HH_261002 - Road paint follows actual Lanelet bounds, not an invented centerline.
  const map = { valid: true, polylines: [{ namespace: 'lanelet/left_bound',
    points: [[10, 20], [30, 20]] }] };
  const positions = boundaryPaintPositions(map, { x: 10, y: 20 });
  expect(LANE_BOUNDARY_WIDTH_M).toBe(0.1);
  expect(positions).toHaveLength(6 * 3);
  const xs = positions.filter((_, index) => index % 3 === 0);
  const heights = positions.filter((_, index) => index % 3 === 1);
  const zs = positions.filter((_, index) => index % 3 === 2);
  expect(Math.min(...xs)).toBeCloseTo(0);
  expect(Math.max(...xs)).toBeCloseTo(20);
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
  expect(positions).toHaveLength(2 * 6 * 3);
  const zFirst = positions.slice(0, 18).filter((_, index) => index % 3 === 2);
  const zSecond = positions.slice(18).filter((_, index) => index % 3 === 2);
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
