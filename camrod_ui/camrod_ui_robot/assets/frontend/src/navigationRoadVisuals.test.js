// HH_261002 - Artwork can have smoother edges, but never alter received map
// centerlines, merge disconnected bounds, or turn sharp bends into giant spikes.
import { illustratedRoadPositions } from './navigationRoadVisuals';

const origin = { x: 0, y: 0 };
const height = 0.027;
const vertexPairs = positions => Array.from({ length: positions.length / 3 }, (_, index) =>
  [positions[index * 3], positions[index * 3 + 2]]);
const cross = (a, b, c) => (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
const covers = (positions, point) => {
  const vertices = vertexPairs(positions);
  for (let index = 0; index < vertices.length; index += 3) {
    const [a, b, c] = vertices.slice(index, index + 3);
    const signs = [cross(a, b, point), cross(b, c, point), cross(c, a, point)];
    if (Math.abs(cross(a, b, c)) > 1e-12
      && (signs.every(value => value <= 1e-9) || signs.every(value => value >= -1e-9))) return true;
  }
  return false;
};
const distanceToSegment = (point, a, b) => {
  const dx = b[0] - a[0], dz = b[1] - a[1];
  const lengthSquared = dx * dx + dz * dz;
  const t = lengthSquared ? Math.max(0, Math.min(1,
    ((point[0] - a[0]) * dx + (point[1] - a[1]) * dz) / lengthSquared)) : 0;
  return Math.hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dz);
};

test.each([0.10, 2.85, 3.8])('straight %s m strokes keep exact body width and round endpoint extent', width => {
  const positions = illustratedRoadPositions([[[10, 20], [30, 20]]], { x: 10, y: 20 }, width, height);
  expect(positions).toHaveLength(26 * 9);
  const vertices = vertexPairs(positions), xs = vertices.map(p => p[0]), zs = vertices.map(p => p[1]);
  expect(Math.min(...xs)).toBeCloseTo(-width / 2);
  expect(Math.max(...xs)).toBeCloseTo(20 + width / 2);
  expect(Math.min(...zs)).toBeCloseTo(-width / 2);
  expect(Math.max(...zs)).toBeCloseTo(width / 2);
  expect(positions.filter((_, index) => index % 3 === 1).every(y => y === height)).toBe(true);
  expect(vertices.some(([x, z]) => x === 0 && z === 0)).toBe(true);
  expect(vertices.some(([x, z]) => x === 20 && z === 0)).toBe(true);
  expect(covers(positions, [-width * 0.49, 0])).toBe(true);
  expect(covers(positions, [-width * 0.51, 0])).toBe(false);
});

test('diagonal width and every received centerline stay fixed without mutating frozen inputs', () => {
  const paths = Object.freeze([Object.freeze([[100, 200], [103, 204], [106, 208]].map(Object.freeze))]);
  const fixedOrigin = Object.freeze({ x: 100, y: 200 });
  const positions = illustratedRoadPositions(paths, fixedOrigin, 0.1, height);
  const vertices = vertexPairs(positions);
  const normalDistances = vertices.map(([x, z]) => x * 0.8 + z * 0.6);
  expect(Math.min(...normalDistances)).toBeCloseTo(-0.05);
  expect(Math.max(...normalDistances)).toBeCloseTo(0.05);
  for (let t = 0; t <= 1; t += 0.025) expect(covers(positions, [6 * t, -8 * t])).toBe(true);
  expect(paths).toEqual([[[100, 200], [103, 204], [106, 208]]]);
  expect(fixedOrigin).toEqual({ x: 100, y: 200 });
});

test('rebasing translates all stroke vertices uniformly, including sub-millimetre source edges', () => {
  const paths = [[[10, 20], [10.0001, 20.0001], [11, 21]]];
  const first = illustratedRoadPositions(paths, { x: 10, y: 20 }, 0.1, height);
  const rebased = illustratedRoadPositions(paths, { x: 5, y: 15 }, 0.1, height);
  expect(first.length).toBeGreaterThan(26 * 9);
  expect(rebased).toHaveLength(first.length);
  for (let index = 0; index < first.length; index += 3) {
    expect(rebased[index]).toBeCloseTo(first[index] + 5);
    expect(rebased[index + 1]).toBe(first[index + 1]);
    expect(rebased[index + 2]).toBeCloseTo(first[index + 2] - 5);
  }
});

test.each([1, -1])('round outer joins cover both turn directions without a corner gap (%s)', sign => {
  const positions = illustratedRoadPositions([[[0, 0], [4, 0], [4, -4 * sign]]], origin, 2, height);
  for (let step = 0; step <= 20; step += 1) {
    const angle = (-Math.PI / 2 + Math.PI / 2 * step / 20) * sign;
    expect(covers(positions, [4 + Math.cos(angle) * 0.98, Math.sin(angle) * 0.98])).toBe(true);
    expect(covers(positions, [4 * step / 20, 0])).toBe(true);
    expect(covers(positions, [4, 4 * sign * step / 20])).toBe(true);
  }
  expect(covers(positions, [4.9, -0.9 * sign])).toBe(false);
});

test('separate paths and over-limit gaps are capped independently and never bridged', () => {
  const first = [[0, 0], [2, 0]], second = [[20, 0], [22, 0]];
  const split = illustratedRoadPositions([first, second], origin, 0.1, height);
  expect(split).toEqual([...illustratedRoadPositions([first], origin, 0.1, height),
    ...illustratedRoadPositions([second], origin, 0.1, height)]);
  expect(illustratedRoadPositions([[...first, ...second]], origin, 0.1, height, 5)).toEqual(split);
  for (let index = 0; index < split.length; index += 9) {
    const xs = [split[index], split[index + 3], split[index + 6]];
    expect(xs.every(x => x <= 2.05) || xs.every(x => x >= 19.95)).toBe(true);
  }
  expect(covers(split, [10, 0])).toBe(false);
  expect(illustratedRoadPositions([[[0, 0], [20, 0]]], origin, 0.1, height).length).toBe(26 * 9);
  expect(illustratedRoadPositions([[[0, 0], [5, 0]]], origin, 0.1, height, 5).length).toBe(26 * 9);
});

test('duplicates are harmless and isolated vertices do not create invented road patches', () => {
  const line = [[0, 0], [2, 0], [2, 2]];
  expect(illustratedRoadPositions([[[0, 0], [0, 0], [2, 0], [2, 0], [2, 2], [2, 2]]],
    origin, 0.1, height)).toEqual(illustratedRoadPositions([line], origin, 0.1, height));
  expect(illustratedRoadPositions([[], [[0, 0]], [[0, 0], [0, 0]]], origin, 0.1, height)).toEqual([]);
});

test.each([null, undefined, [NaN, 0], [0, Infinity], ['2', 0], [1], [1e100, 0]].map(value => [value]))(
  'invalid or unrenderable point %p splits a path without skipping across it', invalid => {
    const first = [[0, 0], [2, 0]], second = [[20, 0], [22, 0]];
    expect(illustratedRoadPositions([[...first, invalid, ...second]], origin, 0.1, height))
      .toEqual(illustratedRoadPositions([first, second], origin, 0.1, height));
  });

test('holes in a sparse input array also split the stroke', () => {
  const first = [[0, 0], [2, 0]], second = [[20, 0], [22, 0]];
  const sparse = [...first]; sparse.length = 3; sparse.push(...second);
  expect(illustratedRoadPositions([sparse], origin, 0.1, height))
    .toEqual(illustratedRoadPositions([first, second], origin, 0.1, height));
});

test('malformed path and rendering parameters fail closed with finite output', () => {
  const paths = [[[0, 0], [2, 0]]];
  for (const width of [0, -1, NaN, Infinity, '0.1', 1e100, Number.MIN_VALUE, Symbol('width')]) {
    expect(illustratedRoadPositions(paths, origin, width, height)).toEqual([]);
  }
  for (const limit of [0, -1, NaN, '5', -Infinity, Symbol('limit')]) {
    expect(illustratedRoadPositions(paths, origin, 0.1, height, limit)).toEqual([]);
  }
  for (const invalidOrigin of [null, {}, { x: NaN, y: 0 }, { x: 0, y: Infinity }]) {
    expect(illustratedRoadPositions(paths, invalidOrigin, 0.1, height)).toEqual([]);
  }
  for (const invalidHeight of [NaN, Infinity, '0.1', 1e100]) {
    expect(illustratedRoadPositions(paths, origin, 0.1, invalidHeight)).toEqual([]);
  }
  expect(illustratedRoadPositions(null, origin, 0.1, height)).toEqual([]);
  expect(illustratedRoadPositions([null, {}, ...paths], origin, 0.1, height))
    .toEqual(illustratedRoadPositions(paths, origin, 0.1, height));
});

test.each([[[0, 0], [2, 0], [0, 0]], [[0, 0], [2, 0], [0, 0.000001], [2, 0.000002]]].map(path => [path]))(
  'tight and reversing bends stay within half-width of their unchanged input segments', path => {
    const positions = illustratedRoadPositions([path], origin, 3.8, height);
    const vertices = vertexPairs(positions), rendered = path.map(([x, y]) => [x, -y]);
    expect(positions.every(Number.isFinite)).toBe(true);
    for (const vertex of vertices) {
      const distance = Math.min(...rendered.slice(1).map((end, index) => distanceToSegment(vertex, rendered[index], end)));
      expect(distance).toBeLessThanOrEqual(1.9 + 1e-8);
    }
    for (let index = 0; index < vertices.length; index += 3) {
      // Clockwise XZ winding gives consistently upward-facing THREE normals.
      expect(cross(...vertices.slice(index, index + 3))).toBeLessThan(0);
    }
  });

test('closed paths join the final edge to the first edge instead of adding endpoint caps', () => {
  const ring = [[0, 0], [4, 0], [4, 4], [0, 4], [0, 0]];
  const positions = illustratedRoadPositions([ring], origin, 0.1, height);
  expect(positions).toHaveLength((4 * 2 + 4 * 6) * 9);
  ring.forEach(([x, y]) => expect(covers(positions, [x, -y])).toBe(true));
  expect(covers(positions, [2, -2])).toBe(false);
});

test('tessellation has a fixed per-segment budget and does not subdivide long straight edges', () => {
  const path = Array.from({ length: 2001 }, (_, index) => [index * 0.1, Math.sin(index * 0.05)]);
  const positions = illustratedRoadPositions([path], origin, 0.1, height);
  expect(positions.length).toBeLessThanOrEqual((path.length - 1) * 26 * 9);
  expect(positions.length).toBeGreaterThan((path.length - 1) * 2 * 9);
  expect(positions.every(Number.isFinite)).toBe(true);
  expect(illustratedRoadPositions([[[0, 0], [1000000, 0]]], origin, 0.1, height)).toHaveLength(26 * 9);
});
