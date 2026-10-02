// HH_261002 - Verify the side-boundary shape itself is curved, with a metric
// deviation bound for whole rendered edges and no changes to received data.
import { displayBoundaryLines, MAX_BOUNDARY_CURVE_POINTS_PER_INPUT_POINT,
  MAX_BOUNDARY_VISUAL_DEVIATION_M, smoothDisplayPolyline } from './navigationBoundaryCurves';

const distanceToSegment = (point, a, b) => {
  const dx = b[0] - a[0], dy = b[1] - a[1], lengthSquared = dx * dx + dy * dy;
  const t = lengthSquared ? Math.max(0, Math.min(1,
    ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / lengthSquared)) : 0;
  return Math.hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy);
};
const distanceToPolyline = (point, source) => Math.min(...source.slice(1).map((end, index) =>
  distanceToSegment(point, source[index], end)));
const maximumDeviation = (result, source) => {
  let maximum = 0;
  for (const run of result) {
    for (let index = 1; index < run.length; index += 1) {
      const a = run[index - 1], b = run[index];
      for (let step = 0; step <= 20; step += 1) {
        const t = step / 20;
        maximum = Math.max(maximum, distanceToPolyline(
          [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t], source));
      }
    }
  }
  return maximum;
};

test.each([1, -1])('a %s-handed 90-degree boundary gets a visibly curved shape, not just round paint', sign => {
  const source = [[0, 0], [4, 0], [4, sign * 4]];
  const result = smoothDisplayPolyline(source), points = result[0];
  expect(result).toHaveLength(1);
  expect(points).toHaveLength(11);
  expect(points[0]).toEqual(source[0]);
  expect(points[points.length - 1]).toEqual(source[2]);
  expect(points).not.toContainEqual(source[1]);
  expect(points.filter(([x, y]) => x < 4 && sign * y > 0)).toHaveLength(7);
  expect(points[5][0]).toBeCloseTo(4 - MAX_BOUNDARY_VISUAL_DEVIATION_M, 8);
  expect(points[5][1]).toBeCloseTo(sign * MAX_BOUNDARY_VISUAL_DEVIATION_M, 8);
  const slopes = points.slice(2, 9).map((point, index) =>
    sign * (point[1] - points[index + 1][1]) / (point[0] - points[index + 1][0]));
  expect(slopes.every((slope, index) => index === 0 || slope > slopes[index - 1])).toBe(true);
  expect(maximumDeviation(result, source)).toBeLessThanOrEqual(MAX_BOUNDARY_VISUAL_DEVIATION_M);
});

test.each([5, 30, 60, 120, 150, 164, -30, -120])('oblique %s-degree corners stay bounded along entire output edges', degrees => {
  const angle = degrees * Math.PI / 180;
  const source = [[-10, 0], [0, 0], [10 * Math.cos(angle), 10 * Math.sin(angle)]];
  const result = smoothDisplayPolyline(source);
  expect(result[0].length).toBeGreaterThan(source.length);
  expect(result.flat(2).every(Number.isFinite)).toBe(true);
  expect(maximumDeviation(result, source)).toBeLessThanOrEqual(MAX_BOUNDARY_VISUAL_DEVIATION_M + 1e-12);
});

test('short adjoining edges cannot overlap their fillets or reverse progression', () => {
  const source = [[0, 0], [0.01, 0], [0.01, 0.01], [0.02, 0.01], [0.02, 0.02]];
  const result = smoothDisplayPolyline(source), points = result[0];
  expect(points.every((point, index) => index === 0
    || (point[0] >= points[index - 1][0] && point[1] >= points[index - 1][1]))).toBe(true);
  expect(maximumDeviation(result, source)).toBeLessThanOrEqual(0.12);
  expect(points[0]).toEqual(source[0]);
  expect(points[points.length - 1]).toEqual(source[source.length - 1]);
});

test.each([166, 175, 179.999, 180, -175])('near-reversal %s degrees retains its corner without a spline loop', degrees => {
  const angle = degrees * Math.PI / 180;
  const source = [[-4, 0], [0, 0], [4 * Math.cos(angle), 4 * Math.sin(angle)]];
  expect(smoothDisplayPolyline(source)).toEqual([source]);
});

test('a longer sparse edge has no extra samples and a gentle bend has a wider fillet', () => {
  const source = [[0, 0], [100000, 0], [200000, 0]];
  expect(smoothDisplayPolyline(source)).toEqual([source]);
  const rightAngle = smoothDisplayPolyline([[0, 0], [10, 0], [10, 10]])[0];
  const gentle = smoothDisplayPolyline([[0, 0], [10, 0], [20, 1]])[0];
  expect(gentle).toHaveLength(rightAngle.length);
  expect(10 - gentle[1][0]).toBeGreaterThan(4);
  expect(10 - rightAngle[1][0]).toBeLessThan(0.5);
  expect(maximumDeviation([gentle], [[0, 0], [10, 0], [20, 1]])).toBeLessThanOrEqual(0.12);
});

test('over-limit edges split independent runs and the limit is inclusive', () => {
  const first = [[0, 0], [2, 0], [2, 2]], second = [[20, 0], [22, 0], [22, 2]];
  const result = smoothDisplayPolyline([...first, ...second], { maxSegmentLength: 2 });
  expect(result).toEqual([...smoothDisplayPolyline(first), ...smoothDisplayPolyline(second)]);
  expect(result[0][result[0].length - 1]).toEqual(first[2]);
  expect(result[1][0]).toEqual(second[0]);
  expect(smoothDisplayPolyline([[0, 0], [1000000, 0]])).toEqual([[[0, 0], [1000000, 0]]]);
});

test.each([null, undefined, [NaN, 0], [0, Infinity], ['2', 0], [1], {}].map(value => [value]))(
  'invalid point %p is a gap, never a shortcut between valid runs', invalid => {
    const first = [[0, 0], [2, 0], [2, 2]], second = [[20, 0], [22, 0], [22, 2]];
    expect(smoothDisplayPolyline([...first, invalid, ...second]))
      .toEqual([...smoothDisplayPolyline(first), ...smoothDisplayPolyline(second)]);
  });

test('sparse holes and nonfinite distances split runs, retaining isolated finite endpoints', () => {
  const source = [[0, 0], [2, 0]];
  source.length = 3;
  source.push([20, 0], [22, 0]);
  expect(smoothDisplayPolyline(source)).toEqual([[[0, 0], [2, 0]], [[20, 0], [22, 0]]]);
  expect(smoothDisplayPolyline([[-Number.MAX_VALUE, 0], [Number.MAX_VALUE, 0]]))
    .toEqual([[[-Number.MAX_VALUE, 0]], [[Number.MAX_VALUE, 0]]]);
  expect(smoothDisplayPolyline([[0, 0], null, [1, 1]])).toEqual([[[0, 0]], [[1, 1]]]);
});

test('duplicates are ignored, immutable inputs stay unchanged, and output points never alias them', () => {
  const source = Object.freeze([[0, 0], [0, 0], [4, 0], [4, 0], [4, 4]].map(Object.freeze));
  const result = smoothDisplayPolyline(source);
  expect(result).toEqual(smoothDisplayPolyline([[0, 0], [4, 0], [4, 4]]));
  expect(source).toEqual([[0, 0], [0, 0], [4, 0], [4, 0], [4, 4]]);
  expect(result.flat().every(point => !source.includes(point))).toBe(true);
  expect(smoothDisplayPolyline([[0, 0], [0, 0]])).toEqual([[[0, 0]]]);
});

test('closed loops round the seam and stay closed without an added gap or original sharp corner', () => {
  const source = [[0, 0], [4, 0], [4, 4], [0, 4], [0, 0]];
  const result = smoothDisplayPolyline(source), points = result[0];
  expect(points[0]).toEqual(points[points.length - 1]);
  expect(points[0]).not.toBe(points[points.length - 1]);
  expect(points).toHaveLength(37);
  source.forEach(point => expect(points).not.toContainEqual(point));
  expect(maximumDeviation(result, source)).toBeLessThanOrEqual(0.12);
  expect(smoothDisplayPolyline([[0, 0], [4, 0], [0, 0]]))
    .toEqual([[[0, 0], [4, 0], [0, 0]]]);
});

test('deviation can be tightened/disabled but never increased above the display cap', () => {
  const source = [[0, 0], [10, 0], [10, 10]];
  expect(smoothDisplayPolyline(source, { maxDeviationM: 0 })).toEqual([source]);
  expect(maximumDeviation(smoothDisplayPolyline(source, { maxDeviationM: 0.02 }), source))
    .toBeLessThanOrEqual(0.02);
  expect(smoothDisplayPolyline(source, { maxDeviationM: 100 })).toEqual(smoothDisplayPolyline(source));
});

test('malformed parameters fail closed and unresolvable huge coordinates remain finite', () => {
  const source = [[0, 0], [10, 0], [10, 10]];
  for (const maxDeviationM of [-1, NaN, Infinity, '0.1', Symbol('deviation')]) {
    expect(smoothDisplayPolyline(source, { maxDeviationM })).toEqual([]);
  }
  for (const maxSegmentLength of [0, -1, NaN, -Infinity, '5', Symbol('limit')]) {
    expect(smoothDisplayPolyline(source, { maxSegmentLength })).toEqual([]);
  }
  for (const points of [null, undefined, {}, 'invalid']) expect(smoothDisplayPolyline(points)).toEqual([]);
  const giant = [[1e100, 1e100], [2e100, 1e100], [2e100, 2e100]];
  expect(smoothDisplayPolyline(giant)).toEqual([giant]);
  expect(smoothDisplayPolyline(giant).flat(2).every(Number.isFinite)).toBe(true);
});

test('translated map coordinates preserve the bounded curve and exact open endpoints', () => {
  const origin = [1234567, -4567890];
  const local = [[0, 0], [8, 0], [8, 8], [12, 9]];
  const source = local.map(([x, y]) => [origin[0] + x, origin[1] + y]);
  const result = smoothDisplayPolyline(source), points = result[0];
  expect(points[0]).toEqual(source[0]);
  expect(points[points.length - 1]).toEqual(source[source.length - 1]);
  expect(maximumDeviation(result, source)).toBeLessThanOrEqual(0.12);
});

test('three thousand zigzag input points have fixed linear output cost and retain the complete shape', () => {
  const source = Array.from({ length: 3000 }, (_, index) => [index, index % 2]);
  const result = smoothDisplayPolyline(source), points = result[0];
  expect(points.length).toBeGreaterThan(source.length);
  expect(points.length).toBeLessThanOrEqual(source.length * MAX_BOUNDARY_CURVE_POINTS_PER_INPUT_POINT);
  expect(points[0]).toEqual(source[0]);
  expect(points[points.length - 1]).toEqual(source[source.length - 1]);
  expect(points.flat().every(Number.isFinite)).toBe(true);
});

test('boundary conversion preserves metadata, separate markers, and untouched non-boundary data', () => {
  const first = Object.freeze({ namespace: 'lanelet/left_bound', marker_id: 7, frame_id: 'map',
    points: Object.freeze([[0, 0], [4, 0], [4, 4]].map(Object.freeze)) });
  const second = Object.freeze({ namespace: 'lanelet/right_bound', marker_id: 8,
    points: Object.freeze([[20, 0], [24, 0], [24, 4]].map(Object.freeze)) });
  const centerline = Object.freeze({ namespace: 'lanelet/centerline', marker_id: 9,
    points: Object.freeze([[10, 0], null, [14, 4]].map(point => point && Object.freeze(point))) });
  const source = Object.freeze([first, second, centerline]);
  const result = displayBoundaryLines(source);
  expect(result).toHaveLength(3);
  expect(result[0]).toMatchObject({ namespace: first.namespace, marker_id: 7, frame_id: 'map' });
  expect(result[1]).toMatchObject({ namespace: second.namespace, marker_id: 8 });
  expect(result[0].points).toEqual(smoothDisplayPolyline(first.points)[0]);
  expect(result[1].points).toEqual(smoothDisplayPolyline(second.points)[0]);
  expect(result[2]).toBe(centerline);
  expect(result[2].points).toBe(centerline.points);
  expect(first.points).toEqual([[0, 0], [4, 0], [4, 4]]);
});

test('boundary gaps flatten to independent metadata-preserving lines, never to cross-gap curves', () => {
  const line = { namespace: 'lanelet/left_bound', marker_id: 4,
    points: [[0, 0], [4, 0], null, [20, 0], [24, 0]] };
  expect(displayBoundaryLines([line])).toEqual([
    { ...line, points: [[0, 0], [4, 0]] }, { ...line, points: [[20, 0], [24, 0]] },
  ]);
  expect(displayBoundaryLines(null)).toEqual([]);
  expect(displayBoundaryLines([null, undefined])).toEqual([]);
});
