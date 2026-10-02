import { distanceToRouteMeters, illustrativeScenerySites,
  illustrativeMapRoads, illustrativeMapScenerySites,
  SHRUB_ROUTE_CLEARANCE_M, TREE_ROUTE_CLEARANCE_M } from './illustrativeScenery';

// HH_261002 - The route is measured navigation data; example decorations must
// never be interpreted as measured obstacles or placed over a route segment.
test('distance is measured to the closest point on each segment, including endpoints', () => {
  const route = [[0, 0], [10, 0], [10, 10]];
  expect(distanceToRouteMeters({ x: 5, y: 3 }, route)).toBeCloseTo(3);
  expect(distanceToRouteMeters({ x: 13, y: 7 }, route)).toBeCloseTo(3);
  expect(distanceToRouteMeters({ x: -3, y: -4 }, route)).toBeCloseTo(5);
});

test('straight route keeps both tree and shrub footprints outside the driving corridor', () => {
  const route = Array.from({ length: 61 }, (_, index) => [index, 0]);
  const { shrubs, trees } = illustrativeScenerySites(route);
  expect(shrubs.length).toBeGreaterThan(0);
  expect(trees.length).toBeGreaterThan(0);
  shrubs.forEach((site) => expect(distanceToRouteMeters(site, route)).toBeGreaterThanOrEqual(SHRUB_ROUTE_CLEARANCE_M));
  trees.forEach((site) => expect(distanceToRouteMeters(site, route)).toBeGreaterThanOrEqual(TREE_ROUTE_CLEARANCE_M));
});

test('nearby return leg rejects trees and shrubs that would occupy either leg', () => {
  const route = [
    ...Array.from({ length: 36 }, (_, index) => [index, 0]),
    ...Array.from({ length: 8 }, (_, index) => [35, index + 1]),
    ...Array.from({ length: 35 }, (_, index) => [34 - index, 8]),
  ];
  const { shrubs, trees } = illustrativeScenerySites(route);
  expect(shrubs.length).toBeGreaterThan(0);
  expect(trees.length).toBeGreaterThan(0);
  [...shrubs, ...trees].forEach((site) => {
    if (site.x >= 5 && site.x <= 30) expect(site.y > 8 || site.y < 0).toBe(true);
  });
  shrubs.forEach((site) => expect(distanceToRouteMeters(site, route)).toBeGreaterThanOrEqual(SHRUB_ROUTE_CLEARANCE_M));
  trees.forEach((site) => expect(distanceToRouteMeters(site, route)).toBeGreaterThanOrEqual(TREE_ROUTE_CLEARANCE_M));
});

test('missing or degenerate path never creates route scenery', () => {
  expect(illustrativeScenerySites([])).toEqual({ shrubs: [], trees: [] });
  expect(illustrativeScenerySites(Array.from({ length: 20 }, () => [1, 1])))
    .toEqual({ shrubs: [], trees: [] });
});

const line = (namespace, marker_id, points) => ({ namespace, marker_id, points });

test('lightweight bounds yield a short independent road and roadside vegetation without a route', () => {
  // HH_261002 - Woraksan publishes left/right bounds, not centerline markers.
  const map = { valid: true, polylines: [
    line('lanelet/left_bound', 20, [[0, 1.8], [6, 1.8]]),
    line('lanelet/right_bound', 21, [[0, -1.8], [6, -1.8]]),
  ] };
  const roads = illustrativeMapRoads(map);
  expect(roads).toHaveLength(1);
  expect(roads[0][0]).toEqual([0, 0]);
  expect(roads[0][roads[0].length - 1]).toEqual([6, 0]);
  const { shrubs, trees } = illustrativeMapScenerySites(map);
  expect(shrubs.length).toBeGreaterThan(0);
  expect(trees.length).toBeGreaterThan(0);
  [...shrubs, ...trees].forEach((site) => expect(site.x).toBeGreaterThanOrEqual(0));
  shrubs.forEach((site) => map.polylines.forEach(({ points }) =>
    expect(distanceToRouteMeters(site, points)).toBeGreaterThanOrEqual(SHRUB_ROUTE_CLEARANCE_M)));
  trees.forEach((site) => map.polylines.forEach(({ points }) =>
    expect(distanceToRouteMeters(site, points)).toBeGreaterThanOrEqual(TREE_ROUTE_CLEARANCE_M)));
});

test('marker IDs pair reordered unequal-count bounds, never unrelated array neighbours', () => {
  const left = line('lanelet/left_bound', 40, [[0, 1], [4, 1], [8, 1]]);
  const right = line('lanelet/right_bound', 41, [[0, -1], [8, -1]]);
  const unrelated = line('lanelet/right_bound', 99, [[100, -1], [108, -1]]);
  const map = { valid: true, polylines: [unrelated, right, left] };
  const roads = illustrativeMapRoads(map);
  expect(roads).toHaveLength(1);
  expect(roads[0][0]).toEqual([0, 0]);
  expect(roads[0][roads[0].length - 1]).toEqual([8, 0]);
  expect(roads[0].length).toBeGreaterThan(2);
  expect(illustrativeMapRoads({ valid: true, polylines: [left, unrelated] })).toEqual([]);
});

test('malformed identities, reversed direction, crossing or implausible widths fail closed', () => {
  const left = line('lanelet/left_bound', 4, [[0, 1], [10, 1]]);
  const right = line('lanelet/right_bound', 5, [[0, -1], [10, -1]]);
  const map = (candidate) => ({ valid: true, polylines: [left, candidate] });
  expect(illustrativeMapRoads(map({ ...right, marker_id: undefined }))).toEqual([]);
  expect(illustrativeMapRoads(map({ ...right, points: [[10, -1], [0, -1]] }))).toEqual([]);
  expect(illustrativeMapRoads(map({ ...right, points: [[0, -1], [10, 2]] }))).toEqual([]);
  expect(illustrativeMapRoads(map({ ...right, points: [[0, -20], [10, -20]] }))).toEqual([]);
  expect(illustrativeMapRoads(map({ ...right, points: [[0, -1], [NaN, -1], [10, -1]] }))).toEqual([]);
  expect(illustrativeMapRoads({ valid: false, polylines: [left, right] })).toEqual([]);
});

test('explicit centerlines take priority and disconnected markers remain disconnected', () => {
  const map = { valid: true, polylines: [
    line('lanelet/centerline', 1, [[0, 0], [8, 0]]),
    line('lanelet/centerline', 2, [[100, 0], [108, 0]]),
    line('lanelet/left_bound', 20, [[0, 2], [8, 2]]),
    line('lanelet/right_bound', 21, [[0, -2], [8, -2]]),
  ] };
  expect(illustrativeMapRoads(map)).toEqual([[[0, 0], [8, 0]], [[100, 0], [108, 0]]]);
  const sites = illustrativeMapScenerySites(map);
  [...sites.shrubs, ...sites.trees].forEach((site) =>
    expect(site.x < 20 || site.x > 90).toBe(true));
});

test('branches, U-turns and the optional active route exclude vegetation across all roads', () => {
  const main = [[0, 0], [20, 0], [40, 0]];
  const branch = [[20, 0], [20, 15]];
  const returnLeg = [[40, 8], [20, 8], [0, 8]];
  const route = [[-2, -8], [42, -8]];
  const map = { valid: true, polylines: [
    line('lanelet/centerline', 1, main), line('lanelet/centerline', 2, branch),
    line('lanelet/centerline', 3, returnLeg),
  ] };
  const { shrubs, trees } = illustrativeMapScenerySites(map, route);
  [...shrubs, ...trees].forEach((site) => {
    const required = shrubs.includes(site) ? SHRUB_ROUTE_CLEARANCE_M : TREE_ROUTE_CLEARANCE_M;
    [main, branch, returnLeg, route].forEach((road) =>
      expect(distanceToRouteMeters(site, road)).toBeGreaterThanOrEqual(required));
  });
});

test('map scenery is deterministic and bounded for many independent roads', () => {
  const map = { valid: true, polylines: Array.from({ length: 80 }, (_, index) =>
    line('lanelet/centerline', index, [[0, index * 20], [100, index * 20]])) };
  const first = illustrativeMapScenerySites(map);
  expect(first).toEqual(illustrativeMapScenerySites(map));
  expect(first.shrubs.length).toBeLessThanOrEqual(120);
  expect(first.trees.length).toBeLessThanOrEqual(48);
});
