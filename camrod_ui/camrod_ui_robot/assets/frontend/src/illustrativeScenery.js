// HH_261002 - These bounds protect the received driving route from illustrative
// scenery only. They are not CARLA collision geometry or obstacle-clearance data.
export const SHRUB_ROUTE_CLEARANCE_M = 4.2;
export const TREE_ROUTE_CLEARANCE_M = 6.2;

const finitePoint = (point) => Array.isArray(point) && Number.isFinite(point[0])
  && Number.isFinite(point[1]);

/** Minimum ground-plane distance to every segment, including bends and returns. */
export function distanceToRouteMeters(site, routePoints) {
  if (!Number.isFinite(site?.x) || !Number.isFinite(site?.y) || !Array.isArray(routePoints)) return Infinity;
  let nearest = Infinity;
  for (let index = 1; index < routePoints.length; index += 1) {
    const start = routePoints[index - 1], end = routePoints[index];
    if (!finitePoint(start) || !finitePoint(end)) continue;
    const dx = end[0] - start[0], dy = end[1] - start[1];
    const lengthSquared = dx * dx + dy * dy;
    const projection = lengthSquared > 0
      ? Math.max(0, Math.min(1, ((site.x - start[0]) * dx + (site.y - start[1]) * dy) / lengthSquared))
      : 0;
    nearest = Math.min(nearest, Math.hypot(site.x - start[0] - projection * dx,
      site.y - start[1] - projection * dy));
  }
  return nearest;
}

/** HH_261002 - Reject every example tree/rock-like shrub that intrudes on any
 * received route segment, not merely the segment from which it was generated.
 * This matters at U-turns, switchbacks, and self-near paths. The path itself is
 * telemetry-derived; the accepted scenery remains a visual illustration. */
export function illustrativeScenerySites(routePoints) {
  const shrubs = [], trees = [];
  if (!Array.isArray(routePoints) || routePoints.length < 11) return { shrubs, trees };
  for (let index = 5; index < routePoints.length - 5; index += 5) {
    const before = routePoints[index - 1], center = routePoints[index], after = routePoints[index + 1];
    if (!finitePoint(before) || !finitePoint(center) || !finitePoint(after)) continue;
    const dx = after[0] - before[0], dy = after[1] - before[1];
    const length = Math.hypot(dx, dy);
    if (length < 0.001) continue;
    const normalX = dy / length, normalY = -dx / length;
    for (const side of [-1, 1]) {
      const shrubOffset = 4.5 + ((index * 7 + (side + 1) * 3) % 5) * 0.42;
      const shrub = { x: center[0] + side * normalX * shrubOffset,
        y: center[1] + side * normalY * shrubOffset, scale: 0.72 + (index % 4) * 0.12 };
      if (distanceToRouteMeters(shrub, routePoints) >= SHRUB_ROUTE_CLEARANCE_M) shrubs.push(shrub);
      if (index % 15 === 5) {
        const treeOffset = 8.2 + ((index * 3 + side + 1) % 4) * 0.62;
        const tree = { x: center[0] + side * normalX * treeOffset,
          y: center[1] + side * normalY * treeOffset, scale: 0.82 + (index % 3) * 0.13 };
        if (distanceToRouteMeters(tree, routePoints) >= TREE_ROUTE_CLEARANCE_M) trees.push(tree);
      }
    }
  }
  return { shrubs, trees };
}

const MAP_ROAD_NAMESPACES = new Set([
  'lanelet/centerline', 'lanelet/left_bound', 'lanelet/right_bound',
]);
const MAP_CENTERLINE = 'lanelet/centerline';
const MAP_MAX_LINES = 512;
const MAP_MAX_POINTS = 3000;
const MAP_MAX_SHRUBS = 120;
const MAP_MAX_TREES = 48;

function finiteMapPoint(point) {
  return finitePoint(point) && Math.abs(point[0]) <= 1e8 && Math.abs(point[1]) <= 1e8;
}

function separatedRuns(points) {
  const runs = [];
  let current = [];
  for (const point of points) {
    if (finiteMapPoint(point)) current.push([point[0], point[1]]);
    else {
      if (current.length >= 2) runs.push(current);
      current = [];
    }
  }
  if (current.length >= 2) runs.push(current);
  return runs;
}

function arcSegments(points) {
  const segments = [];
  let total = 0;
  for (let index = 1; index < points.length; index += 1) {
    const start = points[index - 1], end = points[index];
    const dx = end[0] - start[0], dy = end[1] - start[1];
    const length = Math.hypot(dx, dy);
    if (length < 0.001) continue;
    segments.push({ start, dx, dy, length, from: total });
    total += length;
  }
  return { segments, total };
}

function pointAtArc(arc, distance) {
  const segment = arc.segments.find((entry) => distance <= entry.from + entry.length)
    || arc.segments[arc.segments.length - 1];
  const fraction = Math.min(1, Math.max(0, (distance - segment.from) / segment.length));
  return { x: segment.start[0] + fraction * segment.dx,
    y: segment.start[1] + fraction * segment.dy,
    normalX: -segment.dy / segment.length, normalY: segment.dx / segment.length };
}

function separatedFromSites(site, sites, clearance) {
  return sites.every((other) => Math.hypot(site.x - other.x, site.y - other.y) >= clearance);
}

function pairedCenterline(leftPoints, rightPoints, remainingPoints) {
  const leftArc = arcSegments(leftPoints), rightArc = arcSegments(rightPoints);
  if (leftArc.total < 0.5 || rightArc.total < 0.5 || remainingPoints < 2) return null;
  const separation = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
  const forward = separation(leftPoints[0], rightPoints[0])
    + separation(leftPoints[leftPoints.length - 1], rightPoints[rightPoints.length - 1]);
  const backward = separation(leftPoints[0], rightPoints[rightPoints.length - 1])
    + separation(leftPoints[leftPoints.length - 1], rightPoints[0]);
  if (forward > backward + 1e-6) return null;
  const count = Math.min(remainingPoints, 128, Math.max(2, leftPoints.length, rightPoints.length,
    Math.ceil(Math.max(leftArc.total, rightArc.total) / 2.5) + 1));
  const center = [];
  let side = 0;
  for (let index = 0; index < count; index += 1) {
    const fraction = index / (count - 1);
    const left = pointAtArc(leftArc, leftArc.total * fraction);
    const right = pointAtArc(rightArc, rightArc.total * fraction);
    const dx = right.x - left.x, dy = right.y - left.y;
    const width = Math.hypot(dx, dy);
    const headingAgreement = left.normalX * right.normalX + left.normalY * right.normalY;
    const lateral = dx * left.normalX + dy * left.normalY;
    if (width < 0.3 || width > 10 || headingAgreement < 0.3 || Math.abs(lateral) < 0.25
      || (side && Math.sign(lateral) !== side)) return null;
    side = Math.sign(lateral);
    center.push([(left.x + right.x) / 2, (left.y + right.y) / 2]);
  }
  return center;
}

/** HH_261002 - Return independent, illustration-only road spines. Prefer
 * explicit centerline markers. The lightweight Woraksan map has only bounds:
 * pair left marker N with right marker N+1, validate their physical geometry,
 * and resample unequal point counts by arc length. Missing IDs fail closed;
 * array order must never be treated as lanelet identity. */
export function illustrativeMapRoads(baseMap) {
  if (!baseMap?.valid || !Array.isArray(baseMap.polylines)) return [];
  const explicit = [], leftById = new Map(), rightById = new Map();
  let inputBudget = MAP_MAX_POINTS;
  for (const line of baseMap.polylines.slice(0, MAP_MAX_LINES)) {
    if (inputBudget < 2) break;
    if (!MAP_ROAD_NAMESPACES.has(line?.namespace) || !Array.isArray(line.points)) continue;
    const selected = line.points.slice(0, inputBudget);
    inputBudget -= selected.length;
    if (selected.length < 2 || selected.some((point) => !finiteMapPoint(point))) continue;
    const points = selected.map((point) => [point[0], point[1]]);
    if (line.namespace === MAP_CENTERLINE) { explicit.push(points); continue; }
    if (!Number.isSafeInteger(line.marker_id) || line.marker_id < 0) continue;
    if (line.namespace === 'lanelet/left_bound') leftById.set(line.marker_id, points);
    else rightById.set(line.marker_id, points);
  }
  if (explicit.length) return explicit;
  const derived = [];
  let remaining = MAP_MAX_POINTS;
  [...leftById.entries()].sort(([a], [b]) => a - b).forEach(([id, left]) => {
    if (remaining < 2 || !rightById.has(id + 1)) return;
    const center = pairedCenterline(left, rightById.get(id + 1), remaining);
    if (center) { derived.push(center); remaining -= center.length; }
  });
  return derived;
}

/** HH_261002 - Place only illustrative vegetation beside received map roads.
 * Each centerline is sampled by physical arc length independently, so a short
 * two-point lane works and disconnected lane markers are never bridged. Every
 * candidate is checked against all road center/boundary lines and the optional
 * active route. The resulting plants are artwork, never sensor/map geometry. */
export function illustrativeMapScenerySites(baseMap, routePoints = []) {
  const shrubs = [], trees = [];
  if (!baseMap?.valid || !Array.isArray(baseMap.polylines)) return { shrubs, trees };
  const roads = [];
  let budget = MAP_MAX_POINTS;
  for (const line of baseMap.polylines.slice(0, MAP_MAX_LINES)) {
    if (budget < 2) break;
    if (!MAP_ROAD_NAMESPACES.has(line?.namespace) || !Array.isArray(line.points)) continue;
    const selected = line.points.slice(0, budget);
    budget -= selected.length;
    separatedRuns(selected).forEach((run) => roads.push(run));
  }
  const centerlines = illustrativeMapRoads(baseMap);
  centerlines.forEach((line) => roads.push(line));
  const activeRoute = Array.isArray(routePoints) && routePoints.length >= 2 ? routePoints : [];
  const clearOfRoads = (site, clearance) => roads.every((line) =>
    distanceToRouteMeters(site, line) >= clearance)
    && distanceToRouteMeters(site, activeRoute) >= clearance;
  centerlines.forEach((line, lineIndex) => {
    if (shrubs.length >= MAP_MAX_SHRUBS && trees.length >= MAP_MAX_TREES) return;
    const arc = arcSegments(line);
    if (arc.total < 0.5) return;
    const sampleCount = Math.min(128, Math.max(1, Math.ceil(arc.total / 7)));
    for (let sampleIndex = 0; sampleIndex < sampleCount; sampleIndex += 1) {
      if (shrubs.length >= MAP_MAX_SHRUBS && trees.length >= MAP_MAX_TREES) break;
      const center = pointAtArc(arc, (sampleIndex + 0.5) * arc.total / sampleCount);
      for (const side of [-1, 1]) {
        const seed = lineIndex * 17 + sampleIndex * 7 + (side + 1) * 3;
        const shrubOffset = 7.2 + (seed % 4) * 0.4;
        const shrub = { x: center.x + side * center.normalX * shrubOffset,
          y: center.y + side * center.normalY * shrubOffset,
          scale: 0.72 + (seed % 4) * 0.1 };
        if (shrubs.length < MAP_MAX_SHRUBS && clearOfRoads(shrub, SHRUB_ROUTE_CLEARANCE_M)
          && separatedFromSites(shrub, shrubs, 3.4)
          && separatedFromSites(shrub, trees, 4.0)) shrubs.push(shrub);
        if (sampleIndex % 3 !== 0 || trees.length >= MAP_MAX_TREES) continue;
        const treeOffset = 11.2 + (seed % 4) * 0.6;
        const tree = { x: center.x + side * center.normalX * treeOffset,
          y: center.y + side * center.normalY * treeOffset,
          scale: 0.86 + (seed % 3) * 0.1 };
        if (clearOfRoads(tree, TREE_ROUTE_CLEARANCE_M)
          && separatedFromSites(tree, trees, 8.0)
          && separatedFromSites(tree, shrubs, 4.0)) trees.push(tree);
      }
    }
  });
  return { shrubs, trees };
}
