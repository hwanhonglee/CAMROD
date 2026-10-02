// HH_261002 - Round display-only road/lane strokes without moving received map
// centerlines. Segment-normal faces keep metre widths exact; bounded outer arcs
// fill corner gaps without spline overshoot or unbounded miter spikes. Faces may
// overlap inside a turn and are intended for the scene's opaque road/paint meshes.
const ROUND_ARC_STEP = Math.PI / 12;
const ROUND_ARC_MAX_STEPS = 12;

/**
 * Return non-indexed XYZ triangles (upward-facing) for separate [mapX, mapY] paths.
 * Invalid/unrenderable points and over-limit segments split the stroke; duplicate
 * points are ignored. Input arrays are never changed, interpolated, or resampled.
 *
 * HH_261002 - Work and geometry are linear in input size: two triangles per
 * accepted segment, at most twelve per join and twelve per end cap, hence at
 * most 26 triangles per accepted segment. Long sparse map edges cost no more
 * than short edges. Closed chains get a join, not overlapping endpoint caps.
 */
export function illustratedRoadPositions(paths, origin, width, height, maxSegmentLength = Infinity) {
  const positions = [];
  const radius = Number.isFinite(width) ? width / 2 : 0;
  if (!Array.isArray(paths) || !Number.isFinite(origin?.x) || !Number.isFinite(origin?.y)
    || !Number.isFinite(width) || !(radius > 0) || !Number.isFinite(Math.fround(radius))
    || !Number.isFinite(height) || !Number.isFinite(Math.fround(height))
    || (maxSegmentLength !== Infinity && !Number.isFinite(maxSegmentLength))
    || !(maxSegmentLength > 0)) return positions;

  const triangle = (a, b, c) => {
    positions.push(a.x, height, a.z, b.x, height, b.z, c.x, height, c.z);
  };
  const offset = (point, normal, side) => ({
    x: point.x + side * normal.x, z: point.z + side * normal.z,
  });
  const arc = (center, start, end, sweep) => {
    const steps = Math.min(ROUND_ARC_MAX_STEPS, Math.max(1, Math.ceil(Math.abs(sweep) / ROUND_ARC_STEP)));
    const startAngle = Math.atan2(start.z, start.x);
    let previous = offset(center, start, 1);
    for (let step = 1; step <= steps; step += 1) {
      const angle = startAngle + sweep * step / steps;
      // Exact end offsets share face edges; trig rounding cannot open join seams.
      const next = step === steps ? offset(center, end, 1)
        : { x: center.x + Math.cos(angle) * radius, z: center.z + Math.sin(angle) * radius };
      if (sweep > 0) triangle(center, next, previous);
      else triangle(center, previous, next);
      previous = next;
    }
  };
  const join = (point, incoming, outgoing) => {
    const turn = Math.atan2(incoming.x * outgoing.z - incoming.z * outgoing.x,
      incoming.x * outgoing.x + incoming.z * outgoing.z);
    if (turn === 0) return;
    const side = turn > 0 ? -1 : 1;
    arc(point, { x: incoming.normal.x * side, z: incoming.normal.z * side },
      { x: outgoing.normal.x * side, z: outgoing.normal.z * side }, turn);
  };
  const cap = (point, direction, start) => {
    const side = start ? 1 : -1;
    const normal = { x: direction.normal.x * side, z: direction.normal.z * side };
    arc(point, normal, { x: -normal.x, z: -normal.z }, Math.PI);
  };
  const renderPoint = (point) => {
    if (!Array.isArray(point) || !Number.isFinite(point[0]) || !Number.isFinite(point[1])) return null;
    const x = point[0] - origin.x, z = -(point[1] - origin.y);
    // HH_261002 - BufferGeometry stores float32 values. A corrupt giant point is
    // a discontinuity, not an Infinity attribute or a bridge across missing data.
    return Number.isFinite(Math.fround(Math.abs(x) + radius))
      && Number.isFinite(Math.fround(Math.abs(z) + radius)) ? { x, z } : null;
  };

  for (const points of paths) {
    if (!Array.isArray(points)) continue;
    let previous = null, first = null, incoming = null, firstDirection = null;
    const finish = () => {
      if (incoming) {
        if (previous.x === first.x && previous.z === first.z) join(first, incoming, firstDirection);
        else { cap(first, firstDirection, true); cap(previous, incoming, false); }
      }
      previous = null; first = null; incoming = null; firstDirection = null;
    };
    for (const rawPoint of points) {
      const point = renderPoint(rawPoint);
      if (!point) { finish(); continue; }
      if (!previous) { previous = point; first = point; continue; }
      const dx = point.x - previous.x, dz = point.z - previous.z;
      const length = Math.hypot(dx, dz);
      if (length === 0) continue;
      if (!Number.isFinite(length) || length > maxSegmentLength) {
        finish(); previous = point; first = point; continue;
      }
      const direction = { x: dx / length, z: dz / length,
        normal: { x: -dz / length * radius, z: dx / length * radius } };
      if (incoming) join(previous, incoming, direction);
      else firstDirection = direction;
      const leftA = offset(previous, direction.normal, 1), leftB = offset(point, direction.normal, 1);
      const rightA = offset(previous, direction.normal, -1), rightB = offset(point, direction.normal, -1);
      triangle(leftA, leftB, rightA);
      triangle(rightA, leftB, rightB);
      previous = point; incoming = direction;
    }
    finish();
  }
  return positions;
}
