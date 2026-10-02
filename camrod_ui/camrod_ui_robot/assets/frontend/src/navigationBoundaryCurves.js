// HH_261002 - These curves are display geometry only. Never write them back to
// received lane bounds, routing, collision checking, or sensor/safety data.
export const MAX_BOUNDARY_VISUAL_DEVIATION_M = 0.12;
export const MAX_BOUNDARY_CURVE_POINTS_PER_INPUT_POINT = 9;
const FILLET_STEPS = 8; // Even: the exact midpoint separates the two safe capsules.
const MAX_EDGE_TRIM_FRACTION = 0.45;
const MIN_TURN_SINE = 1e-6;
const REVERSAL_COSINE = Math.cos(165 * Math.PI / 180);

const samePoint = (a, b) => a[0] === b[0] && a[1] === b[1];
const copyPoint = point => [point[0], point[1]];
const validPoint = point => Array.isArray(point)
  && Number.isFinite(point[0]) && Number.isFinite(point[1]);

function cornerSamples(previous, corner, next, maxDeviationM) {
  const incomingX = corner[0] - previous[0], incomingY = corner[1] - previous[1];
  const outgoingX = next[0] - corner[0], outgoingY = next[1] - corner[1];
  const incomingLength = Math.hypot(incomingX, incomingY);
  const outgoingLength = Math.hypot(outgoingX, outgoingY);
  if (!(incomingLength > 0) || !(outgoingLength > 0)
    || !Number.isFinite(incomingLength) || !Number.isFinite(outgoingLength)) return [copyPoint(corner)];
  const ux = incomingX / incomingLength, uy = incomingY / incomingLength;
  const vx = outgoingX / outgoingLength, vy = outgoingY / outgoingLength;
  const sine = Math.abs(ux * vy - uy * vx), cosine = ux * vx + uy * vy;
  // HH_261002 - Almost-straight corners need no curve. Hairpins/reversals retain
  // their real vertex rather than inventing a loop or cutting across a U-turn.
  if (sine < MIN_TURN_SINE || cosine <= REVERSAL_COSINE) return [copyPoint(corner)];

  // HH_261002 - Reserve a conservative floating-point margin. At coordinates
  // too large to resolve the allowed deviation reliably, retain the raw corner.
  const magnitude = Math.max(...previous.map(Math.abs), ...corner.map(Math.abs), ...next.map(Math.abs));
  const availableDeviation = maxDeviationM - 32 * Number.EPSILON * magnitude;
  if (!(availableDeviation > 0)) return [copyPoint(corner)];
  const trim = Math.min(incomingLength * MAX_EDGE_TRIM_FRACTION,
    outgoingLength * MAX_EDGE_TRIM_FRACTION, 4 * availableDeviation / sine);
  if (!(trim > 0)) return [copyPoint(corner)];

  // HH_261002 - Q(t) = B - u*d*(1-t)^2 + v*d*t^2 is a tangent quadratic
  // fillet between A=B-u*d and C=B+v*d. For t<=1/2, its projection lies on
  // segment AB and its distance is d*abs(sin(turn))*t^2 <= maxDeviationM;
  // for t>=1/2 the same bound holds against BC using (1-t)^2. Include t=1/2
  // exactly: every output chord stays in one convex segment capsule, so the
  // bound covers whole rendered edges, not just sampled vertices. Trimming
  // at most 45% of either edge also prevents neighboring fillets overlapping.
  return Array.from({ length: FILLET_STEPS + 1 }, (_, step) => {
    const t = step / FILLET_STEPS, incomingWeight = (1 - t) * (1 - t), outgoingWeight = t * t;
    return [corner[0] + trim * (-ux * incomingWeight + vx * outgoingWeight),
      corner[1] + trim * (-uy * incomingWeight + vy * outgoingWeight)];
  });
}

function smoothRun(points, maxDeviationM) {
  if (points.length < 3 || maxDeviationM === 0) return points.map(copyPoint);
  const closed = points.length > 3 && samePoint(points[0], points[points.length - 1]);
  const count = closed ? points.length - 1 : points.length;
  const result = [];
  const append = point => {
    if (!result.length || !samePoint(result[result.length - 1], point)) result.push(point);
  };
  if (!closed) append(copyPoint(points[0]));
  const start = closed ? 0 : 1, end = closed ? count : count - 1;
  for (let index = start; index < end; index += 1) {
    const previous = points[(index + count - 1) % count], next = points[(index + 1) % count];
    cornerSamples(previous, points[index], next, maxDeviationM).forEach(append);
    // HH_261002 - Fixed work/output per source point. If that budget ever
    // changes, preserve the entire raw run rather than truncating its shape.
    if (result.length > points.length * MAX_BOUNDARY_CURVE_POINTS_PER_INPUT_POINT) return points.map(copyPoint);
  }
  if (closed) append(copyPoint(result[0]));
  else append(copyPoint(points[points.length - 1]));
  return result;
}

/**
 * HH_261002 - Copy one [x,y] polyline into independent, display-only smooth runs.
 * Invalid points, sparse holes, nonfinite edges, and edges over maxSegmentLength
 * split runs; duplicates are removed and singleton runs are retained. Open-run
 * endpoints are exact copies. A valid closed loop has no endpoints: its seam is
 * rounded too, and the returned first/last coordinates match exactly. All points
 * are new arrays; no received input is modified. The optional deviation can be
 * tightened or set to zero, but never raised above the 0.12 m display-only cap.
 * Eight quadratic intervals per corner bound output to nine points per input
 * point. Straight/sparse edges are not subdivided according to their length.
 */
export function smoothDisplayPolyline(points, {
  maxDeviationM = MAX_BOUNDARY_VISUAL_DEVIATION_M, maxSegmentLength = Infinity,
} = {}) {
  if (!Array.isArray(points) || !Number.isFinite(maxDeviationM) || maxDeviationM < 0
    || (maxSegmentLength !== Infinity && !Number.isFinite(maxSegmentLength))
    || !(maxSegmentLength > 0)) return [];
  const deviation = Math.min(maxDeviationM, MAX_BOUNDARY_VISUAL_DEVIATION_M);
  const runs = [];
  let run = [];
  const finish = () => {
    if (run.length) runs.push(smoothRun(run, deviation));
    run = [];
  };
  for (const point of points) {
    if (!validPoint(point)) { finish(); continue; }
    if (run.length) {
      const previous = run[run.length - 1];
      if (samePoint(previous, point)) continue;
      const length = Math.hypot(point[0] - previous[0], point[1] - previous[1]);
      if (!Number.isFinite(length) || length > maxSegmentLength) finish();
    }
    run.push(copyPoint(point));
  }
  finish();
  return runs;
}

/** HH_261002 - Only map-side boundary artwork is curved. Keep every metadata
 * field (including namespace/marker_id), and never join separate map markers.
 * Other namespaces retain the original object and points, with no resampling.
 */
export function displayBoundaryLines(polylines) {
  if (!Array.isArray(polylines)) return [];
  return polylines.flatMap(line => {
    if (!line || typeof line !== 'object') return [];
    if (line.namespace !== 'lanelet/left_bound' && line.namespace !== 'lanelet/right_bound') return [line];
    return smoothDisplayPolyline(line.points).map(points => ({ ...line, points }));
  });
}
