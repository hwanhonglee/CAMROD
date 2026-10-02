// HH_261002 - Camping sites and drop zones are displayed only from configured,
// map-frame polygon corners received from the backend. Never synthesize extents.
const MAX_AREAS = 64;
const MAX_VERTICES = 128;
const finite = (value) => typeof value === 'number' && Number.isFinite(value);
const validPoint = (point) => Array.isArray(point) && point.length >= 2
  && finite(point[0]) && finite(point[1])
  && Math.abs(point[0]) <= 1e8 && Math.abs(point[1]) <= 1e8;
const samePoint = (a, b) => a[0] === b[0] && a[1] === b[1];
const orient = (a, b, c) => (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
const between = (a, b, c) => Math.min(a, b) <= c && c <= Math.max(a, b);

function segmentsIntersect(a, b, c, d) {
  const abC = orient(a, b, c), abD = orient(a, b, d);
  const cdA = orient(c, d, a), cdB = orient(c, d, b);
  if (abC * abD < 0 && cdA * cdB < 0) return true;
  return (abC === 0 && between(a[0], b[0], c[0]) && between(a[1], b[1], c[1]))
    || (abD === 0 && between(a[0], b[0], d[0]) && between(a[1], b[1], d[1]))
    || (cdA === 0 && between(c[0], d[0], a[0]) && between(c[1], d[1], a[1]))
    || (cdB === 0 && between(c[0], d[0], b[0]) && between(c[1], d[1], b[1]));
}

function polygonCentroid(points) {
  let twiceArea = 0, weightedX = 0, weightedY = 0;
  for (let index = 0; index < points.length; index += 1) {
    const a = points[index], b = points[(index + 1) % points.length];
    const cross = a[0] * b[1] - b[0] * a[1];
    twiceArea += cross;
    weightedX += (a[0] + b[0]) * cross;
    weightedY += (a[1] + b[1]) * cross;
  }
  if (!finite(twiceArea) || Math.abs(twiceArea) < 1e-6) return null;
  const x = weightedX / (3 * twiceArea), y = weightedY / (3 * twiceArea);
  return finite(x) && finite(y) ? { x, y } : null;
}

function normalizeArea(raw) {
  if (!raw || typeof raw.id !== 'string' || !/^[A-Za-z0-9_-]{1,64}$/.test(raw.id)
    || typeof raw.label !== 'string' || !raw.label.trim() || raw.label.length > 32
    || !Array.isArray(raw.points) || raw.points.length < 3 || raw.points.length > MAX_VERTICES + 1
    || !raw.points.every(validPoint)) return null;
  const points = raw.points.map((point) => [point[0], point[1]]);
  if (points.length > 3 && samePoint(points[0], points[points.length - 1])) points.pop();
  if (points.length < 3 || points.length > MAX_VERTICES) return null;
  for (let index = 0; index < points.length; index += 1) {
    if (samePoint(points[index], points[(index + 1) % points.length])) return null;
  }
  const camping = raw.kind === 'camping_site' && raw.source === 'camping_sites_yaml'
    && /^B(?:[1-9]|1[0-3])$/.test(raw.site)
    && raw.id === `camping_site_${raw.site.slice(1)}`;
  const drop = raw.kind === 'drop_zone' && raw.source === 'drop_zones_yaml'
    && raw.site == null;
  if (!camping && !drop) return null;
  // HH_261002 - Reject crossed or degenerate rings instead of drawing a
  // plausible-looking but false site footprint.
  for (let first = 0; first < points.length; first += 1) {
    for (let second = first + 2; second < points.length; second += 1) {
      if (first === 0 && second === points.length - 1) continue;
      if (segmentsIntersect(points[first], points[(first + 1) % points.length],
        points[second], points[(second + 1) % points.length])) return null;
    }
  }
  const centroid = polygonCentroid(points);
  return centroid ? { id: raw.id, label: raw.label.trim(), kind: raw.kind,
    site: camping ? raw.site : null, source: raw.source, points, centroid } : null;
}

export function normalizeNavigationAreas(rawAreas) {
  if (!Array.isArray(rawAreas)) return [];
  const seen = new Set();
  const areas = [];
  for (const raw of rawAreas.slice(0, MAX_AREAS)) {
    const area = normalizeArea(raw);
    if (area && !seen.has(area.id)) { areas.push(area); seen.add(area.id); }
  }
  return areas;
}

export function areaLabelPoint(area) {
  const point = area?.centroid;
  return finite(point?.x) && finite(point?.y) ? [point.x, point.y] : null;
}

export function areaIsDestination(area, mission) {
  if (!area || mission?.active !== true) return false;
  const state = mission.state || mission.service_state_name;
  const returning = ['RETURN_WITH_CARGO', 'RETURNING_TO_DROP_ZONE', 'DROP_ZONE_PARKING'].includes(state);
  if (returning) return area.kind === 'drop_zone';
  const site = typeof mission.site === 'string' && /^camping_site_([1-9]|1[0-3])$/.test(mission.site)
    ? `B${mission.site.split('_').pop()}` : mission.site;
  return area.kind === 'camping_site' && area.site === site;
}

// HH_261002 - Keep every actual polygon fixed. Offset only its name into two
// collision-free SVG columns and draw a leader back to the measured centroid.
export function layoutAreaLabels(areas, width = 292, height = 176) {
  if (!Array.isArray(areas) || !areas.length) return [];
  const valid = areas.filter(area => area && typeof area.id === 'string'
    && finite(area.anchor?.x) && finite(area.anchor?.y));
  if (!valid.length) return [];
  const sorted = [...valid].sort((a, b) => a.anchor.y - b.anchor.y
    || a.anchor.x - b.anchor.x || a.id.localeCompare(b.id));
  const minX = Math.min(...sorted.map(area => area.anchor.x));
  const maxX = Math.max(...sorted.map(area => area.anchor.x));
  const leftX = Math.max(15, Math.min(width / 2 - 32, minX - 20));
  const rightX = Math.min(width - 18, Math.max(leftX + 56, maxX + 20));
  const groups = [[], []];
  sorted.forEach((area, index) => groups[index % 2].push(area));
  const minY = 13, maxY = height - 14;
  const gap = Math.min(11, (maxY - minY) / Math.max(1, Math.max(...groups.map(group => group.length)) - 1));
  return groups.flatMap((group, side) => {
    if (!group.length) return [];
    const positions = group.map(area => Math.max(minY, Math.min(maxY, area.anchor.y)));
    for (let index = 1; index < positions.length; index += 1) {
      positions[index] = Math.max(positions[index], positions[index - 1] + gap);
    }
    if (positions[positions.length - 1] > maxY) {
      const shift = positions[positions.length - 1] - maxY;
      for (let index = 0; index < positions.length; index += 1) positions[index] -= shift;
    }
    for (let index = positions.length - 2; index >= 0; index -= 1) {
      positions[index] = Math.min(positions[index], positions[index + 1] - gap);
    }
    if (positions[0] < minY) {
      const shift = minY - positions[0];
      for (let index = 0; index < positions.length; index += 1) positions[index] += shift;
    }
    return group.map((area, index) => ({ id: area.id, side: side ? 'right' : 'left',
      x: side ? rightX : leftX, y: positions[index],
      anchorX: area.anchor.x, anchorY: area.anchor.y }));
  });
}
