// HH_261002 - Display authored service polygons at their map coordinates. These
// transparent fills/borders are not obstacles, routes, or inferred ground truth.
import * as THREE from 'three';

export function areaFillPositions(points, origin, height = 0.012) {
  if (!Array.isArray(points) || points.length < 3) return [];
  const contour = points.map(p => new THREE.Vector2(p[0] - origin.x, -(p[1] - origin.y)));
  return THREE.ShapeUtils.triangulateShape(contour, []).flatMap(face => face.flatMap(index =>
    [contour[index].x, height, contour[index].y]));
}

export function areaOutlinePositions(points, origin, width = 0.09, height = 0.02) {
  const vertices = [];
  if (!Array.isArray(points) || points.length < 3) return vertices;
  for (let index = 0; index < points.length; index += 1) {
    const a = points[index], b = points[(index + 1) % points.length];
    const ax = a[0] - origin.x, az = -(a[1] - origin.y);
    const bx = b[0] - origin.x, bz = -(b[1] - origin.y);
    const length = Math.hypot(bx - ax, bz - az);
    if (length < 1e-8) continue;
    const nx = -(bz - az) / length * width / 2, nz = (bx - ax) / length * width / 2;
    vertices.push(ax + nx, height, az + nz, ax - nx, height, az - nz, bx + nx, height, bz + nz,
      ax - nx, height, az - nz, bx - nx, height, bz - nz, bx + nx, height, bz + nz);
  }
  return vertices;
}

// HH_261002 - Batch matching colors into at most six draw calls, regardless of
// site count. Destination highlighting changes only this cached display group.
export function createNavigationAreaVisuals(areas, origin, isDestination, dark = false) {
  const group = new THREE.Group();
  group.name = 'authored_service_areas';
  const batches = new Map();
  areas.forEach(area => {
    const selected = isDestination(area);
    const key = selected ? 'destination' : area.kind;
    if (!batches.has(key)) batches.set(key, { fills: [], outlines: [], selected });
    const batch = batches.get(key);
    batch.fills.push(...areaFillPositions(area.points, origin));
    batch.outlines.push(...areaOutlinePositions(area.points, origin, selected ? 0.14 : 0.09));
  });
  batches.forEach((batch, key) => {
    const color = key === 'destination' ? '#159bd6' : key === 'drop_zone'
      ? (dark ? '#e8b65d' : '#b27a27') : (dark ? '#74c8a9' : '#449778');
    [[batch.fills, batch.selected ? 0.32 : 0.20], [batch.outlines, 0.9]].forEach(([positions, opacity]) => {
      if (!positions.length) return;
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
      const material = new THREE.MeshBasicMaterial({ color, transparent: true, opacity,
        side: THREE.DoubleSide, depthWrite: false, toneMapped: false });
      group.add(new THREE.Mesh(geometry, material));
    });
  });
  group.userData.areaCount = areas.length;
  return group;
}

// HH_261002 - Decorative vegetation must not obscure authored service areas.
// This clearance affects artwork only, never obstacle or lanelet cost layers.
export function sceneryClearOfAreas(site, areas, margin = 1.2) {
  return areas.every(({ points }) => {
    let inside = false;
    for (let i = 0, j = points.length - 1; i < points.length; j = i, i += 1) {
      const a = points[j], b = points[i];
      if ((a[1] > site.y) !== (b[1] > site.y)
        && site.x < (b[0] - a[0]) * (site.y - a[1]) / (b[1] - a[1]) + a[0]) inside = !inside;
      const dx = b[0] - a[0], dy = b[1] - a[1], norm = dx * dx + dy * dy;
      const t = norm ? Math.max(0, Math.min(1, ((site.x - a[0]) * dx + (site.y - a[1]) * dy) / norm)) : 0;
      if (Math.hypot(site.x - a[0] - t * dx, site.y - a[1] - t * dy) < margin) return false;
    }
    return !inside;
  });
}
