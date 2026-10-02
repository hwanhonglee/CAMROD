// HH_261002 - Validate map-metre fills, independent rings and artwork-only clearance.
import { areaFillPositions, areaOutlinePositions, createNavigationAreaVisuals, sceneryClearOfAreas } from './navigationAreaVisuals';

const square = [[10, 20], [14, 20], [14, 23], [10, 23]];
test('authored quadrilateral keeps exact coordinates and area instead of a fabricated box', () => {
  const vertices = areaFillPositions(square, { x: 10, y: 20 });
  expect(vertices).toHaveLength(18);
  let area = 0;
  for (let i = 0; i < vertices.length; i += 9) {
    area += Math.abs((vertices[i + 3] - vertices[i]) * (vertices[i + 8] - vertices[i + 2])
      - (vertices[i + 5] - vertices[i + 2]) * (vertices[i + 6] - vertices[i])) / 2;
  }
  expect(area).toBeCloseTo(12);
  expect(areaOutlinePositions(square, { x: 10, y: 20 })).toHaveLength(4 * 18);
  expect(areaFillPositions([], { x: 0, y: 0 })).toEqual([]);
});
test('concave area is triangulated without filling the excluded corner', () => {
  const points = [[0, 0], [3, 0], [3, 1], [1, 1], [1, 3], [0, 3]];
  const vertices = areaFillPositions(points, { x: 0, y: 0 });
  let area = 0;
  for (let i = 0; i < vertices.length; i += 9) {
    area += Math.abs((vertices[i + 3] - vertices[i]) * (vertices[i + 8] - vertices[i + 2])
      - (vertices[i + 5] - vertices[i + 2]) * (vertices[i + 6] - vertices[i])) / 2;
  }
  expect(area).toBeCloseTo(5);
});
test('all thirteen sites and drop zone use at most six draw calls including a selected destination', () => {
  const areas = Array.from({ length: 13 }, (_, index) => ({ id: `B${index + 1}`, kind: 'camping_site',
    points: square.map(p => [p[0] + index * 6, p[1]]) }));
  areas.push({ id: 'drop_zone', kind: 'drop_zone', points: square });
  const group = createNavigationAreaVisuals(areas, { x: 10, y: 20 }, a => a.id === 'B9');
  expect(group.userData.areaCount).toBe(14);
  expect(group.children).toHaveLength(6);
  group.children.forEach(mesh => {
    expect(mesh.geometry.attributes.position.count).toBeGreaterThan(0);
    expect(mesh.material.depthWrite).toBe(false);
    mesh.geometry.dispose(); mesh.material.dispose();
  });
});
test('trees/shrubs cannot cover a site interior or its border', () => {
  const areas = [{ points: square }];
  expect(sceneryClearOfAreas({ x: 12, y: 21 }, areas)).toBe(false);
  expect(sceneryClearOfAreas({ x: 14.5, y: 21 }, areas)).toBe(false);
  expect(sceneryClearOfAreas({ x: 16, y: 21 }, areas)).toBe(true);
  expect(sceneryClearOfAreas({ x: 12, y: 21 }, [])).toBe(true);
});
