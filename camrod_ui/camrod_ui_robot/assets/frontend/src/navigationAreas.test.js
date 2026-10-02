import { areaIsDestination, areaLabelPoint, layoutAreaLabels, normalizeNavigationAreas } from './navigationAreas';

const campsite = (number, points = [[0, 0], [2, 0], [2, 2], [0, 2]]) => ({
  id: `camping_site_${number}`, label: `B${number}`, kind: 'camping_site',
  site: `B${number}`, source: 'camping_sites_yaml', points,
});
const dropZone = (points = [[5, 0], [7, 0], [7, 2], [5, 2]]) => ({
  id: 'dz_area_7144', label: '드롭존', kind: 'drop_zone',
  site: null, source: 'drop_zones_yaml', points,
});

test('actual authored open rings retain metre corners and polygon centroids', () => {
  // HH_261002 - Labels come from real area corners, never inferred boxes.
  const [site, drop] = normalizeNavigationAreas([campsite(9), dropZone()]);
  expect(site.points).toEqual([[0, 0], [2, 0], [2, 2], [0, 2]]);
  expect(site.centroid).toEqual({ x: 1, y: 1 });
  expect(areaLabelPoint(site)).toEqual([1, 1]);
  expect(drop.centroid).toEqual({ x: 6, y: 1 });
  const [closed] = normalizeNavigationAreas([campsite(9,
    [[0, 0], [2, 0], [2, 2], [0, 2], [0, 0]])]);
  expect(closed.points).toHaveLength(4);
});

test('malformed, crossed, spoofed and duplicate catalog polygons fail closed', () => {
  const crossed = campsite(1, [[0, 0], [2, 2], [0, 2], [2, 0]]);
  const duplicate = campsite(1, [[0, 0], [2, 0], [2, 0], [0, 2]]);
  const invalid = [crossed, duplicate,
    campsite(14), { ...campsite(1), source: 'illustration' },
    { ...dropZone(), source: 'camping_sites_yaml' },
    campsite(1, [[0, 0], [Infinity, 0], [1, 1]]),
    campsite(1, [[0, 0], [1, 1]]),
  ];
  expect(normalizeNavigationAreas(invalid)).toEqual([]);
  expect(normalizeNavigationAreas([campsite(1), campsite(1)])).toHaveLength(1);
  expect(normalizeNavigationAreas(null)).toEqual([]);
});

test('current site, return drop zone and inactive mission have distinct highlights', () => {
  const [site, drop] = normalizeNavigationAreas([campsite(9), dropZone()]);
  expect(areaIsDestination(site, { active: true, site: 'B9', state: 'MOVING_TO_SITE' })).toBe(true);
  expect(areaIsDestination(site, { active: true, site: 'camping_site_9', state: 'RECALL_TO_SITE_ROAD' })).toBe(true);
  expect(areaIsDestination(drop, { active: true, site: 'B9', state: 'MOVING_TO_SITE' })).toBe(false);
  expect(areaIsDestination(drop, { active: true, site: 'B9', state: 'RETURN_WITH_CARGO' })).toBe(true);
  expect(areaIsDestination(site, { active: true, site: 'B9', state: 'DROP_ZONE_PARKING' })).toBe(false);
  expect(areaIsDestination(drop, { active: false, site: 'B9', state: 'DROP_ZONE_WAIT' })).toBe(false);
});

test('crowded site names split into bounded, non-overlapping rails with unchanged anchors', () => {
  // HH_261002 - Labels move for readability; leader anchors never move away
  // from the received area centroids and no fake area position is introduced.
  const areas = Array.from({ length: 14 }, (_, index) => ({
    id: `site_${index}`, anchor: { x: 84 + index % 2 * 3, y: 40 + index * 2 },
  }));
  const layout = layoutAreaLabels(areas);
  expect(layout).toHaveLength(14);
  expect(layout.map(item => item.id).sort()).toEqual(areas.map(item => item.id).sort());
  for (const [index, area] of areas.entries()) {
    const label = layout.find(item => item.id === area.id);
    expect(label.anchorX).toBe(area.anchor.x);
    expect(label.anchorY).toBe(area.anchor.y);
    expect(label.x).toBeGreaterThanOrEqual(15);
    expect(label.x).toBeLessThanOrEqual(274);
    expect(label.y).toBeGreaterThanOrEqual(13);
    expect(label.y).toBeLessThanOrEqual(162);
    expect(label.side).toBe(index % 2 ? 'right' : 'left');
  }
  for (const side of ['left', 'right']) {
    const ordered = layout.filter(item => item.side === side).sort((a, b) => a.y - b.y);
    for (let index = 1; index < ordered.length; index += 1) {
      expect(ordered[index].y - ordered[index - 1].y).toBeGreaterThanOrEqual(9);
    }
  }
  expect(layoutAreaLabels([])).toEqual([]);
});
