import * as THREE from 'three';
import { createNavigationVegetation } from './navigationSceneryVisuals';

// HH_261002 - Inspect the actual transformed vertices, not just the scale
// settings, so smoothing never silently expands the existing clearance radii.
function instanceVertices(mesh, index) {
  const matrix = new THREE.Matrix4();
  mesh.getMatrixAt(index, matrix);
  const positions = mesh.geometry.getAttribute('position');
  return Array.from({ length: positions.count }, (_, vertex) =>
    new THREE.Vector3().fromBufferAttribute(positions, vertex).applyMatrix4(matrix));
}

const origin = { x: 100, y: 200 };
const siteList = [
  { x: 103, y: 207, scale: 0.72 },
  { x: 91, y: 208, scale: 1 },
  { x: 105, y: 184, scale: 1.21 },
];

test.each([false, true])('rounded plants preserve clearance footprints and map axes (dark=%s)', (dark) => {
  const group = createNavigationVegetation({ shrubs: siteList, trees: siteList }, origin, dark);
  group.children.forEach((mesh) => {
    const shrub = mesh.name === 'rounded_shrubs';
    const instancesPerSite = mesh.name === 'rounded_tree_trunks' ? 1 : 3;
    siteList.forEach((site, siteIndex) => {
      const vertices = Array.from({ length: instancesPerSite }, (_, part) =>
        instanceVertices(mesh, siteIndex * instancesPerSite + part)).flat();
      vertices.forEach((vertex) => {
        expect(Math.hypot(vertex.x - (site.x - origin.x), vertex.z + (site.y - origin.y)))
          .toBeLessThanOrEqual((shrub ? 0.65 : 0.95) * site.scale + 1e-5);
        expect(vertex.y).toBeGreaterThanOrEqual(-1e-6);
        expect(vertex.y).toBeLessThanOrEqual(3.05 * site.scale + 1e-6);
      });
      if (mesh.name === 'layered_tree_canopies') {
        expect(Math.max(...vertices.map((vertex) => vertex.y))).toBeCloseTo(3.05 * site.scale, 5);
        const matrix = new THREE.Matrix4();
        mesh.getMatrixAt(siteIndex * instancesPerSite, matrix);
        expect(matrix.elements[12]).toBeCloseTo(site.x - origin.x, 5);
        expect(matrix.elements[14]).toBeCloseTo(-(site.y - origin.y), 5);
      }
    });
  });
});

test('draw calls stay at three for a full map and the meshes use smooth, modest geometry', () => {
  const shrubs = Array.from({ length: 120 }, (_, index) => ({ x: index * 2, y: 3, scale: 1 }));
  const trees = shrubs.slice(0, 48);
  const group = createNavigationVegetation({ shrubs, trees }, origin);
  expect(group.children).toHaveLength(3);
  expect(group.userData).toEqual({ illustrative: true, shrubs: 120, trees: 48 });
  expect(group.getObjectByName('rounded_shrubs').count).toBe(360);
  expect(group.getObjectByName('layered_tree_canopies').count).toBe(144);
  group.children.forEach((mesh) => {
    expect(mesh.isInstancedMesh).toBe(true);
    expect(mesh.material.flatShading).toBe(false);
    expect(mesh.geometry.index).not.toBeNull();
    expect(mesh.geometry.getAttribute('position').count).toBeLessThan(250);
    expect(mesh.castShadow).toBe(false);
    expect(mesh.receiveShadow).toBe(false);
    expect(mesh.material.map).toBeNull();
    Object.values(mesh.geometry.attributes).forEach((attribute) => {
      expect(Array.from(attribute.array).every(Number.isFinite)).toBe(true);
    });
    expect(Array.from(mesh.instanceMatrix.array).every(Number.isFinite)).toBe(true);
    expect(Array.from(mesh.instanceColor.array).every(Number.isFinite)).toBe(true);
    expect(Number.isFinite(mesh.boundingSphere.radius)).toBe(true);
    const normals = mesh.geometry.getAttribute('normal');
    for (let index = 0; index < normals.count; index += 1) {
      expect(new THREE.Vector3().fromBufferAttribute(normals, index).length()).toBeCloseTo(1, 5);
    }
  });
});

test('light and dark modes keep the established green palette without changing geometry', () => {
  const sites = { shrubs: siteList, trees: siteList };
  const light = createNavigationVegetation(sites, origin);
  const dark = createNavigationVegetation(sites, origin, true);
  expect(light.getObjectByName('rounded_shrubs').material.color.getHex()).toBe(0x688e6e);
  expect(dark.getObjectByName('rounded_shrubs').material.color.getHex()).toBe(0x527868);
  expect(light.getObjectByName('layered_tree_canopies').material.color.getHex()).toBe(0x49775c);
  expect(dark.getObjectByName('layered_tree_canopies').material.color.getHex()).toBe(0x315c53);
  light.children.forEach((mesh, index) => {
    expect(Array.from(mesh.instanceMatrix.array)).toEqual(Array.from(dark.children[index].instanceMatrix.array));
    expect(Array.from(mesh.instanceColor.array)).toEqual(Array.from(dark.children[index].instanceColor.array));
  });
});

test('construction is deterministic, never mutates inputs, and owns disposable resources', () => {
  const sites = Object.freeze({
    shrubs: Object.freeze(siteList.map((site) => Object.freeze({ ...site }))),
    trees: Object.freeze(siteList.map((site) => Object.freeze({ ...site }))),
  });
  const frozenOrigin = Object.freeze({ ...origin }), before = JSON.stringify({ sites, frozenOrigin });
  const first = createNavigationVegetation(sites, frozenOrigin);
  const second = createNavigationVegetation(sites, frozenOrigin);
  expect(JSON.stringify({ sites, frozenOrigin })).toBe(before);
  first.children.forEach((mesh, index) => {
    expect(Array.from(mesh.instanceMatrix.array)).toEqual(Array.from(second.children[index].instanceMatrix.array));
    expect(Array.from(mesh.instanceColor.array)).toEqual(Array.from(second.children[index].instanceColor.array));
    expect(mesh.geometry).not.toBe(second.children[index].geometry);
    expect(mesh.material).not.toBe(second.children[index].material);
    expect(() => { mesh.geometry.dispose(); mesh.material.dispose(); mesh.dispose(); }).not.toThrow();
  });
});

test('empty or invalid inputs allocate no empty meshes and never create nonfinite buffers', () => {
  expect(createNavigationVegetation({ shrubs: [], trees: [] }, origin).children).toHaveLength(0);
  expect(createNavigationVegetation(undefined, origin).children).toHaveLength(0);
  expect(createNavigationVegetation({ shrubs: siteList }, { x: NaN, y: 1 }).children).toHaveLength(0);
  const invalid = [null, {}, { x: 1, y: 2, scale: 0 }, { x: Infinity, y: 2, scale: 1 },
    { x: 1, y: 2, scale: -1 }, { x: 1, y: 2, scale: NaN }, { x: 1e40, y: 2, scale: 1 },
    { x: 1, y: 2, scale: 1e40 }];
  const mixed = createNavigationVegetation({ shrubs: invalid, trees: [...invalid, siteList[0]] }, origin);
  expect(mixed.children).toHaveLength(2);
  expect(mixed.userData).toEqual({ illustrative: true, shrubs: 0, trees: 1 });
  mixed.children.forEach((mesh) => {
    expect(Array.from(mesh.instanceMatrix.array).every(Number.isFinite)).toBe(true);
  });
});
