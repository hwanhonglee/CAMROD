import * as THREE from 'three';
import { classifyNavigationObject, createNavigationObjectVisual, mapQuaternionToThree,
  navigationObjectDimensions, navigationObjectSymbol, updateNavigationObjectVisual } from './navigationObjectVisuals';

// HH_261002 - Keep detector metric extents and semantic display drawings independently testable.
const detection = (className, size = { x: 0.6, y: 0.5, z: 1.7 }, orientation) => ({
  class_name: className,
  bbox: { size, orientation, source: 'observed_lidar_extent' },
});

test.each(['person', 'car', 'truck', 'bus', 'bicycle', 'motorcycle', 'tent'])(
  '%s retains the exact received box and every semantic mesh stays inside it', (kind) => {
    const object = createNavigationObjectVisual(detection(kind));
    const extent = new THREE.Box3().setFromObject(object).getSize(new THREE.Vector3());
    expect(extent.x).toBeCloseTo(0.6, 6);
    expect(extent.y).toBeCloseTo(1.7, 6);
    expect(extent.z).toBeCloseTo(0.5, 6);
    const shape = new THREE.Box3().setFromObject(object.getObjectByName('illustrative_class_silhouette'));
    expect(shape.min.x).toBeGreaterThanOrEqual(-0.300001);
    expect(shape.max.x).toBeLessThanOrEqual(0.300001);
    expect(shape.min.y).toBeGreaterThanOrEqual(-0.850001);
    expect(shape.max.y).toBeLessThanOrEqual(0.850001);
    expect(shape.min.z).toBeGreaterThanOrEqual(-0.250001);
    expect(shape.max.z).toBeLessThanOrEqual(0.250001);
    expect(object.userData.sizeMode).toBe('observed');
    expect(object.userData.semanticShape).toBe('class_illustration');
  },
);

test('map yaw rotates the length axis toward Three negative Z and preserves box centre', () => {
  const orientation = { x: 0, y: 0, z: Math.SQRT1_2, w: Math.SQRT1_2 };
  const quaternion = mapQuaternionToThree(orientation);
  const forward = new THREE.Vector3(1, 0, 0).applyQuaternion(quaternion);
  expect(forward.x).toBeCloseTo(0);
  expect(forward.y).toBeCloseTo(0);
  expect(forward.z).toBeCloseTo(-1);
  const box = new THREE.Box3().setFromObject(createNavigationObjectVisual(
    detection('car', { x: 4, y: 2, z: 1.5 }, orientation)));
  expect(box.getCenter(new THREE.Vector3()).length()).toBeCloseTo(0);
  const size = box.getSize(new THREE.Vector3());
  expect(size.x).toBeCloseTo(2);
  expect(size.y).toBeCloseTo(1.5);
  expect(size.z).toBeCloseTo(4);
});

test('the full quaternion basis agrees with converting a rotated ROS vector', () => {
  const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(0.4, -0.2, 0.7));
  const ros = new THREE.Vector3(1, 2, 3).applyQuaternion(q);
  const three = new THREE.Vector3(1, 3, -2).applyQuaternion(mapQuaternionToThree(q));
  expect(three.distanceTo(new THREE.Vector3(ros.x, ros.z, -ros.y))).toBeLessThan(1e-9);
});

test.each([undefined, null, { x: 1, y: 1, z: 0 }, { x: -1, y: 1, z: 1 },
  { x: '1', y: 1, z: 1 }, { x: NaN, y: 1, z: 1 }, { x: 1, y: Infinity, z: 1 }])(
  'invalid/missing dimensions %p never create a measured box or scaled class body', (size) => {
    const input = { class_name: 'person', bbox: { size } };
    expect(navigationObjectDimensions(input)).toBeNull();
    const object = createNavigationObjectVisual(input);
    expect(object.userData.sizeMode).toBe('symbolic');
    expect(object.getObjectByName('received_bounding_box')).toBeUndefined();
    expect(object.getObjectByName('person_head')).toBeUndefined();
    expect(object.getObjectByName('unmeasured_object_locator')).toBeDefined();
  },
);

test.each(['placeholder', 'fixed', 'illustrative'])('legacy %s sizes are not measurements', (source) => {
  const object = detection('person');
  object.bbox.source = source;
  expect(navigationObjectDimensions(object)).toBeNull();
});

test('class aliases are bounded and an unknown label does not invent a person', () => {
  expect(classifyNavigationObject(' Pedestrian ')).toBe('person');
  expect(classifyNavigationObject('자동차')).toBe('car');
  expect(classifyNavigationObject('parking meter')).toBe('unknown');
  expect(classifyNavigationObject(null)).toBe('unknown');
  expect(navigationObjectSymbol('person')).toBe('🚶');
});

test('an explicit observed source is required even for finite complete metric dimensions', () => {
  expect(navigationObjectDimensions({ dimensions: { x: 1, y: 1, z: 1 } })).toBeNull();
  expect(navigationObjectDimensions({ dimensions: { x: 1, y: 1, z: 1 },
    geometry_source: 'synthetic_fixture' })).toBeNull();
});

test('direct API schema updates dimensions and orientation without allocating new meshes', () => {
  const group = createNavigationObjectVisual(detection('person'));
  const head = group.getObjectByName('person_head');
  expect(updateNavigationObjectVisual(group, { class_name: 'person',
    dimensions: { x: 0.4, y: 0.5, z: 1.4 }, geometry_source: 'observed_lidar_extent',
    orientation: { x: 0, y: 0, z: Math.SQRT1_2, w: Math.SQRT1_2 } })).toBe(true);
  expect(group.getObjectByName('person_head')).toBe(head);
  expect(group.getObjectByName('received_metric_dimensions').scale.toArray()).toEqual([0.4, 1.4, 0.5]);
  expect(group.userData.labelHeight).toBeCloseTo(0.82);
  expect(updateNavigationObjectVisual(group, detection('car'))).toBe(false);
  expect(updateNavigationObjectVisual(group, { class_name: 'person' })).toBe(false);
});

test('zero, nonfinite and absent orientations become finite identity', () => {
  [undefined, { x: 0, y: 0, z: 0, w: 0 }, { x: 0, y: NaN, z: 0, w: 1 }].forEach((q) => {
    expect(mapQuaternionToThree(q).toArray()).toEqual([0, 0, 0, 1]);
  });
});

test('all owned resources are reachable by scene disposal and independent of other detections', () => {
  const first = createNavigationObjectVisual(detection('car'));
  const second = createNavigationObjectVisual(detection('car'));
  const materials = new Set(), geometries = new Set();
  first.traverse((child) => {
    if (child.material) materials.add(child.material);
    if (child.geometry) geometries.add(child.geometry);
  });
  second.traverse((child) => {
    if (child.material) expect(materials.has(child.material)).toBe(false);
    if (child.geometry) expect(geometries.has(child.geometry)).toBe(false);
  });
  geometries.forEach((geometry) => expect(() => geometry.dispose()).not.toThrow());
  materials.forEach((material) => expect(() => material.dispose()).not.toThrow());
});
