import * as THREE from 'three';
import { RANGER_WHEEL_RADIUS_M } from './navigationMath';
import { createCleanTire, createCleanTireParts, replaceWheelMeshesWithCleanTires,
  TIRE_WIDTH_M } from './navigationWheelVisuals';

const fakeModel = (names) => {
  const model = new THREE.Group();
  names.forEach((name) => {
    const steer = new THREE.Group(); steer.name = `wheel_${name}_steer`;
    const spin = new THREE.Group(); spin.name = `wheel_${name}_spin`;
    const wheel = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.3, 0.1));
    wheel.name = `wheel_${name}`;
    spin.add(wheel); steer.add(spin); model.add(steer);
  });
  return model;
};

test('clean tire is centred on the spin axis with the rolling radius and width', () => {
  // HJ_261105 - A symmetric tire cannot wobble around the existing spin pivot.
  const tire = createCleanTire(createCleanTireParts());
  const box = new THREE.Box3().setFromObject(tire);
  expect(box.max.x).toBeCloseTo(RANGER_WHEEL_RADIUS_M, 3);
  expect(box.min.x).toBeCloseTo(-RANGER_WHEEL_RADIUS_M, 3);
  expect(box.max.y).toBeCloseTo(RANGER_WHEEL_RADIUS_M, 3);
  expect(box.max.z - box.min.z).toBeCloseTo(TIRE_WIDTH_M, 3);
  expect(box.max.z + box.min.z).toBeCloseTo(0, 6);
});

test('original wheel meshes are hidden and tires attach under the spin nodes', () => {
  const model = fakeModel(['fl', 'fr', 'rl', 'rr']);
  expect(replaceWheelMeshesWithCleanTires(model, ['fl', 'fr', 'rl', 'rr'])).toBe(4);
  ['fl', 'fr', 'rl', 'rr'].forEach((name) => {
    expect(model.getObjectByName(`wheel_${name}`).visible).toBe(false);
    const spin = model.getObjectByName(`wheel_${name}_spin`);
    expect(spin.children.some(child => child.name === 'clean_tire')).toBe(true);
  });
});

test('a wheel without its spin node keeps the original geometry', () => {
  const model = fakeModel(['fl']);
  expect(replaceWheelMeshesWithCleanTires(model, ['fl', 'rr'])).toBe(1);
  expect(replaceWheelMeshesWithCleanTires(null, ['fl'])).toBe(0);
});
