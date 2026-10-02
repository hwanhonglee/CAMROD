import * as THREE from 'three';

const finite = (value) => typeof value === 'number' && Number.isFinite(value);
const COLORS = { person: 0xdc782c, car: 0x2589b4, truck: 0x417cb4, bus: 0x417cb4,
  bicycle: 0x7464ad, motorcycle: 0x7464ad, unknown: 0xb28542 };

// HH_261002 - A class silhouette explains the detector label; it is not a reconstructed
// appearance or a heading estimate. Metric geometry is allowed only with a received box.
export function classifyNavigationObject(className) {
  const name = String(className || '').trim().toLowerCase();
  if (['person', 'pedestrian', '사람', '보행자'].includes(name)) return 'person';
  if (['car', 'vehicle', '자동차', '차량'].includes(name)) return 'car';
  if (['truck', '트럭'].includes(name)) return 'truck';
  if (['bus', '버스'].includes(name)) return 'bus';
  if (['bicycle', 'bike', '자전거'].includes(name)) return 'bicycle';
  if (['motorcycle', 'motorbike', '오토바이'].includes(name)) return 'motorcycle';
  return 'unknown';
}

export function navigationObjectDimensions(object) {
  const box = object?.bbox, size = object?.dimensions || box?.size;
  const source = object?.geometry_source || box?.source;
  // HH_261002 - Do not promote legacy constant detection-marker cubes to measured size.
  if (box?.measured === false || source !== 'observed_lidar_extent'
    || !size || ![size.x, size.y, size.z].every((value) => finite(value) && value > 0)) return null;
  return { x: size.x, y: size.y, z: size.z };
}

export function navigationObjectSymbol(className) {
  // HH_261002 - Unmeasured detections still have a screen-space class icon; its pixels
  // must never be interpreted as a physical object size or a measured silhouette.
  return { person: '🚶', car: '🚗', truck: '🚚', bus: '🚌', bicycle: '🚲',
    motorcycle: '🏍', unknown: '◇' }[classifyNavigationObject(className)];
}

export function mapQuaternionToThree(orientation) {
  // HH_261002 - Conjugate the ROS orientation by the same basis change as positions:
  // map (x,y,z) -> Three (x,z,-y). Invalid/zero quaternions do not invent a rotation.
  const values = [orientation?.x, orientation?.y, orientation?.z, orientation?.w];
  if (!values.every(finite) || Math.hypot(...values) < 1e-9) return new THREE.Quaternion();
  return new THREE.Quaternion(values[0], values[2], -values[1], values[3]).normalize();
}

function part(group, name, geometry, material, position, scale) {
  const mesh = new THREE.Mesh(geometry, material);
  mesh.name = name;
  mesh.position.set(...position);
  if (scale) mesh.scale.set(...scale);
  group.add(mesh);
  return mesh;
}

function cuboid(group, name, material, position, size) {
  return part(group, name, new THREE.BoxGeometry(...size), material, position);
}

function personShape(group, material, accent) {
  // HH_261002 - Every semantic part is contained by the normalized one-metre unit box.
  // Scaling this group uses the received XYZ extents, never a typical human height.
  part(group, 'person_head', new THREE.SphereGeometry(1, 10, 8), material,
    [0, 0.36, 0], [0.20, 0.14, 0.21]);
  cuboid(group, 'person_torso', material, [0, 0.035, 0], [0.40, 0.42, 0.50]);
  [-1, 1].forEach((side) => {
    cuboid(group, `person_arm_${side}`, material, [0, -0.015, side * 0.35], [0.25, 0.43, 0.16]);
    cuboid(group, `person_leg_${side}`, accent, [0, -0.325, side * 0.16], [0.30, 0.35, 0.23]);
  });
}

function vehicleShape(group, kind, material, windowMaterial, tireMaterial) {
  cuboid(group, 'vehicle_body', material, [0, -0.10, 0], [1, 0.34, 0.88]);
  if (kind === 'car') {
    cuboid(group, 'vehicle_cabin', material, [-0.07, 0.21, 0], [0.53, 0.42, 0.76]);
    cuboid(group, 'vehicle_front_window', windowMaterial, [0.20, 0.25, 0], [0.012, 0.26, 0.66]);
    [-1, 1].forEach((side) => cuboid(group, `vehicle_side_window_${side}`, windowMaterial,
      [-0.07, 0.26, side * 0.385], [0.43, 0.24, 0.012]));
  } else if (kind === 'truck') {
    cuboid(group, 'vehicle_cargo', material, [-0.15, 0.23, 0], [0.70, 0.54, 0.91]);
    cuboid(group, 'vehicle_cabin', material, [0.34, 0.16, 0], [0.30, 0.44, 0.80]);
    cuboid(group, 'vehicle_front_window', windowMaterial, [0.496, 0.23, 0], [0.006, 0.20, 0.68]);
  } else {
    cuboid(group, 'vehicle_cabin', material, [0, 0.17, 0], [0.98, 0.66, 0.90]);
    cuboid(group, 'vehicle_front_window', windowMaterial, [0.494, 0.25, 0], [0.010, 0.31, 0.75]);
    [-1, 1].forEach((side) => cuboid(group, `vehicle_side_window_${side}`, windowMaterial,
      [0, 0.26, side * 0.456], [0.81, 0.27, 0.01]));
  }
  [-1, 1].forEach((axle) => [-1, 1].forEach((side) => {
    const wheel = part(group, `vehicle_wheel_${axle}_${side}`,
      new THREE.CylinderGeometry(0.14, 0.14, 0.12, 10), tireMaterial,
      [axle * 0.30, -0.35, side * 0.44]);
    wheel.rotation.x = Math.PI / 2;
  }));
}

function cycleShape(group, material, tireMaterial) {
  [-1, 1].forEach((side) => {
    const wheel = part(group, `cycle_wheel_${side}`,
      new THREE.TorusGeometry(0.20, 0.035, 5, 12), tireMaterial, [side * 0.265, -0.265, 0]);
    wheel.scale.z = 1.8;
  });
  cuboid(group, 'cycle_frame', material, [0, -0.02, 0], [0.58, 0.16, 0.14]);
  cuboid(group, 'cycle_handle_stem', material, [0.27, 0.15, 0], [0.055, 0.60, 0.055]);
  cuboid(group, 'cycle_handlebar', tireMaterial, [0.27, 0.42, 0], [0.065, 0.065, 0.70]);
  cuboid(group, 'cycle_seat_stem', material, [-0.16, 0.11, 0], [0.055, 0.34, 0.055]);
  cuboid(group, 'cycle_seat', tireMaterial, [-0.16, 0.30, 0], [0.24, 0.08, 0.27]);
}

export function createNavigationObjectVisual(object) {
  const group = new THREE.Group();
  const kind = classifyNavigationObject(object?.class_name);
  const size = navigationObjectDimensions(object);
  const color = COLORS[kind];
  group.name = 'received_navigation_object';
  group.userData = { classKind: kind, sizeMode: size ? 'observed' : 'symbolic',
    geometrySource: size ? 'observed_lidar_extent' : null,
    semanticShape: 'class_illustration', dimensions: size, labelHeight: 0.12 };
  if (!size) {
    // HH_261002 - Missing extents get a symbolic locator, not a fabricated human/car box.
    part(group, 'unmeasured_object_locator', new THREE.OctahedronGeometry(0.08),
      new THREE.MeshBasicMaterial({ color }), [0, 0, 0]);
    return group;
  }

  group.quaternion.copy(mapQuaternionToThree(object.orientation || object.bbox?.orientation));
  const metric = new THREE.Group();
  metric.name = 'received_metric_dimensions';
  metric.scale.set(size.x, size.z, size.y);
  group.add(metric);
  const boxGeometry = new THREE.BoxGeometry(1, 1, 1);
  const edges = new THREE.EdgesGeometry(boxGeometry);
  boxGeometry.dispose();
  const outline = new THREE.LineSegments(edges, new THREE.LineBasicMaterial({
    color, transparent: true, opacity: 0.88, depthWrite: false }));
  outline.name = 'received_bounding_box';
  metric.add(outline);

  const silhouette = new THREE.Group();
  silhouette.name = 'illustrative_class_silhouette';
  metric.add(silhouette);
  const material = new THREE.MeshStandardMaterial({ color, roughness: 0.85 });
  const accent = new THREE.MeshStandardMaterial({ color: 0x344952, roughness: 1 });
  if (kind === 'person') personShape(silhouette, material, accent);
  else if (['car', 'truck', 'bus'].includes(kind)) {
    const windows = new THREE.MeshStandardMaterial({ color: 0xb4dfeb, roughness: 0.45 });
    vehicleShape(silhouette, kind, material, windows, accent);
  } else if (['bicycle', 'motorcycle'].includes(kind)) cycleShape(silhouette, material, accent);
  else {
    // Unknown detections retain their measured box and make no class/shape claim.
    material.transparent = true;
    material.opacity = 0.16;
    material.depthWrite = false;
    cuboid(silhouette, 'unclassified_extent', material, [0, 0, 0], [1, 1, 1]);
    accent.dispose();
  }
  updateNavigationObjectVisual(group, object);
  return group;
}

export function updateNavigationObjectVisual(group, object) {
  // HH_261002 - Reuse geometry/materials across sensor frames; only a class or metric/
  // symbolic transition needs replacement. A false return asks the scene to rebuild.
  const size = navigationObjectDimensions(object);
  const mode = size ? 'observed' : 'symbolic';
  if (group.userData.classKind !== classifyNavigationObject(object?.class_name)
    || group.userData.sizeMode !== mode) return false;
  group.userData.dimensions = size;
  group.userData.labelHeight = 0.12;
  if (size) {
    group.quaternion.copy(mapQuaternionToThree(object.orientation || object.bbox?.orientation));
    group.getObjectByName('received_metric_dimensions').scale.set(size.x, size.z, size.y);
    const matrix = new THREE.Matrix4().makeRotationFromQuaternion(group.quaternion).elements;
    group.userData.labelHeight = (Math.abs(matrix[1]) * size.x
      + Math.abs(matrix[5]) * size.z + Math.abs(matrix[9]) * size.y) / 2 + 0.12;
  }
  return true;
}
