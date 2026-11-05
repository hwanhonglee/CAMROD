// HJ_261105 - The decimated GLB wheels carry stray chassis fragments that
// orbit with the spin joint and a ragged tread silhouette. Draw a clean,
// rotationally symmetric display tire under each existing spin node instead.
// Pivots, rolling radius and steer/spin axes are unchanged (spin axis local Z).
import * as THREE from 'three';
import { RANGER_WHEEL_RADIUS_M } from './navigationMath';

export const TIRE_WIDTH_M = 0.115;
export const RIM_RADIUS_M = 0.098;
const SHOULDER_M = 0.02;
const SPOKE_HOLES = 5;

function tireProfile(radius, halfWidth, rimRadius, shoulder) {
  const points = [new THREE.Vector2(rimRadius, -halfWidth)];
  const arc = (cx, cy, from, to) => {
    for (let index = 0; index <= 6; index += 1) {
      const angle = from + (to - from) * index / 6;
      points.push(new THREE.Vector2(cx + shoulder * Math.cos(angle), cy + shoulder * Math.sin(angle)));
    }
  };
  arc(radius - shoulder, -halfWidth + shoulder, -Math.PI / 2, 0);
  arc(radius - shoulder, halfWidth - shoulder, 0, Math.PI / 2);
  points.push(new THREE.Vector2(rimRadius, halfWidth));
  points.push(new THREE.Vector2(rimRadius, -halfWidth));
  return points;
}

export function createCleanTireParts(dark = false) {
  const halfWidth = TIRE_WIDTH_M / 2;
  // Lathe revolves around +Y; rotate so the tire axis is the spin axis (+Z).
  const tire = new THREE.LatheGeometry(
    tireProfile(RANGER_WHEEL_RADIUS_M, halfWidth, RIM_RADIUS_M, SHOULDER_M), 64);
  tire.rotateX(Math.PI / 2);
  tire.computeVertexNormals();

  const face = new THREE.Shape();
  face.absarc(0, 0, RIM_RADIUS_M + 0.002, 0, Math.PI * 2, false);
  for (let index = 0; index < SPOKE_HOLES; index += 1) {
    const angle = index * 2 * Math.PI / SPOKE_HOLES;
    const hole = new THREE.Path();
    hole.absarc(0.062 * Math.cos(angle), 0.062 * Math.sin(angle), 0.021, 0, Math.PI * 2, true);
    face.holes.push(hole);
  }
  const rimDepth = TIRE_WIDTH_M - 0.03;
  const rim = new THREE.ExtrudeGeometry(face, { depth: rimDepth, bevelEnabled: true,
    bevelThickness: 0.004, bevelSize: 0.003, bevelSegments: 2, curveSegments: 40 });
  rim.translate(0, 0, -rimDepth / 2);
  const hub = new THREE.CylinderGeometry(0.03, 0.03, rimDepth + 0.016, 32);
  hub.rotateX(Math.PI / 2);
  // A dark disc inside the rim gives the spoke holes visible depth.
  const well = new THREE.CircleGeometry(RIM_RADIUS_M, 40);

  const standard = (color, roughness, metalness) => new THREE.MeshStandardMaterial({
    color, roughness, metalness });
  return {
    geometries: { tire, rim, hub, well },
    materials: {
      tire: standard(0x222528, 0.92, 0),
      rim: standard(dark ? 0xd6dcde : 0xc3cacd, 0.38, 0.55),
      hub: standard(0x5b6468, 0.5, 0.45),
      well: new THREE.MeshBasicMaterial({ color: 0x121416, side: THREE.DoubleSide }),
    },
  };
}

export function createCleanTire(parts) {
  const group = new THREE.Group();
  group.name = 'clean_tire';
  ['well', 'rim', 'hub', 'tire'].forEach((key) => {
    const mesh = new THREE.Mesh(parts.geometries[key], parts.materials[key]);
    mesh.name = `clean_tire_${key}`;
    group.add(mesh);
  });
  return group;
}

// Hide each original wheel mesh and attach a clean tire to its spin node.
// Wheels whose nodes are missing keep the original GLB geometry.
export function replaceWheelMeshesWithCleanTires(model, wheelNames, dark = false) {
  if (!model || !Array.isArray(wheelNames)) return 0;
  let parts = null, replaced = 0;
  wheelNames.forEach((name) => {
    const spin = model.getObjectByName(`wheel_${name}_spin`);
    const original = model.getObjectByName(`wheel_${name}`);
    if (!spin || !original) return;
    parts = parts || createCleanTireParts(dark);
    original.visible = false;
    spin.add(createCleanTire(parts));
    replaced += 1;
  });
  return replaced;
}
