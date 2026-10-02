import * as THREE from 'three';

// HH_261002 - These soft plant silhouettes are decorative artwork only. Site
// selection and road/route/service-area exclusions stay with the caller.
const SHRUB_LOBES = [
  { x: -0.09, y: 0.36, z: 0, width: 0.49, height: 0.36, depth: 0.43, shade: 1 },
  { x: 0.28, y: 0.24, z: 0.08, width: 0.32, height: 0.24, depth: 0.29, shade: 1.035 },
  { x: -0.23, y: 0.22, z: -0.16, width: 0.28, height: 0.22, depth: 0.29, shade: 0.965 },
];
const TREE_TIERS = [
  { bottom: 0.70, radius: 0.92, height: 1.60, shade: 0.96 },
  { bottom: 1.32, radius: 0.72, height: 1.38, shade: 1 },
  { bottom: 1.93, radius: 0.49, height: 1.12, shade: 1.04 },
];

function validSites(sites, origin) {
  if (!Array.isArray(sites)) return [];
  return sites.filter((site) => Number.isFinite(site?.x) && Number.isFinite(site?.y)
    && Number.isFinite(site?.scale) && site.scale > 0
    && Math.abs(site.x - origin.x) <= 1e8 && Math.abs(site.y - origin.y) <= 1e8
    && site.scale <= 1e4);
}

function plantBatch(name, geometry, color, count) {
  const mesh = new THREE.InstancedMesh(geometry,
    new THREE.MeshStandardMaterial({ color, roughness: 1, flatShading: false }), count);
  mesh.name = name;
  return mesh;
}

function finishBatch(group, mesh) {
  mesh.instanceMatrix.needsUpdate = true;
  mesh.instanceColor.needsUpdate = true;
  mesh.computeBoundingSphere();
  group.add(mesh);
}

// HH_261002 - A rounded skirt and gently tapering shoulder avoid the hard base
// and seven-sided outline of a single cone. Lathe normals remain smooth across
// its sixteen radial segments; the three tiers share one geometry/draw call.
function roundedCanopyGeometry() {
  const geometry = new THREE.LatheGeometry([
    [0, 0], [0.56, 0.015], [0.86, 0.045], [0.99, 0.09], [1, 0.15],
    [0.92, 0.26], [0.76, 0.43], [0.54, 0.64], [0.29, 0.85],
    [0.10, 0.97], [0, 1],
  ].map(([radius, height]) => new THREE.Vector2(radius, height)), 16);
  geometry.normalizeNormals();
  return geometry;
}

/** HH_261002 - Batch rounded vegetation into at most three draw calls without
 * moving any accepted site. All shrub vertices stay within 0.65 * scale of
 * their site, tree vertices within 0.95 * scale, and tree tops at 3.05 * scale.
 * No textures, shadows, random state, or sensor/map semantics are introduced. */
export function createNavigationVegetation(sites, origin, dark = false) {
  const group = new THREE.Group();
  group.name = 'illustrative_navigation_vegetation';
  group.userData = { illustrative: true, shrubs: 0, trees: 0 };
  if (!Number.isFinite(origin?.x) || !Number.isFinite(origin?.y)) return group;
  const shrubs = validSites(sites?.shrubs, origin), trees = validSites(sites?.trees, origin);
  group.userData.shrubs = shrubs.length;
  group.userData.trees = trees.length;
  const dummy = new THREE.Object3D(), tint = new THREE.Color();

  if (shrubs.length) {
    const mesh = plantBatch('rounded_shrubs', new THREE.SphereGeometry(1, 16, 8),
      dark ? 0x527868 : 0x688e6e, shrubs.length * SHRUB_LOBES.length);
    shrubs.forEach((site, index) => {
      const angle = index * 2.4, cosine = Math.cos(angle), sine = Math.sin(angle);
      SHRUB_LOBES.forEach((lobe, lobeIndex) => {
        const instance = index * SHRUB_LOBES.length + lobeIndex;
        dummy.position.set(site.x - origin.x + (lobe.x * cosine + lobe.z * sine) * site.scale,
          lobe.y * site.scale, -(site.y - origin.y) + (-lobe.x * sine + lobe.z * cosine) * site.scale);
        dummy.rotation.set(0, angle, 0);
        dummy.scale.set(lobe.width * site.scale, lobe.height * site.scale, lobe.depth * site.scale);
        dummy.updateMatrix();
        mesh.setMatrixAt(instance, dummy.matrix);
        const shade = lobe.shade * (0.97 + (index % 5) * 0.015);
        mesh.setColorAt(instance, tint.setRGB(shade, shade, shade));
      });
    });
    finishBatch(group, mesh);
  }

  if (trees.length) {
    const trunks = plantBatch('rounded_tree_trunks', new THREE.CylinderGeometry(0.085, 0.125, 1.35, 16),
      dark ? 0x635f4b : 0x80725b, trees.length);
    const foliage = plantBatch('layered_tree_canopies', roundedCanopyGeometry(),
      dark ? 0x315c53 : 0x49775c, trees.length * TREE_TIERS.length);
    trees.forEach((site, index) => {
      const x = site.x - origin.x, z = -(site.y - origin.y), shade = 0.97 + (index % 5) * 0.015;
      dummy.position.set(x, 0.675 * site.scale, z);
      dummy.rotation.set(0, index * 1.7, 0);
      dummy.scale.setScalar(site.scale);
      dummy.updateMatrix();
      trunks.setMatrixAt(index, dummy.matrix);
      trunks.setColorAt(index, tint.setRGB(shade, shade, shade));
      TREE_TIERS.forEach((tier, tierIndex) => {
        const instance = index * TREE_TIERS.length + tierIndex;
        dummy.position.set(x, tier.bottom * site.scale, z);
        dummy.scale.set(tier.radius * site.scale, tier.height * site.scale, tier.radius * site.scale);
        dummy.updateMatrix();
        foliage.setMatrixAt(instance, dummy.matrix);
        const tierShade = shade * tier.shade;
        foliage.setColorAt(instance, tint.setRGB(tierShade, tierShade, tierShade));
      });
    });
    finishBatch(group, trunks);
    finishBatch(group, foliage);
  }
  return group;
}
