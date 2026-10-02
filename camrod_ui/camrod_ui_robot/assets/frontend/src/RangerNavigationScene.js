import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { advanceWheelRoll, bodyPoseDelta, buildRouteRibbon, clamp, estimateWheelStep, interpolatePose,
  mapToThree, motionMayAnimate, navigationCameraHeading, navigationMotionKind, poseIsFresh, wrapAngle } from './navigationMath';
import { illustrativeScenerySites, illustrativeMapScenerySites, illustrativeMapRoads } from './illustrativeScenery';
import { classifyNavigationObject, createNavigationObjectVisual,
  updateNavigationObjectVisual, navigationObjectDimensions } from './navigationObjectVisuals';
import { RANGER_MODEL_URL, RANGER_SIDE_WRAP_URL, RANGER_FRONT_WRAP_URL, RANGER_REAR_WRAP_URL } from './rangerModelAsset';
import { areaIsDestination, areaLabelPoint } from './navigationAreas';
import { createNavigationAreaVisuals, sceneryClearOfAreas } from './navigationAreaVisuals';
import './RangerNavigationScene.css';

const WHEEL_NAMES = ['fl', 'fr', 'rl', 'rr'];
// HH_261002 - Display-only paint width, not a measured road width or safety margin.
export const LANE_BOUNDARY_WIDTH_M = 0.10;

// HH_261002 - Class icons identify semantic output, not body shape or heading measurements.
export function NavigationObjectIcon({ className }) {
  const kind = classifyNavigationObject(className);
  return <svg className="ranger-navigation-object-icon" viewBox="0 0 24 24" aria-hidden="true"
    fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round">
    {kind === 'person' ? <><circle cx="12" cy="4" r="2.2" /><path d="M8 12V9a4 4 0 0 1 8 0v3M12 7v8m0 0-4 7m4-7 4 7" /></>
      : ['car', 'truck', 'bus'].includes(kind) ? <><path d="M3 16V11l3-6h12l3 6v5ZM4 11h16M8 5l-2 6m10-6 2 6" />
        <circle cx="6" cy="17" r="2" /><circle cx="18" cy="17" r="2" /></>
        : <><path d="m12 2 9 5v10l-9 5-9-5V7Zm0 10L3 7m9 5 9-5m-9 5v10" /></>}
  </svg>;
}

function objectGeometryCaption(object, demo) {
  const size = navigationObjectDimensions(object);
  return size ? `${demo ? '시연 크기' : '관측 범위'} ${size.x.toFixed(2)} × ${size.y.toFixed(2)} × ${size.z.toFixed(2)} m`
    : '크기 미확인 · 분류 아이콘';
}

function softShadowTexture() {
  const size = 64;
  const pixels = new Uint8Array(size * size * 4);
  for (let y = 0; y < size; y += 1) for (let x = 0; x < size; x += 1) {
    const distance = Math.hypot((x + 0.5 - size / 2) / (size / 2), (y + 0.5 - size / 2) / (size / 2));
    const offset = (y * size + x) * 4;
    pixels[offset] = 21; pixels[offset + 1] = 36; pixels[offset + 2] = 31;
    pixels[offset + 3] = Math.round(255 * Math.pow(Math.max(0, 1 - distance), 1.7));
  }
  const texture = new THREE.DataTexture(pixels, size, size, THREE.RGBAFormat);
  texture.magFilter = THREE.LinearFilter;
  texture.minFilter = THREE.LinearFilter;
  texture.needsUpdate = true;
  return texture;
}

// HH_261002 - Map centerlines may be sparse but connected. Keep each line separate;
// only route fallback artwork retains its historical 15 m discontinuity limit.
export function illustratedRoadPositions(paths, origin, width, height, maxSegmentLength = Infinity) {
  const positions = [];
  paths.forEach((points) => {
    for (let index = 0; index < points.length - 1; index += 1) {
      const a = mapToThree(points[index], origin);
      const b = mapToThree(points[index + 1], origin);
      const length = Math.hypot(b.x - a.x, b.z - a.z);
      if (length < 0.001 || length > maxSegmentLength) continue;
      const previous = mapToThree(points[Math.max(0, index - 1)], origin);
      const next = mapToThree(points[Math.min(points.length - 1, index + 2)], origin);
      const edge = (before, center, after) => {
        const tx = after.x - before.x, tz = after.z - before.z;
        const tangentLength = Math.hypot(tx, tz) || 1;
        return { x: center.x - tz / tangentLength * width / 2, z: center.z + tx / tangentLength * width / 2 };
      };
      const leftA = edge(previous, a, b), leftB = edge(a, b, next);
      const rightA = { x: 2 * a.x - leftA.x, z: 2 * a.z - leftA.z };
      const rightB = { x: 2 * b.x - leftB.x, z: 2 * b.z - leftB.z };
      positions.push(leftA.x, height, leftA.z, leftB.x, height, leftB.z, rightA.x, height, rightA.z,
        rightA.x, height, rightA.z, leftB.x, height, leftB.z, rightB.x, height, rightB.z);
    }
  });
  return positions;
}

// HH_261002 - A static map remains visible without a mission or fresh robot pose.
// Width remains illustrative; bounds-only maps use checked producer marker pairs.
export function navigationIllustrationInput(data) {
  const roads = illustrativeMapRoads(data.baseMap);
  const route = hasIllustrativeRoute(data) ? data.route : [];
  if (roads.length) return { source: 'base_map', roads, route };
  return route.length ? { source: 'route', roads: [route], route } : null;
}

function makeNavigationIllustration(input, baseMap, origin, dark) {
  const world = new THREE.Group();
  world.name = `illustrative_${input.source}_environment`;
  const shoulder = new THREE.MeshStandardMaterial({ color: dark ? 0x55594b : 0xc8c6ae,
    roughness: 1, side: THREE.DoubleSide });
  const path = new THREE.MeshStandardMaterial({ color: dark ? 0x858c83 : 0xb8bcb2,
    roughness: 0.96, side: THREE.DoubleSide });
  // HH_261002 - Merge the road network into two draw calls, below the independently
  // received blue route overlay. These surfaces never enter planning/collision data.
  [[3.8, 0.002, shoulder], [2.85, 0.008, path]].forEach(([width, height, material]) => {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(
      illustratedRoadPositions(input.roads, origin, width, height, input.source === 'route' ? 15 : Infinity), 3));
    geometry.computeVertexNormals();
    world.add(new THREE.Mesh(geometry, material));
  });

  // HH_261002 - Clear the entire map network and active route, not just the nearest
  // lane, so decorative trees never occupy another branch of the displayed road.
  const candidates = input.source === 'base_map' ? illustrativeMapScenerySites(baseMap, input.route)
    : illustrativeScenerySites(input.route);
  // HH_261002 - Authored camping/drop-zone footprints remain free of example trees.
  const areas = baseMap?.areas || [];
  const sites = { shrubs: candidates.shrubs.filter(site => sceneryClearOfAreas(site, areas, 0.8)),
    trees: candidates.trees.filter(site => sceneryClearOfAreas(site, areas, 1.5)) };
  world.userData = { source: input.source, roadCount: input.roads.length,
    shrubs: sites.shrubs.length, trees: sites.trees.length };
  const shrubSites = sites.shrubs.map((site) => ({ ...mapToThree([site.x, site.y], origin), scale: site.scale }));
  const treeSites = sites.trees.map((site) => ({ ...mapToThree([site.x, site.y], origin), scale: site.scale }));
  const dummy = new THREE.Object3D();
  const shrubs = new THREE.InstancedMesh(new THREE.IcosahedronGeometry(0.45, 0),
    new THREE.MeshStandardMaterial({ color: dark ? 0x527868 : 0x688e6e, roughness: 1 }), shrubSites.length);
  shrubSites.forEach((site, index) => {
    dummy.position.set(site.x, 0.23 * site.scale, site.z);
    dummy.rotation.set(0, index * 2.4, 0);
    dummy.scale.set(site.scale * 1.35, site.scale * 0.55, site.scale);
    dummy.updateMatrix(); shrubs.setMatrixAt(index, dummy.matrix);
  });
  shrubs.instanceMatrix.needsUpdate = true;
  world.add(shrubs);

  const trunks = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.09, 0.13, 1.35, 5),
    new THREE.MeshStandardMaterial({ color: dark ? 0x635f4b : 0x80725b, roughness: 1 }), treeSites.length);
  const canopies = new THREE.InstancedMesh(new THREE.ConeGeometry(0.95, 2.2, 7),
    new THREE.MeshStandardMaterial({ color: dark ? 0x315c53 : 0x49775c, roughness: 1 }), treeSites.length);
  treeSites.forEach((site, index) => {
    dummy.position.set(site.x, 0.67 * site.scale, site.z);
    dummy.rotation.set(0, index * 1.7, 0);
    dummy.scale.setScalar(site.scale);
    dummy.updateMatrix(); trunks.setMatrixAt(index, dummy.matrix);
    dummy.position.y = 1.95 * site.scale;
    dummy.updateMatrix(); canopies.setMatrixAt(index, dummy.matrix);
  });
  trunks.instanceMatrix.needsUpdate = true;
  canopies.instanceMatrix.needsUpdate = true;
  world.add(trunks, canopies);
  return world;
}

function disposeTree(root) {
  const geometries = new Set(), materials = new Set(), textures = new Set(), instances = new Set();
  root.traverse((object) => {
    if (object.isInstancedMesh) instances.add(object);
    if (object.geometry) geometries.add(object.geometry);
    const list = Array.isArray(object.material) ? object.material : [object.material];
    list.filter(Boolean).forEach((material) => {
      materials.add(material);
      Object.values(material).forEach((value) => { if (value?.isTexture) textures.add(value); });
    });
  });
  instances.forEach((instance) => instance.dispose());
  textures.forEach((texture) => texture.dispose());
  materials.forEach((material) => material.dispose());
  geometries.forEach((geometry) => geometry.dispose());
}

function replacePositions(object, values) {
  const previous = object.geometry;
  object.geometry = new THREE.BufferGeometry();
  object.geometry.setAttribute('position', new THREE.Float32BufferAttribute(values, 3));
  if (values.length) object.geometry.computeBoundingSphere();
  object.visible = values.length > 0;
  previous.dispose();
}

function applyModelSurfaceFinish(model, dark) {
  // HH_261001 - The imported mesh names identify physical surfaces. Keep this cosmetic
  // finish separate from the GLB dimensions, wheel pivots, and telemetry pose.
  model.traverse((object) => {
    if (!object.isMesh) return;
    object.castShadow = false;
    object.receiveShadow = false;
    const materials = Array.isArray(object.material) ? object.material : [object.material];
    materials.forEach((material) => {
      if (!material?.isMeshStandardMaterial) return;
      if (material.name.includes('body_white')) {
        material.color.set(dark ? 0xf0f2ee : 0xe8eeeb);
        material.roughness = 0.7;
        material.metalness = 0.07;
      } else if (material.name.includes('tire_rubber')) {
        material.color.set(0x292d30);
        material.roughness = 1;
        material.metalness = 0;
      } else if (material.name.includes('chassis_metal')) {
        material.color.set(0x7d8987);
        material.roughness = 0.66;
        material.metalness = 0.38;
      }
    });
  });
}

export function configureCargoPreview(model, showCargoPreview) {
  // HH_261001 - Old cached GLBs must not flash the superseded shelf during an asset update.
  const shelf = model.getObjectByName('accessory_shelf_preview');
  if (shelf) shelf.visible = false;
  const cargo = model.getObjectByName('accessory_cargo_preview');
  if (cargo) cargo.visible = Boolean(showCargoPreview);
  return Boolean(cargo && showCargoPreview);
}

/** HH_261002 - Route fallback artwork needs a fresh pose; static map artwork does not. */
export function hasIllustrativeRoute(data) {
  return Boolean(poseIsFresh(data) && Array.isArray(data.route) && data.route.length >= 2);
}

/** HH_261002 - The normal navigation camera tracks directly behind the robot.
 * Reserve the lateral orbit offset for the optional exterior/detail view only. */
export function navigationCameraSideOffset(detailBlend) {
  return 2.75 * clamp(detailBlend, 0, 1);
}

/** HH_261002 - Convert only received Lanelet map lines into metre-space segments.
 * This geometry does not depend on a mission path or illustrative road art. */
export function baseMapLinePositions(baseMap, origin) {
  if (!baseMap?.valid || !Array.isArray(baseMap.polylines)) return [];
  const positions = [];
  baseMap.polylines.forEach((line) => {
    for (let index = 0; index + 1 < line.points.length; index += 1) {
      const a = mapToThree(line.points[index], origin, 0.025);
      const b = mapToThree(line.points[index + 1], origin, 0.025);
      positions.push(a.x, a.y, a.z, b.x, b.y, b.z);
    }
  });
  return positions;
}

/** HH_261002 - Paint only received left/right boundaries green, matching the
 * on-site lane markings. Centerlines remain neutral; the route stays blue.
 * Per-vertex colors keep the cached map in one draw call without altering XY. */
export function baseMapLineColors(baseMap, dark = false) {
  if (!baseMap?.valid || !Array.isArray(baseMap.polylines)) return [];
  const colors = [];
  baseMap.polylines.forEach((line) => {
    const boundary = ['lanelet/left_bound', 'lanelet/right_bound'].includes(line.namespace);
    const color = new THREE.Color(boundary ? (dark ? '#4ade80' : '#1ea65a')
      : (dark ? '#91ada8' : '#506c60'));
    for (let index = 0; index + 1 < line.points.length; index += 1) {
      colors.push(color.r, color.g, color.b, color.r, color.g, color.b);
    }
  });
  return colors;
}

/** HH_261002 - Use a ground ribbon instead of WebGL line width (often limited
 * to one pixel). A 10 cm strip follows each received boundary independently;
 * do not join separate lanes, widen the road, or move the underlying map. */
export function boundaryPaintPositions(baseMap, origin) {
  if (!baseMap?.valid || !Array.isArray(baseMap.polylines)) return [];
  const paths = baseMap.polylines.filter((line) =>
    ['lanelet/left_bound', 'lanelet/right_bound'].includes(line.namespace)).map(line => line.points);
  return illustratedRoadPositions(paths, origin, LANE_BOUNDARY_WIDTH_M, 0.027);
}

/** HH_261002 - Explicitly bind map visibility to received line geometry, not route state. */
export function syncBaseMapVisibility(lines, valid) {
  lines.visible = Boolean(valid && lines.geometry?.attributes?.position?.count > 0);
  return lines.visible;
}

/** HH_261002 - A real off-route detection can project just beyond the camera.
 * Keep its label inside the viewport with an edge direction instead of
 * silently hiding a forward object that is still in the measured sensor field. */
export function projectVisibleObjectLabel(projected, width, height) {
  if (!Number.isFinite(projected?.x) || !Number.isFinite(projected?.y)
    || !Number.isFinite(projected?.z) || projected.z < -1 || projected.z > 1) return null;
  const rawX = (projected.x + 1) * width / 2;
  const rawY = (1 - projected.y) * height / 2;
  const minX = Math.min(width / 2, 72), maxX = Math.max(width / 2, width - 72);
  const minY = Math.min(height / 2, 48), maxY = Math.max(height / 2, height - 48);
  const x = clamp(rawX, minX, maxX), y = clamp(rawY, minY, maxY);
  const edge = rawX > maxX ? 'right' : rawX < minX ? 'left'
    : rawY < minY ? 'top' : rawY > maxY ? 'bottom' : '';
  return { x, y, edge };
}

/** HH_261001 - Read-only, metres-based navigation. Received poses are interpolated, never
 * extrapolated. No robot command, state transition, or network control exists. */
export default function RangerNavigationScene({ data, demo = false, theme = 'light',
  modelUrl = RANGER_MODEL_URL, wrapUrl = RANGER_SIDE_WRAP_URL,
  frontWrapUrl = RANGER_FRONT_WRAP_URL, rearWrapUrl = RANGER_REAR_WRAP_URL, showCargoPreview = true }) {
  const hostRef = useRef(null);
  const controllerRef = useRef(null);
  const latestRef = useRef(data);
  const detailRef = useRef(false);
  const detailAngleRef = useRef(0);
  const objectLabelsRef = useRef(new Map());
  const areaLabelsRef = useRef(new Map());
  const [detailView, setDetailView] = useState(false);
  const [detailAngleIndex, setDetailAngleIndex] = useState(0);
  const [modelStatus, setModelStatus] = useState('loading');
  const [cargoVisible, setCargoVisible] = useState(false);
  const [wrapVisible, setWrapVisible] = useState(false);
  latestRef.current = data;

  useEffect(() => {
    const host = hostRef.current;
    if (!host) return undefined;
    let disposed = false, renderer, animationId = 0, observer;
    let model = null, wheels = [], modelLength = null, motionKind = 'stationary';
    let routeWorld = null, routeWorldOrigin = null, routeWorldKey = '';
    let baseMapWorldOrigin = null, baseMapWorldKey = '';
    let areaWorld = null, areaWorldOrigin = null, areaWorldKey = '';
    let cargoShown = false, orbitAngle = detailAngleRef.current;
    const loadedWrapFaces = new Set();
    let receivedAt = performance.now(), lastTargetAt = receivedAt, lastFrame = 0, lastDebug = 0, renderedFrames = 0;
    let sample = latestRef.current;
    let currentPose = sample.pose ? { ...sample.pose } : null;
    let origin = { x: currentPose?.x || 0, y: currentPose?.y || 0 };
    let transition = null, cameraHeading = currentPose?.yaw || 0, detailBlend = detailRef.current ? 1 : 0;
    const dark = theme === 'dark';
    const scene = new THREE.Scene();
    const fogColor = new THREE.Color(dark ? '#1b2c32' : '#e9eee8');
    scene.fog = new THREE.Fog(fogColor, dark ? 18 : 22, dark ? 56 : 70);
    const camera = new THREE.PerspectiveCamera(44, 1, 0.06, 110);
    const robot = new THREE.Group();
    robot.name = 'telemetry_robot_pose';
    scene.add(robot);
    scene.add(new THREE.HemisphereLight(0xf7fdff, dark ? 0x435d59 : 0x9baaa0, dark ? 1.8 : 1.65));
    const keyLight = new THREE.DirectionalLight(dark ? 0xf5fbff : 0xffffff, dark ? 2.5 : 2.35);
    keyLight.position.set(-5, 9, 7);
    scene.add(keyLight);
    const fillLight = new THREE.DirectionalLight(dark ? 0xaed4cf : 0xd7e9e7, dark ? 1.05 : 0.85);
    fillLight.position.set(5, 5, -6);
    scene.add(fillLight);

    const floor = new THREE.Mesh(new THREE.PlaneGeometry(160, 160),
      new THREE.MeshStandardMaterial({ color: dark ? 0x354a42 : 0x879e81, roughness: 1 }));
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.025;
    scene.add(floor);

    const routeHalo = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({
      color: dark ? 0x65c8ed : 0x1a9bc9, transparent: true, opacity: dark ? 0.2 : 0.16,
      depthWrite: false, side: THREE.DoubleSide }));
    const route = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({
      color: dark ? 0x7bd7ef : 0x087fae, side: THREE.DoubleSide, depthWrite: false }));
    route.name = 'received_metric_route';
    routeHalo.visible = false; route.visible = false;
    scene.add(routeHalo, route);
    const baseMapLines = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({
      vertexColors: true, transparent: true, opacity: 0.92,
      depthWrite: false, toneMapped: false }));
    baseMapLines.name = 'received_lanelet_basemap';
    baseMapLines.visible = false;
    scene.add(baseMapLines);
    // HH_261002 - Merge all painted boundaries into one cached mesh. Depth places
    // paint above the road/line layer but below the independently received route.
    const boundaryPaint = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({
      color: dark ? '#4ade80' : '#1ea65a', side: THREE.DoubleSide, toneMapped: false }));
    boundaryPaint.name = 'received_lanelet_boundary_paint';
    boundaryPaint.visible = false;
    scene.add(boundaryPaint);
    const points = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({
      color: dark ? 0xa0d3bb : 0x4f846d, size: 0.085, transparent: true, opacity: 0.82,
      sizeAttenuation: true, depthWrite: false }));
    // HH_261002 - Reuse per-track meshes; telemetry must not rebuild materials each frame.
    const detections = new THREE.Group();
    const objectVisuals = new Map();
    const labelAnchor = new THREE.Vector3();
    points.visible = false; detections.visible = false;
    scene.add(points, detections);
    const shadowMap = softShadowTexture();
    const shadowMaterial = new THREE.MeshBasicMaterial({ map: shadowMap, transparent: true,
      opacity: dark ? 0.43 : 0.37, depthWrite: false, toneMapped: false, polygonOffset: true,
      polygonOffsetFactor: -1 });
    const contact = new THREE.Mesh(new THREE.PlaneGeometry(2.05, 1.48), shadowMaterial);
    contact.rotation.x = -Math.PI / 2;
    contact.position.y = 0.015;
    robot.add(contact);

    try {
      renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'low-power' });
      renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
      renderer.setClearColor(fogColor, 1);
      renderer.outputColorSpace = THREE.SRGBColorSpace;
      renderer.toneMapping = THREE.ACESFilmicToneMapping;
      renderer.toneMappingExposure = dark ? 1.2 : 1.02;
      renderer.shadowMap.enabled = false;
      renderer.domElement.className = 'ranger-navigation-canvas';
      renderer.domElement.setAttribute('aria-label', demo
        ? 'Ranger 모델과 시연 경로 및 예시 캠핑장 환경의 3D 장면'
        : 'Ranger 모델과 수신 Lanelet 지도·주행 경로 및 예시 도로변 환경의 3D 장면');
      renderer.domElement.dataset.testid = 'ranger-navigation-canvas';
      renderer.domElement.dataset.modelStatus = 'loading';
      host.appendChild(renderer.domElement);
    } catch (_) {
      setModelStatus('webgl-unavailable');
      disposeTree(scene);
      renderer?.dispose?.();
      renderer?.forceContextLoss?.();
      renderer?.domElement?.remove();
      return undefined;
    }

    const resize = () => {
      const width = Math.max(1, host.clientWidth), height = Math.max(1, host.clientHeight);
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    };
    resize();
    if (typeof ResizeObserver !== 'undefined') {
      observer = new ResizeObserver(resize);
      observer.observe(host);
    } else window.addEventListener('resize', resize);

    const updateNavigationIllustration = () => {
      // HH_261002 - Do not resample map bounds or place scenery on every pose tick.
      // Receipt age, mission labels, and camera motion do not change this geometry.
      const routeKey = JSON.stringify([sample.baseMap?.valid ? sample.baseMap.polylines : null, sample.baseMap?.areas,
        hasIllustrativeRoute(sample) ? sample.route : []]);
      if (routeWorld && routeKey === routeWorldKey) {
        routeWorld.position.set(routeWorldOrigin.x - origin.x, 0, origin.y - routeWorldOrigin.y);
        return;
      }
      const input = navigationIllustrationInput(sample);
      if (!input) {
        if (routeWorld) { scene.remove(routeWorld); disposeTree(routeWorld); }
        routeWorld = null;
        routeWorldOrigin = null;
        routeWorldKey = '';
        return;
      }
      // HH_261002 - Geometry is cached across telemetry/mission updates. Include
      // the active path only to keep decorative footprints out of its corridor.
      if (routeKey !== routeWorldKey) {
        if (routeWorld) { scene.remove(routeWorld); disposeTree(routeWorld); }
        routeWorld = makeNavigationIllustration(input, sample.baseMap, origin, dark);
        routeWorldOrigin = { ...origin };
        routeWorldKey = routeKey;
        scene.add(routeWorld);
      }
      // HH_261001 - Keep illustrated landscape fixed in map coordinates as the camera origin resets.
      routeWorld.position.set(routeWorldOrigin.x - origin.x, 0, origin.y - routeWorldOrigin.y);
    };

    // HH_261002 - Service areas are independent of routes and pose freshness.
    // Rebuild only on authored geometry/destination changes, then rebase cheaply.
    const updateAreas = () => {
      const areas = sample.baseMap?.valid ? sample.baseMap.areas || [] : [];
      const key = JSON.stringify([areas, areas.map(area => areaIsDestination(area, sample.mission))]);
      if (key !== areaWorldKey) {
        if (areaWorld) { scene.remove(areaWorld); disposeTree(areaWorld); }
        areaWorld = areas.length ? createNavigationAreaVisuals(areas, origin,
          area => areaIsDestination(area, sample.mission), dark) : null;
        areaWorldOrigin = { ...origin };
        areaWorldKey = key;
        if (areaWorld) scene.add(areaWorld);
      }
      if (areaWorld) areaWorld.position.set(areaWorldOrigin.x - origin.x, 0, origin.y - areaWorldOrigin.y);
    };

    const updateBaseMap = () => {
      updateAreas();
      if (!sample.baseMap?.valid) {
        if (baseMapWorldKey) replacePositions(baseMapLines, []);
        if (baseMapWorldKey) replacePositions(boundaryPaint, []);
        baseMapWorldKey = '';
        baseMapWorldOrigin = null;
        syncBaseMapVisibility(baseMapLines, false);
        syncBaseMapVisibility(boundaryPaint, false);
        return;
      }
      const key = `${sample.baseMap.frame_id}:${JSON.stringify(sample.baseMap.polylines)}`;
      if (key !== baseMapWorldKey) {
        replacePositions(baseMapLines, baseMapLinePositions(sample.baseMap, origin));
        baseMapLines.geometry.setAttribute('color', new THREE.Float32BufferAttribute(
          baseMapLineColors(sample.baseMap, dark), 3));
        replacePositions(boundaryPaint, boundaryPaintPositions(sample.baseMap, origin));
        baseMapWorldOrigin = { ...origin };
        baseMapWorldKey = key;
      }
      // HH_261002 - Rebase a cached static map when the robot origin shifts;
      // route availability and route illustration must never clear this layer.
      baseMapLines.position.set(baseMapWorldOrigin.x - origin.x, 0,
        origin.y - baseMapWorldOrigin.y);
      syncBaseMapVisibility(baseMapLines, true);
      boundaryPaint.position.copy(baseMapLines.position);
      syncBaseMapVisibility(boundaryPaint, true);
    };

    const updateGeometry = () => {
      const valid = poseIsFresh(sample);
      const nearby = (point) => Math.hypot(point[0] - sample.pose.x, point[1] - sample.pose.y) < 65;
      const path = valid ? sample.route.filter(nearby).slice(0, 256) : [];
      replacePositions(route, buildRouteRibbon(path, origin, 0.24, 0.047));
      replacePositions(routeHalo, buildRouteRibbon(path, origin, 0.92, 0.034));
      // HH_261001 - Synthetic demo points look like airborne dust beside the authored path.
      // Live mode still renders every bounded, received cloud point unchanged.
      const cloud = valid && !demo ? sample.points.filter(nearby).slice(0, 600) : [];
      const pointPositions = cloud.flatMap((point) => {
        const position = mapToThree(point, origin, 0.06);
        return position.y >= -2 && position.y <= 5 ? [position.x, position.y, position.z] : [];
      });
      replacePositions(points, pointPositions);
      const seen = new Set();
      (valid && sample.perceptionReady ? sample.objects.slice(0, 32) : []).forEach((object) => {
        const point = [object.x, object.y, object.z];
        if (!nearby(point)) return;
        const center = navigationObjectDimensions(object) ? object.bbox?.center : null;
        const position = mapToThree(center ? [center.x, center.y, center.z] : point, origin, 0.12);
        if (position.y < -2 || position.y > 15) return;
        const key = `${object.id}:${object.class_name}`;
        let visual = objectVisuals.get(key);
        if (visual && !updateNavigationObjectVisual(visual, object)) {
          detections.remove(visual); disposeTree(visual); visual = null;
        }
        if (!visual) {
          visual = createNavigationObjectVisual(object);
          objectVisuals.set(key, visual);
          detections.add(visual);
        }
        visual.position.set(position.x, position.y, position.z);
        seen.add(key);
      });
      objectVisuals.forEach((visual, key) => {
        if (!seen.has(key)) { detections.remove(visual); disposeTree(visual); objectVisuals.delete(key); }
      });
      updateBaseMap();
      updateNavigationIllustration();
    };

    // HH_261001 - A new normalized telemetry snapshot updates pose and metric geometry. Only
    // consecutive nearby poses in the same frame are interpolated; a frame
    // HH_261002 - A frame switch, stale recovery or >2 m jump resets the renderer
    // rather than inventing wheel movement across a missing/discontinuous sample.
    const accept = (next) => {
      const now = performance.now();
      const previousFresh = poseIsFresh(sample, now - receivedAt);
      receivedAt = now;
      sample = next;
      if (!poseIsFresh(next)) transition = null;
      else {
        const newPose = { ...next.pose };
        const reset = !previousFresh || !currentPose || currentPose.frame_id !== newPose.frame_id
          || Math.hypot(newPose.x - currentPose.x, newPose.y - currentPose.y) > 2;
        if (reset) {
          currentPose = newPose;
          origin = { x: newPose.x, y: newPose.y };
          cameraHeading = newPose.yaw;
          motionKind = 'stationary';
          transition = null;
        } else if (Math.hypot(newPose.x - (transition?.to.x ?? currentPose.x), newPose.y - (transition?.to.y ?? currentPose.y)) > 0.00001
          || Math.abs(wrapAngle(newPose.yaw - (transition?.to.yaw ?? currentPose.yaw))) > 0.00001) {
          if (motionMayAnimate(next)) transition = { from: { ...currentPose }, to: newPose, start: now,
            duration: clamp(now - lastTargetAt, 80, 650) };
          else { currentPose = newPose; transition = null; }
          lastTargetAt = now;
        }
        if (!motionMayAnimate(next)) transition = null;
      }
      updateGeometry();
    };
    controllerRef.current = { accept };
    accept(sample);
    setModelStatus('loading');
    setCargoVisible(false);
    setWrapVisible(false);

    new GLTFLoader().load(modelUrl, (gltf) => {
      if (disposed) { disposeTree(gltf.scene); return; }
      model = gltf.scene;
      cargoShown = configureCargoPreview(model, showCargoPreview);
      setCargoVisible(cargoShown);
      const bounds = new THREE.Box3().setFromObject(model).getSize(new THREE.Vector3());
      if (bounds.x < 0.5 || bounds.x > 4 || bounds.y < 0.2 || bounds.y > 4) {
        disposeTree(model); model = null;
        setModelStatus('unit-error'); renderer.domElement.dataset.modelStatus = 'unit-error'; return;
      }
      modelLength = bounds.x;
      // HH_261001 - The model is already metres, +X forward/+Y up/+Z right. Never rescale.
      model.scale.set(1, 1, 1);
      applyModelSurfaceFinish(model, dark);
      robot.add(model);
      robot.updateMatrixWorld(true);
      // HH_261002 - Animate the existing independent steer/spin joints. Derive
      // lever arms from the actual GLB, never a hard-coded replacement vehicle.
      wheels = WHEEL_NAMES.map((name) => {
        const spin = model.getObjectByName(`wheel_${name}_spin`);
        const steer = model.getObjectByName(`wheel_${name}_steer`);
        if (!spin || !steer) return null;
        const pivot = robot.worldToLocal(steer.getWorldPosition(new THREE.Vector3()));
        return { name, spin, steer, spinRest: spin.quaternion.clone(), steerRest: steer.quaternion.clone(),
          position: { x: pivot.x, y: -pivot.z }, steerRad: 0, rollRad: 0, pivot };
      }).filter(Boolean);
      wheels.forEach(({ pivot: wheel }) => {
        const wheelContact = new THREE.Mesh(new THREE.PlaneGeometry(0.48, 0.43), shadowMaterial);
        wheelContact.rotation.x = -Math.PI / 2;
        wheelContact.position.set(wheel.x, 0.016, wheel.z);
        robot.add(wheelContact);
      });
      setModelStatus('ready'); renderer.domElement.dataset.modelStatus = 'ready';
      const wrapGroups = [
        { names: ['wrap_left', 'wrap_right'], url: wrapUrl },
        { names: ['wrap_front'], url: frontWrapUrl },
        { names: ['wrap_rear'], url: rearWrapUrl },
      ];
      wrapGroups.forEach(({ names, url }) => {
        const surfaces = names.map((name) => model.getObjectByName(name)).filter(Boolean);
        surfaces.forEach((surface) => { surface.visible = false; });
        if (!surfaces.length || !url) return;
        new THREE.TextureLoader().load(url, (texture) => {
          if (disposed) { texture.dispose(); return; }
          texture.colorSpace = THREE.SRGBColorSpace;
          texture.flipY = false;
          texture.anisotropy = Math.min(4, renderer.capabilities.getMaxAnisotropy());
          let assigned = false;
          surfaces.forEach((surface) => {
            surface.visible = true;
            surface.traverse((object) => {
              if (!object.isMesh) return;
              const old = object.material;
              object.material = new THREE.MeshStandardMaterial({ map: texture, color: 0xffffff,
                roughness: 0.7, metalness: 0.02, side: THREE.DoubleSide });
              if (Array.isArray(old)) old.forEach((material) => material.dispose()); else old?.dispose();
              assigned = true;
            });
            loadedWrapFaces.add(surface.name);
          });
          if (!assigned) texture.dispose();
          else setWrapVisible(true);
        }, undefined, () => { /* HH_261001 - Missing art leaves the actual white body visible. */ });
      });
    }, undefined, () => {
      if (!disposed) { setModelStatus('load-error'); renderer.domElement.dataset.modelStatus = 'load-error'; }
    });

    const wheelQuaternion = new THREE.Quaternion();
    const wheelAxis = new THREE.Vector3(0, 0, 1);
    const steerAxis = new THREE.Vector3(0, 1, 0);
    const projectedObject = new THREE.Vector3();
    // HH_261001 - The render loop consumes cached, already-normalized telemetry only. It
    // does not poll ROS, change a mission, or extrapolate stale vehicle pose.
    const tick = (now) => {
      if (disposed) return;
      animationId = requestAnimationFrame(tick);
      if (now - lastFrame < 1000 / 60 || document.hidden) return;
      const delta = lastFrame ? clamp((now - lastFrame) / 1000, 0, 0.1) : 1 / 60;
      lastFrame = now;
      const fresh = Boolean(poseIsFresh(sample, now - receivedAt));
      const moving = Boolean(motionMayAnimate(sample, now - receivedAt));
      const previous = currentPose ? { ...currentPose } : null;
      if (transition && moving) {
        const fraction = clamp((now - transition.start) / transition.duration, 0, 1);
        currentPose = interpolatePose(transition.from, transition.to, fraction);
        if (fraction >= 1) transition = null;
      } else if (!moving) transition = null;
      // HH_261002 - Body yaw and lateral movement are independent of forward
      // speed: zero-turn rolls opposite wheels, crab steers all four sideways.
      const step = moving ? bodyPoseDelta(previous, currentPose) : null;
      motionKind = navigationMotionKind(step);
      wheels.forEach((wheel) => {
        const estimate = estimateWheelStep(step, wheel.position, wheel.steerRad);
        wheel.steerRad = estimate.steer;
        wheel.rollRad = advanceWheelRoll(wheel.rollRad, estimate.distance);
        wheelQuaternion.setFromAxisAngle(steerAxis, wheel.steerRad);
        wheel.steer.quaternion.copy(wheel.steerRest).multiply(wheelQuaternion);
        wheelQuaternion.setFromAxisAngle(wheelAxis, wheel.rollRad);
        wheel.spin.quaternion.copy(wheel.spinRest).multiply(wheelQuaternion);
      });
      const displayPose = currentPose || { x: origin.x, y: origin.y, yaw: 0 };
      const position = mapToThree([displayPose.x, displayPose.y, 0], origin);
      robot.position.set(position.x, 0, position.z);
      robot.rotation.y = displayPose.yaw;
      if (fresh && moving) cameraHeading = navigationCameraHeading(cameraHeading, displayPose.yaw, delta, motionKind);
      detailBlend += ((detailRef.current ? 1 : 0) - detailBlend) * Math.min(1, delta * 7);
      orbitAngle += (detailAngleRef.current - orbitAngle) * Math.min(1, delta * 7);
      // HH_261001 - Smoothly mix the navigation-follow camera with the opt-in model-detail
      // camera without moving the actual GLB or altering its metre scale.
      const orbit = cameraHeading + orbitAngle * detailBlend;
      const rear = 4.35 - detailBlend * 2.25;
      const side = navigationCameraSideOffset(detailBlend);
      const height = 2.98 - detailBlend * 0.75;
      const lookahead = 3.05 * (1 - detailBlend);
      camera.position.set(position.x - Math.cos(orbit) * rear + Math.sin(orbit) * side,
        height, position.z + Math.sin(orbit) * rear + Math.cos(orbit) * side);
      camera.lookAt(position.x + Math.cos(cameraHeading) * lookahead,
        0.53 + detailBlend * 0.1, position.z - Math.sin(cameraHeading) * lookahead);
      floor.position.x = Math.round(position.x / 10) * 10;
      floor.position.z = Math.round(position.z / 10) * 10;
      route.visible = fresh && route.geometry.attributes.position.count > 0;
      routeHalo.visible = route.visible;
      points.visible = fresh && points.geometry.attributes.position.count > 0;
      detections.visible = fresh && sample.perceptionReady && objectVisuals.size > 0;
      renderer.render(scene, camera);
      renderedFrames += 1;
      let visibleObjectLabels = 0;
      // HH_261002 - Label only on-screen nearby areas; keep all authored areas
      // in the overview map. Static labels do not imply fresh robot localization.
      let visibleAreaLabels = 0;
      const placedLabels = [];
      areaLabelsRef.current.forEach((element, id) => {
        const area = sample.baseMap?.areas?.find(item => item.id === id);
        const anchor = areaLabelPoint(area);
        if (!anchor || !areaWorld || Math.hypot(anchor[0] - displayPose.x, anchor[1] - displayPose.y) > 55) {
          element.style.visibility = 'hidden'; return;
        }
        const point = mapToThree(anchor, origin, 0.08);
        projectedObject.set(point.x, point.y, point.z).project(camera);
        if (projectedObject.z < -1 || projectedObject.z > 1 || Math.abs(projectedObject.x) > 0.92
          || Math.abs(projectedObject.y) > 0.86) { element.style.visibility = 'hidden'; return; }
        const x = (projectedObject.x + 1) * host.clientWidth / 2;
        const y = (1 - projectedObject.y) * host.clientHeight / 2;
        if (placedLabels.some(other => Math.abs(other.x - x) < 70 && Math.abs(other.y - y) < 28)) {
          element.style.visibility = 'hidden'; return;
        }
        placedLabels.push({ x, y });
        element.style.transform = `translate(${x.toFixed(1)}px, ${y.toFixed(1)}px) translate(-50%, -50%)`;
        element.style.visibility = 'visible';
        visibleAreaLabels += 1;
      });
      objectLabelsRef.current.forEach((element, index) => {
        const object = sample.objects[index];
        if (!fresh || !sample.perceptionReady || !object) { element.style.visibility = 'hidden'; return; }
        const dx = object.x - displayPose.x, dy = object.y - displayPose.y;
        const forward = dx * Math.cos(displayPose.yaw) + dy * Math.sin(displayPose.yaw);
        if (forward < 0.1 || forward > 40) { element.style.visibility = 'hidden'; return; }
        const visual = objectVisuals.get(`${object.id}:${object.class_name}`);
        const size = navigationObjectDimensions(object);
        if (visual && size) {
          // HH_261002 - Attach the label above the oriented observed box, not its old centroid.
          labelAnchor.copy(visual.position);
          labelAnchor.y += visual.userData.labelHeight || 0;
          projectedObject.copy(labelAnchor).project(camera);
        } else {
          const position3d = mapToThree([object.x, object.y, object.z], origin, 0.12);
          projectedObject.set(position3d.x, position3d.y, position3d.z).project(camera);
        }
        const labelPosition = projectVisibleObjectLabel(projectedObject, host.clientWidth, host.clientHeight);
        if (!labelPosition) { element.style.visibility = 'hidden'; return; }
        element.dataset.edge = labelPosition.edge;
        element.style.transform = `translate(${labelPosition.x.toFixed(1)}px, ${(labelPosition.y - 9).toFixed(1)}px) translate(-50%, -100%)`;
        element.style.visibility = 'visible';
        const distance = element.querySelector('[data-object-distance]');
        if (distance) distance.textContent = `${Math.hypot(dx, dy).toFixed(1)} m`;
        visibleObjectLabels += 1;
      });
      if (now - lastDebug > 100) {
        renderer.domElement.dataset.navigationState = JSON.stringify({
          renderedFrames, motionValid: moving, poseFresh: fresh, modelPositionMeters: { x: displayPose.x, y: displayPose.y },
          modelYawRad: displayPose.yaw, cameraYawRad: cameraHeading, wheelRollRad: wheels[0]?.rollRad || 0,
          motionKind, wheels: wheels.map(({ name, steerRad, rollRad }) => ({ name, steerRad, rollRad })),
          cameraSideOffsetMeters: side,
          robotProjectedXNdc: projectedObject.set(position.x, 0.53, position.z).project(camera).x,
          wheelAngleSource: 'estimated-from-received-body-pose', steeringAngleSource: 'estimated-from-received-body-pose',
          wheelNodes: wheels.length, metricRobotLength: modelLength, modelScale: robot.scale.toArray(),
          routeVertices: route.geometry.attributes.position.count, pointCount: points.geometry.attributes.position.count,
          // HH_261002 - A map may not have arrived yet (including UI fixtures).
          // Missing optional geometry must not throw in every render frame.
          baseMapVertices: baseMapLines.geometry.attributes.position?.count || 0,
          baseMapVisible: baseMapLines.visible, baseMapSource: sample.baseMap?.source || null,
          serviceAreaCount: areaWorld?.userData.areaCount || 0, visibleAreaLabels,
          serviceAreaIds: (sample.baseMap?.areas || []).map(area => area.id),
          baseMapBoundaryColor: dark ? '#4ade80' : '#1ea65a',
          baseMapBoundaryWidthMeters: LANE_BOUNDARY_WIDTH_M,
          baseMapBoundaryPaintVertices: boundaryPaint.geometry.attributes.position?.count || 0,
          renderedTriangles: renderer.info.render.triangles,
          cameraMode: detailRef.current ? 'model-detail' : 'navigation-follow',
          detailAngleRad: orbitAngle, detailAngleIndex: Math.round(detailAngleRef.current / (Math.PI / 2)) % 4,
          demoEnvironmentVisible: Boolean(routeWorld && demo),
          illustrativeEnvironmentVisible: Boolean(routeWorld),
          illustrativeEnvironmentSource: routeWorld ? `${routeWorld.userData.source}-aligned-art-not-sensor-or-physical-geometry` : null,
          illustratedRoadCount: routeWorld?.userData.roadCount || 0,
          illustrativeShrubs: routeWorld?.userData.shrubs || 0,
          illustrativeTrees: routeWorld?.userData.trees || 0,
          cargoPreviewVisible: cargoShown, shelfPreviewVisible: false, wrapFacesLoaded: Array.from(loadedWrapFaces),
          visibleObjectLabels,
          observedObjectBoxes: [...objectVisuals.values()].filter(v => v.userData.sizeMode === 'observed').length,
          objectVisualCount: objectVisuals.size,
        });
        lastDebug = now;
      }
    };
    animationId = requestAnimationFrame(tick);
    return () => {
      disposed = true;
      cancelAnimationFrame(animationId);
      observer?.disconnect();
      window.removeEventListener('resize', resize);
      controllerRef.current = null;
      disposeTree(scene);
      renderer.dispose();
      renderer.forceContextLoss();
      renderer.domElement.remove();
    };
  }, [modelUrl, wrapUrl, frontWrapUrl, rearWrapUrl, theme, showCargoPreview, demo]);

  useEffect(() => { controllerRef.current?.accept(data); }, [data]);

  const hasRoute = data.connected && data.pose && data.route.length > 1;
  const loadingText = ({ loading: '로봇 외관 불러오는 중', 'load-error': '로봇 외관을 불러오지 못했습니다',
    'unit-error': 'Ranger 모델의 미터 단위 확인이 필요합니다', 'webgl-unavailable': '이 환경에서 3D 렌더링을 사용할 수 없습니다' })[modelStatus];
  return <div className="ranger-navigation-scene" data-testid="ranger-navigation-scene" data-model-status={modelStatus}
    data-demo={demo ? 'true' : 'false'}
    data-cargo-preview={cargoVisible ? 'true' : 'false'} data-wrap-visible={wrapVisible ? 'true' : 'false'}>
    <div ref={hostRef} className="ranger-navigation-surface" />
    <div className="ranger-navigation-area-labels" aria-label="캠핑사이트 및 출발·복귀 구역">
      {(data.baseMap?.areas || []).map(area => <div key={area.id} className="ranger-navigation-area-label"
        data-kind={area.kind} data-destination={areaIsDestination(area, data.mission) ? 'true' : 'false'}
        ref={element => { if (element) areaLabelsRef.current.set(area.id, element); else areaLabelsRef.current.delete(area.id); }}>
        {area.label}{areaIsDestination(area, data.mission) && <small>목적지</small>}
      </div>)}
    </div>
    <div className="ranger-navigation-object-labels" aria-label="수신된 전방 객체">
      {data.objects.slice(0, 12).map((object, index) => <div className="ranger-navigation-object-label"
        data-size-mode={navigationObjectDimensions(object) ? 'observed' : 'symbolic'}
        key={`${object.id}-${index}`} ref={(element) => { if (element) objectLabelsRef.current.set(index, element); else objectLabelsRef.current.delete(index); }}>
        <NavigationObjectIcon className={object.class_name} />
        <div><div className="ranger-navigation-object-title"><strong>{object.class_name || '감지 객체'}</strong><span data-object-distance /></div>
          <small>{objectGeometryCaption(object, demo)}</small></div>
      </div>)}
    </div>
    <button type="button" className="ranger-navigation-detail" aria-pressed={detailView}
      data-testid="ranger-model-detail-toggle" data-navigation="zoom"
      onPointerDown={(event) => event.stopPropagation()} onPointerUp={(event) => event.stopPropagation()}
      onPointerMove={(event) => event.stopPropagation()} onPointerCancel={(event) => event.stopPropagation()}
      onKeyDown={(event) => { if (['Enter', ' '].includes(event.key)) event.stopPropagation(); }}
      onClick={(event) => { event.preventDefault(); event.stopPropagation(); detailRef.current = !detailRef.current; setDetailView(detailRef.current); }}>
      {detailView ? '주행 시점' : '외관 보기'}
    </button>
    {detailView && <button type="button" className="ranger-navigation-detail ranger-navigation-angle"
      data-navigation="angle" data-view-index={detailAngleIndex}
      aria-label={`방향 보기, 현재 ${['후면·우측', '전면·우측', '전면·좌측', '후면·좌측'][detailAngleIndex]}`}
      onPointerDown={(event) => event.stopPropagation()} onPointerUp={(event) => event.stopPropagation()}
      onPointerMove={(event) => event.stopPropagation()} onPointerCancel={(event) => event.stopPropagation()}
      onKeyDown={(event) => { if (['Enter', ' '].includes(event.key)) event.stopPropagation(); }}
      onClick={(event) => { event.preventDefault(); event.stopPropagation(); detailAngleRef.current += Math.PI / 2;
        setDetailAngleIndex(Math.round(detailAngleRef.current / (Math.PI / 2)) % 4); }}>
      방향 보기 ↻ <span>{['후면·우측', '전면·우측', '전면·좌측', '후면·좌측'][detailAngleIndex]}</span>
    </button>}
    <div className={`ranger-navigation-route-status ${hasRoute ? '' : 'is-waiting'}`}><i />
      {hasRoute ? `${demo ? '시연' : '주행'} 경로` : data.routeReason || '위치·경로 수신 대기'}
    </div>
    {modelStatus !== 'ready' && <div className="ranger-navigation-loading" role="status"><span className="ranger-navigation-loading-icon">3D</span><strong>{loadingText}</strong><span>정적 이미지로 대체하지 않습니다</span></div>}
    {!data.perceptionReady && <span className="ranger-navigation-perception-wait">전방 인지 수신 대기</span>}
    <div className="ranger-navigation-legend"><span>{demo ? '시연 경로 · 예시 배경'
      : `${data.baseMap?.valid ? '수신 Lanelet 지도 · ' : ''}${hasRoute ? '실제 경로' : '경로 대기'} · ${data.baseMap?.valid ? '도로 폭·주변은 예시 (실제 지형 아님)' : '도로변 예시 배경 (실제 지형 아님)'}`}
      {/* HH_261002 - Do not present inferred wheel poses as measured joint telemetry. */}
      {detailView && <><br />바퀴 조향·회전: 차체 이동 기반 추정</>}
    </span></div>
  </div>;
}
