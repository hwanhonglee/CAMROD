import React, { useEffect, useId, useMemo, useRef, useState } from 'react';
import './DrivingDisplay.css';
import RangerNavigationScene from './RangerNavigationScene';
import { RANGER_MODEL_URL } from './rangerModelAsset';
import { areaIsDestination, areaLabelPoint, layoutAreaLabels, normalizeNavigationAreas } from './navigationAreas';

// HH_261001 - Telemetry and geometry are read-only. The optional operator stop action calls
// the existing App handler only after an explicit confirmation.
export const DRIVING_MAX_AGE_S = 3;
const LIMITS = Object.freeze({ route: 256, points: 600, objects: 32, mapLines: 512, mapPoints: 3000 });
const finite = (value) => typeof value === 'number' && Number.isFinite(value);
const textValue = (value, limit = 100) => typeof value === 'string' ? value.slice(0, limit) : '';
const percent = (value) => finite(value) && value >= 0 && value <= 100 ? value : null;
const nonnegative = (value) => finite(value) && value >= 0 ? value : null;
const isFresh = (value, elapsed, maximum = DRIVING_MAX_AGE_S) => finite(value?.age_s)
  && value.age_s >= 0 && value.age_s + elapsed <= maximum;
const scalarFresh = (value, elapsed) => value && (value.age_s === undefined
  ? elapsed <= DRIVING_MAX_AGE_S : isFresh(value, elapsed));
const validPoint = (value) => Array.isArray(value) && finite(value[0])
  && finite(value[1]) && Math.abs(value[0]) <= 1e8 && Math.abs(value[1]) <= 1e8;

// HH_261002 - Only the dedicated observed-extent contract supplies metric boxes.
// Legacy Detection3D/marker glyph sizes and 2D pixel boxes are not physical sizes.
export function normalizeObjectGeometry(object) {
  const unavailable = { dimensions: null, orientation: null, bbox: null, geometry_source: null };
  if (object?.geometry_source !== 'observed_lidar_extent') return unavailable;
  const dimensions = object.dimensions, center = object.bbox?.center, q = object.orientation;
  if (!['x', 'y', 'z'].every(axis => finite(dimensions?.[axis])
      && dimensions[axis] > 0 && dimensions[axis] <= 30
      && finite(center?.[axis]) && Math.abs(center[axis]) <= 1e8)
    || !['x', 'y', 'z', 'w'].every(axis => finite(q?.[axis]))) return unavailable;
  const norm = Math.hypot(q.x, q.y, q.z, q.w);
  if (norm < 1e-8) return unavailable;
  return { dimensions: { x: dimensions.x, y: dimensions.y, z: dimensions.z },
    bbox: { center: { x: center.x, y: center.y, z: center.z } },
    orientation: { x: q.x / norm, y: q.y / norm, z: q.z / norm, w: q.w / norm },
    geometry_source: 'observed_lidar_extent' };
}

function sampleBounded(values, maximum, validate) {
  if (!Array.isArray(values) || !values.length) return [];
  const count = Math.min(maximum, values.length);
  const result = [];
  for (let index = 0; index < count; index += 1) {
    const value = values[count === 1 ? 0 : Math.floor(index * (values.length - 1) / (count - 1))];
    if (validate(value)) result.push(value);
  }
  return result;
}

/** HH_261002 - Static /map/markers road geometry is independent of the active mission.
 * Its receipt age is informational: a latched map must not vanish with the route. */
export function normalizeBaseMap(source, pose = null) {
  if (source?.valid !== true || source.source !== '/map/markers'
    || source.frame_id !== 'map' || (pose && pose.frame_id !== source.frame_id)
    || !Array.isArray(source.polylines)) return { valid: false, frame_id: '', polylines: [], areas: [], source: null };
  const polylines = [];
  let remaining = LIMITS.mapPoints;
  for (const line of source.polylines.slice(0, LIMITS.mapLines)) {
    if (remaining < 2) break;
    const points = sampleBounded(line?.points, Math.min(remaining, 300), validPoint);
    if (points.length < 2) continue;
    // HH_261002 - Keep the producer identity for checked left/right pairing.
    // Missing IDs still render as lines; array order is never a pairing contract.
    polylines.push({ namespace: textValue(line.namespace, 64), points,
      ...(Number.isSafeInteger(line.marker_id) ? { marker_id: line.marker_id } : {}) });
    remaining -= points.length;
  }
  // HH_261002 - Authored map catalog polygons are an independent static layer:
  // area-only maps remain visible without a route or road marker receipt.
  const areas = normalizeNavigationAreas(source.areas);
  return { valid: polylines.length > 0 || areas.length > 0, frame_id: source.frame_id,
    polylines, areas, source: source.source };
}

function routeUnavailableReason(connected, pose, routeSource, route, mission) {
  if (!connected) return '실시간 데이터 수신 대기';
  if (!pose) return '위치 정보 수신 대기';
  if (routeSource?.frame_id && routeSource.frame_id !== pose.frame_id) return '경로 좌표계 확인 중';
  // HH_261001 - The planner publishes a transient-local path once per route. Its
  // receipt age is not its validity: only the backend's mission/leg-bound route
  // authority may keep that path visible or invalidate it on a new mission.
  if (routeSource?.valid !== true) {
    // HH_261002 - Local departure/entry/parking maneuvers deliberately do
    // not expose a global path.  Describe the maneuver, not a broken link.
    const maneuver = {
      DEPARTING_CHARGER: '충전 위치에서 출발 중 · 경로 안내 준비',
      DEPARTING_DROP_ZONE: '출발지에서 출발 중 · 경로 안내 준비',
      SITE_ENTRY: '사이트 진입 동작 중 · 전역 경로 일시 비표시',
      RETURN_WITH_CARGO: '사이트 이탈 동작 중 · 복귀 경로 준비',
      DROP_ZONE_PARKING: '후진 주차 동작 중 · 전역 경로 일시 비표시',
    };
    return maneuver[mission?.service_state_name] || '현재 미션 경로 수신 대기';
  }
  return route.length < 2 ? '경로 수신 대기' : '';
}

/** HH_261001 - Rotate world coordinates by -yaw; ROS forward is +x, left is +y. */
export function toHeadingUp(point, pose) {
  const dx = point[0] - pose.x;
  const dy = point[1] - pose.y;
  const cos = Math.cos(pose.yaw);
  const sin = Math.sin(pose.yaw);
  return { forward: dx * cos + dy * sin, left: -dx * sin + dy * cos };
}

/** HH_261001 - The only ingress for telemetry. Unavailable is represented by null/empty. */
export function normalizeDrivingSnapshot(snapshot, elapsedSeconds = 0) {
  const elapsed = finite(elapsedSeconds) ? Math.max(0, elapsedSeconds) : Infinity;
  const source = snapshot?.schema_version === 1 ? snapshot : {};
  const connected = source.connected === true && elapsed <= DRIVING_MAX_AGE_S;
  const poseData = source.pose;
  const pose = connected && isFresh(poseData, elapsed) && finite(poseData.x)
    && finite(poseData.y) && finite(poseData.yaw)
    && Math.abs(poseData.x) <= 1e8 && Math.abs(poseData.y) <= 1e8
    && textValue(poseData.frame_id).trim()
    ? { x: poseData.x, y: poseData.y, yaw: poseData.yaw, frame_id: poseData.frame_id, age_s: poseData.age_s + elapsed } : null;
  const routeReady = Boolean(connected && pose && source.route?.valid === true
    && typeof source.route.frame_id === 'string' && source.route.frame_id === pose.frame_id);
  const perceptionFrameReady = Boolean(connected && pose && source.perception?.frame_id === pose.frame_id);
  // HH_261001 - The cloud and classified objects have separate source timestamps. A fresh
  // fused object must never make an older LiDAR cloud appear current.
  const pointsAge = source.perception?.points_age_s === undefined ? source.perception?.age_s : source.perception.points_age_s;
  const objectsAge = source.perception?.objects_age_s === undefined ? source.perception?.age_s : source.perception.objects_age_s;
  const pointsReady = perceptionFrameReady && isFresh({ age_s: pointsAge }, elapsed);
  const objectsReady = perceptionFrameReady && isFresh({ age_s: objectsAge }, elapsed);
  const route = routeReady ? sampleBounded(source.route.points, LIMITS.route, validPoint) : [];
  const baseMap = normalizeBaseMap(source.base_map, pose);
  const points = pointsReady ? sampleBounded(source.perception.points, LIMITS.points,
    (point) => validPoint(point) && (point[2] == null || (finite(point[2]) && Math.abs(point[2]) <= 1000))) : [];
  const objects = perceptionFrameReady ? sampleBounded(source.perception.objects, LIMITS.objects,
    (object) => object && validPoint([object.x, object.y])
      && isFresh({ age_s: object.age_s === undefined ? objectsAge : object.age_s }, elapsed)
      && (object.z == null || (finite(object.z) && Math.abs(object.z) <= 1000))).map((object, index) => ({
    id: textValue(String(object.id ?? index), 40),
    class_name: textValue(object.class_name, 30) || '감지 객체',
    x: object.x, y: object.y, z: object.z,
    confidence: finite(object.confidence) && object.confidence >= 0
      && object.confidence <= 1 ? object.confidence : null,
    ...normalizeObjectGeometry(object),
  })) : [];
  const perceptionReady = pointsReady || objectsReady || objects.length > 0;
  const speed = connected && scalarFresh(source.motion, elapsed)
    && finite(source.motion.speed_mps) ? Math.abs(source.motion.speed_mps) : null;
  const signedSpeed = connected && scalarFresh(source.motion, elapsed)
    ? (finite(source.motion.vx_mps) ? source.motion.vx_mps : finite(source.motion.speed_mps) ? source.motion.speed_mps : null) : null;
  const battery = connected && scalarFresh(source.battery, elapsed)
    ? percent(source.battery.percentage) : null;
  const progressReady = connected && source.progress?.valid === true
    && scalarFresh(source.progress, elapsed);
  const sensors = {};
  ['gnss', 'lidar', 'camera', 'radar'].forEach((name) => {
    const sensor = source.sensors?.[name];
    sensors[name] = connected && isFresh(sensor, elapsed) ? textValue(sensor.label, 28) || null : null;
  });
  const routeReason = routeUnavailableReason(connected, pose, source.route, route, source.mission);
  return {
    connected, pose, route, baseMap, points, objects, sensors, speed, signedSpeed,
    speedKmh: speed === null ? null : speed * 3.6,
    battery, routeReason, perceptionReady,
    remainingDistance: progressReady ? nonnegative(source.progress.remaining_distance_m) : null,
    remainingTime: progressReady ? nonnegative(source.progress.remaining_time_s) : null,
    completion: progressReady ? percent(source.progress.completion_pct) : null,
    progressReason: textValue(source.progress?.reason, 100),
    mission: {
      active: connected && source.mission?.active === true,
      intent: ['delivery', 'recall'].includes(source.mission?.intent) ? source.mission.intent : null,
      site: textValue(source.mission?.site, 50),
      state: textValue(source.mission?.service_state_name, 64),
      phase: textValue(source.mission?.phase, 64),
      description: textValue(source.mission?.description, 180),
    },
  };
}

export function formatDistance(value) {
  if (!finite(value) || value < 0) return { value: '—', unit: 'm' };
  return value >= 1000 ? { value: (value / 1000).toFixed(1), unit: 'km' }
    : { value: String(Math.round(value)), unit: 'm' };
}

export function formatDuration(value) {
  if (!finite(value) || value < 0) return { value: '—', unit: '분' };
  if (value < 60) return { value: String(Math.ceil(value)), unit: '초' };
  return { value: String(Math.ceil(value / 60)), unit: '분' };
}

/** HH_261001 - Derive a turn cue only from the received route in the same map frame. */
export function nextRouteInstruction(data) {
  const route = data?.route;
  const pose = data?.pose;
  if (!pose || !Array.isArray(route) || route.length < 2) {
    return { title: '경로 안내 대기', distance: null, detail: data?.routeReason || '최신 경로 수신 대기' };
  }
  // HH_261001 - Project the received pose onto the nearest route segment before measuring
  // distance along the route. No turn is inferred from an arbitrary map image.
  let nearest = { distanceSquared: Infinity, index: 0, fraction: 0 };
  const lengths = [];
  for (let index = 0; index < route.length - 1; index += 1) {
    const dx = route[index + 1][0] - route[index][0];
    const dy = route[index + 1][1] - route[index][1];
    const lengthSquared = dx * dx + dy * dy;
    lengths.push(Math.sqrt(lengthSquared));
    if (lengthSquared < 0.0001) continue;
    const fraction = Math.max(0, Math.min(1,
      ((pose.x - route[index][0]) * dx + (pose.y - route[index][1]) * dy) / lengthSquared));
    const x = route[index][0] + fraction * dx;
    const y = route[index][1] + fraction * dy;
    const distanceSquared = (pose.x - x) ** 2 + (pose.y - y) ** 2;
    if (distanceSquared < nearest.distanceSquared) nearest = { distanceSquared, index, fraction };
  }
  // HH_261001 - More than 10 m away is not a trustworthy route-relative cue.
  if (nearest.distanceSquared > 100) {
    return { title: '경로 위치 확인 중', distance: null, detail: '현재 위치와 수신 경로를 대조합니다' };
  }
  const headingAt = (index) => {
    let end = index + 1;
    let distance = 0;
    while (end < route.length - 1 && distance < 1.8) {
      distance += lengths[end - 1] || 0;
      end += 1;
    }
    return Math.atan2(route[end][1] - route[index][1], route[end][0] - route[index][0]);
  };
  const baseHeading = headingAt(nearest.index);
  let ahead = lengths[nearest.index] * (1 - nearest.fraction);
  for (let index = nearest.index + 1; index < route.length - 1 && ahead <= 60; index += 1) {
    if (ahead >= 2) {
      const angle = Math.atan2(Math.sin(headingAt(index) - baseHeading), Math.cos(headingAt(index) - baseHeading));
      // HH_261001 - 0.58 rad (~33 deg) avoids calling every shallow bend a turn.
      if (Math.abs(angle) >= 0.58) {
        return { title: angle > 0 ? '좌측 경로로 진행' : '우측 경로로 진행',
          distance: ahead, detail: '수신한 경로 형상 기준' };
      }
    }
    ahead += lengths[index];
  }
  return { title: '경로를 따라 진행', distance: null, detail: '전방 수신 경로 기준' };
}

const STATE_LABELS = {
  DROP_ZONE_WAIT: '출발지 대기', MOVING_TO_SITE: '목적지로 이동 중',
  SITE_ARRIVED: '목적지 도착', RETURNING_TO_DROP_ZONE: '출발지로 복귀 중',
  GUEST_RECALL_SERVICE: '호출 서비스 진행 중', SITE_ENTRY: '사이트 진입 중',
  UNLOAD_WAIT: '짐 내리기 대기', RECALL_TO_SITE_ROAD: '호출 위치로 이동 중',
  GUEST_LOADING_WAIT: '짐 싣기 대기', RETURN_WITH_CARGO: '짐을 싣고 복귀 중',
  DROP_ZONE_PARKING: '주차 진행 중', WAITING_FOR_RETURN_REQUEST: '복귀 요청 대기',
  WAITING_FOR_CHARGING: '충전 대기', CHARGING: '충전 중',
  DEPARTING_CHARGER: '충전 위치에서 출발 중', DEPARTING_DROP_ZONE: '출발지에서 출발 중',
  OPERATOR_STOPPED: '운행 정지',
};
const PHASE_LABELS = {
  INITIALIZING: '시스템 준비 중', READY: '주행 준비 완료', GOAL_RECEIVED: '목적지 접수',
  PATH_PREPARING: '경로 준비 중', DRIVING: '주행 중', SAFETY_STOP: '안전 정지',
  ARRIVED: '도착', STOPPED: '운행 정지',
};

export function missionStatus(data) {
  if (!data.connected) return '실시간 정보 대기';
  if (['SAFETY_STOP', 'STOPPED'].includes(data.mission.phase)) return PHASE_LABELS[data.mission.phase];
  return STATE_LABELS[data.mission.state] || PHASE_LABELS[data.mission.phase] || '임무 상태 확인 중';
}

/** HH_261001 - Keep controller debug strings in diagnostics, not the passenger view. */
export function missionDescription(data) {
  if (!data.connected) return '연결 상태와 최신 데이터를 기다리고 있습니다.';
  if (data.mission.phase === 'SAFETY_STOP') return '안전 상태를 확인 중입니다. 서비스 화면에서 상세 진단을 확인하세요.';
  const site = data.mission.site || '목적지';
  const descriptions = {
    DEPARTING_CHARGER: '충전 위치에서 안전하게 출발하고 있습니다.',
    DEPARTING_DROP_ZONE: '출발지에서 안전하게 출발하고 있습니다.',
    MOVING_TO_SITE: `${site} 사이트로 이동하고 있습니다.`,
    RECALL_TO_SITE_ROAD: `${site} 호출 위치로 이동하고 있습니다.`,
    SITE_ENTRY: `${site} 사이트에 진입하고 있습니다.`,
    RETURN_WITH_CARGO: '짐을 싣고 출발지로 복귀하고 있습니다.',
    RETURNING_TO_DROP_ZONE: '출발지로 복귀하고 있습니다.',
    DROP_ZONE_PARKING: '출발지에 후진 주차하고 있습니다.',
  };
  return descriptions[data.mission.state] || '로봇의 현재 임무 상태를 표시합니다.';
}

function Icon({ name, size = 20, ...props }) {
  const paths = {
    arrow: <><path d="M12 20V4m-6 6 6-6 6 6" /><path d="M5 20h14" opacity=".35" /></>,
    pin: <><path d="M19 10c0 5-7 11-7 11S5 15 5 10a7 7 0 1 1 14 0Z" /><circle cx="12" cy="10" r="2.5" /></>,
    satellite: <><path d="m9 9 6 6M5 5l4-3 4 4-4 4ZM14 14l4-4 4 4-3 4ZM4 14a6 6 0 0 1 6 6M3 18a2 2 0 0 1 2 2M12 12l-4 4" /></>,
    battery: <><rect x="2" y="6" width="17" height="12" rx="3" /><path d="M22 10v4M6 10v4m4-4v4m4-4v4" /></>,
    scan: <><path d="M8 3H4a1 1 0 0 0-1 1v4m13-5h4a1 1 0 0 1 1 1v4M3 16v4a1 1 0 0 0 1 1h4m8 0h4a1 1 0 0 0 1-1v-4" /><circle cx="12" cy="12" r="4" /><path d="M12 8v4l3 2" /></>,
    clock: <><circle cx="12" cy="12" r="9" /><path d="M12 7v5l3 2" /></>,
    touch: <><path d="M9 12V5a2 2 0 0 1 4 0v8-4a2 2 0 0 1 4 0v4-2a2 2 0 0 1 4 0v5c0 4-3 6-7 6h-1c-2 0-4-1-5-3l-4-6a2 2 0 0 1 3-2l2 2" /></>,
    chevron: <path d="m9 5 7 7-7 7" />,
  };
  return <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor"
    strokeWidth="1.7" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true" {...props}>{paths[name] || paths.scan}</svg>;
}

function DrivingScene({ data, demo, theme, robotModelUrl }) {
  return <div className="dd-scene" data-testid="driving-scene"><RangerNavigationScene
    data={data} demo={demo} theme={theme} modelUrl={robotModelUrl} /></div>;
}

function RouteMap({ data, id }) {
  const ready = Boolean(data.pose && data.route.length > 1);
  const mapReady = Boolean(data.baseMap?.valid
    && (data.baseMap.polylines.length || data.baseMap.areas.length));
  if (!ready && !mapReady) return <div className="dd-map dd-map--empty" data-testid="driving-map"><Icon name="pin" size={27} /><span>지도 수신 대기</span><small>지도 또는 경로가 수신되면 표시됩니다</small></div>;
  // HH_261002 - The static, measured lanelet basemap is shown without a mission.
  // A route is a separate overlay and appears only when its own authority is valid.
  const xy = (point) => {
    if (!data.pose) return { x: point[0], y: -point[1] };
    const relative = toHeadingUp(point, data.pose);
    return { x: -relative.left, y: -relative.forward };
  };
  const routePoints = ready ? data.route.map(xy) : [];
  const baseLines = mapReady ? data.baseMap.polylines.map((line) => ({ namespace: line.namespace,
    points: line.points.map(xy) })) : [];
  const areas = mapReady ? data.baseMap.areas.map((area) => ({ ...area,
    points: area.points.map(xy), center: xy(areaLabelPoint(area)),
    selected: areaIsDestination(area, data.mission) })) : [];
  // HH_261002 - Fit the real site/drop-zone footprints with the received road
  // map, so idle mode keeps every configured area visible before a path exists.
  const fit = baseLines.flatMap((line) => line.points)
    .concat(areas.flatMap((area) => area.points), routePoints,
      data.pose ? [{ x: 0, y: 0 }] : []);
  const xs = fit.map((point) => point.x), ys = fit.map((point) => point.y);
  const minX = Math.min(...xs), maxX = Math.max(...xs);
  const minY = Math.min(...ys), maxY = Math.max(...ys);
  const scale = Math.min(226 / Math.max(maxX - minX, 8), 126 / Math.max(maxY - minY, 8));
  const px = (x) => 146 + (x - (minX + maxX) / 2) * scale;
  const py = (y) => 88 + (y - (minY + maxY) / 2) * scale;
  const path = (points) => points.map((point, index) => `${index ? 'L' : 'M'}${px(point.x).toFixed(1)},${py(point.y).toFixed(1)}`).join(' ');
  const end = routePoints[routePoints.length - 1];
  const labels = layoutAreaLabels(areas.map((area) => ({ id: area.id,
    anchor: { x: px(area.center.x), y: py(area.center.y) } })));
  const areaById = new Map(areas.map((area) => [area.id, area]));
  return <div className="dd-map" data-testid="driving-map" data-map-source={mapReady ? data.baseMap.source : ''}>
    <svg viewBox="0 0 292 176" role="img" aria-label={mapReady ? '수신된 Lanelet 지도와 유효한 경로의 축소 지도' : '현재 위치와 수신 경로의 축소 지도'}>
      <defs><pattern id={`${id}-map-grid`} width="29" height="29" patternUnits="userSpaceOnUse"><path d="M29 0H0v29" fill="none" className="dd-map-grid" /></pattern></defs>
      <rect width="292" height="176" fill={`url(#${id}-map-grid)`} />
      {areas.map((area) => <path key={`area-${area.id}`} d={`${path(area.points)} Z`}
        data-testid="driving-map-area" data-area-id={area.id} data-kind={area.kind}
        data-site={area.site || ''} data-source={area.source}
        className={`dd-map-area dd-map-area--${area.kind}${area.selected ? ' dd-map-area--selected' : ''}`} />)}
      {baseLines.map((line, index) => <path key={`${line.namespace}-${index}`} d={path(line.points)}
        data-testid="driving-map-base-line" data-namespace={line.namespace} className="dd-map-base-line" fill="none" />)}
      {ready && <><path d={path(routePoints)} fill="none" strokeWidth="10" className="dd-path-halo" strokeLinejoin="round" />
        <path d={path(routePoints)} fill="none" strokeWidth="3.4" className="dd-path-core" strokeLinejoin="round" strokeLinecap="round" />
        <circle cx={px(end.x)} cy={py(end.y)} r="6" className="dd-map-end" /></>}
      {labels.map((label) => {
        const area = areaById.get(label.id);
        return <g key={`label-${label.id}`} data-area-label-group={label.id}>
          <line x1={label.anchorX} y1={label.anchorY} x2={label.x} y2={label.y}
            data-testid="driving-map-area-leader" data-area-id={label.id}
            className={`dd-map-area-leader${area.selected ? ' dd-map-area-leader--selected' : ''}`} />
          <text x={label.x} y={label.y} data-testid="driving-map-area-label"
            data-area-id={label.id} data-label-side={label.side}
            data-anchor-x={label.anchorX} data-anchor-y={label.anchorY}
            className={`dd-map-area-label${area.selected ? ' dd-map-area-label--selected' : ''}`}>{area.label}</text>
        </g>;
      })}
      {data.pose && <><circle cx={px(0)} cy={py(0)} r="13" className="dd-map-location-halo" />
        <path d="m0-8 6 14-6-3-6 3Z" transform={`translate(${px(0)} ${py(0)})`} className="dd-map-location" /></>}
    </svg>
    <span className="dd-map-heading">{data.pose ? '↑ 전방' : '지도 수신 중'}</span>
    {!ready && <span className="dd-map-route-waiting">경로 수신 대기</span>}
  </div>;
}

// HH_261002 - Bind a stop confirmation to the admitted mission, not to a
// screen that may survive a backend restart or a new dispatch.
function activeStopMissionIdentity(snapshot) {
  const mission = snapshot?.mission;
  if (snapshot?.connected !== true || mission?.active !== true
    || !Number.isSafeInteger(mission.generation) || mission.generation <= 0
    || !mission.site || !mission.intent) return null;
  return [mission.generation, mission.site, mission.intent, mission.owner || ''].join(':');
}

export default function DrivingDisplay({ snapshot, onDismiss, onSafetyStop, onToggleTheme,
  systemHealth = 'OK', batteryPolicy = null, theme = 'light', demo = false,
  embedded = true, robotModelUrl = RANGER_MODEL_URL }) {
  const displayRef = useRef(null);
  const gesture = useRef(null);
  const dismissed = useRef(false);
  const id = `dd-${useId().replace(/:/g, '')}`;
  const receivedAt = useMemo(() => Date.now(), [snapshot]);
  const [now, setNow] = useState(() => Date.now());
  const [availableHeight, setAvailableHeight] = useState(null);
  const [confirmStop, setConfirmStop] = useState(null);
  const stopMissionIdentity = activeStopMissionIdentity(snapshot);
  // HH_261002 - A pending confirmation expires as soon as its mission identity
  // changes; the click handler also checks synchronously before issuing Stop.
  useEffect(() => {
    if (confirmStop !== null && confirmStop !== stopMissionIdentity) setConfirmStop(null);
  }, [confirmStop, stopMissionIdentity]);
  // HH_261001 - Desktop has room to show the real sensor feed status without hiding it
  // behind a disclosure. Keep the native details control for compact screens.
  const [sensorDetailsOpen, setSensorDetailsOpen] = useState(() =>
    typeof window !== 'undefined' && window.innerWidth > 940);
  // HH_261001 - Advance sample age even when transport stalls without closing its socket.
  useEffect(() => { const timer = setInterval(() => setNow(Date.now()), 500); return () => clearInterval(timer); }, []);
  // HH_261001 - The original header/status cards consume different heights on each device.
  // Observe this body card, not the viewport, so its footer always stays clear.
  useEffect(() => {
    if (!embedded) return undefined;
    const measure = () => {
      const height = displayRef.current?.getBoundingClientRect().height;
      if (height > 0) setAvailableHeight(Math.round(height));
    };
    measure();
    if (typeof ResizeObserver !== 'undefined') {
      const observer = new ResizeObserver(measure);
      if (displayRef.current) observer.observe(displayRef.current);
      return () => observer.disconnect();
    }
    window.addEventListener('resize', measure);
    return () => window.removeEventListener('resize', measure);
  }, [embedded]);
  useEffect(() => {
    const previous = document.activeElement;
    displayRef.current?.focus({ preventScroll: true });
    return () => { if (previous?.isConnected && typeof previous.focus === 'function') previous.focus({ preventScroll: true }); };
  }, []);
  const data = normalizeDrivingSnapshot(snapshot, Math.max(0, (now - receivedAt) / 1000));
  const distance = formatDistance(data.remainingDistance);
  const duration = formatDuration(data.remainingTime);
  const label = missionStatus(data);
  const nextGuide = nextRouteInstruction(data);
  const returning = ['RETURNING_TO_DROP_ZONE', 'RETURN_WITH_CARGO', 'DROP_ZONE_PARKING'].includes(data.mission.state);
  const destination = data.connected ? returning ? '출발지' : data.mission.site || '목적지 정보 대기' : '목적지 정보 대기';
  const serviceLabel = data.mission.intent === 'recall' ? '회수 서비스'
    : data.mission.intent === 'delivery' ? '배달 서비스' : '주행 현황';
  const sensorCount = ['lidar', 'camera', 'radar'].filter(name => Boolean(data.sensors[name])).length;
  const warningLabels = [
    !data.connected ? '주행 데이터 연결 끊김 · 최신 정보 수신 대기' : null,
    systemHealth !== 'OK' ? `${({ STARTING: '시스템 시작 중', WARNING: '시스템 경고', ERROR: '시스템 오류' })[systemHealth] || '시스템 상태 확인 중'} · 서비스 화면에서 상세 상태 확인` : null,
    batteryPolicy && batteryPolicy.tone !== 'ok' ? batteryPolicy.label : null,
    data.mission.phase === 'SAFETY_STOP' ? '안전 정지 중 · 운행 상태 확인 필요' : null,
  ].filter(Boolean);
  const interactive = (target) => target instanceof Element
    && Boolean(target.closest('button, a, input, select, textarea, summary, [data-dd-interactive]'));
  const keepOnDisplay = (event, callback) => {
    event.preventDefault();
    event.stopPropagation();
    gesture.current = null;
    callback();
  };
  const dismiss = (event, explicit = false) => {
    event.preventDefault();
    event.stopPropagation();
    if (confirmStop || (!explicit && interactive(event.target)) || dismissed.current || (!explicit && gesture.current?.moved)
      || typeof onDismiss !== 'function') return;
    dismissed.current = true;
    onDismiss();
  };
  const pointerDown = (event) => {
    event.stopPropagation();
    if (interactive(event.target)) { gesture.current = { moved: true }; return; }
    // HH_261001 - A drag on the map/scene should not count as the tap-to-return gesture.
    gesture.current = { x: event.clientX, y: event.clientY, moved: false };
    try { event.currentTarget.setPointerCapture(event.pointerId); } catch (_) { /* HH_261001 - Older touch engines may not support capture. */ }
  };
  const pointerMove = (event) => {
    if (!gesture.current) return;
    if (Math.hypot(event.clientX - gesture.current.x, event.clientY - gesture.current.y) > 14) gesture.current.moved = true;
  };
  return (
    <section ref={displayRef} className={`driving-display dd-theme-${theme === 'dark' ? 'dark' : 'light'}${embedded ? ' dd-embedded' : ''}${embedded && availableHeight > 0 && availableHeight <= 620 ? ' dd-density-compact' : ''}${embedded && availableHeight > 0 && availableHeight <= 460 ? ' dd-density-short' : ''}`}
      data-testid="driving-display" data-demo={demo ? 'true' : 'false'}
      role={embedded ? "region" : "dialog"} aria-modal={embedded ? undefined : "true"} aria-label={demo ? '주행 화면 시연. 실제 주행 아님' : '주행 현황'}
      aria-describedby={`${id}-dismiss-hint`} tabIndex={0}
      onPointerDown={pointerDown} onPointerMove={pointerMove}
      onPointerUp={(event) => event.stopPropagation()}
      onPointerCancel={() => { gesture.current = { moved: true }; }}
      onClick={dismiss}
      onKeyDown={(event) => {
        if (event.repeat) return;
        if (event.key === 'Escape' && confirmStop) {
          keepOnDisplay(event, () => setConfirmStop(false));
        } else if ((event.key === 'Escape' || event.key === 'Enter' && event.target === event.currentTarget)
          && !interactive(event.target)) { gesture.current = null; dismiss(event); }
      }}>
      <header className="dd-header">
        <div className="dd-brand"><img src="/월악산_국립공원_로고.jpg" alt="월악산 국립공원 로고" /><div><strong>CAMROD</strong><small>월악산 국립공원</small></div></div>
        <span className="dd-top-mission">현재 미션<strong>{serviceLabel}</strong></span>
        <div className="dd-header-chips">
          {demo && <span className="dd-demo-chip">UI 시연 · 실제 주행 아님</span>}
          <span className={`dd-chip ${data.connected ? '' : 'dd-chip--unavailable'}`}><span className="dd-chip-dot" /><span>{data.connected ? '연결됨' : '연결 끊김'}</span></span>
          <span className={`dd-chip ${!data.sensors.gnss ? 'dd-chip--unavailable' : ''}`}><Icon name="satellite" size={17} /><span>GNSS</span><strong>{data.sensors.gnss || '수신 대기'}</strong></span>
          <span className={`dd-chip ${data.battery === null ? 'dd-chip--unavailable' : data.battery < 25 ? 'dd-chip--warning' : ''}`}><Icon name="battery" size={18} /><strong>{data.battery === null ? '—' : Math.round(data.battery)}<span className="dd-chip-unit">%</span></strong></span>
          <time className="dd-clock" dateTime={new Date(now).toISOString()}>{new Date(now).toLocaleTimeString('ko-KR', { hour: '2-digit', minute: '2-digit', hour12: false })}</time>
        </div>
        <div className="dd-header-actions">
          {onToggleTheme && <button type="button" className="dd-action dd-theme-action" aria-label={theme === 'light' ? '다크 모드' : '라이트 모드'}
            onClick={(event) => keepOnDisplay(event, onToggleTheme)}>{theme === 'light' ? '다크' : '라이트'}</button>}
          {onSafetyStop && !demo && <button type="button" className="dd-action dd-stop-action"
            disabled={!stopMissionIdentity}
            onClick={(event) => keepOnDisplay(event, () => {
              if (stopMissionIdentity) setConfirmStop(stopMissionIdentity);
            })}>운행 정지</button>}
          <button type="button" className="dd-action dd-back-action" onClick={event => dismiss(event, true)}>{data.mission.active ? '서비스 화면' : '홈으로'}</button>
        </div>
      </header>
      {warningLabels.length > 0 && <div className="dd-alerts" role="alert">{warningLabels.map((warning, index) => <span key={`${index}-${warning}`}>{warning}</span>)}</div>}

      <div className="dd-main">
        <aside className="dd-left">
          <div className="dd-eyebrow">현재 속도</div>
          <div className="dd-speed" data-testid="driving-speed"><span className="dd-speed-number">{data.speedKmh === null ? '—' : data.speedKmh.toFixed(1)}</span><span className="dd-speed-unit">km/h</span></div>
          <div className="dd-speed-caption">{data.speed === null ? '속도 정보 수신 대기' : '로봇 기준 주행 속도'}</div>

          <div className="dd-mission">
            <span className="dd-section-label">주행 상태</span>
            <h1 className={`dd-mission-title${['SAFETY_STOP', 'STOPPED'].includes(data.mission.phase) ? ' dd-mission-title--warning' : ''}`}>{label}</h1>
            <p className="dd-mission-description">{missionDescription(data)}</p>
            <div className="dd-destination"><span className="dd-destination-icon"><Icon name="pin" size={21} /></span><div><span>{returning ? '복귀 위치' : '목적지'}</span><strong>{destination}</strong></div></div>
          </div>

          <details className="dd-sensors" data-dd-interactive open={sensorDetailsOpen}
            onToggle={event => setSensorDetailsOpen(event.currentTarget.open)}
            onClick={event => event.stopPropagation()}
            onPointerDown={event => event.stopPropagation()} onPointerUp={event => event.stopPropagation()}
            onKeyDown={event => event.stopPropagation()}>
            <summary><Icon name="scan" size={17} /><span>센서 상태</span><small>{sensorCount}/3 수신</small></summary>
            {['lidar', 'camera', 'radar'].map((name) => <div className="dd-sensor-row" key={name}><span>{({ lidar: 'LiDAR', camera: 'Camera', radar: 'Radar' })[name]}</span><span className={data.sensors[name] ? 'dd-sensor-value' : 'dd-sensor-value dd-sensor-value--missing'}><i />{data.sensors[name] || '정보 없음'}</span></div>)}
            <p className="dd-sensor-note">{data.perceptionReady ? `${demo ? '시연 ' : ''}감지 데이터 표시` : '좌표가 확인된 감지 데이터만 표시합니다'}</p>
          </details>
        </aside>

        <DrivingScene data={data} demo={demo} theme={theme} robotModelUrl={robotModelUrl} />

        <aside className="dd-right">
          <div className="dd-route-card">
            <div className="dd-route-heading"><span className="dd-route-icon"><Icon name="arrow" size={22} /></span><div><span className="dd-section-label">다음 이동 안내</span><h2>{nextGuide.title}</h2><small>{nextGuide.distance === null ? nextGuide.detail : `약 ${formatDistance(nextGuide.distance).value}${formatDistance(nextGuide.distance).unit} 앞 · ${nextGuide.detail}`}</small></div></div>
            <div className="dd-map-group">
              <span className="dd-map-title">{demo ? '시연 경로 지도' : '현재 경로 지도'}</span>
              <RouteMap data={data} id={id} />
              <div className="dd-map-caption"><span><i />{demo ? '시연 위치' : '현재 위치'}</span><span>{data.route.length > 1 ? '수신 경로 기준' : '데이터 대기'}</span></div>
            </div>
            <div className="dd-progress-group">
              <div className="dd-route-metrics">
                <div><span className="dd-metric-label">남은 거리</span><strong>{distance.value}<small>{distance.unit}</small></strong></div>
                <div><span className="dd-metric-label">예상 소요</span><strong>{duration.value}<small>{duration.unit}</small></strong></div>
              </div>
              <div className="dd-progress-label"><span>경로 진행률</span><span>{data.completion === null ? '산출 대기' : `${Math.round(data.completion)}%`}</span></div>
              <div className="dd-progress-track" role="progressbar" aria-label="경로 진행률" aria-valuemin={0} aria-valuemax={100} aria-valuenow={data.completion ?? undefined} aria-valuetext={data.completion === null ? '정보 없음' : undefined}><span style={{ width: `${data.completion ?? 0}%` }} /></div>
              <p className="dd-estimate-note"><Icon name="clock" size={13} />{data.remainingTime === null ? '유효한 주행 예측 정보가 없습니다' : '수신한 경로 기반 추정치입니다'}</p>
            </div>
          </div>
        </aside>
      </div>

      <footer className="dd-footer"><span id={`${id}-dismiss-hint`}><Icon name="touch" size={16} />일반 화면을 터치하면 서비스 화면으로 돌아갑니다</span><span>ESC</span></footer>
      {confirmStop && <div className="dd-stop-layer" data-dd-interactive onClick={event => event.stopPropagation()}
        onPointerDown={event => event.stopPropagation()} onPointerUp={event => event.stopPropagation()}>
        <div className="dd-stop-dialog" role="alertdialog" aria-modal="true" aria-labelledby={`${id}-stop-title`}>
          <h2 id={`${id}-stop-title`}>운행을 정지하시겠습니까?</h2>
          <p>기존 운행 정지 기능을 실행합니다.</p>
          <div><button type="button" onClick={event => keepOnDisplay(event, () => setConfirmStop(null))}>계속 주행</button>
            <button type="button" className="dd-stop-confirm" onClick={event => keepOnDisplay(event, () => {
              const sameMission = confirmStop !== null && confirmStop === stopMissionIdentity;
              setConfirmStop(null);
              if (sameMission && typeof onSafetyStop === 'function') onSafetyStop();
            })}>예, 운행 정지</button></div>
        </div>
      </div>}
    </section>
  );
}
