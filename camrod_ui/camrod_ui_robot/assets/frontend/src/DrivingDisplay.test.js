import React, { act } from 'react';
import { createRoot } from 'react-dom/client';
import DrivingDisplay, {
  normalizeDrivingSnapshot, toHeadingUp, missionStatus, formatDistance, formatDuration,
  missionDescription, nextRouteInstruction,
  normalizeObjectGeometry, normalizeBaseMap,
} from './DrivingDisplay';

jest.mock('./RangerNavigationScene', () => function MockRangerNavigationScene({ data, modelUrl }) {
  return <div data-testid="ranger-navigation-scene" data-model-url={modelUrl}>
    실시간 모델
    {data.pose && data.route.length > 1 && <span data-testid="driving-actual-route" />}
  </div>;
});

const fixture = () => ({
  schema_version: 1,
  connected: true,
  mission: { active: true, generation: 41, owner: 'operator', intent: 'delivery', site: 'A3', service_state_name: 'MOVING_TO_SITE', phase: 'DRIVING' },
  pose: { x: 0, y: 0, yaw: 0, frame_id: 'map', age_s: 0.1 },
  route: { points: [[0, 0], [5, 0], [10, 1]], frame_id: 'map', age_s: 0.1, valid: true },
  perception: { points: [[2, 1, 0], [4, 1, null]], objects: [{ id: '1', class_name: 'object', x: 2, y: 2, z: null }], frame_id: 'map', age_s: 0.1 },
  motion: { speed_mps: 0.5 }, battery: { percentage: 85 },
  progress: { valid: true, remaining_distance_m: 42, remaining_time_s: 40, completion_pct: 25 },
  sensors: { gnss: { label: 'FIX', age_s: 0.1 }, lidar: { label: '수신 중', age_s: 0.1 } },
});

describe('read-only driving snapshot normalization', () => {
  test('observed metre boxes retain their own center and normalized map orientation', () => {
    // HH_261002 - Box centers differ from the semantic closest-point centroid.
    const geometry = { geometry_source: 'observed_lidar_extent',
      dimensions: { x: 0.5, y: 0.6, z: 1.7 },
      bbox: { center: { x: 3, y: 2, z: 0.85 } },
      orientation: { x: 0, y: 0, z: 2, w: 2 } };
    const snapshot = fixture();
    Object.assign(snapshot.perception.objects[0], geometry);
    const object = normalizeDrivingSnapshot(snapshot).objects[0];
    expect(object.dimensions).toEqual(geometry.dimensions);
    expect(object.bbox.center).toEqual(geometry.bbox.center);
    expect(object.x).toBe(2);
    expect(object.orientation.z).toBeCloseTo(Math.SQRT1_2);
    for (const change of [ { geometry_source: 'fixed_marker' },
      { dimensions: { x: -1, y: 1, z: 1 } }, { dimensions: { x: Infinity, y: 1, z: 1 } },
      { orientation: { x: 0, y: 0, z: 0, w: 0 } }, { bbox: null } ]) {
      expect(normalizeObjectGeometry({ ...geometry, ...change }).dimensions).toBeNull();
    }
  });
  test('missing data is unavailable, never fabricated zero speed or battery', () => {
    const data = normalizeDrivingSnapshot(null);
    expect(data.speed).toBeNull();
    expect(data.battery).toBeNull();
    expect(data.pose).toBeNull();
    expect(data.route).toEqual([]);
    expect(data.remainingDistance).toBeNull();
    expect(formatDistance(null).value).toBe('—');
    expect(formatDuration(null).value).toBe('—');
  });
  test('connected fresh measurements and XY-only detections survive unchanged', () => {
    const data = normalizeDrivingSnapshot(fixture());
    expect(data.speedKmh).toBe(1.8);
    expect(data.route).toHaveLength(3);
    expect(data.points).toHaveLength(2);
    expect(data.objects[0].z).toBeNull();
    expect(data.remainingDistance).toBe(42);
    expect(data.sensors.gnss).toBe('FIX');
  });
  test('disconnect overrides fresh-looking fields', () => {
    const data = normalizeDrivingSnapshot({ ...fixture(), connected: false });
    expect(data.speed).toBeNull();
    expect(data.battery).toBeNull();
    expect(data.pose).toBeNull();
    expect(data.route).toEqual([]);
    expect(data.points).toEqual([]);
    expect(data.objects).toEqual([]);
    expect(data.sensors.gnss).toBeNull();
    expect(data.remainingTime).toBeNull();
  });
  test('mission-valid latched paths outlive 30s; pose and perception still expire', () => {
    const snapshot = fixture();
    snapshot.route.age_s = 20;
    expect(normalizeDrivingSnapshot(snapshot).route).toHaveLength(3);
    snapshot.route.age_s = 30.1;
    expect(normalizeDrivingSnapshot(snapshot).route).toHaveLength(3);
    snapshot.route.valid = false;
    expect(normalizeDrivingSnapshot(snapshot).route).toEqual([]);
    snapshot.route.valid = true;
    snapshot.perception.age_s = 3.1;
    expect(normalizeDrivingSnapshot(snapshot).points).toEqual([]);
    snapshot.pose.age_s = 3.1;
    expect(normalizeDrivingSnapshot(snapshot).route).toEqual([]);
  });
  test('the real static map remains valid without a route or recent receipt age', () => {
    // HH_261002 - Map validity is independent of the mission-bound route latch.
    const snapshot = fixture();
    snapshot.route.valid = false;
    snapshot.base_map = { valid: true, frame_id: 'map', source: '/map/markers', age_s: 600,
      polylines: [{ namespace: 'lanelet/left_bound', marker_id: 4, points: [[0, 1], [4, 1]] }] };
    const normalized = normalizeDrivingSnapshot(snapshot);
    expect(normalized.route).toEqual([]);
    expect(normalized.baseMap.valid).toBe(true);
    expect(normalized.baseMap.polylines[0].points).toEqual([[0, 1], [4, 1]]);
    expect(normalized.baseMap.polylines[0].marker_id).toBe(4);
    expect(normalizeBaseMap({ ...snapshot.base_map, polylines: [
      { ...snapshot.base_map.polylines[0], marker_id: '4' },
    ] }).polylines[0].marker_id).toBeUndefined();
    expect(normalizeBaseMap({ ...snapshot.base_map, frame_id: 'odom' }).valid).toBe(false);
    expect(normalizeBaseMap({ ...snapshot.base_map, source: 'illustration' }).valid).toBe(false);
  });
  test('all configured sites and drop-zone polygons render before a route and highlight only the destination', () => {
    // HH_261002 - The overview shows received YAML footprints, not invented
    // site rectangles; route authority stays independent of static map areas.
    const snapshot = fixture();
    snapshot.route.valid = false;
    snapshot.mission.active = false;
    snapshot.base_map = { valid: true, frame_id: 'map', source: '/map/markers',
      polylines: [], areas: [
        ...Array.from({ length: 13 }, (_, index) => ({
          id: `camping_site_${index + 1}`, label: `B${index + 1}`,
          kind: 'camping_site', site: `B${index + 1}`,
          source: 'camping_sites_yaml',
          points: [[index * 4, 0], [index * 4 + 2, 0],
            [index * 4 + 2, 2], [index * 4, 2]],
        })),
        { id: 'dz_area_7144', label: '드롭존', kind: 'drop_zone', site: null,
          source: 'drop_zones_yaml', points: [[-4, 0], [-2, 0], [-2, 2], [-4, 2]] },
      ] };
    expect(normalizeDrivingSnapshot(snapshot).baseMap.areas).toHaveLength(14);
    global.IS_REACT_ACT_ENVIRONMENT = true;
    const host = document.createElement('div');
    document.body.appendChild(host);
    const root = createRoot(host);
    act(() => root.render(<DrivingDisplay snapshot={snapshot} onDismiss={() => {}} />));
    const map = host.querySelector('[data-testid="driving-map"]');
    expect(map.querySelectorAll('[data-testid="driving-map-area"]')).toHaveLength(14);
    expect(map.querySelectorAll('[data-testid="driving-map-area-label"]')).toHaveLength(14);
    expect(map.querySelectorAll('[data-testid="driving-map-area-leader"]')).toHaveLength(14);
    for (const label of map.querySelectorAll('[data-testid="driving-map-area-label"]')) {
      const leader = map.querySelector(`[data-testid="driving-map-area-leader"][data-area-id="${label.getAttribute('data-area-id')}"]`);
      expect(Number(leader.getAttribute('x1'))).toBe(Number(label.getAttribute('data-anchor-x')));
      expect(Number(leader.getAttribute('y1'))).toBe(Number(label.getAttribute('data-anchor-y')));
    }
    expect(map.textContent).toContain('B13');
    expect(map.textContent).toContain('드롭존');
    expect(map.querySelector('.dd-path-core')).toBeNull();
    expect(map.querySelector('.dd-map-area--selected')).toBeNull();

    const delivery = { ...snapshot, mission: { ...snapshot.mission, active: true, site: 'B9' } };
    act(() => root.render(<DrivingDisplay snapshot={delivery} onDismiss={() => {}} />));
    expect(map.querySelector('.dd-map-area--selected').getAttribute('data-site')).toBe('B9');
    const returning = { ...delivery, mission: { ...delivery.mission,
      service_state_name: 'RETURN_WITH_CARGO' } };
    act(() => root.render(<DrivingDisplay snapshot={returning} onDismiss={() => {}} />));
    expect(map.querySelector('.dd-map-area--selected').getAttribute('data-kind')).toBe('drop_zone');
    expect(normalizeBaseMap({ ...snapshot.base_map, frame_id: 'odom' }).areas).toEqual([]);
    act(() => root.unmount());
    host.remove();
  });
  test('a route without explicit backend mission validity stays hidden', () => {
    const snapshot = fixture();
    delete snapshot.route.valid;
    expect(normalizeDrivingSnapshot(snapshot).route).toEqual([]);
    expect(normalizeDrivingSnapshot(snapshot).routeReason).toBe('현재 미션 경로 수신 대기');
  });
  test('local maneuvers explain the planned route gap without claiming disconnection', () => {
    // HH_261002 - The CARLA B9 run exposed misleading route-loss copy during
    // site entry even while telemetry and vehicle pose remained connected.
    const snapshot = fixture();
    snapshot.route.valid = false;
    snapshot.mission.service_state_name = 'SITE_ENTRY';
    const data = normalizeDrivingSnapshot(snapshot);
    expect(data.connected).toBe(true);
    expect(data.route).toEqual([]);
    expect(data.routeReason).toContain('사이트 진입 동작 중');
  });
  test('a stopped robot can show fresh remaining distance without a false ETA', () => {
    const snapshot = fixture();
    snapshot.motion.speed_mps = 0;
    snapshot.progress = { ...snapshot.progress, reason: 'stopped', remaining_time_s: null };
    const data = normalizeDrivingSnapshot(snapshot);
    expect(data.connected).toBe(true);
    expect(data.remainingDistance).toBe(42);
    expect(data.remainingTime).toBeNull();
  });
  test('stalled transport ages out even when connected flag remains true', () => {
    const data = normalizeDrivingSnapshot(fixture(), 3.1);
    expect(data.connected).toBe(false);
    expect(data.speed).toBeNull();
    expect(data.route).toEqual([]);
  });
  test('stale GNSS never remains a fix in the live status bar', () => {
    const sample = fixture();
    sample.sensors.gnss.age_s = 3.2;
    expect(normalizeDrivingSnapshot(sample).sensors.gnss).toBeNull();
  });
  test('frame mismatch and unknown age suppress geometry', () => {
    const snapshot = fixture();
    snapshot.route.frame_id = 'odom';
    snapshot.perception.frame_id = 'base_link';
    expect(normalizeDrivingSnapshot(snapshot).route).toEqual([]);
    expect(normalizeDrivingSnapshot(snapshot).points).toEqual([]);
    delete snapshot.pose.age_s;
    expect(normalizeDrivingSnapshot(snapshot).pose).toBeNull();
  });
  test('large arrays and malformed coordinates are bounded', () => {
    const snapshot = fixture();
    snapshot.route.points = Array.from({ length: 10000 }, (_, index) => [index, 0]);
    snapshot.perception.points = Array.from({ length: 10000 }, (_, index) => [index, 0, 0]);
    snapshot.perception.objects = Array.from({ length: 1000 }, (_, index) => ({ x: index, y: 0, z: 0 }));
    const data = normalizeDrivingSnapshot(snapshot);
    expect(data.route).toHaveLength(256);
    expect(data.points).toHaveLength(600);
    expect(data.objects).toHaveLength(32);
    snapshot.route.points = [[null, 2], [Infinity, 4], [5, NaN]];
    expect(normalizeDrivingSnapshot(snapshot).route).toEqual([]);
  });
  test('negative-yaw transform makes true heading forward', () => {
    const result = toHeadingUp([10, 30], { x: 10, y: 20, yaw: Math.PI / 2 });
    expect(result.forward).toBeCloseTo(10);
    expect(result.left).toBeCloseTo(0);
  });
  test('next movement cue comes only from a fresh route and matching pose', () => {
    const route = [[0, 0], [3, 0], [6, 0], [9, 0], [11, 1], [12, 3], [12, 6]];
    const outbound = normalizeDrivingSnapshot({ ...fixture(),
      route: { points: route, frame_id: 'map', age_s: 0.1, valid: true },
      pose: { x: 1, y: 0, yaw: 0, frame_id: 'map', age_s: 0.1 },
    });
    expect(nextRouteInstruction(outbound).title).toBe('좌측 경로로 진행');
    expect(nextRouteInstruction(outbound).distance).toBeGreaterThan(0);
    const stale = normalizeDrivingSnapshot({ ...fixture(),
      route: { points: route, frame_id: 'odom', age_s: 0.1, valid: true },
    });
    expect(nextRouteInstruction(stale).title).toBe('경로 안내 대기');
    expect(nextRouteInstruction(normalizeDrivingSnapshot(null)).distance).toBeNull();
  });
  test('fresh fused objects never rejuvenate stale cloud points or stale per-object samples', () => {
    const snapshot = fixture();
    snapshot.perception.age_s = 0.1;
    snapshot.perception.points_age_s = 9;
    snapshot.perception.objects_age_s = 0.1;
    expect(normalizeDrivingSnapshot(snapshot).points).toEqual([]);
    expect(normalizeDrivingSnapshot(snapshot).objects).toHaveLength(1);
    snapshot.perception.points_age_s = 0.1;
    snapshot.perception.objects[0].age_s = 9;
    expect(normalizeDrivingSnapshot(snapshot).points).toHaveLength(2);
    expect(normalizeDrivingSnapshot(snapshot).objects).toEqual([]);
    snapshot.perception.points_age_s = null;
    snapshot.perception.objects_age_s = null;
    snapshot.perception.objects[0].age_s = 0.1;
    expect(normalizeDrivingSnapshot(snapshot).points).toEqual([]);
    expect(normalizeDrivingSnapshot(snapshot).objects).toHaveLength(1);
    expect(normalizeDrivingSnapshot(snapshot).perceptionReady).toBe(true);
  });
  test('loading, parking, return and safety stop labels do not imply forward travel', () => {
    const snapshot = fixture();
    snapshot.mission.service_state_name = 'GUEST_LOADING_WAIT';
    expect(missionStatus(normalizeDrivingSnapshot(snapshot))).toBe('짐 싣기 대기');
    snapshot.mission.service_state_name = 'DROP_ZONE_PARKING';
    expect(missionStatus(normalizeDrivingSnapshot(snapshot))).toBe('주차 진행 중');
    snapshot.mission.service_state_name = 'RETURN_WITH_CARGO';
    expect(missionStatus(normalizeDrivingSnapshot(snapshot))).toBe('짐을 싣고 복귀 중');
    snapshot.mission.phase = 'SAFETY_STOP';
    expect(missionStatus(normalizeDrivingSnapshot(snapshot))).toBe('안전 정지');
  });
  test('live mission copy hides raw controller internals from the passenger view', () => {
    const snapshot = fixture();
    snapshot.mission.site = 'B7';
    snapshot.mission.description = 'camping_site_maneuver_controller:DONE:lanelet handoff current=(1,2)';
    expect(missionDescription(normalizeDrivingSnapshot(snapshot))).toBe('B7 사이트로 이동하고 있습니다.');
    snapshot.mission.service_state_name = 'RETURN_WITH_CARGO';
    expect(missionDescription(normalizeDrivingSnapshot(snapshot))).toBe('짐을 싣고 출발지로 복귀하고 있습니다.');
    snapshot.mission.phase = 'SAFETY_STOP';
    expect(missionDescription(normalizeDrivingSnapshot(snapshot))).toContain('상세 진단');
  });
});

describe('display interactions', () => {
  let host, root;
  beforeEach(() => {
    global.IS_REACT_ACT_ENVIRONMENT = true;
    jest.useFakeTimers();
    host = document.createElement('div');
    document.body.appendChild(host);
    root = createRoot(host);
  });
  afterEach(() => { act(() => root.unmount()); host.remove(); jest.useRealTimers(); });

  test('embedded body has a compact brand/status bar and uses the actual Ranger asset', () => {
    act(() => root.render(<DrivingDisplay snapshot={fixture()} onDismiss={() => {}} />));
    const display = host.querySelector('[data-testid="driving-display"]');
    expect(display.classList.contains('dd-embedded')).toBe(true);
    expect(display.getAttribute('role')).toBe('region');
    expect(display.hasAttribute('aria-modal')).toBe(false);
    expect(host.querySelector('.dd-header')).not.toBeNull();
    expect(host.querySelector('.dd-header').textContent).toContain('CAMROD');
    expect(host.textContent).not.toContain('DRIVING VIEW');
    expect(host.querySelector('[data-testid="driving-ranger-schematic"]')).toBeNull();
    expect(host.querySelector('[data-testid="ranger-navigation-scene"]').getAttribute('data-model-url'))
      .toMatch(/^\/models\/ranger-navigation\.glb\?v=[^&]+$/);
    expect(host.textContent).not.toContain('Ranger 3D 모델');
    expect(host.querySelector('image')).toBeNull();
  });

  test('base map is visible before a route and the route overlays it after receipt', () => {
    // HH_261002 - No synthetic route is drawn while the real map is already known.
    const snapshot = fixture();
    snapshot.route.valid = false;
    snapshot.base_map = { valid: true, frame_id: 'map', source: '/map/markers', age_s: 100,
      polylines: [
        { namespace: 'lanelet/left_bound', points: [[0, 2], [10, 2]] },
        { namespace: 'lanelet/right_bound', points: [[0, -2], [10, -2]] },
        { namespace: 'lanelet/centerline', points: [[0, 0], [10, 0]] },
      ] };
    act(() => root.render(<DrivingDisplay snapshot={snapshot} onDismiss={() => {}} />));
    const map = host.querySelector('[data-testid="driving-map"]');
    expect(map.getAttribute('data-map-source')).toBe('/map/markers');
    // HH_261002 - CSS uses these exact namespaces to distinguish green painted
    // bounds from muted centerline; a later blue route is a separate SVG layer.
    expect(map.querySelectorAll('[data-testid="driving-map-base-line"]')).toHaveLength(3);
    expect(map.querySelectorAll('[data-namespace="lanelet/left_bound"]')).toHaveLength(1);
    expect(map.querySelectorAll('[data-namespace="lanelet/right_bound"]')).toHaveLength(1);
    expect(map.querySelectorAll('[data-namespace="lanelet/centerline"]')).toHaveLength(1);
    expect(map.querySelector('.dd-path-core')).toBeNull();
    expect(map.textContent).toContain('경로 수신 대기');
    act(() => root.render(<DrivingDisplay snapshot={{ ...snapshot, route: { ...fixture().route } }} onDismiss={() => {}} />));
    expect(map.querySelectorAll('[data-testid="driving-map-base-line"]')).toHaveLength(3);
    expect(map.querySelector('.dd-path-core')).not.toBeNull();
  });

  test('the model asset may be supplied explicitly without recreating geometry', () => {
    act(() => root.render(<DrivingDisplay snapshot={fixture()} robotModelUrl="/models/actual-ranger.glb" onDismiss={() => {}} />));
    expect(host.querySelector('[data-testid="ranger-navigation-scene"]').getAttribute('data-model-url')).toBe('/models/actual-ranger.glb');
  });

  test('mini-map rounds only display boundary corners while keeping received XY intact', () => {
    // HH_261002 - Mini-map and 3D paint share a visual approximation, not a new map.
    const snapshot = fixture();
    snapshot.base_map = { valid: true, frame_id: 'map', source: '/map/markers',
      polylines: [{ namespace: 'lanelet/left_bound', points: [[0, 2], [5, 2], [5, 6]] }] };
    const original = JSON.stringify(snapshot);
    act(() => root.render(<DrivingDisplay snapshot={snapshot} onDismiss={() => {}} />));
    const path = host.querySelector('[data-namespace="lanelet/left_bound"]').getAttribute('d');
    expect((path.match(/L/g) || []).length).toBeGreaterThan(2);
    expect(normalizeBaseMap(snapshot.base_map).polylines[0].points).toEqual(snapshot.base_map.polylines[0].points);
    expect(JSON.stringify(snapshot)).toBe(original);
  });

  test('layout density follows available card height rather than viewport height', () => {
    const bounds = jest.spyOn(HTMLElement.prototype, 'getBoundingClientRect').mockReturnValue({ height: 410, width: 1200 });
    act(() => root.render(<DrivingDisplay snapshot={fixture()} onDismiss={() => {}} />));
    const display = host.querySelector('[data-testid="driving-display"]');
    expect(display.classList.contains('dd-density-short')).toBe(true);
    expect(display.classList.contains('dd-density-compact')).toBe(true);
    bounds.mockReturnValue({ height: 584, width: 1840 });
    act(() => window.dispatchEvent(new Event('resize')));
    expect(display.classList.contains('dd-density-short')).toBe(false);
    expect(display.classList.contains('dd-density-compact')).toBe(true);
    bounds.mockRestore();
  });

  test('demo label is visible; click is consumed and only dismisses once', () => {
    const onDismiss = jest.fn();
    const outerClick = jest.fn();
    act(() => root.render(<div onClick={outerClick}><DrivingDisplay snapshot={fixture()} onDismiss={onDismiss} demo /></div>));
    expect(host.textContent).toContain('UI 시연 · 실제 주행 아님');
    const display = host.querySelector('[data-testid="driving-display"]');
    act(() => display.dispatchEvent(new MouseEvent('pointerdown', { bubbles: true, clientX: 30, clientY: 30 })));
    act(() => display.dispatchEvent(new MouseEvent('pointerup', { bubbles: true, clientX: 30, clientY: 30 })));
    expect(onDismiss).not.toHaveBeenCalled();
    act(() => display.dispatchEvent(new MouseEvent('click', { bubbles: true, cancelable: true })));
    act(() => display.dispatchEvent(new MouseEvent('click', { bubbles: true, cancelable: true })));
    expect(onDismiss).toHaveBeenCalledTimes(1);
    expect(outerClick).not.toHaveBeenCalled();
  });
  test('Escape dismisses and blocks propagation', () => {
    const onDismiss = jest.fn();
    const outerKey = jest.fn();
    act(() => root.render(<div onKeyDown={outerKey}><DrivingDisplay snapshot={fixture()} onDismiss={onDismiss} /></div>));
    act(() => host.querySelector('[data-testid="driving-display"]').dispatchEvent(new KeyboardEvent('keydown', { key: 'Escape', bubbles: true, cancelable: true })));
    expect(onDismiss).toHaveBeenCalledTimes(1);
    expect(outerKey).not.toHaveBeenCalled();
  });
  test('drag gesture does not dismiss', () => {
    const onDismiss = jest.fn();
    act(() => root.render(<DrivingDisplay snapshot={fixture()} onDismiss={onDismiss} />));
    const display = host.querySelector('[data-testid="driving-display"]');
    act(() => display.dispatchEvent(new MouseEvent('pointerdown', { bubbles: true, clientX: 10, clientY: 10 })));
    act(() => display.dispatchEvent(new MouseEvent('pointermove', { bubbles: true, clientX: 100, clientY: 100 })));
    act(() => display.dispatchEvent(new MouseEvent('click', { bubbles: true, cancelable: true })));
    expect(onDismiss).not.toHaveBeenCalled();
  });
  test('theme, sensors and existing stop handler do not trigger presentation dismissal', () => {
    const onDismiss = jest.fn(), onSafetyStop = jest.fn(), onToggleTheme = jest.fn();
    act(() => root.render(<DrivingDisplay snapshot={fixture()} onDismiss={onDismiss}
      onSafetyStop={onSafetyStop} onToggleTheme={onToggleTheme} />));
    act(() => host.querySelector('.dd-theme-action').click());
    expect(onToggleTheme).toHaveBeenCalledTimes(1);
    expect(onDismiss).not.toHaveBeenCalled();
    expect(host.querySelector('.dd-sensors').open).toBe(true);
    act(() => host.querySelector('.dd-sensors summary').click());
    expect(onDismiss).not.toHaveBeenCalled();
    expect(host.querySelector('.dd-map-group')).not.toBeNull();
    expect(host.querySelector('.dd-progress-group')).not.toBeNull();
    act(() => host.querySelector('.dd-stop-action').click());
    expect(host.querySelector('.dd-stop-dialog')).not.toBeNull();
    expect(onSafetyStop).not.toHaveBeenCalled();
    act(() => host.querySelector('.dd-stop-dialog button').click());
    expect(onSafetyStop).not.toHaveBeenCalled();
    expect(onDismiss).not.toHaveBeenCalled();
    act(() => host.querySelector('.dd-stop-action').click());
    act(() => host.querySelector('.dd-stop-confirm').click());
    expect(onSafetyStop).toHaveBeenCalledTimes(1);
    expect(onDismiss).not.toHaveBeenCalled();
    act(() => host.querySelector('.dd-back-action').click());
    expect(onDismiss).toHaveBeenCalledTimes(1);
  });
  test('a pending stop confirmation expires when the active mission revision changes', () => {
    // HH_261002 - A previous mission's confirmation must never stop its successor.
    const onSafetyStop = jest.fn();
    const snapshot = fixture();
    act(() => root.render(<DrivingDisplay snapshot={snapshot} onSafetyStop={onSafetyStop} />));
    act(() => host.querySelector('.dd-stop-action').click());
    expect(host.querySelector('.dd-stop-dialog')).not.toBeNull();

    const next = { ...snapshot, mission: { ...snapshot.mission, generation: 42 } };
    act(() => root.render(<DrivingDisplay snapshot={next} onSafetyStop={onSafetyStop} />));
    expect(host.querySelector('.dd-stop-dialog')).toBeNull();
    expect(onSafetyStop).not.toHaveBeenCalled();

    act(() => host.querySelector('.dd-stop-action').click());
    expect(host.querySelector('.dd-stop-dialog')).not.toBeNull();
    act(() => host.querySelector('.dd-stop-confirm').click());
    expect(onSafetyStop).toHaveBeenCalledTimes(1);
  });
  test('cancelled stop confirmation does not call the stop handler', () => {
    const onSafetyStop = jest.fn();
    act(() => root.render(<DrivingDisplay snapshot={fixture()} onSafetyStop={onSafetyStop} />));
    act(() => host.querySelector('.dd-stop-action').click());
    act(() => host.querySelector('.dd-stop-dialog button').click());
    expect(host.querySelector('.dd-stop-dialog')).toBeNull();
    expect(onSafetyStop).not.toHaveBeenCalled();
  });
  test('stop confirmation requires an authoritative active mission revision', () => {
    const onSafetyStop = jest.fn();
    const snapshot = fixture();
    delete snapshot.mission.generation;
    act(() => root.render(<DrivingDisplay snapshot={snapshot} onSafetyStop={onSafetyStop} />));
    expect(host.querySelector('.dd-stop-action').disabled).toBe(true);
    expect(host.querySelector('.dd-stop-dialog')).toBeNull();
    expect(onSafetyStop).not.toHaveBeenCalled();
  });
  test('the compact bar keeps system and battery warnings visible', () => {
    act(() => root.render(<DrivingDisplay snapshot={fixture()} onDismiss={() => {}}
      systemHealth="WARNING" batteryPolicy={{ tone: 'error', label: '배터리 부족 · 즉시 복귀 18%' }} />));
    const alert = host.querySelector('.dd-alerts');
    expect(alert.getAttribute('role')).toBe('alert');
    expect(alert.textContent).toContain('시스템 경고');
    expect(alert.textContent).toContain('배터리 부족 · 즉시 복귀 18%');
  });
  test('snapshot stalls hide previously rendered geometry', () => {
    act(() => root.render(<DrivingDisplay snapshot={fixture()} onDismiss={() => {}} />));
    expect(host.querySelector('[data-testid="driving-actual-route"]')).not.toBeNull();
    act(() => jest.advanceTimersByTime(4000));
    expect(host.querySelector('[data-testid="driving-actual-route"]')).toBeNull();
    expect(host.querySelector('[data-testid="driving-speed"]').textContent).toContain('—');
  });
});
