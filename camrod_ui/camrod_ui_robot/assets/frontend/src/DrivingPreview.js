// HH_261001 - Explicit local UI fixture. Never connects to ROS or sends commands.
// This is a visual acceptance preview, not driving or perception evidence.
import React, { useEffect, useMemo, useState } from 'react';
import App from './App';
import './DrivingPreview.css';

const DEMO_SPEED = 0.92;
const DEMO_RADIUS = 6;
const DEMO_TURN_END = 8 + Math.PI * DEMO_RADIUS / 2;
const DEMO_LENGTH = DEMO_TURN_END + 18;
const DEMO_TRAVEL_LENGTH = DEMO_LENGTH - 4;
const FOUR_WHEEL_DEMO_DURATION_S = 30;
const PAUSED_PREVIEW_MODES = ['stop', 'offline', 'stale'];
const FOUR_WHEEL_PREVIEWS = {
  zero_turn_left: '시연: 제자리 좌회전',
  zero_turn_right: '시연: 제자리 우회전',
  crab_left: '시연: 왼쪽 크랩',
  crab_right: '시연: 오른쪽 크랩',
  reverse: '시연: 후진',
};

// HH_261002 - These deterministic body poses are UI fixtures, never actuator
// feedback or commands. A fixed yaw during crab/reverse makes sideways and
// backward travel visibly distinguishable from turning the robot's body.
export function fourWheelPreviewMotion(mode, elapsed) {
  if (!Object.prototype.hasOwnProperty.call(FOUR_WHEEL_PREVIEWS, mode)) return null;
  const t = Math.max(0, Math.min(FOUR_WHEEL_DEMO_DURATION_S, elapsed));
  const direction = mode.endsWith('_right') ? -1 : 1;
  if (mode.startsWith('zero_turn_')) {
    return { pose: { x: 4, y: 0, yaw: direction * 0.35 * t }, vx: 0, vy: 0, yawRate: direction * 0.35 };
  }
  if (mode.startsWith('crab_')) {
    return { pose: { x: 4, y: direction * 0.45 * t, yaw: 0 }, vx: 0, vy: direction * 0.45, yawRate: 0 };
  }
  return { pose: { x: 4 - 0.45 * t, y: 0, yaw: 0 }, vx: -0.45, vy: 0, yawRate: 0 };
}

// HH_261001 - A meter-scale straight -> smooth 90 degree bend -> straight route. Position,
// heading and speed describe the same synthetic trajectory (never live ROS).
export function demoRoutePose(distance) {
  const d = Math.max(0, Math.min(DEMO_LENGTH, distance));
  if (d < 8) return { x: d, y: 0, yaw: 0 };
  if (d <= DEMO_TURN_END) {
    const angle = (d - 8) / DEMO_RADIUS;
    return { x: 8 + DEMO_RADIUS * Math.sin(angle), y: DEMO_RADIUS * (1 - Math.cos(angle)), yaw: angle };
  }
  return { x: 14, y: 6 + d - DEMO_TURN_END, yaw: Math.PI / 2 };
}
const DEMO_ROUTE = Array.from({ length: 81 }, (_, index) => demoRoutePose(index * DEMO_LENGTH / 80));

function previewServiceState({ idle, arrived, returning, leg }) {
  if (idle) return 'DROP_ZONE_WAIT';
  if (arrived) {
    if (returning) return 'DROP_ZONE_PARKING';
    return leg === 'recall' ? 'GUEST_LOADING_WAIT' : 'SITE_ARRIVED';
  }
  if (returning) return 'RETURNING_TO_DROP_ZONE';
  return leg === 'recall' ? 'RECALL_TO_SITE_ROAD' : 'MOVING_TO_SITE';
}

function previewDescription({ stopped, arrived, returning, leg }) {
  if (stopped) return '시연: 전방 장애물로 안전 정지';
  if (arrived && FOUR_WHEEL_PREVIEWS[leg]) return `${FOUR_WHEEL_PREVIEWS[leg]} · 시연 완료`;
  if (arrived) return returning ? '시연: 복귀 위치에 도착' : '시연: 목적지에 도착';
  if (FOUR_WHEEL_PREVIEWS[leg]) return `${FOUR_WHEEL_PREVIEWS[leg]} · 합성 위치 입력, 실제 주행 아님`;
  if (returning) return '대기·충전 장소로 복귀 중';
  return leg === 'recall' ? 'B7 이용객 호출 위치로 이동 중' : 'B7 배송 목적지로 이동 중';
}

export function makeDrivingPreviewSnapshot(mode = 'delivery', elapsed = 0, activeLeg = 'delivery') {
  const leg = PAUSED_PREVIEW_MODES.includes(mode) ? activeLeg : mode;
  const returning = leg === 'return';
  const stopped = mode === 'stop';
  const offline = mode === 'offline';
  const stale = mode === 'stale';
  const idle = mode === 'idle';
  const wheelFixture = fourWheelPreviewMotion(leg, elapsed);
  const travelled = Math.min(DEMO_TRAVEL_LENGTH, Math.max(0, elapsed) * DEMO_SPEED);
  const arrived = !stopped && !offline && !stale && !idle && (wheelFixture
    ? elapsed >= FOUR_WHEEL_DEMO_DURATION_S : travelled >= DEMO_TRAVEL_LENGTH);
  const routeDistance = returning ? DEMO_TRAVEL_LENGTH - travelled : 4 + travelled;
  const position = wheelFixture ? wheelFixture.pose : demoRoutePose(routeDistance);
  if (returning) position.yaw += Math.PI;
  const frozen = stopped || offline || stale || idle || arrived;
  const speed = frozen ? 0 : wheelFixture ? Math.hypot(wheelFixture.vx, wheelFixture.vy) : DEMO_SPEED;
  const points = [];
  // HH_261001 - Deterministic fixture data; the scene does not paint these synthetic cloud
  // points as if they were live LiDAR. Object labels remain demo-labelled.
  for (let i = 0; i < DEMO_ROUTE.length; i += 1) {
    const p = DEMO_ROUTE[i];
    for (const side of [-1, 1]) {
      points.push([p.x - Math.sin(p.yaw) * side * 2.6, p.y + Math.cos(p.yaw) * side * 2.6, 0.2 + (i % 5) * 0.22]);
      points.push([p.x - Math.sin(p.yaw) * side * 3.2, p.y + Math.cos(p.yaw) * side * 3.2, 0.6 + (i % 8) * 0.3]);
    }
  }
  return {
    schema_version: 1,
    connected: !offline,
    mission: {
      active: !idle, generation: idle ? 0 : 42, intent: leg === 'recall' ? 'recall' : 'delivery',
      site: idle ? '' : 'B7', service_state_name: previewServiceState({ idle, arrived, returning, leg }),
      phase: idle ? 'READY' : stopped ? 'SAFETY_STOP' : arrived ? 'ARRIVED' : 'DRIVING',
      description: stale ? '시연: 수신 지연 · 마지막 위치 유지' : previewDescription({ stopped, arrived, returning, leg }),
    },
    motion: { speed_mps: speed, vx_mps: frozen ? 0 : wheelFixture ? wheelFixture.vx : speed,
      vy_mps: frozen ? 0 : wheelFixture ? wheelFixture.vy : 0,
      yaw_rate_rps: frozen ? 0 : wheelFixture ? wheelFixture.yawRate
        : speed && routeDistance > 8 && routeDistance < DEMO_TURN_END ? (returning ? -speed : speed) / DEMO_RADIUS : 0,
      frame_id: 'robot_center_link', age_s: offline || stale ? 9 : 0.02 },
    battery: { percentage: 82 },
    pose: { ...position, frame_id: 'map', age_s: offline || stale ? 9 : 0.02 },
    route: { points: (returning ? [...DEMO_ROUTE].reverse() : DEMO_ROUTE).map(p => [p.x, p.y]),
      frame_id: 'map', age_s: offline || stale ? 9 : 0.05, valid: !wheelFixture && !offline && !stale && !idle && !arrived },
    progress: { remaining_distance_m: offline || stale || wheelFixture ? null : DEMO_TRAVEL_LENGTH - travelled,
      remaining_time_s: wheelFixture ? null : arrived ? 0 : speed ? (DEMO_TRAVEL_LENGTH - travelled) / speed : null,
      completion_pct: wheelFixture ? null : travelled / DEMO_TRAVEL_LENGTH * 100,
      valid: !wheelFixture && !offline && !stale && !stopped && !idle,
      reason: stopped ? '안전 정지 중' : offline ? '연결 끊김' : stale ? '시연: 수신 지연' : wheelFixture ? '시연: 운동학 검사' : '시연 추정값' },
    perception: { frame_id: 'map', age_s: offline || stale ? 9 : 0.2, points, objects: [
      // HH_261002 - Known fixture boxes exercise metric rendering only; the preview
      // banner explicitly identifies synthetic data, never sensor measurements.
      { id: 'demo-person', class_name: 'person', x: 10, y: -2.8, z: 0.85, confidence: 0.94,
        geometry_source: 'observed_lidar_extent', dimensions: { x: 0.55, y: 0.65, z: 1.7 },
        bbox: { center: { x: 10, y: -2.8, z: 0.85 } }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
      // HH_261002 - Keep the whole car box outside every bend, not merely away
      // from the initial straight. Preview animation has no collision controller.
      { id: 'demo-car', class_name: 'car', x: 14, y: -5, z: 0.75, confidence: 0.91,
        geometry_source: 'observed_lidar_extent', dimensions: { x: 4.2, y: 1.8, z: 1.5 },
        bbox: { center: { x: 14, y: -5, z: 0.75 } }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
      { id: 'demo-obstacle', class_name: '미분류', x: 17, y: 9, z: 0.5, confidence: null },
    ] },
    sensors: Object.fromEntries(['gnss', 'lidar', 'camera', 'radar'].map(name => [name, { label: '시연 입력', age_s: offline || stale ? 9 : 0.2 }])),
  };
}

export default function DrivingPreview() {
  const [theme, setTheme] = useState('light');
  const [mode, setMode] = useState('delivery');
  const [activeLeg, setActiveLeg] = useState('delivery');
  const [animate, setAnimate] = useState(true);
  const [elapsed, setElapsed] = useState(0);
  const [pulse, setPulse] = useState(0);
  useEffect(() => {
    // HH_261001 - Pausing position animation must not simulate losing the sensor stream.
    const timer = setInterval(() => {
      setPulse(value => value + 1);
      if (animate && ![...PAUSED_PREVIEW_MODES, 'idle'].includes(mode)) {
        setElapsed(value => Math.min(Math.max(DEMO_TRAVEL_LENGTH / DEMO_SPEED, FOUR_WHEEL_DEMO_DURATION_S), value + 0.1));
      }
    }, 100);
    return () => clearInterval(timer);
  }, [animate, mode]);
  const snapshot = useMemo(() => ({ ...makeDrivingPreviewSnapshot(mode, elapsed, activeLeg), preview_sequence: pulse }), [mode, elapsed, pulse, activeLeg]);
  const choose = value => {
    // HH_261001 - Stop/offline must freeze the current trajectory, not teleport to its start.
    if (!PAUSED_PREVIEW_MODES.includes(value)) {
      setActiveLeg(value);
      if (!PAUSED_PREVIEW_MODES.includes(mode) || activeLeg !== value) setElapsed(0);
    }
    setMode(value);
  };
  return <div className="driving-preview-shell" data-theme={theme}>
    <header className="driving-preview-toolbar">
      <div><strong>CAMROD · 3D 주행 화면 시연</strong><small>시연 상태·테마 선택용 개발 패널 · 로봇 명령 전송 없음</small></div>
      <nav aria-label="시연 상태 선택">
        {[['idle', '기존 홈'], ['delivery', '배송'], ['recall', '호출'], ['return', '복귀'], ['stop', '안전 정지'], ['stale', '시연: 수신 지연'], ['offline', '연결 끊김']].map(([value, label]) =>
          <button key={value} data-preview={value} aria-pressed={mode === value} onClick={() => choose(value)}>{label}</button>)}
        {/* HH_261002 - Local-only 4WS cases exercise the actual scene without a robot connection. */}
        <select data-preview="four-wheel-mode" aria-label="4륜 표시 시연"
          value={FOUR_WHEEL_PREVIEWS[mode] ? mode : ''}
          onChange={event => { if (event.target.value) choose(event.target.value); }}>
          <option value="">4륜 표시 시연 선택</option>
          {Object.entries(FOUR_WHEEL_PREVIEWS).map(([value, label]) => <option key={value} value={value}>{label}</option>)}
        </select>
        <button data-preview="theme" onClick={() => setTheme(value => value === 'light' ? 'dark' : 'light')}>{theme === 'light' ? '다크 모드' : '라이트 모드'}</button>
        <button data-preview="animate" onClick={() => setAnimate(value => !value)}>{animate ? '시연 애니메이션 정지' : '시연 애니메이션 재생'}</button>
      </nav>
    </header>
    <main className="driving-preview-canvas">
      <App drivingPreviewSnapshot={snapshot} drivingPreviewTheme={theme} />
    </main>
  </div>;
}
